import krpc  
import time 
import numpy as np
import math  
from math import *
from coord_rotations import *
from PIDtools import PositionPID

conn = krpc.connect(name='Flight Data')  
vessel = conn.space_center.active_vessel  
GravSource = vessel.orbit.body
control = vessel.control  
# 体轴系
vessel_ref = vessel.reference_frame   
# 获取表面参考系  
surface_frame = vessel.surface_reference_frame
# 获取惯性参考系（非旋转天体参考系）  
inertial_frame = vessel.orbit.body.non_rotating_reference_frame
# # 使用表面参考系获取飞行数据
# flight = vessel.flight(surface_frame)

dt = 0.05

# 期望指向（北-天-东坐标系）
target_point_ = np.array([0, 1, 0])

# 姿态控制增益
yaw_gain = 1.0
pitch_gain = 1.0

# 自动定高 PID（与 3自动定高.py 相同）
target_height = 200.0
height_controller = PositionPID(max=1, min=0, p=0.1, i=0, d=0.07)

p_turn = 2.0
i_turn = 0.0
d_turn = 1.4
yaw_pid = PositionPID(max=1, min=-1, p=p_turn, i=i_turn, d=d_turn)
pitch_pid = PositionPID(max=1, min=-1, p=p_turn, i=i_turn, d=d_turn)
roll_pid = PositionPID(max=1, min=-1, p=p_turn/70, i=i_turn, d=0)

# 体轴基本方向  
body_axes = {  
    'forward': (0, 1, 0),  # 机头  
    'right': (1, 0, 0),    # 右侧  
    'down': (0, 0, 1)      # 下方  
}  
body_axes_custom = {
    'x': None, # 前
    'y': None, # 上
    'z': None, # 右
}

# 天北东到北天东
def UNE2NUE(direction0):
    direction = np.zeros(3)
    direction[0] = direction0[1]
    direction[1] = direction0[0]
    direction[2] = direction0[2]
    return direction

def transform_surface_velocity(velocity_vec, longitude=0, latitude=0):
    # 经纬度为角度制
    v = np.array(velocity_vec, copy=True)
    v[2] = -v[2]
    v = passive_rotation(v, -longitude*pi/180, latitude*pi/180)
    out = np.zeros(3)
    out[0] = v[1]
    out[1] = v[0]
    out[2] = -v[2]
    return out

for i in range(int(120/dt)):
    # 获取速度矢量
    
    # 相对星球质心速度  
    orbital_frame = GravSource.non_rotating_reference_frame  
    orbital_vel = vessel.flight(orbital_frame).velocity
    
    # 相对地表速度？  东，北，地？不对，是相对旋转地表的速度, 上北东？
    Globe_frame = GravSource.reference_frame  
    surface_vel = vessel.flight(Globe_frame).velocity  

    # 左手系速度（0E, N, 90E）
    velocity0 = np.array(surface_vel)
    
    # 速度转到地表系
    velocity = transform_surface_velocity(velocity0, 
                                          vessel.flight(surface_frame).longitude, 
                                          vessel.flight(surface_frame).latitude)

    # 机头指向(天北东？)
    direction0 = np.array(vessel.direction(surface_frame))
    # 机头指向(转为习惯的北天东)
    direction_head = UNE2NUE(direction0)
    print("速度矢量:", np.round(velocity, 2))
    print("机头指向:", np.round(direction_head, 2))
    
    # 转换到表面参考系，封装好的坐标转换，但是左手系  
    surface_axes = {}  
    for axis_name, direction in body_axes.items():  
        surface_axes[axis_name] = conn.space_center.transform_direction(  
            direction, vessel_ref, surface_frame  
        )
    body_axes_custom={}
    body_axes_custom['x'] = UNE2NUE(surface_axes['forward'])
    body_axes_custom['y'] = - UNE2NUE(surface_axes['down'])
    body_axes_custom['z'] = UNE2NUE(surface_axes['right'])
    # 姿态控制：基于 body_axes_custom 与 target_point_
    bx = np.array(body_axes_custom['x'])
    by = np.array(body_axes_custom['y'])
    bz = np.array(body_axes_custom['z'])

    tmp = np.cross(bx, target_point_)
    
    # 没有仔细分析理论，瞎写的
    yaw_cmd = float(np.dot(tmp, by)*0.99999)
    pitch_cmd = float(np.dot(tmp, bz)*0.99999)
    
    # control.yaw = -float(np.clip(yaw_gain * yaw_cmd, -1.0, 1.0))
    # control.pitch = float(np.clip(pitch_gain * pitch_cmd, -1.0, 1.0))

    control.yaw = yaw_pid.calculate(-yaw_cmd, dt=dt)
    control.pitch = pitch_pid.calculate(pitch_cmd, dt=dt)
    # 获取机体在惯性系中的角速度  
    angular_velocity_inertial = vessel.angular_velocity(inertial_frame)  
    
    # 将角速度变换到机体坐标系  
    angular_velocity_body = conn.space_center.transform_direction(  
        angular_velocity_inertial, 
        inertial_frame, 
        vessel_ref,
    )
    body_ang_vel = np.zeros(3)
    # 左后上? 到前右下pqr
    # body_ang_vel = np.array(angular_velocity_body)
    body_ang_vel[0] = -angular_velocity_body[1] # p
    body_ang_vel[1] = -angular_velocity_body[0] # q
    body_ang_vel[2] = -angular_velocity_body[2] # r

    control.roll = roll_pid.calculate(-body_ang_vel[0]*180/pi, dt=dt)


    # 垂直速度（米/秒）  
    vertical_speed = vessel.flight(Globe_frame).vertical_speed  
      
    # 表面高度（米）  
    surface_altitude = vessel.flight(Globe_frame).surface_altitude  
    
    # 获取当前油门值（0-1之间）  
    throttle = vessel.control.throttle  
    
    if surface_altitude < 3 and abs(vertical_speed)<1:
        control.throttle = 0
        break
    
    # 期望下降速度
    if surface_altitude > 150:
        target_descend_speed = np.clip(surface_altitude/100, 0, 1)*-50
    else:
        target_descend_speed = np.clip(surface_altitude/100, 0, 1)*-10
    # 速度误差
    speed_error = vertical_speed-target_descend_speed
    
    if speed_error > 0:
        control.throttle = 0
    else:
        control.throttle = np.clip(speed_error / -10, 0,1)
      
    print(f"垂直速度: {vertical_speed:.2f} m/s")  
    print(f"表面高度: {surface_altitude:.2f} m")  
    print(f"油门值: {throttle:.2f}")
    
    time.sleep(dt)