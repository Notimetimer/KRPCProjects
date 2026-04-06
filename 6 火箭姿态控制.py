import krpc  
import time 
import numpy as np
import math  
from math import *
from coord_rotations import *
from PIDtools import PositionPID

"""
# 获取控制对象  
control = vessel.control  

- 油门控制, 范围 0~1
control.throttle = 0.7

- 油门以外的控制：数值范围-1~1

# 设置姿态控制  
control.pitch = 0.5  # 向上俯仰  
control.yaw = -0.3   # 向左偏航    
control.roll = 1.0   # 向右滚转  
  
# 设置平移控制  
control.forward = 0.8  # 向前平移  
control.up = -0.5      # 向下平移  
control.right = 0.2    # 向右平移
"""

dt = 0.1

conn = krpc.connect(name='Velocity Vectors')  
vessel = conn.space_center.active_vessel  
GravSource = vessel.orbit.body
control = vessel.control  
# 体轴系
vessel_ref = vessel.reference_frame   
# 获取表面参考系  
surface_frame = vessel.surface_reference_frame
# 使用表面参考系获取飞行数据
flight = vessel.flight(surface_frame)  

# 期望指向（北-天-东坐标系）
target_point_ = np.array([0, 1, 0])

# 姿态控制增益
yaw_gain = 1.0
pitch_gain = 1.0

# 自动定高 PID（与 3自动定高.py 相同）
target_height = 200.0
height_controller = PositionPID(max=1, min=0, p=0.1, i=0, d=0.07)

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


for i in range(int(60*5/dt)):
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
    velocity = transform_surface_velocity(velocity0, flight.longitude, flight.latitude)


    # 机头指向(天北东？)
    direction0 = np.array(vessel.direction(surface_frame))
    # 机头指向(转为习惯的北天东)
    direction = UNE2NUE(direction0)
    print("速度矢量:", np.round(velocity, 2))
    print("机头指向:", np.round(direction, 2))
    
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
    
    print("飞行器体轴在表面参考系中的表示：")  
    for axis_name, vector in body_axes_custom.items():  
        print(f"{axis_name}: {np.round(vector,2)}")  

    # 姿态控制：基于 body_axes_custom 与 target_point_
    bx = np.array(body_axes_custom['x'])
    by = np.array(body_axes_custom['y'])
    bz = np.array(body_axes_custom['z'])

    tmp = np.cross(bx, target_point_)
    yaw_cmd = float(np.dot(tmp, by))
    pitch_cmd = float(np.dot(tmp, bz))
    control.yaw = float(np.clip(yaw_gain * yaw_cmd, -1.0, 1.0))
    control.pitch = float(np.clip(pitch_gain * pitch_cmd, -1.0, 1.0))
    # 自动定高 PID 输出油门
    surface_altitude = flight.surface_altitude
    throttle_cmd = height_controller.calculate(target_height - surface_altitude, dt=dt)
    control.throttle = float(np.clip(throttle_cmd, 0.0, 1.0))
    print(f"yaw_cmd={yaw_cmd:.3f}, pitch_cmd={pitch_cmd:.3f}, throttle={control.throttle:.3f}")

    # distance_to_center = vessel.orbit.radius
    # print("距地心距离：", distance_to_center/1000 ,"km")
    # rotational_speed_rad_per_sec = round(GravSource.rotational_speed*180/pi,10) # 弧度变角度
    # rotational_period = round(GravSource.rotational_period/3600, 3)  # 秒变时  
    # print("当前星球自转角速度", rotational_speed_rad_per_sec, "deg/s")
    # print("当前星球自转周期", rotational_period, "h")
    print(f"纬度: {flight.latitude:.6f}°")  
    print(f"经度: {flight.longitude:.6f}°") 
    print()
    time.sleep(dt)

