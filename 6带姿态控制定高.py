import krpc  
import time 
import numpy as np
import math  
from math import *
from coord_rotations import *
from PIDtools import PositionPID
from numpy.linalg import norm
"""
# 使用轨道参考系（惯性系）获取角速度  
angular_velocity = vessel.angular_velocity(vessel.orbital_reference_frame)  
  
# 或者使用天体的非旋转参考系  
angular_velocity = vessel.angular_velocity(vessel.orbit.body.non_rotating_reference_frame)
import krpc  
  
conn = krpc.connect(name='角速度示例')  
space_center = conn.space_center  
vessel = space_center.active_vessel  
  
# 获取惯性系角速度  
inertial_angular_velocity = vessel.angular_velocity(vessel.orbital_reference_frame)  
print(f"惯性系角速度: {inertial_angular_velocity} rad/s")  
  
# 获取机体坐标系角速度  
body_angular_velocity = vessel.angular_velocity(vessel.reference_frame)  
print(f"机体坐标系角速度: {body_angular_velocity} rad/s")  
  
# 获取大小  
angular_speed = (inertial_angular_velocity[0]**2 +   
                inertial_angular_velocity[1]**2 +   
                inertial_angular_velocity[2]**2)**0.5  
print(f"角速度大小: {angular_speed} rad/s")

# 将惯性系角速度转换到机体坐标系  
transformed_velocity = space_center.transform_velocity(  
    inertial_angular_velocity,  
    vessel.orbital_reference_frame,  
    vessel.reference_frame  
)
"""

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

dt = 0.05

conn = krpc.connect(name='Velocity Vectors')  
vessel = conn.space_center.active_vessel  
GravSource = vessel.orbit.body
control = vessel.control  
# 体轴系
vessel_ref = vessel.reference_frame   
# 获取表面参考系  
surface_frame = vessel.surface_reference_frame
# 获取惯性参考系（非旋转天体参考系）  
inertial_frame = vessel.orbit.body.non_rotating_reference_frame
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
    
    print("飞行器体轴在表面参考系中的表示：")  
    for axis_name, vector in body_axes_custom.items():  
        print(f"{axis_name}: {np.round(vector,2)}")  

    # 姿态控制：基于 body_axes_custom 与 target_point_
    bx = np.array(body_axes_custom['x'])
    by = np.array(body_axes_custom['y'])
    bz = np.array(body_axes_custom['z'])

    # 归一化
    target_point_ = target_point_/(np.linalg.norm(target_point_)+1e-5)

    tmp = np.cross(bx, target_point_)
    
    # 没有仔细分析理论，瞎写的
    yaw_cmd = float(np.dot(tmp, by)*0.99999)
    pitch_cmd = float(np.dot(tmp, bz)*0.99999)

    # 认真算了差角效果却更差，为啥？
    # L_ = target_point_
    # # 俯仰误差角
    # L_xy_b_ = L_ - np.dot(L_, bz) * bz / (norm(bz)+1e-8)
    # x_b_2L_xy_b_ = np.cross(bx, L_xy_b_) / (norm(L_xy_b_)+1e-8) # 省略一个norm
    # x_b_2L_xy_b_sin = np.dot(x_b_2L_xy_b_, bz)
    # x_b_2L_xy_b_cos = np.dot(bx, L_xy_b_) / (norm(L_xy_b_)+1e-8) # 省略一个norm
    # delta_z_angle = np.arctan2(x_b_2L_xy_b_sin, x_b_2L_xy_b_cos)
    # # 偏航误差角
    # L_xz_b_ = L_ - np.dot(L_, by) * by / (norm(by)+1e-8)
    # x_b_2L_xz_b_ = np.cross(bx, L_xz_b_) / (norm(L_xz_b_)+1e-8) # 省略一个norm
    # x_b_2L_xz_b_sin = np.dot(x_b_2L_xz_b_, by)
    # x_b_2L_xz_b_cos = np.dot(bx, L_xz_b_) / (norm(L_xz_b_)+1e-8)
    # delta_y_angle = np.arctan2(x_b_2L_xz_b_sin, x_b_2L_xz_b_cos)
    # # 滚转方向上暂时没有目标滚转方向，待续

    # pitch_cmd = delta_z_angle/pi
    # yaw_cmd = delta_y_angle/pi


    control.yaw = yaw_pid.calculate(-yaw_cmd, dt=dt)
    control.pitch = pitch_pid.calculate(pitch_cmd, dt=dt)

    # 自动定高 PID 输出油门
    surface_altitude = flight.surface_altitude
    throttle_cmd = height_controller.calculate(target_height - surface_altitude, dt=dt)
    # 头朝下的时候强制收油门
    if direction_head[1]<0:
        throttle_cmd=0
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

    # 获取机体坐标系角速度并打印
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

    body_ang_speed = np.linalg.norm(body_ang_vel)

    print("机体坐标系角速度:", np.round(body_ang_vel)*180/pi, "deg/s, 大小:", round(body_ang_speed, 6))

    print()
    time.sleep(dt)

