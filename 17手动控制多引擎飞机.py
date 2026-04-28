import krpc  
import time 
import numpy as np
import math  
from math import *
from coord_rotations import *
from PIDtools import PositionPID
from numpy.linalg import norm
import keyboard
import copy
import time

conn = krpc.connect(name='Velocity Vectors')  
vessel = conn.space_center.active_vessel  
parts = vessel.parts 
rotations = parts.robotic_rotations  
GravSource = vessel.orbit.body
control = vessel.control  
engines = vessel.parts.engines  
# 体轴系
vessel_ref = vessel.reference_frame   
# 获取表面参考系  
surface_frame = vessel.surface_reference_frame
# 获取惯性参考系（非旋转天体参考系）  
inertial_frame = vessel.orbit.body.non_rotating_reference_frame
# # 使用表面参考系获取飞行数据
# flight = vessel.flight(surface_frame)

dt = 0.05
k_pitch_yaw = 0.3

# 右前下转前右下
def RFD2FRD(input_vector):
    input_vector = np.array(input_vector) # 不管进来是什么一律转成array
    output_vector = np.zeros_like(input_vector)
    output_vector[0] = input_vector[1] # 前
    output_vector[1] = input_vector[0] # 右
    output_vector[2] = input_vector[2] # 下
    return output_vector

engines_dict = {}
engines_Mb = {}
# 获取部件位置和推力方向
for i, engine in enumerate(engines):  
    # 获取引擎部件在体轴系中的位置（相对于质心）  
    position = RFD2FRD(engine.part.position(vessel.reference_frame))
    engines_dict[i] = RFD2FRD(engine.part.position(vessel.reference_frame))
    engines_dict[i][2] = 0 # 忽视上下分量
    engines_dict[i] /= (norm(engines_dict[i]) + 1e-6) # 坐标归一化
    print(f"{i}: ({position[0]:.2f}, {position[1]:.2f}, {position[2]:.2f}) - {engine.part.title}")
    # 获取推力方向（相对与体轴，但是左手系）
    direction = engine.thrusters[0].thrust_direction(vessel.reference_frame)
    F_direction = RFD2FRD(direction)
    F_direction /= (norm(F_direction)+1e-8)
    print(F_direction)
    engines_Mb[i] = np.cross(position/(norm(position)+1e-8), F_direction)

x_b_in_b_ = np.array([1,0,0]) # 前
y_b_in_b_ = np.array([0,1,0]) # 右
z_b_in_b_ = np.array([0,0,1]) # 下

while True:
    # 获取油门控制量 (0.0 - 1.0)  
    throttle_cmd = control.throttle  
    
    # 获取姿态控制量 (-1.0 - 1.0)  
    pitch_cmd = control.pitch    # 俯仰 (w/s键)  
    # yaw_cmd = control.yaw        # 偏航 (a/d键)    
    roll_cmd = control.roll      # 滚转 (q/e键)

    # 油门补偿量
    for i, engine in enumerate(engines):
        engine.independent_throttle = True
        pitch_add_up = np.dot(y_b_in_b_, engines_Mb[i])
        roll_add_up = np.dot(x_b_in_b_, engines_dict[i])
        # pitch_add_up = np.dot(x_b_in_b_, engines_dict[i])
        # roll_add_up = np.dot(-y_b_in_b_, engines_dict[i])
        engine.throttle = throttle_cmd + \
            k_pitch_yaw * (
                pitch_add_up * pitch_cmd +
                 roll_add_up * roll_cmd)

    # # 舵机控制
    # for rotation in rotations:  
    #     rotation.target_angle = 10 * yaw_cmd

    time.sleep(dt)