import krpc  
import time 
import numpy as np
import math  
from math import *
from coord_rotations import *
'''
参考系类型

1.体轴系，以载具质心为中心，顺序为右前下（左手）
Vessel.reference_frame

2.以载具质心为中心，x轴指向轨道内测，y指向前（切向），z为法向
Vessel.orbital_reference_frame

3.地面系，以载具质心为中心，xyz顺序为天北东
Vessel.surface_reference_frame

4.速度系，以载具质心为中心，x垂直于y和z向上，y严格指向地速矢量，z指向地平线
Vessel.surface_velocity_reference_frame

5.以天体质心为中心，x指向0°经线，y指北，z指东经90°
CelestialBody.reference_frame

6.惯性系，除了y指北，其余都不跟着自转
CelestialBody.non_rotating_reference_frame

7.以天体为中心（可能是考虑天体自身公转轨道的情况）
CelestialBody.orbital_reference_frame

Node.reference_frame

Node.orbital_reference_frame

Part.reference_frame

Part.center_of_mass_reference_frame

DockingPort.reference_frame

Thruster.thrust_reference_frame

# 特殊情况
1.
vessel.orbit.body.reference_frame 不是飞行器的参考系，而是通过
.orbit.body方位到了CelestialBody的参考系类型

2.
# 创建混合参考系：位置来自天体，旋转来自飞行器表面  
hybrid_frame = conn.space_center.ReferenceFrame.create_hybrid(  
    position=vessel.orbit.body.reference_frame,  
    rotation=vessel.surface_reference_frame  
)
混合参考系通常的传参
ReferenceFrame.create_hybrid(  
    position=position_frame,      # 位置原点来源  
    rotation=rotation_frame,      # 坐标轴方向来源    
    velocity=velocity_frame,      # 速度计算来源  
    angular_velocity=angular_frame # 角速度来源  
)
  
# 获取相对于表面的速度矢量  
surface_velocity_vector = vessel.flight(hybrid_frame).velocity  
print(f"表面速度矢量: {surface_velocity_vector}")
'''

dt = 1

conn = krpc.connect(name='Velocity Vectors')  
vessel = conn.space_center.active_vessel  
GravSource = vessel.orbit.body
control = vessel.control    
# 获取表面参考系  
s_frame = vessel.surface_reference_frame 
# 使用表面参考系获取飞行数据
flight = vessel.flight(s_frame)  

def transform_surface_velocity(velocity_vec, longitude=0, latitude=0):
    # 经纬度为角度制
    v = np.array(velocity_vec, copy=True)
    v[2] = -v[2]
    if abs(latitude) > 90 - 1e-1:
        longitude = 0
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
    
    # # 相对地表速度？  东，北，地？不对，是相对旋转地表的速度, 上北东？
    # surface_frame = GravSource.reference_frame  
    # surface_vel = vessel.flight(surface_frame).velocity  

    # 左手系速度（0E, N, 90E）
    velocity0 = np.array(s_frame)
    
    # 速度转到地表系
    velocity = transform_surface_velocity(velocity0, flight.longitude, flight.latitude)


    # 机头指向()
    direction = np.array(vessel.direction(s_frame))
    

    print("速度矢量:", np.round(velocity, 2))
    print("机头指向:", np.round(direction, 2))
    distance_to_center = vessel.orbit.radius
    print("距地心距离：", distance_to_center/1000 ,"km")
    rotational_speed_rad_per_sec = round(GravSource.rotational_speed*180/pi,10) # 弧度变角度
    rotational_period = round(GravSource.rotational_period/3600, 3)  # 秒变时  
    print("当前星球自转角速度", rotational_speed_rad_per_sec, "deg/s")
    print("当前星球自转周期", rotational_period, "h")
    print(f"纬度: {flight.latitude:.6f}°")  
    print(f"经度: {flight.longitude:.6f}°") 
    print()
    time.sleep(dt)

