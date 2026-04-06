from PIDtools import *
import krpc  
import time 
import numpy as np

conn = krpc.connect(name='Flight Data')  
vessel = conn.space_center.active_vessel  
control = vessel.control  


# 获取表面参考系  
surface_frame = vessel.orbit.body.reference_frame  



# 获取表面参考系  
surface_frame = vessel.orbit.body.reference_frame  

dt = 0.05

target_height = float(200)

height_controller = PositionPID(max=1, min=0, p=0.1, i=0, d=0.07)

for i in range(int(120/dt)):

    # 获取飞行数据  
    flight = vessel.flight(surface_frame)  

    # 垂直速度（米/秒）  
    vertical_speed = flight.vertical_speed  
    
    # 表面高度（米）  
    surface_altitude = flight.surface_altitude  
    
    # 当前油门值（0-1之间）  
    throttle = vessel.control.throttle
    
    height_error = surface_altitude-target_height
    
    # 比例控制器
    throttle = height_controller.calculate(-height_error, dt=dt)
    
    control.throttle = np.clip(throttle, 0.0, 1.0)
    print(throttle)
    
    time.sleep(dt)
    