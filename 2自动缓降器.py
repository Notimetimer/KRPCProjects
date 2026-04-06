import krpc  
import time 
import numpy as np
 
conn = krpc.connect(name='Flight Data')  
vessel = conn.space_center.active_vessel  
  
control = vessel.control  

# 获取表面参考系  
surface_frame = vessel.orbit.body.reference_frame  

dt = 0.05


for i in range(int(120/dt)):

    # 获取飞行数据  
    flight = vessel.flight(surface_frame)  
      
    # 垂直速度（米/秒）  
    vertical_speed = flight.vertical_speed  
      
    # 表面高度（米）  
    surface_altitude = flight.surface_altitude  
    
    # 当前油门值（0-1之间）  
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