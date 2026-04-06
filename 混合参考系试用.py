import krpc  
import time  
import numpy as np  
  
conn = krpc.connect(name="Surface Velocity")  
vessel = conn.space_center.active_vessel  
body = vessel.orbit.body  
  
# 创建混合参考系  
hybrid_frame = conn.space_center.ReferenceFrame.create_hybrid(  
    position=body.reference_frame,  
    rotation=vessel.surface_reference_frame  
)  

"""
# 完全自定义组合  
custom_frame = conn.space_center.ReferenceFrame.create_hybrid(  
    position=vessel.orbit.body.reference_frame,    # 位置：星球中心  
    rotation=vessel.surface_reference_frame,       # 旋转：地表对齐  
    velocity=vessel.orbital_reference_frame,       # 速度：轨道参考系  
    angular_velocity=body.reference_frame          # 角速度：星球旋转  
)
"""
  
print("开始监测地表速度（北天东坐标系）...")  
print("格式：北速度(m/s) | 天速度(m/s) | 东速度(m/s) | 总速度(m/s)")  

# 发现这样的东西向速度有问题，会被地球自转的速度干扰
  
try:  
    while True:  
        velocity = vessel.flight(hybrid_frame).velocity  
        north_vel = velocity[1]  # Y分量：北方向速度  
        up_vel = velocity[0]     # X分量：天顶方向速度    
        east_vel = velocity[2]   # Z分量：东方向速度  
        
        print(f"北: {north_vel:.1f} m/s, 天: {up_vel:.1f} m/s, 东: {east_vel:.1f} m/s")  
        time.sleep(1)  
          
except KeyboardInterrupt:  
    print("\n监测停止")