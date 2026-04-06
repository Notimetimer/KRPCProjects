import krpc  
import time  

conn = krpc.connect(name="姿态控制示例")  
vessel = conn.space_center.active_vessel  
control = vessel.control  
flight = vessel.flight()  
  
# 获取当前姿态  
print(f"当前姿态: 俯仰={flight.pitch:.1f}° 偏航={flight.heading:.1f}° 滚转={flight.roll:.1f}°")  
  
# 设置油门和姿态控制  
control.throttle = 0.7  
time.sleep(1)  
  
# 向上俯仰  
control.pitch = 0.3  
time.sleep(2)  
control.pitch = 0  
  
# 使用自动导航指向特定方向  
ap = vessel.auto_pilot  
ap.reference_frame = vessel.surface_reference_frame  
ap.target_pitch_and_heading(30, 45)  # 俯仰30°，偏航45°  
ap.engage()  
ap.wait()  
ap.disengage()