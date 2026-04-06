import krpc  
import math  
  
conn = krpc.connect(name='Velocity Vectors')  
vessel = conn.space_center.active_vessel  
  
# 选择参考系  
surface_frame = vessel.surface_reference_frame  
body_frame = vessel.orbit.body.reference_frame  
  
# 获取速度矢量  
velocity = vessel.flight(body_frame).velocity  
direction = vessel.direction(body_frame)  
  
print(f"速度矢量: {velocity}")  
print(f"机头指向: {direction}")

# 使用表面参考系获取飞行数据  
flight = vessel.flight(surface_frame)  
  
# 直接获取角度信息  
pitch = flight.pitch  
heading = flight.heading  
roll = flight.roll  
angle_of_attack = flight.angle_of_attack  
sideslip_angle = flight.sideslip_angle  
  
print(f"俯仰角: {pitch:.2f}°")  
print(f"偏航角: {heading:.2f}°")  
print(f"滚转角: {roll:.2f}°")  
print(f"攻角: {angle_of_attack:.2f}°")  
print(f"侧滑角: {sideslip_angle:.2f}°")

# 轨道速度（惯性系）  
orbital_velocity = vessel.flight(vessel.orbit.body.non_rotating_reference_frame).velocity  
  
# 表面速度（随天体旋转）  
surface_velocity = vessel.flight(vessel.orbit.body.reference_frame).velocity  
  
# 飞行器相对速度  
vessel_velocity = vessel.flight(vessel.reference_frame).velocity  
  
print(f"轨道速度: {orbital_velocity}")  
print(f"表面速度: {surface_velocity}")  
print(f"飞行器相对速度: {vessel_velocity}")

# 创建混合参考系：位置来自天体，旋转来自飞行器表面  
hybrid_frame = conn.space_center.ReferenceFrame.create_hybrid(  
    position=vessel.orbit.body.reference_frame,  
    rotation=vessel.surface_reference_frame  
)  
  
# 获取相对于表面的速度矢量  
surface_velocity_vector = vessel.flight(hybrid_frame).velocity  
print(f"表面速度矢量: {surface_velocity_vector}")