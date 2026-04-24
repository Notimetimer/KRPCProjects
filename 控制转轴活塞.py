"""
RoboticRotor (转子/步进电机)
    转速范围: -100 到 100 RPM 
    控制属性: TargetRPM, CurrentRPM
    速度控制: 通过调整TargetRPM值控制转速变化

RoboticPiston (活塞)
    延伸范围: 0 到 10 米 (具体数值取决于部件配置)
    控制属性: TargetExtension, CurrentExtension
    速度控制: Rate属性控制运动速度(米/秒)

RoboticHinge (铰链)
    角度范围: -180° 到 180° (具体数值取决于部件配置)
    控制属性: TargetAngle, CurrentAngle
    速度控制: Rate属性控制运动速度(度/秒)

RoboticRotation (旋转关节)
    角度范围: -180° 到 180° (具体数值取决于部件配置)
    控制属性: TargetAngle, CurrentAngle
    速度控制: Rate属性控制运动速度(度/秒) 

速度控制说明：
    转子: 通过TargetRPM直接控制转速
    活塞/铰链/伺服: 通过Rate属性控制运动速度
    阻尼控制: 所有部件都有Damping属性用于控制运动阻尼

要获取部件，使用vessel.parts.robotic_rotors、vessel.parts.robotic_pistons等属性。
"""

import krpc  
import time  
  
conn = krpc.connect(name='机器人部件控制')  
vessel = conn.space_center.active_vessel  
parts = vessel.parts  
  
print("=== 机器人部件控制测试 ===")  
  
# 获取所有机器人部件  
rotors = parts.robotic_rotors  
pistons = parts.robotic_pistons    
hinges = parts.robotic_hinges  
rotations = parts.robotic_rotations  
  
print(f"找到 {len(rotors)} 个转子, {len(pistons)} 个活塞, {len(hinges)} 个铰链, {len(rotations)} 个伺服电机")  
  
def control_to_max():  
    print("\n--- 控制到最大状态 ---")  
      
    # 控制转子到最大转速  
    for rotor in rotors:  
        rotor.motor_engaged = True  
        rotor.target_rpm = 100.0  # 设置最大转速  
        print(f"转子 {rotor.part.title}: 目标转速 = {rotor.target_rpm} RPM")  
      
    # 控制活塞到最大延伸  
    for piston in pistons:  
        piston.motor_engaged = True  
        piston.target_extension = 10.0  # 设置最大延伸  
        print(f"活塞 {piston.part.title}: 目标延伸 = {piston.target_extension} m")  
      
    # 控制铰链到最大角度  
    for hinge in hinges:  
        hinge.motor_engaged = True  
        hinge.target_angle = 180.0  # 设置最大角度  
        print(f"铰链 {hinge.part.title}: 目标角度 = {hinge.target_angle}°")  
      
    # 控制伺服电机到最大角度  
    for rotation in rotations:  
        rotation.motor_engaged = True  
        rotation.target_angle = 180.0  # 设置最大角度  
        print(f"伺服电机 {rotation.part.title}: 目标角度 = {rotation.target_angle}°")  
  
def control_to_min():  
    print("\n--- 控制到最小状态 ---")  
      
    # 控制转子到负最大转速  
    for rotor in rotors:  
        rotor.target_rpm = -100.0  # 设置负最大转速  
        print(f"转子 {rotor.part.title}: 目标转速 = {rotor.target_rpm} RPM")  
      
    # 控制活塞到最小延伸  
    for piston in pistons:  
        piston.target_extension = 0.0  # 设置最小延伸  
        print(f"活塞 {piston.part.title}: 目标延伸 = {piston.target_extension} m")  
      
    # 控制铰链到最小角度  
    for hinge in hinges:  
        hinge.target_angle = -180.0  # 设置最小角度  
        print(f"铰链 {hinge.part.title}: 目标角度 = {hinge.target_angle}°")  
      
    # 控制伺服电机到最小角度  
    for rotation in rotations:  
        rotation.target_angle = 0 #-180.0  # 设置最小角度  
        print(f"伺服电机 {rotation.part.title}: 目标角度 = {rotation.target_angle}°")  
  
def show_ranges_and_speed_control():  
    print("\n--- 数值范围和速度控制 ---")  
      
    for rotor in rotors:  
        print(f"\n转子 {rotor.part.title}:")  
        print(f"  当前转速: {rotor.current_rpm} RPM")  
        print(f"  目标转速: {rotor.target_rpm} RPM")  
        print(f"  转速范围: -100 到 100 RPM")  
        print(f"  可控制速度: 是 (通过target_rpm)")  
      
    for piston in pistons:  
        print(f"\n活塞 {piston.part.title}:")  
        print(f"  当前延伸: {piston.current_extension} m")  
        print(f"  目标延伸: {piston.target_extension} m")  
        print(f"  延伸范围: 0 到 10 m")  
        print(f"  运动速度控制: 是 (Rate属性: {piston.rate} m/s)")  
      
    for hinge in hinges:  
        print(f"\n铰链 {hinge.part.title}:")  
        print(f"  当前角度: {hinge.current_angle}°")  
        print(f"  目标角度: {hinge.target_angle}°")  
        print(f"  角度范围: -180° 到 180°")  
        print(f"  运动速度控制: 是 (Rate属性: {hinge.rate} °/s)")  
      
    for rotation in rotations:  
        print(f"\n伺服电机 {rotation.part.title}:")  
        print(f"  当前角度: {rotation.current_angle}°")  
        print(f"  目标角度: {rotation.target_angle}°")  
        print(f"  角度范围: -180° 到 180°")  
        print(f"  运动速度控制: 是 (Rate属性: {rotation.rate} °/s)")  
  
# 执行控制序列  
try:  
    show_ranges_and_speed_control()  
      
    control_to_max()  
    time.sleep(5)  # 等待运动完成  
      
    control_to_min()  
    time.sleep(5)  # 等待运动完成  
      
    print("\n=== 控制测试完成 ===")  
      
except Exception as e:  
    print(f"错误: {e}")  
finally:  
    conn.close()