import krpc  
import numpy as np  
  
conn = krpc.connect()  
vessel = conn.space_center.active_vessel  
body_frame = vessel.reference_frame  
  
def get_hinge_axis_vector(hinge, name):  
    """获取铰链的转轴矢量在体轴系中的表示"""
    """注意，铰链的转轴可以是部件模型的x,y或z轴，而不确定转轴位于哪个方向上"""
    part_frame = hinge.part.reference_frame  
    body_frame = vessel.reference_frame  
      
    # 测试哪个轴是转轴  
    axes = {  
        'X轴': conn.space_center.transform_direction((1, 0, 0), part_frame, body_frame),  
        'Y轴': conn.space_center.transform_direction((0, 1, 0), part_frame, body_frame),  
        'Z轴': conn.space_center.transform_direction((0, 0, 1), part_frame, body_frame)  
    }  
      
    print("铰链可能的转轴：")  
    for name, vector in axes.items():  
        print(f"  {name}: {vector}")  
      
    # 实际使用时需要通过观察铰链旋转来确定哪个是真正的转轴  
    return axes 
  
# 检查所有机器人部件  
if vessel.parts.robotic_pistons:  
    for piston in vessel.parts.robotic_pistons:  
        direction = piston.part.direction(body_frame)  
        print(f"活塞杆方向: {direction}")  
  
if vessel.parts.robotic_rotors:  
    for rotor in vessel.parts.robotic_rotors:  
        direction = rotor.part.direction(body_frame)  
        print(f"转子转轴方向: {direction}")  
  
if vessel.parts.robotic_rotations:  
    for rotation in vessel.parts.robotic_rotations:  
        direction = rotation.part.direction(body_frame)  
        print(f"步进电机转轴方向: {direction}")  
  
if vessel.parts.robotic_hinges:  
    for hinge in vessel.parts.robotic_hinges:  
        # 对于铰链，获取转轴矢量而不是部件方向  
        get_hinge_axis_vector(hinge, "铰链转轴方向")