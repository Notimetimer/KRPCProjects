import krpc  
import time  
  
conn = krpc.connect()  
vessel = conn.space_center.active_vessel  
body_frame = vessel.reference_frame  
  
def get_hinge_axes_projection(hinge):  
    """获取铰链3个轴在体轴系的投影"""  
    part_frame = hinge.part.reference_frame  
      
    # 获取部件参考系的3个轴并转换到体轴系  
    axis_x = conn.space_center.transform_direction((1, 0, 0), part_frame, body_frame)  # 右向轴  
    axis_y = conn.space_center.transform_direction((0, 1, 0), part_frame, body_frame)  # 上向轴  
    axis_z = conn.space_center.transform_direction((0, 0, 1), part_frame, body_frame)  # 前向轴  
      
    print(f"铰链3个轴在体轴系的投影:")  
    print(f"  X轴(右): {axis_x}")  
    print(f"  Y轴(上): {axis_y}")  
    print(f"  Z轴(前): {axis_z}")  
      
    return axis_x, axis_y, axis_z  
  
def multiply_vector(vector, scalar):  
    """向量标量乘法 - 修复版本"""  
    return (vector[0] * scalar, vector[1] * scalar, vector[2] * scalar)  
  
def add_vectors(v1, v2):  
    """向量加法 - 修复版本"""  
    return (v1[0] + v2[0], v1[1] + v2[1], v1[2] + v2[2])

def visualize_hinge_axes(hinge):  
    """可视化铰链的3个轴"""  
    hinge_pos = hinge.part.position(body_frame)  
    axis_x, axis_y, axis_z = get_hinge_axes_projection(hinge)  
      
    # 绘制3个轴的方向线  
    conn.drawing.add_direction(axis_x, body_frame, length=3, visible=True)  
    conn.drawing.add_direction(axis_y, body_frame, length=3, visible=True)  
    conn.drawing.add_direction(axis_z, body_frame, length=3, visible=True)  
      
    # 计算文字位置并添加标注  
    text_pos_x = add_vectors(hinge_pos, multiply_vector(axis_x, 3.5))  
    text_pos_y = add_vectors(hinge_pos, multiply_vector(axis_y, 3.5))  
    text_pos_z = add_vectors(hinge_pos, multiply_vector(axis_z, 3.5))  
      
    conn.drawing.add_text("X", body_frame, text_pos_x, (0, 0, 0, 1), visible=True)  
    conn.drawing.add_text("Y", body_frame, text_pos_y, (0, 0, 0, 1), visible=True)  
    conn.drawing.add_text("Z", body_frame, text_pos_z, (0, 0, 0, 1), visible=True)
  
# 测试铰链  
if vessel.parts.robotic_hinges:  
    hinge = vessel.parts.robotic_hinges[0]  
      
    print("初始状态:") 
    hinge.target_angle = 180 
    visualize_hinge_axes(hinge)  
    time.sleep(3)  
      
    # 旋转铰链并观察坐标变化  
    print("旋转铰链到30度:")  
    hinge.target_angle = 30  
    time.sleep(2)  
      
    conn.drawing.clear()  
    visualize_hinge_axes(hinge)  
    time.sleep(3)