# 获取飞船上的所有铰链  
hinges = vessel.parts.robotic_hinges  
  
# 控制单个铰链  
hinge = hinges[0]  
print(f"当前角度: {hinge.current_angle}")  
print(f"目标角度: {hinge.target_angle}")  
  
# 设置目标角度和移动速度  
hinge.target_angle = 45.0  # 设置目标角度为45度  
hinge.rate = 10.0  # 设置移动速度为10度/秒  
  
# 控制电机状态  
hinge.motor_engaged = True  # 启用电机  
hinge.locked = False  # 解锁移动  
  
# 移动到初始位置  
hinge.move_home()