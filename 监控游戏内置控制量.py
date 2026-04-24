import krpc  
import time
  
# 连接到kRPC服务器  
conn = krpc.connect(name='控制量读取')  
space_center = conn.space_center  
vessel = space_center.active_vessel  
control = vessel.control  
  
# 获取油门控制量 (0.0 - 1.0)  
throttle_value = control.throttle  
  
# 获取姿态控制量 (-1.0 - 1.0)  
pitch_value = control.pitch    # 俯仰 (w/s键)  
yaw_value = control.yaw        # 偏航 (a/d键)    
roll_value = control.roll      # 滚转 (q/e键)  
  
# 获取平移控制量 (-1.0 - 1.0)  
forward_value = control.forward  # 前后平移 (h/n键)  
up_value = control.up           # 上下平移 (i/k键)  
right_value = control.right     # 左右平移 (j/l键)  
  
# 获取起落架状态 (True/False)  
gear_state = control.gear  
  
# 获取刹车状态 (True/False)  
brakes_state = control.brakes

# 实时监控控制量  
while True:  
    print(f"油门: {control.throttle:.2f}")  
    print(f"俯仰: {control.pitch:.2f}, 偏航: {control.yaw:.2f}, 滚转: {control.roll:.2f}")  
    print(f"平移 - 前: {control.forward:.2f}, 上: {control.up:.2f}, 右: {control.right:.2f}")  
    print(f"起落架: {'收起' if control.gear else '放下'}")  
    print(f"刹车: {'制动' if control.brakes else '释放'}")  
    print("-" * 30)  
    time.sleep(0.1)