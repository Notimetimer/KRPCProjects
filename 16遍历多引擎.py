"""
引擎顺序推测：
- 面对称的序号从主动安装的到从动安装的逐渐变大
- 有安装顺序的序号从先装到后装逐渐增大
- 中心对称的序号从主动安装的位置开始，
    针对机体前轴遵循左手螺旋定则逐渐变大
"""


import krpc  
import time  
  
conn = krpc.connect()  
vessel = conn.space_center.active_vessel  
engines = vessel.parts.engines  

print("逐个控制引擎推力:")  
for i, engine in enumerate(engines):  
    print(f"\n引擎 {i}: {engine.part.title}")  
      
    # 保存当前状态  
    original_throttle = engine.throttle  
    original_thrust_limit = engine.thrust_limit  
    original_independent = engine.independent_throttle  
      
    # 启用独立油门控制  
    engine.independent_throttle = True  
      
    # 推力调到最大  
    print("  推力调到最大...才怪")  
    engine.thrust_limit = 1.0   *0.1
    engine.throttle = 1.0   *0.1
    time.sleep(2)  
      
    # 推力调到0  
    print("  推力调到0...")  
    engine.throttle = 0.0  
    time.sleep(1)  
      
    # 恢复原始状态  
    # 关闭所有引擎的独立油门控制  
    for engine in vessel.parts.engines:  
        engine.independent_throttle = False  
    print("  恢复原始状态...")  
    engine.throttle = original_throttle  
    engine.thrust_limit = original_thrust_limit  
    engine.independent_throttle = original_independent

# import krpc  
# import time  
  
# conn = krpc.connect()  
# vessel = conn.space_center.active_vessel  
  
# # 获取所有引擎  
# engines = vessel.parts.engines  
  
# print("飞行器引擎列表:")  
# for i, engine in enumerate(engines):  
#     print(f"  {i}: {engine.part.title} (阶段 {engine.part.stage})")  
  
# # 逐个点火和关闭  
# for i, engine in enumerate(engines):  
#     print(f"\n点火引擎 {i}: {engine.part.title}")  
#     engine.active = True  
#     time.sleep(1)  
      
#     print(f"关闭引擎 {i}: {engine.part.title}")  
#     engine.active = False  
#     time.sleep(0)