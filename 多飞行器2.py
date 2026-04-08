import krpc  
import time  
  
conn = krpc.connect(name='灯光秀')  
space_center = conn.space_center  
  
# 获取所有飞行器信息  
print("=== 所有飞行器信息 ===")  
vessels = space_center.vessels  
for i, vessel in enumerate(vessels):  
    # 获取飞行器基本信息  
    name = vessel.name  
      
    # 获取位置信息（使用表面参考系）  
    flight = vessel.flight(vessel.surface_reference_frame)  
    latitude = flight.latitude  
    longitude = flight.longitude  
    altitude = flight.mean_altitude  
      
    print(f"序号: {i}")  
    print(f"名称: {name}")  
    print(f"经度: {longitude:.6f}°")  
    print(f"纬度: {latitude:.6f}°")   
    print(f"高度: {altitude:.2f}m")  
    print("-" * 30)  
  
# 灯光秀效果  
print("\n=== 开始灯光秀 ===")  
start_time = time.time()  
duration = 120  # 2分钟  
  
while time.time() - start_time < duration:  
    # 所有飞行器开灯  
    for vessel in vessels:  
        try:  
            vessel.control.lights = True  
        except:  
            pass  # 忽略无法控制的飞行器  
      
    time.sleep(1)  
      
    # 所有飞行器关灯  
    for vessel in vessels:  
        try:  
            vessel.control.lights = False  
        except:  
            pass  
      
    time.sleep(2)  
  
print("灯光秀结束！")