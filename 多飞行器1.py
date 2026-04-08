import krpc  
  
conn = krpc.connect(name='多飞行器控制')  
space_center = conn.space_center  
vessels = space_center.vessels  
  
print("=== 所有飞行器信息 ===")  
for vessel in vessels:  
    vessel_name = vessel.name  
    vessel_type = str(vessel.type)  
    vessel_situation = str(vessel.situation)  
      
    # Get position info  
    flight_info = vessel.flight(vessel.surface_reference_frame)  
    altitude = flight_info.mean_altitude  
      
    print(f"飞行器: {vessel_name}")  
    print(f"  类型: {vessel_type}")  
    print(f"  状态: {vessel_situation}")  
    print(f"  高度: {altitude:.2f}m")  
    print()  
  
print("=== 将所有飞行器油门置为0 ===")  
for vessel in vessels:  
    try:  
        vessel.control.throttle = 0.0  
        print(f"已将 {vessel.name} 的油门设置为0")  
    except Exception as e:  
        print(f"控制 {vessel.name} 时出错: {e}")