import krpc  
  
conn = krpc.connect(name='Vessel List')  
vessels = conn.space_center.vessels  
  
for vessel in vessels:  
    print(f"飞行器名称: {vessel.name}")