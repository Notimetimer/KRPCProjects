import krpc  
  
conn = krpc.connect()  
vessel = conn.space_center.active_vessel  
body = vessel.orbit.body  
  
# 经纬度  
flight = vessel.flight()  
print(f"纬度: {flight.latitude:.6f}°")  
print(f"经度: {flight.longitude:.6f}°")  
  
# 星球半径  
print(f"星球半径: {body.equatorial_radius:.1f}m")  
  
# 相对星球质心速度  
orbital_frame = body.non_rotating_reference_frame  
orbital_vel = vessel.flight(orbital_frame).velocity  
print(f"轨道速度: {orbital_vel}")  
  
# 相对地表速度  
surface_frame = body.reference_frame  
surface_vel = vessel.flight(surface_frame).velocity  
print(f"地表速度: {surface_vel}")