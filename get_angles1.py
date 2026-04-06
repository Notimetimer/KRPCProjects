import time
import krpc  
conn = krpc.connect()  
vessel = conn.space_center.active_vessel  
flight = vessel.flight()  # 使用默认表面参考系  
control = vessel.control
auto_pilot = vessel.auto_pilot

auto_pilot.target_pitch_and_heading(90, 90)
auto_pilot.engage()

obt_frame = vessel.orbit.body.non_rotating_reference_frame
srf_frame = vessel.orbit.body.reference_frame

control.throttle = 1
# time.sleep(1)

# print('Launch!')

dt = 0.1


for i in range(int(120/dt)):
    obt_speed = vessel.flight(obt_frame).speed
    srf_speed = vessel.flight(srf_frame).speed
    print('Orbital speed = %.1f m/s, Surface speed = %.1f m/s' %
          (obt_speed, srf_speed))
    time.sleep(dt)

# vessel.control.activate_next_stage()

# fuel_amount = conn.get_call(vessel.resources.amount, 'SolidFuel')
# expr = conn.krpc.Expression.less_than(
#     conn.krpc.Expression.call(fuel_amount),
#     conn.krpc.Expression.constant_float(0.1))
# event = conn.krpc.add_event(expr)
# with event.condition:
#     event.wait()
# print('Booster separation')
# vessel.control.activate_next_stage()

# for i in range(120):
#     # 获取姿态角度  
#     pitch = flight.pitch      # 俯仰角 (-90° 到 +90°)  
#     heading = flight.heading  # 偏航角 (0° 到 360°)   
#     roll = flight.roll        # 滚转角 (-180° 到 +180°)
#     print("heading", heading)
#     time.sleep(1)