import krpc
import time
import numpy as np
from simple_pid import PID

conn = krpc.connect(
    name='recovery',
    address='127.0.0.1',
    rpc_port=50000,
    stream_port=50001
)
space_center = conn.space_center
vessel = space_center.active_vessel

USING_AP = True
root = vessel.parts.root
stack = [(root, 0)]
KS25Engine_parts = []
while stack:
    part, depth = stack.pop()
    if part.title == "S3 KS-25“矢量”液体燃料引擎":
        KS25Engine_parts.append(part)
    for child in part.children:
        stack.append((child, depth + 1))


class PIDUsingV:
    def __init__(self):
        self.kp = 1
        self.ki = 0
        self.kd = 0

        self.maximum = 1
        self.minimum = -1

        self.prev_t = time.time()
        self.integral = 0

    def init(self, kp, ki, kd, upper, lower):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.maximum = upper
        self.minimum = lower

    def update(self, err, v):
        dt = time.time() - self.prev_t
        self.prev_t = time.time()
        self.integral += err * self.ki * dt
        self.integral = max(min(self.integral, self.maximum), self.minimum)
        return max(min(err * self.kp + self.integral + v * self.kd, self.maximum), self.minimum)


def set_engine_gimbal(lists: list, gim: float):
    gim = max(min(gim, 1), 0)
    for item in lists:
        item.engine.gimbal_limit = gim


def create_relative_reference_frame():
    body_frame = vessel.orbit.body.reference_frame  # 地固系
    lon = vessel.flight(body_frame).longitude
    lat = vessel.flight(body_frame).latitude
    # 绕y轴旋转-lon度
    temp1 = space_center.ReferenceFrame.create_relative(body_frame, rotation=(0., np.sin(-lon / 2. * np.pi / 180), 0., np.cos(-lon / 2. * np.pi / 180)))
    # 绕z轴旋转lat度
    temp2 = space_center.ReferenceFrame.create_relative(temp1, rotation=(0., 0., np.sin(lat / 2. * np.pi / 180), np.cos(lat / 2. * np.pi / 180)))
    # 沿x轴平移
    height = vessel.orbit.body.surface_height(lat, lon) + vessel.orbit.body.equatorial_radius
    target_frame = space_center.ReferenceFrame.create_relative(temp2, position=(height, 0., 0.))
    return target_frame


def steering(pitch_bias: float):
    ref = create_relative_reference_frame()
    v = vessel.flight(ref).velocity
    retro_steering = v / np.linalg.norm(v)

    pitch_controller.setpoint = - np.arcsin(retro_steering[0]) / np.pi * 180 + pitch_bias
    heading_controller.setpoint = - np.arctan(retro_steering[1] / retro_steering[2]) / np.pi * 180 + 270

    vessel.control.pitch = pitch_controller(vessel.flight(ref).pitch)
    vessel.control.yaw = heading_controller(vessel.flight(ref).heading)
    vessel.control.roll = roll_controller(vessel.flight(ref).roll)


count = 0


def calculate_angle(vec1: np.ndarray, vec2: np.ndarray) -> float:
    dot_product = np.dot(vec1, vec2)
    norm1 = np.linalg.norm(vec1)
    norm2 = np.linalg.norm(vec2)
    cos_theta = dot_product / (norm1 * norm2)
    angle_radians = np.arccos(np.clip(cos_theta, -1.0, 1.0))
    return angle_radians


def steering_ap(pitch_bias: float):
    vessel.auto_pilot.reference_frame = create_relative_reference_frame()
    bias_radius = pitch_bias * np.pi / 180
    rot_mat = np.array([[np.cos(bias_radius), 0, -np.sin(bias_radius)], [0, 1, 0], [np.sin(bias_radius), 0, np.cos(bias_radius)]])
    retro = -np.array(vessel.flight(vessel.auto_pilot.reference_frame).velocity)
    retro_with_bias = rot_mat @ retro
    vessel.auto_pilot.target_direction = retro_with_bias
    global count
    count += 1
    if count > 10:
        print("angle diff {:.3f}".format(calculate_angle(retro, retro_with_bias) / np.pi * 180))
        print("pitch_pid_gains", vessel.auto_pilot.pitch_pid_gains, "  error {:.2f}".format(vessel.auto_pilot.pitch_error))
        print("yaw_pid_gains", vessel.auto_pilot.yaw_pid_gains, "  error {:.2f}".format(vessel.auto_pilot.heading_error))
        print("roll_pid_gains", vessel.auto_pilot.roll_pid_gains, "  error {:.2f}".format(vessel.auto_pilot.roll_error), "  ctrl {:.2f}".format(vessel.control.roll))
        print()
        count = 0


print("auto landing started")
vessel.control.gear = False
vessel.control.sas = False
vessel.control.rcs = True
vessel.control.toggle_action_group(8)  # grid fin deploy

if USING_AP:
    vessel.auto_pilot.engage()
    vessel.auto_pilot.reference_frame = create_relative_reference_frame()
    vessel.auto_pilot.target_roll = 0
    vessel.auto_pilot.pitch_pid_gains = (10, 1, 4)
    vessel.auto_pilot.yaw_pid_gains = (10, 1, 4)
    vessel.auto_pilot.roll_pid_gains = (0.2, 0, 1)
    vessel.auto_pilot.auto_tune = False
    pitch_controller = None
    heading_controller = None
    roll_controller = None
else:
    pitch_controller = PID(Kp=0.1, Ki=0.01, Kd=0.4, output_limits=(-1, 1), differential_on_measurement=False)
    heading_controller = PID(Kp=0.1, Ki=0.01, Kd=0.4, output_limits=(-1, 1), differential_on_measurement=False)
    roll_controller = PID(Kp=0.02, Ki=0, Kd=0.01, output_limits=(-1, 1), differential_on_measurement=False, setpoint=0)

final_throttle = PIDUsingV()
final_throttle.init(kp=0.01, ki=0, kd=0.03, upper=0.8, lower=0)

set_engine_gimbal(KS25Engine_parts, 0.2)

while vessel.flight().mean_altitude > 1000:
    if USING_AP:
        steering_ap(-10)
    else:
        steering(-10)

    if vessel.flight().mean_altitude < 26000 and vessel.flight(create_relative_reference_frame()).speed > 1300:
        vessel.control.throttle = 0.33
    else:
        vessel.control.throttle = 0
    time.sleep(0.05)

print("final stage")
if not USING_AP:
    pitch_controller.tunings = (0.4, 0.01, 1)
    heading_controller.tunings = (0.4, 0.01, 1)
bGearDeploy = True
while vessel.flight().surface_altitude > 14:
    vessel.control.throttle = final_throttle.update(13 - vessel.flight(vessel.orbit.body.reference_frame).surface_altitude, -vessel.flight(vessel.orbit.body.reference_frame).vertical_speed)
    if np.linalg.norm(vessel.flight(vessel.orbit.body.reference_frame).velocity[1:]) > 50:
        if USING_AP:
            steering_ap(-10)
        else:
            steering(-10)
    else:
        if USING_AP:
            steering_ap(0)
        else:
            steering(0)

    if bGearDeploy and vessel.flight().surface_altitude < 250:
        bGearDeploy = False
        vessel.control.gear = True
    time.sleep(0.01)

vessel.control.throttle = 0
vessel.control.rcs = False
print("landing complete")
