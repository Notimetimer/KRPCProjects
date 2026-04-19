import krpc
import time
import numpy as np
import math  
from math import *
from coord_rotations import *
from PIDtools import PositionPID
from numpy.linalg import norm
import keyboard
import copy
conn = krpc.connect(name='Flight Data')

space_center = conn.space_center

# 天北东到北天东
def UNE2NUE(direction0):
    direction = np.zeros(3)
    direction[0] = direction0[1]
    direction[1] = direction0[0]
    direction[2] = direction0[2]
    return direction

def transform_surface_velocity(velocity_vec, longitude=0, latitude=0):
    # 经纬度为角度制
    v = np.array(velocity_vec, copy=True)
    v[2] = -v[2]
    v = passive_rotation(v, -longitude*pi/180, latitude*pi/180)
    out = np.zeros(3)
    out[0] = v[1]
    out[1] = v[0]
    out[2] = -v[2]
    return out


class rocket_control(object):
    def __init__(self, vessel):
        self.vessel = vessel
        self.name = vessel.name
        self.GravSource = vessel.orbit.body
        # 体轴系
        self.vessel_ref = vessel.reference_frame
        # 获取表面参考系  
        self.surface_frame = vessel.surface_reference_frame
        # 获取惯性参考系（非旋转天体参考系）  
        self.inertial_frame = vessel.orbit.body.non_rotating_reference_frame
        self.orbital_frame = self.GravSource.non_rotating_reference_frame
        self.Globe_frame = self.GravSource.reference_frame

        self.control = vessel.control
        self.flight_surface = self.vessel.flight(self.surface_frame)

        # 水平减速参数
        self.hor_controller = PositionPID(max=0.3, min=0, p=0.1, i=0, d=0.07)

        p_turn = 2.0
        i_turn = 0.0
        d_turn = 1.6 # 1.4
        self.yaw_pid = PositionPID(max=1, min=-1, p=p_turn, i=i_turn, d=d_turn)
        self.pitch_pid = PositionPID(max=1, min=-1, p=p_turn, i=i_turn, d=d_turn)
        self.roll_pid = PositionPID(max=1, min=-1, p=p_turn/70, i=i_turn, d=0)
        self.height_controller = PositionPID(max=1, min=0, p=0.1, i=0, d=0.07)

        # 体轴基本方向  
        self.body_axes = {  
            'forward': (0, 1, 0),  # 机头  
            'right': (1, 0, 0),    # 右侧  
            'down': (0, 0, 1)      # 下方  
        } 

    def get_state_of_NUE(self,):
        vessel = self.vessel
        body_axes_custom = {
            'x': None, # 前
            'y': None, # 上
            'z': None, # 右
        }
        # 体轴矢量转换到表面参考系，封装好的坐标转换，但是左手系  
        surface_axes = {}  
        for axis_name, direction in self.body_axes.items():  
            surface_axes[axis_name] = conn.space_center.transform_direction(  
                direction, self.vessel_ref, self.surface_frame  
            )
        body_axes_custom['x'] = UNE2NUE(surface_axes['forward'])
        body_axes_custom['y'] = - UNE2NUE(surface_axes['down'])
        body_axes_custom['z'] = UNE2NUE(surface_axes['right'])
        # 姿态控制：基于 body_axes_custom 与 target_point_
        self.bx_surf = np.array(body_axes_custom['x'])
        self.by_surf = np.array(body_axes_custom['y'])
        self.bz_surf = np.array(body_axes_custom['z'])
        # 机头指向(天北东顺序)
        direction0 = np.array(self.vessel.direction(self.surface_frame))
        # 机头指向(转为习惯的北天东)
        self.direction_head = UNE2NUE(direction0)
        # 速度矢量(地面系)
        surface_vel = self.vessel.flight(self.Globe_frame).velocity
        # 左手系速度（0E, N, 90E）
        velocity0 = np.array(surface_vel)
        # 速度转到地表系（北天东）
        self.velocity_surf = transform_surface_velocity(velocity0, 
                                            self.vessel.flight(self.surface_frame).longitude, 
                                            self.vessel.flight(self.surface_frame).latitude)
        self.speed_surf = norm(self.velocity_surf)
        # 获取机体在惯性系中的角速度  
        angular_velocity_inertial = self.vessel.angular_velocity(self.inertial_frame)  
        
        # 将角速度变换到机体坐标系  
        angular_velocity_body = conn.space_center.transform_direction(  
            angular_velocity_inertial, 
            self.inertial_frame, 
            self.vessel_ref,
        )
        body_ang_vel = np.zeros(3)
        # 左后上到前右下pqr
        body_ang_vel[0] = -angular_velocity_body[1] # p
        body_ang_vel[1] = -angular_velocity_body[0] # q
        body_ang_vel[2] = -angular_velocity_body[2] # r
        self.body_ang_vel = body_ang_vel
        self.p, self.q, self.r = body_ang_vel
        # 经纬高
        self.latitude = self.vessel.flight(self.surface_frame).latitude
        self.longitude = self.vessel.flight(self.surface_frame).longitude
        self.surface_altitude = self.vessel.flight(self.surface_frame).surface_altitude
        # 垂直速度（米/秒）  
        self.vertical_speed = vessel.flight(self.Globe_frame).vertical_speed
        # 重量
        self.mass = self.vessel.mass
        self.g = self.vessel.orbit.body.surface_gravity
        # 当前推重比
        self.current_twr = self.vessel.thrust / (self.mass * self.g)
        # 最大推重比
        self.max_twr = self.vessel.available_thrust / (self.mass * self.g)
        # 真空比冲
        self.vacuum_isp = self.vessel.vacuum_specific_impulse
        # 海平面比冲
        self.sea_level_isp = self.vessel.kerbin_sea_level_specific_impulse

        # 获取当前油门值（0-1之间）
        self.current_throttle = vessel.control.throttle
        self.current_elevator = vessel.control.pitch
        self.current_ailerons = vessel.control.roll
        self.current_rudder = vessel.control.yaw

        # 读取欧拉角和气流角
        # 角度制
        self.pitch_angle = self.flight_surface.pitch
        self.heading_angle = self.flight_surface.heading
        self.roll_angle = self.flight_surface.roll
        self.angle_of_attack_angle = self.flight_surface.angle_of_attack
        self.sideslip_angle_angle = self.flight_surface.sideslip_angle
        # 弧度制
        self.pitch_angle_rad = self.pitch_angle * pi/180
        self.heading_rad = self.heading_angle * pi/180
        self.roll_rad = self.roll_angle * pi/180

    # 自动缓降
    def fast_landing_controll(self, dt=0.05):
        # 读取观测数据
        self.get_state_of_NUE()
        # 接入控制
        control = self.vessel.control
        target_point0_ = np.array([0, 1, 0], dtype='float64')
        # 切断控制
        if self.surface_altitude < 3 and abs(self.vertical_speed)<1:
            control.throttle = 0
            control.pitch = 0
            control.yaw = 0
            control.roll = 0
            return
        
        # 启动最大推力高度
        h_max_thrust = (self.vertical_speed**2)/(2*self.g*(self.max_twr * (self.bx_surf[1]/(norm(self.bx_surf)+1e-3)) - 1)) * 1.2 # 安全高度倍率

        print("垂直速度", self.vertical_speed)
        print("对地速度", self.speed_surf)
        print("当前高度", self.surface_altitude)
        print("最大推力高度", h_max_thrust)
        print()

        # 高于最大推力高度时， 优先控制姿态
        if self.surface_altitude > max(h_max_thrust, 50):
            target_point0_ = - self.velocity_surf / (self.speed_surf + 1e-5)
            target_point_ = target_point0_
            control.throttle = 0.01 # 用微小的油门控制姿态
            target_descend_speed = self.vertical_speed  # 保持当前下降速度

            # 防止气动过载
            if self.speed_surf > 300 and self.surface_altitude < 6000:
                target_descend_speed = -300
        else:

            # # 目标指向考虑减速
            # v_hor = float((self.velocity_surf[0]**2 + self.velocity_surf[2]**2)**0.5) # 水平分速度大小

            # target_point_ = target_point0_ - self.hor_controller.calculate(v_hor, dt=dt) \
            #         * np.array([self.velocity_surf[0], 0.0, self.velocity_surf[2]])/(v_hor+1e-5)
    
            target_point0_ = - self.velocity_surf / (self.speed_surf + 1e-5)
            if self.surface_altitude > 30:
                target_point0_[1] = max(target_point0_[1], 0.3) # 至少都要朝上
            else:
                target_point0_[1] = max(target_point0_[1], 5.0) # 至少都要朝上

            target_point_ = target_point0_/(np.linalg.norm(target_point0_)+1e-5)
            
            # 速度过快，满推力减速
            if abs(self.vertical_speed) > 10:
                target_descend_speed = 100 # 制造饱和
            # 速度可接受，按高度指定下降速度
            elif self.surface_altitude > 12:
                target_descend_speed = -5 # np.clip(self.surface_altitude/100, 0, 1)*-10
            else:
                target_descend_speed = -2 # np.clip(self.surface_altitude/100, 0, 1)*-2

            # 垂直速度误差
            speed_error = self.vertical_speed-target_descend_speed
            control.throttle = np.clip(speed_error / -2, 0,1)

        # 姿态稳定
        # 半手动滚转控制
        if keyboard.is_pressed('e'):
            control.roll = 1
        elif keyboard.is_pressed('q'):
            control.roll = -1
        else:
            control.roll = self.roll_pid.calculate(-self.p *180/pi, dt=dt)
        
        # 归一化
        target_point_ = target_point_/(np.linalg.norm(target_point_)+1e-5)
        
        # # 变换，防止bx_surf与期望之间角度为钝角
        temp = copy.deepcopy(self.bx_surf)*0.9 + copy.deepcopy(target_point_) # 拷贝数值
        target_point1_ = temp/(norm(temp)+1e-5) # 重新引用

        # 1
        tmp = np.cross(self.bx_surf, target_point1_)
        # print("target_point_", target_point1_)

        # 瞎写的，效果竟然挺好
        yaw_cmd = float(np.dot(tmp, self.by_surf)*0.99999) *2
        pitch_cmd = float(np.dot(tmp, self.bz_surf)*0.99999) *2

        control.pitch = self.pitch_pid.calculate(pitch_cmd, dt=dt, d_error=-self.q)
        control.yaw = self.yaw_pid.calculate(-yaw_cmd, dt=dt, d_error=-self.r)

    # 自动缓降
    def slow_descend_controll(self, dt=0.05):
        # 读取观测数据
        self.get_state_of_NUE()
        # 接入控制
        control = self.vessel.control
        target_point_0 = np.array([0, 1, 0], dtype='float64')
        # 切断控制
        if self.surface_altitude < 3 and abs(self.vertical_speed)<1:
            control.throttle = 0
            control.pitch = 0
            control.yaw = 0
            control.roll = 0
            return
        
        # 期望下降速度
        if self.surface_altitude > 150:
            target_descend_speed = max(np.clip(self.surface_altitude/100, 0, 1)*-50, -300)
        elif self.surface_altitude > 20:
            target_descend_speed = np.clip(self.surface_altitude/100, 0, 1)*-10
        else:
            target_descend_speed = np.clip(self.surface_altitude/100, 0, 1)*-3

        # 垂直速度误差
        speed_error = self.vertical_speed-target_descend_speed
        
        if speed_error > 0:
            control.throttle = 0.0
        else:
            control.throttle = np.clip(speed_error / -10, 0,1)

        # 姿态稳定
        # 半手动滚转控制
        if keyboard.is_pressed('e'):
            control.roll = 1
        elif keyboard.is_pressed('q'):
            control.roll = -1
        else:
            control.roll = self.roll_pid.calculate(-self.p *180/pi, dt=dt)
        # 目标指向考虑减速
        v_hor = float((self.velocity_surf[0]**2 + self.velocity_surf[2]**2)**0.5) # 水平分速度大小

        target_point_ = target_point_0 - self.hor_controller.calculate(v_hor, dt=dt) \
                * np.array([self.velocity_surf[0], 0.0, self.velocity_surf[2]])/(v_hor+1e-5)
        # 归一化
        target_point_ = target_point_/(np.linalg.norm(target_point_)+1e-5)
        
        # # 变换，防止bx_surf与期望之间角度为钝角
        temp = copy.deepcopy(self.bx_surf)*0.9 + copy.deepcopy(target_point_) # 拷贝数值
        target_point_1 = temp/(norm(temp)+1e-5) # 重新引用

        # 1
        tmp = np.cross(self.bx_surf, target_point_1)
        # print("target_point_", target_point_1)

        # 瞎写的，效果竟然挺好
        yaw_cmd = float(np.dot(tmp, self.by_surf)*0.99999) *2
        pitch_cmd = float(np.dot(tmp, self.bz_surf)*0.99999) *2

        control.pitch = self.pitch_pid.calculate(pitch_cmd, dt=dt, d_error=-self.q)
        control.yaw = self.yaw_pid.calculate(-yaw_cmd, dt=dt, d_error=-self.r)

    # 定高
    def rocket_height_maintainence(self, 
                                target_height=100,
                                target_point0_ = np.array([0, 1, 0], dtype='float64'),
                                dt=0.05,
                                ):
        # 读取观测数据
        self.get_state_of_NUE()
        # 接入控制
        control = self.vessel.control
        # 半手动滚转控制
        if keyboard.is_pressed('e'):
            control.roll = 1
        elif keyboard.is_pressed('q'):
            control.roll = -1
        else:
            control.roll = self.roll_pid.calculate(-self.p *180/pi, dt=dt)
        # 目标指向考虑减速
        v_hor = float((self.velocity_surf[0]**2 + self.velocity_surf[2]**2)**0.5) # 水平分速度大小

        target_point_ = target_point0_ - self.hor_controller.calculate(v_hor, dt=dt) \
                * np.array([self.velocity_surf[0], 0.0, self.velocity_surf[2]])/(v_hor+1e-5)
        # 归一化
        target_point_ = target_point_/(np.linalg.norm(target_point_)+1e-5)
        
        # # 变换，防止bx_surf与期望之间角度为钝角
        temp = copy.deepcopy(self.bx_surf)*0.9 + copy.deepcopy(target_point_) # 拷贝数值
        target_point1_ = temp/(norm(temp)+1e-5) # 重新引用

        # 1
        tmp = np.cross(self.bx_surf, target_point1_)
        # print("target_point_", target_point1_)

        # 瞎写的，效果竟然挺好
        yaw_cmd = float(np.dot(tmp, self.by_surf)*0.99999) *2
        pitch_cmd = float(np.dot(tmp, self.bz_surf)*0.99999) *2

        # # 按键映射覆盖：w/s 控制 pitch，a/d 控制 yaw，e/q 控制 roll，z/x 控制油门
        # if keyboard.is_pressed('w'):
        #     control.pitch = -1
        # elif keyboard.is_pressed('s'):
        #     control.pitch = 1
        # else:
        control.pitch = self.pitch_pid.calculate(pitch_cmd, dt=dt, d_error=-self.q)

        # if keyboard.is_pressed('a'):
        #     control.yaw = -1
        # elif keyboard.is_pressed('d'):
        #     control.yaw = 1
        # else:
        control.yaw = self.yaw_pid.calculate(-yaw_cmd, dt=dt, d_error=-self.r)

        # 自动定高 PID 输出油门
        throttle_cmd = self.height_controller.calculate(target_height - self.surface_altitude, dt=dt)
        # 头朝下的时候强制收油门
        if self.direction_head[1]<0:
            throttle_cmd=0
        if keyboard.is_pressed('z'):
            control.throttle = 1.0
            control.yaw = 0.0
            control.pitch = 0.0

        elif keyboard.is_pressed('x'):
            control.throttle = 0.0
        else:
            control.throttle = float(np.clip(throttle_cmd, 0.0, 1.0))
        # print(f"yaw_cmd={yaw_cmd:.3f}, pitch_cmd={pitch_cmd:.3f}, throttle={control.throttle:.3f}")

if __name__ == '__main__':
    dt = 0.05
    print("开始运行")

    # # 单飞行器
    # vessel = conn.space_center.active_vessel
    # Rocket = rocket_control(vessel)
    # for i in range(int(60*5/dt)):
    #     # Rocket.rocket_height_maintainence(target_height=100, dt=dt)
    #     Rocket.fast_landing_controll(dt=dt)
    #     time.sleep(dt)
    
    # 多飞行器
    vessels = space_center.vessels
    name_list = ['testship2', 'testship3', 'testship4']
    Rocket_list = []
    # 循环加载所有飞行器
    for vessel in vessels:
        if vessel.name in name_list:
            Rocket = rocket_control(vessel)
            Rocket_list.append(Rocket)

    # # 遍历控制
    for i in range(int(40/dt)):
        t = float(i)*dt
        if t < 10:
            for i, Rocket in enumerate(Rocket_list):
                Rocket.rocket_height_maintainence(target_height=100, dt=dt)
        else:
            for i, Rocket in enumerate(Rocket_list):
                Rocket.fast_landing_controll(dt=dt)
        time.sleep(dt)