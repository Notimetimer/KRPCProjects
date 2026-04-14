# 待续 定高时的速度控制器直接控水平速度很容易震荡，
# 更换气压高度仍然难以解决，怀疑应该先控加速度再控速度

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

# 新增导入
import tkinter as tk
from tkinter import ttk
import threading
from collections import deque
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

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
        # 下降姿态环
        self.pointing_pid = PositionPID(max=0.4, min=0, p=0.12, i=0, d=0.24) # max=0.4, min=0, p=0.12, i=0, d=0.09
        # 减摇姿态环
        self.stable_pointing_pid = PositionPID(max=0.3, min=0, p=0.1, i=0, d=0.07)
        # 快速姿态环
        self.fast_pointing_pid = PositionPID(max=1.0, min=0, p=1.0, i=0, d=0.7)

        # 水平加速度环
        self.hor_acc_pid = PositionPID(max=2.0, min=0, p=2.0, i=0.001, d=2.0) # p=0.2, i=0.001, d=0.14
        # 水平速度环
        self.hor_speed_pid = PositionPID(max=5.0, min=0, p=5.0, i=0.001, d=5.0) # p=0.2, i=0.001, d=0.14
        # 编队水平相对距离环
        self.hor_rel_pos_pid = PositionPID(max=1.0, min=0, p=0.5, i=0, d=1.0)

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
        
        # 编队保持目标位移（相对于虚拟中心）
        self.formation_offset_N = 0.0
        self.formation_offset_E = 0.0

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
            # 补充一次归一化
            surface_axes[axis_name] = np.array(surface_axes[axis_name]) / (norm(surface_axes[axis_name]) + 1e-5)
        body_axes_custom['x'] = UNE2NUE(surface_axes['forward'])
        body_axes_custom['y'] = - UNE2NUE(surface_axes['down'])
        body_axes_custom['z'] = UNE2NUE(surface_axes['right'])
        # 姿态控制：基于 body_axes_custom 与 target_point_
        self.bx = np.array(body_axes_custom['x'])
        self.by = np.array(body_axes_custom['y'])
        self.bz = np.array(body_axes_custom['z'])
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
        self.surface_altitude = self.vessel.flight(self.surface_frame).surface_altitude # 雷达高度
        self.altitude_sea_level = self.vessel.flight(self.surface_frame).mean_altitude # 海平面高度
        self.radius_from_center = self.vessel.orbit.radius # 球心距离
        # 垂直速度（米/秒）  
        self.vertical_speed = vessel.flight(self.Globe_frame).vertical_speed
        # 重量
        self.mass = self.vessel.mass
        self.g = self.vessel.orbit.body.surface_gravity
        # 转动惯量
        # 获取载具的转动惯量 (pitch, roll, yaw)  
        self.moment_of_inertia = self.vessel.moment_of_inertia  # 返回三个轴向的转动惯量，单位为 kg.m²
        # 获取完整的惯性张量 (3x3矩阵，行主序)  
        self.inertia_tensor = self.vessel.inertia_tensor  # 返回9个元素的列表

        # 当前推重比
        self.current_twr = self.vessel.thrust / (self.mass * self.g)
        # 最大推重比
        self.max_twr = self.vessel.available_thrust / (self.mass * self.g)
        # 获取发动机的可用力矩  
        self.engine = self.vessel.parts.engines[0]  
        self.torque = self.engine.available_torque  # 返回 (pitch, roll, yaw) 的力矩值，单位为 N.m
        # 载具总可用力矩  
        self.total_torque = self.vessel.available_torque  # 所有部件的总力矩  
        self.engine_torque = self.vessel.available_engine_torque  # 仅发动机力矩

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
        self.pitch_rad = self.pitch_angle * pi/180
        self.heading_rad = self.heading_angle * pi/180
        self.roll_rad = self.roll_angle * pi/180

    def get_displacement_to_center(self, center_lat, center_lon):
        """
        计算飞行器相对于指定中心点的北向和东向位移 (单位: 米)
        计算公式符合用户要求：不考虑高度，基于经纬度的水平位移
        """
        # 获取两个点的世界坐标位置 (Sea Level)
        pos_center = self.GravSource.surface_position(center_lat, center_lon, self.Globe_frame)
        pos_vessel = self.GravSource.surface_position(self.latitude, self.longitude, self.Globe_frame)

        # 转换到表面参考帧 (x=East, y=North, z=Up)
        pos_center_surface = conn.space_center.transform_position(pos_center, self.Globe_frame, self.surface_frame)
        pos_vessel_surface = conn.space_center.transform_position(pos_vessel, self.Globe_frame, self.surface_frame)
        
        # 计算位移 (Vessel - Center)
        # 北向位移 (index 1)
        disp_N = pos_vessel_surface[1] - pos_center_surface[1]
        # 东向位移 (index 0)
        disp_E = pos_vessel_surface[0] - pos_center_surface[0]
        
        return disp_N, disp_E
    
    def shut_down(self, dt=0.05):
        self.control.throttle = 0
        self.control.pitch = 0
        self.control.yaw = 0
        self.control.roll = 0
    
    def empty_control(self, dt=0.05):
        pass

    # 自动快速降落
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
        # 只考虑垂直速度
        # h_max_thrust = (self.vertical_speed**2)/(2*self.g*(self.max_twr * sin(self.pitch_rad) - 1)) * 1.2 # 安全高度倍率
        # 用总速度估算
        h_max_thrust = (self.speed_surf**2)/(2*self.g*(self.max_twr * sin(self.pitch_rad) - 1)) * 1.2 # 安全高度倍率

        # print("垂直速度", self.vertical_speed)
        # print("对地速度", self.speed_surf)
        # print("当前高度", self.surface_altitude)
        # print("最大推力高度", h_max_thrust)
        # print()

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

            # 目标指向考虑减速
            v_hor = float((self.velocity_surf[0]**2 + self.velocity_surf[2]**2)**0.5) # 水平分速度大小
    
            target_point0_ = - self.velocity_surf / (self.speed_surf + 1e-5)
            if self.surface_altitude > 30:
                target_point0_[1] = max(target_point0_[1], 0.3)
            elif self.surface_altitude > 10:
                target_point0_[1] = max(target_point0_[1], 5.0)
            else:
                target_point0_[1] = max(target_point0_[1], 10.0)

            target_point_ = target_point0_/(np.linalg.norm(target_point0_)+1e-5) - self.pointing_pid.calculate(v_hor, dt=dt) \
                    * np.array([self.velocity_surf[0], 0.0, self.velocity_surf[2]])/(v_hor+1e-5)
            
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
        
        # # 变换，防止bx与期望之间角度为钝角
        temp = copy.deepcopy(self.bx)*0.9 + copy.deepcopy(target_point_) # 拷贝数值
        target_point1_ = temp/(norm(temp)+1e-5) # 重新引用

        # 1
        tmp = np.cross(self.bx, target_point1_)
        # print("target_point_", target_point1_)

        # 瞎写的，效果竟然挺好
        yaw_cmd = float(np.dot(tmp, self.by)*0.99999) *2
        pitch_cmd = float(np.dot(tmp, self.bz)*0.99999) *2

        control.pitch = self.pitch_pid.calculate(pitch_cmd, dt=dt, d_error=-self.q)
        control.yaw = self.yaw_pid.calculate(-yaw_cmd, dt=dt, d_error=-self.r)

    # 定高
    def rocket_height_maintainence(self, 
                                target_height=100,
                                target_N_speed = 0.0,
                                target_E_speed = 0.0,
                                dt=0.05,
                                type="simple",
                                # 新增中心点参数
                                current_avg_lat = None,
                                current_avg_lon = None,
                                ):
        # 读取观测数据
        self.get_state_of_NUE()

        # --- 编队保持逻辑 ---
        if current_avg_lat is not None and current_avg_lon is not None:
            # 计算当前相对于最新中心的位移
            curr_dn, curr_de = self.get_displacement_to_center(current_avg_lat, current_avg_lon)
            
            # 误差 = 目标位移 - 当前位移
            err_n = self.formation_offset_N - curr_dn
            err_e = self.formation_offset_E - curr_de
            err_hor = (err_n**2 + err_e**2)**0.5
            
            # 计算速度补偿：位移误差 50m 时饱和为 20m/s (系数 0.4)
            err_hor_normal = 50 * self.hor_rel_pos_pid.calculate(err_hor/50, dt=dt)

            v_n_comp = np.clip(0.4 * err_n / (err_hor+1e-5) * err_hor_normal, -20.0, 20.0)
            v_e_comp = np.clip(0.4 * err_e / (err_hor+1e-5) * err_hor_normal, -20.0, 20.0)
            
            # 在期望速度基础上叠加补偿量
            target_N_speed += v_n_comp
            target_E_speed += v_e_comp
        # --------------------

        # 接入控制
        control = self.vessel.control
        # 半手动滚转控制
        if keyboard.is_pressed('e'):
            control.roll = 1
        elif keyboard.is_pressed('q'):
            control.roll = -1
        else:
            control.roll = self.roll_pid.calculate(-self.p *180/pi, dt=dt)

        # 最小允许俯仰角正切值
        min_tan_theta_allowed = np.sqrt(self.max_twr**2 - 1)

        # 根据水平速度误差调节倾斜方向和大小
        target_point0_ = np.array([0, 1, 0], dtype='float64')
        v_N = self.velocity_surf[0]
        v_E = self.velocity_surf[2]

        # 速度环误差
        v_N_error = v_N - target_N_speed
        v_E_error = v_E - target_E_speed
        v_hor_error = np.sqrt(v_N_error**2 + v_E_error**2)

        if type == "simple":
            "无加速度环期望姿态"
            if v_hor_error < 5.0:
                # 需要更为平稳
                target_point_ = target_point0_ - self.stable_pointing_pid.calculate(v_hor_error, dt=dt) \
                    * np.array([v_N_error, 0.0, v_E_error])/(v_hor_error+1e-5)
            else:
                # 需要快速
                target_point_ = target_point0_ - \
                    min(min_tan_theta_allowed, self.fast_pointing_pid.calculate(v_hor_error, dt=dt)) \
                    * np.array([v_N_error, 0.0, v_E_error])/(v_hor_error+1e-5)
        else:
            "带加速度环期望姿态"
            # 计算当前水平加速度分量（使用推力投影，未考虑空气阻力）
            a_N = self.current_twr * self.g * self.bx[0]
            a_E = self.current_twr * self.g * self.bx[2]
            # 期望加速度
            target_a_hor = - self.hor_speed_pid.calculate(v_hor_error, dt=dt)
            target_N_a = target_a_hor * v_N_error/v_hor_error
            target_E_a = target_a_hor * v_E_error/v_hor_error
            # 加速度环误差
            a_N_error = a_N - target_N_a
            a_E_error = a_E - target_E_a
            a_hor_error = np.sqrt(a_N_error**2 + a_E_error**2)
            # 期望指向计算
            target_point_hor0_ = np.array([a_N_error, 0.0, a_E_error])
            target_point_hor_ = - self.hor_acc_pid.calculate(a_hor_error, dt=dt) * \
                target_point_hor0_/(norm(target_point_hor0_)+1e-5)
            # 加速度环后指向控制
            target_point_ = target_point0_ + target_point_hor_ * \
                min(min_tan_theta_allowed, self.pointing_pid.calculate(v_hor_error, dt=dt)*norm(target_point_hor_))/ \
                (norm(target_point_hor_)+1e-5)

        "姿态控制"    
        # 归一化
        target_point_ = target_point_/(np.linalg.norm(target_point_)+1e-5)
        
        # # 变换，防止bx与期望之间角度为钝角
        temp = copy.deepcopy(self.bx)*0.9 + copy.deepcopy(target_point_) # 拷贝数值
        target_point1_ = temp/(norm(temp)+1e-5) # 重新引用

        # 1
        tmp = np.cross(self.bx, target_point1_)
        # print("target_point_", target_point1_)

        # 用sin值做近似
        # yaw_cmd = float(np.dot(tmp, self.by)*0.99999) *2
        # pitch_cmd = float(np.dot(tmp, self.bz)*0.99999) *2
        # # 直接用角度
        yaw_cmd = np.arcsin(float(np.dot(tmp, self.by)*0.99999)) * 2/pi
        pitch_cmd = np.arcsin(float(np.dot(tmp, self.bz)*0.99999)) * 2/pi

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
        throttle_cmd = self.height_controller.calculate(target_height - self.altitude_sea_level, dt=dt)

        # 头朝下的时候强制收油门
        if self.direction_head[1]<0:
            throttle_cmd=0.01 # 不能全收，留一点用来转向

        # 预计爬升会飞过的时候关小火只保留姿态控制功能
        if self.vertical_speed > 0 and target_height > self.altitude_sea_level: 
            max_h_predict = self.altitude_sea_level + (self.vertical_speed)**2 / (2*self.g)
            if max_h_predict - target_height > 5:
                throttle_cmd=0.01 # 预计会飞过的时候关小火只保留姿态控制
        # 预计下降会飞过的时候满油门减速
        if self.vertical_speed < 0 and\
            target_height < self.altitude_sea_level and\
                self.pitch_angle > 0:
            # 启动最大推力高度
            h_max_thrust = target_height + (self.vertical_speed**2)/ \
                (2*self.g*(self.max_twr * sin(self.pitch_rad) - 1)) * 1.2 # 安全高度倍率
            if self.altitude_sea_level > h_max_thrust + 5:
                throttle_cmd=max(0.01, throttle_cmd) # 至少要保留姿态控制
            elif self.altitude_sea_level - target_height > 5:
                throttle_cmd=1.0

        if keyboard.is_pressed('z'):
            control.throttle = 1.0
            control.yaw = 0.0
            control.pitch = 0.0

        elif keyboard.is_pressed('x'):
            control.throttle = 0.0
        else:
            control.throttle = float(np.clip(throttle_cmd, 0.0, 1.0))
        # print(f"yaw_cmd={yaw_cmd:.3f}, pitch_cmd={pitch_cmd:.3f}, throttle={control.throttle:.3f}")

# 新增界面类
class RocketControlGUI:
    def __init__(self, root, rocket_list):
        self.root = root
        self.rocket_list = rocket_list
        self.root.title("KSP 编队多线程控制终端")
        self.root.geometry("950x530") # 扩大窗口以容纳曲线图
        self.root.attributes("-topmost", True) # 窗口置顶，方便在 KSP 运行期间观察
        
        # 状态变量
        self.current_mode = tk.StringVar(value="CUT")  # 默认切断控制
        self.target_height_var = tk.StringVar(value="200.0")
        self.target_N_speed_var = tk.StringVar(value="0.0")
        self.target_E_speed_var = tk.StringVar(value="0.0")
        self.dt = 0.05
        
        # 编队数据状态
        self.avg_lat = 0.0
        self.avg_lon = 0.0
        
        # --- 修复点：必须先定义 running，后面的线程才能跑起来 ---
        self.running = True 
        # ----------------------------------------------------
        
        # 绘图数据准备 (10秒窗口)
        self.plot_len = 200 # 预估点数
        self.data_time = deque(maxlen=self.plot_len)
        self.data_pitch = deque(maxlen=self.plot_len)
        self.data_yaw = deque(maxlen=self.plot_len)
        self.data_roll = deque(maxlen=self.plot_len)
        self.data_throttle = deque(maxlen=self.plot_len)
        self.start_time = time.time()
        self.plotted_rocket = self.rocket_list[0] if self.rocket_list else None

        self.setup_ui()
        
        # --- 核心改进：为每艘船分配独立线程 ---
        self.control_threads = []
        for rocket in self.rocket_list:
            t = threading.Thread(
                target=self.single_rocket_thread, 
                args=(rocket,), 
                daemon=True
            )
            t.start()
            self.control_threads.append(t)
        
        # 启动全局计算逻辑线程 (计算虚拟中心及记录位移)
        threading.Thread(target=self.global_formation_thread, daemon=True).start()

    def adjust_value(self, var, increment):
        """通用增量调节函数"""
        try:
            val = float(var.get() or 0)
            var.set(f"{val + increment:.1f}")
        except ValueError:
            pass

    def setup_ui(self):
        # 创建主分栏容器
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill="both", expand=True)
        
        # 左侧控制面板
        self.frame_left = ttk.Frame(main_frame, padding=5)
        self.frame_left.pack(side="left", fill="y", padx=5)

        style = ttk.Style()
        style.configure("TLabel", font=("Microsoft YaHei", 10))
        
        # 设定高度/速度参数
        frame_input = ttk.LabelFrame(self.frame_left, text=" 参数设定 ", padding=15)
        frame_input.pack(fill="x", padx=15, pady=10)
        
        # 目标海拔高度
        row_h = ttk.Frame(frame_input)
        row_h.pack(fill="x", pady=2)
        ttk.Label(row_h, text="目标海拔高度 (m):").pack(side="left")
        
        btn_frame_h = tk.Frame(row_h)
        btn_frame_h.pack(side="right")
        tk.Button(btn_frame_h, text="+", width=2, font=("Arial", 6), command=lambda: self.adjust_value(self.target_height_var, 20)).pack(side="top", fill="x")
        tk.Button(btn_frame_h, text="-", width=2, font=("Arial", 6), command=lambda: self.adjust_value(self.target_height_var, -20)).pack(side="bottom", fill="x")
        
        self.entry_height = ttk.Entry(row_h, textvariable=self.target_height_var, width=8)
        self.entry_height.pack(side="right", padx=5)

        # 目标北向速度
        row_n = ttk.Frame(frame_input)
        row_n.pack(fill="x", pady=2)
        ttk.Label(row_n, text="北向速度 (m/s):").pack(side="left")
        
        btn_frame_n = tk.Frame(row_n)
        btn_frame_n.pack(side="right")
        tk.Button(btn_frame_n, text="+", width=2, font=("Arial", 6), command=lambda: self.adjust_value(self.target_N_speed_var, 5)).pack(side="top", fill="x")
        tk.Button(btn_frame_n, text="-", width=2, font=("Arial", 6), command=lambda: self.adjust_value(self.target_N_speed_var, -5)).pack(side="bottom", fill="x")
        
        self.entry_N = ttk.Entry(row_n, textvariable=self.target_N_speed_var, width=8)
        self.entry_N.pack(side="right", padx=5)

        # 目标东向速度
        row_e = ttk.Frame(frame_input)
        row_e.pack(fill="x", pady=2)
        ttk.Label(row_e, text="东向速度 (m/s):").pack(side="left")
        
        # 恢复左右排布，但保持极小尺寸
        tk.Button(row_e, text="+", width=2, font=("Arial", 6), command=lambda: self.adjust_value(self.target_E_speed_var, 5)).pack(side="right", padx=1)
        tk.Button(row_e, text="-", width=2, font=("Arial", 6), command=lambda: self.adjust_value(self.target_E_speed_var, -5)).pack(side="right", padx=1)
        
        self.entry_E = ttk.Entry(row_e, textvariable=self.target_E_speed_var, width=8)
        self.entry_E.pack(side="right", padx=5)

        # 模式控制开关
        frame_btns = ttk.LabelFrame(self.frame_left, text=" 指令序列 ", padding=15)
        frame_btns.pack(fill="both", expand=True, padx=15, pady=10)
        # 模式按钮配置
        modes = [
            ("P1: 自动定高模式", "HEIGHT", "#e8f5e9", "#4CAF50"), # 绿色
            ("P2: 自动降落程序", "LANDING", "#e3f2fd", "#2196F3"),# 蓝色
            ("P0: 手动/切断控制", "CUT", "#ffebee", "#F44336"),    # 红色
        ]
        for text, mode_val, bg, active_fg in modes:
            btn = tk.Radiobutton(frame_btns, text=text, 
                                variable=self.current_mode, value=mode_val,
                                indicatoron=0, # 变成按钮外观
                                selectcolor=active_fg, # 选中时的颜色
                                font=("Microsoft YaHei", 10, "bold"),
                                padx=20, pady=12,
                                cursor="hand2")
            btn.pack(pady=8, fill="x")
        self.status_label = ttk.Label(self.frame_left, text="系统已就绪 | 实时绘图激活", foreground="gray")
        self.status_label.pack(side="bottom", pady=15)

        # --- 右侧绘图板 ---
        self.frame_right = ttk.Frame(main_frame, padding=5)
        self.frame_right.pack(side="right", fill="both", expand=True)
        
        plt.rcParams['font.sans-serif'] = ['SimHei'] # 解决中文显示
        plt.rcParams['axes.unicode_minus'] = False
        
        self.fig, self.ax = plt.subplots(figsize=(6, 4))
        self.ax.set_ylim(-1.1, 1.1)
        self.ax.set_title("实时飞行控制指令反馈 (10s)")
        self.ax.set_xlabel("时间 (s)")
        self.ax.grid(True, alpha=0.3)
        
        self.line_p, = self.ax.plot([], [], label="Pitch", color="#FF5252")
        self.line_y, = self.ax.plot([], [], label="Yaw", color="#448AFF")
        self.line_r, = self.ax.plot([], [], label="Roll", color="#4CAF50")
        self.line_t, = self.ax.plot([], [], label="Throttle", color="#FFA000", linewidth=2)
        
        self.ax.legend(loc="upper right", frameon=True, fontsize='small')
        
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.frame_right)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)
        
        self.plot_timer_id = None # 记录定时器 ID 以便取消
        # 绑定窗口关闭协议
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        
        # 启动动态刷新
        self.plot_timer_id = self.root.after(100, self.update_plot_ui)

    def on_closing(self):
        """处理窗口关闭事件，确保线程安全退出"""
        print("[系统] 正在关闭监控中心...")
        self.running = False
        if self.plot_timer_id:
            try:
                self.root.after_cancel(self.plot_timer_id)
            except:
                pass
        plt.close(self.fig) # 清理 matplotlib 资源
        self.root.destroy()
        print("[系统] 资源已释放，程序正常退出")

    def update_plot_ui(self):
        """定时刷新曲线图"""
        if not self.running:
            return

        if self.plotted_rocket:
            try:
                t_now = time.time() - self.start_time
                ctrl = self.plotted_rocket.control
                
                # 数据入队
                self.data_time.append(t_now)
                self.data_pitch.append(ctrl.pitch)
                self.data_yaw.append(ctrl.yaw)
                self.data_roll.append(ctrl.roll)
                self.data_throttle.append(ctrl.throttle)
                
                # 更新曲线数据
                self.line_p.set_data(self.data_time, self.data_pitch)
                self.line_y.set_data(self.data_time, self.data_yaw)
                self.line_r.set_data(self.data_time, self.data_roll)
                self.line_t.set_data(self.data_time, self.data_throttle)
                
                # 动态调整 X 轴
                self.ax.set_xlim(max(0, t_now - 10), t_now + 0.5)
                
                self.canvas.draw()
            except Exception as e:
                pass
        
        # 递归调用
        if self.running:
            self.plot_timer_id = self.root.after(100, self.update_plot_ui)

    def global_formation_thread(self):
        """后台线程：实时计算所有活着的飞行器的虚拟几何中心"""
        while self.running:
            try:
                # 只有在 CUT 模式下才更新中心点，锁定编队位置
                # 这样当你切换到 HEIGHT 模式时，位置目标就不再随受控物体的晃动而晃动，实现了真正的“定点”
                if self.current_mode.get() == "CUT":
                    lats = []
                    lons = []
                    active_ctrls = []
                    
                    # 遍历识别活着的船
                    for r in self.rocket_list:
                        try:
                            # 检查船只是否仍有效 (KRPC 抛错则认为已炸)
                            lat = r.vessel.flight(r.surface_frame).latitude
                            lon = r.vessel.flight(r.surface_frame).longitude
                            lats.append(lat)
                            lons.append(lon)
                            active_ctrls.append(r)
                        except:
                            continue
                    
                    if lats:
                        # 计算虚拟几何中心 (不考虑高度的平均经纬度)
                        self.avg_lat = sum(lats) / len(lats)
                        self.avg_lon = sum(lons) / len(lons)
                        
                        # 关键逻辑：处于 CUT 模式时，实时记录并更新每个船的相对位移作为目标
                    for r in active_ctrls:
                        # 提前更新一次状态以获得最新的 latitude/longitude
                        r.get_state_of_NUE()
                        dn, de = r.get_displacement_to_center(self.avg_lat, self.avg_lon)
                        r.formation_offset_N = dn
                        r.formation_offset_E = de
                else:
                    # 非 CUT 模式下，avg_lat/lon 将保持为进入控制前最后一次记录的值
                    pass
            except Exception as e:
                pass 
            
            time.sleep(0.1) # 10Hz 更新中心点足够
    
    def single_rocket_thread(self, r):
        """
        单个火箭的独立控制逻辑。
        即使 r 船炸了，报错也只会被这个线程捕获并退出，不影响主程序和其他船。
        """
        print(f"[系统] 线程启动：开始监控飞行器 -> {r.name}")
        
        while self.running:
            # --- 记录本轮控制开始的绝对时间 ---
            start_tick = time.time()
            
            mode = self.current_mode.get()
            try:
                # 统一参数读取
                try: h = float(self.target_height_var.get())
                except: h = 200.0
                try: vn = float(self.target_N_speed_var.get())
                except: vn = 0.0
                try: ve = float(self.target_E_speed_var.get())
                except: ve = 0.0
                
                # 根据模式执行对应的控制逻辑
                if mode == "HEIGHT":
                    # 传入当前中心点经纬度以启用队形保持
                    r.rocket_height_maintainence(
                        target_height=h, 
                        target_N_speed=vn, 
                        target_E_speed=ve, 
                        dt=self.dt,
                        current_avg_lat=self.avg_lat,
                        current_avg_lon=self.avg_lon
                    )
                elif mode == "LANDING":
                    r.fast_landing_controll(dt=self.dt)
                elif mode == "CUT":
                    r.empty_control(dt=self.dt)
                
                # --- 精确时间步补偿 ---
                # 计算代码运行时间（包含网络排队时间）
                elapsed = time.time() - start_tick
                # 动态计算需要补多少 sleep，确保总周期严格等于 self.dt
                # 这样即使 A 船排队多花点时间，B 船也会在紧随其后的固定时间点补回来
                sleep_time = max(0.001, self.dt - elapsed)
                time.sleep(sleep_time)

            except Exception as e:
                print(f"\n[警告] 飞行器 {r.name} 信道异常: {e}")
                # 报错后跳出，不影响主连接上的其他飞行器线程
                break 


if __name__ == '__main__':
    dt = 0.05
    print("正在建立 KRPC 通信...")
    
    # 自动识别列表中的船只
    vessels = space_center.vessels
    name_list = ['testship1', 'testship2']
    # ['testship1', 'testship2', 'testship3'] 
    ctrl_list = []
    
    for v in vessels:
        if v.name in name_list:
            ctrl_list.append(rocket_control(v))
    
    # 如果没找到列表中的，默认控制当前飞船
    if not ctrl_list:
        print("未检测到预设船只，切换至当前活动飞行器")
        ctrl_list.append(rocket_control(conn.space_center.active_vessel))
    # 启动 GUI 主循环
    root = tk.Tk()
    app = RocketControlGUI(root, ctrl_list)
    root.mainloop()

    # # 单飞行器
    # vessel = conn.space_center.active_vessel
    # Rocket = rocket_control(vessel)
    # for i in range(int(60*5/dt)):
    #     # Rocket.rocket_height_maintainence(target_height=100, dt=dt)
    #     Rocket.fast_landing_controll(dt=dt)
    #     time.sleep(dt)
    
    # # 多飞行器
    # vessels = space_center.vessels
    # name_list = ['testship2', 'testship3', 'testship4']
    # Rocket_list = []
    # # 循环加载所有飞行器
    # for vessel in vessels:
    #     if vessel.name in name_list:
    #         Rocket = rocket_control(vessel)
    #         Rocket_list.append(Rocket)

    # # # 遍历控制
    # for i in range(int(40/dt)):
    #     t = float(i)*dt
    #     if t < 10:
    #         for i, Rocket in enumerate(Rocket_list):
    #             Rocket.rocket_height_maintainence(target_height=100, dt=dt)
    #     else:
    #         for i, Rocket in enumerate(Rocket_list):
    #             Rocket.fast_landing_controll(dt=dt)
    #     time.sleep(dt)