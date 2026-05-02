from math import *
import numpy as np
from numpy.linalg import norm, inv
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
import time

from tools import *

"""
规定一下正方向：
x轴指向末梢，
主要转轴方向为z，
基座以上为转轴，同时也是z轴方向
"""


# 先在牵连/惯性系算了
class MultiArms:
    def __init__(self, 
        lengths_of_arms,
        ):
        self.lengths0 = lengths_of_arms
        self.joint0_ = np.array([0.0, 0.0, 0.0])

    def reset(self):
        # 恢复到初始固定样式 (直线伸展)
        self.lengths = self.lengths0.copy()
        self.delta_phis = np.zeros(len(self.lengths0))
        self.delta_thetas = np.zeros(len(self.lengths0))
        self.delta_psis = np.zeros(len(self.lengths0))
        self.forward_kinematics()

    def set_state(self,
        delta_lengths,
        delta_phis,
        delta_thetas,
        delta_psis,
        ):
        # 设置特定状态并更新运动学
        self.lengths = self.lengths0 + delta_lengths
        self.delta_phis = delta_phis.copy()
        self.delta_thetas = delta_thetas.copy()
        self.delta_psis = delta_psis.copy()
        self.forward_kinematics()

    def forward_kinematics(self):
        # 重新计算整个运动链
        self.joint_s = [self.joint0_]
        self.x_s = [np.array([1, 0, 0])]
        self.y_s = [np.array([0, 1, 0])]
        self.z_s = [np.array([0, 0, 1])]
        
        prev_joint_ = self.joint0_
        prev_Rotation_matrix = np.eye(3)
        # 循环计算
        for i in range(len(self.lengths0)):
            # 每个关节相对于上一级的旋转
            Rotation_i = RotMat_zyx(self.delta_phis[i], self.delta_thetas[i], self.delta_psis[i])
            # 累积旋转矩阵 (从基座到当前关节)
            Rotation_matrix = Rotation_i @ prev_Rotation_matrix
            
            # 计算当前杆件在全局坐标系下的末端位置
            curr_delta_x = np.array([self.lengths[i], 0, 0])
            current_joint_ = prev_joint_ + left_multiply(Rotation_matrix.T, curr_delta_x)
            
            self.joint_s.append(current_joint_)
            # 记录当前关节的局部坐标轴在全局坐标系下的方向
            self.x_s.append(left_multiply(Rotation_matrix.T, np.array([1, 0, 0])))
            self.y_s.append(left_multiply(Rotation_matrix.T, np.array([0, 1, 0])))
            self.z_s.append(left_multiply(Rotation_matrix.T, np.array([0, 0, 1])))
            
            prev_Rotation_matrix = Rotation_matrix
            prev_joint_ = current_joint_
            
    def move_wt_f(self, dt, 
        length_dots,
        phi_dots,
        theta_dots,
        psi_dots,):
        # 仅考虑运动学的前进
        prev_joint_ = self.joint0_
        prev_Rotation_matrix = np.eye(3)
        # 循环计算
        for i in range(len(self.lengths0)):
            # 计算下一个节点相对上一个节点的位置
            self.lengths[i] += length_dots[i] * dt
            # 根据角速度前进
            self.delta_phis[i] += phi_dots[i] * dt
            self.delta_thetas[i] += theta_dots[i] * dt
            self.delta_psis[i] += psi_dots * dt
            Rotation_i = RotMat_zyx(self.delta_phis[i], self.delta_thetas[i], self.delta_psis[i])
            # 处理累积旋转矩阵
            Rotation_matrix = Rotation_i @ prev_Rotation_matrix
            curr_delta_x = np.array([self.lengths[i], 0, 0])
            current_joint_ = prev_joint_ + left_multiply(Rotation_matrix.T, curr_delta_x)
            
            self.joint_s.append(current_joint_)
            # 记录当前关节的局部坐标轴在全局坐标系下的方向
            self.x_s[i] = left_multiply(Rotation_matrix.T, np.array([1, 0, 0]))
            self.y_s[i] = left_multiply(Rotation_matrix.T, np.array([0, 1, 0]))
            self.z_s[i] = left_multiply(Rotation_matrix.T, np.array([0, 0, 1]))

            prev_Rotation_matrix = Rotation_matrix
            prev_joint_ = current_joint_


    def render(self, ax):
        # 将关节坐标转换为 numpy 数组方便切片
        joints = np.array(self.joint_s)
        
        # 1. 绘制机械臂连杆（用三维线段的plot方法）
        ax.plot(joints[:, 0], joints[:, 1], joints[:, 2], 
                color='black', linewidth=4, marker='o', markersize=6, label='Robot Arm')
        
        # 2. 绘制每个关节处的坐标轴（用quiver更好）
        axis_length = 2.0  # 坐标轴显示长度
        for i in range(len(self.joint_s)):
            if i==0:
                continue
            pos = self.joint_s[i-1]
            # x轴 (红)
            ax.quiver(pos[0], pos[1], pos[2], self.x_s[i][0], self.x_s[i][1], self.x_s[i][2], 
                      color='r', length=axis_length, normalize=True)
            # y轴 (绿)
            ax.quiver(pos[0], pos[1], pos[2], self.y_s[i][0], self.y_s[i][1], self.y_s[i][2], 
                      color='g', length=axis_length, normalize=True)
            # z轴 (蓝)
            ax.quiver(pos[0], pos[1], pos[2], self.z_s[i][0], self.z_s[i][1], self.z_s[i][2], 
                      color='b', length=axis_length, normalize=True)
        
        # 3. 使用自定义的函数让坐标轴等比例
        set_axes_equal(ax)
        
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
            



if __name__=='__main__':
    # 多段机械臂建模
    lengths_of_arms0 = np.array([7.0, 5.0, 3.0, 2.0])
    delta_lengths0 = np.array([0, 0, 0, 0])
    
    # # 设置一些旋转角度来测试 (弧度)
    delta_phis0 = np.radians([0, 30, 30, 30])
    delta_thetas0 = np.radians([0, 0, 0, 0])
    delta_psis0 = np.radians([0, 90, 0, 90])
    arms1 = MultiArms(lengths_of_arms0)
    
    # 创建 3D 图形
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    start_time = time.time()

    def update(frame):
        # 计算现实中过去的时间 (s)
        t = time.time() - start_time
        
        # 初始化角度数组
        phis = np.zeros(4)
        thetas = np.zeros(4)
        psis = np.zeros(4)
        
        # 运动逻辑设定:
        # 1. 臂1 和 臂2：在 psi 方向做缓慢的正弦运动
        psis[0] = delta_psis0[0] + np.radians(20) * sin(0.5 * t)
        psis[1] = delta_psis0[1] + np.radians(30) * sin(0.5 * t)
        
        # 2. 臂3：在 theta 方向做正弦运动
        psis[2] = delta_psis0[2] + np.radians(20) * sin(0.5 * t)

        # 3. 臂4：psi 锁定为 90°
        psis[3] = np.radians(90)
        phis[3] = delta_phis0[3] + 5.0 * t
        
        
        # 使用 set_state 更新状态，不破坏 reset 的含义
        delta_lengths = np.zeros(4)
        arms1.set_state(delta_lengths, phis, thetas, psis)
        
        # 渲染
        ax.cla()
        arms1.render(ax)
        
        # 设置范围
        total_len = np.sum(lengths_of_arms0)
        ax.set_xlim([-total_len, total_len])
        ax.set_ylim([-total_len, total_len])
        ax.set_zlim([-total_len, total_len])
        
        ax.set_title(f"Robot Arm Real-time Animation | Time: {t:.2f}s")
        ax.legend(["Arm Segments"])

    # 创建动画
    ani = animation.FuncAnimation(fig, update, interval=30)
    
    plt.show()
