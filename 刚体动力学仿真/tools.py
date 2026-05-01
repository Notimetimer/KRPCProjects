from math import *
import numpy as np
from numpy.linalg import norm, inv
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# --- 核心算法类 ---
def left_multiply(M, v):
    # 输入行向量和矩阵，当做列向量左乘矩阵来算
    return v @ (M.T)

def RotMat_zyx(phi, theta, psi):
    R_phi = np.array([
        [1, 0, 0],
        [0, cos(phi), sin(phi)],
        [0, -sin(phi), cos(phi)],
    ])
    R_theta = np.array([
        [cos(theta), 0, -sin(theta)],
        [0, 1, 0],
        [sin(theta), 0, cos(theta)],
    ])
    R_psi = np.array([
        [cos(psi), sin(psi), 0],
        [-sin(psi), cos(psi), 0],
        [0, 0, 1],
    ])
    return R_phi@R_theta@R_psi

# 惯性系下的RK4积分（核心状态：p_i, v_i, quat）
def rk4_step(obj, F_b, M_b, dt):
    # 定义状态导数函数（惯性系）
    def state_deriv(p, v, quat, w):
        R_i2b = Quat2RotMat(quat).T  # 惯性→体轴旋转矩阵
        F_i = R_i2b.T @ F_b          # 体轴力转惯性系
        M_i = R_i2b.T @ M_b          # 体轴力矩转惯性系
        I_i = R_i2b.T @ obj.I_b @ R_i2b  # 体轴惯性张量转惯性系
        
        # 惯性系动力学导数
        v_dot = F_i / obj.m
        w_dot = inv(I_i) @ (M_i - np.cross(w, I_i @ w))
        quat_dot = QuatDerivative(quat, R_i2b @ w)  # 四元数导数（体轴角速度输入）
        return np.hstack([v, v_dot, w, quat_dot])
    
    # RK4四步迭代
    k1 = state_deriv(obj.p_, obj.v_, obj.quat, obj.w_)
    k2 = state_deriv(obj.p_+k1[:3]*dt/2, obj.v_+k1[3:6]*dt/2, obj.quat+k1[9:]*dt/2, obj.w_+k1[6:9]*dt/2)
    k3 = state_deriv(obj.p_+k2[:3]*dt/2, obj.v_+k2[3:6]*dt/2, obj.quat+k2[9:]*dt/2, obj.w_+k2[6:9]*dt/2)
    k4 = state_deriv(obj.p_+k3[:3]*dt, obj.v_+k3[3:6]*dt, obj.quat+k3[9:]*dt, obj.w_+k3[6:9]*dt)
    
    # 更新状态
    obj.p_ += (k1[:3] + 2*k2[:3] + 2*k3[:3] + k4[:3]) * dt/6
    obj.v_ += (k1[3:6] + 2*k2[3:6] + 2*k3[3:6] + k4[3:6]) * dt/6
    obj.w_ += (k1[6:9] + 2*k2[6:9] + 2*k3[6:9] + k4[6:9]) * dt/6
    obj.quat += (k1[9:] + 2*k2[9:] + 2*k3[9:] + k4[9:]) * dt/6
    obj.quat /= norm(obj.quat)  # 四元数归一化（关键！）

# 【重大修复】：工业级鲁棒的旋转矩阵转四元数，防止 np.sign(0) 导致奇异崩溃
# 注意：该函数标准期待输入为 Rb->i 矩阵
def RotMat2Quat(R):
    tr = R[0,0] + R[1,1] + R[2,2]
    if tr > 0:
        S = np.sqrt(tr + 1.0) * 2
        qw = 0.25 * S
        qx = (R[2,1] - R[1,2]) / S
        qy = (R[0,2] - R[2,0]) / S
        qz = (R[1,0] - R[0,1]) / S 
    elif (R[0,0] > R[1,1]) and (R[0,0] > R[2,2]):
        S = np.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2]) * 2 
        qw = (R[2,1] - R[1,2]) / S
        qx = 0.25 * S
        qy = (R[0,1] + R[1,0]) / S 
        qz = (R[0,2] + R[2,0]) / S 
    elif R[1,1] > R[2,2]:
        S = np.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2]) * 2 
        qw = (R[0,2] - R[2,0]) / S
        qx = (R[0,1] + R[1,0]) / S 
        qy = 0.25 * S
        qz = (R[1,2] + R[2,1]) / S 
    else:
        S = np.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1]) * 2 
        qw = (R[1,0] - R[0,1]) / S
        qx = (R[0,2] + R[2,0]) / S
        qy = (R[1,2] + R[2,1]) / S
        qz = 0.25 * S
    q = np.array([qw, qx, qy, qz])
    if q[0] < 0:
        q = -q # 保持 q0 恒正
    return q

# 【注意】：标准的公式其实输出的是 Rb->i 矩阵
def Quat2RotMat(q):
    """
    单位四元数 q = [q0, q1, q2, q3] → 旋转矩阵 R (3x3)
    矩阵每一行对应机体系x/y/z轴在惯性系下的投影，和你用的行向量格式完全匹配
    """
    q0, q1, q2, q3 = q
    R = np.zeros((3, 3))
    R[0, 0] = q0**2 + q1**2 - q2**2 - q3**2
    R[0, 1] = 2 * (q1*q2 - q0*q3)
    R[0, 2] = 2 * (q1*q3 + q0*q2)
    R[1, 0] = 2 * (q1*q2 + q0*q3)
    R[1, 1] = q0**2 - q1**2 + q2**2 - q3**2
    R[1, 2] = 2 * (q2*q3 - q0*q1)
    R[2, 0] = 2 * (q1*q3 - q0*q2)
    R[2, 1] = 2 * (q2*q3 + q0*q1)
    R[2, 2] = q0**2 - q1**2 - q2**2 + q3**2
    return R # 此处 R 是 Body to Inertial

# 罗德里格斯旋转公式
def RodRot(V, u, alpha):
    # 原向量，转轴，转角(弧度)
    V1 = V*cos(alpha)+np.cross(u,V)*sin(alpha)+u*np.dot(u,V)*(1-cos(alpha))
    return V1

# 求两个坐标系之间相差的旋转轴和角度（四元数表示）
def QuatMatBetweenFrams(R_target, R_current, v_type="row"):
    # 注意R_target和R_current由行向量按列堆叠
    # A 坐标系 转到 B 坐标系正确矩阵永远是：RA→B​=B⋅AT
    if v_type=="row":
        R_diff = R_target.T @ R_current
    elif v_type=="col":
        R_diff = R_target @ R_current.T
    else:
        return None
    q = RotMat2Quat(R_diff)
    return q, R_diff

def Quat2AxisAngle(q):
    """
    单位四元数 q = [q0, q1, q2, q3] → 单位旋转轴u和旋转角alpha
    返回: u (3x1单位向量), alpha (弧度)
    """
    q0, q1, q2, q3 = q
    sin_half_alpha = np.sqrt(q1**2 + q2**2 + q3**2)
    # 数值稳定性处理
    if sin_half_alpha < 1e-8:
        # 转角接近0，轴方向可以取任意单位向量，这里用[1,0,0]作为默认
        return np.array([1, 0, 0]), 0.0
    # 求转角
    half_alpha = np.arctan2(sin_half_alpha, q0)
    alpha = 2 * half_alpha
    # 还原单位轴
    u = np.array([q1, q2, q3]) / sin_half_alpha
    return u, alpha

def QuatDerivative(q_i2b_, w_b_):
    """
    四元数微分方程：dot(q) = 1/2 * w(w) * q
    w: 机体角速度 [p, q, r]
    """
    p, q_w, r = w_b_
    q0, q1, q2, q3 = q_i2b_
    q_dot = 0.5 * np.array([
        -p*q1 - q_w*q2 - r*q3,
        p*q0 + r*q2 - q_w*q3,
        q_w*q0 - r*q1 + p*q3,
        r*q0 + q_w*q1 - p*q2
    ])
    return q_dot

# 积分q_dot 后需要补充四元数归一化 q = q / np.linalg.norm(q)


# 设置坐标轴等比例
def axes_equal(ax, x_range, y_range, z_range):
    """确保3D图的坐标轴单位长度相等。"""
    min_x, max_x = x_range
    min_y, max_y = y_range
    min_z, max_z = z_range
    x_range = abs(max_x - min_x)
    y_range = abs(max_y - min_y)
    z_range = abs(max_z - min_z)
    ax.set_xlim3d([max_x, min_x])
    ax.set_ylim3d([min_y, max_y])
    ax.set_zlim3d([min_z, max_z])
    # 设置等显示缩放比例
    ax.set_box_aspect([x_range, y_range, z_range])

def set_axes_equal(ax):
    """确保3D图的坐标轴单位长度相等。"""
    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()
    x_range = abs(x_limits[1] - x_limits[0])
    y_range = abs(y_limits[1] - y_limits[0])
    z_range = abs(z_limits[1] - z_limits[0])
    max_range = max(x_range, y_range, z_range)
    x_middle = np.mean(x_limits)
    y_middle = np.mean(y_limits)
    z_middle = np.mean(z_limits)
    ax.set_xlim3d([x_middle - max_range / 2, x_middle + max_range / 2])
    ax.set_ylim3d([y_middle - max_range / 2, y_middle + max_range / 2])
    ax.set_zlim3d([z_middle - max_range / 2, z_middle + max_range / 2])
    # 设置等显示缩放比例
    ax.set_box_aspect([1, 1, 1])


def set_axes_equal_manual(ax, x_limits=None, y_limits=None, z_limits=None):
    """确保3D图的坐标轴单位长度相等。"""
    if x_limits is None:
        x_limits1 = ax.get_xlim3d()
    else:
        x_limits1 = x_limits
    if y_limits is None:
        y_limits1 = ax.get_ylim3d()
    else:
        y_limits1 = y_limits
    if z_limits is None:
        z_limits1 = ax.get_zlim3d()
    else:
        z_limits1 = z_limits
    
    x_min, x_max = x_limits1
    y_min, y_max = y_limits1
    z_min, z_max = z_limits1

    x_range = x_max - x_min
    y_range = y_max - y_min
    z_range = z_max - z_min

    max_range = max(x_range, y_range, z_range)

    x_temp = max_range if x_limits is None else x_range
    y_temp = max_range if y_limits is None else y_range
    z_temp = max_range if z_limits is None else z_range

    x_middle = (x_max + x_min) / 2
    y_middle = (y_max + y_min) / 2
    z_middle = (z_max + z_min) / 2
    
    ax.set_xlim3d([x_middle - x_temp / 2, x_middle + x_temp / 2])
    ax.set_ylim3d([y_middle - y_temp / 2, y_middle + y_temp / 2])
    ax.set_zlim3d([z_middle - z_temp / 2, z_middle + z_temp / 2])

    # 设置等显示缩放比例
    ax.set_box_aspect([x_temp, y_temp, z_temp])

