from math import *
import numpy as np

def left_multiply(M, v):
    # 输入行向量和矩阵，当做列向量左乘矩阵来算
    return v@(M.T)

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

# 旋转矩阵转四元数
def RotMat2Quat(R):
    tr = R[0,0]+R[1,1]+R[2,2]
    q0 = 0.5*np.sqrt(tr+1) # 我选择q0恒正
    q1 = 0.5*np.sqrt(1+2*R[0,0]-tr)*np.sign(R[1,2]-R[2,1])
    q2 = 0.5*np.sqrt(1+2*R[1,1]-tr)*np.sign(R[2,0]-R[0,2])
    q3 = 0.5*np.sqrt(1+2*R[2,2]-tr)*np.sign(R[0,1]-R[1,0])
    return np.array([q0,q1,q2,q3])

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
    return R

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

def QuatDerivative(q, w):
    """
    四元数微分方程：dot(q) = 1/2 * w(w) * q
    w: 机体角速度 [p, q, r]
    """
    p, q_w, r = w
    q0, q1, q2, q3 = q
    q_dot = 0.5 * np.array([
        -p*q1 - q_w*q2 - r*q3,
        p*q0 + r*q2 - q_w*q3,
        q_w*q0 - r*q1 + p*q3,
        r*q0 + q_w*q1 - p*q2
    ])
    return q_dot

# 积分q_dot 后需要补充四元数归一化 q = q / np.linalg.norm(q)
