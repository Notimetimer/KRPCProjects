from math import *
import numpy as np
from numpy.linalg import norm, inv
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# --- 核心算法类 ---
def left_mutiple(M, v):
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

# 短期飞控动力学可以再体轴系做，但导航计算必须在惯性系
# 辛普森积分在体轴系根本用不了，RK4也救不了体轴系积分运动的固有误差

# 飞行器，不考虑牵连运动，且考虑相对惯性系的运动
class FlyingObject:
    def __init__(self, m, I_b):
        self.m = m # 质量
        self.I_b = I_b # 惯性张量矩阵
        self.I_inv = inv(I_b)
        self.R_i2b = np.eye(3) # 旋转矩阵

    def reset(self, p0_, v0_, quat0, w0_):
        # 向量均为行向量，传入的均为惯性系下的观测量
        self.p_ = p0_.astype(float) # 相对惯性系
        self.v_ = v0_.astype(float) # 相对惯性系
        self.quat = quat0.astype(float) # 相对惯性系
        self.w_ = w0_.astype(float) # 相对惯性系
        # 【重大修复】: 必须转置！Quat2RotMat 输出的是 b->i，其转置才是你定义的 i->b
        self.R_i2b = Quat2RotMat(self.quat).T 
        self.xb_ = self.R_i2b[0,:]
        self.yb_ = self.R_i2b[1,:]
        self.zb_ = self.R_i2b[2,:]
        self.Gyroscopic_moment_b_ = np.zeros(3)
        self.Gyroscopic_moment_i_ = np.zeros(3)
        # 非线性（含旋转不能用（旋转惯性系下不要用）梯形积分，辛普森积分也不适合用在体轴系）
        # # 辛普森积分记忆项
        # self.v_last_ = np.zeros(3) # 惯性系
        # self.v_dot_last_ = np.zeros(3) # 跟随计算所用的坐标系
        # self.w_dot_last_ = np.zeros(3) # 跟随计算所用的坐标系
    
    # 体轴系动力学计算业务拆分：先算完姿态的所有，再从加速度开始算起质心运动
    def calc_w_dot_body_frame(self, M_b_, w_b_):
        # 旋转运动角加速度方程
        self.Gyroscopic_moment_b_ = - np.cross(w_b_, 
                            left_mutiple(self.I_b, w_b_))
        w_b_dot_ = left_mutiple(
            self.I_inv, 
            M_b_ + self.Gyroscopic_moment_b_
            )
        return w_b_dot_
    def calc_acc_body_frame(self, F_b_, v_b_, w_b_):
        # 体轴系质心动力学方程
        v_b_dot_ = np.cross(v_b_, w_b_) + F_b_/self.m
        return v_b_dot_
    
    # 在体轴系计算加速度的状态更新，失败的做法，体轴只能算动力学，运动学只有在惯性系计算才不会有显著的误差累积
    def move_calc_in_b(self, F_b_, M_b_, dt):
        # 非惯性系先算旋转，再算平移
        w_b_ = left_mutiple(self.R_i2b, self.w_)
        w_b_dot_ = self.calc_w_dot_body_frame(M_b_, w_b_)

        # # 无处安放的旋转系质点动力学
        # v_b_ = left_mutiple(self.R_i2b, self.v_)
        # w_b_ = left_mutiple(self.R_i2b, self.w_)
        # v_b_dot_ = self.calc_acc_body_frame(F_b_, v_b_, w_b_)

        # 角运动
        w_b_next_ = w_b_ + w_b_dot_ * dt # 欧拉积分2
        # w_b_next_ = w_b_ + (w_b_dot_+self.w_dot_last_)/2 * dt # （旋转惯性系下不要用）梯形积分2
        # self.w_dot_last_ = w_b_dot_
        if norm(w_b_) > 1e-3:
            w_b_dot_mag = np.dot(w_b_dot_, w_b_)/(norm(w_b_)+1e-8)
            w_b_next_ = w_b_next_ / (norm(w_b_next_)+1e-8) * (norm(w_b_) + w_b_dot_mag * dt)


        # 无处安放的旋转系质点动力学
        v_b_ = left_mutiple(self.R_i2b, self.v_)
        v_b_dot_ = self.calc_acc_body_frame(F_b_, v_b_, w_b_next_)


        # 【极其关键的修复】：必须先更新姿态，再把体轴系的速度转回惯性系
        quat_dot = QuatDerivative(self.quat, w_b_next_)
        quat_next = self.quat + quat_dot * dt
        # 更新旋转矩阵和旋转四元数
        self.quat = quat_next / norm(quat_next)
        # 【重大修复】：转置提取 i->b
        self.R_i2b = Quat2RotMat(self.quat).T
        # 姿态向量
        self.xb_ = self.R_i2b[0,:]
        self.yb_ = self.R_i2b[1,:]
        self.zb_ = self.R_i2b[2,:]
        self.w_ = left_mutiple(self.R_i2b.T, w_b_next_)

        # 在体轴系更新速度与角速度
        # 线运动
        v_b_next_ = v_b_ + v_b_dot_ * dt # 欧拉积分1
        # v_b_next_ = v_b_ + (v_b_dot_+self.v_dot_last_)/2 * dt # （旋转惯性系下不要用）梯形积分1
        # self.v_dot_last_ = v_b_dot_
        # 大小计算，防止被向心力加速
        if norm(v_b_) > 1e-3:
            v_b_dot_mag = np.dot(v_b_dot_, v_b_)/(norm(v_b_)+1e-8)
            v_b_next_ = v_b_next_ / (norm(v_b_next_)+1e-8) * (norm(v_b_) + v_b_dot_mag * dt)
        
        # 回到惯性系更新位置
        self.v_ = left_mutiple(self.R_i2b.T, v_b_next_)

        self.p_ += self.v_ * dt # 欧拉积分3
        # self.p_ += (self.v_ + self.v_last_)/2 * dt # 叠两层（旋转惯性系下不要用）梯形积分就是辛普森积分3
        # self.v_last_ = self.v_

    def calc_acc_inert_frame(self, F_i_, M_i_, v_i_, w_i_):
        # I_inertial = R.T * I_body * R
        I_i = self.R_i2b.T @ self.I_b @ self.R_i2b
        # 惯性系质心动力学方程
        v_i_dot_ = F_i_ / self.m
        # 惯性系角加速度方程
        self.Gyroscopic_moment_i_ =  - np.cross(w_i_,
                                left_mutiple(I_i, w_i_))
        w_i_dot_ = left_mutiple(
            inv(I_i),
            M_i_ + self.Gyroscopic_moment_i_
            )
        return v_i_dot_, w_i_dot_

    # 在惯性系计算加速度的状态更新，运动学只有在惯性系计算才不会有显著的误差累积
    def move(self, F_b_, M_b_, dt):
        # 力和力矩、惯性张量变换到惯性系
        F_i_= left_mutiple(self.R_i2b.T, F_b_)
        M_i_ = left_mutiple(self.R_i2b.T, M_b_)
        v_i_ = self.v_
        w_i_ = self.w_

        # 惯性系动力学解算
        v_i_dot_, w_i_dot_ = self.calc_acc_inert_frame(F_i_, M_i_, v_i_, w_i_)

        # 在惯性系更新速度与角速度
        # 线运动
        v_i_next_ = self.v_ + v_i_dot_ * dt # 欧拉积分1
        # v_i_next_ = self.v_ + (v_i_dot_ + self.v_dot_last_)/2 * dt # （旋转惯性系下不要用）梯形积分1
        # self.v_dot_last_ = v_i_dot_
        if norm(v_i_) > 1e-3:
            v_i_dot_mag = np.dot(v_i_dot_, v_i_)/(norm(v_i_)+1e-8)
            v_i = norm(v_i_) + v_i_dot_mag * dt
            v_i_next_ = v_i_next_/(norm(v_i_next_) + 1e-8) * v_i

        # 角运动
        w_i_next_ = w_i_ + w_i_dot_ * dt # 欧拉积分2
        # w_i_next_ = w_i_ + (w_i_dot_ + self.w_dot_last_)/2 * dt # （旋转惯性系下不要用）梯形积分2
        # self.w_dot_last_ = w_i_dot_
        if norm(w_i_) > 1e-3:
            w_i_dot_mag = np.dot(w_i_dot_, w_i_)/(norm(w_i_)+1e-8)
            w_i = norm(w_i_) + w_i_dot_mag * dt
            w_i_next_ = w_i_next_ / (norm(w_i_next_)+1e-8) * w_i

        # 更新角速度和速度
        self.v_ = v_i_next_
        self.w_ = w_i_next_

        # 在惯性系更新位置和姿态
        self.p_ += self.v_ * dt # 欧拉积分3
        # self.p_ += (self.v_+self.v_last_)/2 * dt  # 叠两层（旋转惯性系下不要用）梯形积分就是辛普森积分3
        # self.v_last_ = self.v_
        if norm(self.w_) > 1e-8:
            axis_ = self.w_ / norm(self.w_)
            alpha = norm(self.w_) * dt
            self.xb_ = RodRot(self.xb_, axis_, alpha)
            self.yb_ = RodRot(self.yb_, axis_, alpha)
            self.zb_ = RodRot(self.zb_, axis_, alpha)
            # 归一化
            self.xb_ /= norm(self.xb_)
            self.yb_ /= norm(self.yb_)
            self.zb_ /= norm(self.zb_)
            # 通过新的姿态向量倒推旋转矩阵, 因为是行向量所以省去一次转置
            self.R_i2b = np.stack((self.xb_, self.yb_, self.zb_), axis=0)
        # 如果用QuatMatBetweenFrams不知道是不是一样，但并没有更方便就没这么做
        # 通过新的旋转矩阵倒推四元数
        # 【重大修复】: RotMat2Quat 期待传入 b->i 矩阵，而当前 self.R_i2b 是 i->b
        self.quat = RotMat2Quat(self.R_i2b.T)

# --- 仿真与绘图 ---
if __name__ == '__main__':
    m = 10
    # 1. 先定义惯性主轴上的惯性矩 (必须满足三角不等式，例如 1+1.5 > 2)
    I_principal = np.diag([1.0, 1.5, 2.0])
    # 2. 定义一个旋转矩阵 (表示体轴相对于惯性主轴偏转了多少)
    # 比如绕 Y 轴偏转 10 度
    phi1 = np.radians(1)
    theta1 = np.radians(1)
    psi1 = np.radians(1)

    R = RotMat_zyx(phi1, theta1, psi1)
    # 3. 计算体轴系下的非对角惯性张量
    # 公式: I_body = R * I_principal * R.T
    I_mat = R @ I_principal @ R.T
    print("非对角惯性矩阵:\n", I_mat)

    obj1, obj2 = FlyingObject(m, I_mat), FlyingObject(m, I_mat)
    
    p0, v0 = np.array([0,0,0]), np.array([20,0,0])
    q0, w0 = np.array([1,0,0,0]), np.array([0.001, 8.001, 0.21])
    
    obj1.reset(p0, v0, q0, w0)
    obj2.reset(p0, v0, q0, w0)
    
    Fb = np.array([0, 4, 0]) 
    Mb = np.array([0.0, 0.0, 0.0]) 
    
    dt = 0.01
    max_time = 100
    steps = int(max_time/dt)

    hist1, hist2 = [], []

    for i in range(steps):
        obj1.move_calc_in_b(Fb, Mb, dt)
        obj2.move(Fb, Mb, dt)
        hist1.append({'p': obj1.p_.copy(), 'x': obj1.xb_.copy(), 'y':obj1.yb_.copy(), 'z':obj1.zb_.copy()})
        hist2.append({'p': obj2.p_.copy(), 'x': obj2.xb_.copy(), 'y':obj2.yb_.copy(), 'z':obj2.zb_.copy()})

    # 数据转换
    p1 = np.array([h['p'] for h in hist1])
    x1 = np.array([h['x'] for h in hist1])
    y1 = np.array([h['y'] for h in hist1])
    z1 = np.array([h['z'] for h in hist1])
    p2 = np.array([h['p'] for h in hist2])
    x2 = np.array([h['x'] for h in hist2])
    y2 = np.array([h['y'] for h in hist2])
    z2 = np.array([h['z'] for h in hist2])
    p_e = np.array([norm(pt) for pt in (p1-p2)])
    x_e = np.array([norm(xt) for xt in (x1-x2)])

    print(f"Avg Position Error: {norm(p_e)/steps:.6e}")
    print(f"Avg Attitude Vector Error: {norm(x_e)/steps:.6e}")

    print(f"Final Position Error: {norm(p1[-1] - p2[-1]):.6e}")
    print(f"Final Attitude Vector Error: {norm(x1[-1] - x2[-1]):.6e}")

    # --- 同步动画与绘图 (4个子图同步对比) ---
    from matplotlib.animation import FuncAnimation

    fig = plt.figure(figsize=(15, 10))
    fig.suptitle("6-DOF Dynamics Sync Animation: Move 1 (Body) vs Move 2 (Inertial)", fontsize=16)

    # 创建4个子图 (2x2 布局)
    ax_att1 = fig.add_subplot(2, 2, 1, projection='3d')
    ax_att2 = fig.add_subplot(2, 2, 2, projection='3d')
    ax_pos1 = fig.add_subplot(2, 2, 3, projection='3d')
    ax_pos2 = fig.add_subplot(2, 2, 4, projection='3d')

    # --- 初始化绘图对象 ---
    # 姿态 1 & 2 的轨迹和轴矢量
    lines_att1 = [ax_att1.plot([], [], [], color, alpha=0.3)[0] for color in ['r', 'g', 'b']]
    vecs_att1  = [ax_att1.plot([], [], [], color, linewidth=2)[0] for color in ['r', 'g', 'b']]
    lines_att2 = [ax_att2.plot([], [], [], color, alpha=0.3)[0] for color in ['r', 'g', 'b']]
    vecs_att2  = [ax_att2.plot([], [], [], color, linewidth=2)[0] for color in ['r', 'g', 'b']]
    
    # 位置 1 & 2 的轨迹和点
    line_p1,  = ax_pos1.plot([], [], [], 'k-', linewidth=1.5, label='Traj 1')
    point_p1, = ax_pos1.plot([], [], [], 'ro')
    line_p2,  = ax_pos2.plot([], [], [], 'k-', linewidth=1.5, label='Traj 2')
    point_p2, = ax_pos2.plot([], [], [], 'bo')

    def init():
        # 姿态子图设置
        for ax, title in [(ax_att1, "Attitude (Move 1)"), (ax_att2, "Attitude (Move 2)")]:
            ax.set_xlim([-1.2, 1.2]); ax.set_ylim([-1.2, 1.2]); ax.set_zlim([-1.2, 1.2])
            ax.set_title(title); ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')

        # 位置子图设置 (自动缩放)
        margin = 2
        all_p = np.vstack([p1, p2])
        p_min, p_max = all_p.min(axis=0), all_p.max(axis=0)
        for ax, title in [(ax_pos1, "Position (Move 1)"), (ax_pos2, "Position (Move 2)")]:
            ax.set_xlim([p_min[0]-margin, p_max[0]+margin])
            ax.set_ylim([p_min[1]-margin, p_max[1]+margin])
            ax.set_zlim([p_min[2]-margin, p_max[2]+margin])
            ax.set_title(title); ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z')
        
        return (*lines_att1, *vecs_att1, *lines_att2, *vecs_att2, line_p1, point_p1, line_p2, point_p2)

    def update(frame):
        idx = min(frame * 5, steps - 1) # 5倍速播放
        
        # 数据组
        att1_data = [x1, y1, z1]
        att2_data = [x2, y2, z2]
        
        # 更新姿态 1
        for i, data in enumerate(att1_data):
            lines_att1[i].set_data(data[:idx, 0], data[:idx, 1]); lines_att1[i].set_3d_properties(data[:idx, 2])
            vecs_att1[i].set_data([0, data[idx, 0]], [0, data[idx, 1]]); vecs_att1[i].set_3d_properties([0, data[idx, 2]])
        
        # 更新姿态 2
        for i, data in enumerate(att2_data):
            lines_att2[i].set_data(data[:idx, 0], data[:idx, 1]); lines_att2[i].set_3d_properties(data[:idx, 2])
            vecs_att2[i].set_data([0, data[idx, 0]], [0, data[idx, 1]]); vecs_att2[i].set_3d_properties([0, data[idx, 2]])

        # 更新位置
        line_p1.set_data(p1[:idx, 0], p1[:idx, 1]); line_p1.set_3d_properties(p1[:idx, 2])
        point_p1.set_data([p1[idx, 0]], [p1[idx, 1]]); point_p1.set_3d_properties([p1[idx, 2]])
        line_p2.set_data(p2[:idx, 0], p2[:idx, 1]); line_p2.set_3d_properties(p2[:idx, 2])
        point_p2.set_data([p2[idx, 0]], [p2[idx, 1]]); point_p2.set_3d_properties([p2[idx, 2]])

        return (*lines_att1, *vecs_att1, *lines_att2, *vecs_att2, line_p1, point_p1, line_p2, point_p2)

    ani = FuncAnimation(fig, update, frames=steps//5, init_func=init, blit=True, interval=20, 
                        repeat=1) # repeat表示循环播放

    plt.tight_layout()
    plt.show()