import numpy as np
from numpy.linalg import norm
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation

# -----------------------------------------------------------------------------
# 工具函数
# -----------------------------------------------------------------------------
def skew(v):
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])

def quat_update(q, w, dt):
    q = q / norm(q)
    w = np.array([
        [0, -w[0], -w[1], -w[2]],
        [w[0], 0, w[2], -w[1]],
        [w[1], -w[2], 0, w[0]],
        [w[2], w[1], -w[0], 0]
    ])
    q_dot = 0.5 * w @ q
    q_new = q + q_dot * dt
    return q_new / norm(q_new)

def quat_to_rot(q):
    q0,q1,q2,q3 = q
    return np.array([
        [q0**2+q1**2-q2**2-q3**2, 2*(q1*q2-q0*q3), 2*(q1*q3+q0*q2)],
        [2*(q1*q2+q0*q3), q0**2-q1**2+q2**2-q3**2, 2*(q2*q3-q0*q1)],
        [2*(q1*q3-q0*q2), 2*(q2*q3+q0*q1), q0**2-q1**2-q2**2+q3**2]
    ])

# -----------------------------------------------------------------------------
# 连杆类
# -----------------------------------------------------------------------------
class Link:
    def __init__(self, m, J, L0, k_rot, k_axial, d_rot, d_axial):
        self.m = m
        self.J = J
        self.L0 = L0
        self.k_rot = k_rot
        self.k_axial = k_axial
        self.d_rot = d_rot
        self.d_axial = d_axial

        self.q = np.array([1,0,0,0])
        self.d = L0
        self.w = np.zeros(3)
        self.d_dot = 0.0

        self.R = np.eye(3)
        self.p = np.zeros(3)
        self.v = np.zeros(3)
        self.a = np.zeros(3)

# -----------------------------------------------------------------------------
# 多连杆弹簧动力学（递归牛顿-欧拉，前向递推）
# -----------------------------------------------------------------------------
class MultiLinkSim:
    def __init__(self, links):
        self.links = links

    def step(self, dt):
        R_p = np.eye(3)
        p_p = np.zeros(3)
        v_p = np.zeros(3)
        a_p = np.zeros(3)
        w_p = np.zeros(3)
        a_w_p = np.zeros(3)

        for i, ln in enumerate(self.links):
            ln.R = quat_to_rot(ln.q)
            ln.p = np.array([ln.d, 0, 0])

            w_rel = ln.w
            ln.w = w_p + R_p.T @ w_rel

            a_w_rel = (w_rel - (w_p - (w_p if i==0 else self.links[i-1].w))) / dt if dt>1e-6 else np.zeros(3)
            ln.a_w = a_w_p + R_p.T @ a_w_rel

            v_rot = np.cross(ln.w, ln.p)
            ln.v = R_p.T @ v_p + v_rot + np.array([ln.d_dot,0,0])

            a_rot = np.cross(ln.a_w, ln.p) + np.cross(ln.w, np.cross(ln.w, ln.p))
            a_cor = 2 * np.cross(ln.w, np.array([ln.d_dot,0,0]))
            a_ax = np.array([0,0,0])
            ln.a = R_p.T @ a_p + a_rot + a_cor + a_ax

            ang_err = 2 * ln.q[1:]
            tau = -ln.k_rot * ang_err - ln.d_rot * ln.w
            ln.w += tau / np.diag(ln.J).mean() * dt

            delta = ln.d - ln.L0
            f = -ln.k_axial * delta - ln.d_axial * ln.d_dot
            ln.d_dot += f / ln.m * dt

            ln.q = quat_update(ln.q, ln.w, dt)
            ln.d += ln.d_dot * dt

            R_p = ln.R @ R_p
            p_p += R_p @ ln.p
            v_p = ln.v
            a_p = ln.a
            w_p = ln.w
            a_w_p = ln.a_w

# -----------------------------------------------------------------------------
# 3D 动画
# -----------------------------------------------------------------------------
if __name__ == "__main__":
    link1 = Link(1.0, np.diag([0.1,0.2,0.2]), 1.0, 5.0, 30.0, 0.3, 0.6)
    link2 = Link(1.0, np.diag([0.1,0.2,0.2]), 1.0, 5.0, 30.0, 0.3, 0.6)
    link3 = Link(1.0, np.diag([0.1,0.2,0.2]), 1.0, 5.0, 30.0, 0.3, 0.6)

    sim = MultiLinkSim([link1, link2, link3])

    link1.w = np.array([0, 0.8, 0])
    link2.d = 1.3
    link3.w = np.array([0, 0, 0.7])

    dt = 0.02
    steps = 700

    fig = plt.figure(figsize=(8,8))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlim(-4,4)
    ax.set_ylim(-4,4)
    ax.set_zlim(-4,4)
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title("Multi-Link Spring Arm (Ball Joint + Telescopic)")

    line, = ax.plot([], [], [], 'o-', lw=3, color='royalblue')

    def update(frame):
        sim.step(dt)
        pos = [[0,0,0]]
        R = np.eye(3)
        p = np.zeros(3)
        for ln in sim.links:
            R = R @ ln.R
            p = p + R @ ln.p
            pos.append(p.copy())
        pos = np.array(pos).T
        line.set_data(pos[0], pos[1])
        line.set_3d_properties(pos[2])
        return line,

    ani = animation.FuncAnimation(fig, update, frames=steps, interval=20, blit=True)
    plt.show()