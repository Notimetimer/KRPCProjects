import numpy as np

def left_mutiple(M, v):
    # 输入行向量和矩阵，当做列向量左乘矩阵来算
    return v@(M.T)

# 惯性张量（体轴系）
I = np.array([
    [1, 0, 0],
    [0, 1.5, 0],
    [0, 0, 2],
])

I_inv = np.linalg.inv(I)

omega0_ = np.array([0,0,0]) # 角速度
M_ = np.array([2,3,4]) # 力矩（体轴系）

dt = 0.01
omega_=omega0_
for i in range(1000):
    # 线加速度方程

    # 旋转运动角加速度方程
    omega_dot_ = left_mutiple(I_inv, 
        -np.cross(omega_, 
            left_mutiple(I, omega_))
        +M_)
    
    # 运动方程

    