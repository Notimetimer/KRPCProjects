"""
惯性张量变换：
一、相似变换：
    1、旋转：惯性张量只是在右手系内旋转的话，可以用 I2 = R @ I @ inv(R) 来做到
    2、轴交换：同时交换对应的行和列
        如果交换1和2轴，那么惯性张量先交换1，2行，再交换1，2列
    3、轴反向：对反向的轴对应的行列同时取反
        如果2轴被反向了，那么2行和2列同时乘以-1，同时保证对角线不变，几个轴反向这么搞几个轴
    广义上，
    - 所有旋转/反向/减缓轴的变换，仍然是 I2 = P @ I @ inv(P)，只不过旋转矩阵R符合正交性和正定性，P不一定符合

二、涉及平移：
    令：
    Ic​：质心坐标系下的惯性张量
    Io​：任意原点 O 下的惯性张量
    m: 刚体总质量
    从质心c指向新原点o的位置矢量： [dx,dy,dz].T
    那么有：
    D = [
        [0, -dx, dy],
        [dz, 0, -dx],
        [-dy, dx, 0],
    ]
    使得：
    Io = Ic + m * D @ D.T

三、平移和相似变换混合：
    先平移，后相似变换

KSP的惯性张量默认是体轴系下的，顺序是行主序（先行后列）

体轴系默认顺序：右前下
"""

import krpc  
import numpy as np 
import copy

conn = krpc.connect()  
vessel = conn.space_center.active_vessel  

# 获取整个飞行器的惯性张量（体轴系）
inertia_tensor = vessel.inertia_tensor
# 右前下
matrix_3x3 = np.array([  
    [inertia_tensor[0], inertia_tensor[1], inertia_tensor[2]],  
    [inertia_tensor[3], inertia_tensor[4], inertia_tensor[5]],  
    [inertia_tensor[6], inertia_tensor[7], inertia_tensor[8]]  
])
# 交换行
matrix_3x3[0,:], matrix_3x3[1,:] = matrix_3x3[1,:], matrix_3x3[0,:]
# 交换列
matrix_3x3[:,0], matrix_3x3[:,1] = matrix_3x3[:,1], matrix_3x3[:,0]
# 前右下顺序的惯性张量
I_b_FRD = matrix_3x3
print("前右下", I_b_FRD)

temp2 = copy.deepcopy(I_b_FRD)
# 反转行
temp2[2,:] *= -1
# 反转列
temp2[:,2] *= -1 # 前右上
# 交换行
temp2[1,:], temp2[2,:] = temp2[2,:], temp2[1,:]
# 交换列
temp2[:,1], temp2[:,2] = temp2[:,2], temp2[:,1] # 前上右
I_b_FUR = temp2
print("前上右", I_b_FUR)

# 如果要换到惯性系下需要使用 R I inv(R) 计算
