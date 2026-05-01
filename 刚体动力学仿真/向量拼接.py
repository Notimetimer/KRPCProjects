import numpy as np

"分块矩阵拼接"
left = np.array([
    [1,2,3],
    [4,5,6],
    [7,8,9],
    ])

right_vector= np.array([1,10,100])

# 向量的维度扩增和变列向量
right = np.array([right_vector]).T
right2 = np.expand_dims(right_vector, axis=1)

# 齐次矩阵水平拼接
UP1 = np.append(left, right, axis=1)
UP2 = np.concatenate((left, right),axis=1)
UP3 = np.hstack((left, right))

DOWN = np.array([[0,0,0,1]])

# 齐次矩阵垂直拼接
homo1 = np.concatenate((UP1, DOWN))
homo2 = np.append(UP1, DOWN, axis=0)
homo3 = np.vstack((UP1, DOWN))

"分块矩阵拆分"
upleft = homo1[0:3, 0:3]
upright = homo1[0:3:, 3:]
downleft = homo1[3:, 0:3]
downright = homo1[3:, 3:]
