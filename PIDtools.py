import numpy as np
from numpy.linalg import norm
from math import *

# 一阶惯性环节
class FirstOrderIneritialElement(object):
    def __init__(self, rate=np.inf) -> None:
        self.rate = rate
        self.last_state = None
    def calculate(self, input, dt=0.02):
        if self.last_state is None:
            output = input
        else:
            state_change_req = input - self.last_state
            output = self.last_state + np.sign(state_change_req) * min(self.rate * dt, abs(state_change_req))
        self.last_state = output
        return output

class DeltaPID(object):
    """增量式PID算法实现"""
    def __init__(self, p=0, i=0, d=0) -> None:
        self.k_p = p  # 比例系数
        self.k_i = i  # 积分系数
        self.k_d = d  # 微分系数
        self._pre_error = 0  # t-1 时刻误差值
        self._pre_pre_error = 0  # t-2 时刻误差值
    def calculate(self, error, dt=0.02):
        p_change = self.k_p * (error - self._pre_error)
        i_change = self.k_i * error * dt
        d_change = self.k_d * (error - 2 * self._pre_error + self._pre_pre_error) / dt
        delta_output = p_change + i_change + d_change  # 本次增量
        self._pre_pre_error = self._pre_error
        self._pre_error = error
        return delta_output

class PositionPID(object):
    """位置式PID算法实现"""

    # output_list = []
    def __init__(self, max, min, p, i, d) -> None:
        self._max = max  # 最大输出限制，规避过冲
        self._min = min  # 最小输出限制
        self.k_p = p  # 比例系数
        self.k_i = i  # 积分系数
        self.k_d = d  # 微分系数
        self._pre_error = 0  # t-1 时刻误差值
        self._integral = 0  # 误差积分值

    def calculate(self, error, dt, d_error=None, i_error=None):
        """
        计算t时刻PID输出值cur_val
        """
        # 比例项
        p_out = self.k_p * error
        # 积分项
        if i_error is None: # 不可直接获取，再做积分
            self._integral += (error * dt)
        else:
            self._integral = i_error

        # 仿照simple_pid，将积分项整项预先限幅
        self._integral = np.clip(self._integral, self._min/self.k_i, self._max/self.k_i) \
            if self.k_i!=0 else self._integral
        
        i_out = self.k_i * self._integral
        # 微分项
        if d_error is None: # 不可直接获取，再做差分
            derivative = (error - self._pre_error) / dt
        else:
            derivative = d_error
        d_out = self.k_d * derivative
        # t 时刻pid输出
        output = p_out + i_out + d_out
        # 限制输出值
        if output > self._max:
            output = self._max
        elif output < self._min:
            output = self._min
        self._pre_error = error
        return output

    def clear_integral(self):
        self._integral = 0


def active_rotation(vector,heading,theta,gamma):
    # 飞机的轴在外界看来是怎样的， 前上右在北天东看来朝向哪个方向
    # vector是行向量，旋转顺序仍然是 psi, theta, gamma，但是计算顺序相反
    # 主动旋转，-在1下
    # 注意：北天东坐标
    psi = - heading
    Rpsi=np.array([
        [cos(psi), 0, sin(psi)],
        [0, 1, 0],
        [-sin(psi), 0, cos(psi)]
        ])
    Rtheta=np.array([
        [cos(theta), -sin(theta), 0],
        [sin(theta), cos(theta), 0],
        [0, 0, 1]
        ])
    Rgamma=np.array([
        [1, 0, 0],
        [0, cos(gamma), -sin(gamma)],
        [0, sin(gamma), cos(gamma)]
        ])
    return vector@Rgamma.T@Rtheta.T@Rpsi.T


def sub_of_radian(input1, input2=0):
    # 计算两个弧度的差值，范围为[-pi, pi]
    diff = input1 - input2
    diff = (diff + pi) % (2 * pi) - pi
    return diff


def sub_of_degree(input1, input2=0):
    # 计算两个角度的差值，范围为[-180, 180]
    diff = input1 - input2
    diff = (diff + 180) % 360 - 180
    return diff
