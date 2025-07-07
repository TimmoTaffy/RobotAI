"""
MPC 控制器模块: 基于模型预测控制生成线速与转向命令
"""
import numpy as np
import cvxpy as cp
from common.types import Pose2D
from typing import List, Tuple

class MPCController:
    def __init__(self,
                 horizon: int = 10,
                 dt: float = 0.1,
                 Q: np.ndarray = np.diag([1.0, 1.0, 0.1]),
                 R: np.ndarray = np.diag([0.1, 0.1])):
        """
        :param horizon: 预测步数
        :param dt: 离散时间步长
        :param Q: 状态误差权重矩阵
        :param R: 控制输入权重矩阵
        """
        self.N = horizon
        self.dt = dt
        self.Q = Q
        self.R = R

    def update(self,
               current_pose: Pose2D,
               ref_trajectory: np.ndarray) -> Tuple[float, float]:
        """
        计算当前时刻的最优控制命令 (v, omega)
        :param current_pose: 当前车体位姿 [x, y, theta]
        :param ref_trajectory: 参考轨迹，形状 (N+1, 3)，包括 x,y,theta
        :return: 当前时刻的控制命令 (线速度 v, 角速度 omega)
        """
        # 状态和控制变量
        x = cp.Variable((3, self.N+1))  # [x, y, theta]
        u = cp.Variable((2, self.N))    # [v, omega]

        cost = 0
        constr = []
        # 初始状态约束
        constr += [x[:,0] == np.array([*current_pose.position, current_pose.theta])]

        for k in range(self.N):
            # 状态误差
            err = x[:,k] - ref_trajectory[k]
            cost += cp.quad_form(err, self.Q)
            # 控制cost
            cost += cp.quad_form(u[:,k], self.R)
            # 系统动力学 (unicycle model)
            theta_k = x[2,k]
            constr += [x[0,k+1] == x[0,k] + u[0,k]*cp.cos(theta_k)*self.dt]
            constr += [x[1,k+1] == x[1,k] + u[0,k]*cp.sin(theta_k)*self.dt]
            constr += [x[2,k+1] == x[2,k] + u[1,k]*self.dt]
            # 输入约束，可自行调整
            constr += [u[0,k] >= 0, u[0,k] <= 2.0]      # v ∈ [0,2] m/s
            constr += [u[1,k] >= -1.0, u[1,k] <= 1.0]   # omega ∈ [-1,1] rad/s

        # 终端状态cost
        err_N = x[:,self.N] - ref_trajectory[self.N]
        cost += cp.quad_form(err_N, self.Q)

        # 求解二次规划
        problem = cp.Problem(cp.Minimize(cost), constr)
        problem.solve(solver=cp.OSQP, warm_start=True)

        # 返回第一步的控制命令
        v_opt = u.value[0,0]
        omega_opt = u.value[1,0]
        return float(v_opt), float(omega_opt)
