"""
轨迹跟踪模块：实现基于纯追踪(Pure Pursuit)的速度和转向控制器
"""
from typing import Tuple
import numpy as np
from common.types import Pose2D

class TrajectoryTracker:
    def __init__(self, lookahead_distance: float = 1.0, kp_speed: float = 1.0, desired_speed: float = 1.0):
        """
        :param lookahead_distance: 纯追踪算法的前瞻距离 (m)
        :param kp_speed: 线速度控制增益
        :param desired_speed: 期望巡航速度 (m/s)
        """
        self.Ld = lookahead_distance
        self.kp_speed = kp_speed
        self.desired_speed = desired_speed

    def update(self, current_pose: Pose2D, trajectory: np.ndarray) -> Tuple[float, float]:
        """
        生成线速度和转向角命令
        :param current_pose: 车辆当前位姿 (Pose2D)
        :param trajectory: N×2 路径点数组 [[x,y], ...]
        :return: (v, steering_angle)
        """
        # 提取当前位置
        x, y = current_pose.position
        theta = current_pose.theta

        # 计算与路径点的距离
        deltas = trajectory - np.array([x, y])
        dists = np.linalg.norm(deltas, axis=1)
        # 找到第一个超过前瞻距离的目标点
        idx = np.where(dists > self.Ld)[0]
        if len(idx) > 0:
            goal = trajectory[idx[0]]
        else:
            goal = trajectory[-1]

        dx = goal[0] - x
        dy = goal[1] - y
        # 目标点相对于车体坐标系下的角度
        angle_to_goal = np.arctan2(dy, dx)
        alpha = angle_to_goal - theta
        # 归一化到 [-pi, pi]
        alpha = (alpha + np.pi) % (2 * np.pi) - np.pi

        # 转向角 (假设小车前轮转向模型)
        # steering = arctan(2*L*sin(alpha)/Ld)，L 为轴距，可设置为 1.0
        L = 1.0
        steering_angle = np.arctan2(2 * L * np.sin(alpha), self.Ld)

        # 线速度控制
        v = self.kp_speed * (self.desired_speed)
        return v, steering_angle
