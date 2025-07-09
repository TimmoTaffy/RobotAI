"""
世界模型定义，使用 common/types 中的标准数据类型，避免重复定义
"""
from dataclasses import dataclass
from typing import List
import numpy as np
from common.types import Pose2D, TurretState, RobotInfo


@dataclass
class WorldModel:
    # 自车定位位姿，含位置和朝向
    self_pose: Pose2D
    # 云台状态，含姿态和电机角度
    turret_state: TurretState
    # 任务点列表（保留原数据结构或可移至 common/types）
    task_points: List  
    # 静态障碍和动态障碍列表（自定义结构）
    static_obstacles: List
    dynamic_obstacles: List
    # 最新生成的二维栅格地图
    occupancy_grid: np.ndarray
    # 地面坡度分析结果，每个距离区间对应的坡度角 (弧度)
    ground_slopes: dict
    # 敌我机器人信息列表，使用共用类型
    robots: List[RobotInfo]