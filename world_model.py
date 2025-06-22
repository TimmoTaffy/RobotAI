from dataclasses import dataclass
from typing import List

@dataclass
class SelfPose:
    x: float
    y: float
    theta: float  # 朝向角
    velocity: float  # 线速度
    turret_pitch: float = 0.0  # 云台俯仰角
    turret_yaw: float = 0.0  # 云台偏航角

@dataclass
class EnemyTarget:
    x: float
    y: float
    direction: float  # 移动方向角
    confidence: float  # 置信度
    velocity: float = 0.0  # 预测速度

@dataclass
class TaskPoint:
    name: str  # 任务点标识
    x: float
    y: float

@dataclass
class StaticObstacle:
    x: float
    y: float
    size: float  # 障碍物半径或近似边长

@dataclass
class DynamicObstacle:
    x: float
    y: float
    vx: float  # x方向速度
    vy: float  # y方向速度
    size: float  # 障碍物大小

@dataclass
class RobotInfo:
    id: int
    team: str  # "ally" 或 "enemy"
    x: float
    y: float
    color: str

@dataclass
class WorldModel:
    self_pose: SelfPose
    enemy_targets: List[EnemyTarget]
    task_points: List[TaskPoint]
    static_obstacles: List[StaticObstacle]
    dynamic_obstacles: List[DynamicObstacle]
    robots: List[RobotInfo]  # 敌我机器人信息