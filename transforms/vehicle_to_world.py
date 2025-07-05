import numpy as np
from typing import Tuple
from common.types import Pose2D

def transform_vehicle_to_world(
    local_pose: Pose2D,
    vehicle_pose: Pose2D
) -> Pose2D:
    """
    将车体坐标系下的位姿转换到世界坐标系。
    
    坐标系定义：
    - 车体坐标系：x 轴向前，y 轴向左
    - 世界坐标系：x 轴向东，y 轴向北
    
    参数：
    :param local_pose: 车体坐标系下的目标位姿
    :param vehicle_pose: 自车在世界坐标系下的位姿
    
    返回：
    :return: 世界坐标系下的目标位姿
    """
    # 计算旋转矩阵
    cos_theta = np.cos(vehicle_pose.theta)
    sin_theta = np.sin(vehicle_pose.theta)
    R = np.array([[cos_theta, -sin_theta],
                  [sin_theta, cos_theta]])
    
    # 转换位置
    world_position = R @ local_pose.position + vehicle_pose.position
    
    # 转换朝向角
    world_theta = local_pose.theta + vehicle_pose.theta
    
    # 规范化角度到 [-pi, pi]
    world_theta = np.arctan2(np.sin(world_theta), np.cos(world_theta))
    
    return Pose2D(
        position=world_position,
        theta=world_theta,
        timestamp=local_pose.timestamp
    )

def transform_world_to_vehicle(
    world_pose: Pose2D,
    vehicle_pose: Pose2D
) -> Pose2D:
    """
    将世界坐标系下的位姿转换到车体坐标系。
    
    参数：
    :param world_pose: 世界坐标系下的目标位姿
    :param vehicle_pose: 自车在世界坐标系下的位姿
    
    返回：
    :return: 车体坐标系下的目标位姿
    """
    # 计算反向旋转矩阵
    cos_theta = np.cos(-vehicle_pose.theta)
    sin_theta = np.sin(-vehicle_pose.theta)
    R_inv = np.array([[cos_theta, -sin_theta],
                      [sin_theta, cos_theta]])
    
    # 转换位置
    local_position = R_inv @ (world_pose.position - vehicle_pose.position)
    
    # 转换朝向角
    local_theta = world_pose.theta - vehicle_pose.theta
    
    # 规范化角度到 [-pi, pi]
    local_theta = np.arctan2(np.sin(local_theta), np.cos(local_theta))
    
    return Pose2D(
        position=local_position,
        theta=local_theta,
        timestamp=world_pose.timestamp
    )