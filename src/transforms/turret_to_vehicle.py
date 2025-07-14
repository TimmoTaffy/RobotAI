import numpy as np
from typing import Union, List, Optional
import numpy.typing as npt
from .transform_utils import TransformError, validate_vectors, compute_rotation_matrix
from src.common.types import TurretState, Transform, Pose3D

def transform_turret_to_vehicle(
    turret_pose: Pose3D,
    transform: Transform
) -> Pose3D:
    """
    将云台坐标系下的位置和方向转换到车体坐标系。
    
    坐标系定义：
    - 云台坐标系：x 轴向前（枪管方向），y 轴向左，z 轴向上
    - 车体坐标系：x 轴向前，y 轴向左，z 轴向上
    
    参数：
    :param turret_pose: 云台坐标系下的目标位姿
    :param transform: 云台到车体的固定变换关系
    
    返回：
    :return: 车体坐标系下的目标位姿
    :raises TransformError: 当输入参数格式不正确时
    """
    # 验证输入
    if transform.parent_frame != "vehicle" or transform.child_frame != "turret":
        raise TransformError("Transform must be from turret to vehicle frame")

    # 获取旋转矩阵
    R = compute_rotation_matrix(turret_pose.orientation)
    T = compute_rotation_matrix(transform.rotation)
    
    # 转换位置
    vehicle_position = T @ turret_pose.position + transform.translation
    
    # 转换姿态
    vehicle_orientation = T @ R @ transform.rotation
    
    # 返回转换后的位姿
    return Pose3D(
        position=vehicle_position,
        orientation=vehicle_orientation,
        timestamp=turret_pose.timestamp
    )

def transform_vehicle_to_turret(
    vehicle_pose: Pose3D,
    transform: Transform
) -> Pose3D:
    """
    将车体坐标系下的位置和方向转换到云台坐标系。
    
    参数：
    :param vehicle_pose: 车体坐标系下的目标位姿
    :param transform: 云台到车体的固定变换关系
    
    返回：
    :return: 云台坐标系下的目标位姿
    :raises TransformError: 当输入参数格式不正确时
    """
    # 验证输入
    if transform.parent_frame != "vehicle" or transform.child_frame != "turret":
        raise TransformError("Transform must be from turret to vehicle frame")
        
    # 计算反向变换
    R_inv = compute_rotation_matrix(transform.rotation).T
    
    # 转换位置
    turret_position = R_inv @ (vehicle_pose.position - transform.translation)
    
    # 转换姿态
    turret_orientation = R_inv @ vehicle_pose.orientation
    
    # 返回转换后的位姿
    return Pose3D(
        position=turret_position,
        orientation=turret_orientation,
        timestamp=vehicle_pose.timestamp
    )