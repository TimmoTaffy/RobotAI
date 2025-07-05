import numpy as np
from typing import Union, List
import numpy.typing as npt
from .transform_utils import TransformError, validate_vectors, compute_rotation_matrix
from common.types import Transform, Pose3D

def transform_lidar_to_vehicle(
    point_cloud: np.ndarray,
    transform: Transform
) -> np.ndarray:
    """
    将激光雷达点云从激光雷达坐标系转换到车体坐标系。
    
    坐标系定义：
    - 激光雷达坐标系：x 轴向前，y 轴向左，z 轴向上
    - 车体坐标系：x 轴向前，y 轴向左，z 轴向上
    
    参数：
    :param point_cloud: N×3 的点云数据
    :param transform: 激光雷达到车体的固定变换关系
    
    返回：
    :return: N×3 的转换后点云数据
    :raises TransformError: 当输入参数格式不正确时
    """
    # 验证输入
    if transform.parent_frame != "vehicle" or transform.child_frame != "lidar":
        raise TransformError("Transform must be from lidar to vehicle frame")
        
    if not isinstance(point_cloud, np.ndarray) or point_cloud.ndim != 2:
        raise TransformError("Point cloud must be a 2D numpy array")
    
    # 计算旋转矩阵
    R = compute_rotation_matrix(transform.rotation)
    
    # 批量转换点云
    vehicle_points = (R @ point_cloud.T).T + transform.translation
    
    return vehicle_points

def transform_vehicle_to_lidar(
    vehicle_points: np.ndarray,
    transform: Transform
) -> np.ndarray:
    """
    将车体坐标系下的点云转换到激光雷达坐标系。
    
    参数：
    :param vehicle_points: N×3 的点云数据
    :param transform: 激光雷达到车体的固定变换关系
    
    返回：
    :return: N×3 的转换后点云数据
    :raises TransformError: 当输入参数格式不正确时
    """
    # 验证输入
    if transform.parent_frame != "vehicle" or transform.child_frame != "lidar":
        raise TransformError("Transform must be from lidar to vehicle frame")
        
    if not isinstance(vehicle_points, np.ndarray) or vehicle_points.ndim != 2:
        raise TransformError("Points must be a 2D numpy array")
    
    # 计算反向变换
    R_inv = compute_rotation_matrix(transform.rotation).T
    
    # 批量转换点云
    lidar_points = (R_inv @ (vehicle_points - transform.translation).T).T
    
    return lidar_points