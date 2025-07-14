import numpy as np
from typing import List, Union
import numpy.typing as npt
from .transform_utils import TransformError, validate_vectors, compute_rotation_matrix
from src.common.types import Pose3D, Transform

def transform_camera_to_vehicle(
    camera_points: np.ndarray,
    transform: Transform
) -> np.ndarray:
    """
    批量将相机坐标系下的点转换到车体坐标系。
    
    坐标系变换链:
    相机坐标系 -[旋转]-> 相机方向对齐的坐标系 -[平移]-> 车体坐标系
    
    坐标系定义：
    - 相机坐标系：z 轴指向前方，x 轴向右，y 轴向下
    - 车体坐标系：x 轴指向前方，y 轴向左，z 轴向上
    
    参数：
    :param camera_points: Nx3 数组，N个相机坐标系下的点
    :param transform: 相机到车体的固定变换关系
    
    返回：
    :return: Nx3 数组，N个车体坐标系下的点
    :raises TransformError: 当输入参数格式不正确时
    """
    # 验证输入
    if transform.parent_frame != "vehicle" or transform.child_frame != "camera":
        raise TransformError("Transform must be from camera to vehicle frame")
    
    if not isinstance(camera_points, np.ndarray) or camera_points.ndim != 2:
        raise TransformError("Points must be a 2D numpy array")
    
    # 计算旋转矩阵
    R = compute_rotation_matrix(transform.rotation)
    
    # 批量转换点云
    vehicle_points = (R @ camera_points.T).T + transform.translation
    
    return vehicle_points

def transform_vehicle_to_camera(
    vehicle_points: np.ndarray,
    transform: Transform
) -> np.ndarray:
    """
    批量将车体坐标系下的点转换到相机坐标系。
    
    参数：
    :param vehicle_points: Nx3 数组，N个车体坐标系下的点
    :param transform: 相机到车体的固定变换关系
    
    返回：
    :return: Nx3 数组，N个相机坐标系下的点
    :raises TransformError: 当输入参数格式不正确时
    """
    # 验证输入
    if transform.parent_frame != "vehicle" or transform.child_frame != "camera":
        raise TransformError("Transform must be from camera to vehicle frame")
    
    if not isinstance(vehicle_points, np.ndarray) or vehicle_points.ndim != 2:
        raise TransformError("Points must be a 2D numpy array")
    
    # 计算反向变换
    R_inv = compute_rotation_matrix(transform.rotation).T
    
    # 批量转换点云
    camera_points = (R_inv @ (vehicle_points - transform.translation).T).T
    
    return camera_points

def project_to_image(
    points_3d: np.ndarray,
    camera_matrix: np.ndarray,
    distortion_coeffs: np.ndarray
) -> np.ndarray:
    """
    将3D点投影到图像平面。
    
    参数：
    :param points_3d: Nx3 数组，N个3D点的坐标
    :param camera_matrix: 3x3 相机内参矩阵
    :param distortion_coeffs: 畸变参数
    
    返回：
    :return: Nx2 数组，N个点的图像坐标 (u,v)
    """
    # 确保点是相机坐标系下的
    if points_3d.shape[1] != 3:
        raise TransformError("Points must be in 3D (Nx3 array)")
        
    # 投影到归一化平面
    x = points_3d[:, 0] / points_3d[:, 2]
    y = points_3d[:, 1] / points_3d[:, 2]
    
    # 应用畸变
    r2 = x*x + y*y
    k1, k2, p1, p2, k3 = distortion_coeffs[:5]
    
    x_distorted = x * (1 + k1*r2 + k2*r2*r2 + k3*r2*r2*r2)
    y_distorted = y * (1 + k1*r2 + k2*r2*r2 + k3*r2*r2*r2)
    
    x_distorted = x_distorted + (2*p1*x*y + p2*(r2 + 2*x*x))
    y_distorted = y_distorted + (p1*(r2 + 2*y*y) + 2*p2*x*y)
    
    # 应用相机内参
    pixel_points = np.zeros((points_3d.shape[0], 2))
    pixel_points[:, 0] = camera_matrix[0,0]*x_distorted + camera_matrix[0,2]
    pixel_points[:, 1] = camera_matrix[1,1]*y_distorted + camera_matrix[1,2]
    
    return pixel_points