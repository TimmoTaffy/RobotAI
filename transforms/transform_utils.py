import numpy as np
from typing import List, Tuple, Union
import numpy.typing as npt

class TransformError(Exception):
    """坐标变换相关的异常类"""
    pass

def validate_vectors(*vectors: Union[List[float], npt.NDArray], expected_size: int = 3) -> List[npt.NDArray]:
    """
    验证向量格式并转换为 numpy 数组
    
    :param vectors: 需要验证的向量列表
    :param expected_size: 期望的向量维度
    :return: 转换后的 numpy 数组列表
    :raises TransformError: 当输入向量格式不正确时
    """
    try:
        result = []
        for vec in vectors:
            arr = np.array(vec, dtype=float)
            if arr.shape != (expected_size,):
                raise ValueError(f"向量维度必须为 {expected_size}")
            result.append(arr)
        return result
    except (ValueError, TypeError) as e:
        raise TransformError(f"向量验证失败: {str(e)}")

def compute_rotation_matrix(roll: float, pitch: float, yaw: float) -> npt.NDArray:
    """
    计算旋转矩阵 R = Rz(yaw) @ Ry(pitch) @ Rx(roll)
    
    :param roll: 绕 x 轴的旋转角度（单位：弧度）
    :param pitch: 绕 y 轴的旋转角度（单位：弧度）
    :param yaw: 绕 z 轴的旋转角度（单位：弧度）
    :return: 3x3 旋转矩阵
    """
    Rx = np.array([[1, 0, 0],
                   [0, np.cos(roll), -np.sin(roll)],
                   [0, np.sin(roll), np.cos(roll)]])
    Ry = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                   [0, 1, 0],
                   [-np.sin(pitch), 0, np.cos(pitch)]])
    Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                   [np.sin(yaw), np.cos(yaw), 0],
                   [0, 0, 1]])
    return Rz @ Ry @ Rx
