"""
运动规划模块：将路径点转换为带速度和加速度的平滑轨迹。
"""
import numpy as np
from typing import List, Tuple

def smooth_path(path: List[Tuple[float,float]], num_points: int = 100) -> np.ndarray:
    """
    使用多项式插值或 B 样条，将离散路径点转换为平滑轨迹。
    :param path: 世界坐标下的路径点列表
    :param num_points: 插值后轨迹点数
    :return: num_points×2 的平滑轨迹点数组
    """
    path = np.array(path)
    t = np.linspace(0, 1, len(path))
    ti = np.linspace(0, 1, num_points)
    from scipy.interpolate import CubicSpline
    cs_x = CubicSpline(t, path[:,0])
    cs_y = CubicSpline(t, path[:,1])
    xs = cs_x(ti)
    ys = cs_y(ti)
    return np.vstack((xs, ys)).T
