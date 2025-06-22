import numpy as np

def transform_lidar_to_vehicle(point_cloud, translation, rotation):
    """
    将激光雷达点云从激光雷达坐标系转换到车体坐标系
    :param point_cloud: N×3 的点云数据
    :param translation: 激光雷达相对于车体的平移 [x, y, z]
    :param rotation: 激光雷达相对于车体的旋转 [roll, pitch, yaw]
    :return: 转换后的点云数据
    """
    # 平移补偿
    point_cloud = point_cloud - np.array(translation)

    # 旋转补偿
    roll, pitch, yaw = rotation
    rotation_matrix = compute_rotation_matrix(roll, pitch, yaw)
    point_cloud = np.dot(point_cloud, rotation_matrix.T)

    return point_cloud

def compute_rotation_matrix(roll, pitch, yaw):
    """计算旋转矩阵"""
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