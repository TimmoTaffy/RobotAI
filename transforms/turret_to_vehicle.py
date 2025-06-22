import numpy as np

def transform_turret_to_vehicle(turret_position, turret_orientation, turret_to_vehicle_offset):
    """
    将云台坐标系下的位置和方向转换到车体坐标系。
    :param turret_position: [x, y, z] 云台坐标系下的位置
    :param turret_orientation: [roll, pitch, yaw] 云台的姿态
    :param turret_to_vehicle_offset: [x, y, z] 云台相对于车体的固定偏移
    :return: 车体坐标系下的位置和方向
    """
    # 计算旋转矩阵
    roll, pitch, yaw = turret_orientation
    rotation_matrix = compute_rotation_matrix(roll, pitch, yaw)

    # 位置转换
    vehicle_position = np.dot(rotation_matrix, turret_position) + turret_to_vehicle_offset

    return vehicle_position

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