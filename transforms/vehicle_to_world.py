import numpy as np

def transform_vehicle_to_world(position, velocity, vehicle_pose):
    """
    将车体坐标系下的位置和速度转换到世界坐标系
    :param position: [x, y] 车体坐标系下的位置
    :param velocity: [vx, vy] 车体坐标系下的速度
    :param vehicle_pose: [x, y, theta] 自车在世界坐标系下的位置和朝向
    :return: 世界坐标系下的位置和速度
    """
    x, y, theta = vehicle_pose

    # 位置转换
    rotation_matrix = np.array([[np.cos(theta), -np.sin(theta)],
                                [np.sin(theta), np.cos(theta)]])
    world_position = np.dot(rotation_matrix, position) + np.array([x, y])

    # 速度转换
    world_velocity = np.dot(rotation_matrix, velocity)

    return world_position, world_velocity