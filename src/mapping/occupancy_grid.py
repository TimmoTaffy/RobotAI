import numpy as np

def generate_occupancy_grid(point_cloud, grid_size, map_size):
    """
    根据点云数据生成占据栅格地图。
    :param point_cloud: N×3 的点云数据
    :param grid_size: 栅格大小（每个栅格的边长）
    :param map_size: 地图大小（宽和高）
    :return: 占据栅格地图
    """
    grid = np.zeros((map_size[0] // grid_size, map_size[1] // grid_size))

    for point in point_cloud:
        x, y = int(point[0] // grid_size), int(point[1] // grid_size)
        if 0 <= x < grid.shape[0] and 0 <= y < grid.shape[1]:
            grid[x, y] = 1

    return grid