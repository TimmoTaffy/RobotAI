from mapping.lidar_processor import filter_ground_and_cluster
from mapping.occupancy_grid import generate_occupancy_grid
from world_model import WorldModel

def build_map(point_cloud, grid_size, map_size, world_model: WorldModel):
    """
    构建地图，整合点云处理和栅格地图生成。
    :param point_cloud: N×3 的点云数据
    :param grid_size: 栅格大小
    :param map_size: 地图大小
    :return: 占据栅格地图
    """
    # 处理点云数据
    filtered_points, clusters = filter_ground_and_cluster(point_cloud)

    # 生成占据栅格地图
    occupancy_grid = generate_occupancy_grid(filtered_points, grid_size, map_size)
    
    # 将生成的栅格地图及动态障碍写入 WorldModel 对象
    world_model.occupancy_grid = occupancy_grid
    world_model.dynamic_obstacles = clusters
    return occupancy_grid, clusters