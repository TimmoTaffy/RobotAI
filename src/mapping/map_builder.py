from src.mapping.lidar_processor import LidarProcessor
from src.mapping.occupancy_grid import generate_occupancy_grid
from world_model import WorldModel

def build_map(point_cloud, grid_size, map_size, world_model: WorldModel, map_conf: dict):
    """
    构建地图，整合点云处理、地面坡度分析和栅格地图生成。
    :param point_cloud: N×3 的点云数据
    :param grid_size: 栅格大小
    :param map_size: 地图大小
    :param map_conf: map 配置字典，包含地面/聚类阈值及坡度分段
    :return: 占据栅格地图和动态障碍列表
    """
    # 初始化点云处理器
    processor = LidarProcessor(
        ground_threshold=map_conf.get('ground_threshold', 0.2),
        cluster_distance=map_conf.get('cluster_distance', 0.5),
        use_ransac=map_conf.get('use_ransac', False)
    )
    # 1. 地面坡度分析
    slopes = processor.analyze_ground_slope(
        point_cloud,
        map_conf.get('slope_distances', [])
    )
    world_model.ground_slopes = slopes
    # 2. 地面分割并聚类障碍
    non_ground = processor.segment_ground(point_cloud)
    cluster_labels, clustered_points = processor.cluster_obstacles(non_ground)
    # 3. 生成栅格地图
    occupancy_grid = generate_occupancy_grid(non_ground, grid_size, map_size)
    # 4. 更新 WorldModel
    world_model.occupancy_grid = occupancy_grid
    world_model.dynamic_obstacles = cluster_labels
    return occupancy_grid, cluster_labels