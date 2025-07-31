import numpy as np
from src.mapping.lidar_processor import LidarProcessor
from src.mapping.occupancy_grid import generate_occupancy_grid
from world_model import WorldModel

def build_map(point_cloud, grid_size, map_size, world_model: WorldModel, map_conf: dict):
    """
    地图构建主流程：整合激光雷达处理、地形分析和栅格地图生成
    
    功能概述：
    地图构建是环境感知的核心模块，将原始激光点云转换为机器人可用的导航地图，
    集成了战术地形分析、障碍物检测、栅格化等多个算法模块，
    直接更新WorldModel，为后续的路径规划和控制提供环境信息。
    
    处理流水线：
    1. LidarProcessor初始化：根据配置参数创建点云处理器
    2. 战术地形分析：分析地形高度优势，生成战术代价图
    3. 地面分割和聚类：识别地面点和障碍物，进行聚类分组
    4. 栅格地图生成：将三维点云转换为二维占用栅格
    5. WorldModel更新：将处理结果同步到全局世界模型
    
    配置参数：
    - ground_threshold: 地面分割阈值，影响地面点识别精度
    - cluster_distance: 聚类半径，控制障碍物分组粒度
    - use_ransac: 地面拟合方法，影响倾斜地形的处理效果
    - terrain_analysis: 战术地形分析配置
    
    :param point_cloud: 激光雷达点云数据，N×3 数组 [x, y, z]
    :param grid_size: 栅格地图的分辨率，单位米/格
    :param map_size: 地图的物理尺寸，[宽度, 高度]，单位米
    :param world_model: 全局世界模型，用于存储处理结果
    :param map_conf: 地图处理配置字典，包含各算法参数
    :return: (占用栅格地图, 聚类标签)
        - occupancy_grid: 二维numpy数组，0表示空闲，1表示占用
        - cluster_labels: 障碍物聚类结果，用于动态目标跟踪
    """
    # 初始化点云处理器
    processor = LidarProcessor(
        ground_threshold=map_conf.get('ground_threshold', 0.2),
        cluster_distance=map_conf.get('cluster_distance', 0.5),
        use_ransac=map_conf.get('use_ransac', False)
    )
    
    # 1. 战术地形分析：为路径规划提供战术优势信息
    terrain_conf = map_conf.get('terrain_analysis', {})
    if terrain_conf.get('enable', True):
        tactical_terrain_map = processor.analyze_tactical_terrain(
            point_cloud,
            grid_size,
            map_size,
            height_advantage_factor=terrain_conf.get('height_advantage_factor', 0.9)
        )
        world_model.terrain_cost_map = tactical_terrain_map
    else:
        # 地形分析禁用时，生成全平地代价图
        grid_height = int(map_size[1] // grid_size)
        grid_width = int(map_size[0] // grid_size)
        world_model.terrain_cost_map = np.ones((grid_height, grid_width))
    
    # 2. 地面分割：分离地面点和非地面点
    non_ground = processor.segment_ground(point_cloud)
    
    # 3. 障碍物聚类：识别独立的障碍物实体
    cluster_labels, clustered_points = processor.cluster_obstacles(non_ground)
    
    # 4. 生成栅格地图：将三维点云离散化为二维导航地图
    occupancy_grid = generate_occupancy_grid(non_ground, grid_size, map_size)
    
    # 5. 更新世界模型：将处理结果同步到全局状态
    world_model.occupancy_grid = occupancy_grid
    world_model.dynamic_obstacles = cluster_labels
    
    return occupancy_grid, cluster_labels