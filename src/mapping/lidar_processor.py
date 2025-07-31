import numpy as np
from sklearn.cluster import DBSCAN
from sklearn.linear_model import LinearRegression
from sklearn.linear_model import RANSACRegressor

class LidarProcessor:
    """
    激光雷达点云处理核心模块
    
    功能概述：
    • 地面分割：从点云中分离地面点和非地面点，支持简单阈值法和RANSAC平面拟合
    • 障碍聚类：使用DBSCAN对非地面点进行聚类，识别独立障碍物
    • 质量过滤：基于反射率过滤低质量激光点，提升后续处理稳定性
    • 坡度分析：分段分析地面坡度，为路径规划提供地形信息
    
    算法特点：
    • 双模式地面分割：平坦地形用阈值法（快速），坡地用RANSAC（精确）
    • 自适应聚类：DBSCAN自动确定聚类数量，有效处理噪声点
    • 分段坡度分析：环形区域的渐进式地面坡度计算
    """
    def __init__(self, ground_threshold=0.2, cluster_distance=0.5, use_ransac=False):  
        """
        初始化激光雷达处理器
        
        参数说明：
        :param ground_threshold: 地面分割阈值(m)
            - 简单模式：Z > threshold 的点为非地面点
            - RANSAC模式：平面拟合的残差阈值
            - 调优建议：室内0.1-0.2m，室外0.2-0.3m
        :param cluster_distance: DBSCAN聚类半径ε(m)
            - 两点距离<ε才可能在同一聚类中
            - 调优建议：密集环境0.3-0.4m，稀疏环境0.6-0.8m
        :param use_ransac: 是否启用RANSAC平面拟合地面分割
            - False：简单阈值法，速度快，适用平坦地形
            - True：RANSAC拟合，精度高，适用坡地环境
        """
        # 可调参数：地面分割高度阈值（或 RANSAC 残差阈值）
        self.ground_threshold = ground_threshold  # 默认 0.2 m
        # 可调参数：DBSCAN 聚类半径 ε
        self.cluster_distance = cluster_distance  # 默认 0.5 m
        # 可选参数：是否启用 RANSAC 平面拟合剔除地面（默认 False）
        self.use_ransac = use_ransac             

    def filter_points(self, point_cloud, reflectivity):
        """
        基于反射率过滤低质量激光点
        
        算法原理：
        • 激光雷达的反射率反映了目标表面的光学特性
        • 低反射率通常对应：
          - 远距离点（信号弱）
          - 透明/镜面材料（反射异常）  
          - 边缘点（部分遮挡）
          - 噪声点（随机干扰）
        
        过滤策略：
        • 保留 reflectivity > 0.5 的高质量点
        • 提升后续地面分割和聚类的稳定性
        • 减少噪声对算法的干扰
        
        :param point_cloud: 点云数据，N×3 的数组 [x, y, z]
        :param reflectivity: 点云反射率，N×1 的数组，取值范围[0,1]
        :return: 过滤后的点云数据，M×3（M≤N）
        """
        # 可调参数：反射率过滤阈值
        reflectivity_threshold = 0.5  # 默认阈值
        # 保留反射率高于阈值的点
        filtered_points = point_cloud[reflectivity > reflectivity_threshold]
        return filtered_points

    def segment_ground(self, point_cloud):
        """
        地面分割：从点云中分离地面点和非地面点
        
        双模式算法：
        
        RANSAC平面拟合模式（use_ransac=True）
        • 数学模型：Z = aX + bY + c（地面平面方程）
        • 算法流程：
          1. 将XY作为自变量，Z作为因变量
          2. RANSAC随机采样一致性算法拟合平面
          3. residual_threshold控制内点/外点判断
          4. inlier_mask_标识地面点，取反得到非地面点
        • 适用场景：倾斜地形、不规则地面、噪声较多的环境
        • 计算复杂度：O(N×迭代次数)，较慢但精确
        
        简单阈值模式（use_ransac=False）  
        • 数学模型：直接判断 Z > ground_threshold
        • 算法假设：地面基本水平，Z坐标接近0
        • 适用场景：平坦地形、室内环境、实时性要求高
        • 计算复杂度：O(N)，快速但对倾斜地面效果差
        
        参数调优指导：
        • ground_threshold: 室内0.1-0.2m，室外0.2-0.3m  
        • 点云数量需>10才启用RANSAC（避免过拟合）
        
        :param point_cloud: 输入点云数据，N×3 的数组 [x, y, z]
        :return: 非地面点云数据，M×3（M≤N）
        """
        # 如果启用 RANSAC 平面拟合，则对 Z 当作因变量，XY 作为自变量拟合地面平面
        if self.use_ransac and point_cloud.shape[0] > 10:
            X = point_cloud[:, :2]  # XY坐标作为特征
            y = point_cloud[:, 2]   # Z坐标作为目标
            model = RANSACRegressor(LinearRegression(), residual_threshold=self.ground_threshold)
            model.fit(X, y)
            # inlier_mask=True 为地面点，取反保留非地面点
            non_ground_points = point_cloud[~model.inlier_mask_]
        else:
            # 简单高度阈值分割：Z > threshold 的点为非地面点
            non_ground_points = point_cloud[point_cloud[:, 2] > self.ground_threshold]
        return non_ground_points

    def cluster_obstacles(self, point_cloud):
        """
        障碍物聚类：使用DBSCAN对非地面点进行密度聚类
        
        DBSCAN算法原理：
        • 密度可达性：如果点p的ε邻域内至少有min_samples个点，则p是核心点
        • 聚类形成：核心点及其密度可达的点形成一个聚类
        • 噪声点：不属于任何聚类的点标记为-1
        • 优势：自动确定聚类数量，有效处理异常点
        
        参数设计：
        • eps=cluster_distance: 邻域半径，决定聚类的紧密程度
          - 较小：生成更多细分的聚类（精细分割）
          - 较大：生成更少但范围更大的聚类（合并相近）
        • min_samples=5: 形成聚类的最少点数
          - 较小：更容易形成聚类（检测小目标）
          - 较大：更严格的聚类条件（降噪效果好）
        
        应用场景：
        • 多障碍物环境：区分不同的独立障碍物
        • 动态目标检测：运动物体的聚类跟踪
        • 噪声过滤：有效排除稀疏的噪声点
        
        :param point_cloud: 非地面点云数据，N×3 的数组
        :return: (聚类标签, 原点云数据)
            - 聚类标签：N×1数组，值≥0表示聚类ID，-1表示噪声点
            - 原点云数据：保持输入点云不变，便于后续处理
        """
        # 可调参数：min_samples 最小聚类样本数
        min_samples = 5  # 默认最少点数
        # 只使用XY坐标进行聚类（忽略高度变化）
        clustering = DBSCAN(eps=self.cluster_distance, min_samples=min_samples).fit(point_cloud[:, :2])
        return clustering.labels_, point_cloud

    def process_point_cloud(self, point_cloud, reflectivity):
        """
        点云处理完整流水线：质量过滤 → 地面分割 → 障碍聚类
        
        处理流程：
        1. 质量过滤：基于反射率移除低质量点，提升数据可靠性
        2. 地面分割：分离地面点和非地面点，专注障碍物检测
        3. 障碍聚类：对非地面点聚类，识别独立障碍物实体
        
        流水线设计考虑：
        • 顺序依赖：地面分割依赖高质量点云，聚类依赖地面分割结果
        • 数据量递减：每步骤都会减少待处理的点数，提升后续效率
        • 模块化：每个步骤可独立调优和调试
        
        应用场景：
        • 静态地图构建：识别环境中的静态障碍物结构
        • 动态目标检测：检测和跟踪运动中的物体
        • 路径规划支持：为导航算法提供障碍物分布信息
        
        :param point_cloud: 原始点云数据，N×3 的数组 [x, y, z]
        :param reflectivity: 点云反射率，N×1 的数组，取值[0,1]
        :return: (聚类标签, 聚类点云数据)
            - 聚类标签：最终聚类结果，标识每个点的归属
            - 聚类点云：经过处理的非地面点云
        """
        # 1. 过滤低质量点：基于反射率提升点云质量
        filtered_points = self.filter_points(point_cloud, reflectivity)

        # 2. 地面分割：分离地面和非地面点
        non_ground_points = self.segment_ground(filtered_points)

        # 3. 障碍物聚类：识别独立的障碍物实体
        cluster_labels, clustered_points = self.cluster_obstacles(non_ground_points)

        return cluster_labels, clustered_points

    def analyze_tactical_terrain(self, point_cloud: np.ndarray, 
                                grid_size: float, 
                                map_size: tuple,
                                height_advantage_factor: float = 0.9) -> np.ndarray:
        """
        战术地形分析：为路径规划生成战术优势图
        
        赛场实战导向：
        • 现实：赛场无陡坡，所有地形都可通行，坡度易行
        • 价值：坡度提供战术优势 - 居高临下 vs 仰攻劣势
        • 目标：识别战术高地，为AI决策提供地形优势信息
        
        战术地形分类：
        • HIGH_GROUND（高地）：相对周围较高，战术优势，代价0.9（更优选）
        • NORMAL_GROUND（平地）：普通地形，正常通行，代价1.0
        • LOW_GROUND（低地）：相对较低，战术劣势，代价1.1（轻微劣势）
        
        设计思路：
        • 所有区域都可通行（无np.inf禁止区域）
        • 高地略微优选（代价<1.0），鼓励抢占制高点
        • 低地轻微劣势（代价>1.0），避免被压制
        - 保持与A*算法的接口统一
        
        :param point_cloud: 输入点云数据，N×3 数组 [x, y, z]
        :param grid_size: 栅格大小，单位米
        :param map_size: 地图尺寸 (宽度, 高度)，单位米
        :param height_advantage_factor: 高地优势因子，<1.0表示更优选高地
        :return: 战术地形图，与occupancy_grid同尺寸
            - 0.9: 高地，战术优势，优先选择
            - 1.0: 平地，正常通行
            - 1.1: 低地，战术劣势，尽量避免
        """
        # 1. 提取地面点：复用现有的地面分割逻辑
        if self.use_ransac and point_cloud.shape[0] > 10:
            X = point_cloud[:, :2]
            y = point_cloud[:, 2]
            model = RANSACRegressor(LinearRegression(), residual_threshold=self.ground_threshold)
            model.fit(X, y)
            ground_points = point_cloud[model.inlier_mask_]
        else:
            # 简单阈值：Z ≤ threshold 的点为地面点
            ground_points = point_cloud[point_cloud[:, 2] <= self.ground_threshold]
        
        # 2. 初始化战术地形图：默认为正常地形
        grid_height = int(map_size[1] // grid_size)
        grid_width = int(map_size[0] // grid_size)
        tactical_terrain = np.ones((grid_height, grid_width), dtype=float)
        
        # 3. 计算全局高度统计，用于相对高度判断
        if len(ground_points) > 0:
            global_height_mean = np.mean(ground_points[:, 2])
            global_height_std = np.std(ground_points[:, 2])
        else:
            global_height_mean = 0.0
            global_height_std = 1.0
        
        # 4. 为每个栅格计算战术地形类型
        for i in range(grid_height):
            for j in range(grid_width):
                # 计算当前栅格的世界坐标范围
                x_min, x_max = j * grid_size, (j + 1) * grid_size
                y_min, y_max = i * grid_size, (i + 1) * grid_size
                
                # 提取当前栅格内的地面点
                mask = ((ground_points[:, 0] >= x_min) & (ground_points[:, 0] < x_max) &
                        (ground_points[:, 1] >= y_min) & (ground_points[:, 1] < y_max))
                cell_points = ground_points[mask]
                
                if len(cell_points) >= 3:
                    # 计算栅格平均高度
                    cell_height = np.mean(cell_points[:, 2])
                    
                    # 战术地形分类：基于相对高度而非绝对坡度
                    height_diff = cell_height - global_height_mean
                    
                    if height_diff > 0.5 * global_height_std:
                        # 高地：战术优势，路径规划更优选
                        tactical_terrain[i, j] = height_advantage_factor  # 0.9
                    elif height_diff < -0.5 * global_height_std:
                        # 低地：战术劣势，轻微避免
                        tactical_terrain[i, j] = 1.1
                    else:
                        # 平地：正常地形
                        tactical_terrain[i, j] = 1.0
                else:
                    # 点数不足，默认为正常地形
                    tactical_terrain[i, j] = 1.0
        
        return tactical_terrain
    
    def _calculate_relative_height(self, points: np.ndarray) -> float:
        """
        计算栅格的平均高度：用于战术地形分析
        
        战术考虑：
        • 高度本身就是战术优势的体现
        • 不需要复杂的坡度计算，只需要相对高度
        • 平均高度能很好地代表该区域的战术价值
        
        相比坡度分析的优势：
        • 计算简单：平均值计算，O(N)复杂度
        • 语义清晰：高度直接对应战术优势
        • 实用性强：符合"居高临下"的战术思路
        
        :param points: 栅格内的地面点，M×3数组
        :return: 平均高度值（米），用于战术评估
        """
        if len(points) < 1:
            return 0.0
        
        # 直接返回Z坐标的平均值
        return np.mean(points[:, 2])