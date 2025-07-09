import numpy as np
from sklearn.cluster import DBSCAN
from sklearn.linear_model import LinearRegression
from sklearn.linear_model import RANSACRegressor

class LidarProcessor:
    def __init__(self, ground_threshold=0.2, cluster_distance=0.5, use_ransac=False):  
        # 可调参数：地面分割高度阈值（或 RANSAC 残差阈值）
        self.ground_threshold = ground_threshold  # 默认 0.2 m
        # 可调参数：DBSCAN 聚类半径 ε
        self.cluster_distance = cluster_distance  # 默认 0.5 m
        # 可选参数：是否启用 RANSAC 平面拟合剔除地面（默认 False）
        self.use_ransac = use_ransac             

    def filter_points(self, point_cloud, reflectivity):
        """
        根据反射率过滤低质量点。
        :param point_cloud: 点云数据，N×3 的数组。
        :param reflectivity: 点云反射率，N×1 的数组。
        :return: 过滤后的点云数据。
        """
        # 可调参数：反射率过滤阈值
        reflectivity_threshold = 0.5  # 默认阈值
        # 保留反射率高于阈值的点
        filtered_points = point_cloud[reflectivity > reflectivity_threshold]
        return filtered_points

    def segment_ground(self, point_cloud):
        """
        地面分割：根据高度阈值分离地面点和非地面点。
        :param point_cloud: 点云数据，N×3 的数组。
        :return: 非地面点云数据。
        """
        # 如果启用 RANSAC 平面拟合，则对 Z 当作因变量，XY 作为自变量拟合地面平面
        if self.use_ransac and point_cloud.shape[0] > 10:
            X = point_cloud[:, :2]
            y = point_cloud[:, 2]
            model = RANSACRegressor(LinearRegression(), residual_threshold=self.ground_threshold)
            model.fit(X, y)
            # inlier_mask=True 为地面点，取反保留非地面点
            non_ground_points = point_cloud[~model.inlier_mask_]
        else:
            # 简单高度阈值分割
            non_ground_points = point_cloud[point_cloud[:, 2] > self.ground_threshold]
        return non_ground_points

    def cluster_obstacles(self, point_cloud):
        """
        障碍物聚类：使用 DBSCAN 对点云进行聚类。
        :param point_cloud: 点云数据，N×3 的数组。
        :return: 聚类标签和点云数据。
        """
        # 可调参数：min_samples 最小聚类样本数
        min_samples = 5  # 默认最少点数
        clustering = DBSCAN(eps=self.cluster_distance, min_samples=min_samples).fit(point_cloud[:, :2])
        return clustering.labels_, point_cloud

    def process_point_cloud(self, point_cloud, reflectivity):
        """
        处理点云数据，包括过滤低质量点、地面分割和障碍物聚类。
        :param point_cloud: 点云数据，N×3 的数组。
        :param reflectivity: 点云反射率，N×1 的数组。
        :return: 聚类结果和非地面点云数据。
        """
        # 1. 过滤低质量点
        filtered_points = self.filter_points(point_cloud, reflectivity)

        # 2. 地面分割
        non_ground_points = self.segment_ground(filtered_points)

        # 3. 障碍物聚类
        cluster_labels, clustered_points = self.cluster_obstacles(non_ground_points)

        return cluster_labels, clustered_points

    def analyze_ground_slope(self, point_cloud: np.ndarray, distances: list) -> dict:
        """
        计算不同距离范围的地面坡度角。
        :param point_cloud: N×3 点云数组
        :param distances: 递增距离列表，例如 [1,2,3] 表示分段 [0,1], [1,2], [2,3]
        :return: dict，键为 (min_d, max_d)，值为坡度角（弧度），若点数不足返回 None
        """
        # 1. 提取地面点
        if self.use_ransac and point_cloud.shape[0] > 10:
            X = point_cloud[:, :2]
            y = point_cloud[:, 2]
            model = RANSACRegressor(LinearRegression(), residual_threshold=self.ground_threshold)
            model.fit(X, y)
            # inlier_mask_ 为地面点
            ground_pts = point_cloud[model.inlier_mask_]
        else:
            # 简易高度阈值
            ground_pts = point_cloud[point_cloud[:, 2] <= self.ground_threshold]

        slopes = {}
        prev_d = 0.0
        # 遍历每个距离区间
        for d in distances:
            # 环形区域掩码
            dist_xy = np.linalg.norm(ground_pts[:, :2], axis=1)
            mask = (dist_xy >= prev_d) & (dist_xy < d)
            seg_pts = ground_pts[mask]
            if seg_pts.shape[0] >= 3:
                # 平面拟合（最小二乘）
                X_seg = seg_pts[:, :2]
                z_seg = seg_pts[:, 2]
                lr = LinearRegression().fit(X_seg, z_seg)
                a, b = lr.coef_
                # 法向量
                normal = np.array([a, b, -1.0])
                normal /= np.linalg.norm(normal)
                # 坡度角 = arccos( |n·[0,0,1]| )
                slope = np.arccos(abs(normal.dot([0, 0, 1])))
                slopes[(prev_d, d)] = float(slope)
            else:
                slopes[(prev_d, d)] = None
            prev_d = d
        return slopes