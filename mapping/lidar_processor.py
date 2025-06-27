import numpy as np
from sklearn.cluster import DBSCAN

class LidarProcessor:
    def __init__(self, ground_threshold=0.2, cluster_distance=0.5):
        self.ground_threshold = ground_threshold  # 地面高度阈值
        self.cluster_distance = cluster_distance  # 聚类距离阈值

    def filter_points(self, point_cloud, reflectivity):
        """
        根据反射率过滤低质量点。
        :param point_cloud: 点云数据，N×3 的数组。
        :param reflectivity: 点云反射率，N×1 的数组。
        :return: 过滤后的点云数据。
        """
        # 保留反射率高于阈值的点
        filtered_points = point_cloud[reflectivity > 0.5]
        return filtered_points

    def segment_ground(self, point_cloud):
        """
        地面分割：根据高度阈值分离地面点和非地面点。
        :param point_cloud: 点云数据，N×3 的数组。
        :return: 非地面点云数据。
        """
        non_ground_points = point_cloud[point_cloud[:, 2] > self.ground_threshold]
        return non_ground_points

    def cluster_obstacles(self, point_cloud):
        """
        障碍物聚类：使用 DBSCAN 对点云进行聚类。
        :param point_cloud: 点云数据，N×3 的数组。
        :return: 聚类标签和点云数据。
        """
        clustering = DBSCAN(eps=self.cluster_distance, min_samples=5).fit(point_cloud[:, :2])
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