import numpy as np

def filter_ground_and_cluster(point_cloud, ground_threshold=0.2):
    """
    对点云数据进行地面滤波和障碍物聚类。
    :param point_cloud: N×3 的点云数据
    :param ground_threshold: 地面高度阈值
    :return: 过滤后的点云和聚类结果
    """
    # 地面滤波
    ground_mask = point_cloud[:, 2] < ground_threshold
    non_ground_points = point_cloud[~ground_mask]

    # 简单聚类（基于距离的聚类）
    clusters = cluster_points(non_ground_points)

    return non_ground_points, clusters

def cluster_points(points, distance_threshold=0.5):
    """简单的基于距离的点云聚类"""
    from sklearn.cluster import DBSCAN
    clustering = DBSCAN(eps=distance_threshold, min_samples=5).fit(points[:, :2])
    return clustering.labels_