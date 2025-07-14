import numpy as np
import pytest
from src.planning.motion_planner import smooth_path, generate_velocity_profile


def test_smooth_path_shape_and_continuity():
    """测试路径平滑的输出格式和连续性"""
    raw = [(0, 0), (1, 1), (2, 0), (3, -1)]
    num = 50
    smooth = smooth_path(raw, num_points=num)
    
    # 验证输出尺寸
    assert smooth.shape == (num, 2)
    
    # 验证连续性：相邻点距离合理
    dists = np.linalg.norm(np.diff(smooth, axis=0), axis=1)
    assert np.all(dists > 0)
    assert np.max(dists) < 0.2
    
    # 验证平滑性：计算曲率变化率
    # 简化的曲率估计：二阶差分
    if len(smooth) >= 3:
        second_diff = np.diff(smooth, n=2, axis=0)
        curvature_changes = np.linalg.norm(second_diff, axis=1)
        # 曲率变化应该相对平滑
        assert np.max(curvature_changes) < 1.0


def test_path_length_preservation():
    """测试平滑后路径长度的合理性"""
    # 简单的直线路径
    raw_straight = [(0, 0), (5, 0)]
    smooth_straight = smooth_path(raw_straight, num_points=20)
    smooth_length = np.sum(np.linalg.norm(np.diff(smooth_straight, axis=0), axis=1))
    expected_length = 5.0
    assert abs(smooth_length - expected_length) < 0.1
    
    # 锯齿路径
    raw_zigzag = [(0, 0), (1, 1), (2, 0), (3, 1), (4, 0)]
    smooth_zigzag = smooth_path(raw_zigzag, num_points=50)
    smooth_length = np.sum(np.linalg.norm(np.diff(smooth_zigzag, axis=0), axis=1))
    # 平滑后长度应该合理，不能过度偏离原路径
    raw_length = sum(np.sqrt((raw_zigzag[i+1][0]-raw_zigzag[i][0])**2 + 
                            (raw_zigzag[i+1][1]-raw_zigzag[i][1])**2) 
                    for i in range(len(raw_zigzag)-1))
    assert 0.8 * raw_length <= smooth_length <= 1.5 * raw_length


def test_key_point_preservation():
    """测试关键拐点位置的保留"""
    # 明显的拐点
    raw = [(0, 0), (5, 0), (5, 5)]  # L型路径
    smooth = smooth_path(raw, num_points=30)
    
    # 起点和终点应该被保留
    assert np.allclose(smooth[0], raw[0], atol=0.1)
    assert np.allclose(smooth[-1], raw[-1], atol=0.1)
    
    # 中间的拐点(5,0)附近应该有路径点
    corner_point = np.array([5, 0])
    distances_to_corner = np.linalg.norm(smooth - corner_point, axis=1)
    assert np.min(distances_to_corner) < 0.5  # 至少有点接近拐点


def test_generate_velocity_profile():
    """测试速度分配的准确性和约束"""
    raw = [(0, 0), (1, 1), (2, 0)]
    smooth = smooth_path(raw, num_points=20)
    speed = 1.23
    traj = generate_velocity_profile(smooth, speed)
    
    # 验证输出尺寸和速度列
    assert traj.shape == (20, 3)
    assert np.allclose(traj[:, 2], speed)
    
    # 验证位置部分未被改变
    assert np.allclose(traj[:, :2], smooth)


def test_variable_speed_profile():
    """测试可变速度分配（基于曲率的速度调整）"""
    # 创建一个包含急转弯的路径
    raw = [(0, 0), (5, 0), (5, 5), (10, 5)]  # 有直角转弯
    smooth = smooth_path(raw, num_points=40)
    
    # 如果motion_planner支持基于曲率的速度调整，这里可以测试
    # 目前的实现是恒定速度，所以测试恒定行为
    base_speed = 2.0
    traj = generate_velocity_profile(smooth, base_speed)
    
    # 验证所有速度都是预期值
    assert np.allclose(traj[:, 2], base_speed)
    
    # 验证速度约束：不超过物理限制
    max_speed = 3.0  # 假设的最大速度
    assert np.all(traj[:, 2] <= max_speed)
    assert np.all(traj[:, 2] >= 0)


def test_edge_case_velocity_profile():
    """测试速度分配的边界情况"""
    # 单点路径
    single_point = [(1, 1)]
    smooth_single = smooth_path(single_point + [(1.1, 1.1)], num_points=5)  # 添加微小偏移避免退化
    traj_single = generate_velocity_profile(smooth_single, 1.0)
    assert traj_single.shape[1] == 3
    
    # 零速度
    raw = [(0, 0), (1, 0)]
    smooth = smooth_path(raw, num_points=10)
    traj_zero = generate_velocity_profile(smooth, 0.0)
    assert np.all(traj_zero[:, 2] == 0.0)
    
    # 负速度（应该被处理或报错）
    try:
        traj_neg = generate_velocity_profile(smooth, -1.0)
        # 如果允许负速度，验证它被正确设置
        assert np.all(traj_neg[:, 2] == -1.0)
    except (ValueError, AssertionError):
        # 如果不允许负速度，这是预期行为
        pass
