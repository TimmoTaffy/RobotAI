import numpy as np
import pytest
from common.types import Pose2D
from control.trajectory_tracker import TrajectoryTracker


def make_straight_traj(length=10, step=1.0, speed=1.0):
    """创建直线轨迹辅助函数"""
    xs = np.arange(0, length, step)
    ys = np.zeros_like(xs)
    vs = np.full_like(xs, speed)
    return np.vstack((xs, ys, vs)).T


def make_circular_traj(radius=5.0, num_points=20, speed=1.0):
    """创建圆形轨迹辅助函数"""
    angles = np.linspace(0, 2*np.pi, num_points)
    xs = radius * np.cos(angles)
    ys = radius * np.sin(angles)
    vs = np.full_like(xs, speed)
    return np.vstack((xs, ys, vs)).T


def test_straight_line_tracking_accuracy():
    """测试直线轨迹跟踪精度"""
    traj = make_straight_traj(length=10, step=1.0)
    current = Pose2D(position=np.array([0.0, 0.0]), theta=0.0, timestamp=0.0)
    tracker = TrajectoryTracker(lookahead_distance=2.0)
    v, steering = tracker.update(current, traj)
    
    # 验证输出类型和范围
    assert isinstance(v, (int, float))
    assert isinstance(steering, (int, float))
    assert v >= 0  # 速度非负
    assert abs(steering) <= np.pi/2  # 转向角合理范围
    
    # 直线跟踪：转向角应该很小
    assert abs(steering) < 0.1


def test_lookahead_distance_sensitivity():
    """测试不同前瞻距离的影响"""
    traj = make_straight_traj(length=10, step=0.5)
    current = Pose2D(position=np.array([0.0, 0.0]), theta=0.0, timestamp=0.0)
    
    lookaheads = [0.5, 1.0, 1.5, 2.0]
    steerings = []
    
    for ld in lookaheads:
        tracker = TrajectoryTracker(lookahead_distance=ld)
        v, steering = tracker.update(current, traj)
        steerings.append(abs(steering))
    
    # 对于直线，所有前瞻距离都应产生小转向角
    for s in steerings:
        assert s < 0.1


def test_curved_path_tracking():
    """测试曲线路径跟踪能力"""
    traj = make_circular_traj(radius=5.0, num_points=50)
    # 从圆上某点开始，朝向切线方向
    start_angle = np.pi/4
    current = Pose2D(
        position=np.array([5*np.cos(start_angle), 5*np.sin(start_angle)]), 
        theta=start_angle + np.pi/2,  # 切线方向
        timestamp=0.0
    )
    
    tracker = TrajectoryTracker(lookahead_distance=1.0)
    v, steering = tracker.update(current, traj)
    
    # 曲线跟踪应产生明显转向
    assert abs(steering) > 0.05
    assert v > 0


def test_large_heading_error_correction():
    """测试大航向误差的纠正能力"""
    traj = make_straight_traj(length=5, step=1.0)
    # 航向偏差90度
    current = Pose2D(position=np.array([0.0, 0.0]), theta=np.pi/2, timestamp=0.0)
    tracker = TrajectoryTracker(lookahead_distance=1.0)
    v, steering = tracker.update(current, traj)
    
    # 应产生明显的纠正转向
    assert abs(steering) > 0.3
    assert v > 0


def test_lateral_offset_correction():
    """测试横向偏移的纠正能力"""
    traj = make_straight_traj(length=10, step=1.0)
    # 横向偏移1米
    current = Pose2D(position=np.array([0.0, 1.0]), theta=0.0, timestamp=0.0)
    tracker = TrajectoryTracker(lookahead_distance=2.0)
    v, steering = tracker.update(current, traj)
    
    # 应产生向轨迹靠拢的转向
    assert steering < 0  # 向右转向轨迹
    assert abs(steering) > 0.1


def test_trajectory_completion():
    """测试轨迹末端处理"""
    short_traj = make_straight_traj(length=3, step=1.0)
    # 接近轨迹终点
    current = Pose2D(position=np.array([2.5, 0.0]), theta=0.0, timestamp=0.0)
    tracker = TrajectoryTracker(lookahead_distance=2.0)
    v, steering = tracker.update(current, short_traj)
    
    # 接近终点时仍应有合理控制输出
    assert v >= 0
    assert abs(steering) < np.pi/2


def test_speed_control_response():
    """测试速度控制响应"""
    high_speed_traj = make_straight_traj(length=10, step=1.0, speed=2.0)
    current = Pose2D(position=np.array([0.0, 0.0]), theta=0.0, timestamp=0.0)
    
    # 测试不同增益
    tracker1 = TrajectoryTracker(kp_speed=1.0)
    tracker2 = TrajectoryTracker(kp_speed=0.5)
    
    v1, _ = tracker1.update(current, high_speed_traj)
    v2, _ = tracker2.update(current, high_speed_traj)
    
    # 更高增益应产生更高速度输出
    assert v1 > v2
