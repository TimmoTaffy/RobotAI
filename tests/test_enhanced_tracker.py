"""
测试增强型目标跟踪器
"""
import pytest
import numpy as np
import time
from unittest.mock import Mock
from src.tracking.enhanced_tracker import EnhancedTargetTracker, KalmanTarget
from src.common.types import RobotInfo, VisionRobot


@pytest.fixture
def tracker():
    """创建测试用的跟踪器"""
    return EnhancedTargetTracker(
        dt=0.1,
        max_age=1.0,
        association_threshold=3.0,
        process_noise=0.5,
        measurement_noise=1.0
    )


@pytest.fixture
def mock_radar_data():
    """模拟雷达数据"""
    radar_data = Mock()
    radar_data.robots = [
        Mock(id=1, team='enemy', x=5.0, y=3.0),
        Mock(id=2, team='ally', x=2.0, y=1.0)
    ]
    return radar_data


@pytest.fixture
def mock_vision_data():
    """模拟视觉数据"""
    vision_data = Mock()
    vision_robot1 = Mock(id=1, team='enemy', x=5.1, y=3.1)
    vision_robot1.confidence = 0.9
    vision_robot2 = Mock(id=3, team='enemy', x=10.0, y=5.0)
    vision_robot2.confidence = 0.8
    
    vision_data.robots = [vision_robot1, vision_robot2]
    return vision_data


class TestEnhancedTargetTracker:
    
    def test_initialization(self, tracker):
        """测试跟踪器初始化"""
        assert tracker.dt == 0.1
        assert tracker.max_age == 1.0
        assert len(tracker.tracks) == 0
        assert tracker.next_id == 1000
        
        # 检查卡尔曼矩阵
        assert tracker.F.shape == (4, 4)
        assert tracker.H.shape == (2, 4)
        assert tracker.Q.shape == (4, 4)
        assert tracker.R.shape == (2, 2)
    
    def test_kalman_predict(self, tracker):
        """测试卡尔曼预测步骤"""
        track = KalmanTarget(
            id=1,
            team='enemy',
            state=np.array([1.0, 2.0, 0.5, 0.3]),  # x, y, vx, vy
            covariance=np.eye(4)
        )
        
        predicted_track = tracker.predict(track)
        
        # 检查状态预测（位置应该根据速度更新）
        expected_x = 1.0 + 0.5 * 0.1  # x + vx * dt
        expected_y = 2.0 + 0.3 * 0.1  # y + vy * dt
        
        assert np.isclose(predicted_track.state[0], expected_x)
        assert np.isclose(predicted_track.state[1], expected_y)
        assert predicted_track.state[2] == 0.5  # 速度不变
        assert predicted_track.state[3] == 0.3
    
    def test_kalman_update(self, tracker):
        """测试卡尔曼更新步骤"""
        track = KalmanTarget(
            id=1,
            team='enemy',
            state=np.array([1.0, 2.0, 0.5, 0.3]),
            covariance=np.eye(4) * 2.0
        )
        
        measurement = np.array([1.1, 1.9])  # 观测到的位置
        updated_track = tracker.update(track, measurement)
        
        # 状态应该向测量值调整
        assert updated_track.state[0] != 1.0  # x应该改变
        assert updated_track.state[1] != 2.0  # y应该改变
        
        # 协方差应该减小（更确定）
        assert np.trace(updated_track.covariance) < np.trace(np.eye(4) * 2.0)
    
    def test_mahalanobis_distance(self, tracker):
        """测试马哈拉诺比斯距离计算"""
        track = KalmanTarget(
            id=1,
            team='enemy',
            state=np.array([5.0, 3.0, 0.0, 0.0]),
            covariance=np.eye(4)
        )
        
        # 测试相同位置（距离应该很小）
        measurement1 = np.array([5.0, 3.0])
        dist1 = tracker._mahalanobis_distance(track, measurement1)
        assert dist1 < 1.0
        
        # 测试远距离位置
        measurement2 = np.array([10.0, 8.0])
        dist2 = tracker._mahalanobis_distance(track, measurement2)
        assert dist2 > dist1
    
    def test_threat_assessment(self, tracker):
        """测试威胁评估"""
        # 敌方目标
        enemy_track = KalmanTarget(
            id=1,
            team='enemy',
            state=np.array([2.0, 1.0, 1.0, 0.5]),  # 接近且快速移动
            detection_count=10
        )
        
        my_pos = (0.0, 0.0)
        my_vel = (0.0, 0.0)
        
        threat = tracker._assess_threat_level(enemy_track, my_pos, my_vel)
        assert threat > 0.5  # 应该是高威胁
        
        # 友方目标
        ally_track = KalmanTarget(
            id=2,
            team='ally',
            state=np.array([2.0, 1.0, 0.1, 0.1]),  # 友方且慢速
            detection_count=5
        )
        
        threat_ally = tracker._assess_threat_level(ally_track, my_pos, my_vel)
        assert threat_ally < threat  # 友方威胁应该更低
    
    def test_data_association(self, tracker):
        """测试数据关联"""
        # 预先设置一些轨迹
        track1 = KalmanTarget(
            id=1001,
            team='enemy',
            state=np.array([5.0, 3.0, 0.0, 0.0]),
            covariance=np.eye(4)
        )
        track2 = KalmanTarget(
            id=1002,
            team='ally',
            state=np.array([2.0, 1.0, 0.0, 0.0]),
            covariance=np.eye(4)
        )
        
        tracker.tracks[1001] = track1
        tracker.tracks[1002] = track2
        
        # 模拟检测数据
        detections = [
            (5.1, 3.1, 'enemy', 0.9),  # 应该关联到track1
            (1.9, 1.1, 'ally', 0.8),   # 应该关联到track2
            (10.0, 10.0, 'enemy', 0.7) # 新目标
        ]
        
        associations = tracker._associate_detections(detections)
        
        assert len(associations['matched']) == 2  # 应该有2个匹配
        assert len(associations['unmatched_detections']) == 1  # 1个未匹配检测
        assert len(associations['unmatched_tracks']) == 0  # 0个未匹配轨迹
    
    def test_process_detections_radar_only(self, tracker, mock_radar_data):
        """测试仅雷达数据处理"""
        targets = tracker.process_detections(
            radar_data=mock_radar_data,
            my_position=(0.0, 0.0)
        )
        
        assert len(targets) == 2  # 应该创建2个新轨迹
        assert len(tracker.tracks) == 2
        
        # 检查状态初始化
        for target in targets:
            assert target.detection_count == 1
            assert target.confidence == 1.0
            assert target.team in ['enemy', 'ally']
    
    def test_process_detections_fusion(self, tracker, mock_radar_data, mock_vision_data):
        """测试雷达和视觉数据融合"""
        targets = tracker.process_detections(
            radar_data=mock_radar_data,
            vision_data=mock_vision_data,
            my_position=(0.0, 0.0)
        )
        
        # 应该有雷达的2个目标 + 视觉的1个新目标 = 3个
        assert len(targets) >= 2
        
        # 验证数据融合（相同ID的目标应该被关联）
        enemy_targets = [t for t in targets if t.team == 'enemy']
        assert len(enemy_targets) >= 1
    
    def test_prediction(self, tracker):
        """测试位置预测"""
        # 创建一个移动的目标
        track = KalmanTarget(
            id=1001,
            team='enemy',
            state=np.array([5.0, 3.0, 1.0, 0.5]),  # 有速度
            last_update=time.time()
        )
        tracker.tracks[1001] = track
        
        # 预测0.5秒后的位置
        future_time = time.time() + 0.5
        pred_pos = tracker.predict_target_position(1001, future_time)
        
        assert pred_pos is not None
        expected_x = 5.0 + 1.0 * 0.5  # x + vx * dt
        expected_y = 3.0 + 0.5 * 0.5  # y + vy * dt
        
        assert np.isclose(pred_pos[0], expected_x, atol=0.1)
        assert np.isclose(pred_pos[1], expected_y, atol=0.1)
    
    def test_high_priority_targets(self, tracker):
        """测试高优先级目标获取"""
        # 创建不同威胁等级的目标
        high_threat = KalmanTarget(id=1, team='enemy', threat_level=0.8, confidence=0.9)
        medium_threat = KalmanTarget(id=2, team='enemy', threat_level=0.5, confidence=0.7)
        low_threat = KalmanTarget(id=3, team='ally', threat_level=0.2, confidence=0.8)
        
        tracker.tracks[1] = high_threat
        tracker.tracks[2] = medium_threat
        tracker.tracks[3] = low_threat
        
        priority_targets = tracker.get_high_priority_targets(
            threat_threshold=0.6,
            max_targets=2
        )
        
        assert len(priority_targets) == 1  # 只有high_threat超过阈值
        assert priority_targets[0].id == 1
    
    def test_track_lifecycle(self, tracker, mock_radar_data):
        """测试轨迹生命周期管理"""
        # 第一次处理 - 创建轨迹
        targets1 = tracker.process_detections(radar_data=mock_radar_data)
        initial_count = len(targets1)
        
        # 验证轨迹创建
        assert initial_count > 0
        for target in targets1:
            assert target.detection_count == 1
            assert target.miss_count == 0
        
        # 第二次处理相同数据 - 更新轨迹
        targets2 = tracker.process_detections(radar_data=mock_radar_data)
        
        assert len(targets2) == initial_count  # 轨迹数量不变
        for target in targets2:
            assert target.detection_count == 2  # 检测计数增加
            assert target.miss_count == 0
        
        # 第三次处理空数据 - 轨迹丢失
        empty_radar = Mock()
        empty_radar.robots = []
        
        targets3 = tracker.process_detections(radar_data=empty_radar)
        
        for target in targets3:
            assert target.miss_count == 1  # 丢失计数增加
            assert target.confidence < 1.0  # 置信度下降
    
    def test_statistics(self, tracker, mock_radar_data):
        """测试性能统计"""
        initial_stats = tracker.get_statistics()
        assert initial_stats['total_detections'] == 0
        assert initial_stats['active_tracks'] == 0
        
        # 处理一些数据
        tracker.process_detections(radar_data=mock_radar_data)
        
        final_stats = tracker.get_statistics()
        assert final_stats['total_detections'] > 0
        assert final_stats['new_tracks'] > 0
        assert final_stats['active_tracks'] > 0
        assert 0 <= final_stats['association_rate'] <= 1
    
    def test_reset(self, tracker, mock_radar_data):
        """测试重置功能"""
        # 处理一些数据
        tracker.process_detections(radar_data=mock_radar_data)
        assert len(tracker.tracks) > 0
        
        # 重置
        tracker.reset()
        
        assert len(tracker.tracks) == 0
        assert tracker.next_id == 1000
        stats = tracker.get_statistics()
        assert all(v == 0 for v in stats.values() if v != 0.0)  # 除了比率都应该是0
