"""
测试增强版 WorldModel 功能
"""
import pytest
import time
import numpy as np
from world_model import WorldModel, TacticalInfo
from src.common.types import Pose2D, TurretState
from src.tracking.enhanced_tracker import KalmanTarget


def create_test_pose():
    """创建测试用位姿"""
    return Pose2D(
        position=np.array([0.0, 0.0]),
        theta=0.0,
        timestamp=time.time()
    )


def create_test_worldmodel(tracked_targets=None):
    """创建测试用 WorldModel"""
    if tracked_targets is None:
        tracked_targets = []
    
    return WorldModel(
        self_pose=create_test_pose(),
        turret_state=TurretState(
            timestamp=time.time(),
            frame_id="turret",
            orientation=np.array([0.0, 0.0, 0.0]),     # [roll, pitch, yaw]
            angular_velocity=np.array([0.0, 0.0, 0.0]), # [wx, wy, wz]
            motor_angles=np.array([0.0, 0.0])          # [horizontal, vertical]
        ),
        task_points=[],
        static_obstacles=[],
        dynamic_obstacles=[],
        occupancy_grid=np.zeros((100, 100)),
        ground_slopes={},
        tracked_targets=tracked_targets,
        tactical_info=TacticalInfo()
    )


class TestWorldModel:
    """测试 WorldModel 基本功能"""
    
    def test_world_model_initialization(self):
        """测试 WorldModel 初始化"""
        wm = create_test_worldmodel()
        
        assert wm.self_pose.position[0] == 0.0
        assert wm.tracked_targets == []
        assert wm.tactical_info.threat_level == 0.0
        assert wm.tactical_info.recommended_action == "PATROL"
        assert wm.system_health['sensors'] is True

    def test_get_enemy_targets(self):
        """测试获取敌方目标"""
        # 创建测试目标
        enemy1 = KalmanTarget(id=1, team='enemy')
        enemy1.state = np.array([1.0, 1.0, 0.0, 0.0])
        enemy2 = KalmanTarget(id=2, team='enemy')
        enemy2.state = np.array([2.0, 2.0, 0.0, 0.0])
        ally1 = KalmanTarget(id=3, team='ally')
        ally1.state = np.array([3.0, 3.0, 0.0, 0.0])
        
        wm = create_test_worldmodel([enemy1, enemy2, ally1])
        
        enemies = wm.get_enemy_targets()
        assert len(enemies) == 2
        assert all(target.team == 'enemy' for target in enemies)

    def test_get_ally_targets(self):
        """测试获取友方目标"""
        enemy1 = KalmanTarget(id=1, team='enemy')
        enemy1.state = np.array([1.0, 1.0, 0.0, 0.0])
        ally1 = KalmanTarget(id=2, team='ally')
        ally1.state = np.array([2.0, 2.0, 0.0, 0.0])
        ally2 = KalmanTarget(id=3, team='ally')
        ally2.state = np.array([3.0, 3.0, 0.0, 0.0])
        
        wm = create_test_worldmodel([enemy1, ally1, ally2])
        
        allies = wm.get_ally_targets()
        assert len(allies) == 2
        assert all(target.team == 'ally' for target in allies)

    def test_get_highest_threat_target(self):
        """测试获取最高威胁目标"""
        enemy1 = KalmanTarget(id=1, team='enemy')
        enemy1.state = np.array([1.0, 1.0, 0.0, 0.0])
        enemy1.threat_level = 0.7
        enemy2 = KalmanTarget(id=2, team='enemy')
        enemy2.state = np.array([2.0, 2.0, 0.0, 0.0])
        enemy2.threat_level = 0.9  # 最高威胁
        enemy3 = KalmanTarget(id=3, team='enemy')
        enemy3.state = np.array([3.0, 3.0, 0.0, 0.0])
        enemy3.threat_level = 0.3
        
        wm = create_test_worldmodel([enemy1, enemy2, enemy3])
        
        highest_threat = wm.get_highest_threat_target()
        assert highest_threat.id == 2
        assert highest_threat.threat_level == 0.9

    def test_get_highest_threat_target_no_enemies(self):
        """测试无敌方目标时获取最高威胁目标"""
        ally1 = KalmanTarget(id=1, team='ally')
        ally1.state = np.array([1.0, 1.0, 0.0, 0.0])
        
        wm = create_test_worldmodel([ally1])
        
        highest_threat = wm.get_highest_threat_target()
        assert highest_threat is None


class TestTacticalAssessment:
    """测试战术评估功能"""
    
    def test_update_tactical_assessment_no_enemies(self):
        """测试无敌方目标时的战术评估"""
        wm = create_test_worldmodel()
        
        wm.update_tactical_assessment()
        
        assert wm.tactical_info.threat_level == 0.0
        assert wm.tactical_info.recommended_action == "PATROL"
        assert wm.tactical_info.priority_targets == []

    def test_update_tactical_assessment_low_threat(self):
        """测试低威胁情况的战术评估"""
        enemy1 = KalmanTarget(id=1, team='enemy')
        enemy1.state = np.array([1.0, 1.0, 0.0, 0.0])
        enemy1.threat_level = 0.3
        enemy2 = KalmanTarget(id=2, team='enemy')
        enemy2.state = np.array([2.0, 2.0, 0.0, 0.0])
        enemy2.threat_level = 0.2
        
        wm = create_test_worldmodel([enemy1, enemy2])
        
        wm.update_tactical_assessment()
        
        expected_threat = 0.3 * 0.7 + 0.25 * 0.3  # max * 0.7 + avg * 0.3
        assert abs(wm.tactical_info.threat_level - expected_threat) < 0.01
        assert wm.tactical_info.recommended_action == "PATROL"
        assert wm.tactical_info.priority_targets == [1, 2]  # 按威胁等级排序

    def test_update_tactical_assessment_medium_threat(self):
        """测试中等威胁情况的战术评估"""
        enemy1 = KalmanTarget(id=1, team='enemy')
        enemy1.state = np.array([1.0, 1.0, 0.0, 0.0])
        enemy1.threat_level = 0.6
        
        wm = create_test_worldmodel([enemy1])
        
        wm.update_tactical_assessment()
        
        assert wm.tactical_info.threat_level == 0.6
        assert wm.tactical_info.recommended_action == "TRACKING"

    def test_update_tactical_assessment_high_threat(self):
        """测试高威胁情况的战术评估"""
        enemy1 = KalmanTarget(id=1, team='enemy')
        enemy1.state = np.array([1.0, 1.0, 0.0, 0.0])
        enemy1.threat_level = 0.9
        
        wm = create_test_worldmodel([enemy1])
        
        wm.update_tactical_assessment()
        
        assert wm.tactical_info.threat_level == 0.9
        assert wm.tactical_info.recommended_action == "ENGAGE"

    def test_priority_targets_ordering(self):
        """测试优先目标排序"""
        enemy1 = KalmanTarget(id=1, team='enemy')
        enemy1.state = np.array([1.0, 1.0, 0.0, 0.0])
        enemy1.threat_level = 0.5
        enemy2 = KalmanTarget(id=2, team='enemy')
        enemy2.state = np.array([2.0, 2.0, 0.0, 0.0])
        enemy2.threat_level = 0.9  # 最高威胁
        enemy3 = KalmanTarget(id=3, team='enemy')
        enemy3.state = np.array([3.0, 3.0, 0.0, 0.0])
        enemy3.threat_level = 0.7
        enemy4 = KalmanTarget(id=4, team='enemy')
        enemy4.state = np.array([4.0, 4.0, 0.0, 0.0])
        enemy4.threat_level = 0.3
        
        wm = create_test_worldmodel([enemy1, enemy2, enemy3, enemy4])
        
        wm.update_tactical_assessment()
        
        # 应该按威胁等级降序排列，只保留前3个
        assert wm.tactical_info.priority_targets == [2, 3, 1]  # 0.9, 0.7, 0.5
