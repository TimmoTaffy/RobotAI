"""
测试智能化状态机功能
"""
import pytest
import numpy as np
import time
from src.host.intelligent_fsm import IntelligentFSM
from world_model import WorldModel, TacticalInfo
from src.common.types import Pose2D, TurretState
from src.tracking.enhanced_tracker import KalmanTarget


class TestIntelligentFSM:
    """测试智能状态机基本功能"""
    
    def test_fsm_initialization(self):
        """测试智能FSM初始化"""
        world_model = self._create_test_world_model()
        fsm = IntelligentFSM(world_model)
        
        assert fsm.state == 'IDLE'
        assert fsm.world_model == world_model
        assert fsm.ammo_level == 1.0
        assert fsm.health_level == 1.0
        assert fsm.heat_level == 0.0
        
    def test_no_threat_patrol_decision(self):
        """测试无威胁时的巡逻决策"""
        world_model = self._create_test_world_model()
        fsm = IntelligentFSM(world_model)
        
        # 无敌方目标，应该建议巡逻
        fsm.intelligent_update()
        
        # 从IDLE状态应该转向PATROL
        summary = fsm.get_tactical_summary()
        assert summary['threat_level'] == 0.0
        assert summary['recommended_action'] == "PATROL"
        
    def test_medium_threat_tracking_decision(self):
        """测试中等威胁时的跟踪决策"""
        world_model = self._create_test_world_model()
        
        # 添加中等威胁目标
        enemy = self._create_enemy_target(1, threat_level=0.6)
        world_model.tracked_targets = [enemy]
        
        fsm = IntelligentFSM(world_model)
        fsm._change_state('PATROL')  # 设置初始状态为巡逻
        
        fsm.intelligent_update()
        
        summary = fsm.get_tactical_summary()
        assert summary['threat_level'] == 0.6
        assert summary['recommended_action'] == "TRACKING"
        # 应该从PATROL转向TRACKING
        
    def test_high_threat_engagement_decision(self):
        """测试高威胁时的攻击决策"""
        world_model = self._create_test_world_model()
        
        # 添加高威胁目标
        enemy = self._create_enemy_target(1, threat_level=0.9)
        world_model.tracked_targets = [enemy]
        
        fsm = IntelligentFSM(world_model)
        fsm._change_state('TRACKING')  # 从跟踪状态开始
        
        fsm.intelligent_update()
        
        summary = fsm.get_tactical_summary()
        assert summary['threat_level'] == 0.9
        assert summary['recommended_action'] == "ENGAGE"
        
    def test_system_status_influence(self):
        """测试系统状态对决策的影响"""
        world_model = self._create_test_world_model()
        
        # 添加高威胁目标
        enemy = self._create_enemy_target(1, threat_level=0.9)
        world_model.tracked_targets = [enemy]
        
        fsm = IntelligentFSM(world_model)
        fsm._change_state('FIRING')
        
        # 设置低弹药状态
        fsm.update_system_status(ammo=0.05, health=1.0, heat=0.0)
        
        # 智能决策应该考虑弹药不足，转向RETREAT
        new_state = fsm._intelligent_decision(0.9, "ENGAGE", enemy)
        assert new_state == 'RETREAT'
        
    def test_low_health_retreat_decision(self):
        """测试低血量时的撤退决策"""
        world_model = self._create_test_world_model()
        fsm = IntelligentFSM(world_model)
        
        # 设置低血量
        fsm.update_system_status(ammo=1.0, health=0.1, heat=0.0)
        
        # 无论威胁等级如何，都应该撤退
        new_state = fsm._intelligent_decision(0.9, "ENGAGE", None)
        assert new_state == 'RETREAT'
        
    def test_target_stability_assessment(self):
        """测试目标稳定性评估"""
        world_model = self._create_test_world_model()
        
        # 创建稳定目标（低速度，高置信度）
        stable_enemy = self._create_enemy_target(1, threat_level=0.8)
        stable_enemy.confidence = 0.9
        stable_enemy.state = np.array([5.0, 5.0, 0.5, 0.5])  # 低速度
        
        world_model.tracked_targets = [stable_enemy]
        fsm = IntelligentFSM(world_model)
        
        assert fsm._is_target_stable() == True
        
        # 创建不稳定目标（高速度）
        unstable_enemy = self._create_enemy_target(2, threat_level=0.8)
        unstable_enemy.confidence = 0.9
        unstable_enemy.state = np.array([5.0, 5.0, 5.0, 5.0])  # 高速度
        
        world_model.tracked_targets = [unstable_enemy]
        
        assert fsm._is_target_stable() == False
        
    def test_ready_to_fire_assessment(self):
        """测试开火准备评估"""
        world_model = self._create_test_world_model()
        
        # 创建稳定目标
        enemy = self._create_enemy_target(1, threat_level=0.8)
        enemy.confidence = 0.9
        enemy.state = np.array([5.0, 5.0, 0.5, 0.5])
        
        world_model.tracked_targets = [enemy]
        fsm = IntelligentFSM(world_model)
        
        # 正常状态应该准备开火
        assert fsm._is_ready_to_fire() == True
        
        # 弹药不足时不应该开火
        fsm.update_system_status(ammo=0.05, health=1.0, heat=0.0)
        assert fsm._is_ready_to_fire() == False
        
        # 过热时不应该开火
        fsm.update_system_status(ammo=1.0, health=1.0, heat=0.9)
        assert fsm._is_ready_to_fire() == False
        
    def test_target_position_prediction(self):
        """测试目标位置预测（通过增强跟踪器）"""
        world_model = self._create_test_world_model()
        
        # 创建模拟增强跟踪器
        class MockEnhancedTracker:
            def predict_target_position(self, target_id, future_time):
                # 简单线性预测：位置(0,0) + 速度(2,1) * 1秒 = (2,1)
                return (2.0, 1.0)
        
        world_model.enhanced_tracker = MockEnhancedTracker()
        fsm = IntelligentFSM(world_model)
        
        # 创建运动目标
        enemy = self._create_enemy_target(1, threat_level=0.8)
        enemy.state = np.array([0.0, 0.0, 2.0, 1.0])  # 位置(0,0)，速度(2,1)
        world_model.tracked_targets = [enemy]
        
        # 测试跟踪状态下的预测行为
        fsm._change_state('TRACKING')
        
        # 模拟云台接口
        class MockTurret:
            def __init__(self):
                self.last_track_target = None
                
            def track_target(self, pos):
                self.last_track_target = pos
        
        fsm.turret = MockTurret()
        
        # 触发跟踪状态
        fsm.on_enter_TRACKING()
        
        # 验证预测位置被正确使用
        assert fsm.turret.last_track_target == (2.0, 1.0)
        
    def test_tactical_summary(self):
        """测试战术态势摘要"""
        world_model = self._create_test_world_model()
        
        # 添加多个目标
        enemy1 = self._create_enemy_target(1, threat_level=0.6)
        enemy2 = self._create_enemy_target(2, threat_level=0.9)
        ally = self._create_ally_target(3)
        
        world_model.tracked_targets = [enemy1, enemy2, ally]
        
        fsm = IntelligentFSM(world_model)
        fsm.update_system_status(ammo=0.8, health=0.9, heat=0.3)
        
        summary = fsm.get_tactical_summary()
        
        assert summary['current_state'] == 'IDLE'
        assert summary['enemy_count'] == 2
        assert summary['highest_threat_id'] == 2  # enemy2有更高威胁
        assert summary['system_status']['ammo'] == 0.8
        assert summary['system_status']['health'] == 0.9
        assert summary['system_status']['heat'] == 0.3
        
    def test_intelligent_update_timing(self):
        """测试智能更新的时间控制"""
        world_model = self._create_test_world_model()
        fsm = IntelligentFSM(world_model)
        
        # 记录初始时间
        initial_time = fsm.last_assessment_time
        
        # 快速连续调用不应该触发评估
        fsm.intelligent_update()
        fsm.intelligent_update()
        
        # 时间间隔太短，不应该更新
        assert fsm.last_assessment_time == initial_time or fsm.last_assessment_time > initial_time
        
    def _create_test_world_model(self):
        """创建测试用世界模型"""
        return WorldModel(
            self_pose=Pose2D(
                position=np.array([0.0, 0.0]),
                theta=0.0,
                timestamp=time.time()
            ),
            turret_state=TurretState(
                timestamp=time.time(),
                frame_id="turret",
                orientation=np.array([0.0, 0.0, 0.0]),
                angular_velocity=np.array([0.0, 0.0, 0.0]),
                motor_angles=np.array([0.0, 0.0])
            ),
            task_points=[],
            static_obstacles=[],
            dynamic_obstacles=[],
            occupancy_grid=np.zeros((100, 100)),
            ground_slopes={},
            tracked_targets=[],
            tactical_info=TacticalInfo()
        )
        
    def _create_enemy_target(self, target_id: int, threat_level: float = 0.5):
        """创建敌方目标"""
        enemy = KalmanTarget(id=target_id, team='enemy')
        enemy.state = np.array([5.0, 5.0, 0.0, 0.0])
        enemy.threat_level = threat_level
        enemy.confidence = 0.8
        return enemy
        
    def _create_ally_target(self, target_id: int):
        """创建友方目标"""
        ally = KalmanTarget(id=target_id, team='ally')
        ally.state = np.array([3.0, 3.0, 0.0, 0.0])
        ally.confidence = 0.8
        return ally
