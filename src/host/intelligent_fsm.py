"""
智能化高级状态机 - 集成世界模型的战术评估
"""
from typing import Optional
from world_model import WorldModel
from src.host.high_level_fsm import HighLevelStateMachine
import time


class IntelligentFSM(HighLevelStateMachine):
    """
    智能化状态机：基于世界模型战术评估的自主决策
    
    核心改进：
    1. 集成世界模型，实时感知战术态势
    2. 基于威胁等级和推荐行动自动状态转换
    3. 多因素决策：威胁、弹药、血量、目标数量
    4. 预测性行为：提前规避高危险区域
    """
    
    def __init__(self, world_model: WorldModel, navigator=None, vision=None, 
                 turret=None, weapon=None, patrol_timer=None):
        super().__init__(navigator, vision, turret, weapon, patrol_timer)
        
        self.world_model = world_model
        self.last_assessment_time = 0.0
        self.assessment_interval = 0.1  # 100ms评估一次
        
        # 智能决策参数
        self.threat_thresholds = {
            'low': 0.3,      # 低威胁阈值
            'medium': 0.6,   # 中威胁阈值  
            'high': 0.8      # 高威胁阈值
        }
        
        # 系统状态监控
        self.ammo_level = 1.0      # 弹药水平 [0-1]
        self.health_level = 1.0    # 血量水平 [0-1]
        self.heat_level = 0.0      # 热量水平 [0-1]
        
    def intelligent_update(self):
        """
        智能状态评估和自动转换
        基于世界模型的战术信息自主决策状态转换
        """
        current_time = time.time()
        if current_time - self.last_assessment_time < self.assessment_interval:
            return
            
        self.last_assessment_time = current_time
        
        # 更新世界模型战术评估
        self.world_model.update_tactical_assessment()
        
        # 获取战术信息
        tactical_info = self.world_model.tactical_info
        threat_level = tactical_info.threat_level
        recommended_action = tactical_info.recommended_action
        highest_threat = self.world_model.get_highest_threat_target()
        
        # 基于多因素进行智能决策
        new_state = self._intelligent_decision(
            threat_level, recommended_action, highest_threat
        )
        
        if new_state and new_state != self.state:
            self._change_state(new_state)
            
    def _intelligent_decision(self, threat_level: float, 
                            recommended_action: str, 
                            highest_threat) -> Optional[str]:
        """
        智能决策核心逻辑
        综合考虑威胁、系统状态、战术建议
        """
        # 1. 系统故障或极低资源 → 强制退守
        if (self.health_level < 0.2 or 
            self.ammo_level < 0.1 or 
            self.heat_level > 0.9):
            return 'RETREAT'
            
        # 2. 无威胁或系统空闲 → 巡逻
        if threat_level < self.threat_thresholds['low']:
            if self.state in ['TRACKING', 'AIMING', 'FIRING']:
                return 'PATROL'
            elif self.state == 'IDLE':
                return 'PATROL'
                
        # 3. 中等威胁 → 跟踪模式
        elif threat_level < self.threat_thresholds['medium']:
            if recommended_action == "TRACKING":
                if self.state == 'PATROL':
                    return 'TRACKING'
                elif self.state in ['AIMING', 'FIRING'] and not self._is_target_stable():
                    return 'TRACKING'
                    
        # 4. 高威胁 → 攻击模式
        elif threat_level >= self.threat_thresholds['high']:
            if recommended_action == "ENGAGE":
                if self.state == 'TRACKING' and self._is_target_stable():
                    return 'AIMING'
                elif self.state == 'AIMING' and self._is_ready_to_fire():
                    return 'FIRING'
                elif self.state == 'PATROL':
                    return 'TRACKING'  # 直接进入跟踪
                    
        # 5. 特殊情况处理
        if self.state == 'FIRING':
            # 射击中如果目标丢失或弹药耗尽
            if not highest_threat or self.ammo_level < 0.05:
                return 'RETREAT'
            elif not self._is_target_stable():
                return 'TRACKING'
                
        return None  # 保持当前状态
        
    def _is_target_stable(self) -> bool:
        """
        判断目标是否稳定（可以瞄准）
        基于目标运动状态和跟踪置信度
        """
        highest_threat = self.world_model.get_highest_threat_target()
        if not highest_threat:
            return False
            
        # 检查目标置信度和运动状态
        confidence_stable = highest_threat.confidence > 0.8
        velocity = (highest_threat.state[2]**2 + highest_threat.state[3]**2)**0.5
        motion_stable = velocity < 2.0  # 速度小于2m/s认为相对稳定
        
        return confidence_stable and motion_stable
        
    def _is_ready_to_fire(self) -> bool:
        """
        判断是否准备好开火
        综合考虑瞄准精度、系统状态等
        """
        # 检查弹药和热量
        if self.ammo_level < 0.1 or self.heat_level > 0.8:
            return False
            
        # 检查目标稳定性
        if not self._is_target_stable():
            return False
            
        # 检查瞄准精度（这里简化处理）
        # 实际应该检查云台是否已经瞄准到位
        return True
        
    def update_system_status(self, ammo: float, health: float, heat: float):
        """
        更新系统状态信息
        """
        self.ammo_level = max(0.0, min(1.0, ammo))
        self.health_level = max(0.0, min(1.0, health))  
        self.heat_level = max(0.0, min(1.0, heat))
        
    def get_tactical_summary(self) -> dict:
        """
        获取战术态势摘要，用于调试和监控
        """
        tactical_info = self.world_model.tactical_info
        highest_threat = self.world_model.get_highest_threat_target()
        
        return {
            'current_state': self.state,
            'threat_level': tactical_info.threat_level,
            'recommended_action': tactical_info.recommended_action,
            'enemy_count': len(self.world_model.get_enemy_targets()),
            'highest_threat_id': highest_threat.id if highest_threat else None,
            'system_status': {
                'ammo': self.ammo_level,
                'health': self.health_level,
                'heat': self.heat_level
            },
            'decision_factors': {
                'target_stable': self._is_target_stable(),
                'ready_to_fire': self._is_ready_to_fire()
            }
        }
        
    # 重写状态回调，增加智能行为
    def on_enter_TRACKING(self):
        """智能跟踪模式：选择最高威胁目标"""
        super().on_enter_TRACKING()
        
        # 自动选择最高威胁目标
        highest_threat = self.world_model.get_highest_threat_target()
        if highest_threat and self.turret:
            # 使用跟踪器的专业预测功能
            import time
            predicted_pos = self.world_model.enhanced_tracker.predict_target_position(
                highest_threat.id, time.time() + 0.5
            )
            if predicted_pos:
                self.turret.track_target(predicted_pos)
                
    def on_enter_AIMING(self):
        """智能瞄准模式：预测性瞄准"""
        super().on_enter_AIMING()
        
        highest_threat = self.world_model.get_highest_threat_target()
        if highest_threat and self.turret:
            # 使用跟踪器预测0.2秒后的目标位置（考虑弹道飞行时间）
            import time
            predicted_pos = self.world_model.enhanced_tracker.predict_target_position(
                highest_threat.id, time.time() + 0.2
            )
            if predicted_pos:
                self.turret.aim_at(predicted_pos)
