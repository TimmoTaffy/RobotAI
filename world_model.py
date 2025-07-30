"""
世界模型定义，集成增强型跟踪器和战术评估系统
"""
from dataclasses import dataclass
from typing import List, Dict, Optional
import numpy as np
from src.common.types import Pose2D, TurretState
from src.tracking.enhanced_tracker import KalmanTarget


@dataclass
class TacticalInfo:
    """战术信息"""
    threat_level: float = 0.0
    safety_level: float = 1.0
    recommended_action: str = "PATROL"
    priority_targets: List[int] = None
    
    def __post_init__(self):
        if self.priority_targets is None:
            self.priority_targets = []


@dataclass 
class WorldModel:
    """世界模型 - 系统状态的中央枢纽"""
    # 自车定位位姿，含位置和朝向
    self_pose: Pose2D
    # 云台状态，含姿态和电机角度
    turret_state: TurretState
    # 任务点列表
    task_points: List
    # 静态障碍和动态障碍列表
    static_obstacles: List
    dynamic_obstacles: List
    # 最新生成的二维栅格地图
    occupancy_grid: np.ndarray
    # 地面坡度分析结果，每个距离区间对应的坡度角 (弧度)
    ground_slopes: Dict[str, float]
    # 增强型跟踪目标列表（使用卡尔曼滤波器）
    tracked_targets: List[KalmanTarget]
    # 战术信息评估
    tactical_info: TacticalInfo
    # 系统健康状态
    system_health: Dict[str, bool] = None
    # 增强型跟踪器引用（用于预测功能）
    enhanced_tracker: Optional[object] = None
    
    def __post_init__(self):
        if self.system_health is None:
            self.system_health = {
                'sensors': True,
                'navigation': True, 
                'tracking': True,
                'control': True,
                'weapon': True
            }
    
    def get_enemy_targets(self) -> List[KalmanTarget]:
        """获取所有敌方目标"""
        return [target for target in self.tracked_targets if target.team == 'enemy']
    
    def get_ally_targets(self) -> List[KalmanTarget]:
        """获取所有友方目标"""
        return [target for target in self.tracked_targets if target.team == 'ally']
    
    def get_highest_threat_target(self) -> Optional[KalmanTarget]:
        """获取威胁等级最高的目标"""
        enemy_targets = self.get_enemy_targets()
        if not enemy_targets:
            return None
        return max(enemy_targets, key=lambda t: t.threat_level)
    
    def update_tactical_assessment(self):
        """更新战术评估"""
        enemy_targets = self.get_enemy_targets()
        if not enemy_targets:
            self.tactical_info.threat_level = 0.0
            self.tactical_info.recommended_action = "PATROL"
            self.tactical_info.priority_targets = []
            return
        
        # 计算总体威胁等级
        max_threat = max(target.threat_level for target in enemy_targets)
        avg_threat = sum(target.threat_level for target in enemy_targets) / len(enemy_targets)
        self.tactical_info.threat_level = 0.7 * max_threat + 0.3 * avg_threat
        
        # 优先目标排序（按威胁等级降序）
        sorted_targets = sorted(enemy_targets, key=lambda t: t.threat_level, reverse=True)
        self.tactical_info.priority_targets = [t.id for t in sorted_targets[:3]]  # 前3个
        
        # 推荐行动
        if self.tactical_info.threat_level > 0.8:
            self.tactical_info.recommended_action = "ENGAGE"
        elif self.tactical_info.threat_level > 0.5:
            self.tactical_info.recommended_action = "TRACKING"
        else:
            self.tactical_info.recommended_action = "PATROL"