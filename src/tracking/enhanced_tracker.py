"""
增强型目标跟踪器：基于卡尔曼滤波的多目标跟踪系统

## 核心功能
1. **多目标跟踪**: 使用卡尔曼滤波同时跟踪多个运动目标
2. **数据关联**: 马哈拉诺比斯距离匹配新检测与现有轨迹
3. **运动预测**: 基于状态估计预测目标未来位置
4. **威胁评估**: 综合距离、速度、碰撞风险计算威胁等级
5. **多传感器融合**: 整合雷达和视觉检测数据

## 智能预测原理
- **状态模型**: [x, y, vx, vy] 位置+速度 4维状态向量
- **运动模型**: 匀速直线运动 (Constant Velocity Model)
- **预测方程**: 位置(t+dt) = 位置(t) + 速度(t) × dt
- **不确定性**: 协方差矩阵跟踪预测精度

## 威胁评估算法
基于多因素加权计算：
- 队伍关系: enemy(0.4) > unknown(0.2) > ally(0.0) 
- 距离威胁: <3m(0.4) <6m(0.2) <10m(0.1)
- 速度威胁: >3m/s(0.3) >1.5m/s(0.1)
- 碰撞预测: CPA算法预测5秒内碰撞风险(0.3)
- 跟踪稳定性: 稳定目标可信度加成(0.1)

## 使用场景
- 自动瞄准: 预测射击时目标位置
- 路径规划: 避开高威胁目标
- 态势感知: 实时威胁等级评估
"""
import numpy as np
import time
from typing import Dict, List, Optional, Tuple
from dataclasses import dataclass, field
from scipy.linalg import block_diag
from src.common.types import RobotInfo, VisionRobot


@dataclass
class KalmanTarget:
    """
    基于卡尔曼滤波的目标状态表示
    
    核心思想：每个目标不仅有位置，还有速度估计和不确定性描述
    这使得我们能够：
    1. 预测目标未来运动轨迹
    2. 评估预测的可信度  
    3. 进行智能的数据关联
    """
    id: int                    # 目标唯一标识符
    team: str                  # 队伍：'ally', 'enemy', 'unknown'
    
    # === 卡尔曼滤波核心状态 ===
    # 状态向量：[x, y, vx, vy] (位置和速度)
    # 这是预测的关键：知道位置和速度就能预测未来位置
    state: np.ndarray = field(default_factory=lambda: np.zeros(4))
    
    # 状态协方差矩阵 4x4
    # 描述状态估计的不确定性，用于数据关联和预测置信度
    covariance: np.ndarray = field(default_factory=lambda: np.eye(4) * 10.0)
    
    # === 跟踪质量指标 ===
    confidence: float = 1.0    # 跟踪置信度 [0-1]
    last_update: float = 0.0   # 最后更新时间戳
    detection_count: int = 0   # 成功检测次数
    miss_count: int = 0        # 连续丢失次数
    
    # === 威胁评估结果 ===
    threat_level: float = 0.0          # 当前威胁等级 [0-1]
    threat_history: List[float] = field(default_factory=list)  # 威胁历史


class EnhancedTargetTracker:
    """
    增强型目标跟踪器：多目标跟踪的智能引擎
    
    ## 工作原理
    1. **预测步**: 基于运动模型预测所有目标的新位置
    2. **关联步**: 将新检测与预测位置匹配（马哈拉诺比斯距离）
    3. **更新步**: 用匹配的检测更新目标状态（卡尔曼滤波）
    4. **管理步**: 创建新轨迹、删除过期轨迹
    
    ## 智能特性
    - **运动预测**: 不仅知道目标现在在哪，还知道下一秒会在哪
    - **不确定性量化**: 每个预测都有置信度，用于决策
    - **多传感器融合**: 自动整合雷达、视觉等多源数据
    - **威胁评估**: 实时计算每个目标的危险程度
    
    ## 应用场景
    - 自动瞄准系统：预测0.2秒后目标位置进行提前瞄准
    - 避障导航：预测其他机器人轨迹避免碰撞
    - 战术决策：识别最高威胁目标优先处理
    """
    
    def __init__(self, 
                 dt: float = 0.1,
                 max_age: float = 1.0,
                 association_threshold: float = 3.0,
                 process_noise: float = 0.5,
                 measurement_noise: float = 1.0):
        """
        初始化增强型跟踪器
        
        Args:
            dt: 时间步长
            max_age: 目标最大存活时间
            association_threshold: 数据关联阈值(马哈拉诺比斯距离)
            process_noise: 过程噪声标准差
            measurement_noise: 测量噪声标准差
        """
        self.dt = dt
        self.max_age = max_age
        self.association_threshold = association_threshold
        
        # 卡尔曼滤波器参数
        self._setup_kalman_matrices(process_noise, measurement_noise)
        
        # 跟踪状态
        self.tracks: Dict[int, KalmanTarget] = {}
        self.next_id = 1000  # 从1000开始分配新ID
        
        # 性能统计
        self.stats = {
            'total_detections': 0,
            'successful_associations': 0,
            'new_tracks': 0,
            'lost_tracks': 0
        }
    
    def _setup_kalman_matrices(self, process_noise: float, measurement_noise: float):
        """设置卡尔曼滤波器矩阵"""
        # 状态转移矩阵 (匀速运动模型)
        self.F = np.array([
            [1, 0, self.dt, 0],
            [0, 1, 0, self.dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
        
        # 观测矩阵 (只能观测位置)
        self.H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ])
        
        # 过程噪声协方差矩阵
        q = process_noise ** 2
        self.Q = np.array([
            [self.dt**4/4, 0, self.dt**3/2, 0],
            [0, self.dt**4/4, 0, self.dt**3/2],
            [self.dt**3/2, 0, self.dt**2, 0],
            [0, self.dt**3/2, 0, self.dt**2]
        ]) * q
        
        # 测量噪声协方差矩阵
        r = measurement_noise ** 2
        self.R = np.eye(2) * r
    
    def predict(self, track: KalmanTarget) -> KalmanTarget:
        """
        卡尔曼滤波预测步：预测目标下一时刻的状态
        
        核心思想：根据当前状态和运动模型，推算目标在dt时间后的位置和速度
        
        运动模型（匀速直线运动）：
        - 新位置 = 当前位置 + 当前速度 × 时间步长
        - 新速度 = 当前速度（假设速度不变）
        
        同时更新不确定性：
        - 时间越长，预测不确定性越大
        - 过程噪声反映运动模型的局限性
        """
        # 状态预测：x(k|k-1) = F * x(k-1|k-1)
        track.state = self.F @ track.state
        
        # 协方差预测：P(k|k-1) = F * P(k-1|k-1) * F^T + Q
        track.covariance = self.F @ track.covariance @ self.F.T + self.Q
        
        return track
    
    def update(self, track: KalmanTarget, measurement: np.ndarray) -> KalmanTarget:
        """卡尔曼滤波更新步"""
        # 计算卡尔曼增益
        S = self.H @ track.covariance @ self.H.T + self.R
        K = track.covariance @ self.H.T @ np.linalg.inv(S)
        
        # 状态更新
        y = measurement - self.H @ track.state  # 测量余差
        track.state = track.state + K @ y
        
        # 协方差更新
        I = np.eye(len(track.state))
        track.covariance = (I - K @ self.H) @ track.covariance
        
        return track
    
    def _mahalanobis_distance(self, track: KalmanTarget, 
                             measurement: np.ndarray) -> float:
        """
        计算马哈拉诺比斯距离：智能数据关联的核心
        
        为什么不用欧几里得距离？
        - 欧几里得距离只考虑位置差异
        - 马哈拉诺比斯距离考虑预测的不确定性
        
        举例：
        - 如果目标A预测位置不确定性大，即使检测点离得远也可能是同一目标
        - 如果目标B预测位置很精确，检测点稍远就不太可能是同一目标
        
        公式：distance = sqrt((z-Hx)^T * S^-1 * (z-Hx))
        其中 S = H*P*H^T + R 是创新协方差
        """
        predicted_measurement = self.H @ track.state  # 预测的观测值
        residual = measurement - predicted_measurement # 观测残差
        
        # 创新协方差：预测不确定性 + 测量不确定性
        S = self.H @ track.covariance @ self.H.T + self.R
        try:
            distance = np.sqrt(residual.T @ np.linalg.inv(S) @ residual)
            return float(distance)
        except np.linalg.LinAlgError:
            # 如果矩阵奇异，退化为欧几里得距离
            return np.linalg.norm(residual)
    
    def _associate_detections(self, detections: List[Tuple[float, float, str, float]]) -> Dict:
        """
        数据关联：匈牙利算法或贪心匹配
        
        Args:
            detections: [(x, y, team, confidence), ...]
            
        Returns:
            关联结果字典
        """
        associations = {}
        unmatched_detections = []
        unmatched_tracks = list(self.tracks.keys())
        
        # 构建距离矩阵
        if detections and self.tracks:
            distances = {}
            
            for det_idx, (x, y, team, conf) in enumerate(detections):
                measurement = np.array([x, y])
                
                for track_id in self.tracks:
                    track = self.tracks[track_id]
                    
                    # 同队伍才能关联
                    if track.team == team:
                        dist = self._mahalanobis_distance(track, measurement)
                        
                        if dist < self.association_threshold:
                            distances[(det_idx, track_id)] = dist
            
            # 贪心匹配（简化版匈牙利算法）
            used_detections = set()
            used_tracks = set()
            
            # 按距离排序
            sorted_pairs = sorted(distances.items(), key=lambda x: x[1])
            
            for (det_idx, track_id), dist in sorted_pairs:
                if det_idx not in used_detections and track_id not in used_tracks:
                    associations[track_id] = det_idx
                    used_detections.add(det_idx)
                    used_tracks.add(track_id)
            
            # 找出未匹配的检测和轨迹
            unmatched_detections = [i for i in range(len(detections)) 
                                  if i not in used_detections]
            unmatched_tracks = [tid for tid in self.tracks.keys() 
                              if tid not in used_tracks]
        else:
            unmatched_detections = list(range(len(detections)))
            
        return {
            'matched': associations,
            'unmatched_detections': unmatched_detections,
            'unmatched_tracks': unmatched_tracks
        }
    
    def _assess_threat_level(self, track: KalmanTarget, 
                           my_position: Optional[Tuple[float, float]] = None,
                           my_velocity: Optional[Tuple[float, float]] = None) -> float:
        """
        多因素威胁评估算法：量化目标的危险程度
        
        ## 评估维度
        1. **队伍关系** (40%基础威胁)
           - enemy: 0.4 (敌方必然有威胁)
           - unknown: 0.2 (未知目标需警惕)
           - ally: 0.0 (友方无威胁)
        
        2. **距离威胁** (最高40%加成)
           - <3m: +0.4 (近距离极危险)
           - <6m: +0.2 (中距离有威胁)
           - <10m: +0.1 (远距离小威胁)
        
        3. **速度威胁** (最高30%加成)
           - >3m/s: +0.3 (高速移动威胁大)
           - >1.5m/s: +0.1 (中速移动有威胁)
        
        4. **碰撞预测** (最高30%加成)
           - 使用CPA算法预测5秒内碰撞
           - 预测碰撞距离<2m时视为高威胁
        
        5. **跟踪稳定性** (10%可信度加成)
           - 稳定跟踪的目标威胁评估更可信
        
        ## 输出范围
        [0.0, 1.0] 威胁等级，1.0为最高威胁
        """
        threat = 0.0
        
        # 1. 基础威胁（队伍）
        if track.team == 'enemy':
            threat += 0.4
        elif track.team == 'unknown':
            threat += 0.2
        
        # 2. 距离威胁
        if my_position:
            distance = np.linalg.norm(track.state[:2] - np.array(my_position))
            if distance < 3.0:
                threat += 0.4    # 近距离极危险
            elif distance < 6.0:
                threat += 0.2    # 中距离有威胁
            elif distance < 10.0:
                threat += 0.1    # 远距离小威胁
        
        # 3. 运动威胁
        speed = np.linalg.norm(track.state[2:4])
        if speed > 3.0:
            threat += 0.3        # 高速移动
        elif speed > 1.5:
            threat += 0.1        # 中速移动
        
        # 4. 预测碰撞威胁 (CPA: Closest Point of Approach)
        if my_position and my_velocity:
            rel_pos = track.state[:2] - np.array(my_position)    # 相对位置
            rel_vel = track.state[2:4] - np.array(my_velocity)   # 相对速度
            
            if np.linalg.norm(rel_vel) > 0.1:  # 避免除零
                # 计算最近接近时间
                t_cpa = -np.dot(rel_pos, rel_vel) / np.dot(rel_vel, rel_vel)
                if 0 < t_cpa < 5.0:  # 5秒内可能碰撞
                    # 计算最近接近距离
                    cpa_distance = np.linalg.norm(rel_pos + rel_vel * t_cpa)
                    if cpa_distance < 2.0:
                        threat += 0.3    # 预测碰撞风险
        
        # 5. 跟踪稳定性加成
        if track.detection_count > 5:
            threat += 0.1        # 稳定跟踪的目标更可信
        
        return min(threat, 1.0)  # 威胁等级上限为1.0
    
    def process_detections(self, 
                          radar_data=None, 
                          vision_data=None,
                          my_position: Optional[Tuple[float, float]] = None,
                          my_velocity: Optional[Tuple[float, float]] = None) -> List[KalmanTarget]:
        """
        处理新的检测数据
        
        Args:
            radar_data: 雷达数据
            vision_data: 视觉数据  
            my_position: 己方位置
            my_velocity: 己方速度
            
        Returns:
            当前有效目标列表
        """
        current_time = time.time()
        
        # 1. 预测所有现有轨迹
        for track in self.tracks.values():
            self.predict(track)
        
        # 2. 合并检测数据
        detections = []
        
        if radar_data:
            for robot in radar_data.robots:
                detections.append((robot.x, robot.y, robot.team, 1.0))
        
        if vision_data:
            for robot in vision_data.robots:
                conf = getattr(robot, 'confidence', 0.8)
                if conf > 0.5:  # 只处理高置信度的视觉检测
                    detections.append((robot.x, robot.y, robot.team, conf))
        
        self.stats['total_detections'] += len(detections)
        
        # 3. 数据关联
        associations = self._associate_detections(detections)
        
        # 4. 更新匹配的轨迹
        for track_id, det_idx in associations['matched'].items():
            x, y, team, conf = detections[det_idx]
            measurement = np.array([x, y])
            
            track = self.tracks[track_id]
            self.update(track, measurement)
            
            track.confidence = min(track.confidence + 0.1, 1.0)
            track.last_update = current_time
            track.detection_count += 1
            track.miss_count = 0
            
            # 更新威胁评估
            track.threat_level = self._assess_threat_level(track, my_position, my_velocity)
            track.threat_history.append(track.threat_level)
            if len(track.threat_history) > 20:  # 保持历史长度
                track.threat_history.pop(0)
            
            self.stats['successful_associations'] += 1
        
        # 5. 处理未匹配的检测（创建新轨迹）
        for det_idx in associations['unmatched_detections']:
            x, y, team, conf = detections[det_idx]
            
            # 只为高置信度检测创建新轨迹
            if conf > 0.7:
                new_track = KalmanTarget(
                    id=self.next_id,
                    team=team,
                    confidence=conf,
                    last_update=current_time,
                    detection_count=1
                )
                
                # 初始化状态 [x, y, 0, 0] (位置已知，速度未知)
                new_track.state = np.array([x, y, 0.0, 0.0])
                
                # 初始协方差 (位置不确定性小，速度不确定性大)
                new_track.covariance = np.diag([1.0, 1.0, 10.0, 10.0])
                
                new_track.threat_level = self._assess_threat_level(new_track, my_position, my_velocity)
                
                self.tracks[self.next_id] = new_track
                self.next_id += 1
                self.stats['new_tracks'] += 1
        
        # 6. 处理未匹配的轨迹（增加丢失计数）
        for track_id in associations['unmatched_tracks']:
            track = self.tracks[track_id]
            track.miss_count += 1
            track.confidence = max(track.confidence - 0.15, 0.0)
        
        # 7. 删除过期轨迹
        stale_ids = []
        for track_id, track in self.tracks.items():
            age = current_time - track.last_update
            if age > self.max_age or track.miss_count > 5 or track.confidence < 0.1:
                stale_ids.append(track_id)
        
        for track_id in stale_ids:
            del self.tracks[track_id]
            self.stats['lost_tracks'] += 1
        
        return list(self.tracks.values())
    
    def get_high_priority_targets(self, 
                                 threat_threshold: float = 0.6,
                                 max_targets: int = 3) -> List[KalmanTarget]:
        """获取高优先级目标"""
        high_threat = [t for t in self.tracks.values() 
                      if t.threat_level > threat_threshold]
        
        # 按威胁等级和置信度排序
        high_threat.sort(key=lambda t: t.threat_level * t.confidence, reverse=True)
        
        return high_threat[:max_targets]
    
    def predict_target_position(self, target_id: int, 
                               prediction_time: float) -> Optional[Tuple[float, float]]:
        """
        智能目标位置预测：自动瞄准和避障的核心功能
        
        ## 预测原理
        基于卡尔曼滤波状态 [x, y, vx, vy] 进行线性外推：
        - 未来位置 = 当前位置 + 速度 × 时间差
        - 考虑目标的运动历史和速度估计
        
        ## 应用场景
        1. **自动瞄准**: 预测0.1-0.3秒后位置，补偿弹道飞行时间
        2. **避障规划**: 预测1-3秒后位置，规划安全路径
        3. **拦截轨迹**: 预测长期运动趋势，计算拦截点
        
        ## 预测精度
        - 短期预测(0.1-0.5s): 高精度，适合瞄准
        - 中期预测(0.5-2s): 中等精度，适合路径规划
        - 长期预测(>2s): 低精度，仅供参考
        
        Args:
            target_id: 目标ID
            prediction_time: 预测时间点（时间戳）
            
        Returns:
            预测位置 (x, y) 或 None（目标不存在）
        """
        if target_id not in self.tracks:
            return None
        
        track = self.tracks[target_id]
        dt = prediction_time - track.last_update
        
        if dt <= 0:
            # 请求过去时间，返回当前位置
            return (float(track.state[0]), float(track.state[1]))
        
        # 使用卡尔曼状态进行线性预测
        future_state = track.state.copy()
        
        # 构建预测转移矩阵
        F_pred = np.array([
            [1, 0, dt, 0],   # x_new = x + vx * dt
            [0, 1, 0, dt],   # y_new = y + vy * dt  
            [0, 0, 1, 0],    # vx_new = vx (匀速假设)
            [0, 0, 0, 1]     # vy_new = vy (匀速假设)
        ])
        
        future_state = F_pred @ future_state
        return (float(future_state[0]), float(future_state[1]))
    
    def get_statistics(self) -> Dict:
        """获取跟踪器性能统计"""
        return {
            **self.stats,
            'active_tracks': len(self.tracks),
            'association_rate': (self.stats['successful_associations'] / 
                               max(self.stats['total_detections'], 1))
        }
    
    def reset(self):
        """重置跟踪器"""
        self.tracks.clear()
        self.next_id = 1000
        self.stats = {k: 0 for k in self.stats}
