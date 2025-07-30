"""
战术路径规划器：在A*基础上增加战术考量，包括威胁评估、掩体利用、射击位置优化等
"""
import numpy as np
from typing import List, Tuple, Optional, Dict
from src.planning.path_planner import AStarPlanner
from src.tracking import KalmanTarget
from dataclasses import dataclass

@dataclass
class TacticalWaypoint:
    """战术路径点，包含位置和战术属性"""
    x: float
    y: float
    safety_level: float  # 安全等级 0-1
    tactical_value: float  # 战术价值 0-1
    threat_level: float  # 威胁等级 0-1

class TacticalPathPlanner:
    def __init__(self, grid: np.ndarray, grid_size: float, weapon_range: float = 5.0):
        """
        :param grid: 占据栅格地图
        :param grid_size: 栅格尺寸
        :param weapon_range: 武器射程（米）
        """
        self.base_planner = AStarPlanner(grid, grid_size)
        self.grid = grid
        self.grid_size = grid_size
        self.weapon_range = weapon_range
        self.height, self.width = grid.shape
        
        # 战术权重配置
        self.weights = {
            'distance': 1.0,      # 距离权重
            'safety': 2.0,        # 安全权重
            'tactical': 1.5,      # 战术价值权重
            'threat': 3.0         # 威胁惩罚权重
        }
        
        # 预计算掩体和高价值位置
        self._compute_tactical_map()

    def _compute_tactical_map(self):
        """预计算战术地图，标识掩体、制高点等"""
        self.cover_map = np.zeros((self.height, self.width))
        self.vantage_map = np.zeros((self.height, self.width))
        
        for i in range(self.height):
            for j in range(self.width):
                if self.grid[i, j] == 0:  # 自由格子
                    # 计算掩体价值：周围障碍物密度
                    cover_score = self._calculate_cover_score(i, j)
                    self.cover_map[i, j] = cover_score
                    
                    # 计算制高点价值：视野开阔度
                    vantage_score = self._calculate_vantage_score(i, j)
                    self.vantage_map[i, j] = vantage_score

    def _calculate_cover_score(self, row: int, col: int, radius: int = 2) -> float:
        """计算位置的掩体价值"""
        obstacle_count = 0
        total_count = 0
        
        for di in range(-radius, radius + 1):
            for dj in range(-radius, radius + 1):
                ni, nj = row + di, col + dj
                if 0 <= ni < self.height and 0 <= nj < self.width:
                    total_count += 1
                    if self.grid[ni, nj] == 1:
                        obstacle_count += 1
        
        return obstacle_count / total_count if total_count > 0 else 0

    def _calculate_vantage_score(self, row: int, col: int, radius: int = 5) -> float:
        """计算位置的制高点价值（视野开阔度）"""
        visible_count = 0
        total_count = 0
        
        for di in range(-radius, radius + 1):
            for dj in range(-radius, radius + 1):
                if di == 0 and dj == 0:
                    continue
                ni, nj = row + di, col + dj
                if 0 <= ni < self.height and 0 <= nj < self.width:
                    total_count += 1
                    # 使用简单的视线检查
                    if self._has_line_of_sight((col, row), (nj, ni)):
                        visible_count += 1
        
        return visible_count / total_count if total_count > 0 else 0

    def _has_line_of_sight(self, start: Tuple[int, int], end: Tuple[int, int]) -> bool:
        """检查两点间是否有视线"""
        return self.base_planner.line_of_sight(start, end)

    def _evaluate_threat_level(self, pos: Tuple[float, float], targets: List[KalmanTarget]) -> float:
        """评估位置的威胁等级"""
        if not targets:
            return 0.0
        
        threat = 0.0
        for target in targets:
            if target.team == 'enemy':  # 只考虑敌方目标
                distance = np.hypot(pos[0] - target.state[0], pos[1] - target.state[1])
                if distance < self.weapon_range:
                    # 距离越近威胁越大
                    threat += (self.weapon_range - distance) / self.weapon_range * target.confidence
        
        return min(threat, 1.0)  # 限制在0-1范围内

    def _evaluate_tactical_value(self, pos: Tuple[float, float], 
                                targets: List[KalmanTarget], 
                                objective: Optional[Tuple[float, float]] = None) -> float:
        """评估位置的战术价值"""
        gx, gy = int(pos[0] / self.grid_size), int(pos[1] / self.grid_size)
        if not (0 <= gx < self.width and 0 <= gy < self.height):
            return 0.0
        
        tactical_value = 0.0
        
        # 1. 基础制高点价值
        tactical_value += self.vantage_map[gy, gx] * 0.4
        
        # 2. 对敌方目标的射击优势
        for target in targets:
            if target.team == 'enemy':
                distance = np.hypot(pos[0] - target.state[0], pos[1] - target.state[1])
                if distance <= self.weapon_range:
                    # 在射程内且有视线
                    target_grid = (int(target.state[0] / self.grid_size), int(target.state[1] / self.grid_size))
                    if self._has_line_of_sight((gx, gy), target_grid):
                        # 最佳射击距离权重
                        optimal_distance = self.weapon_range * 0.6
                        distance_factor = 1.0 - abs(distance - optimal_distance) / self.weapon_range
                        tactical_value += distance_factor * target.confidence * 0.6
        
        return min(tactical_value, 1.0)

    def _calculate_safety_level(self, pos: Tuple[float, float], targets: List[KalmanTarget]) -> float:
        """计算位置的安全等级"""
        gx, gy = int(pos[0] / self.grid_size), int(pos[1] / self.grid_size)
        if not (0 <= gx < self.width and 0 <= gy < self.height):
            return 0.0
        
        # 基础掩体安全性
        safety = self.cover_map[gy, gx] * 0.7
        
        # 距离敌方目标的安全距离
        min_safe_distance = self.weapon_range * 0.3  # 最小安全距离
        for target in targets:
            if target.team == 'enemy':
                distance = np.hypot(pos[0] - target.state[0], pos[1] - target.state[1])
                if distance < min_safe_distance:
                    safety *= (distance / min_safe_distance)  # 距离过近降低安全性
        
        return min(safety + 0.3, 1.0)  # 基础安全性 + 掩体安全性

    def tactical_cost(self, pos: Tuple[int, int], goal: Tuple[int, int], 
                     targets: List[KalmanTarget]) -> float:
        """计算考虑战术因素的代价函数"""
        # 基础距离代价
        world_pos = (pos[0] * self.grid_size, pos[1] * self.grid_size)
        distance_cost = np.hypot(pos[0] - goal[0], pos[1] - goal[1])
        
        # 战术评估
        threat = self._evaluate_threat_level(world_pos, targets)
        safety = self._calculate_safety_level(world_pos, targets)
        tactical_value = self._evaluate_tactical_value(world_pos, targets)
        
        # 综合代价计算
        total_cost = (self.weights['distance'] * distance_cost +
                     self.weights['threat'] * threat +
                     self.weights['safety'] * (1.0 - safety) -
                     self.weights['tactical'] * tactical_value)
        
        return max(total_cost, 0.1)  # 确保代价为正

    def plan_tactical_path(self, start: Tuple[float, float], 
                          goal: Tuple[float, float],
                          targets: List[KalmanTarget],
                          prefer_cover: bool = True) -> List[TacticalWaypoint]:
        """
        规划战术路径
        :param start: 起点
        :param goal: 目标点
        :param targets: 当前跟踪的目标列表
        :param prefer_cover: 是否优先选择掩体路径
        :return: 战术路径点列表
        """
        # 1. 首先使用基础A*获得几何最优路径
        base_path = self.base_planner.plan(start, goal)
        if not base_path:
            return []
        
        # 2. 对路径点进行战术分析和优化
        tactical_path = []
        for point in base_path:
            threat = self._evaluate_threat_level(point, targets)
            safety = self._calculate_safety_level(point, targets)
            tactical_value = self._evaluate_tactical_value(point, targets)
            
            waypoint = TacticalWaypoint(
                x=point[0],
                y=point[1],
                safety_level=safety,
                tactical_value=tactical_value,
                threat_level=threat
            )
            tactical_path.append(waypoint)
        
        # 3. 如果路径危险度过高，寻找替代路径
        if self._path_too_dangerous(tactical_path):
            return self._find_safer_path(start, goal, targets)
        
        return tactical_path

    def _path_too_dangerous(self, path: List[TacticalWaypoint], threshold: float = 0.7) -> bool:
        """检查路径是否过于危险"""
        avg_threat = sum(wp.threat_level for wp in path) / len(path)
        return avg_threat > threshold

    def _find_safer_path(self, start: Tuple[float, float], 
                        goal: Tuple[float, float],
                        targets: List[KalmanTarget]) -> List[TacticalWaypoint]:
        """寻找更安全的替代路径（可以实现中间掩体点）"""
        # 简化实现：在起点和终点间寻找安全的中继点
        mid_x = (start[0] + goal[0]) / 2
        mid_y = (start[1] + goal[1]) / 2
        
        # 在中点附近寻找最安全的位置
        best_mid_point = self._find_safest_nearby_point((mid_x, mid_y), targets, radius=3.0)
        
        if best_mid_point:
            # 分段规划路径
            path1 = self.base_planner.plan(start, best_mid_point)
            path2 = self.base_planner.plan(best_mid_point, goal)
            
            if path1 and path2:
                combined_path = path1 + path2[1:]  # 避免重复中间点
                return [TacticalWaypoint(
                    x=p[0], y=p[1],
                    safety_level=self._calculate_safety_level(p, targets),
                    tactical_value=self._evaluate_tactical_value(p, targets),
                    threat_level=self._evaluate_threat_level(p, targets)
                ) for p in combined_path]
        
        # 如果找不到更好的路径，返回原始路径
        return self.plan_tactical_path(start, goal, targets, prefer_cover=False)

    def _find_safest_nearby_point(self, center: Tuple[float, float], 
                                 targets: List[KalmanTarget], 
                                 radius: float = 2.0) -> Optional[Tuple[float, float]]:
        """在指定半径内寻找最安全的点"""
        best_point = None
        best_safety = 0.0
        
        # 在半径内采样多个点
        for angle in np.linspace(0, 2*np.pi, 8):
            for r in np.linspace(0.5, radius, 3):
                candidate_x = center[0] + r * np.cos(angle)
                candidate_y = center[1] + r * np.sin(angle)
                candidate = (candidate_x, candidate_y)
                
                # 检查点是否可达（不在障碍物上）
                gx, gy = int(candidate_x / self.grid_size), int(candidate_y / self.grid_size)
                if (0 <= gx < self.width and 0 <= gy < self.height and 
                    self.grid[gy, gx] == 0):
                    
                    safety = self._calculate_safety_level(candidate, targets)
                    if safety > best_safety:
                        best_safety = safety
                        best_point = candidate
        
        return best_point

    def get_tactical_advice(self, current_pos: Tuple[float, float], 
                           targets: List[KalmanTarget]) -> Dict[str, float]:
        """获取当前位置的战术建议"""
        return {
            'threat_level': self._evaluate_threat_level(current_pos, targets),
            'safety_level': self._calculate_safety_level(current_pos, targets),
            'tactical_value': self._evaluate_tactical_value(current_pos, targets),
            'should_relocate': self._evaluate_threat_level(current_pos, targets) > 0.6
        }
