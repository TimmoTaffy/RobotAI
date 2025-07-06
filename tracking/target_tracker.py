import time
from typing import Dict, List
from dataclasses import dataclass
from common.types import RobotInfo, VisionRobot

@dataclass
class TrackedTarget:
    id: int
    team: str
    x: float
    y: float
    confidence: float
    last_update: float

class TargetTracker:
    def __init__(self, max_age: float = 0.5):
        """构造跟踪器
        :param max_age: 目标在无更新后被剔除的最大时长（秒）"""
        self.max_age = max_age
        self.tracks: Dict[int, TrackedTarget] = {}

    def update(self, radar_data, vision_data) -> List[TrackedTarget]:
        """
        使用雷达和视觉数据更新跟踪目标列表。
        :param radar_data: RadarStationData
        :param vision_data: VisionData
        :return: 当前有效的目标列表
        """
        now = time.time()
        # 1. 更新雷达检测（置信度为 1.0）
        for r in radar_data.robots:
            self.tracks[r.id] = TrackedTarget(r.id, r.team, r.x, r.y, confidence=1.0, last_update=now)
        # 2. 更新视觉检测（按置信度融合）
        for v in vision_data.robots:
            conf = getattr(v, 'confidence', 1.0)
            if v.id in self.tracks:
                t = self.tracks[v.id]
                # 简单位置融合：平均
                t.x = (t.x + v.x) / 2
                t.y = (t.y + v.y) / 2
                t.confidence = max(t.confidence, conf)
                t.last_update = now
            else:
                self.tracks[v.id] = TrackedTarget(v.id, v.team, v.x, v.y, confidence=conf, last_update=now)
        # 3. 剔除超时目标
        stale_ids = [tid for tid, tgt in self.tracks.items() if now - tgt.last_update > self.max_age]
        for tid in stale_ids:
            del self.tracks[tid]
        return list(self.tracks.values())
