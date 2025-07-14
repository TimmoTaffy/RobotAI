import time
from src.sensors.serial_receiver import SerialReceiver
from src.common.types import VisionData, VisionRobot

class VisionSensor:
    """视觉传感器处理类

    负责：
    1. 从串口接收器获取原始视觉识别结果
    2. 转换为标准的 VisionData 格式
    3. 提供最新的识别数据访问
    """
    def __init__(self, receiver: SerialReceiver, frame_id: str = "camera"):
        self.receiver = receiver
        self.frame_id = frame_id
        self._last_data = VisionData(
            timestamp=time.time(),
            frame_id=frame_id,
            robots=[]
        )

    def update(self) -> VisionData:
        """更新并返回最新的视觉识别数据"""
        raw = self.receiver.get_data()
        vs = raw.get("vision", {})
        robots_raw = vs.get("robots", [])
        robots = []
        for r in robots_raw:
            robots.append(VisionRobot(
                id=r.get("id"),
                team=r.get("team"),
                x=r.get("x"),
                y=r.get("y")
            ))
        ts = raw.get("timestamp", time.time())
        self._last_data = VisionData(
            timestamp=ts,
            frame_id=self.frame_id,
            robots=robots
        )
        return self._last_data

    def get_latest_data(self) -> VisionData:
        """获取最新的视觉数据，不等待新数据"""
        return self._last_data