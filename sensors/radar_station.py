import time
from sensors.serial_receiver import SerialReceiver
from common.types import RadarStationData, RobotInfo

class RadarStation:
    """雷达站传感器处理类

    负责：
    1. 从串口接收器获取原始雷达站数据
    2. 转换为标准的 RadarStationData 格式
    3. 提供最新的雷达站数据访问
    """
    def __init__(self, receiver: SerialReceiver, frame_id: str = "radar_station"):
        self.receiver = receiver
        self.frame_id = frame_id
        self._last_data = RadarStationData(
            timestamp=time.time(),
            frame_id=frame_id,
            robots=[]
        )

    def update(self) -> RadarStationData:
        """更新并返回最新的雷达站数据"""
        raw = self.receiver.get_data()
        rs = raw.get("radar_station", {})
        robots_raw = rs.get("robots", [])
        robots = []
        for r in robots_raw:
            robots.append(RobotInfo(
                id=r.get("id"),
                team=r.get("team"),
                x=r.get("x"),
                y=r.get("y"),
                color=r.get("color", "")
            ))
        ts = raw.get("timestamp", time.time())
        self._last_data = RadarStationData(
            timestamp=ts,
            frame_id=self.frame_id,
            robots=robots
        )
        return self._last_data

    def get_latest_data(self) -> RadarStationData:
        """获取最新的雷达站数据，不等待新数据"""
        return self._last_data