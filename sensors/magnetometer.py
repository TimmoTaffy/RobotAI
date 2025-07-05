import numpy as np
import time
from sensors.serial_receiver import SerialReceiver
from common.types import MagData

class Magnetometer:
    """磁力计传感器处理类

    负责：
    1. 从串口接收器获取原始磁力计数据
    2. 转换为标准的 MagData 格式
    3. 提供最新的磁力计数据访问
    """
    def __init__(self, receiver: SerialReceiver, frame_id: str = "mag_link"):
        self.receiver = receiver
        self.frame_id = frame_id
        # 初始化为零场强
        self._last_data = MagData(
            timestamp=time.time(),
            frame_id=frame_id,
            field=np.zeros(3)
        )

    def update(self) -> MagData:
        """更新并返回最新的磁力计数据"""
        raw = self.receiver.get_data()
        mag = raw.get("magnetometer", {})
        field = np.array(mag.get("field", [0.0, 0.0, 0.0]))
        ts = raw.get("timestamp", time.time())
        self._last_data = MagData(
            timestamp=ts,
            frame_id=self.frame_id,
            field=field
        )
        return self._last_data

    def get_latest_data(self) -> MagData:
        """获取最新的磁力计数据，不等待新数据"""
        return self._last_data