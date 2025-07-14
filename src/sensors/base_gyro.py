import numpy as np
import time
from src.sensors.serial_receiver import SerialReceiver
from src.common.types import GyroData

class BaseGyro:
    """底盘陀螺仪处理类

    负责：
    1. 从串口接收器获取原始陀螺仪角速度
    2. 转换为标准的 GyroData 格式
    3. 提供最新的陀螺仪数据访问
    """
    def __init__(self, receiver: SerialReceiver, frame_id: str = "gyro_link"):
        self.receiver = receiver
        self.frame_id = frame_id
        self._last_data = GyroData(
            timestamp=time.time(),
            frame_id=frame_id,
            angular_velocity=np.zeros(3)
        )

    def update(self) -> GyroData:
        raw = self.receiver.get_data()
        gyro = raw.get("base_gyro", {})
        ang = np.array(gyro.get("angular_velocity", [0.0, 0.0, 0.0]))
        ts = raw.get("timestamp", time.time())
        self._last_data = GyroData(
            timestamp=ts,
            frame_id=self.frame_id,
            angular_velocity=ang
        )
        return self._last_data

    def get_latest_data(self) -> GyroData:
        return self._last_data