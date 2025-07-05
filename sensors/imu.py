import numpy as np
from sensors.serial_receiver import SerialReceiver
from common.types import ImuData
import time

class IMU:
    """IMU传感器处理类
    
    负责：
    1. 从串口接收器获取原始IMU数据
    2. 将数据转换为标准的ImuData格式
    3. 提供最新的IMU状态访问
    """
    def __init__(self, receiver: SerialReceiver, frame_id: str = "imu_link"):
        self.receiver = receiver
        self.frame_id = frame_id
        self._last_data = ImuData(
            timestamp=time.time(),
            frame_id=frame_id,
            angular_velocity=np.zeros(3),
            linear_acceleration=np.zeros(3),
            orientation=np.zeros(3)
        )

    def update(self) -> ImuData:
        """更新并返回最新的IMU数据
        
        Returns:
            ImuData: 包含完整IMU信息的数据对象
        """
        data = self.receiver.get_data()
        imu_data = data.get("imu", {})
        
        # 转换为numpy数组并确保正确的单位
        gyro = np.array(imu_data.get("gyro", [0.0, 0.0, 0.0]))
        acc = np.array(imu_data.get("acc", [0.0, 0.0, 0.0]))
        yaw = imu_data.get("yaw", 0.0)
        
        # 创建新的IMU数据对象
        self._last_data = ImuData(
            timestamp=time.time(),
            frame_id=self.frame_id,
            angular_velocity=gyro,      # rad/s
            linear_acceleration=acc,    # m/s^2
            orientation=np.array([0.0, 0.0, yaw])  # [roll, pitch, yaw]
        )
        
        return self._last_data
    
    def get_latest_data(self) -> ImuData:
        """获取最新的IMU数据
        
        Returns:
            ImuData: 最近一次更新的IMU数据
        """
        return self._last_data