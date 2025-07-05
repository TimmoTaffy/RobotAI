from sensors.serial_receiver import SerialReceiver
from common.types import WheelData
import time

class WheelEncoder:
    """轮速计处理类
    
    负责：
    1. 从串口接收器获取轮速数据
    2. 转换为标准的WheelData格式
    3. 提供最新的轮速状态访问
    """
    def __init__(self, receiver: SerialReceiver, frame_id: str = "base_link"):
        self.receiver = receiver
        self.frame_id = frame_id
        self._last_data = WheelData(
            timestamp=time.time(),
            frame_id=frame_id,
            left_speed=0.0,
            right_speed=0.0
        )

    def update(self) -> WheelData:
        """更新并返回最新的轮速数据
        
        Returns:
            WheelData: 包含两轮速度的数据对象
        """
        data = self.receiver.get_data()
        wheel_data = data.get("wheel", {})
        
        # 获取速度并转换单位（如果需要的话）
        left_speed = wheel_data.get("left", 0.0)
        right_speed = wheel_data.get("right", 0.0)
        
        # 创建新的轮速数据对象
        self._last_data = WheelData(
            timestamp=time.time(),
            frame_id=self.frame_id,
            left_speed=left_speed,
            right_speed=right_speed
        )
        
        return self._last_data
    
    def get_latest_data(self) -> WheelData:
        """获取最新的轮速数据
        
        Returns:
            WheelData: 最近一次更新的轮速数据
        """
        return self._last_data