import numpy as np
from sensors.serial_receiver import SerialReceiver
from common.types import LidarData
import time

class LidarSensor:
    """激光雷达传感器处理类
    
    负责：
    1. 从串口接收器获取激光雷达数据
    2. 转换为标准的LidarData格式
    3. 提供点云数据访问
    """
    def __init__(self, receiver: SerialReceiver, frame_id: str = "lidar_link"):
        self.receiver = receiver
        self.frame_id = frame_id
        self._last_data = None

    def get_scan(self) -> LidarData:
        """获取最新的激光雷达扫描数据
        
        Returns:
            LidarData: 包含点云和反射率的数据对象
        """
        data = self.receiver.get_data()
        lidar_data = data.get("lidar", {})
        
        # 获取原始数据
        raw_points = np.array(lidar_data.get("point_cloud", []))
        raw_reflectivity = np.array(lidar_data.get("reflectivity", []))
        raw_ranges = np.array(lidar_data.get("ranges", []))
        raw_angles = np.array(lidar_data.get("angles", []))
        
        # 确保数据有效性
        if len(raw_points) == 0:
            if self._last_data:
                return self._last_data
            # 如果没有数据，返回空扫描
            return LidarData(
                timestamp=time.time(),
                frame_id=self.frame_id,
                points=np.zeros((0, 3)),
                intensities=np.array([]),
                ranges=np.array([]),
                angles=np.array([])
            )
        
        # 创建新的激光雷达数据对象
        self._last_data = LidarData(
            timestamp=time.time(),
            frame_id=self.frame_id,
            points=raw_points,
            intensities=raw_reflectivity,
            ranges=raw_ranges,
            angles=raw_angles
        )
        
        return self._last_data
    
    def get_latest_scan(self) -> LidarData:
        """获取最新的一帧扫描数据，不等待新数据
        
        Returns:
            LidarData: 最近一次的激光雷达数据
        """
        return self._last_data if self._last_data else self.get_scan()