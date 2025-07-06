import numpy as np
from sensors.serial_receiver import SerialReceiver
from common.types import LidarData
import time

class LidarSensor:
    """Unitree4D LiDAR-L1 传感器处理类

    该类负责：
    1. 从 SerialReceiver 获取原始 L1 点云及传感器数据
    2. 解析并填充到标准 LidarData:
       - points: Nx3 点云坐标
       - intensities: 反射率/强度
       - ranges: 距离值 (米)
       - angles: 采样角度 (弧度)
       - times: 每点相对于消息时间戳的时间偏移 (秒)
       - rings: 每点所在激光线圈编号
       - dirty_percentage: 雷达窗口脏污程度 (%)
       - imu_quaternion: 内置 IMU 四元数 [x, y, z, w]
    3. 支持空扫描回退，提供最新扫描缓存
    4. 方法:
       - get_scan(): 拉取并返回最新 LidarData
       - get_latest_scan(): 获取缓存的上一次扫描
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
        # 1. 从串口接收器获取原始 JSON 数据，并带有主控和本地时间戳
        data = self.receiver.get_data()
        lidar_data = data.get("lidar", {})

        # 2. 提取并转换各字段为 numpy 数组或标量
        #    注意 key 名称与 Unitree L1 SDK 输出保持一致
        raw_points = np.array(lidar_data.get("point_cloud", []))      # Nx3 坐标
        raw_reflectivity = np.array(lidar_data.get("reflectivity", []))
        raw_ranges = np.array(lidar_data.get("ranges", []))
        if raw_ranges.size == 0 and raw_points.size > 0:
            raw_ranges = np.linalg.norm(raw_points, axis=1)  # 根据点云坐标计算距离
        raw_angles = np.array(lidar_data.get("angles", []))
        if raw_angles.size == 0 and raw_points.size > 0:
            # 根据 XY 投影计算水平角度
            raw_angles = np.arctan2(raw_points[:,1], raw_points[:,0])
        raw_times = np.array(lidar_data.get("time", []))              # 每点时间偏移
        raw_rings = np.array(lidar_data.get("ring", []))              # 线圈编号
        dirty_pct = lidar_data.get("dirty_percentage", 0.0)           # 雷达脏污百分比
        imu_quat = np.array(lidar_data.get("quaternion", []))        # 内置 IMU 四元数

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

        # 3. 构建标准 LidarData 对象并缓存
        #    未包含 fields 则用默认值
        self._last_data = LidarData(
            timestamp=time.time(),
            frame_id=self.frame_id,
            points=raw_points,
            intensities=raw_reflectivity,
            ranges=raw_ranges,
            angles=raw_angles,
            times=raw_times,
            rings=raw_rings,
            dirty_percentage=dirty_pct,
            imu_quaternion=imu_quat
        )

        return self._last_data

    def get_latest_scan(self) -> LidarData:
        """获取最新的一帧扫描数据，不等待新数据

        Returns:
            LidarData: 最近一次的激光雷达数据
        """
        return self._last_data if self._last_data else self.get_scan()