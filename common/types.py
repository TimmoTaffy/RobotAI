"""公共数据类型定义

这个模块定义了整个系统中共用的数据结构，主要用于：
1. 传感器数据的标准化表示
2. 位姿和坐标变换的统一描述
3. 不同模块间的数据交换
"""

from dataclasses import dataclass
import numpy as np
from typing import Optional, List

@dataclass
class Pose3D:
    """3D 位姿表示
    
    主要用于：
    1. 相机、激光雷达等3D传感器的位姿表示
    2. transforms 模块进行3D坐标变换时的输入/输出
    3. world_model 中记录目标的3D位置和姿态
    
    使用示例：
    ```python
    camera_pose = Pose3D(
        position=np.array([1.0, 0.0, 0.5]),  # 相机在车体坐标系下的位置
        orientation=np.array([0.0, 0.0, 0.0]), # 相机朝向
        timestamp=time.time()  # 获取位姿的时间
    )
    ```
    """
    position: np.ndarray    # [x, y, z]
    orientation: np.ndarray # [roll, pitch, yaw]，单位：弧度
    timestamp: float        # UNIX 时间戳

@dataclass
class Pose2D:
    """2D 位姿表示（平面运动）
    
    主要用于：
    1. 车体在世界坐标系下的位姿表示
    2. 平面目标跟踪和路径规划
    3. 2D 地图中的位置标记
    
    使用示例：
    ```python
    vehicle_pose = Pose2D(
        position=np.array([2.5, 1.0]),  # 车体在场地中的位置
        theta=np.pi/2,  # 车头朝北
        timestamp=time.time()
    )
    ```
    """
    position: np.ndarray    # [x, y]
    theta: float           # 朝向角，单位：弧度
    timestamp: float       # UNIX 时间戳

@dataclass
class Transform:
    """坐标系之间的变换关系
    
    主要用于：
    1. transforms 模块中描述两个坐标系之间的相对位姿
    2. 构建和维护坐标变换树
    3. 缓存静态坐标变换（如相机外参）
    
    使用示例：
    ```python
    camera_to_vehicle = Transform(
        translation=np.array([0.2, 0.0, 0.5]),  # 相机相对车体的平移
        rotation=np.array([0.0, 0.0, 0.0]),     # 相机相对车体的旋转
        parent_frame="vehicle",                  # 父坐标系：车体
        child_frame="camera"                     # 子坐标系：相机
    )
    ```
    """
    translation: np.ndarray # [x, y, z]
    rotation: np.ndarray   # [roll, pitch, yaw]，单位：弧度
    parent_frame: str      # 父坐标系名称
    child_frame: str       # 子坐标系名称

@dataclass
class SensorData:
    """传感器数据基类
    
    作为所有传感器数据类型的基类，提供：
    1. 统一的时间戳字段，用于传感器同步
    2. 统一的坐标系标识，用于数据关联
    3. 继承此类可以实现新的传感器数据类型
    """
    timestamp: float       # UNIX 时间戳
    frame_id: str         # 坐标系 ID

@dataclass
class ImuData(SensorData):
    """IMU 数据
    
    主要用于：
    1. 姿态解算，获取车体或云台的方向
    2. 速度和位置的积分估计
    3. EKF 定位中的运动预测
    
    使用示例：
    ```python
    imu_data = ImuData(
        timestamp=time.time(),
        frame_id="imu_link",
        angular_velocity=np.array([0.1, 0.0, 0.2]),    # 角速度
        linear_acceleration=np.array([0.0, 0.0, 9.81]), # 加速度
        orientation=np.array([0.0, 0.0, 0.0])          # 可选的姿态
    )
    ```
    """
    angular_velocity: np.ndarray  # [wx, wy, wz]，单位：rad/s
    linear_acceleration: np.ndarray  # [ax, ay, az]，单位：m/s^2
    orientation: Optional[np.ndarray] = None  # [roll, pitch, yaw]，如果有

@dataclass
class TurretState(SensorData):
    """云台状态
    
    主要用于：
    1. 记录云台当前姿态和运动状态
    2. transforms 模块中的云台坐标系变换
    3. 视觉和弹道解算的输入
    
    使用示例：
    ```python
    turret_state = TurretState(
        timestamp=time.time(),
        frame_id="turret",
        orientation=np.array([0.0, 0.1, 0.0]),     # 云台姿态
        angular_velocity=np.array([0.0, 0.1, 0.0]), # 云台角速度
        motor_angles=np.array([0.1, 0.2])          # 电机角度
    )
    ```
    """
    orientation: np.ndarray      # [roll, pitch, yaw]
    angular_velocity: np.ndarray # [wx, wy, wz]
    motor_angles: np.ndarray    # [horizontal, vertical]

@dataclass
class WheelData(SensorData):
    """轮速计数据
    
    主要用于：
    1. 里程计计算
    2. 速度估计
    3. 运动控制反馈
    
    使用示例：
    ```python
    wheel_data = WheelData(
        timestamp=time.time(),
        frame_id="base_link",
        left_speed=1.5,   # 左轮线速度，单位：m/s
        right_speed=1.5,  # 右轮线速度，单位：m/s
        left_pos=0.0,     # 左轮位置（可选），单位：rad
        right_pos=0.0     # 右轮位置（可选），单位：rad
    )
    ```
    """
    left_speed: float     # 左轮线速度，单位：m/s
    right_speed: float    # 右轮线速度，单位：m/s
    left_pos: float = 0.0  # 左轮位置，单位：rad
    right_pos: float = 0.0 # 右轮位置，单位：rad

@dataclass
class LidarData(SensorData):
    """激光雷达数据
    
    主要用于：
    1. 点云处理和障碍物检测
    2. 环境建图和定位
    3. 扫描匹配和位姿估计
    
    使用示例：
    ```python
    lidar_data = LidarData(
        timestamp=time.time(),
        frame_id="lidar_link",
        points=np.array([[x, y, z], ...]),      # Nx3点云数组
        intensities=np.array([0.5, ...]),       # N个反射率值
        ranges=np.array([1.2, ...]),            # N个距离值
        angles=np.array([0.1, ...])             # N个角度值
    )
    ```
    """
    points: np.ndarray       # Nx3点云数组，每行[x, y, z]
    intensities: np.ndarray  # 反射率/强度数组
    ranges: np.ndarray       # 距离数组
    angles: np.ndarray       # 角度数组（弧度）
    times: Optional[np.ndarray] = None    # 每点时间偏移（秒）
    rings: Optional[np.ndarray] = None    # 每点激光线圈编号
    dirty_percentage: float = 0.0         # 雷达脏污百分比
    imu_quaternion: Optional[np.ndarray] = None  # L1输出的IMU四元数

@dataclass
class MagData(SensorData):
    """磁力计数据
    
    主要用于：
    1. 姿态估计中的地磁参考
    
    使用示例：
    ```python
    mag = MagData(
        timestamp=time.time(),
        frame_id="mag_link",
        field=np.array([30.5, -15.2, 50.1])
    )
    ```
    """
    field: np.ndarray  # [x, y, z] 磁场强度 (μT)

@dataclass
class GyroData(SensorData):
    """底盘陀螺仪数据
    
    主要用于：
    1. EKF 定位中的角速度输入
    
    使用示例：
    ```python
    gyro = GyroData(
        timestamp=time.time(),
        frame_id="gyro_link",
        angular_velocity=np.array([0.01, 0.02, 0.03])
    )
    ```
    """
    angular_velocity: np.ndarray  # [wx, wy, wz] (rad/s)

@dataclass
class RobotInfo:
    """单个机器人信息
    """
    id: int
    team: str
    x: float
    y: float
    color: str

@dataclass
class RadarStationData(SensorData):
    """雷达站输出数据
    
    主要用于：
    1. 获取雷达站识别的机器人列表
    
    使用示例：
    ```python
    rs = RadarStationData(
        timestamp=time.time(),
        frame_id="radar_station",
        robots=[RobotInfo(id=1, team="ally", x=1.2, y=3.4, color="blue")]
    )
    ```
    """
    robots: List[RobotInfo]

@dataclass
class VisionRobot:
    """视觉识别的单个机器人信息
    """
    id: int
    team: str
    x: float
    y: float

@dataclass
class VisionData(SensorData):
    """视觉识别数据
    
    主要用于：
    1. 获取视觉识别的机器人列表
    
    使用示例：
    ```python
    vd = VisionData(
        timestamp=time.time(),
        frame_id="camera",
        robots=[VisionRobot(id=3, team="enemy", x=2.5, y=4.5)]
    )
    ```
    """
    robots: List[VisionRobot]
