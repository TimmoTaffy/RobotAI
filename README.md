# RobotAI

## 项目概述

本项目实现哨兵机器人多传感器融合与实时世界建模，主要功能模块包括：

- 传感层（Sensors）：从串口接收 IMU、轮速、LiDAR、雷达站、视觉数据，并标准化为统一数据类型
- 坐标变换（Transforms）：将 LiDAR 点云和其他数据从传感器坐标系转换到车体/世界坐标系
- 本地化（Localization）：基于轮速里程计、IMU 校准与 EKF 融合，估计自车位姿，及云台姿态估计
- 地图构建（Mapping）：对点云做滤地面、障碍聚类，并基于占据栅格算法生成环境地图
- 跟踪（Tracking）：融合雷达和视觉识别结果，维护机器人目标跟踪列表
- 导航与规划（Planning）：基于占据栅格地图做全局/局部路径规划（A*）、运动轨迹平滑（B 样条）
- 控制（Control）：轨迹跟踪（PID/MPC 控制器）
- 主循环（main.py）：集成以上各模块，支持配置加载、日志记录和异常处理，提供实时数据流

## 安装依赖

```bash
pip install -r requirements.txt
```

- 核心依赖：`numpy`, `scikit-learn`, `scipy`

## 运行

```bash
python main.py --config config.json
```

- `config.json`（可选）用于覆盖默认参数，如串口端口、循环频率、地图分辨率、跟踪时长等

## 项目结构

```
RobotAI/
├── common/
│   └── types.py            # 标准数据类型：Pose2D/3D, SensorData, LidarData, RobotInfo 等
├── sensors/                # 传感器接口层
│   ├── serial_receiver.py  # 串口接收与预处理
│   ├── imu.py              # IMU → ImuData
│   ├── wheel_encoder.py    # 轮速 → WheelData
│   ├── lidar.py            # LiDAR → LidarData
│   ├── magnetometer.py     # 磁力计 → MagData
│   ├── base_gyro.py        # 底盘陀螺仪 → GyroData
│   ├── radar_station.py    # 雷达站 → RadarStationData
│   └── vision.py           # 视觉 → VisionData
├── transforms/             # 坐标变换
│   ├── lidar_to_vehicle.py # LiDAR → 车体坐标
│   ├── camera_to_vehicle.py
│   └── vehicle_to_world.py # 车体 → 世界坐标（平面Pose2D）
├── localization/           # 本地化与云台姿态估计
│   ├── dead_reckoning.py
│   ├── imu_calibration.py
│   ├── ekf_fusion.py
│   └── turret_pose.py
├── mapping/                # 地图构建
│   ├── lidar_processor.py  # 滤噪、地面分割、障碍聚类
│   ├── occupancy_grid.py   # 占据栅格生成
│   └── map_builder.py      # 构建并更新 WorldModel
├── tracking/               # 目标跟踪
│   └── target_tracker.py   # 雷达+视觉目标融合与跟踪维护
├── planning/               # 导航与轨迹规划
│   ├── path_planner.py     # A* 路径规划
│   └── motion_planner.py   # B 样条路径平滑
├── control/                # 运动控制
│   ├── trajectory_tracker.py
│   └── mpc_controller.py
├── world_model.py          # 世界模型数据结构，含地图、障碍、位姿、目标等
├── main.py                 # 系统集成主循环脚本
├── config.json             # 运行配置文件
├── config说明.md           # 配置字段说明文档
└── requirements.txt        # Python 包依赖
```

## 核心流程

1. **传感**：`SerialReceiver` 接收 JSON → 转为 `ImuData`、`LidarData` 等
2. **本地化**：轮速里程计 + IMU 校准 + EKF 融合 → `Pose2D`; 云台姿态 → `TurretState`
3. **坐标变换**：点云从 LiDAR 坐标系变到车体，再可进一步到世界
4. **地图构建**：点云滤地面 → 障碍聚类 → 占据栅格刷新 → `WorldModel.occupancy_grid`
5. **目标跟踪**：雷达站 + 视觉识别 → 多源融合 → `WorldModel.robots`
6. **规划**：A* 对占据栅格寻路 → B 样条平滑 → 速度/加速度配置
7. **控制**：轨迹跟踪（PID/MPC）→ 电机指令下发

## 日志与异常

- 使用内置 `logging` 模块，主循环捕获并记录异常堆栈，保证持续运行

---

更多细节见各模块代码注释和协议文档 `2.3 主控输出JSON协议`。
