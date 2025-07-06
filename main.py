"""
主循环脚本: 系统集成和实时数据流
功能：
  - 串口接收 → 数据标准化 → 坐标变换 → 地图构建 → 目标跟踪
  - 配置加载、参数管理、日志与异常处理
"""
import time
import logging
import argparse
import json

from sensors.serial_receiver import SerialReceiver
from sensors.imu import IMU
from sensors.wheel_encoder import WheelEncoder
from sensors.lidar import LidarSensor
from sensors.radar_station import RadarStation
from sensors.vision import VisionSensor
from common.types import Transform
from transforms.lidar_to_vehicle import transform_lidar_to_vehicle
from mapping.map_builder import build_map
from tracking.target_tracker import TargetTracker
from world_model import WorldModel
from localization.dead_reckoning import DeadReckoning
from localization.imu_calibration import IMUCalibration
from localization.ekf_fusion import EKFFusion
from localization.turret_pose import TurretPose

# 默认配置，可通过 --config 指定 JSON 文件覆盖
default_config = {
    "serial": {"port": "/dev/ttyUSB0", "baudrate": 115200},
    "loop_rate": 10,             # 主循环频率 (Hz)
    "transforms": {
        "lidar_to_vehicle": None # 需在 config.json 中填入 Transform dict
    },
    "map": {"grid_size": 0.1, "size": [100, 100]},
    "tracker": {"max_age": 0.5}
}

def load_config(path):
    with open(path, 'r', encoding='utf-8') as f:
        return json.load(f)

def setup_logger(level=logging.INFO):
    logging.basicConfig(
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
        level=level
    )
    return logging.getLogger("RobotAI")

def main():
    parser = argparse.ArgumentParser(description="RobotAI 主循环脚本")
    parser.add_argument('--config', help='JSON 配置文件路径')
    args = parser.parse_args()
    cfg = default_config
    if args.config:
        try:
            user_cfg = load_config(args.config)
            cfg.update(user_cfg)
        except Exception as e:
            print(f"配置加载失败: {e}")
            return

    logger = setup_logger()
    logger.info("启动主循环")

    # 模块初始化
    receiver = SerialReceiver(cfg['serial']['port'], cfg['serial']['baudrate'])
    imu_sensor = IMU(receiver)
    wheel_sensor = WheelEncoder(receiver)
    lidar_sensor = LidarSensor(receiver)
    radar_sensor = RadarStation(receiver)
    vision_sensor = VisionSensor(receiver)
    # Localization 模块初始化
    dr = DeadReckoning()
    imu_calib = IMUCalibration()
    ekf = EKFFusion()
    turret_localizer = TurretPose()
    tracker = TargetTracker(cfg['tracker']['max_age'])

    # world model 初始化
    wm = WorldModel(
        self_pose=None,
        turret_state=None,
        task_points=[],
        static_obstacles=[],
        dynamic_obstacles=[],
        occupancy_grid=None,
        robots=[]
    )

    period = 1.0 / cfg['loop_rate']
    while True:
        start_t = time.time()
        try:
            # 1. 传感器更新 & 标准化
            imu_data = imu_sensor.update()
            wheel_data = wheel_sensor.update()
            raw_lidar = lidar_sensor.get_scan()
            radar_data = radar_sensor.update()
            vision_data = vision_sensor.update()
            # 1.5 位姿估计（Localization）
            # dr_pose = dr.update(wheel_data)
            # calib_imu = imu_calib.update(imu_data)
            # self_pose = ekf.update(dr_pose, imu_data, calib_imu)
            # turret_state = turret_localizer.update(imu_data)
            # 更新 WorldModel 位姿
            # wm.self_pose = self_pose
            # wm.turret_state = turret_state

            # 2. 坐标变换
            if cfg['transforms']['lidar_to_vehicle']:
                raw_lidar = transform_lidar_to_vehicle(
                    raw_lidar,
                    Transform(**cfg['transforms']['lidar_to_vehicle'])
                )

            # 3. 地图构建 & 更新 WorldModel
            grid, clusters = build_map(
                raw_lidar.points,
                cfg['map']['grid_size'],
                cfg['map']['size'],
                wm
            )

            # 4. 目标跟踪 & 更新 WorldModel
            tracked = tracker.update(radar_data, vision_data)
            wm.robots = tracked

            # 5. 其他处理（可选：日志、可视化、路径规划）
            # ...

        except Exception as e:
            logger.error(f"运行时错误: {e}", exc_info=True)

        elapsed = time.time() - start_t
        sleep_time = max(0, period - elapsed)
        time.sleep(sleep_time)

if __name__ == '__main__':
    main()