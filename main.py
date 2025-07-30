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
import numpy as np

from src.sensors.serial_receiver import SerialReceiver
from src.sensors.imu import IMU
from src.sensors.wheel_encoder import WheelEncoder
from src.sensors.lidar import LidarSensor
from src.sensors.radar_station import RadarStation
from src.sensors.vision import VisionSensor
from src.common.types import Transform
from src.transforms.lidar_to_vehicle import transform_lidar_to_vehicle
from src.mapping.map_builder import build_map
from src.tracking import create_production_tracker
from world_model import WorldModel, TacticalInfo
from src.planning.path_planner import AStarPlanner
from src.planning.motion_planner import smooth_path, generate_velocity_profile
from src.control.trajectory_tracker import TrajectoryTracker as PurePursuitTracker
from src.control.mpc_controller import MPCController
from src.localization.dead_reckoning import DeadReckoning
from src.localization.imu_calibration import IMUCalibration
from src.localization.ekf_fusion import EKFFusion
from src.localization.turret_pose import TurretPose

# 默认配置，可通过 --config 指定 JSON 文件覆盖
default_config = {
    "serial": {"port": "/dev/ttyUSB0", "baudrate": 115200},
    "loop_rate": 10,             # 主循环频率 (Hz)
    "transforms": {
        "lidar_to_vehicle": None # 需在 config.json 中填入 Transform dict
    },
    "map": {"grid_size": 0.1, "size": [100, 100]},
    "tracker": {"max_age": 0.5},
    # 导航与规划配置
    "planning": {
        "goal": [50, 50],        # 目标位置 (x, y)
        "num_points": 100        # 平滑轨迹点数
    },
    # 控制配置
    "control": {
        "method": "pure_pursuit",  # "pure_pursuit" 或 "mpc"
        "lookahead_distance": 1.0,
        "desired_speed": 1.0,
        "mpc_horizon": 10,
        "mpc_dt": 0.1
    }
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
    tracker = create_production_tracker(dt=0.1)
    # 规划与控制模块初始化
    planner = None  # 在获得地图后初始化
    pure_tracker = PurePursuitTracker(
        lookahead_distance=cfg['control']['lookahead_distance'],
        desired_speed=cfg['control']['desired_speed']
    )
    mpc = MPCController(
        horizon=cfg['control']['mpc_horizon'],
        dt=cfg['control']['mpc_dt']
    )

    # Localization 模块初始化
    dr = DeadReckoning()
    imu_calib = IMUCalibration()
    ekf = EKFFusion()
    turret_localizer = TurretPose()

    # world model 初始化
    wm = WorldModel(
        self_pose=None,
        turret_state=None,
        task_points=[],
        static_obstacles=[],
        dynamic_obstacles=[],
        occupancy_grid=None,
        ground_slopes={},
        tracked_targets=[],
        tactical_info=TacticalInfo()
    )

    # Host AI 模块及高阶状态机初始化
    from src.host.modules.navigator import Navigator
    from src.host.modules.vision_module import VisionModule
    from src.host.modules.turret_controller import TurretController
    from src.host.modules.weapon_controller import WeaponController
    from src.host.modules.patrol_timer import PatrolTimer
    from src.host.high_level_fsm import HighLevelStateMachine
    # 实例化子模块
    navigator = Navigator(None, cfg['map']['grid_size'], cfg['control']['desired_speed'])
    vision_mod = VisionModule()
    turret_ctrl = TurretController()
    weapon_ctrl = WeaponController(ammo_count=cfg.get('initial_ammo', 10))
    patrol_timer = PatrolTimer(interval=cfg.get('patrol_interval', 5.0))
    # 创建状态机并绑定定时器回调
    fsm = HighLevelStateMachine(navigator, vision_mod, turret_ctrl, weapon_ctrl, patrol_timer)
    patrol_timer.callback = lambda: fsm.post_event('START_PATROL')
    # 启动状态机初始巡逻
    fsm.post_event('START_PATROL')
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

            # Host AI: 根据子模块状态触发高阶状态机事件
            # TODO: 用实际数据判断条件
            # 如果扫描中发现目标
            fsm.post_event('TARGET_DETECTED')  # 示例触发，替换为实际检测条件
            # 如果跟踪稳定后可进入瞄准
            fsm.post_event('TARGET_STABLE')   # 示例触发，替换为实际稳定条件
            # 如果瞄准完成可准备开火
            fsm.post_event('READY_TO_FIRE')   # 示例触发，替换为实际就绪条件
            # 如果弹药耗尽
            if not weapon_ctrl.can_fire():
                fsm.post_event('OUT_OF_AMMO')
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
                wm,
                cfg['map']
            )

            # 4. 目标跟踪 & 更新 WorldModel
            tracked = tracker.update(radar_data, vision_data)
            wm.tracked_targets = tracked
            wm.update_tactical_assessment()  # 更新战术评估
            # 5. 路径规划 & 运动规划
            # 初始化 planner 需在首次循环中执行
            if planner is None:
                planner = AStarPlanner(wm.occupancy_grid, cfg['map']['grid_size'])
            start = tuple(wm.self_pose.position)
            goal = tuple(cfg['planning']['goal'])
            raw_path = planner.plan(start, goal)
            smooth_traj = smooth_path(raw_path, cfg['planning']['num_points'])
            # 5.5 运动规划：给平滑轨迹分配速度，生成带速度的轨迹点
            motion_traj = generate_velocity_profile(smooth_traj, cfg['control']['desired_speed'])
            # 6. 控制命令
            if cfg['control']['method'] == 'mpc':
                # 使用带速度的运动轨迹进行 MPC 控制
                v_cmd, omega_cmd = mpc.update(wm.self_pose, motion_traj)
            else:
                v_cmd, omega_cmd = pure_tracker.update(wm.self_pose, motion_traj)
            # 发送或记录命令
            logger.debug(f"Control cmd: v={v_cmd:.2f}, omega={omega_cmd:.2f}")

            # 5. 其他处理（可选：日志、可视化、路径规划）
            # ...

        except Exception as e:
            logger.error(f"运行时错误: {e}", exc_info=True)

        elapsed = time.time() - start_t
        sleep_time = max(0, period - elapsed)
        time.sleep(sleep_time)

if __name__ == '__main__':
    main()