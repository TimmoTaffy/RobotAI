"""
集成测试模块：测试导航与控制模块的端到端功能
"""
import numpy as np
import pytest
import time
from planning.path_planner import AStarPlanner
from planning.motion_planner import smooth_path, generate_velocity_profile
from control.trajectory_tracker import TrajectoryTracker
from control.mpc_controller import MPCController
from common.types import Pose2D


class TestNavigationControlIntegration:
    """导航与控制集成测试"""
    
    def setup_method(self):
        """测试初始化"""
        self.grid_size = 0.5
        self.grid = np.zeros((20, 20), dtype=int)
        self.planner = AStarPlanner(self.grid, self.grid_size)
        
    def test_end_to_end_straight_navigation(self):
        """端到端直线导航测试"""
        start = (0.0, 0.0)
        goal = (9.0, 0.0)
        
        # 1. 路径规划
        raw_path = self.planner.plan(start, goal)
        assert len(raw_path) > 0
        
        # 2. 轨迹平滑
        smooth_traj = smooth_path(raw_path, num_points=50)
        assert smooth_traj.shape[0] == 50
        
        # 3. 速度分配
        velocity_traj = generate_velocity_profile(smooth_traj, speed=1.0)
        assert velocity_traj.shape == (50, 3)
        
        # 4. 控制器跟踪测试
        current_pose = Pose2D(position=np.array([0.0, 0.0]), theta=0.0, timestamp=0.0)
        
        # Pure Pursuit
        pp_tracker = TrajectoryTracker(lookahead_distance=1.0)
        v_pp, steer_pp = pp_tracker.update(current_pose, velocity_traj)
        assert v_pp > 0
        assert abs(steer_pp) < 0.1  # 直线，转向应很小
        
        # MPC
        ref_traj_mpc = np.zeros((11, 3))  # horizon=10
        for i in range(11):
            if i < len(velocity_traj):
                ref_traj_mpc[i] = [velocity_traj[i, 0], velocity_traj[i, 1], 0.0]
            else:
                ref_traj_mpc[i] = ref_traj_mpc[i-1]
        
        mpc = MPCController(horizon=10, dt=0.1)
        v_mpc, omega_mpc = mpc.update(current_pose, ref_traj_mpc)
        assert v_mpc > 0
        assert abs(omega_mpc) < 0.1
        
    def test_obstacle_avoidance_navigation(self):
        """障碍物避障导航测试"""
        # 设置障碍物
        self.grid[5:15, 8:12] = 1
        self.grid[10, 10] = 0  # 开个口子
        
        start = (0.0, 10.0)
        goal = (19.0, 10.0)
        
        # 完整流程
        raw_path = self.planner.plan(start, goal)
        assert len(raw_path) > 2  # 应该绕行，不只是直线
        
        # 验证路径避开障碍
        for x, y in raw_path:
            i, j = int(x/self.grid_size), int(y/self.grid_size)
            if 0 <= i < self.grid.shape[1] and 0 <= j < self.grid.shape[0]:
                assert self.grid[j, i] == 0
        
        smooth_traj = smooth_path(raw_path, num_points=30)
        velocity_traj = generate_velocity_profile(smooth_traj, speed=1.5)
        
        # 测试控制器在复杂路径上的表现
        current_pose = Pose2D(position=np.array([0.0, 10.0]), theta=0.0, timestamp=0.0)
        tracker = TrajectoryTracker(lookahead_distance=2.0)
        v, steer = tracker.update(current_pose, velocity_traj)
        
        assert v > 0
        # 复杂路径可能需要转向
        assert abs(steer) <= np.pi/2
        
    def test_controller_comparison(self):
        """控制器性能对比测试"""
        # 创建一个曲线轨迹
        raw_path = [(0, 0), (2, 0), (4, 2), (6, 4), (8, 4)]
        smooth_traj = smooth_path(raw_path, num_points=40)
        velocity_traj = generate_velocity_profile(smooth_traj, speed=1.0)
        
        current_pose = Pose2D(position=np.array([0.0, 0.5]), theta=0.1, timestamp=0.0)  # 有初始偏差
        
        # Pure Pursuit
        pp_tracker = TrajectoryTracker(lookahead_distance=1.5)
        v_pp, steer_pp = pp_tracker.update(current_pose, velocity_traj)
        
        # MPC (构建参考轨迹)
        horizon = 8
        ref_traj = np.zeros((horizon+1, 3))
        for i in range(horizon+1):
            if i < len(velocity_traj):
                ref_traj[i] = [velocity_traj[i, 0], velocity_traj[i, 1], 
                              np.arctan2(velocity_traj[min(i+1, len(velocity_traj)-1), 1] - velocity_traj[i, 1],
                                        velocity_traj[min(i+1, len(velocity_traj)-1), 0] - velocity_traj[i, 0])]
            else:
                ref_traj[i] = ref_traj[i-1]
        
        mpc = MPCController(horizon=horizon, dt=0.1)
        v_mpc, omega_mpc = mpc.update(current_pose, ref_traj)
        
        # 验证两个控制器都产生合理输出
        assert v_pp > 0 and v_mpc > 0
        assert abs(steer_pp) <= np.pi/2
        assert abs(omega_mpc) <= 1.0
        
    def test_performance_benchmarks(self):
        """性能基准测试"""
        # 大规模路径规划性能
        large_grid = np.zeros((50, 50), dtype=int)
        # 添加一些随机障碍
        np.random.seed(42)
        obstacle_count = 200
        for _ in range(obstacle_count):
            i, j = np.random.randint(0, 50, 2)
            large_grid[i, j] = 1
        
        large_planner = AStarPlanner(large_grid, grid_size=0.2)
        start_time = time.time()
        path = large_planner.plan((0.0, 0.0), (9.8, 9.8))
        planning_time = time.time() - start_time
        
        assert planning_time < 0.5  # 规划时间 < 0.5秒
        assert len(path) > 0
        
        # 轨迹平滑性能
        if len(path) > 2:
            start_time = time.time()
            smooth_traj = smooth_path(path, num_points=100)
            smoothing_time = time.time() - start_time
            
            assert smoothing_time < 0.1  # 平滑时间 < 0.1秒
            
            # 控制器计算性能
            velocity_traj = generate_velocity_profile(smooth_traj, speed=2.0)
            current_pose = Pose2D(position=np.array([0.0, 0.0]), theta=0.0, timestamp=0.0)
            
            # Pure Pursuit性能
            tracker = TrajectoryTracker()
            start_time = time.time()
            for _ in range(100):
                v, steer = tracker.update(current_pose, velocity_traj)
            pp_time = time.time() - start_time
            
            assert pp_time < 0.01  # 100次PP计算 < 0.01秒
            
    def test_edge_cases_robustness(self):
        """边界情况鲁棒性测试"""
        # 极短路径
        short_start = (1.0, 1.0)
        short_goal = (1.5, 1.0)
        short_path = self.planner.plan(short_start, short_goal)
        
        if len(short_path) >= 2:
            short_smooth = smooth_path(short_path, num_points=10)
            short_velocity = generate_velocity_profile(short_smooth, speed=0.5)
            
            pose = Pose2D(position=np.array([1.0, 1.0]), theta=0.0, timestamp=0.0)
            tracker = TrajectoryTracker(lookahead_distance=0.5)
            v, steer = tracker.update(pose, short_velocity)
            
            assert v >= 0
            assert abs(steer) <= np.pi/2
        
        # 零速度轨迹
        zero_speed_traj = generate_velocity_profile(
            np.array([[0, 0], [1, 0], [2, 0]]), speed=0.0
        )
        pose = Pose2D(position=np.array([0.0, 0.0]), theta=0.0, timestamp=0.0)
        tracker = TrajectoryTracker()
        v, steer = tracker.update(pose, zero_speed_traj)
        
        assert v == 0.0
        assert abs(steer) <= np.pi/2
