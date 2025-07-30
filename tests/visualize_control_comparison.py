"""
控制器性能对比可视化工具：展示Pure Pursuit vs MPC的控制效果
"""
import matplotlib.pyplot as plt
import numpy as np
from src.control.trajectory_tracker import TrajectoryTracker
from src.control.mpc_controller import MPCController
from src.common.types import Pose2D
import time


class ControllerComparison:
    """控制器性能对比分析工具"""
    
    def __init__(self, dt: float = 0.1):
        self.dt = dt
        
    def create_test_trajectory(self, traj_type: str = "s_curve", length: int = 50):
        """创建测试轨迹"""
        if traj_type == "straight":
            # 直线轨迹
            t = np.linspace(0, 5, length)
            x = t
            y = np.zeros_like(t)
            theta = np.zeros_like(t)
            
        elif traj_type == "circle":
            # 圆形轨迹
            t = np.linspace(0, 2*np.pi, length)
            radius = 3.0
            x = radius * np.cos(t)
            y = radius * np.sin(t)
            theta = t + np.pi/2  # 切线方向
            
        elif traj_type == "s_curve":
            # S型轨迹
            t = np.linspace(0, 4*np.pi, length)
            x = t / (4*np.pi) * 10  # 0到10米
            y = 2 * np.sin(t / 2)  # S型摆动
            # 计算切线角度
            dx_dt = 10 / (4*np.pi)
            dy_dt = np.cos(t / 2)
            theta = np.arctan2(dy_dt, dx_dt)
            
        elif traj_type == "figure_8":
            # 8字形轨迹
            t = np.linspace(0, 4*np.pi, length)
            scale = 2.0
            x = scale * np.sin(t)
            y = scale * np.sin(t) * np.cos(t)
            # 切线角度
            dx_dt = scale * np.cos(t)
            dy_dt = scale * (np.cos(2*t))
            theta = np.arctan2(dy_dt, dx_dt)
            
        return np.column_stack([x, y, theta])
    
    def simulate_controller(self, controller, trajectory, initial_pose, simulation_time=5.0):
        """仿真单个控制器"""
        steps = int(simulation_time / self.dt)
        
        # 记录数据
        poses = []
        controls = []
        times = []
        
        current_pose = initial_pose
        
        for step in range(steps):
            t = step * self.dt
            
            # 计算控制命令
            start_time = time.time()
            if isinstance(controller, MPCController):
                # MPC需要未来轨迹
                horizon = controller.N
                start_idx = min(step, len(trajectory) - horizon - 1)
                ref_traj = trajectory[start_idx:start_idx + horizon + 1]
                if len(ref_traj) < horizon + 1:
                    # 扩展轨迹
                    last_point = trajectory[-1]
                    extension = np.tile(last_point, (horizon + 1 - len(ref_traj), 1))
                    ref_traj = np.vstack([ref_traj, extension])
                v, omega = controller.update(current_pose, ref_traj)
            else:
                # Pure Pursuit只需要路径点
                v, steering = controller.update(current_pose, trajectory)
                # 转换转向角到角速度 (简化运动学)
                L = controller.L if hasattr(controller, 'L') else 1.0
                omega = v * np.tan(steering) / L
                
            compute_time = time.time() - start_time
            
            # 记录状态
            poses.append([current_pose.position[0], current_pose.position[1], current_pose.theta])
            controls.append([v, omega])
            times.append(compute_time)
            
            # 更新状态 (简单积分)
            x, y, theta = poses[-1]
            x_new = x + v * np.cos(theta) * self.dt
            y_new = y + v * np.sin(theta) * self.dt
            theta_new = theta + omega * self.dt
            
            current_pose = Pose2D(position=np.array([x_new, y_new]), 
                                theta=theta_new, 
                                timestamp=t + self.dt)
        
        return np.array(poses), np.array(controls), np.array(times)
    
    def compare_controllers(self, trajectory_type="s_curve"):
        """对比两种控制器的性能"""
        # 创建测试轨迹
        trajectory = self.create_test_trajectory(trajectory_type)
        
        # 初始位姿 (稍微偏离轨迹起点)
        initial_pose = Pose2D(
            position=np.array([trajectory[0,0] + 0.5, trajectory[0,1] + 0.3]), 
            theta=trajectory[0,2] + 0.2,
            timestamp=0.0
        )
        
        # 创建控制器
        pp_controller = TrajectoryTracker(
            lookahead_distance=1.5,
            wheelbase=1.0,
            max_speed=2.0,
            desired_speed=1.0
        )
        
        mpc_controller = MPCController(
            horizon=10,
            dt=self.dt,
            v_bounds=(0.0, 2.0),
            omega_bounds=(-1.0, 1.0)
        )
        
        print(f"🚗 Controller Performance Comparison - {trajectory_type} trajectory")
        
        # 仿真Pure Pursuit
        print("  📊 Pure Pursuit simulation...")
        pp_poses, pp_controls, pp_times = self.simulate_controller(
            pp_controller, trajectory, initial_pose)
        
        # 仿真MPC
        print("  📊 MPC simulation...")
        mpc_poses, mpc_controls, mpc_times = self.simulate_controller(
            mpc_controller, trajectory, initial_pose)
        
        # 性能分析
        results = self._analyze_performance(trajectory, pp_poses, pp_controls, 
                                          mpc_poses, mpc_controls, pp_times, mpc_times)
        
        # 可视化结果
        self._plot_comparison(trajectory, pp_poses, mpc_poses, pp_controls, 
                            mpc_controls, results, trajectory_type)
        
        return results
    
    def _analyze_performance(self, ref_traj, pp_poses, pp_controls, 
                           mpc_poses, mpc_controls, pp_times, mpc_times):
        """分析控制器性能指标"""
        
        def compute_tracking_error(poses, reference):
            """计算跟踪误差"""
            errors = []
            for i, pose in enumerate(poses):
                # 找到最近的参考点
                distances = np.linalg.norm(reference[:, :2] - pose[:2], axis=1)
                closest_idx = np.argmin(distances)
                ref_point = reference[closest_idx]
                
                # 横向误差
                lateral_error = distances[closest_idx]
                
                # 航向误差
                heading_error = abs(pose[2] - ref_point[2])
                heading_error = min(heading_error, 2*np.pi - heading_error)  # 归一化
                
                errors.append([lateral_error, heading_error])
            
            return np.array(errors)
        
        # 计算跟踪误差
        pp_errors = compute_tracking_error(pp_poses, ref_traj)
        mpc_errors = compute_tracking_error(mpc_poses, ref_traj)
        
        # 控制平滑性 (控制变化率)
        pp_smoothness = np.mean(np.diff(pp_controls, axis=0)**2)
        mpc_smoothness = np.mean(np.diff(mpc_controls, axis=0)**2)
        
        results = {
            'pure_pursuit': {
                'mean_lateral_error': np.mean(pp_errors[:, 0]),
                'max_lateral_error': np.max(pp_errors[:, 0]),
                'mean_heading_error': np.mean(pp_errors[:, 1]),
                'control_smoothness': pp_smoothness,
                'avg_compute_time': np.mean(pp_times),
                'max_compute_time': np.max(pp_times)
            },
            'mpc': {
                'mean_lateral_error': np.mean(mpc_errors[:, 0]),
                'max_lateral_error': np.max(mpc_errors[:, 0]),
                'mean_heading_error': np.mean(mpc_errors[:, 1]),
                'control_smoothness': mpc_smoothness,
                'avg_compute_time': np.mean(mpc_times),
                'max_compute_time': np.max(mpc_times)
            }
        }
        
        return results
    
    def _plot_comparison(self, ref_traj, pp_poses, mpc_poses, pp_controls, 
                        mpc_controls, results, traj_type):
        """绘制对比结果"""
        fig, axes = plt.subplots(2, 3, figsize=(12, 8))
        
        # 1. 轨迹跟踪对比
        ax = axes[0, 0]
        ax.plot(ref_traj[:, 0], ref_traj[:, 1], 'k--', linewidth=2, label='Reference')
        ax.plot(pp_poses[:, 0], pp_poses[:, 1], 'b-', linewidth=2, label='Pure Pursuit')
        ax.plot(mpc_poses[:, 0], mpc_poses[:, 1], 'r-', linewidth=2, label='MPC')
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_title(f'Trajectory Tracking ({traj_type})')
        ax.legend()
        ax.grid(True, alpha=0.3)
        ax.axis('equal')
        
        # 2. 速度控制对比
        ax = axes[0, 1]
        time_steps = np.arange(len(pp_controls)) * self.dt
        ax.plot(time_steps, pp_controls[:, 0], 'b-', label='PP Velocity')
        ax.plot(time_steps, mpc_controls[:, 0], 'r-', label='MPC Velocity')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Linear Velocity (m/s)')
        ax.set_title('Velocity Control')
        ax.legend()
        ax.grid(True, alpha=0.3)
        
        # 3. 角速度控制对比
        ax = axes[0, 2]
        ax.plot(time_steps, pp_controls[:, 1], 'b-', label='PP Angular Vel')
        ax.plot(time_steps, mpc_controls[:, 1], 'r-', label='MPC Angular Vel')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Angular Velocity (rad/s)')
        ax.set_title('Angular Velocity Control')
        ax.legend()
        ax.grid(True, alpha=0.3)
        
        # 4. 性能指标柱状图
        ax = axes[1, 0]
        metrics = ['Avg Lateral', 'Max Lateral', 'Avg Heading']
        pp_values = [results['pure_pursuit']['mean_lateral_error'],
                    results['pure_pursuit']['max_lateral_error'],
                    results['pure_pursuit']['mean_heading_error']]
        mpc_values = [results['mpc']['mean_lateral_error'],
                     results['mpc']['max_lateral_error'],
                     results['mpc']['mean_heading_error']]
        
        x = np.arange(len(metrics))
        width = 0.35
        ax.bar(x - width/2, pp_values, width, label='Pure Pursuit', color='blue', alpha=0.7)
        ax.bar(x + width/2, mpc_values, width, label='MPC', color='red', alpha=0.7)
        ax.set_xlabel('Performance Metrics')
        ax.set_ylabel('Error Value')
        ax.set_title('Tracking Accuracy')
        ax.set_xticks(x)
        ax.set_xticklabels(metrics)
        ax.legend()
        ax.grid(True, alpha=0.3)
        
        # 5. 计算时间对比
        ax = axes[1, 1]
        controllers = ['Pure Pursuit', 'MPC']
        avg_times = [results['pure_pursuit']['avg_compute_time']*1000,  # 转换为毫秒
                    results['mpc']['avg_compute_time']*1000]
        max_times = [results['pure_pursuit']['max_compute_time']*1000,
                    results['mpc']['max_compute_time']*1000]
        
        x = np.arange(len(controllers))
        ax.bar(x, avg_times, alpha=0.7, label='Avg Time')
        ax.bar(x, max_times, alpha=0.5, label='Max Time')
        ax.set_xlabel('Controller')
        ax.set_ylabel('Compute Time (ms)')
        ax.set_title('Computational Efficiency')
        ax.set_xticks(x)
        ax.set_xticklabels(controllers)
        ax.legend()
        ax.grid(True, alpha=0.3)
        
        # 6. 控制平滑性对比
        ax = axes[1, 2]
        smoothness_metrics = ['Control Smoothness']
        pp_smooth = [results['pure_pursuit']['control_smoothness']]
        mpc_smooth = [results['mpc']['control_smoothness']]
        
        x = np.arange(len(smoothness_metrics))
        ax.bar(x - width/2, pp_smooth, width, label='Pure Pursuit', color='blue', alpha=0.7)
        ax.bar(x + width/2, mpc_smooth, width, label='MPC', color='red', alpha=0.7)
        ax.set_xlabel('Metric')
        ax.set_ylabel('Smoothness (lower is better)')
        ax.set_title('Control Smoothness')
        ax.set_xticks(x)
        ax.set_xticklabels(smoothness_metrics)
        ax.legend()
        ax.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.show()
        
        # 打印性能总结
        self._print_performance_summary(results)
    
    def _print_performance_summary(self, results):
        """打印性能总结报告"""
        print("\n" + "="*80)
        print("🏆 Controller Performance Comparison Report")
        print("="*80)
        
        pp = results['pure_pursuit']
        mpc = results['mpc']
        
        print(f"📊 Tracking Accuracy:")
        print(f"  Pure Pursuit - Avg lateral error: {pp['mean_lateral_error']:.3f}m, Max: {pp['max_lateral_error']:.3f}m")
        print(f"  MPC         - Avg lateral error: {mpc['mean_lateral_error']:.3f}m, Max: {mpc['max_lateral_error']:.3f}m")
        
        print(f"\n📐 Heading Tracking:")
        print(f"  Pure Pursuit - Avg heading error: {pp['mean_heading_error']:.3f}rad ({np.degrees(pp['mean_heading_error']):.1f}°)")
        print(f"  MPC         - Avg heading error: {mpc['mean_heading_error']:.3f}rad ({np.degrees(mpc['mean_heading_error']):.1f}°)")
        
        print(f"\n⚡ Computational Efficiency:")
        print(f"  Pure Pursuit - Avg: {pp['avg_compute_time']*1000:.3f}ms, Max: {pp['max_compute_time']*1000:.3f}ms")
        print(f"  MPC         - Avg: {mpc['avg_compute_time']*1000:.3f}ms, Max: {mpc['max_compute_time']*1000:.3f}ms")
        
        print(f"\n🎯 Control Smoothness:")
        print(f"  Pure Pursuit - Smoothness index: {pp['control_smoothness']:.6f}")
        print(f"  MPC         - Smoothness index: {mpc['control_smoothness']:.6f}")
        
        # 综合评价
        print(f"\n🏅 Overall Assessment:")
        if pp['mean_lateral_error'] < mpc['mean_lateral_error']:
            print("  🎯 Tracking Accuracy: Pure Pursuit wins")
        else:
            print("  🎯 Tracking Accuracy: MPC wins")
            
        if pp['avg_compute_time'] < mpc['avg_compute_time']:
            print("  ⚡ Computational Efficiency: Pure Pursuit wins")
        else:
            print("  ⚡ Computational Efficiency: MPC wins")
            
        if pp['control_smoothness'] < mpc['control_smoothness']:
            print("  🎯 Control Smoothness: Pure Pursuit wins")
        else:
            print("  🎯 Control Smoothness: MPC wins")


if __name__ == "__main__":
    # 运行控制器对比测试
    comparator = ControllerComparison(dt=0.1)
    
    # 测试不同轨迹类型
    test_trajectories = ["straight", "circle", "s_curve", "figure_8"]
    
    for traj_type in test_trajectories:
        print(f"\n{'='*60}")
        print(f"🧪 Testing trajectory type: {traj_type}")
        print(f"{'='*60}")
        
        try:
            results = comparator.compare_controllers(traj_type)
        except Exception as e:
            print(f"❌ {traj_type} trajectory test failed: {str(e)}")
            continue
        
        input("\nPress Enter to continue to next test...")
