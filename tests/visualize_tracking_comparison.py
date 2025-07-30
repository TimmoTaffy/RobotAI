"""
增强跟踪器配置对比可视化：展示标准配置 vs 高精度配置的效果
"""
import matplotlib.pyplot as plt
import numpy as np
import time
from typing import List, Tuple
from unittest.mock import Mock

from src.tracking.enhanced_tracker import EnhancedTargetTracker
from src.tracking import create_production_tracker, create_high_precision_tracker
from src.common.types import VisionRobot


class TrackingScenarioGenerator:
    """跟踪场景生成器"""
    
    def __init__(self, dt: float = 0.1):
        self.dt = dt
    
    def generate_moving_targets(self, scenario: str, duration: float = 10.0) -> List[List]:
        """生成移动目标检测序列"""
        steps = int(duration / self.dt)
        detection_sequence = []
        
        if scenario == "single_moving":
            # 单个目标直线运动
            for step in range(steps):
                t = step * self.dt
                detection = self._create_vision_robot(
                    x=2.0 + t * 0.5,  # 0.5 m/s
                    y=3.0 + t * 0.3,  # 0.3 m/s  
                    team='enemy',
                    conf=0.9 + 0.1 * np.sin(t)  # 置信度轻微波动
                )
                detection_sequence.append([detection])
                
        elif scenario == "multiple_crossing":
            # 多目标交叉运动
            for step in range(steps):
                t = step * self.dt
                detections = []
                
                # 目标1：从左到右
                if t < 8.0:  # 在视野内8秒
                    det1 = self._create_vision_robot(
                        x=1.0 + t * 0.6,
                        y=5.0,
                        team='enemy',
                        conf=0.85 + 0.15 * np.sin(t * 2)
                    )
                    detections.append(det1)
                
                # 目标2：从下到上
                if 2.0 < t < 9.0:
                    det2 = self._create_vision_robot(
                        x=6.0 + 0.2 * np.sin(t),  # 略微摆动
                        y=1.0 + (t - 2.0) * 0.8,
                        team='enemy', 
                        conf=0.8 + 0.2 * np.sin(t * 1.5)
                    )
                    detections.append(det2)
                
                # 目标3：盟友，相对静止
                if t > 1.0:
                    det3 = self._create_vision_robot(
                        x=8.0 + 0.1 * np.sin(t * 0.5),
                        y=8.0 + 0.1 * np.cos(t * 0.5),
                        team='ally',
                        conf=1.0
                    )
                    detections.append(det3)
                
                detection_sequence.append(detections)
        
        return detection_sequence
    
    def _create_vision_robot(self, x: float, y: float, team: str, conf: float) -> VisionRobot:
        """创建视觉检测机器人"""
        detection = VisionRobot(id=1, team=team, x=x, y=y)
        detection.confidence = conf
        return detection


class TrackingComparison:
    """跟踪器配置对比分析"""
    
    def __init__(self, dt: float = 0.1):
        self.dt = dt
        self.scenario_gen = TrackingScenarioGenerator(dt)
    
    def run_comparison(self, scenario: str = "multiple_crossing", duration: float = 10.0):
        """运行跟踪器对比测试"""
        print(f"🎯 Starting tracker configuration comparison test: {scenario}")
        
        # 生成测试场景
        detections_sequence = self.scenario_gen.generate_moving_targets(scenario, duration)
        
        # 创建不同配置的跟踪器
        standard_tracker = create_production_tracker(dt=self.dt)
        precision_tracker = create_high_precision_tracker(dt=self.dt)
        
        # 跟踪结果记录
        standard_results = []
        precision_results = []
        compute_times = {'standard': [], 'precision': []}
        
        # 模拟己方位置（用于威胁评估）
        my_position = (0.0, 0.0)
        
        print("  📊 Processing detections...")
        
        # 处理每一帧检测
        for step, detections in enumerate(detections_sequence):
            if not detections:
                continue
                
            # 转换为跟踪器所需格式
            vision_data = Mock()
            vision_data.robots = detections
            
            radar_data = Mock()
            radar_data.robots = []  # 这个场景只用视觉数据
            
            # 标准配置跟踪器
            start_time = time.time()
            standard_targets = standard_tracker.process_detections(
                radar_data, vision_data, my_position
            )
            standard_time = time.time() - start_time
            compute_times['standard'].append(standard_time)
            
            # 高精度配置跟踪器
            start_time = time.time()
            precision_targets = precision_tracker.process_detections(
                radar_data, vision_data, my_position
            )
            precision_time = time.time() - start_time
            compute_times['precision'].append(precision_time)
            
            # 记录结果
            standard_results.append({
                'time': step * self.dt,
                'targets': [(t.state[0], t.state[1], t.team, t.confidence, t.threat_level) 
                           for t in standard_targets],
                'count': len(standard_targets)
            })
            
            precision_results.append({
                'time': step * self.dt,
                'targets': [(t.state[0], t.state[1], t.team, t.confidence, t.threat_level) 
                           for t in precision_targets],
                'count': len(precision_targets)
            })
        
        # 性能分析
        performance_stats = self._analyze_performance(
            standard_results, precision_results, compute_times, detections_sequence
        )
        
        # 可视化结果
        self._plot_comparison(
            standard_results, precision_results, performance_stats, 
            scenario, detections_sequence
        )
        
        return performance_stats
    
    def _analyze_performance(self, standard_results, precision_results, 
                           compute_times, detections_sequence):
        """分析跟踪性能"""
        
        # 计算跟踪连续性
        def calculate_track_continuity(results):
            if not results:
                return 0
            track_lifetimes = {}
            for frame in results:
                active_ids = set()
                for target in frame['targets']:
                    target_id = f"{target[2]}_{int(target[0]//2)}_{int(target[1]//2)}"
                    active_ids.add(target_id)
                    
                    if target_id not in track_lifetimes:
                        track_lifetimes[target_id] = 0
                    track_lifetimes[target_id] += 1
            
            return np.mean(list(track_lifetimes.values())) if track_lifetimes else 0
        
        # 计算检测覆盖率
        def calculate_detection_coverage(results, detections):
            total_detections = sum(len(frame) for frame in detections)
            total_tracks = sum(r['count'] for r in results)
            return total_tracks / total_detections if total_detections > 0 else 0
        
        stats = {
            'standard_tracker': {
                'avg_targets': np.mean([r['count'] for r in standard_results]),
                'track_continuity': calculate_track_continuity(standard_results),
                'detection_coverage': calculate_detection_coverage(standard_results, detections_sequence),
                'avg_compute_time': np.mean(compute_times['standard']) * 1000,  # ms
                'max_compute_time': np.max(compute_times['standard']) * 1000
            },
            'precision_tracker': {
                'avg_targets': np.mean([r['count'] for r in precision_results]),
                'track_continuity': calculate_track_continuity(precision_results),
                'detection_coverage': calculate_detection_coverage(precision_results, detections_sequence),
                'avg_compute_time': np.mean(compute_times['precision']) * 1000,  # ms
                'max_compute_time': np.max(compute_times['precision']) * 1000
            }
        }
        
        return stats
    
    def _plot_comparison(self, standard_results, precision_results, stats, 
                        scenario, detections_sequence):
        """绘制对比结果"""
        fig, axes = plt.subplots(2, 3, figsize=(18, 12))
        
        # 1. 轨迹跟踪对比
        ax = axes[0, 0]
        
        # 绘制标准配置轨迹
        for frame in standard_results:
            for target in frame['targets']:
                color = 'red' if target[2] == 'enemy' else 'blue'
                ax.scatter(target[0], target[1], c=color, alpha=0.6, s=20, marker='o')
        
        # 绘制高精度配置轨迹
        for frame in precision_results:
            for target in frame['targets']:
                color = 'darkred' if target[2] == 'enemy' else 'darkblue'
                ax.scatter(target[0], target[1], c=color, alpha=0.8, s=30, marker='x')
        
        ax.set_title('Trajectory Tracking Comparison\\n(Circles: Standard, X-marks: High Precision)')
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.grid(True, alpha=0.3)
        ax.legend(['Standard-Enemy', 'Standard-Ally', 'Precision-Enemy', 'Precision-Ally'])
        
        # 2. 目标数量随时间变化
        ax = axes[0, 1]
        times = [r['time'] for r in standard_results]
        standard_counts = [r['count'] for r in standard_results]
        precision_counts = [r['count'] for r in precision_results]
        
        ax.plot(times, standard_counts, 'b-', label='Standard Config', linewidth=2)
        ax.plot(times, precision_counts, 'r--', label='High Precision Config', linewidth=2)
        ax.set_title('Tracked Target Count')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Target Count')
        ax.legend()
        ax.grid(True, alpha=0.3)
        
        # 3. 置信度分布
        ax = axes[0, 2]
        standard_confidences = []
        precision_confidences = []
        
        for frame in standard_results:
            for target in frame['targets']:
                standard_confidences.append(target[3])
                
        for frame in precision_results:
            for target in frame['targets']:
                precision_confidences.append(target[3])
        
        ax.hist(standard_confidences, bins=20, alpha=0.7, label='Standard Config', density=True)
        ax.hist(precision_confidences, bins=20, alpha=0.7, label='High Precision Config', density=True)
        ax.set_title('Target Confidence Distribution')
        ax.set_xlabel('Confidence')
        ax.set_ylabel('Density')
        ax.legend()
        ax.grid(True, alpha=0.3)
        
        # 4. 性能指标对比
        ax = axes[1, 0]
        metrics = ['Avg Targets', 'Track Continuity', 'Detection Coverage']
        standard_values = [
            stats['standard_tracker']['avg_targets'],
            stats['standard_tracker']['track_continuity'],
            stats['standard_tracker']['detection_coverage']
        ]
        precision_values = [
            stats['precision_tracker']['avg_targets'],
            stats['precision_tracker']['track_continuity'],
            stats['precision_tracker']['detection_coverage']
        ]
        
        x = np.arange(len(metrics))
        width = 0.35
        
        ax.bar(x - width/2, standard_values, width, label='Standard Config', color='skyblue')
        ax.bar(x + width/2, precision_values, width, label='High Precision Config', color='lightcoral')
        
        ax.set_title('Performance Metrics Comparison')
        ax.set_ylabel('Metric Value')
        ax.set_xticks(x)
        ax.set_xticklabels(metrics)
        ax.legend()
        ax.grid(True, alpha=0.3)
        
        # 5. 计算时间对比
        ax = axes[1, 1]
        time_metrics = ['Avg Time', 'Max Time']
        standard_times = [
            stats['standard_tracker']['avg_compute_time'],
            stats['standard_tracker']['max_compute_time']
        ]
        precision_times = [
            stats['precision_tracker']['avg_compute_time'],
            stats['precision_tracker']['max_compute_time']
        ]
        
        x = np.arange(len(time_metrics))
        ax.bar(x - width/2, standard_times, width, label='Standard Config', color='lightgreen')
        ax.bar(x + width/2, precision_times, width, label='High Precision Config', color='orange')
        
        ax.set_title('Computation Time Comparison')
        ax.set_ylabel('Time (ms)')
        ax.set_xticks(x)
        ax.set_xticklabels(time_metrics)
        ax.legend()
        ax.grid(True, alpha=0.3)
        
        # 6. 威胁等级热图
        ax = axes[1, 2]
        threat_grid = np.zeros((10, 10))
        
        for frame in precision_results:  # 使用高精度结果
            for target in frame['targets']:
                if target[2] == 'enemy':  # 只看敌方目标
                    grid_x = min(int(target[0]), 9)
                    grid_y = min(int(target[1]), 9)
                    if 0 <= grid_x < 10 and 0 <= grid_y < 10:
                        threat_grid[grid_y, grid_x] += target[4]  # 威胁等级
        
        im = ax.imshow(threat_grid, cmap='Reds', origin='lower')
        ax.set_title('Threat Level Heatmap (High Precision)')
        ax.set_xlabel('X Grid')
        ax.set_ylabel('Y Grid')
        plt.colorbar(im, ax=ax)
        
        plt.tight_layout()
        plt.savefig('tracking_configuration_comparison.png', dpi=300, bbox_inches='tight')
        print("  📈 Comparison chart saved: tracking_configuration_comparison.png")
        
        # 打印性能总结
        print("\\n📊 跟踪器配置对比结果:")
        print("=" * 60)
        
        for tracker_name, tracker_stats in stats.items():
            print(f"\\n{tracker_name.replace('_', ' ').title()}:")
            print(f"  平均目标数: {tracker_stats['avg_targets']:.1f}")
            print(f"  跟踪连续性: {tracker_stats['track_continuity']:.1f}")
            print(f"  检测覆盖率: {tracker_stats['detection_coverage']:.2%}")
            print(f"  平均计算时间: {tracker_stats['avg_compute_time']:.2f} ms")
            print(f"  最大计算时间: {tracker_stats['max_compute_time']:.2f} ms")


def main():
    """主函数"""
    print("🎯 增强跟踪器配置对比分析")
    print("=" * 50)
    
    comparison = TrackingComparison(dt=0.1)
    
    # 运行多目标交叉场景测试
    stats = comparison.run_comparison("multiple_crossing", duration=10.0)
    
    print("\\n✅ 分析完成！")
    print("📈 图表文件: tracking_configuration_comparison.png")
    print("🔍 这个分析展示了标准配置和高精度配置跟踪器的性能差异")

if __name__ == "__main__":
    main()
