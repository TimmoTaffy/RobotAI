"""
统一的路径规划可视化工具：支持基础A*和战术规划的对比展示
"""
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import LinearSegmentedColormap
from src.planning.path_planner import AStarPlanner
from src.planning.tactical_planner import TacticalPathPlanner
from src.tracking import KalmanTarget


class PlanningVisualizer:
    """路径规划可视化器"""
    
    def __init__(self, figsize=(12, 8)):
        self.figsize = figsize
        
    def visualize_comparison(self, grid, start, goal, targets=None):
        """对比展示基础A*和战术规划的结果"""
        fig, axes = plt.subplots(1, 2, figsize=self.figsize)
        
        # 基础A*规划
        basic_planner = AStarPlanner(grid, grid_size=1.0)
        basic_path = basic_planner.plan(start, goal)
        
        self._plot_basic_path(axes[0], grid, start, goal, basic_path)
        axes[0].set_title('Basic A* Path Planning', fontsize=14, fontweight='bold')
        
        # 战术规划
        if targets is None:
            targets = []
        
        tactical_planner = TacticalPathPlanner(grid, grid_size=1.0, weapon_range=6.0)
        tactical_path = tactical_planner.plan_tactical_path(start, goal, targets)
        
        self._plot_tactical_path(axes[1], grid, start, goal, tactical_path, targets)
        axes[1].set_title('Tactical Path Planning', fontsize=14, fontweight='bold')
        
        plt.tight_layout()
        return fig, (basic_path, tactical_path)
    
    def _plot_basic_path(self, ax, grid, start, goal, basic_path):
        """绘制基础A*路径"""
        # 绘制栅格地图背景
        ax.imshow(grid, cmap='binary', origin='lower', alpha=0.8)
        
        # 绘制基础路径
        if basic_path and len(basic_path) > 1:
            path_x = [p[0] for p in basic_path]
            path_y = [p[1] for p in basic_path]
            ax.plot(path_x, path_y, 'b-', linewidth=3, alpha=0.8, label='A* Path')
            ax.plot(path_x, path_y, 'bo', markersize=4, alpha=0.7)
        
        # 绘制起点和终点
        ax.plot(start[0], start[1], 'go', markersize=12, label='Start')
        ax.plot(goal[0], goal[1], 'ro', markersize=12, label='Goal')
        
        ax.set_xlabel('X (grid units)')
        ax.set_ylabel('Y (grid units)')
        ax.legend()
        ax.grid(True, alpha=0.3)
    
    def _plot_comparison(self, results, scenario):
        """绘制性能对比结果"""
        fig, axes = plt.subplots(2, 3, figsize=(12, 8))
    
    def _plot_tactical_path(self, ax, grid, start, goal, tactical_path, targets):
        """绘制战术路径"""
        # 创建威胁热力图
        threat_map = np.zeros_like(grid, dtype=float)
        safety_map = np.zeros_like(grid, dtype=float)
        
        if tactical_path and hasattr(tactical_path[0], 'threat_level'):
            planner = TacticalPathPlanner(grid, grid_size=1.0)
            
            for i in range(grid.shape[0]):
                for j in range(grid.shape[1]):
                    if grid[i, j] == 0:  # 只计算自由区域
                        pos = (j, i)  # 注意坐标转换
                        threat_map[i, j] = planner._evaluate_threat_level(pos, targets)
                        safety_map[i, j] = planner._calculate_safety_level(pos, targets)
        
        # 绘制安全/威胁背景
        combined_map = safety_map - threat_map  # 安全为正，威胁为负
        im = ax.imshow(combined_map, cmap='RdYlGn', origin='lower', alpha=0.6, vmin=-1, vmax=1)
        
        # 绘制障碍物
        obstacle_mask = grid == 1
        ax.imshow(obstacle_mask, cmap='gray', origin='lower', alpha=0.8)
        
        # 绘制敌方目标
        for target in targets:
            # 从state数组获取x, y位置
            x, y = target.state[0], target.state[1]
            
            if target.team == 'enemy':
                # 敌方目标
                ax.plot(x, y, 'r^', markersize=10, label='Enemy')
                # 武器射程圆
                circle = plt.Circle((x, y), 6.0, fill=False, 
                                  color='red', linestyle='--', alpha=0.7)
                ax.add_patch(circle)
            elif target.team == 'ally':
                # 友方目标
                ax.plot(x, y, 'b^', markersize=10, label='Ally')
        
        # 绘制战术路径
        if tactical_path and len(tactical_path) > 1:
            path_x = [wp.x for wp in tactical_path]
            path_y = [wp.y for wp in tactical_path]
            
            # 根据威胁等级着色路径
            threat_levels = [wp.threat_level if hasattr(wp, 'threat_level') else 0 
                           for wp in tactical_path]
            
            # 绘制路径段，颜色反映威胁等级
            for i in range(len(path_x) - 1):
                threat = threat_levels[i]
                color = 'green' if threat < 0.3 else 'orange' if threat < 0.7 else 'red'
                ax.plot([path_x[i], path_x[i+1]], [path_y[i], path_y[i+1]], 
                       color=color, linewidth=3, alpha=0.8)
            
            ax.plot(path_x, path_y, 'ko', markersize=3, alpha=0.7)
        
        # 绘制起点和终点
        ax.plot(start[0], start[1], 'go', markersize=12, label='Start')
        ax.plot(goal[0], goal[1], 'ro', markersize=12, label='Goal')
        
        # 添加颜色条
        cbar = plt.colorbar(im, ax=ax, shrink=0.8)
        cbar.set_label('Safety ← → Threat')
        
        ax.set_xlabel('X (grid units)')
        ax.set_ylabel('Y (grid units)')
        ax.legend()
        ax.grid(True, alpha=0.3)
    
    def create_demo_scenario(self):
        """创建演示场景"""
        # 创建复杂地图
        grid = np.zeros((20, 20), dtype=int)
        
        # 添加建筑物
        grid[6:10, 4:8] = 1    # 左侧建筑
        grid[12:16, 14:18] = 1 # 右侧建筑
        grid[3:6, 10:13] = 1   # 上方小建筑
        
        # 设置起终点
        start = (1.0, 1.0)
        goal = (18.0, 18.0)
        
        # 创建战术目标
        targets = [
            KalmanTarget(id=1, team='enemy', confidence=0.9, last_update=0.0, threat_level=0.8),
            KalmanTarget(id=2, team='enemy', confidence=0.7, last_update=0.0, threat_level=0.6),
            KalmanTarget(id=3, team='ally', confidence=1.0, last_update=0.0, threat_level=0.0)
        ]
        
        # 手动设置位置（因为KalmanTarget使用state数组）
        targets[0].state = np.array([10.0, 12.0, 0.0, 0.0])  # x, y, vx, vy
        targets[1].state = np.array([15.0, 6.0, 0.0, 0.0])
        targets[2].state = np.array([3.0, 15.0, 0.0, 0.0])
        
        return grid, start, goal, targets
    
    def generate_performance_report(self, grid, start, goal, targets):
        """生成性能对比报告"""
        import time
        
        # 基础A*测试
        basic_planner = AStarPlanner(grid, grid_size=1.0)
        start_time = time.time()
        basic_path = basic_planner.plan(start, goal)
        basic_time = time.time() - start_time
        
        # 战术规划测试
        tactical_planner = TacticalPathPlanner(grid, grid_size=1.0)
        start_time = time.time()
        tactical_path = tactical_planner.plan_tactical_path(start, goal, targets)
        tactical_time = time.time() - start_time
        
        # 计算路径指标
        basic_length = sum(np.sqrt((basic_path[i+1][0]-basic_path[i][0])**2 + 
                                 (basic_path[i+1][1]-basic_path[i][1])**2) 
                         for i in range(len(basic_path)-1)) if basic_path else 0
                         
        tactical_length = sum(np.sqrt((tactical_path[i+1].x-tactical_path[i].x)**2 + 
                                    (tactical_path[i+1].y-tactical_path[i].y)**2) 
                            for i in range(len(tactical_path)-1)) if tactical_path else 0
        
        avg_threat = sum(wp.threat_level for wp in tactical_path) / len(tactical_path) if tactical_path else 0
        avg_safety = sum(wp.safety_level for wp in tactical_path) / len(tactical_path) if tactical_path else 0
        
        report = {
            'basic_planning': {
                'time': basic_time,
                'path_length': basic_length,
                'waypoints': len(basic_path)
            },
            'tactical_planning': {
                'time': tactical_time,
                'path_length': tactical_length,
                'waypoints': len(tactical_path),
                'avg_threat': avg_threat,
                'avg_safety': avg_safety
            }
        }
        
        return report


def run_comprehensive_demo():
    """运行综合演示"""
    print("🤖 启动路径规划综合演示...")
    
    visualizer = PlanningVisualizer()
    grid, start, goal, targets = visualizer.create_demo_scenario()
    
    print("📊 生成对比可视化...")
    fig, (basic_path, tactical_path) = visualizer.visualize_comparison(
        grid, start, goal, targets)
    
    print("⚡ 生成性能报告...")
    report = visualizer.generate_performance_report(grid, start, goal, targets)
    
    print("\n" + "="*60)
    print("📈 性能对比报告")
    print("="*60)
    print(f"基础A*规划:")
    print(f"  ⏱️  计算时间: {report['basic_planning']['time']:.4f}s")
    print(f"  📏 路径长度: {report['basic_planning']['path_length']:.2f}")
    print(f"  📍 路径点数: {report['basic_planning']['waypoints']}")
    
    print(f"\n战术规划:")
    print(f"  ⏱️  计算时间: {report['tactical_planning']['time']:.4f}s")
    print(f"  📏 路径长度: {report['tactical_planning']['path_length']:.2f}")
    print(f"  📍 路径点数: {report['tactical_planning']['waypoints']}")
    print(f"  ⚠️  平均威胁: {report['tactical_planning']['avg_threat']:.3f}")
    print(f"  🛡️  平均安全: {report['tactical_planning']['avg_safety']:.3f}")
    
    # 计算改进指标
    if report['basic_planning']['path_length'] > 0:
        length_diff = ((report['tactical_planning']['path_length'] - 
                       report['basic_planning']['path_length']) / 
                      report['basic_planning']['path_length'] * 100)
        print(f"\n📊 路径长度变化: {length_diff:+.1f}%")
    
    time_ratio = report['tactical_planning']['time'] / report['basic_planning']['time']
    print(f"⏱️  计算时间比: {time_ratio:.1f}x")
    
    plt.show()
    
    return fig, report


if __name__ == "__main__":
    run_comprehensive_demo()
