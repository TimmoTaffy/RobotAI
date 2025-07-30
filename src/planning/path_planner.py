"""
路径规划模块：基于 A* 算法在占据栅格地图上计算从起点到目标的可行路径。
"""
import heapq
import numpy as np
from typing import List, Tuple

class AStarPlanner:
    def __init__(self, grid: np.ndarray, grid_size: float):
        """
        :param grid: 二值占据栅格地图，1 表示障碍，0 表示自由
        :param grid_size: 栅格尺寸（米）
        """
        self.grid = grid
        self.grid_size = grid_size
        self.height, self.width = grid.shape
        # 8 邻域动作：dx, dy, cost
        self.moves = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
                      (-1, -1, np.sqrt(2)), (-1, 1, np.sqrt(2)), (1, -1, np.sqrt(2)), (1, 1, np.sqrt(2))]

    def heuristic(self, a: Tuple[int,int], b: Tuple[int,int]) -> float:
        # 欧氏距离启发式
        return np.hypot(b[0] - a[0], b[1] - a[1])

    def line_of_sight(self, a: Tuple[int,int], b: Tuple[int,int]) -> bool:
        """检查两个栅格点之间是否存在直视路径（无障碍）"""
        x0, y0 = a
        x1, y1 = b
        dx = x1 - x0
        dy = y1 - y0
        steps = max(abs(dx), abs(dy))
        if steps == 0:
            return True
        for i in range(1, steps):
            t = i / steps
            xi = int(round(x0 + dx * t))
            yi = int(round(y0 + dy * t))
            if xi < 0 or xi >= self.width or yi < 0 or yi >= self.height or self.grid[yi, xi] == 1:
                return False
        return True

    def plan(self, start: Tuple[float,float], goal: Tuple[float,float]) -> List[Tuple[float,float]]:
        """
        计算从 start 到 goal 的路径。
        :param start: 起点 (x, y)（世界坐标）
        :param goal: 目标点 (x, y)
        :return: 世界坐标下的路径点列表
        """
        # 转换到栅格索引
        sx, sy = int(start[0] / self.grid_size), int(start[1] / self.grid_size)
        gx, gy = int(goal[0] / self.grid_size), int(goal[1] / self.grid_size)

        # 检查起点和终点是否在障碍物上
        if self.grid[sy, sx] == 1 or self.grid[gy, gx] == 1:
            return []

        open_set = []
        heapq.heappush(open_set, (0 + self.heuristic((sx, sy), (gx, gy)), 0, (sx, sy)))
        came_from = {}
        cost_so_far = {(sx, sy): 0}
        closed_set = set()

        while open_set:
            _, cost, current = heapq.heappop(open_set)
            
            if current in closed_set:
                continue
            closed_set.add(current)
            
            if current == (gx, gy):
                break
                
            for dx, dy, move_cost in self.moves:
                nx, ny = current[0] + dx, current[1] + dy
                neighbor = (nx, ny)
                
                # 边界检查
                if not (0 <= nx < self.width and 0 <= ny < self.height):
                    continue
                    
                # 障碍物检查
                if self.grid[ny, nx] == 1:
                    continue
                    
                # 对角线移动时检查切角
                if dx != 0 and dy != 0:
                    # 检查相邻的两个格子是否有障碍
                    if (current[1] + dy < self.height and self.grid[current[1] + dy, current[0]] == 1) or \
                       (current[0] + dx < self.width and self.grid[current[1], current[0] + dx] == 1):
                        continue
                
                if neighbor in closed_set:
                    continue
                    
                new_cost = cost_so_far[current] + move_cost
                
                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    priority = new_cost + self.heuristic(neighbor, (gx, gy))
                    heapq.heappush(open_set, (priority, new_cost, neighbor))
                    came_from[neighbor] = current

        # 重建路径
        if (gx, gy) not in came_from and (gx, gy) != (sx, sy):
            return []  # 无法到达目标
            
        path = []
        node = (gx, gy)
        while node != (sx, sy):
            path.append((node[0] * self.grid_size, node[1] * self.grid_size))
            if node not in came_from:
                break
            node = came_from[node]
        path.append((start[0], start[1]))
        return list(reversed(path))
