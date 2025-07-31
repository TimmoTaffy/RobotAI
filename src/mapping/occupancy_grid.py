import numpy as np

def generate_occupancy_grid(point_cloud, grid_size, map_size):
    """
    占用栅格地图生成：将三维点云离散化为二维导航地图
    
    算法原理：
    
    【栅格化方法】
    • 将连续的三维空间离散化为二维栅格网格
    • 每个栅格代表一个正方形区域：边长 = grid_size
    • 坐标映射：世界坐标 (x,y) → 栅格坐标 (x//grid_size, y//grid_size)
    
    【占用判断策略】
    • 空闲栅格：grid[i,j] = 0，该区域内无激光点
    • 占用栅格：grid[i,j] = 1，该区域内有激光点击中
    • 简化假设：只要有点云击中就认为占用，不考虑高度信息
    
    【坐标系统】
    • 输入点云：车体坐标系或世界坐标系的 [x, y, z] 坐标
    • 输出栅格：二维数组索引 [row, col]，对应空间位置
    • 边界处理：超出地图范围的点被忽略
    
    应用场景：
    • 路径规划：A*等算法的输入地图
    • 碰撞检测：判断路径是否与障碍物冲突
    • 可视化显示：将环境信息以图像形式展示
    
    性能特点：
    • 时间复杂度：O(N)，N为点云数量
    • 空间复杂度：O(W×H/grid_size²)，W和H为地图宽高
    • 实时性：适合高频率地图更新
    
    参数调优：
    • grid_size较小：地图精度高，但内存占用大，计算量大
    • grid_size较大：内存节省，计算快，但细节丢失
    • 建议取值：室内0.05-0.1m，室外0.1-0.2m
    
    :param point_cloud: 非地面点云数据，N×3 数组 [x, y, z]
    :param grid_size: 栅格分辨率，单位米/格，决定地图精度
    :param map_size: 地图物理尺寸，[宽度, 高度]，单位米
    :return: 占用栅格地图，二维numpy数组
        - 数组形状：(map_size[0]//grid_size, map_size[1]//grid_size)
        - 数值含义：0=空闲可通行，1=占用有障碍
    
    使用示例：
    ```python
    # 生成0.1米分辨率的20×20米地图
    grid = generate_occupancy_grid(points, 0.1, [20, 20])
    # 结果：200×200的栅格数组，每格代表0.1×0.1米区域
    ```
    """
    # 计算栅格地图的行列数：物理尺寸除以栅格大小
    grid_rows = int(map_size[0] // grid_size)
    grid_cols = int(map_size[1] // grid_size)
    
    # 初始化占用栅格：全部设为0（空闲状态）
    grid = np.zeros((grid_rows, grid_cols))

    # 遍历每个点云点，进行栅格标记
    for point in point_cloud:
        # 坐标离散化：连续坐标转换为栅格索引
        x, y = int(point[0] // grid_size), int(point[1] // grid_size)
        
        # 边界检查：确保栅格索引在有效范围内
        if 0 <= x < grid.shape[0] and 0 <= y < grid.shape[1]:
            # 标记该栅格为占用状态
            grid[x, y] = 1

    return grid