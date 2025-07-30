"""
统一跟踪器接口 - 推荐的跟踪器使用方式
"""
from src.tracking.enhanced_tracker import EnhancedTargetTracker, KalmanTarget

# 推荐使用的跟踪器配置
def create_production_tracker(dt: float = 0.1) -> EnhancedTargetTracker:
    """
    创建用于实战的跟踪器配置
    
    Args:
        dt: 时间步长，应与主控制循环一致
        
    Returns:
        配置好的增强型跟踪器
    """
    return EnhancedTargetTracker(
        dt=dt,
        max_age=1.0,                    # 目标最大存活时间
        association_threshold=3.0,       # 数据关联阈值
        process_noise=0.3,              # 过程噪声(运动不确定性)
        measurement_noise=0.8           # 测量噪声(传感器精度)
    )

def create_high_precision_tracker(dt: float = 0.1) -> EnhancedTargetTracker:
    """
    创建高精度跟踪器配置(用于精确瞄准)
    
    Args:
        dt: 时间步长
        
    Returns:
        高精度配置的跟踪器
    """
    return EnhancedTargetTracker(
        dt=dt,
        max_age=1.5,                    # 更长的目标保持时间
        association_threshold=2.0,       # 更严格的关联阈值
        process_noise=0.2,              # 更低的过程噪声
        measurement_noise=0.5           # 更精确的测量假设
    )

def create_fast_tracker(dt: float = 0.1) -> EnhancedTargetTracker:
    """
    创建快速跟踪器配置(用于高速场景)
    
    Args:
        dt: 时间步长
        
    Returns:
        快速响应配置的跟踪器
    """
    return EnhancedTargetTracker(
        dt=dt,
        max_age=0.8,                    # 更短的目标保持时间
        association_threshold=4.0,       # 更宽松的关联阈值
        process_noise=0.5,              # 更高的过程噪声适应快速运动
        measurement_noise=1.0           # 允许更大的测量不确定性
    )

__all__ = [
    'EnhancedTargetTracker',
    'KalmanTarget', 
    'create_production_tracker',
    'create_high_precision_tracker',
    'create_fast_tracker'
]
