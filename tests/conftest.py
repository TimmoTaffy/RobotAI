import pytest
import numpy as np
from src.planning.path_planner import AStarPlanner
from src.planning.tactical_planner import TacticalPathPlanner
from src.tracking.enhanced_tracker import KalmanTarget


def create_test_kalman_target(id, team, x, y, confidence=1.0, threat_level=0.0):
    """创建测试用的KalmanTarget"""
    target = KalmanTarget(id=id, team=team)
    target.state = np.array([x, y, 0.0, 0.0])  # [x, y, vx, vy]
    target.confidence = confidence
    target.threat_level = threat_level
    target.last_update = 0.0
    return target
from src.control.mpc_controller import MPCController
from src.control.trajectory_tracker import TrajectoryTracker
from src.common.types import Pose2D

@pytest.fixture
def grid():
    """Return an empty grid map fixture"""
    return np.zeros((20, 20), dtype=int)

@pytest.fixture
def planner(grid):
    """Return an AStarPlanner using the grid fixture"""
    return AStarPlanner(grid, grid_size=1.0)

@pytest.fixture
def mpc_controller():
    """Return a default MPCController instance"""
    return MPCController(horizon=10, dt=0.1)

@pytest.fixture
def trajectory_tracker():
    """Return a default TrajectoryTracker instance"""
    return TrajectoryTracker(lookahead_distance=1.0)

@pytest.fixture
def tactical_planner(grid):
    """Return a TacticalPathPlanner using the grid fixture"""
    return TacticalPathPlanner(grid, grid_size=1.0, weapon_range=6.0)

@pytest.fixture
def sample_targets():
    """Return sample tracked targets for testing"""
    return [
        create_test_kalman_target(id=1, team='enemy', x=10.0, y=12.0, confidence=0.9, threat_level=0.8),
        create_test_kalman_target(id=2, team='ally', x=5.0, y=15.0, confidence=1.0, threat_level=0.0)
    ]
    
# Pose2D fixture for default robot pose
@pytest.fixture
def pose2d():
    """Return a default Pose2D at origin"""
    return Pose2D(position=np.array([0.0, 0.0]), theta=0.0, timestamp=0.0)

# Reference trajectory generator
def make_reference_trajectory(horizon, dt, trajectory_type="straight", speed=1.0):
    """Create reference trajectory array"""
    ref_traj = np.zeros((horizon+1, 3))
    if trajectory_type == "straight":
        for i in range(horizon+1):
            ref_traj[i, 0] = speed * i * dt
            ref_traj[i, 1] = 0.0
            ref_traj[i, 2] = 0.0
    elif trajectory_type == "curve":
        radius = 5.0
        angular_speed = speed / radius
        for i in range(horizon+1):
            t = i * dt
            angle = angular_speed * t
            ref_traj[i, 0] = radius * np.sin(angle)
            ref_traj[i, 1] = radius * (1 - np.cos(angle))
            ref_traj[i, 2] = angle
    return ref_traj

@pytest.fixture
def ref_traj_generator():
    """Return reference trajectory generator function"""
    return make_reference_trajectory
