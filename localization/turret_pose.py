import math
from typing import Tuple

class TurretPose:
    def __init__(self, gyro_data: Tuple[float, float, float], magnetometer_data: Tuple[float, float, float]):
        """
        初始化云台姿态解算器。
        :param gyro_data: 陀螺仪数据 (roll_rate, pitch_rate, yaw_rate)
        :param magnetometer_data: 磁力计数据 (mag_x, mag_y, mag_z)
        """
        self.gyro_data = gyro_data
        self.magnetometer_data = magnetometer_data

    def calculate_turret_pose(self) -> Tuple[float, float]:
        """
        根据陀螺仪和磁力计数据解算云台姿态。
        :return: 云台俯仰角 (pitch) 和偏航角 (yaw)
        """
        roll_rate, pitch_rate, yaw_rate = self.gyro_data
        mag_x, mag_y, mag_z = self.magnetometer_data

        # 计算俯仰角 (pitch)
        pitch = math.atan2(-mag_y, math.sqrt(mag_x**2 + mag_z**2))

        # 计算偏航角 (yaw)
        yaw = math.atan2(mag_x, mag_z)

        return math.degrees(pitch), math.degrees(yaw)

# 示例用法
if __name__ == "__main__":
    gyro_data = (0.01, 0.02, 0.03)  # 示例陀螺仪数据
    magnetometer_data = (30, 40, 50)  # 示例磁力计数据

    turret_pose = TurretPose(gyro_data, magnetometer_data)
    pitch, yaw = turret_pose.calculate_turret_pose()
    print(f"Turret Pitch: {pitch} degrees, Turret Yaw: {yaw} degrees")
