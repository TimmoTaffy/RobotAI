import math
from typing import Tuple, Dict

class TurretPose:
    def __init__(self, gyro_data: Tuple[float, float, float], magnetometer_data: Tuple[float, float, float], motor_angles: Dict[str, float]):
        """
        初始化云台姿态解算器。
        :param gyro_data: 陀螺仪数据 (roll_rate, pitch_rate, yaw_rate)
        :param magnetometer_data: 磁力计数据 (mag_x, mag_y, mag_z)
        :param motor_angles: 电机角度数据 {"horizontal": float, "vertical": float}
        """
        self.gyro_data = gyro_data
        self.magnetometer_data = magnetometer_data
        self.motor_angles = motor_angles

    def calculate_turret_pose(self) -> Tuple[float, float, float, float]:
        """
        根据陀螺仪、磁力计和电机角度数据解算云台姿态。
        :return: 云台俯仰角 (pitch)、偏航角 (yaw)、水平电机角度和垂直电机角度
        """
        pitch, yaw = self._calculate_orientation()
        horizontal_angle = self.motor_angles.get("horizontal", 0.0)
        vertical_angle = self.motor_angles.get("vertical", 0.0)
        return pitch, yaw, horizontal_angle, vertical_angle

    def _calculate_orientation(self) -> Tuple[float, float]:
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
    motor_angles = {"horizontal": 10.0, "vertical": 20.0}  # 示例电机角度数据

    turret_pose = TurretPose(gyro_data, magnetometer_data, motor_angles)
    pitch, yaw, horizontal_angle, vertical_angle = turret_pose.calculate_turret_pose()
    print(f"Turret Pitch: {pitch} degrees, Turret Yaw: {yaw} degrees, Horizontal Motor Angle: {horizontal_angle} degrees, Vertical Motor Angle: {vertical_angle} degrees")
