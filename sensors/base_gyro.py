from sensors.serial_receiver import SerialReceiver

class BaseGyro:
    def __init__(self, receiver: SerialReceiver):
        self.receiver = receiver
        self.angular_velocity = [0.0, 0.0, 0.0]  # 底盘陀螺仪角速度 (x, y, z)

    def update(self):
        data = self.receiver.get_data()
        gyro_data = data.get("base_gyro", {})
        self.angular_velocity = gyro_data.get("angular_velocity", [0.0, 0.0, 0.0])