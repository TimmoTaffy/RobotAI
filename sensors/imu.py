from sensors.serial_receiver import SerialReceiver

class IMU:
    def __init__(self, receiver: SerialReceiver):
        self.receiver = receiver
        self.yaw = 0.0
        self.gyro = [0.0, 0.0, 0.0]
        self.acc = [0.0, 0.0, 0.0]

    def update(self):
        data = self.receiver.get_data()
        imu_data = data.get("imu", {})
        self.yaw = imu_data.get("yaw", 0.0)
        self.gyro = imu_data.get("gyro", [0.0, 0.0, 0.0])
        self.acc = imu_data.get("acc", [0.0, 0.0, 0.0])