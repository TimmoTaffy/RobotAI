from sensors.serial_receiver import SerialReceiver

class Magnetometer:
    def __init__(self, receiver: SerialReceiver):
        self.receiver = receiver
        self.magnetic_field = [0.0, 0.0, 0.0]  # 磁场强度 (x, y, z)

    def update(self):
        data = self.receiver.get_data()
        mag_data = data.get("magnetometer", {})
        self.magnetic_field = mag_data.get("field", [0.0, 0.0, 0.0])