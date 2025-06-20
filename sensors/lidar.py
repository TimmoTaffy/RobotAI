from sensors.serial_receiver import SerialReceiver

class LidarSensor:
    def __init__(self, receiver: SerialReceiver):
        self.receiver = receiver

    def get_point_cloud(self):
        data = self.receiver.get_data()
        return data.get("lidar", {}).get("point_cloud", [])