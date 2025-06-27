from sensors.serial_receiver import SerialReceiver

class LidarSensor:
    def __init__(self, receiver: SerialReceiver):
        self.receiver = receiver

    def get_point_cloud(self):
        """
        获取点云数据，包括距离值、反射率等。
        :return: 点云数据和反射率。
        """
        data = self.receiver.get_data()
        point_cloud = data.get("lidar", {}).get("point_cloud", [])
        reflectivity = data.get("lidar", {}).get("reflectivity", [])
        return point_cloud, reflectivity