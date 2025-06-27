from sensors.serial_receiver import SerialReceiver

class RadarStation:
    def __init__(self, receiver: SerialReceiver):
        self.receiver = receiver
    # 如果需要在 RadarStation 类中处理或存储机器人信息，可以扩展数据成员。
    # 如果需要在 RadarStation 类中对数据进行预处理、存储或提供额外的功能，则可以考虑扩展。

    def get_radar_data(self):
        data = self.receiver.get_data()
        return data.get("radar_station", {})