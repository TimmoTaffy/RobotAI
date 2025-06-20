from sensors.serial_receiver import SerialReceiver

class RadarStation:
    def __init__(self, receiver: SerialReceiver):
        self.receiver = receiver

    def get_radar_data(self):
        data = self.receiver.get_data()
        return data.get("radar_station", {})