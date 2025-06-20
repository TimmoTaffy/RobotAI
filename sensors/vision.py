from sensors.serial_receiver import SerialReceiver

class VisionSensor:
    def __init__(self, receiver: SerialReceiver):
        self.receiver = receiver

    def get_vision_data(self):
        data = self.receiver.get_data()
        return data.get("vision", {}).get("robots", [])