from sensors.serial_receiver import SerialReceiver

class WheelEncoder:
    def __init__(self, receiver: SerialReceiver):
        self.receiver = receiver
        self.left_speed = 0.0
        self.right_speed = 0.0

    def update(self):
        data = self.receiver.get_data()
        wheel_data = data.get("wheel", {})
        self.left_speed = wheel_data.get("left", 0.0)
        self.right_speed = wheel_data.get("right", 0.0)