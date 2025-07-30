class TurretController:
    def __init__(self):
        self.yaw = 0.0
        self.pitch = 0.0

    def set_yaw(self, angle):
        # 设置云台偏航角度
        self.yaw = angle

    def set_pitch(self, angle):
        # 设置云台俯仰角度
        self.pitch = angle

    def stop(self):
        # 停止云台运动
        pass
