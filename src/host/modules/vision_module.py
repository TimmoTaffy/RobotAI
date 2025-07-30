class VisionModule:
    def __init__(self):
        self.scanning = False
        self.tracking = False

    def enable_scanning(self):
        # 启动扫场检测
        self.scanning = True

    def disable_scanning(self):
        # 停止扫场检测
        self.scanning = False

    def start_tracking(self):
        # 启动目标跟踪
        self.tracking = True

    def disable_all(self):
        # 停止所有视觉功能
        self.scanning = False
        self.tracking = False

    def get_target_angles(self):
        # TODO: 调用检测算法或 SDK 获取目标云台角度
        return 0.0, 0.0
