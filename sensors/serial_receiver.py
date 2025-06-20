# 负责接收主控发来的 USB 串口 JSON 数据，并交给 sensors 模块中的各个子模块

import serial
import json
import threading
from queue import Queue

class SerialReceiver:
    def __init__(self, port="/dev/ttyUSB0", baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.ser = serial.Serial(port, baudrate, timeout=1)
        self.queue = Queue()
        self.thread = threading.Thread(target=self._read_loop)
        self.thread.daemon = True
        self.thread.start()

    def _read_loop(self):
        while True:
            try:
                line = self.ser.readline().decode().strip()
                data = json.loads(line)
                self.queue.put(data)
            except (json.JSONDecodeError, UnicodeDecodeError):
                continue

    def get_data(self):
        """获取最新的数据，如果没有就阻塞等一会"""
        return self.queue.get()