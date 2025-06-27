# 负责接收主控发来的 USB 串口 JSON 数据，并交给 sensors 模块中的各个子模块

import serial
import json
import threading
from queue import Queue

class SerialReceiver:
    def __init__(self, port="/dev/ttyUSB0", baudrate=115200):   # port: 串口端口, 默认值 /dev/ttyUSB0，可根据实际硬件调整
        self.port = port
        self.baudrate = baudrate
        self.ser = serial.Serial(port, baudrate, timeout=1)
        self.queue = Queue()                                    # 创建一个 Queue 对象，用于存储接收到的数据
        self.thread = threading.Thread(target=self._read_loop)  # 启动一个后台线程，运行 _read_loop 方法，持续接收数据
        self.thread.daemon = True
        self.thread.start()

    def _read_loop(self):
        while True:
            try:
                line = self.ser.readline().decode().strip()     # 持续从串口读取数据（self.ser.readline()）。将读取到的字节流解码为字符串
                data = json.loads(line)                         # 将字符串解析为 JSON 格式
                self.queue.put(data)                            # 将解析后的数据存入队列（self.queue.put(data)）
            except (json.JSONDecodeError, UnicodeDecodeError):
                continue

    def get_data(self):
        """获取最新的数据，如果没有就阻塞等一会"""
        return self.queue.get()     #############################################效率？