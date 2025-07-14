# 负责接收主控发来的 USB 串口 JSON 数据，并交给 sensors 模块中的各个子模块

import serial
import json
import threading
import time
from queue import Queue
from typing import Dict, Any
import numpy as np
from src.common.types import SensorData

class SerialReceiver:
    """串口数据接收器
    
    负责：
    1. 接收主控发来的 USB 串口 JSON 数据
    2. 解析并添加时间戳
    3. 分发给各个传感器模块
    """
    def __init__(self, port="/dev/ttyUSB0", baudrate=115200):   # port: 串口端口, 默认值 /dev/ttyUSB0，可根据实际硬件调整
        self.port = port
        self.baudrate = baudrate
        self.ser = serial.Serial(port, baudrate, timeout=1)
        self.queue = Queue()                                    # 创建一个 Queue 对象，用于存储接收到的数据
        self.last_data: Dict[str, Any] = {}
        self.thread = threading.Thread(target=self._read_loop)  # 启动一个后台线程，运行 _read_loop 方法，持续接收数据
        self.thread.daemon = True
        self.thread.start()

    def _read_loop(self):
        while True:
            try:
                line = self.ser.readline().decode().strip()     # 持续从串口读取数据（self.ser.readline()）。将读取到的字节流解码为字符串
                data = json.loads(line)                         # 将字符串解析为 JSON 格式
                
                # 添加时间戳
                timestamp = time.time()
                data["timestamp"] = timestamp
                
                # 预处理数据：转换为numpy数组
                if "imu" in data:
                    imu = data["imu"]
                    if "gyro" in imu:
                        imu["gyro"] = np.array(imu["gyro"])
                    if "acc" in imu:
                        imu["acc"] = np.array(imu["acc"])
                
                if "turret" in data:
                    turret = data["turret"]
                    if "angles" in turret:
                        turret["angles"] = np.array(turret["angles"])
                    if "motor_angles" in turret:
                        turret["motor_angles"] = np.array(turret["motor_angles"])
                
                self.last_data = data
                self.queue.put(data)                            # 将解析后的数据存入队列（self.queue.put(data)）
            except (json.JSONDecodeError, UnicodeDecodeError) as e:
                print(f"Error decoding data: {e}")
                continue
            except Exception as e:
                print(f"Unexpected error in serial receiver: {e}")
                continue

    def get_data(self, timeout: float = 1.0) -> Dict[str, Any]:
        """获取最新的传感器数据
        
        Args:
            timeout: 等待新数据的超时时间（秒）
        
        Returns:
            Dict[str, Any]: 包含所有传感器数据的字典，带时间戳
        """
        try:
            return self.queue.get(timeout=timeout)
        except Queue.Empty:
            return self.last_data.copy()  # 如果超时，返回最后一次的数据
    
    def get_latest_data(self) -> Dict[str, Any]:
        """获取最新的数据，不等待
        
        Returns:
            Dict[str, Any]: 最后接收到的传感器数据
        """
        return self.last_data.copy()

    def close(self):
        """安全关闭串口"""
        if self.ser.is_open:
            self.ser.close()