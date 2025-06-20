from sensors.serial_receiver import SerialReceiver
from sensors.imu import IMU
from sensors.wheel_encoder import WheelEncoder

if __name__ == "__main__":
    receiver = SerialReceiver("/dev/ttyUSB0")
    imu = IMU()
    wheel = WheelEncoder()

    while True:
        data = receiver.get_data()
        imu.update(data)
        wheel.update(data)

        print("[IMU] yaw: %.2f, gyro: %s, acc: %s" % (imu.yaw, imu.gyro, imu.acc))
        print("[Wheel] left: %.2f, right: %.2f" % (wheel.left_speed, wheel.right_speed))