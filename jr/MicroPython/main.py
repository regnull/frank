# Runs all the IMU sensors, displaying values on the screen and
# printing them to the USB serial port.

from zumo_2040_robot import robot
from bno055 import *
import time
import machine

print("Starting...")

i2c = machine.I2C(id=0, sda=machine.Pin(4), scl=machine.Pin(5), freq=400_000)
imu = BNO055(i2c)

display = robot.Display()

calibrated = False
while True:
    time.sleep(1)
    display.fill(0)

    if not calibrated:
        calibrated = imu.calibrated()
        cal_status = imu.cal_status()
        display.text('sys {} {} {} {}'.format(
            *imu.cal_status()
        ), 0, 0)
    euler = imu.euler()
    display.text('hd {:4.0f}'.format(euler[0]), 0, 10)
    display.text('rl {:4.0f}'.format(euler[1]), 0, 20)
    display.text('pi {:4.0f}'.format(euler[2]), 0, 30)
    display.show()
