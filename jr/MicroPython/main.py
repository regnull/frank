# Runs all the IMU sensors, displaying values on the screen and
# printing them to the USB serial port.

from zumo_2040_robot import robot
from bno055 import *
import time
import machine

proximity_sensors = robot.ProximitySensors()

print("Starting...")

i2c = machine.I2C(id=0, sda=machine.Pin(4), scl=machine.Pin(5), freq=400_000)
imu = BNO055(i2c)

display = robot.Display()

calibrated = False
while True:
    time.sleep(1)
    display.fill(0)

    proximity_sensors.read()
    reading_front_left = proximity_sensors.front_counts_with_left_leds()
    reading_front_right = proximity_sensors.front_counts_with_right_leds()

    print(reading_front_left, reading_front_right)

    if not calibrated:
        calibrated = imu.calibrated()
        cal_status = imu.cal_status()
        display.text('sys {} {} {} {}'.format(
            *imu.cal_status()
        ), 0, 0)
    euler = imu.euler()
    display.text('hd {:4.0f}'.format(euler[0]), 0, 10)
    display.text('left {}'.format(reading_front_left), 0, 20)
    display.text('right {}'.format(reading_front_right), 0, 30)
    display.show()
