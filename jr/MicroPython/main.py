# Runs all the IMU sensors, displaying values on the screen and
# printing them to the USB serial port.

from zumo_2040_robot import robot
import machine
from bno055 import *
import time
from program import parse_program

print("Starting...")

i2c = machine.I2C(id=0, sda=machine.Pin(4), scl=machine.Pin(5), freq=400_000)
imu = BNO055(i2c)

display = robot.Display()

# Parse the program file
time_goal, commands = parse_program('prog_a.json')
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
    display.show()
