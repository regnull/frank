# Runs all the IMU sensors, displaying values on the screen and
# printing them to the USB serial port.

from zumo_2040_robot import robot
import machine
from bno055 import *
import time
import math
from program import parse_program

rotations_per_mm = 10.47
rgb_leds = robot.RGBLEDs()
motors = robot.Motors()
encoders = robot.Encoders()
display = robot.Display()
i2c = machine.I2C(id=0, sda=machine.Pin(4), scl=machine.Pin(5), freq=400_000)
imu = BNO055(i2c)


def forward(distance: float, speed: int):
    # This is our heading.
    heading = imu.euler()[0]

    # Compute goal rotations.
    goal_rotations = distance * rotations_per_mm

    while True:
        
        c = encoders.get_counts()
        if c[0] > goal_rotations and c[1] > goal_rotations:
            break

        display.fill_rect(0, 0, 128, 18, 0)
        display.text("Left: "+str(c[0]), 0, 0)
        display.text("Right: "+str(c[1]), 0, 10)
        display.text("Press B to reset", 0, 30)
        display.show()

        time.sleep_ms(10)

    motors.set_speeds(speed, speed)

def main():
    for i in range(6):
        rgb_leds.set_brightness(1, i)

    print("Starting...")

    # Make sure the LEDs are not too bright
    rgb_leds.set(0, [255, 0, 0])
    rgb_leds.show()

    # Parse the program file
    time_goal, commands = parse_program('prog_a.json')

    # Show the program name
    rgb_leds.set(0, [0, 255, 0])
    rgb_leds.show()

    button_c = robot.ButtonC()

    # Wait for the user to start the program
    while not button_c.is_pressed():
        time.sleep_ms(10)

    # Pause and show some fun lights
    start_time = time.ticks_us()
    while time.ticks_us() - start_time < 2000000:
        rgb_leds.set(1, [255, 0, 0])
        rgb_leds.show()
        time.sleep_ms(200)
        rgb_leds.set(1, [0, 0, 255])
        rgb_leds.show()
        time.sleep_ms(200)

    rgb_leds.set(1, [0, 0, 0])
    rgb_leds.show()

    # calibrated = False
    # while True:
    #     time.sleep(1)
    #     display.fill(0)

    #     if not calibrated:
    #         calibrated = imu.calibrated()
    #         cal_status = imu.cal_status()
    #         display.text('sys {} {} {} {}'.format(
    #             *imu.cal_status()
    #         ), 0, 0)
    #     euler = imu.euler()
    #     display.text('hd {:4.0f}'.format(euler[0]), 0, 10)
    #     display.show()

    heading = imu.euler()[0]

    forward(100, 1000)

if __name__ == "__main__":
    main()
