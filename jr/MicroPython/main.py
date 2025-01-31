# Runs all the IMU sensors, displaying values on the screen and
# printing them to the USB serial port.

from zumo_2040_robot import robot
import machine
from bno055 import *
import time
import math
from program import parse_program
import sys

rotations_per_mm = 10.45
angle_correction_factor = 0.02
rgb_leds = robot.RGBLEDs()
motors = robot.Motors()
encoders = robot.Encoders()
display = robot.Display()
i2c = machine.I2C(id=0, sda=machine.Pin(4), scl=machine.Pin(5), freq=400_000)
imu = BNO055(i2c)

def correct_angle(angle: float):
    if angle > 180.0:
        angle -= 360.0
    elif angle < -180.0:
        angle += 360.0
    return angle

def prints(s: str):
    display.fill(0)
    display.text(s, 0, 0)
    display.show()

def printa(s: any):
    display.fill(0)
    for i, line in enumerate(s):
        display.text(line, 0, i * 10)
    display.show()

def forward(distance: float, speed: int):
    # This is our heading.
    heading = imu.euler()[0]

    # Compute goal rotations.
    goal_rotations = distance * rotations_per_mm

    motors.set_speeds(speed, speed)
    while True:
        c = encoders.get_counts()


        if speed > 0 and (c[0] + c[1]) / 2.0 > goal_rotations:
            break
        elif speed < 0 and (c[0] + c[1]) / 2.0 < goal_rotations:
            break

        actual_heading = imu.euler()[0]
        drift = actual_heading - heading

        if drift > 180.0:
            drift -= 360.0
        elif drift < -180.0:
            drift += 360.0

        correction = drift * angle_correction_factor * speed
        
        if speed > 0:
            motors.set_speeds(speed - correction, speed + correction)
        else:
            motors.set_speeds(speed + correction, speed - correction)

        print(drift)

        display.fill(0)
        display.text("Left: "+str(c[0]), 0, 0)
        display.text("Right: "+str(c[1]), 0, 10)
        display.text("Drift: {:4.1f}".format(drift), 0, 20)
        display.show()

        time.sleep_ms(10)
    motors.set_speeds(0, 0)

def face(heading: float):
    prints("Face")
    angle = correct_angle(imu.euler()[0] - heading)
    initial_angle = angle

    prints("Angle: {:4.1f}".format(angle))

    if angle > 0:
        motors.set_speeds(-1000, 1000)
    else:
        motors.set_speeds(1000, -1000)

    while True:
        actual_heading = imu.euler()[0]
        angle = correct_angle(actual_heading - heading)

        printa([
            "act: {:4.1f}".format(actual_heading),
            "ang: {:4.1f}".format(angle),
            "ian: {:4.1f}".format(initial_angle),
            "hdn: {:4.1f}".format(heading),
        ])

        if angle * initial_angle < 0:
            # prints("Angle reached")
            break

        # prints("Angle: {:4.1f}".format(angle))

        time.sleep_ms(10)
    motors.set_speeds(0, 0)

def main():
    for i in range(6):
        rgb_leds.set_brightness(1, i)

    print("Starting...")

    # Make sure the LEDs are not too bright
    rgb_leds.set(0, [255, 0, 0])
    rgb_leds.show()

    # Parse the program file
    time_goal, commands = parse_program('prog_a.json')

    rgb_leds.set(0, [0, 255, 0])
    rgb_leds.show()

    display.fill(0)
    display.text("Ready", 0, 0)
    display.show()

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

    prints("Starting...")
    # heading = imu.euler()[0]
    # forward(-1000, -2000)
    face(90.0)
    time.sleep_ms(500)
    face(180.0)
    time.sleep_ms(500)
    face(270.0)
    time.sleep_ms(500)
    face(0.0)

if __name__ == "__main__":
    try: 
        main()
    except Exception as e:
        file = open("error.txt", "w")
        sys.print_exception(e, file)
        file.close()
        raise e
