# Runs all the IMU sensors, displaying values on the screen and
# printing them to the USB serial port.

from zumo_2040_robot import robot
import machine
from bno055 import *
import time
from program import Program
import sys
from vl53l4cd import VL53L4CD

rotations_per_mm = 10.45
angle_correction_factor = 0.02
dowel_to_middle = 56
sensor_to_middle = 50
grid_size = 500
rgb_leds = robot.RGBLEDs()
motors = robot.Motors()
encoders = robot.Encoders()
display = robot.Display()
i2c = machine.I2C(id=0, sda=machine.Pin(4), scl=machine.Pin(5), freq=400_000)
imu = BNO055(i2c)

# Distance sensor
vl53 = VL53L4CD(i2c)

heading = 0.0
speed = 2000

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

def move(distance: float, speed: int):
    # Make sure the speed and distance have the same sign.
    if distance < 0 and speed > 0:
        speed = -speed
    elif distance > 0 and speed < 0:
        speed = -speed
    
    # This is our heading.
    heading = imu.euler()[0]

    # Compute goal rotations.
    goal_rotations = distance * rotations_per_mm
    zero_counts = encoders.get_counts()

    motors.set_speeds(speed, speed)
    while True:
        c = encoders.get_counts()
        
        counts = (c[0] - zero_counts[0] + c[1] - zero_counts[1]) / 2.0

        if speed > 0 and counts > goal_rotations:
            break
        elif speed < 0 and counts < goal_rotations:
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
        display.text("Left: "+str(c[0] - zero_counts[0]), 0, 0)
        display.text("Right: "+str(c[1] - zero_counts[1]), 0, 10)
        display.text("Drift: {:4.1f}".format(drift), 0, 20)
        display.show()

        time.sleep_ms(10)
    motors.set_speeds(0, 0)
    
def go_in():
    move(grid_size/2 - dowel_to_middle, speed)
    
def forward():
    move(grid_size, speed)
    
def adjust():
    print("Adjusting...")
    dist = vl53.get_distance() * 10
    print(f"Distance: {dist} cm")
    desired_distance = grid_size / 2 - sensor_to_middle
    delta = dist - desired_distance
    move(delta, 1000)

def backward():
    move(-grid_size, speed)
    
def turn_left():
    heading -= 90.0
    heading = correct_angle(heading)
    face(heading)
    
def turn_right():
    heading += 90.0
    heading = correct_angle(heading)
    face(heading)
    
def forward_to_target():
    move(grid_size - dowel_to_middle, speed)
    
def backward_to_target():
    move(-dowel_to_middle, -speed)
    
def delay():
    time.sleep_ms(1000)

def north():
    face(0.0)
    
def east():
    face(90.0)
    
def south():
    face(180.0)
    
def west():
    face(270.0)
    
def stop():
    # TODO: Implement this
    pass

command_map = {
    'GO_IN': go_in,
    'FORWARD': forward,
    'ADJUST': adjust,
    'BACKWARD': backward,
    'TURN_LEFT': turn_left,
    'TURN_RIGHT': turn_right,
    'FORWARD_TO_TARGET': forward_to_target,
    'BACKWARD_TO_TARGET': backward_to_target,
    'DELAY': delay,
    'NORTH': north,
    'EAST': east,
    'SOUTH': south,
    'WEST': west,
    'STOP': stop,
}

def face(desired_heading: float):
    global heading
    
    prints("Face")
    angle = correct_angle(imu.euler()[0] - desired_heading)
    initial_angle = angle

    prints("Angle: {:4.1f}".format(angle))

    if angle > 0:
        motors.set_speeds(-1000, 1000)
    else:
        motors.set_speeds(1000, -1000)

    while True:
        actual_heading = imu.euler()[0]
        angle = correct_angle(actual_heading - desired_heading)

        printa([
            "act: {:4.1f}".format(actual_heading),
            "ang: {:4.1f}".format(angle),
            "ian: {:4.1f}".format(initial_angle),
            "hdn: {:4.1f}".format(desired_heading),
        ])

        if angle * initial_angle < 0:
            # prints("Angle reached")
            break

        # prints("Angle: {:4.1f}".format(angle))

        time.sleep_ms(10)
    motors.set_speeds(0, 0)
    heading = desired_heading

def main():
    '''
    vl53.inter_measurement = 0 # makes sensor run in "continuous mode" (default)
    vl53.timing_budget = 20 # spend 20ms on each measurement

    print("VL53L4CD Simple Test.")
    print("--------------------")
    model_id, module_type = vl53.model_info
    print("Model ID: 0x{:0X}".format(model_id))
    print("Module Type: 0x{:0X}".format(module_type))
    print("Timing Budget: {}".format(vl53.timing_budget))
    print("Inter-Measurement: {}".format(vl53.inter_measurement))
    print("--------------------")

    vl53.start_ranging()

    while True:
        ### OLD
        # while not vl53.data_ready:
        #     pass
        # vl53.clear_interrupt()
        # print("Distance: {} cm".format(vl53.distance))

        ### NEW (convenience method):
        dist = vl53.get_distance()
        print(f"Distance: {dist} cm")
    return
    '''
    
    
    global heading

    for i in range(6):
        rgb_leds.set_brightness(1, i)

    # Make sure the LEDs are not too bright
    rgb_leds.set(0, [255, 0, 0])
    rgb_leds.show()

    # Parse the program file
    program = Program()
    program.load('program.frank')
    
    # Initialize the distance sensor
    vl53.inter_measurement = 0 # makes sensor run in "continuous mode" (default)
    vl53.timing_budget = 20 # spend 20ms on each measurement
    vl53.start_ranging()

    rgb_leds.set(0, [0, 255, 0])
    rgb_leds.show()

    display.fill(0)
    display.text("Ready", 0, 0)
    display.show()

    button_b = robot.ButtonB()

    # Wait for the user to start the program
    while not button_b.is_pressed():
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
    heading = imu.euler()[0]
    program.reset()
    while program.has_more_commands():
        cmd = program.next_command()
        prints("cmd: {}".format(cmd))
        time.sleep_ms(1000)
        command_map[cmd]()
        time.sleep_ms(1000)
        
    for i in range(6):
        rgb_leds.set(i, [255, 0, 0])
    rgb_leds.show()

if __name__ == "__main__":
    try: 
        main()
    except Exception as e:
        file = open("error.txt", "w")
        sys.print_exception(e, file)
        file.close()
        rgb_leds.set(1, [255, 0, 0])
        rgb_leds.show()

        raise e
