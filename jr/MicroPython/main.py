from zumo_2040_robot import robot
import machine
from bno055 import *
import time
from program import Program
from vl53l4cd import VL53L4CD
from flog import Log

log = Log("log/run")

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

north_heading = 0.0
heading = 0.0
speed = 1000

def correct_angle(angle: float):
    while angle > 180.0:
        angle -= 360.0
    while angle < -180.0:
        angle += 360.0
    return angle

def prints(s: str):
    ypos = 0
    display.fill(0)
    while len(s) > 16:
        display.text(s[:16], 0, ypos)
        s = s[16:]
        ypos += 10
    display.text(s, 0, ypos)
    display.show()

def printa(s: any):
    display.fill(0)
    for i, line in enumerate(s):
        display.text(line, 0, i * 10)
    display.show()

def move(distance: float, speed: int):
    log.print(f"move, distance: {distance}, speed: {speed}")
    # Make sure the speed and distance have the same sign.
    if distance < 0 and speed > 0:
        speed = -speed
    elif distance > 0 and speed < 0:
        speed = -speed
    
    # This is our heading.
    heading = imu.euler()[0]

    # Compute goal rotations.
    goal_rotations = distance * rotations_per_mm
    log.print(f"goal_rotations: {goal_rotations}")
    zero_counts = encoders.get_counts()
    log.print(f"zero_counts: {zero_counts}")
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
    log.print("move finished, actual heading: {}".format(imu.euler()[0]))
    count_left = encoders.get_counts()[0] - zero_counts[0]
    count_right = encoders.get_counts()[1] - zero_counts[1]
    log.print(f"count_left: {count_left}, count_right: {count_right}")

def face(desired_heading: float):
    global heading
    
    log.print("Face, desired_heading: {:4.1f}".format(desired_heading))
    log.print(f"Actual heading: {imu.euler()}")
    angle = correct_angle(imu.euler()[0] - desired_heading)
    initial_angle = angle

    log.print("Angle: {:4.1f}".format(angle))

    if angle > 0:
        motors.set_speeds(-speed, speed)
    else:
        motors.set_speeds(speed, -speed)

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
            break

        time.sleep_ms(10)
    motors.set_speeds(0, 0)
    heading = desired_heading
    log.print(f"Turn finished, actual heading: {imu.euler()}")

def go_in():
    move(grid_size/2 + dowel_to_middle, speed)
    
def forward():
    move(grid_size, speed)
    
def adjust():
    dist = vl53.get_distance() * 10
    desired_distance = grid_size / 2 - sensor_to_middle
    delta = dist - desired_distance
    log.print(f"Distance: {dist} mm, delta: {delta} mm")
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
    face(north_heading)
    
def east():
    face(north_heading + 90.0)
    
def south():
    face(north_heading + 180.0)
    
def west():
    face(north_heading + 270.0)
    
def stop():
    log.print("stop")

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

def main():
    global heading

    log.print("Starting...")

    for i in range(6):
        rgb_leds.set_brightness(1, i)

    # Make sure the LEDs are not too bright
    rgb_leds.set(0, [255, 0, 0])
    rgb_leds.show()

    # Parse the program file
    program = Program()
    program.load('program.frank')
    
    log.print("Program loaded")
    log.print(f"Commands: {program.commands}")
    
    # Initialize the distance sensor
    vl53.inter_measurement = 0 # makes sensor run in "continuous mode" (default)
    vl53.timing_budget = 20 # spend 20ms on each measurement
    vl53.start_ranging()
    log.print("Distance sensor initialized")
    
    rgb_leds.set(0, [0, 255, 0])
    rgb_leds.show()

    printa(["Ready to start", "tgoal: {}".format(program.time_goal), 'Press B to start'])
    log.print("Ready to start")
    
    button_b = robot.ButtonB()

    # Wait for the user to start the program
    log.print("Waiting for button press...")
    while not button_b.is_pressed():
        time.sleep_ms(10)
    log.print("Button pressed, starting the countdown")

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
    
    log.print("Time to go!")
    heading = imu.euler()[0]
    north_heading = heading
    program.reset()
    while program.has_more_commands():
        cmd = program.next_command()
        log.print(f"cmd: {cmd}")
        time.sleep_ms(1000)
        command_map[cmd]()
        time.sleep_ms(1000)
        
    log.print("Finished")
    for i in range(6):
        rgb_leds.set(i, [0, 255, 0])
    rgb_leds.show()
    log.close()

if __name__ == "__main__":
    try: 
        main()
    except Exception as e:
        log.print("Error")
        log.print_exception(e)
        prints(str(e))
        for i in range(10):
            for i in range(6):
                rgb_leds.set(i, [255, 0, 0])
                rgb_leds.show()
                time.sleep_ms(500)
            for i in range(6):
                rgb_leds.set(i, [0, 0, 0])
                rgb_leds.show()
                time.sleep_ms(500)

        raise e
