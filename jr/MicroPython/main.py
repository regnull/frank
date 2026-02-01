from zumo_2040_robot import robot
import machine
from bno055 import *
import time
from program import Program
from vl53l4cd import VL53L4CD
from flog import Log
import math
log = Log("log/run")

speed_adjustment_enabled = True

rotations_per_mm = 10.45
overshoot_counts = 10  # Encoder counts to compensate for coasting after motor stop.
angle_correction_factor = 0.02
dowel_to_middle = 60
sensor_to_middle = 60
grid_size = 500
rgb_leds = robot.RGBLEDs()
motors = robot.Motors()
encoders = robot.Encoders()
display = robot.Display()
mm_per_sec = 51.4 # At 1000 speed.
width = 84 # Width of the robot, in millimeters.
arc_slip_factor = 0.90

i2c = machine.I2C(id=0, sda=machine.Pin(4), scl=machine.Pin(5), freq=400_000)

# IMU
imu = BNO055(i2c)
imu.mode(IMUPLUS_MODE)

# Distance sensor
vl53 = VL53L4CD(i2c)

north_heading = 0.0
heading = 0.0
speed = 2000
turn_speed = 3000
command_pause = 100
avg_turn_time = 2 
avg_adjust_time = 0.5
avg_stop_time = 0.1
min_speed = 1000
max_speed = 6000
board_width = 37
speed_correction_factor = 0.97

def compute_speed(time_goal: float, commands: list[str]):
    global speed_adjustment_enabled
    global speed
    
    if not speed_adjustment_enabled:
        return speed
    
    fixed_delay = 0.0
    distance = 0.0
    for cmd in commands:
        if cmd == 'TURN_LEFT' or cmd == 'TURN_RIGHT' or cmd == 'NORTH' or cmd == 'EAST' or cmd == 'SOUTH' or cmd == 'WEST':
            fixed_delay += avg_turn_time
        elif cmd == 'ADJUST':
            fixed_delay += avg_adjust_time
        elif cmd == 'GO_IN':
            distance += grid_size / 2 + dowel_to_middle
            fixed_delay += avg_stop_time
        elif cmd == 'FORWARD' or cmd == 'BACKWARD':
            distance += grid_size
            fixed_delay += avg_stop_time
        elif cmd == 'HALF_FORWARD':
            distance += grid_size / 2
            fixed_delay += avg_stop_time
        elif cmd == '2FORWARD' or cmd == '2BACKWARD':
            distance += grid_size * 2
            fixed_delay += avg_stop_time
        elif cmd == '3FORWARD' or cmd == '3BACKWARD':
            distance += grid_size * 3
            fixed_delay += avg_stop_time
        elif cmd == '4FORWARD' or cmd == '4BACKWARD':
            distance += grid_size * 4
            fixed_delay += avg_stop_time
        elif cmd == 'ARC_LEFT' or cmd == 'ARC_RIGHT':
            distance += grid_size / 2 * math.pi / 2.0
            fixed_delay += avg_stop_time
    fixed_delay += (len(commands) - 1) * command_pause / 1000.0
    travel_time = time_goal - fixed_delay
    if travel_time < 0.0:
        speed = max_speed
        return speed
    speed = distance * 1000.0 / travel_time / mm_per_sec
    
    if speed < min_speed:
        speed = min_speed
    elif speed > max_speed:
        speed = max_speed
    log.print(f"speed: {speed}, distance: {distance}, travel_time: {travel_time}, fixed_delay: {fixed_delay}")
    return speed * speed_correction_factor

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
    global heading
    
    log.print(f"move, distance: {distance}, speed: {speed}")
    
    # If the distance is small, go slowly.
    if math.fabs(distance) < 50.0:
        if speed > 0:
            speed = 1000
        else:
            speed = -1000
    
    # Make sure the speed and distance have the same sign.
    if distance < 0 and speed > 0:
        speed = -speed
    elif distance > 0 and speed < 0:
        speed = -speed
    
    # Compute goal rotations, compensating for overshoot.
    if distance > 0:
        goal_rotations = distance * rotations_per_mm - overshoot_counts
    else:
        goal_rotations = distance * rotations_per_mm + overshoot_counts
    log.print(f"goal_rotations: {goal_rotations}")
    zero_counts = encoders.get_counts()
    log.print(f"zero_counts: {zero_counts}")
    motors.set_speeds(speed, speed)
    speed_adjusted = False
    while True:
        c = encoders.get_counts()
        counts = (c[0] - zero_counts[0] + c[1] - zero_counts[1]) / 2.0
        if speed > 0 and counts > goal_rotations:
            motors.set_speeds(0, 0)
            log.print(f"speed: {speed}, counts: {counts}, goal_rotations: {goal_rotations}")
            break
        elif speed < 0 and counts < goal_rotations:
            motors.set_speeds(0, 0)
            log.print(f"speed: {speed}, counts: {counts}, goal_rotations: {goal_rotations}")
            break
        
        distance_left = (goal_rotations - counts) / rotations_per_mm
        # Go slower if we're close to the goal.
        if speed > 1000.0 and not speed_adjusted and math.fabs(distance_left) < 50.0:
            speed_adjusted = True
            speed = 1000
        if speed < -1000.0 and not speed_adjusted and math.fabs(distance_left) < 50.0:
            speed_adjusted = True
            speed = -1000

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
        display.text("lft: "+str(c[0] - zero_counts[0]), 0, 0)
        display.text("rgt: "+str(c[1] - zero_counts[1]), 0, 10)
        display.text("dft: {:4.1f}".format(drift), 0, 20)
        display.text("spd: {:4.1f}".format(speed), 0, 30)
        display.show()

        time.sleep_ms(10)
    # motors.set_speeds(0, 0)
    time.sleep_ms(100) # Allow it to settle.
    log.print("move finished, actual heading: {}".format(imu.euler()[0]))
    count_left = encoders.get_counts()[0] - zero_counts[0]
    count_right = encoders.get_counts()[1] - zero_counts[1]
    avg_count = (count_left + count_right) / 2.0
    log.print(f"count_left: {count_left}, count_right: {count_right}, avg_count: {avg_count}")
    log.print(f"distance: {avg_count / rotations_per_mm} mm")
    
def arc(radius: float, right_turn: bool, speed: int):
    # Arc can be done only with positive speed. We don't handle negative speeds here.
    
    global heading
    radius = radius * 0.825
    current_heading = heading
    
    if right_turn:
        target_heading = correct_angle(heading + 90.0)
    else:
        target_heading = correct_angle(heading - 90.0)
    
    log.print(f"arc, radius: {radius}, right_turn: {right_turn}, speed: {speed}")
    
    # Compute wheel distances for 90-degree arc.
    if right_turn:
        distance_left = (radius + width / 2) * math.pi / 2.0
        distance_right = (radius - width / 2) * math.pi / 2.0
    else:
        distance_left = (radius - width / 2) * math.pi / 2.0
        distance_right = (radius + width / 2) * math.pi / 2.0
    
    # Average distance for goal tracking (with slip factor compensation).
    distance_avg = (distance_left + distance_right) / 2.0 / arc_slip_factor
    # Arc is always forward, so subtract overshoot compensation.
    goal_rotations = distance_avg * rotations_per_mm - overshoot_counts
    log.print(f"goal_rotations: {goal_rotations}")
    zero_counts = encoders.get_counts()
    log.print(f"zero_counts: {zero_counts}")
        
    print(f"distance_left: {distance_left}, distance_right: {distance_right}")
        
    # Scale speeds proportionally to wheel distances.
    speed_left = speed * distance_left / distance_avg
    speed_right = speed * distance_right / distance_avg
    
    motors.set_speeds(speed_left, speed_right)
    speed_adjusted = False
    while True:
        c = encoders.get_counts()
        counts = (c[0] - zero_counts[0] + c[1] - zero_counts[1]) / 2.0
        
        distance_traveled = counts / rotations_per_mm
        
        log.print(f"distance_traveled: {distance_traveled}")
        
        angle_traveled = distance_traveled / radius * 180.0 / math.pi
        
        log.print(f"angle_traveled: {angle_traveled}")
        
        # Use current_heading (captured at start) instead of global heading.
        if right_turn:
            desired_heading = current_heading + angle_traveled
        else:  
            desired_heading = current_heading - angle_traveled
            
        desired_heading = correct_angle(desired_heading)
        
        log.print(f"desired_heading: {desired_heading}")
        
        # actual_heading = correct_angle(imu.euler()[0])
        
        # Check if we have finished the arc.
        # if right_turn:
        #     if actual_heading > target_heading:
        #         break
        # else:
        #     if actual_heading < target_heading:
        #         break
                
        if counts > goal_rotations:
            motors.set_speeds(0, 0)
            log.print(f"speed: {speed}, counts: {counts}, goal_rotations: {goal_rotations}")
            break
        
        distance_remaining = (goal_rotations - counts) / rotations_per_mm
        # Go slower if we're close to the goal.
        if not speed_adjusted and math.fabs(distance_remaining) < 50.0:
            speed_adjusted = True
            speed = 1000
            # Recalculate wheel speeds when base speed changes.
            speed_left = speed * distance_left / distance_avg
            speed_right = speed * distance_right / distance_avg

        actual_heading = correct_angle(imu.euler()[0])
        log.print(f"actual_heading: {actual_heading}")
        drift = actual_heading - desired_heading

        if drift > 180.0:
            drift -= 360.0
        elif drift < -180.0:
            drift += 360.0

        log.print(f"drift: {drift}")
        # Apply correction to individual wheel speeds, not base speed.
        correction = drift * angle_correction_factor * speed
        
        motors.set_speeds(speed_left - correction, speed_right + correction)

        print(drift)

        display.fill(0)
        display.text("dst: {:4.1f}".format(distance_traveled), 0, 0)
        display.text("ang: {:4.1f}".format(angle_traveled), 0, 10)
        display.text("hdg: {:4.1f}".format(current_heading), 0, 20)
        display.show()

        time.sleep_ms(10)
        
    if right_turn:
        heading = correct_angle(heading + 90.0)
    else:
        heading = correct_angle(heading - 90.0)
        
    # motors.set_speeds(0, 0)
    time.sleep_ms(100) # Allow it to settle.
    log.print("move finished, actual heading: {}".format(imu.euler()[0]))
    count_left = encoders.get_counts()[0] - zero_counts[0]
    count_right = encoders.get_counts()[1] - zero_counts[1]
    avg_count = (count_left + count_right) / 2.0
    log.print(f"count_left: {count_left}, count_right: {count_right}, avg_count: {avg_count}")
    log.print(f"distance: {avg_count / rotations_per_mm} mm")


def face(desired_heading: float, attempt: int = 1):
    global heading
    
    if attempt > 1:
        return
    
    start_time = time.ticks_ms()
    log.print("Face, desired_heading: {:4.1f}, attempt: {}".format(desired_heading, attempt))
    log.print(f"Actual heading: {imu.euler()}")
    angle = correct_angle(imu.euler()[0] - desired_heading)
    initial_angle = angle

    log.print("Angle: {:4.1f}".format(angle))

    if angle > 0:
        motors.set_speeds(-turn_speed, turn_speed)
    else:
        motors.set_speeds(turn_speed, -turn_speed)

    while True:
        # Hard limit on turn time.
        if time.ticks_ms() - start_time > 3000:
            break
        
        actual_heading = imu.euler()[0]
        angle = correct_angle(actual_heading - desired_heading)
        
        if math.fabs(angle) < 10.0:
            speed = 1000
        elif math.fabs(angle) < 20.0:
            speed = 2000
        elif math.fabs(angle) < 30.0:
            speed = 3000
        else:
            speed = turn_speed

        if initial_angle > 0:
            motors.set_speeds(-speed, speed)
        else:
            motors.set_speeds(speed, -speed)

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
    time.sleep_ms(100)
    actual_heading = correct_angle(imu.euler()[0])
    if math.fabs(actual_heading - desired_heading) > 1.0:
        face(desired_heading, attempt + 1)
    heading = desired_heading
    log.print(f"Turn finished, actual heading: {imu.euler()}")

def go_in():
    move(grid_size/2 + dowel_to_middle, speed)
    
def forward():
    move(grid_size, speed)
    
def half_forward():
    move(grid_size / 2, speed)

def forward2():
    move(grid_size * 2, speed)

def forward3():
    move(grid_size * 3, speed)

def forward4():
    move(grid_size * 4, speed)
    
def adjust():
    dist = vl53.get_distance() * 10.0
    desired_distance = grid_size / 2 - sensor_to_middle - board_width / 2
    delta = dist - desired_distance
    printa([
        "Adjusting", 
        "dist: {:4.1f}".format(dist), 
        'ddist: {:4.1f}'.format(desired_distance),
        'delta: {:4.1f}'.format(delta)])
    
    log.print(f"Distance: {dist} mm, delta: {delta} mm")
    move(delta, 1000)
    time.sleep_ms(100)

def backward():
    move(-grid_size, speed)

def backward2():
    move(-grid_size * 2, speed)

def backward3():
    move(-grid_size * 3, speed)

def backward4():
    move(-grid_size * 4, speed)
    
def turn_left():
    global heading
    heading -= 90.0
    heading = correct_angle(heading)
    face(heading)
    
def turn_right():
    global heading
    heading += 90.0
    heading = correct_angle(heading)
    face(heading)
    
def forward_to_target():
    move(grid_size - dowel_to_middle, speed)
    
def backward_to_target():
    move(-dowel_to_middle, -1000)
    
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

def arc_left():
    arc(grid_size / 2, False, speed)

def arc_right():
    arc(grid_size / 2, True, speed)

command_map = {
    'GO_IN': go_in,
    'FORWARD': forward,
    'HALF_FORWARD': half_forward,
    '2FORWARD': forward2,
    '3FORWARD': forward3,
    '4FORWARD': forward4,
    'ADJUST': adjust,
    'BACKWARD': backward,
    '2BACKWARD': backward2,
    '3BACKWARD': backward3,
    '4BACKWARD': backward4,
    'TURN_LEFT': turn_left,
    'TURN_RIGHT': turn_right,
    'FORWARD_TO_TARGET': forward_to_target,
    'BACKWARD_TO_TARGET': backward_to_target,
    'DELAY': delay,
    'NORTH': north,
    'EAST': east,
    'SOUTH': south,
    'WEST': west,
    'ARC_LEFT': arc_left,
    'ARC_RIGHT': arc_right,
    'STOP': stop,
}

def main():
    global heading
    global north_heading
    global speed
    
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
    vl53.timing_budget = 50 # spend 20ms on each measurement
    vl53.start_ranging()
    log.print("Distance sensor initialized")
    
    rgb_leds.set(0, [0, 255, 0])
    rgb_leds.show()

    log.print("Ready to start")
    
    # while True:
    #     dist = vl53.get_distance() * 10.0
    #     desired_distance = grid_size / 2 - sensor_to_middle - board_width / 2
    #     delta = dist - desired_distance
    #     printa([
    #         "Adjusting", 
    #         "dist: {:4.1f}".format(dist), 
    #         'ddist: {:4.1f}'.format(desired_distance),
    #         'delta: {:4.1f}'.format(delta)])
    #     time.sleep_ms(100)
    
    button_b = robot.ButtonB()

    # Wait for the user to start the program
    log.print("Waiting for button press...")
    speed = compute_speed(program.time_goal, program.commands)
    while not button_b.is_pressed():
        actual_heading = imu.euler()[0]
        dist = vl53.get_distance() * 10.0
        printa([
            "Ready to start", 
            "tgoal: {}".format(program.time_goal), 
            'hdng: {:4.1f}'.format(actual_heading),
            'dist: {:4.1f}'.format(dist),
            'spd: {:4.1f}'.format(speed),
            'Press B to start'])

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
    start_time = time.ticks_ms()
    while program.has_more_commands():
        cmd = program.next_command()
        log.print(f"cmd: {cmd}")
        elapsed_time = (time.ticks_ms() - start_time) / 1000.0 # In seconds.
        command_map[cmd]()
        speed = compute_speed(program.time_goal - elapsed_time, program.remaining_commands())
        log.print(f"new speed: {speed}")
        
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
