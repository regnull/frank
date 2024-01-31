#include <math.h>
#include <stdio.h>
#include <ezButton.h>
#include <Wire.h>
#include <VL53L1X.h>
#include <Adafruit_LIS2MDL.h>
#include <Adafruit_LSM303_Accel.h>
#include <Adafruit_Sensor.h>
#include <SPI.h>
#include <SD.h>

#define MEASURE_DISTANCE_BEFORE_FORWARD
#undef MEASURE_DISTANCE_WHILE_FORWARD

const String version = "0.1.105";
const String log_message = "";

// Forward declarations

struct Distance;
Distance get_distance(bool blocking = true);
int get_distance_r(bool blocking = true);
int get_distance_l(bool blocking = true);

struct Distance {
  double left;
  double right;
};

struct Accel {
  bool valid;  // If this is true, the data in the other fields is valid.
  double x;
  double y;
  double z;
};

// Distance measurements & related

const int measure_distance_max_attempts = 5;    // Max attempts to measure distance before giving up.
const int forward_measurements          = 1;    // Number of distance measurements before moving forward.
const int adjust_angle_measurements     = 5;    // Number of distance measurements before adjusting the angle.
const int adjust_distance_measurements  = 5;    // Number of distance measurements before adjusting the distance.
const double max_accel_at_rest          = 0.2;  // Meters/second^2, below this means we are at rest.
const int adjust_distance_attempts      = 3;
const int adjust_angle_attempts         = 3;

// !!!!!!!!!!! Activate this if everything else fails.
const bool safe_mode = false;                 // Safe mode ignores all the sensors.

// Robot dimensions

const int dowel_to_middle         = 130;  // Distance between the dowel and the middle of the robot
const int sensors_base            = 83;   // Distance between the sensors, millimeters.
const int separator_width         = 36;   // Width of the separator, millimeters.
const int left_sensor_correction  = 0;
const int right_sensor_correction = 0;
const int left_wheel_correction   = 2;

// Motion

const int grid_distance = 500;             // Grid distance, in millimeters.
const int adjust_distance_horizon = 400;   // Don't adjust distance if farther than that 
const int adjust_angle_horizon    = 400;   // Don't adjust angle if farther than that 
const int distance_factor         = 240;   // !!! Adjust this to get the distance right
const int min_move_delay          = 200;   // Minimum move delay (after coming to a stop)
const int max_move_delay          = 2000;  // Maximum move delay
const int msec_per_move           = 1600;  // Approx. milliseconds per move.
const int angle                   = 90;    // Degrees
const int angle_factor            = 710;   // !!! Adjust this to get the turn angle right
const int shift_distance          = 500;   // Millimeters
const int shift_factor            = 600;   // !!! Adjust this to get the shift distance right
const int stop_distance           = 50;    // Stop if there is an obstacle at this distance

// LEDs

const int RED_LED_PIN    = 37;
const int YELLOW_LED_PIN = 41;
const int GREEN_LED_PIN  = 35;

// Switches

const int SWITCH_A_PIN = 39;
const int SWITCH_B_PIN = 43;

ezButton switchA(SWITCH_A_PIN);
ezButton switchB(SWITCH_B_PIN);

// Motors 

const int FR_MOTOR_SPEED_PIN = 9;   // Front Wheel PWM pin connect Model-Y M_B ENA 
const int FL_MOTOR_SPEED_PIN = 10;  // Front Wheel PWM pin connect Model-Y M_B ENB
const int FR_MOTOR_DIR_PIN_1 = 22;  // Front Right Motor direction pin 1 to Model-Y M_B IN1  (K1)
const int FR_MOTOR_DIR_PIN_2 = 24;  // Front Right Motor direction pin 2 to Model-Y M_B IN2   (K1)                                 
const int FL_MOTOR_DIR_PIN_1 = 26;  // Front Left Motor direction pin 1 to Model-Y M_B IN3 (K3)
const int FL_MOTOR_DIR_PIN_2 = 28;  // Front Left Motor direction pin 2 to Model-Y M_B IN4 (K3)

const int RR_MOTOR_SPEED_PIN = 11;  // Rear Wheel PWM pin connect Left Model-Y M_A ENA 
const int RL_MOTOR_SPEED_PIN = 12;  // Rear Wheel PWM pin connect Model-Y M_A ENB
const int RR_MOTOR_DIR_PIN_1 = 5;   // Rear Right Motor direction pin 1 to Model-Y M_A IN1 (K1)
const int RR_MOTOR_DIR_PIN_2 = 6;   // Rear Right Motor direction pin 2 to Model-Y M_A IN2 (K1) 
const int RL_MOTOR_DIR_PIN_1 = 7;   // Rear Left Motor direction pin 1 to Model-Y M_A IN3  (K3)
const int RL_MOTOR_DIR_PIN_2 = 8;   // Rear Left Motor direction pin 2 to Model-Y M_A IN4 (K3)

// Sensors

const int R_SENSOR_IRQ_PIN = 44;
const int R_SENSOR_XSHUT_PIN = 42;

const int L_SENSOR_IRQ_PIN = 38;
const int L_SENSOR_XSHUT_PIN = 40;

const uint32_t sensor_timeout = 500;         // Milliseconds
const uint32_t sensor_timing_budget = 50000; // Microseconds

VL53L1X vl53_l;
VL53L1X vl53_r;

// I2C multiplexor

const int TCAADDR = 0x70;

void tcaselect(uint8_t i) {
  if (i > 7) return;
 
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}

// Magnetometer

const int32_t MAG_SENSOR_ID = 1001;
Adafruit_LIS2MDL mag = Adafruit_LIS2MDL(MAG_SENSOR_ID);

// Accelerometer

const int32_t ACCEL_SENSOR_ID = 1002;
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(ACCEL_SENSOR_ID);

// SD Card

const int CS_PIN = 53;
File log_file;
bool log_available = false;

enum DIRECTION {
  NORTH,
  EAST,
  SOUTH,
  WEST
};

enum STATE {
  START,
  WAIT_FOR_READY,
  READY,
  IN_MOTION,
  FINISH
};

STATE state = START;

enum MOVE_STATE {
  GO_IN, GI,               // Start from the edge of the grid, go into the first square. Must be the first command!
  FORWARD, F,              // Go forward one square
  ADJUST, A,               // Adjust the position (assuming there is a board in front)
  BACKWARD, B,             // Go backward one square
  TURN_LEFT, L,            // Turn left 90 degrees
  TURN_RIGHT, R,           // Turn right 90 degrees
  LEFT_SHIFT, LS,          // Shift left one square
  RIGHT_SHIFT, RS,         // Shift right one square
  TEST_MOVE,               // Test only, do not use!
  FORWARD_TO_TARGET, FT,   // Go into the target square, stop with the dowel over the target. Must be the last command!
  BACKWARD_TO_TARGET, BT,  // Move back from the center of the square so that the dowel is in the center 
  DELAY, D,
  STOP, S
};

unsigned int speed = 100;
unsigned long move_delay = 500;  // Calculated from time_goal and moves.
unsigned long start_time;

const double DIR_NORTH = 10.0;
const double DIR_EAST = 126.0;
const double DIR_SOUTH = 175.0;
const double DIR_WEST = 223.0;

DIRECTION direction = NORTH;

// !!!!! Change this!
unsigned int time_goal = 60;

// !!!!!!! Robot moves

const MOVE_STATE moves_a[] = {
  GO_IN,
  A, R,
  F, A, L,
  F, A, R,
  F, A, L,
  F, A, L,
  F, A, R,
  F, R,
  F, R, A, L, A, BT,
  STOP,
};

const MOVE_STATE moves_b[] = {
  GO_IN,
  A, R,
  F, A, L,
  F, A, L,
  F, R,
  F, R, A, L,
  F, R, A, R,
  F,
  F, A, L,
  F,
  F, A, L,
  F, A, R,
  F, L, L,
  F, R, A, L,
  F, A, R,
  F, R,
  F, R, A, L, A, BT,
  STOP,
};

int current_move = 0;

MOVE_STATE *moves;

bool mag_available = false;
bool accel_available = false;
Accel zero_accel;  // Acceleration at rest

void setup() {
  start_time = millis();

  Serial.begin(115200);

  // Log file
  init_log();

  // Switches
  switchA.setDebounceTime(50);
  switchB.setDebounceTime(50);

  // Motors
  init_motors_GPIO();

  // LEDs
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(YELLOW_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);

  if(safe_mode) {
    if(log_available) {
      log_file.println("safe mode activated");
    }
    return;
  }

  // Sensors
  init_sensors();

  // Magnetometer
  tcaselect(0);
  if(!mag.begin()) {
    if(log_available) {
      log_file.println("Ooops, no LIS2MDL detected ... Check your wiring!");
    }
  }
  mag_available = true;

  // Accelerometer
  tcaselect(0);
  if (!accel.begin()) {
    if(log_available) {
      log_file.println("Ooops, no LSM303 detected ... Check your wiring!");
    }
  } else {
    accel.setRange(LSM303_RANGE_2G);
    accel.setMode(LSM303_MODE_HIGH_RESOLUTION);
  }
  accel_available = true;
}

void loop() {
  switch(state) {
    case START:
      start();
      break;
    case WAIT_FOR_READY:
      waitForReady();
      break;
    case READY:
      ready();
      break;
    case IN_MOTION:
      in_motion();
      break;
    case FINISH:
      finish();
      break;
    default:
      if(log_available) {
        log_file.println("Unknown state!");
      }
      delay(1000);
  }
}

void start() {
  if(log_available) {
    log_file.println("!! START");
  }

  // All LEDs off.
  led(false, false, false);

  switchA.loop();
  if(switchA.getState() == HIGH) {
    // Time to go!
    state = READY; 
  } else {
    state = WAIT_FOR_READY;
  }
}

void waitForReady() {
  // Wait for the ready switch to be on, turn on the green LED and wait forever.
  if(log_available) {
    log_file.println("!! WAIT_FOR_READY");
  }

  while(true) {
      switchA.loop();
      if(switchA.getState() == HIGH) {
        digitalWrite(GREEN_LED_PIN, HIGH);
        digitalWrite(YELLOW_LED_PIN, LOW);
        digitalWrite(RED_LED_PIN, LOW);
      } else {
        digitalWrite(GREEN_LED_PIN, LOW);
        digitalWrite(YELLOW_LED_PIN, HIGH);
        digitalWrite(RED_LED_PIN, LOW);
      }

      delay(100);
  }
}

void ready() {
  if(log_available) {
    log_file.println("!! READY");
  }

  led(false, true, false);
  // Warm up the sensors.
  Distance d = get_distance();
  delay(500);

  // Compute zero accel.
  zero_accel = get_accel();

  if(log_available) {
    log_file.print("zero accel, x: "); log_file.print(zero_accel.x); log_file.print(", y: ");
    log_file.print(zero_accel.y); log_file.print(", z: "); log_file.println(zero_accel.z);
  }

  if(switchB.getState() == LOW) {
    if(log_available) {
      log_file.println("using program A");
    }
    moves = moves_a;
  } else {
    if(log_available) {
      log_file.println("using program B");
    }
    moves = moves_b;
  }

  led(true, false, false);
  state = IN_MOTION;
}

void in_motion() {
  if(log_available) {
    log_file.println("!! IN_MOTION");
  }

  MOVE_STATE next_move = moves[current_move];

  compute_move_delay();

  switchA.loop();
  if(switchA.getState() == LOW) {
    if(log_available) {
      log_file.println("not ready, finish");
    }
    state = FINISH;
    return;
  }

  switch(next_move) {
    case GO_IN:
    case GI:
      if(log_available) {
        log_file.println(">> GO_IN");
      }
      move_into_grid(speed);
      break;
    case FORWARD:
    case F:
      if(log_available) {
        log_file.println(">> FORWARD");
      }
      move_forward(speed);
      break;
    case ADJUST:
    case A:
      if(log_available) {
        log_file.println(">> ADJUST");
      }
      adjust_distance();
      adjust_angle();
      break;
    case BACKWARD:
    case B:
      if(log_available) {
        log_file.println(">> BACKWARD");
      }
      move_backward(speed);
      break;
    case TURN_LEFT:
    case L:
      if(log_available) {
        log_file.println(">> TURN_LEFT");
      }
      move_turn_left(speed);
      break;
    case TURN_RIGHT:
    case R:
      if(log_available) {
        log_file.println(">> TURN_RIGHT");
      }
      move_turn_right(speed);
      break;
    case LEFT_SHIFT:
    case LS:
      if(log_available) {
        log_file.println(">> LEFT_SHIFT");
      }
      move_left_shift(speed);
      break;
    case RIGHT_SHIFT:
    case RS:
      if(log_available) {
        log_file.println(">> RIGHT_SHIFT");
      }
      move_right_shift(speed);
      break;
    case TEST_MOVE:
      if(log_available) {
        log_file.println(">> TEST_MOVE");
      }
      move_test(speed);
      break;
    case FORWARD_TO_TARGET:
    case FT:
      if(log_available) {
        log_file.println(">> FORWARD_TO_TARGET");
      }
      move_forward_to_target(speed);
      state = FINISH;
      break;
    case BACKWARD_TO_TARGET:
    case BT:
      if(log_available) {
        log_file.println(">> BACKWARD_TO_TARGET");
      }
      move_backward_to_target(speed);
      state = FINISH;
      break;
    case DELAY:
    case D:
      if(log_available) {
        log_file.println(">> DELAY");
      }
      delay(5000);
      break;
    case STOP:
    case S:
      if(log_available) {
        log_file.println(">> STOP");
      }
      state = FINISH;
      break;
  }
  current_move++;
  if(safe_mode && (next_move == ADJUST || next_move == A)) {
    // In safe mode, we don't wait after adjust, because it does nothing.
    return;
  }
  delay(move_delay);

}

void finish() {
  if(log_available) {
    log_file.println(">> FINISH");
  }

  led(true, true, true);

  if(log_available) {
    log_file.close();
  }
  while(true) {
    delay(1000);
  }
}

// Motor control

void init_motors_GPIO() {
  pinMode(FR_MOTOR_DIR_PIN_1, OUTPUT); 
  pinMode(FR_MOTOR_DIR_PIN_2, OUTPUT); 
  pinMode(FR_MOTOR_SPEED_PIN, OUTPUT);  
 
  pinMode(FL_MOTOR_DIR_PIN_1, OUTPUT);
  pinMode(FL_MOTOR_DIR_PIN_2, OUTPUT); 
  pinMode(FL_MOTOR_SPEED_PIN, OUTPUT);

  pinMode(RR_MOTOR_DIR_PIN_1, OUTPUT); 
  pinMode(RR_MOTOR_DIR_PIN_2, OUTPUT); 
  pinMode(RR_MOTOR_SPEED_PIN, OUTPUT);  
 
  pinMode(RL_MOTOR_DIR_PIN_1, OUTPUT); 
  pinMode(RL_MOTOR_DIR_PIN_2, OUTPUT); 
  pinMode(RL_MOTOR_SPEED_PIN, OUTPUT);  
   
  stop_motors();
}

void FR_fwd(int speed) {
  digitalWrite(FR_MOTOR_DIR_PIN_1, HIGH);
  digitalWrite(FR_MOTOR_DIR_PIN_2, LOW); 
  analogWrite(FR_MOTOR_SPEED_PIN, speed);
}

void FR_bck(int speed) {
  digitalWrite(FR_MOTOR_DIR_PIN_1, LOW);
  digitalWrite(FR_MOTOR_DIR_PIN_2, HIGH); 
  analogWrite(FR_MOTOR_SPEED_PIN, speed);
}

void FL_fwd(int speed) {
  digitalWrite(FL_MOTOR_DIR_PIN_1, HIGH);
  digitalWrite(FL_MOTOR_DIR_PIN_2, LOW);
  analogWrite(FL_MOTOR_SPEED_PIN, speed);
}

void FL_bck(int speed) {
  digitalWrite(FL_MOTOR_DIR_PIN_1, LOW);
  digitalWrite(FL_MOTOR_DIR_PIN_2, HIGH);
  analogWrite(FL_MOTOR_SPEED_PIN, speed);
}

void RR_fwd(int speed) {
  digitalWrite(RR_MOTOR_DIR_PIN_1, HIGH);
  digitalWrite(RR_MOTOR_DIR_PIN_2, LOW); 
  analogWrite(RR_MOTOR_SPEED_PIN, speed);
}

void RR_bck(int speed) {
  digitalWrite(RR_MOTOR_DIR_PIN_1, LOW);
  digitalWrite(RR_MOTOR_DIR_PIN_2, HIGH); 
  analogWrite(RR_MOTOR_SPEED_PIN, speed);
}

void RL_fwd(int speed) {
  digitalWrite(RL_MOTOR_DIR_PIN_1, HIGH);
  digitalWrite(RL_MOTOR_DIR_PIN_2, LOW);
  analogWrite(RL_MOTOR_SPEED_PIN, speed);
}

void RL_bck(int speed) {
  digitalWrite(RL_MOTOR_DIR_PIN_1, LOW);
  digitalWrite(RL_MOTOR_DIR_PIN_2, HIGH);
  analogWrite(RL_MOTOR_SPEED_PIN, speed);
}

void stop_motors() {
  analogWrite(FR_MOTOR_SPEED_PIN, 0);
  analogWrite(FL_MOTOR_SPEED_PIN, 0);
  analogWrite(RR_MOTOR_SPEED_PIN, 0);
  analogWrite(RL_MOTOR_SPEED_PIN, 0);
}

void go_forward(int speed) {
   RL_fwd(speed + left_wheel_correction);
   RR_fwd(speed);
   FR_fwd(speed);
   FL_fwd(speed + left_wheel_correction); 
}

void go_backward(int speed) {
  RL_bck(speed);
  RR_bck(speed);
  FR_bck(speed);
  FL_bck(speed);
}

void turn_left(int speed) {
  RL_bck(speed);
  RR_fwd(speed);
  FR_fwd(speed);
  FL_bck(speed); 
}

void turn_right(int speed) {
  RL_fwd(speed);
  RR_bck(speed);
  FR_bck(speed);
  FL_fwd(speed); 
}

void right_shift(int speed_fl_fwd, int speed_rl_bck, int speed_rr_fwd, int speed_fr_bck) {
  FL_fwd(speed_fl_fwd); 
  RL_bck(speed_rl_bck); 
  RR_fwd(speed_rr_fwd);
  FR_bck(speed_fr_bck);
}

void left_shift(int speed_fl_bck, int speed_rl_fwd, int speed_rr_bck, int speed_fr_fwd) {
   FL_bck(speed_fl_bck);
   RL_fwd(speed_rl_fwd);
   RR_bck(speed_rr_bck);
   FR_fwd(speed_fr_fwd);
}

void move_into_grid(int speed) {
  go_forward(speed);
  delay(compute_move_time(grid_distance - dowel_to_middle, distance_factor, speed));
  stop_motors();
  wait_for_stop();
}

void move_forward_to_target(int speed) {
  go_forward(speed);
  delay(compute_move_time(grid_distance - dowel_to_middle, distance_factor, speed));
  stop_motors();
  wait_for_stop();
}

void move_backward_to_target(int speed) {
  go_backward(speed);
  delay(compute_move_time(dowel_to_middle, distance_factor, speed));
  stop_motors();
  wait_for_stop();
}

void move_forward(int speed) {
  led(true, false, false);
  bool use_accel = false;

  double distance = grid_distance;
  
#ifdef MEASURE_DISTANCE_BEFORE_FORWARD
  // If we can get distance measurement, use it to make sure we don't bump into things.
  Distance d = get_average_distance(forward_measurements);
  print_distance(d);
  if(d.left > 0 && d.right > 0) {
    if(log_available) {
      log_file.print("distance left: "); log_file.print(d.left); log_file.print(", right "); log_file.println(d.right);
    }
    double dm = min(d.left, d.right) - stop_distance;
    if(dm < distance) {
      if(log_available) {
        log_file.print("obstacle detected at "); log_file.println(dm);
      }
      distance = dm;
    }
    if(distance < 0.0) {
      distance = 0.0;
    }
  }
#endif

  int time = compute_move_time(distance, distance_factor, speed);
  if(log_available) {
    log_file.print("move time "); log_file.println(time);
  }
  if(time <= 0) {
    return;
  }

  uint64_t start = millis();
  uint64_t last = start;
  double actual_distance = 0.0;
  double actual_speed = 0.0;
  double actual_accel = 0.0;
  go_forward(speed);
  if(!accel_available) {
    delay(time);
    stop_motors();
    return;
  }
  zero_accel = get_accel();
  while(true) {

#ifdef MEASURE_DISTANCE_WHILE_FORWARD
  Distance d = get_distance(false);
  if(d.left > 0 && d.right > 0) {
    if(log_available) {
      log_file.print("distance left: "); log_file.print(d.left); log_file.print(", right "); log_file.println(d.right);
    }
    if(min(d.left, d.right) < stop_distance) {
      if(log_available) {
        log_file.println("too close to an obstacle, stop!");
      }
      break;
    }
  }
#endif

    uint64_t now = millis();
    // If use acceleration is disabled, just go by time.
    if(now - start >= time) {
     break;
    }
    delay(10);
  }
  stop_motors();
  wait_for_stop();
}

void move_backward(int speed) {
  go_backward(speed);
  delay(compute_move_time(grid_distance, distance_factor, speed));
  stop_motors();
  wait_for_stop();
}

void move_turn_left(int speed) {
  led(false, true, false);
  turn_left(speed);
  delay(compute_move_time(angle, angle_factor, speed));
  stop_motors();
  wait_for_stop();
  switch(direction) {
    case NORTH:
      direction = WEST;
      break;
    case EAST:
      direction = NORTH;
      break;
    case SOUTH:
      direction = EAST;
      break;
    case WEST:
      direction = SOUTH;
      break;
  }
}

void move_turn_right(int speed) {
  led(false, false, true);
  turn_right(speed);
  delay(compute_move_time(angle, angle_factor, speed));
  stop_motors();
  wait_for_stop();
  switch(direction) {
    case NORTH:
      direction = EAST;
      break;
    case EAST:
      direction = SOUTH;
      break;
    case SOUTH:
      direction = WEST;
      break;
    case WEST:
      direction = NORTH;
      break;
  }
}

void move_right_shift(int speed) {
  right_shift(speed, speed, speed, speed);  
  delay(compute_move_time(shift_distance, shift_factor, speed));
  stop_motors();
  wait_for_stop();
}

void move_left_shift(int speed) {
  left_shift(speed, speed, speed, speed);  
  delay(compute_move_time(shift_distance, shift_factor, speed));
  stop_motors();
  wait_for_stop();
}

void move_test(int speed) {
  while(true) {
    Distance d = get_average_distance(3);
    print_distance(d);

    double heading = get_heading();

    Accel a = get_accel();
    delay(1000);
  }
  return;
}

long compute_move_time(int distance, int factor, int speed) {
  long move_time = long(distance) * long(factor) / long(speed);
  // If move time is too short, adjust it
  if(move_time < 0 && move_time > -100) {
    move_time = -100;
  }
  if(move_time > 0 && move_time < 100) {
    move_time = 100;
  }
  return move_time;
}

// Sensors control

bool init_sensor(int tca, VL53L1X& vl53) {
  tcaselect(tca);
  vl53.setTimeout(sensor_timeout);
  if (!vl53.init())
  {
    if(log_available) {
      log_file.println(F("Error on init of VL sensor"));
    }
    return false;
  }
  vl53.setDistanceMode(VL53L1X::Short);
  vl53.setROISize(4, 4);  // Focus on the area straight ahead
  vl53.setMeasurementTimingBudget(sensor_timing_budget);
  vl53.startContinuous(sensor_timing_budget/1000);
}

void init_sensors() {
  if(log_available) {
    log_file.println("init sensors");
  }

  Wire.begin();
  Wire.setClock(400000); // use 400 kHz I2C

  if(log_available) {
    log_file.println(F("Initializing left sensor..."));
  }
  if(!init_sensor(0, vl53_l)) {
    if(log_available) {
      log_file.println("failed to initialize left sensor");
    }
    state = FINISH;
    return;
  }

  if(log_available) {
    log_file.println(F("Initializing right sensor..."));
  }
  if(!init_sensor(1, vl53_r)) {
    if(log_available) {
      log_file.println("failed to initialize right sensor");
    }
    state = FINISH;
    return;
  }
}

Distance get_average_distance(int n) {
  if(safe_mode) {
     Distance d;
     d.left = 999.99;
     d.right = 999.99;
     return d;
  }
  double sum_r = 0.0;
  double sum_l = 0.0;
  int measurements = 0;
  int attempts = 0;
  Distance d;
  while(measurements < n) {
    attempts++;
    if(attempts > n * 3) {
      d.left = -1.0;
      d.right = -1.0;
      return d;        
    }
    Distance d1 = get_distance();
    if(d1.right < 0 || d1.left < 0) {
      delay(10);
      continue;
    }
    measurements++;
    sum_r += d1.right;
    sum_l += d1.left;
    delay(10);
  }
  d.left = sum_l / double(n);
  d.right = sum_r / double(n);
  return d;
}

Distance get_distance(bool blocking = true) {
  if(safe_mode) {
     Distance d;
     d.left = 999.99;
     d.right = 999.99;
     return d;
  }
  Distance d;
  int attempts = 0;
  while(true) {
    attempts++;
    if(attempts >= measure_distance_max_attempts) {
      d.left = -1.0;
      d.right = -1.0;
      return d;
    }
    double dr = get_distance_r();
    double dl = get_distance_l();
    if(dr <= 0 || dl <= 0) {
      delay(10);
      continue;
    }
    d.left = dl;
    d.right = dr;
    return d;
  }
}

int get_distance_sensor(int tca, VL53L1X& vl53, bool blocking = true) {
  if(safe_mode) {
    return 999.99;
  }
  tcaselect(tca);
  if(!blocking) {
    if(!vl53.dataReady()) {
      if(log_available) {
        log_file.println("distance data is not ready");
      }
      return 999.99;
    }
  }
  unsigned long start = millis();
  vl53.read(blocking);
  unsigned long duration = millis() - start;
  log_distance_measurement(vl53.ranging_data, duration);
  if(vl53.ranging_data.range_status != VL53L1X::RangeValid) {
    return 999.99;
  }
  return vl53.ranging_data.range_mm;
}

int get_distance_l(bool blocking = true) {
  if(safe_mode) {
    return 999.99;
  }
  if(log_available) {
    log_file.println("getting left distance");
  }
  int d = get_distance_sensor(0, vl53_l, blocking);
  if(d <= 0) {
    return d;
  }
  return d + left_sensor_correction;
}

int get_distance_r(bool blocking = true) {
  if(safe_mode) {
    return 999.99;
  }
  if(log_available) {
    log_file.println("getting right distance");
  }
  int d = get_distance_sensor(1, vl53_r, blocking);
  if(d <= 0) {
    return d;
  }
  return d + right_sensor_correction;
}

void adjust_angle() {
  if(safe_mode) {
    return;
  }

  led(true, false, true);
  if(log_available) {
    log_file.println("adjusting angle");
  }

  Distance d = get_average_distance(adjust_angle_measurements);
  if(d.left > adjust_angle_horizon || d.right > adjust_angle_horizon) {
    if(log_available) {
      log_file.println("Cannot adjust angle, too far");
    }
    return;
  }

  double distance_delta = d.right - d.left;
  double angle = compute_angle(distance_delta);
  int turn_time = angle * angle_factor / double(speed);

  if(log_available) {
    log_file.print("got distance left: "); log_file.print(d.left); log_file.print(", right: ");
    log_file.print(d.right); log_file.print(", angle: "); log_file.println(angle);
  }

  int attempts = 1;

  while(abs(angle) > 1.0) {
    if(attempts == adjust_angle_attempts) {
      break;
    }
    attempts++;
    if(turn_time > 0) {
      turn_left(speed);
    } else {
      turn_right(speed);
    }
    delay(abs(turn_time));
    stop_motors();
    d = get_average_distance(adjust_angle_measurements);
    distance_delta = d.right - d.left;
    angle = compute_angle(distance_delta);
    turn_time = angle * angle_factor / double(speed);
    if(log_available) {
      log_file.print("got distance left: "); log_file.print(d.left); log_file.print(", right: ");
      log_file.print(d.right); log_file.print(", angle: "); log_file.println(angle);
    }
  }
}

void adjust_distance() {
  if(safe_mode) {
    return;
  }

  led(true, true, false);
  if(log_available) {
    log_file.println("adjusting distance");
  }
  int target_distance = grid_distance / 2 - dowel_to_middle - separator_width / 2;
  if(log_available) {
    log_file.print("target distance: "); log_file.println(target_distance);
  }
  for(int i = 0; i < adjust_distance_attempts; i++) {
    Distance d = get_average_distance(adjust_distance_measurements);
    print_distance(d);
    if(d.right > adjust_distance_horizon || d.left > adjust_distance_horizon) {
      if(log_available) {
        log_file.println("Cannot adjust grid distance, too far");
      }
      return;
    }
    double min_distance = min(d.right, d.left);
    double distance = min_distance - target_distance;
    if(log_available) {
      log_file.print("distance to go: "); log_file.println(distance);
    }
    if(abs(distance - target_distance) < 5) {
      if(log_available) {
        log_file.println("reached target distance");
      }
      return;
    }
    long time = compute_move_time(distance, distance_factor, speed);
    if(time > 0) {
      go_forward(speed);
    } else {
      go_backward(speed);
    }
    delay(abs(time));
    stop_motors();
  }
}

double compute_angle(double distance_delta) {
  double angle_rad = atan2(distance_delta, sensors_base);
  return angle_rad / M_PI * 180.0;
}

void print_distance(const Distance& d) {
  if(!log_available) {
    return;
  }

  log_file.print("distance left: "); 
  log_file.print(d.left); 
  log_file.print(", right: "); 
  log_file.println(d.right);
}

double get_heading() {
  if(!mag_available || safe_mode) {
    return 0.0;
  }
  tcaselect(0);
  sensors_event_t event;
  mag.getEvent(&event);

  // Calculate the angle of the vector y,x
  double heading = (atan2(-event.magnetic.y,event.magnetic.x) * 180.0) / M_PI;

  // Normalize to 0-360
  if (heading < 0)
  {
    heading = 360 + heading;
  }
  return heading;
}

Accel get_accel() {
  if(!accel_available || safe_mode) {
    Accel a;
    a.valid = true;
    a.x = 0.0;
    a.y = 0.0;
    a.z = 0.0;
    return a;
  }
  tcaselect(0);
  Accel a;
  for(int i = 0; i < 5; i++) {
    sensors_event_t event;
    bool valid = accel.getEvent(&event);
    a.valid = valid;
    a.x = event.acceleration.x;
    a.y = event.acceleration.y;
    a.z = event.acceleration.z;
    if(a.valid) {
      break;
    }
    if(log_available) {
      log_file.println("failed to get accelaration, will try again");
    }
    delay(10);
  }
  return a;
}

void compute_move_delay() {
  int count = 0;
  MOVE_STATE* m = moves;
  m += current_move;
  while(*m != STOP && *m != S) {
    count++;
    m++;
  }
  double time_diff = double(time_goal) - double(millis() - start_time) / 1000.0 - double(msec_per_move) / 1000.0 * count;
  if(log_available) {
    log_file.print("got time diff: "); log_file.println(time_diff);
  }
  if(time_diff < 0) {
    move_delay = min_move_delay;
  } else {
    move_delay = int(time_diff / double(count) * 1000.0);
    if(move_delay > max_move_delay) {
      move_delay = max_move_delay;
    }
    if(move_delay < min_move_delay) {
      move_delay = min_move_delay;
    }
  }
  if(log_available) {
    log_file.print("got move delay: "); log_file.println(move_delay); 
  }
}

void wait_for_stop() {
  if(!accel_available || safe_mode) {
    return;
  }
  for(int i = 0; i < 20; i++) {
    Accel a = get_accel();
    a.x -= zero_accel.x;
    double b = abs(a.x);
    if(log_available) {
      log_file.print("got accel: "); log_file.print(b); log_file.print(", max: "); log_file.println(max_accel_at_rest);
    }
    if(b < max_accel_at_rest) {
      if(log_available) {
        log_file.println("at rest!");
      }
      break;
    }
    delay(50);
  }
}

void led(bool red, bool yellow, bool green) {
  digitalWrite(GREEN_LED_PIN, green ? HIGH : LOW);
  digitalWrite(YELLOW_LED_PIN, yellow? HIGH : LOW);
  digitalWrite(RED_LED_PIN, red ? HIGH : LOW);
}

void init_log() {
  if (!SD.begin(CS_PIN)) {
    Serial.println("SD card initialization failed!");
    log_available = false;
    return;
  }

  String file_name;
  bool found_file_name = false;
  for(int i = 0; i < 999; i++) {
    file_name = "log";
    file_name += i;
    file_name += ".txt";
    if(!SD.exists(file_name)) {
      found_file_name = true;
      break;
    }
  }
  if(!found_file_name) {
    log_available = false;
    Serial.println("cannot find log file name, log is disabled");
    return;
  }

  log_file = SD.open(file_name, FILE_WRITE);
  if(!log_file) {
    Serial.println("error opening log file");
    log_available = false;
    return;
  }
  Serial.println("opened log file");
  log_available = true;
  log_file.print("Frank OS, version "); 
  log_file.println(version);
  if(log_message != "") {
    log_file.println(log_message);
  }
}

void log_distance_measurement(const VL53L1X::RangingData& rd, unsigned long duration) {
  if(!log_available) {
    return;
  }
  String status;
  switch(rd.range_status) {
    case VL53L1X::RangeValid:
      status = "RangeValid";
      break;
    case VL53L1X::SigmaFail:
      status = "SigmaFail";
      break;
    case VL53L1X::SignalFail:
      status = "SignalFail";
        break;
    case VL53L1X::RangeValidMinRangeClipped:
      status = "RangeValidMinRangeClipped";
      break;
    case VL53L1X::OutOfBoundsFail:
      status = "OutOfBoundsFail";
      break;
    case VL53L1X::HardwareFail:
      status = "HardwareFail";
      break;
    case VL53L1X::RangeValidNoWrapCheckFail:
      status = "RangeValidNoWrapCheckFail";
      break;
    case VL53L1X::WrapTargetFail:
      status = "WrapTargetFail";
      break;
    case VL53L1X::XtalkSignalFail:
      status = "XtalkSignalFail";
      break;
    case VL53L1X::SynchronizationInt:
      status = "SynchronizationInt";
      break;
    case VL53L1X::MinRangeFail:
      status = "MinRangeFail";
      break;
    case VL53L1X::None:
      status = "None";
      break;
  }
  log_file.print("duration: ");
  log_file.print(duration);
  log_file.print(", status: ");
  log_file.print(status);
  log_file.print(", range: ");
  log_file.print(rd.range_mm);
  log_file.print(", peak signal rate: ");
  log_file.print(rd.peak_signal_count_rate_MCPS);
  log_file.print(", ambient_count: ");
  log_file.println(rd.ambient_count_rate_MCPS);
}