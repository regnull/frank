#include <math.h>
#include <stdio.h>
#include <stream.h>
#include <Wire.h>
#include <VL53L1X.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <SPI.h>
#include <SD.h>
#include <ArduinoJson.h>

#define MEASURE_DISTANCE_BEFORE_FORWARD
#define MEASURE_DISTANCE_WHILE_FORWARD

const String version = "0.2.12";
const String log_message = "Gradarius Firmus Victoria";

// Forward declarations

struct Distance;
struct Accel;
Distance get_distance(bool blocking = true);
int get_distance_r(bool blocking = true);
int get_distance_l(bool blocking = true);
double get_heading();
double normalize_direction(double dir, double min_dir = 0.0, double max_dir = 360.0);
void init_motors_GPIO();
bool init_sensors();
void ready();
void in_motion();
void finish();
void fail();
bool switch_a();
bool switch_b();
void led(bool red, bool yellow, bool green);
bool init_sd();
void init_log();
bool read_program(const String& name);
Accel get_accel();
void compute_move_delay();

enum MOVE_STATE {
  GO_IN,               // Start from the edge of the grid, go into the first square. Must be the first command!
  FORWARD,             // Go forward one square
  ADJUST,              // Adjust the position (assuming there is a board in front)
  BACKWARD,            // Go backward one square
  TURN_LEFT,           // Turn left 90 degrees
  TURN_RIGHT,          // Turn right 90 degrees
  LEFT_SHIFT,          // Shift left one square
  RIGHT_SHIFT,         // Shift right one square
  TEST_MOVE,           // Test only, do not use!
  FORWARD_TO_TARGET,   // Go into the target square, stop with the dowel over the target. Must be the last command!
  BACKWARD_TO_TARGET,  // Move back from the center of the square so that the dowel is in the center 
  DELAY,               // Long delay, mostly for debugging
  NORTH,               // Face north, i.e. the go in direction
  EAST,
  SOUTH,
  WEST,
  INVALID,
  STOP
};

Stream* logger = &Serial;

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
const double max_accel_at_rest          = 0.1;  // Meters/second^2, below this means we are at rest.
const int adjust_distance_attempts      = 3;
const int adjust_angle_attempts         = 10;

// The last resort
bool safe_mode = false;                 // Safe mode ignores all the sensors.

// Robot dimensions

const int dowel_to_middle         = 130;  // Distance between the dowel and the middle of the robot
const int sensors_base            = 83;   // Distance between the sensors, millimeters.
const int separator_width         = 36;   // Width of the separator, millimeters.
const int left_sensor_correction  = 0;
const int right_sensor_correction = 0;
const int left_wheel_correction   = 2;

// Motion

const int grid_distance = 500;             // Grid distance, in millimeters.
const int adjust_distance_horizon  = 400;   // Don't adjust distance if farther than that 
const int adjust_angle_horizon     = 400;   // Don't adjust angle if farther than that 
//const int distance_factor          = 240;   // !!! Adjust this to get the distance right
const int distance_factor          = 245;   // !!! Adjust this to get the distance right
const int min_move_delay           = 0;   // Minimum move delay (after coming to a stop)
const int max_move_delay           = 2000;  // Maximum move delay
const int msec_per_move            = 1700;  // Approx. milliseconds per move.
const int angle                    = 90;    // Degrees
const int angle_factor             = 710;   // !!! Adjust this to get the turn angle right
const int shift_distance           = 500;   // Millimeters
const int shift_factor             = 600;   // !!! Adjust this to get the shift distance right
const int stop_distance            = 100;   // Stop if there is an obstacle at this distance
const int min_move_time            = 0; 
const int min_angle_adjust_time    = 40;

// LEDs

const int RED_LED_PIN    = 37;
const int YELLOW_LED_PIN = 41;
const int GREEN_LED_PIN  = 35;

// Switches

const int SWITCH_A_PIN = 39;
const int SWITCH_B_PIN = 43;

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

// Distance sensors

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

// IMU

double base_direction;
double direction;
double actual_direction;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// SD Card

const int CS_PIN = 53;
File log_file;
bool log_available = false;

enum STATE {
  START,
  WAIT_FOR_READY,
  READY,
  IN_MOTION,
  FINISH,
  FAIL,
};

STATE state = START;

unsigned int speed = 100;
unsigned long move_delay = 500;  // Calculated from time_goal and moves.
unsigned long start_time;

const double DIR_NORTH = 10.0;
const double DIR_EAST = 126.0;
const double DIR_SOUTH = 175.0;
const double DIR_WEST = 223.0;


// !!!!! Change this!
unsigned int time_goal = 80;

// !!!!!!! Robot moves

int current_move = 0;

int moves_count = 0;
MOVE_STATE moves[256];

bool mag_available = false;
bool accel_available = false;
Accel zero_accel;  // Acceleration at rest

void setup() {
  Serial.begin(115200);

  // Switches
  pinMode(SWITCH_A_PIN, INPUT);
  pinMode(SWITCH_B_PIN, INPUT);

  // LEDs
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(YELLOW_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);

  safe_mode = switch_b();
  if(safe_mode) {
    logger->println("*** SAFE MODE IS ACTIVE");
  }

  // Motors
  init_motors_GPIO();

  if(safe_mode) {
    return;
  }

  // Sensors
  if(!init_sensors()) {
    return;
  }

  // IMU

  tcaselect(1);
  logger->print("initializing IMU...");
  if (!bno.begin(OPERATION_MODE_IMUPLUS)) {
    logger->println("FAIL");
    state = FAIL;
    return;
  }
  logger->println("OK");
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
    case FAIL:
      fail();
      break;
    default:
      logger->println("Unknown state!");
      fail();
      break;
  }
}

void start() {
  logger->println("!! START");

  // All LEDs off.
  led(false, false, false);

  if(switch_a()) {
    // Time to go!
    state = READY; 
  } else {
    state = WAIT_FOR_READY;
  }
}

void waitForReady() {
  logger->println("!! WAIT_FOR_READY");

  if(safe_mode) {
    // Safe mode!
    while(true) {
      led(true, false, false);
      delay(500);
      led(false, false, true);
      delay(500);      

      if(switch_a()) {
        // It's go time
        led(true, false, false);
        state = READY;
        return;
      }
    }
  }

  tcaselect(1);
  uint8_t system;
  uint8_t gyro;
  uint8_t accel;
  uint8_t mag;
  while(true) {
    bno.getCalibration(&system, &gyro, &accel, &mag);

    if(bno.isFullyCalibrated()) {
      led(false, false, true);
    } else if(gyro) {
      led(false, true, false);
    } else {
      led(true, false, false);
    }

    if(switch_a()) {
      // It's go time
      led(true, false, false);
      state = READY;
      return;
    }
    delay(100);
  }
 }

void ready() {
  logger->println("!! READY");

  led(false, true, false);

  if(!init_sd()) {
    return;
  }

  init_log();

  if(!read_program("prog.jsn")) {
    return;
  }

  unsigned int start = millis();

  // Warm up the sensors.
  Distance d = get_distance();
  delay(500);


  // Compute zero accel.
  zero_accel = get_accel();

  logger->print("zero accel, x: "); logger->print(zero_accel.x); logger->print(", y: ");
  logger->print(zero_accel.y); logger->print(", z: "); logger->println(zero_accel.z);

  if(safe_mode) {
    base_direction = 0.0;
  } else {
    base_direction = get_heading();
  }
  direction = base_direction;
  actual_direction = direction;
  logger->print("got base direction: "); logger->println(base_direction);

  led(true, false, false);
  
  // Start time right before we are about to move
  start_time = millis();
  
  state = IN_MOTION;
}

void in_motion() {
  logger->print("!! IN_MOTION, move ");
  logger->println(current_move);

  unsigned int move_start = millis();

  if(!safe_mode) {
    actual_direction = get_heading();
  }
  logger->print("direction: "); logger->print(direction);
  logger->print(", heading: "); logger->println(actual_direction);

  MOVE_STATE next_move = moves[current_move];

  compute_move_delay();

  if(!switch_a()) {
    logger->println("not ready, finish");
    state = FINISH;
    return;
  }

  switch(next_move) {
    case GO_IN:
      logger->println(">> GO_IN");
      move_into_grid(speed);
      break;
    case FORWARD:
      logger->println(">> FORWARD");
      adjust_angle();
      move_forward(speed);
      break;
    case ADJUST:
      logger->println(">> ADJUST");
      adjust_distance();
      adjust_angle();
      break;
    case BACKWARD:
      logger->println(">> BACKWARD");
      move_backward(speed);
      break;
    case TURN_LEFT:
      logger->println(">> TURN_LEFT");
      led(false, true, false);
      direction = normalize_direction(direction - 90.0);
      adjust_angle();
      break;
    case TURN_RIGHT:
      logger->println(">> TURN_RIGHT");
      led(false, false, true);
      direction = normalize_direction(direction + 90.0);
      adjust_angle();
      break;
    case LEFT_SHIFT:
      logger->println(">> LEFT_SHIFT");
      move_left_shift(speed);
      break;
    case RIGHT_SHIFT:
      logger->println(">> RIGHT_SHIFT");
      move_right_shift(speed);
      break;
    case TEST_MOVE:
      logger->println(">> TEST_MOVE");
      move_test(speed);
      break;
    case FORWARD_TO_TARGET:
      logger->println(">> FORWARD_TO_TARGET");
      move_forward_to_target(speed);
      state = FINISH;
      break;
    case BACKWARD_TO_TARGET:
      logger->println(">> BACKWARD_TO_TARGET");
      move_backward_to_target(speed);
      state = FINISH;
      break;
    case DELAY:
      logger->println(">> DELAY");
      delay(5000);
      break;
    case STOP:
      logger->println(">> STOP");
      state = FINISH;
      break;
    case NORTH:
      logger->println(">> NORTH");
      direction = base_direction;
      adjust_angle();
      break;
    case EAST:
      logger->println(">> EAST");
      led(false, false, true);
      direction = normalize_direction(base_direction + 90.0);
      adjust_angle();
      break;
    case SOUTH:
      logger->println(">> SOUTH");
      led(false, false, true);
      direction = normalize_direction(base_direction + 180.0);
      adjust_angle();
      break;
    case WEST:
      logger->println(">> WEST");
      led(false, true, false);
      direction = normalize_direction(base_direction - 90.0);
      adjust_angle();
      break;
  }
  current_move++;
  if(safe_mode && next_move == ADJUST) {
    // In safe mode, we don't wait after adjust, because it does nothing.
    return;
  }
  logger->flush();
  delay(move_delay);

  unsigned int move_time = millis() - move_start;
  logger->print("move time: "); logger->println(move_time);
}

void finish() {
  logger->println(">> FINISH");

  unsigned long final_time = millis() - start_time;
  logger->print("final time: "); logger->println(final_time);

  led(true, true, true);

  current_move = 0;
  if(log_available) {
    log_file.flush();
  }

  while(true) {
    if(!switch_a()) {
      led(false, false, false);
      state = WAIT_FOR_READY;
      return;
    }
    delay(100);
  }
}

void fail() {
  // Once we enter the fail state, we stay here until a hard reset
  logger->println(">> FAIL");
  while(true) {
    led(true, true, true);
    delay(500);
    led(false, false, false);
    delay(500);
    if(!switch_a()) {
      led(false, false, false);
      state = WAIT_FOR_READY;
      return;
    }
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
  Distance d = get_distance(false);
  print_distance(d);
  if(d.left < 999 && d.right < 999) {
    double dm = min(d.left, d.right) - stop_distance;
    if(dm < distance) {
      logger->print("obstacle detected at "); logger->println(dm);
      distance = dm;
    }
    if(distance < 0.0) {
      distance = 0.0;
    }
  } else {
    int count = 0;
    while(d.left < 300 && d.right > 500) {
      logger->println("shifting right");
      if(count++ == 3) {
        break;
      }
      right_shift(speed, speed, speed, speed);  
      delay(compute_move_time(100, shift_factor, speed));
      stop_motors();
      wait_for_stop();
      adjust_angle();
      d = get_average_distance(forward_measurements);
    }

    count = 0;
    while(d.left > 500 && d.right < 300) {
      logger->println("shifting left");
      if(count++ == 3) {
        break;
      }
      left_shift(speed, speed, speed, speed);  
      delay(compute_move_time(100, shift_factor, speed));
      stop_motors();
      wait_for_stop();
      adjust_angle();
      d = get_average_distance(forward_measurements);
    }
  }
#endif

  int time = compute_move_time(distance, distance_factor, speed);
  logger->print("move forward time: "); logger->println(time);
  if(time <= 0) {
    return;
  }

  uint64_t start = millis();
  uint64_t last = start;
  go_forward(speed);
  if(!accel_available || safe_mode) {
    delay(time);
    stop_motors();
    wait_for_stop();
    return;
  }
  zero_accel = get_accel();
  
  while(true) {

#ifdef MEASURE_DISTANCE_WHILE_FORWARD
  Distance d = get_distance(false);
  if(d.left <999 && d.right < 999) {
    logger->print("distance left: "); logger->print(d.left); logger->print(", right "); logger->println(d.right);
    if(min(d.left, d.right) < stop_distance) {
      logger->println("too close to an obstacle, stop!");
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
  return min_time(move_time, min_move_time);
}

// Sensors control

bool init_sensor(int tca, VL53L1X& vl53) {
  tcaselect(tca);
  vl53.setTimeout(sensor_timeout);
  if (!vl53.init()) {
    logger->println(F("Error on init of VL sensor"));
    return false;
  }
  vl53.setDistanceMode(VL53L1X::Short);
  vl53.setROISize(6, 4);  // Focus on the area straight ahead
  vl53.setMeasurementTimingBudget(sensor_timing_budget);
  vl53.startContinuous(sensor_timing_budget/1000);
}

bool init_sensors() {
  logger->println("init sensors");

  Wire.begin();
  Wire.setClock(400000); // use 400 kHz I2C

  logger->print(F("Initializing left sensor..."));
  if(!init_sensor(0, vl53_l)) {
    logger->println("FAIL");
    state = FAIL;
    return false;
  }
  logger->println(F("OK"));

  logger->print(F("Initializing right sensor..."));
  if(!init_sensor(1, vl53_r)) {
    logger->println("FAIL");
    state = FAIL;
    return false;
  }
  logger->println(F("OK"));

  return true;
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
     d.left = 9999;
     d.right = 9999;
     return d;
  }
  Distance d;
  double dr = get_distance_r(blocking);
  double dl = get_distance_l(blocking);
  d.left = dl;
  d.right = dr;
  return d;
}

int get_distance_sensor(int tca, VL53L1X& vl53, bool blocking = true) {
  if(safe_mode) {
    return 9999;
  }
  tcaselect(tca);
  if(!blocking) {
    if(!vl53.dataReady()) {
      logger->println("distance data is not ready");
      return 9999;
    }
  }
  unsigned long start = millis();
  vl53.read(blocking);
  unsigned long duration = millis() - start;
  log_distance_measurement(vl53.ranging_data, duration);
  if(vl53.ranging_data.range_status != VL53L1X::RangeValid) {
    return 9999;
  }
  return vl53.ranging_data.range_mm;
}

int get_distance_l(bool blocking = true) {
  if(safe_mode) {
    return 9999;
  }
  logger->println("getting left distance");
  int d = get_distance_sensor(0, vl53_l, blocking);
  if(d <= 0) {
    return d;
  }
  return d + left_sensor_correction;
}

int get_distance_r(bool blocking = true) {
  if(safe_mode) {
    return 9999;
  }
  logger->println("getting right distance");
  int d = get_distance_sensor(1, vl53_r, blocking);
  if(d <= 0) {
    return d;
  }
  return d + right_sensor_correction;
}

void adjust_angle() {
  // if(safe_mode) {
  //   return;
  // }

  led(true, false, true);
  logger->println("adjusting angle");

  if(!safe_mode) {
    actual_direction = get_heading();
  }
  double angle = normalize_turn_angle(direction - actual_direction);
  int turn_time = angle * angle_factor / double(speed);
  turn_time = min_time(turn_time, min_angle_adjust_time);
  int attempts = 1;
  logger->print("angle: "); logger->print(angle);
  logger->print(", turn time: "); logger->println(turn_time);
  bool reached_goal = true;
  while(abs(angle) > 1.0) {
    if(attempts == adjust_angle_attempts) {
      reached_goal = false;
      break;
    }
    attempts++;
    if(turn_time > 0) {
      turn_right(speed);
    } else {
      turn_left(speed);
    }
    delay(abs(turn_time));
    stop_motors();
    if(safe_mode) {
      actual_direction = direction;
    } else {
      actual_direction = get_heading();
    }
    angle = normalize_turn_angle(direction - actual_direction);
    turn_time = angle * angle_factor / double(speed);
    turn_time = min_time(turn_time, min_angle_adjust_time);
    logger->print("angle: "); logger->print(angle);
    logger->print(", turn time: "); logger->println(turn_time);
  }

  if(reached_goal) {
    logger->println("reached target angle");
  }
}

void adjust_distance() {
  if(safe_mode) {
    return;
  }

  led(true, true, false);
  logger->println("adjusting distance");
  int target_distance = grid_distance / 2 - dowel_to_middle - separator_width / 2;
  logger->print("target distance: "); logger->println(target_distance);
  for(int i = 0; i < adjust_distance_attempts; i++) {
    Distance d = get_average_distance(adjust_distance_measurements);
    print_distance(d);
    if(d.right > adjust_distance_horizon || d.left > adjust_distance_horizon) {
      logger->println("Cannot adjust grid distance, too far");
      return;
    }
    double min_distance = min(d.right, d.left);
    double distance = min_distance - target_distance;
    logger->print("distance to go: "); logger->println(distance);
    if(abs(distance) < 5) {
      logger->println("reached target distance");
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
  logger->print("distance left: "); 
  logger->print(d.left); 
  logger->print(", right: "); 
  logger->println(d.right);
}

double get_heading() {
  if(safe_mode) {
    return 0.0;
  }
  tcaselect(1);
  double sum = 0.0;
  sensors_event_t orientationData;
  double base;
  for(int i = 0; i < 5; i++) {
    bool res = bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    // We are adding to 360 degrees to the angle before averaging to avoid adding up
    // angles that cross the origin, i.e. adding 1 and 359.
    double d = orientationData.orientation.x;
    if(i == 0) {
      base = d;
    }
    if((base >= 0.0 && base < 90.0) || (base > 270.0 && base <= 360.0)) {
      d = normalize_direction(d, -180.0, 180.0);
    }
    sum += d;
    logger->print("get event: "); logger->print(res);
    logger->print(", get_heading: "); logger->print(orientationData.orientation.x);
    logger->print(", d: "); logger->print(d);
    logger->print(", sum: "); logger->println(sum);
    delay(10);
  }
  double avg_heading = sum / 5;
  avg_heading = normalize_direction(avg_heading);
  logger->print("avg_heading, normalized: "); logger->println(avg_heading);
  return avg_heading;
}

Accel get_accel() {
  if(safe_mode) {
    Accel a;
    a.valid = true;
    a.x = 0.0;
    a.y = 0.0;
    a.z = 0.0;
    return a;
  }
  tcaselect(1);
  sensors_event_t accelerationData;
  bno.getEvent(&accelerationData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  Accel a;
  a.x = accelerationData.acceleration.x;
  a.y = accelerationData.acceleration.y;
  a.z = accelerationData.acceleration.z;
  a.valid = true;
  return a;
}

void compute_move_delay() {
  int count = 0;
  MOVE_STATE* m = moves;
  m += current_move;
  while(*m != STOP) {
    count++;
    m++;
  }
  unsigned long elapsed = millis() - start_time;
  double moves_time = double(msec_per_move) / 1000.0 * count;
  double time_diff = double(time_goal) - double(elapsed) / 1000.0 - moves_time;
  logger->print("elapsed: "); logger->print(elapsed);
  logger->print(", moves left: "); logger->print(count);
  logger->print(", est moves time: "); logger->print(moves_time);
  logger->print(", time diff: "); logger->println(time_diff);
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
  logger->print("got move delay: "); logger->println(move_delay); 
}

void wait_for_stop() {
  if(safe_mode) {
    // In safe mode, we just wait a fixed time, and hopefully we will stop by then.
    delay(300);
    return;
  }
  for(int i = 0; i < 20; i++) {
    Accel a = get_accel();
    a.x -= zero_accel.x;
    double b = abs(a.x);
    logger->print("got accel: "); logger->print(b); logger->print(", max: "); logger->println(max_accel_at_rest);
    if(b < max_accel_at_rest) {
      logger->println("at rest!");
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

bool init_sd() {
  if (!SD.begin(CS_PIN)) {
    logger->println("SD card initialization failed!");
    state = FAIL;
    return false;
  }
  logger->println("SD card initialized");
  return true;
}

void init_log() {
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
    logger->println("cannot find log file name, log is disabled");
    return;
  }

  log_file = SD.open(file_name, FILE_WRITE);
  if(!log_file) {
    Serial.println("error opening log file");
    log_available = false;
    return;
  }
  logger = &log_file;
  Serial.print("opened log file "); Serial.println(file_name);
  log_available = true;
  logger->print("Frank OS, version "); 
  logger->println(version);
  if(log_message != "") {
    logger->println(log_message);
  }
}

void log_distance_measurement(const VL53L1X::RangingData& rd, unsigned long duration) {
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
  logger->print("duration: ");
  logger->print(duration);
  logger->print(", status: ");
  logger->print(status);
  logger->print(", range: ");
  logger->print(rd.range_mm);
  logger->print(", peak signal rate: ");
  logger->print(rd.peak_signal_count_rate_MCPS);
  logger->print(", ambient_count: ");
  logger->println(rd.ambient_count_rate_MCPS);
}

double normalize_direction(double dir, double min_dir = 0.0, double max_dir = 360.0) {
  double d = dir;
  while(d > max_dir) {
    d -= 360.0;
  }
  while(d < min_dir) {
    d += 360.0;
  }
  return d;
}

double normalize_turn_angle(double angle) {
  double a = angle;
  while(a > 180.0) {
    a -= 360.0;
  }
  while(a < -180.0) {
    a += 360.0;
  }
  return a;
}

int min_time(int time, int min_time) {
  if(time > 0 && time < min_time) {
    return min_time;
  }
  if(time < 0 && time > -min_time) {
    return -min_time;
  }
  return time;
}

bool read_program(const String& name) {
  moves_count = 0;

  File file = SD.open(name.c_str());
  if(!file) {
    logger->print("failed to read program: ");
    logger->println(name);

    state = FAIL;
    return false;
  }
  JsonDocument program;
  DeserializationError error = deserializeJson(program, file);
  if(error) {
    logger->println("JSON deserialization failed");
    logger->println(error.f_str());
    state = FAIL;
    return false;
  }
  time_goal = program["time_goal"].as<int>();
  logger->print("time_goal: ");
  logger->println(time_goal);

  moves_count = 0; 
  JsonArray commands = program["commands"].as<JsonArray>();
  for(JsonVariant cmd: commands) {
    const String& line = cmd.as<String>();
    char s[32];
    int count = 0;
    for(int i = 0; i < line.length(); i++) {
      if(line.charAt(i) == ' ') {
        continue;
      }
      if(line.charAt(i) == ',') {
        s[count++] = '\0';
        if(strlen(s) > 0) {
          MOVE_STATE st = parse_move(s);
          if(st == INVALID) {
            state = FAIL;
            return;
          }
          moves[moves_count++] = st;
        }
        count = 0;
        continue;
      }
      s[count++] = line.charAt(i);
    }
    s[count++] = '\0';
    if(strlen(s) > 0) {
      MOVE_STATE st = parse_move(s);
      if(st == INVALID) {
        state = FAIL;
        return;
      }
      moves[moves_count++] = st;
    }
  }
  if(moves_count == 0) {
    logger->println("got zero moves!");
    state = FAIL;
    return false;
  }
  if(moves[moves_count - 1] != STOP) {
    moves[moves_count++] = STOP;
  }
  return true;
}

MOVE_STATE parse_move(const String& s) {
  logger->println(s);
  if(s == "GO_IN" || s == "GI") {
    return GO_IN;
  }
  if(s == "FORWARD" || s == "F") {
    return FORWARD;
  }
  if(s == "ADJUST" || s == "A") {
    return ADJUST;
  }
  if(s == "BACKWARD" || s == "B") {
    return BACKWARD;
  } 
  if(s == "TURN_LEFT" || s == "L") {
    return TURN_LEFT;
  }
  if(s == "TURN_RIGHT" || s == "R") {
    return TURN_RIGHT;
  }
  if(s == "LEFT_SHIFT" || s == "LS") {
    return LEFT_SHIFT;
  }
  if(s == "RIGHT_SHIFT" || s == "RS") {
    return RIGHT_SHIFT;
  }
  if(s == "TEST_MOVE") {
    return TEST_MOVE;
  }
  if(s == "FORWARD_TO_TARGET" || s == "FT") {
    return FORWARD_TO_TARGET;
  }
  if(s == "BACKWARD_TO_TARGET" || s == "BT") {
    return BACKWARD_TO_TARGET;
  }
  if(s == "DELAY" || s == "D") {
    return DELAY;
  }
  if(s == "NORTH" || s == "N") {
    return NORTH;
  }
  if(s == "EAST" || s == "E") {
    return EAST;
  }
  if(s == "SOUTH" || s == "S") {
    return SOUTH;
  }
  if(s == "WEST" || s == "W") {
    return WEST;
  }
  if( s == "STOP") {
    return STOP;
  }
  logger->print("Unknown command: "); logger->println(s);
  return INVALID;
}

const int debounce_delay = 10;
const int max_debounce_delay = 100;

bool debounce(int pin) {
  bool state;
  bool prev_state;
  prev_state = digitalRead(pin);
  int count = 0;
  for(int i = 0; i < debounce_delay; i++) {
    if(count++ == max_debounce_delay) {
      return state;
    }
    delay(1);
    state = digitalRead(pin);
    if(state != prev_state) {
      i = 0;
      prev_state = state;
    }
  }
  return state;
}

bool switch_a() {
  return debounce(SWITCH_A_PIN);
}

bool switch_b() {
  return debounce(SWITCH_B_PIN);
}