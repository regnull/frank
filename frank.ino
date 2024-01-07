#include <ezButton.h>
#include "Adafruit_VL53L1X.h"

// Robot dimensions

const int dowel_to_middle = 130;  // Distance between the dowel and the middle of the robot.

// Motion

const int grid_distance = 500;    // Grid distance, in millimeters.
const int distance_factor = 310;  // !!! Adjust this to get the distance right
const int move_delay = 500;
const int angle = 90;             // Degrees
const int angle_factor = 850;     // !!! Adjust this to get the turn angle right
const int shift_distance = 500;   // Millimeters
const int shift_factor = 600;     // !!! Adjust this to get the shift distance right

// LEDs

const int RED_LED_PIN = 51;
const int YELLOW_LED_PIN = 50;
const int GREEN_LED_PIN = 49;

// Switches

const int SWITCH_A_PIN = 53;
const int SWITCH_B_PIN = 52;

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

const int IRQ_PIN = 44;
const int XSHUT_PIN = 42;

Adafruit_VL53L1X vl53 = Adafruit_VL53L1X(XSHUT_PIN, IRQ_PIN);


enum STATE {
  START,
  WAIT_FOR_READY,
  READY,
  IN_MOTION,
  FINISH
};

STATE state = START;

enum MOVE_STATE {
  GO_IN,        // Start from the edge of the grid, go into the first square. Must be the first command!
  FORWARD,      // Go forward one square
  BACKWARD,     // Go backward one square
  TURN_LEFT,    // Turn left 90 degrees
  TURN_RIGHT,   // Turn right 90 degrees
  LEFT_SHIFT,   // Shift left one square
  RIGHT_SHIFT,  // Shift right one square
  TEST_MOVE,    // Test only, do not use!
  GO_TO_TARGET, // Go into the target square, stop with the dowel over the target. Must be the last command!
  STOP
};

// !!!!!!! Speed

int speed = 100;

// !!!!!!! Robot moves

const MOVE_STATE moves[] = {
  // GO_IN,       // GO_IN must be the first_command!
  // FORWARD,
  TEST_MOVE,
  STOP,
  // TURN_LEFT,
  // TURN_RIGHT,
  // RIGHT_SHIFT,
  // LEFT_SHIFT,
  // GO_TO_TARGET // GO_TO_TARGET must be the last command!
};

int current_move = 0;

void setup() {
  Serial.begin(9600);
  Serial.println();
  Serial.println();

  // Switches
  switchA.setDebounceTime(50);
  switchB.setDebounceTime(50);

  // Motors
  init_motors_GPIO();

  // LEDs
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(YELLOW_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);

  // Sensors
  init_sensors();
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
      Serial.println("Unknown state!");
      delay(1000);
  }
}

void start() {
  Serial.println("START");

  // All LEDs off.
  digitalWrite(GREEN_LED_PIN, LOW);
  digitalWrite(YELLOW_LED_PIN, LOW);
  digitalWrite(RED_LED_PIN, LOW);

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
  Serial.println("WAIT_FOR_READY");

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
  Serial.println("READY");

  digitalWrite(GREEN_LED_PIN, LOW);
  digitalWrite(YELLOW_LED_PIN, HIGH);
  digitalWrite(RED_LED_PIN, LOW);

  // TODO: Compute speed here.
  delay(1000);

  digitalWrite(GREEN_LED_PIN, LOW);
  digitalWrite(YELLOW_LED_PIN, LOW);
  digitalWrite(RED_LED_PIN, HIGH);
  state = IN_MOTION;
}

void in_motion() {
  Serial.println("IN_MOTION");

  MOVE_STATE next_move = moves[current_move];
  Serial.print("move: ");
  Serial.println(next_move);

  switch(next_move) {
    case GO_IN:
      move_into_grid(speed);
      break;
    case FORWARD:
      move_forward(speed);
      break;  
    case BACKWARD:
      move_backward(speed);
      break;
    case TURN_LEFT:
      move_turn_left(speed);
      break;
    case TURN_RIGHT:
      move_turn_right(speed);
      break;
    case LEFT_SHIFT:
      move_left_shift(speed);
      break;
    case RIGHT_SHIFT:
      move_right_shift(speed);
      break;
    case TEST_MOVE:
      move_test(speed);
      break;
    case GO_TO_TARGET:
      move_go_to_target(speed);
      state = FINISH;
      break;
    case STOP:
      state = FINISH;
      break;
  }

  current_move++;
}

void finish() {
  Serial.println("FINISH");

  digitalWrite(GREEN_LED_PIN, HIGH);
  digitalWrite(YELLOW_LED_PIN, HIGH);
  digitalWrite(RED_LED_PIN, HIGH);

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
   RL_fwd(speed);
   RR_fwd(speed);
   FR_fwd(speed);
   FL_fwd(speed); 
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
  delay(move_delay);
}

void move_go_to_target(int speed) {
  go_forward(speed);
  delay(compute_move_time(grid_distance - dowel_to_middle, distance_factor, speed));
  stop_motors();
  delay(move_delay);
}

void move_forward(int speed) {
  go_forward(speed);
  delay(compute_move_time(grid_distance, distance_factor, speed));
  stop_motors();
  delay(move_delay);
}

void move_backward(int speed) {
  go_backward(speed);
  delay(compute_move_time(grid_distance, distance_factor, speed));
  stop_motors();
  delay(move_delay);
}

void move_turn_left(int speed) {
  turn_left(speed);
  delay(compute_move_time(angle, angle_factor, speed));
  stop_motors();
  delay(move_delay);
}

void move_turn_right(int speed) {
  turn_right(speed);
  delay(compute_move_time(angle, angle_factor, speed));
  stop_motors();
  delay(move_delay);
}

void move_right_shift(int speed) {
  right_shift(speed, speed, speed, speed);  
  delay(compute_move_time(shift_distance, shift_factor, speed));
  stop_motors();
  delay(move_delay);
}

void move_left_shift(int speed) {
  left_shift(speed, speed, speed, speed);  
  delay(compute_move_time(shift_distance, shift_factor, speed));
  stop_motors();
  delay(move_delay);
}

void move_test(int speed) {
  // double min_distance = get_average_distance(5);
  double goal = 80;
  for(int i = 0; i < 100; i++) {
    double d = get_average_distance(10);
    Serial.print("average distance: ");
    Serial.print(d);
    Serial.print(", ");
    Serial.println(goal);
    if(d < goal) {
      break;
    }
    go_forward(speed);
    delay(100);
    stop_motors();
    delay(50);
  }
  //   turn_left(speed);
  //   delay(30);
  //   stop_motors();
  //   delay(30);
  //   double d = get_average_distance(5);
  //   Serial.print(F("Avg distance: "));
  //   Serial.print(d);
  //   Serial.print(", ");
  //   Serial.print(min_distance);
  //   if(d - min_distance > 5.0) {
  //     break;
  //   }
  //   if(d < min_distance) {
  //     min_distance = d;
  //   }
  // }
}

long compute_move_time(int distance, int factor, int speed) {
  return long(distance) * long(factor) / long(speed);
}

// Sensors control

void init_sensors() {
  Wire.begin();
  if (! vl53.begin(0x29, &Wire)) {
    Serial.print(F("Error on init of VL sensor: "));
    Serial.println(vl53.vl_status);
    while (1)       delay(10);
  }
  Serial.println(F("VL53L1X sensor OK!"));

    Serial.print(F("Sensor ID: 0x"));
  Serial.println(vl53.sensorID(), HEX);

  if (! vl53.startRanging()) {
    Serial.print(F("Couldn't start ranging: "));
    Serial.println(vl53.vl_status);
    while(true) {
      delay(10);
    } 
  }
  Serial.println(F("Ranging started"));

  // Valid timing budgets: 15, 20, 33, 50, 100, 200 and 500ms!
  vl53.setTimingBudget(50);
  Serial.print(F("Timing budget (ms): "));
  Serial.println(vl53.getTimingBudget());
}

int get_distance() {
  while(true) {
    if(vl53.dataReady()) {
      break;
    }
    delay(10);
  }
  // new measurement for the taking!
  int distance = vl53.distance();
  if (distance == -1) {
    // something went wrong!
    Serial.print(F("Couldn't get distance: "));
    Serial.println(vl53.vl_status);
    return -1;
  }
  Serial.print(F("Distance: "));
  Serial.print(distance);
  Serial.println(" mm");

  // data is read out, time for another reading!
  vl53.clearInterrupt();
  return distance;
}

double get_average_distance(int n) {
    double sum = 0.0;
    for(int j = 0; j < n; j++) {
      double d = get_distance();
      sum += d;
      delay(10);
    }
    return sum / double(n);
}