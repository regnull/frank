#include <math.h>
#include <stdio.h>
#include <ezButton.h>
#include <Wire.h>
#include <VL53L1X.h>

struct Distance {
  double left;
  double right;
};

const int measure_distance_max_attempts = 5;  // Max attempts to measure distance before giving up.

// Robot dimensions

const int dowel_to_middle = 130;  // Distance between the dowel and the middle of the robot
const int sensors_base = 83;      // Distance between the sensors, millimeters.
const int separator_width = 36;   // Width of the separator, millimeters.
const int right_sensor_correction = 8;

// Motion

const int grid_distance = 500;    // Grid distance, in millimeters.
const int distance_factor = 245;  // !!! Adjust this to get the distance right
const int move_delay = 500;       // Delay between moves, milliseconds.
const int angle = 90;             // Degrees
const int angle_factor = 660;     // !!! Adjust this to get the turn angle right
const int shift_distance = 500;   // Millimeters
const int shift_factor = 600;     // !!! Adjust this to get the shift distance right
const int stop_distance = 50;     // Stop if there is an obstacle at this distance

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

const int R_SENSOR_IRQ_PIN = 44;
const int R_SENSOR_XSHUT_PIN = 42;

const int L_SENSOR_IRQ_PIN = 38;
const int L_SENSOR_XSHUT_PIN = 40;

VL53L1X vl53_r;
VL53L1X vl53_l;

enum STATE {
  START,
  WAIT_FOR_READY,
  READY,
  IN_MOTION,
  FINISH
};

STATE state = START;

int serial_putc(char c, FILE *) {
  Serial.write(c);
  return c;
} 

void printf_begin() {
  fdevopen(&serial_putc, 0);
}

enum MOVE_STATE {
  GO_IN,            // Start from the edge of the grid, go into the first square. Must be the first command!
  FORWARD, F,       // Go forward one square
  ADJUST, A,        // Adjust the position (assuming there is a board in front)
  FNA,              // Go forward one square, no adjustment.
  BACKWARD, B,      // Go backward one square
  TURN_LEFT, L,     // Turn left 90 degrees
  TURN_RIGHT, R,    // Turn right 90 degrees
  LEFT_SHIFT, LS,   // Shift left one square
  RIGHT_SHIFT, RS,  // Shift right one square
  TEST_MOVE,        // Test only, do not use!
  GO_TO_TARGET,     // Go into the target square, stop with the dowel over the target. Must be the last command!
  DELAY, D,
  INTERACT,
  STOP, S
};

// !!!!!!! Speed

int speed = 100;

// !!!!!!! Robot moves

const MOVE_STATE moves_a[] = {
  S,
  // GO_IN,       // GO_IN must be the first_command!
  // L,
  // FNA,
  // R,
  // F,
  // R,
  // F,
  // L,
  // F,
  // L,
  // FNA,
  // R,
  // FNA,
  // R,
  // GO_TO_TARGET // GO_TO_TARGET must be the last command!
};

const MOVE_STATE moves_b[] = {
  INTERACT, S,
  //INTERACT,
  // GO_IN,       // GO_IN must be the first_command!
  // GO_TO_TARGET // GO_TO_TARGET must be the last command!
};

int current_move = 0;

MOVE_STATE *moves;

void setup() {
  Serial.begin(9600);
  printf_begin();
  printf("\n\n");

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
  Serial.println("!! START");

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
  Serial.println("!! WAIT_FOR_READY");

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
  Serial.println("!! READY");

  digitalWrite(GREEN_LED_PIN, LOW);
  digitalWrite(YELLOW_LED_PIN, HIGH);
  digitalWrite(RED_LED_PIN, LOW);

  // TODO: Compute speed here.
  // Warm up the sensors.
  Distance d = get_distance();
  delay(1000);

  if(switchB.getState() == LOW) {
    Serial.println("using program A");
    moves = moves_a;
  } else {
    Serial.println("using program B");
    moves = moves_b;
  }

  digitalWrite(GREEN_LED_PIN, LOW);
  digitalWrite(YELLOW_LED_PIN, LOW);
  digitalWrite(RED_LED_PIN, HIGH);
  state = IN_MOTION;
}

void in_motion() {
  Serial.println("!! IN_MOTION");

  MOVE_STATE next_move = moves[current_move];

  switch(next_move) {
    case GO_IN:
      Serial.println(">> GO_IN");
      move_into_grid(speed);
      adjust_distance();
      adjust_angle();
      break;
    case FORWARD:
    case F:
      Serial.println(">> FORWARD");
      move_forward(speed);
      adjust_distance();
      adjust_angle();
      break;
    case ADJUST:
    case A:
      Serial.println(">> ADJUST");
      adjust_distance();
      adjust_angle();
      break;
    case FNA:
      Serial.println(">> FNA");
      move_forward(speed);
      break;  
    case BACKWARD:
    case B:
      Serial.println(">> BACKWARD");
      move_backward(speed);
      break;
    case TURN_LEFT:
    case L:
      Serial.println(">> TURN_LEFT");
      move_turn_left(speed);
      break;
    case TURN_RIGHT:
    case R:
      Serial.println(">> TURN_RIGHT");
      move_turn_right(speed);
      break;
    case LEFT_SHIFT:
    case LS:
      Serial.println(">> LEFT_SHIFT");
      move_left_shift(speed);
      break;
    case RIGHT_SHIFT:
    case RS:
      Serial.println(">> RIGHT_SHIFT");
      move_right_shift(speed);
      break;
    case TEST_MOVE:
      Serial.println(">> TEST_MOVE");
      move_test(speed);
      break;
    case GO_TO_TARGET:
      Serial.println(">> GO_TO_TARGET");
      move_go_to_target(speed);
      state = FINISH;
      break;
    case DELAY:
    case D:
      Serial.println(">> DELAY");
      delay(5000);
      break;
    case INTERACT:
      Serial.println(">> INTERACT");
      move_interact();
      break;
    case STOP:
    case S:
      Serial.println(">> STOP");
      state = FINISH;
      break;
  }

  delay(move_delay);

  current_move++;
}

void finish() {
  Serial.println("!! FINISH");

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
}

void move_go_to_target(int speed) {
  go_forward(speed);
  delay(compute_move_time(grid_distance - dowel_to_middle, distance_factor, speed));
  stop_motors();
}

void move_forward(int speed) {
  // If we can get distance measurement, use it to make sure we don't bump into things.
  Distance d = get_average_distance(5);
  double distance = grid_distance;
  if(d.left > 0 && d.right > 0) {
    Serial.print("distance right: "); Serial.print(d.right); Serial.print(", left "); Serial.println(d.left);
    double dm = min(d.left, d.right) - stop_distance;
    if(dm < distance) {
      Serial.print("obstacle detected at "); Serial.println(dm);
      distance = dm;
    }
    if(distance < 0.0) {
      distance = 0.0;
    }
  } else {
    Serial.println("failed to get distance");
  }
  int time = compute_move_time(distance, distance_factor, speed);
  Serial.print("move time "); Serial.println(time);
  if(time <= 0) {
    return;
  }

  go_forward(speed);
  delay(time);
  stop_motors();
}

void move_backward(int speed) {
  go_backward(speed);
  delay(compute_move_time(grid_distance, distance_factor, speed));
  stop_motors();
}

void move_turn_left(int speed) {
  turn_left(speed);
  delay(compute_move_time(angle, angle_factor, speed));
  stop_motors();
}

void move_turn_right(int speed) {
  turn_right(speed);
  delay(compute_move_time(angle, angle_factor, speed));
  stop_motors();
}

void move_right_shift(int speed) {
  right_shift(speed, speed, speed, speed);  
  delay(compute_move_time(shift_distance, shift_factor, speed));
  stop_motors();
}

void move_left_shift(int speed) {
  left_shift(speed, speed, speed, speed);  
  delay(compute_move_time(shift_distance, shift_factor, speed));
  stop_motors();
}

void move_test(int speed) {
  adjust_angle();
}

void move_interact() {
  char inChar = 0;
  char command[16];
  static int i = 0; // static ensures the variable retains its value between function calls

  while(true) {
    Serial.print("command >>> ");

    while(true) {
      if(Serial.available() <= 0) {
        delay(100);
        continue;
      }
      inChar = Serial.read();

      if (inChar != '\n') {
        Serial.print(inChar);
        command[i++] = inChar;

        // Check if the buffer is full to avoid overflow
        if (i >= sizeof(command) - 1) {
          i = 0; // Reset the index if the buffer is full
        }
      } else {
        Serial.println();
        // Process the received string or reset if needed
        command[i] = '\0'; // Null-terminate the string
        Serial.print("Received: ");
        Serial.println(command);

        if(strcmp(command, "F") == 0) {
          move_forward(speed);
        } else if (strcmp(command, "B") == 0) {
          move_backward(speed);
        } else if (strcmp(command, "L") == 0) {
          move_turn_left(speed);
        } else if (strcmp(command, "R") == 0) {
          move_turn_right(speed);
        } else if (strcmp(command, "AA") == 0) {
          adjust_angle();
        } else if (strcmp(command, "AD") == 0) {
          adjust_distance();
        } else if (strcmp(command, "S") == 0) {
          state = FINISH;
          return;
        } else {
          Serial.println("bad command");
        }

        // Clear the buffer for the next string
        memset(command, 0, sizeof(command));

        // Reset the index
        i = 0;
      }
    }
  }
}

long compute_move_time(int distance, int factor, int speed) {
  return long(distance) * long(factor) / long(speed);
}

// Sensors control

void init_sensors() {
  Serial.println("init sensors");

  Wire.begin();
  Wire.setClock(400000); // use 400 kHz I2C

  pinMode(L_SENSOR_XSHUT_PIN, OUTPUT);
  digitalWrite(L_SENSOR_XSHUT_PIN, LOW);
  pinMode(R_SENSOR_XSHUT_PIN, OUTPUT);
  digitalWrite(R_SENSOR_XSHUT_PIN, LOW);

  pinMode(L_SENSOR_XSHUT_PIN, INPUT);
  delay(10);
  vl53_l.setTimeout(500);
  if(!vl53_l.init()) {
    Serial.println("Failed to detect and initialize left sensor");
    while (1);
  }
  vl53_l.setAddress(0x2A);
  vl53_l.startContinuous(50);
  Serial.println("Left sensor initialized");

  pinMode(R_SENSOR_XSHUT_PIN, INPUT);
  delay(10);
  vl53_r.setTimeout(500);
  if(!vl53_r.init()) {
    Serial.println("Failed to detect and initialize right sensor");
    while (1);
  }
  vl53_r.setAddress(0x2B);
  vl53_r.startContinuous(50);
  Serial.println("Right sensor initialized");
}

Distance get_average_distance(int n) {
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
      double dr = get_distance_r();
      double dl = get_distance_l();
      if(dr <= 0 || dl <= 0) {
        continue;
        delay(10);
      }
      measurements++;
      sum_r += dr;
      sum_l += dl;
      delay(10);
    }
    d.left = sum_l / double(n);
    d.right = sum_r / double(n);
    return d;
}

Distance get_distance() {
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
      Serial.println("failed to get distance");
      delay(10);
      continue;
    }
    d.left = dl;
    d.right = dr;
    return d;
  }
}

int get_distance_l() {
  while(true) {
    if(vl53_l.dataReady()) {
      break;
    }
    delay(10);
  }
  // new measurement for the taking!
  int distance = vl53_l.read();
  // Serial.print("distance l: "); Serial.printl(distance);
  if (distance == -1) {
    // something went wrong!
    Serial.println(F("Couldn't get distance (L)"));
    return -1;
  }
  return distance;
}

int get_distance_r() {
  while(true) {
    if(vl53_r.dataReady()) {
      break;
    }
    delay(10);
  }
  // new measurement for the taking!
  int distance = vl53_r.read();
  //Serial.print("distance r: "); Serial.println(distance);
  if (distance == -1) {
    // something went wrong!
    Serial.println(F("Couldn't get distance (R)"));
    return -1;
  }
  if(distance > 0.0) {
    distance += right_sensor_correction;
  }
  return distance;
}

void adjust_angle() {
  Distance d = get_average_distance(5);
  if(d.left > grid_distance || d.right > grid_distance) {
    printf("Cannot adjust angle, too far\n");
    return;
  }
  Serial.print("got distances, left: ");
  Serial.print(d.left);
  Serial.print(", right: ");
  Serial.println(d.right);
  double distance_delta = d.right - d.left;
  Serial.print("distance delta: "); Serial.println(distance_delta);
  double angle = compute_angle(distance_delta);
  Serial.print("angle: "); Serial.println(angle);
  int turn_time = angle * angle_factor / double(speed);
  Serial.print("turn time: "); Serial.println(turn_time);

  int attempts = 1;

  while(abs(angle) > 1.0) {
    if(attempts == 10) {
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
    d = get_average_distance(5);
    distance_delta = d.right - d.left;
    angle = compute_angle(distance_delta);
    turn_time = angle * angle_factor / double(speed);
    Serial.print("got distances, left: ");
    Serial.print(d.left);
    Serial.print(", right: ");
    Serial.println(d.right);
    Serial.print("distance delta: "); Serial.println(distance_delta);
    Serial.print("angle: "); Serial.println(angle);
    Serial.print("turn time: "); Serial.println(turn_time);

  //   printf("distance delta: %f\n", distance_delta);
  //   printf("angle: %f\n", angle);
  //   printf("turn time: %d\n", turn_time);
  }
}

void adjust_distance() {
  int target_distance = grid_distance / 2 - dowel_to_middle - separator_width / 2;
  double dr = get_distance_r();
  double dl = get_distance_l();
  if(dr > grid_distance || dl > grid_distance) {
    Serial.println("Cannot adjust grid distance, too far");
    return;
  }
  double min_distance = min(dr, dl);
  while(abs(target_distance - min_distance) > 10) {
    if(min_distance > target_distance) {
      go_forward(speed);
    } else {
      go_backward(speed);
    }
    delay(50);
    stop_motors();
    delay(50);
    dr = get_distance_r();
    dl = get_distance_l();
    min_distance = min(dr, dl);
  } 
}

double compute_angle(double distance_delta) {
  double angle_rad = atan2(distance_delta, sensors_base);
  return angle_rad / M_PI * 180.0;
}
