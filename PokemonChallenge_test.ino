/*

  0: obstacle avoidance left (can be used as LED or serial)
  1: LED one color channel (LED coupled with serial)
  2: right optical encoder
  3: left optical encoder
  4: (IN1) left motor -
  5: (ENA) left motor  (pwm)
  6: (ENB) right motor (pwm)
  7: (IN2) left motor +
  8: X1 (right line following)
  9: gripper servo
  10: platform servo
  11: X2 (middle right line following)
  12: X3 (middle left line following)
  13: X4 (left line following)

  A0: Trig (ultrasonic)
  A1: Echo (ultrasonic)
  A2: (IN3) right motr +
  A3: (IN4) right motor -
  A4: obstacle avoidance right (can be used as LED)
  A5: grey scale

*/

#define Left_obstacle  0
#define LED 1
#define Right_optical_encoder 2
#define Left_optical_encoder 3
#define Left_motor_back 4
#define Left_motor_pwm 5
#define Right_motor_pwm 6
#define Left_motor_go 7
#define X1 8
#define Gripper_servo_pin 9
#define Platform_servo_pin 10
#define X2 11
#define X3 12
#define X4 13

#define Trig A0
#define Echo A1
#define Right_motor_go A2
#define Right_motor_back A3
#define Right_obstacle A4
#define Grey_scale A5

#include "Servo.h"

Servo Gripper_servo;
Servo Platform_servo;
volatile long right_pulse = 0;
volatile long left_pulse = 0;
long current_road_mark_left = 0;
long current_road_mark_right = 0;
int current_step = 0;
const int steps = 6;
// 0:stop, 1:forward, 2:backward, 3:turn_left, 4:turn_right, 5:spin_left, 6:spin_right

long road_mark_left_array[steps] = { -1, -1, -1, 60, -1, 0};
long road_mark_right_array[steps] = {200, 60, 200, -1, 200, 0};
int action_array[steps] = {1, 5, 1, 6, 1, 0};
int speed_left[steps] = {178, 94, 178, 94, 178, 178};
int speed_right[steps] = {170, 84, 170, 84, 170, 170};
bool obstacle_detection = true;

// task assignments 0: forward, 1: left, 2: right, 3: catch Pokemon, retract and turn back,
const int maxTaskSteps = 20;
int redTaskTurns[maxTaskSteps] = {1, 0, 1, 1, 3, -1};
int greenTaskTurns[maxTaskSteps] = {1, 2, 0, 3, -1};
int blueTaskTurns[maxTaskSteps] = {0, 0, 2, 0, 0, 0, 0, 1, 3, -1};
int cyanTaskTurns[maxTaskSteps] = {0, 2, 0, 2 , 1 , 3, -1};
int selectedTaskTurns[maxTaskSteps];
int currentTaskIndex = 0;
int greyScaleValue;
long redIgnore[maxTaskSteps] = {50, 50, 50, 50, 50, 0};
long greenIgnore[maxTaskSteps] = {40, 50, 50, 50, 0};
long blueIgnore[maxTaskSteps] = {50, 50, 50, 50, 50, 50, 50, 50, 50, 0};
long cyanIgnore[maxTaskSteps] = {40, 50, 50, 50, 50, 50, 0};
long ignoreCountsBefore[maxTaskSteps] ;
// 0 is black line detected, 1 is no black line
int line_left = 1;
int line_mid_left = 1;
int line_mid_right = 1;
int line_right = 1;
int current_task = 0; //0: red, 1: green, 2: blue, 3: cyan

long checkPointRight = 0;


void setup() {
  Serial.begin(9600);
  initialize();
  full_stop();
  delay(500);
  greyScaleValue = analogRead(Grey_scale);
  if (greyScaleValue > 350) {
    current_task = 2; // blue
  }
  else if (greyScaleValue < 255) {
    current_task = 3; //cyan
  }
  else {
    while (1) {
      if (!digitalRead(Left_obstacle)) {
        current_task = 0;
        break;
      }
      if (!digitalRead(Right_obstacle)) {
        current_task = 1;
        break;
      }
    }
  }

  if (current_task == 0) {
    for ( int ii = 0; ii < maxTaskSteps; ++ii) {
      selectedTaskTurns[ii] = redTaskTurns[ii];
      ignoreCountsBefore[ii] = redIgnore[ii];
    }
  }
  else if (current_task == 1) {
    for ( int ii = 0; ii < maxTaskSteps; ++ii) {
      selectedTaskTurns[ii] = greenTaskTurns[ii];
      ignoreCountsBefore[ii] = greenIgnore[ii];
    }
  }
  else if (current_task == 2) {
    for ( int ii = 0; ii < maxTaskSteps; ++ii) {
      selectedTaskTurns[ii] = blueTaskTurns[ii];
      ignoreCountsBefore[ii] = blueIgnore[ii];
    }
  }
  else if (current_task == 3) {
    for ( int ii = 0; ii < maxTaskSteps; ++ii) {
      selectedTaskTurns[ii] = cyanTaskTurns[ii];
      ignoreCountsBefore[ii] = cyanIgnore[ii];
    }
  }
  move_forward(88, 88);
  delay(300);
  checkPointRight = right_pulse;
}


void loop() {
  //Serial.println(right_pulse);
  //delay(200);
  //for stopping the vehicle
  if (obstacle_avoidance() && obstacle_detection) {
    while (1) {
    }
  }
  trackLine();
  // 0XXX or XXX0
  if ((line_left == 0 || line_right == 0) && right_pulse >= checkPointRight + ignoreCountsBefore[currentTaskIndex] ) {
    // ignore cross road, just go forward, modify this part for project
    //gripper_cheer();
    Serial.print("Current Task:");
    Serial.println(selectedTaskTurns[currentTaskIndex]);
    applyTask(selectedTaskTurns[currentTaskIndex]);
    ++currentTaskIndex;
    if (selectedTaskTurns[currentTaskIndex] == -1) {
      while (1) {
        full_stop();
      }
    }
    if (selectedTaskTurns[currentTaskIndex] == 3) {
      Gripper_servo.write(180);
      delay(1000);
    }
    checkPointRight = right_pulse;
  }
  else {
    if (line_mid_right == 1) {
      spin_left(38, 103);
    }
    else {
      spin_right(103, 38);
      delay(50);
    }
  }
}


// task assignments 0: forward, 1: left, 2: right, 3: grab Pokemon, 4: retract and turn back, 5:release Pokemon
void applyTask(int taskID) {

  current_road_mark_left = left_pulse;
  current_road_mark_right = right_pulse;
  if (taskID == 0) {

    current_road_mark_left = left_pulse;
    while (left_pulse <= current_road_mark_left + 5) {
      spin_right(88, 48);
    }
    current_road_mark_right = right_pulse;
    while (right_pulse <= current_road_mark_right + 5) {
      move_forward(88, 88);
    }
    trackLine();
    while (line_mid_right == 1) {
      spin_left(48, 88);
      trackLine();
    }
  }
  else if (taskID == 1) { // left
    full_stop(); //stop the car to avoid dynamic drifting effect
    delay(1000);

    while (right_pulse <= current_road_mark_right + 10) {
      move_forward(108, 108);
    }
    current_road_mark_right = right_pulse;
    while (right_pulse <= current_road_mark_right + 60) {
      spin_left(48, 118);
    }
    full_stop();
    delay(1000);
  }
  else if (taskID == 2) { // right
    full_stop(); //stop the car to avoid dynamic drifting effect
    delay(1000);
    while (right_pulse <= current_road_mark_right + 5) {
      move_forward(108, 108);
    }
    current_road_mark_left = left_pulse;
    while (left_pulse <= current_road_mark_left + 75) {
      spin_right(118, 48);
    }
    full_stop();
    delay(1000);
  }
  else if (taskID == 3) { // grab pokemon
    long pulsetoRetract = 0;
    full_stop();
    delay(1000);
    Gripper_servo.write(90);
    while (1) {
      full_stop();
      delay(1000);
    }


  }
  else {

    while (1) {
      full_stop();
    }
  }


}


//obtain line following sensor read out
void trackLine() {
  line_left = digitalRead(X2);
  line_mid_left = digitalRead(X1);
  line_mid_right = digitalRead(X3);
  line_right = digitalRead(X4);
}

void regain_track() {
  trackLine();
  if (line_right == 0) {
    turn_right(100, 50);
    delay(30);
  }
  else if (line_left == 0) {
    turn_left(50, 100);
    delay(30);
  }
}

// ultrasonic distance sensor measurement
float measure_distance()   // measure distance
{
  digitalWrite(Trig, LOW);   // send low for 2 micro seconds
  delayMicroseconds(2);
  digitalWrite(Trig, HIGH);  // send high for 10 micro seconds as a pulse
  delayMicroseconds(10);
  digitalWrite(Trig, LOW);    // send low
  float Fdistance = pulseIn(Echo, HIGH);  // read high level time in micro seconds
  float Distance = 0;
  Distance = Fdistance / 58;    //convert from micro seconds to centi meter  Y in meter, X in seconds, Y=（X*344）/2
  // X seconds =（ 2*Y meters）/344 ==》X seconds =0.0058*Y meter ==》cm = microseconds /58
  //Serial.print("Distance:");      //Display distance
  //Serial.println(Distance);
  return Distance;
}

// function to initialize all the pins
void initialize() {
  pinMode(Left_obstacle, INPUT);
  pinMode(LED, OUTPUT);
  pinMode(Right_optical_encoder, INPUT);
  pinMode(Left_optical_encoder, INPUT);
  pinMode(Left_motor_go, OUTPUT);
  pinMode(Left_motor_pwm, OUTPUT);
  pinMode(Right_motor_pwm, OUTPUT);
  pinMode(Left_motor_back, OUTPUT);
  pinMode(X1, INPUT);
  pinMode(Gripper_servo_pin, OUTPUT);
  pinMode(Platform_servo_pin, OUTPUT);
  pinMode(X2, INPUT);
  pinMode(X3, INPUT);
  pinMode(X4, INPUT);

  pinMode(Trig, OUTPUT);
  pinMode(Echo, INPUT);
  pinMode(Right_motor_go, OUTPUT);
  pinMode(Right_motor_back, OUTPUT);
  pinMode(Right_obstacle, INPUT);

  pinMode(Grey_scale, INPUT);

  attachInterrupt(digitalPinToInterrupt(Right_optical_encoder), right_count, CHANGE);
  attachInterrupt(digitalPinToInterrupt(Left_optical_encoder), left_count, CHANGE);

  digitalWrite(LED, HIGH);

  Gripper_servo.attach(Gripper_servo_pin, 70, 180);
  Platform_servo.attach(Platform_servo_pin);

  Gripper_servo.write(130);
  Platform_servo.write(90);
}

// function for obstacle avoidance checking
bool obstacle_avoidance() {
  if (obstacle_detection) {
    if (digitalRead(Left_obstacle) && digitalRead(Right_obstacle)) {
      //do nothing, no obstacle
    }
    else {
      delay(20);
      if (digitalRead(Left_obstacle) && digitalRead(Right_obstacle)) {
        //do nothing, this is sensor glitch
      }
      else {
        full_stop();
        current_step = steps + 1;
        return true;
      }
    }
  }
  return false;
}

// call back for interrupt function
void right_count() {
  ++right_pulse;
}

// call back for interrupt function
void left_count() {
  ++left_pulse;
}

// move based on pre-defined motion
void apply_movement(int this_step) {
  if (this_step < steps) {
    switch (action_array[this_step]) {
      case 0:
        full_stop(); break;
      case 1:
        move_forward(speed_left[current_step], speed_right[current_step]); break;
      case 2:
        move_backward(speed_left[current_step], speed_right[current_step]); break;
      case 3:
        turn_left(speed_left[current_step], speed_right[current_step]); break;
      case 4:
        turn_right(speed_left[current_step], speed_right[current_step]); break;
      case 5:
        spin_left(speed_left[current_step], speed_right[current_step]); break;
      case 6:
        spin_right(speed_left[current_step], speed_right[current_step]); break;
      default:
        full_stop(); break;
    }
  }
  else {
    full_stop();
  }
}

// gripper open and close motion
void gripper_cheer() {
  Gripper_servo.write(180);
  delay(500);
  Gripper_servo.write(90);
  delay(500);
  Gripper_servo.write(180);
  delay(500);
  Gripper_servo.write(90);
  delay(500);
  Gripper_servo.write(130);
}


// action 0
void full_stop() {
  analogWrite(Left_motor_pwm, 0);
  analogWrite(Right_motor_pwm, 0);
  digitalWrite(Left_motor_go, LOW);
  digitalWrite(Left_motor_back, LOW);
  digitalWrite(Right_motor_go, LOW);
  digitalWrite(Right_motor_back, LOW);
}
// action 1
void move_forward(int PWM_Left, int PWM_Right) {
  analogWrite(Left_motor_pwm, PWM_Left);
  analogWrite(Right_motor_pwm, PWM_Right);
  digitalWrite(Left_motor_go, HIGH);
  digitalWrite(Left_motor_back, LOW);
  digitalWrite(Right_motor_go, HIGH);
  digitalWrite(Right_motor_back, LOW);
}

// action 2
void move_backward(int PWM_Left, int PWM_Right) {
  analogWrite(Left_motor_pwm, PWM_Left);
  analogWrite(Right_motor_pwm, PWM_Right);
  digitalWrite(Left_motor_go, LOW);
  digitalWrite(Left_motor_back, HIGH);
  digitalWrite(Right_motor_go, LOW);
  digitalWrite(Right_motor_back, HIGH);
}

// action 3
void turn_left(int PWM_Left, int PWM_Right) {
  analogWrite(Left_motor_pwm, 0);
  analogWrite(Right_motor_pwm, PWM_Right);
  digitalWrite(Left_motor_go, LOW);
  digitalWrite(Left_motor_back, LOW);
  digitalWrite(Right_motor_go, HIGH);
  digitalWrite(Right_motor_back, LOW);
}

// action 4
void turn_right(int PWM_Left, int PWM_Right) {
  analogWrite(Left_motor_pwm, PWM_Left);
  analogWrite(Right_motor_pwm, 0);
  digitalWrite(Left_motor_go, HIGH);
  digitalWrite(Left_motor_back, LOW);
  digitalWrite(Right_motor_go, LOW);
  digitalWrite(Right_motor_back, LOW);
}

// action 5
void spin_left(int PWM_Left, int PWM_Right) {
  analogWrite(Left_motor_pwm, PWM_Left);
  analogWrite(Right_motor_pwm, PWM_Right);
  digitalWrite(Left_motor_go, LOW);
  digitalWrite(Left_motor_back, HIGH);
  digitalWrite(Right_motor_go, HIGH);
  digitalWrite(Right_motor_back, LOW);
}

// action 6
void spin_right(int PWM_Left, int PWM_Right) {
  analogWrite(Left_motor_pwm, PWM_Left);
  analogWrite(Right_motor_pwm, PWM_Right);
  digitalWrite(Left_motor_go, HIGH);
  digitalWrite(Left_motor_back, LOW);
  digitalWrite(Right_motor_go, LOW);
  digitalWrite(Right_motor_back, HIGH);
}

