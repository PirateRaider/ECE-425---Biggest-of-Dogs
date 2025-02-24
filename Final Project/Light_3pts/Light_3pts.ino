/*
  Brooks-Lab2.ino
  Authors: Justin Roberts & Katie Collins
  Date Created: 1/9/24

  This program will introduce using the stepper motor library to create motion algorithms for the robot.
  The motions will be go to angle, go to goal, move in a circle, square, figure eight and basic movement (stop, forward, spin, reverse, turn)
  It will also include wired commmunication for serial monitor output.
  The primary functions created are:
  moveCircle - given the diameter in inches and direction of clockwise or counterclockwise, move the robot in a circle with that diameter
  moveFigure8 - given the diameter in inches, use the moveCircle() function with direction input to create a Figure 8
  moveSquare - given the length of the side in inches and the direction to turn each leg, create a square
  forward, reverse - both wheels move with same velocity, same direction
  pivot- one wheel stationary, one wheel moves forward or back
  spin - both wheels move with same velocity opposite direction
  turn - both wheels move with same direction different velocity
  stop -both wheels stationary

  Hardware Connections:
  Arduino pin mappings: https: // docs.arduino.cc/tutorials/giga-r1-wifi/cheat-sheet#pins
  A4988 Stepper Motor Driver Pinout: https: // www.pololu.com/product/1182 

  digital pin 48 - enable PIN on A4988 Stepper Motor Driver StepSTICK
  digital pin 50 - right stepper motor step pin
  digital pin 51 - right stepper motor direction pin
  digital pin 52 - left stepper motor step pin
  digital pin 53 - left stepper motor direction pin
  digital pin 13 - enable LED on microcontroller

  digital pin 5 - red LED in series with 220 ohm resistor
  digital pin 6 - green LED in series with 220 ohm resistor
  digital pin 7 - yellow LED in series with 220 ohm resistor

  digital pin 18 - left encoder pin
  digital pin 19 - right encoder pin

  digital pin 8 - back lidar sensor
  digital pin 9 - front lidar sensor
  digital pin 10 - right lidar sensor
  digital pin 11 - left lidar sensor
*/

#include <Arduino.h>       // include for PlatformIO Ide
#include <AccelStepper.h>  // include the stepper motor library
#include <MultiStepper.h>  // include multiple stepper motor library
#include "RPC.h"
#include <MedianFilterLib2.h>

#define redLED 5            // red LED for displaying states
#define grnLED 7            // green LED for displaying states
#define ylwLED 6            // yellow LED for displaying states
#define enableLED 13        // stepper enabled LED
#define stepperEnable 48    // stepper enable pin on stepStick
#define rtStepPin 50        // right stepper motor step pin
#define rtDirPin 51         // right stepper motor direction pin
#define ltStepPin 52        // left stepper motor step pin
#define ltDirPin 53         // left stepper motor direction pin
#define stepperEnTrue false // variable for enabling stepper motor
#define stepperEnFalse true // variable for disabling stepper motor
#define avg_speed 200       // default stepper motor speed
#define wander_speed 200    // default stepper motor speed
#define max_speed 1500      // default stepper motor speed
#define max_accel 10000     // maximum motor acceleration
#define ONE_SECOND 1000     // one second worth of delay in ms
#define LEFT 0              // left encoder
#define RIGHT 1             // right encoder
#define forward_inch_conversion 925.0 / 12.0  //  The following are to convert between stepper ticks and inches.
#define turn_conversion_factor 4000.0 / 44.0
#define spin_angle_conversion 5.7
#define radius_conversion 0.9
#define width_of_robot 7    //  These are defined because they are used multiple times
#define full_circle 360.0   //  in the math for functions like pivot and turn
#define OBJECT_DISTANCE 26
#define movementDistance 6
#define FRONT_LIDAR 2       // Sensor headings based on 0 degrees being forward
#define BACK_LIDAR 3
#define RIGHT_LIDAR 4
#define LEFT_LIDAR 5
#define frontLdr 9          // Pin definitions
#define backLdr 8
#define leftLdr 11
#define rightLdr 10
#define led 6
#define MIN_DIST 9
#define MAX_DIST 25
#define BLIND_FRONT 4
#define BLIND_REV 6
#define Photo_Offset 200
using namespace rtos;
Thread stateMachineThread;
AccelStepper stepperRight(AccelStepper::DRIVER, rtStepPin, rtDirPin);  // create instance of right stepper motor object (2 driver pins, low to high transition step pin 52, direction input pin 53 (high means forward)
AccelStepper stepperLeft(AccelStepper::DRIVER, ltStepPin, ltDirPin);   // create instance of left stepper motor object (2 driver pins, step pin 50, direction input pin 51)
MultiStepper steppers;                                                 // create instance to control multiple steppers at the same time
const int ltEncoder = 18;             // left encoder pin (Mega Interrupt pins 2,3 18,19,20,21)
const int rtEncoder = 19;             // right encoder pin (Mega Interrupt pins 2,3 18,19,20,21)
// resulting angle for aggregate direction
int angle = 0;
int numWalls = 0;
int dist_counter = 0;
int orientation = 0;
int environment_avg = 0;
int leftLightDetected;
int rightLightDetected;
int pauseTime = 2500;  // time before robot moves
int stepTime = 500;    // delay time between high and low on step pin
int lastSpeed[2] = { 0, 0 };          // variable to hold encoder speed (left, right)
int accumTicks[2] = { 0, 0 };         // variable to hold accumulated ticks since last reset
int sensorAngles[6] = {45, -45, 0, 180, 90, -90};
int leds[3] = { 5, 6, 7 };  // array of LED pin numbers
bool frontDetected = false; // wall detection booleans per sensor
bool backDetected = false;
bool leftDetected = false;
bool rightDetected = false; 
bool frontLeftDetected = false;
bool frontRightDetected = false;
bool wallDetected = false;
bool seen_box = false;
unsigned long lastWallSeen = 0;
volatile long encoder[2] = { 0, 0 };  // interrupt variable to hold number of encoder counts (left, right)

// a struct to hold lidar data
struct lidar {
  // this can easily be extended to contain sonar data as well
  int front;
  int back;
  int left;
  int right;
  bool frontDetected;
  bool backDetected;
  bool leftDetected;
  bool rightDetected;
  bool wallDetected;
  int numWallsDetected;
  int leftPhoto;
  int rightPhoto;
  // this defines some helper functions that allow RPC to send our struct (I found this on a random forum)
  MSGPACK_DEFINE_ARRAY(front, back, left, right, frontDetected, backDetected, leftDetected, 
    rightDetected, wallDetected, numWallsDetected, leftPhoto, rightPhoto);
  //https://stackoverflow.com/questions/37322145/msgpack-to-pack-structures https://www.appsloveworld.com/cplus/100/391/msgpack-to-pack-structures
} dist;

// function to set all stepper motor variables, outputs and LEDs
void init_stepper() {
  pinMode(rtStepPin, OUTPUT);                   // sets pin as output
  pinMode(rtDirPin, OUTPUT);                    // sets pin as output
  pinMode(ltStepPin, OUTPUT);                   // sets pin as output
  pinMode(ltDirPin, OUTPUT);                    // sets pin as output
  pinMode(stepperEnable, OUTPUT);               // sets pin as output
  digitalWrite(stepperEnable, stepperEnFalse);  // turns off the stepper motor driver
  pinMode(enableLED, OUTPUT);                   // set enable LED as output
  digitalWrite(enableLED, LOW);                 // turn off enable LED
  pinMode(redLED, OUTPUT);                      // set red LED as output
  pinMode(grnLED, OUTPUT);                      // set green LED as output
  pinMode(ylwLED, OUTPUT);                      // set yellow LED as output
  digitalWrite(redLED, HIGH);                   // turn on red LED
  digitalWrite(ylwLED, HIGH);                   // turn on yellow LED
  digitalWrite(grnLED, HIGH);                   // turn on green LED
  delay(pauseTime / 5);                         // wait 0.5 seconds
  digitalWrite(redLED, LOW);                    // turn off red LED
  digitalWrite(ylwLED, LOW);                    // turn off yellow LED
  digitalWrite(grnLED, LOW);                    // turn off green LED

  stepperRight.setMaxSpeed(max_speed);         // set the maximum permitted speed limited by processor and clock speed, no greater than 4000 steps/sec on Arduino
  stepperRight.setAcceleration(max_accel);     // set desired acceleration in steps/s^2
  stepperLeft.setMaxSpeed(max_speed);          // set the maximum permitted speed limited by processor and clock speed, no greater than 4000 steps/sec on Arduino
  stepperLeft.setAcceleration(max_accel);      // set desired acceleration in steps/s^2
  steppers.addStepper(stepperRight);           // add right motor to MultiStepper
  steppers.addStepper(stepperLeft);            // add left motor to MultiStepper
  digitalWrite(stepperEnable, stepperEnTrue);  // turns on the stepper motor driver
  digitalWrite(enableLED, HIGH);               // turn on enable LED
}

// read_lidars is the function used to get lidar data to the M7
struct lidar read_lidars() {
  return dist;
}

// reads a lidar given a pin
int read_lidar(int pin) {
  int d;
  int16_t t = pulseIn(pin, HIGH, 500000); // microseconds
  d = (t - 1000) * 3 / 40;
  if (t == 0 || t > 1850 || d < 0) { d = -1; } // -1 commonly represents illegal/incorrect values. 
  return d;
}


//  turn off all LEDs
void allOFF() {
  for (int i = 0; i < 3; i++) {
    digitalWrite(leds[i], LOW);
  }
}

/*
  Pivot takes in a direction (1 for RIGHT, 0 for LEFT) and an angle in degrees
  and computes the number of steps needed, then moves one stepper using move() and setSpeed()
  to pivot the robot
  move() is a library function for relative movement to set a target position
  stop() is a library function that causes the stepper to stop as quickly as possible
  runSpeedToPosition() is a library function that uses constant speed to achieve target posiiton, no blocking
*/
void pivot(int direction, int angle) {
  int leftPos = (int)(angle * spin_angle_conversion);   // left stepper relative position, from inches
  int rightPos = (int)(angle * spin_angle_conversion);  // right stepper relative position, from inches
  int leftSpd = avg_speed;                              // left stepper speed
  int rightSpd = avg_speed;                             // right stepper speed
  if (direction) {                                      // direction is 1, pivot right
    stepperRight.move(-1 * rightPos);                   // move right wheel to relative position
    stepperRight.setSpeed(-1 * rightSpd);               // set right stepper speed
    stepperLeft.move(0);                                // move left wheel to relative position
    stepperLeft.setSpeed(0);                            // set left stepper speed
  } else {                                              // direction is 0, pivot left
    stepperLeft.move(-1 * leftPos);                     // move left wheel to relative position
    stepperLeft.setSpeed(-1 * leftSpd);                 // set left stepper speed
    stepperRight.move(0);                               // move right wheel to relative position
    stepperRight.setSpeed(0);                           // set right stepper speed
  }
  runToStop();  // NOT Blocks until all are in position
  stop();                         // Stops both steppers, preventing odd stepper outputs
}

/*
  Spin takes in a direction (1 for RIGHT, 0 for LEFT) and an angle in degrees
  and computes the number of steps needed, then moves both steppers for an equal amount of steps in
  opposite directions using move() and setSpeed() to spin. This is the same general
  logic as pivot, but with the opposite direction for the opposing stepper.
  move() is a library function for relative movement to set a target position
  stop() is a library function that causes the stepper to stop as quickly as possible
  runSpeedToPosition() is a library function that uses constant speed to achieve target posiiton, no blocking
*/
void spin(int direction, int angle) {
  int leftPos = (int) (angle * spin_angle_conversion);     // left stepper relative position, from inches
  int rightPos = (int) (angle * spin_angle_conversion);    // right stepper relative position, from inches
  int leftSpd = (int) (avg_speed);             // left stepper speed
  int rightSpd = (int) (avg_speed);            // right stepper speed
  if (direction) {                                         // direction is 1, pivot right
    stepperLeft.move(-leftPos);                            // move left wheel to relative position
    stepperLeft.setSpeed(-leftSpd);                        // set left stepper speed
    stepperRight.move(rightPos);                           // move right wheel to relative position
    stepperRight.setSpeed(rightSpd);                       // set right stepper speed
  } else {                                                 // direction is 0, pivot left
    stepperRight.move(-rightPos);                          // move right wheel to relative position
    stepperRight.setSpeed(-rightSpd);                      // set right stepper speed
    stepperLeft.move(leftPos);                             // move left wheel to relative position
    stepperLeft.setSpeed(leftSpd);                         // set left stepper speed
  }
  steppers.runSpeedToPosition();                           // Blocks until all are in position
  stop();                                                  // Stops both steppers, preventing odd stepper outputs
}

/*
  Turn takes in a direction (1 for RIGHT, 0 for LEFT), a turn radius in inches, and an angle in degrees
  and computes the number of steps needed for each wheel, the speeds relative to each other, and the 
  amount of a complete circle the turn is executing. The wheels will travel different distances, 
  but the speeds are set so that the motors will stop at the same time. 
  move() is a library function for relative movement to set a target position
  stop() is a library function that causes the stepper to stop as quickly as possible
  runSpeedToPosition() is a library function that uses constant speed to achieve target posiiton, no blocking
  r_* denotes a radius
  c_* denotes a circumference
  step_* denotes a distance in stepper steps
*/
void turn(int direction, double radius, double angle) {
  double r_outer = radius + (width_of_robot / 2.0);           // find the outer wheel radius
  double r_inner = radius - (width_of_robot / 2.0);           // find the inner wheel radius
  double c_outer = 2 * M_PI * r_outer;                        // find the outer wheel circumference
  double c_inner = 2 * M_PI * r_inner;                        // find the inner wheel circumference
  double step_outer = c_outer * turn_conversion_factor;       // translate circumference to inches of path
  double step_inner = c_inner * turn_conversion_factor;       // translate circumference to inches of path
  double amount_of_circle = angle / full_circle;              // find the percent of the circle the turn is to travel
  double speed = avg_speed;                         // Move a little faster but not enough for slippage
  if (direction) {                                            // turn RIGHT
    stepperRight.move((int)(step_inner * amount_of_circle));  // set the right stepper to the inner radius
    stepperRight.setSpeed((int)(speed * r_inner / r_outer));  // set the right stepper speed as a fraction of the left stepper
    stepperLeft.move((int)(step_outer * amount_of_circle));   // set the left stepper to the outer radius
    stepperLeft.setSpeed((int)(speed));                       // set the left stepper speed
  } else {                                                    // turn LEFT
    stepperLeft.move((int)(step_inner * amount_of_circle));   // set the left stepper to the inner radius
    stepperLeft.setSpeed((int)(speed * r_inner / r_outer));   // set the left stepper speed as a fraction of the right stepper
    stepperRight.move((int)(step_outer * amount_of_circle));  // set the right stepper to the outer radius
    stepperRight.setSpeed((int)(speed));                      // set the right stepper speed
  }
  runToStop();  // NOT Blocks until all are in position
  stop();                         // Stops both steppers, preventing odd stepper outputs
}

/*
  runAway() checks the global variables to determine where the walls are
  detected and navigate away from the walls
  spin() rotates the robot about its center
  forward() moves the robot forward a specified distance in inches
*/
void forward(int distance) {
  int leftPos = (int) (distance * forward_inch_conversion);       // left stepper absolute position
  int rightPos = (int) (distance * forward_inch_conversion);      // right stepper absolute position
  int leftSpd = avg_speed;                                        // left stepper speed
  int rightSpd = avg_speed;                                       // right stepper speed
  // protection from backwards distances resulting in infinite runtimes with positive speed
  if (distance < 0) {                                             
    stepperLeft.move(leftPos);                                    // move left wheel to relative position
    stepperLeft.setSpeed(-1 * leftSpd);                           // set left stepper speed
    stepperRight.move(rightPos);                                  // move right wheel to relative position
    stepperRight.setSpeed(-1 * rightSpd);                         // set right stepper speed
  } else {
    stepperLeft.move(leftPos);                                    // move left wheel to relative position
    stepperLeft.setSpeed(leftSpd);                                // set left stepper speed
    stepperRight.move(rightPos);                                  // move right wheel to relative position
    stepperRight.setSpeed(rightSpd);                              // set right stepper speed
  }
  steppers.runSpeedToPosition();                                  // Blocks until all are in position
  stop();
}

/*
  The reverse() function calls the existing forward() function with a negative value
  to move the robot backwards, since forward() is defined relative to the robot's
  orientation and not the underlying functionality. The forward() function can 
  handle negative values. 
  forward() moves the robot forward a specified distance in inches
*/
void reverse(int distance) {
  forward(-1 * distance);  // call forward() with the reverse distance
}

/*
  The stop function takes in no arguments and stops both the left and right steppers. 
  It uses the Multistepper library to achieve this. 
  stop() is a library function that causes the stepper to stop as quickly as possible
  allOFF() turns off all LEDs
*/
void stop() {
  stepperRight.stop();  // stop right stepper
  stepperLeft.stop();   // stop left stepper
}

/*function to run both wheels continuously at a speed*/
void runAtSpeed ( void ) {
  while (stepperRight.runSpeed() || stepperLeft.runSpeed()) {
    // add check in here for collisions?
  }
}

/*
  This function, runToStop(), will run the robot until the target is achieved and
  then stop it
*/
void runToStop(void) {
  int runNow = 1;
  int rightStopped = 0;
  int leftStopped = 0;

  while (runNow) {
    if (!stepperRight.run()) {
      rightStopped = 1;
      stepperRight.stop();//stop right motor
    }
    if (!stepperLeft.run()) {
      leftStopped = 1;
      stepperLeft.stop();//stop ledt motor
    }
    if (rightStopped && leftStopped) {
      runNow = 0;
    }
  }
}

/*function to run both wheels to a position at speed*/
void runAtSpeedToPosition() {
  stepperRight.runSpeedToPosition();
  stepperLeft.runSpeedToPosition();
}

//set up the M7 to be the client and run the state machine
void setupM7() {
  Serial.println("M7 setup");
  delay(1000);
  struct lidar data = readLidar(0);
  environment_avg = data.leftPhoto + data.rightPhoto;
  data = readLidar(0);
  environment_avg = environment_avg + data.leftPhoto + data.rightPhoto;
  data = readLidar(0);
  environment_avg = environment_avg + data.leftPhoto + data.rightPhoto;
  data = readLidar(0);
  environment_avg = environment_avg + data.leftPhoto + data.rightPhoto;
  data = readLidar(0);
  environment_avg = environment_avg + data.leftPhoto + data.rightPhoto;
  environment_avg = (int) (environment_avg / 10.0);
  Serial.println(environment_avg);
}

//set up the M4 to be the server for the sensors data
void setupM4() {
  RPC.bind("read_lidars", read_lidars);  // bind a method to return the lidar data all at once
}

/*
  readSensors() gathers the most recent sensor readings on teh M4 core and normalizes it
  so that -1 is "no reading/too far away" and 0-31 can be assumed valid. 
  read_lidar(pin) reads the lidar sensor attached to the passed-in pin
  read_sonar(pin) reads the sonar sensor attached to the passed-in pin
*/
void readSensors() {
  dist.front = read_lidar(frontLdr);
  dist.front = dist.front > 0 ? dist.front - 1 : dist.front;
  dist.back = read_lidar(backLdr);
  dist.back = dist.back > 0 ? dist.back - 1 : dist.back;
  dist.left = read_lidar(leftLdr);
  dist.left = dist.left > 0 ? dist.left - 26 : dist.left;  // -26
  dist.right = read_lidar(rightLdr);
  dist.right = dist.right > 0 ? dist.right - 26 : dist.right;  // -26
  dist.frontDetected = dist.front <= OBJECT_DISTANCE && dist.front >= 0;
  dist.backDetected = dist.back <= OBJECT_DISTANCE && dist.back >= 0;
  dist.rightDetected = dist.right <= OBJECT_DISTANCE && dist.right >= 0;
  dist.leftDetected = dist.left <= OBJECT_DISTANCE && dist.left >= 0;
  dist.wallDetected = dist.frontDetected || dist.backDetected || dist.rightDetected || dist.leftDetected;
  dist.numWallsDetected = (int)dist.frontDetected + (int)dist.backDetected + (int)dist.rightDetected + (int)dist.leftDetected;
  dist.leftPhoto = analogRead(A0);
  dist.rightPhoto = analogRead(A1);
  delay(20);
}

/*
  readLidar() moves the most recent sensor reading from the M4 to the M7 core and normalizes it
  so that -1 is "no reading/too far away" and 0-31 can be assumed valid. 
  read_lidar(pin) reads the lidar sensor attached to the passed-in pin
*/
struct lidar readLidar(int debug) {
  struct lidar data = RPC.call("read_lidars").as<struct lidar>();
  leftLightDetected = abs(data.leftPhoto - environment_avg) > Photo_Offset;
  rightLightDetected = abs(data.rightPhoto - environment_avg) > Photo_Offset;
  if (debug) {
    // print lidar data
    Serial.print("lidar: ");
    Serial.print(data.front);
    Serial.print(", ");
    Serial.print(data.back);
    Serial.print(", ");
    Serial.print(data.left);
    Serial.print(", ");
    Serial.println(data.right);
    Serial.print("bools:  ");
    Serial.print(data.frontDetected);
    Serial.print(",  ");
    Serial.print(data.backDetected);
    Serial.print(",  ");
    Serial.print(data.rightDetected);
    Serial.print(",  ");
    Serial.print(data.leftDetected);
    Serial.print(",  ");
    Serial.print(data.wallDetected);
    Serial.print(",  ");
    Serial.println(data.numWallsDetected);
    Serial.print("Photoresistors:  ");
    Serial.print(data.leftPhoto);
    Serial.print(",  ");
    Serial.println(data.rightPhoto);
    Serial.print("Bools:  ");
    Serial.print(leftLightDetected);
    Serial.print(",  ");
    Serial.println(rightLightDetected);
  }
  return data;
}

/*
  randomWander() wanders with no care for the outside environment, using 
  the basic level-0 movement functions. 
  forward() moves the robot forward a specified distance in inches  reverse() 
  spin() rotates the robot about its center
  turn() rotates the robot about its center while moving forward
  pivot() rotates the robot about its center with only one wheel
*/
void randomWander() {
  long initial_choice = random(0, 6);
  digitalWrite(grnLED, HIGH);
  switch (initial_choice) {
    case 0:
      forward(random(8) + 5);
      break;
    case 1:
      reverse(random(8) + 5);
      break;
    case 2:
      spin(random(2), random(315) + 45);
      break;
    case 3:
      turn(random(2), random(8) + 5, random(315) + 45);
      break;
    case 4:
      pivot(random(2), random(315) + 45);
      break;
    default:
      stop();
      break;
  }
  delay(ONE_SECOND);
}

// collide() prevents collisions by running up to an object
// and stopping at a specified distance
void collide(struct lidar data) {
  while (data.front >= OBJECT_DISTANCE) {
    stepperRight.setSpeed(wander_speed);
    stepperLeft.setSpeed(wander_speed);
    runAtSpeed();
    Serial.println("In collide");
    data = readLidar(0);
  }
}

/*
  runAway() checks the global variables to determine where the walls are
  detected and navigate away from the walls
  spin() rotates the robot about its center
  forward() moves the robot forward a specified distance in inches
*/
void runAway() {
  if (frontDetected && backDetected && leftDetected && rightDetected) {
    stop();
  } else if (frontDetected && backDetected && leftDetected) {
    // turn right, move forward
    spin(LEFT, 90);
    forward(movementDistance);
  } else if (frontDetected && backDetected && rightDetected) {
    // turn left, move forward
    spin(RIGHT, 90);
    forward(movementDistance);
  } else if (frontDetected && leftDetected && rightDetected) {
    // backwards
    forward(-1 * movementDistance);
  } else if (backDetected && leftDetected && rightDetected) {
    // forwards
    forward(movementDistance);
  } else if (frontDetected && backDetected) {
    // turn left
    spin(RIGHT, 90);
    forward(movementDistance);
  } else if (frontDetected && leftDetected) {
    // reverse
    forward(-1 * movementDistance);
  } else if (frontDetected && rightDetected) {
    // reverse
    forward(-1 * movementDistance);
  } else if (backDetected && rightDetected) {
    // forward
    forward(movementDistance);
  } else if (backDetected && leftDetected) {
    // forward
    forward(movementDistance);
  } else if (leftDetected && rightDetected) {
    // forward
    forward(movementDistance);
  } else if (frontDetected) {
    // reverse
    forward(-1 * movementDistance);
  } else if (backDetected) {
    // forward
    forward(movementDistance);
  } else if (rightDetected) {
    // left
    spin(RIGHT, 90);
    forward(movementDistance);
  } else if (leftDetected) {
    // right
    spin(LEFT, 90);
    forward(movementDistance);
  }
  // if (frontDetected && backDetected && leftDetected && rightDetected) {
  //   stop();
  //   Serial.println("I see everything.");
  // } else if (frontDetected && backDetected) {
  //   spin(LEFT, 90);
  //   forward(6);
  // } else if (frontDetected) {
  //   forward(-6);
  // } else {
  //   forward(6);
  // }
}

int mapPhoto(int photoReading) {
  float output = (photoReading - 100 > 0) ? photoReading - 100 : 0;
  output = output / 900;
  output = output * 500;
  output = output + 100;
  return (int) output;
}

//setup function with infinite loops to send and receive sensor data between M4 and M7
void setup() {
  RPC.begin();
  // begin serial interface
  Serial.begin(9600);
  init_stepper();

  delay(1000);
  if (HAL_GetCurrentCPUID() == CM7_CPUID) {
    // if on M7 CPU, run M7 setup & loop
    setupM7();
    // continues on to loop() {} below
  } else {
    // if on M4 CPU, run M4 setup & loop
    setupM4();
    while(1) {readSensors();}
  }
}

// main loop, runs infintely on the M7 core. 
void loop() {
  // read lidar data from struct
  struct lidar data = readLidar(0);
  Serial.println(data.rightPhoto);
  Serial.println(data.leftPhoto);
  Serial.println();
  delay(500);
  

  // follow
  if (data.front < 0) {
     digitalWrite(ylwLED, LOW);
     digitalWrite(grnLED, HIGH);
     digitalWrite(redLED, LOW);
     // stop();
     randomWander();
  } else if (data.front >= 20) {
    digitalWrite(ylwLED, LOW);
    digitalWrite(grnLED, LOW);
    digitalWrite(redLED, HIGH);
    // collide(data);
    forward(2);
  } else if (data.front <= 8) { // data.front is between ~25 and 15
    digitalWrite(ylwLED, HIGH);
    digitalWrite(grnLED, HIGH);
    digitalWrite(redLED, LOW);
    // follow(data);
    forward(-2);
  } else {
    stop();
  }
}
