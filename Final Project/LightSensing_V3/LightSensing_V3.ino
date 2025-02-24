/*
***********************************
  RobotIntro.ino
  Donald & Ben 2/23/25

  This program has basic motion algorithms for the robot.
  This includes goToAngle, goForward, goToGoal, circle, square, figureEight, pivot, spin, turn, and stop.
  
  The primary functions created are:
  moveCircle - given the diameter in cm, direction of rotation, and arc length, the robot will move along in an arc with that given diameter, in the specified direction
  moveFigure8 - given the diameter in cm, use the moveCircle() function with an arc length of 360 to create two back to back circles.
  forward, reverse - both wheels move with same velocity, same direction
  pivot- one wheel stationary, one wheel moves forward or back
  spin - both wheels move with same velocity opposite direction
  turn - both wheels move with same direction different velocity
  stop - both wheels stationary

  Interrupts
  https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/
  https://www.arduino.cc/en/Tutorial/CurieTimer1Interrupt
  https://playground.arduino.cc/code/timer1
  https://playground.arduino.cc/Main/TimerPWMCheatsheet
  http://arduinoinfo.mywikis.net/wiki/HOME

  Hardware Connections:
  Arduino pin mappings: https://docs.arduino.cc/tutorials/giga-r1-wifi/cheat-sheet#pins
  A4988 Stepper Motor Driver Pinout: https://www.pololu.com/product/1182 

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

  Wheel Diameter = 8.5 cm, 26.7035375555 cm circumference
  Wheel Distance = 21.5 cm
  Stepper motor = 0.45 degree steps
*/

//includew all necessary libraries
#include <Arduino.h>//include for PlatformIO Ide
#include <AccelStepper.h>//include the stepper motor library
#include <MultiStepper.h>//include multiple stepper motor library
#include <RPC.h>
#include "MedianFilterLib2.h"

//state LEDs connections
#define redLED 5            //red LED for displaying states
#define grnLED 6            //green LED for displaying states
#define ylwLED 7            //yellow LED for displaying states
#define enableLED 13        //stepper enabled LED
int leds[3] = {5,6,7};      //array of LED pin numbers

//define motor pin numbers
#define stepperEnable 48    //stepper enable pin on stepStick 
#define rtStepPin 50 //right stepper motor step pin 
#define rtDirPin 51  // right stepper motor direction pin 
#define ltStepPin 52 //left stepper motor step pin 
#define ltDirPin 53  //left stepper motor direction pin 

AccelStepper stepperRight(AccelStepper::DRIVER, rtStepPin, rtDirPin);//create instance of right stepper motor object (2 driver pins, low to high transition step pin 52, direction input pin 53 (high means forward)
AccelStepper stepperLeft(AccelStepper::DRIVER, ltStepPin, ltDirPin);//create instance of left stepper motor object (2 driver pins, step pin 50, direction input pin 51)
MultiStepper steppers;//create instance to control multiple steppers at the same time

#define stepperEnTrue false //variable for enabling stepper motor
#define stepperEnFalse true //variable for disabling stepper motor
#define max_speed 1500 //maximum stepper motor speed
#define max_accel 10000 //maximum motor acceleration

int pauseTime = 2500;   //time before robot moves
int stepTime = 500;     //delay time between high and low on step pin
int wait_time = 100;   //delay for printing data
double angleConversion = 6;
double distanceConversion = 30;
int closeIRDistance = 15;
int minIRDistance = 2;
int changeEveryOther = 1; // boolean to changeEveryOther every other instance

//define encoder pins
#define LEFT 0        //left encoder
#define RIGHT 1       //right encoder
const int ltEncoder = 18;        //left encoder pin (Mega Interrupt pins 2,3 18,19,20,21)
const int rtEncoder = 19;        //right encoder pin (Mega Interrupt pins 2,3 18,19,20,21)
volatile long encoder[2] = {0, 0};  //interrupt variable to hold number of encoder counts (left, right)
int lastSpeed[2] = {0, 0};          //variable to hold encoder speed (left, right)
int accumTicks[2] = {0, 0};         //variable to hold accumulated ticks since last reset

// define Sensor pins
#define frontLdr 8
#define backLdr 9
#define leftLdr 10
#define rightLdr 11
#define leftSnr 4
#define rightSnr 3
#define leftLight A1
#define rightLight A0

// Global Variables
String state = "random";  // robot starts in Random Wander
int leftDetect;           // Boolean for if light is detected left
int rightDetect;          // Boolean for if light is detected right
int photoOffset = 200;    // Offset for the photonresistors before they activate/detect light
int environmentAverage = 0; // holds environmental average when initializing
int baseSpeed = 500;        // base speed for the robot when trakcing light

// Structures
// a struct to hold lidar data
struct lidar {
  // this can easily be extended to contain sonar data as well
  int front;
  int back;
  int left;
  int right;
  // this defines some helper functions that allow RPC to send our struct (I found this on a random forum)
  MSGPACK_DEFINE_ARRAY(front, back, left, right);  //https://stackoverflow.com/questions/37322145/msgpack-to-pack-structures https://www.appsloveworld.com/cplus/100/391/msgpack-to-pack-structures
} dist;


// a struct to hold sonar data
struct sonar {
  // this can easily be extended to contain sonar data as well
  int left;
  int right;
  // this defines some helper functions that allow RPC to send our struct (I found this on a random forum)
  MSGPACK_DEFINE_ARRAY(left, right);  //https://stackoverflow.com/questions/37322145/msgpack-to-pack-structures https://www.appsloveworld.com/cplus/100/391/msgpack-to-pack-structures
} dist2;

struct light {
  int left;
  int right;
  // defines helper functions
  MSGPACK_DEFINE_ARRAY(left, right);
} dist3;

struct lidar read_lidars() {
  return dist;
}

// read_lidars is the function used to get lidar data to the M7
struct sonar read_sonars() {
  return dist2;
}

struct light read_lights() {
  return dist3;
}

int frontFilterFunction() {
  struct lidar data = RPC.call("read_lidars").as<struct lidar>();
  
  if(frontFilter != data.front) {
    frontLdrFilt.AddValue(data.front);
    frontFilter = data.front;
    Serial.println(data.front);
  }
  
  return frontLdrFilt.GetFiltered();
}

int leftFilterFunction() {
  struct lidar data = RPC.call("read_lidars").as<struct lidar>();
  
  if(frontFilter != data.left) {
    leftLdrFilt.AddValue(data.left);
    leftFilter = data.left;
    Serial.println(data.left);
  }
  
  return leftLdrFilt.GetFiltered();
}

int rightFilterFunction() {
  struct lidar data = RPC.call("read_lidars").as<struct lidar>();
  
  if(rightFilter != data.right) {
    rightLdrFilt.AddValue(data.right);
    rightFilter = data.right;
    Serial.println(data.right);
  }
  
  return rightLdrFilt.GetFiltered();
}

int backFilterFunction() {
  struct lidar data = RPC.call("read_lidars").as<struct lidar>();
  
  if(backFilter != data.back) {
    backLdrFilt.AddValue(data.back);
    backFilter = data.back;
    Serial.println(data.back);
  }
  
  return backLdrFilt.GetFiltered();
}

void frontFilterClear() {

}

// helper functions
// reads a lidar given a pin
int read_lidar(int pin) {
  int d;
  int16_t t = pulseIn(pin, HIGH);
  d = (t - 1000) * 3 / 40;
  if (t == 0 || t > 1850 || d < 0) { d = 0; }
  return d;
}

// reads a sonar given a pin
int read_sonar(int pin) {
  float velocity ((331.5 + 0.6 * (float)(20)) * 100 / 1000000.0);
  uint16_t distance, pulseWidthUs;

  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
  digitalWrite(pin, HIGH);            //Set the trig pin High
  delayMicroseconds(10);              //Delay of 10 microseconds
  digitalWrite(pin, LOW);             //Set the trig pin Low
  pinMode(pin, INPUT);                //Set the pin to input mode
  pulseWidthUs = pulseIn(pin, HIGH);  //Detect the high level time on the echo pin, the output high level time represents the ultrasonic flight time (unit: us)
  distance = pulseWidthUs * velocity / 2.0;
  if (distance < 0 || distance > 50) { distance = 0; }
  return distance;
} 

// reads a light given a pin
int read_light(int pin) {

  pinMode(pin, OUTPUT);

  return analogRead(pin);
} 

//interrupt function to count left encoder tickes
void LwheelSpeed()
{
  encoder[LEFT] ++;  //count the left wheel encoder interrupts
}

void readSensors() {
  dist3.left = analogRead(A0);
  dist3.right = analogRead(A1);
  dist.front = read_lidar(8);
  dist.back = read_lidar(9);
  dist.left = read_lidar(10);
  dist.right = read_lidar(11);
}

//interrupt function to count right encoder ticks
void RwheelSpeed()
{
  encoder[RIGHT] ++; //count the right wheel encoder interrupts
}

void allOFF(){
  for (int i = 0;i<3;i++){
    digitalWrite(leds[i],LOW);
  }
}

//function to set all stepper motor variables, outputs and LEDs
void init_stepper(){
  pinMode(rtStepPin, OUTPUT);//sets pin as output
  pinMode(rtDirPin, OUTPUT);//sets pin as output
  pinMode(ltStepPin, OUTPUT);//sets pin as output
  pinMode(ltDirPin, OUTPUT);//sets pin as output
  pinMode(stepperEnable, OUTPUT);//sets pin as output
  digitalWrite(stepperEnable, stepperEnFalse);//turns off the stepper motor driver
  pinMode(enableLED, OUTPUT);//set enable LED as output
  digitalWrite(enableLED, LOW);//turn off enable LED
  pinMode(redLED, OUTPUT);//set red LED as output
  pinMode(grnLED, OUTPUT);//set green LED as output
  pinMode(ylwLED, OUTPUT);//set yellow LED as output
  digitalWrite(redLED, HIGH);//turn on red LED
  digitalWrite(ylwLED, HIGH);//turn on yellow LED
  digitalWrite(grnLED, HIGH);//turn on green LED
  delay(pauseTime / 5); //wait 0.5 seconds
  digitalWrite(redLED, LOW);//turn off red LED
  digitalWrite(ylwLED, LOW);//turn off yellow LED
  digitalWrite(grnLED, LOW);//turn off green LED

  stepperRight.setMaxSpeed(max_speed);//set the maximum permitted speed limited by processor and clock speed, no greater than 4000 steps/sec on Arduino
  stepperRight.setAcceleration(max_accel);//set desired acceleration in steps/s^2
  stepperLeft.setMaxSpeed(max_speed);//set the maximum permitted speed limited by processor and clock speed, no greater than 4000 steps/sec on Arduino
  stepperLeft.setAcceleration(max_accel);//set desired acceleration in steps/s^2
  steppers.addStepper(stepperRight);//add right motor to MultiStepper
  steppers.addStepper(stepperLeft);//add left motor to MultiStepper
  digitalWrite(stepperEnable, stepperEnTrue);//turns on the stepper motor driver
  digitalWrite(enableLED, HIGH);//turn on enable LED
}

//function prints Lidar Data
void print_lidar() {
  // read lidar data from struct
  struct lidar data = RPC.call("read_lidars").as<struct lidar>();

  // print lidar data
  Serial.println("lidar (front, back, left, right): ");
  Serial.print(data.front);
  Serial.print(", ");
  Serial.print(data.back);
  Serial.print(", ");
  Serial.print(data.left);
  Serial.print(", ");
  Serial.print(data.right);
  Serial.println();
}
  
/*function to run both wheels to a position at speed*/
void runAtSpeedToPosition() {
  stepperRight.runSpeedToPosition();
  stepperLeft.runSpeedToPosition();
}

/*function to run both wheels continuously at a speed*/
void runAtSpeed ( void ) {
  while (stepperRight.runSpeed() || stepperLeft.runSpeed()) {
  }
}

/*
  goToAngle takes in an angle in degrees, and moves the robot that many degrees, moving both motors at equal and opposite speeds so as to stay in position
  It uses an arbitrary speed of 500, and the function's stepperLeft/stepperRight to control each wheel independently
  If the angle is negative, the robot will move in the opposite direction.
*/
void goToAngle(int angle) {
  int leftPos = -angle*angleConversion;//right motor absolute position
  int rightPos = (angle*angleConversion);//left motor absolute position
  int leftSpd = -500;//right motor speed
  int rightSpd = 500; //left motor speed

  if(angle < 0) {
    rightSpd = -rightSpd;
    leftSpd = -leftSpd;
  }

  stepperLeft.move(leftPos);//move left wheel to relative position
  stepperRight.move(rightPos);//move right wheel to relative position

  stepperLeft.setSpeed(leftSpd);//set left motor speed
  stepperRight.setSpeed(rightSpd);//set right motor speed

  steppers.runSpeedToPosition(); // Blocks until all are in position
}

/*
  This function moves the robot a certain distance in cm, moving both motors at equal speed in one direction.
  It uses an arbitrary speed of 500, and the function's stepperLeft/stepperRight to control each wheel independently
  If the distance is negative, the robot will move backwards. 
*/
void goForward(double distance) {
  int leftPos = distance*distanceConversion;//right motor absolute position
  int rightPos = distance*distanceConversion;//left motor absolute position
  int leftSpd = 500;//right motor speed
  int rightSpd = 500; //left motor speed

  if(distance < 0) {
    rightSpd = -rightSpd;
    leftSpd = -leftSpd;
  }

  stepperLeft.move(leftPos);//move left wheel to relative position
  stepperRight.move(rightPos);//move right wheel to relative position

  stepperLeft.setSpeed(leftSpd);//set left motor speed
  stepperRight.setSpeed(rightSpd);//set right motor speed

  steppers.runSpeedToPosition(); // Blocks until all are in position
}

/*
  stop sets both motor's speeds to 0, causing the robot to stop in place.
*/
void stop() {
  stepperLeft.setSpeed(0);
  stepperRight.setSpeed(0);

  runAtSpeed();
}

/*
  randomWander picks a random distance between 10 and 30, and a random angle between -180 and 180, and then turns that angle, and moves that distance.
  It uses the arbitrary speeds of the goForward and goToAngle functions.
  This is combined with the other states to create higher level behaviours.
*/
void randomWander() {

  digitalWrite(redLED, LOW);//turn off red LED
  digitalWrite(ylwLED, LOW);//turn off yellow LED
  digitalWrite(grnLED, HIGH);//turn on green LED
  double randAngle = random(-180,180);
  double randDist = random(10,30);

  goToAngle(randAngle);
  delay(1000);

  goForward(randDist);
  delay(1000); 
}

/*
  shyKid takes in inputs from all the lidar sensors, and outputs speeds to the motors to have the robot avoid obstacles.
  Currently if it detects all walls, the robot sits in place, if the robot senses something in front, it will turn away.
  If the robot senses nothing in front, it moves away proportional from the wall/something in front.
  If nothing is detected, the robot just moves forward. 
  If front & back is detected, but not the sides, the robot will randomly turn 90 degrees to the left or right.
  
  This function can be combined with other to amke higher level behaviour.
*/
void shyKid() {

  // read lidar data from struct
  readSensors();
  int rightSpd = 250;
  int leftSpd = 250;
  int gain = 40;
  int delay_int = 1500;

  if(dist.front > minIRDistance && dist.left > minIRDistance && dist.right > minIRDistance && dist.back > minIRDistance) {
    //detecting all Walls
    leftSpd = 0;
    rightSpd = 0;
    stepperLeft.setSpeed(leftSpd); //set left motor speed
    stepperRight.setSpeed(rightSpd); //set right motor speed
    stepperLeft.runSpeed();
    stepperRight.runSpeed();
    delay(delay_int);

  } else if (dist.front > minIRDistance && dist.left > minIRDistance && dist.right > minIRDistance && dist.back < minIRDistance) {
    // detecting Front, Left, & Right
    goToAngle(180);
    delay(delay_int);
    readSensors();

  } else if (dist.front > minIRDistance && dist.left > minIRDistance && dist.right < minIRDistance) {
    // detecting Front & Left
    goToAngle(-90);
    delay(delay_int);

  } else if (dist.front > minIRDistance && dist.left < minIRDistance && dist.right > minIRDistance) {
    //Detecting Front & Right
    goToAngle(90);
    delay(delay_int);

  } else if (dist.front > minIRDistance && dist.left < minIRDistance && dist.right < minIRDistance && dist.back < minIRDistance) {
    // Just Detecting Front
    int backSpeed = (15 - dist.front) * 15 + 50;

    rightSpd = -backSpeed;
    leftSpd = -backSpeed;

    stepperLeft.setSpeed(leftSpd);//set left motor speed
    stepperRight.setSpeed(rightSpd);//set right motor speed
    stepperLeft.runSpeed();
    stepperRight.runSpeed();

  } else if (dist.front > minIRDistance && dist.left < minIRDistance && dist.right < minIRDistance && dist.back > minIRDistance) {
    // Detecting Front & Back
    if (changeEveryOther ==1) {
      goToAngle(90);
      delay(delay_int);
      changeEveryOther = 0;
    } else {
      goToAngle(90);
      delay(delay_int);
      changeEveryOther = 1;
    }
  
  } else if (dist.front < minIRDistance && dist.left < minIRDistance && dist.right < minIRDistance && dist.back < minIRDistance) {
    // Detects Nothing
    leftSpd = 0;
    rightSpd = 0;
    stepperLeft.setSpeed(leftSpd);//set left motor speed
    stepperRight.setSpeed(rightSpd);//set right motor speed
    stepperLeft.runSpeed();
    stepperRight.runSpeed();

  } else {
    // Detecting Nothing in Front
    if (dist.left > minIRDistance && dist.left < closeIRDistance) {
      leftSpd = leftSpd + (closeIRDistance-dist.left) * gain;
    }

    if (dist.right > minIRDistance && dist.right < closeIRDistance) {
      rightSpd = rightSpd + (closeIRDistance-dist.left) * gain;
    }

    stepperLeft.setSpeed(leftSpd);//set left motor speed
    stepperRight.setSpeed(rightSpd);//set right motor speed
    stepperLeft.runSpeed();
    stepperRight.runSpeed();
  }
}

/*
  lightFollow takes in inputs from the light sensors, and proportionally applies it to the motors to have the robot track the light source, and move backwards if the robot is too close.
  It was found that a distance of around 3 inches is obtained before the robot will oscillate back and forth.
*/

void lightFollow() {
  leftSpd = (baseSpeed * (800 - dist3.left)) / 1024;
  rightSpd = (baseSpeed * (800 - dist3.right)) / 1024;

  stepperLeft.setSpeed(leftSpd);//set left motor speed
  stepperRight.setSpeed(rightSpd);//set right motor speed
  runAtSpeed();
}

// Setup for Multi-Core
//set up the M4 to be the server for the sensors data
void setupM4() {
  RPC.bind("read_lidars", read_lidars);  // bind a method to return the lidar data all at once
  RPC.bind("read_sonars", read_sonars);  // bind a method to return the lidar data all at once
  RPC.bind("read_lights", read_lights);  // bind a method to return the light data at once
}

//poll the M4 to read the data
void loopM4() {
  // update the struct with current lidar data
  dist3.left = read_light(A0);
  dist3.right = read_light(A1);
}

//set up the M7 to be the client and run the state machine
void setupM7() {

  delay(500);

  // begin serial interface
  int baudrate = 9600; //serial monitor baud rate'
  init_stepper(); //set up stepper motor
  
  attachInterrupt(digitalPinToInterrupt(ltEncoder), LwheelSpeed, CHANGE);    //init the interrupt mode for the left encoder
  attachInterrupt(digitalPinToInterrupt(rtEncoder), RwheelSpeed, CHANGE);   //init the interrupt mode for the right encoder

  Serial.begin(baudrate);     //start serial monitor communication
  Serial.println("Robot starting...Put ON TEST STAND");
  delay(pauseTime); //always wait 2.5 seconds before the robot moves

  
  for (int i = 0; i < 5; i++) {
    readSensors();
    photoOffset = photoOffset + dist3.left + dist3.right;
  }

  photoOffset = photoOffset/10;

  Serial.print("Photo-Offset is: ");
  Serial.println(photoOffset);
  Serial.println();
  
}

//read sensor data from M4 and write to M7
void loopM7() {
  readSensors();
  leftDetect = dist3.left - environmentAverage > photoOffset;
  rightDetect = dist3.right - environmentAverage > photoOffset;
  if(leftDetect == 1 || rightDetect == 1) {
    state = "lightFollow";
    lightFollow();
    digitalWrite(redLED, HIGH);//turn off red LED
    digitalWrite(ylwLED, LOW);//turn off yellow LED
    digitalWrite(grnLED, LOW);//turn on green LED
  } else if ((dist.left > 1 && dist.left < 10) || (dist.right > 1 && dist.right < 10) || (dist.front > 1 && dist.front < 10) || (dist.back > 1 && dist.back < 10)) {
    shyKid();
    state = "avoid";
    digitalWrite(redLED, LOW);//turn off red LED
    digitalWrite(ylwLED, HIGH);//turn off yellow LED
    digitalWrite(grnLED, LOW);//turn on green LED
  } else {
    state = "random";
    randomWander();
    digitalWrite(redLED, LOW);//turn off red LED
    digitalWrite(ylwLED, LOW);//turn off yellow LED
    digitalWrite(grnLED, HIGH);//turn on green LED
  }

  Serial.println(dist3.left);
  Serial.println(dist3.right);
  Serial.println();
}

//setup function with infinite loops to send and receive sensor data between M4 and M7
void setup() {
  RPC.begin();
  if (HAL_GetCurrentCPUID() == CM7_CPUID) {
    // if on M7 CPU, run M7 setup & loop
    setupM7();
    while (1) loopM7();
  } else {
    // if on M4 CPU, run M7 setup & loop
    setupM4();
    while (1) loopM4();
  }
}

// loop() is never called as setup() never returns
// this may need to be modified to run th estate machine.
// consider usingnamespace rtos Threads as seen in previous example
void loop() {}
