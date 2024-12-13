
/*
  NOTE:
   THIS IS THE STANDARD FOR HOW TO PROPERLY COMMENT CODE
   Header comment has program, name, author name, date created
   Header comment has brief description of what program does
   Header comment has list of key functions and variables created with decription
   There are sufficient in line and block comments in the body of the program
   Variables and functions have logical, intuitive names
   Functions are used to improve modularity, clarity, and readability
***********************************
  RobotIntro.ino
  Carlotta Berry 11.21.16

  This program will introduce using the stepper motor library to create motion algorithms for the robot.
  The motions will be go to angle, go to goal, move in a circle, square, figure eight and teleoperation (stop, forward, spin, reverse, turn)
  It will also include wireless commmunication for remote control of the robot by using a game controller or serial monitor.
  The primary functions created are
  moveCircle - given the diameter in inches and direction of clockwise or counterclockwise, move the robot in a circle with that diameter
  moveFigure8 - given the diameter in inches, use the moveCircle() function with direction input to create a Figure 8
  forward, reverse - both wheels move with same velocity, same direction
  pivot- one wheel stationary, one wheel moves forward or back
  spin - both wheels move with same velocity opposite direction
  turn - both wheels move with same direction different velocity
  stop -both wheels stationary

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

  INSTALL THE LIBRARY
  AccelStepper Library: https://www.airspayce.com/mikem/arduino/AccelStepper/
  
  Sketch->Include Library->Manage Libraries...->AccelStepper->Include
  OR
  Sketch->Include Library->Add .ZIP Library...->AccelStepper-1.53.zip
  See PlatformIO documentation for proper way to install libraries in Visual Studio
*/

// Wheel Diamter = 8.5 cm, 26.7035375555 cm circumference
// Wheel Distance = 21.5 cm, 2.52941176471 conversion
// Stepper motor = 0.45 degree steps

//includew all necessary libraries
#include <Arduino.h>//include for PlatformIO Ide
#include <AccelStepper.h>//include the stepper motor library
#include <MultiStepper.h>//include multiple stepper motor library

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

//define encoder pins
#define LEFT 0        //left encoder
#define RIGHT 1       //right encoder
const int ltEncoder = 18;        //left encoder pin (Mega Interrupt pins 2,3 18,19,20,21)
const int rtEncoder = 19;        //right encoder pin (Mega Interrupt pins 2,3 18,19,20,21)
volatile long encoder[2] = {0, 0};  //interrupt variable to hold number of encoder counts (left, right)
int lastSpeed[2] = {0, 0};          //variable to hold encoder speed (left, right)
int accumTicks[2] = {0, 0};         //variable to hold accumulated ticks since last reset


// Helper Functions

//interrupt function to count left encoder tickes
void LwheelSpeed()
{
  encoder[LEFT] ++;  //count the left wheel encoder interrupts
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

//function prints encoder data to serial monitor
void print_encoder_data() {
  static unsigned long timer = 0;                           //print manager timer
  if (millis() - timer > 100) {                             //print encoder data every 100 ms or so
    lastSpeed[LEFT] = encoder[LEFT];                        //record the latest left speed value
    lastSpeed[RIGHT] = encoder[RIGHT];                      //record the latest right speed value
    accumTicks[LEFT] = accumTicks[LEFT] + encoder[LEFT];    //record accumulated left ticks
    accumTicks[RIGHT] = accumTicks[RIGHT] + encoder[RIGHT]; //record accumulated right ticks
    Serial.println("Encoder value:");
    Serial.print("\tLeft:\t");
    Serial.print(encoder[LEFT]);
    Serial.print("\tRight:\t");
    Serial.println(encoder[RIGHT]);
    Serial.println("Accumulated Ticks: ");
    Serial.print("\tLeft:\t");
    Serial.print(accumTicks[LEFT]);
    Serial.print("\tRight:\t");
    Serial.println(accumTicks[RIGHT]);
    encoder[LEFT] = 0;                          //clear the left encoder data buffer
    encoder[RIGHT] = 0;                         //clear the right encoder data buffer
    timer = millis();                           //record current time since program started
  }
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

/*This function, runToStop(), will run the robot until the target is achieved and
   then stop it
*/
void runToStop ( void ) {
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

/*
   The move1() function will move the robot forward one full rotation and backwared on
   full rotation.  Recall that that there 200 steps in one full rotation or 1.8 degrees per
   step. This function uses setting the step pins high and low with delays to move. The speed is set by
   the length of the delay.
*/
void move1() {
  Serial.println("move1 function");
  digitalWrite(redLED, HIGH);//turn on red LED
  digitalWrite(grnLED, LOW);//turn off green LED
  digitalWrite(ylwLED, LOW);//turn off yellow LED
  digitalWrite(ltDirPin, HIGH); // Enables the motor to move in a particular direction
  digitalWrite(rtDirPin, HIGH); // Enables the motor to move in a particular direction
  // Makes 800 pulses for making one full cycle rotation
  for (int x = 0; x < 800; x++) {
    digitalWrite(rtStepPin, HIGH);
    digitalWrite(ltStepPin, HIGH);
    delayMicroseconds(stepTime);
    digitalWrite(rtStepPin, LOW);
    digitalWrite(ltStepPin, LOW);
    delayMicroseconds(stepTime);
  }
  delay(1000); // One second delay
  digitalWrite(ltDirPin, LOW); // Enables the motor to move in opposite direction
  digitalWrite(rtDirPin, LOW); // Enables the motor to move in opposite direction
  // Makes 800 pulses for making one full cycle rotation
  for (int x = 0; x < 800; x++) {
    digitalWrite(rtStepPin, HIGH);
    digitalWrite(ltStepPin, HIGH);
    delayMicroseconds(stepTime);
    digitalWrite(rtStepPin, LOW);
    digitalWrite(ltStepPin, LOW);
    delayMicroseconds(stepTime);
  }
  delay(1000); // One second delay
}

/*
   The move2() function will use AccelStepper library functions to move the robot
   move() is a library function for relative movement to set a target position
   moveTo() is a library function for absolute movement to set a target position
   stop() is a library function that causes the stepper to stop as quickly as possible
   run() is a library function that uses accel and decel to achieve target position, no blocking
   runSpeed() is a library function that uses constant speed to achieve target position, no blocking
   runToPosition() is a library function that uses blocking with accel/decel to achieve target position
   runSpeedToPosition() is a library function that uses constant speed to achieve target posiiton, no blocking
   runToNewPosition() is a library function that uses blocking with accel/decel to achieve target posiiton
*/
void move2() {
  Serial.println("move2 function");
  digitalWrite(redLED, LOW);//turn off red LED
  digitalWrite(grnLED, HIGH);//turn on green LED
  digitalWrite(ylwLED, LOW);//turn off yellow LED
  stepperRight.moveTo(800);//move one full rotation forward relative to current position
  stepperLeft.moveTo(800);//move one full rotation forward relative to current position
  stepperRight.setSpeed(1000);//set right motor speed
  stepperLeft.setSpeed(1000);//set left motor speed
  stepperRight.runSpeedToPosition();//move right motor
  stepperLeft.runSpeedToPosition();//move left motor
  runToStop();//run until the robot reaches the target
  delay(1000); // One second delay
  stepperRight.moveTo(0);//move one full rotation backward relative to current position
  stepperLeft.moveTo(0);//move one full rotation backward relative to current position
  stepperRight.setSpeed(1000);//set right motor speed
  stepperLeft.setSpeed(1000);//set left motor speed
  stepperRight.runSpeedToPosition();//move right motor
  stepperLeft.runSpeedToPosition();//move left motor
  runToStop();//run until the robot reaches the target
  delay(1000); // One second delay
}

/*
   The move3() function will use the MultiStepper() class to move both motors at once
   move() is a library function for relative movement to set a target position
   moveTo() is a library function for absolute movement to set a target position
   stop() is a library function that causes the stepper to stop as quickly as possible
   run() is a library function that uses accel and decel to achieve target position, no blocking
   runSpeed() is a library function that uses constant speed to achieve target position, no blocking
   runToPosition() is a library function that uses blocking with accel/decel to achieve target position
   runSpeedToPosition() is a library function that uses constant speed to achieve target posiiton, no blocking
   runToNewPosition() is a library function that uses blocking with accel/decel to achieve target posiiton
*/
void move3() {
  Serial.println("move3 function");
  digitalWrite(redLED, LOW);//turn off red LED
  digitalWrite(grnLED, LOW);//turn off green LED
  digitalWrite(ylwLED, HIGH);//turn on yellow LED
  long positions[2]; // Array of desired stepper positions
  positions[0] = 800;//right motor absolute position
  positions[1] = 800;//left motor absolute position
  steppers.moveTo(positions);
  steppers.runSpeedToPosition(); // Blocks until all are in position
  delay(1000);//wait one second
  // Move to a different coordinate
  positions[0] = 0;//right motor absolute position
  positions[1] = 0;//left motor absolute position
  steppers.moveTo(positions);
  steppers.runSpeedToPosition(); // Blocks until all are in position
  delay(1000);//wait one second
}

/*this function will move to target at 2 different speeds - absolute position*/
void move4() {

  Serial.println("move4 function");
  int leftPos = 5000;//right motor absolute position
  int rightPos = 1000;//left motor absolute position
  int leftSpd = 1000;//right motor speed
  int rightSpd = 200; //left motor speed

  digitalWrite(redLED, HIGH);//turn on red LED
  digitalWrite(grnLED, HIGH);//turn on green LED
  digitalWrite(ylwLED, LOW);//turn off yellow LED

  stepperLeft.moveTo(leftPos);//left motor absolute position
  stepperRight.moveTo(rightPos);//right motor absolute position
  stepperLeft.setSpeed(leftSpd);
  stepperRight.setSpeed(rightSpd);
  steppers.runSpeedToPosition(); // Blocks until all are in position

  stepperLeft.setCurrentPosition(0);
  stepperRight.setCurrentPosition(0);

}

/*This function will move continuously at 2 different speeds*/
void move5() {
  Serial.println("move5 function");
  digitalWrite(redLED, LOW);//turn off red LED
  digitalWrite(grnLED, HIGH);//turn on green LED
  digitalWrite(ylwLED, HIGH);//turn on yellow LED

  int leftSpd = 1000;//right motor speed
  int rightSpd = 200; //left motor speed

  stepperLeft.setSpeed(leftSpd);//set left motor speed
  stepperRight.setSpeed(rightSpd);//set right motor speed

  while(true){
      runAtSpeed();
  }
}


/*this function will move to target at 2 different speeds - relative position*/
void move6() {

  Serial.println("move6 function");
  int leftPos = 5000;//right motor absolute position
  int rightPos = 1000;//left motor absolute position
  int leftSpd = 1000;//right motor speed
  int rightSpd = 200; //left motor speed

  digitalWrite(redLED, HIGH);//turn on red LED
  digitalWrite(grnLED, HIGH);//turn on green LED
  digitalWrite(ylwLED, HIGH);//turn off yellow LED

  //Unomment the next 2 lines for relative movement
  stepperLeft.move(leftPos);//move left wheel to relative position
  stepperRight.move(rightPos);//move right wheel to relative position

  stepperLeft.setSpeed(leftSpd);//set left motor speed
  stepperRight.setSpeed(rightSpd);//set right motor speed

  steppers.runSpeedToPosition(); // Blocks until all are in position
}

void goToAngle(int angle) {
  Serial.println("Moving to Angle");
  int leftPos = angle*angleConversion;//right motor absolute position
  int rightPos = -(angle*angleConversion);//left motor absolute position
  int leftSpd = 500;//right motor speed
  int rightSpd = -500; //left motor speed

  if(angle < 0) {
    rightSpd = -rightSpd;
    leftSpd = -leftSpd;
  }

  digitalWrite(redLED, HIGH);//turn on red LED
  digitalWrite(grnLED, HIGH);//turn on green LED
  digitalWrite(ylwLED, HIGH);//turn off yellow LED

  //Unomment the next 2 lines for relative movement
  stepperLeft.move(leftPos);//move left wheel to relative position
  stepperRight.move(rightPos);//move right wheel to relative position

  stepperLeft.setSpeed(leftSpd);//set left motor speed
  stepperRight.setSpeed(rightSpd);//set right motor speed

  steppers.runSpeedToPosition(); // Blocks until all are in position
}

void goForward(double distance) {
  int leftPos = distance*distanceConversion;//right motor absolute position
  int rightPos = distance*distanceConversion;//left motor absolute position
  int leftSpd = 500;//right motor speed
  int rightSpd = 500; //left motor speed

  if(distance < 0) {
    rightSpd = -rightSpd;
    leftSpd = -leftSpd;
  }

  //Unomment the next 2 lines for relative movement
  stepperLeft.move(leftPos);//move left wheel to relative position
  stepperRight.move(rightPos);//move right wheel to relative position

  stepperLeft.setSpeed(leftSpd);//set left motor speed
  stepperRight.setSpeed(rightSpd);//set right motor speed

  steppers.runSpeedToPosition(); // Blocks until all are in position
}

void goBackward(double distance) {
  int leftPos = distance*distanceConversion;//right motor absolute position
  int rightPos = distance*distanceConversion;//left motor absolute position
  int leftSpd = -500;//right motor speed
  int rightSpd = -500; //left motor speed

  //Unomment the next 2 lines for relative movement
  stepperLeft.move(leftPos);//move left wheel to relative position
  stepperRight.move(rightPos);//move right wheel to relative position

  stepperLeft.setSpeed(leftSpd);//set left motor speed
  stepperRight.setSpeed(rightSpd);//set right motor speed

  steppers.runSpeedToPosition(); // Blocks until all are in position
}

/*
Goes to goal
*/
void goToGoal(double x, double y) {
  Serial.println("Going To Goal");
  double desiredAngle = atan2(y,x) * 180 / Pi;
  Serial.println(desiredAngle);
  goToAngle(desiredAngle);
  delay(400);
  double distance = sqrt( x*x + y*y );
  Serial.println(distance);
	goForward(distance);
  delay(400);
}

/*
  0 for turn right, 1 for turn left
*/
void pivotLeft(int angle) {
  Serial.println("Pivoting Left");

  int leftPos = angle*angleConversion*2;//right motor absolute position
  int rightPos = 0;
  int leftSpd = 500;//right motor speed
  int rightSpd = 0;

  Serial.println(leftPos);
  Serial.println(rightPos);
  Serial.println(leftSpd);
  Serial.println(rightSpd);

  //Unomment the next 2 lines for relative movement
  stepperLeft.move(leftPos);  //move left wheel to relative position
  stepperRight.move(rightPos);

  stepperLeft.setSpeed(leftSpd);//set left motor speed
  stepperRight.setSpeed(rightSpd);

  steppers.runSpeedToPosition(); // Blocks until all are in position
}

void pivotRight(int angle) {
  Serial.println("Pivoting Right");

  int leftPos = 0;//right motor absolute position
  int rightPos = angle*angleConversion*2;
  int leftSpd = 0;//right motor speed
  int rightSpd = 500;

  Serial.println(leftPos);
  Serial.println(rightPos);
  Serial.println(leftSpd);
  Serial.println(rightSpd);

  //Unomment the next 2 lines for relative movement
  stepperLeft.move(leftPos);  //move left wheel to relative position
  stepperRight.move(rightPos);

  stepperLeft.setSpeed(leftSpd);//set left motor speed
  stepperRight.setSpeed(rightSpd);

  steppers.runSpeedToPosition(); // Blocks until all are in position
}

/*
  0 for turn right, 1 for turn left
*/
void spin(int angle) {
  goToAngle(angle);
}

/*
  0 for spin right, 1 for spin left
*/
void turnRight(int angle) {
  
  int leftPos = angle*angleConversion*1.5;//right motor absolute position
  int rightPos = angle*angleConversion*0.5;
  int leftSpd = 500;//right motor speed
  int rightSpd = 200;

  Serial.println(leftPos);
  Serial.println(rightPos);
  Serial.println(leftSpd);
  Serial.println(rightSpd);

  //Unomment the next 2 lines for relative movement
  stepperLeft.move(leftPos);//move left wheel to relative position
  stepperRight.move(rightPos);

  stepperLeft.setSpeed(leftSpd);//set left motor speed
  stepperRight.setSpeed(rightSpd);

  steppers.runSpeedToPosition(); // Blocks until all are in position
}

void turnLeft(int angle) {
  
  int leftPos = angle*angleConversion*0.5;//right motor absolute position
  int rightPos = angle*angleConversion*1.5;
  int leftSpd = 200;//right motor speed
  int rightSpd = 500;

  Serial.println(leftPos);
  Serial.println(rightPos);
  Serial.println(leftSpd);
  Serial.println(rightSpd);

  //Unomment the next 2 lines for relative movement
  stepperLeft.move(leftPos);//move left wheel to relative position
  stepperRight.move(rightPos);

  stepperLeft.setSpeed(leftSpd);//set left motor speed
  stepperRight.setSpeed(rightSpd);

  steppers.runSpeedToPosition(); // Blocks until all are in position
}

/*
  INSERT DESCRIPTION HERE, what are the inputs, what does it do, functions used
*/
void forward(int distance) {
  Serial.println("Forwarding");
  goForward(distance);
}
/*
  INSERT DESCRIPTION HERE, what are the inputs, what does it do, functions used
*/
void reverse(int distance) {
  goForward(-distance);
}
/*
  INSERT DESCRIPTION HERE, what are the inputs, what does it do, functions used
*/
void stop() {
  stepperLeft.setSpeed(0);
  stepperRight.setSpeed(0);

  runAtSpeed();
}

/*
  Wheel Distance = 21.25 cm distance b/w wheels
  0, curves robot to the right, 1 curves robot to the left
*/
void moveCircle(int diam, int dir, double arclength) {

  double wheelDistance = 22;

  if(diam < wheelDistance) {
    goToAngle(360);
    return;
  }
  double outerDiam = diam + wheelDistance;
  double innerDiam = diam - wheelDistance;

  int outerSpd = 500;
  int innerSpd = outerSpd * (innerDiam/outerDiam);

  int innerPos = 3.14159*innerDiam*distanceConversion*arclength/360;
  int outerPos = 3.14159*outerDiam*distanceConversion*arclength/360;

  Serial.println(outerDiam);
  Serial.println(outerSpd);
  Serial.println(outerPos);

  Serial.println(innerDiam);
  Serial.println(innerSpd);
  Serial.println(innerPos);

  if(dir == 0) {
    stepperLeft.move(outerPos);//move left wheel to relative position
    stepperRight.move(innerPos);//move right wheel to relative position

    stepperLeft.setSpeed(outerSpd);//set left motor speed
    stepperRight.setSpeed(innerSpd);//set right motor speed
  } else {
  stepperLeft.move(innerPos);//move left wheel to relative position
  stepperRight.move(outerPos);//move right wheel to relative position

  stepperLeft.setSpeed(innerSpd);//set left motor speed
  stepperRight.setSpeed(outerSpd);//set right motor speed
  }
  steppers.runSpeedToPosition(); // Blocks until all are in position
}

/*
  The moveFigure8() function takes the diameter in inches as the input. It uses the moveCircle() function
  twice with 2 different direcitons to create a figure 8 with circles of the given diameter.
*/
void moveFigure8(int diam) {

  digitalWrite(redLED, HIGH);//turn on red LED
  digitalWrite(ylwLED, HIGH);//turn on yellow LED

  moveCircle(diam,1,360);
  delay(200);
  moveCircle(diam,0,360);

}


//// MAIN
void setup()
{
  int baudrate = 9600; //serial monitor baud rate'
  init_stepper(); //set up stepper motor

  attachInterrupt(digitalPinToInterrupt(ltEncoder), LwheelSpeed, CHANGE);    //init the interrupt mode for the left encoder
  attachInterrupt(digitalPinToInterrupt(rtEncoder), RwheelSpeed, CHANGE);   //init the interrupt mode for the right encoder


  Serial.begin(baudrate);     //start serial monitor communication
  Serial.println("Robot starting...Put ON TEST STAND");
  delay(pauseTime); //always wait 2.5 seconds before the robot moves
}

void loop()
{
  //uncomment each function one at a time to see what the code does
  // move1();//call move back and forth function
  // move2();//call move back and forth function with AccelStepper library functions
  // move3();//call move back and forth function with MultiStepper library functions
  // move4(); //move to target position with 2 different speeds - absolute position
  // move5(); //move continuously with 2 different speeds
  // move6(); //move to target position with 2 different speeds - relative position

  //Uncomment to read Encoder Data (uncomment to read on serial monitor)
  //print_encoder_data();   //prints encoder data

  
  
  goToGoal(-91.44,-121.92);
  delay(10000);
  

}