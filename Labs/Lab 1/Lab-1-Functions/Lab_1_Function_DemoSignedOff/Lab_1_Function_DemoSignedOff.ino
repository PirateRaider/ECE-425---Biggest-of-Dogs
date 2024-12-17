
/*
***********************************
  RobotIntro.ino
  Donald & Ben 12/17/24

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

  Wheel Diamter = 8.5 cm, 26.7035375555 cm circumference
  Wheel Distance = 21.5 cm
  Stepper motor = 0.45 degree steps
*/

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
  goToAngle takes in an angle in degrees, and moves the robot that many degrees, moving both motors at equal and opposite speeds so as to stay in position
  It uses an arbitrary speed of 500, and the function's stepperLeft/stepperRight to control each wheel independently
  If the angle is negative, the robot will move in the opposite direction.
*/
void goToAngle(int angle) {
  Serial.println("Moving to Angle");  // Print for debugging
  Serial.println(angle);              // Print for debugging
  int leftPos = -angle*angleConversion;//right motor absolute position
  int rightPos = (angle*angleConversion);//left motor absolute position
  int leftSpd = -500;//right motor speed
  int rightSpd = 500; //left motor speed

  if(angle < 0) {
    Serial.println("Angle is Negative");
    rightSpd = -rightSpd;
    leftSpd = -leftSpd;
  }

  digitalWrite(redLED, HIGH);//turn on red LED
  digitalWrite(grnLED, HIGH);//turn on green LED
  digitalWrite(ylwLED, HIGH);//turn off yellow LED

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
  This function moves the robot backwards a certain distance in cm, moving both motors at equal speed in one direction.
  It uses an arbitrary speed of 500, and the function's stepperLeft/stepperRight to control each wheel independently
  If the distance is negative, the robot will run forever, since the wheels only spin backwards.
*/
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
  This function moves the robot backwards a certain distance in cm, moving both motors at equal speed in one direction.
  It uses an arbitrary speed of 500, and the function's stepperLeft/stepperRight to control each wheel independently
  If the distance is negative, the robot will run forever, since the wheels only spin backwards.
*/

void goToGoal(double x, double y) {
  Serial.println("Going To Goal");
  double desiredAngle = atan2(y,x) * 180 / 3.14159;
  Serial.println("Desired Angle Is:");
  Serial.println(desiredAngle);
  goToAngle(desiredAngle);
  delay(400);
  double distance = sqrt( x*x + y*y );
  Serial.println(distance);
	goForward(distance);
  delay(400);
}

/*
  This function has the robot pivot to the left, to a certain angle in degrees, keeping one motor stationary and another motor moving at an arbitrary speed of 500.
  It uses the function's stepperLeft/stepperRight to control each wheel independently
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

/*
  This function has the robot pivot to the right, to a certain angle in degrees, keeping one motor stationary and another motor moving at an arbitrary speed of 500.
  It uses the function's stepperLeft/stepperRight to control each wheel independently
*/
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

  stepperLeft.move(leftPos);  //move left wheel to relative position
  stepperRight.move(rightPos);

  stepperLeft.setSpeed(leftSpd);//set left motor speed
  stepperRight.setSpeed(rightSpd);

  steppers.runSpeedToPosition(); // Blocks until all are in position
}

/*
  Same as goToAngle:
  spin takes in an angle in degrees, and moves the robot that many degrees, moving both motors at equal and opposite speeds so as to stay in position
  It uses an arbitrary speed of 500, and the function's stepperLeft/stepperRight to control each wheel independently
  If the angle is negative, the robot will move in the opposite direction.
*/
void spin(int angle) {
  goToAngle(angle);
}

/*
  turnRight takes a specific angle, and runs both motors in the same direction, with one being slightly faster to make the robot turn.
  leftSpd was arbitrarily chosen to move at a speed of 500, and rightSpd was arbitrarily chosen to move at a speed of 200.
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

/*
  turnLeft takes a specific angle, and runs both motors in the same direction, with one being slightly faster to make the robot turn.
  rightSpd was arbitrarily chosen to move at a speed of 500, and leftSpd was arbitrarily chosen to move at a speed of 200.
*/

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
  stop sets both motor's speeds to 0, causing the robot to stop in place.
*/
void stop() {
  stepperLeft.setSpeed(0);
  stepperRight.setSpeed(0);

  runAtSpeed();
}

/*
  circle causes both wheels to spin in the same direction to create a circle of a specific diameter, and along an arc length in degrees.
  Robot uses a fudge factor implemented based on the distance between the wheels to calculate what speed the wheels should spin at.
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
  //Uncomment to read Encoder Data (uncomment to read on serial monitor)
  //print_encoder_data();   //prints encoder data

  goToGoal(-91.44,-121.92);
  delay(10000);

}