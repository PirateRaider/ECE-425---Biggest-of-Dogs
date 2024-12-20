/*&StateMachine.ino
  Author: Carlotta. A. Berry
  Date: December 3, 2016
  This program will provide a template for an example of implementing a behavior-based control architecture
  for a mobile robot to implement obstacle avoidance and random wander. There are many ways to create a state machine
  and this is just one. It is to help get you started and brainstorm ideas, you are not required to use it.
  Feel free to create your own version of state machine.

  The flag byte (8 bits) variable will hold the IR and sonar data [X X snrRight snrLeft irLeft irRight irRear irFront]
  The state byte (8 bits) variable will hold the state information as well as motor motion [X X X wander runAway collide rev fwd]

  Use the following functions to read, clear and set bits in the byte
  bitRead(state, wander)) { // check if the wander state is active
  bitClear(state, wander);//clear the the wander state
  bitSet(state, wander);//set the wander state

  Hardware Connections:
  digital pin 48 - enable PIN on A4988 Stepper Motor Driver StepSTICK
  digital pin 50 - right stepper motor step pin
  digital pin 51 - right stepper motor direction pin
  digital pin 52 - left stepper motor step pin
  digital pin 53 - left stepper motor direction pin
  digital pin 13 - enable LED on microcontroller

  digital pin 5 - red LED in series with 220 ohm resistor
  digital pin 6 - green LED in series with 220 ohm resistor
  digital pin 7 - yellow LED in series with 220 ohm resistor

  Front IR    A0
  Back IR     A1
  Right IR    A2
  Left IR     A3
  Left Sonar  A8
  Right Sonar A9
  Pushbutton  A15
*/

#include <AccelStepper.h>//include the stepper motor library
#include <MultiStepper.h>//include multiple stepper motor library
#include <NewPing.h> //include sonar library
#include <TimerOne.h>


//define motor pin numbers
#define stepperEnable 48    //stepper enable pin on stepStick 
#define rtStepPin 50 //right stepper motor step pin 
#define rtDirPin 51  // right stepper motor direction pin 
#define ltStepPin 52 //left stepper motor step pin 
#define ltDirPin 53  //left stepper motor direction pin 

AccelStepper stepperRight(AccelStepper::DRIVER, rtStepPin, rtDirPin);//create instance of right stepper motor object (2 driver pins, low to high transition step pin 52, direction input pin 53 (high means forward)
AccelStepper stepperLeft(AccelStepper::DRIVER, ltStepPin, ltDirPin);//create instance of left stepper motor object (2 driver pins, step pin 50, direction input pin 51)
MultiStepper steppers;//create instance to control multiple steppers at the same time

//define stepper motor constants
#define enableLED 13 //stepper enabled LED
#define stepperEnTrue false //variable for enabling stepper motor
#define stepperEnFalse true //variable for disabling stepper motor
#define test_led 13 //test led to test interrupt heartbeat

#define robot_spd 1500 //set robot speed
#define max_accel 10000//maximum robot acceleration
#define max_spd 2500//maximum robot speed

#define quarter_rotation 200  //stepper quarter rotation
#define half_rotation 400     //stepper half rotation
#define one_rotation  800     //stepper motor runs in 1/4 steps so 800 steps is one full rotation
#define two_rotation  1600    //stepper motor 2 rotations
#define three_rotation 2400   //stepper rotation 3 rotations
#define four_rotation 3200    //stepper rotation 3 rotations
#define five_rotation 4000    //stepper rotation 3 rotations

//define IR sensor connections
#define irFront A0 //front IR analog pin
#define irRear  A1//back IR analog pin
#define irRight A2 //right IR analog pin
#define irLeft  A3 //left IR analog pin
#define button A15    //pushbutton 

//defome sonar connections
#define snrLeft   A8   //front left sonar 
#define snrRight  A9  //front right sonar 
#define button    A15    //pushbutton 

NewPing sonarLt(snrLeft, snrLeft);    //create an instance of the left sonar
NewPing sonarRt(snrRight, snrRight);  //create an instance of the right sonar

#define irThresh    400 // The IR threshold for presence of an obstacle
#define snrThresh   5   // The sonar threshold for presence of an obstacle
#define minThresh   0   // The sonar minimum threshold to filter out noise
#define stopThresh  150 // If the robot has been stopped for this threshold move
#define baud_rate 9600//set serial communication baud rate

int irFrontArray[5] = {0, 0, 0, 0, 0}; //array to hold 5 front IR readings
int irRearArray[5] = {0, 0, 0, 0, 0}; //array to hold 5 back IR readings
int irRightArray[5] = {0, 0, 0, 0, 0}; //array to hold 5 right IR readings
int irLeftArray[5] = {0, 0, 0, 0, 0}; //array to hold 5 left IR readings
int irFrontAvg;  //variable to hold average of current front IR reading
int irLeftAvg;   //variable to hold average of current left IR reading
int irRearAvg;   //variable to hold average of current rear IR reading
int irRightAvg;   //variable to hold average of current right IR reading
int irIdx = 0;//index for 5 IR readings to take the average

int srLeftArray[5] = {0, 0, 0, 0, 0}; //array to hold 5 left sonar readings
int srRightArray[5] = {0, 0, 0, 0, 0}; //array to hold 5 right sonar readings
int srIdx = 0;//index for 5 sonar readings to take the average
int srLeft;   //variable to hold average of left sonar current reading
int srRight;  //variable to hold average or right sonar current reading
int srLeftAvg;  //variable to holde left sonar data
int srRightAvg; //variable to hold right sonar data

volatile boolean test_state; //variable to hold test led state for timer interrupt



//flag byte to hold sensor data
volatile byte flag = 0;    // Flag to hold IR & Sonar data - used to create the state machine

//bit definitions for sensor data flag byte
#define obFront   0 // Front IR trip
#define obRear    1 // Rear IR trip
#define obRight   2 // Right IR trip
#define obLeft    3 // Left IR trip
#define obFLeft   4 // Left Sonar trip
#define obFRight  5 // Right Sonar trip

int count; //count number of times collide has tripped
#define max_collide 250 //maximum number of collides before robot reverses

//state byte to hold robot motion and state data
volatile byte state = 0;   //state to hold robot states and motor motion

//bit definitions for robot motion and state byte
#define fwd     0
#define rev     1
#define collide 2
#define runAway 3
#define wander  4

//define layers of subsumption architecture that are active
byte layers = 2; //[wander runAway collide]
//bit definitions for layers
#define cLayer 0
#define rLayer 1
#define wLayer 2

#define timer_int 250000 // 1/2 second (500000 us) period for timer interrupt

/*
  This is a sample updateSensors() function and it should be updated along with the description to reflect what you actually implemented
  to meet the lab requirements.
*/
void updateSensors() {
  //  Serial.print("updateSensors\t");
  //  Serial.println(test_state);
  //test_state = !test_state;//LED to test the heartbeat of the timer interrupt routine
  //digitalWrite(enableLED, test_state);  // Toggles the LED to let you know the timer is working
  test_state = !test_state;
  digitalWrite(test_led, test_state);
  flag = 0;       //clear all sensor flags
  state = 0;      //clear all state flags
  updateIR();     //update IR readings and update flag variable and state machine
  //updateSonar();  //update Sonar readings and update flag variable and state machine
  //updateSonar2(); //there are 2 ways to read sonar data, this is the 2nd option, use whichever one works best for your hardware
  updateState();  //update State Machine based upon sensor readings
  //delay(1000);     //added so that you can read the data on the serial monitor
}

/*
   This is a sample updateIR() function, the description and code should be updated to take an average, consider all sensor and reflect
   the necesary changes for the lab requirements.
*/

void updateIR() {
  int front, back, left, right;
  front = analogRead(irFront);
  back = analogRead(irRear);
  left = analogRead(irLeft);
  right = analogRead(irRight);
  //  print IR data
  //  Serial.println("frontIR\tbackIR\tleftIR\trightIR");
  //  Serial.print(front); Serial.print("\t");
  //  Serial.print(back); Serial.print("\t");
  //  Serial.print(left); Serial.print("\t");
  //  Serial.println(right);
  if (right > irThresh)
    bitSet(flag, obRight);//set the right obstacle
  else
    bitClear(flag, obRight);//clear the right obstacle
  if (left > irThresh)
    bitSet(flag, obLeft);//set the left obstacle
  else
    bitClear(flag, obLeft);//clear the left obstacle
  if (front > irThresh) {
    Serial.println("set front obstacle bit");
    bitSet(flag, obFront);//set the front obstacle
  }
  else
    bitClear(flag, obFront);//clear the front obstacle
  if (back > irThresh)
    bitSet(flag, obRear);//set the back obstacle
  else
    bitClear(flag, obRear);//clear the back obstacle
}

/*
   This is a sample updateSonar() function, the description and code should be updated to take an average, consider all sensors and reflect
   the necesary changes for the lab requirements.
*/
void updateSonar() {
  long left, right;
  //read right sonar
  pinMode(snrRight, OUTPUT);//set the PING pin as an output
  digitalWrite(snrRight, LOW);//set the PING pin low first
  delayMicroseconds(2);//wait 2 us
  digitalWrite(snrRight, HIGH);//trigger sonar by a 2 us HIGH PULSE
  delayMicroseconds(5);//wait 5 us
  digitalWrite(snrRight, LOW);//set pin low first again
  pinMode(snrRight, INPUT);//set pin as input with duration as reception
  right = pulseIn(snrRight, HIGH);//measures how long the pin is high

  //read left sonar
  pinMode(snrLeft, OUTPUT);//set the PING pin as an output
  digitalWrite(snrLeft, LOW);//set the PING pin low first
  delayMicroseconds(2);//wait 2 us
  digitalWrite(snrLeft, HIGH);//trigger sonar by a 2 us HIGH PULSE
  delayMicroseconds(5);//wait 5 us
  digitalWrite(snrLeft, LOW);//set pin low first again
  pinMode(snrLeft, INPUT);//set pin as input with duration as reception
  left = pulseIn(snrLeft, HIGH);//measures how long the pin is high
  //  print sonar data
  //    Serial.println("leftSNR\trightSNR");
  //    Serial.print(left); Serial.print("\t");
  //    Serial.println(right);
  if (right < snrThresh)
    bitSet(flag, obFRight);//set the front right obstacle
  else
    bitClear(flag, obFRight);//clear the front right obstacle
  if (left < snrThresh)
    bitSet(flag, obFLeft);//set the front left obstacle
  else
    bitClear(flag, obFLeft);//clear the front left obstacle
}

/*
  This is a sample updateSonar2() function, the description and code should be updated to take an average, consider all sensors and reflect
  the necesary changes for the lab requirements.
*/
void updateSonar2() {
  srRightAvg =  sonarRt.ping_in(); //right sonara in inches
  delay(50);
  srLeftAvg = sonarLt.ping_in(); //left sonar in inches
  //    Serial.print("lt snr:\t");
  //    Serial.print(srLeftAvg);
  //    Serial.print("rt snr:\t");
  //    Serial.println(srRightAvg);
  if (srRightAvg < snrThresh && srRightAvg > minThresh)
    bitSet(flag, obFRight);//set the front right obstacle
  else
    bitClear(flag, obFRight);//clear the front right obstacle
  if (srLeftAvg < snrThresh && srLeftAvg > minThresh)
    bitSet(flag, obFLeft);//set the front left obstacle
  else
    bitClear(flag, obFLeft);//clear the front left obstacle
}

/*
   This is a sample updateState() function, the description and code should be updated to reflect the actual state machine that you will implement
   based upon the the lab requirements.
*/
void updateState() {
  if (!(flag)) { //no sensors triggered
    bitSet(state, fwd); //set forward motion
    bitClear(state, collide);//clear collide state
    count--;//decrement collide counter
  }
  else if (flag & 0b1) { //front sensors triggered
    bitClear(state, fwd); //clear reverse motion
    bitSet(state, collide);//set collide state
  }
  //print flag byte
  //    Serial.println("\trtSNR\tltSNR\tltIR\trtIR\trearIR\tftIR");
  //    Serial.print("flag byte: ");
  //    Serial.println(flag, BIN);
  //print state byte
  //    Serial.println("\twander\trunAway\tcollide\treverse\tforward");
  //    Serial.print("state byte: ");
  //    Serial.println(state, BIN);
}

/*
   This is a sample robotMotion() function, the description and code should be updated to reflect the actual robot motion function that you will implement
   based upon the the lab requirements.  Some things to consider, you cannot use a blocking motor function because you need to use sensor data to update
   movement.  You also need to continue to poll    the sensors during the motion and update flags and state because this will serve as your interrupt to
   stop or change movement.
*/
void robotMotion() {
  if ((flag & 0b1) || bitRead(state, collide)) { //check for a collide state
    stop();
    Serial.println("robot stop");
  }
  else{
    Serial.println("robot forward");
    forward(one_rotation);//move forward as long as all sensors are clear
  }
}

void forward(int rot) {
  long positions[2]; // Array of desired stepper positions
  stepperRight.setCurrentPosition(0);
  stepperLeft.setCurrentPosition(0);
  positions[0] = stepperRight.currentPosition() + rot;  //right motor absolute position
  positions[1] = stepperLeft.currentPosition() + rot;   //left motor absolute position
  stepperRight.move(positions[0]);  //move right motor to position
  stepperLeft.move(positions[1]);   //move left motor to position
  runToStop();//run until the robot reaches the target
}

void stop() {
  stepperRight.stop();
  stepperLeft.stop();
}

/*This function, runToStop(), will run the robot until the target is achieved and
   then stop it
*/
void runToStop ( void ) {
  int runNow = 1;
  while (runNow) {
    if (!stepperRight.run() ) {
      runNow = 0;
      stop();
    }
    if (!stepperLeft.run()) {
      runNow = 0;
      stop();
    }
  }
}


void setup()
{
  //stepper Motor set up
  pinMode(rtStepPin, OUTPUT);//sets pin as output
  pinMode(rtDirPin, OUTPUT);//sets pin as output
  pinMode(ltStepPin, OUTPUT);//sets pin as output
  pinMode(ltDirPin, OUTPUT);//sets pin as output
  pinMode(stepperEnable, OUTPUT);//sets pin as output
  digitalWrite(stepperEnable, stepperEnFalse);//turns off the stepper motor driver
  pinMode(enableLED, OUTPUT);//set LED as output
  digitalWrite(enableLED, LOW);//turn off enable LED
  stepperRight.setMaxSpeed(max_spd);//set the maximum permitted speed limited by processor and clock speed, no greater than 4000 steps/sec on Arduino
  stepperRight.setAcceleration(max_accel);//set desired acceleration in steps/s^2
  stepperLeft.setMaxSpeed(max_spd);//set the maximum permitted speed limited by processor and clock speed, no greater than 4000 steps/sec on Arduino
  stepperLeft.setAcceleration(max_accel);//set desired acceleration in steps/s^2

  stepperRight.setSpeed(robot_spd);//set right motor speed
  stepperLeft.setSpeed(robot_spd);//set left motor speed

  steppers.addStepper(stepperRight);//add right motor to MultiStepper
  steppers.addStepper(stepperLeft);//add left motor to MultiStepper
  digitalWrite(stepperEnable, stepperEnTrue);//turns on the stepper motor driver
  digitalWrite(enableLED, HIGH);//turn on enable LED
  //Timer Interrupt Set Up
  Timer1.initialize(timer_int);         // initialize timer1, and set a period in microseconds
  Timer1.attachInterrupt(updateSensors);  // attaches updateSensors() as a timer overflow interrupt

  Serial.begin(baud_rate);//start serial communication in order to debug the software while coding
  delay(3000);//wait 3 seconds before robot moves
}

void loop()
{
  robotMotion();  //execute robot motions based upon sensor data and current state
}
