/*RobotInterrupt.ino
  Author: Carlotta. A. Berry
  Date: December 10, 2016
  This program will test the sonar interrupt code in order to use this for
  obstacle avoidance. Only the 2 front sonar sensors are used.

  The following pins are usable for PinChangeInterrupt:
  Arduino Uno/Nano/Mini: All pins are usable
  Arduino Mega: 10, 11, 12, 13, 50, 51, 52, 53, A8 (62), A9 (63), A10 (64),
              A11 (65), A12 (66), A13 (67), A14 (68), A15 (69)
  Hardware Connections:
  pin mappings: https://www.arduino.cc/en/Hacking/PinMapping2560
  digital pin 13 - enable LED on microcontroller
  digital pin 48 - enable PIN on A4988 Stepper Motor Driver StepSTICK
  digital pin 46 - right stepper motor step pin
  digital pin 53 - right stepper motor direction pin
  digital pin 44 - left stepper motor step pin
  digital pin 49 - left stepper motor direction pin

  Hardware Connections:
  Arduino pin mappings: https://www.arduino.cc/en/Hacking/PinMapping2560
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

  digital pin 2 - IMU INT
  digital pin 20 - IMU SDA
  digital pin 21 - IMU SCL
  
  Front IR    A0
  Back IR     A1
  Right IR    A2
  Left IR     A3
  Left Sonar  A8
  Right Sonar A9
  Pushbutton  A15
*/

#include <PinChangeInt.h>//include pin change interrupt library
#include <AccelStepper.h>//include the stepper motor library
#include <MultiStepper.h>//include multiple stepper motor library

//state LEDs connections
#define redLED 5            //red LED for displaying states
#define grnLED 6            //green LED for displaying states
#define ylwLED 7            //yellow LED for displaying states
#define enableLED 13        //stepper enabled LED

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
#define stepperEnTrue false //variable for enabling stepper motor
#define stepperEnFalse true //variable for disabling stepper motor

#define one_rotation  400//stepper motor runs in 1/4 steps so 800 steps is one full rotation
#define two_rotation  800 //stepper motor 2 rotations
#define three_rotation 1200 //stepper rotation 3 rotations
#define max_accel     10000//maximum robot acceleration
#define robot_spd     250 //set robot speed
#define max_spd       500//maximum robot speed

#define irFront A0 //front IR analog pin
#define irRear  A1//back IR analog pin
#define irRight A2 //right IR analog pin
#define irLeft  A3 //left IR analog pin
#define snrLeft A8 //front left sonar [INTERRUPT]
#define snrRight A9 //front right sonar [INTERRUPT]
#define button A15 //pushbutton [Interrupt]

#define snrThresh   750 // The sonar threshold for presence of an obstacle
#define minThresh   10  // minimum sonar threshold to reduce noise error
#define stopThresh  150  // If the robot has been stopped for this threshold move

#define baud_rate   9600  //set serial communication baud rate
#define ping_interval 1500//interval between sonar pulses
#define TIME      500   //pause time


//sonar Interrupt variables
volatile unsigned long last_detection = 0;
volatile unsigned long last_stop = 0;
volatile uint8_t stopCount = 0; // counter on how long the robot has been stopped
unsigned long left, right; //holds left and right sonar data

//obsRoutine.ino Interrupt Service Routine for falling edge of echo

void obsRoutine() {
  //  print sonar data
  if ((right > minThresh && right < snrThresh) && (left > minThresh && left < snrThresh)) {
    Serial.println("obstacle detected: stop Robot");
    stop();//stop the robot
    Serial.println("leftSNR\trightSNR");
    Serial.print(left);
    Serial.print("\t");
    Serial.println(right);
    last_stop = millis();
    stopCount = 0;
    while (stopCount < stopThresh) {
      stopCount++;
      Serial.print("obstacle detected: stop Robot\t");
      Serial.println(stopCount);
      //updateSonar();//update sonar data
    }
  }
}

/* Motion Commands */

/*
  move() is a library function for relative movement to set a target position
  moveTo() is a library function for absolute movement to set a target position
  stop() is a library function that causes the stepper to stop as quickly as possible
  runToPosition() is a library function that uses blocking with accel/decel to achieve target position
  runToNewPosition() is a library function that uses blocking with accel/decel to achieve target posiiton
  run() is a library function that uses accel and decel to achieve target position, NO BLOCKING
  runSpeed() is a library function that uses constant speed to achieve target position, NO BLOCKING
  runSpeedToPosition() is a library function that uses constant speed to achieve target posiiton, NO BLOCKING
*/

void forward(int rot) {
  long positions[2]; // Array of desired stepper positions
  Serial.print("forward\t");
  stepperRight.setCurrentPosition(0);
  stepperLeft.setCurrentPosition(0);
  positions[0] = stepperRight.currentPosition() + rot; //right motor absolute position
  positions[1] = stepperLeft.currentPosition() + rot; //left motor absolute position
  Serial.print(positions[0]);
  Serial.print("\t");
  Serial.println(positions[1]);
  steppers.moveTo(positions);
  steppers.runSpeedToPosition(); // Blocks until all are in position
}

void turnRight(int rot) {
  long positions[2]; // Array of desired stepper positions
  Serial.print("reverse\t");
  stepperRight.setCurrentPosition(0);
  stepperLeft.setCurrentPosition(0);
  positions[0] = stepperRight.currentPosition() - two_rotation; //right motor absolute position
  positions[1] = stepperLeft.currentPosition() - two_rotation; //left motor absolute position
  Serial.print(positions[0]);
  Serial.print("\t");
  Serial.println(positions[1]);
  steppers.moveTo(positions);
  steppers.runSpeedToPosition(); // Blocks until all are in 
  positions[0] = stepperRight.currentPosition() - rot; //right motor absolute position
  positions[1] = stepperLeft.currentPosition() + rot; //left motor absolute position
  Serial.print(positions[0]);
  Serial.print("\t");
  Serial.println(positions[1]);
  steppers.moveTo(positions);
  steppers.runSpeedToPosition(); // Blocks until all are in position
}

void stop() {
  stepperRight.stop();
  stepperLeft.stop();
}

/*
  This is a sample updateSonar() function, the description and code should be updated to take an average, consider all sensors and reflect
  the necesary changes for the lab requirements.
*/
void updateSonar() {
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
}

void setup() {
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
  //Interrupt Set Up
  attachPinChangeInterrupt(snrLeft, obsRoutine, FALLING);
  attachPinChangeInterrupt(snrRight, obsRoutine, FALLING);
  Serial.begin(baud_rate);//start serial communication in order to debug the software while coding
  Serial.println("Sonar Interrupt....");
  delay(2500); //seconds before the robot moves
}

void loop() {

  if (micros() - last_detection > ping_interval) {
    updateSonar();//update sonar data
    last_detection = micros();
  }
  //  Serial.print("loop:\t" );
  //  Serial.println(stopCount);
  if (stopCount == stopThresh) {
    Serial.println("robot back up and turn");
    turnRight(one_rotation);//spin robot
    stopCount = 0;
  }
  if (millis() - last_stop > stopThresh) {
    forward(one_rotation);//move robot forward
  }
}
