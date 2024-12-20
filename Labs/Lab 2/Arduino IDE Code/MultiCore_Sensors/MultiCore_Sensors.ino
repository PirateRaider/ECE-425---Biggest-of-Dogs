//Justin Dewitt 12.12.23
// read about multicore: https://docs.arduino.cc/tutorials/giga-r1-wifi/giga-dual-core
//This code will show how to run the same code on the M4 and M7 to server and client, respectively
//M4 will read sensor data and send to M7 to run the state machine


#include "Arduino.h"
#include "RPC.h"
#define frontLdr 8
#define backLdr 9
#define leftLdr 10
#define rightLdr 11
#define leftSnr 7
#define rightSnr 6
#define led 12
#define VELOCITY_TEMP(temp) ((331.5 + 0.6 * (float)(temp)) * 100 / 1000000.0)  // The ultrasonic velocity (cm/us) compensated by temperature
int16_t trig_EchoPin[2] = { 3,4 };

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


// a struct to hold lidar data
struct sonar {
  // this can easily be extended to contain sonar data as well
  int left;
  int right;
  // this defines some helper functions that allow RPC to send our struct (I found this on a random forum)
  MSGPACK_DEFINE_ARRAY(left, right);  //https://stackoverflow.com/questions/37322145/msgpack-to-pack-structures https://www.appsloveworld.com/cplus/100/391/msgpack-to-pack-structures
} dist2;

// read_lidars is the function used to get lidar data to the M7
struct lidar read_lidars() {
  return dist;
}

// read_lidars is the function used to get lidar data to the M7
struct sonar read_sonars() {
  return dist2;
}

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
/*
uint16_t read_sonar(uint16_t side) {
  uint16_t distance;
  uint32_t pulseWidthUs;
  int16_t dist, temp, dist_in;

  pinMode(trig_EchoPin[side], OUTPUT);
  digitalWrite(trig_EchoPin[side], LOW);
  digitalWrite(trig_EchoPin[side], HIGH);  //Set the trig pin High
  delayMicroseconds(10);               //Delay of 10 microseconds
  digitalWrite(trig_EchoPin[side], LOW);   //Set the trig pin Low
  pinMode(trig_EchoPin[side], INPUT);                //Set the pin to input mode
  pulseWidthUs = pulseIn(trig_EchoPin[side], HIGH);  //Detect the high level time on the echo pin, the output high level time represents the ultrasonic flight time (unit: us)
  distance = pulseWidthUs * VELOCITY_TEMP(20) / 2.0;  //The distance can be calculated according to the flight time of ultrasonic wave,/
                                                        //and the ultrasonic sound speed can be compensated according to the actual ambient temperature
  dist_in = 0.394*distance;    //convert cm to inches
  Serial.print(dist_in, DEC);   //print inches
  Serial.print(" inches ");                                          
  Serial.print(distance, DEC);  //print cm
  Serial.println(" cm");
  return distance;
} */

//set up the M4 to be the server for the sensors data
void setupM4() {
  RPC.bind("read_lidars", read_lidars);  // bind a method to return the lidar data all at once
  RPC.bind("read_sonars", read_sonars);  // bind a method to return the lidar data all at once
}

//poll the M4 to read the data
void loopM4() {
  // update the struct with current lidar data
  dist.front = read_lidar(8);
  dist.back = read_lidar(9);
  dist.left = read_lidar(10);
  dist.right = read_lidar(11);
  dist2.right = read_sonar(6);
  dist2.left = read_sonar(7);
}

//set up the M7 to be the client and run the state machine
void setupM7() {
  // begin serial interface
  Serial.begin(9600);
  delay(1000);
}

//read sensor data from M4 and write to M7
void loopM7() {
  // read lidar data from struct
  struct lidar data = RPC.call("read_lidars").as<struct lidar>();
  struct sonar data2 = RPC.call("read_sonars").as<struct sonar>();
  // print lidar data
  Serial.print("lidar: ");
  Serial.print(data.front);
  Serial.print(", ");
  Serial.print(data.back);
  Serial.print(", ");
  Serial.print(data.left);
  Serial.print(", ");
  Serial.print(data.right);
  Serial.println();
  Serial.print("sonar: ");
  Serial.print(data2.left);
  Serial.print(", ");
  Serial.print(data2.right);
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
