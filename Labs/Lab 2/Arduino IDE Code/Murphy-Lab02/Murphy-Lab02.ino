//Justin Dewitt 12.12.23
// read about multicore: https://docs.arduino.cc/tutorials/giga-r1-wifi/giga-dual-core
//This code will show how to run the same code on the M4 and M7 to server and client, respectively
//M4 will read sensor data and send to M7 to run the state machine


#include "Arduino.h"
#include "RPC.h"

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

// read_lidars is the function used to get lidar data to the M7
struct lidar read_lidars() {
  return dist;
}

// reads a lidar given a pin
int read_lidar(int pin) {
  int16_t t = pulseIn(pin, HIGH);
  if (t == 0 || t > 1850)
    return 100000;
  int d = (t - 1000) * 3 / 40;
  if (d < 0) { d = 0; }
  return d;
}

//set up the M4 to be the server for the sensors data
void setupM4() {
  RPC.bind("read_lidars", read_lidars);  // bind a method to return the lidar data all at once
}

//poll the M4 to read the data
void loopM4() {
  // update the struct with current lidar data
  dist.front = read_lidar(8);
  dist.back = read_lidar(9);
  dist.left = read_lidar(10);
  dist.right = read_lidar(11);
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
  // print lidar data
  Serial.print("data: ");
  Serial.print(data.front);
  Serial.print(", ");
  Serial.print(data.back);
  Serial.print(", ");
  Serial.print(data.left);
  Serial.print(", ");
  Serial.print(data.right);
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
// this may need to be modified to run the state machine.
// consider usingnamespace rtos Threads as seen in previous example
void loop() {}
