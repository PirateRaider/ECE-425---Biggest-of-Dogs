//MultiCore_Sensors_M7server.ino
//based off work of Justin Dewitt 12.12.23
//CAB 12.26.23
//This code will show how to run the same code on the M4 and M7 to server and client, respectively
//M4 will read sensor data and send to M7 to run the state machine
//upload M4 first and then turn on M7 serial monitor
//  Read more here: Guide to GIGA R1 Dual Cores | Arduino Documentation
//  https://docs.arduino.cc/tutorials/giga-r1-wifi/giga-dual-core
//  https://docs.arduino.cc/tutorials/giga-r1-wifi/cheat-sheet
//  To upload to the main core click Tools->Flash Split: "1.5MB M7 + 0.5MB M4""
//  Then click Tools->Target Core" "Main Core"
//  Click Check to Verify and Right Arrow to Upload
// M7 is the server and M4 is the client


#include "Arduino.h"
#include "RPC.h"

using namespace rtos;

Thread sensorThread;

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
  float velocity((331.5 + 0.6 * (float)(20)) * 100 / 1000000.0);
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

/**
 * Returns the CPU that's currently running the sketch (M7 or M4)
 * Note that the sketch has to be uploaded to both cores. 
 **/
String currentCPU() {
  if (HAL_GetCurrentCPUID() == CM7_CPUID) {
    return "M7";
  } else {
    return "M4";
  }
}

void callSensorReadFromM4() {
  while (true) {
    delay(50);
    dist.front = read_lidar(8);
    dist.back = read_lidar(9);
    dist.left = read_lidar(10);
    dist.right = read_lidar(11);
    dist2.right = read_sonar(6);
    dist2.left = read_sonar(7);

    RPC.println(currentCPU() + ": front: " + String(dist.front) + ", back: " + String(dist.back) + ", left: " + String(dist.left) + ", right: " + String(dist.right));
    RPC.println(currentCPU() + ": front left: " + String(dist2.left) + ", front right: " + String(dist2.right));
  }
}

void setup() {

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(12, OUTPUT);

  // Initialize RPC library; this also boots the M4 core
  RPC.begin();
  Serial.begin(115200);
  delay(100);
  Serial.println("starting...");
  while (!Serial) {}  // Uncomment this to wait until the Serial connection is ready

  // Both CPUs will execute this instruction, just at different times
  randomSeed(analogRead(A0));  // Initializes the pseudo-random number generator

  if (currentCPU() == "M7") {
    RPC.bind("read_lidars", read_lidars);
    RPC.bind("read_sonars", read_sonars);
  }

  if (currentCPU() == "M4") {
    sensorThread.start(callSensorReadFromM4);
  }
}

void loop() {
  if (currentCPU() == "M4") {
    delay(50);
    dist.front = read_lidar(8);
    dist.back = read_lidar(9);
    dist.left = read_lidar(10);
    dist.right = read_lidar(11);
    dist2.right = read_sonar(6);
    dist2.left = read_sonar(7);

    RPC.println(currentCPU() + ": front: " + String(dist.front) + ", back: " + String(dist.back) + ", left: " + String(dist.left) + ", right: " + String(dist.right));
    RPC.println(currentCPU() + ": front left: " + String(dist2.left) + ", front right: " + String(dist2.right));
  }

  if (currentCPU() == "M7") {
    String buffer = "";
    while (RPC.available()) {
      buffer += (char)RPC.read();  // Fill the buffer with characters
    }

    if (buffer.length() > 0) {
      Serial.print(buffer);
    }
  }
}
