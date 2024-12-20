/*Lidar2024.ino
       This example is the Pololu Lidar object detection sensor
       It produced a digital output to indicate the presence or abesnce of an object
       https://www.pololu.com/product/4052/blog
       This small lidar-based distance sensor detects the presence of objects within 10 cm (4″). 
       It has a single digital output that drives low when an object is being detected; otherwise, it is high. 
       CAB 12.3.23
*/

//define the constant and variables
#define FRONT 0
#define BANK 1
#define LEFT 2
#define RIGHT 3
#define numOfSens 4

uint16_t wait = 100;
int16_t ft_lidar = 8;
int16_t bk_lidar = 9;
int16_t lt_lidar = 10;
int16_t rt_lidar = 11;
int16_t lidar_pins[4] = {8,9,10,11 };


void setup() {
  int baud_rate = 9600;  //baud rate for serial communication
  for (int i = 0; i<numOfSens;i++){
    pinMode(lidar_pins[i],OUTPUT);
  }

  Serial.begin(baud_rate);
  delay(wait);
}

//thie function will read the left or right sensor based upon input value
int readLidar(uint16_t side) {
  int16_t t = pulseIn(lidar_pins[side], HIGH);
  int d; //distance to  object
  if (t == 0){
    // pulseIn() did not detect the start of a pulse within 1 second.
    //Serial.println("timeout");
    d = 100000; //no object detected
  }
  else if (t > 1850)  {
    //Serial.println("timeout");
    d = 100000; //no object detected
  }
  else  {
    // Valid pulse width reading. Convert pulse width in microseconds to distance in millimeters.
    d = (t - 1000) * 3 / 40;
 
    // Limit minimum distance to 0.
    if (d < 0) { d = 0; } 
  }
  //   Serial.print(d);
  // Serial.print(" cm, ");
  return d;
}


void loop() {
  for (int i = 0;i<4;i++){
    Serial.print(i);
    Serial.print(": ");
    Serial.print(readLidar(i));
    Serial.print(" ");
    delay(wait);
  }
  Serial.println();
}