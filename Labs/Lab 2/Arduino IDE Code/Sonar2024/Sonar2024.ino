/*Sonar2024.ino
       This example is the ultrasonic distance measurement of the module.
       This code will confirm that the left and right sonar on the GigaBot are working

       Copyright   [DFRobot](http://www.dfrobot.com), 2020
       Copyright   GNU Lesser General Public License

       version  V1.0
       date  29/10/2020
       https://www.pololu.com/product/2476
       This code will output an analog signal in cm or inches proportional to distance to an object.
       CAB 12.3.23
*/

//define the constant and variables
#define VELOCITY_TEMP(temp) ((331.5 + 0.6 * (float)(temp)) * 100 / 1000000.0)  // The ultrasonic velocity (cm/us) compensated by temperature
#define RIGHT 0
#define LEFT 1

int16_t rt_trigechoPin = 3;
int16_t lt_trigechoPin = 4;
int16_t trig_EchoPin[2] = { 3,4 };
uint16_t wait = 250;

void setup() {
  int baud_rate = 9600;  //baud rate for serial communication
  Serial.begin(baud_rate);
  delay(wait);
}

//thie function will read the left or right sensor based upon input value
uint16_t readSonar(uint16_t side) {
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
}

void loop() {
  //uncomment next 2 lines to test left sonar
  Serial.print("LEFT: ");
  readSonar(LEFT);

  //uncomment next 2 lines to test right sonar
  Serial.print("RIGHT: ");
  readSonar(RIGHT);
  
  delay(500);
}