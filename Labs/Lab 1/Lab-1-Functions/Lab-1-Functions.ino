/*
Authors: Benjamin Williams & Donald Woodruff
12/4/2024
Program Description:
Basic functions for robot movement

Summary of Elements:
goToAngle   --> put robot header at a certain angle using encoder feedback
goToGoal    --> Go to goal position
circle      --> Robot drives in a circle of prescribed diameter
figureEight --> Robot drives in a figure eight
square      --> Robot drives in a square of specific size
*/

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}

/*
Function: getAngle
Description:
Gets robot's angle
E.g. Robot is at a heading of 30 degrees
Robot will return 30 degrees
*/
int getAngle() {
  return robot.encoder.getAngle() // get angle from the gyro or encoder.
}

/*
Function: turn
Description:
Turns robot by a number of degrees
E.g. Robot is at a heading of 30 degrees, and input is a heading of 20 degrees
Robot will turn 20 degrees to the right, leading to a final heaidng of 50 degrees
*/
void turn(int turnAngle) {
  int currentAngle = robot.getAngle()
  int desiredAngle = startAngle + turnAngle;
  while(abs(desiredAngle-startAngle) > turnError) {   // While the angle is less then the input angle +/- an acceptable error, run below:
    robot.motorSteps((inputAngle-currentAngle) * conversion) // Find difference in angle, and then tell robot to turn that much based off of a conversion factor
    currentAngle = robot.getAngle()     // Get the current angle of the robot to see if while can be exited
  }
}

/*
Function: goToAngle
Description:
Takes in desired angle of robot, and tells robot to go to that specific angle. 
E.g. Robot is at a heading of 30 degrees, and input is a heading of 20 degrees,
then robot will move heading by -10 degrees.
*/
void goToAngle(int desiredAngle) {
  currentAngle = robot.getAngle() //Get the currentangle of the robot
  robot.turn(desiredAngle-currentAngle) // Find difference in angle, and then tell robot to turn that much
}

/*
Function: goToGoal
Description:
Takes in desired position of the robot, and robot moves to that position
E.g. Robot is at a position of (2,0), and input is a position of (3,0)
Robot will turn to an angle of 0, and then move forward 1, to get to position of (3,0).
*/
void goToGoal(int[] desiredPosition) {
  int currentPosition[] = robot.gyro.getPosition();
  desiredAngle = arctan((desiredPosition[2]-currentPosition[2])/(desiredPosition[1]-currentPosition[1]))
  robot.goToAngle(desiredAngle);
  int currentPosition[] = robot.gyro.getPosition();
}

/*
Function: goDistance
Description:
Robot goes forward a specific distance
E.g. Robot is at a position of (2,0) with a heading facing due East, and input of 1.
Robot will move forward 1, meaning the ending position will be (3,0)

void goDistance(int distance) {
  int currentEncoder = robot.encoder.getAngle();
  int desiredEncoder = currentEncoder + distance*conversion;
  while(abs(currentEncoder*distanceConversion-distance) > distanceError) {   // While the angle is less then the input angle +/- an acceptable error, run below:
    robot.motorSteps((currentEncoder * conversion) // Find difference in angle, and then tell robot to turn that much based off of a conversion factor
    currentAngle = robot.getAngle()     // Get the current angle of the robot to see if while can be exited
  }
  robot.motorSteps(distance) // Move steppers a certain number of steps based off of calculated conversion
}
*/
