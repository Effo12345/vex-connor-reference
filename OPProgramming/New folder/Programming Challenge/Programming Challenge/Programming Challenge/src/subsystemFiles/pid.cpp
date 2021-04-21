#include "main.h"
double kP = 0.33; // proportional multiplier
double turnkP = 0.33; // turning proportional multiplier
int degreesPerTurn = 3260; // how many degrees of rotation needed to turn 360 degrees


int driveFor = 0;
int turnTo = 0;

int desiredValue = 0;
int desiredTurnDegrees = 0;

int error = 0; //sensorValue - desired value :positon
int prevError = 0; //position 20 miliseconds ago
int derivative = 0; // error - prevError : speed
int turnError = 0; //sensorValue - desired value :positon
int turnPrevError = 0; //position 20 miliseconds ago
int turnDerivative = 0; // error - prevError : speed

int leftDist = 0; // left tracking wheel degrees
int rightDist = 0; // right tracking wheel degrees
int xDist = 0; // back tracking wheel degrees

bool resetDriveSensors = false;
bool enableDriveP = false;
//The whole P loop
int driveP() {
  //Constants


  while (enableDriveP == true) {
    if (resetDriveSensors) {
      resetDriveSensors = false;
      leftTrack.reset();
      rightTrack.reset();
      backTrack.reset();
    }
    if (driveFor != 0) {
      desiredValue = desiredValue + driveFor;
      driveFor = 0;
    }

    leftDist = leftTrack.get_value(); //wheel pos
    rightDist = rightTrack.get_value(); //wheel pos
    xDist = backTrack.get_value(); //wheel pos


    int averagePos = (leftDist + rightDist)/2; //average position
    error = desiredValue - averagePos; //proportional
    derivative = error - prevError; //derivative

    double lateralMotorPower = error * kP + derivative; //lateral power


    desiredTurnDegrees = turnTo * (degreesPerTurn/360);
    int turnDifference = (leftDist) - (rightDist); //average position
    turnError = desiredTurnDegrees - turnDifference; //proportional
    turnDerivative = turnError - turnPrevError; //derivative

    double turnMotorPower = turnError * turnkP + turnDerivative; // turn Power


    // set the motor voltage
    fl = lateralMotorPower + turnMotorPower;
    bl = lateralMotorPower + turnMotorPower;
    fr = lateralMotorPower - turnMotorPower;
    br = lateralMotorPower - turnMotorPower;

    // set the previous error
    prevError = error;
    turnPrevError = turnError;


    // display values on the brain
    lcd::set_text(1,std::to_string(leftDist)); // display the left tracking wheel distance
    lcd::set_text(2,std::to_string(rightDist)); // display the right tracking wheel distance
    lcd::set_text(3,std::to_string(xDist)); // display the back tracking wheel distance
    encoderGyro(4); // display the current heading based on tracking wheel degrees



    delay(10); // delay 10 miliseconds so we dont waste cpu resources

  }
  return 1;
}
