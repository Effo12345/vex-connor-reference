#ifndef PID_HPP
#define PID_HPP

#include "main.h"

extern double kP; // proportional multiplier
extern double turnkP; // turning proportional multiplier

extern int degreesPerTurn; // how many degrees of rotation needed to turn 360 degrees

extern int driveFor; // the target value
extern int turnTo; //

extern int desiredValue;
extern int desiredTurnDegrees;

extern int error; //sensorValue - desired value :positon
extern int prevError; //position 20 miliseconds ago
extern int derivative; // error - prevError : speed

extern int turnError; //sensorValue - desired value :positon
extern int turnPrevError; //position 20 miliseconds ago
extern int turnDerivative; // error - prevError : speed

extern int leftDist; // left tracking wheel degrees
extern int rightDist; // right tracking wheel degrees
extern int xDist; // back tracking wheel degrees

extern bool resetDriveSensors;
extern bool enableDriveP;

int driveP();

#endif
