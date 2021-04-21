#include "main.h"
/*
  add externs in main.h
  ex:
extern pros::motor left_mtr;
*/
using namespace pros;   //makes it so I dont have
                        //to constantly write pros::

// bad ports 1,6,7,20,21

//Motor ports

#define INTAKE_RIGHT_PORT         10
#define INTAKE_LEFT_PORT          14
#define ROLLER_RIGHT_PORT         2
#define ROLLER_LEFT_PORT          16
#define DRIVE_FRONT_RIGHT_PORT    5
#define DRIVE_BACK_RIGHT_PORT     4
#define DRIVE_FRONT_LEFT_PORT     17
#define DRIVE_BACK_LEFT_PORT      18

// sensor Ports

#define IMU_SENSOR_PORT           19
#define TOP_BALL_PORT             1 // A
#define BOT_BALL_PORT             2 // B
#define TRACKING_FIRST_PORT       3 // C marked silver
#define TRACKING_SECOND_PORT      4 // D
#define DISTANCE_SENSOR_PORT      3



const double circumference_m = 4.14*PI;     //calculates the circumference of
                                            // wheels connected to motors
const double circumference = 2.705*PI;
const double degToInchConstant = circumference/360;
const double inchToDegConstant = 360/circumference;

// used for the
double degToInch_t(double deg){return deg*degToInchConstant;}
double inchToDeg_t(double inch){return inch*inchToDegConstant;}
double radToDeg(double rad){return rad*57.29577951308232286;}
double degToRad(double deg){return deg*0.01745329251994329547;}

void delay(int delay){
  pros::c::taskDelay(delay);
  // delays a task by delay milliseconds
}
// used for back wheels of robot
int inchToDeg(float inch){
  float deg = 1.5*(inch/circumference_m)*360; //calculates the degrees to drive a
                                            //distance in inches
  return deg;                               // returns the amount of degrees
}

//pros::Motor name(port,gearset,bool reverse,encoder units)

Motor rightIntake(INTAKE_RIGHT_PORT, MOTOR_GEARSET_18,
              true, E_MOTOR_ENCODER_DEGREES);
Motor leftIntake(INTAKE_LEFT_PORT, MOTOR_GEARSET_18,
              false, E_MOTOR_ENCODER_DEGREES);
Motor rollerRight(ROLLER_RIGHT_PORT, MOTOR_GEARSET_6,
              false, E_MOTOR_ENCODER_DEGREES);
Motor rollerLeft(ROLLER_LEFT_PORT, MOTOR_GEARSET_6,
              true, E_MOTOR_ENCODER_DEGREES);
Motor driveFR(DRIVE_FRONT_RIGHT_PORT, MOTOR_GEARSET_18,
              false, E_MOTOR_ENCODER_DEGREES);
Motor driveBR(DRIVE_BACK_RIGHT_PORT, MOTOR_GEARSET_18,
              true, E_MOTOR_ENCODER_DEGREES);
Motor driveFL(DRIVE_FRONT_LEFT_PORT, MOTOR_GEARSET_18,
              true, E_MOTOR_ENCODER_DEGREES);
Motor driveBL(DRIVE_BACK_LEFT_PORT, MOTOR_GEARSET_18,
              false, E_MOTOR_ENCODER_DEGREES);

Imu gyro(IMU_SENSOR_PORT);
ADIEncoder tracking(TRACKING_FIRST_PORT,TRACKING_SECOND_PORT,true);
ADIPort top_ball(TOP_BALL_PORT,ADI_ANALOG_IN);
ADIPort bot_ball(BOT_BALL_PORT,ADI_ANALOG_IN);
Controller C1(E_CONTROLLER_MASTER);
Distance d1(DISTANCE_SENSOR_PORT);
