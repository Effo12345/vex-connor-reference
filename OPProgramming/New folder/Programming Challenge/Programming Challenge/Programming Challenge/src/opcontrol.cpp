#include "main.h"
using namespace pros;

void opcontrol() {
/*
  fl.set_brake_mode(MOTOR_BRAKE_BRAKE);
  bl.set_brake_mode(MOTOR_BRAKE_BRAKE);
  fr.set_brake_mode(MOTOR_BRAKE_BRAKE);
  br.set_brake_mode(MOTOR_BRAKE_BRAKE);
*/
  enableDriveP = false;

  while(true) {

  //Controls the drive motors
    index();
    setDriveMotors();
    encoderGyro(5);

    lcd::set_text(1,std::to_string(leftTrack.get_value()));
    lcd::set_text(2,std::to_string(rightTrack.get_value()));
    lcd::set_text(3,std::to_string(backTrack.get_value()));
    lcd::set_text(4,std::to_string(inertial.get_heading()));
    
    pros::delay(10);
  }
}
