#include "main.h"

void initialize() {
  lcd::initialize();
    inertial.reset();

  fr.set_brake_mode(E_MOTOR_BRAKE_COAST);
  br.set_brake_mode(E_MOTOR_BRAKE_COAST);
  fl.set_brake_mode(E_MOTOR_BRAKE_COAST);
  bl.set_brake_mode(E_MOTOR_BRAKE_COAST);
  fly.set_brake_mode(MOTOR_BRAKE_BRAKE);
  pros::delay(10);
}
