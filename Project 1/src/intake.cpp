#include "main.h"

// file for functions mainly dealing with the intake chain rollers

/******************************************************************************/
/*          Autonomous Functions                                              */
/******************************************************************************/


int time_out_lower_stage = 0;
bool quit_stage_lower = false;

timer T2;

bool in(){
  if(bot_ball.get_value() < 1700)
    return true;
  else
    return false;
}

void set_intake(int speed){
  leftIntake.move_voltage((speed/100)*12000);   //adjusts speed so that it is in
  rightIntake.move_voltage((speed/100)*12000);  // percent
}

void intDeg(int speed, int deg, bool wait){
  leftIntake.move_relative(deg,speed);
  rightIntake.move_relative(deg,speed);
  if(wait){
    while(leftIntake.is_stopped() == 0){
      pros::c::taskDelay(10);
    }
  }
}

void stage_lower(){
  T2.reset();
  while(quit_stage_lower != true){
    set_intake(100);
    delay(10);
    if(in())
      quit_stage_lower = true;
  }
  delay(100);
  set_intake(0);
}

void intake_ball(){
  quit_stage_lower = false;
  delay(10);
  pros::Task intake_task (stage_lower,"intake_task");
}

void quit_intake(){
  quit_stage_lower = true;
}

void deploy(){
  set_intake(100);
  delay(100);
  set_intake(0);
}

/******************************************************************************/
/*          Driver Control Functions                                          */
/******************************************************************************/

// macros (for future use)


// Function to be recalled into driver control

// short for operator intake
void OPintake(){
  //used to be R
  if(C1.get_digital(DIGITAL_L1) == 1 || C1.get_digital(DIGITAL_R1) == 1){
    leftIntake.move_voltage(12000);
    rightIntake.move_voltage(12000);
  }
  else if(C1.get_digital(DIGITAL_L2) == 1){
    leftIntake.move_voltage(-12000);
    rightIntake.move_voltage(-12000);
  }
  else{
    leftIntake.move(0);
    rightIntake.move(0);
  }
}
