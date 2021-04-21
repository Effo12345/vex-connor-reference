#include "main.h"
#include "Functions.hpp"

// file for functions dealing mainly with the rubber band roller stack

/******************************************************************************/
/*          Autonomous Functions                                              */
/******************************************************************************/

timer T1;


//checks if a ball is ready to be deposited
int chambered(){
  if(top_ball.get_value() < 2200)
    return true;
  else
    return false;
}
// checks if a ball is ready with looser tolerances
int chamberedHigh(){
  if(top_ball.get_value() < 2700)
    return true;
  else
    return false;
}

int low_set(){
  if(bot_ball.get_value() < 2400)
    return true;
  else
    return false;
}

void set_rollers(int speed){
  rollerRight.move_voltage(speed*1200);
  rollerLeft.move_voltage(speed*1200);
}

void set_rollers_velocity(int speed){
  rollerRight.move_velocity(speed*6);
  rollerLeft.move_velocity(speed*6);
}

void rollDeg(int speed, int deg, bool wait){
  rollerRight.move_relative(deg,speed*6);
  rollerLeft.move_relative(deg,speed*6);
  if(wait){
    while(rollerRight.is_stopped() == 0 || rollerLeft.is_stopped() == 0){
      pros::c::taskDelay(10);
    }
  }
}

void chamber(){
  bool already_there = chambered();
  while(chambered() != true && already_there != true){
    set_rollers_velocity(40);
    delay(10);
  }
  set_rollers(0);
  delay(100);
  while(chambered() != true && already_there != true){
    set_rollers_velocity(-25);
    delay(10);
  }
  rollDeg(600,-400,true);
  set_rollers(0);
}

void stage(bool top_ball, bool bottom_ball, float time){
  bool top_done = false;
  bool bottom_done = false;
  bool through = false;
  int time_out = time*1000;
  T1.reset();
  while((top_done != true || bottom_done != true) && time_out > T1.time()){
    if(top_ball && top_done != true){
      if(chambered() != true)
        set_rollers_velocity(25);
      else{
        set_rollers(-100);
        top_done = true;
        delay(10);
      }
    }
    else{
      set_rollers(0);
      top_done = true;
    }
    if((bottom_ball && bottom_done == false) ||
      (top_done != true && through != true)){
      if(in() != true)
        set_intake(100);
      else{
        if(top_done != true && through != true)
          through = true;
        else
          bottom_done = true;
        set_intake(0);
      }
    }
    else{
      set_intake(0);
      bottom_done = true;
    }
    delay(10);
  }
  set_intake(0);
  set_rollers(0);
}
void stage_bot(bool top_ball, bool bottom_ball, float time){
  bool top_done = false;
  bool bottom_done = false;
  bool through = false;
  int time_out = time*1000;
  T1.reset();
  while((top_done != true || bottom_done != true) && time_out > T1.time()){
    if(top_ball && top_done != true){
      if(low_set() != true)
        set_rollers_velocity(25);
      else{
        set_rollers(-100);
        top_done = true;
        delay(10);
      }
    }
    else{
      set_rollers(0);
      top_done = true;
    }
    if((bottom_ball && bottom_done == false) ||
      (top_done != true && through != true)){
      if(in() != true)
        set_intake(100);
      else{
        if(top_done != true && through != true)
          through = true;
        else
          bottom_done = true;
        set_intake(0);
      }
    }
    else{
      set_intake(0);
      bottom_done = true;
    }
    delay(10);
  }
  set_intake(0);
  set_rollers(0);
}

void set_bot(){
  while(low_set() != true){
    set_rollers_velocity(-10);
    delay(10);
  }
  set_rollers(0);
}

void set_bot_up(){
  while(low_set() != true){
    set_rollers_velocity(50);
    delay(10);
  }
  set_rollers(0);
  delay(100);
  while(low_set() != true){
    set_rollers_velocity(-25);
    delay(10);
  }
  set_rollers(0);
}

int stage_down(){
  T1.reset();
  while(in() != true && T1.time() < 1000){
    set_rollers(-100);
    delay(10);
  }
  if(T1.time() < 1000)
    return 1;
  else
    return 0;
  set_rollers(0);
}

void shoot(){
  rollDeg(1400,600,1);
}

void deposit(int balls, int Auhfswitch){
  int deposited = 0;                  // keeps track of balls deposited
  int intook    = 0;                  // keeps track of balls taken in
  bool in_progress = false;           // keeps track of when a ball is leaving
                                      // the robot
  bool in_intake = false;
  int start_time = 0;                 // used to keep track of when last ball
                                      // leaves
  bool chamberedEnd = false;          // flags if deposit ended on a ball
                                      // reaching the sensor
  bool delayed  = false;
  bool intake_delayed = false;
  int  counter1 = 0;
  int  counter2 = 0;
  set_rollers(110);
  while(deposited != balls){
    if(chamberedHigh() && in_progress != true){
      in_progress = true;
    }
    else if(in_progress && delayed != true){
      counter1++;
      if(counter1 > 10){
        counter1 = 0;
        delayed = true;
      }
    }
    else if(in_progress && chamberedHigh() != true){
      deposited++;
      in_progress = false;
      delayed = false;
    }
    if(intook < Auhfswitch){
      set_intake(100);
      if(in() && in_intake != true){
        in_intake = true;
        intook++;
      }
      else if(in_intake && intake_delayed != true){
        counter2++;
        if(counter2 > 15){
          counter2 = 0;
          intake_delayed = true;
        }
      }
      else if(in_intake && in() != true){
        in_intake = false;
        intake_delayed = false;
      }
    }
    else{
      set_intake(0);
    }
    delay(10);
  }
  delay(50);
  set_intake(0);
  T1.reset();
  while(T1.time() < 500){
    if(chambered()){
      break;
      delay(100);
    }
    delay(1);
  }
  set_rollers(-12000);
  delay(100);
  set_rollers_velocity(0);
  set_intake(0);
}



/******************************************************************************/
/*          Driver Control Functions                                          */
/******************************************************************************/

// integers for driver control

bool autoChamberSwitch = false;
bool btn_prsd_roller = false;

// macros (for future use)

double indexDown = 1;
void autoChamber(){
  if(chamberedHigh() != true){
    rollerRight.move_voltage(7000);
    rollerLeft.move_voltage(7000);
  }
  else{
    rollerRight.move(0);
    rollerLeft.move(0);
  }
  if(in() != true || (chamberedHigh() != true && low_set() != true)){
    set_intake(100);
  }
  else{
    set_intake(0);
  }
}

void stage_down_auto(){
  if(in() != true){
    set_rollers(-25);
    delay(10);
  }
  set_rollers(0);
}

// Function to be recalled into driver control

// short for operator rollers
void OProllers(){
  if(C1.get_digital(DIGITAL_L1) || C1.get_digital(DIGITAL_R2) == 1){
    rollerRight.move_voltage(12000);
    rollerLeft.move_voltage(12000);
  }
  else if(C1.get_digital(DIGITAL_L2)){
    rollerRight.move_voltage(-12000);
    rollerLeft.move_voltage(-12000);
  }
  else{
    rollerRight.move(0);
    rollerLeft.move(0);
  }
}

/*

relative:
360
360
 = 720

 absolute:
 360
 360
  = 360



*/


void OProllersC(){
  if(C1.get_digital(DIGITAL_L1)){
    rollerRight.move_voltage(12000);
    rollerLeft.move_voltage(12000);
    leftIntake.move_voltage(12000);
    rightIntake.move_voltage(12000);
    indexDown = 1;
  }
  else if(C1.get_digital(DIGITAL_L2) && C1.get_digital(DIGITAL_R2)){
    rollerRight.move_voltage(12000);
    rollerLeft.move_voltage(12000);
    leftIntake.move_voltage(-12000);
    rightIntake.move_voltage(-12000);
    indexDown = 1;
  }
  else if(C1.get_digital(DIGITAL_L2)){
    rollerRight.move_voltage(12000);
    rollerLeft.move_voltage(12000);
    leftIntake.move(0);
    rightIntake.move(0);
    indexDown = 1;
  }
  else if(C1.get_digital(DIGITAL_R2)){
    leftIntake.move_voltage(-12000);
    rightIntake.move_voltage(-12000);
    rollerRight.move(-12000);
    rollerLeft.move(-12000);
  }
  else if(C1.get_digital(DIGITAL_R1))
    autoChamber();
  else if(C1.get_digital(DIGITAL_X))
    stage_down_auto();
  else{
    rollerRight.move(0);
    rollerLeft.move(0);
    leftIntake.move(0);
    rightIntake.move(0);
  }
}
