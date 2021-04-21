

#include "main.h"
#include "Functions.hpp"

void skillsAuton(){

  // Structure of Program
  // premove: activities done before moving to a new position
  //          normally a deposit or turning on intakes
  // movement: a turn followed by a drive
  //           some contain an Async turn to fix heading
  // Async Action: actions done during an asynchronous
  //               movement
  //               normally chambering a ball



  int s = 40; // speed of auton movements in RPM
  // premove 1
  // deployment
  intake_ball();
  chamber();
  // movement 1
  drive(s*.8,21.5,3);
  // premove 2
  quit_intake();
  // movement 2
  delay(100);
  turn(s*1.5,135,2);
  delay(100);
  drive(s,30,2);
  delay(500);
  set_Ki_turn(1);
  set_Ki_active(45);
  // premove 3
  deposit(1,0);
  turn(100,135,3.5);
  delay(500);
  // movement 3
  set_Ki_turn(0);
  set_Ki_active(10);
  delay(20);
  driveAsync(s,-47.75,3);
  // movement 3 Async Action
  delay(500);
  set_intake(100);
  chamber();
  set_intake(0);
  drive_wait();
  // premove 4
  intake_ball();
  // movement 4
  turn(s*1.5,270,1);
  delay(10);
  set_Ki_turn(1);
  set_Ki_active(45);
  turn(1,270,2);
  set_Ki_turn(0);
  set_Ki_active(10);
  delay(100);
  drive(s,22,2);
  delay(100);
  // pre move 5
  quit_intake();
  // movement 5
  turn(s*1.5,175,2);
  delay(50);
  drive(s,34.5,2);
  delay(50);
  set_Ki_turn(1);
  set_Ki_active(45);
  turnAsync(100,180,2);
  // premove 6
  deposit(1,0);
  delay(500);
  // movement 6
  drive_kill();
  set_Ki_turn(0);
  set_Ki_active(10);
  delay(50);
  driveAsync(s/2,-7.5,2);
  // movement 6 Async action
  delay(500);
  set_intake(100);
  chamber();
  set_intake(0);
  drive_wait();
  delay(100);
  // premove 7
  intake_ball();
  // movement 7
  turn(s*1.5,270,1);
  delay(50);
  set_Ki_turn(1);
  set_Ki_active(45);
  turn(1,270,2);
  set_Ki_turn(0);
  set_Ki_active(10);
  delay(10);
  drive(s,42,2);
  delay(50);
  // premove 8
  // movement 8
  turn(s*1.5,225,1);
  delay(50);
  drive(s,22,1);
  delay(50);
  set_Ki_turn(1);
  set_Ki_active(45);
  turnAsync(100,225,3);
  // premove 9
  deposit(1,0);
  delay(1000);
  // movement 9
  drive_kill();
  set_Ki_turn(0);
  set_Ki_active(10);
  delay(100);
  driveAsync(2*s/5,-17.5,2);
  // movement 9 Async action
  delay(500);
  set_intake(100);
  chamber();
  set_intake(0);
  delay(10);
  drive_wait();
  delay(100);
  // premove 10
  intake_ball();
  // movement 10
  turn(s*1.5,0,2);
  delay(100);
  set_Ki_turn(1);
  set_Ki_active(10);
  turn(1,0,2);
  set_Ki_turn(0);
  set_Ki_active(10);
  delay(10);
  drive(s,46,2);
  delay(10);
  // premove 11

  // movement 11
  turn(s*1.5,260,2);
  delay(100);
  drive(s,12,1);
  set_Ki_turn(1);
  set_Ki_active(45);
  delay(100);
  turnAsync(100,270,2);
  // premove 12
  deposit(1,0);
  //stage_down();
  delay(750);
  // movement 12
  drive_kill();
  set_Ki_turn(0);
  set_Ki_active(10);
  delay(10);
  driveAsync(s*.75,-19.5,2);
  // Async Action movement 12
  delay(100);
  set_intake(100);
  chamber();
  set_intake(0);
  drive_wait();
  delay(500);
  /*
  // premove 13
  // movement 13
  turn(s*1.5,338,2);
  delay(10);
  delay(10);
  drive(s,66,4);
  set_Ki_turn(.01);
  turnAsync(100,315,3);
  // premove 14
  deposit(1,0);
  delay(1000);
  // movement 14
  drive_kill();
  delay(10);
  drive(s,-4,2);
  // premove 15
  intake_ball();
  delay(100);
  // movement 15
  turn(s*1.5,90,2);
  delay(10);
  drive(s,42,4);
  delay(10);
  // premove 16
  chamber();
  // movement 16
  turn(s*1.5,10,2);
  delay(10);
  turn(s,1,2);
  */
  // premove 13
  intake_ball();
  // movement 13
  turn(s*1.5,0,2);
  delay(100);
  drive(s,44,4);
  delay(100);
  // premove 14
  // movement 14
  turn(s*1.5,300,1);
  delay(10);
  drive(s,31,2);
  delay(10);
  //turnAsync(100,330,2);
  // premove 15
  set_Ki_turn(1);
  set_Ki_active(45);
  deposit(1,0);
  turn(100,300,1.5);
  // movement 15
  drive_kill();
  set_Ki_turn(0);
  set_Ki_active(10);
  delay(500);
  drive(s,-63,3);
  // premove 16
  set_intake(100);
  chamber();
  // movement 16
  delay(10);
  turn(s*1.5,180,1);
  delay(50);
  set_intake(-100);
  set_drive(100,100);
  delay(1000);
  set_rollers(100);
  delay(500);
  // mid goal rush
  //shoopdewhoop();
  // premove 17
  // movement 17
  drive(100,-12,2);
  delay(50);
  // 72 points baby
  set_drive(100,-100);
}
