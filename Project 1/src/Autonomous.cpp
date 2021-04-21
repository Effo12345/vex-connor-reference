#ifndef AUTONOMOUS_CPP
#define AUTONOMOUS_CPP

#include "main.h"
#include "Functions.hpp"
//#include "odometry.hpp"

int Time1 = 0;
int Time2 = 0;

void shoopdewhoop(){
  set_intake(100);
  delay(1000);
  set_drive(50,-50);
  delay(1000);
  set_drive(0,50);
  delay(1000);
  deposit(1,0);
  set_drive(0,-50);
}

void one_ball(){
  set_intake(100);
  delay(100);
  driveradius(25,18,40,1,1.5);
  delay(100);
  driveAsync(10,-.125,2);
	//drive(50,3,2);
  deposit(3,2);
  drive_kill();
  delay(10);
  //set_bot();
  set_intake(-50);
	drive(50,-12,15);
}

void one_ball_right(){
  set_intake(100);
  delay(100);
  driveradius(25,18,40,0,1.5);
  delay(100);
  driveAsync(10,-.125,2);
	//drive(50,3,2);
  deposit(3,2);
  drive_kill();
  delay(10);
  //set_bot();
  set_intake(-50);
	drive(50,-12,15);
}

void two_ball(){
	set_intake(100);
  delay(100);
  driveradius(25,18,40,1,1.5);
  delay(100);
  driveAsync(10,-.125,2);
	//drive(50,3,2);
  deposit(3,2);
  drive_kill();
  delay(10);
  turn(100,315,1);
  delay(10);
  //set_bot();
  set_intake(-50);
	drive(50,-6,15);
  delay(100);
  turn(50,0,1);
  set_intake(-100);
  set_rollers(-100);
  delay(100);
	drive(50,-54,15);
  delay(10);
  set_rollers(0);
	turn(50,270,1.5);
  delay(500);
  set_intake(100);
	driveAsync(50,12,2);
  deposit(1,2);
  //driveAsync(10,-1,2);
  drive_kill();
  delay(500);
	drive(100,-12,15);
  delay(50);
}

void two_ball_right(){
  set_intake(100);
  delay(100);
  driveradius(25,18,40,0,1.5);
  delay(100);
  driveAsync(10,-.125,2);
  //drive(50,3,2);
  deposit(3,2);
  drive_kill();
  delay(10);
  set_Ki_active(45);
  turn(100,45,1);
  set_Ki_active(10);
  delay(10);
  //set_bot();
  drive(50,-6,15);
  delay(100);
  turn(50,10,2.5);
  set_intake(-100);
  set_rollers(-100);
  delay(100);
  drive(50,-53,15);
  delay(10);
  set_rollers(0);
  turn(50,90,1.5);
  delay(500);
  set_intake(100);
  driveAsync(50,15,2);
  deposit(1,2);
  //driveAsync(10,-1,2);
  drive_kill();
  delay(500);
  drive(100,-12,15);
  delay(50);
}

void three_ball(){
  set_intake(100);
  delay(50);
  //driftDrive(100,20,1,315,1);
  driveradius(25,18,40,1,1);
  //driveradius(100,9,60,1,1.5);
  delay(10);
  //drive(50,3,2);
  //deposit(3,2);
  driveAsync(10,-.375,1);
  deposit(3,2);
  drive_kill();
  delay(10);
  driftDriveAsync(100,-62,-1,0,3);
  change_heading_wait();
  delay(500);
  set_intake(-100);
  set_rollers(-100);
  drive_wait();
  set_intake(0);
  set_rollers(0);
  delay(10);
  turn(50,270,1);
  delay(10);
  set_intake(100);
  driveAsync(100,24,2);
  delay(750);
  deposit(1,2);
  set_intake(-100);
  //driveAsync(10,-1,2);
  drive_kill();
  delay(10);
  driveAsync(100,-12,15);
  delay(100);
  set_intake(-100);
  set_rollers(100);
  drive_wait();
  delay(50);
  turn(180,180,1);
  delay(10);
  driftDriveAsync(100,65,18,225,2);
  delay(10);
  deposit(2,3);
  drive_kill();
  delay(10);
  drive(100,-10,12);
}

void skills(){
  /*
  movements:
  movement 1: move forward
  movement 2: turn and move into goal
    premove: deposit
  movement 3: back up to white line
  movement 4: drive all the way to oppo home row
    premove: intake_ball
  movement 5: drive all the way to oppo corner
  movement 6: drive into corner goal
    premove: deposit
  movement 7: drive to white line
  movement 8: drive to double white line
  movement 9: turn to center goal and hit out ball
    premove: deposit
  movement 10: back away from goal
  */

  int s = 40; // speed of auton movements in RPM
  // premove 1
  // deployment
  intake_ball();
  chamber();
  // movement 1
  drive(s,23,3);
  // premove 2
  quit_intake();
  // movement 2
  delay(100);
  turn(s*1.5,135,2);
  delay(100);
  drive(s,29,2);
  delay(500);
  turnAsync(100,135,5);
  // premove 3
  deposit(1,0);
  stage_down();
  chamber();
  delay(500);
  // movement 3
  drive(s,-8,2);
  // premove 4
  delay(10);
  intake_ball();
  // movement 4
  turn(s*1.5,270,3);
  delay(10);
  turn(s*1.5,270,1);
  delay(10);
  drive(s,88,10);
  delay(10);
  // premove 5
  // movement 5
  turn(s*1.5,358,2);
  delay(10);
  turn(s*1.5,0,2);
  delay(10);
  drive(s,88,5);
  delay(10);
  // premove 6
  //
}


#endif
