#include "main.h"

void autonomous() {

  // set the motors to coast
  fl.set_brake_mode(MOTOR_BRAKE_COAST);
  bl.set_brake_mode(MOTOR_BRAKE_COAST);
  fr.set_brake_mode(MOTOR_BRAKE_COAST);
  br.set_brake_mode(MOTOR_BRAKE_COAST);

  enableDriveP = true; // tell the P loop to run
  Task go(driveP); // start the P loop as a seperate task


//start intaking and go to the first tower
intake(-127,-127); // intake the first ball
delay(400); // wait for flip out

turnTo = -145; // set the turning target to -145 so we end up facing the goal
driveFor = 1120; // set the average target to 1120, so the P loop takes the robot to the goal and curves as it does
setRoll(30,0); // start the lower rollers so that they start indexing the ball we just intaked


//score the first ball
waitR(1400, 100); // wait for the right tracking wheel to be within 100 degrees from 1400
setRoll(40,127); // deploy the hood and shoot the first ball
intake(-10,-10); // start outtaking so we dont pick up any blue balls by accident
delay(800); //wait to come to a complete stop


//outtake and back up
kP = 0.42; // increase the proportional value so the turns are faster and more accurate, although the overshot takes extra time to compensate for
turnkP = 0.6; // increase the proportional value so the turns are faster and more accurate, although the overshot takes extra time to compensate for
setRoll(0,0); // stop all of the rollers
driveFor = -700; // back out of the tower and line up with the next ball
delay(1200); // wait for the robot to have backed up and came to a complete stop


// turn to the next ball
turnTo = 1; // set the target heading to 5 degrees
delay(1000); // wait for the turn to finish and have stopped completely


// drive and grab the ball
driveFor = 1840; // drive to be directly in front of the second tower
intake(-127,-127); // start full intaking, so we can get the ball
setRoll(40,0); // index the ball already in the robot and prepare to index the newly intaked ball
delay(1500); // wait for the robot to have reached the target and stopped


//turn to the second tower
turnTo = -87; // turn into the tower
delay(1000); // wait for the turn to have completely finished


// drive into tower
driveFor = 250; // drive slightly closer to the tower
intake(0,0); // stop the intakes
delay(300); // wait to be done


//shoot the ball
setRoll(90, 127); //shoot 1 ball
delay(1000); // give the ball time to be scored

//shoot the ball
setRoll(100, 127); //shoot 1 ball
delay(1000); // give the balls time to be scored

//back up
driveFor = -400; // back out of the tower
delay(1000); // wait to have reached the target


// turn to the next ball
turnTo = 8; // turn towards the next ball
setRoll(40, 0); // slow the rollers to normal indexing speed
delay(2000); // wait for the turn to have completely finished


// drive to the next ball and index it
driveFor = 1600; // drive into the next ball
intake(-127,-127); //start intaking
delay(1500); // wait for the drive to have reached the target


// turn to the third goal
turnTo = -45; // turn towards the goal
delay(1500); // wait for the turn to have completely finished


//drive into the goal
driveFor = 1000; // drive to the goal
intake(0,0); // stop the intakes
delay(1400);// wait to have reached the target


// score the third tower
setRoll(127, 127); //score one ball into the tower
delay(800); // give the rollers time to shoot


// back out of the third tower
driveFor = -500; // drive out of the goal
setRoll(40, 0); // slow the rollers to normal indexing speed
delay(1500); // wait to have reached the target


// turn back to 0 degrees
turnTo = 0; // turn to back to 0
delay(1500); // wait for the turn to have completely finished


// back up to be able to intake the next ball
driveFor = -1050; // back up to be in line with the next ball
delay(1500); // wait to have reached the target


// turn to the next red ball
turnTo = 90; // turn towards the next ball
delay(1500); // wait for the turn to have completely finished


// drive to the red ball and intake it
driveFor = 1900; // drive to the ball
intake(-127,-127); // start intaking
delay(1700); // wait to have reached the target


//turn into the fourth tower
turnTo = 0; // turn towards the fourth goal
delay(1700); // wait for the turn to have completely finished


// drive to the fourth tower
driveFor = 1150; // drive into the goal
intake(0,0); // start intaking

delay(2000); // wait for the flywheel to be done


// score the red ball
setRoll(127,127); // set the rollers to full power
delay(1700); // wait for the flywheel to be done


//back out of the tower to prepare for the next red ball
driveFor = -600; // drive out of the goal
setRoll(40,0); // slow the rollers to normal indexing speed
delay(1700); // wait to have reached the target


// turn to the next ball
turnTo = 80; // turn to the next red ball
intake(-127,-127);
delay(1700); // wait for the turn to have completely finished


// drive into the last red ball
driveFor = 1800;
delay(1700); // wait for the turn to have completely finished


// turn into the tower
turnTo = 60; // turn to the tower
intake(0,0); // stop the intakes
delay(1700); // wait for the turn to have completely finished


// drive into the last tower
driveFor = 500; // drive into the tower
delay(1700); // wait for the turn to have completely finished


// score the last ball
setRoll(127,127); // fullrun the rollers
delay(1000); // wait for the ball to be scored


//back out of the tower
driveFor = -800; // back up

}
