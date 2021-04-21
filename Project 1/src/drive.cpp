#include "main.h"
#include "Functions.hpp"

    // this file contains all the functions that mainly
    // perform actions through the drive train
    // declare all voids in Functions.hpp header file

/******************************************************************************/
/*          Autonomous variables                                              */
/******************************************************************************/

float go_to = 0;          // the distance to drive. Set by drive function
float turn_to = 0;      // distance to turn to. in absolute degrees
int max_speed = 100;    // the max speed of the PID. Set by drive functions
int time_limit = 0;     // specifies the max length of time the PID can run
bool done = false;      // tells if PID_drive is done
bool marker1 = false;
float driftActive = 0;
float changeHeading = 0;
bool move_function_done = false;
const int drive_motor_speed = 200;  // max motor speed in RPM

const float offset = 25/4;

// .5,0,1.5

const float Kp_drive    = .5;  // the coeffcient of the proportional
const float Ki_drive    = 0.001;  // the coefficient of the integral
const float Kd_drive    = 11;   // the coefficient of the derivative
const float drift_drive = 0.1;      // the coefficient of the drift control
const float I_active_d  = 1080;    // distance to target where integral becomes
                                  // active

const float Kp_turn     = 2;
      float Ki_turn     = 0.001;
const float Kd_turn     = 20.75;
      float Ki_active_t = 10;
const float Ki_limit_t  = 100000;

float Kp_radius   = 0;
float Kd_radius   = 0;
float radius_drift_control = .01;


timer watch;

/******************************************************************************/
/*          Autonomous Functions                                              */
/******************************************************************************/

// speed control functions

void set_drive(float speedR, float speedL){
  driveFR.move_velocity(speedR*2);
  driveBR.move_velocity(speedR*2);
  driveFL.move_velocity(speedL*2);
  driveBL.move_velocity(speedL*2);
}

void set_drive_voltage(float speedR, float speedL){
  driveFR.move_voltage(speedR*120);
  driveBR.move_voltage(speedR*120);
  driveFL.move_voltage(speedL*120);
  driveBL.move_voltage(speedL*120);
}

// position and direction control functions

void drive_simple(int speed, int rotation, bool wait){
  //int rotation = inchToDeg(inches);
  driveFR.move_relative(rotation,speed*2);
  driveBR.move_relative(rotation,speed*2);
  driveFL.move_relative(rotation,speed*2);
  driveBL.move_relative(rotation,speed*2);
  if(wait){
    while(driveBL.is_stopped() != true || driveBR.is_stopped() != true){
      delay(10);
    }
  }
}

// right positive
// left negative
// hotel Trivago

void encoder_turn(int speed, int degrees){
  float radians = degrees*PI/180;
  float r = inchToDeg(25/4);
  float distance_right = radians*r;
  float distance_left = -1*radians*r;
  float pwr_left = speed;
  float pwr_right = speed;
  int error  = 10;
  bool in_progress_r = true;
  bool in_progress_l = true;
  float lStart = driveBL.get_position();
  float rStart = driveBR.get_position();
  float d_right = 0;
  float d_left = 0;
  if(distance_right < 0)
    pwr_right = pwr_right*-1;
  else if(distance_left < 0 )
    pwr_left = pwr_left*-1;

  while(in_progress_r && in_progress_l){
    d_left = driveBL.get_position() - lStart;
    d_right = driveBR.get_position() - rStart;
      if(distance_left + error < d_left || distance_left - error > d_left){
        driveBL.move_velocity(pwr_left);
        driveFL.move_velocity(pwr_left);
      }
      else
        in_progress_l = false;
      if(distance_right + error < d_right || distance_right - error > d_right){
        driveBR.move_velocity(pwr_right);
        driveFR.move_velocity(pwr_right);
      }
      else
        in_progress_r = false;
  }
  driveBL.move_velocity(-1*pwr_left);
  driveFL.move_velocity(-1*pwr_left);
  driveBR.move_velocity(-1*pwr_right);
  driveFR.move_velocity(-1*pwr_right);
  delay(10);
  driveBL.move_velocity(0);
  driveFL.move_velocity(0);
  driveBR.move_velocity(0);
  driveFR.move_velocity(0);
}


void PID_turn(){
  done = false;
  int time_out = time_limit*1000;
  int speed = max_speed;
  int desired_heading = turn_to;
  float adj_heading = 0;
  float pwr_l = 0;
  float pwr_r = 0;
  float angle_error = 0;
  float integral_error = 0;
  float last_angle_error = 0;
  FILE* Testdata = fopen("/usd/test_data_turn.txt", "w");
  // a do loop is used to make sure that adj_heading is calculated at least once
  watch.reset();
  do {
    adj_heading = gyro.get_heading();
    if(adj_heading < desired_heading - 180){
      adj_heading = adj_heading + 360;
    }
    else if(adj_heading > desired_heading + 180){
      adj_heading = adj_heading - 360;
    }

    last_angle_error = angle_error;

    angle_error = desired_heading - adj_heading;

    if(std::fabs(angle_error) < Ki_active_t)
      integral_error = integral_error + angle_error;
    else
      integral_error = 0;

    if(integral_error > Ki_limit_t)
      integral_error = Ki_limit_t;
    else if(integral_error < -1*Ki_limit_t)
      integral_error = -1*Ki_limit_t;

    pwr_r = (angle_error*Kp_turn + integral_error*Ki_turn +
            (angle_error - last_angle_error)*Kd_turn)*-1;
    pwr_l = angle_error*Kp_turn + integral_error*Ki_turn +
            (angle_error - last_angle_error)*Kd_turn;

    set_drive_voltage(pwr_r,pwr_l);
    delay(10);
    fprintf(Testdata,"%G , %G ,\n",angle_error,adj_heading);
  }
  while(std::fabs(angle_error) > .1 && watch.time() < time_out && done != true);
  done = true;
  set_drive(0,0);
  fclose(Testdata);
}

void turnAsync(int speed, float desired_heading, double time){
  turn_to = desired_heading;             // sets the distance to drive
  max_speed = speed;            // sets max speed of the PID loop
  time_limit = time;            // sets the time limit for the PID loop
                                // if time limit is over 15, time is irrelevant
  pros::Task drive_task (PID_turn,"turn_task");
                                // creates a drive task seperate from main task
}

void turn(int speed, float desired_heading, double time){
  turn_to = desired_heading;             // sets the distance to drive in inches
  max_speed = speed;            // sets max speed of the PID loop
  time_limit = time;            // sets the time limit for the PID loop

  done = false;
  delay(10);
  pros::Task drive_task (PID_turn,"turn_task");
                                // creates a drive task seperate from main task
  done = false;
  while(done !=true){delay(10);}// delays main task until PID finishes
}

void set_Ki_turn(bool new_value){
  if(new_value)
    Ki_turn = .075;
  else
    Ki_turn = .001;
}

void set_Ki_active(double new_value){
  Ki_active_t = new_value;
}


void driveradius(int speed, int radius, int degrees, bool direction,float time){
  //radius is radius from turning point to center of robot
  //degrees is the degrees rotated around turning point
  //if direction if 1 then left, else right
  // left turn: right is long
  // right turn: left is long
  //distance is 10 5/8 inches between both wheels front
  //distance is 12 1/2 inches between both wheels back
  int time_out = time*1000;
  float radians = degrees*PI/180;
  int distance_left = 0;
  int distance_right = 0;
  float r = inchToDeg(radius);
  float far_distance = radians*(r+inchToDeg(25/4));
  float low_distance = radians*(r-inchToDeg(25/4));
  float pwr_left = 0;
  float pwr_right = 0;
  float low_speed = speed*(low_distance/far_distance);
  done = false;
  int lStart = driveBL.get_position();
  int rStart = driveBR.get_position();
  watch.reset();
  if(direction){
    distance_right = far_distance;
    distance_left = low_distance;
    pwr_right = speed;
    pwr_left = low_speed;
  }
  else{
    distance_left = far_distance;
    distance_right = low_distance;
    pwr_right = low_speed;
    pwr_left = speed;
  }

  if(distance_right < 0)
    pwr_left = pwr_left*-1;
  if(distance_left < 0)
    pwr_right = pwr_right*-1;

  while(done != true && time_out > watch.time()){
    if( distance_right < driveBR.get_position() - rStart+ 10   &&
        distance_right > driveBR.get_position() - rStart - 10)
      done = true;
    if( distance_left < driveBL.get_position() - lStart + 10   &&
        distance_left > driveBL.get_position() - lStart - 10)
      done = true;

    set_drive(pwr_right,pwr_left);
  }
  set_drive(-pwr_right,-pwr_left);
  delay(100);
  set_drive(0,0);
}


// right positive
// left negative
// hotel Trivago

//////////////////////////////////////////////////
//  heading readings (degrees)                  //
//       +/-360;0                               //
// (-90)270 x 90(-270)                          //
//      +/-180                                  //
//////////////////////////////////////////////////

/*
void driveradius(int speed, int radius, int degrees, bool direction,float time){
  //radius is radius from turning point to center of robot
  //degrees is the degrees rotated around turning point
  //if direction if 1 then left, else right
  // left turn: right is long
  // right turn: left is long
  int time_out = time*1000;
  float radians = degToRad(degrees);
  float distance = inchToDeg_t(radians*radius);
  float pwr_left = 0;
  float pwr_right = 0;
  float low_speed = (radius+offset)/(radius - offset);
  done = false;
  float startPos = tracking.get_value();
  float start_heading = gyro.get_heading();
  float error_d = 0;
  float last_error_d = 0;
  float desired_heading = 0;
  float error_a = 0;
  float last_error_a = 0;
  float adj_heading = 0;
  while(done != false){
    // set last error
    last_error_d = error_d;
    last_error_a = error_a;
    // find current error
    error_d = distance - (tracking.get_value() - startPos);
    // find desired angle
    if(direction)
      desired_heading = start_heading + (1 - (distance/error_d))*degrees;
    else
      desired_heading = start_heading - (1 - (distance/error_d))*degrees;
    // find adjusted heading
    adj_heading = gyro.get_heading();
    while(adj_heading < desired_heading - 180){
      adj_heading = adj_heading + 360;
      delay(10);
    }
    while(adj_heading > desired_heading + 180){
      adj_heading = adj_heading - 360;
      delay(10);
    }
    // find error to desired heading
    error_a = desired_heading - adj_heading;
    // calculate motor powers
    if(distance)
      pwr_left  = error_d*Kp_radius + (last_error_d - error_d)*Kd_radius;
    else
      pwr_right = error_d*Kp_radius + (last_error_d - error_d)*Kd_radius;
    // set inside motor power
    if(pwr_left > speed)
      pwr_left = speed;
    if(pwr_right > speed)
      pwr_right = speed;
    if(distance)
      pwr_right = pwr_left*low_speed;
    else
      pwr_left  = pwr_right*low_speed;
    // adjust motor powers based on angle error
    if(error_a > 0)
      pwr_left = pwr_left*(1 - error_a*radius_drift_control);
    else if(error_a < 0)
      pwr_right = pwr_right*fabs(1 - error_a*radius_drift_control);
    // set motors
    set_drive(pwr_right,pwr_left);
    // check end condition
    if(abs(error_d) < 10)
      done = true;

  }
}
*/

/*
New Idea for drive_radius control:
two tasks for PID of both sides
constantly track progress of one side and calculate the
position that other side should be on
use derivative + current position to calc next inside distance

Send that into the PID loop
*/


void PID_drive(){
  watch.reset();
  int endpoint = inchToDeg_t(go_to);
  int I_drive_max = .5*(max_speed/Ki_drive);
                        // the max the integral is allowed to ramp to
                        // sets the endpoint of the drive function in degrees
  float P_power = 0;    // power of proportional control system
  float I_power = 0;    // power of integral control system
  float D_power = 0;    // power of derivative control system
  float error = 0;      // error of system (end value - current value)
  float drv_val = 0;    // averaged value of all drive encoders
  float error_past = 0; // error of previous run through
  float pwr = 0;        // original power of motors in percent
  float drift = 0;      // measures the drift of the drivetrain as a difference
                        // in rotation
  float pwr_R = 1;      // power of the right motors in percent of pwr
  float pwr_L = 1;      // power of the left motors in percent of pwr
  int time_out = time_limit*1000;
  int relative_time = 0;

  int startPos  = tracking.get_value();
  int start_heading = gyro.get_rotation();
  done = false;
  delay(10);            // gives the brain time to set done to false
  //FILE* Testdata = fopen("/usd/test_data_drive.txt", "w");
                        // setting start position to stop from reseting encoders
  while(done == false && time_out > relative_time){
    error_past = error; // Sets the past error for the differential
    drv_val = (tracking.get_value() - startPos);
                        // averages drive encoder positions for both sides

    error = endpoint - drv_val;   // finds error by subtracting endpoint
                                  // from current averaged value
    P_power = error;              // speeds up robot proportional to
                                  // the current error
    if(std::fabs(error) < I_active_d)
      I_power = I_power + error;    // allows robot to keep speed up while also
                                    // allowing for the robot to slow down
    else
      I_power = 0;

    if(I_power < -1*I_drive_max)
      I_power = -1*I_drive_max;   // limits integral to a set value to stop
    if(I_power > I_drive_max)     // integral saturation
      I_power = I_drive_max;

    D_power = error - error_past; // allows robot to predict the future to
                                  // slow down before reaching the endpoint
    pwr = (P_power*Kp_drive) +
            (I_power*Ki_drive) +
            (D_power*Kd_drive);   // calculates power by adding all the weighted
                                  // values of the control systems together
    if(pwr > 100)
      pwr = 100;            // limits max motor speed to 100 percent
    else if(pwr < -100)
      pwr = -100;         // limits max motor speed to 100 percent
    if(std::abs(error) < 5){
      done = true;                // sets while loop to end
      pwr = 0;                    // sets power to zero to stop motors
      std::cout << "DONE\n";
    }

    drift = gyro.get_rotation() - start_heading;
    // drift right +
    if(error > 0){
      if(drift < 0)
        pwr_R = 1-(fabs(drift)*drift_drive);
      else if(drift > 0)
        pwr_L = 1-(fabs(drift)*drift_drive);
    }
    else{
      // do nothing
    }
                        // set drive power to the set power of the PID loop
    set_drive_voltage(pwr*pwr_R,pwr*pwr_L);
    //std::cout << "power:" << pwr << "\n";
    delay(10);
    //fprintf(Testdata, "%G , %G , \n", error, pwr);
    relative_time = watch.time();
    printf("power: %G ",error);
    //printf("error: %G",error);
    printf("\n");
  }
  done = true;
  set_drive(0,0);
  go_to = 0;
  time_limit = 0;
  delay(10);                    // gives brain time to set the variables
  //fclose(Testdata);
}

void driveAsync(int speed, float endpoint, int time){
  go_to = endpoint;             // sets the distance to drive
  max_speed = speed;            // sets max speed of the PID loop
  time_limit = time;            // sets the time limit for the PID loop
                                // if time limit is over 15, time is irrelevant
  pros::Task drive_task (PID_drive,"drive_task");
                                // creates a drive task seperate from main task
}

void drive(int speed, float endpoint, int time){
  go_to = endpoint;             // sets the distance to drive in inches
  max_speed = speed;            // sets max speed of the PID loop
  time_limit = time;            // sets the time limit for the PID loop
                                // if time limit is over 15, time is irrelevant
  delay(10);
  pros::Task drive_task (PID_drive,"drive_task");
                                // creates a drive task seperate from main task
  done = false;
  while(done !=true){delay(10);}// delays main task until PID finishes
}

void driftDriveTask(){
  watch.reset();
  int I_drive_max = .1*(max_speed/Ki_drive);
                        // the max the integral is allowed to ramp to
                        // sets the endpoint of the drive function in degrees
  float P_power = 0;    // power of proportional control system
  float I_power = 0;    // power of integral control system
  float D_power = 0;    // power of derivative control system
  float error = 0;      // error of system (end value - current value)
  float drv_val = 0;    // averaged value of all drive encoders
  float error_past = 0; // error of previous run through
  float pwr = 0;        // original power of motors in percent
  float drift = inchToDeg_t(driftActive);
  int endpoint = inchToDeg_t(go_to);
                        // holds where to drift robot

  float pwr_l = 0;
  float pwr_r = 0;
  float angle_error = 0;
  float integral_error = 0;
  float last_angle_error = 0;

  int time = time_limit*1000;
  int relative_time = 0;
  float adj_heading = 0;

  int startPos  = tracking.get_value();
  int desired_heading = gyro.get_rotation();
  done = false;
  marker1 = false;
  do{
    //
    //  drive PID portion of drift drive
    //
    error_past = error; // Sets the past error for the differential
    drv_val = (tracking.get_value() - startPos);
                        // averages drive encoder positions for both sides

    error = endpoint - drv_val;   // finds error by subtracting endpoint
                                  // from current averaged value
    P_power = error;              // speeds up robot proportional to
                                  // the current error
    if(std::fabs(error) < I_active_d)
      I_power = I_power + error;    // allows robot to keep speed up while also
                                    // allowing for the robot to slow down
    else
      I_power = 0;

    if(I_power < -1*I_drive_max)
      I_power = -1*I_drive_max;   // limits integral to a set value to stop
    if(I_power > I_drive_max)     // integral saturation
      I_power = I_drive_max;

    D_power = error - error_past; // allows robot to predict the future to
                                  // slow down before reaching the endpoint
    pwr = (P_power*Kp_drive) +
            (I_power*Ki_drive) +
            (D_power*Kd_drive);   // calculates power by adding all the weighted
                                  // values of the control systems together
    if(pwr > 100)
      pwr = 100;            // limits max motor speed to 100 percent
    else if(pwr < -100)
      pwr = -100;         // limits max motor speed to 100 percent

    //
    // Turn PID portion of drift drive
    //

    adj_heading = gyro.get_heading();
    while(adj_heading < desired_heading - 180){
      adj_heading = adj_heading + 360;
      delay(10);
    }
    while(adj_heading > desired_heading + 180){
      adj_heading = adj_heading - 360;
      delay(10);
    }

    last_angle_error = angle_error;

    angle_error = desired_heading - adj_heading;

    if(std::fabs(angle_error) < Ki_active_t)
      integral_error = integral_error + angle_error;
    else
      integral_error = 0;

    if(integral_error > Ki_limit_t)
      integral_error = Ki_limit_t;
    else if(integral_error < -1*Ki_limit_t)
      integral_error = -1*Ki_limit_t;

    pwr_r = (angle_error*Kp_turn + integral_error*Ki_turn +
            (angle_error - last_angle_error)*Kd_turn)*-1;
    pwr_l = angle_error*Kp_turn + integral_error*Ki_turn +
            (angle_error - last_angle_error)*Kd_turn;

    if(pwr_l > 1.5*max_speed)
      pwr_l = 1.5*max_speed;
    else if(pwr_l < -1.5*max_speed)
      pwr_l = -1.5*max_speed;
    if(pwr_r > 1.5*max_speed)
      pwr_r = 1.5*max_speed;
    else if(pwr_r < -1.5*max_speed)
      pwr_r = -1.5*max_speed;

    if(abs(error) < 720){
      pwr_r = 0;
      pwr_l = 0;
    }

    //
    // combining values from both PID loops
    //

    pwr_r = pwr_r + pwr;
    pwr_l = pwr_l + pwr;

    //
    // limiting values to within bounds +/- max_speed
    //

    if(pwr_r > max_speed)
      pwr_r = max_speed;
    else if(pwr_r < -1*max_speed)
      pwr_r = -1*max_speed;

    if(pwr_l > max_speed)
      pwr_l = max_speed;
    else if(pwr_l < -1*max_speed)
      pwr_l = -1*max_speed;

    //
    // setting drive motors
    //
    delay(10);
    set_drive_voltage(pwr_r,pwr_l);

    //
    // setting drift at correct times
    //

    if(drift < drv_val){
      desired_heading = changeHeading;
      marker1 = true;
    }
  }while(done != true && fabs(error) > 5 && watch.time() < time);
  marker1 = false;
  done = true;
  set_drive(0,0);

}


void driftDrive(int speed, float distance, float driftDistance,
                float heading, float max_time){
  //
  max_speed = speed;
  done = false;
  go_to = distance;
  driftActive = driftDistance;
  changeHeading = heading;
  time_limit = max_time;
  //
  delay(10);
  //
  pros::Task drift_drive_task (driftDriveTask,"drift_drive_task");

  while(done != true){delay(10);}
  delay(10);
  done = false;
}

void driftDriveAsync(int speed, float distance, float driftDistance,
                float heading, float max_time){
  //
  max_speed = speed;
  done = false;
  go_to = distance;
  driftActive = driftDistance;
  changeHeading = heading;
  time_limit = max_time;
  //
  delay(10);
  //
  pros::Task drift_drive_task (driftDriveTask,"drift_drive_task");
}

void change_max_speed(int bruh){
  max_speed = bruh;
}

void drive_wait(){
  while(done !=true){delay(10);}
}
void change_heading_wait(){
  while(marker1 != true){delay(10);}
}


void drive_kill(){
  done = true;
}

// odometry based functions

void move_tos(int x, int y, int speed, int timeOut, bool direction){
  watch.reset();
  double x_magnitude = x - pos_x();
  double y_magnitude = y - pos_y();
  double magnitude = sqrt(pow(x_magnitude,2) + pow(y_magnitude,2));
  // find angle to travel upon
  double angle = radToDeg(asin(std::fabs(x_magnitude/magnitude)));
  // angle will be a value of degrees between 0 and 90
  // we need to get the actual angle by figuring direction of the vector
  // this will be done by finding the quadrant of the vector
  if(x_magnitude > 0){
    //  angle is now between 0 and 180
    if(y_magnitude > 0){}
      // if both positive no adjustment is needed
    else
      angle = 180 - angle;
      // x positive y negative 90 < angle < 180
      // angle needs to be mirrored across x axis
  }
  else{
    // angle is between 180 and 360
    if(y_magnitude > 0)
      angle = 360 - angle;
      // x negative y positive 270 < angle < 360
      // angle needs to be mirrored across y axis
    else
      angle = 180 + angle;
      // x negative y negative 180 < angle < 270
      // angle needs to be mirrored across x and y axis
  }
  if(direction){
    if(angle < 180)
      angle = angle + 180;
    else
      angle = angle - 180;
    magnitude = magnitude * -1;
  }
  turn(speed,angle,timeOut);
  //double timeOutDrive = timeOut - (watch.time()/1000);
  delay(500);
  drive(speed,magnitude,timeOut);
}
void move_tos_overshoot(int x, int y,int overshoot, int speed, int timeOut,
                        bool direction){
  watch.reset();
  double x_magnitude = x - pos_x();
  double y_magnitude = y - pos_y();
  double magnitude = sqrt(pow(x_magnitude,2) + pow(y_magnitude,2));
  // find angle to travel upon
  double angle = radToDeg(asin(std::fabs(x_magnitude/magnitude)));
  // angle will be a value of degrees between 0 and 90
  // we need to get the actual angle by figuring direction of the vector
  // this will be done by finding the quadrant of the vector
  if(x_magnitude > 0){
    //  angle is now between 0 and 180
    if(y_magnitude > 0){}
      // if both positive no adjustment is needed
    else
      angle = 180 - angle;
      // x positive y negative 90 < angle < 180
      // angle needs to be mirrored across x axis
  }
  else{
    // angle is between 180 and 360
    if(y_magnitude > 0)
      angle = 360 - angle;
      // x negative y positive 270 < angle < 360
      // angle needs to be mirrored across y axis
    else
      angle = 180 + angle;
      // x negative y negative 180 < angle < 270
      // angle needs to be mirrored across x and y axis
  }
  magnitude = magnitude + overshoot;
  if(direction){
    if(angle < 180)
      angle = angle + 180;
    else
      angle = angle - 180;
    magnitude = magnitude * -1;
  }
  turn(speed,angle,timeOut);
  //double timeOutDrive = timeOut - (watch.time()/1000);
  delay(500);
  drive(speed,magnitude,timeOut);
}

void move_tos_Async(int x, int y, int speed, int timeOut, bool direction){
  watch.reset();
  double x_magnitude = x - pos_x();
  double y_magnitude = y - pos_y();
  double magnitude = sqrt(pow(x_magnitude,2) + pow(y_magnitude,2));
  // find angle to travel upon
  double angle = radToDeg(asin(std::fabs(x_magnitude/magnitude)));
  // angle will be a value of degrees between 0 and 90
  // we need to get the actual angle by figuring direction of the vector
  // this will be done by finding the quadrant of the vector
  if(x_magnitude > 0){
    //  angle is now between 0 and 180
    if(y_magnitude > 0){}
      // if both positive no adjustment is needed
    else
      angle = 180 - angle;
      // x positive y negative 90 < angle < 180
      // angle needs to be mirrored across x axis
  }
  else{
    // angle is between 180 and 360
    if(y_magnitude > 0)
      angle = 360 - angle;
      // x negative y positive 270 < angle < 360
      // angle needs to be mirrored across y axis
    else
      angle = 180 + angle;
      // x negative y negative 180 < angle < 270
      // angle needs to be mirrored across x and y axis
  }
  if(direction){
    if(angle < 180)
      angle = angle + 180;
    else
      angle = angle - 180;
  }
  turn(speed,angle,timeOut);
  double timeOutDrive = timeOut - (watch.time()/1000);
  driveAsync(speed,magnitude,timeOut);
  move_function_done = true;
}

void move_wait(){
  while(move_function_done != true){
    delay(10);
  }
  move_function_done = false;
}

void point_ats(float x, float y, int speed, float timeout){
  double x_magnitude = x - pos_x();
  double y_magnitude = y - pos_y();
  double magnitude = sqrt(pow(x_magnitude,2) + pow(y_magnitude,2));
  // find angle to travel upon
  double angle = radToDeg(asin(std::fabs(x_magnitude/magnitude)));
  // angle will be a value of degrees between 0 and 90
  // we need to get the actual angle by figuring direction of the vector
  // this will be done by finding the quadrant of the vector
  if(x_magnitude > 0){
    //  angle is now between 0 and 180
    if(y_magnitude > 0){}
      // if both positive no adjustment is needed
    else
      angle = 180 - angle;
      // x positive y negative 90 < angle < 180
      // angle needs to be mirrored across x axis
  }
  else{
    // angle is between 180 and 360
    if(y_magnitude > 0)
      angle = 360 - angle;
      // x negative y positive 270 < angle < 360
      // angle needs to be mirrored across y axis
    else
      angle = 180 + angle;
      // x negative y negative 180 < angle < 270
      // angle needs to be mirrored across x and y axis
  }
  turn(speed,angle,timeout);
}

/******************************************************************************/
/*          Driver Control Functions                                          */
/******************************************************************************/

//int left_power = 0;
//int right_power = 0;

// macros (for future use)


// Function to be recalled into driver control
// short for operator drive
void OPdrive(){
  //right_power = C1.get_analog(ANALOG_RIGHT_X);
  //left_power = C1.get_analog(ANALOG_LEFT_X);
  driveFR.move(C1.get_analog(ANALOG_RIGHT_Y));
  driveBR.move(C1.get_analog(ANALOG_RIGHT_Y));
  driveFL.move(C1.get_analog(ANALOG_LEFT_Y));
  driveBL.move(C1.get_analog(ANALOG_LEFT_Y));
}
