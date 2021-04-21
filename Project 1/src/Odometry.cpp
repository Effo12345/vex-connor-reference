#include "main.h"
#include "Functions.hpp"
#include <cmath>



bool finished = false;
bool force_quit = false;

const float Kp_drive    = 15;  // the cooeffcient of the proportional
const float Ki_drive    = 0;    // the coefficient of the integral
const float Kd_drive    = 20;  // the coefficient of the derivative
const float Kdrift_drive= .2;    // the coefficient of the drift control

const float Kp_turn     = .65;   // coefficient of porportional on turns
const float Ki_turn     = 0;    // coefficient of integral on turns
const float Kd_turn     = 8.5;  // coefficient of derivative on turns

// length = angle * radius
// radius = length/angle

//////////////////////////////////////////////////
//  signs of the variables and their directions //
//      front of robot                          //
//        ^ +                                   //
//     y: |    x: -<-->+                         //
//        v -                                   //
//      back of robot                           //
//////////////////////////////////////////////////
//  heading readings (degrees)                  //
//       +/-360;0                               //
// (-90)270 x 90(-270)                          //
//      +/-180                                  //
//////////////////////////////////////////////////
//                                              //
//  Sin = opp/hyp   x = opp                     //
//  Cos = adj/hyp   y = adj                     //
//  distance vector = hyp                       //
//                                              //
//////////////////////////////////////////////////

class odom{
  private:
    const double r_offset = 0;    // offset of tracking wheels
    //double distance_r = 0;              // distance traveled by right wheel
    //double distance_l = 0;              // distance traveled by left wheel
    double distance;                // distance traveled by tracking wheel
    //double delta_r;                     // distance traveled this cycle right
    //double delta_l;                     // distance traveled this cycle left
    double delta_d;                     // distance traveled this cycle
    //double distance_last_r = 0;         // last distance of right wheel
    //double distance_last_l = 0;         // last distance of left wheel
    double distance_last = 0;           // last distance of tracking wheel
    double radius;                      // radius of arc
    double offset[2] = {0,0};           // offset of 2d vector {x,y}
    double heading = 0;
    double offset_heading = 0;          // measured in radians
    double last_heading = 0;            // last heading of robot
    double chord_length;                // length of 2d vector
    double pos[2] = {0,0};              // position of the robot {x,y}

  public:

    void calc_change_pos(){
      // find total distance traveled by wheels
      distance = degToInch_t(tracking.get_value());
      // translate total distance travelled to distance since last compute
      delta_d = distance - distance_last;
      // set last distances to current distances
      distance_last = distance;
      // find offset_heading
      heading = gyro.get_rotation();
      offset_heading = heading - last_heading;
      last_heading = heading;
      // find offset position (offset[0] is x; offset[1] is y)
      // finds a straight line if deviation from heading is
      // less than 1/100 of a degree
      if(offset_heading < .01 && offset_heading > -.01 && false){
        offset[0] = sin(degToRad(heading))*delta_d;
        offset[1] = cos(degToRad(heading))*delta_d;
      }
      // uses a chord of the radius of the curve to determine offset
      else{
        // determines the way the robot is curving to apply appropriate
        // offset to radius
        if(offset_heading < 0 && offset_heading != 0)
          radius = (delta_d/offset_heading)-r_offset;
        else if (offset_heading > 0 && offset_heading != 0)
          radius = (delta_d/offset_heading)+r_offset;
        //calculates the chord traveled
        chord_length = 2*(radius*(offset_heading/2));
        // calculates the components of the chord_length std::vector
        // these components are the offsets
        offset[0] = sin(degToRad(heading-(offset_heading/2)))
                    *chord_length;
        offset[1] = cos(degToRad(heading-(offset_heading/2)))
                    *chord_length;

      }
      if(isnan(offset[0]) || isnan(offset[1])){
        printf("somethin' aint right chief \n");
      }
      else{
        pos[0] = pos[0]+offset[0];
        pos[1] = pos[1]+offset[1];
        printf("x: %G \n   y: %G \n",pos[0],pos[1]);
      }
      //printf("distance: %G \n",chord_length);
    }
    double posx(){return pos[0];};
    double posy(){return pos[1];};
/*****************************************************************************/
/*                  movement functions based on odometry                     */
/*****************************************************************************/
    void move_to(float x, float y, int speed, float timeout, bool direction){
      double x_magnitude;         // holds x quantity of 2d vector
      double y_magnitude;         // holds y quantity of 2d vector
      double magnitude;           // holds 2d vector magnitude
      double angle;               // holds 2d vector angle
      double adj_heading;             // holds the adjusted heading of robot
      double error;               // holds the current position error
      double last_error = 0;      // holds last error for derivative
      double angle_error;         // holds value to control drift
      double last_angle_error = 0;
      float pwr_right = 0;        // holds power of right motor in % speed
      float pwr_left = 0;         // holds power of left motor in % speed
      double drift_drive = 0;
      bool arrived = false;       // holds if the robot has arrived
                                  // completes movement when true
      float pwr_R;
      float pwr_L;
      bool turning = false;
      bool drift = false;
      //////////////////////////////////////////////
      //            ORDER OF OPERATIONS           //
      //                                          //
      // find distance to travel                  //
      // find angle to travel upon                //
      // if angle is off by more than 5 degrees   //
      //     turn to point in direction of travel //
      // else if angle is off by about 1/4 degree //
      //     drive with a drift to counteract     //
      // else                                     //
      //     drive as as straight as possible     //
      // implement PID based on vector magnitude  //
      //////////////////////////////////////////////

      FILE* Testdata = fopen("/usd/test_data_odom.txt", "w");
      fprintf(Testdata, "Start of data \n");
      fclose(Testdata);
      while(arrived == false){
        // find distance to travel
        // finding the magnitude of a distance vector (inches)
        x_magnitude = x - pos[0];
        y_magnitude = y - pos[1];
        // distance formula c = sqrt(a^2 + b^2)
        magnitude = sqrt(pow(x_magnitude,2) + pow(y_magnitude,2));
        // find angle to travel upon
        angle = radToDeg(asin(std::fabs(x_magnitude/magnitude)));
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
        // check how far from angle robot currently is
        // if more than 5 then turn
        // if between 5 and 1 then drift
        // if between 1 and 0 then drive straight
        // normalize heading based on where desired angle is
        // this is done to make sure that the robot turns in
        // the most efficient direction and so that if it is
        // already angled correctly it does not turn
        adj_heading = gyro.get_heading();
        // makes sure that heading is within the bounds of angle +/- 180
        // this ensures that the most efficient turn will take place
        // heading is incremented by 360 to ensure angle stays accurate
        while(adj_heading < angle - 180){
          adj_heading = adj_heading + 360;
          delay(10);
        }
        while(adj_heading > angle + 180){
          adj_heading = adj_heading - 360;
          delay(10);
        }
        // if angle is above 5 degrees off
        if(adj_heading > angle + 2 || adj_heading < angle - 2){
          // when error is positive turn left
          // when error is negative turn right
          last_angle_error = angle_error;
          angle_error = angle - heading;
          pwr_left =  angle_error*Kp_turn + ((angle_error-last_angle_error)*Kd_turn);
          pwr_right= (angle_error*Kp_turn + ((angle_error-last_angle_error)*Kd_turn))*-1;
          turning = true;
          /*
          if(pwr_right > speed)
            pwr_right = speed;
          else if(pwr_right < -1*speed)
            pwr_left = -1*speed;
          if(pwr_left > speed)
            pwr_left = speed;
          else if(pwr_left < -1*speed)
            pwr_left = -1*speed;
          */
        }
        // if angle is between .25 and 5 degrees off
        // drift drive to get back on line with target
        else{
          last_error = error;
          error = magnitude;
          angle_error = angle - adj_heading;
          pwr_left =  error*Kp_drive + ((error-last_error)*Kd_drive);
          pwr_right=  error*Kp_drive + ((error-last_error)*Kd_drive);
          if(angle_error < 0){
            pwr_R = 1-(fabs(angle_error)*drift_drive);
            pwr_L = 1;
          }
          else if(angle_error > 0){
            pwr_L = 1-(fabs(angle_error)*drift_drive);
            pwr_R = 1;
          }
          if(pwr_right > speed)
            pwr_right = speed;
          if(pwr_left > speed)
            pwr_left = speed;

          pwr_left = pwr_left*pwr_R;
          pwr_right = pwr_right*pwr_L;

          turning = false;
        }
        // if angle is less than .25 degree off
        // drive as straight as possible
        // drift drive will go back if angle strays too far
        if(direction){
          if(turning){
            // do nothing
          }
          else{
            pwr_left  = -1* pwr_left;
            pwr_right = -1* pwr_right;
            // switches to negative to make robot drive backwards
          }
        }

        set_drive(pwr_right,pwr_left);

        // check for exit condiion
        if(std::fabs(x_magnitude) < .125 && std::fabs(y_magnitude) < .125)
           arrived = true;

        FILE* Testdata = fopen("/usd/test_data_odom.txt", "a");
        fprintf(Testdata, "%G , %G , %G, \n", error, angle, angle_error);
        fclose(Testdata);
        //printf("pwr_l %G \npwr_r %G",pwr_left,pwr_right);
        printf("heading: %G \nangle: %G",adj_heading,angle);
        delay(10);
      }
      set_drive(0,0);
    }
    // this function is a cut down move_to
    // allows for an asynchronous PID_drive to be used
    void point_at(float x, float y, int speed, float timeout){
      double x_magnitude;         // holds x quantity of 2d vector
      double y_magnitude;         // holds y quantity of 2d vector
      double magnitude;           // holds 2d vector magnitude
      double angle;               // holds 2d vector angle
      double adj_heading;         // holds the adjusted heading of robot
      double angle_error;         // holds value to control drift
      double last_angle_error = 0;
      float pwr_right = 0;        // holds power of right motor in mV
      float pwr_left = 0;         // holds power of left motor in mV
      do {
        // find distance to travel
        // finding the magnitude of a distance vector (inches)
        x_magnitude = x - pos[0];
        y_magnitude = y - pos[1];
        // distance formula c = sqrt(a^2 + b^2)
        magnitude = sqrt(pow(x_magnitude,2) + pow(y_magnitude,2));
        // find angle to travel upon
        angle = radToDeg(asin(std::fabs(x_magnitude/magnitude)));
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
        // check how far from angle robot currently is
        // if more than 5 then turn
        // if between 5 and 1 then drift
        // if between 1 and 0 then drive straight
        // normalize heading based on where desired angle is
        // this is done to make sure that the robot turns in
        // the most efficient direction and so that if it is
        // already angled correctly it does not turn
        adj_heading = gyro.get_heading();
        // makes sure that heading is within the bounds of angle +/- 180
        // this ensures that the most efficient turn will take place
        // heading is incremented by 360 to ensure angle stays accurate
        while(adj_heading < angle - 180){
          adj_heading = adj_heading + 360;
          delay(10);
        }
        while(adj_heading > angle + 180){
          adj_heading = adj_heading - 360;
          delay(10);
        }
          // when error is positive turn left
          // when error is negative turn right
          last_angle_error = angle_error;
          angle_error = angle - heading;
          pwr_left =  angle_error*Kp_turn + ((angle_error-last_angle_error)*Kd_turn);
          pwr_right= (angle_error*Kp_turn + ((angle_error-last_angle_error)*Kd_turn))*-1;

          set_drive(pwr_right,pwr_left);
      }
      while (fabs(angle_error) > .1);
    }
};

odom chassis;
void start_odom(){
  pros::Task odometry_task (odom_calc,"odometry");
}
void odom_calc(){
  while(force_quit == false){
    chassis.calc_change_pos();
    delay(10);
  }
  force_quit = false;
}

void odom_quit(){
  force_quit = true;
}

double pos_x(){return chassis.posx();}
double pos_y(){return chassis.posy();}

void move_to(float x, float y, int speed, float time){
  chassis.move_to(x,y,speed,time,0);
}
void move_to_back(float x, float y, int speed, float time){
  chassis.move_to(x,y,speed,time,1);
}
void point_at(float x, float y, int speed, float time){
  chassis.point_at(x,y,speed,time);
}
