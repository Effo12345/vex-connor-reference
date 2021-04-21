#ifndef FUNCTIONS_HPP
#define FUNCTIONS_HPP

//driveTrain Function declarations

void set_drive(float speedR, float speedL);
void drive_simple(int speed, int inches, bool wait);
// turn
void encoder_turn(int speed, int degrees);
void GTurn(int speed, int desired_heading, float time_limit);
void turn(int speed, float desired_heading, double time);
void turnAsync(int speed, float desired_heading, double time);
void set_Ki_turn(bool new_value);
void set_Ki_active(double new_value);
// radius
void driveradius(int speed, int radius, int degrees, bool direction,float time);
// drift
void driftDrive(int speed, float distance, float driftDistance,
                float heading, float max_time);
void driftDriveAsync(int speed, float distance, float driftDistance,
                float heading, float max_time);
// drive staight
void driveAsync(int speed, float endpoint, int time);
void drive(int speed, float endpoint, int time);
void move_tos(int x, int y, int speed, int timeOut, bool direction);
void move_tos_overshoot(int x, int y,int overshoot, int speed, int timeOut,
                        bool direction);
void move_tos_Async(int x, int y, int speed, int timeOut, bool direction);
void move_wait();
void point_ats(float x, float y, int speed, float timeout);
// task manipulate functions
void change_max_speed(int bruh);
void drive_wait();
void change_heading_wait();
void drive_kill();
// operator control
void OPdrive();

// rollers function declarations

int chambered();
void set_rollers(int speed);
void set_rollers_velocity(int speed);
void rollDeg(int speed,int deg, bool wait);
void chamber();
void set_bot();
void set_bot_up();
int stage_down();
void shoot();
void OProllers();
void OProllersC();

// intake functions declarations

bool in();
void deploy();
void set_intake(int speed);
void intake_ball();
void quit_intake();
void intdeg(int speed, int deg, bool wait);
void OPintake();

// intake and roller functions

void stage(bool top_ball, bool bottom_ball, float time);
void stage_bot(bool top_ball, bool bottom_ball, float time);
void deposit(int balls, int to_intake);

// odometry functions declarations

void start_odom();
void odom_calc();
void odom_quit();
double pos_x();
double pos_y();
void move_to(float x, float y, int speed, float time);
void move_to_back(float x, float y, int speed, float time);
void point_at(float x, float y, int speed, float time);


#endif
