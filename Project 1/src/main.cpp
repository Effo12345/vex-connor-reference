#include "main.h"
#include "Functions.hpp"
#include "Autonomous.hpp"
#include <stdio.h>

int auton_select = 0;
const int left_button = 4;		// 100 from binary to decimal
const int right_button = 1;		// 001 from binary to decimal
const int center_button = 2;	// 010 from binary to decimal

std::string skills_auton = "skillz";
std::string auton_names[7] = {"one ball","one_ball_right","two ball","two_ball_right","three ball"
								,skills_auton,"blank"};
int max_auton = 6;
bool selected;
bool gyro_stable = false;

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	gyro.reset();
	selected = false;
	bool btn_prsd = false;
	delay(100);
	pros::lcd::set_text(1, "Autonomous program");
	while(selected != true){
						// button_pressed is used to check if the button has been released
						// this allows for more controlable increments
		if(pros::lcd::read_buttons() == left_button && btn_prsd != true){
			auton_select--;
			btn_prsd = true;
		}
		else if(pros::lcd::read_buttons() == right_button && btn_prsd != true){
			auton_select++;
			btn_prsd = true;
		}
		else if(pros::lcd::read_buttons() == 0)
			btn_prsd = false;
						//increments the auton selected by one auton
		if(auton_select == -1)						// checks if variable is out of bounds
			auton_select = max_auton;				// sets variable to higher bound
		if(auton_select == (max_auton+1))	// checks if variable is out of bounds
			auton_select = 0;								// sets variable to lower bound
						// makes sure the autonomous selected stays within the bounds
		pros::lcd::set_text(2,auton_names[auton_select]);
						// prints autonomous that is currently selected
		if (pros::lcd::read_buttons() == center_button)
			selected = true;
						// gives a way of leaving while loop
		pros::c::taskDelay(10);
	}
	pros::lcd::clear();
	while(gyro.is_calibrating()){
		delay(10);
	}
	delay(100);
	// resets gyro to make it more stable in skills auton
	/*
	if(auton_names[auton_select] == skills_auton){
		while(gyro_stable != true){
			gyro.reset();
			delay(100);
			while(gyro.is_calibrating()){
				delay(10);
			}
			delay(2000);
			if(gyro.get_heading() > 359.9 || gyro.get_heading() < .1)
				gyro_stable = true;
				// checks stability of gyroscope to make sure it is acceptable
			delay(10);
		}

	}
	*/
	pros::lcd::set_text(1,"Program Ready!");
	start_odom();
	delay(100);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
	/*
	gyro.reset();
	while(gyro.is_calibrating()){
		pros::c::taskDelay(10);
	}
	*/
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous(){
	//start_drive_PID();
	if(auton_select == 0)
		one_ball();
	else if(auton_select == 1)
		one_ball_right();
	else if(auton_select == 2)
		two_ball();
	else if(auton_select == 3)
		two_ball_right();
	else if(auton_select == 4)
		three_ball();
	else if(auton_select == 5)
		skillsAuton();
	else
		pros::c::taskDelay(12000);
}
/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	// makes sure that all tasks are completed
	// ensure that the driver can control the robot
	//quit_speed_PID();
	//start_drive_PID();
	//FILE* file = fopen("/usd/output.txt", "w");
	//fprintf(file, "Hello world!");
	//fclose(file);
	int driverSwitch = 0;
	bool btn_ready = true;
	odom_quit();
	while (true) {
		printf("%d \n",tracking.get_value());
		OPdrive();							// joysticks set in tank drive configuration
		if(C1.get_digital(DIGITAL_B)){
			//driftDrive(100,60,0,0,5);
			//turn(100,180,5);
			//delay(1000);
			//turn(100,90,5);
			//delay(1000);
			//turn(100,45,5);
			//delay(1000);
		}
		switch(driverSwitch){
			case 1:
				OPintake();
				OProllers();
				if(C1.get_digital(DIGITAL_A) && btn_ready){
					driverSwitch = 0;
					btn_ready = false;
				}
				else if(C1.get_digital(DIGITAL_A) != true)
					btn_ready = true;
				break;
			default:
				OProllersC();
				if(C1.get_digital(DIGITAL_A) && btn_ready){
					driverSwitch = 1;
					btn_ready = false;
				}
				else if(C1.get_digital(DIGITAL_A) != true)
					btn_ready = true;
				break;
		}
		//printf("%d",chambered());
		//printf("\n");

														// L2 is out
														// L1 is all in
														// R1 is just rollers
														// R2 is just intake
		delay(10);// a delay is added to not stress cortex resources
	}
}
// User control is done through call back functions placed in the robot parts
// cpp files
// 4/25/2021 9:21
// length: 2256 lines
// file count: 9 (2 .hpp; 7 .cpp)
// end of file :)

// hi Trento
// hi Bulia
