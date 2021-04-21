#include "main.h"

//FUNCS
void waitR(int target, int dist) { // wait for the right shaft encoder to be within "dist" of the target
  while (true) {
    if (abs(target - rightTrack.get_value()) < dist) {
      break;
    }
  }
}

void encoderGyro(int row) { // set the defined brain row to the current heading
  lcd::set_text(row ,std::to_string(gyroValue()));
}

  double gyroValue() { // get the robot heading based on the shaft encoder values
    //constants
    double diameter = 12.4; // distance between the tracking wheels
    double trackerRotation = 2.755905512 * 3.141592653589793238462643383279; // each 360 rotation of the encoder is 8.639379797 inches (diameter of tracking wheel * Pi)

    // encoder values
    int ltrack = leftTrack.get_value();
    int rtrack = rightTrack.get_value();

    // math
    double lDistance = ltrack * (trackerRotation/360); // getting the distance traveled of the left wheel
    double rDistance = rtrack * (trackerRotation/360); // getting the distance traveled of the right wheel
    double theta = (lDistance - rDistance)/diameter; // getting the angle in radians
    double heading = theta * 57.29577951; // converting to degrees

    return (heading); // return the heading so this function can be used
  }


//DRIVERCONTROL
void setDriveMotors() { // set the drive motors based on the left and right joysticks
  int ltrack = leftTrack.get_value();
  lcd::set_text(2,std::to_string(ltrack));

  int rtrack = rightTrack.get_value();
  lcd::set_text(3, std::to_string(rtrack));

  int power  = controller.get_analog(E_CONTROLLER_ANALOG_LEFT_Y);
  int turn   = controller.get_analog(E_CONTROLLER_ANALOG_RIGHT_X);
  int strafe = controller.get_analog(E_CONTROLLER_ANALOG_LEFT_X);

  int lf  = power + turn + strafe;
  int lb  = power + turn - strafe;
  int rf  = power - turn - strafe;
  int rb  = power - turn + strafe;

  fl.move(lf);
  bl.move(lb);
  fr.move(rf);
  br.move(rb);
}
