#include "main.h"

// Autonomous Functions
void intake (int left, int right) {
  li = left;
  ri = right;
}

void setRoll(int upper, int low) {
  roller = low;
  fly = upper;
}


// Driver control Functions
void index() { // control the intakes and motors based on the controller triggers
  int l1 = controller.get_digital(E_CONTROLLER_DIGITAL_L1);
  int l2 = controller.get_digital(E_CONTROLLER_DIGITAL_L2);
  int r1 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
  int r2 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);

  int speed = -127 * (r1 - r2);
  li.move(speed);
  ri.move(speed);
if ((l1 + l2) == 2) {

  roller.move(-127);
  fly.move(127);

}else if ((l1 == 1) & (l2 != 1)) {

  roller.move(127);
  fly.move(127);

} else if ((l1 != 1) & (l2 == 1)) {

  roller.move(-127);
  fly.move(-127);

} else if (r1 == 1) {
  roller = 25;
  fly = 70;
} else {
  roller.move(0);
  fly.move(0);
}
  }
