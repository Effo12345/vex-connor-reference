#include "main.h"

//MOTORS
Motor fl(12, E_MOTOR_GEARSET_18, false);
Motor bl(2, E_MOTOR_GEARSET_18, false);
Motor fr(15, E_MOTOR_GEARSET_18, true);
Motor br(14, E_MOTOR_GEARSET_18, true);
Motor li(17, E_MOTOR_GEARSET_18, false);
Motor ri(16, E_MOTOR_GEARSET_18, true);
Motor fly(19, E_MOTOR_GEARSET_06, false);
Motor roller(18, E_MOTOR_GEARSET_06, false);

//SENSORS
Imu inertial(9);
ADIUltrasonic topSonic({{13, 'A', 'B'}});
ADIEncoder leftTrack('G', 'H', false);
ADIEncoder rightTrack('F', 'E', true);
ADIEncoder backTrack('A', 'B');
//CONTROLLER
pros::Controller controller(pros::E_CONTROLLER_MASTER);
