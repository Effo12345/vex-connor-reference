#include "main.h"

// declare all electronics so any file can recognize them

//motor setup
extern Motor fl;
extern Motor bl;
extern Motor fr;
extern Motor br;
extern Motor li;
extern Motor ri;
extern Motor roller;
extern Motor fly;

//sensor setup
extern Imu inertial;
extern ADIEncoder leftTrack;
extern ADIEncoder rightTrack;
extern ADIEncoder backTrack;
extern ADIUltrasonic topSonic;

//controller setup
extern Controller controller;
