#ifndef __DRIVE_H__
#define __DRIVE_H__
#include "main.h"

//declaring motor ports
extern const int8_t leftFront;
extern const int8_t leftMiddle;
extern const int8_t leftBack;

extern const int8_t rightFront;
extern const int8_t rightMiddle; 
extern const int8_t rightBack;

// declaring motors
extern okapi::Motor leftFrontMotor;
extern okapi::Motor leftMiddleMotor;
extern okapi::Motor leftBackMotor;

extern okapi::Motor rightFrontMotor;
extern okapi::Motor rightMiddleMotor;
extern okapi::Motor rightBackMotor;

//declare chassis
extern std::shared_ptr<okapi::OdomChassisController> chassis;
#endif