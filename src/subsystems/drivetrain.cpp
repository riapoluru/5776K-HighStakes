#include "subsystems/drivetrain.hpp"
using namespace okapi::literals;

//intializing motor ports
const int8_t leftFront = 2;
const int8_t leftMiddle = -5;
const int8_t leftBack = 3;

const int8_t rightFront = -1;
const int8_t rightMiddle = 6;
const int8_t rightBack = -7;

//initializing motors with motor ports, cartridge, etc.
okapi::Motor leftFrontMotor(leftFront, true, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::degrees);
okapi::Motor leftMiddleMotor(leftMiddle, true, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::degrees);
okapi::Motor leftBackMotor(leftBack, true, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::degrees);

okapi::Motor rightFrontMotor(rightFront, true, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::degrees);
okapi::Motor rightMiddleMotor(rightMiddle, true, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::degrees);
okapi::Motor rightBackMotor(rightBack, true, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::degrees);


//initializing odom chassis
std::shared_ptr<okapi::OdomChassisController> chassis = okapi::ChassisControllerBuilder()
.withMotors({leftFrontMotor, leftMiddleMotor, leftBackMotor}, {rightFrontMotor, rightMiddleMotor, rightBackMotor})
.withDimensions(okapi::AbstractMotor::gearset::blue, {{4_in, 14.45_in}, okapi::imev5BlueTPR})
//s.withSensors(leftFrontMotor.getEncoder(), rightFrontMotor.getEncoder())
.withOdometry()
.buildOdometry();