#include "main.h"
#include "globals.hpp"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "subsystems/auton.hpp"
#include "subsystems/cata.hpp"
#include "subsystems/drive.hpp"
#include "subsystems/intake.hpp"
#include "subsystems/odo.hpp"
#include "subsystems/pistons.hpp"
#include "subsystems/skills.hpp"

#include "autoSelect/selection.h"

#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"

// drive motors

// const std::uint8_t rightFrontPort = 12;
// const std::uint8_t leftFrontPort = 3;

// const std::uint8_t leftBackPort = 5;
// const std::uint8_t rightBackPort = 14;

// const std::uint8_t leftTopPort = 4;
// const std::uint8_t rightTopPort = 13;

pros::Motor lF(-6, pros::E_MOTOR_GEARSET_06);
pros::Motor lM(-19, pros::E_MOTOR_GEARSET_06);
pros::Motor lB(-9, pros::E_MOTOR_GEARSET_06);

pros::Motor rF(12, pros::E_MOTOR_GEARSET_06);
pros::Motor rM(13, pros::E_MOTOR_GEARSET_06);
pros::Motor rB(14, pros::E_MOTOR_GEARSET_06);

// motor groups
pros::MotorGroup leftMotors({lF, lM, lB});  // left motor group
pros::MotorGroup rightMotors({rF, rM, rB}); // right motor group

lemlib::Drivetrain drive{
    &leftMotors, &rightMotors, 11.5, lemlib::Omniwheel::NEW_325, 450, 4,
};

lemlib::ControllerSettings movePID{
    7,   // kP
    0,   // kI
    1,   // kD
    3,   // anti windup
    1,   // small error range
    100, // small error timeout
    4,   // large error range
    500, // large error timeout
    0    // slew rate
};

pros::Imu intertial1(1);

// pros::Imu intertial2(imuPort2);

lemlib::OdomSensors sensors{nullptr, // no tracking wheels
                            nullptr, nullptr, nullptr, &intertial1};

lemlib::ControllerSettings turnPID{
    2,   // kP
    0,   // kI
    11,  // kD
    3,   // anti windup
    1,   // small error range
    100, // small error timeout
    3,   // large error range
    500, // large error timeout
    0    // slew rate
};

lemlib::Chassis Chassis(drive, movePID, turnPID, sensors);

/*
  ______________________________________________________________________________________________

  Controller Stuff
  ______________________________________________________________________________________________

*/

void screen() {
  while (true) {
    lemlib::Pose pose = Chassis.getPose(); // get chassis position
    pros::lcd::print(0, "X: %f", pose.x);
    pros::lcd::print(1, "Y: %f", pose.y);
    // pros::lcd::print(2, "Theta: %f", pose.theta);
    pros::delay(10);
  }
}

// okapi::IntegratedEncoder leftEncoder = IntegratedEncoder(rightTopPort, true);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  Chassis.calibrate();

  pros::lcd::initialize();

  // print odom values to the brain
  pros::lcd::initialize();
  // pros::Task screen_task(screen);
  selector::init();
  // selector::init();
  // IEInnit();

  pros::Task screenTask([&]() {
    lemlib::Pose pose(0, 0, 0);
    while (true) {

      // print robot location to the brain screen

      pros::lcd::print(0, "X: %f", Chassis.getPose().x); // x
      pros::lcd::print(1, "Y: %f", Chassis.getPose().y); // y
      pros::lcd::print(2, "Theta: %f", Chassis.getPose().theta);
      pros::lcd::print(3, "Encoder LF: %f", lF.get_position());
      pros::lcd::print(4, "Encoder LM: %f", lM.get_position());
      pros::lcd::print(5, "Encoder LB: %f", lB.get_position());
      pros::lcd::print(6, "Encoder RF: %f", rF.get_position());
      pros::lcd::print(7, "Encoder RM: %f", rM.get_position());

      // log position telemetry
      lemlib::telemetrySink()->info("Chassis pose: {}", Chassis.getPose());
      // delay to save resources$
      pros::delay(50);
    }
  });

  // imuInnit();
  intakeInit();
  cataInit();
  hangInit();
  // flipoutMechInnit();
  // resetEncoders();
  // pistonsInnit();
  // lMechInit();
  // balanceInit();
  // blockerInit();
  // PtoInit();
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

int side = -1;
// int num = -1;
void competition_initialize() {
  // if (selector::auton == 1){side = 2;}
  // if (selector::auton == 2){side = 1;}
  // if (selector::auton == 3) {side = 4;}
  // if (selector::auton == -1) {side = 2;}
  // if (selector::auton == -2) {side = 1;}
  // if (selector::auton == -3) {side = 4;}
  // if (selector::auton == 0) {side = 0;}
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

void autonomous() {

  if (selector::auton == 1) {
    closeSide();
  } else if (selector::auton == 2) {
    farTB();
  } else if (selector::auton == 3) {
    noPreload();
  }

  // skills(); // SKILLS

  // closeSide(); //AWP

  // closeSideDisrupt(); //DISRUPT

  // farSide(); //AWP FAR SIDE

  // farTB(); // SIX TRIBALL
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
 * If the robot is disablsed or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
// bool hasRunMacro = false;

// int ArcadeTankToggle = 0;

void opcontrol() {

  pros::Controller controller1(pros::E_CONTROLLER_MASTER);
  while (true) {
    // int rightY = controller1.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    // int leftY = controller1.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

    // Chassis.tank(rightY, leftY);

    int leftJoy = controller1.get_analog(
        pros::E_CONTROLLER_ANALOG_LEFT_Y); // vert left joystick
    int rightJoy = controller1.get_analog(
        pros::E_CONTROLLER_ANALOG_RIGHT_X); // horiz right joystick
    int tankRightJoy = controller1.get_analog(
        pros::E_CONTROLLER_ANALOG_RIGHT_Y); // horiz right joystick
    // Chassis.tank(leftJoy, tankRightJoy, -5);
    Chassis.setBrakeMode(MOTOR_BRAKE_COAST);
    // Chassis.setBrakeMode(MOTOR_BRAKE_BRAKE);

    // Chassis.tank(leftJoy, tankRightJoy, 0);

    // Chassis.curvature(abs(leftJoy) > 16 ? leftJoy: 0, abs(rightJoy) > 16 ?
    // rightJoy: 0); Chassis.tank(abs(leftJoy) > 5 ? leftJoy: 0,
    // abs(tankRightJoy) > 30 ? tankRightJoy: 0);
    Chassis.arcade(leftJoy, 1.05 * rightJoy, 2);

    // // DRIVE MODE TOGGLE
    // if (controller.getDigital(ControllerDigital::left) == 1){
    //   if (ArcadeTankToggle == 0) {
    //     ArcadeTankToggle = 1;
    //   }
    //   if (ArcadeTankToggle == 2) {
    //     ArcadeTankToggle = 3;
    //   }
    // }
    // else if (controller.getDigital(ControllerDigital::left) == 0){
    //   if (ArcadeTankToggle == 1) {
    //     ArcadeTankToggle = 2;
    //   }
    //   if (ArcadeTankToggle == 3) {
    //     ArcadeTankToggle = 0;
    //   }
    // }

    // if (ArcadeTankToggle == 0 || ArcadeTankToggle == 3) { //ARCADE TOGGLE
    //   //Chassis.arcade(leftJoy, 1.05*rightJoy, 2);
    //   Chassis.tank(leftJoy, tankRightJoy, -7.5);
    //   Chassis.setBrakeMode(MOTOR_BRAKE_COAST);
    // }
    // else if (ArcadeTankToggle == 1 || ArcadeTankToggle == 2) { //TANK TOGGLE
    //   //Chassis.tank(leftJoy, tankRightJoy, -10);
    //   Chassis.arcade(leftJoy, 1.05*rightJoy, 2);

    //   Chassis.setBrakeMode(MOTOR_BRAKE_COAST);
    // }

    pros::delay(10);
  }
}