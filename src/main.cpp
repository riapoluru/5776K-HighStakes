#include "main.h"
#include "lemlib/api.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "subsystems/globals.hpp"

#include "lemlib/api.hpp"

using namespace okapi::literals; // used for units

int lF = 0;
int lM = 0;
int lB = 0;

int rF = 0;
int rM = 0;
int rB = 0;

pros::MotorGroup leftMotors({lF, lM, lB}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({rF, rM, rB}, pros::MotorGearset::blue);

pros::Imu imu(0);

lemlib::Drivetrain drivetrain(&leftMotors, &rightMotors, 10, 3.25, 450, 8);

lemlib::ControllerSettings movePID(10,  // kp
                                   0,   // Ki
                                   3,   // Kd
                                   3,   // windup range
                                   1,   // small error ragne
                                   100, // small error range timeout (ms)
                                   3,   // large error range
                                   500, // large error range timeout (ms)
                                   20   // slew
);

lemlib::ControllerSettings
    turnPID(2,   // proportional gain (kP)
            0,   // integral gain (kI)
            10,  // derivative gain (kD)
            3,   // anti windup
            1,   // small error range, in degrees
            100, // small error range timeout, in milliseconds
            3,   // large error range, in degrees
            500, // large error range timeout, in milliseconds
            0    // maximum acceleration (slew)
    );

lemlib::OdomSensors sensors(nullptr, nullptr, nullptr, mullptr, &imu);

// Driver control fn tanish
//  lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
//                                       10, // minimum output where drivetrain
//                                       will move out of 127 1.019 // expo
//                                       curve gain
//  );

// // input curve for steer input during driver control
// lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
//                                   10, // minimum output where drivetrain will
//                                   move out of 127 1.019 // expo curve gain
// );

lemlib::Chassis chassis(drivetrain, movePID, turnPID,
                        sensors /*,&throttleCurve, //&steerCurve*/);

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
  static bool pressed = false;
  pressed = !pressed;
  if (pressed) {
    pros::lcd::set_text(2, "I was pressed!");
  } else {
    pros::lcd::clear_line(2);
  }
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  pros::lcd::initialize();
  ` chassis.calibrate();

  pros::lcd::register_btn1_cb(on_center_button);

  pros::Task screenTask([&]() {
    while (true) {
      // print robot location to the brain screen
      pros::lcd::print(0, "X: %f", chassis.getPose().x);         // x
      pros::lcd::print(1, "Y: %f", chassis.getPose().y);         // y
      pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
      // log position telemetry
      lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
      // delay to save resources
      pros::delay(50);
    }
  });
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
void competition_initialize() {}

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
void autonomous() {}

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
  pros::Controller controller;

  okapi::Rate rate;

  while (true) {

    // get joystick positions
    int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
    // move the chassis with curvature drive
    chassis.arcade(leftY, rightX);
    // delay to save resources
    pros::delay(10);

    // function calls for intake, outtake, and shooter

    pros::delay(50);
  }
}