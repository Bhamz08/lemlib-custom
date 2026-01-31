#include "main.h"
#include "intake.hpp"
#include "lemlib-tarball/api.hpp"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/asset.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include "robodash/api.h"
#include "subsystems.hpp"
#include <vector>


// controller
pros::Controller master(pros::E_CONTROLLER_MASTER);

// motor groups
pros::MotorGroup leftMotors({-12, -10, 9}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({15, 8, -18}, pros::MotorGearset::blue);

// Inertial Sensor on port 10
pros::Imu imu(14);

// drivetrain settings
lemlib::Drivetrain
    drivetrain(&leftMotors,                // left motor group
               &rightMotors,               // right motor group
               10,                         // 10 inch track width
               lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
               450,                        // drivetrain rpm
               2                           // horizontal drift
    );

lemlib::ControllerSettings
    lateral_controller(5.8, // proportional gain (kP)
                       0,   // integral gain (kI)
                       5,   // derivative gain (kD)
                       0,   // anti windup
                       3,   // small error range, in inches
                       250, // small error range timeout, in milliseconds
                       5,   // large error range, in inches
                       500, // large error range timeout, in milliseconds
                       0    // maximum acceleration (slew)
    );

lemlib::ControllerSettings
    angularController(3.16, // proportional gain (kP)
                      0,    // integral gain (kI)
                      20,   // derivative gain (kD)
                      5,    // anti windup
                      1,    // small error range, in degrees
                      100,  // small error range timeout, in milliseconds
                      5,    // large error range, in degrees
                      500,  // large error range timeout, in milliseconds
                      0     // maximum acceleration (slew)
    );

// sensors for odometry
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2
                            nullptr, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2
                            &imu     // inertial sensor
);

// input curves
lemlib::ExpoDriveCurve throttleCurve(3, 10, 1.019);
lemlib::ExpoDriveCurve steerCurve(3, 10, 1.019);

// create the chassis
lemlib::Chassis chassis(drivetrain, lateral_controller, angularController,
                        sensors, &throttleCurve, &steerCurve);

void cdrift(float lV, float rV, int timeout, bool cst) {
  if (cst) {
    leftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
    rightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
  } else {
    leftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
    rightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
  }
  leftMotors.move(lV);
  rightMotors.move(rV);
  pros::delay(timeout);
  leftMotors.brake();
  rightMotors.brake();
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 */
void initialize() {
  // intake control
  pros::Task intakeTask(asyncIntakeControl);

  pros::lcd::initialize(); // initialize brain screen
  chassis.calibrate();     // calibrate sensors

  // print position to brain screen
  pros::Task screen_task([&]() {
    while (true) {
      pros::lcd::print(0, "X: %f", chassis.getPose().x);
      pros::lcd::print(1, "Y: %f", chassis.getPose().y);
      pros::lcd::print(2, "Theta: %f", chassis.getPose().theta);
      pros::delay(20);
    }
  });
}

void disabled() {}

void competition_initialize() {}

void linpid() {
  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
  chassis.setPose(lemlib::Pose(-48, 24, 180));
  chassis.moveToPoint(-48, -24, 2000);
  chassis.waitUntilDone();
}

void angpid() {
  chassis.setPose(lemlib::Pose(0, 0, 0));
  chassis.turnToHeading(90, 2000);
  chassis.waitUntilDone();
}

void sawp() {
  wing.set_value(true);
  hood.set_value(false);
  intakeLoad();
  chassis.setPose(0, 0, 0);

  // matchload 1
  chassis.moveToPoint(0, 28.5, 800, {.maxSpeed = 127});
  chassis.turnToHeading(90, 600);
  tongue.set_value(true);
  chassis.moveToPose(14, 33.5, 90, 800, {.lead = 0.1, .maxSpeed = 127});
  chassis.waitUntilDone();
  pros::delay(285);
  // go to long goal
  chassis.moveToPoint(-24, 34.5, 800, {.forwards = false, .maxSpeed = 127});
  chassis.waitUntilDone();
  intakeLongGoal();
  pros::delay(875);

  // turn to mid
  chassis.moveToPoint(-7, 32, 700, {.maxSpeed = 127});
  tongue.set_value(false);
  chassis.turnToHeading(215, 600);
  intakeLoad();
  // go to mid
  chassis.moveToPoint(-24.5, 10.5, 800, {.maxSpeed = 127});
  chassis.waitUntil(19);
  tongue.set_value(true);
  chassis.waitUntilDone();
  chassis.turnToHeading(-181, 600);
  tongue.set_value(false);
  // second mid
  chassis.moveToPoint(-24.5, -38, 1200, {.maxSpeed = 127});
  chassis.waitUntil(27.95);
  tongue.set_value(true);
  chassis.waitUntilDone();
  hood.set_value(true);
  chassis.turnToHeading(128, 550, {.maxSpeed = 127});

  // go to mid goal and score
  chassis.moveToPose(-38, -19, 128, 1000,
                     {.forwards = false, .lead = 0.2, .maxSpeed = 127});
  chassis.waitUntilDone();
  intakeCenterGoal();
  pros::delay(550);
  intakeLoad();
  chassis.moveToPoint(-2, -60, 1200, {.maxSpeed = 127});
  chassis.turnToHeading(90, 600);
  chassis.moveToPose(19.5, -60, 90, 800, {.lead = 0.1, .maxSpeed = 127});
  chassis.waitUntilDone();
  hood.set_value(false);
  pros::delay(285);
  chassis.moveToPose(-25, -61, 90, 900, {.forwards = false, .lead = 0.1,.maxSpeed = 127});
  chassis.waitUntilDone();
  intakeLongGoal();
}

void elims() {
  // go to mid 3 and tongue
  wing.set_value(true);
  chassis.setPose(0, 0, 0);
  intakeLoad();
  chassis.moveToPose(
      7.41, 23.92, 40, 1100,
      {.lead = 0.23, .maxSpeed = 110, .minSpeed = 20, .earlyExitRange = 0.75});
  chassis.waitUntil(18.75);
  tongue.set_value(true);
  chassis.turnToHeading(127, 600, {.maxSpeed = 127});
  // go to matchload pos
  chassis.moveToPoint(31.5, -2.24, 900, {.maxSpeed = 127});
  chassis.turnToHeading(180, 600);
  // enter matchload
  chassis.moveToPose(33, -20, 180, 900, {.lead = 0.05, .maxSpeed = 127});
  chassis.waitUntilDone();
  pros::delay(405);
  // go to goal and score
  chassis.moveToPose(33.5, 19, 180, 1000,
                     {.forwards = false, .lead = 0.1, .maxSpeed = 127});
  chassis.waitUntilDone();
  intakeLongGoal();
  pros::delay(1700);
  intakeLoad();
  tongue.set_value(false);
  // goes to wing pos and turns
  wing.set_value(false);
  chassis.moveToPose(21.75, 7, 355, 1500, {.maxSpeed = 127});
  // go into wing
  chassis.moveToPoint(21.75, 33, 1000, {.maxSpeed = 127});
  chassis.waitUntilDone();
  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
}

void quals() {
  // go to matchload and matchload
  wing.set_value(true);
  chassis.setPose(0, 0, 0);
  intakeLoad();
  hood.set_value(false);
  tongue.set_value(true);
  chassis.moveToPoint(0, 32, 1000, {.maxSpeed = 127});
  chassis.turnToHeading(-90, 700);
  chassis.moveToPose(-16, 32.75, -90, 800, {.lead = 0.1, .maxSpeed = 127});
  chassis.waitUntilDone();
  pros::delay(310);
  // go to long goal
  chassis.moveToPose(22.5, 33, -90, 800,
                     {.forwards = false, .lead = 0.1, .maxSpeed = 127});
  chassis.waitUntilDone();
  intakeLongGoal();
  pros::delay(1000);
  tongue.set_value(false);
  // go to middle 3
  chassis.moveToPoint(12, 30, 800, {.maxSpeed = 127});
  chassis.turnToHeading(-200, 600);
  intakeLoad();
  // tongue
  chassis.moveToPoint(28.35, 9.2, 1100, {.maxSpeed = 100});
  chassis.waitUntil(19);
  tongue.set_value(true);
  chassis.waitUntilDone();
  chassis.turnToHeading(-47, 800, {.maxSpeed = 127});
  // go to mid goal and score
  chassis.moveToPose(44.5, -7, -47, 1200, {.forwards = false, .lead = 0.1, .maxSpeed = 127});
  chassis.waitUntilDone();
  intakeCenterGoal();
  tongue.set_value(false);
  pros::delay(400);
  intakeLoad();
  wing.set_value(true);
  // // go to wing pos
  chassis.moveToPoint(21, 22.45, 1000, {.maxSpeed = 127});
  chassis.waitUntilDone();
  chassis.turnToHeading(-90, 700);
  chassis.moveToPoint(43, 22.45, 800, {.forwards = false, .maxSpeed = 127});
  chassis.waitUntilDone();
  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
}

void Rush() {
  // go to mid 3 and tongue
  wing.set_value(true);
  hood.set_value(false);
  chassis.setPose(0, 0, 0);
  intakeLoad();
  chassis.moveToPose(7.67, 24, 40, 1100,{.lead = 0.23, .maxSpeed = 127, .minSpeed = 20, .earlyExitRange = 0.75});
  chassis.waitUntil(18.75);
  tongue.set_value(true);
  //  go to matchload pos
  chassis.moveToPose(44, 29.75, -180, 1100,{.forwards = false, .lead = 0.55, .maxSpeed = 127});
  // chassis.turnToHeading(160, 600);
  chassis.swingToHeading(-220, lemlib::DriveSide::RIGHT, 1000, {.maxSpeed = 127, .minSpeed = 120});
  chassis.waitUntilDone();
  intakeLongGoal();
  pros::delay(625);
  tongue.set_value(false);
  // goes to wing pos and turns
  intakeHalt();
  wing.set_value(false);
  chassis.moveToPose(45, 5, -175, 1050, {.maxSpeed = 127});
  // go into wing
  chassis.moveToPoint(45.25, 44, 1000, {.forwards = false, .maxSpeed = 127, .minSpeed = 20});
  chassis.waitUntilDone();
  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
}

void skills() {
  wing.set_value(true);
  hood.set_value(true);
  chassis.setPose(0, 0, 0);
  intakeLoad();
  // intake center goal blocks
  chassis.moveToPoint(0, 15, 700, {.maxSpeed = 127});
  chassis.turnToHeading(-45, 600);
  chassis.moveToPoint(-8, 22, 700, {.maxSpeed = 127});
  chassis.turnToHeading(-140, 600);
  // score on center goal
  chassis.moveToPoint(6.5, 34.5, 800, {.forwards = false, .maxSpeed = 127});
  chassis.waitUntilDone();
  setIntake(120, 110);
  pros::delay(420);
  intakeLoad();
  // get matchloader
  chassis.moveToPoint(-31, 0, 1250, {.maxSpeed = 110});
  chassis.turnToHeading(-180, 400);
  tongue.set_value(true);
  chassis.moveToPoint(-31, -15.5, 800);
  // go to long goal
  pros::delay(2100);
  chassis.moveToPoint(-31, 0, 1100, {.forwards = false});
  chassis.turnToHeading(-225, 500);
  chassis.moveToPoint(-43, 15, 800, {.forwards = false});
  tongue.set_value(false);
  intakeHalt();
  setIntake(0, 0);
  chassis.turnToHeading(0, 700);
  wing.set_value(false);
  chassis.moveToPoint(-47, 90, 2500);
  chassis.turnToHeading(90, 500);
  hood.set_value(false);
  //score on long goal
  chassis.moveToPoint(-40, 93, 800);
  chassis.turnToHeading(0, 600);
  chassis.moveToPose(-40, 60, 0, 2000, {.forwards = false});
  pros::delay(1000);
  intakeLongGoal();
  tongue.set_value(true);
  pros::delay(2000);
  chassis.turnToHeading(5, 600);
  intakeLoad();
  // get second matchloader
  chassis.setPose(0, 0, 0);
  chassis.moveToPose(0, 43, 0, 2000);
  pros::delay(3000);
  chassis.moveToPose(0, -30, 0, 1500, {.forwards = false});
  pros::delay(1000);
  intakeLongGoal();
  tongue.set_value(false);
  pros::delay(3000);
  //barrier cross
  chassis.setPose(0, 0, 0);
  chassis.moveToPoint(0, 23, 800);
  chassis.turnToHeading(60, 600);
  chassis.moveToPose(32, 34, 90, 1200, {.minSpeed = 70});
  //cross
  intakeLoad();
  chassis.moveToPoint(100, 34, 3000, {.maxSpeed = 80});
  //wall reset
  chassis.moveToPose(67, 29, 90, 800, {.forwards = false});
  chassis.turnToHeading(180, 600);
  chassis.moveToPoint(67, 50, 900, {.forwards = false});
  chassis.setPose(0, 0, 0);

  
  /*
  chassis.moveToPoint(-30, 80, 1100, {.forwards = false});
  chassis.turnToHeading(0, 500);
  chassis.waitUntilDone();
  hood.set_value(false);
  chassis.moveToPoint(-30, 60, 1000, {.forwards = false});
  chassis.waitUntilDone();
  intakeOuttake();
  pros::delay(75);
  intakeLongGoal();
  tongue.set_value(true);*/

  /*
  //get second matchloader
  pros::delay(5000);
  intakeLoad();
  chassis.turnToHeading(0, 250);
  chassis.setPose(0, 0, 0);
  chassis.moveToPose(0, 27, 0, 900);
  chassis.waitUntilDone();
  pros::delay(2500);
  chassis.moveToPoint(0, 0, 900);
  intakeLongGoal();*/
}
/*
rd::Selector selector({{"Best auton", sawp},
                       {"Simple auton", elims},
                       {"Beat C", Rush},
                       {"Good auton", quals},
                       {"Skills", skills}});*/
void autonomous() {
  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
  sawp();
  // selector.run_auton();
}

void opcontrol() {
  bool tongue_state = false;
  bool wing_state = false;
  bool hood_state = false;
  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
  while (true) {
    int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int rightX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
    int rightY = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
    chassis.arcade(leftY, rightX);

    if (master.get_digital(DIGITAL_R1)) {
      intakeCenterGoal();
    } else if (master.get_digital(DIGITAL_R2)) {
      intakeLongGoal();
    } else if (master.get_digital(DIGITAL_L1)) {
      intakeLoad();
    } else if (master.get_digital(DIGITAL_L2)) {
      intakeOuttake();
    } else {
      intakeHalt();
    }

    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
      tongue_state = !tongue_state;
      tongue.set_value(tongue_state);
    }
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
      wing_state = !wing_state;
      wing.set_value(wing_state);
    }
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
      hood_state = !hood_state;
      hood.set_value(hood_state);
    }

    // Heat sensing
    double intakeTemp = lowerIntake.get_temperature();

    // Drive Heat sensing
    std::vector<double> leftTemps = leftMotors.get_temperature_all();
    std::vector<double> rightTemps = rightMotors.get_temperature_all();
    double driveTotalTemp = 0;
    int motorCount = 0;
    for (double t : leftTemps) {
      if (t != PROS_ERR_F) {
        driveTotalTemp += t;
        motorCount++;
      }
    }
    for (double t : rightTemps) {
      if (t != PROS_ERR_F) {
        driveTotalTemp += t;
        motorCount++;
      }
    }
    double driveAvgTemp = (motorCount > 0) ? (driveTotalTemp / motorCount) : 0;

    // print interval
    if (pros::millis() % 200 < 15) {
      master.print(0, 0, "Intake Temp: %.0fC", intakeTemp);
      pros::delay(50);
      master.print(1, 0, "Drive Temp: %.0fC", driveAvgTemp);
    }
    // vibration warning
    if ((intakeTemp > 45 || driveAvgTemp > 45) && pros::millis() % 1000 < 15) {
      master.rumble("---");
    }

    pros::delay(10);
  }
}
