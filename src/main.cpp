#include "main.h"
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

// controller
pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::Motor lowerintake(3);
pros::Motor upperintake(1);
pros::MotorGroup intake({-3, 1});
pros::adi::DigitalOut tongue('G');
pros::adi::DigitalOut wing('F');
pros::adi::DigitalOut hood('H');
std::uint32_t now = pros::millis();
// motor groups
pros::MotorGroup
    leftMotors({-12, -10, 9},
               pros::MotorGearset::blue); // left motor group - ports 3
                                          // (reversed), 4, 5 (reversed)
pros::MotorGroup rightMotors(
    {15, 8, -18},
    pros::MotorGearset::blue); // right motor group - ports 6, 7, 9 (reversed)

// Inertial Sensor on port 10
pros::Imu imu(14);

// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed
// pros::Rotation horizontalEnc(20);
// vertical tracking wheel encoder. Rotation sensor, port 11, reversed
// pros::Rotation verticalEnc(-11);
// horizontal tracking wheel. 2.75" diameter, 5.75" offset, back of the robot
// (negative)
// lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_275,
// -5.75);
// vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot
// (negative)
// lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_275,
// -2.5);

// drivetrain settings
lemlib::Drivetrain drivetrain(
    &leftMotors,                // left motor group
    &rightMotors,               // right motor group
    10,                         // 10 inch track width
    lemlib::Omniwheel::NEW_325, // using new 4" omnis
    450,                        // drivetrain rpm is 360
    2 // horizontal drift is 2. If we had traction wheels, it would have been 8
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
// angular motion controller
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
                            nullptr, // vertical tracking wheel 2, set to
                                     // nullptr as we don't have a second one
                            nullptr, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to
                                     // nullptr as we don't have a second one
                            &imu     // inertial sensor
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve
    throttleCurve(3,    // joystick deadband out of 127
                  10,   // minimum output where drivetrain will move out of 127
                  1.019 // expo curve gain
    );

// input curve for steer input during driver control
lemlib::ExpoDriveCurve
    steerCurve(3,    // joystick deadband out of 127
               10,   // minimum output where drivetrain will move out of 127
               1.019 // expo curve gain
    );

// create the chassis
lemlib::Chassis chassis(drivetrain, lateral_controller, angularController,
                        sensors, &throttleCurve, &steerCurve);

void cdrift(float lV, float rV, int timeout, bool cst) {
  (cst == true) ? (leftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST),
                   rightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST))
                : (leftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE),
                   rightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE));
  leftMotors.move(lV);
  rightMotors.move(rV);
  pros::delay(timeout);
  leftMotors.brake();
  rightMotors.brake();
}
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
// initialize function. Runs on program startup
void initialize() {
  pros::lcd::initialize(); // initialize brain screen
  chassis.calibrate();     // calibrate sensors
  // print position to brain screen
  pros::Task screen_task([&]() {
    while (true) {
      // print robot location to the brain screen
      pros::lcd::print(0, "X: %f", chassis.getPose().x);         // x
      pros::lcd::print(1, "Y: %f", chassis.getPose().y);         // y
      pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
      // delay to save resources
      pros::delay(20);
    }
  });
}

/**
 * Runs while the robot is disabled
 */
void disabled() {}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {}

void linpid() {
  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
  chassis.setPose(lemlib::Pose(-48, 24, 180));
  chassis.moveToPoint(-48, -24, 2000);
  // chassis.follow(linpid_txt, 3, 2000);
  chassis.waitUntilDone();
}
void angpid() {
  chassis.setPose(lemlib::Pose(0, 0, 0));
  chassis.turnToHeading(90, 2000);
  chassis.waitUntilDone();
}

void sawp(){
wing.set_value(true);
hood.set_value(false);
upperintake.move(127);
lowerintake.move(-127);
chassis.setPose(0, 0, 0);
//matchload 1
chassis.moveToPoint(0, 30, 800, {.maxSpeed = 127});
chassis.turnToHeading(90, 600);
tongue.set_value(true);
chassis.moveToPose(14, 35, 90, 800, {.lead = 0.1, .maxSpeed = 127 });
chassis.waitUntilDone();
pros::delay(420);
//go to long goal
chassis.moveToPose(-20.25, 35.5, 90, 800, {.forwards = false, .lead = 0.1, .maxSpeed = 127});
chassis.waitUntilDone();
upperintake.move(-127);
lowerintake.move(-127);
pros::delay(850);
//turn to mid
chassis.moveToPoint(-7, 32, 700, {.maxSpeed = 127});
tongue.set_value(false);
chassis.turnToHeading(225, 600);
upperintake.move(127);
lowerintake.move(-127);
//go to mid
chassis.moveToPoint(-24.5, 10.5, 900, {.maxSpeed = 127});
chassis.waitUntil(19);
tongue.set_value(true);
chassis.waitUntilDone();
tongue.set_value(false);
chassis.turnToHeading(-180, 600);
//second mid
chassis.moveToPoint(-24.5, -38, 1200, {.maxSpeed = 127});
chassis.waitUntil(27.8);
tongue.set_value(true);
chassis.waitUntilDone();
hood.set_value(true);
chassis.turnToHeading(132, 700, {.maxSpeed = 127});
//go to mid goal and score
chassis.moveToPose(-37, -23.2, 132, 1000, {.forwards = false, .lead = 0.1,  .maxSpeed = 127});
chassis.waitUntilDone();
lowerintake.move(127);
pros::delay(150);
lowerintake.move(-127);
upperintake.move(-127);
pros::delay(550);
lowerintake.move(-127);
upperintake.move(127);
hood.set_value(false);
chassis.moveToPoint(2, -58, 1100, {.maxSpeed = 127});
chassis.turnToHeading(90, 600);
chassis.moveToPose(14, -58, 90, 800, {.lead = 0.1, .maxSpeed = 127 });
chassis.waitUntilDone();
pros::delay(400);
chassis.moveToPose(-20, -58, 90, 800, {.forwards = false, .lead = 0.1, .maxSpeed = 127});
chassis.waitUntilDone();
upperintake.move(-127);
lowerintake.move(-127);
chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
}

void elims(){
    //go to mid 3 and tongue
    wing.set_value(true);
    chassis.setPose(0, 0, 0);
    upperintake.move(127);
    lowerintake.move(-127);
    chassis.moveToPose(7.41, 23.92, 38.63, 1000, {.lead = 0.23, .maxSpeed = 127, .minSpeed = 20, .earlyExitRange = 0.75});
    chassis.waitUntil(18.9);
    tongue.set_value(true);
    chassis.turnToHeading(127, 600, {.maxSpeed=127});
    //go to matchload pos
    chassis.moveToPoint(31.95, -2.24, 900, {.maxSpeed = 127});
    chassis.turnToHeading(180, 600);
    //enter matchload
    chassis.moveToPose(33, -20, 180, 900, {.lead = 0.05, .maxSpeed=127});
    chassis.waitUntilDone();
    pros::delay(450);
    //go to goal and score
    chassis.moveToPose(33.5, 20, 180, 1200, {.forwards = false, .lead = 0.1, .maxSpeed = 127});
    chassis.waitUntil(15.5);
    chassis.waitUntilDone();
    lowerintake.move(127);
    pros::delay(250);
    lowerintake.move(-127);
    upperintake.move(-127);
    pros::delay(1800);
    upperintake.move(127);
    tongue.set_value(false);
    //goes to wing pos and turns
    // chassis.moveToPoint(32.7, 5, 800, {.maxSpeed=127});
    // chassis.turnToHeading(302, 600, {.maxSpeed=127});
    // chassis.waitUntil(5);
    lowerintake.move(0);
    upperintake.move(0);
    wing.set_value(false);
    chassis.moveToPose(20, 7, 0, 1200, {.maxSpeed=127});
    //go into wing
    chassis.moveToPoint(19.7, 32, 1000, {.maxSpeed=127});
    chassis.waitUntilDone();
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);   
}

void quals() {
  // go to matchload and matchload
  wing.set_value(true);
  chassis.setPose(0, 0, 0);
  upperintake.move(127);
  lowerintake.move(-127);
  hood.set_value(false);
  chassis.moveToPoint(2, 32, 1200, {.maxSpeed = 127});
  chassis.turnToHeading(-90, 700);
  tongue.set_value(true);
  ;
  chassis.moveToPose(-12, 30, -90, 800, {.lead = 0.1, .maxSpeed = 127});
  chassis.waitUntilDone();
  pros::delay(517);
  // go to long goal
  chassis.moveToPose(21.1, 30.5, -90, 800,
                     {.forwards = false, .lead = 0.1, .maxSpeed = 127});
  chassis.waitUntilDone();
  lowerintake.move(-127);
  upperintake.move(-127);
  pros::delay(1050);
  tongue.set_value(false);
  // go to middle 3
  chassis.moveToPoint(11, 30, 700, {.maxSpeed = 127});
  chassis.turnToHeading(-213, 600);
  upperintake.move(127);
  // tongue
  chassis.moveToPoint(29.3, 8, 1200, {.maxSpeed = 127});
  chassis.waitUntil(21.57);
  tongue.set_value(true);
  chassis.waitUntilDone();
  chassis.turnToHeading(-47, 800, {.maxSpeed = 127});
  // go to mid goal and score
  chassis.moveToPose(48.5, -15, -47, 1200,
                     {.forwards = false, .lead = 0.1, .maxSpeed = 127});
  chassis.waitUntilDone();
  lowerintake.move(-110);
  upperintake.move(-95);
  hood.set_value(true);
  pros::delay(750);
  hood.set_value(false);
  tongue.set_value(false);
  wing.set_value(false);
  // go to wing pos
  chassis.moveToPoint(23.27, 13, 1000, {.maxSpeed = 127});
  chassis.waitUntilDone();
  chassis.turnToHeading(-90, 700);
  chassis.moveToPoint(47, 13, 800, {.forwards = false, .maxSpeed = 127});
  chassis.waitUntilDone();
  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
}



/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the
 * features LemLib has to offer
 */
//  	rd::Selector selector({
//     {"SAWP", sawp},
//      {"Eliminations", elims},
//      {"Quals 4+3", quals},
//      {"4 Ball Rush", beatC},
//      {"skillsdoesnt work", skills}
// });

 void autonomous() {
   sawp();

   
}

/**
 * Runs in driver control
 */
void opcontrol() {
  // controller
  bool tongue_state = false;
  bool wing_state = false;
  bool hood_state = false;
  // loop to continuously update motors
  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
  while (true) {
    // get joystick positions
    int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int rightX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
    // move the chassis with curvature drive
    chassis.arcade(leftY, rightX);
    // Intake / lock logic
    if (master.get_digital(DIGITAL_R1)) {
      hood.set_value(true);
      upperintake.move(-127);
      lowerintake.move(-127);
    } else if (master.get_digital(DIGITAL_R2)) {
      hood.set_value(false);
      upperintake.move(-127);
      lowerintake.move(-127);
    } else if (master.get_digital(DIGITAL_L1)) {
      upperintake.move(127);
      lowerintake.move(-127);
    } else if (master.get_digital(DIGITAL_L2)) {
      upperintake.move(127);
      lowerintake.move(127);
    } else {
      upperintake.brake();
      lowerintake.brake();
    }

    if ((master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B))) {
      tongue_state = !tongue_state;
      tongue.set_value(tongue_state);
    }
    if ((master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN))) {
      wing_state = !wing_state;
      wing.set_value(wing_state);
    }
    if ((master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT))) {
      hood_state = !hood_state;
      hood.set_value(hood_state);
    }

    // // //move hood manually
    // hood.button_toggle(master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT));

    // delay to save resources
    pros::delay(10);
  }
}
