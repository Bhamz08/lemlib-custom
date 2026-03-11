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
//#include "auton.hpp"


// controller
pros::Controller master(pros::E_CONTROLLER_MASTER);

// motor groups
pros::MotorGroup leftMotors({-15, -13, 14}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({17, 20, -18}, pros::MotorGearset::blue);

// Inertial Sensor on port 10
pros::Imu imu(2);

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



namespace Misc{
    void cdrift(float lV, float rV, int timeout, bool cst = true){
        (cst == true) ? (leftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST), rightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST)) : (leftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE), rightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE));
        leftMotors.move(lV);
        rightMotors.move(rV);
        pros::delay(timeout);
        leftMotors.brake();
        rightMotors.brake();
    }
    void cdrift(float lV, float rV){
        leftMotors.move(lV);
        rightMotors.move(rV);
    }
  }

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

void counter(){
    wing.set_value(true);
  hood.set_value(true);
  intakeLoad();
  chassis.setPose(0, 0, 0);

  // matchload 1
  chassis.moveToPoint(0, 29, 800, {.maxSpeed = 127});
  chassis.turnToHeading(90, 700, {.maxSpeed = 127});
  tongue.set_value(true);
  chassis.moveToPose(16, 31.5, 90, 800, {.lead = 0.1,.maxSpeed = 127,.minSpeed = 5});
  chassis.waitUntilDone();
  pros::delay(245);
  // go to long goal
  chassis.moveToPoint(-24, 32.5, 1000, {.forwards = false, .maxSpeed = 115});
  chassis.waitUntilDone();
  intakeLongGoal();
  tongue.set_value(false);
  pros::delay(1000);
  intakeLoad();


  //turn to mid and go to first stack
  chassis.turnToHeading(185.5, 700);
  chassis.moveToPoint(-16, 14.76, 800, {.maxSpeed = 115});
  chassis.waitUntil(7);
  tongue.set_value(true);

  //second stack
  chassis.moveToPoint(-19, -36, 1500, {.maxSpeed = 127});
  chassis.waitUntil(15);
  tongue.set_value(false);
  chassis.waitUntil(43.5);
  tongue.set_value(true);
  chassis.waitUntilDone();
  hood.set_value(true);
  chassis.turnToHeading(128, 700, {.maxSpeed = 127});

  //go to long goal
  chassis.moveToPoint(0, -56.5, 1000, {.maxSpeed = 127});
  chassis.turnToHeading(90, 700);
  chassis.moveToPoint(-25, -56.5, 1000, {.forwards = false, .maxSpeed = 127});
  chassis.waitUntilDone();
  intakeLongGoal();
  pros::delay(1000);
  intakeLoad();

  //go to matchload
  chassis.moveToPose(27, -55.25, 90, 1050, {.lead = 0.1, .maxSpeed = 110});
  chassis.waitUntilDone();
  pros::delay(260);
  intakeLoad();

  //go to mid goal
  chassis.moveToPoint(0, -58.5, 1000, {.forwards = false, .maxSpeed = 127});
  chassis.moveToPose(-30.5, -19.5, 128, 1000,{.forwards = false, .lead = 0.15, .maxSpeed = 127});
  chassis.waitUntilDone();
  intakeCenterGoal();

  //wing



}

void sawp() {
  wing.set_value(true);
  hood.set_value(true);
  intakeLoad();
  chassis.setPose(0, 0, 0);

  // matchload 1
  chassis.moveToPoint(0, 27, 800, {.maxSpeed = 127});
  chassis.turnToHeading(90, 700, {.maxSpeed = 127});
  tongue.set_value(true);
  chassis.moveToPose(16.5, 31.5, 90, 800, {.lead = 0.1,.maxSpeed = 127,.minSpeed = 5});
  chassis.waitUntilDone();
  pros::delay(260);
  // go to long goal
  chassis.moveToPoint(-24, 33, 1000, {.forwards = false, .maxSpeed = 115});
  chassis.waitUntilDone();
  intakeLongGoal();
  tongue.set_value(false);
  pros::delay(1000);
  intakeLoad();


  //turn to mid and go to first stack
  chassis.turnToHeading(185.5, 700);
  chassis.moveToPoint(-16, 14.76, 800, {.maxSpeed = 120});
  chassis.waitUntil(6);
  tongue.set_value(true);

  //second stack
  chassis.moveToPoint(-19, -39, 1500, {.maxSpeed = 127});
  chassis.waitUntil(10);
  tongue.set_value(false);
  chassis.waitUntil(44.5);
  tongue.set_value(true);
  chassis.waitUntilDone();
  hood.set_value(true);
  chassis.turnToHeading(130, 700, {.maxSpeed = 127});

  //go to mid goal and score
  hood.set_value(true);
  chassis.moveToPose(-30.5, -19.5, 130, 1000,{.forwards = false, .lead = 0.15, .maxSpeed = 127});
  chassis.waitUntilDone();
  intakeCenterGoal();
  pros::delay(600);  
  intakeLoad();

  // // //go to matchload
  chassis.moveToPoint(0, -56.5, 1000, {.maxSpeed = 127});
  chassis.turnToHeading(90, 700);
  chassis.moveToPose(27, -55.25, 90, 1000, {.lead = 0.1, .maxSpeed = 127});
  chassis.waitUntilDone();
  pros::delay(260);
  intakeLoad();

  // //go to long goal
  hood.set_value(true);
  chassis.moveToPoint(-25, -56, 1000, {.forwards = false, .maxSpeed = 127});
  chassis.waitUntilDone();
  tongue.set_value(false);
  intakeLongGoal();

}

void rightelims(){
  wing.set_value(true);
  hood.set_value(true);
  intakeLoad();
  chassis.setPose(0, 0, 0);

  // matchload 1
  chassis.moveToPoint(0, 28.25, 800, {.maxSpeed = 127});
  chassis.turnToHeading(90, 600, {.maxSpeed = 127});
  tongue.set_value(true);
  chassis.moveToPose(14, 31.25, 90, 800, {.lead = 0.05, .maxSpeed = 127});
  chassis.waitUntilDone();
  pros::delay(260);
  // go to long goal
  chassis.moveToPoint(-24, 32.2, 950, {.forwards = false, .maxSpeed = 115});
  chassis.waitUntilDone();
  intakeLongGoal();
  tongue.set_value(false);
  pros::delay(900);
  intakeLoad();

  //turn to mid
  chassis.turnToHeading(185.5, 700);
  chassis.moveToPoint(-16, 14.76, 900, {.maxSpeed = 115});
  chassis.waitUntil(7);
  tongue.set_value(true);

  //go to low goal and score
  chassis.moveToPose(-29.6, -1.35, 225, 900, {.lead = 0.2, .maxSpeed = 127});
  chassis.waitUntil(3.75);
  tongue.set_value(false);
  chassis.waitUntilDone();
  intakeOuttake();
  pros::delay(1250);
  intakeLoad();

  // //wing
  chassis.moveToPoint(-12, 23, 800, {.forwards = false, .maxSpeed = 127});
  chassis.turnToHeading(270, 500, {.maxSpeed = 127, .minSpeed = 20});
  chassis.waitUntilDone();
  wing.set_value(false);
  chassis.moveToPoint(-26, 24, 800, {.forwards = true, .maxSpeed = 127, .minSpeed = 100});

}

void leftelims(){
  wing.set_value(true);
  hood.set_value(true);
  intakeLoad();
  chassis.setPose(0, 0, 0);

  // matchload 1
  chassis.moveToPoint(0, 28.25, 800, {.maxSpeed = 127});
  chassis.turnToHeading(-90, 600, {.maxSpeed = 127});
  tongue.set_value(true);
  chassis.moveToPoint(-16, 28, 1000, {.maxSpeed = 127});
  //chassis.moveToPose(-16, 28, 90, 1000, {.lead = 0.05, .maxSpeed = 127});
  chassis.waitUntilDone();
  pros::delay(100);
  // go to long goal
  chassis.moveToPoint(24, 30, 950, {.forwards = false, .maxSpeed = 115});
  chassis.waitUntilDone();
  intakeLongGoal();
  tongue.set_value(false);
  pros::delay(900);
  intakeLoad();

  //turn to mid
  chassis.turnToHeading(-185, 700);
  chassis.moveToPoint(23, 15, 900, {.maxSpeed = 115});
  chassis.waitUntil(7);
  tongue.set_value(true);
  chassis.turnToHeading(-47, 700);

  //go to low goal and score
  //chassis.moveToPoint(36.5, -10, 900, {.forwards = false, .maxSpeed = 127});
  chassis.moveToPose(39, -7, -47, 900, {.forwards = false, .lead = 0.2, .maxSpeed = 127});
  chassis.waitUntilDone();
  
  tongue.set_value(false);
  intakeCenterGoal();
  pros::delay(1250);
  intakeLoad();

  //wing
  chassis.moveToPoint(12, 16, 800, {.maxSpeed = 127});
  chassis.turnToHeading(-90, 500, {.maxSpeed = 127, .minSpeed = 20});
  chassis.waitUntilDone();
  wing.set_value(false);
  chassis.moveToPoint(28, 16, 800, {.forwards = false, .maxSpeed = 127, .minSpeed = 100});
}

void elims() {
  wing.set_value(true);
  hood.set_value(true);
  intakeLoad();
  chassis.setPose(0, 0, 0);
  // go to mid 3 and tongue
  chassis.moveToPose(7.55, 24.35, 40, 1150,{.lead = 0.23, .maxSpeed = 110, .minSpeed = 20, .earlyExitRange = 0.7});
  chassis.waitUntil(17.5);
  tongue.set_value(true);
  chassis.turnToHeading(127, 550, {.maxSpeed = 127});
  
  // go to matchload pos
  chassis.moveToPose(33, -8, 180, 900, {.lead = .3, .maxSpeed = 110, .minSpeed = 72, .earlyExitRange = 6});
  chassis.moveToPose(34, -28, 180, 900, {.lead = 0.12, .maxSpeed = 70});
  chassis.waitUntilDone();
  chassis.moveToPose(33, -32, 180, 300);
  pros::delay(400);
  // go to goal and score
  chassis.moveToPose(30, 22, 180, 1000, {.forwards = false, .lead = 0.1, .maxSpeed = 127});
  chassis.waitUntilDone();
  intakeOuttake();
  pros::delay(150);
  intakeLongGoal();
  pros::delay(1500);
  intakeLoad();
  tongue.set_value(false);
  // wing
  /*
  // goes to wing pos and turns
  intakeHalt();
  wing.set_value(false);
  chassis.moveToPose(45, 5, -175, 1050, {.maxSpeed = 127});
  // go into wing
  chassis.moveToPoint(45.25, 44, 1000, {.forwards = false, .maxSpeed = 127, .minSpeed = 20});
  chassis.waitUntilDone();
  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);*/
  
  chassis.moveToPoint(20, 0, 800, { .maxSpeed = 127});
  chassis.turnToHeading(0, 650, {.maxSpeed = 127, .minSpeed = 20});
  chassis.waitUntilDone();
  wing.set_value(false);
  chassis.moveToPoint(20, 28, 800, {.forwards = true, .maxSpeed = 127, .minSpeed = 100});
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
  /*
  // set up
  wing.set_value(true);
  hood.set_value(true);
  chassis.setPose(0, 0, -90);
  chassis.setBrakeMode(MOTOR_BRAKE_BRAKE);

  // go to first match loader
  chassis.moveToPoint(-26.5, 0, 800, {.maxSpeed = 127});
  chassis.turnToHeading(-180, 600, {.maxSpeed = 127});
  tongue.set_value(true);
  //intakeLoad();
  chassis.moveToPoint(-26.5, -12, 800, {.maxSpeed = 127});
  pros::delay(1000);

  // go to next load
  chassis.moveToPoint(-27, 0, 800, {.forwards = false, .maxSpeed = 127});
  chassis.turnToHeading(-225, 600, {.maxSpeed = 127});
  intakeHalt();
  tongue.set_value(false);
  chassis.moveToPoint(-41, 12.5, 1000, {.forwards = false, .maxSpeed = 127});
  chassis.turnToHeading(0, 800, {.maxSpeed = 127});
  wing.set_value(false);
  chassis.moveToPoint(-42, 82, 3000, {.maxSpeed = 127});
  chassis.turnToHeading(45, 600, {.maxSpeed = 127});
  chassis.moveToPoint(-29, 98, 1000, {.maxSpeed = 127});
  chassis.turnToHeading(0, 600, {.maxSpeed = 127});
  chassis.moveToPose(-29.5, 75, 0, 1000, {.forwards = false, .maxSpeed = 127});
  chassis.moveToPoint(-29.5, 70, 500, {.forwards = false, .maxSpeed = 127});
  //intakeLongGoal();
  pros::delay(2000);
  //intakeLoad();
  tongue.set_value(true);

  // get 2nd match loader
  chassis.setPose(0, 0, 0);
  chassis.moveToPose(1, 34, 0, 1500, {.maxSpeed = 127});
  pros::delay(2000);
  chassis.moveToPose(0, -13, 0, 1500, {.forwards = false, .maxSpeed = 127});
  chassis.moveToPoint(0, -15, 500, {.forwards = false, .maxSpeed = 127});
  chassis.setPose(0, 0, 0);
  //intakeLongGoal();
  pros::delay(2000);
  tongue.set_value(false);

*/
  // get match barrier

  /*
  chassis.setPose(0, 0, 0);
  intakeLoad();
  chassis.moveToPoint(0, 12, 1000, {.maxSpeed = 127});
  chassis.turnToHeading(90, 650, {.maxSpeed = 127});
  chassis.moveToPoint(49, 12, 1500, {.maxSpeed = 127});
  chassis.turnToHeading(0, 650, {.maxSpeed = 127});
  chassis.moveToPoint(47, 22, 1000, {.maxSpeed = 70});

  // cross
  //chassis.moveToPoint(47, 16, 1000, {.forwards = false, .minSpeed = 90}); // reset before going into barrier
  chassis.moveToPoint(47, 31, 2500, {.maxSpeed = 127, .minSpeed = 110}); // first movement into barrier
  chassis.turnToHeading(15, 200); // c drift
  chassis.turnToHeading(-15, 200);
  chassis.turnToHeading(0, 1500);
  chassis.waitUntilDone();
  //tongue.set_value(true);
  pros::delay(500);
  chassis.moveToPoint(47, 48, 2500, {.maxSpeed = 80}); // 2nd movement in barrier
  chassis.turnToHeading(25, 200); // c drift
  chassis.turnToHeading(-25, 200);
  chassis.turnToHeading(25, 200);
  chassis.turnToHeading(-25, 200);
  chassis.turnToHeading(0, 200);
  //chassis.moveToPoint(47, 45, 1000, {.maxSpeed = 70});
  //chassis.setPose(0, 0, 0);

  // reset after cross
  //chassis.moveToPoint(-1, -30, 2000, {.forwards = false, .maxSpeed = 90});
  //chassis.moveToPoint(-1, 0, 1000, {.maxSpeed = 60});
  //chassis.setPose(0, 0, 0);

  chassis.moveToPoint(47, 10, 2000, {.forwards = false, .maxSpeed = 110});
  chassis.moveToPoint(47, 22, 1500, {.maxSpeed = 70});*/


  // get park barrier 3102026
  chassis.setPose(0, 0, 0);
  tongue.set_value(false);
  intakeLoad();
  chassis.moveToPoint(0, 25, 800);
  chassis.turnToHeading(60, 600);
  chassis.moveToPose(90, 38, 90, 10000, {.minSpeed = 80});
  chassis.waitUntil(50);
  chassis.cancelMotion();
  chassis.moveToPose(105, 38, 90, 5000, {.maxSpeed = 80});
  chassis.waitUntil(10);
  tongue.set_value(true);
  chassis.waitUntilDone();
  pros::delay(500);
  tongue.set_value(false);
  pros::delay(500);
  chassis.setPose(0, 0, 0);
  chassis.moveToPoint(0, 15, 1000);
  
  // reset on barrier
  chassis.moveToPose(-3, -20, 0, 1500, {.forwards = false, .maxSpeed = 80});


  /*
  // score on center
  chassis.moveToPoint(47, -1, 1000, {.forwards = false, .maxSpeed = 127});
  chassis.turnToHeading(90, 650);
  chassis.moveToPoint(75, 0, 1000, {.maxSpeed = 127});
  chassis.turnToHeading(45, 650);
  hood.set_value(false);
  chassis.moveToPose(53.75, -20, 45, 1500, {.forwards = false, .lead = 0.2, .maxSpeed = 127});
  //intakeCenterGoal();
  //chassis.setPose(0, 0, 45);
  pros::delay(2000);

  // long goal reset
  chassis.moveToPoint(94, 15, 1500, {.maxSpeed = 127});
  chassis.turnToHeading(0, 650);
  hood.set_value(true);
  chassis.moveToPoint(95, -15, 1500, {.forwards = false, .maxSpeed = 90});
  chassis.waitUntilDone();
  

  chassis.setPose(0, 0, 0);
  intakeLoad();
  tongue.set_value(true);

  // get 3rd match loader
  chassis.moveToPoint(-0.5, 34, 1500, {.maxSpeed = 127});
  pros::delay(1500);
  chassis.moveToPoint(-0.5, 34, 1500, {.maxSpeed = 127});
  hood.set_value(true);
  chassis.moveToPoint(-1, 14, 1000, {.forwards = false, .maxSpeed = 127});
  chassis.turnToHeading(-45, 650);
  tongue.set_value(false);
  intakeHalt();
  chassis.moveToPoint(13, 1.5, 1000, {.forwards = false, .maxSpeed = 127});
  chassis.turnToHeading(180, 650);
  chassis.moveToPoint(13.5, -65, 1500, {.maxSpeed = 127});
  chassis.turnToHeading(225, 650);
  chassis.moveToPoint(0, -80, 1000, {.maxSpeed = 127});
  chassis.turnToHeading(180, 600);
  chassis.moveToPose(0, -50, 180, 1000, {.forwards = false, .maxSpeed = 127});
  chassis.waitUntilDone();
  intakeLongGoal();
  pros::delay(2000);
  intakeLoad();

  // get 4th match loader
  chassis.setPose(0, 0, 0);
  tongue.set_value(true);
  chassis.moveToPose(1, 34, 0, 1500, {.maxSpeed = 127});
  pros::delay(1500);
  chassis.moveToPose(1, 34, 0, 1500, {.maxSpeed = 127});
  chassis.moveToPose(0, -13, 0, 1500, {.forwards = false, .maxSpeed = 127});
  chassis.moveToPoint(0, -15, 500, {.forwards = false, .maxSpeed = 127});
  chassis.setPose(0, 0, 0);
  intakeLongGoal();
  pros::delay(2000);
  tongue.set_value(false);

  // park
  chassis.setPose(0, 0, 0);
  tongue.set_value(false);
  chassis.moveToPoint(0, 23, 800);
  chassis.turnToHeading(60, 600);
  chassis.moveToPose(32, 34, 90, 1200, {.minSpeed = 70});
  chassis.waitUntilDone();
  tongue.set_value(true);
  
  //cross
  intakeLoad();
  chassis.moveToPoint(70, 34, 3000, {.maxSpeed = 95});
  chassis.waitUntil(30);
  tongue.set_value(false);




  /*
  chassis.moveToPoint(93, 34, 1000, {.maxSpeed = 90}); 
  pros::delay(2000);

  // go to 2nd long goal
  chassis.moveToPoint(92, 16, 800, {.forwards = false, .maxSpeed = 127});
  chassis.turnToHeading(-45, 650);
  tongue.set_value(false);
  
  chassis.moveToPoint(104, 0, 1000, {.forwards = false, .maxSpeed = 127});
  chassis.turnToHeading(180, 650);
  chassis.moveToPoint(104, -60, 2000, {.maxSpeed = 127});
  chassis.turnToHeading(225, 650);
  chassis.moveToPoint(95, -76, 800, {.maxSpeed = 127});
  chassis.turnToHeading(180, 650);*/
  
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

/*
rd::Selector selector({{"Best auton", sawp},
                       {"Simple auton", elims},
                       {"Beat C", Rush},
                       {"Good auton", quals},
                       {"Skills", skills}});*/
void autonomous() {
  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
  skills();
  // selector.run_auton();
}

void opcontrol() {
  bool tongue_state = false;
  bool wing_state = false;
  bool hood_state = true;
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
