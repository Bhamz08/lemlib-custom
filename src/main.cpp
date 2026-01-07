#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/trackingWheel.hpp"
#include "lemlib-tarball/api.hpp"
#include "pros/misc.h"
#include "robodash/api.h"


// controller
pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::Motor lowerintake(3);
pros::Motor upperintake(1);
pros::MotorGroup intake({-3, 1});
pros::adi::DigitalOut tongue ('G');
pros::adi::DigitalOut wing ('F');
pros::adi::DigitalOut hood ('H');
// motor groups
pros::MotorGroup leftMotors({-12, -10, 9},
                            pros::MotorGearset::blue); // left motor group - ports 3 (reversed), 4, 5 (reversed)
pros::MotorGroup rightMotors({15, 8, -18}, pros::MotorGearset::blue); // right motor group - ports 6, 7, 9 (reversed)

// Inertial Sensor on port 10
pros::Imu imu(17);

// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed
pros::Rotation horizontalEnc(20);
// vertical tracking wheel encoder. Rotation sensor, port 11, reversed
pros::Rotation verticalEnc(-11);
// horizontal tracking wheel. 2.75" diameter, 5.75" offset, back of the robot (negative)
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_275, -5.75);
// vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_275, -2.5);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              10, // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 4" omnis
                              450, // drivetrain rpm is 360
                              2 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings linearController(7.6, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            24, // derivative gain (kD)
                                            3, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            20 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(3.16, // proportional gain (kP)
                                             0.1, // integral gain (kI)
                                             20, // derivative gain (kD)
                                             3, // anti windup
                                             1, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

// sensors for odometry
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            nullptr, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);


/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    //pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
	imu.reset(); // reset imu to 0 degrees

    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs

    // thread to for brain screen and position logging
    // pros::Task screenTask([&]() {
    //     while (true) {
    //         // print robot location to the brain screen
    //         pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
    //         pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
    //         pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
    //         // log position telemetry
    //         lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
    //         // delay to save resources
    //         pros::delay(50);
    //     }
    // });
}

/**
 * Runs while the robot is disabled
 */
void disabled() {}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {}

// get a path used for pure pursuit
// this needs to be put outside a function
ASSET(Leftquals_txt); // '.' replaced with "_" to make c++ happy
ASSET(Leftelims_txt);
ASSET(sawp_txt);
// Create the decoder
lemlib_tarball::Decoder leftqualsdecode(Leftquals_txt);
lemlib_tarball::Decoder leftelimsdecode(Leftelims_txt);
lemlib_tarball::Decoder sawpdecode(sawp_txt);
//add staring pos
void leftquals(){
chassis.setPose(-46.665, 14.467, 90.1);        
	upperintake.move(127);
            lowerintake.move(-127);
	chassis.follow(leftqualsdecode["Drive to the 3 blocks"], 15, 2000);
	chassis.waitUntil(20);
	tongue.set_value(true);
	chassis.waitUntilDone();
	chassis.turnToHeading(137, 2000);
	chassis.follow(leftqualsdecode["move to long goal"], 15, 2000, false);
	chassis.waitUntilDone();	
		        upperintake.move(-80);
			pros::delay(500);
			upperintake.move(127);
	chassis.turnToHeading(300, 2000);
	chassis.follow(leftqualsdecode["matchload"], 15, 2000, true);
	chassis.waitUntilDone();
	pros::delay(1000);
	chassis.turnToHeading(270, 2000);
	chassis.follow(leftqualsdecode["score long goal"], 15, 2000, false);
	chassis.waitUntilDone();
	lowerintake.move(127);
	pros::delay(250);
	lowerintake.move(-127);
	upperintake.move(-127);
	pros::delay(1000);
	chassis.turnToHeading(209.4, 2000);
	chassis.follow(leftqualsdecode["Descore"], 15, 2000);
}

void LeftElims(){
chassis.setPose(-46.714, 13.681, 89.8);
	     upperintake.move(127);
         lowerintake.move(-127);
chassis.follow(leftelimsdecode["3 Balls"], 15, 2000);
	chassis.waitUntil(20);
	tongue.set_value(true);
	chassis.waitUntilDone();
	chassis.turnToHeading(315.4, 2000);
	chassis.follow(leftelimsdecode["Matchload"], 15, 2000);
	chassis.waitUntilDone();
	pros::delay(1000);
	chassis.turnToHeading(270, 2000);
	chassis.follow(leftelimsdecode["Long goal scoring"], 15, 2000);
	chassis.waitUntilDone();
	lowerintake.move(127);
	pros::delay(250);
	lowerintake.move(-127);
	upperintake.move(-127);
	pros::delay(1000);
	chassis.turnToHeading(169.8, 2000);
	chassis.follow(leftelimsdecode["Descore"], 15, 2000);
}
void sawp(){
chassis.setPose(-49.026, -16.442, 179.2);
	upperintake.move(127);
	lowerintake.move(-127);
	tongue.set_value(true);
chassis.follow(sawpdecode["matchload"], 15, 2000);
chassis.waitUntilDone();
pros::delay(500);
chassis.turnToHeading(270, 2000);
chassis.follow(sawpdecode["scoring 1st goal"], 15, 2000, false);
chassis.waitUntilDone();
lowerintake.move(127);
pros::delay(250);
lowerintake.move(-127);
upperintake.move(-127);
pros::delay(1000);
tongue.set_value(false);
lowerintake.move(-127);
upperintake.move(127);
chassis.turnToHeading(268.3, 2000);
chassis.follow(sawpdecode["3 ball 1st"], 15, 2000);
chassis.waitUntilDone();
tongue.set_value(true);
chassis.turnToHeading(32.1, 2000);
chassis.follow(sawpdecode["3 ball 2nd"], 15, 2000);
chassis.waitUntil(5);
tongue.set_value(false);
chassis.waitUntilDone();
tongue.set_value(true);
chassis.turnToHeading(322.6, 2000);
chassis.follow(sawpdecode["Scoring Mid Goal"], 15, 2000, false);
chassis.waitUntilDone();
upperintake.move(-80);
pros::delay(250);
upperintake.move(127);
chassis.turnToHeading(318, 2000);
chassis.follow(sawpdecode["matchload 2"], 15, 2000);
chassis.waitUntilDone();
	pros::delay(250);
chassis.turnToHeading(270, 2000);
chassis.follow(sawpdecode["long goal"], 15, 2000, false);
chassis.waitUntilDone();
lowerintake.move(127);
pros::delay(250);
lowerintake.move(-127);
upperintake.move(-127);
pros::delay(1000);
}

/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */
	rd::Selector selector({
    {"Left Quals", leftquals},
     {"Left Eliminations", LeftElims},
	{"SAWP", sawp},
    // {"Good auton", good_auton},
});

 void autonomous() {
   selector.run_auton();
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
        }
        else if (master.get_digital(DIGITAL_R2)) {
            hood.set_value(false);
            upperintake.move(-127);
            lowerintake.move(-127);
        }
        else if (master.get_digital(DIGITAL_L1)) {
            upperintake.move(127);
            lowerintake.move(-127);
        }
        else if (master.get_digital(DIGITAL_L2)) {
            upperintake.move(127);
            lowerintake.move(127);
        }
        else {
            upperintake.brake();
            lowerintake.brake();
        }

        if ((master.get_digital(pros::E_CONTROLLER_DIGITAL_B))) {
				tongue_state = !tongue_state;
				tongue.set_value(tongue_state);
		}
		        if ((master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN))) {
				wing_state = !wing_state;
				wing.set_value(wing_state);
		}
		        if ((master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT))) {
				hood_state = !hood_state;
				hood.set_value(hood_state);
		}		



        // // //move hood manually
        // hood.button_toggle(master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT));



		// delay to save resources
        pros::delay(10);
    }
}