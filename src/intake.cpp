#include "main.h"
#include "pros/rtos.hpp"
#include "subsystems.hpp"

// intake motors
pros::Motor lowerIntake(-3);
pros::Motor upperIntake(-1);

// intake speeds
int lowerIntakeSpeed, upperIntakeSpeed;

// states
bool loading = false;
bool longGoalScoring = false;
bool centerGoalScoring = false;
bool outtaking = false;
bool halted = false;

// anti jam timer
u_int32_t lastCheckTime = pros::millis();
bool intakeJammed = false;

// set intake speed
void setIntake(int lowerSpeed, int upperSpeed) {
  lowerIntakeSpeed = lowerSpeed;
  upperIntakeSpeed = upperSpeed;

  // states
  loading = false;
  longGoalScoring = false;
  centerGoalScoring = false;
  outtaking = false;
  halted = false;

  lastCheckTime = pros::millis();
}

void intakeLoad() {
  // speed
  lowerIntakeSpeed = 127;
  upperIntakeSpeed = -127;

  // states
  if (!loading)
    lastCheckTime = pros::millis();
  loading = true;
  longGoalScoring = false;
  centerGoalScoring = false;
  outtaking = false;
  halted = false;
}

void intakeCenterGoal() {
  // speed
  lowerIntakeSpeed = 110;
  upperIntakeSpeed = 90;

  // states
  if (!centerGoalScoring)
    lastCheckTime = pros::millis();
  loading = false;
  longGoalScoring = false;
  centerGoalScoring = true;
  outtaking = false;
  halted = false;
}

void intakeLongGoal() {
  // speed
  lowerIntakeSpeed = 127;
  upperIntakeSpeed = 127;

  // states
  if (!longGoalScoring)
    lastCheckTime = pros::millis();
  loading = false;
  longGoalScoring = true;
  centerGoalScoring = false;
  outtaking = false;
  halted = false;
}

void intakeOuttake() {
  // speed
  lowerIntakeSpeed = -127;
  upperIntakeSpeed = -127;

  // states
  if (!outtaking)
    lastCheckTime = pros::millis();
  loading = false;
  longGoalScoring = false;
  centerGoalScoring = false;
  outtaking = true;
  halted = false;
}

void intakeHalt() {
  // speed
  lowerIntakeSpeed = 0;
  upperIntakeSpeed = 0;

  // states
  loading = false;
  longGoalScoring = false;
  centerGoalScoring = false;
  outtaking = false;
  halted = true;
}

// intake async control
void asyncIntakeControl(void *param) {
  printf("Intake Task Started\n");
  while (true) {
    // run intake
    lowerIntake.move(lowerIntakeSpeed);
    upperIntake.move(upperIntakeSpeed);

    if (centerGoalScoring) {
      hood.set_value(false);
    }
    if (longGoalScoring) {
      hood.set_value(true);
    }

    // anti jam
    if (pros::millis() - lastCheckTime > 750) {
      if (!halted) {
        bool jammed = false;
        // check if any motors are jammed while intaking
        if (longGoalScoring || centerGoalScoring || loading) {
          if (fabs(upperIntake.get_actual_velocity()) < 5 ||
              fabs(lowerIntake.get_actual_velocity()) < 5) {
            upperIntake.move(-127);
            lowerIntake.move(-127);
            jammed = true;
          }
        } else if (outtaking) {
          if (fabs(upperIntake.get_actual_velocity()) < 5 ||
              fabs(lowerIntake.get_actual_velocity()) < 5) {
            upperIntake.move(127);
            lowerIntake.move(127);
            jammed = true;
          }
        }

        if (jammed) {
          // delay to let the "pulse" happen
          pros::delay(300);
          // reset timer
          lastCheckTime = pros::millis();
        }
      }
    }

    // delay
    pros::delay(20);
  }
}
