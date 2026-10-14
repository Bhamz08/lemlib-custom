# VEX Robot Code + lemlib-ext

Competition code for VEX V5RC team **293Y** (Great Valley High School, 2025–26 season), written in C++ with
[PROS](https://pros.cs.purdue.edu/) and [LemLib](https://github.com/LemLib/LemLib) 0.5.5. It also includes
**lemlib-ext**, an opt-in module I wrote that adds fault-tolerant odometry and a time-aware PID controller on top of
LemLib.

The robot code was written by our team. The lemlib-ext module and its tests are my own work.

## What's in here

### Robot code (`src/main.cpp`, `src/intake.cpp`)

- **Drivetrain:** a 6-motor drive with an IMU, configured through LemLib's chassis, PID controllers, and exponential
  drive curves for arcade control.
- **Autonomous routines:** solo AWP, qualification and elimination routines for both sides, a rush auton, and a full
  skills run. A [robodash](https://github.com/unwieldycat/robodash) brain-screen selector for them is included but
  currently commented out, and the code runs the skills routine directly.
- **Intake state machine:** separate modes for loading, long-goal scoring, center-goal scoring, and outtaking, with
  anti-jam detection. If either intake motor stalls while scoring or outtaking, the code reverses both motors in a
  short pulse to clear the jam.
- **Motor temperature monitoring:** during driver control, intake and drive temperatures show on the controller
  screen, and the controller vibrates when the intake or the drive average passes 45 °C.

### lemlib-ext (`include/lemlib-ext/`, `src/lemlib-ext/`)

LemLib 0.5.5 ships as a precompiled archive, so its headers can't be changed safely. lemlib-ext lives alongside it
instead. Nothing runs unless you construct and start these classes, so the robot behaves exactly as before by default.

- **`lemlib::ext::Odometry`** replaces LemLib's tracking task:
  - Rejects non-finite or physically impossible sensor readings, so an unplugged or recalibrating IMU can't turn
    the pose into `NaN` for the rest of a match.
  - Wraps heading changes across ±180° and averages both drive sides, where LemLib used only the left side.
  - Adds an optional IMU scale correction.
  - Reads and writes the pose through `lemlib::getPose`/`setPose`, so every chassis motion works unchanged.
- **`lemlib::ext::PID`** is a drop-in replacement for `lemlib::PID`:
  - Uses the measured time between updates and removes derivative kick.
  - Adds an optional output limit with anti-windup, an integral limit, and a derivative filter.
  - Handles bad inputs (NaN error, zero or negative dt).
  - Gives the same output as LemLib's PID when updates are 10 ms apart, so existing gains need no retuning.

Full equations, design notes, and usage are in [`docs/lemlib-ext.md`](docs/lemlib-ext.md).

## Tests

The math in lemlib-ext doesn't depend on PROS, so it has host unit tests (`tests/lemlib_ext_tests.cpp`). Among other
things, they check:

- heading wrapping
- tracking-wheel compensation against closed-form circle geometry
- full-circle closure (the chord method stays within 1e-4" on a 24" circle, where simple Euler integration drifts
  more than 5")
- sensor-failure fallback
- matching LemLib's PID output at 10 ms
- saturation and anti-windup

Run them from the repo root with any C++20 compiler:

```bash
mkdir -p bin && g++ -std=c++20 -Wall -Wextra -Iinclude tests/lemlib_ext_tests.cpp \
  src/lemlib-ext/odomMath.cpp src/lemlib-ext/pid.cpp -o bin/lemlib_ext_tests && ./bin/lemlib_ext_tests
```

## Status

- **Done:** the unit tests pass, and the new sources compile cleanly (`-Wall -Wextra`, C++20) for both PC and the
  V5's ARM Cortex-A9.
- **Not done yet:** a full `pros make` build and testing on a robot. Field accuracy hasn't been measured yet. The plan
  is to drive a repeatable path with LemLib's odometry and with lemlib-ext, then compare the final pose error over
  several runs.

## Building for the robot

Install the [PROS CLI](https://pros.cs.purdue.edu/v5/getting-started/index.html), then from the repo root:

```bash
pros make
pros upload
```
