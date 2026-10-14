# lemlib-ext: odometry and PID extensions for LemLib 0.5

This project links LemLib 0.5.5 as a precompiled archive (`firmware/LemLib.a`). Its headers in `include/lemlib/`
must match that archive exactly, so they are **not modified**. Changing a class layout such as `lemlib::PID` would
compile but break at runtime. The improvements live in a separate, opt-in module instead:

| File | Purpose | Depends on PROS? |
| --- | --- | --- |
| `include/lemlib-ext/odomMath.hpp`, `src/lemlib-ext/odomMath.cpp` | Pure odometry math | No |
| `include/lemlib-ext/odometry.hpp`, `src/lemlib-ext/odometry.cpp` | `lemlib::ext::Odometry`, a replacement odometry task | Yes |
| `include/lemlib-ext/pid.hpp`, `src/lemlib-ext/pid.cpp` | `lemlib::ext::PID`, a time-aware PID | No |
| `src/lemlib-ext/clock.cpp` | `pros::micros()` clock used by `PID::update(error)` | Yes |
| `include/lemlib-ext/api.hpp` | Includes all of the above | |
| `tests/lemlib_ext_tests.cpp` | Unit tests for the pure math, run on a PC | No |

If you don't use any of this, the robot behaves exactly as before: the new sources are compiled but nothing runs
until you construct and start these classes.

## What LemLib 0.5.5 does today

These notes come from the LemLib v0.5.5 source (`src/lemlib/chassis/odom.cpp`, `src/lemlib/pid.cpp`). The headers in
this repo are identical to that release.

**Odometry.** Every 10 ms LemLib reads the tracking wheels and the IMU and does the following:

- It picks the heading change from two horizontal wheels, then two unpowered vertical wheels, then the IMU, then the
  drive motors.
- It converts each wheel's travel into a chord using `2 sin(Δθ/2) · (Δ/Δθ + offset)`.
- It rotates the chord by the midpoint heading.

That math is already correct for arcs. The problems are in the edge cases:

1. **Sensor errors corrupt the pose permanently.** When the IMU is unplugged or recalibrating,
   `imu.get_rotation()` returns `PROS_ERR_F` (infinity). LemLib adds that to the heading and the pose becomes `NaN`
   for the rest of the match. A sensor that reconnects and restarts from 0 causes one huge jump.
2. **Only one drive side is used.** With no tracking wheels (this robot's setup), forward travel comes from the left
   motor group only. The right side's encoders are ignored.
3. **Fixed 10 ms assumption.** Speeds are computed as `Δ / 0.01` whatever the real loop time was.
4. **No IMU scale correction.** Small gyro scale errors (commonly 0.5–2 %) add up over every turn.

**PID.** `lemlib::PID::update(error)` computes `integral += error` and `derivative = error − prevError`, so:

1. It assumes every update is exactly one loop apart. A late update makes the derivative too large and adds too
   little to the integral.
2. **Derivative kick.** After `reset()`, `prevError = 0`, so the first derivative is the whole error. That adds
   `kD · error` to the first output. With the angular `kD = 20` in `src/main.cpp`, a 3° correction gets an extra
   60 on its first cycle.
3. There is no output limit inside the controller and no anti-windup beyond `windupRange`/`signFlipReset`.
4. A `NaN` error makes the integral `NaN` until the next `reset()`.

## Odometry improvements (`lemlib::ext::Odometry`)

### Conventions

These are the same as LemLib, so the pose can be shared with `chassis`:

- Heading θ is a compass heading in radians: 0 faces +Y, and clockwise is positive.
- Local frame: y is forwards and x is to the left. The left-positive x is the sign LemLib's horizontal-wheel math
  uses.
- Wheel offsets are measured from the tracking center. Vertical wheels: left is negative, right is positive.
  Horizontal wheels: behind is negative, in front is positive.

### Equations

For one update, let Δθ be the heading change and Δᵢ the change in wheel *i*'s distance.

**Rotation compensation.** A wheel at offset dᵢ rides on an arc whose radius differs from the center's by dᵢ.
Subtract the part caused purely by rotation:

$$s_i = \Delta_i + d_i \, \Delta\theta$$

For a turn in place, a wheel 5" right of center reads −5Δθ, and *s* = 0.

**Combining wheels.** Forward travel *s_y* is the mean of *sᵢ* over all valid unpowered vertical wheels. If there are
none, it falls back to all valid drive-motor "wheels". With drive motors only, both sides are averaged, which cuts
independent encoder noise by about √2. Sideways travel *s_x* is the mean over horizontal wheels.

**Arc → chord.** If the robot moves on a constant-curvature arc, the straight-line displacement is the arc length times

$$k(\Delta\theta) = \frac{2\sin(\Delta\theta/2)}{\Delta\theta}, \qquad k \approx 1 - \frac{\Delta\theta^2}{24} \text{ for } |\Delta\theta| < 10^{-3}$$

Using the Taylor series near zero removes the division by zero and the `Δθ == 0` special case. Multiplied out,
`k·(Δ + dΔθ)` is the same as LemLib's `2 sin(Δθ/2)(Δ/Δθ + d)`.

**Midpoint rotation into the field frame.** The chord of an arc points along the average of the start and end
headings, θₘ = θ₀ + Δθ/2:

$$\Delta x = l_y \sin\theta_m - l_x \cos\theta_m, \qquad \Delta y = l_y \cos\theta_m + l_x \sin\theta_m$$

with l = k · s. Using the chord and the midpoint heading together is exact for constant-curvature motion. Simple Euler
integration (arc length along the starting heading) is not. The tests drive a 24" radius circle in 30° steps: the
chord method stays within 1e-4" the whole way round, while Euler is off by more than 5".

**Heading.** The heading source follows LemLib's priority order but skips any source that is missing or rejected in
that update:

$$\Delta\theta_{\text{pair}} = -\frac{\Delta_1 - \Delta_2}{d_1 - d_2}, \qquad \Delta\theta_{\text{IMU}} = \text{wrap}(r_t - r_{t-1}) \cdot \text{imuScale}$$

`wrap()` maps into (−π, π], so a heading that crosses ±180° or 0/360° gives the small true change, not a ±360° jump.
The stored pose heading stays continuous (unwrapped), exactly like LemLib, because LemLib's motions rely on that.

**Rejecting bad readings.** A reading is ignored for one update if it is not finite, or if it moved faster than
physically possible (`|Δ| > maxRate · dt`, using the real dt measured with `pros::micros()`). The reference point
always moves to the newest finite reading, so a sensor that reconnects at 0 causes one skipped update, not a jump.
If the IMU is rejected, the heading comes from the drive encoders for that update. If one drive side is rejected,
the other side still measures forward travel.

### Usage

```cpp
#include "lemlib-ext/api.hpp"

// same sensors and drivetrain objects that are passed to lemlib::Chassis
lemlib::ext::Odometry odometry(sensors, drivetrain);

void initialize() {
    chassis.calibrate(); // first: calibrates the IMU and starts LemLib's own odometry task
    odometry.start();    // suspends LemLib's odometry task and runs this one instead
}
```

`chassis.setPose()`, `chassis.getPose()` and all chassis motions work unchanged, because the new task reads and
writes the pose through `lemlib::getPose()`/`lemlib::setPose()`. `odometry.stop()` hands control back to LemLib's
task without double-counting the motion in between.

`OdometryConfig` options (all optional):

| Field | Default | Meaning |
| --- | --- | --- |
| `imuScale` | `1` | Multiplier on every IMU heading change. Measure it by turning the robot exactly N full turns: `imuScale = 360·N / reported degrees`. Average several runs in both directions. `1` = no correction. |
| `maxAngularVelocity` | `1500` deg/s | Faster IMU changes are rejected. The V5 IMU saturates at 1000 deg/s. `0` disables the check. |
| `maxLinearVelocity` | `200` in/s | Faster tracking-wheel changes are rejected. `0` disables the check. |
| `periodMs` | `10` | Update period. The loop uses `delay_until`, so the period stays fixed even if an update runs long. |

`odometry.getHeadingSource()` and `odometry.getRejectedReadings()` help confirm on the brain screen that the IMU is
being used and that no sensor is dropping out.

**Compatibility notes.**

- The task suspends LemLib's internal global `trackingTask` (defined in `odom.cpp` inside `LemLib.a`). This depends
  on LemLib 0.5.x internals. Re-check it if LemLib is upgraded.
- While the new task is running, `lemlib::getSpeed()`, `getLocalSpeed()` and `estimatePose()` are not updated. None
  of LemLib's motions use them.
- With healthy sensors and `imuScale = 1`, the only difference from LemLib is which vertical wheels are averaged. On
  this robot that means both drive sides instead of only the left.

## PID improvements (`lemlib::ext::PID`)

It has the same constructor, `update(error)` and `reset()` as `lemlib::PID`, so swapping the type is enough to use it
in your own loops (lift, arm, custom drive functions). `lemlib::Chassis` builds its own `lemlib::PID` objects inside
the precompiled library, so chassis motions keep using LemLib's PID.

### Equations

With dt the measured time since the last update and T the nominal period (10 ms by default), let p = dt / T:

$$I \mathrel{+}= e \cdot p, \qquad D_{\text{raw}} = \frac{e - e_{\text{prev}}}{p}, \qquad u = k_P e + k_I I + k_D D$$

The gains keep LemLib's per-loop units, so **existing gains need no retuning**. When updates really are 10 ms apart,
p = 1 and the output equals `lemlib::PID`'s. The unit tests check this against a copy of LemLib's code. The one
exception is the first update, where the derivative kick is removed.

- **No derivative kick.** D = 0 on the first update after construction or `reset()`.
- **Derivative filter** (optional, `setDerivativeFilter(tau)`). A first-order low-pass:
  D ← D + α(D_raw − D) with α = dt / (τ + dt). Because α depends on dt, the filter's time constant doesn't change
  with loop rate. Start with τ = 2–5 loop periods.
- **Output limit** (optional, `setOutputLimit(limit)`). Clamps u to ±limit. While the unclamped output is past the
  limit and the error pushes it further, the integral is not allowed to grow (conditional-integration anti-windup).
  It may still shrink.
- **Integral limit** (optional, `setIntegralLimit(limit)`). Keeps |k_I · I| ≤ limit, in output units.
- `windupRange` and `signFlipReset` behave as in LemLib. One difference: the sign-flip check is skipped on the first
  update, because there is no previous error yet. LemLib compares against 0, so a negative first error discarded
  its own integral.
- **Bad inputs.** A non-finite error returns 0 and leaves the controller untouched. If dt is zero, negative or
  non-finite, nothing is integrated or differentiated. `update(error)` doesn't advance its clock when no time has
  passed.
- `setNominalPeriod(seconds)`: use this if your loop gains were tuned at a period other than 10 ms.

```cpp
#include "lemlib-ext/pid.hpp"

lemlib::ext::PID liftPID(2, 0.05, 10); // same arguments as lemlib::PID
liftPID.setOutputLimit(127).setDerivativeFilter(0.03);

while (true) {
    lift.move(liftPID.update(target - lift.get_position()));
    pros::delay(10);
}
```

**Settling.** LemLib's `ExitCondition` already requires the error to stay inside a range for a set time, which
handles passing quickly through the target. Chassis motions can't use a different exit condition, so no new one was
added. `PID::getDerivative()` is exposed for custom loops that want to require low error *and* low error rate.

## Running the tests

The tests cover only the PROS-free code (`odomMath.cpp`, `pid.cpp`), with a fake clock. Any C++20 compiler works
(g++, clang++, or `zig c++`). From the repository root:

```bash
mkdir -p bin && g++ -std=c++20 -Wall -Wextra -Iinclude tests/lemlib_ext_tests.cpp src/lemlib-ext/odomMath.cpp src/lemlib-ext/pid.cpp -o bin/lemlib_ext_tests && ./bin/lemlib_ext_tests
```

The output ends with `N checks, 0 failures`, and the exit code is non-zero if anything fails. `bin/` is already
git-ignored. The PROS build only compiles `src/`, so `tests/` never ends up on the robot.

What they cover:

- angle wrapping and heading deltas across ±180° and 0/360°
- chord ratio continuity at zero
- tracking-wheel offset compensation for pure rotation and for combined translation and rotation, compared with
  closed-form circle geometry
- coordinate transforms at several headings
- exact closure of a full circle, with Euler integration for comparison
- agreement with LemLib 0.5.5's odometry equations
- heading-source priority and fallback when the IMU or a wheel fails
- drive-side averaging
- zero and tiny movements
- PID equivalence with LemLib at 10 ms
- PID behavior under constant error, changing error, irregular dt, zero/negative/NaN dt, NaN error, saturation and
  anti-windup, integral limit, derivative filter, and measured time

## What has and hasn't been verified

- **Verified:** the unit tests above pass. All new sources compile without warnings (`-Wall -Wextra`, C++20) against
  this repo's PROS and LemLib headers, for both a PC target and ARM Cortex-A9. The symbols the new code uses from
  `LemLib.a` (`trackingTask`, `lemlib::update/getPose/setPose`) exist in the archive.
- **Not verified:** a full `pros make` link (the PROS toolchain wasn't available) and any testing on the robot. No
  claim is made that field accuracy improved. To measure it, drive a repeatable path (e.g. a square back to a marked
  start) with LemLib's odometry and then with `lemlib::ext::Odometry`, and compare the final pose error over several
  runs. Measure `imuScale` before enabling it.
