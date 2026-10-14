// Host-side unit tests for the pure math in src/lemlib-ext (odomMath.cpp and pid.cpp).
// These run on a PC, not the V5 brain. See docs/lemlib-ext.md for how to run them.

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <initializer_list>
#include "lemlib-ext/odomMath.hpp"
#include "lemlib-ext/pid.hpp"

using namespace lemlib::ext;

// fake clock for PID::update(error). Replaces src/lemlib-ext/clock.cpp, which needs PROS
static std::uint64_t fakeMicros = 0;

std::uint64_t lemlib::ext::microsNow() { return fakeMicros; }

static int failures = 0;
static int checks = 0;

#define CHECK(cond)                                                                                                    \
    do {                                                                                                               \
        checks++;                                                                                                      \
        if (!(cond)) {                                                                                                 \
            failures++;                                                                                                \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);                                                \
        }                                                                                                              \
    } while (0)

#define CHECK_NEAR(a, b, tol)                                                                                          \
    do {                                                                                                               \
        checks++;                                                                                                      \
        const double va = (a), vb = (b);                                                                               \
        if (!(std::fabs(va - vb) <= (tol))) {                                                                          \
            failures++;                                                                                                \
            std::printf("FAIL %s:%d: %s = %.9g, expected %s = %.9g (tol %g)\n", __FILE__, __LINE__, #a, va, #b, vb,    \
                        double(tol));                                                                                  \
        }                                                                                                              \
    } while (0)

constexpr double PI = 3.14159265358979323846;

static float deg(float d) { return float(d * PI / 180); }

// ---------------------------------------------------------------------------------------------------------------------
// Reference implementations of the original LemLib 0.5.5 code (src/lemlib/pid.cpp and the translation part of
// src/lemlib/chassis/odom.cpp, LemLib, MIT license), used to check that the extensions stay compatible
// ---------------------------------------------------------------------------------------------------------------------

struct LemLibPID {
        float kP, kI, kD, windupRange;
        bool signFlipReset;
        float integral = 0;
        float prevError = 0;

        float update(float error) {
            integral += error;
            if ((error < 0 ? -1 : 1) != (prevError < 0 ? -1 : 1) && signFlipReset) integral = 0;
            if (std::fabs(error) > windupRange && windupRange != 0) integral = 0;
            const float derivative = error - prevError;
            prevError = error;
            return error * kP + integral * kI + derivative * kD;
        }
};

// LemLib 0.5.5 odometry translation for one vertical and one horizontal wheel
static Vec2 lemlibOdomStep(float prevTheta, float deltaTheta, float deltaY, float verticalOffset, float deltaX,
                           float horizontalOffset) {
    float localX, localY;
    if (deltaTheta == 0) {
        localX = deltaX;
        localY = deltaY;
    } else {
        localX = 2 * std::sin(deltaTheta / 2) * (deltaX / deltaTheta + horizontalOffset);
        localY = 2 * std::sin(deltaTheta / 2) * (deltaY / deltaTheta + verticalOffset);
    }
    const float avgHeading = prevTheta + deltaTheta / 2;
    Vec2 out;
    out.x = localY * std::sin(avgHeading) + localX * -std::cos(avgHeading);
    out.y = localY * std::cos(avgHeading) + localX * std::sin(avgHeading);
    return out;
}

// ---------------------------------------------------------------------------------------------------------------------
// Angles
// ---------------------------------------------------------------------------------------------------------------------

static void testWrapAngle() {
    CHECK_NEAR(wrapAngle(float(3 * PI / 2)), -PI / 2, 1e-6);
    CHECK_NEAR(wrapAngle(float(-3 * PI / 2)), PI / 2, 1e-6);
    CHECK_NEAR(wrapAngle(float(PI)), PI, 1e-6);
    CHECK_NEAR(wrapAngle(float(-PI)), PI, 1e-6); // -pi maps to +pi: range is (-pi, pi]
    CHECK_NEAR(wrapAngle(float(7 * PI)), PI, 1e-5);
    CHECK_NEAR(wrapAngle(0), 0, 0);
    // a heading accumulated over many turns still wraps correctly
    CHECK_NEAR(wrapAngle(float(100 * 2 * PI + 0.25)), 0.25, 1e-4);
    CHECK(std::isnan(wrapAngle(NAN)));

    CHECK_NEAR(wrapAngleDeg(190), -170, 1e-4);
    CHECK_NEAR(wrapAngleDeg(-190), 170, 1e-4);
    CHECK_NEAR(wrapAngleDeg(-180), 180, 1e-4);
    CHECK_NEAR(wrapAngleDeg(360), 0, 1e-4);
    CHECK_NEAR(wrapAngleDeg(-540), 180, 1e-4);
    CHECK_NEAR(wrapAngleDeg(725), 5, 1e-4);
}

static void testHeadingDelta() {
    // crossing the -180/180 boundary
    CHECK_NEAR(headingDelta(deg(-179), deg(179)), deg(2), 1e-6);
    CHECK_NEAR(headingDelta(deg(179), deg(-179)), deg(-2), 1e-6);
    // crossing the 0/360 boundary
    CHECK_NEAR(headingDelta(deg(1), deg(359)), deg(2), 1e-6);
    CHECK_NEAR(headingDelta(deg(359), deg(1)), deg(-2), 1e-6);
    // continuous (unwrapped) readings behave like a plain difference for small changes
    CHECK_NEAR(headingDelta(deg(725), deg(720)), deg(5), 1e-5);
    CHECK_NEAR(headingDelta(deg(10), deg(10)), 0, 0);
}

// ---------------------------------------------------------------------------------------------------------------------
// Odometry building blocks
// ---------------------------------------------------------------------------------------------------------------------

static void testChordRatio() {
    CHECK_NEAR(chordRatio(0), 1, 0);
    CHECK_NEAR(chordRatio(float(PI)), 2 / PI, 1e-6); // half circle: chord = diameter, arc = pi * r
    CHECK_NEAR(chordRatio(float(PI / 2)), 2 * std::sin(PI / 4) / (PI / 2), 1e-6);
    // continuous across the Taylor-series switch at 1e-3
    CHECK_NEAR(chordRatio(0.000999f), chordRatio(0.001001f), 1e-7);
    // even function
    CHECK_NEAR(chordRatio(0.3f), chordRatio(-0.3f), 0);
    // tiny but non-zero headings don't blow up
    CHECK_NEAR(chordRatio(1e-20f), 1, 0);
}

static void testCenterArcAndWheelHeading() {
    const float dTheta = deg(10);
    // turn in place, clockwise: a wheel 5" right of center moves backwards 5 * dTheta and the center doesn't move
    CHECK_NEAR(centerArc(-5 * dTheta, 5, dTheta), 0, 1e-7);
    CHECK_NEAR(centerArc(5 * dTheta, -5, dTheta), 0, 1e-7);
    // horizontal wheel 3" behind center (offset -3). LemLib's sideways sign is left-positive, so during a clockwise
    // turn the wheel reads +3 * dTheta, and the center doesn't move sideways
    CHECK_NEAR(centerArc(3 * dTheta, -3, dTheta), 0, 1e-7);
    // straight driving: the offset doesn't matter
    CHECK_NEAR(centerArc(12, 5, 0), 12, 0);

    // heading from two wheels: left at -5, right at +5, turning clockwise in place
    CHECK_NEAR(wheelHeadingDelta(5 * dTheta, -5, -5 * dTheta, 5), dTheta, 1e-7);
    // asymmetric offsets: left wheel at -2, right at +6, arc of radius 20 (center) turning clockwise by dTheta
    const float r = 20;
    CHECK_NEAR(wheelHeadingDelta((r + 2) * dTheta, -2, (r - 6) * dTheta, 6), dTheta, 1e-6);
    // same offset can't measure heading
    CHECK(std::isnan(wheelHeadingDelta(1, 2, 3, 2)));
}

static void testCoordinateTransforms() {
    // 10" straight forward at several headings
    Vec2 g = localToGlobal(localDisplacement(10, 0, 0), 0);
    CHECK_NEAR(g.x, 0, 1e-6);
    CHECK_NEAR(g.y, 10, 1e-6);
    g = localToGlobal(localDisplacement(10, 0, 0), deg(90));
    CHECK_NEAR(g.x, 10, 1e-5);
    CHECK_NEAR(g.y, 0, 1e-5);
    g = localToGlobal(localDisplacement(10, 0, 0), deg(45));
    CHECK_NEAR(g.x, 10 / std::sqrt(2.0), 1e-5);
    CHECK_NEAR(g.y, 10 / std::sqrt(2.0), 1e-5);
    g = localToGlobal(localDisplacement(10, 0, 0), deg(-135));
    CHECK_NEAR(g.x, -10 / std::sqrt(2.0), 1e-5);
    CHECK_NEAR(g.y, -10 / std::sqrt(2.0), 1e-5);
    // 4" sideways to the left while facing +Y goes to -X; facing +X, left is +Y
    g = localToGlobal(localDisplacement(0, 4, 0), 0);
    CHECK_NEAR(g.x, -4, 1e-6);
    CHECK_NEAR(g.y, 0, 1e-6);
    g = localToGlobal(localDisplacement(0, 4, 0), deg(90));
    CHECK_NEAR(g.x, 0, 1e-5);
    CHECK_NEAR(g.y, 4, 1e-5);
}

static void testPlausibleDelta() {
    CHECK(plausibleDelta(1, 200, 0.01)); // 100 in/s
    CHECK(!plausibleDelta(3, 200, 0.01)); // 300 in/s
    CHECK(!plausibleDelta(INFINITY, 200, 0.01));
    CHECK(!plausibleDelta(NAN, 0, 0.01));
    CHECK(plausibleDelta(1000, 0, 0.01)); // rate check disabled
    CHECK(plausibleDelta(1000, 200, 0)); // no dt: only the finite check applies
}

// ---------------------------------------------------------------------------------------------------------------------
// Full odometry steps
// ---------------------------------------------------------------------------------------------------------------------

static OdomStepInput drivetrainInput(float leftDelta, float rightDelta, float trackWidth) {
    OdomStepInput in;
    in.vertical[0] = {leftDelta, -trackWidth / 2, true, true};
    in.vertical[1] = {rightDelta, trackWidth / 2, true, true};
    return in;
}

// Drive the tracking center around a circle of radius R, clockwise, using IMU heading and drivetrain encoders.
// Exact position after turning phi from (0, 0) facing +Y: (R - R cos phi, R sin phi)
static void testArcIsExact() {
    const float R = 24, trackWidth = 10;
    const int steps = 12;
    const float dTheta = deg(30); // deliberately coarse steps: the chord method is exact for arcs anyway
    double x = 0, y = 0, theta = 0;
    double eulerX = 0, eulerY = 0, eulerTheta = 0, eulerMaxError = 0;
    for (int i = 0; i < steps; i++) {
        OdomStepInput in = drivetrainInput((R + trackWidth / 2) * dTheta, (R - trackWidth / 2) * dTheta, trackWidth);
        in.imuDelta = dTheta;
        in.imuValid = true;
        in.prevHeading = float(theta);
        const OdomStepResult step = computeOdomStep(in);
        CHECK(step.headingSource == HeadingSource::IMU);
        CHECK(step.forwardValid);
        x += step.globalDelta.x;
        y += step.globalDelta.y;
        theta += step.deltaTheta;
        const double phi = (i + 1) * double(dTheta);
        CHECK_NEAR(x, R - R * std::cos(phi), 1e-4);
        CHECK_NEAR(y, R * std::sin(phi), 1e-4);

        // naive Euler integration (arc length along the starting heading), for comparison
        const double arc = R * dTheta;
        eulerX += arc * std::sin(eulerTheta);
        eulerY += arc * std::cos(eulerTheta);
        eulerTheta += dTheta;
        eulerMaxError = std::fmax(eulerMaxError, std::hypot(eulerX - (R - R * std::cos(phi)), eulerY - R * std::sin(phi)));
    }
    // a full circle returns to the start
    CHECK_NEAR(x, 0, 1e-4);
    CHECK_NEAR(y, 0, 1e-4);
    CHECK_NEAR(theta, 2 * PI, 1e-5);
    // Euler is off by inches part way round with 30 degree steps, which is what the chord + midpoint method avoids
    CHECK(eulerMaxError > 5);
}

// offset tracking wheels on a combined translate + rotate motion
static void testTrackingWheelOffsets() {
    // center travels a 30 degree clockwise arc of radius 40. Vertical tracking wheel 2" right of center,
    // horizontal wheel 3" behind center. Heading from the IMU
    const float R = 40, dTheta = deg(30), vOff = 2, hOff = -3;
    OdomStepInput in;
    in.vertical[0] = {(R - vOff) * dTheta, vOff, true, false};
    in.horizontal[0] = {-hOff * dTheta, hOff, true, false}; // left-positive, no real sideways slip
    in.imuDelta = dTheta;
    in.imuValid = true;
    in.prevHeading = deg(90); // facing +X, so a clockwise arc curves towards -Y
    const OdomStepResult step = computeOdomStep(in);
    // independently: start facing +X at origin, circle center is R to the right of the robot = (0, -R)
    // position after phi: (R sin phi, -R + R cos phi)
    CHECK_NEAR(step.globalDelta.x, R * std::sin(double(dTheta)), 1e-4);
    CHECK_NEAR(step.globalDelta.y, -R + R * std::cos(double(dTheta)), 1e-4);

    // pure rotation: offset wheels read movement, but the center doesn't move
    OdomStepInput spin;
    spin.vertical[0] = {-vOff * deg(45), vOff, true, false};
    spin.horizontal[0] = {-hOff * deg(45), hOff, true, false};
    spin.imuDelta = deg(45);
    spin.imuValid = true;
    const OdomStepResult spinStep = computeOdomStep(spin);
    CHECK_NEAR(spinStep.globalDelta.x, 0, 1e-6);
    CHECK_NEAR(spinStep.globalDelta.y, 0, 1e-6);
}

// with a single vertical and horizontal wheel, the result must match LemLib 0.5.5's equations
static void testMatchesLemLib() {
    const float cases[][6] = {
        // prevTheta, deltaTheta, deltaY, vOffset, deltaX, hOffset
        {0, 0, 1.5f, -5, 0.2f, -2},          {0.3f, 0.01f, 1.2f, 2, -0.1f, -3}, {-2.5f, -0.04f, -0.8f, -1.5f, 0.3f, 1},
        {10.0f, 0.2f, 3.0f, 4, 0.0f, -2.5f}, {1.0f, -0.5f, 0, 5, 0, -3},        {4.0f, 1e-6f, 0.5f, 3, 0.1f, 2},
    };
    for (const auto& c : cases) {
        OdomStepInput in;
        in.vertical[0] = {c[2], c[3], true, false};
        in.horizontal[0] = {c[4], c[5], true, false};
        in.imuDelta = c[1];
        in.imuValid = true;
        in.prevHeading = c[0];
        const OdomStepResult step = computeOdomStep(in);
        const Vec2 ref = lemlibOdomStep(c[0], c[1], c[2], c[3], c[4], c[5]);
        CHECK_NEAR(step.globalDelta.x, ref.x, 1e-5);
        CHECK_NEAR(step.globalDelta.y, ref.y, 1e-5);
    }
}

static void testHeadingSourcesAndFallbacks() {
    const float tw = 10, dTheta = deg(2);
    // IMU and drivetrain disagree (wheel scrub): IMU wins, like LemLib
    OdomStepInput in = drivetrainInput(5 * dTheta * 1.2f, -5 * dTheta * 1.2f, tw);
    in.imuDelta = dTheta;
    in.imuValid = true;
    OdomStepResult step = computeOdomStep(in);
    CHECK(step.headingSource == HeadingSource::IMU);
    CHECK_NEAR(step.deltaTheta, dTheta, 0);

    // IMU unplugged: falls back to the drivetrain instead of producing NaN
    in.imuValid = false;
    in.imuDelta = INFINITY;
    step = computeOdomStep(in);
    CHECK(step.headingSource == HeadingSource::DRIVETRAIN);
    CHECK_NEAR(step.deltaTheta, dTheta * 1.2f, 1e-7);
    CHECK(std::isfinite(step.globalDelta.x) && std::isfinite(step.globalDelta.y));

    // a non-finite IMU delta that is (wrongly) flagged valid is still not used
    in.imuValid = true;
    in.imuDelta = NAN;
    step = computeOdomStep(in);
    CHECK(step.headingSource == HeadingSource::DRIVETRAIN);

    // two unpowered vertical tracking wheels outrank the IMU, like LemLib
    OdomStepInput tracking;
    tracking.vertical[0] = {-3 * dTheta, 3, true, false};
    tracking.vertical[1] = {3 * dTheta, -3, true, false};
    tracking.imuDelta = 5 * dTheta;
    tracking.imuValid = true;
    step = computeOdomStep(tracking);
    CHECK(step.headingSource == HeadingSource::VERTICAL_WHEELS);
    CHECK_NEAR(step.deltaTheta, dTheta, 1e-7);

    // two horizontal wheels outrank everything
    tracking.horizontal[0] = {-4 * dTheta, 4, true, false};
    tracking.horizontal[1] = {4 * dTheta, -4, true, false};
    step = computeOdomStep(tracking);
    CHECK(step.headingSource == HeadingSource::HORIZONTAL_WHEELS);

    // nothing valid: hold heading and position
    OdomStepInput none;
    step = computeOdomStep(none);
    CHECK(step.headingSource == HeadingSource::NONE);
    CHECK(!step.forwardValid);
    CHECK_NEAR(step.globalDelta.x, 0, 0);
    CHECK_NEAR(step.globalDelta.y, 0, 0);
}

static void testDrivetrainAveraging() {
    const float tw = 10;
    // straight 1" forward; left encoder reads +0.02 noise, right reads -0.02. LemLib 0.5 reads only the left side
    // (1.02"), averaging gives 1.00"
    OdomStepInput in = drivetrainInput(1.02f, 0.98f, tw);
    in.imuDelta = 0;
    in.imuValid = true;
    OdomStepResult step = computeOdomStep(in);
    CHECK_NEAR(step.globalDelta.y, 1.0, 1e-6);

    // one side unplugged: the other side still measures forward travel
    in.vertical[0].valid = false;
    step = computeOdomStep(in);
    CHECK(step.forwardValid);
    CHECK_NEAR(step.globalDelta.y, 0.98, 1e-6);

    // unpowered tracking wheel present: it is used instead of the drive motors
    OdomStepInput mixed = drivetrainInput(1.5f, 1.5f, tw);
    mixed.vertical[0] = {1.0f, 0, true, false};
    mixed.imuDelta = 0;
    mixed.imuValid = true;
    step = computeOdomStep(mixed);
    CHECK_NEAR(step.globalDelta.y, 1.0, 1e-6);
    // and the drive motors take over if it's unplugged
    mixed.vertical[0].valid = false;
    step = computeOdomStep(mixed);
    CHECK_NEAR(step.globalDelta.y, 1.5, 1e-6);
}

static void testZeroAndSmallMovement() {
    OdomStepInput in = drivetrainInput(0, 0, 10);
    in.imuValid = true;
    in.prevHeading = deg(37);
    OdomStepResult step = computeOdomStep(in);
    CHECK_NEAR(step.globalDelta.x, 0, 0);
    CHECK_NEAR(step.globalDelta.y, 0, 0);

    // tiny IMU noise while driving straight must not distort a 1" step
    in = drivetrainInput(1, 1, 10);
    in.imuDelta = 1e-7f;
    in.imuValid = true;
    step = computeOdomStep(in);
    CHECK_NEAR(step.globalDelta.y, 1, 1e-6);
    CHECK_NEAR(step.globalDelta.x, 0, 1e-6);
}

// ---------------------------------------------------------------------------------------------------------------------
// PID
// ---------------------------------------------------------------------------------------------------------------------

static void testPIDMatchesLemLibAtNominalPeriod() {
    const float errors[] = {10, 9.2f, 7.5f, 5.1f, 2.4f, 0.6f, -0.4f, -0.9f, -0.3f, 0.2f, 0.1f, 0.05f};
    const float configs[][5] = {{5.8f, 0, 5, 0, 1}, {3.16f, 0.01f, 20, 5, 1}, {2, 0.1f, 10, 0, 0}};
    for (const auto& c : configs) {
        LemLibPID ref {c[0], c[1], c[2], c[3], c[4] != 0};
        PID pid(c[0], c[1], c[2], c[3], c[4] != 0);
        for (int i = 0; i < int(sizeof(errors) / sizeof(errors[0])); i++) {
            const float expected = ref.update(errors[i]);
            const float actual = pid.update(errors[i], 0.01f);
            // the first update differs only by LemLib's derivative kick (kD * error)
            if (i == 0) CHECK_NEAR(actual, expected - c[2] * errors[0], 1e-4);
            else CHECK_NEAR(actual, expected, 1e-4);
        }
    }
}

static void testPIDNoDerivativeKick() {
    PID pid(2, 0, 20);
    CHECK_NEAR(pid.update(3, 0.01f), 6, 1e-6); // LemLib would output 2*3 + 20*3 = 66
    CHECK_NEAR(pid.update(3, 0.01f), 6, 1e-6);
    pid.reset();
    CHECK_NEAR(pid.update(-4, 0.01f), -8, 1e-6);
}

static void testPIDConstantError() {
    // P-only: output doesn't depend on timing
    PID p(2, 0, 0);
    CHECK_NEAR(p.update(5, 0.01f), 10, 1e-6);
    CHECK_NEAR(p.update(5, 0.037f), 10, 1e-6);

    // integral grows with elapsed time, not with the number of updates
    PID fast(0, 1, 0), slow(0, 1, 0);
    for (int i = 0; i < 4; i++) fast.update(2, 0.01f);
    for (int i = 0; i < 2; i++) slow.update(2, 0.02f);
    CHECK_NEAR(fast.getIntegral(), 8, 1e-5); // 2 * 4 periods
    CHECK_NEAR(slow.getIntegral(), 8, 1e-5);
}

static void testPIDChangingError() {
    // error falling at 50 units/second = 0.5 units per 10ms period, regardless of loop timing
    PID a(0, 0, 1), b(0, 0, 1);
    float t = 0;
    for (int i = 0; i < 5; i++, t += 0.01f) a.update(100 - 50 * t, 0.01f);
    t = 0;
    for (int i = 0; i < 5; i++, t += 0.025f) b.update(100 - 50 * t, 0.025f);
    CHECK_NEAR(a.getDerivative(), -0.5, 1e-4);
    CHECK_NEAR(b.getDerivative(), -0.5, 1e-4);

    // a late update (20ms instead of 10ms) no longer doubles the derivative
    PID late(0, 0, 1);
    late.update(10, 0.01f);
    late.update(9, 0.02f); // -1 over two periods
    CHECK_NEAR(late.getDerivative(), -0.5, 1e-6);
}

static void testPIDBadTiming() {
    PID pid(1, 1, 1);
    pid.update(4, 0.01f);
    const float out = pid.update(3, 0.01f);
    const float integral = pid.getIntegral();
    const float derivative = pid.getDerivative();
    // zero, negative and non-finite dt: no integration or differentiation, finite output
    for (float dt : {0.0f, -0.01f, float(NAN), float(INFINITY)}) {
        const float o = pid.update(3, dt);
        CHECK(std::isfinite(o));
        CHECK_NEAR(o, out, 1e-6);
        CHECK_NEAR(pid.getIntegral(), integral, 0);
        CHECK_NEAR(pid.getDerivative(), derivative, 0);
    }
    // non-finite error: output 0 and state untouched
    CHECK_NEAR(pid.update(NAN, 0.01f), 0, 0);
    CHECK_NEAR(pid.update(INFINITY, 0.01f), 0, 0);
    CHECK_NEAR(pid.getIntegral(), integral, 0);
    // next good update still differentiates against the last good error (3)
    pid.update(2, 0.01f);
    CHECK_NEAR(pid.getDerivative(), -1, 1e-6);
}

static void testPIDMeasuredTime() {
    PID pid(0, 1, 1);
    fakeMicros = 1000000;
    pid.update(1); // first update assumes one nominal period
    CHECK_NEAR(pid.getIntegral(), 1, 1e-6);
    fakeMicros += 20000; // 20ms later
    pid.update(1);
    CHECK_NEAR(pid.getIntegral(), 3, 1e-5);
    // same timestamp: nothing integrates, and the clock isn't advanced
    pid.update(1);
    CHECK_NEAR(pid.getIntegral(), 3, 1e-5);
    fakeMicros += 10000;
    pid.update(0); // 1 -> 0 over exactly one period
    CHECK_NEAR(pid.getDerivative(), -1, 1e-5);
    // reset() restarts the timing
    pid.reset();
    fakeMicros += 5000000;
    pid.update(2);
    CHECK_NEAR(pid.getIntegral(), 2, 1e-6);
}

static void testPIDOutputLimitAndAntiWindup() {
    PID limited(10, 1, 0);
    limited.setOutputLimit(127);
    PID unlimited(10, 1, 0);
    for (int i = 0; i < 100; i++) {
        CHECK(std::fabs(limited.update(50, 0.01f)) <= 127);
        unlimited.update(50, 0.01f);
    }
    // saturated the whole time, so the limited controller's integral stopped growing at the first step
    CHECK_NEAR(limited.getIntegral(), 0, 0);
    CHECK_NEAR(unlimited.getIntegral(), 5000, 1e-2);
    // when the error drops the limited controller responds immediately, the unlimited one is still wound up
    CHECK(limited.update(1, 0.01f) < 20);
    CHECK(unlimited.update(1, 0.01f) > 5000);

    // integral can still shrink while saturated
    PID shrink(10, 1, 0);
    shrink.setOutputLimit(127);
    for (int i = 0; i < 10; i++) shrink.update(5, 0.01f); // 50 + I, not saturated, integral reaches 50
    CHECK_NEAR(shrink.getIntegral(), 50, 1e-4);
    shrink.update(-20, 0.01f); // saturated negative, integral moving towards 0 is allowed
    CHECK_NEAR(shrink.getIntegral(), 30, 1e-4);

    // negative saturation clamps too
    PID neg(10, 0, 0);
    neg.setOutputLimit(127);
    CHECK_NEAR(neg.update(-100, 0.01f), -127, 0);
}

static void testPIDIntegralLimitAndWindupRange() {
    PID pid(0, 0.5f, 0);
    pid.setIntegralLimit(10); // kI * integral <= 10, so integral <= 20
    for (int i = 0; i < 100; i++) pid.update(3, 0.01f);
    CHECK_NEAR(pid.getIntegral(), 20, 1e-5);
    CHECK_NEAR(pid.update(3, 0.01f), 10, 1e-5);

    // LemLib's windupRange and signFlipReset still work
    PID range(0, 1, 0, 5, false);
    range.update(2, 0.01f);
    range.update(2, 0.01f);
    CHECK_NEAR(range.getIntegral(), 4, 1e-6);
    range.update(6, 0.01f); // outside windup range
    CHECK_NEAR(range.getIntegral(), 0, 0);
    PID flip(0, 1, 0, 0, true);
    flip.update(2, 0.01f);
    flip.update(-1, 0.01f);
    CHECK_NEAR(flip.getIntegral(), 0, 0);
    // a negative first error keeps its integral (lemlib::PID would zero it)
    PID firstNegative(0, 1, 0, 0, true);
    firstNegative.update(-2, 0.01f);
    CHECK_NEAR(firstNegative.getIntegral(), -2, 0);
}

static void testPIDDerivativeFilter() {
    PID filtered(0, 0, 1);
    filtered.setDerivativeFilter(0.01f); // tau = dt, so alpha = 0.5
    filtered.update(0, 0.01f);
    filtered.update(1, 0.01f); // raw derivative 1
    CHECK_NEAR(filtered.getDerivative(), 0.5, 1e-6);
    filtered.update(2, 0.01f); // raw 1 again
    CHECK_NEAR(filtered.getDerivative(), 0.75, 1e-6);
    // converges to the true slope
    for (int i = 3; i < 40; i++) filtered.update(float(i), 0.01f);
    CHECK_NEAR(filtered.getDerivative(), 1, 1e-4);

    // alternating noise on a constant error is attenuated
    PID raw(0, 0, 1), smooth(0, 0, 1);
    smooth.setDerivativeFilter(0.04f);
    float rawPeak = 0, smoothPeak = 0;
    for (int i = 0; i < 50; i++) {
        const float e = 10 + ((i % 2) ? 0.2f : -0.2f);
        raw.update(e, 0.01f);
        smooth.update(e, 0.01f);
        if (i > 20) {
            rawPeak = std::fmax(rawPeak, std::fabs(raw.getDerivative()));
            smoothPeak = std::fmax(smoothPeak, std::fabs(smooth.getDerivative()));
        }
    }
    CHECK_NEAR(rawPeak, 0.4, 1e-5);
    CHECK(smoothPeak < rawPeak / 4);
}

static void testPIDNominalPeriod() {
    // gains expressed per 20ms: updating every 20ms gives LemLib-identical numbers
    PID pid(0, 1, 1);
    pid.setNominalPeriod(0.02f);
    pid.update(1, 0.02f);
    pid.update(2, 0.02f);
    CHECK_NEAR(pid.getIntegral(), 3, 1e-6);
    CHECK_NEAR(pid.getDerivative(), 1, 1e-6);
    pid.setNominalPeriod(0); // ignored
    pid.setNominalPeriod(NAN); // ignored
    pid.update(3, 0.02f);
    CHECK_NEAR(pid.getIntegral(), 6, 1e-6);
}

int main() {
    testWrapAngle();
    testHeadingDelta();
    testChordRatio();
    testCenterArcAndWheelHeading();
    testCoordinateTransforms();
    testPlausibleDelta();
    testArcIsExact();
    testTrackingWheelOffsets();
    testMatchesLemLib();
    testHeadingSourcesAndFallbacks();
    testDrivetrainAveraging();
    testZeroAndSmallMovement();
    testPIDMatchesLemLibAtNominalPeriod();
    testPIDNoDerivativeKick();
    testPIDConstantError();
    testPIDChangingError();
    testPIDBadTiming();
    testPIDMeasuredTime();
    testPIDOutputLimitAndAntiWindup();
    testPIDIntegralLimitAndWindupRange();
    testPIDDerivativeFilter();
    testPIDNominalPeriod();

    std::printf("%d checks, %d failures\n", checks, failures);
    return failures == 0 ? 0 : 1;
}
