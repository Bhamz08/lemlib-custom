#include <cmath>
#include "lemlib-ext/odomMath.hpp"

namespace lemlib {
namespace ext {
namespace {
constexpr double PI = 3.14159265358979323846;
// below this heading change (radians), chordRatio uses its Taylor series. At 1e-3 the next term (x^4 / 1920) is
// ~5e-16, far below float precision
constexpr float SMALL_ANGLE = 1e-3f;
// two parallel wheels closer together than this (inches) can't measure heading
constexpr float MIN_WHEEL_SEPARATION = 1e-3f;
} // namespace

float wrapAngle(float angle) {
    if (!std::isfinite(angle)) return angle;
    // remainder() returns a value in [-pi, pi]. Done in double so large accumulated headings wrap accurately
    double wrapped = std::remainder(double(angle), 2 * PI);
    // float(M_PI) is slightly larger than pi and would land just above -pi. Treat anything within float precision of
    // -pi as -pi so it maps to +pi
    if (wrapped <= -PI + 1e-6) wrapped += 2 * PI;
    return float(wrapped);
}

float wrapAngleDeg(float angle) {
    if (!std::isfinite(angle)) return angle;
    double wrapped = std::remainder(double(angle), 360.0);
    if (wrapped <= -180.0) wrapped += 360.0;
    return float(wrapped);
}

float headingDelta(float current, float previous) { return wrapAngle(current - previous); }

float chordRatio(float deltaTheta) {
    // 2 sin(x/2) / x = 1 - x^2/24 + x^4/1920 - ...
    if (std::fabs(deltaTheta) < SMALL_ANGLE) return 1 - deltaTheta * deltaTheta / 24;
    return 2 * std::sin(deltaTheta / 2) / deltaTheta;
}

float centerArc(float wheelDelta, float offset, float deltaTheta) { return wheelDelta + offset * deltaTheta; }

float wheelHeadingDelta(float delta1, float offset1, float delta2, float offset2) {
    const float separation = offset1 - offset2;
    if (std::fabs(separation) < MIN_WHEEL_SEPARATION) return NAN;
    return -(delta1 - delta2) / separation;
}

Vec2 localDisplacement(float forwardArc, float sidewaysArc, float deltaTheta) {
    // chord length = arc length * 2 sin(dTheta/2) / dTheta. This is algebraically the same as LemLib's
    // 2 sin(dTheta/2) * (delta / dTheta + offset), with the offset term folded into the arc by centerArc()
    const float ratio = chordRatio(deltaTheta);
    return {sidewaysArc * ratio, forwardArc * ratio};
}

Vec2 localToGlobal(Vec2 local, float heading) {
    const float s = std::sin(heading);
    const float c = std::cos(heading);
    // local Y is forwards, local X is left; heading is a compass heading (0 = +Y, clockwise positive)
    return {local.y * s - local.x * c, local.y * c + local.x * s};
}

bool plausibleDelta(float delta, float maxRate, float dt) {
    if (!std::isfinite(delta)) return false;
    if (maxRate <= 0 || !(dt > 0)) return true;
    return std::fabs(delta) <= maxRate * dt;
}

/**
 * Average centerArc() over the valid wheels in a pair, optionally only the powered or unpowered ones.
 * Returns false if no wheel qualified.
 */
static bool averageCenterArc(const WheelSample (&wheels)[2], float deltaTheta, bool powered, float& out) {
    float sum = 0;
    int count = 0;
    for (const WheelSample& wheel : wheels) {
        if (!wheel.valid || wheel.powered != powered) continue;
        sum += centerArc(wheel.delta, wheel.offset, deltaTheta);
        count++;
    }
    if (count == 0) return false;
    out = sum / count;
    return true;
}

static bool pairHeading(const WheelSample (&wheels)[2], float& out) {
    if (!wheels[0].valid || !wheels[1].valid) return false;
    const float delta = wheelHeadingDelta(wheels[0].delta, wheels[0].offset, wheels[1].delta, wheels[1].offset);
    if (!std::isfinite(delta)) return false;
    out = delta;
    return true;
}

OdomStepResult computeOdomStep(const OdomStepInput& input) {
    OdomStepResult result;

    // heading, using the same priority as LemLib 0.5, but skipping sources that are missing or invalid this step
    float deltaTheta = 0;
    const bool verticalUnpowered = !input.vertical[0].powered && !input.vertical[1].powered;
    if (pairHeading(input.horizontal, deltaTheta)) {
        result.headingSource = HeadingSource::HORIZONTAL_WHEELS;
    } else if (verticalUnpowered && pairHeading(input.vertical, deltaTheta)) {
        result.headingSource = HeadingSource::VERTICAL_WHEELS;
    } else if (input.imuValid && std::isfinite(input.imuDelta)) {
        deltaTheta = input.imuDelta;
        result.headingSource = HeadingSource::IMU;
    } else if (pairHeading(input.vertical, deltaTheta)) {
        result.headingSource = verticalUnpowered ? HeadingSource::VERTICAL_WHEELS : HeadingSource::DRIVETRAIN;
    } else {
        deltaTheta = 0;
        result.headingSource = HeadingSource::NONE;
    }
    result.deltaTheta = deltaTheta;

    // translation along each local axis, measured at the tracking center
    float forwardArc = 0;
    result.forwardValid = averageCenterArc(input.vertical, deltaTheta, false, forwardArc) ||
                          averageCenterArc(input.vertical, deltaTheta, true, forwardArc);
    if (!result.forwardValid) forwardArc = 0;
    float sidewaysArc = 0;
    if (!averageCenterArc(input.horizontal, deltaTheta, false, sidewaysArc) &&
        !averageCenterArc(input.horizontal, deltaTheta, true, sidewaysArc))
        sidewaysArc = 0;

    // arc -> chord, then rotate by the midpoint heading. For a constant-curvature step the chord points exactly along
    // the average of the start and end heading, so this has no integration error for arcs
    const Vec2 local = localDisplacement(forwardArc, sidewaysArc, deltaTheta);
    result.globalDelta = localToGlobal(local, input.prevHeading + deltaTheta / 2);
    return result;
}

} // namespace ext
} // namespace lemlib
