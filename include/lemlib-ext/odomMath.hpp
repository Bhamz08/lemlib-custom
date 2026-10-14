#pragma once

/**
 * @file odomMath.hpp
 * @brief Pure odometry math used by lemlib::ext::Odometry
 *
 * Nothing in this file touches PROS or the precompiled LemLib archive, so it can be unit tested on a PC
 * (see tests/lemlib_ext_tests.cpp).
 *
 * Conventions match LemLib 0.5 (firmware/LemLib.a) so results can be written straight into lemlib::setPose():
 * - angles are in radians
 * - heading is a compass heading: 0 faces +Y, positive is clockwise
 * - local Y is forwards, local X is to the LEFT (this is the sign LemLib's horizontal tracking wheel math uses)
 * - tracking wheel offsets are measured from the tracking center: vertical wheels are negative on the left and
 *   positive on the right, horizontal wheels are negative behind and positive in front
 * - distances are in inches
 */

namespace lemlib {
namespace ext {

/** @brief A plain 2D vector. Units are whatever the caller uses (inches for odometry). */
struct Vec2 {
        float x = 0;
        float y = 0;
};

/**
 * @brief Wrap an angle into the range (-pi, pi]
 *
 * @param angle angle in radians. Any finite value is accepted
 * @return float equivalent angle in (-pi, pi]. Non-finite input is returned unchanged
 *
 * @b Example
 * @code {.cpp}
 * wrapAngle(3 * M_PI / 2); // returns -pi/2
 * @endcode
 */
float wrapAngle(float angle);

/**
 * @brief Wrap an angle into the range (-180, 180]
 *
 * @param angle angle in degrees
 * @return float equivalent angle in (-180, 180]. Non-finite input is returned unchanged
 */
float wrapAngleDeg(float angle);

/**
 * @brief Shortest signed change between two heading readings
 *
 * Works for sensors that wrap (0-360 headings) as well as continuous ones (IMU rotation), because the difference is
 * wrapped into (-pi, pi]. At a 10ms update period this only misreads a rotation faster than 18000 deg/s.
 *
 * @param current current heading in radians
 * @param previous previous heading in radians
 * @return float current - previous, wrapped into (-pi, pi]
 *
 * @b Example
 * @code {.cpp}
 * headingDelta(degToRad(-179), degToRad(179)); // returns degToRad(2), not degToRad(-358)
 * @endcode
 */
float headingDelta(float current, float previous);

/**
 * @brief Ratio between the chord and the arc of a circular arc that turns through deltaTheta
 *
 * chordRatio(dTheta) = 2 * sin(dTheta / 2) / dTheta
 *
 * For |dTheta| < 1e-3 rad a Taylor series (1 - dTheta^2 / 24) is used instead, so the result is continuous and there
 * is never a division by zero or loss of precision when the robot is driving straight.
 *
 * @param deltaTheta change in heading in radians
 * @return float ratio, 1 when deltaTheta is 0
 */
float chordRatio(float deltaTheta);

/**
 * @brief Distance travelled by the tracking center, measured by one tracking wheel
 *
 * A wheel that is `offset` inches from the tracking center travels on an arc whose radius differs from the center's
 * arc by `offset`, so: centerArc = wheelDelta + offset * deltaTheta.
 *
 * This removes the part of the wheel reading caused purely by rotation. For example a vertical wheel 5 inches to the
 * right (offset = 5) reads -5 * deltaTheta during a turn in place, and centerArc returns 0.
 *
 * @param wheelDelta change in the wheel's distance this update, in inches
 * @param offset the wheel's offset from the tracking center, in inches (same sign convention as lemlib::TrackingWheel)
 * @param deltaTheta change in heading this update, in radians
 * @return float distance travelled by the tracking center along the wheel's axis, in inches
 */
float centerArc(float wheelDelta, float offset, float deltaTheta);

/**
 * @brief Heading change measured by two parallel tracking wheels
 *
 * deltaTheta = -(delta1 - delta2) / (offset1 - offset2)
 *
 * This is the same equation LemLib 0.5 uses for both vertical and horizontal wheel pairs.
 *
 * @param delta1 change in distance of wheel 1, in inches
 * @param offset1 offset of wheel 1 from the tracking center, in inches
 * @param delta2 change in distance of wheel 2, in inches
 * @param offset2 offset of wheel 2 from the tracking center, in inches
 * @return float change in heading in radians, or NaN if the wheels have (almost) the same offset
 */
float wheelHeadingDelta(float delta1, float offset1, float delta2, float offset2);

/**
 * @brief Convert center travel along a circular arc into a straight-line local displacement
 *
 * If the robot moves along a constant-curvature arc, the straight line between the start and end points (the chord)
 * has length arc * chordRatio(deltaTheta), and it points along the average of the start and end heading. Using the
 * chord instead of the arc length is what makes odometry exact for arcs.
 *
 * @param forwardArc distance travelled forwards by the tracking center, in inches
 * @param sidewaysArc distance travelled left by the tracking center, in inches
 * @param deltaTheta change in heading this update, in radians
 * @return Vec2 local displacement {x = left, y = forwards}, in the robot's frame at the midpoint heading
 */
Vec2 localDisplacement(float forwardArc, float sidewaysArc, float deltaTheta);

/**
 * @brief Rotate a local displacement into the global (field) frame
 *
 * globalX = localY * sin(heading) - localX * cos(heading)
 * globalY = localY * cos(heading) + localX * sin(heading)
 *
 * @param local local displacement {x = left, y = forwards}
 * @param heading compass heading of the local frame in radians. For odometry pass the midpoint heading
 *        (previous heading + deltaTheta / 2)
 * @return Vec2 global displacement
 */
Vec2 localToGlobal(Vec2 local, float heading);

/**
 * @brief Check that a sensor delta is finite and physically possible
 *
 * PROS returns PROS_ERR_F (infinity) or PROS_ERR (INT32_MAX) when a sensor is unplugged, and sensors that reconnect
 * restart from 0, which shows up as one enormous jump. Both are rejected here.
 *
 * @param delta change in the reading
 * @param maxRate largest believable rate of change, in units per second. 0 or less disables the rate check
 * @param dt time since the last reading, in seconds
 * @return true if the delta can be used
 */
bool plausibleDelta(float delta, float maxRate, float dt);

/** @brief Which measurement an odometry step took its heading change from */
enum class HeadingSource {
    HORIZONTAL_WHEELS, /**< two horizontal tracking wheels */
    VERTICAL_WHEELS, /**< two unpowered vertical tracking wheels */
    IMU, /**< inertial sensor */
    DRIVETRAIN, /**< left and right drivetrain motor encoders */
    NONE /**< nothing valid this update; heading was held constant */
};

/** @brief One tracking wheel's reading for a single odometry step */
struct WheelSample {
        /** change in distance since the last step, in inches */
        float delta = 0;
        /** offset from the tracking center, in inches */
        float offset = 0;
        /** false if the wheel is missing, or its reading this step was rejected */
        bool valid = false;
        /** true if this "wheel" is a drivetrain motor group (lemlib::TrackingWheel::getType() == 1) */
        bool powered = false;
};

/** @brief All sensor data for a single odometry step */
struct OdomStepInput {
        /** vertical (forwards-measuring) wheels */
        WheelSample vertical[2];
        /** horizontal (sideways-measuring) wheels */
        WheelSample horizontal[2];
        /** change in IMU heading in radians, already scaled and wrapped */
        float imuDelta = 0;
        /** false if there is no IMU or its reading this step was rejected */
        bool imuValid = false;
        /** heading at the start of the step, in radians */
        float prevHeading = 0;
};

/** @brief Result of a single odometry step */
struct OdomStepResult {
        /** change in global position, in inches */
        Vec2 globalDelta;
        /** change in heading in radians */
        float deltaTheta = 0;
        /** where deltaTheta came from */
        HeadingSource headingSource = HeadingSource::NONE;
        /** false if no vertical wheel was valid, so forward motion this step could not be measured */
        bool forwardValid = false;
};

/**
 * @brief Compute one odometry step
 *
 * Heading change, in order of preference (the same order LemLib 0.5 uses), skipping any source that is invalid:
 * 1. two horizontal tracking wheels
 * 2. two unpowered vertical tracking wheels
 * 3. the IMU
 * 4. the drivetrain motor encoders
 *
 * Forward travel is the average of centerArc() over every valid unpowered vertical wheel, falling back to every valid
 * powered one. Averaging both drive sides (LemLib 0.5 only reads the left side) reduces independent encoder noise by a
 * factor of about sqrt(2), and the fallback keeps odometry running if a tracking wheel is unplugged. Sideways travel is the
 * average over valid horizontal wheels.
 *
 * The local chord is then rotated by the midpoint heading, which is exact for constant-curvature motion.
 *
 * @param input sensor data for the step
 * @return OdomStepResult
 */
OdomStepResult computeOdomStep(const OdomStepInput& input);

} // namespace ext
} // namespace lemlib
