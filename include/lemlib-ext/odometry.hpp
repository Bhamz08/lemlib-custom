#pragma once

#include <cstdint>
#include <memory>
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "lemlib-ext/odomMath.hpp"
#include "pros/rtos.hpp"

namespace lemlib {
namespace ext {

/** @brief Settings for lemlib::ext::Odometry. The defaults do not change LemLib's math except where noted. */
struct OdometryConfig {
        /**
         * Multiplier applied to every IMU heading change. 1 by default (no correction).
         *
         * V5 IMUs commonly under- or over-report rotation by about 0.5-2%. To measure it, turn the robot exactly
         * N full turns (e.g. against a wall/corner reference) and set imuScale = (360 * N) / (reported change in
         * degrees). Measure several times in both directions and average.
         */
        float imuScale = 1;
        /**
         * Largest believable rotation rate in degrees per second. IMU heading changes faster than this (e.g. an IMU
         * that reconnected and restarted from 0) are rejected for that update and the drivetrain encoders are used
         * for the heading instead.
         * The V5 IMU's gyro saturates at 1000 deg/s. 0 disables the check. 1500 by default.
         */
        float maxAngularVelocity = 1500;
        /**
         * Largest believable speed of any tracking wheel in inches per second. Readings that change faster than
         * this (e.g. an encoder reconnecting and restarting from 0) are rejected for that update.
         * 0 disables the check. 200 by default (several times faster than a VEX drivetrain).
         */
        float maxLinearVelocity = 200;
        /** time between updates of the odometry task, in milliseconds. 10 by default, the same as LemLib */
        std::uint32_t periodMs = 10;
};

/**
 * @brief Drop-in replacement for LemLib 0.5's odometry loop
 *
 * LemLib (firmware/LemLib.a) runs its own odometry task. Calling start() suspends that task and runs this one
 * instead. It still reads and writes the pose through lemlib::getPose()/lemlib::setPose(), so chassis.setPose(),
 * chassis.getPose() and every chassis motion keep working unchanged.
 *
 * Differences from LemLib 0.5's update():
 * - a sensor that returns an error (PROS_ERR / PROS_ERR_F) or an impossible jump is ignored for that update and the
 *   next-best sensor is used, instead of permanently turning the pose into NaN
 * - the heading change between updates is wrapped, so heading sources that wrap at 0/360 or -180/180 can't cause a
 *   full-turn jump
 * - when the drivetrain motors are used for forward travel, both sides are averaged instead of only the left side
 * - optional IMU scale correction (OdometryConfig::imuScale)
 * - plausibility checks use the measured time between updates instead of assuming exactly 10ms
 *
 * The arc/chord and midpoint-heading equations are the same as LemLib's, so with healthy sensors and imuScale = 1 the
 * result only differs from LemLib's in which vertical wheels are averaged for forward travel.
 *
 * @note lemlib::getSpeed(), getLocalSpeed() and estimatePose() are not updated while this task runs. LemLib's own
 * motions don't use them.
 *
 * @b Example
 * @code {.cpp}
 * lemlib::ext::Odometry odometry(sensors, drivetrain, {.imuScale = 1.0045});
 *
 * void initialize() {
 *     chassis.calibrate(); // must come first: it calibrates the IMU and starts LemLib's task
 *     odometry.start(); // takes over from LemLib's odometry task
 * }
 * @endcode
 */
class Odometry {
    public:
        /**
         * @brief Create a new odometry tracker
         *
         * @param sensors the same OdomSensors passed to the Chassis. Missing vertical wheels are replaced with the
         *        drivetrain motors, exactly like Chassis::calibrate() does
         * @param drivetrain the same Drivetrain passed to the Chassis
         * @param config extra settings, see OdometryConfig
         */
        Odometry(OdomSensors sensors, Drivetrain drivetrain, OdometryConfig config = {});
        ~Odometry();

        /**
         * @brief Suspend LemLib's odometry task and start this one
         *
         * Call after chassis.calibrate(). If it is called earlier, LemLib's task is suspended as soon as
         * calibrate() creates it. Calling start() again while running does nothing.
         */
        void start();
        /**
         * @brief Stop this task and resume LemLib's odometry task
         */
        void stop();
        /**
         * @brief Run a single update. start() calls this every OdometryConfig::periodMs
         */
        void update();
        /**
         * @brief Get the heading source used by the most recent update
         *
         * Useful for checking that the IMU is being used, e.g. print it to the brain screen
         */
        HeadingSource getHeadingSource() const;
        /**
         * @brief Number of updates since start() where a sensor reading was rejected
         */
        std::uint32_t getRejectedReadings() const;
    private:
        struct WheelState {
                TrackingWheel* wheel = nullptr;
                float prevDistance = 0;
                bool hasPrev = false;
        };

        void resetReadings();
        WheelSample sampleWheel(WheelState& state, float dt);

        OdometryConfig config;
        std::unique_ptr<TrackingWheel> leftDrive;
        std::unique_ptr<TrackingWheel> rightDrive;
        WheelState vertical[2];
        WheelState horizontal[2];
        pros::Imu* imu = nullptr;
        float prevImu = 0;
        bool hasPrevImu = false;
        std::uint64_t prevTime = 0;
        HeadingSource headingSource = HeadingSource::NONE;
        std::uint32_t rejectedReadings = 0;
        pros::Task* task = nullptr;
        bool rejectedThisUpdate = false;
};

} // namespace ext
} // namespace lemlib
