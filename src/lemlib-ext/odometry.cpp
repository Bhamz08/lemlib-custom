#include <cmath>
#include "lemlib/chassis/odom.hpp"
#include "lemlib/pose.hpp"
#include "lemlib-ext/odometry.hpp"

// LemLib 0.5's odometry task, defined in odom.cpp inside firmware/LemLib.a. It's a plain global, so it can be
// suspended while lemlib::ext::Odometry is running. Created by lemlib::init(), which Chassis::calibrate() calls
extern pros::Task* trackingTask;

namespace lemlib {
namespace ext {
namespace {
constexpr float DEG_TO_RAD = 3.14159265358979323846f / 180;
} // namespace

Odometry::Odometry(OdomSensors sensors, Drivetrain drivetrain, OdometryConfig config)
    : config(config),
      imu(sensors.imu) {
    // same substitution Chassis::calibrate() makes when no vertical tracking wheels are given
    if (sensors.vertical1 == nullptr) {
        leftDrive = std::make_unique<TrackingWheel>(drivetrain.leftMotors, drivetrain.wheelDiameter,
                                                    -(drivetrain.trackWidth / 2), drivetrain.rpm);
        sensors.vertical1 = leftDrive.get();
    }
    if (sensors.vertical2 == nullptr) {
        rightDrive = std::make_unique<TrackingWheel>(drivetrain.rightMotors, drivetrain.wheelDiameter,
                                                     drivetrain.trackWidth / 2, drivetrain.rpm);
        sensors.vertical2 = rightDrive.get();
    }
    vertical[0].wheel = sensors.vertical1;
    vertical[1].wheel = sensors.vertical2;
    horizontal[0].wheel = sensors.horizontal1;
    horizontal[1].wheel = sensors.horizontal2;
}

Odometry::~Odometry() { stop(); }

void Odometry::resetReadings() {
    for (WheelState& state : vertical) state.hasPrev = false;
    for (WheelState& state : horizontal) state.hasPrev = false;
    hasPrevImu = false;
    prevTime = 0;
}

void Odometry::start() {
    if (task != nullptr) return;
    resetReadings();
    if (trackingTask != nullptr) trackingTask->suspend();
    task = new pros::Task([this] {
        std::uint32_t wakeTime = pros::millis();
        while (true) {
            // if start() was called before chassis.calibrate(), LemLib's task appears later. Keep it suspended so
            // the motion isn't integrated twice
            if (trackingTask != nullptr && trackingTask->get_state() != pros::E_TASK_STATE_SUSPENDED)
                trackingTask->suspend();
            update();
            // delay_until keeps a fixed period even if update() takes a variable amount of time
            pros::Task::delay_until(&wakeTime, config.periodMs);
        }
    });
}

void Odometry::stop() {
    if (task == nullptr) return;
    task->remove();
    delete task;
    task = nullptr;
    if (trackingTask != nullptr) {
        // LemLib's task still holds the sensor readings from before start(). Run one update while it's suspended so
        // they catch up, then put back the pose, otherwise all the motion since start() would be added a second time
        const Pose pose = lemlib::getPose(true);
        lemlib::update();
        lemlib::setPose(pose, true);
        trackingTask->resume();
    }
}

WheelSample Odometry::sampleWheel(WheelState& state, float dt) {
    WheelSample sample;
    if (state.wheel == nullptr) return sample;
    sample.offset = state.wheel->getOffset();
    sample.powered = state.wheel->getType() == 1;

    const float distance = state.wheel->getDistanceTraveled();
    // PROS_ERR_F (a disconnected rotation sensor or motor). Keep the last good reading so the next good one is
    // measured against it
    if (!std::isfinite(distance)) {
        rejectedThisUpdate = true;
        return sample;
    }
    if (!state.hasPrev) {
        state.prevDistance = distance;
        state.hasPrev = true;
        sample.valid = true; // first reading: no movement yet
        return sample;
    }
    const float delta = distance - state.prevDistance;
    // always move the baseline forward. If this reading was a glitch (e.g. a sensor that reconnected and restarted
    // from 0) the next delta is measured from the new value instead of repeating the jump
    state.prevDistance = distance;
    if (!plausibleDelta(delta, config.maxLinearVelocity, dt)) {
        rejectedThisUpdate = true;
        return sample;
    }
    sample.delta = delta;
    sample.valid = true;
    return sample;
}

void Odometry::update() {
    // time since the last update. The first update after start() assumes the nominal period
    const std::uint64_t now = pros::micros();
    float dt = config.periodMs / 1000.0f;
    if (prevTime != 0 && now > prevTime) dt = (now - prevTime) / 1e6f;
    prevTime = now;
    rejectedThisUpdate = false;

    OdomStepInput input;
    for (int i = 0; i < 2; i++) {
        input.vertical[i] = sampleWheel(vertical[i], dt);
        input.horizontal[i] = sampleWheel(horizontal[i], dt);
    }

    if (imu != nullptr) {
        // get_rotation() is PROS_ERR_F while the IMU is unplugged or calibrating
        const float rotation = float(imu->get_rotation()) * DEG_TO_RAD;
        if (!std::isfinite(rotation)) {
            rejectedThisUpdate = true;
        } else {
            if (hasPrevImu) {
                input.imuDelta = headingDelta(rotation, prevImu) * config.imuScale;
                input.imuValid = plausibleDelta(input.imuDelta, config.maxAngularVelocity * DEG_TO_RAD, dt);
                if (!input.imuValid) rejectedThisUpdate = true;
            } else {
                input.imuValid = true; // first reading: no rotation yet
            }
            prevImu = rotation;
            hasPrevImu = true;
        }
    }

    // read-modify-write, so poses set with chassis.setPose() are picked up straight away
    Pose pose = lemlib::getPose(true);
    input.prevHeading = pose.theta;
    const OdomStepResult step = computeOdomStep(input);
    pose.x += step.globalDelta.x;
    pose.y += step.globalDelta.y;
    // heading is left unwrapped, like LemLib, so motions that use the continuous heading are unaffected
    pose.theta += step.deltaTheta;
    lemlib::setPose(pose, true);

    headingSource = step.headingSource;
    if (rejectedThisUpdate) rejectedReadings++;
}

HeadingSource Odometry::getHeadingSource() const { return headingSource; }

std::uint32_t Odometry::getRejectedReadings() const { return rejectedReadings; }

} // namespace ext
} // namespace lemlib
