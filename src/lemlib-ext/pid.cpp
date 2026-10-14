#include <cmath>
#include "lemlib-ext/pid.hpp"

namespace lemlib {
namespace ext {
namespace {
// same convention as lemlib::sgn: 0 counts as positive
int sign(float x) { return x < 0 ? -1 : 1; }

float clamp(float x, float limit) {
    if (x > limit) return limit;
    if (x < -limit) return -limit;
    return x;
}
} // namespace

PID::PID(float kP, float kI, float kD, float windupRange, bool signFlipReset)
    : kP(kP),
      kI(kI),
      kD(kD),
      windupRange(windupRange),
      signFlipReset(signFlipReset) {}

PID& PID::setOutputLimit(float limit) {
    outputLimit = std::fabs(limit);
    return *this;
}

PID& PID::setIntegralLimit(float limit) {
    integralLimit = std::fabs(limit);
    return *this;
}

PID& PID::setDerivativeFilter(float tau) {
    derivativeTau = std::fabs(tau);
    return *this;
}

PID& PID::setNominalPeriod(float seconds) {
    if (std::isfinite(seconds) && seconds > 0) nominalPeriod = seconds;
    return *this;
}

float PID::update(float error) {
    if (!std::isfinite(error)) return 0;
    const std::uint64_t now = microsNow();
    float dt = nominalPeriod;
    if (hasPrevTime) {
        // no time has passed: don't advance the clock, so the next update measures the full interval
        if (now <= prevTime) return update(error, 0);
        dt = (now - prevTime) / 1e6f;
    }
    prevTime = now;
    hasPrevTime = true;
    return update(error, dt);
}

float PID::update(float error, float dt) {
    // a NaN/inf error (e.g. from a NaN pose) would poison the integral and derivative forever. Output nothing
    if (!std::isfinite(error)) return 0;

    // only integrate and differentiate when time has actually passed. Otherwise reuse the last integral/derivative
    if (std::isfinite(dt) && dt > 0) {
        // fraction of a nominal period that passed. 1 when updates are exactly nominalPeriod apart, which makes the
        // math below identical to lemlib::PID
        const float periods = dt / nominalPeriod;

        // integral: rectangle rule, error * time, in units of error * nominal periods
        float newIntegral = integral + error * periods;
        if (signFlipReset && hasPrevError && sign(error) != sign(prevError)) newIntegral = 0;
        if (windupRange != 0 && std::fabs(error) > windupRange) newIntegral = 0;
        if (integralLimit > 0 && kI != 0) newIntegral = clamp(newIntegral, integralLimit / std::fabs(kI));

        // derivative: change in error per nominal period. 0 on the first update, so there's no derivative kick
        float rawDerivative = 0;
        if (hasPrevError) rawDerivative = (error - prevError) / periods;
        if (derivativeTau > 0 && hasPrevError) {
            // first-order low-pass. alpha depends on dt so the filter's time constant doesn't change with loop rate
            const float alpha = dt / (derivativeTau + dt);
            derivative += alpha * (rawDerivative - derivative);
        } else {
            derivative = rawDerivative;
        }

        // conditional integration anti-windup: if the output is already past the limit and the error is pushing it
        // further, don't let the integral grow
        if (outputLimit > 0 && std::fabs(newIntegral) > std::fabs(integral)) {
            const float unclamped = kP * error + kI * newIntegral + kD * derivative;
            if (std::fabs(unclamped) > outputLimit && sign(unclamped) == sign(error)) newIntegral = integral;
        }

        integral = newIntegral;
        prevError = error;
        hasPrevError = true;
    }

    float output = kP * error + kI * integral + kD * derivative;
    if (!std::isfinite(output)) return 0;
    if (outputLimit > 0) output = clamp(output, outputLimit);
    return output;
}

void PID::reset() {
    integral = 0;
    derivative = 0;
    prevError = 0;
    hasPrevError = false;
    hasPrevTime = false;
}

float PID::getIntegral() const { return integral; }

float PID::getDerivative() const { return derivative; }

} // namespace ext
} // namespace lemlib
