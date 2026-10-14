#pragma once

#include <cstdint>

namespace lemlib {
namespace ext {

/**
 * @brief Microseconds since the program started. Uses pros::micros() (src/lemlib-ext/clock.cpp)
 *
 * Kept separate so the unit tests can supply a fake clock.
 */
std::uint64_t microsNow();

/**
 * @brief PID controller that accounts for the real time between updates
 *
 * Constructor, update(error) and reset() match lemlib::PID, so it can replace lemlib::PID in your own control loops
 * without retuning. The gains keep LemLib's units: the integral and derivative are expressed per nominal period (10ms
 * by default) instead of per second, so when updates really are 10ms apart the output is the same as lemlib::PID.
 *
 * With dt = time since the last update and T = nominal period:
 *
 *   integral   += error * (dt / T)
 *   derivative  = (error - prevError) / (dt / T)
 *   output      = kP * error + kI * integral + kD * derivative
 *
 * Differences from lemlib::PID (LemLib 0.5):
 * - real elapsed time is used, so a late update doesn't inflate the derivative or shrink the integral
 * - no derivative kick: the first update after construction or reset() has a derivative of 0 instead of
 *   (error - 0), which could add kD * error to the first output
 * - the first update after reset() never triggers the sign-flip integral reset (lemlib::PID compares against a
 *   previous error of 0, so a negative first error always discards its own integral)
 * - a non-finite error returns 0 and leaves the controller untouched; a dt that is zero, negative or non-finite
 *   doesn't integrate or differentiate
 * - optional extras, all off by default: output limit with anti-windup, integral limit, derivative low-pass filter
 *
 * @b Example
 * @code {.cpp}
 * lemlib::ext::PID armPID(2, 0.05, 10); // same arguments as lemlib::PID
 * armPID.setOutputLimit(127) // motor voltage range, stops the integral winding up while saturated
 *       .setDerivativeFilter(0.03); // 30ms low-pass on the derivative
 *
 * while (true) {
 *     arm.move(armPID.update(target - arm.get_position()));
 *     pros::delay(10);
 * }
 * @endcode
 */
class PID {
    public:
        /**
         * @brief Construct a new PID. Same arguments and defaults as lemlib::PID
         *
         * @param kP proportional gain
         * @param kI integral gain, per nominal period
         * @param kD derivative gain, per nominal period
         * @param windupRange the integral is reset to 0 while |error| is larger than this. 0 disables it
         * @param signFlipReset reset the integral when the error changes sign
         */
        PID(float kP, float kI, float kD, float windupRange = 0, bool signFlipReset = false);

        /**
         * @brief Limit the output to [-limit, limit]
         *
         * While the output is clamped and the error would push it further, the integral stops growing
         * (conditional-integration anti-windup), so it doesn't overshoot after a long saturated move.
         *
         * @param limit largest output magnitude, e.g. 127 for motor.move(). 0 (default) disables the limit
         * @return PID& this, so calls can be chained
         */
        PID& setOutputLimit(float limit);
        /**
         * @brief Limit how much the integral term can contribute to the output
         *
         * @param limit largest magnitude of kI * integral, in output units. 0 (default) disables the limit
         * @return PID& this, so calls can be chained
         */
        PID& setIntegralLimit(float limit);
        /**
         * @brief Low-pass filter the derivative to reduce sensor noise
         *
         * First-order filter: d = d_prev + alpha * (d_raw - d_prev), with alpha = dt / (tau + dt).
         * Larger tau = smoother but more delayed derivative. A good starting point is 2-5 update periods.
         *
         * @param tau filter time constant in seconds. 0 (default) disables the filter
         * @return PID& this, so calls can be chained
         */
        PID& setDerivativeFilter(float tau);
        /**
         * @brief Set the period the gains are expressed in
         *
         * @param seconds nominal time between updates in seconds. 0.01 (10ms) by default, matching LemLib's loops.
         *        Values that are not positive and finite are ignored
         * @return PID& this, so calls can be chained
         */
        PID& setNominalPeriod(float seconds);

        /**
         * @brief Update the PID, measuring the time since the previous update
         *
         * The first update after construction or reset() assumes one nominal period has passed.
         *
         * @param error target minus position
         * @return float output
         */
        float update(float error);
        /**
         * @brief Update the PID with a known time step
         *
         * @param error target minus position
         * @param dt time since the previous update, in seconds
         * @return float output
         */
        float update(float error, float dt);
        /**
         * @brief Reset the integral, derivative and timing
         */
        void reset();

        /** @brief Current integral, in error * nominal periods */
        float getIntegral() const;
        /** @brief Current (filtered) derivative, in error per nominal period */
        float getDerivative() const;
    protected:
        // gains
        const float kP;
        const float kI;
        const float kD;

        // optimizations
        const float windupRange;
        const bool signFlipReset;

        // extensions. 0 = disabled
        float outputLimit = 0;
        float integralLimit = 0;
        float derivativeTau = 0;
        float nominalPeriod = 0.01;

        float integral = 0;
        float derivative = 0;
        float prevError = 0;
        bool hasPrevError = false;
        std::uint64_t prevTime = 0;
        bool hasPrevTime = false;
};

} // namespace ext
} // namespace lemlib
