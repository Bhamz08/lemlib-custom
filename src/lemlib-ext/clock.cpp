#include "pros/rtos.hpp"
#include "lemlib-ext/pid.hpp"

namespace lemlib {
namespace ext {
std::uint64_t microsNow() { return pros::micros(); }
} // namespace ext
} // namespace lemlib
