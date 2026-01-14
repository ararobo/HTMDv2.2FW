#pragma once

#include <cfloat>
#include <algorithm>

namespace common::control {

template <typename T>
class AccelerationLimiter {
  public:
    AccelerationLimiter(T max_acceleration, T initial_value = T(0))
        : max_acceleration_(max_acceleration), previous_value_(initial_value) {}

    T limit(T target_value, T dt) {
        T max_delta = max_acceleration_ * dt;
        T desired_delta = target_value - previous_value_;
        T limited_delta = std::clamp(desired_delta, -max_delta, max_delta);
        previous_value_ += limited_delta;
        return previous_value_;
    }

    void reset(T value = T(0)) {
        previous_value_ = value;
    }

  private:
    T max_acceleration_;
    T previous_value_;
};

}  // namespace common::control