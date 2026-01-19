#pragma once

#include <algorithm>

namespace common::control {

template <typename T>
struct PIDConfig {
    T kp             = T{0};
    T ki             = T{0};
    T kd             = T{0};
    T integral_limit = T{0};
    T output_limit   = T{0};
};

template <typename T>
class PID {
    // テンプレート引数が浮動小数点型であることを保証する (C++17)
    static_assert(std::is_floating_point_v<T>, "PID class only supports floating point types.");

  public:
    explicit PID(const PIDConfig<T>& config) : config_(config) {}

    T update(T setpoint, T measurement, T dt) {
        // dtが0以下の場合は計算をスキップ（ゼロ除算防止）
        if (dt <= T{0}) return T{0};

        T error = setpoint - measurement;

        T p_term = config_.kp * error;

        integral_ += error * dt;

        // 積分蓄積値を制限
        integral_ = std::clamp(integral_, -config_.integral_limit, config_.integral_limit);

        T i_term = config_.ki * integral_;

        // 微分先行 (Derivative on Measurement)
        T derivative = T{0};
        if (dt > T{0}) {
            derivative = (measurement - previous_measurement_) / dt;
        }

        T d_term = -config_.kd * derivative;

        T output = p_term + i_term + d_term;

        // 出力制限
        output = std::clamp(output, -config_.output_limit, config_.output_limit);

        previous_measurement_ = measurement;

        return output;
    }

    void reset(T current_measurement = T{0}) {
        integral_             = T{0};
        previous_measurement_ = current_measurement;
    }

    void set_config(const PIDConfig<T>& config) {
        config_   = config;
        integral_ = T{0};
    }

  private:
    PIDConfig<T> config_;
    T integral_             = T{0};
    T previous_measurement_ = T{0};
};

}  // namespace common::control