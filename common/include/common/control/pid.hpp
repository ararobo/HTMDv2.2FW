#pragma once

#include <cfloat>
#include <cmath>
#include <algorithm>

namespace common::control {

struct PIDConfig {
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;
    float integral_limit = FLT_MAX; // 積分項（蓄積値）の上限
    float output_limit = FLT_MAX;   // 最終出力の上限
};

class PID {
 public:
    explicit PID(const PIDConfig& config) : config_(config) {}

    float update(float setpoint, float measurement, float dt) {
        float error = setpoint - measurement;

        float p_term = config_.kp * error;

        integral_ += error * dt;
        
        // 積分蓄積値を制限
        integral_ = std::clamp(integral_, -config_.integral_limit, config_.integral_limit);
        
        float i_term = config_.ki * integral_;

        // 微分先行 (Derivative on Measurement)
        float derivative = 0.0f;
        if (dt > 0.0f) {
            derivative = (measurement - previous_measurement_) / dt;
        }
        
        float d_term = -config_.kd * derivative;

        float output = p_term + i_term + d_term;

        // 出力制限
        output = std::clamp(output, -config_.output_limit, config_.output_limit);

        previous_measurement_ = measurement;

        return output;
    }

    void reset() {
        integral_ = 0.0f;
        previous_measurement_ = 0.0f;
    }

    // パラメータ動的変更用
    void set_config(const PIDConfig& config) {
        config_ = config;
        reset(); // ゲイン変更時は安全のためリセット
    }

 private:
    PIDConfig config_;
    float integral_ = 0.0f;
    float previous_measurement_ = 0.0f;
};

}  // namespace common::control