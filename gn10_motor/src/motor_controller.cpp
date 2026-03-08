/**
 * @file motor_controller.cpp
 * @author Gento Aiba (aiba-gento)
 * @brief DCモーター制御クラスの実装
 * @version 0.2.0
 * @date 2026-02-23
 *
 * @copyright Copyright (c) 2026 ararobo
 * SPDX-License-Identifier: Apache-2.0
 */
#include "gn10_motor/motor_controller.hpp"

#include <algorithm>
#include <cfloat>

namespace gn10_motor {

// accel_ratio(0.0〜1.0) を AccelerationLimiter の max_acceleration [/s] に変換するスケール係数
// accel_ratio = 1.0 のとき、1ms で正規化デューティ全域(-1→1)を変化できる = 2000/s
// 実用的に accel_ratio ≈ 0.02 (5/255) → 20/s = 50msで0→1 相当
static constexpr float ACCEL_SCALE = 1000.0f;

// accel_ratio = 0 のとき制限なし (FLT_MAX に相当)
static constexpr float ACCEL_NO_LIMIT = FLT_MAX;

// PID積分項の最大値: 出力正規化空間 [-1, 1] の 30%
static constexpr float DEFAULT_INTEGRAL_LIMIT = 0.3f;

// -----------------------------------------------------------------------

MotorController::MotorController(
    IGateDriver& driver, IEncoder& encoder, gn10_can::devices::MotorDriverServer& can_server
)
    : driver_(driver),
      encoder_(encoder),
      can_server_(can_server),
      pid_(PIDConfig<float>{}),
      accel_limiter_(ACCEL_NO_LIMIT),
      target_(0.0f),
      feedback_value_(0.0f),
      initialized_(false),
      no_target_count_(0)
{
    gains_.fill(0.0f);
}

// -----------------------------------------------------------------------

void MotorController::update(float dt_s, uint8_t limit_switch_state)
{
    // CAN受信を polling して設定・ゲイン・目標値を更新
    poll_can();

    // init パケット受信前は制御を行わない
    if (!initialized_) {
        return;
    }

    // --- タイムアウト処理: 長時間目標値が更新されなければ停止 ---
    if (no_target_count_ >= NO_TARGET_TIMEOUT_CYCLES) {
        stop();
        return;
    }

    // --- エンコーダ読み取り & フィードバック値計算 ---
    const int16_t count = encoder_.read_and_reset_count();
    feedback_value_     = compute_feedback(count, dt_s);

    // --- 制御演算: エンコーダありなら PID、なしならオープンループ ---
    float duty          = 0.0f;
    const auto enc_type = config_.get_encoder_type();
    const bool use_pid =
        (enc_type != gn10_can::devices::EncoderType::None) &&
        (gains_[static_cast<std::size_t>(gn10_can::devices::GainType::Kp)] != 0.0f);
    if (use_pid) {
        duty = pid_.update(target_, feedback_value_, dt_s);
    } else {
        // オープンループ: target_ をそのままデューティ [-1.0, 1.0] として扱う
        duty = target_;
    }

    // --- max_duty_ratio による出力制限 ---
    const float max_duty = config_.get_max_duty_ratio();
    duty                 = std::clamp(duty, -max_duty, max_duty);

    // --- 加速度制限 (台形制御) ---
    duty = accel_limiter_.update(duty, dt_s);

    // --- リミットスイッチによる出力制限 ---
    duty = apply_limit_switch(duty, limit_switch_state);

    // --- モーター出力 & フィードバック送信 ---
    driver_.output(duty);
    can_server_.send_feedback(feedback_value_, limit_switch_state);
}

void MotorController::stop()
{
    driver_.output(0.0f);
    // encoder_.reset() は呼ばない: 停止しても位置・速度情報は保持する
    pid_.reset(feedback_value_);
    accel_limiter_.reset(0.0f);
    no_target_count_ = 0;
}

void MotorController::reset()
{
    pid_.reset(feedback_value_);
    accel_limiter_.reset(0.0f);
    encoder_.reset();
    no_target_count_ = 0;
}

// -----------------------------------------------------------------------
// 内部処理
// -----------------------------------------------------------------------

void MotorController::poll_can()
{
    // 設定 (初期化パケット)
    if (gn10_can::devices::MotorConfig new_config; can_server_.get_new_init(new_config)) {
        config_ = new_config;
        apply_config_to_controllers();
        initialized_ = true;
        reset();
    }

    // ゲイン更新
    bool gain_updated = false;
    for (std::size_t idx = 0; idx < gains_.size(); ++idx) {
        const auto type = static_cast<gn10_can::devices::GainType>(idx);
        if (can_server_.get_new_gain(type, gains_[idx])) {
            gain_updated = true;
        }
    }
    if (gain_updated) {
        apply_config_to_controllers();
    }

    // 目標値
    if (can_server_.get_new_target(target_)) {
        no_target_count_ = 0;
    } else {
        ++no_target_count_;
    }
}

void MotorController::apply_config_to_controllers()
{
    // GainType を配列インデックスに変換するローカルラムダ
    auto idx = [](gn10_can::devices::GainType type) { return static_cast<std::size_t>(type); };

    // PIDConfig を再構築
    PIDConfig<float> pid_config;
    pid_config.kp             = gains_[idx(gn10_can::devices::GainType::Kp)];
    pid_config.ki             = gains_[idx(gn10_can::devices::GainType::Ki)];
    pid_config.kd             = gains_[idx(gn10_can::devices::GainType::Kd)];
    pid_config.integral_limit = DEFAULT_INTEGRAL_LIMIT;
    pid_config.output_limit   = config_.get_max_duty_ratio();
    pid_.update_config(pid_config);

    // AccelerationLimiter の max_acceleration を再計算
    const float accel_ratio = config_.get_accel_ratio();
    const float max_accel   = (accel_ratio > 0.0f) ? (accel_ratio * ACCEL_SCALE) : ACCEL_NO_LIMIT;
    accel_limiter_.set_max_acceleration(max_accel);
}

float MotorController::apply_limit_switch(float duty, uint8_t limit_sw_state) const
{
    // 正転停止設定の確認
    bool stop_fwd  = false;
    uint8_t fwd_id = 0;
    config_.get_forward_limit_switch(stop_fwd, fwd_id);
    if (stop_fwd && (limit_sw_state & (1U << fwd_id)) && duty > 0.0f) {
        return 0.0f;
    }

    // 逆転停止設定の確認
    bool stop_bwd  = false;
    uint8_t bwd_id = 0;
    config_.get_reverse_limit_switch(stop_bwd, bwd_id);
    if (stop_bwd && (limit_sw_state & (1U << bwd_id)) && duty < 0.0f) {
        return 0.0f;
    }

    return duty;
}

float MotorController::compute_feedback(int16_t count, float dt_s)
{
    switch (config_.get_encoder_type()) {
        case gn10_can::devices::EncoderType::IncrementalSpeed:
            return encoder_.count_to_angular_velocity(count, dt_s);

        case gn10_can::devices::EncoderType::IncrementalTotal:
            return encoder_.accumulate_angle_rad(count);

        // アブソリュートエンコーダは HTMDv2.2c では未対応のため出力を止める
        case gn10_can::devices::EncoderType::Absolute:
            driver_.output(0.0f);
            return 0.0f;

        case gn10_can::devices::EncoderType::None:
        default:
            return 0.0f;
    }
}

}  // namespace gn10_motor
