/**
 * @file acceleration_limiter.hpp
 * @author Gento Aiba (aiba-gento)
 * @brief 加速度制限クラス
 * @version 0.1
 * @date 2026-02-21
 * 
 * @copyright Copyright (c) 2026 Gento Aiba
 * SPDX-License-Identifier: Apache-2.0
 * 
 */
#pragma once

#include <algorithm>
#include <cfloat>
#include <type_traits>

namespace gn10_motor {

/**
 * @brief 単純な加速度リミッター（slew rate limiter）
 * @tparam T 浮動小数点型 (float, double)
 */
template <typename T>
class AccelerationLimiter {
    static_assert(std::is_floating_point_v<T>, "AccelerationLimiter only supports floating point types.");

  public:
    /**
     * @brief コンストラクタ
     * @param max_acceleration 最大加速度 (単位/s^2) 絶対値で指定
     * @param initial_value 初期値
     */
    AccelerationLimiter(T max_acceleration, T initial_value = T{0})
        : max_acceleration_(std::abs(max_acceleration)), previous_value_(initial_value) {}

    /**
     * @brief 値を更新する
     * @param target_value 目標値
     * @param dt 経過時間 [s]
     * @return T 制限された新しい値
     */
    T update(T target_value, T dt) {
        if (dt <= T{0}) {
            return previous_value_;
        }

        // 最大変化量
        const T max_delta = max_acceleration_ * dt;
        
        // 要求変化量
        const T desired_delta = target_value - previous_value_;
        
        // 変化量制限
        const T limited_delta = std::clamp(desired_delta, -max_delta, max_delta);
        
        previous_value_ += limited_delta;
        
        return previous_value_;
    }

    /**
     * @brief 現在値を強制的にリセットする
     * 制御開始時や、外部要因で値が飛んだ場合に同期させるために使用
     * @param value 新しい現在値
     */
    void reset(T value = T{0}) {
        previous_value_ = value;
    }

    /**
     * @brief 最大加速度設定の更新
     * @param max_acceleration 新しい最大加速度
     */
    void set_max_acceleration(T max_acceleration) {
        max_acceleration_ = std::abs(max_acceleration);
    }

  private:
    T max_acceleration_;
    T previous_value_;
};

}  // namespace gn10_motor
