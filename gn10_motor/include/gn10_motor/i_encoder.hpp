/**
 * @file i_encoder.hpp
 * @author Gento Aiba (aiba-gento)
 * @brief インクリメンタルエンコーダの抽象インターフェース
 * @version 0.1
 * @date 2026-02-23
 *
 * @copyright Copyright (c) 2026 Gento Aiba
 * SPDX-License-Identifier: Apache-2.0
 *
 */
#pragma once

#include <cstdint>

namespace gn10_motor {

/**
 * @brief インクリメンタルエンコーダの抽象インターフェース
 *
 * ハードウェアタイマーのカウンタを読み取り、角速度・積算角度を提供する。
 * read_and_reset_count() はカウンタ読み取りと同時にリセットを行うため、
 * 呼び出し側は毎制御周期に1回だけ呼び出すこと。
 */
class IEncoder {
  public:
    virtual ~IEncoder() = default;

    /**
     * @brief ハードウェアの初期化
     */
    virtual void hardware_init() = 0;

    /**
     * @brief カウンタ値を読み取り、同時にリセットする
     * @return int16_t 前回呼び出しからの差分カウント
     */
    virtual int16_t read_and_reset_count() = 0;

    /**
     * @brief カウント差分を角速度 [rad/s] に変換する
     * @param count  read_and_reset_count() の戻り値
     * @param period_s 制御周期 [s]
     * @return float 角速度 [rad/s]
     */
    virtual float count_to_angular_velocity(int16_t count, float period_s) = 0;

    /**
     * @brief カウント差分を積算し、積算角度 [rad] を返す
     * @param count read_and_reset_count() の戻り値
     * @return float 積算角度 [rad]
     */
    virtual float accumulate_angle_rad(int16_t count) = 0;

    /**
     * @brief 積算角度とカウンタをリセットする
     */
    virtual void reset() = 0;
};

}  // namespace gn10_motor
