/**
 * @file incremental_encoder.hpp
 * @author Gento Aiba (aiba-gento), Watanabe-Koichiro
 * @brief インクリメンタルエンコーダの具象クラス
 * @version 3.0
 * @date 2026-02-23
 *
 * @copyright Copyright (c) 2025
 *
 */
#pragma once

#include <cstdint>

#include "gn10_motor/i_encoder.hpp"

/**
 * @brief htim1 エンコーダモードを使用するインクリメンタルエンコーダの具象クラス
 */
class IncrementalEncoder : public gn10_motor::IEncoder {
  public:
    /**
     * @brief コンストラクタ
     * @param max_count エンコーダ1回転あたりのカウント数 (分解能)
     */
    explicit IncrementalEncoder(uint16_t max_count);

    /** @brief エンコーダタイマーの初期化 */
    void hardware_init() override;

    /**
     * @brief カウンタ値を読み取り、同時にリセットする
     * @return int16_t 前回呼び出しからの差分カウント
     */
    int16_t read_and_reset_count() override;

    /**
     * @brief カウント差分を角速度 [rad/s] に変換する
     * @param count  read_and_reset_count() の戻り値
     * @param period_s 制御周期 [s]
     * @return float 角速度 [rad/s]
     */
    float count_to_angular_velocity(int16_t count, float period_s) override;

    /**
     * @brief カウント差分を積算し、積算角度 [rad] を返す
     * @param count read_and_reset_count() の戻り値
     * @return float 積算角度 [rad]
     */
    float accumulate_angle_rad(int16_t count) override;

    /** @brief 積算角度とカウンタをリセットする */
    void reset() override;

  private:
    /**
     * @brief カウント値をラジアンに変換する内部ユーティリティ
     * @param count カウント値
     * @return float ラジアン値
     */
    float count_to_rad(int16_t count) const;

    const uint16_t max_count_;  ///< 1回転あたりのカウント数 (分解能)
    float enc_total_;           ///< 積算角度 [rad]
};
