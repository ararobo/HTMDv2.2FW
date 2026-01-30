#pragma once

namespace common::config {

// --- システム制御周期設定 ---
/**
 * @brief メイン制御ループの周波数 [Hz]
 */
static constexpr float CONTROL_LOOP_FREQ_HZ = 1000.0f;

/**
 * @brief メイン制御ループの周期 [s]
 */
static constexpr float CONTROL_LOOP_DT = 1.0f / CONTROL_LOOP_FREQ_HZ;

// --- モーター制御デフォルト設定 ---

struct DefaultPID {
    static constexpr float Kp = 1.0f;
    static constexpr float Ki = 0.0f;
    static constexpr float Kd = 0.0f;
    static constexpr float IntegralLimit = 100.0f;
    static constexpr float OutputLimit = 0.95f; // Duty比 (0.0~1.0)
};

/**
 * @brief 出力変化率リミッターのデフォルト値 [(Duty or Val)/s]
 */
static constexpr float DEFAULT_ACCEL_LIMIT = 5.0f; 

// --- ハードウェア/エンコーダ設定 ---

/**
 * @brief エンコーダの1回転あたりのカウント数 (CPR)
 * 4逓倍後の値を設定すること (例: 512 PPR * 4 = 2048)
 */
static constexpr float DEFAULT_ENCODER_CPR = 2048.0f;

}  // namespace common::config
