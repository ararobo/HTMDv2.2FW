/**
 * @file md_config.hpp
 * @author aiba-gento
 * @brief MDの設定を扱う型
 * @version 3.1
 * @date 2025-08-03
 *
 * @copyright Copyright (c) 2025
 *
 */
#pragma once
#include <stdint.h>

union md_config_t
{
    struct
    {
        uint16_t max_output;           // 最大出力 duty
        uint8_t max_acceleration;      // 台形制御の最大加速 duty/ms
        uint8_t control_period;        // 制御周期 ms
        uint8_t encoder_period;        // エンコーダーのサンプリング周期 ms
        uint8_t encoder_type;          // 0:無し、1:インクリメンタル（速度）、2:アブソリュート、3:インクリメンタル（トータル）
        uint8_t limit_switch_behavior; // リミットスイッチの動作設定
        uint8_t option;                // 基板の固有機能や使用用途に合わせて決定（未定義）
    } __attribute__((__packed__));
    uint8_t code[8]; // 送信バイト配列
} __attribute__((__packed__));