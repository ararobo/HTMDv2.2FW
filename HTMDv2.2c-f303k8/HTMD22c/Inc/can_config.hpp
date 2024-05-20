/**
 * @file can_id.hpp
 * @author Gento Aiba
 * @brief CAN通信の設定
 * @version 3.0
 * @date 2024-05-20
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once
#include <stdint.h>

namespace can_config
{
    namespace dir // 通信方向
    {
        static constexpr uint16_t to_slave = 0;  // PCやマザーへの通信
        static constexpr uint16_t to_master = 1; // MDやドライバへの通信
    }
    namespace dev // デバイスの種類
    {
        static constexpr uint16_t common = 0;          // 共通
        static constexpr uint16_t motor_driver = 1;    // モータードライバ
        static constexpr uint16_t servo_driver = 2;    // サーボドライバ
        static constexpr uint16_t solenoid_driver = 3; // ソレノイドドライバ
        static constexpr uint16_t led_driver = 4;      // LEDドライバ
    }
    namespace data_name // データの種類
    {
        namespace md
        {
            static constexpr uint16_t targets = 0; // 目標値
            static constexpr uint16_t init = 1;    // 初期化コマンド
            static constexpr uint16_t mode = 2;    // モード
            static constexpr uint16_t p_gain = 3;  // PID制御の比例ゲイン
            static constexpr uint16_t i_gain = 4;  // PID制御の積分ゲイン
            static constexpr uint16_t d_gain = 5;  // PID制御の微分ゲイン
            static constexpr uint16_t sensor = 6;  // センサー(リミットスイッチ、エンコーダ、電流センサー)
            static constexpr uint16_t status = 7;  // ステータス
        }
        namespace servo
        {
            static constexpr uint16_t targets = 0; // 目標値
            static constexpr uint16_t init = 1;    // 初期化コマンド
            static constexpr uint16_t freq = 2;    // 周波数
        }
        namespace solenoid
        {
            static constexpr uint16_t targets = 0; // 目標値
            static constexpr uint16_t init = 1;    // 初期化コマンド
        }
        namespace led
        {
            static constexpr uint16_t targets = 0; // 目標値
            static constexpr uint16_t init = 1;    // 初期化コマンド
        }
    }
    namespace dlc // Data Length Code
    {
        namespace md
        {
            static constexpr uint8_t targets_1 = 2;             // 一つのモーターの目標値
            static constexpr uint8_t targets_4 = 8;             // 4つのモーターの目標値
            static constexpr uint8_t init = 1;                  // 初期化コマンド
            static constexpr uint8_t mode = 8;                  // モード
            static constexpr uint8_t p_gain = 6;                // PID制御の比例ゲイン
            static constexpr uint8_t i_gain = 6;                // PID制御の積分ゲイン
            static constexpr uint8_t d_gain = 6;                // PID制御の微分ゲイン
            static constexpr uint8_t limit = 1;                 // リミットスイッチの状態
            static constexpr uint8_t limit_encoder = 3;         // エンコーダとリミットスイッチの状態
            static constexpr uint8_t limit_encoder_current = 7; // エンコーダとリミットスイッチの状態と電流
            static constexpr uint8_t state = 1;                 // MDの状態
            static constexpr uint8_t state_temp = 3;            // MDの状態と温度
        }
        namespace servo
        {
            static constexpr uint8_t targets_1 = 2; // 一つのサーボの目標値
            static constexpr uint8_t targets_4 = 8; // 4つのサーボの目標値
            static constexpr uint8_t init = 1;      // 初期化コマンド
            static constexpr uint8_t freq = 1;      // 周波数
        }
        namespace solenoid
        {
            static constexpr uint8_t targets = 1; // ソレノイドx8の目標値(1bitづつ対応)
            static constexpr uint8_t init = 1;    // 初期化コマンド
        }
        namespace led
        {
            static constexpr uint8_t targets = 1; // LEDの点灯パターン
            static constexpr uint8_t init = 1;    // 初期化コマンド
        }
    }
}

/**
 * @brief CANのIDをデコードする
 *
 * @param can_id CANのID(入力)
 * @param dir 通信方向
 * @param dev デバイスの種類
 * @param device_id デバイスのID
 * @param data_name データの種類
 */
void decodeCanID(uint16_t can_id, uint8_t *dir, uint8_t *dev, uint8_t *device_id, uint8_t *data_name)
{
    *dir = (can_id & 0x400) >> 10;
    *dev = (can_id & 0x380) >> 7;
    *device_id = (can_id & 0x78) >> 3;
    *data_name = (can_id & 0x7);
}

/**
 * @brief CANのIDをエンコードする
 *
 * @param dir 通信方向
 * @param dev デバイスの種類
 * @param device_id デバイスのID
 * @param data_name データの種類
 * @return uint16_t CANのID
 */
uint16_t encodeCanID(uint8_t dir, uint8_t dev, uint8_t device_id, uint8_t data_name)
{
    return (dir << 10) | (dev << 7) | (device_id << 3) | data_name;
}