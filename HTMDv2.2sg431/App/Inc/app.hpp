#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "can_data_manager.hpp"
#include <string>
#include <vector>
class App
{
private:
    uint8_t md_id = 0;                      // MDのID
    uint8_t control_cycle = 5;              // 制御周期
    float Kp = 0.0f;                        // PID制御の比例ゲイン
    float Ki = 0.0f;                        // PID制御の積分ゲイン
    float Kd = 0.0f;                        // PID制御の微分ゲイン
    uint64_t no_update_max = 100;           // CANで目標値が更新されない回数の最大（CANが死んで何回制御周期が回ったら出力をゼロにするか）
    float motor_transfer_cofficient = 1.0f; // モーターの伝達関数
    uint16_t current = 0;                   // 電流センサーの値
    int16_t temp = 0;                       // 温度センサーの値

public:
    /**
     * @brief MDの初期化
     *
     */
    void init();

    /**
     * @brief メインループで実行する関数
     *
     */
    void mainLoop();

    /**
     * @brief CAN通信のコールバック関数
     *
     * @param hcan_ CANのハンドラ
     */
    void CANCallbackProcess(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);

    /**
     * @brief タイマー処理のコールバック関数
     *
     */
    void timerTask();

    /**
     * @brief DIPスイッチからMD_IDを計算して代入する
     *
     * @param md_id_ MD_IDのポインタ
     */
    void getMDIdFromDispSW(uint8_t *md_id_);

    /**
     * @brief 自作printf関数(UART)
     *
     * @tparam Args
     * @param fmt
     * @param args
     */
    template <typename... Args>
    void serial_printf(const std::string &fmt, Args... args);

    void resetControlVal();

private:
    int16_t output;                  // 出力
    int16_t encoder_value = 0;       // エンコーダーの値
    bool limit_switch_state = false; // リミットスイッチの状態
    int16_t target = 0;              // 目標値
    uint64_t no_update_count = 0;    // 目標値が更新されないで何回制御したか
    uint8_t control_count = 0;       // タイマーの回数をカウントして、制御周期に合わせる
    bool initialized = false;        // 初期化されたかどうか
};