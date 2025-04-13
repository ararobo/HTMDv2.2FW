#pragma once
#include <stdint.h>
#include "fdcan.h"
#include "can_driver.hpp"

class App
{
private:
    uint8_t md_id;                    // 基板のID
    bool initialized;                 // 初期化フラグ
    md_config_t md_config;            // モータードライバの設定
    int16_t target;                   // 目標値
    int16_t output;                   // 出力値
    int16_t encoder;                  // エンコーダのカウント
    uint8_t limit_switch;             // リミットスイッチの状態
    float pid_gain[3];                // PIDゲインの値
    uint16_t update_target_count;     // 目標位置の更新カウント
    uint16_t update_target_count_max; // 目標位置の更新カウントの最大値
    uint16_t timer_count;             // エンコーダー用のタイマーカウント
    uint16_t loop_count;              // 定期的な処理のカウント
    uint16_t loop_count_max;          // 定期的な処理のカウントの最大値
    uint32_t last_tick;               // 最後の制御周期のタイムスタンプ

private:
    /// @brief モーターを制御する
    void control_motor();

    /// @brief リミットスイッチの状態でモーターを制御する
    void limit_switch_control();

    /// @brief MDの設定を更新し、初期化処理を行う
    void update_md_config();

    /**
     * @brief ゲインの更新を行う
     *
     * @param gain_type PIDゲインの種類
     * 0: Pゲイン、1: Iゲイン、2: Dゲイン
     */
    void update_gain(uint8_t gain_type);

    /// @brief 制御周期に合わせて待機する
    void wait_for_next_period();

    /// @brief DIPスイッチの状態を読み取り、md_idを更新する
    void update_md_id();

public:
    App();

    /// @brief プログラムの始めに一回だけ呼び出す
    void init();

    /// @brief ループ処理
    void main_loop();

    /// @brief タイマーの割り込み処理（1ms毎）
    void timer_task();

    /// @brief CANのコールバック処理
    void can_callback_process(FDCAN_HandleTypeDef *hcan, uint32_t RxFifo0ITs);
};