#pragma once
#include <stdint.h>
#include "can_driver.hpp"

class App
{
private:
    uint8_t md_id;                    // 基板のID
    bool initialized;                 // 初期化フラグ
    md_config_t md_config;            // モータードライバの設定
    int16_t target;                   // 目標値
    uint8_t limit_switch;             // リミットスイッチの状態
    float pid_gain[3];                // PIDゲインの値
    uint16_t update_target_count;     // 目標位置の更新カウント
    uint16_t update_target_count_max; // 目標位置の更新カウントの最大値
    uint16_t timer_count;             // エンコーダー用のタイマーカウント
    uint16_t loop_count;              // 定期的な処理のカウント
    uint16_t loop_count_max;          // 定期的な処理のカウントの最大値
    uint32_t last_tick;               // 最後の制御周期のタイムスタンプ
    bool limit_stop;                  // リミットスイッチによる停止フラグ

private:
    /// @brief モーターを制御する
    void control_motor();

    /**
     * @brief リミットスイッチによるモーターの制御
     *
     * @return true モーターを停止する
     * @return false モーターを停止しない
     */
    bool limit_switch_control();

    /**
     * @brief モータードライバの設定を更新する
     *
     */
    void update_md_config();

    /**
     * @brief ゲインの更新を行う
     *
     * @param gain_type PIDゲインの種類
     * 0: Pゲイン、1: Iゲイン、2: Dゲイン
     */
    void update_gain(uint8_t gain_type);

    /**
     * @brief 制御周期に合わせて待機する
     *
     */
    void wait_for_next_period();

    /**
     * @brief DIPスイッチの状態を取得し、MDのIDを更新する
     *
     */
    void update_md_id();

public:
    App();

    /**
     * @brief プログラムのはじめに1回だけ呼び出す
     *
     */
    void setup();

    /**
     * @brief プログラムのメインループ
     *
     */
    void loop();

    /**
     * @brief エンコーダー用のタイマー
     *
     */
    void timer_callback();

    /**
     * @brief CANのコールバック処理
     *
     * @param hcan CANハンドル
     */
    void can_callback_process(CAN_HandleTypeDef *hcan);
};