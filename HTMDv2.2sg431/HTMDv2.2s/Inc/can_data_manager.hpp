#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "fdcan.h"
#include "can_configure.hpp"

class CANDataManager
{
private:
    // param
    uint8_t md_id;
    static constexpr float PID_GAIN_QUALITY = 10000.0f;
    // buffer
    uint8_t buff_init_pid[can_configure::manage::dlc::pid];
    uint8_t buff_init_mode[can_configure::manage::dlc::mode];
    uint8_t buff_init_command[can_configure::manage::dlc::init];
    uint8_t buff_targets[can_configure::control::dlc::md_targets];
    // flag
    bool flag_init_pid;
    bool flag_init_mode;
    bool flag_init_command;
    bool flag_targets;

public:
    /**
     * @brief MDのIDを設定してCAN通信を開始する。
     *
     * @param md_id_ MDのID
     */
    void init(uint8_t md_id_);

    /**
     * @brief MDの初期化コマンドを取得する
     *
     * @return true 初期化コマンドがある
     * @return false 初期化コマンドが無い
     */
    bool getMDInit();

    /**
     * @brief モーターの目標値（出力）を取得
     *
     * @param target モーターの目標値を格納したい変数のポインタ
     * @return true 新しい目標値に更新した
     * @return false 新しい目標値が無い
     */
    bool getMotorTarget(int16_t *target);

    /**
     * @brief MDのモードを取得
     *
     * @param mode_code モードのコードを格納したい変数のポインタ
     * @return true モードを更新した
     * @return false モードを更新していない
     */
    bool getMDMode(uint8_t *mode_code);

    /**
     * @brief PIDゲインを取得
     *
     * @param p_gain 比例ゲインを格納したい変数のポインタ
     * @param i_gain 積分ゲインを格納したい変数のポインタ
     * @param d_gain 微分ゲインを格納したい変数のポインタ
     * @return true ゲインを更新した
     * @return false ゲインを更新していない
     */
    bool getPIDGain(float *p_gain, float *i_gain, float *d_gain);

    /**
     * @brief PIDゲインが更新されたことを送信
     *
     * @param p_gain 比例ゲイン
     * @param i_gain 積分ゲイン
     * @param d_gain 微分ゲイン
     */
    void sendReInitPID(float p_gain, float i_gain, float d_gain);

    /**
     * @brief モードが変更されたことを送信
     *
     * @param mode_code モードのデータ
     */
    void sendReInitMode(uint8_t *mode_code);

    /**
     * @brief 受信コールバック関数の内部処理
     *
     * @param hcan_ よくわかんない
     */
    void onReceiveTask(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);

    void sendSensorLimit(bool limit_switch1, bool limit_switch2);

    void sendSensorEncoder(int16_t encoder_value);

    void sendSensorCurrent(int16_t current);

    void sendSensorLimitAndEncoder(bool limit_switch1, bool limit_switch2, int16_t encoder_value);

    void sendSensorLimitAndCurrent(int16_t encoder_value, int16_t current);

    void sendSensorAll(bool limit_switch1, bool limit_switch2, int16_t encoder_value, int16_t current);

    void sendStateMD(uint8_t state_code);

    void sendStateAll(uint8_t state_code, uint8_t state_temp);

private:
    /**
     * @brief 簡単にパケットを送信する
     *
     * @param can_id CANのID
     * @param tx_buffer 送信するデータの入った変数のポインタ（配列）
     * @param data_length 送信データ長
     */
    void sendPacket(uint16_t can_id, uint8_t *tx_buffer, uint8_t data_length);

private:
    // 内部使用データ
    FDCAN_RxHeaderTypeDef RxHeader;
    FDCAN_FilterTypeDef RxFilter;
    FDCAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailbox;
    uint8_t RxData[8];
};