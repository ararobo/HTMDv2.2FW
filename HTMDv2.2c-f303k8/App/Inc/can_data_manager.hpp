/**
 * @file can_data_manager.hpp
 * @author Gento Aiba
 * @brief CAN通信のデータ管理クラス
 * @version 0.1
 * @date 2024-05-21
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "can.h"
#include "can_config.hpp"

class CANDataManager
{
private:
    // param
    uint8_t md_id;
    // buffer
    uint8_t buff_init_p_gain[can_config::dlc::md::p_gain];
    uint8_t buff_init_i_gain[can_config::dlc::md::i_gain];
    uint8_t buff_init_d_gain[can_config::dlc::md::d_gain];
    uint8_t buff_init_mode[can_config::dlc::md::mode];
    uint8_t buff_init_command[can_config::dlc::md::init];
    uint8_t buff_targets_4[can_config::dlc::md::targets_4];
    uint8_t buff_targets_1[can_config::dlc::md::targets_1];
    // flag
    bool flag_pid[3];
    bool flag_mode;
    bool flag_init;
    bool flag_targets_4;
    bool flag_targets_1;

public:
    /**
     * @brief MDのIDを設定してCAN通信を開始する。
     *
     * @param md_id_ MDのID
     */
    void init(uint8_t md_id_);

    /**
     * @brief 受信データを解析して、データを分類する
     *
     * @param can_id CANのID
     * @param rx_buffer 受信データのバッファ
     * @param data_length 受信データの長さ
     */
    void classifyData(uint16_t can_id, uint8_t *rx_buffer, uint8_t data_length);

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
    void onReceiveTask(CAN_HandleTypeDef *hcan_);

    /**
     * @brief リミットスイッチの状態を送信
     *
     * @param limit_switch1 １つ目のリミットスイッチ
     * @param limit_switch2 ２つ目のリミットスイッチ
     */
    void sendSensorLimit(bool limit_switch1, bool limit_switch2);

    /**
     * @brief リミットスイッチの状態とエンコーダの値を送信
     *
     * @param limit_switch1 １つ目のリミットスイッチ
     * @param limit_switch2 ２つ目のリミットスイッチ
     * @param encoder_value エンコーダの値
     */
    void sendSensorLimitAndEncoder(bool limit_switch1, bool limit_switch2, int16_t encoder_value);

    /**
     * @brief リミットスイッチの状態、エンコーダの値、電流センサの値を送信
     *
     * @param limit_switch1 １つ目のリミットスイッチ
     * @param limit_switch2 ２つ目のリミットスイッチ
     * @param encoder_value エンコーダの値
     * @param current 電流センサの値
     */
    void sendSensorLimitEncoderAndCurrent(bool limit_switch1, bool limit_switch2, int16_t encoder_value, uint16_t current);

    /**
     * @brief MDの状態を送信
     *
     * @param state_code 状態コード
     */
    void sendStateMD(uint8_t state_code);

    /**
     * @brief 全てのMDの状態を送信
     *
     * @param state_code 状態コード
     * @param state_temp 状態温度
     */
    void sendStateAndTemp(uint8_t state_code, uint8_t state_temp);

private:
    /**
     * @brief 簡単にパケットを送信する
     *
     * @param can_id CANのID
     * @param tx_buffer 送信するデータの入った変数のポインタ（配列）
     * @param data_length 送信データ長
     */
    void sendPacket(uint16_t can_id, uint8_t *tx_buffer, uint8_t data_length);

    /**
     * @brief CANのIDをデコードする
     *
     * @param can_id CANのID(入力)
     * @param dir 通信方向
     * @param dev デバイスの種類
     * @param device_id デバイスのID
     * @param data_name データの種類
     */
    void decodeCanID(uint16_t can_id, uint8_t *dir, uint8_t *dev, uint8_t *device_id, uint8_t *data_name);

    /**
     * @brief CANのIDをエンコードする
     *
     * @param dir 通信方向
     * @param dev デバイスの種類
     * @param device_id デバイスのID
     * @param data_name データの種類
     * @return uint16_t CANのID
     */
    uint16_t encodeCanID(uint8_t dir, uint8_t dev, uint8_t device_id, uint8_t data_name);

private:
    // 内部使用データ
    CAN_RxHeaderTypeDef RxHeader;
    CAN_FilterTypeDef RxFilter;
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailbox;
    uint8_t RxData[8];
};