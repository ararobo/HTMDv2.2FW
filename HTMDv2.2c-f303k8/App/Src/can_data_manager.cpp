/**
 * @file can_data_manager.cpp
 * @author Gento Aiba
 * @brief CAN通信のデータを管理
 * @version 0.1
 * @date 2024-05-21
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "can_data_manager.hpp"
#include "main.h"
#include "indicator.hpp"

void CANDataManager::init(uint8_t md_id_)
{
    md_id = md_id_;
    // CANのフィルタ設定
    RxFilter.FilterIdHigh = 0;                    // フィルタのIDの上位16ビット
    RxFilter.FilterIdLow = 0;                     // フィルタのIDの下位16ビット
    RxFilter.FilterMaskIdHigh = 0;                // フィルタのマスクのIDの上位16ビット
    RxFilter.FilterMaskIdLow = 0;                 // フィルタのマスクのIDの下位16ビット
    RxFilter.FilterScale = CAN_FILTERSCALE_32BIT; // フィルタのスケール
    RxFilter.FilterBank = 0;                      // フィルタのバンク
    RxFilter.FilterMode = CAN_FILTERMODE_IDMASK;  // フィルタのモード
    RxFilter.SlaveStartFilterBank = 14;           // スレーブの開始フィルタバンク
    RxFilter.FilterActivation = ENABLE;           // フィルタの有効化
    // CAN通信のスタート
    HAL_CAN_Start(&hcan);                   // CANのスタート
    HAL_CAN_ConfigFilter(&hcan, &RxFilter); // CANのフィルタの設定
    // 割り込み有効
    HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING); // FIFO0のメッセージペンディング割り込みを有効
}

void CANDataManager::classifyData(uint16_t can_id, uint8_t *rx_buffer, uint8_t data_length)
{
    uint8_t direction;
    uint8_t device;
    uint8_t device_id;
    uint8_t data_name;
    decodeCanID(can_id, &direction, &device, &device_id, &data_name);                      // CANのIDをデコード
    if (device == can_config::dev::motor_driver && direction == can_config::dir::to_slave) // モータードライバーへの通信の場合
    {
        if (device_id == md_id) // MDのIDが一致する場合
        {
            switch (data_name) // データの種類によって処理を分岐
            {
            case can_config::data_name::md::targets:
                if (data_length == can_config::dlc::md::targets_1)
                {
                    for (uint8_t i = 0; i < can_config::dlc::md::targets_1; i++)
                    {
                        buff_targets_1[i] = rx_buffer[i];
                    }
                    flag_targets_1 = true;
                }
                break;

            case can_config::data_name::md::init:
                if (data_length == can_config::dlc::md::init)
                {
                    for (uint8_t i = 0; i < can_config::dlc::md::init; i++)
                    {
                        buff_init_command[i] = rx_buffer[i];
                    }
                    flag_init = true;
                }
                break;

            case can_config::data_name::md::mode:
                if (data_length == can_config::dlc::md::mode)
                {
                    for (uint8_t i = 0; i < can_config::dlc::md::mode; i++)
                    {
                        buff_init_mode[i] = rx_buffer[i];
                    }
                    flag_mode = true;
                }
                break;

            case can_config::data_name::md::p_gain:
                if (data_length == can_config::dlc::md::p_gain)
                {
                    for (uint8_t i = 0; i < can_config::dlc::md::p_gain; i++)
                    {
                        buff_init_p_gain[i] = rx_buffer[i];
                    }
                    flag_pid[0] = true;
                }
                break;

            case can_config::data_name::md::i_gain:
                if (data_length == can_config::dlc::md::i_gain)
                {
                    for (uint8_t i = 0; i < can_config::dlc::md::i_gain; i++)
                    {
                        buff_init_i_gain[i] = rx_buffer[i];
                    }
                    flag_pid[1] = true;
                }
                break;

            case can_config::data_name::md::d_gain:
                if (data_length == can_config::dlc::md::d_gain)
                {
                    for (uint8_t i = 0; i < can_config::dlc::md::d_gain; i++)
                    {
                        buff_init_d_gain[i] = rx_buffer[i];
                    }
                    flag_pid[2] = true;
                }
                break;

            default:
                break;
            }
        }
        else if (device_id == md_id / 4) // MDのIDの4で割った商が一致する場合
        {
            if (data_name == can_config::data_name::md::targets) // データの種類が目標値の場合
            {
                if (data_length == can_config::dlc::md::targets_4) // データ長が4つの目標値の場合
                {
                    for (uint8_t i = 0; i < can_config::dlc::md::targets_4; i++)
                    {
                        buff_targets_4[i] = rx_buffer[i];
                    }
                    flag_targets_4 = true;
                }
            }
        }
    }
}

void CANDataManager::sendPacket(uint16_t can_id, uint8_t *tx_buffer, uint8_t data_length)
{
    if (data_length > 8) // データ長が8より大きい場合
    {
        indicateError(true); // エラー処理
        return;
    }
    if (0 < HAL_CAN_GetTxMailboxesFreeLevel(&hcan))
    {
        TxHeader.StdId = can_id;               // CANのID
        TxHeader.RTR = CAN_RTR_DATA;           // リモートフレーム
        TxHeader.IDE = CAN_ID_STD;             // 標準フレーム
        TxHeader.DLC = data_length;            // データ長
        TxHeader.TransmitGlobalTime = DISABLE; // グローバルタイム

        HAL_CAN_AddTxMessage(&hcan, &TxHeader, tx_buffer, &TxMailbox); // 送信
    }
    else
    {
        return;
    }
}

bool CANDataManager::getMDInit()
{
    if (flag_init) // 初期化コマンドのフラグが立っている場合
    {
        flag_init = false;             // フラグを下ろす
        if (buff_init_command[0] == 0) // 初期化コマンドが実行された場合
        {
            uint8_t tx_data[can_config::dlc::md::init] = {0};                                                                                // 送信データの設定
            uint16_t tx_id = encodeCanID(can_config::dir::to_master, can_config::dev::motor_driver, md_id, can_config::data_name::md::init); // 送信IDの設定
            sendPacket(tx_id, tx_data, can_config::dlc::md::init);                                                                           // 送信
            return true;
        }
    }
    return false;
}

bool CANDataManager::getMotorTarget(int16_t *target)
{
    if (flag_targets_4) // 目標値のフラグが立っている場合
    {
        flag_targets_4 = false;                                        // フラグを下ろす
        uint16_t target_ = uint16_t(buff_targets_4[(md_id % 4) * 2]);  // 目標値の下位8ビット
        target_ |= uint16_t(buff_targets_4[(md_id % 4) * 2 + 1]) << 8; // 目標値の上位8ビット
        *target = static_cast<int16_t>(target_);                       // 目標値をint16_tに変換
        return true;
    }
    else if (flag_targets_1)
    {
        flag_targets_1 = false;                         // フラグを下ろす
        uint16_t target_ = uint16_t(buff_targets_1[0]); // 目標値の下位8ビット
        target_ |= uint16_t(buff_targets_1[1]) << 8;    // 目標値の上位8ビット
        *target = static_cast<int16_t>(target_);        // 目標値をint16_tに変換
        return true;
    }

    return false;
}

bool CANDataManager::getMDMode(uint8_t *mode_code)
{
    if (flag_mode) // モード指定のフラグが立っている場合
    {
        flag_mode = false;                                      // フラグを下ろす
        for (uint8_t i = 0; i < can_config::dlc::md::mode; i++) // モード指定のデータ長に合わせて繰り返す
        {
            mode_code[i] = buff_init_mode[i]; // モード指定のデータをモードコードに格納
        }
        return true;
    }
    return false;
}

bool CANDataManager::getPIDGain(float *p_gain, float *i_gain, float *d_gain)
{
    if (flag_pid[0] && flag_pid[1] && flag_pid[2]) // PIDゲイン指定のフラグが立っている場合
    {
        uint32_t raw_pid_gain[3]; // PIDゲインの生データ
        // フラグを下ろす
        flag_pid[0] = false;
        flag_pid[1] = false;
        flag_pid[2] = false;
        // データを結合
        raw_pid_gain[0] = buff_init_p_gain[0] | (buff_init_p_gain[1] << 8) | (buff_init_p_gain[2] << 16) | (buff_init_p_gain[3] << 24); // 比例ゲイン
        raw_pid_gain[1] = buff_init_i_gain[0] | (buff_init_i_gain[1] << 8) | (buff_init_i_gain[2] << 16) | (buff_init_i_gain[3] << 24); // 積分ゲイン
        raw_pid_gain[2] = buff_init_d_gain[0] | (buff_init_d_gain[1] << 8) | (buff_init_d_gain[2] << 16) | (buff_init_d_gain[3] << 24); // 微分ゲイン
        // ゲインを格納
        *p_gain = static_cast<float>(raw_pid_gain[0]); // 比例ゲイン
        *i_gain = static_cast<float>(raw_pid_gain[1]); // 積分ゲイン
        *d_gain = static_cast<float>(raw_pid_gain[2]); // 微分ゲイン

        return true;
    }
    return false;
}

void CANDataManager::sendReInitPID(float p_gain, float i_gain, float d_gain)
{
    uint8_t tx_data[3][can_config::dlc::md::p_gain];
    uint32_t raw_pid_gain[3] = {static_cast<uint32_t>(p_gain), static_cast<uint32_t>(i_gain), static_cast<uint32_t>(d_gain)}; // ゲインの生データ
    // 生データを分割
    for (uint8_t i = 0; i < 3; i++) // ゲインの数だけ繰り返す
    {
        tx_data[i][0] = static_cast<uint8_t>(raw_pid_gain[i] & 0xFF);         // 下位8ビット
        tx_data[i][1] = static_cast<uint8_t>((raw_pid_gain[i] >> 8) & 0xFF);  // 8ビット右シフトして下位8ビット
        tx_data[i][2] = static_cast<uint8_t>((raw_pid_gain[i] >> 16) & 0xFF); // 16ビット右シフトして下位8ビット
        tx_data[i][3] = static_cast<uint8_t>((raw_pid_gain[i] >> 24) & 0xFF); // 24ビット右シフトして下位8ビット
    }
    uint16_t tx_id = encodeCanID(can_config::dir::to_master, can_config::dev::motor_driver, md_id, can_config::data_name::md::p_gain); // 送信IDの設定
    sendPacket(tx_id, tx_data[0], can_config::dlc::md::p_gain);                                                                        // 送信
    tx_id = encodeCanID(can_config::dir::to_master, can_config::dev::motor_driver, md_id, can_config::data_name::md::i_gain);          // 送信IDの設定
    sendPacket(tx_id, tx_data[1], can_config::dlc::md::i_gain);                                                                        // 送信
    tx_id = encodeCanID(can_config::dir::to_master, can_config::dev::motor_driver, md_id, can_config::data_name::md::d_gain);          // 送信IDの設定
    sendPacket(tx_id, tx_data[2], can_config::dlc::md::d_gain);                                                                        // 送信
}

void CANDataManager::sendReInitMode(uint8_t *mode_code)
{
    uint16_t tx_id = encodeCanID(can_config::dir::to_master, can_config::dev::motor_driver, md_id, can_config::data_name::md::mode); // 送信IDの設定
    sendPacket(tx_id, mode_code, can_config::dlc::md::mode);                                                                         // 送信
}

void CANDataManager::onReceiveTask(CAN_HandleTypeDef *hcan_)
{
    if (HAL_CAN_GetRxMessage(hcan_, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) // 正常に受信できた場合
    {
        classifyData(RxHeader.StdId, RxData, RxHeader.DLC); // 受信データの分類
    }
}

void CANDataManager::sendSensorLimit(bool limit_switch1, bool limit_switch2)
{
    uint8_t tx_data[can_config::dlc::md::limit] = {0};
    tx_data[0] |= uint8_t(limit_switch1) & 0b1;
    tx_data[0] |= (uint8_t(limit_switch2) << 1) & 0b10;                                                                                // 送信データ
    uint16_t tx_id = encodeCanID(can_config::dir::to_master, can_config::dev::motor_driver, md_id, can_config::data_name::md::sensor); // 送信IDの設定
    sendPacket(tx_id, tx_data, can_config::dlc::md::limit);                                                                            // 送信
}

void CANDataManager::sendSensorLimitAndEncoder(bool limit_switch1, bool limit_switch2, int16_t encoder_value)
{
    uint8_t tx_data[can_config::dlc::md::limit_encoder] = {0};
    tx_data[0] |= uint8_t(limit_switch1) & 0b1;
    tx_data[0] |= (uint8_t(limit_switch2) << 1) & 0b10;                                                                                // リミットスイッチ
    tx_data[1] = static_cast<uint8_t>(static_cast<uint16_t>(encoder_value) & 0xFF);                                                    // エンコーダーの下位8ビット
    tx_data[2] = static_cast<uint8_t>(static_cast<uint16_t>(encoder_value) >> 8);                                                      // エンコーダーの上位8ビット
    uint16_t tx_id = encodeCanID(can_config::dir::to_master, can_config::dev::motor_driver, md_id, can_config::data_name::md::sensor); // 送信IDの設定
    sendPacket(tx_id, tx_data, can_config::dlc::md::limit_encoder);                                                                    // 送信
}

void CANDataManager::sendSensorLimitEncoderAndCurrent(bool limit_switch1, bool limit_switch2, int16_t encoder_value, uint16_t current)
{
    uint8_t tx_data[can_config::dlc::md::limit_encoder_current] = {0};
    tx_data[0] |= uint8_t(limit_switch1) & 0b1;
    tx_data[0] |= (uint8_t(limit_switch2) << 1) & 0b10;                                                                                // リミットスイッチ
    tx_data[1] = static_cast<uint8_t>(static_cast<uint16_t>(encoder_value) & 0xFF);                                                    // エンコーダーの下位8ビット
    tx_data[2] = static_cast<uint8_t>(static_cast<uint16_t>(encoder_value) >> 8);                                                      // エンコーダーの上位8ビット
    tx_data[3] = static_cast<uint8_t>(current & 0xFF);                                                                                 // 電流の下位8ビット
    tx_data[4] = static_cast<uint8_t>(current >> 8);                                                                                   // 電流の上位8ビット
    uint16_t tx_id = encodeCanID(can_config::dir::to_master, can_config::dev::motor_driver, md_id, can_config::data_name::md::sensor); // 送信IDの設定
    sendPacket(tx_id, tx_data, can_config::dlc::md::limit_encoder_current);                                                            // 送信
}

void CANDataManager::sendStateMD(uint8_t state_code)
{
    uint8_t tx_data[can_config::dlc::md::state] = {state_code};                                                                        // 送信データ
    uint16_t tx_id = encodeCanID(can_config::dir::to_master, can_config::dev::motor_driver, md_id, can_config::data_name::md::status); // 送信IDの設定
    sendPacket(tx_id, tx_data, can_config::dlc::md::state);                                                                            // 送信
}

void CANDataManager::sendStateAndTemp(uint8_t state_code, uint8_t state_temp)
{
    uint8_t tx_data[can_config::dlc::md::state_temp] = {state_code, state_temp};                                                       // 送信データ
    uint16_t tx_id = encodeCanID(can_config::dir::to_master, can_config::dev::motor_driver, md_id, can_config::data_name::md::status); // 送信IDの設定
    sendPacket(tx_id, tx_data, can_config::dlc::md::state_temp);                                                                       // 送信
}

uint16_t CANDataManager::encodeCanID(uint8_t dir, uint8_t dev, uint8_t device_id, uint8_t data_name)
{
    return (dir << 10) | (dev << 7) | (device_id << 3) | data_name;
}

void CANDataManager::decodeCanID(uint16_t can_id, uint8_t *dir, uint8_t *dev, uint8_t *device_id, uint8_t *data_name)
{
    *dir = (can_id & 0x400) >> 10;
    *dev = (can_id & 0x380) >> 7;
    *device_id = (can_id & 0x78) >> 3;
    *data_name = (can_id & 0x7);
}