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

bool CANDataManager::getMDInit()
{
    if (flag_init_command) // 初期化コマンドのフラグが立っている場合
    {
        flag_init_command = false;                                           // フラグを下ろす
        if (buff_init_command[0] == can_configure::manage::command::do_init) // 初期化コマンドが実行された場合
        {
            uint8_t tx_data[can_configure::manage::dlc::init] = {can_configure::manage::command::success}; // 送信データの設定
            sendPacket(can_configure::manage::id::re_init, tx_data, can_configure::manage::dlc::re_init);  // 送信
            return true;
        }
    }
    return false;
}

bool CANDataManager::getMotorTarget(int16_t *target)
{
    if (flag_targets) // 目標値のフラグが立っている場合
    {
        flag_targets = false;                                        // フラグを下ろす
        uint16_t target_ = uint16_t(buff_targets[(md_id % 4) * 2]);  // 目標値の下位8ビット
        target_ |= uint16_t(buff_targets[(md_id % 4) * 2 + 1]) << 8; // 目標値の上位8ビット
        *target = static_cast<int16_t>(target_);                     // 目標値をint16_tに変換
        return true;
    }
    return false;
}

bool CANDataManager::getMDMode(uint8_t *mode_code)
{
    if (flag_init_mode) // モード指定のフラグが立っている場合
    {
        flag_init_mode = false;                                        // フラグを下ろす
        for (uint8_t i = 0; i < can_configure::manage::dlc::mode; i++) // モード指定のデータ長に合わせて繰り返す
        {
            mode_code[i] = buff_init_mode[i]; // モード指定のデータをモードコードに格納
        }
        return true;
    }
    return false;
}

bool CANDataManager::getPIDGain(float *p_gain, float *i_gain, float *d_gain)
{
    if (flag_init_pid) // PIDゲイン指定のフラグが立っている場合
    {
        flag_init_pid = false;                                                                            // フラグを下ろす
        *p_gain = float(uint16_t(buff_init_pid[1]) | uint16_t(buff_init_pid[2] << 8)) / PID_GAIN_QUALITY; // 比例ゲイン
        *i_gain = float(uint16_t(buff_init_pid[3]) | uint16_t(buff_init_pid[4] << 8)) / PID_GAIN_QUALITY; // 積分ゲイン
        *d_gain = float(uint16_t(buff_init_pid[5]) | uint16_t(buff_init_pid[6] << 8)) / PID_GAIN_QUALITY; // 微分ゲイン
        return true;
    }
    return false;
}

void CANDataManager::sendReInitPID(float p_gain, float i_gain, float d_gain)
{
    uint8_t tx_data[7];         // 送信データ
    p_gain *= PID_GAIN_QUALITY; // PIDゲインを10000倍
    i_gain *= PID_GAIN_QUALITY;
    d_gain *= PID_GAIN_QUALITY;
    tx_data[0] = md_id;                                                                                 // MDのID
    tx_data[1] = uint16_t(p_gain);                                                                      // 比例ゲインの下位8ビット
    tx_data[2] = uint16_t(p_gain) >> 8;                                                                 // 比例ゲインの上位8ビット
    tx_data[3] = uint16_t(i_gain);                                                                      // 積分ゲインの下位8ビット
    tx_data[4] = uint16_t(i_gain) >> 8;                                                                 // 積分ゲインの上位8ビット
    tx_data[5] = uint16_t(d_gain);                                                                      // 微分ゲインの下位8ビット
    tx_data[6] = uint16_t(d_gain) >> 8;                                                                 // 微分ゲインの上位8ビット
    sendPacket(can_configure::manage::id::re_pid + md_id, tx_data, can_configure::manage::dlc::re_pid); // 送信
}

void CANDataManager::sendReInitMode(uint8_t *mode_code)
{
    sendPacket(can_configure::manage::id::re_mode + md_id, mode_code, can_configure::manage::dlc::re_mode); // 送信
}

void CANDataManager::onReceiveTask(CAN_HandleTypeDef *hcan_)
{
    if (HAL_CAN_GetRxMessage(hcan_, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) // 正常に受信できた場合
    {
        uint16_t rx_id = RxHeader.StdId & 0x7F0;                                              // CANのIDをマスクして全MD共通で種類分けする
        uint8_t rx_md_id = RxHeader.StdId & 0x00F;                                            // CANのIDをマスクしてMDのIDを取得
        if (uint16_t(RxHeader.StdId) == (can_configure::control::id::md_targets + md_id / 4)) // MDのIDに対応したモーターの目標値データであった場合
        {
            if (RxHeader.DLC == can_configure::control::dlc::md_targets) // 受信したデータの長さが正しい場合
            {
                for (uint8_t i = 0; i < can_configure::control::dlc::md_targets; i++) // データ長に合わせて繰り返す
                {
                    buff_targets[i] = RxData[i]; // 受信データを目標値バッファに格納
                }
                flag_targets = true; // 目標値のフラグを立てる
            }
            else
            {
                indicateError(true); // エラー処理
            }
        }
        else if (rx_md_id == md_id) // 対応するMDのIDである場合
        {
            if (rx_id == can_configure::manage::id::init) // 初期化コマンドのCANのIDである場合
            {
                if (RxHeader.DLC == can_configure::manage::dlc::init) // 受信したデータの長さが正しい場合
                {
                    for (uint8_t i = 0; i < can_configure::manage::dlc::init; i++) // データ長に合わせて繰り返す
                    {
                        buff_init_command[i] = RxData[i]; // 受信データを初期化コマンドバッファに格納
                    }
                    flag_init_command = true; // 初期化コマンドのフラグを立てる
                }
                else
                {
                    indicateError(true); // エラー処理
                }
            }
            else if (rx_id == can_configure::manage::id::mode) // モード指定のCANのIDである場合
            {
                if (RxHeader.DLC == can_configure::manage::dlc::mode) // 受信したデータの長さが正しい場合
                {
                    for (uint8_t i = 0; i < can_configure::manage::dlc::mode; i++) // データ長に合わせて繰り返す
                    {
                        buff_init_mode[i] = RxData[i]; // 受信データをモード指定バッファに格納
                    }
                    flag_init_mode = true; // モード指定のフラグを立てる
                }
                else
                {
                    indicateError(true); // エラー処理
                }
            }
            else if (rx_id == can_configure::manage::id::pid) // PIDゲイン指定のCANのIDである場合
            {
                if (RxHeader.DLC == can_configure::manage::dlc::pid) // 受信したデータの長さが正しい場合
                {
                    for (uint8_t i = 0; i < can_configure::manage::dlc::pid; i++) // データ長に合わせて繰り返す
                    {
                        buff_init_pid[i] = RxData[i]; // 受信データをPIDゲイン指定バッファに格納
                    }
                    flag_init_pid = true; // PIDゲイン指定のフラグを立てる
                }
                else
                {
                    indicateError(true); // エラー処理
                }
            }
        }
    }
}

void CANDataManager::sendSensorLimit(bool limit_switch1, bool limit_switch2)
{
    uint8_t tx_data[can_configure::sensor::dlc::limit] = {0};
    tx_data[0] |= uint8_t(limit_switch1) & 0b1;
    tx_data[0] |= (uint8_t(limit_switch2) << 1) & 0b10;                                               // 送信データ
    sendPacket(can_configure::sensor::id::limit + md_id, tx_data, can_configure::sensor::dlc::limit); // 送信
}

void CANDataManager::sendSensorEncoder(int16_t encoder_value)
{
    uint8_t tx_data[can_configure::sensor::dlc::encoder] = {static_cast<uint8_t>(static_cast<uint16_t>(encoder_value) & 0xFF), static_cast<uint8_t>(static_cast<uint16_t>(encoder_value) >> 8)}; // 送信データ
    sendPacket(can_configure::sensor::id::encoder + md_id, tx_data, can_configure::sensor::dlc::encoder);                                                                                        // 送信
}

void CANDataManager::sendSensorCurrent(int16_t current)
{
    uint8_t tx_data[can_configure::sensor::dlc::current] = {static_cast<uint8_t>(static_cast<uint16_t>(current) & 0xFF), static_cast<uint8_t>(static_cast<uint16_t>(current) >> 8)}; // 送信データ
    sendPacket(can_configure::sensor::id::current + md_id, tx_data, can_configure::sensor::dlc::current);                                                                            // 送信
}

void CANDataManager::sendSensorAll(bool limit_switch1, bool limit_switch2, int16_t encoder_value, int16_t current)
{
    uint8_t tx_data[can_configure::sensor::dlc::all] = {0};
    tx_data[0] |= uint8_t(limit_switch1) & 0b1;
    tx_data[0] |= (uint8_t(limit_switch2) << 1) & 0b10;                                           // リミットスイッチ
    tx_data[1] = static_cast<uint8_t>(static_cast<uint16_t>(encoder_value) & 0xFF);               // エンコーダーの下位8ビット
    tx_data[2] = static_cast<uint8_t>(static_cast<uint16_t>(encoder_value) >> 8);                 // エンコーダーの上位8ビット
    tx_data[3] = static_cast<uint8_t>(static_cast<uint16_t>(current) & 0xFF);                     // 電流の下位8ビット
    tx_data[4] = static_cast<uint8_t>(static_cast<uint16_t>(current) >> 8);                       // 電流の上位8ビット
    sendPacket(can_configure::sensor::id::all + md_id, tx_data, can_configure::sensor::dlc::all); // 送信
}

void CANDataManager::sendStateMD(uint8_t state_code)
{
    uint8_t tx_data[can_configure::state::dlc::md] = {state_code};                            // 送信データ
    sendPacket(can_configure::state::id::md + md_id, tx_data, can_configure::state::dlc::md); // 送信
}

void CANDataManager::sendStateTemp(uint8_t state_temp)
{
    uint8_t tx_data[can_configure::state::dlc::temp] = {state_temp};                              // 送信データ
    sendPacket(can_configure::state::id::temp + md_id, tx_data, can_configure::state::dlc::temp); // 送信
}

void CANDataManager::sendStateAll(uint8_t state_code, uint8_t state_temp)
{
    uint8_t tx_data[can_configure::state::dlc::all] = {state_code, state_temp};                 // 送信データ
    sendPacket(can_configure::state::id::all + md_id, tx_data, can_configure::state::dlc::all); // 送信
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
        indicateError(true); // エラー処理
    }
}