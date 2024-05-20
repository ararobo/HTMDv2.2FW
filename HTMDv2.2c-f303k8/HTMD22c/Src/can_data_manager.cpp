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
        flag_init_command = false;     // フラグを下ろす
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
    if (flag_targets) // 目標値のフラグが立っている場合
    {
        flag_targets = false;                                          // フラグを下ろす
        uint16_t target_ = uint16_t(buff_targets_8[(md_id % 4) * 2]);  // 目標値の下位8ビット
        target_ |= uint16_t(buff_targets_8[(md_id % 4) * 2 + 1]) << 8; // 目標値の上位8ビット
        *target = static_cast<int16_t>(target_);                       // 目標値をint16_tに変換
        return true;
    }
    return false;
}

bool CANDataManager::getMDMode(uint8_t *mode_code)
{
    if (flag_init_mode) // モード指定のフラグが立っている場合
    {
        flag_init_mode = false;                                 // フラグを下ろす
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
    if (flag_init_pid[0] && flag_init_pid[1] && flag_init_pid[2]) // PIDゲイン指定のフラグが立っている場合
    {
        uint32_t raw_pid_gain[3]; // PIDゲインの生データ
        // フラグを下ろす
        flag_init_pid[0] = false;
        flag_init_pid[1] = false;
        flag_init_pid[2] = false;
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
        uint16_t rx_id = RxHeader.StdId; // 受信したCANのID
        uint8_t direction;
        uint8_t device;
        uint8_t device_id;
        uint8_t data_name;
        decodeCanID(rx_id, &direction, &device, &device_id, &data_name);               // CANのIDをデコード
        if (device_id == md_id / 4 && data_name == can_config::data_name::md::targets) // 目標値のCANのIDである場合
        {
            if (RxHeader.DLC == can_configure::control::dlc::md_targets) // 受信したデータの長さが正しい場合
            {
                for (uint8_t i = 0; i < can_configure::control::dlc::md_targets; i++) // データ長に合わせて繰り返す
                {
                    buff_targets_8[i] = RxData[i]; // 受信データを目標値バッファに格納
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
            if (rx_id == can_configure::manage::id::md_mode) // モード指定のCANのIDである場合
            {
                if (RxHeader.DLC == can_configure::manage::dlc::md_mode) // 受信したデータの長さが正しい場合
                {
                    for (uint8_t i = 0; i < can_configure::manage::dlc::md_mode; i++) // データ長に合わせて繰り返す
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
            else if (rx_id == can_configure::manage::id::p_gain) // 比例ゲイン指定のCANのIDである場合
            {
                if (RxHeader.DLC == can_configure::manage::dlc::pid) // 受信したデータの長さが正しい場合
                {
                    for (uint8_t i = 0; i < can_configure::manage::dlc::pid; i++) // データ長に合わせて繰り返す
                    {
                        buff_init_p_gain[i] = RxData[i]; // 受信データを比例ゲイン指定バッファに格納
                    }
                    flag_init_pid[0] = true; // 比例ゲイン指定のフラグを立てる
                }
                else
                {
                    indicateError(true); // エラー処理
                }
            }
            else if (rx_id == can_configure::manage::id::i_gain) // 積分ゲイン指定のCANのIDである場合
            {
                if (RxHeader.DLC == can_configure::manage::dlc::pid) // 受信したデータの長さが正しい場合
                {
                    for (uint8_t i = 0; i < can_configure::manage::dlc::pid; i++) // データ長に合わせて繰り返す
                    {
                        buff_init_i_gain[i] = RxData[i]; // 受信データを積分ゲイン指定バッファに格納
                    }
                    flag_init_pid[1] = true; // 積分ゲイン指定のフラグを立てる
                }
                else
                {
                    indicateError(true); // エラー処理
                }
            }
            else if (rx_id == can_configure::manage::id::d_gain) // 微分ゲイン指定のCANのIDである場合
            {
                if (RxHeader.DLC == can_configure::manage::dlc::pid) // 受信したデータの長さが正しい場合
                {
                    for (uint8_t i = 0; i < can_configure::manage::dlc::pid; i++) // データ長に合わせて繰り返す
                    {
                        buff_init_d_gain[i] = RxData[i]; // 受信データを微分ゲイン指定バッファに格納
                    }
                    flag_init_pid[2] = true; // 微分ゲイン指定のフラグを立てる
                }
                else
                {
                    indicateError(true); // エラー処理
                }
            }
        }
        if (rx_id == can_configure::manage::id::init) // 初期化コマンドのCANのIDである場合
        {
            if (RxHeader.DLC == can_configure::manage::dlc::init) // 受信したデータの長さが正しい場合
            {
                for (uint8_t i = 0; i < can_configure::manage::dlc::init; i++) // データ長に合わせて繰り返す
                {
                    buff_init_command[i] = RxData[i]; // 受信データを初期化コマンドバッファに格納
                }
                if (buff_init_command[0] == md_id) // MDのIDが一致した場合
                {
                    flag_init_command = true; // 初期化コマンドのフラグを立てる
                }
            }
            else
            {
                indicateError(true); // エラー処理
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

void CANDataManager::sendSensorLimitAndEncoder(bool limit_switch1, bool limit_switch2, int16_t encoder_value)
{
    uint8_t tx_data[can_configure::sensor::dlc::limit_and_encoder] = {0};
    tx_data[0] |= uint8_t(limit_switch1) & 0b1;
    tx_data[0] |= (uint8_t(limit_switch2) << 1) & 0b10;                                                                       // リミットスイッチ
    tx_data[1] = static_cast<uint8_t>(static_cast<uint16_t>(encoder_value) & 0xFF);                                           // エンコーダーの下位8ビット
    tx_data[2] = static_cast<uint8_t>(static_cast<uint16_t>(encoder_value) >> 8);                                             // エンコーダーの上位8ビット
    sendPacket(can_configure::sensor::id::limit_and_encoder + md_id, tx_data, can_configure::sensor::dlc::limit_and_encoder); // 送信
}

void CANDataManager::sendSensorLimitAndCurrent(int16_t encoder_value, int16_t current)
{
    uint8_t tx_data[can_configure::sensor::dlc::limit_and_current] = {0};
    tx_data[0] = static_cast<uint8_t>(static_cast<uint16_t>(encoder_value) & 0xFF);                                           // エンコーダーの下位8ビット
    tx_data[1] = static_cast<uint8_t>(static_cast<uint16_t>(encoder_value) >> 8);                                             // エンコーダーの上位8ビット
    tx_data[2] = static_cast<uint8_t>(static_cast<uint16_t>(current) & 0xFF);                                                 // 電流の下位8ビット
    tx_data[3] = static_cast<uint8_t>(static_cast<uint16_t>(current) >> 8);                                                   // 電流の上位8ビット
    sendPacket(can_configure::sensor::id::limit_and_current + md_id, tx_data, can_configure::sensor::dlc::limit_and_current); // 送信
}

void CANDataManager::sendSensorAll(bool limit_switch1, bool limit_switch2, int16_t encoder_value, uint16_t current)
{
    uint8_t tx_data[can_configure::sensor::dlc::all] = {0};
    tx_data[0] |= uint8_t(limit_switch1) & 0b1;
    tx_data[0] |= (uint8_t(limit_switch2) << 1) & 0b10;                                           // リミットスイッチ
    tx_data[1] = static_cast<uint8_t>(static_cast<uint16_t>(encoder_value) & 0xFF);               // エンコーダーの下位8ビット
    tx_data[2] = static_cast<uint8_t>(static_cast<uint16_t>(encoder_value) >> 8);                 // エンコーダーの上位8ビット
    tx_data[3] = static_cast<uint8_t>(current & 0xFF);                                            // 電流の下位8ビット
    tx_data[4] = static_cast<uint8_t>(current >> 8);                                              // 電流の上位8ビット
    sendPacket(can_configure::sensor::id::all + md_id, tx_data, can_configure::sensor::dlc::all); // 送信
}

void CANDataManager::sendStateMD(uint8_t state_code)
{
    uint8_t tx_data[can_configure::state::dlc::md] = {state_code};                            // 送信データ
    sendPacket(can_configure::state::id::md + md_id, tx_data, can_configure::state::dlc::md); // 送信
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
        return;
    }
}