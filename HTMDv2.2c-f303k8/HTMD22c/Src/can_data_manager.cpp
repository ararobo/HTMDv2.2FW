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
        flag_init_command = false;                    // フラグを下ろす
        if (buff_init_command[1] == command::DO_INIT) // 初期化コマンドが実行された場合
        {
            uint8_t tx_data[data_length::INIT_COMMAND] = {md_id, command::SUCCESS}; // 送信データの設定
            sendPacket(can_id::RE_INIT_COMMAND, tx_data, 2);                        // 送信
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
        flag_init_mode = false;                              // フラグを下ろす
        for (uint8_t i = 0; i < data_length::INIT_MODE; i++) // モード指定のデータ長に合わせて繰り返す
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

void CANDataManager::sendMDStatus(bool lim_sw1, bool lim_sw2, int16_t encoder)
{
    uint8_t tx_data[data_length::MD_STATE];                                                // 送信データ
    uint16_t encoder_ = static_cast<uint16_t>(encoder);                                    // エンコーダの値をuint16_tに変換
    tx_data[0] = uint8_t(encoder_ & 0xFF);                                                 // エンコーダの下位8ビット
    tx_data[1] = uint8_t((encoder_ & 0xFF00) >> 8);                                        // エンコーダの上位8ビット
    tx_data[2] = lim_sw1 & 0b1;                                                            // リミットスイッチ1
    tx_data[2] |= (lim_sw2 << 1) & 0b10;                                                   // リミットスイッチ2
    sendPacket(can_id::MD_STATE | (uint16_t(md_id) << 4), tx_data, data_length::MD_STATE); // 送信
}

void CANDataManager::sendReInitCommand()
{
    uint8_t tx_data[data_length::INIT_COMMAND];                           // 送信データ
    tx_data[0] = md_id;                                                   // MDのID
    tx_data[1] = command::SUCCESS;                                        // 成功
    sendPacket(can_id::INIT_COMMAND, tx_data, data_length::INIT_COMMAND); // 送信
}

void CANDataManager::sendReInitPID(float p_gain, float i_gain, float d_gain)
{
    uint8_t tx_data[7];         // 送信データ
    p_gain *= PID_GAIN_QUALITY; // PIDゲインを10000倍
    i_gain *= PID_GAIN_QUALITY;
    d_gain *= PID_GAIN_QUALITY;
    tx_data[0] = md_id;                                              // MDのID
    tx_data[1] = uint16_t(p_gain);                                   // 比例ゲインの下位8ビット
    tx_data[2] = uint16_t(p_gain) >> 8;                              // 比例ゲインの上位8ビット
    tx_data[3] = uint16_t(i_gain);                                   // 積分ゲインの下位8ビット
    tx_data[4] = uint16_t(i_gain) >> 8;                              // 積分ゲインの上位8ビット
    tx_data[5] = uint16_t(d_gain);                                   // 微分ゲインの下位8ビット
    tx_data[6] = uint16_t(d_gain) >> 8;                              // 微分ゲインの上位8ビット
    sendPacket(can_id::RE_INIT_PID, tx_data, data_length::INIT_PID); // 送信
}

void CANDataManager::sendReInitMode(uint8_t *mode_code)
{
    sendPacket(can_id::RE_INIT_MODE, mode_code, data_length::INIT_MODE); // 送信
}

void CANDataManager::onReceiveTask(CAN_HandleTypeDef *hcan_)
{
    if (HAL_CAN_GetRxMessage(hcan_, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) // 正常に受信できた場合
    {
        uint16_t rx_id = RxHeader.StdId & 0x70F;                                // CANのIDをマスクして全MD共通で種類分けする
        if (uint16_t(RxHeader.StdId) == (can_id::TARGETS | ((md_id / 4) << 4))) // MDのIDに対応したモーターの目標値データであった場合
        {
            if (RxHeader.DLC == data_length::TARGETS) // 受信したデータの長さが正しい場合
            {
                for (uint8_t i = 0; i < data_length::TARGETS; i++) // データ長に合わせて繰り返す
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
        else if (RxData[0] == md_id) // 対応するMDのIDである場合
        {
            if (rx_id == can_id::INIT_COMMAND) // 初期化コマンドのCANのIDである場合
            {
                if (RxHeader.DLC == data_length::INIT_COMMAND) // 受信したデータの長さが正しい場合
                {
                    for (uint8_t i = 0; i < data_length::INIT_COMMAND; i++) // データ長に合わせて繰り返す
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
            else if (rx_id == can_id::INIT_MODE) // モード指定のCANのIDである場合
            {
                if (RxHeader.DLC == data_length::INIT_MODE) // 受信したデータの長さが正しい場合
                {
                    for (uint8_t i = 0; i < data_length::INIT_MODE; i++) // データ長に合わせて繰り返す
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
            else if (rx_id == can_id::INIT_PID) // PIDゲイン指定のCANのIDである場合
            {
                if (RxHeader.DLC == data_length::INIT_PID) // 受信したデータの長さが正しい場合
                {
                    for (uint8_t i = 0; i < data_length::INIT_PID; i++) // データ長に合わせて繰り返す
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