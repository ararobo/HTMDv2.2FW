#include "can_data_manager.hpp"

void CANDataManager::init(uint8_t md_id_)
{
    md_id = md_id_;
    // CANのフィルタ設定
    RxFilter.IdType = FDCAN_STANDARD_ID;             // 標準ID
    RxFilter.FilterIndex = 0;                        // フィルタインデックス
    RxFilter.FilterType = FDCAN_FILTER_MASK;         // マスク
    RxFilter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; // FIFO0にフィルタ
    RxFilter.FilterID1 = 0x000;
    RxFilter.FilterID2 = 0x000;
    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &RxFilter) != HAL_OK)
    {
        Error_Handler();
        HAL_GPIO_WritePin(LED_LIM2_GPIO_Port, LED_LIM2_Pin, GPIO_PIN_SET);
    }
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
    {
        Error_Handler();
        HAL_GPIO_WritePin(LED_LIM2_GPIO_Port, LED_LIM2_Pin, GPIO_PIN_SET);
    }
    // 割り込み有効
    if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
    {
        Error_Handler();
        HAL_GPIO_WritePin(LED_LIM2_GPIO_Port, LED_LIM2_Pin, GPIO_PIN_SET);
    }
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
    uint8_t tx_data[3][can_configure::manage::dlc::pid];
    uint32_t raw_pid_gain[3] = {static_cast<uint32_t>(p_gain), static_cast<uint32_t>(i_gain), static_cast<uint32_t>(d_gain)}; // ゲインの生データ
    // 生データを分割
    for (uint8_t i = 0; i < 3; i++) // ゲインの数だけ繰り返す
    {
        tx_data[i][0] = static_cast<uint8_t>(raw_pid_gain[i] & 0xFF);         // 下位8ビット
        tx_data[i][1] = static_cast<uint8_t>((raw_pid_gain[i] >> 8) & 0xFF);  // 8ビット右シフトして下位8ビット
        tx_data[i][2] = static_cast<uint8_t>((raw_pid_gain[i] >> 16) & 0xFF); // 16ビット右シフトして下位8ビット
        tx_data[i][3] = static_cast<uint8_t>((raw_pid_gain[i] >> 24) & 0xFF); // 24ビット右シフトして下位8ビット
    }
    sendPacket(can_configure::manage::id::re_p_gain, tx_data[0], can_configure::manage::dlc::pid); // 送信
    sendPacket(can_configure::manage::id::re_i_gain, tx_data[1], can_configure::manage::dlc::pid); // 送信
    sendPacket(can_configure::manage::id::re_d_gain, tx_data[2], can_configure::manage::dlc::pid); // 送信
}

void CANDataManager::sendReInitMode(uint8_t *mode_code)
{
    sendPacket(can_configure::manage::id::re_mode + md_id, mode_code, can_configure::manage::dlc::re_mode); // 送信
}

bool CANDataManager::onReceiveTask(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if (hfdcan->Instance == hfdcan1.Instance)
    {
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
        {
            Error_Handler();
            HAL_GPIO_WritePin(LED_LIM2_GPIO_Port, LED_LIM2_Pin, GPIO_PIN_SET);
        }
        else
        {
            uint16_t rx_id = RxHeader.Identifier & 0x7F0;                                              // CANのIDをマスクして全MD共通で種類分けする
            uint8_t rx_md_id = RxHeader.Identifier & 0x00F;                                            // CANのIDをマスクしてMDのIDを取得
            if (uint16_t(RxHeader.Identifier) == (can_configure::control::id::md_targets + md_id / 4)) // MDのIDに対応したモーターの目標値データであった場合
            {
                if (RxHeader.DataLength == can_configure::control::dlc::md_targets) // 受信したデータの長さが正しい場合
                {
                    for (uint8_t i = 0; i < can_configure::control::dlc::md_targets; i++) // データ長に合わせて繰り返す
                    {
                        buff_targets[i] = RxData[i]; // 受信データを目標値バッファに格納
                    }
                    flag_targets = true; // 目標値のフラグを立てる
                }
                else
                {
                    HAL_GPIO_WritePin(LED_LIM2_GPIO_Port, LED_LIM2_Pin, GPIO_PIN_SET); // エラー処理
                }
            }
            else if (rx_md_id == md_id) // 対応するMDのIDである場合
            {
                if (rx_id == can_configure::manage::id::init) // 初期化コマンドのCANのIDである場合
                {
                    if (RxHeader.DataLength == can_configure::manage::dlc::init) // 受信したデータの長さが正しい場合
                    {
                        for (uint8_t i = 0; i < can_configure::manage::dlc::init; i++) // データ長に合わせて繰り返す
                        {
                            buff_init_command[i] = RxData[i]; // 受信データを初期化コマンドバッファに格納
                        }
                        flag_init_command = true; // 初期化コマンドのフラグを立てる
                    }
                    else
                    {
                        HAL_GPIO_WritePin(LED_LIM2_GPIO_Port, LED_LIM2_Pin, GPIO_PIN_SET); // エラー処理
                    }
                }
                else if (rx_id == can_configure::manage::id::mode) // モード指定のCANのIDである場合
                {
                    if (RxHeader.DataLength == can_configure::manage::dlc::mode) // 受信したデータの長さが正しい場合
                    {
                        for (uint8_t i = 0; i < can_configure::manage::dlc::mode; i++) // データ長に合わせて繰り返す
                        {
                            buff_init_mode[i] = RxData[i]; // 受信データをモード指定バッファに格納
                        }
                        flag_init_mode = true; // モード指定のフラグを立てる
                    }
                    else
                    {
                        HAL_GPIO_WritePin(LED_LIM2_GPIO_Port, LED_LIM2_Pin, GPIO_PIN_SET); // エラー処理
                    }
                }
                else if (rx_id == can_configure::manage::id::p_gain) // 比例ゲイン指定のCANのIDである場合
                {
                    if (RxHeader.DataLength == can_configure::manage::dlc::pid) // 受信したデータの長さが正しい場合
                    {
                        for (uint8_t i = 0; i < can_configure::manage::dlc::pid; i++) // データ長に合わせて繰り返す
                        {
                            buff_init_p_gain[i] = RxData[i]; // 受信データを比例ゲイン指定バッファに格納
                        }
                        flag_init_pid[0] = true; // 比例ゲイン指定のフラグを立てる
                    }
                    else
                    {
                        HAL_GPIO_WritePin(LED_LIM2_GPIO_Port, LED_LIM2_Pin, GPIO_PIN_SET); // エラー処理
                    }
                }
                else if (rx_id == can_configure::manage::id::i_gain) // 積分ゲイン指定のCANのIDである場合
                {
                    if (RxHeader.DataLength == can_configure::manage::dlc::pid) // 受信したデータの長さが正しい場合
                    {
                        for (uint8_t i = 0; i < can_configure::manage::dlc::pid; i++) // データ長に合わせて繰り返す
                        {
                            buff_init_i_gain[i] = RxData[i]; // 受信データを積分ゲイン指定バッファに格納
                        }
                        flag_init_pid[1] = true; // 積分ゲイン指定のフラグを立てる
                    }
                    else
                    {
                        HAL_GPIO_WritePin(LED_LIM2_GPIO_Port, LED_LIM2_Pin, GPIO_PIN_SET); // エラー処理
                    }
                }
                else if (rx_id == can_configure::manage::id::d_gain) // 微分ゲイン指定のCANのIDである場合
                {
                    if (RxHeader.DataLength == can_configure::manage::dlc::pid) // 受信したデータの長さが正しい場合
                    {
                        for (uint8_t i = 0; i < can_configure::manage::dlc::pid; i++) // データ長に合わせて繰り返す
                        {
                            buff_init_d_gain[i] = RxData[i]; // 受信データを微分ゲイン指定バッファに格納
                        }
                        flag_init_pid[2] = true; // 微分ゲイン指定のフラグを立てる
                    }
                    else
                    {
                        HAL_GPIO_WritePin(LED_LIM2_GPIO_Port, LED_LIM2_Pin, GPIO_PIN_SET); // エラー処理
                    }
                }
            }
            return true;
        }
    }
    return false;
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

void CANDataManager::sendSensorCurrent(uint16_t current)
{
    uint8_t tx_data[can_configure::sensor::dlc::current] = {static_cast<uint8_t>(current & 0xFF), static_cast<uint8_t>(current >> 8)}; // 送信データ
    sendPacket(can_configure::sensor::id::current + md_id, tx_data, can_configure::sensor::dlc::current);                              // 送信
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

void CANDataManager::sendStateAll(uint8_t state_code, uint16_t state_temp)
{
    uint8_t tx_data[can_configure::state::dlc::all] = {state_code, state_temp && 0xFF, uint8_t(state_temp >> 8)}; // 送信データ
    sendPacket(can_configure::state::id::all + md_id, tx_data, can_configure::state::dlc::all);                   // 送信
}

void CANDataManager::sendPacket(uint16_t can_id, uint8_t *tx_buffer, uint8_t data_length)
{
    if (data_length > 8)
    {
        HAL_GPIO_WritePin(LED_LIM2_GPIO_Port, LED_LIM2_Pin, GPIO_PIN_SET); // エラー処理
    }

    TxHeader.Identifier = can_id;
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = data_length;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    if (0 < HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1))
    {
        if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, tx_buffer) != HAL_OK)
        {
            Error_Handler();
            HAL_GPIO_WritePin(LED_LIM2_GPIO_Port, LED_LIM2_Pin, GPIO_PIN_SET); // エラー処理
        }
    }
    else
    {
        HAL_GPIO_WritePin(LED_LIM2_GPIO_Port, LED_LIM2_Pin, GPIO_PIN_SET); // エラー処理
    }
}