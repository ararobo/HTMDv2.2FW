/**
 * @file can_driver.hpp
 * @author aiba-gento
 * @brief STM32のCAN通信用クラス
 * @version 2.0
 * @date 2025-07-05
 * @note hcanのところをhcan1やhcan2に変更することで、複数のCANを扱えるようなります
 *
 * @copyright Copyright (c) 2025
 *
 */
#pragma once
#include <stdint.h>
#include "can.h"
#include "md_data_slave.hpp"

class CANDriver : public MDDataSlave
{
private:
    CAN_FilterTypeDef RxFilter;
    CAN_RxHeaderTypeDef RxHeader;
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailbox;
    uint8_t RxData[8];

public:
    CANDriver() : MDDataSlave() {}

    /**
     * @brief CANの初期化
     *
     * @param filter_id
     * @param filter_mask
     */
    void init(uint32_t filter_id, uint32_t filter_mask);

    /**
     * @brief CANの送信処理
     *
     * @param id CANのID
     * @param data 送信するデータ
     * @param len データの長さ
     */
    void send(uint16_t id, uint8_t *data, uint8_t len) override;

    /**
     * @brief CANのコールバック処理
     *
     * @param hcan CANのハンドル
     */
    void can_callback_process(CAN_HandleTypeDef *hcan);
};