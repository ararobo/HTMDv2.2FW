/**
 * @file stm32_fdcan1_driver.hpp
 * @author aiba-gento
 * @brief STM32のFDCAN1通信用クラス
 * @version 2.0
 * @date 2025-07-05
 *
 * @copyright Copyright (c) 2025
 *
 */
#pragma once
#include <stdint.h>
#include "fdcan.h"
#include "md_data_slave.hpp"

class STM32FDCAN1Driver : public MDDataSlave
{
private:
    FDCAN_RxHeaderTypeDef RxHeader;
    FDCAN_FilterTypeDef RxFilter;
    FDCAN_TxHeaderTypeDef TxHeader;
    uint8_t RxData[8];

public:
    STM32FDCAN1Driver() : MDDataSlave() {}

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
    void can_callback_process(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);
};