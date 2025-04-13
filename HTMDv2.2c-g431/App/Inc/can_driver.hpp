#pragma once
#include "md_data_slave.hpp"
#include "fdcan.h"

class CANDriver : public MDDataSlave
{
private:
    FDCAN_RxHeaderTypeDef RxHeader;
    FDCAN_FilterTypeDef RxFilter;
    FDCAN_TxHeaderTypeDef TxHeader;
    uint8_t RxData[8];
    uint32_t filter_mask; // フィルタマスク
    uint32_t filter_id;   // フィルタID

    void send(uint16_t id, uint8_t *data, uint8_t len) override;

public:
    CANDriver(uint8_t board_id, uint8_t board_type, uint8_t fw_version);

    void init();

    void can_callback_process(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);
};