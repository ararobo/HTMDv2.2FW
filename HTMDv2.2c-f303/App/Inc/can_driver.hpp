#pragma once
#include "md_data_slave.hpp"
#include "can.h"

class CANDriver : public MDController
{
private:
    CAN_FilterTypeDef RxFilter;
    CAN_RxHeaderTypeDef RxHeader;
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailbox;
    uint8_t RxData[8];
    uint32_t filter_mask; // フィルタマスク
    uint32_t filter_id;   // フィルタID

    /**
     * @brief CANの送信処理(オーバーライド)
     *
     * @param id CANのID
     * @param data 送信するデータ
     * @param len データの長さ
     */
    void send(uint16_t id, uint8_t *data, uint8_t len) override;

public:
    CANDriver(uint8_t board_id, uint8_t board_type, uint8_t fw_version);
    /// @brief プログラムの始めに一回だけ呼び出す
    void init();
    /// @brief CANのコールバック処理
    void can_callback_process(CAN_HandleTypeDef *hcan);
};