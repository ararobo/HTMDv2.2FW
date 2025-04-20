#pragma once
#include "can.h"

class CANDriver
{
private:
    CAN_FilterTypeDef RxFilter;
    CAN_RxHeaderTypeDef RxHeader;
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailbox;
    uint8_t RxData[8];

protected:
    /**
     * @brief CANの受信処理(オーバーライドしてください)
     *
     * @param id CANのID
     * @param data 受信データ
     * @param len データの長さ
     */
    virtual void receive(uint16_t id, uint8_t *data, uint8_t len) = 0;

public:
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
    void send(uint16_t id, uint8_t *data, uint8_t len);

    /**
     * @brief CANのコールバック処理
     *
     * @param hcan CANのハンドル
     */
    void can_callback_process(CAN_HandleTypeDef *hcan);
};