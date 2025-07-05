/**
 * @file can_driver.cpp
 * @author aiba-gento
 * @brief STM32のCAN通信用クラス
 * @version 2.0
 * @date 2025-05-10
 * @note hcanのところをhcan1やhcan2に変更することで、複数のCANを扱えるようなります
 *
 * @copyright Copyright (c) 2025
 *
 */
#include "can_driver.hpp"

void CANDriver::init(uint32_t filter_id, uint32_t filter_mask)
{
    filter_mask = (filter_mask << 21) | 0x4; // フィルタマスク
    filter_id = filter_id << 21;             // フィルタID
    // CANのフィルタ設定
    RxFilter.FilterIdHigh = filter_id >> 16;
    RxFilter.FilterIdLow = filter_id;
    RxFilter.FilterMaskIdHigh = filter_mask >> 16;
    RxFilter.FilterMaskIdLow = filter_mask;
    RxFilter.FilterScale = CAN_FILTERSCALE_32BIT;
    RxFilter.FilterFIFOAssignment = CAN_FILTER_FIFO0; // FIFO0に割り当て
    RxFilter.FilterBank = 0;
    RxFilter.FilterMode = CAN_FILTERMODE_IDMASK;
    RxFilter.SlaveStartFilterBank = 14;
    RxFilter.FilterActivation = ENABLE;

    HAL_CAN_ConfigFilter(&hcan, &RxFilter); // CANのフィルタの設定
    HAL_CAN_Start(&hcan);                   // CANのスタート
    // 割り込み有効
    HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING); // FIFO0のメッセージペンディング割り込みを有効
}

void CANDriver::can_callback_process(CAN_HandleTypeDef *hcan)
{
    // 受信
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
    {
        Error_Handler();
    }
    // 受信データの処理
    receive(RxHeader.StdId, RxData, RxHeader.DLC);
}

void CANDriver::send(uint16_t id, uint8_t *data, uint8_t len)
{
    if (len > 8)
    {
        Error_Handler();
    }
    TxHeader.StdId = id; // CANのID
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.DLC = len; // データ長
    TxHeader.TransmitGlobalTime = DISABLE;
    // 送信FIFOが空になるまで待機
    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0)
        ;
    // 送信
    if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, data, &TxMailbox) != HAL_OK)
    {
        Error_Handler();
    }
}