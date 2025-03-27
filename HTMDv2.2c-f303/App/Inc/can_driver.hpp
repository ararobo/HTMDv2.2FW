#include "md_data_manager.hpp"
#include "ems_data_manager.hpp"
#include "can.h"

class CANDriver : public MDDataManager<false>, EMSDataManager
{
private:
    CAN_FilterTypeDef RxFilter;
    CAN_RxHeaderTypeDef RxHeader;
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailbox;
    uint8_t RxData[8];

    void send(uint16_t id, uint8_t *data, uint8_t len) override
    {
        if (len > 8)
        {
            Error_Handler();
        }
        TxHeader.StdId = id;                   // CANのID
        TxHeader.RTR = CAN_RTR_DATA;           // リモートフレーム
        TxHeader.IDE = CAN_ID_STD;             // 標準フレーム
        TxHeader.DLC = len;                    // データ長
        TxHeader.TransmitGlobalTime = DISABLE; // グローバルタイム
        if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, data, &TxMailbox) != HAL_OK)
        {
            Error_Handler();
        }
    }

public:
    CANDriver(uint8_t md_id) : MDDataManager(md_id) {}

    void init()
    {
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

    void can_callback_process(CAN_HandleTypeDef *hcan)
    {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
        {
            Error_Handler();
        }
        receive(RxHeader.StdId, RxData, RxHeader.DLC);
    }
};