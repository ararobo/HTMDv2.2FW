#include "driver_stm32_can.hpp"

namespace gn10_can {
namespace drivers {

bool DriverSTM32CAN::init() {
    filter_.FilterIdHigh         = 0;
    filter_.FilterIdLow          = 0;
    filter_.FilterMaskIdHigh     = 0;
    filter_.FilterMaskIdLow      = 0;
    filter_.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter_.FilterBank           = 0;
    filter_.FilterMode           = CAN_FILTERMODE_IDMASK;
    filter_.FilterScale          = CAN_FILTERSCALE_32BIT;
    filter_.FilterActivation     = ENABLE;
    filter_.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(hcan_, &filter_) != HAL_OK) {
        return false;
    }

    if (HAL_CAN_Start(hcan_) != HAL_OK) {
        return false;
    }

    if (HAL_CAN_ActivateNotification(hcan_, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
        return false;
    }

    return true;
}

bool DriverSTM32CAN::send(const CANFrame& frame) {
    CAN_TxHeaderTypeDef tx_header;
    uint32_t tx_mailbox;

    tx_header.StdId              = frame.id;
    tx_header.ExtId              = frame.id;
    tx_header.IDE                = (frame.is_extended) ? CAN_ID_EXT : CAN_ID_STD;
    tx_header.RTR                = (frame.is_rtr) ? CAN_RTR_REMOTE : CAN_RTR_DATA;
    tx_header.DLC                = frame.dlc;
    tx_header.TransmitGlobalTime = DISABLE;

    if (HAL_CAN_AddTxMessage(
            hcan_, &tx_header, const_cast<uint8_t*>(frame.data.data()), &tx_mailbox) != HAL_OK) {
        return false;
    }
    return true;
}

bool DriverSTM32CAN::receive(CANFrame& out_frame) {
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    if (HAL_CAN_GetRxMessage(hcan_, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK) {
        return false;
    }

    out_frame.id          = (rx_header.IDE == CAN_ID_EXT) ? rx_header.ExtId : rx_header.StdId;
    out_frame.dlc         = rx_header.DLC;
    out_frame.is_extended = (rx_header.IDE == CAN_ID_EXT);
    out_frame.is_rtr      = (rx_header.RTR == CAN_RTR_REMOTE);
    out_frame.is_error    = false;

    for (uint8_t i = 0; i < out_frame.dlc; ++i) {
        out_frame.data[i] = rx_data[i];
    }

    return true;
}

}  // namespace drivers
}  // namespace gn10_can
