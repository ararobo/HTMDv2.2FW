#include "driver_stm32_fdcan.hpp"

namespace gn10_can {
namespace drivers {

DriverSTM32FDCAN::DriverSTM32FDCAN(FDCAN_HandleTypeDef* hfdcan) : hfdcan_(hfdcan) {
    filter_.IdType       = FDCAN_STANDARD_ID;
    filter_.FilterIndex  = 0;
    filter_.FilterType   = FDCAN_FILTER_MASK;
    filter_.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter_.FilterID1    = 0x000;
    filter_.FilterID2    = 0x000;
}

bool DriverSTM32FDCAN::init() {
    if (HAL_FDCAN_ConfigFilter(hfdcan_, &filter_) != HAL_OK) {
        return false;
    }
    if (HAL_FDCAN_Start(hfdcan_) != HAL_OK) {
        return false;
    }
    if (HAL_FDCAN_ActivateNotification(hfdcan_, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
        return false;
    }
    return true;
}

bool DriverSTM32FDCAN::send(const CANFrame& frame) {
    FDCAN_TxHeaderTypeDef tx_header;
    tx_header.IdType              = (frame.is_extended) ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
    tx_header.Identifier          = frame.id;
    tx_header.TxFrameType         = FDCAN_DATA_FRAME;
    tx_header.DataLength          = len_to_dlc(frame.dlc);
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch       = FDCAN_BRS_OFF;
    tx_header.FDFormat            = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker       = 0;

    if (HAL_FDCAN_AddMessageToTxFifoQ(
            hfdcan_, &tx_header, const_cast<uint8_t*>(frame.data.data())) != HAL_OK) {
        return false;
    }
    return true;
}

bool DriverSTM32FDCAN::receive(CANFrame& out_frame) {
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    if (HAL_FDCAN_GetRxMessage(hfdcan_, FDCAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK) {
        return false;
    }

    out_frame.id          = rx_header.Identifier;
    out_frame.dlc         = dlc_to_len(rx_header.DataLength);
    out_frame.is_extended = (rx_header.IdType == FDCAN_EXTENDED_ID);
    out_frame.is_rtr      = (rx_header.RxFrameType == FDCAN_REMOTE_FRAME);
    out_frame.is_error    = false;

    for (uint8_t i = 0; i < out_frame.dlc; ++i) {
        out_frame.data[i] = rx_data[i];
    }

    return true;
}

uint32_t DriverSTM32FDCAN::len_to_dlc(uint8_t len) {
    switch (len) {
        case 0:
            return FDCAN_DLC_BYTES_0;
        case 1:
            return FDCAN_DLC_BYTES_1;
        case 2:
            return FDCAN_DLC_BYTES_2;
        case 3:
            return FDCAN_DLC_BYTES_3;
        case 4:
            return FDCAN_DLC_BYTES_4;
        case 5:
            return FDCAN_DLC_BYTES_5;
        case 6:
            return FDCAN_DLC_BYTES_6;
        case 7:
            return FDCAN_DLC_BYTES_7;
        case 8:
        default:
            return FDCAN_DLC_BYTES_8;
    }
}

uint8_t DriverSTM32FDCAN::dlc_to_len(uint32_t dlc) {
    switch (dlc) {
        case FDCAN_DLC_BYTES_0:
            return 0;
        case FDCAN_DLC_BYTES_1:
            return 1;
        case FDCAN_DLC_BYTES_2:
            return 2;
        case FDCAN_DLC_BYTES_3:
            return 3;
        case FDCAN_DLC_BYTES_4:
            return 4;
        case FDCAN_DLC_BYTES_5:
            return 5;
        case FDCAN_DLC_BYTES_6:
            return 6;
        case FDCAN_DLC_BYTES_7:
            return 7;
        case FDCAN_DLC_BYTES_8:
        default:
            return 8;
    }
}
}  // namespace drivers
}  // namespace gn10_can