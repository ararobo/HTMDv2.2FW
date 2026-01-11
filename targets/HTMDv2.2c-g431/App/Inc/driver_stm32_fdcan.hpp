#pragma once

#include "fdcan.h"
#include "gn10_can/drivers/driver_interface.hpp"

namespace gn10_can {
namespace drivers {

class DriverSTM32FDCAN : public DriverInterface {
  public:
    DriverSTM32FDCAN(FDCAN_HandleTypeDef* hfdcan) : hfdcan_(hfdcan) {}

    bool init();
    bool send(const CANFrame& frame) override;
    bool receive(CANFrame& out_frame) override;

  private:
    FDCAN_HandleTypeDef* hfdcan_;

    uint32_t len_to_dlc(uint8_t len);
    uint8_t dlc_to_len(uint32_t dlc);
};
}  // namespace drivers
}  // namespace gn10_can