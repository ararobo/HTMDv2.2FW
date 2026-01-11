/**
 * @file driver_stm32_fdcan.hpp
 * @author Gento Aiba (aiba-gento)
 * @brief STM32 FDCANのドライバ具体化クラスのヘッダファイル
 * @version 0.1
 * @date 2026-01-11
 *
 * @copyright Copyright (c) 2026
 * SPDX-License-Identifier: GPL-3.0
 */
#pragma once

#include "fdcan.h"
#include "gn10_can/drivers/driver_interface.hpp"

namespace gn10_can {
namespace drivers {

class DriverSTM32FDCAN : public DriverInterface {
  public:
    DriverSTM32FDCAN(FDCAN_HandleTypeDef* hfdcan);

    bool init();
    bool send(const CANFrame& frame) override;
    bool receive(CANFrame& out_frame) override;

  private:
    FDCAN_HandleTypeDef* hfdcan_;
    FDCAN_FilterTypeDef filter_;

    uint32_t len_to_dlc(uint8_t len);
    uint8_t dlc_to_len(uint32_t dlc);
};
}  // namespace drivers
}  // namespace gn10_can