#pragma once

#include "common/interfaces/gate_driver_interface.hpp"
#include "main.h"

namespace targets::f303 {

class Stm32GateDriver : public common::interfaces::GateDriverInterface {
  public:
    Stm32GateDriver(TIM_HandleTypeDef* htim_pwm);

    bool init() override;

    bool output(float output) override;

    void set_brake_mode(bool enabled) override;

  private:
    TIM_HandleTypeDef* htim_pwm_;
    bool brake_mode_ = false;
};

}  // namespace targets::f303
