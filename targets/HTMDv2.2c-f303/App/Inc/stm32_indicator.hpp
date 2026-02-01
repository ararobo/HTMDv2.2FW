#pragma once

#include "common/interfaces/indicator_interface.hpp"
#include "main.h"

namespace targets::f303 {

class Stm32Indicator : public common::interfaces::IndicatorInterface {
  public:
    Stm32Indicator();

    bool init() override;

    void set(common::interfaces::IndicatorId id, bool enabled) override;
};

}  // namespace targets::f303
