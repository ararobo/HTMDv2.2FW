#pragma once

#include <cstdint>

namespace common::interfaces {

enum class IndicatorId {
    HEARTBEAT,  // LED1: 制御周期確認
    DIRECTION,  // LED2: 逆回転確認
    ACTIVITY,   // LED3: 回転中
    POWER       // LED4: 電源確認
};

class IndicatorInterface {
  public:
    virtual bool init() = 0;

    virtual void set(IndicatorId id, bool enabled) = 0;
};
}  // namespace common::interfaces