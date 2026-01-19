#pragma once

#include "common/control/acceleration_limiter.hpp"
#include "common/control/pid.hpp"
#include "common/interfaces/encoder_interface.hpp"
#include "common/interfaces/gate_driver_interface.hpp"
#include "common/interfaces/indicator_interface.hpp"
#include "common/logic/state_machine.hpp"

namespace common::logic {

class MotorManager {
  public:
    MotorManager(interfaces::EncoderInterface& encoder,
                 interfaces::GateDriverInterface& gate_driver,
                 interfaces::IndicatorInterface& indicator);

    void init();
    void update();

  private:
    interfaces::EncoderInterface& encoder_;
    interfaces::GateDriverInterface& gate_driver_;
    interfaces::IndicatorInterface& indicator_;
    StateMachine state_machine_;
    control::PID<float> pid_controller_;
    control::AccelerationLimiter<float> accel_limiter_;
};

}  // namespace common::logic