#pragma once

#include "common/control/acceleration_limiter.hpp"
#include "common/control/pid.hpp"
#include "common/interfaces/encoder_interface.hpp"
#include "common/interfaces/gate_driver_interface.hpp"
#include "common/interfaces/indicator_interface.hpp"
#include "common/logic/state_machine.hpp"
#include "common/logic/can_protocol.hpp"

// Forward declaration
namespace gn10_can {
class CANBus;
}

namespace common::logic {

struct MotorManagerConfig {
    float control_loop_dt;
    control::PIDConfig<float> pid_config;
    float default_accel_limit;
};

class MotorManager {
  public:
    MotorManager(interfaces::EncoderInterface& encoder,
                 interfaces::GateDriverInterface& gate_driver,
                 interfaces::IndicatorInterface& indicator,
                 gn10_can::CANBus& can_bus,
                 const MotorManagerConfig& config,
                 uint8_t device_id);

    void init();
    void update();

  private:
    interfaces::EncoderInterface& encoder_;
    interfaces::GateDriverInterface& gate_driver_;
    interfaces::IndicatorInterface& indicator_;
    
    MotorDriverSlave can_slave_;
    
    StateMachine state_machine_;
    control::PID<float> pid_controller_;
    control::AccelerationLimiter<float> accel_limiter_;

    MotorManagerConfig config_;

    float target_setpoint_ = 0.0f;
    float measured_value_ = 0.0f;
};

}  // namespace common::logic