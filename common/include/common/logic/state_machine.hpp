#pragma once
#include <cstdint>

namespace common::logic {

enum class SystemState : uint8_t {
    INITIALIZATION,
    RUNNING,
    ERROR,
};

class StateMachine {
  public:
    StateMachine() : current_state_(SystemState::INITIALIZATION) {}

    SystemState get_state() const { return current_state_; }

    void set_state(SystemState new_state) { current_state_ = new_state; }

  private:
    SystemState current_state_;
};
}  // namespace common::logic