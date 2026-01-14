#pragma once

#include <cfloat>
#include <cstdint>

namespace common::interfaces {

class GateDriverInterface {
  public:
    virtual ~GateDriverInterface() = default;
    
    virtual bool init() = 0;

    virtual bool output(float output) = 0;

    virtual void set_brake_mode(bool enabled) = 0;
};

}  // namespace common::interfaces