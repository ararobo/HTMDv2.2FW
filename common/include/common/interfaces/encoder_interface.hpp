#pragma once

#include <cfloat>
#include <cstdint>

namespace common::interfaces {

class EncoderInterface {
  public:
    virtual ~EncoderInterface() = default;

    virtual bool init() = 0;

    virtual int32_t get_counts() = 0;

    /**
     * @brief 現在の速度を取得[rad/s]
     * 
     * @return float 現在の速度[rad/s]
     */
    virtual float get_velocity() = 0;

    virtual void reset_counts() = 0;
};

}  // namespace common::interfaces