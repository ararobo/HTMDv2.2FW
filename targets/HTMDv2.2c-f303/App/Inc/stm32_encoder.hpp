#pragma once

#include "common/interfaces/encoder_interface.hpp"
#include "main.h"

namespace targets::f303 {

class Stm32Encoder : public common::interfaces::EncoderInterface {
  public:
    /**
     * @param htim_enc タイマーハンドル
     * @param cpr Counts Per Revolution (4逓倍後の値)
     */
    Stm32Encoder(TIM_HandleTypeDef* htim_enc, float cpr);

    bool init() override;

    int32_t get_counts() override;

    float get_velocity() override;

    void reset_counts() override;

  private:
    TIM_HandleTypeDef* htim_enc_;
    int32_t last_count_ = 0;
    uint32_t last_time_ms_ = 0;
    float cpr_;
    float last_velocity_rad_s_ = 0.0f;
};

}  // namespace targets::f303
