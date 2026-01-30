#include "stm32_encoder.hpp"

namespace targets::f303 {

Stm32Encoder::Stm32Encoder(TIM_HandleTypeDef* htim_enc, float cpr)
    : htim_enc_(htim_enc), cpr_(cpr) {}

bool Stm32Encoder::init() {
    return HAL_TIM_Encoder_Start(htim_enc_, TIM_CHANNEL_ALL) == HAL_OK;
}

int32_t Stm32Encoder::get_counts() {
    // 16bitタイマなので符号付き拡張が必要な場合があるが、
    // HALのCNTはuint32_tで返ってくる。
    // 1回転あたりのパルス数にもよるが、オーバーフローを考慮してint16_tキャストを使う手法が一般的。
    return static_cast<int16_t>(__HAL_TIM_GET_COUNTER(htim_enc_));
}

float Stm32Encoder::get_velocity() {
    uint32_t current_time = HAL_GetTick();
    float dt = (current_time - last_time_ms_) / 1000.0f;
    
    // 時間経過が小さすぎる（計測分解能以下）の場合は、
    // 0を返すとスパイクノイズになるため、前回の値を維持する（ゼロ次ホールド）
    if (dt <= 0.0001f) {
        return last_velocity_rad_s_; 
    }

    int32_t current_count = get_counts();
    
    // アンダーフロー・オーバーフローを考慮した差分計算
    // int16_t同士の引き算で自動的にラップアラウンドが処理される
    // (キャストを多用して明示的に16bit差分であることを示す)
    int16_t diff_16 = static_cast<int16_t>(current_count) - static_cast<int16_t>(last_count_);
    int32_t delta_count = static_cast<int32_t>(diff_16);
    
    float velocity_cnt_per_sec = delta_count / dt;

    // Convert to rad/s
    // 2 * PI * (counts / sec) / CPR
    float velocity_rad_s = (velocity_cnt_per_sec / cpr_) * 6.28318530718f;

    last_count_ = current_count;
    last_time_ms_ = current_time;
    last_velocity_rad_s_ = velocity_rad_s;

    return velocity_rad_s; 
}

void Stm32Encoder::reset_counts() {
    __HAL_TIM_SET_COUNTER(htim_enc_, 0);
    last_count_ = 0;
}

}  // namespace targets::f303
