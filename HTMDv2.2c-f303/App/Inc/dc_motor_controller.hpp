#pragma once
#include "md_config.hpp"

class MotorController
{
private:
    md_config_t md_config; // モータードライバの設定
    int16_t encoder_count; // エンコーダのカウント

public:
    MotorController();

    void init();

    void reset();

    void run(int16_t output);

    void set_config(md_config_t config);

    void stop();

    void set_pid_gain(float p_gain, float i_gain, float d_gain);

    void sample_encoder();
};