#include "pid_calculator.hpp"

PIDCalculator::PIDCalculator(float dt)
{
    this->dt = dt;
}

float PIDCalculator::calculate_pid(float target, float now_value)
{
    // PID制御の計算
    float error = target - now_value;
    i_out += error * dt;
    float d_out = (error - prev_error) / dt;
    prev_error = error;

    return Kp * error + Ki * i_out + Kd * d_out;
}

void PIDCalculator::set_pid_gain(float p_gain, float i_gain, float d_gain)
{
    Kp = p_gain;
    Ki = i_gain;
    Kd = d_gain;
    reset();
}

void PIDCalculator::get_pid_gain(float *p_gain, float *i_gain, float *d_gain)
{
    *p_gain = Kp;
    *i_gain = Ki;
    *d_gain = Kd;
}

void PIDCalculator::set_dt(float dt)
{
    this->dt = dt;
}

void PIDCalculator::reset()
{
    i_out = 0.0f;
    prev_error = 0.0f;
}