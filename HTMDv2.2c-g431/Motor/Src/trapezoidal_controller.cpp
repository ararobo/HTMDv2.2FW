#include "trapezoidal_controller.hpp"

int16_t TrapezoidalController::trapezoidal_control(int16_t output, uint8_t max_acceleration)
{
    // 台形制御の計算
    if (output > prev_out + max_acceleration)
    {
        output = prev_out + max_acceleration;
    }
    else if (output < prev_out - max_acceleration)
    {
        output = prev_out - max_acceleration;
    }
    prev_out = output;

    return output;
}

void TrapezoidalController::reset_trapezoidal_control()
{
    prev_out = 0;
}