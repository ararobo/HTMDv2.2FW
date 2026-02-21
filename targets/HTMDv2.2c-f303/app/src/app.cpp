#include "app/app.hpp"

void setup() {}
void loop() {}
extern "C" {
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan) {}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {}
}
