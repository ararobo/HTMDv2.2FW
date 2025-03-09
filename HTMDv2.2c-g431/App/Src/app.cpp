#include "app.hpp"
#include "can_driver.hpp"

CANDriver can_driver(1);

void App::init()
{
}

void App::mainLoop()
{
}

void App::timerTask()
{
}

void App::CANCallbackProcess(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
}