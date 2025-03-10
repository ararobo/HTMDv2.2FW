#include "app.hpp"
#include "can_driver.hpp"

CANDriver can_driver(0);

void App::init()
{
    update_md_id();
    can_driver.set_md_id(md_id);
    can_driver.init();
}

void App::main_loop()
{
}

void App::timer_task()
{
}

void App::can_callback_process(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    can_driver.can_callback_process(hfdcan, RxFifo0ITs);
}

void App::update_md_id()
{
    uint8_t id = 0;
    if (HAL_GPIO_ReadPin(DIP4_GPIO_Port, DIP4_Pin))
        id = 1;
    if (HAL_GPIO_ReadPin(DIP3_GPIO_Port, DIP3_Pin))
        id |= 0b10;
    if (HAL_GPIO_ReadPin(DIP2_GPIO_Port, DIP2_Pin))
        id |= 0b100;
    if (HAL_GPIO_ReadPin(DIP1_GPIO_Port, DIP1_Pin))
        id |= 0b1000;
    md_id = id;
}