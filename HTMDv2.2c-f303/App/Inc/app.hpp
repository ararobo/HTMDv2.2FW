#include <stdint.h>
#include "can.h"
#include "can_driver.hpp"

class App
{
private:
    uint8_t md_id;
    bool initialized = false;
    md_config_t md_config;
    int16_t target;
    int16_t output;
    int16_t encoder;
    uint8_t limit_switch;
    float pid_gain[3];
    uint16_t update_target_count = 0;
    uint16_t update_target_count_max = 100;
    uint16_t timer_count = 0;
    uint16_t loop_count = 0;
    uint16_t loop_count_max = 100;

private:
    void update_md_id();

public:
    void init();
    void main_loop();
    void timer_task();
    void can_callback_process(CAN_HandleTypeDef *hcan);
};