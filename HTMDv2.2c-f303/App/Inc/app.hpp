#include <stdint.h>
#include "can.h"
#include "can_driver.hpp"

class App
{
private:
    uint8_t md_id;
    bool initialized;
    md_config_t md_config;
    int16_t target;
    int16_t output;
    int16_t encoder;
    uint8_t limit_switch;
    float pid_gains[3];
    bool pid_gains_updated;
    uint16_t update_target_count;
    uint16_t update_target_count_max;
    uint16_t timer_count;
    uint16_t loop_count;
    uint16_t loop_count_max;

    uint32_t last_tick;

private:
    void update_md_id();
    void wait_for_next_period();

public:
    App();
    void init();
    void main_loop();
    void timer_task();
    void can_callback_process(CAN_HandleTypeDef *hcan);
};