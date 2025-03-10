#include <stdint.h>
#include "fdcan.h"

class App
{
private:
    uint8_t md_id;

private:
    void update_md_id();

public:
    void init();
    void main_loop();
    void timer_task();
    void can_callback_process(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);
};