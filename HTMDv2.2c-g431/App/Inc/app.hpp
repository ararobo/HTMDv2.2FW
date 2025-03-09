#include <stdint.h>
#include "fdcan.h"

class App
{
private:
public:
    void init();
    void mainLoop();
    void timerTask();
    void CANCallbackProcess(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);
};