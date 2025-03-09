#include <stdint.h>
#include "fdcan.h"

class App
{
private:
public:
    App();
    void init();
    void mainLoop();
    void CANCallbackProcess(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);
};