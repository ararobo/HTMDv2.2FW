#include "md_data_manager.hpp"
#include "fdcan.h"

class CANDriver : public MDDataManager<false>
{
private:
    void send()
    {
    }

public:
    CANDriver(uint8_t md_id) : MDDataManager(md_id) {}

    void init()
    {
    }

    void CANCallbackProcess(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
    {
    }
};