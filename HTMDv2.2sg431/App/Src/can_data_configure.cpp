#include "can_data_configure.hpp"

uint16_t encodeCanID(uint8_t dir, uint8_t dev, uint8_t device_id, uint8_t data_name)
{
    return ((dir & 0x1) << 10) | ((dev & 0x7) << 7) | ((device_id & 0xf) << 3) | (data_name & 0x7);
}

void decodeCanID(uint16_t can_id, uint8_t *dir, uint8_t *dev, uint8_t *device_id, uint8_t *data_name)
{
    *dir = (can_id & 0x400) >> 10;
    *dev = (can_id & 0x380) >> 7;
    *device_id = (can_id & 0x78) >> 3;
    *data_name = (can_id & 0x7);
}