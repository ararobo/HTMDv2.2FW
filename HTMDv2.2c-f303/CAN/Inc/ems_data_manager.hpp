#include "can_config.hpp"
#include "md_config.hpp"

class EMSDataManager
{
private:
protected:
    virtual void send(uint16_t id, uint8_t *data, uint8_t len) = 0;
};