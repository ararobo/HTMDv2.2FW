#include "can_config.hpp"
#include "md_config.hpp"

using namespace can_config;

template <bool is_master>
class MDDataManager
{
private:
    uint8_t md_id;
    bool is_received_init;
    bool is_received_target;
    bool is_received_limit_switch;
    bool is_received_gain;
    uint8_t md_kind;
    md_config_t md_config;
    int16_t target;
    uint8_t limit_switch;
    float pid_gain[3];

protected:
    void send(uint16_t id, uint8_t *data, uint8_t len);
    void receive(uint16_t id, uint8_t *data, uint8_t len);

public:
    MDDataManager(uint8_t md_id) : md_id(md_id), is_received_init(false), is_received_target(false), is_received_limit_switch(false), is_received_gain(false) {};

    void send_init(md_config_t md_config)
    {
        static_assert(!is_master, "This function is only for master.");
        send(encode_id(direction::slave, board_type::md, md_id, data_type::md::init), md_config.code, sizeof(md_config.code));
    }

    void send_init(uint8_t md_kind)
    {
        static_assert(is_master, "This function is only for slave.");
        send(encode_id(direction::master, board_type::md, md_id, data_type::md::init), &md_kind, sizeof(md_kind));
    }

    void send_target(int16_t target)
    {
        uint8_t data[2];
        memcpy(data, &target, sizeof(target));
        if (is_master)
        {
            send(encode_id(direction::slave, board_type::md, md_id, data_type::md::target), data, sizeof(data));
        }
        else
        {
            send(encode_id(direction::master, board_type::md, md_id, data_type::md::target), data, sizeof(data));
        }
    }

    void send_limit_switch(uint8_t limit_switch)
    {
        static_assert(is_master, "This function is slave only.");
        send(encode_id(direction::master, board_type::md, md_id, data_type::md::limit_switch), &limit_switch, sizeof(limit_switch));
    }

    void send_gain(float p_gain, float i_gain, float d_gain);
    bool is_md_init();
    md_config_t get_md_config();
    uint8_t get_md_kind();
    bool get_target(int16_t *target);
    bool get_limit_switch(uint8_t *limit_switch);
    bool get_gain(float *p_gain, float *i_gain, float *d_gain);
};