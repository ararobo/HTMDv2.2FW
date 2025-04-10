#include "md_manager.hpp"
#include <cstring>

MDManager::MDManager(uint8_t board_id, uint8_t board_kind, uint8_t fw_version) : board_id(board_id), board_kind(board_kind), fw_version(fw_version)
{
}

void MDManager::receive(uint16_t id, uint8_t *data, uint8_t len)
{
    can_config::decode_id(id, this->packet_direction, this->packet_board_type, this->packet_board_id, this->packet_data_type);
    if (this->packet_direction == can_config::direction::slave && this->packet_board_type == can_config::board_type::md && this->packet_board_id == this->board_id)
    {
        switch (this->packet_data_type)
        {
        case can_config::data_type::md::init:
            this->init_flag = true;
            memcpy(this->init_buffer, data, len);
            break;
        case can_config::data_type::md::target:
            this->target_flag = true;
            memcpy(this->target_buffer, data, len);
            break;
        case can_config::data_type::md::limit_switch:
            this->limit_switch_flag = true;
            memcpy(this->rx_buffer, data, len);
            break;
        case can_config::data_type::md::gain:
            this->gain_flag = true;
            memcpy(this->gain_buffer, data, len);
            break;
        case can_config::data_type::md::multi_target:
            this->multi_target_flag = true;
            memcpy(this->multi_target_buffer, data, len);
            break;
        default:
            break;
        }
    }
}

bool MDManager::get_init(md_config_t *md_config)
{
    if (this->init_flag)
    {
        this->init_flag = false;
        memcpy(md_config, this->init_buffer, sizeof(md_config_t));
        return true;
    }
    return false;
}

void MDManager::send_init(uint8_t md_kind)
{
    uint8_t data[1];
    memcpy(data, &md_kind, sizeof(uint8_t));
    this->send(can_config::encode_id(can_config::direction::master, can_config::board_type::md, this->board_id, can_config::data_type::md::init), data, sizeof(uint8_t));
}

bool MDManager::get_target(int16_t *target)
{
    if (this->target_flag)
    {
        this->target_flag = false;
        memcpy(target, this->target_buffer, sizeof(int16_t));
        return true;
    }
    return false;
}

void MDManager::send_encoder(int16_t encoder)
{
    uint8_t data[2];
    memcpy(data, &encoder, sizeof(int16_t));
    this->send(can_config::encode_id(can_config::direction::master, can_config::board_type::md, this->board_id, can_config::data_type::md::target), data, sizeof(int16_t));
}

void MDManager::send_limit_switch(uint8_t limit_switch)
{
    uint8_t data[1];
    memcpy(data, &limit_switch, sizeof(uint8_t));
    this->send(can_config::encode_id(can_config::direction::master, can_config::board_type::md, this->board_id, can_config::data_type::md::limit_switch), data, sizeof(uint8_t));
}

bool MDManager::get_gain(uint8_t gain_kind, float *gain)
{
    if (this->gain_flag)
    {
        this->gain_flag = false;
        memcpy(gain, this->gain_buffer, sizeof(float));
        return true;
    }
    return false;
}

void MDManager::send_gain(uint8_t gain_kind, float gain)
{
    uint8_t data[5];
    memcpy(data, &gain, sizeof(float));
    this->send(can_config::encode_id(can_config::direction::master, can_config::board_type::md, this->board_id, can_config::data_type::md::gain), data, sizeof(float));
}
