#include "md_data_slave.hpp"
#include <cstring>

MDDataSlave::MDDataSlave(uint8_t board_id, uint8_t board_kind)
    : board_id(board_id), board_kind(board_kind)
{
}

void MDDataSlave::receive(uint16_t id, uint8_t *data, uint8_t len)
{
    // 受信パケットのCAN-IDからパケットの情報を取得
    can_config::decode_id(id, this->packet_direction, this->packet_board_type,
                          this->packet_board_id, this->packet_data_type);

    // 受信データのフィルタリング
    if (this->packet_direction == can_config::direction::slave &&
        this->packet_board_type == can_config::board_type::md &&
        this->packet_board_id == this->board_id)
    {
        // 受信フラグを立て、受信データをバッファに格納
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
            this->gain_flag[data[0]] = true;                   // gain_type別にフラグを立てる
            memcpy(this->gain_buffer[data[0]], data + 1, len); // gain_typeを除いたデータを該当するバッファにコピー
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

void MDDataSlave::send_init(uint8_t md_type)
{
    // CAN-IDの生成
    uint16_t can_id = can_config::encode_id(
        can_config::direction::master,
        can_config::board_type::md,
        this->board_id,
        can_config::data_type::md::init);

    // 送信
    this->send(can_id, &md_type, sizeof(uint8_t));
}

void MDDataSlave::send_encoder(int16_t encoder)
{
    uint8_t data[2];
    // CAN-IDの生成
    uint16_t can_id = can_config::encode_id(
        can_config::direction::master,
        can_config::board_type::md,
        this->board_id,
        can_config::data_type::md::target);
    // エンコーダーの値をコピー
    memcpy(data, &encoder, sizeof(int16_t));
    // 送信
    this->send(can_id, data, sizeof(int16_t));
}

void MDDataSlave::send_limit_switch(uint8_t limit_switch)
{
    // CAN-IDの生成
    uint16_t can_id = can_config::encode_id(
        can_config::direction::master,
        can_config::board_type::md,
        this->board_id,
        can_config::data_type::md::limit_switch);
    // 送信
    this->send(can_id, &limit_switch, sizeof(uint8_t));
}

void MDDataSlave::send_gain(uint8_t gain_type, float gain)
{
    uint8_t data[5];
    // CAN-IDの生成
    uint16_t can_id = can_config::encode_id(
        can_config::direction::master,
        can_config::board_type::md,
        this->board_id,
        can_config::data_type::md::gain);
    // gain_typeをコピー
    data[0] = gain_type;
    // gain_typeを除いたデータを該当するバッファにコピー
    memcpy(data + 1, &gain, sizeof(float));
    // 送信
    this->send(can_id, data, sizeof(data));
}

bool MDDataSlave::get_init(md_config_t *md_config)
{
    if (this->init_flag)
    {
        this->init_flag = false;
        memcpy(md_config, this->init_buffer, sizeof(md_config_t));
        return true;
    }
    return false;
}

bool MDDataSlave::get_target(int16_t *target)
{
    if (this->target_flag)
    {
        this->target_flag = false;
        memcpy(target, this->target_buffer, sizeof(int16_t));
        return true;
    }
    return false;
}

bool MDDataSlave::get_gain(uint8_t gain_type, float *gain)
{
    if (this->gain_flag[gain_type])
    {
        this->gain_flag[gain_type] = false;
        memcpy(gain, this->gain_buffer[gain_type], sizeof(float));
        return true;
    }
    return false;
}