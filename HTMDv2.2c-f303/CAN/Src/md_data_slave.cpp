/**
 * @file md_data_slave.cpp
 * @author gn10g (8gn24gn25@gmail.com)
 * @brief MDのデータを扱うクラス
 * @version 2.2
 * @date 2025-06-11
 *
 * @copyright Copyright (c) 2025
 *
 */
#include "md_data_slave.hpp"
#include <cstring>

MDDataSlave::MDDataSlave()
{
    // 受信フラグの初期化
    this->init_flag = false;
    this->target_flag = false;
    this->limit_switch_flag = false;
    this->float_target_flag = false;
    for (int i = 0; i < 3; i++)
    {
        this->gain_flag[i] = false;
    }
}

void MDDataSlave::set_board_id(uint8_t board_id)
{
    this->board_id = board_id;
    // multi_target受信用の計算
    this->multi_target_id = board_id / 4;
    this->multi_target_position = (board_id % 4) * sizeof(int16_t);
}

void MDDataSlave::receive(uint16_t id, uint8_t *data, uint8_t len)
{
    // 受信パケットのCAN-IDからパケットの情報を取得
    can_config::decode_id(id, this->packet_direction, this->packet_board_type,
                          this->packet_board_id, this->packet_data_type);
    // 受信データのフィルタリング
    if (this->packet_direction != can_config::direction::slave ||
        this->packet_board_type != can_config::board_type::md)
        return;
    // multi_target受信時の処理
    if (this->packet_board_id == multi_target_id &&
        this->packet_data_type == can_config::data_type::md::multi_target)
    {
        if (len != 8)
            return;
        // 受信フラグを立て、受信データをバッファに格納
        this->target_flag = true;
        memcpy(this->target_buffer, data + multi_target_position, len);
    }
    // multi_target以外の処理
    else if (this->packet_board_id == this->board_id)
    {
        // 受信フラグを立て、受信データをバッファに格納
        switch (this->packet_data_type)
        {
        case can_config::data_type::md::init:
            if (len != 8)
                return;
            this->init_flag = true;
            memcpy(this->init_buffer, data, len);
            break;

        case can_config::data_type::md::target:
            if (len != 2)
                return;
            this->target_flag = true;
            memcpy(this->target_buffer, data, len);
            break;

        case can_config::data_type::md::gain:
            if (len != 5)
                return;
            this->gain_flag[data[0]] = true;                   // gain_type別にフラグを立てる
            memcpy(this->gain_buffer[data[0]], data + 1, len); // gain_typeを除いたデータを該当するバッファにコピー
            break;
        case can_config::data_type::md::float_target:
            if (len != 4)
                return;
            this->float_target_flag = true;
            memcpy(this->float_target_buffer, data, len);
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

void MDDataSlave::send_float_encoder(float encoder)
{
    uint8_t data[4];
    // CAN-IDの生成
    uint16_t can_id = can_config::encode_id(
        can_config::direction::master,
        can_config::board_type::md,
        this->board_id,
        can_config::data_type::md::float_target);
    // エンコーダーの値をコピー
    memcpy(data, &encoder, sizeof(float));
    // 送信
    this->send(can_id, data, sizeof(float));
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

bool MDDataSlave::get_float_target(float *target)
{
    if (this->float_target_flag)
    {
        this->float_target_flag = false;
        memcpy(target, this->float_target_buffer, sizeof(float));
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