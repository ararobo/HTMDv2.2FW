/**
 * @file can_driver.cpp
 * @author gn10g (8gn24gn25@gmail.com)
 * @brief ESP32のCAN通信クラス
 * @version 0.1
 * @date 2025-04-26
 *
 * @copyright Copyright (c) 2025
 *
 */
#include "CAN.h"
#include "can_driver.hpp"

CANDriver::CANDriver()
{
    instance = this;
}

void CANDriver::set_pins(uint8_t rx_pin, uint8_t tx_pin)
{
    can_rx_pin = rx_pin; // RXピンの設定
    can_tx_pin = tx_pin; // TXピンの設定
}

void CANDriver::init(uint32_t filter_id, uint32_t filter_mask)
{
    CAN.setPins(can_rx_pin, can_tx_pin); // RX/TXピンの設定
    CAN.begin(1000E3);                   // ボーレートの設定
    volatile uint32_t *pREG_IER = (volatile uint32_t *)0x3ff6b010;
    *pREG_IER &= ~(uint8_t)0x10;
    CAN.onReceive(can_clallback_static); // 受信コールバックの設定
}

void CANDriver::send(uint16_t id, uint8_t *data, uint8_t len)
{
    CAN.beginPacket(id); // CANのIDを設定
    for (uint8_t i = 0; i < len; i++)
    {
        CAN.write(data[i]); // データの送信
    }
    CAN.endPacket(); // 送信完了
}

void CANDriver::can_callback_process(int packet_size)
{
    uint16_t id = CAN.packetId(); // 受信したCANのID
    uint8_t data[8];              // 受信データ
    for (uint8_t i = 0; i < packet_size; i++)
    {
        data[i] = CAN.read(); // データの受信
    }

    receive(id, data, packet_size); // 受信処理
}

void CANDriver::receive(uint16_t id, uint8_t *data, uint8_t len)
{
    // 受信処理をオーバーライドしてください
}

CANDriver *CANDriver::instance = nullptr; // インスタンスのポインタ