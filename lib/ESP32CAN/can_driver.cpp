#include "CAN.h"
#include "can_driver.hpp"

CANDriver::CANDriver(uint8_t rx_pin, uint8_t tx_pin)
    : can_rx_pin(rx_pin), can_tx_pin(tx_pin)
{
    instance = this;
}

CANDriver::init()
{
    CAN.setPins(can_rx_pin, can_tx_pin); // RX/TXピンの設定
    CAN.begin(1000E3);                   // ボーレートの設定
    CAN.onReceive(can_clallback_static); // 受信コールバックの設定
}

CANDriver::send(uint16_t id, uint8_t *data, uint8_t len)
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

CANDriver *CANDriver::instance = nullptr; // インスタンスのポインタ