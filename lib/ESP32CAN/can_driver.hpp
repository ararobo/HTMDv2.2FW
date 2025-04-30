/**
 * @file can_driver.hpp
 * @author gn10g (8gn24gn25@gmail.com)
 * @brief ESP32のCAN通信クラス
 * @version 0.1
 * @date 2025-04-26
 *
 * @copyright Copyright (c) 2025
 *
 */
#pragma once
#include <stdint.h>

class CANDriver
{
private:
    uint8_t can_rx_pin;
    uint8_t can_tx_pin;
    static CANDriver *instance; // インスタンスのポインタ

    static void can_clallback_static(int packet_size)
    {
        if (instance != nullptr)
        {
            instance->can_callback_process(packet_size);
        }
    }

protected:
    /**
     * @brief CANの受信処理(オーバーライドしてください)
     *
     * @param id CANのID
     * @param data 受信データ
     * @param len データの長さ
     */
    virtual void receive(uint16_t id, uint8_t *data, uint8_t len);

public:
    /**
     * @brief コンストラクタ
     *
     */
    CANDriver();

    /**
     * @brief CANのRX/TXピンを設定する
     *
     * @param rx_pin RXピン
     * @param tx_pin TXピン
     */
    void set_pins(uint8_t rx_pin, uint8_t tx_pin);

    /**
     * @brief CANの初期化
     *
     * @param filter_id
     * @param filter_mask
     */
    void init(uint32_t filter_id, uint32_t filter_mask);

    /**
     * @brief CANの送信処理
     *
     * @param id CANのID
     * @param data 送信するデータ
     * @param len データの長さ
     */
    void send(uint16_t id, uint8_t *data, uint8_t len);

    /**
     * @brief CANのコールバック処理
     *
     */
    void can_callback_process(int packet_size);
};