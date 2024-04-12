/**
 * @file my_i2c.hpp
 * @author Gento Aiba
 * @brief 温度センサーと電流センサーの値をI2C通信で取得するクラス
 * @version 0.1
 * @date 2024-04-02
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "i2c.h"
#include "gpio.h"

/**
 * @brief 温度センサーと電流センサーの値をI2C通信で取得するクラス
 *
 */
class MyI2C
{
private:
    // buffer
    uint8_t buff[4];
    // flag
    bool flag_temp;
    bool flag_current;

public:
    /**
     * @brief I2C通信の初期化
     *
     */
    void init();

    /**
     * @brief 温度センサーの値を取得
     *
     * @param temp 温度センサーの値を格納したい変数のポインタ
     * @return true 新しい値に更新した
     * @return false 新しい値が無い
     */
    bool getTemp(float *temp);

    /**
     * @brief 電流センサーの値を取得
     *
     * @param current 電流センサーの値を格納したい変数のポインタ
     * @return true 新しい値に更新した
     * @return false 新しい値が無い
     */
    bool getCurrent(float *current);
};