/**
 * @file my_i2c.cpp
 * @author Gento Aiba
 * @brief 温度センサーと電流センサーの値をI2C通信で取得するクラス
 * @version 0.1
 * @date 2024-04-02
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "my_i2c.hpp"

/**
 * @brief I2C通信の初期化
 *
 */
void MyI2C::init()
{
    flag_temp = false;
    flag_current = false;
}

/**
 * @brief 温度センサーの値を取得
 *
 * @param temp 温度センサーの値を格納したい変数のポインタ
 * @return true 新しい値に更新した
 * @return false 新しい値が無い
 */
bool MyI2C::getTemp(float *temp)
{
    if (flag_temp)
    {
        *temp = (float)((buff[0] << 8) | buff[1]) / 100.0f;
        flag_temp = false;
        return true;
    }
    return false;
}

/**
 * @brief 電流センサーの値を取得
 *
 * @param current 電流センサーの値を格納したい変数のポインタ
 * @return true 新しい値に更新した
 * @return false 新しい値が無い
 */
bool MyI2C::getCurrent(float *current)
{
    if (flag_current)
    {
        *current = (float)((buff[2] << 8) | buff[3]) / 100.0f;
        flag_current = false;
        return true;
    }
    return false;
}