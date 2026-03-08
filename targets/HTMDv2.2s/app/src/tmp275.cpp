/**
 * @file tmp275.cpp
 * @author Gento Aiba (aiba-gento)
 * @brief TMP275 温度センサードライバ
 * @version 0.2.0
 * @date 2026-03-08
 *
 * @copyright Copyright (c) 2026 ararobo
 * SPDX-License-Identifier: Apache-2.0
 *
 */
#include "app/tmp275.hpp"

TMP275::TMP275(I2C_HandleTypeDef& hi2c) : hi2c_(hi2c) {}

/**
 * @brief ポインタレジスタを温度レジスタ (0x00) に設定する
 *
 */
void TMP275::init()
{
    uint8_t pointer = 0x00;
    HAL_I2C_Master_Transmit(&hi2c_, ADDRESS << 1, &pointer, 1, TIMEOUT_MS);
}

/**
 * @brief 温度の生データを読み取る
 *
 * @param raw 生データを格納する変数のポインタ
 *            変換式: temperature_degC = static_cast<int16_t>(raw) / 256.0f
 * @return true 正常に読み取れた
 * @return false 読み取り失敗
 */
bool TMP275::read(int16_t* raw)
{
    if (HAL_I2C_Master_Receive(&hi2c_, ADDRESS << 1, buff_, 2, TIMEOUT_MS) != HAL_OK) {
        return false;
    }
    *raw = static_cast<int16_t>((buff_[0] << 8) | buff_[1]);
    return true;
}
