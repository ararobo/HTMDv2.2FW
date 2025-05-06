/**
 * @file a3921.hpp
 * @author  (8gn24gn25@gmail.com)
 * @brief A3921ゲートドライバのクラス
 * @version 0.1
 * @date 2025-05-06
 *
 * @copyright Copyright (c) 2025
 *
 */
#pragma once
#include <stdint.h>

class GateDriver
{
private:
    uint16_t max_duty = 3200; // 最大デューティ比

public:
    /**
     * @brief GateDriverのコンストラクタ
     *
     * @param max_duty 最大デューティ
     */
    GateDriver(uint16_t max_duty);

    /**
     * @brief ゲートドライバーの初期化
     *
     */
    void hardware_init();

    /**
     * @brief モーターの出力を設定する
     *
     * @param output 出力(duty)
     */
    void output(int16_t output);

    /**
     * @brief ブレーキを設定する
     *
     * @param brake ブレーキの有効/無効
     */
    void set_brake(bool brake);
};