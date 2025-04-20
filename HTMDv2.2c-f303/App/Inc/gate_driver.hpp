#pragma once
#include <stdint.h>
#include "motor_controller.hpp"

class GateDriver : public MotorController
{
private:
public:
    /**
     * @brief モーターの初期化
     *
     * @param max_output 最大出力
     * @param control_cycle 制御周期
     */
    void hardware_init() override;

    /**
     * @brief モーターを回す
     *
     * @param output 出力
     * @param max_output 最大出力
     */
    void run(int16_t output, uint16_t max_output) override;

    /**
     * @brief エンコーダのカウントを取得する
     *
     * @return int16_t エンコーダのカウント
     */
    int16_t get_count() override;

    /**
     * @brief ブレーキを設定する
     *
     * @param brake ブレーキの有効/無効
     */
    void set_brake(bool brake) override;
};