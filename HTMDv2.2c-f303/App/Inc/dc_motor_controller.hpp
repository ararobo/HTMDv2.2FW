/**
 * @file dc_motor_controller.hpp
 * @author  (8gn24gn25@gmail.com)
 * @brief DCモーターの制御クラス
 * @version 1.0
 * @date 2025-05-10
 *
 * @copyright Copyright (c) 2025
 *
 */
#pragma once
#include "md_config.hpp"

class MotorController
{
private:
    md_config_t md_config; // モータードライバの設定
    int16_t encoder_count; // エンコーダのカウント

public:
    MotorController();

    float encoder_total;

    /**
     * @brief エンコーダーとゲートドライバの初期化
     *
     */
    void init();

    /**
     * @brief 台形制御とPID制御の初期化
     *
     */
    void reset();

    /**
     * @brief モーターの制御を行う
     *
     * @param output 制御値
     * @note PID制御（Pゲイン設定時）と台形制御あり
     */
    void run(float output);

    /**
     * @brief MDの設定
     *
     * @param config md_config
     */
    void set_config(md_config_t config);

    /**
     * @brief モーターの出力を停止する
     *
     */
    void stop();

    /**
     * @brief PIDゲインの設定
     *
     * @param p_gain Pゲイン
     * @param i_gain Iゲイン
     * @param d_gain Dゲイン
     * @note PID制御を行う場合は設定してください
     */
    void set_pid_gain(float p_gain, float i_gain, float d_gain);

    /**
     * @brief エンコーダーのサンプリング
     *
     */
    void sample_encoder();

    /**
     * @brief エンコーダーの値を取得する
     *
     * @return int16_t エンコーダーのカウント
     * @note カウンタのリセットも行う
     */
    int16_t get_encoder_count();
};