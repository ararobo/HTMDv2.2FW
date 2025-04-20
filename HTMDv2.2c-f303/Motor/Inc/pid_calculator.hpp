#pragma once
#include <stdint.h>

class PIDCalculator
{
private:
    float Kp = 0.0f;         // PID制御の比例ゲイン
    float Ki = 0.0f;         // PID制御の積分ゲイン
    float Kd = 0.0f;         // PID制御の微分ゲイン
    float i_out = 0.0f;      // I制御の出力
    float prev_error = 0.0f; // 前回のエラー
    float dt;                // 制御周期

public:
    /**
     * @brief PID制御のコンストラクタ
     *
     * @param dt 制御周期[s]
     */
    PIDCalculator(float dt);

    /**
     * @brief PID制御を行う
     *
     * @param target 目標値
     * @param now_value 現在の値
     * @return float PID制御の出力
     */
    float calculate_pid(float target, float now_value);

    /**
     * @brief PIDゲインを設定する
     *
     * @param p_gain 比例ゲイン
     * @param i_gain 積分ゲイン
     * @param d_gain 微分ゲイン
     */
    void set_pid_gain(float p_gain, float i_gain, float d_gain);

    /**
     * @brief PIDゲインを取得する
     *
     * @param p_gain 比例ゲイン
     * @param i_gain 積分ゲイン
     * @param d_gain 微分ゲイン
     */
    void get_pid_gain(float *p_gain, float *i_gain, float *d_gain);

    /**
     * @brief 制御周期を設定する
     *
     * @param dt 制御周期[s]
     */
    void set_dt(float dt);

    /**
     * @brief PID制御の初期化
     *
     */
    void reset_pid();
};