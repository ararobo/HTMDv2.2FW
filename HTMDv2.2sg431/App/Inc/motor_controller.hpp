#pragma once
#include <stdint.h>

class MotorController
{
private:
    static constexpr uint16_t ENCODER_MAX_COUNT = 65535; // エンコーダの最大カウント
    float Kp = 0.0f;                                     // PID制御の比例ゲイン
    float Ki = 0.0f;                                     // PID制御の積分ゲイン
    float Kd = 0.0f;                                     // PID制御の微分ゲイン
    float i_out = 0.0f;                                  // I制御の出力
    float prev_error = 0.0f;                             // 前回のエラー
    int16_t prev_out = 0;                                // 前回の出力
    uint8_t control_cycle = 5;                           // 制御周期

public:
    /**
     * @brief モーターの初期化
     *
     * @param max_output 最大出力
     * @param control_cycle 制御周期
     */
    void init(uint16_t max_output, uint8_t control_cycle);

    /**
     * @brief モーターを制御する
     *
     * @param output 出力
     * @param max_output 最大出力
     */
    void run(int16_t output, uint16_t max_output);

    /**
     * @brief トラペゾイダル制御を行う
     *
     * @param output 出力
     * @param max_acceleration 最大加速度
     * @return int16_t トラペゾイダル制御の出力
     */
    int16_t trapezoidalControl(int16_t output, uint8_t max_acceleration);

    /**
     * @brief PID制御を行う
     *
     * @param target 目標値
     * @param now_value 現在の値
     * @return float PID制御の出力
     */
    float calculatePID(float target, float now_value);

    /**
     * @brief エンコーダのカウントを取得する
     *
     * @return int16_t エンコーダのカウント
     */
    int16_t getCount();

    /**
     * @brief PIDゲインを設定する
     *
     * @param p_gain 比例ゲイン
     * @param i_gain 積分ゲイン
     * @param d_gain 微分ゲイン
     */
    void setPIDGain(float p_gain, float i_gain, float d_gain);

    /**
     * @brief 過去のPIDをリセットする
     *
     */
    void resetPID();

    /**
     * @brief 値を指定された範囲内に制限する汎用関数
     *
     * @tparam T 値の型
     * @param value 制限する値
     * @param min_value 最小値
     * @param max_value 最大値
     * @return T 制限された値
     */
    template <typename T>
    T saturate(T value, T min_value, T max_value);

    /**
     * @brief ブレーキをかける
     *
     * @param brake ブレーキをかけるかどうか
     */
    void setBrake(bool brake);
};
