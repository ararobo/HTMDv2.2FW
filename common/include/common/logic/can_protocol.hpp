#pragma once

#include <functional>
#include <optional>

#include "gn10_can/core/can_device.hpp"
#include "gn10_can/devices/motor_driver_types.hpp"
#include "gn10_can/utils/can_converter.hpp"

namespace common::logic {

/**
 * @brief モータードライバー（スレーブ側）のCANプロトコルハンドラ
 * 
 * ホスト（コントローラ）からの指令を受信し、内部状態を更新します。
 * また、フィードバック・ステータス情報をホストへ送信します。
 */
class MotorDriverSlave : public gn10_can::CANDevice {
  public:
    using MotorConfig = gn10_can::devices::MotorConfig;
    using GainType = gn10_can::devices::GainType;

    /**
     * @brief コンストラクタ
     * @param bus CANバスインスタンス
     * @param dev_id デバイスID
     */
    MotorDriverSlave(gn10_can::CANBus& bus, uint8_t dev_id)
        : gn10_can::CANDevice(bus, gn10_can::id::DeviceType::MotorDriver, dev_id) {}

    /**
     * @brief CANメッセージ受信時のコールバック
     * @param frame 受信フレーム
     */
    void on_receive(const gn10_can::CANFrame& frame) override {
        // IDのデコード
        auto id_fields = gn10_can::id::unpack(frame.id);

        // 自分宛てでなければ無視 (基本CANDBusでフィルタされるはずだが念のため)
        if (id_fields.type != device_type_ || id_fields.dev_id != device_id_) {
            return;
        }

        using MsgType = gn10_can::id::MsgTypeMotorDriver;

        // コマンド別処理
        if (id_fields.is_command(MsgType::Init)) {
            // 初期化・設定受信
            MotorConfig config;
            config.from_bytes(frame.data);
            if (on_config_received_) {
                on_config_received_(config);
            }
        
        } else if (id_fields.is_command(MsgType::Target)) {
            // 目標値受信
            float target = 0.0f;
            gn10_can::converter::unpack(frame.data.data(), frame.dlc, 0, target);
            if (on_target_received_) {
                on_target_received_(target);
            }

        } else if (id_fields.is_command(MsgType::Gain)) {
            // ゲイン設定受信
            if (frame.dlc >= 5) {
                auto type = static_cast<GainType>(frame.data[0]);
                float value = 0.0f;
                gn10_can::converter::unpack(frame.data.data(), frame.dlc, 1, value);
                if (on_gain_received_) {
                    on_gain_received_(type, value);
                }
            }
        }
    }

    // --- 送信メソッド ---

    /**
     * @brief フィードバック情報の送信 (周期送信推奨)
     * @param current_val 現在の値（位置または速度）
     * @param limit_switch_state リミットスイッチの状態ビットマスク
     */
    bool send_feedback(float current_val, uint8_t limit_switch_state) {
        std::array<uint8_t, 5> payload{};
        gn10_can::converter::pack(payload, 0, current_val);
        gn10_can::converter::pack(payload, 4, limit_switch_state);
        return send(gn10_can::id::MsgTypeMotorDriver::Feedback, payload);
    }

    /**
     * @brief ステータス情報の送信 (低頻度または変化時)
     * @param load_current 負荷電流 [A]
     * @param temperature 温度 [degC]
     */
    bool send_status(float load_current, int8_t temperature) {
        std::array<uint8_t, 5> payload{};
        gn10_can::converter::pack(payload, 0, load_current);
        gn10_can::converter::pack(payload, 4, temperature);
        return send(gn10_can::id::MsgTypeMotorDriver::Status, payload);
    }

    // --- コールバック登録 ---

    void set_on_config_received(std::function<void(const MotorConfig&)> cb) {
        on_config_received_ = cb;
    }

    void set_on_target_received(std::function<void(float)> cb) {
        on_target_received_ = cb;
    }

    void set_on_gain_received(std::function<void(GainType, float)> cb) {
        on_gain_received_ = cb;
    }

  private:
    std::function<void(const MotorConfig&)> on_config_received_;
    std::function<void(float)> on_target_received_;
    std::function<void(GainType, float)> on_gain_received_;
};

}  // namespace common::logic
