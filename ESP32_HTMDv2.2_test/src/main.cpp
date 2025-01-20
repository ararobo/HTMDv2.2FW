#include <Arduino.h>
#include <CAN.h>                  // CAN通信のライブラリ
#include <can_data_configure.hpp> // CAN通信のIDを生成するため&CAN通信でのパケットのサイズを取得するため。
#include <htmd_mode.hpp>          // HTMDのモードを生成するため。

union can_data_md_target_t // MDの目標値とCAN通信のデータを変換するための共用体
{
  int16_t target[4];
  uint8_t raw_data[8];
};
can_data_md_target_t can_data_md_target; // MDの目標値を格納するための共用体

// CAN Buffer
bool can_rx_buffer_init[8];         // 初期化の状態を格納するためのバッファ。
md_mode_t can_rx_buffer_md_mode[8]; // MDのモード型で受信バッファを生成。
float can_rx_buffer_pid_gain[8][3]; // PID制御のゲインの値を格納するためのバッファ。
uint8_t can_rx_buffer_limit[8];     // リミットスイッチの状態を格納するためのバッファ。
int16_t can_rx_buffer_encoder[8];   // エンコーダの値を格納するためのバッファ。
uint8_t can_rx_buffer_current[8];   // 電流センサーの値を格納するためのバッファ。
// CAN ID
uint16_t can_id_md_target;
// 制御
int16_t motor_targets[8] = {0}; // モーターの出力値

/**
 * @brief CAN通信の受信時に呼び出される関数
 *
 * @param packet_size 受信したパケットのサイズ
 */
void onReceive(int packet_size)
{
  uint8_t dir, dev, device_id, data_name;                  // CAN通信のIDをデコードするための変数
  uint16_t can_id = CAN.packetId();                        // 受信したCAN通信のIDを取得
  decodeCanID(can_id, &dir, &dev, &device_id, &data_name); // CAN通信のIDをデコード
  if (dir == can_config::dir::to_master)                   // メイン基板への通信の場合
  {
    if (dev == can_config::dev::motor_driver) // デバイスの種類がモータードライバの場合
    {
      switch (data_name) // データの種類によって処理を分岐
      {
      case can_config::data_name::md::init: // 初期化のデータの場合
      {
        can_rx_buffer_init[device_id] = true; // 初期化の状態をtrueにする。
        break;
      }
      case can_config::data_name::md::mode: // モードのデータの場合
      {
        for (uint8_t i = 0; i < packet_size; i++) // パケットサイズ分データを読み込む
        {
          can_rx_buffer_md_mode[device_id].code[i] = CAN.read(); // モードのデータを受信バッファに格納
        }
        break;
      }
      case can_config::data_name::md::p_gain: // PID制御の比例ゲインのデータの場合
      {
        uint32_t p_gain_raw = 0; // 比例ゲインの値を格納するための変数
        for (uint8_t i = 0; i < packet_size; i++)
        {
          p_gain_raw |= (uint32_t)CAN.read() << (8 * i);
        }
        can_rx_buffer_pid_gain[device_id][0] = static_cast<float>(p_gain_raw); // static_castでfloat型に変換して受信バッファに格納
      }
      case can_config::data_name::md::i_gain: // PID制御の積分ゲインのデータの場合
      {
        uint32_t i_gain_raw = 0;
        for (uint8_t i = 0; i < packet_size; i++)
        {
          i_gain_raw |= (uint32_t)CAN.read() << (8 * i);
        }
        can_rx_buffer_pid_gain[device_id][1] = static_cast<float>(i_gain_raw);
      }
      case can_config::data_name::md::d_gain: // PID制御の微分ゲインのデータの場合
      {
        uint32_t d_gain_raw = 0;
        for (uint8_t i = 0; i < packet_size; i++)
        {
          d_gain_raw |= (uint32_t)CAN.read() << (8 * i);
        }
        can_rx_buffer_pid_gain[device_id][2] = static_cast<float>(d_gain_raw);
      }
      case can_config::data_name::md::sensor: // センサーのデータの場合
      {
        // パケットサイズによって内容を判別
        if (packet_size == can_config::dlc::md::limit) // リミットスイッチのデータの場合
        {
          can_rx_buffer_limit[device_id] = CAN.read(); // リミットスイッチの状態を受信バッファに格納
        }
        else if (packet_size == can_config::dlc::md::limit_encoder) // リミットスイッチとエンコーダーのデータの場合
        {
          uint16_t encoder_raw = 0;
          can_rx_buffer_limit[device_id] = CAN.read();
          for (uint8_t i = 0; i < 2; i++)
          {
            encoder_raw |= (uint16_t)CAN.read() << (8 * i);
          }
          can_rx_buffer_encoder[device_id] = static_cast<int16_t>(encoder_raw); // エンコーダの値を受信バッファに格納
        }
        else if (packet_size == can_config::dlc::md::limit_encoder_current) // リミットスイッチとエンコーダーと電流センサーのデータの場合
        {
          uint16_t encoder_raw = 0;
          can_rx_buffer_limit[device_id] = CAN.read();
          for (uint8_t i = 0; i < 2; i++)
          {
            encoder_raw |= (uint16_t)CAN.read() << (8 * i);
          }
          can_rx_buffer_encoder[device_id] = static_cast<int16_t>(encoder_raw);
          can_rx_buffer_current[device_id] = CAN.read();
        }
      }
      }
    }
  }
}
void setup()
{
  /* CAN通信を開始する */
  CAN.setPins(4, 5); // TODO : ESP32とCANトランシーバーの接続PinのRXとTXに編集してください。
  CAN.begin(1000E3);
  // ESP32のCANライブラリのバグを回避するためのコード
  volatile uint32_t *pREG_IER = (volatile uint32_t *)0x3ff6b010;
  *pREG_IER &= ~(uint8_t)0x10;
  CAN.onReceive(onReceive); // CAN通信でデータを受信した際に呼び出される関数を設定

  /* MDのモードを設定する */
  md_mode_t can_tx_buffer_md_mode;                              // MDのモード型で送信バッファを生成。
  can_tx_buffer_md_mode.flags.absolute_encoder = false;         // 絶対値エンコーダーを設定(無効)
  can_tx_buffer_md_mode.flags.incremental_encoder = false;      // 速度エンコーダーを設定(無効)
  can_tx_buffer_md_mode.flags.brake = true;                     // 出力値がゼロの時にモーターにブレーキをかけるか設定(有効)
  can_tx_buffer_md_mode.flags.pid = false;                      // PID制御を設定(無効)
  can_tx_buffer_md_mode.flags.reverse_encoder = false;          // エンコーダーの値の正負を反転するか設定(無効)
  can_tx_buffer_md_mode.flags.state = false;                    // MDの状態(初期化されたかどうか)を送信するか設定(無効)
  can_tx_buffer_md_mode.flags.stop_by_limit_switch = false;     // MDに接続されたリミットスイッチが押された際にモーターを停止し、出力値がゼロになるまでモーターを停止し続けるか設定(無効)
  can_tx_buffer_md_mode.flags.torque_control = false;           // 電流を読んでトルク制御するかどうか設定(無効)
  can_tx_buffer_md_mode.values.max_acceleration = 100;          // モーターの1ミリ秒あたりの出力値の増加の最大値を設定(100)
  can_tx_buffer_md_mode.values.max_output = 3199;               // モーターの出力値の最大値を設定(3199)
  can_tx_buffer_md_mode.values.max_current = 0;                 // モーターの最大電流値を設定(使わないのでゼロ)
  can_tx_buffer_md_mode.values.motor_transfer_coefficient = 10; // PID制御使用時に出力値とエンコーダーの値の間の比例定数を設定(10倍)
  can_tx_buffer_md_mode.values.report_rate = 100;               // エンコーダーやリミットスイッチの状態を何ミリ秒ごとにCAN通信経由で送信するか設定(100ミリ秒ごと)
  uint16_t can_id_md_mode = encodeCanID(can_config::dir::to_slave, can_config::dev::motor_driver, 0, can_config::data_name::md::mode);
  CAN.beginPacket(can_id_md_mode);
  for (uint8_t i = 0; i < can_config::dlc::md::mode; i++)
  {
    CAN.write(can_tx_buffer_md_mode.code[i]);
  }
  CAN.endPacket();

  /* MDの初期化する */
  uint16_t can_id_md_init = encodeCanID(
      can_config::dir::to_slave, can_config::dev::motor_driver, 0, can_config::data_name::md::init);
  CAN.beginPacket(can_id_md_init);     // CAN通信パケットの生成をMDの初期化用のIDで開始する。
  uint8_t can_tx_buffer_md_init[1];    // 初期化のデータのサイズで送信バッファを生成。
  can_tx_buffer_md_init[0] = 0;        // 初期化は値0なので0を代入。
  CAN.write(can_tx_buffer_md_init[0]); // 送信バッファをCAN通信のパケットに追加。
  CAN.endPacket();                     // 完成したパケットをCAN通信で送信する。
}

void loop()
{
  /* MDに出力値を送信する(0~3までのIDのMDの目標値をまとめて送信する。) */
  can_id_md_target = encodeCanID(
      can_config::dir::to_slave, can_config::dev::motor_driver, 0, can_config::data_name::md::targets);
  can_data_md_target.target[0] = motor_targets[0]; // IDが0のモータードライバの目標値に1000を入れる
  can_data_md_target.target[1] = motor_targets[1]; // IDが1のモータードライバの目標値に1000を入れる
  can_data_md_target.target[2] = motor_targets[2]; // IDが2のモータードライバの目標値に1000を入れる
  can_data_md_target.target[3] = motor_targets[3]; // IDが3のモータードライバの目標値に1000を入れる
  CAN.beginPacket(can_id_md_target);
  for (uint8_t i = 0; i < can_config::dlc::md::targets_4; i++)
  {
    CAN.write(can_data_md_target.raw_data[i]);
  }
  CAN.endPacket();

  /* MDに出力値を送信する(4~8までのIDのMDの目標値をまとめて送信する。) */
  can_id_md_target = encodeCanID(
      can_config::dir::to_slave, can_config::dev::motor_driver, 1, can_config::data_name::md::targets); // IDを1にすると4~8のIDになる。
  can_data_md_target.target[0] = motor_targets[4];                                                      // IDが0のモータードライバの目標値に1000を入れる
  can_data_md_target.target[1] = motor_targets[5];                                                      // IDが1のモータードライバの目標値に1000を入れる
  can_data_md_target.target[2] = motor_targets[6];                                                      // IDが2のモータードライバの目標値に1000を入れる
  can_data_md_target.target[3] = motor_targets[7];                                                      // IDが3のモータードライバの目標値に1000を入れる
  CAN.beginPacket(can_id_md_target);
  for (uint8_t i = 0; i < can_config::dlc::md::targets_4; i++)
  {
    CAN.write(can_data_md_target.raw_data[i]);
  }
  CAN.endPacket();

  delay(10);
}