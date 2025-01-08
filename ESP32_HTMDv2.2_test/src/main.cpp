#include <Arduino.h>
#include <CAN.h> // CAN通信のライブラリ
#include <can_data_configure.hpp> // CAN通信のIDを生成するため&CAN通信でのパケットのサイズを取得するため。
#include <htmd_mode.hpp> // HTMDのモードを生成するため。

uint16_t can_id_md_target;

union can_data_md_target_t
{
  int16_t target[4];
  uint8_t raw_data[8];
};
can_data_md_target_t can_data_md_target;

void setup() {
  /* CAN通信を開始する */
  CAN.setPins(4, 5); // TODO : ESP32とCANトランシーバーの接続PinのRXとTXに編集してください。
  CAN.begin(1000E3);

  /* MDの初期化する */
  uint16_t can_id_md_init = encodeCanID(
    can_config::dir::to_slave, can_config::dev::motor_driver, 0, can_config::data_name::md::init);
  CAN.beginPacket(can_id_md_init); // CAN通信パケットの生成をMDの初期化用のIDで開始する。
  uint8_t can_tx_buffer_md_init[1]; // 初期化のデータのサイズで送信バッファを生成。
  can_tx_buffer_md_init[0] = 0; // 初期化は値0なので0を代入。
  CAN.write(can_tx_buffer_md_init[0]); // 送信バッファをCAN通信のパケットに追加。
  CAN.endPacket(); // 完成したパケットをCAN通信で送信する。

  /* MDのモードを設定する */
  md_mode_t can_tx_buffer_md_mode; // MDのモード型で送信バッファを生成。
  can_tx_buffer_md_mode.flags.absolute_encoder = false; // 絶対値エンコーダーを設定(無効)
  can_tx_buffer_md_mode.flags.incremental_encoder = false; // 速度エンコーダーを設定(無効)
  can_tx_buffer_md_mode.flags.brake = true; // 出力値がゼロの時にモーターにブレーキをかけるか設定(有効)
  can_tx_buffer_md_mode.flags.pid = false; // PID制御を設定(無効)
  can_tx_buffer_md_mode.flags.reverse_encoder = false; // エンコーダーの値の正負を反転するか設定(無効)
  can_tx_buffer_md_mode.flags.state = false; // MDの状態(初期化されたかどうか)を送信するか設定(無効)
  can_tx_buffer_md_mode.flags.stop_by_limit_switch = false; // MDに接続されたリミットスイッチが押された際にモーターを停止し、出力値がゼロになるまでモーターを停止し続けるか設定(無効)
  can_tx_buffer_md_mode.flags.torque_control = false; // 電流を読んでトルク制御するかどうか設定(無効)
  can_tx_buffer_md_mode.values.max_acceleration = 100; // モーターの1ミリ秒あたりの出力値の増加の最大値を設定(100)
  can_tx_buffer_md_mode.values.max_output = 3199; // モーターの出力値の最大値を設定(3199)
  can_tx_buffer_md_mode.values.max_current = 0; // モーターの最大電流値を設定(使わないのでゼロ)
  can_tx_buffer_md_mode.values.motor_transfer_coefficient = 10; // PID制御使用時に出力値とエンコーダーの値の間の比例定数を設定(10倍)
  can_tx_buffer_md_mode.values.report_rate = 100; // エンコーダーやリミットスイッチの状態を何ミリ秒ごとにCAN通信経由で送信するか設定(100ミリ秒ごと)
  uint16_t can_id_md_mode = encodeCanID(can_config::dir::to_slave, can_config::dev::motor_driver, 0, can_config::data_name::md::mode);
  CAN.beginPacket(can_id_md_mode);
  for (uint8_t i = 0; i < can_config::dlc::md::mode; i++)
  {
    CAN.write(can_tx_buffer_md_mode.code[i]);
  }
  CAN.endPacket();
}

void loop() {
  /* MDに出力値を送信する(0~3までのIDのMDの目標値をまとめて送信する。) */
  can_id_md_target = encodeCanID(
    can_config::dir::to_slave, can_config::dev::motor_driver, 0, can_config::data_name::md::targets);
  can_data_md_target.target[0] = 1000; // IDが0のモータードライバの目標値に1000を入れる
  can_data_md_target.target[1] = 1000; // IDが1のモータードライバの目標値に1000を入れる
  can_data_md_target.target[2] = 1000; // IDが2のモータードライバの目標値に1000を入れる
  can_data_md_target.target[3] = 1000; // IDが3のモータードライバの目標値に1000を入れる
  CAN.beginPacket(can_id_md_target);
  for (uint8_t i = 0; i < can_config::dlc::md::targets_4; i++)
  {
    CAN.write(can_data_md_target.raw_data[i]);
  }
  CAN.endPacket();

  /* MDに出力値を送信する(4~8までのIDのMDの目標値をまとめて送信する。) */
  can_id_md_target = encodeCanID(
    can_config::dir::to_slave, can_config::dev::motor_driver, 1, can_config::data_name::md::targets); // IDを1にすると4~8のIDになる。
  can_data_md_target.target[0] = 1000; // IDが0のモータードライバの目標値に1000を入れる
  can_data_md_target.target[1] = 1000; // IDが1のモータードライバの目標値に1000を入れる
  can_data_md_target.target[2] = 1000; // IDが2のモータードライバの目標値に1000を入れる
  can_data_md_target.target[3] = 1000; // IDが3のモータードライバの目標値に1000を入れる
  CAN.beginPacket(can_id_md_target);
  for (uint8_t i = 0; i < can_config::dlc::md::targets_4; i++)
  {
    CAN.write(can_data_md_target.raw_data[i]);
  }
  CAN.endPacket();

  delay(10);
}