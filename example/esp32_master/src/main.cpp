#include <Arduino.h>
#include <can_driver.hpp>

CANDriver canDriver;
md_config_t mdConfig[8];
int8_t ps4_l_x = 100;

void setup()
{
  canDriver.set_pins(32, 33); // RXピン32, TXピン33を設定
  canDriver.init(0, 0);       // CANの初期化
  mdConfig[0].max_output = 3199;
  mdConfig[0].max_acceleration = 10;
  mdConfig[0].control_period = 10;
  mdConfig[0].encoder_period = 10;
  mdConfig[0].encoder_type = 1;
  mdConfig[0].limit_switch_behavior = 0;
  mdConfig[0].option = 0;

  canDriver.send_init(0, &mdConfig[0]); // MDの初期化コマンドを送信
}

void loop()
{
  float controller_l_x = ps4_l_x / 128.0;
  float motor_1_target = controller_l_x * 2.0;
  canDriver.send_targets(0, motor_1_target);
  delay(10);
}