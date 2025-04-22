# HTMDv2.2FW
GN10が開発しているHTMDv2.2系のファームウェアです。

HTMDv2.2cはSTM32F303K8T6と、STM32G431K8T6の2つのMCUに対応している。

## ファイル紹介

[CAN通信v2.0について](docs/can_communication_v2.0.md)

[HTMDv2.2cのSTM32F303用ファームウェア](HTMDv2.2c-f303)

## [ライブラリ](lib)

MDのCANv2.0のデータを管理するクラスです。
これはどのマイコンやPCでも使うことができます。
継承元やインクルードパスを変更して対応してください。

[CAN通信のデータを管理するクラス](lib\Common)

説明：[MDDataManagerの説明](lib/Common/MDDataMaster.md)

自分の使いたい環境に対応するクラスを用いてCAN通信しましょう。

[ESP32でCAN通信をするときに継承するクラス](lib\ESP32CAN)

[CAN付きのSTM32でCAN通信するときに継承するクラス](lib\STM32CAN)

[FDCAN付きのSTM32でCAN通信するときに継承するクラス](lib\STM32FDCAN)