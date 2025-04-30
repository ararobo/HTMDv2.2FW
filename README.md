# HTMDv2.2FW
GN10が開発しているHTMDv2.2系のファームウェアです。

HTMDv2.2cはSTM32F303K8T6と、STM32G431K8T6の2つのMCUに対応している。

## ファイル紹介

[CAN通信v2.0について](docs/CANv2.0.md)

[HTMDv2.2cのSTM32F303用ファームウェア](HTMDv2.2c-f303)

## [ライブラリ](lib)

MDのCANv2.0のデータを管理するクラスです。

説明：[MDDataManagerの説明](lib/Common/MDDataMaster.md)

これはどのマイコンやPCでも使うことができます。

ESP32の場合は[ESP32でCAN通信をするときに使用する](lib\ESP32CAN)このフォルダをPlatform ioの`lib`フォルダに直接入れてください。

ESP32の場合はCANのRXとTXピンを設定しなくてはならないため、initメソッドの前にset_pinsメソッドを呼び出して設定してください。

STM32の場合はCommonとSTM32G4シリーズならCANFDなので、STM32FDCANを組み合わせて使用する。それ以外のF3等では普通のCANなのでSTM32CANを組み合わせてください。

その際はCommon内部のMDDataMaster/Slaveの継承元やインクルードパスを変更して対応してください。

[CAN通信のデータを管理するクラス](lib\Common)

自分の使いたい環境に対応するクラスを用いてCAN通信しましょう。

[CAN付きのSTM32でCAN通信するときに継承するクラス](lib\STM32CAN)

[FDCAN付きのSTM32でCAN通信するときに継承するクラス](lib\STM32FDCAN)