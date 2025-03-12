# CAN通信
GN10が考えるCAN通信

## CAN-ID
11bitの標準IDを定義します。

|id|10(MSB)|9|8|7|6|5|4|3|2|1|0(LSB)|
|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|
|用途|DIR|DEV|DEV|DEV|ID|ID|ID|ID|KIND|KIND|KIND|

DIR : 1.direction
DEV : 2.device
ID  : 3.identifire
KIND: 4.data_kind

### 1. direction
制御基板(Master)とその他基板(Slave)の通信方向を0と1で示す。

|向き|1.の値|
|:-:|:-:|
|to_slave(PCからMD等)|0|
|to_master(MDからPC等)|1|

### 2. device
デバイスの種類を0~7表す。

|デバイス|2.の値|
|:-:|:-:|
|共通|0|
|モータードライバー|1|
|サーボモータードライバー|2|
|ソレノイドドライバー|3|
|インジケーター|4|
|遠隔非常停止|5|
|センサー|6|
|未定|7|

### 3. identifire
デバイスのIDを0~15で示す。
例）モータードライバーが複数ある場合

### 4. data_kind
通信内容を0~7で表す。
それぞれのデバイスに対して定義される。
詳しくは下のデバイスごとの説明を参照してください。

## モータードライバー
多くの通信が定義されていますが、MDによって有効なものと無効なものがあります。
4つのモータの出力を同時に送信するデータフレームがありますが、これは一つのIDに対して送るデータではなく、４つのIDに対して送るデータです。これは、CANのバスが溢れないようにするため、一回の通信で複数のデバイスに送信する工夫です。

|4.data_kindの値|データ長|名称|説明|データフレーム|
|:-:|:-:|:-:|:-:|:-:|
|0|1|init|初期化|0(固定) : uint8|
|1|2|targets|モーターの出力|出力値 : int16|
|1|8|targets|4つのモーターの出力|出力値x4 : int16[4]|
|2|8|mode|MDのモード|md_mode : md_mode_t|
|3|6|p_gain|Pゲイン|P_gain : float|
|4|6|i_gain|Iゲイン|I_gain : float|
|5|6|d_gain|Dゲイン|D_gain : float|
|6|1|sensor|リミットスイッチ|limSW1(0x1)とlimSW2(0x2)の論理和 : uint8|
|6|3|sensor|リミットスイッチとエンコーダー|limSW1(0x1)とlimSW2(0x2)の論理和 : uint8, encoder : int16|
|6|5|sensor|リミットスイッチとエンコーダーと電流値|limSW1(0x1)とlimSW2(0x2)の論理和 : uint8, encoder : int16, current : uint16|
|7|1|status|MDの状態|状態 : uint8|
|7|3|status|MDの状態と温度|状態 : uint8, 温度 : uint16|

#### MDのモードについて
md_mode_tとされているが、中身がわからないと思うので、変換する際に使用するunionを書いておく
```c++
union md_mode_t
{
    struct
    {
        struct
        {
            unsigned char incremental_encoder : 1;
            unsigned char absolute_encoder : 1;
            unsigned char reverse_encoder : 1;
            unsigned char brake : 1;
            unsigned char pid : 1;
            unsigned char stop_by_limit_switch : 1;
            unsigned char torque_control : 1;
            unsigned char state : 1;
        } __attribute__((__packed__)) flags;
        struct
        {
            uint8_t max_acceleration;
            uint8_t max_current;
            uint8_t report_rate;
            uint16_t max_output;
            uint16_t motor_transfer_coefficient;
        } __attribute__((__packed__)) values;
    } __attribute__((__packed__));
    uint8_t code[8];
} __attribute__((__packed__));
```

使用方法
```c++
md_mode_t md_mode;
md_mode.flags.incremental_encoder = 1; // インクリメンタルエンコーダーを有効にする。
/* その他フラグに値を代入する */
md_mode.values.max_acceleration = 255; // 台形制御の最大加速を255に設定。
/* その他パラメーターに値を代入する */
// ESP32の例
CAN.beginPacket(0x82); // md_id=0としてIDを指定
for (uint8 i = 0; i < 8; i++) // 8byte
{
	CAN.write(md_mode.code[i]); // unionで変換されたデータを送信する
}
CAN.endPacket();
```

#### MDを動かすときに送信するCAN通信の順番
1. init
2. md_mode（初期値はすべて0のため、一切動かないので必須）
3. PID制御をするのであれば、p_gain,i_gain,d_gain（初期値0）
4. 出力値等


## サーボモータードライバー

|4.の値|データ長|内容|データフレーム|
|:-:|:-:|:-:|:-:|
|0|2|パルス幅(servo_id:0~F)[us]|パルス幅 : uint16|
|0|8|パルス幅(servo_id:0~3, ..., 3C~3F)[us]|パルス幅x4 : uint16[4]|
|1|1|初期化|0(固定) : uint8|
|2|6|周波数|freq : float|

## ソレノイドドライバー

|4.の値|データ長|内容|データフレーム|
|:-:|:-:|:-:|:-:|
|0|1|出力x8|各bitごとに一つのソレノイドに対応する : uint8|
|1|1|初期化|0(固定) : uint8|

## LEDドライバー

未定

## 遠隔非常停止

未定