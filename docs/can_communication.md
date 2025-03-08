# MDのCAN通信
CAN IDやデータフレームを定義し、その際のモータードライバーの動きも記載する。

# IDについて
IDには、bit毎に役割を設けております。
多くのデバイスを干渉すること無く使用することが考慮されており、また、通信方向が区別されるため、CAN通信にフィルタをかけることが可能です。 
bit毎の割り当てを下記に記します。

|bit|名称|内容|詳細|
|:-:|:-:|:-:|:-:|
|1~3|data_type|データの種類を示す|各基板毎に違うので、基板毎の資料を確認すること|
|4~7|board_id|同じ種類の基板を区別|0~15個の基板を同じCANBUS内で扱える(MD_idに等しい)|
|8~10|board_type|基板の種類を示す|0~7の値に基板が割り当てられる|
|11|direction|通信方向|0:Main基板からMD等の制御値、1:MD等からMain基板へのフィードバック|

## data_type (MD)
以下の表のようにMDではdata_typeを活用する。

|data_type|名称|サイズ[byte]|dir:0の時の意味|dir:1の時の意味|
|:-:|:-:|:-:|:-:|:-:|
|0|init|8|MDの設定|基板の種類|
|1|target|2|モーター１つの制御値|エンコーダーの値|
|2|limit_switch|1|※１リミットスイッチが押されたときの振る舞い|リミットスイッチの状態(bit毎にSW1, SW2...SW8)|
|3|gain|5|1バイト: ゲイン種別 (0=P, 1=I, 2=D) + 4バイト: float値|現在のゲイン値|

※１リミットスイッチが押されたときの振る舞い
- 0:何もしない
- 1:制御値がゼロになるまで出力をゼロにする
- 2:正回転を停止
- 3:逆回転を停止
- 4:正回転をスイッチ１で停止,逆回転をスイッチ２で停止

# データフレームについて
若干data_typeの説明に被ってしまうが、ここではデータフレームに重きを置いて説明する。

## init
このデータを送らないとモーターは回らない。
以下に内容を記載する。

### モータードライバーの設定（direction:0）
|名称|サイズ|型|説明|
|:-:|:-:|:-:|:-:|
|max_output|2byte|uint16_t|最大出力 duty|
|max_acceleration|1byte|uint8_t|台形制御の最大加速 duty/ms|
|brake|1byte|uint8_t|0:自然減速、1:電気ブレーキ（推奨）|
|control_period|1byte|uint8_t|制御周期 ms|
|encoder_period|1byte|uint8_t|エンコーダーのサンプリング周期 ms（0でエンコーダー無し）|
|encoder_type|1byte|uint8_t|0:無し、1:インクリメンタル、2:アブソリュート|
|control_mode|1byte|uint8_t|0:未使用、1:PID制御|

```c++
union md_init_t
{
    struct
    {
        uint16_t max_output; // 最大出力 duty
        uint8_t max_acceleration; // 台形制御の最大加速 duty/ms
        uint8_t brake; // 0:自然減速、1:電気ブレーキ（推奨）
        uint8_t control_period; // 制御周期 ms
        uint8_t encoder_period; // エンコーダーのサンプリング周期 ms
        uint8_t encoder_type; // 0:無し、1:インクリメンタル、2:アブソリュート
        uint8_t control_mode; // 0:未使用、1:PID制御
    } __attribute__((__packed__));
    uint8_t code[8]; // 送信バイト配列
} __attribute__((__packed__));
```
上記unionを用いると簡単にCAN通信で送信できる。
データは下位バイト（LSB）から順に送信する。
```c++
board_id = 0;

md_init_t md_init;
md_init.max_output = 3199;
md_init.max_acceleration = 100;
md_init.brake = 1;
md_init.control_period = 1;
md_init.encoder_period = 10;
md_init.encoder_type = 0;
md_init.control_mode = 0;

can_id = encodeCANID(
    can_conf::direction::to_slave, 
    can_conf::board_type::md, 
    board_id, 
    can_conf::data_type::init);
CAN.beginPacket(can_id);

for (uint8_t i = 0; i < 8; i++) // 0~7の順にLSBから送信する。
{
    CAN.write(md_init.code[i]);
}
CAN.endPacket();
```

### モータードライバー基板の種類（direction:1）
uint8_tで、
|値|MDの種類|
|:-:|:-:|
|0|YamaMDv4|
|1|YamaMDv5|
|2|HTMDv1.5|
|3|HTMDv2.2c|
|4|HTMDv2.2s|
|5|ikeda_MD|

## target
モーターの制御値やエンコーダーの値。
以下に内容を記載する。

### モーターの制御値（direction:0）
`int16_t`の制御値の送信する。
データは下位バイト（LSB）から順に送信する。
以下に送信コードの例を記載する。
```c++
uint8_t board_id = 0; // MDのID
int16_t target = 1000; // 制御値
uint8_t tx_data_buffer[2]; // 送信バッファ
memcpy(tx_data_buffer, &target, 2); // メモリコピーで送信バッファに制御値を分割して入れる。

uint16_t can_id = encodeCANID(
    can_conf::direction::to_slave, 
    can_conf::board_type::md, 
    board_id, 
    can_conf::data_type::target);

CAN.beginPacket(can_id);

for (uint8_t i = 0; i < 2; i++)
{
    CAN.write(tx_data_buffer[i]);
}
CAN.endPacket();
```

### エンコーダーの値（direction:1）
`int16_t`のエンコーダーのカウント値が送られてくる。
フォーマットは制御値と同じように送られるので、自分で考えて(⋈◍＞◡＜◍)。✧♡

## limit_switch
リミットスイッチの値や振る舞いを通信する。

### リミットスイッチの動作設定（direction:0）
リミットスイッチが押された際のモーターの動作を`uint8_t`の値で指定する。
以下の値を設定することで、リミットスイッチの挙動を制御できる。
|値|動作|
|:-:|:-:|
|0|リミットスイッチが押されても何もしない|
|1|リミットスイッチが押されたら、制御値がゼロになるまでモーターを回さない|
|2|リミットスイッチが押されたら、正回転のみ停止する|
|3|リミットスイッチが押されたら、逆回転のみ停止する|
|4|リミットスイッチ１で正回転を停止し、リミットスイッチ２で逆回転を停止する|

### リミットスイッチの状態（direction:1）
リミットスイッチの状態が`uint8_t`の値で送信される。
各ビットが１つのリミットスイッチに対応しており、1bitにつき1つのスイッチの状態を表す。
下位ビット（LSB）からSW1,SW2,SW3...SW8リミットスイッチと割り当てられている。

## gain
PID制御などのゲインを通信する。
また、下位バイト（LSB）にはゲインの識別用に`uint8_t`の対応した値を加える。
ゲインは4byteでC/C++のfloat型で送信する。
以下にゲインの識別用の値を示す。
|値|ゲイン|
|:-:|:-:|
|0|比例ゲイン（P）|
|1|積分ゲイン（I）|
|2|微分ゲイン（D）|

direction:0ではPIDゲインを設定し、direction:1ではMDに設定されているPIDゲインが送られてくる。
データフレームはdirectionに依存せず一様である。

以下に送信コード、受信コードの例を記載する。
