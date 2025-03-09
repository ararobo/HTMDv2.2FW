# CAN通信 v2.0
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

## direction（通信方向）
0 or 1で通信方向を示す。
|値|方向|説明|
|:-:|:-:|:-:|
|0|master → slave|メイン基板から周辺基板への送信|
|1|slave → master|周辺基板からメイン基板へのフィードバック|

## board_type（基板の種類）
0~7で基板の種類を表す。
|値|名称|説明|
|:-:|:-:|:-:|
|0|emergency_stop_board|非常停止基板|
|1|motor_driver|モータードライバー基板|
|2|servo_driver|サーボモータードライバー基板|
|3|solenoid_driver|電磁弁制御基板|
|4|未定|未定|
|5|未定|未定|
|6|未定|未定|
|7|未定|未定|

## data_type (MDの場合)
以下の表のようにMDではdata_typeを活用する。

|data_type|名称|サイズ[byte]|dir:0の時の意味|dir:1の時の意味|
|:-:|:-:|:-:|:-:|:-:|
|0|init|8/1|MDの設定（サイズ:8byte）|基板の種類（サイズ:1byte）|
|1|target|2|モーター１つの制御値|エンコーダーの値|
|2|limit_switch|1|未使用|リミットスイッチの状態(bit毎にSW1, SW2...SW8)|
|3|gain|5|1バイト: ゲイン種別 (0=P, 1=I, 2=D) + 4バイト: float値|現在のゲイン値|

# データフレームについて
MDとのCAN通信でのデータフレームについて示す。

## init（初期化）
このデータを送らないとモーターは回らない。
以下に内容を記載する。

### モータードライバーの設定（direction:0）
|名称|サイズ|型|説明|
|:-:|:-:|:-:|:-:|
|max_output|2byte|uint16_t|最大出力 duty|
|max_acceleration|1byte|uint8_t|台形制御の最大加速 duty/10ms|
|control_period|1byte|uint8_t|制御周期 ms|
|encoder_period|1byte|uint8_t|エンコーダーのサンプリング周期 ms（0でエンコーダー無し）|
|encoder_type|1byte|uint8_t|0:無し、1:インクリメンタル、2:アブソリュート|
|limit_switch_behavior|1byte|uint8_t|リミットスイッチが押された際の動作設定（以下参照）|
|option|1byte|uint8_t|基板の固有機能や使用用途に合わせて決定（未定義）|

#### limit_switch_behavior（リミットスイッチの動作設定）
リミットスイッチが押された際のモーターの動作を、`uint8_t`の値で指定します。
以下の値を設定することで、リミットスイッチの挙動を制御できる。
|値|動作|
|:-:|:-:|
|0|リミットスイッチが押されても何もしない|
|1|リミットスイッチが押されたら、制御値がゼロになるまでモーターを回さない|
|2|リミットスイッチが押されたら、正回転のみ停止する|
|3|リミットスイッチが押されたら、逆回転のみ停止する|
|4|リミットスイッチ１で正回転を停止し、リミットスイッチ２で逆回転を停止する|


```c++
union md_config_t
{
    struct
    {
        uint16_t max_output; // 最大出力 duty
        uint8_t max_acceleration; // 台形制御の最大加速 duty/10ms
        uint8_t control_period; // 制御周期 ms
        uint8_t encoder_period; // エンコーダーのサンプリング周期 ms
        uint8_t encoder_type; // 0:無し、1:インクリメンタル、2:アブソリュート
        uint8_t limit_switch_behavior; // リミットスイッチの動作設定
        uint8_t option; // 基板の固有機能や使用用途に合わせて決定（未定義）
    } __attribute__((__packed__));
    uint8_t code[8]; // 送信バイト配列
} __attribute__((__packed__));
```
上記unionを用いると簡単にCAN通信で送信できる。
ただし、エンディアンによる影響を受けるため、マイコンを変更する際は要注意。
データは下位バイト（LSB）から順に送信する。
```c++
board_id = 0;

md_config_t md_config;
md_config.max_output = 3199;
md_config.max_acceleration = 100;
md_config.control_period = 1;
md_config.encoder_period = 10;
md_config.encoder_type = 0;
md_config.limit_switch_behavior = 0;

can_id = encodeCANID(
    can_conf::direction::to_slave, 
    can_conf::board_type::motor_driver, 
    board_id, 
    can_conf::data_type::init);
CAN.beginPacket(can_id);

for (uint8_t i = 0; i < 8; i++) // 0~7の順にLSBから送信する。
{
    CAN.write(md_config.code[i]);
}
CAN.endPacket();
```

### モータードライバー基板の種類（direction:1）
`uint8_t`の1バイトで内容は以下に記載した。
|値|MDの種類|
|:-:|:-:|
|0|YamaMDv4|
|1|YamaMDv5|
|2|HTMDv1.5|
|3|HTMDv2.2c|
|4|HTMDv2.2s|
|5|ikeda_MD|

## target（制御値とエンコーダー）
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
    can_conf::board_type::motor_driver, 
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
リミットスイッチの状態を通信する。
direction:0は未使用です。

### リミットスイッチの状態（direction:1）
リミットスイッチの状態が`uint8_t`の値で送信される。
各ビットが１つのリミットスイッチに対応しており、1bitにつき1つのスイッチの状態を表す。
下位ビット（LSB）からSW1,SW2,SW3...SW8リミットスイッチと割り当てられている。

## gain（ゲイン設定）
PID制御などのゲインを通信する。
データは5バイトで構成され、下位バイト（LSB）にゲインの識別用`uint8_t`加え、続く4バイトは`float`型のゲイン値を格納する。

### ゲイン識別コード
|値|ゲイン|
|:-:|:-:|
|0|比例ゲイン（P）|
|1|積分ゲイン（I）|
|2|微分ゲイン（D）|

### 通信方向による動作
- direction:0（送信）　PIDゲインを設定する。
- direction:1（受信）　MDに現在設定されているPIDゲインが送信される。
- データフレームの形式はdirectionに関わらず共通。
