# MDのデータ管理用クラス
クラス名はMDDataManagerである。
このクラスを継承して新たにクラスを作成することによって使用可能である。
継承後のクラスでCAN通信の処理を記述する必要がある。

## 受信系
CAN通信で受信したデータは、can_callbackメソッドによってクラス内のバッファに保存されている。

### bool get_init(md_config_t *md_config)
初期化時に送られてくるMDの設定を取得するメソッド。
引数はmd_config_t型のポインタで定義は[このファイル](CANv2.0MDDataframe.md#モータードライバーの設定direction0)にある。
与えられたポインタのオブジェクトに受信したデータを入れる。
初期化の命令が送られてきた場合にtrueを返す。
一度trueを返すと受信フラグはリセットされるので、次回受信したときにtrueを返す。

### bool get_target(int16_t *target)
制御値（出力）を取得するメソッド。
与えられたポインタのオブジェクトに受信したデータを入れる。
前回メソッドを用いてから制御値の受信があった場合、trueを返す。

### bool get_gain(float *p_gain, float *i_gain, float *d_gain)
PID制御のゲインを取得するメソッド。
与えられたポインタのうちCAN通信で更新されているデータがあれば、ポインタの指すオブジェクトを受信したゲインに書き換える。
前回メソッドを用いてからPIDゲインの一つでも受信があった場合、trueを返す。

## 送信系
送信時はsend(uint16_t id, uint8_t *data, uint8_t len)メソッドを用いる。
sendメソッドは継承後のクラスでオーバーライドすることによってCAN通信の送信を行う。

### void send_init(uint8_t md_kind)
MDの種類を送信するメソッド。

### void send_encoder(int16_t encoder)
エンコーダーの値を引数に与えると送信するメソッド。

### void send_limit_switch(uint8_t limit_switch_state)
リミットスイッチの状態を1bitにつき1SWで送信するメソッド。

### void send_gain(float *p_gain, float *i_gain, float *d_gain)
PIDゲインを送信するメソッド。