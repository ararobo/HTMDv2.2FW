# メイン基板のデータ管理用クラス
非常停止、MD、サーボモーター、電磁弁の４つの基板のCAN通信データを扱うことができるクラス。

## 送信系
送信時はsend(uint16_t id, uint8_t *data, uint8_t len)メソッドを用いる。
sendメソッドは継承後のクラスでオーバーライドすることによってCAN通信の送信を行う。

### void send_ems_init(uint8_t config)
非常停止基板の初期化を送信する。
非常停止基板の設定を引数に取る。

### void send_md_init(uint8_t md_id, md_config_t md_config)
MDの初期化を送信する。
MDのIDとそのMDの設定を引数に取る。

### void send_servo_init(uint8_t config)
サーボモータードライバーの初期化を送信する。
サーボモータードライバーの設定を引数に取る。

### void send_solenoid_init()
電磁弁基板に初期化を送信する。

### void send_ems_