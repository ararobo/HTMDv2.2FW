# MotorController クラス仕様書

## 概要
`MotorController` クラスは、DCモーターの制御を行うためのクラスです。  
PID制御や台形制御を用いて、モーターのスムーズな動作を実現します。  
エンコーダを使用してモーターの位置や速度をフィードバック制御します。

---

## クラス定義

### ヘッダーファイル
`dc_motor_controller.hpp`

### 実装ファイル
`dc_motor_controller.cpp`

---

## メンバ変数

| メンバ変数名    | 型           | 説明                           |
|-----------------|--------------|--------------------------------|
| `md_config`     | `md_config_t`| モータードライバの設定         |
| `encoder_count` | `int16_t`    | エンコーダの現在のカウント値   |

---

## md_config_t 構造体

`md_config_t` は、モータードライバの設定を保持する構造体です。  
以下のメンバを持ち、モーター制御の挙動を柔軟に設定できます。

| メンバ名                | 型         | 説明                                           |
|-------------------------|------------|----------------------------------------------|
| `max_output`            | `uint16_t` | 最大出力 duty                                 |
| `max_acceleration`      | `uint8_t`  | 台形制御の最大加速 duty/10ms                  |
| `control_period`        | `uint8_t`  | 制御周期 ms                                   |
| `encoder_period`        | `uint8_t`  | エンコーダのサンプリング周期 ms               |
| `encoder_type`          | `uint8_t`  | エンコーダの種類 (0: 無し, 1: インクリメンタル, 2: アブソリュート) |
| `limit_switch_behavior` | `uint8_t`  | リミットスイッチの動作設定                    |
| `option`                | `uint8_t`  | 基板固有のオプション設定                     |

---

## メソッド

### コンストラクタ

#### `MotorController()`
`MotorController` クラスのインスタンスを初期化します。

---

### 初期化

#### `void init()`
モーター制御に必要なハードウェアを初期化します。  
エンコーダの初期化も行います。

---

### リセット

#### `void reset()`
モーター制御の状態をリセットします。  
台形制御やPID制御の内部状態を初期化します。  
エンコーダのカウント値もリセットします。

---

### モーターの動作

#### `void run(int16_t output)`
モーターを動作させます。  
エンコーダのカウント値を使用してPID制御を行い、台形制御でスムーズな出力を実現します。

| 引数名   | 型         | 説明                           |
|----------|------------|--------------------------------|
| `output` | `int16_t`  | 目標出力値                    |

---

### 設定の更新

#### `void set_config(md_config_t config)`
モータードライバの設定を更新します。  
エンコーダのサンプリング周期や制御周期も設定します。

| 引数名   | 型             | 説明                           |
|----------|----------------|--------------------------------|
| `config` | `md_config_t`  | モータードライバの設定         |

---

### モーターの停止

#### `void stop()`
モーターを停止し、ブレーキをかけます。  
エンコーダのカウント値をリセットします。

---

### PIDゲインの設定

#### `void set_pid_gain(float p_gain, float i_gain, float d_gain)`
PID制御のゲインを設定します。

| 引数名    | 型      | 説明                           |
|-----------|---------|--------------------------------|
| `p_gain`  | `float` | 比例ゲイン                     |
| `i_gain`  | `float` | 積分ゲイン                     |
| `d_gain`  | `float` | 微分ゲイン                     |

---

### エンコーダのサンプリング

#### `void sample_encoder()`
エンコーダの現在のカウント値を取得し、`encoder_count` に保存します。

---

## 使用例

```cpp
#include "dc_motor_controller.hpp"

int main()
{
    MotorController motor;

    // 初期化
    motor.init();

    // PIDゲインの設定
    motor.set_pid_gain(1.0f, 0.5f, 0.1f);

    // モータードライバの設定
    md_config_t config;
    config.max_output = 3200;
    config.max_acceleration = 10;
    config.control_period = 10;
    config.encoder_period = 10;
    config.encoder_type = 1; // インクリメンタルエンコーダ
    config.limit_switch_behavior = 0;
    config.option = 0;

    motor.set_config(config);

    // エンコーダのサンプリング
    motor.sample_encoder();

    // モーターを動作させる
    motor.run(100);

    // モーターを停止する
    motor.stop();

    return 0;
}