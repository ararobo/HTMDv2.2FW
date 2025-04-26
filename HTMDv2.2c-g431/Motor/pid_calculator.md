# PIDCalculator クラス仕様書

## 概要
`PIDCalculator` クラスは、PID制御を実現するためのクラスです。  
目標値 (`target`) と現在値 (`now_value`) を入力としてPID制御を計算し、制御出力を提供します。

---

## クラス定義

### ヘッダーファイル
`pid_calculator.hpp`

### 実装ファイル
`pid_calculator.cpp`

---

## メンバ変数

| メンバ変数名    | 型       | 説明                           |
|-----------------|----------|--------------------------------|
| `Kp`            | `float`  | 比例ゲイン                    |
| `Ki`            | `float`  | 積分ゲイン                    |
| `Kd`            | `float`  | 微分ゲイン                    |
| `i_out`         | `float`  | 積分制御の出力値              |
| `prev_error`    | `float`  | 前回の誤差                   |
| `dt`            | `float`  | 制御周期（秒単位）            |

---

## メソッド

### コンストラクタ

#### `PIDCalculator(float dt)`
制御周期を指定して `PIDCalculator` クラスを初期化します。

| 引数名 | 型      | 説明               |
|--------|---------|--------------------|
| `dt`   | `float` | 制御周期 [秒]      |

---

### PID制御の計算

#### `float calculate_pid(float target, float now_value)`
PID制御を計算し、制御出力を返します。

| 引数名       | 型      | 説明               |
|--------------|---------|--------------------|
| `target`     | `float` | 目標値             |
| `now_value`  | `float` | 現在値             |

**戻り値**:  
PID制御の計算結果（制御出力）

---

### PIDゲインの設定

#### `void set_pid_gain(float p_gain, float i_gain, float d_gain)`
比例ゲイン、積分ゲイン、微分ゲインを設定します。

| 引数名    | 型      | 説明               |
|-----------|---------|--------------------|
| `p_gain`  | `float` | 比例ゲイン         |
| `i_gain`  | `float` | 積分ゲイン         |
| `d_gain`  | `float` | 微分ゲイン         |

---

### PIDゲインの取得

#### `void get_pid_gain(float *p_gain, float *i_gain, float *d_gain)`
現在のPIDゲインを取得します。

| 引数名    | 型         | 説明               |
|-----------|------------|--------------------|
| `p_gain`  | `float*`   | 比例ゲイン         |
| `i_gain`  | `float*`   | 積分ゲイン         |
| `d_gain`  | `float*`   | 微分ゲイン         |

---

### 制御周期の設定

#### `void set_dt(float dt)`
制御周期を設定します。

| 引数名 | 型      | 説明               |
|--------|---------|--------------------|
| `dt`   | `float` | 制御周期 [秒]      |

---

### PID制御の初期化

#### `void reset_pid()`
積分制御の出力値や前回の誤差をリセットします。

---

## 使用例

```cpp
#include "pid_calculator.hpp"

int main()
{
    // 制御周期を0.01秒としてPIDCalculatorを初期化
    PIDCalculator pid(0.01f);

    // PIDゲインの設定
    pid.set_pid_gain(1.0f, 0.5f, 0.1f);

    // PID制御の計算
    float target = 100.0f;   // 目標値
    float now_value = 90.0f; // 現在値
    float output = pid.calculate_pid(target, now_value);

    // 制御出力を表示
    printf("Control Output: %f\n", output);

    return 0;
}