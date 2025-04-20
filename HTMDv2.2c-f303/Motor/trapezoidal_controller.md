# TrapezoidalController クラス仕様書

## 概要
`TrapezoidalController` クラスは、台形制御を実現するためのクラスです。  
目標出力値に対して最大加速度を制限し、スムーズな制御を提供します。

---

## クラス定義

### ヘッダーファイル
`trapezoidal_controller.hpp`

### 実装ファイル
`trapezoidal_controller.cpp`

---

## メンバ変数

| メンバ変数名    | 型       | 説明                           |
|-----------------|----------|--------------------------------|
| `prev_out`      | `int16_t`| 前回の出力値                  |
| `control_cycle` | `uint8_t`| 制御周期（デフォルト: 5）     |

---

## メソッド

### 台形制御の計算

#### `int16_t trapezoidal_control(int16_t output, uint8_t max_acceleration)`
台形制御を計算し、制限された出力値を返します。

| 引数名            | 型       | 説明                           |
|-------------------|----------|--------------------------------|
| `output`          | `int16_t`| 目標出力値                    |
| `max_acceleration`| `uint8_t`| 最大加速度                    |

**戻り値**:  
制限された出力値（`int16_t`）

---

### 台形制御の初期化

#### `void reset_trapezoidal_control()`
台形制御の状態を初期化します。

---

## 使用例

```cpp
#include "trapezoidal_controller.hpp"
#include <iostream>

int main()
{
    TrapezoidalController controller;

    // 台形制御の計算
    int16_t target_output = 100;   // 目標出力値
    uint8_t max_acceleration = 10; // 最大加速度
    int16_t output = controller.trapezoidal_control(target_output, max_acceleration);

    // 制御出力を表示
    std::cout << "Trapezoidal Output: " << output << std::endl;

    // 台形制御の初期化
    controller.reset_trapezoidal_control();

    return 0;
}