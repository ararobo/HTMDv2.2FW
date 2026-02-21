# gn10_motor

HTMDv2.2FW 向けのモーター制御ライブラリです。
組み込みシステムおよび ROS 2 ノード向けに設計された、ヘッダーオンリーの C++17 ライブラリです。

## 特徴

- **ヘッダーオンリー**: CMake の `INTERFACE` ライブラリとして簡単に統合できます。
- **テンプレートベース**: `float` および `double` をサポートします。
- **動的割り当てなし**: 組み込みシステムの制約に準拠しています。
- **モダンC++**: `std::clamp` や `static_assert` などの C++17 機能を使用しています。

## コンポーネント

### PID コントローラ (`gn10_motor/pid.hpp`)

標準的な PID コントローラで、以下の機能を備えています：
- 積分ワインドアップ防止（クランピング）
- 測定値微分（微分の急変防止 / Derivative on Measurement）
- 出力制限

**使用法:**

```cpp
#include <gn10_motor/pid.hpp>

// 1. PIDコントローラの設定
gn10_motor::PIDConfig<float> pid_config;
pid_config.kp = 1.5f;
pid_config.ki = 0.05f;
pid_config.kd = 0.01f;
pid_config.integral_limit = 10.0f; // アンチワインドアップ制限
pid_config.output_limit   = 24.0f; // 最大出力電圧/電流など

// 2. インスタンス化
gn10_motor::PID<float> pid(pid_config);

// 3. 制御ループ内での更新
float dt = 0.001f; // 1ms
float command = pid.update(target_velocity, current_velocity, dt);
```

### 加速度リミッター (`gn10_motor/acceleration_limiter.hpp`)

目標値の急激な変化を滑らかにしたり、物理的な加速度を制限するためのスルーレートリミッターです。

**使用法:**

```cpp
#include <gn10_motor/acceleration_limiter.hpp>

// 最大加速度: 100.0 rad/s^2
gn10_motor::AccelerationLimiter<float> limiter(100.0f);

// 制御ループ内での更新
float smooth_target = limiter.update(raw_target, dt);
```

## 統合方法

`CMakeLists.txt` に以下を追加してください：

```cmake
add_subdirectory(path/to/gn10_motor)
target_link_libraries(your_target PRIVATE gn10_motor)
```
