/**
 * @file i_gate_driver.hpp
 * @author Gento Aiba (aiba-gento)
 * @brief ゲートドライバの抽象インターフェース
 * @version 0.1
 * @date 2026-02-23
 *
 * @copyright Copyright (c) 2026 Gento Aiba
 * SPDX-License-Identifier: Apache-2.0
 *
 */
#pragma once

namespace gn10_motor {

/**
 * @brief ゲートドライバの抽象インターフェース
 *
 * モータードライバICの具象クラスはこのインターフェースを継承して実装する。
 * output() の引数はデューティ比の絶対値とし、符号で方向を示す。
 * 値域は具象クラスで定義された max_duty に依存する。
 */
class IGateDriver {
  public:
    virtual ~IGateDriver() = default;

    /**
     * @brief ハードウェアの初期化
     */
    virtual void hardware_init() = 0;

    /**
     * @brief モーター出力を設定する
     * @param output 出力値 (デューティ比、正負で回転方向を示す)
     */
    virtual void output(float output) = 0;

    /**
     * @brief ブレーキの有効/無効を設定する
     * @param brake true: ブレーキ有効, false: ブレーキ解除
     */
    virtual void set_brake(bool brake) = 0;
};

}  // namespace gn10_motor
