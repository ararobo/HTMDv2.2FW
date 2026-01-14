#pragma once

#include <cfloat>
#include <cstdint>

namespace common::interfaces {

class GateDriverInterface {
  public:
    virtual ~GateDriverInterface() = default;

    virtual bool init() = 0;

    /**
     * @brief ゲートドライバ出力設定
     * 
     * @param output 出力値（-1.0 ~ 1.0）
     * @return true 出力成功
     * @return false 出力失敗
     */
    virtual bool output(float output) = 0;

    virtual void set_brake_mode(bool enabled) = 0;
};

}  // namespace common::interfaces