#pragma once

namespace can_id
{
    // 受信
    static constexpr uint16_t INIT_PID = 0x205;
    static constexpr uint16_t INIT_MODE = 0x206;
    static constexpr uint16_t INIT_COMMAND = 0x207;
    static constexpr uint16_t TARGETS = 0x100;
    // 送信
    static constexpr uint16_t RE_INIT_PID = 0x200;
    static constexpr uint16_t RE_INIT_MODE = 0x201;
    static constexpr uint16_t RE_INIT_COMMAND = 0x202;
    static constexpr uint16_t MD_STATE = 0x203;
}

namespace data_length
{
    static constexpr uint8_t INIT_PID = 7;
    static constexpr uint8_t INIT_MODE = 8;
    static constexpr uint8_t INIT_COMMAND = 2;
    static constexpr uint8_t TARGETS = 8;
    static constexpr uint8_t MD_STATE = 3;
}

namespace command
{
    static constexpr uint16_t SUCCESS = 0x00;
    static constexpr uint16_t FAILED = 0x01;
    static constexpr uint16_t DO_INIT = 0x02;
}