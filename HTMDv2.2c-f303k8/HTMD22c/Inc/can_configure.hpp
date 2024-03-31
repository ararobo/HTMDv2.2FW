namespace can_configure
{
    namespace control
    {
        namespace dlc
        {
            static constexpr int md_targets = 8;
        }
        namespace id
        {
            static constexpr int md_targets = 0x100;
        }
    }
    namespace manage
    {
        namespace dlc
        {
            static constexpr int init = 1;
            static constexpr int re_init = 1;
            static constexpr int mode = 8;
            static constexpr int re_mode = 8;
            static constexpr int pid = 6;
            static constexpr int re_pid = 6;
        }
        namespace id
        {
            static constexpr int init = 0x200;
            static constexpr int re_init = 0x210;
            static constexpr int mode = 0x220;
            static constexpr int re_mode = 0x230;
            static constexpr int pid = 0x240;
            static constexpr int re_pid = 0x250;
        }
        namespace command
        {
            static constexpr int do_init = 0x00;
            static constexpr int success = 0x01;
            static constexpr int fail = 0x02;
        }
    }
    namespace sensor
    {
        namespace dlc
        {
            static constexpr int limit = 1;
            static constexpr int encoder = 2;
            static constexpr int current = 4;
            static constexpr int limit_and_encoder = 3;
            static constexpr int encoder_and_current = 6;
            static constexpr int all = 7;
        }
        namespace id
        {
            static constexpr int limit = 0x300;
            static constexpr int encoder = 0x310;
            static constexpr int current = 0x320;
            static constexpr int limit_and_encoder = 0x350;
            static constexpr int encoder_and_current = 0x360;
            static constexpr int all = 0x370;
        }
    }
    namespace state
    {
        namespace dlc
        {
            static constexpr int md = 1;
            static constexpr int temp = 1;
            static constexpr int all = 2;
        }
        namespace id
        {
            static constexpr int md = 0x400;
            static constexpr int temp = 0x410;
            static constexpr int all = 0x470;
        }
        namespace state
        {
            static constexpr int init = 0x00;
            static constexpr int ready = 0x01;
            static constexpr int busy = 0x02;
            static constexpr int error = 0x03;
        }
    }
}