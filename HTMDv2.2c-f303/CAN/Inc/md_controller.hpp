#include "can_config.hpp"
#include "md_config.hpp"

using namespace can_config;

class MDController
{
private:
    /* 固有値 */
    uint8_t board_id;   // 基板のID
    uint8_t board_kind; // 基板の種類
    uint8_t fw_version; // ファームウェアのバージョン
    /* 受信バッファ */
    uint8_t rx_buffer[8];           // 受信バッファ
    uint8_t init_buffer[8];         // 初期化バッファ
    uint8_t target_buffer[2];       // 目標位置バッファ
    uint8_t gain_buffer[3][4];      // ゲインバッファ
    uint8_t multi_target_buffer[8]; // 複数の目標値バッファ
    /* 受信フラグ */
    bool init_flag;         // 初期化フラグ
    bool target_flag;       // 目標位置フラグ
    bool limit_switch_flag; // リミットスイッチフラグ
    bool gain_flag[3];      // ゲインフラグ
    bool multi_target_flag; // 複数の目標値フラグ
    /* 一時処理用変数 */
    uint8_t packet_direction;  // 進行方向
    uint8_t packet_board_type; // 基板の種類
    uint8_t packet_board_id;   // 基板のID
    uint8_t packet_data_type;  // データの種類

protected:
    virtual void send(uint16_t id, uint8_t *data, uint8_t len) = 0;
    void receive(uint16_t id, uint8_t *data, uint8_t len);

public:
    MDController(uint8_t board_id, uint8_t board_kind, uint8_t fw_version);
    void set_board_id(uint8_t board_id) { this->board_id = board_id; }
    void send_init(uint8_t md_kind);
    void send_encoder(int16_t encoder);
    void send_limit_switch(uint8_t limit_switch);
    void send_pid_gain(float p_gain, float i_gain, float d_gain);
    bool get_init(md_config_t *md_config);
    bool get_target(int16_t *target);
    bool get_gain(uint8_t gain_kind, float *gain);
    void send_gain(uint8_t gain_kind, float gain);
};