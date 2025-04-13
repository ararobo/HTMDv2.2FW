#pragma once
#include "can_config.hpp"
#include "md_config.hpp"

class MDController
{
private:
    /* 固有値 */
    uint8_t board_id;   // 基板のID
    uint8_t board_type; // 基板の種類
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
    /**
     * @brief CANの送信処理(オーバーライドしてください)
     *
     * @param id CANのID
     * @param data 送信するデータ
     * @param len データの長さ
     */
    virtual void send(uint16_t id, uint8_t *data, uint8_t len) = 0;

    /**
     * @brief 受信データの処理を行う
     *
     * @param id CANのID
     * @param data 受信データ
     * @param len データの長さ
     */
    void receive(uint16_t id, uint8_t *data, uint8_t len);

public:
    MDController(uint8_t board_id, uint8_t board_type, uint8_t fw_version);
    /**
     * @brief 基板のIDを設定する
     *
     * @param board_id
     */
    void set_board_id(uint8_t board_id) { this->board_id = board_id; }

    /**
     * @brief 基板の種類を設定する
     *
     * @param md_type 基板の種類
     */
    void send_init(uint8_t md_type);

    /**
     * @brief エンコーダーの値を送信する
     *
     * @param encoder エンコーダーの値
     */
    void send_encoder(int16_t encoder);

    /**
     * @brief リミットスイッチの状態を送信する
     *
     * @param limit_switch リミットスイッチの状態
     */
    void send_limit_switch(uint8_t limit_switch);

    /**
     * @brief ゲインを送信する
     *
     * @param gain_kind ゲインの種類 0: Pゲイン、1: Iゲイン、2: Dゲイン
     * @param gain ゲイン
     */
    void send_gain(uint8_t gain_kind, float gain);

    /**
     * @brief MDの設定を取得する
     *
     * @param md_config MDの設定
     * @return true 初期化フラグが立っている
     * @return false 初期化フラグが立っていない
     */
    bool get_init(md_config_t *md_config);

    /**
     * @brief 目標値を取得する
     *
     * @param target 目標値
     * @return true 更新されている
     * @return false 更新されていない
     */
    bool get_target(int16_t *target);

    /**
     * @brief ゲインを取得する
     *
     * @param gain_kind ゲインの種類 0: Pゲイン、1: Iゲイン、2: Dゲイン
     * @param gain ゲイン
     * @return true 更新されている
     * @return false 更新されていない
     */
    bool get_gain(uint8_t gain_kind, float *gain);
};