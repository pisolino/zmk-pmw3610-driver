#pragma once

/**
 * @file pixart.h
 *
 * @brief Common header file for all optical motion sensor by PIXART
 */

#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>

#ifdef __cplusplus
extern "C" {
#endif

enum pixart_input_mode { 
    MOVE = 0,      // 通常の移動モード
    SCROLL,        // スクロールモード
    SCROLL_KB,     // キーボール風スクロールモード
    SNIPE,         // 精密狙いモード
#ifdef CONFIG_PMW3610_PROFILE_SWITCHING
    PRECISION,     // 精密作業プロファイル
    SPEED          // 高速移動プロファイル
#endif
};

/* device data structure */
struct pixart_data {
    const struct device *dev;

    enum pixart_input_mode curr_mode;
    uint32_t curr_cpi;
    int32_t scroll_delta_x;
    int32_t scroll_delta_y;
    
    // スクロール加速のための変数
    int64_t scroll_last_movement_time;  // 最後の動きが検出された時間
    float scroll_acceleration;          // 現在の加速度
    int16_t scroll_consecutive_movements; // 連続した動きのカウント
    int16_t scroll_prev_movement_size;    // 前回の動きの大きさ
    float scroll_prev_movement_velocity;  // 前回の動きの速度（単位時間あたりの移動量）
    
    // スクロール補間のための変数
    int16_t scroll_last_direction_x;    // 前回のスクロール方向 X (+1, 0, -1)
    int16_t scroll_last_direction_y;    // 前回のスクロール方向 Y (+1, 0, -1) 
    int16_t scroll_missed_detection_count; // 連続した取りこぼし検出カウント
    int16_t scroll_consistent_direction_count; // 同一方向への連続スクロールカウント
    int64_t scroll_last_real_movement_time; // 実際の動きが最後に検出された時間
    
    #ifdef CONFIG_PMW3610_KEYBALL_SCROLL
    // キーボール風スクロールのための変数
    int64_t kb_scroll_mode_changed;      // スクロールモードが最後に変更された時間
    uint8_t kb_scroll_div;               // スクロール除数（1-7、0は未設定）
    int32_t kb_scroll_snap_tension_h;    // 水平スクロールの蓄積テンション
    int32_t kb_scroll_snap_tension_v;    // 垂直スクロールの蓄積テンション
    int64_t kb_scroll_snap_last;         // 最後のスクロールスナップ時間
#endif
    
    // 速度ベースのスクロールのための履歴配列
    #ifdef CONFIG_PMW3610_VELOCITY_BASED_SCROLLING
    #define VELOCITY_HISTORY_SIZE 5
    float velocity_history[VELOCITY_HISTORY_SIZE]; // 速度の履歴
    int velocity_history_index;                    // 履歴配列の現在のインデックス
    #endif
    
    // MOVEモードの高度な加速制御のための変数
    float move_acceleration;          // 現在の加速度
    int16_t move_consecutive_movements; // 連続した動きのカウント
    int64_t move_last_movement_time;  // 最後の動きが検出された時間
    float move_prev_movement_velocity; // 前回の動きの速度

#ifdef CONFIG_PMW3610_POLLING_RATE_125_SW
    int64_t last_poll_time;
    int16_t last_x;
    int16_t last_y;
    int16_t last_raw_movement;  // ポーリングレート制限時に生の動きの値を保持するための変数
#endif

#ifdef CONFIG_PMW3610_SMOOTHING_FILTER
    // 平滑化フィルター用の前回の値を保持する変数
    int16_t prev_x;
    int16_t prev_y;
    uint8_t current_smoothing_weight; // 現在のスムージング係数
#endif

#ifdef CONFIG_PMW3610_PROFILE_SWITCHING
    bool precision_profile_active;
    bool speed_profile_active;
    uint32_t backup_cpi; // 元のCPI値を保存
#endif

    // motion interrupt isr
    struct gpio_callback irq_gpio_cb;
    // the work structure holding the trigger job
    struct k_work trigger_work;

    // the work structure for delayable init steps
    struct k_work_delayable init_work;
    int async_init_step;

    //
    bool ready;           // whether init is finished successfully
    bool last_read_burst; // todo: needed?
    int err;              // error code during async init

    // for pmw3610 smart algorithm
    bool sw_smart_flag;
};

// device config data structure
struct pixart_config {
    struct gpio_dt_spec irq_gpio;
    struct spi_dt_spec bus;
    struct gpio_dt_spec cs_gpio;
    size_t scroll_layers_len;
    int32_t *scroll_layers;
    size_t snipe_layers_len;
    int32_t *snipe_layers;
};

#ifdef __cplusplus
}
#endif

/**
 * @}
 */
