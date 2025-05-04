/*
 * Copyright (c) 2022 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT pixart_pmw3610

// 12-bit two's complement value to int16_t
// adapted from https://stackoverflow.com/questions/70802306/convert-a-12-bit-signed-number-in-c
#define TOINT16(val, bits) (((struct { int16_t value : bits; }){val}).value)

#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/input/input.h>
#include <zmk/keymap.h>
#include "pmw3610.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(pmw3610, CONFIG_INPUT_LOG_LEVEL);

#include <zephyr/sys/util.h>

//////// Sensor initialization steps definition //////////
// init is done in non-blocking manner (i.e., async), a //
// delayable work is defined for this purpose           //
enum pmw3610_init_step {
    ASYNC_INIT_STEP_POWER_UP,  // reset cs line and assert power-up reset
    ASYNC_INIT_STEP_CLEAR_OB1, // clear observation1 register for self-test check
    ASYNC_INIT_STEP_CHECK_OB1, // check the value of observation1 register after self-test check
    ASYNC_INIT_STEP_CONFIGURE, // set other registes like cpi and donwshift time (run, rest1, rest2)
                               // and clear motion registers

    ASYNC_INIT_STEP_COUNT // end flag
};

/* Timings (in ms) needed in between steps to allow each step finishes succussfully. */
// - Since MCU is not involved in the sensor init process, i is allowed to do other tasks.
//   Thus, k_sleep or delayed schedule can be used.
static const int32_t async_init_delay[ASYNC_INIT_STEP_COUNT] = {
    [ASYNC_INIT_STEP_POWER_UP] = 10, // test shows > 5ms needed
    [ASYNC_INIT_STEP_CLEAR_OB1] =
        200,                          // 150 us required, test shows too short,
                                      // also power-up reset is added in this step, thus using 50 ms
    [ASYNC_INIT_STEP_CHECK_OB1] = 50, // 10 ms required in spec,
                                      // test shows too short,
                                      // especially when integrated with display,
                                      // > 50ms is needed
    [ASYNC_INIT_STEP_CONFIGURE] = 0,
};

static int pmw3610_async_init_power_up(const struct device *dev);
static int pmw3610_async_init_clear_ob1(const struct device *dev);
static int pmw3610_async_init_check_ob1(const struct device *dev);
static int pmw3610_async_init_configure(const struct device *dev);

static int (*const async_init_fn[ASYNC_INIT_STEP_COUNT])(const struct device *dev) = {
    [ASYNC_INIT_STEP_POWER_UP] = pmw3610_async_init_power_up,
    [ASYNC_INIT_STEP_CLEAR_OB1] = pmw3610_async_init_clear_ob1,
    [ASYNC_INIT_STEP_CHECK_OB1] = pmw3610_async_init_check_ob1,
    [ASYNC_INIT_STEP_CONFIGURE] = pmw3610_async_init_configure,
};

//////// Function definitions //////////

// checked and keep
static int spi_cs_ctrl(const struct device *dev, bool enable) {
    const struct pixart_config *config = dev->config;
    int err;

    if (!enable) {
        k_busy_wait(T_NCS_SCLK);
    }

    err = gpio_pin_set_dt(&config->cs_gpio, (int)enable);
    if (err) {
        LOG_ERR("SPI CS ctrl failed");
    }

    if (enable) {
        k_busy_wait(T_NCS_SCLK);
    }

    return err;
}

// checked and keep
static int reg_read(const struct device *dev, uint8_t reg, uint8_t *buf) {
    int err;
    /* struct pixart_data *data = dev->data; */
    const struct pixart_config *config = dev->config;

    __ASSERT_NO_MSG((reg & SPI_WRITE_BIT) == 0);

    err = spi_cs_ctrl(dev, true);
    if (err) {
        return err;
    }

    /* Write register address. */
    const struct spi_buf tx_buf = {.buf = &reg, .len = 1};
    const struct spi_buf_set tx = {.buffers = &tx_buf, .count = 1};

    err = spi_write_dt(&config->bus, &tx);
    if (err) {
        LOG_ERR("Reg read failed on SPI write");
        return err;
    }

    k_busy_wait(T_SRAD);

    /* Read register value. */
    struct spi_buf rx_buf = {
        .buf = buf,
        .len = 1,
    };
    const struct spi_buf_set rx = {
        .buffers = &rx_buf,
        .count = 1,
    };

    err = spi_read_dt(&config->bus, &rx);
    if (err) {
        LOG_ERR("Reg read failed on SPI read");
        return err;
    }

    err = spi_cs_ctrl(dev, false);
    if (err) {
        return err;
    }

    k_busy_wait(T_SRX);

    return 0;
}

// primitive write without enable/disable spi clock on the sensor
static int _reg_write(const struct device *dev, uint8_t reg, uint8_t val) {
    int err;
    /* struct pixart_data *data = dev->data; */
    const struct pixart_config *config = dev->config;

    __ASSERT_NO_MSG((reg & SPI_WRITE_BIT) == 0);

    err = spi_cs_ctrl(dev, true);
    if (err) {
        return err;
    }

    uint8_t buf[2];
    buf[0] = reg | SPI_WRITE_BIT;
    buf[1] = val;

    const struct spi_buf tx_buf = {
        .buf = buf,
        .len = ARRAY_SIZE(buf),
    };
    const struct spi_buf_set tx = {
        .buffers = &tx_buf,
        .count = 1,
    };

    err = spi_write_dt(&config->bus, &tx);
    if (err) {
        LOG_ERR("Reg write failed on SPI write");
        return err;
    }

    // no spi cs ctrl -- allowing multiple regs write
    return 0;
}

// checked and keep
static int burst_write(const struct device *dev, uint8_t reg, const uint8_t *buf, uint8_t size) {
    int err;
    /* struct pixart_data *data = dev->data; */
    const struct pixart_config *config = dev->config;

    __ASSERT_NO_MSG((reg & SPI_WRITE_BIT) == 0);
    __ASSERT_NO_MSG(buf);
    __ASSERT_NO_MSG(size <= PMW3610_MAX_BURST_SIZE);

    err = spi_cs_ctrl(dev, true);
    if (err) {
        return err;
    }

    /* Write register address. */
    const uint8_t wr_addr = reg | SPI_WRITE_BIT;
    struct spi_buf tx_buf = {
        .buf = &wr_addr,
        .len = 1,
    };
    const struct spi_buf_set tx = {
        .buffers = &tx_buf,
        .count = 1,
    };

    err = spi_write_dt(&config->bus, &tx);
    if (err) {
        LOG_ERR("Burst write failed on SPI write");
        return err;
    }

    /* Write data - writing 4 at a time increases the write speed considerably*/
    for (uint8_t i = 0; i < size - 3; i += 4) {
        const struct spi_buf tx_bufs[4] = {
            {.buf = (uint8_t *)&buf[i], .len = 1},
            {.buf = (uint8_t *)&buf[i + 1], .len = 1},
            {.buf = (uint8_t *)&buf[i + 2], .len = 1},
            {.buf = (uint8_t *)&buf[i + 3], .len = 1},
        };
        const struct spi_buf_set tx2 = {
            .buffers = tx_bufs,
            .count = 4,
        };
        err = spi_write_dt(&config->bus, &tx2);
        if (err) {
            LOG_ERR("Burst write failed on SPI write2 %d", err);
            return err;
        }
    }

    /* Write the remaining data */
    for (uint8_t i = (size - (size % 4)); i < size; i++) {
        const struct spi_buf tx_buf3 = {
            .buf = (uint8_t *)&buf[i],
            .len = 1,
        };
        const struct spi_buf_set tx3 = {
            .buffers = &tx_buf3,
            .count = 1,
        };
        err = spi_write_dt(&config->bus, &tx3);
        if (err) {
            LOG_ERR("Burst write failed on SPI write3 %d", err);
            return err;
        }
    }

    err = spi_cs_ctrl(dev, false);
    if (err) {
        return err;
    }

    return 0;
}

// checked and keep
static int reg_write(const struct device *dev, uint8_t reg, uint8_t val) {
    int err;

    err = _reg_write(dev, reg, val);
    if (err) {
        return err;
    }

    err = spi_cs_ctrl(dev, false);
    if (err) {
        return err;
    }

    return 0;
}

// Enabling/disabling the spisclk on the sensor is needed before read/write regs
static int spi_sclk_enable(const struct device *dev, bool enable) {
    int err = reg_write(dev, Control_2, enable ? 0x06 : 0x0C);

    if (enable) {
        k_busy_wait(CLKDELAY_US);
    }

    return err;
}

// read multiple regs at once
static int read_burst(const struct device *dev, uint8_t reg, uint8_t *buf, uint8_t size) {
    int err;
    /* struct pixart_data *data = dev->data; */
    const struct pixart_config *config = dev->config;

    __ASSERT_NO_MSG((reg & SPI_WRITE_BIT) == 0);
    __ASSERT_NO_MSG(buf);
    __ASSERT_NO_MSG(size <= PMW3610_MAX_BURST_SIZE);

    err = spi_cs_ctrl(dev, true);
    if (err) {
        return err;
    }

    /* Write register address. */
    const struct spi_buf tx_buf = {
        .buf = &reg,
        .len = 1,
    };
    const struct spi_buf_set tx = {
        .buffers = &tx_buf,
        .count = 1,
    };

    err = spi_write_dt(&config->bus, &tx);
    if (err) {
        LOG_ERR("Burst read failed on SPI write");
        return err;
    }

    k_busy_wait(T_SRAD);

    /* Read register value - reading 4 at a time increases the read speed considerably*/
    for (uint8_t i = 0; i < size - 3; i += 4) {
        struct spi_buf rx_bufs[4] = {
            {.buf = &buf[i], .len = 1},
            {.buf = &buf[i + 1], .len = 1},
            {.buf = &buf[i + 2], .len = 1},
            {.buf = &buf[i + 3], .len = 1},
        };
        const struct spi_buf_set rx = {
            .buffers = rx_bufs,
            .count = 4,
        };

        err = spi_read_dt(&config->bus, &rx);
        if (err) {
            LOG_ERR("Burst read failed on SPI read");
            return err;
        }
    }

    // Read the remaining registers
    for (uint8_t i = (size - (size % 4)); i < size; i++) {
        struct spi_buf rx_buf = {
            .buf = &buf[i],
            .len = 1,
        };
        const struct spi_buf_set rx = {
            .buffers = &rx_buf,
            .count = 1,
        };

        err = spi_read_dt(&config->bus, &rx);
        if (err) {
            LOG_ERR("Burst read failed on SPI read");
            return err;
        }
    }

    err = spi_cs_ctrl(dev, false);
    if (err) {
        return err;
    }

    k_busy_wait(T_SRX);

    return 0;
}

#ifdef CONFIG_PMW3610_SMART_ALGORITHM
static void update_smart_algorithm_flag(const struct device *dev) {
    struct pixart_data *data = dev->data;

    static bool sw_smart_algorithm = false;
    static uint8_t _product_id;
    static uint8_t _revision_id;
    static uint8_t _inverse_product_id;

    if (data->sw_smart_flag == false) {
        if (sw_smart_algorithm == true) {
            // LOG_DBG("SW Smart Algorithm is active");
            data->sw_smart_flag = true;
        } else {
            // Reading registers to check if our device is already using smart algorithm.
            // Otherwise we will use SW implementation
            spi_sclk_enable(dev, true);
            reg_read(dev, Observation, &_product_id);
            spi_sclk_enable(dev, false);

            spi_sclk_enable(dev, true);
            reg_read(dev, 0x2a, &_revision_id);
            spi_sclk_enable(dev, false);

            spi_sclk_enable(dev, true);
            reg_read(dev, 0x3f, &_inverse_product_id);
            spi_sclk_enable(dev, false);

            if ((_product_id == 0x2A) && (_revision_id == 0x80) && (_inverse_product_id == 0xd5)) {
                // LOG_DBG("HW Smart Algorithm detected");
                data->sw_smart_flag = false;
            } else {
                // LOG_DBG("HW Smart Algorithm not found. Using SW implementation");
                sw_smart_algorithm = true;
                data->sw_smart_flag = true;
            }
        }
    }
}
#endif

static void pixart_irq_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    struct pixart_data *data = CONTAINER_OF(cb, struct pixart_data, irq_gpio_cb);

    // don't do anything if not ready
    if (!data->ready) {
        return;
    }

    // trigger work execution
    k_work_submit(&data->trigger_work);
}

static void trigger_handler(struct k_work *work) {
    struct pixart_data *data = CONTAINER_OF(work, struct pixart_data, trigger_work);

    pmw3610_report_data(data->dev);
}

static int reg_write_switch_bank(const struct device *dev, uint8_t bank, bool *current_bank) {
    int err = -ENOTSUP;
    uint8_t tmp;

    if (bank != 0 && bank != 1) {
        return err;
    }

    // If current bank is already valid and same as requested, we're done
    if (current_bank != NULL && *current_bank == (bank == 1)) {
        return 0;
    }

    err = reg_write(dev, 0x7f, 0x00); // set bank 0 used for reading bank register
    if (err) {
        LOG_ERR("switch bank 0 failed");
        return err;
    }
    // Read REG_BANK_SEL (bit 4 for bank selection)
    spi_sclk_enable(dev, true);
    err = reg_read(dev, 0x09, &tmp);
    if (err) {
        LOG_ERR("read REG_BANK_SEL failed");
        return err;
    }
    spi_sclk_enable(dev, false);

    if (tmp & 0x10) {         // 0x10: bit 4 of REG_BANK_SEL
        if (bank == 0) {      // Curr bank 1, and need bank 0
            tmp &= ~0x10;     // clear bit 4 of REG_BANK_SEL
            err = reg_write(dev, 0x09, tmp);
        }
    } else {                  // Curr bank 0
        if (bank == 1) {      // need bank 1
            tmp |= 0x10;      // set bit 4 of REG_BANK_SEL
            err = reg_write(dev, 0x09, tmp);
        }
    }

    if (err) {
        LOG_ERR("set REG_BANK_SEL to %d failed: %d", !!(tmp & 0x10), err);
        return err;
    }

    // Save the new bank state if pointer is provided
    if (current_bank != NULL) {
        *current_bank = (bank == 1);
    }

    return 0;
}

static int set_cpi(const struct device *dev, int32_t cpi) {
    struct pixart_data *data = dev->data;
    int32_t requested_cpi;
    uint8_t cpival, cpival2;
    int err;

    requested_cpi = MIN(cpi, 3200);
    requested_cpi = MAX(requested_cpi, 200);
    requested_cpi = (requested_cpi / 200) * 200;
    cpival = (requested_cpi / 200) - 1;

    // Switch to bank 0 for CPI setting
    bool current_bank = false;
    err = reg_write_switch_bank(dev, 0, &current_bank);
    if (err) {
        LOG_ERR("Failed to switch to bank 0");
        return err;
    }

    err = reg_write(dev, Motion_Burst_Length, 6);
    if (err) {
        LOG_ERR("Failed to set motion burst length");
        return err;
    }

    spi_sclk_enable(dev, true);
    err = reg_read(dev, Config1, &cpival2);
    if (err) {
        LOG_ERR("Failed to read Config1");
        spi_sclk_enable(dev, false);
        return err;
    }
    spi_sclk_enable(dev, false);

    cpival2 = (cpival2 & 0xf0) | cpival;
    err = reg_write(dev, Config1, cpival2);
    if (err) {
        LOG_ERR("Failed to write Config1");
        return err;
    }

    // Update the current CPI value in the data structure
    data->curr_cpi = requested_cpi;

    return err;
}

static int update_last_report_time(const struct device *dev, int16_t x, int16_t y) {
    struct pixart_data *data = dev->data;

    data->last_x = x;
    data->last_y = y;
    data->last_poll_time = k_uptime_get();

    return 0;
}

#ifdef CONFIG_PMW3610_SMOOTHING_FILTER
static void apply_smoothing_filter(struct pixart_data *data, int16_t *x, int16_t *y) {
    // 平滑化用の重み係数 (0-100)
    float weight = CONFIG_PMW3610_SMOOTHING_WEIGHT / 100.0f;

    // プロファイルに応じて重み係数を調整
#ifdef CONFIG_PMW3610_PROFILE_SWITCHING
    if (data->precision_profile_active) {
        weight = CONFIG_PMW3610_PRECISION_PROFILE_SMOOTHING / 100.0f;
    } else if (data->speed_profile_active) {
        weight = CONFIG_PMW3610_SPEED_PROFILE_SMOOTHING / 100.0f;
    }
#endif

    int16_t cpi_factor = 1;
    if (IS_ENABLED(CONFIG_PMW3610_DPI_SCALING)) {
        cpi_factor = CONFIG_PMW3610_CPI / 400; // 基準CPI=400として調整係数を計算
        if (cpi_factor < 1) cpi_factor = 1;
    }

    // 動きの大きさに応じて適応的に重みを調整
    int16_t movement_size = abs(*x) + abs(*y);
    float adaptive_weight = weight;
    
    // CPIに応じて閾値を調整
    int16_t large_movement_threshold = (int16_t)(20 * cpi_factor);
    int16_t medium_movement_threshold = (int16_t)(10 * cpi_factor);
    
    if (movement_size > large_movement_threshold) {
        // 大きな動きの場合は重みを下げる（より即応的に）
        adaptive_weight = weight * 0.5f;
    } else if (movement_size > medium_movement_threshold) {
        // 中程度の動きの場合は重みを少し下げる
        adaptive_weight = weight * 0.7f;
    }

    // 初回の動きの場合は平滑化をスキップ
    if (data->prev_x == 0 && data->prev_y == 0 && (*x != 0 || *y != 0)) {
        // 値を記録するだけで平滑化はスキップ
        data->prev_x = *x;
        data->prev_y = *y;
        data->current_smoothing_weight = (uint8_t)(adaptive_weight * 100);
        return;
    }

    // 非常に小さな動きはノイズと見なして抑制
    if (abs(*x) <= 1 && abs(data->prev_x) <= 1) {
        *x = 0;
    }
    if (abs(*y) <= 1 && abs(data->prev_y) <= 1) {
        *y = 0;
    }

    // 指数移動平均を適用
    int16_t smoothed_x = (int16_t)(adaptive_weight * data->prev_x + (1.0f - adaptive_weight) * (*x));
    int16_t smoothed_y = (int16_t)(adaptive_weight * data->prev_y + (1.0f - adaptive_weight) * (*y));

    // 現在の値を保存
    data->prev_x = smoothed_x;
    data->prev_y = smoothed_y;
    
    // 現在のスムージング係数を保存（デバッグ用）
    data->current_smoothing_weight = (uint8_t)(adaptive_weight * 100);

    // 平滑化された値を出力
    *x = smoothed_x;
    *y = smoothed_y;
}
#endif

// === 意図的な動きパターン検出のための変数と関数 ===
// 意図的な動きパターン検出用の履歴保持
#define MOTION_HISTORY_SIZE 3
static int16_t motion_history_x[MOTION_HISTORY_SIZE] = {0};
static int16_t motion_history_y[MOTION_HISTORY_SIZE] = {0};
static int8_t history_index = 0;

// 意図的な動きかどうかをパターンから判断する関数
static bool is_intentional_movement(int16_t x, int16_t y, int16_t raw_movement) {
    // 新しい動きを履歴に記録
    motion_history_x[history_index] = x;
    motion_history_y[history_index] = y;
    history_index = (history_index + 1) % MOTION_HISTORY_SIZE;
    
    // 動きが非常に小さい場合は意図的でないと判断
    if (raw_movement <= 2) {
        return false;
    }
    
    // 方向の一貫性をチェック - 同じ方向への継続的な動きは意図的と判断
    bool consistent_direction = false;
    
    // x方向の一貫性をチェック（最新の2つの動き）
    if ((motion_history_x[0] > 1 && motion_history_x[1] > 1) ||
        (motion_history_x[0] < -1 && motion_history_x[1] < -1)) {
        consistent_direction = true;
    }
    
    // y方向の一貫性をチェック（最新の2つの動き）
    if ((motion_history_y[0] > 1 && motion_history_y[1] > 1) ||
        (motion_history_y[0] < -1 && motion_history_y[1] < -1)) {
        consistent_direction = true;
    }
    
    // 動きの大きさをチェック
    int16_t movement_size = abs(x) + abs(y);
    
    // CPIに応じて閾値を調整
    int16_t cpi_factor = 1;
    if (IS_ENABLED(CONFIG_PMW3610_DPI_SCALING)) {
        cpi_factor = CONFIG_PMW3610_CPI / 400; // 基準CPI=400として調整係数を計算
        if (cpi_factor < 1) cpi_factor = 1;
    }
    
    int16_t significant_threshold = 3 * cpi_factor;
    bool significant_movement = movement_size > significant_threshold;
    
    // 一貫した方向の動きかつ十分な大きさの動きの場合は意図的と判断
    return consistent_direction && significant_movement;
}

#define AUTOMOUSE_LAYER (DT_PROP(DT_DRV_INST(0), automouse_layer))
#if AUTOMOUSE_LAYER > 0
struct k_timer automouse_layer_timer;
static bool automouse_triggered = false;

static void activate_automouse_layer() {
    // すでにトリガーされていて正しいレイヤーがアクティブなら何もしない
    if (automouse_triggered && zmk_keymap_highest_layer_active() == AUTOMOUSE_LAYER) {
        // タイマーだけリセット
        k_timer_start(&automouse_layer_timer, K_MSEC(CONFIG_PMW3610_AUTOMOUSE_TIMEOUT_MS), K_NO_WAIT);
        return;
    }

    // レイヤー切り替え
    automouse_triggered = true;
    zmk_keymap_layer_activate(AUTOMOUSE_LAYER);
    
    // タイマー開始
    k_timer_start(&automouse_layer_timer, K_MSEC(CONFIG_PMW3610_AUTOMOUSE_TIMEOUT_MS), K_NO_WAIT);
    
    // デバッグ情報
    // LOG_DBG("Mouse layer activated, highest layer: %d", zmk_keymap_highest_layer_active());
}

static void deactivate_automouse_layer(struct k_timer *timer) {
    // 最近の動きがある場合は非アクティブ化をキャンセル
    int64_t current_time = k_uptime_get();
    if (current_time - g_last_movement_time < CONFIG_PMW3610_MOVEMENT_ACCUMULATION_TIME_MS) {
        // まだ動きが続いている可能性があるため、レイヤーをアクティブに保つ
        // LOG_DBG("Recent movement detected, extending mouse layer activation");
        k_timer_start(&automouse_layer_timer, K_MSEC(CONFIG_PMW3610_AUTOMOUSE_TIMEOUT_MS), K_NO_WAIT);
        return;
    }

    automouse_triggered = false;
    zmk_keymap_layer_deactivate(AUTOMOUSE_LAYER);
    // LOG_DBG("Mouse layer deactivated");
}

K_TIMER_DEFINE(automouse_layer_timer, deactivate_automouse_layer, NULL);
#endif

static int pmw3610_report_data(const struct device *dev) {
    struct pixart_data *data = dev->data;
    const struct pixart_config *config = dev->config;
    uint8_t buf[12];
    int16_t x = 0, y = 0;
    int16_t delta_x_raw = 0, delta_y_raw = 0;
    int err;
    enum pixart_input_mode input_mode = data->curr_mode;

    // don't do anything if not ready
    if (!data->ready) {
        LOG_ERR("%s: Device is not ready", dev->name);
        return -EBUSY;
    }

    // disabling SPI sclk
    err = spi_sclk_enable(dev, true);
    if (err) {
        LOG_ERR("Enabling SPI clock failed");
        return err;
    }

    // Motion is asserted
    err = read_burst(dev, Motion_Burst, buf, 12);
    if (err) {
        LOG_ERR("Motion burst read failed");
        return err;
    }

    err = spi_sclk_enable(dev, false);
    if (err) {
        LOG_ERR("Disabling SPI clock failed");
        return err;
    }

    // ushort = u16 = uint16_t
    // motion burst: https://shorturl.at/ikvzD (p77 of datasheet)
    delta_x_raw = TOINT16((((buf[3] & 0x0f) << 8) | buf[1]), 12);
    delta_y_raw = TOINT16((((buf[3] & 0xf0) << 4) | buf[2]), 12);

    const int16_t raw_movement = abs(delta_x_raw) + abs(delta_y_raw);
    
    // Apply the divisor to the raw values if needed
    x = delta_x_raw / CONFIG_PMW3610_CPI_DIVIDOR;
    y = delta_y_raw / CONFIG_PMW3610_CPI_DIVIDOR;

    // AUTOMOUSE_LAYER有効時にマウスレイヤーの有効を精密に判定するためのグローバル変数
    static int16_t g_movement_accumulator = 0;
    static int64_t g_last_movement_time = 0;
    // センサーの生の値を保存するための変数を追加
    static int16_t g_raw_movement_accumulator = 0;

    // apply rotation and axis inversion based on Kconfigs
#if defined(CONFIG_PMW3610_ORIENTATION_90)
    int16_t x_rotated = y;
    int16_t y_rotated = -x;
    x = x_rotated;
    y = y_rotated;
#elif defined(CONFIG_PMW3610_ORIENTATION_180)
    x = -x;
    y = -y;
#elif defined(CONFIG_PMW3610_ORIENTATION_270)
    int16_t x_rotated = -y;
    int16_t y_rotated = x;
    x = x_rotated;
    y = y_rotated;
#endif

#if defined(CONFIG_PMW3610_INVERT_X)
    x = -x;
#endif
#if defined(CONFIG_PMW3610_INVERT_Y)
    y = -y;
#endif

#ifdef CONFIG_PMW3610_SMOOTHING_FILTER
    apply_smoothing_filter(data, &x, &y);
#endif

#if AUTOMOUSE_LAYER > 0
    // 利用側のCPI設定値に依存せずに、 PMW3610_AUTOMOUSE_PIXEL_THRESHOLD で設定したピクセル相当の移動を検出するための閾値を計算
    // static const int16_t MOVEMENT_THRESHOLD = ceil(CONFIG_PMW3610_AUTOMOUSE_PIXEL_THRESHOLD / CONFIG_PMW3610_CPI / CONFIG_PMW3610_CPI_DIVIDOR * 1000);
    static const int16_t MOVEMENT_THRESHOLD = CONFIG_PMW3610_MOVEMENT_THRESHOLD;
    static const int64_t ACCUMULATION_TIME_MS = CONFIG_PMW3610_MOVEMENT_ACCUMULATION_TIME_MS;
    
    // CPIに基づく調整係数を計算
    static const float CPI_SCALING_FACTOR = 800.0f / CONFIG_PMW3610_CPI;
    // CPI値が低いほど、raw_movementの閾値係数を小さくする（より敏感に）
    static const int16_t RAW_THRESHOLD_MULTIPLIER = CPI_SCALING_FACTOR < 1.0f ? 2 : (int16_t)(2 * CPI_SCALING_FACTOR);

    if (input_mode == MOVE) {
        const int16_t movement_size = abs(x) + abs(y);

        int64_t current_time = k_uptime_get();
        // 前回の有効な動きからの経過時間を計算
        int64_t time_since_last_movement = current_time - g_last_movement_time;
        
        // 移動量を累積（通常の処理後の値と生の値の両方を累積）
        g_movement_accumulator += movement_size;
        g_raw_movement_accumulator += raw_movement;

        // 意図的な動きのパターン検出
        bool intentional = is_intentional_movement(x, y, raw_movement);
        
        // 意図的な動きと判断された場合は即座にレイヤー切り替え
        if (intentional && 
            (automouse_triggered || zmk_keymap_highest_layer_active() != AUTOMOUSE_LAYER)) {
            // LOG_DBG("Intentional movement detected, activating mouse layer immediately");
            activate_automouse_layer();
        } 
        // 通常の累積ベースの判定（従来のコード）
        else if (time_since_last_movement >= ACCUMULATION_TIME_MS) {
            // 通常の閾値チェックか生の動き値のチェックのどちらかで条件を満たせばレイヤーをアクティブにする
            if ((g_movement_accumulator > MOVEMENT_THRESHOLD || 
                 g_raw_movement_accumulator > MOVEMENT_THRESHOLD * RAW_THRESHOLD_MULTIPLIER) &&
                (automouse_triggered || zmk_keymap_highest_layer_active() != AUTOMOUSE_LAYER)) {
                activate_automouse_layer();
                
                /* LOG_INF("Activating mouse layer: acc=%d, raw_acc=%d, thresh=%d, raw_thresh=%d", 
                        g_movement_accumulator, g_raw_movement_accumulator, 
                        MOVEMENT_THRESHOLD, MOVEMENT_THRESHOLD * RAW_THRESHOLD_MULTIPLIER); */
            }
            
            // 累積値をリセット
            g_last_movement_time = current_time;
            g_movement_accumulator = 0;
            g_raw_movement_accumulator = 0;
        }
    }
#endif

// スクロールモードの実装
#if defined(CONFIG_ZMK_MOUSE_TICK_DURATION)
    size_t total_scroll_layers = config->scroll_layers_len;
    bool is_scroll = false;

    if (total_scroll_layers > 0) {
        int32_t active_layer = zmk_keymap_highest_layer_active();
        for (int i = 0; i < total_scroll_layers; i++) {
            if (active_layer == config->scroll_layers[i]) {
                is_scroll = true;
                break;
            }
        }
    }

    if (is_scroll) {
        input_mode = SCROLL;
        data->curr_mode = SCROLL;

        data->scroll_delta_x += x;
        data->scroll_delta_y += y;

        int32_t threshold = CONFIG_PMW3610_SCROLL_TICK;
        int16_t scroll_x = 0, scroll_y = 0;

        if (abs(data->scroll_delta_x) >= threshold) {
            scroll_x = data->scroll_delta_x > 0 ? 1 : -1;
            data->scroll_delta_x = 0;
        }

        if (abs(data->scroll_delta_y) >= threshold) {
            scroll_y = data->scroll_delta_y > 0 ? 1 : -1;
            data->scroll_delta_y = 0;
        }

#if defined(CONFIG_PMW3610_INVERT_SCROLL_X)
        scroll_x = -scroll_x;
#endif
#if defined(CONFIG_PMW3610_INVERT_SCROLL_Y)
        scroll_y = -scroll_y;
#endif

        if (scroll_x != 0 || scroll_y != 0) {
            input_report_rel(dev, INPUT_REL_WHEEL_X, scroll_x, false, K_FOREVER);
            input_report_rel(dev, INPUT_REL_WHEEL, scroll_y, true, K_FOREVER);
        }
    } else {
        // update to MOVE mode only when previous mode is not MOVE
        if (data->curr_mode != MOVE && input_mode == MOVE) {
            input_mode = MOVE;
            data->curr_mode = MOVE;
        }
    }
#endif

// スナイピングモードの実装
#define TOTAL_SNIPE_LAYERS (config->snipe_layers_len)
#if TOTAL_SNIPE_LAYERS > 0
    bool is_snipe = false;
    int32_t active_layer = zmk_keymap_highest_layer_active();
    for (int i = 0; i < TOTAL_SNIPE_LAYERS; i++) {
        if (active_layer == config->snipe_layers[i]) {
            is_snipe = true;
            break;
        }
    }

    if (is_snipe) {
        if (data->curr_mode != SNIPE) {
            data->curr_mode = SNIPE;
            int32_t snipe_cpi = CONFIG_PMW3610_SNIPE_CPI;
            // Store current CPI and set to snipe CPI
            set_cpi(dev, snipe_cpi);
        }
    } else if (data->curr_mode == SNIPE) {
        // Restore previous CPI
        int32_t cpi = data->curr_cpi;
        if (cpi != CONFIG_PMW3610_CPI) {
            set_cpi(dev, CONFIG_PMW3610_CPI);
        }
        data->curr_mode = MOVE;
    }
#endif

#ifdef CONFIG_PMW3610_POLLING_RATE_125_SW
    // Check time since last report.
    int64_t current_time = k_uptime_get();
    int64_t time_since_last_report = current_time - data->last_poll_time;

    // 時間ベースのポーリングレート制限（125Hzソフトウェア実装）
    if (time_since_last_report < 8) { // 125Hz = 8ms
        // ポーリングレート制限のため、現在の動きを保存して次回処理させる
        update_last_report_time(dev, x, y);

#if AUTOMOUSE_LAYER > 0
        if (input_mode == MOVE) {
            // 次回の呼び出し時まで生の動きの累積値を保持
            data->last_raw_movement = raw_movement;
        }
#endif
        
        return 0;
    } else {
        x += data->last_x;
        y += data->last_y;
        
#if AUTOMOUSE_LAYER > 0
        if (input_mode == MOVE) {
            // 保存していた生の動きの値を現在の累積値に加算
            g_raw_movement_accumulator += data->last_raw_movement;
        }
#endif

        update_last_report_time(dev, 0, 0);
        data->last_raw_movement = 0;
    }
#endif

// CPI スケーリング
#ifdef CONFIG_PMW3610_DPI_SCALING
    uint32_t current_cpi = CONFIG_PMW3610_CPI;
    uint32_t target_cpi = CONFIG_PMW3610_TARGET_CPI;
    uint32_t scaling_ratio = 1;

    if (IS_ENABLED(CONFIG_PMW3610_DPI_SCALING)) {
        scaling_ratio = target_cpi / current_cpi;
    }

    // スケーリングを適用
    if (current_cpi > 0) {
        x = x * scaling_ratio;
        y = y * scaling_ratio;
    }
#endif

    if (input_mode == MOVE) {
        // Updating sensor readings to the kernel
        if (x != 0 || y != 0) {
            input_report_rel(dev, INPUT_REL_X, x, false, K_FOREVER);
            input_report_rel(dev, INPUT_REL_Y, y, true, K_FOREVER);
        }
    }

    return 0;
}

#ifdef CONFIG_PMW3610_PROFILE_SWITCHING
int pmw3610_activate_precision_profile(bool enable) {
    struct pixart_data *data = ((const struct device *)DEVICE_DT_GET(DT_DRV_INST(0)))->data;
    
    // 変更がない場合は何もしない
    if (data->precision_profile_active == enable) {
        return 0;
    }
    
    // 精密プロファイルをアクティブにする前に現在のCPIをバックアップ
    if (enable && !data->precision_profile_active) {
        // 別のプロファイルがアクティブなら先に無効化
        if (data->speed_profile_active) {
            pmw3610_activate_speed_profile(false);
        }
        
        // 現在のCPIを保存
        data->backup_cpi = data->curr_cpi;
        
        // 精密プロファイル用のCPIを設定
        set_cpi(DEVICE_DT_GET(DT_DRV_INST(0)), CONFIG_PMW3610_PRECISION_PROFILE_CPI);
        
        // プロファイルフラグを設定
        data->precision_profile_active = true;
        
        // LOG_DBG("Precision profile activated. CPI=%d (backup=%d)", 
                CONFIG_PMW3610_PRECISION_PROFILE_CPI, data->backup_cpi);
    } 
    // 精密プロファイルを無効化
    else if (!enable && data->precision_profile_active) {
        // 元のCPIに戻す
        set_cpi(DEVICE_DT_GET(DT_DRV_INST(0)), data->backup_cpi);
        
        // プロファイルフラグをリセット
        data->precision_profile_active = false;
        
        // LOG_DBG("Precision profile deactivated. Restored CPI=%d", data->backup_cpi);
    }
    
    return 0;
}

int pmw3610_activate_speed_profile(bool enable) {
    struct pixart_data *data = ((const struct device *)DEVICE_DT_GET(DT_DRV_INST(0)))->data;
    
    // 変更がない場合は何もしない
    if (data->speed_profile_active == enable) {
        return 0;
    }
    
    // 高速プロファイルをアクティブにする前に現在のCPIをバックアップ
    if (enable && !data->speed_profile_active) {
        // 別のプロファイルがアクティブなら先に無効化
        if (data->precision_profile_active) {
            pmw3610_activate_precision_profile(false);
        }
        
        // 現在のCPIを保存
        data->backup_cpi = data->curr_cpi;
        
        // 高速プロファイル用のCPIを設定
        set_cpi(DEVICE_DT_GET(DT_DRV_INST(0)), CONFIG_PMW3610_SPEED_PROFILE_CPI);
        
        // プロファイルフラグを設定
        data->speed_profile_active = true;
        
        // LOG_DBG("Speed profile activated. CPI=%d (backup=%d)", 
                CONFIG_PMW3610_SPEED_PROFILE_CPI, data->backup_cpi);
    } 
    // 高速プロファイルを無効化
    else if (!enable && data->speed_profile_active) {
        // 元のCPIに戻す
        set_cpi(DEVICE_DT_GET(DT_DRV_INST(0)), data->backup_cpi);
        
        // プロファイルフラグをリセット
        data->speed_profile_active = false;
        
        // LOG_DBG("Speed profile deactivated. Restored CPI=%d", data->backup_cpi);
    }
    
    return 0;
}
#endif

// irq configure helper
static int pixart_irq_enable(const struct device *dev, bool enable) {
    struct pixart_data *data = dev->data;
    const struct pixart_config *config = dev->config;
    gpio_flags_t flags;

    if (enable) {
        flags = GPIO_INT_LEVEL_ACTIVE;
    } else {
        flags = GPIO_INT_DISABLE;
    }

    return gpio_pin_interrupt_configure_dt(&config->irq_gpio, flags);
}

// TODO: add to async init and export via api
static int motion_burst_read(const struct device *dev, uint8_t *buf, uint8_t size) {
    int err;

    //
    err = pixart_irq_enable(dev, false);
    if (err) {
        LOG_ERR("IRQ disable failed");
        return err;
    }

    err = spi_sclk_enable(dev, true);
    if (err) {
        LOG_ERR("Enabling SPI clock failed");
        return err;
    }

    err = read_burst(dev, Motion_Burst, buf, size);
    if (err) {
        LOG_ERR("Motion burst read failed");
        return err;
    }

    err = spi_sclk_enable(dev, false);
    if (err) {
        LOG_ERR("Disabling SPI clock failed");
        return err;
    }

    err = pixart_irq_enable(dev, true);
    if (err) {
        LOG_ERR("IRQ enable failed");
        return err;
    }

    return 0;
}

// ========================================================================
// Init sequence step implementation
//

/*
 * We assume init stage check has properly done, i.e.,
 * data->async_init_step == ASYNC_INIT_STEP_POWER_UP
 */
static int pmw3610_async_init_power_up(const struct device *dev) {
    struct pixart_data *data = dev->data;
    const struct pixart_config *config = dev->config;
    int err = 0;

    LOG_INF("Powering up");

    // irq enabling
    err = pixart_irq_enable(dev, false);
    if (err) {
        LOG_ERR("IRQ disable failed");
        return err;
    }

    // hardware config
    // --- spi cs line
    if (device_is_ready(config->cs_gpio.port)) {
        err = gpio_pin_configure_dt(&config->cs_gpio, GPIO_OUTPUT_INACTIVE);
        if (err) {
            LOG_ERR("CS GPIO configuration failed");
            return err;
        }
    } else {
        LOG_ERR("CS GPIO device not ready");
        return -ENODEV;
    }

    // --- irq line
    if (device_is_ready(config->irq_gpio.port)) {
        err = gpio_pin_configure_dt(&config->irq_gpio, GPIO_INPUT);
        if (err) {
            LOG_ERR("IRQ GPIO configuration failed");
            return err;
        }

        gpio_init_callback(&data->irq_gpio_cb, pixart_irq_handler, BIT(config->irq_gpio.pin));

        err = gpio_add_callback(config->irq_gpio.port, &data->irq_gpio_cb);
        if (err) {
            LOG_ERR("IRQ GPIO add callback failed");
            return err;
        }
    } else {
        LOG_ERR("IRQ GPIO device not ready");
        return -ENODEV;
    }

    // --- spi bus
    if (!device_is_ready(config->bus.bus)) {
        LOG_ERR("SPI bus device not ready");
        return -ENODEV;
    }

    // power-up reset
    err = spi_cs_ctrl(dev, false);
    if (err) {
        LOG_ERR("SPI cs toggle failed");
        return err;
    }

    k_sleep(K_MSEC(1));

    err = spi_cs_ctrl(dev, true);
    if (err) {
        LOG_ERR("SPI cs toggle failed");
        return err;
    }

    k_sleep(K_MSEC(50));

    err = spi_cs_ctrl(dev, false);
    if (err) {
        LOG_ERR("SPI cs toggle failed");
        return err;
    }

    k_sleep(K_MSEC(15));

    // allow datasheet define async schedule to next step
    // data->async_init_step = ASYNC_INIT_STEP_CLEAR_OB1;

    return err;
}

static int pmw3610_async_init_clear_ob1(const struct device *dev) {
    struct pixart_data *data = dev->data;
    int err = 0;

    LOG_INF("Enable clock");

    err = spi_sclk_enable(dev, true);
    if (err) {
        LOG_ERR("Enabling SPI clock failed");
        return err;
    }

    LOG_INF("ASYNC_INIT_STEP_CLEAR_OB1");

    uint8_t value;
    err = reg_read(dev, Observation, &value);
    if (err) {
        LOG_ERR("Failed to read Observation register");
        return err;
    }
    // LOG_DBG("Sensor observation: 0x%x", value);

    /* 
     * 
     */
    err = reg_write(dev, Config1, 0x14); //400cpi
    if (err) {
        LOG_ERR("Failed to set Config1");
        return err;
    }

    /*
     * Observation1 register used to check if the sensor is ready
     * for operation. It was set to 0x00 during power-up.
     */
    err = reg_write(dev, Observation, 0x00);
    if (err) {
        LOG_ERR("Failed to set Observation");
        return err;
    }

    // this is to let the sensor run self-test
    err = spi_sclk_enable(dev, false);
    if (err) {
        LOG_ERR("Disabling SPI clock failed");
        return err;
    }

    // allow datasheet define async schedule to next step
    // data->async_init_step = ASYNC_INIT_STEP_CHECK_OB1;

    return err;
}

static int pmw3610_async_init_check_ob1(const struct device *dev) {
    struct pixart_data *data = dev->data;
    int err = 0;

    LOG_INF("ASYNC_INIT_STEP_CHECK_OB1");

    err = spi_sclk_enable(dev, true);
    if (err) {
        LOG_ERR("Enabling SPI clock failed");
        return err;
    }

    /*
     * Observation1 register is self-populated by sensor doing
     * self-test during power-up. Its value must be 0xa0 to
     * verify the self-test finishes.
     */
    uint8_t value;
    err = reg_read(dev, Observation, &value);
    if (err) {
        LOG_ERR("Failed to read Observation");
        return err;
    }

    // LOG_DBG("Sensor observation: 0x%x", value);

    if (value != 0xA0 && value != 0x80) {
        LOG_ERR("Sensor self-test failed: 0x%x", value);
        return -EIO;
    }

    uint8_t b_pid, b_rid, b_inid;
    err = reg_read(dev, Observation, &b_pid);
    if (err) {
        LOG_ERR("Failed to read PID");
        return err;
    }

    err = reg_read(dev, 0x2a, &b_rid);
    if (err) {
        LOG_ERR("Failed to read RID");
        return err;
    }

    err = reg_read(dev, 0x3f, &b_inid);
    if (err) {
        LOG_ERR("Failed to read InversePID");
        return err;
    }

    LOG_INF("PID: 0x%x, RID: 0x%x, InversePID: 0x%x", b_pid, b_rid, b_inid);

    if ((b_pid != 0x2A)) {
        LOG_ERR("Product ID mismatch");
        return -EIO;
    }

    err = spi_sclk_enable(dev, false);
    if (err) {
        LOG_ERR("Disabling SPI clock failed");
        return err;
    }

    // allow datasheet define async schedule to next step
    // data->async_init_step = ASYNC_INIT_STEP_CONFIGURE;

    return err;
}

static int pmw3610_async_init_configure(const struct device *dev) {
    struct pixart_data *data = dev->data;
    const struct pixart_config *config = dev->config;
    int err = 0;
    uint8_t value;

    LOG_INF("ASYNC_INIT_STEP_CONFIGURE");

    err = spi_sclk_enable(dev, true);
    if (err) {
        LOG_ERR("Enabling SPI clock failed");
        return err;
    }

    /* Sensor configuration. */
    err = reg_write(dev, Config1, (CONFIG_PMW3610_CPI / 200) - 1);
    if (err) {
        LOG_ERR("Failed to set Config1");
        return err;
    }

    err = reg_read(dev, Config1, &value);
    if (err) {
        LOG_ERR("Failed to read Config1");
        return err;
    }
    // LOG_DBG("Config1: 0x%x", value);
    data->curr_cpi = (value & 0x0f) * 200 + 200;

    /* Set run mode */
    err = reg_write(dev, Run_Downshift, CONFIG_PMW3610_RUN_DOWNSHIFT_TIME_MS / 10);
    if (err) {
        LOG_ERR("Failed to set run downshift time");
        return err;
    }

    /* Rest1 mode */
    if (CONFIG_PMW3610_REST1_SAMPLE_TIME_MS < 10) {
        LOG_ERR("Rest1 mode sample time below 10ms not supported");
        return -ENOTSUP;
    }
    uint8_t rest1_sample_time = CONFIG_PMW3610_REST1_SAMPLE_TIME_MS / 10 - 1;
    if (rest1_sample_time > 0xff) {
        rest1_sample_time = 0xff;
    }
    err = reg_write(dev, Rest1_Rate, rest1_sample_time);
    if (err) {
        LOG_ERR("Failed to set rest1 sample time");
        return err;
    }

    err = reg_write(dev, Rest1_Downshift, CONFIG_PMW3610_REST1_DOWNSHIFT_TIME_MS / 10);
    if (err) {
        LOG_ERR("Failed to set rest1 downshift time");
        return err;
    }

    /* Rest2 mode */
    if (CONFIG_PMW3610_REST2_SAMPLE_TIME_MS > 0) {
        if (CONFIG_PMW3610_REST2_SAMPLE_TIME_MS < 10) {
            LOG_ERR("Rest2 mode sample time below 10ms not supported");
            return -ENOTSUP;
        }
        uint8_t rest2_sample_time = CONFIG_PMW3610_REST2_SAMPLE_TIME_MS / 10 - 1;
        if (rest2_sample_time > 0xff) {
            rest2_sample_time = 0xff;
        }
        err = reg_write(dev, Rest2_Rate, rest2_sample_time);
        if (err) {
            LOG_ERR("Failed to set rest2 sample time");
            return err;
        }
    }

    if (CONFIG_PMW3610_REST2_DOWNSHIFT_TIME_MS > 0) {
        err = reg_write(dev, Rest2_Downshift, CONFIG_PMW3610_REST2_DOWNSHIFT_TIME_MS / 10);
        if (err) {
            LOG_ERR("Failed to set rest2 downshift time");
            return err;
        }
    }

    /* Rest3 mode */
    if (CONFIG_PMW3610_REST3_SAMPLE_TIME_MS > 0) {
        if (CONFIG_PMW3610_REST3_SAMPLE_TIME_MS < 10) {
            LOG_ERR("Rest3 mode sample time below 10ms not supported");
            return -ENOTSUP;
        }
        uint8_t rest3_sample_time = CONFIG_PMW3610_REST3_SAMPLE_TIME_MS / 10 - 1;
        if (rest3_sample_time > 0xff) {
            rest3_sample_time = 0xff;
        }
        err = reg_write(dev, Rest3_Rate, rest3_sample_time);
        if (err) {
            LOG_ERR("Failed to set rest3 sample time");
            return err;
        }
    }

#if defined(CONFIG_PMW3610_POLLING_RATE_250)
    err = reg_write(dev, 0x1d, 0xAA);
    if (err) {
        LOG_ERR("Failed to set polling rate to 250Hz");
        return err;
    }
#endif

    /* Motion Burst */
    err = reg_write(dev, Motion_Burst_Length, 6);
    if (err) {
        LOG_ERR("Failed to set motion burst length");
        return err;
    }

    /* Disable RESTn auto low power mode */
    if (IS_ENABLED(CONFIG_PMW3610_FORCE_AWAKE)) {
        err = reg_write(dev, 0x0C, 0x19);
        if (err) {
            LOG_ERR("Failed to force awake");
            return err;
        }
    }

    /* Clear motion registers (just in case). */
    for (uint8_t i = 0; i < 3; i++) {
        err = motion_burst_read(dev, NULL, 6);
        if (err) {
            LOG_ERR("Failed to clear motion registers");
            return err;
        }
    }

    err = spi_sclk_enable(dev, false);
    if (err) {
        LOG_ERR("Disabling SPI clock failed");
        return err;
    }

    data->ready = true;
    pixart_irq_enable(dev, true);

    LOG_INF("Finished initialization");

    return err;
}

static void pmw3610_async_init(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct pixart_data *data = CONTAINER_OF(dwork, struct pixart_data, init_work);
    const struct device *dev = data->dev;

    LOG_INF("=pmw3610_async_init %d=", data->async_init_step);

    __ASSERT_NO_MSG(data->async_init_step < ASYNC_INIT_STEP_COUNT);

    // Check if there's any error during init
    if (data->err) {
        LOG_ERR("Init aborted due to previous error: %d", data->err);
        return;
    }

    data->err = async_init_fn[data->async_init_step](dev);
    if (data->err) {
        LOG_ERR("Init step failed: %d, err: %d", data->async_init_step, data->err);
        return;
    }

    data->async_init_step++;
    if (data->async_init_step < ASYNC_INIT_STEP_COUNT) {
        LOG_INF("Scheduled next init step. Sleep for %dms", async_init_delay[data->async_init_step]);
        k_work_schedule(&data->init_work, K_MSEC(async_init_delay[data->async_init_step]));
    } 
#ifdef CONFIG_PMW3610_SMART_ALGORITHM
    else {
        // only check after init is completed
        update_smart_algorithm_flag(dev);
    }
#endif
}

static int pmw3610_init(const struct device *dev) {
    struct pixart_data *data = dev->data;
    data->dev = dev;

    LOG_INF("PMW3610 initializing [CPI=%d]", CONFIG_PMW3610_CPI);

    data->ready = false;
    data->async_init_step = ASYNC_INIT_STEP_POWER_UP;
    data->curr_mode = MOVE;

    /* Initialize work structures */
    k_work_init(&data->trigger_work, trigger_handler);
    k_work_init_delayable(&data->init_work, pmw3610_async_init);

    /* Start adaptive initialization */
    k_work_schedule(&data->init_work, K_MSEC(async_init_delay[ASYNC_INIT_STEP_POWER_UP]));

    return 0;
}

static const struct sensor_driver_api pmw3610_driver_api = {
    .sample_fetch = (void *)1, //disable API for now
    .channel_get = (void *)1,  //disable API for now
};

#define PMW3610_DEFINE(inst)                                                                        \
                                                                                                     \
    static struct pixart_data pmw3610_data_##inst = {                                               \
        .curr_mode = MOVE,                                                                           \
    };                                                                                               \
                                                                                                     \
    static int32_t snipe_layers_##inst[] = DT_INST_PROP_OR(inst, snipe_layers, {});                 \
    static int32_t scroll_layers_##inst[] = DT_INST_PROP_OR(inst, scroll_layers, {});               \
                                                                                                     \
    static const struct pixart_config pmw3610_config_##inst = {                                     \
        .irq_gpio = GPIO_DT_SPEC_INST_GET(inst, irq_gpios),                                          \
        .bus = SPI_DT_SPEC_INST_GET(inst, SPI_WORD_SET(8) | SPI_TRANSFER_MSB, 1),                   \
        .cs_gpio = SPI_CS_GPIOS_DT_SPEC_GET(DT_INST(inst, DT_DRV_COMPAT)),                          \
        .snipe_layers_len = ARRAY_SIZE(snipe_layers_##inst),                                         \
        .snipe_layers = snipe_layers_##inst,                                                         \
        .scroll_layers_len = ARRAY_SIZE(scroll_layers_##inst),                                       \
        .scroll_layers = scroll_layers_##inst,                                                       \
    };                                                                                               \
                                                                                                     \
    DEVICE_DT_INST_DEFINE(inst, pmw3610_init, NULL, &pmw3610_data_##inst, &pmw3610_config_##inst,   \
                            POST_KERNEL, CONFIG_INPUT_INIT_PRIORITY, &pmw3610_driver_api);

DT_INST_FOREACH_STATUS_OKAY(PMW3610_DEFINE)