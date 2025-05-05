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
#include <math.h>

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

    uint8_t buf[] = {SPI_WRITE_BIT | reg, val};
    const struct spi_buf tx_buf = {.buf = buf, .len = ARRAY_SIZE(buf)};
    const struct spi_buf_set tx = {.buffers = &tx_buf, .count = 1};

    err = spi_write_dt(&config->bus, &tx);
    if (err) {
        LOG_ERR("Reg write failed on SPI write");
        return err;
    }

    k_busy_wait(T_SCLK_NCS_WR);

    err = spi_cs_ctrl(dev, false);
    if (err) {
        return err;
    }

    k_busy_wait(T_SWX);

    return 0;
}

static int reg_write(const struct device *dev, uint8_t reg, uint8_t val) {
    int err;

    // enable spi clock
    err = _reg_write(dev, PMW3610_REG_SPI_CLK_ON_REQ, PMW3610_SPI_CLOCK_CMD_ENABLE);
    if (unlikely(err != 0)) {
        return err;
    }

    // write the target register
    err = _reg_write(dev, reg, val);
    if (unlikely(err != 0)) {
        return err;
    }

    // disable spi clock to save power
    err = _reg_write(dev, PMW3610_REG_SPI_CLK_ON_REQ, PMW3610_SPI_CLOCK_CMD_DISABLE);
    if (unlikely(err != 0)) {
        return err;
    }

    return 0;
}

static int motion_burst_read(const struct device *dev, uint8_t *buf, size_t burst_size) {
    int err;
    /* struct pixart_data *data = dev->data; */
    const struct pixart_config *config = dev->config;

    __ASSERT_NO_MSG(burst_size <= PMW3610_MAX_BURST_SIZE);

    err = spi_cs_ctrl(dev, true);
    if (err) {
        return err;
    }

    /* Send motion burst address */
    uint8_t reg_buf[] = {PMW3610_REG_MOTION_BURST};
    const struct spi_buf tx_buf = {.buf = reg_buf, .len = ARRAY_SIZE(reg_buf)};
    const struct spi_buf_set tx = {.buffers = &tx_buf, .count = 1};

    err = spi_write_dt(&config->bus, &tx);
    if (err) {
        LOG_ERR("Motion burst failed on SPI write");
        return err;
    }

    k_busy_wait(T_SRAD_MOTBR);

    const struct spi_buf rx_buf = {
        .buf = buf,
        .len = burst_size,
    };
    const struct spi_buf_set rx = {.buffers = &rx_buf, .count = 1};

    err = spi_read_dt(&config->bus, &rx);
    if (err) {
        LOG_ERR("Motion burst failed on SPI read");
        return err;
    }

    err = spi_cs_ctrl(dev, false);
    if (err) {
        return err;
    }

    /* Terminate burst */
    k_busy_wait(T_BEXIT);

    return 0;
}

/** Writing an array of registers in sequence, used in power-up register initialization and running
 * mode switching */
static int burst_write(const struct device *dev, const uint8_t *addr, const uint8_t *buf,
                       size_t size) {
    int err;

    // enable spi clock
    err = _reg_write(dev, PMW3610_REG_SPI_CLK_ON_REQ, PMW3610_SPI_CLOCK_CMD_ENABLE);
    if (unlikely(err != 0)) {
        return err;
    }

    /* Write data */
    for (size_t i = 0; i < size; i++) {
        err = _reg_write(dev, addr[i], buf[i]);

        if (err) {
            LOG_ERR("Burst write failed on SPI write (data)");
            return err;
        }
    }

    // disable spi clock to save power
    err = _reg_write(dev, PMW3610_REG_SPI_CLK_ON_REQ, PMW3610_SPI_CLOCK_CMD_DISABLE);
    if (unlikely(err != 0)) {
        return err;
    }

    return 0;
}

static int check_product_id(const struct device *dev) {
    uint8_t product_id = 0x01;
    int err = reg_read(dev, PMW3610_REG_PRODUCT_ID, &product_id);
    if (err) {
        LOG_ERR("Cannot obtain product id");
        return err;
    }

    if (product_id != PMW3610_PRODUCT_ID) {
        LOG_ERR("Incorrect product id 0x%x (expecting 0x%x)!", product_id, PMW3610_PRODUCT_ID);
        return -EIO;
    }

    return 0;
}

static int set_cpi(const struct device *dev, uint32_t cpi) {
    /* Set resolution with CPI step of 200 cpi
     * 0x1: 200 cpi (minimum cpi)
     * 0x2: 400 cpi
     * 0x3: 600 cpi
     * :
     */

    if ((cpi > PMW3610_MAX_CPI) || (cpi < PMW3610_MIN_CPI)) {
        LOG_ERR("CPI value %u out of range", cpi);
        return -EINVAL;
    }

    // Convert CPI to register value
    uint8_t value = (cpi / 200);
    LOG_INF("Setting CPI to %u (reg value 0x%x)", cpi, value);

    /* set the cpi */
    uint8_t addr[] = {0x7F, PMW3610_REG_RES_STEP, 0x7F};
    uint8_t data[] = {0xFF, value, 0x00};
    int err = burst_write(dev, addr, data, 3);
    if (err) {
        LOG_ERR("Failed to set CPI");
        return err;
    }

    struct pixart_data *dev_data = dev->data;
    dev_data->curr_cpi = cpi;

    return 0;
}

static int set_cpi_if_needed(const struct device *dev, uint32_t cpi) {
    struct pixart_data *data = dev->data;
    if (cpi != data->curr_cpi) {
        return set_cpi(dev, cpi);
    }
    return 0;
}

/* Set sampling rate in each mode (in ms) */
static int set_sample_time(const struct device *dev, uint8_t reg_addr, uint32_t sample_time) {
    uint32_t maxtime = 2550;
    uint32_t mintime = 10;
    if ((sample_time > maxtime) || (sample_time < mintime)) {
        LOG_WRN("Sample time %u out of range [%u, %u]", sample_time, mintime, maxtime);
        return -EINVAL;
    }

    uint8_t value = sample_time / mintime;
    LOG_INF("Set sample time to %u ms (reg value: 0x%x)", sample_time, value);

    /* The sample time is (reg_value * mintime ) ms. 0x00 is rounded to 0x1 */
    int err = reg_write(dev, reg_addr, value);
    if (err) {
        LOG_ERR("Failed to change sample time");
    }

    return err;
}

/* Set downshift time in ms. */
// NOTE: The unit of run-mode downshift is related to pos mode rate, which is hard coded to be 4 ms
// The pos-mode rate is configured in pmw3610_async_init_configure
static int set_downshift_time(const struct device *dev, uint8_t reg_addr, uint32_t time) {
    uint32_t maxtime;
    uint32_t mintime;

    switch (reg_addr) {
    case PMW3610_REG_RUN_DOWNSHIFT:
        /*
         * Run downshift time = PMW3610_REG_RUN_DOWNSHIFT
         *                      * 8 * pos-rate (fixed to 4ms)
         */
        maxtime = 32 * 255;
        mintime = 32; // hard-coded in pmw3610_async_init_configure
        break;

    case PMW3610_REG_REST1_DOWNSHIFT:
        /*
         * Rest1 downshift time = PMW3610_REG_RUN_DOWNSHIFT
         *                        * 16 * Rest1_sample_period (default 40 ms)
         */
        maxtime = 255 * 16 * CONFIG_PMW3610_REST1_SAMPLE_TIME_MS;
        mintime = 16 * CONFIG_PMW3610_REST1_SAMPLE_TIME_MS;
        break;

    case PMW3610_REG_REST2_DOWNSHIFT:
        /*
         * Rest2 downshift time = PMW3610_REG_REST2_DOWNSHIFT
         *                        * 128 * Rest2 rate (default 100 ms)
         */
        maxtime = 255 * 128 * CONFIG_PMW3610_REST2_SAMPLE_TIME_MS;
        mintime = 128 * CONFIG_PMW3610_REST2_SAMPLE_TIME_MS;
        break;

    default:
        LOG_ERR("Not supported");
        return -ENOTSUP;
    }

    if ((time > maxtime) || (time < mintime)) {
        LOG_WRN("Downshift time %u out of range", time);
        return -EINVAL;
    }

    __ASSERT_NO_MSG((mintime > 0) && (maxtime / mintime <= UINT8_MAX));

    /* Convert time to register value */
    uint8_t value = time / mintime;

    LOG_INF("Set downshift time to %u ms (reg value 0x%x)", time, value);

    int err = reg_write(dev, reg_addr, value);
    if (err) {
        LOG_ERR("Failed to change downshift time");
    }

    return err;
}

static void set_interrupt(const struct device *dev, const bool en) {
    const struct pixart_config *config = dev->config;
    int ret = gpio_pin_interrupt_configure_dt(&config->irq_gpio,
                                              en ? GPIO_INT_LEVEL_ACTIVE : GPIO_INT_DISABLE);
    if (ret < 0) {
        LOG_ERR("can't set interrupt");
    }
}

static int pmw3610_async_init_power_up(const struct device *dev) {
    LOG_INF("async_init_power_up");

    /* Reset spi port */
    spi_cs_ctrl(dev, false);
    spi_cs_ctrl(dev, true);

    /* not required in datashet, but added any way to have a clear state */
    return reg_write(dev, PMW3610_REG_POWER_UP_RESET, PMW3610_POWERUP_CMD_RESET);
}

static int pmw3610_async_init_clear_ob1(const struct device *dev) {
    LOG_INF("async_init_clear_ob1");

    return reg_write(dev, PMW3610_REG_OBSERVATION, 0x00);
}

static int pmw3610_async_init_check_ob1(const struct device *dev) {
    LOG_INF("async_init_check_ob1");

    uint8_t value;
    int err = reg_read(dev, PMW3610_REG_OBSERVATION, &value);
    if (err) {
        LOG_ERR("Can't do self-test");
        return err;
    }

    if ((value & 0x0F) != 0x0F) {
        LOG_ERR("Failed self-test (0x%x)", value);
        return -EINVAL;
    }

    err = check_product_id(dev);
    if (err) {
        LOG_ERR("Failed checking product id");
        return err;
    }

    return 0;
}

static int pmw3610_async_init_configure(const struct device *dev) {
    LOG_INF("async_init_configure");

    int err = 0;

    // clear motion registers first (required in datasheet)
    for (uint8_t reg = 0x02; (reg <= 0x05) && !err; reg++) {
        uint8_t buf[1];
        err = reg_read(dev, reg, buf);
    }

    // cpi
    if (!err) {
        err = set_cpi(dev, CONFIG_PMW3610_CPI);
    }

    // set performace register: run mode, vel_rate, poshi_rate, poslo_rate
    if (!err) {
        err = reg_write(dev, PMW3610_REG_PERFORMANCE, PMW3610_PERFORMANCE_VALUE);
        LOG_INF("Set performance register (reg value 0x%x)", PMW3610_PERFORMANCE_VALUE);
    }

    // required downshift and rate registers
    if (!err) {
        err = set_downshift_time(dev, PMW3610_REG_RUN_DOWNSHIFT,
                                 CONFIG_PMW3610_RUN_DOWNSHIFT_TIME_MS);
    }
    if (!err) {
        err = set_sample_time(dev, PMW3610_REG_REST1_PERIOD, CONFIG_PMW3610_REST1_SAMPLE_TIME_MS);
    }
    if (!err) {
        err = set_downshift_time(dev, PMW3610_REG_REST1_DOWNSHIFT,
                                 CONFIG_PMW3610_REST1_DOWNSHIFT_TIME_MS);
    }

    // downshift time for each rest mode
#if CONFIG_PMW3610_REST2_DOWNSHIFT_TIME_MS > 0
    if (!err) {
        err = set_downshift_time(dev, PMW3610_REG_REST2_DOWNSHIFT,
                                 CONFIG_PMW3610_REST2_DOWNSHIFT_TIME_MS);
    }
#endif
#if CONFIG_PMW3610_REST2_SAMPLE_TIME_MS >= 10
    if (!err) {
        err = set_sample_time(dev, PMW3610_REG_REST2_PERIOD, CONFIG_PMW3610_REST2_SAMPLE_TIME_MS);
    }
#endif
#if CONFIG_PMW3610_REST3_SAMPLE_TIME_MS >= 10
    if (!err) {
        err = set_sample_time(dev, PMW3610_REG_REST3_PERIOD, CONFIG_PMW3610_REST3_SAMPLE_TIME_MS);
    }
#endif
    if (err) {
        LOG_ERR("Config the sensor failed");
        return err;
    }

    return 0;
}

// checked and keep
static void pmw3610_async_init(struct k_work *work) {
    struct k_work_delayable *work2 = (struct k_work_delayable *)work;
    struct pixart_data *data = CONTAINER_OF(work2, struct pixart_data, init_work);
    const struct device *dev = data->dev;

    LOG_INF("PMW3610 async init step %d", data->async_init_step);

    data->err = async_init_fn[data->async_init_step](dev);
    if (data->err) {
        LOG_ERR("PMW3610 initialization failed");
    } else {
        data->async_init_step++;

        if (data->async_init_step == ASYNC_INIT_STEP_COUNT) {
            data->ready = true; // sensor is ready to work
            LOG_INF("PMW3610 initialized");
            set_interrupt(dev, true);
        } else {
            k_work_schedule(&data->init_work, K_MSEC(async_init_delay[data->async_init_step]));
        }
    }
}

#define AUTOMOUSE_LAYER (DT_PROP(DT_DRV_INST(0), automouse_layer))
#if AUTOMOUSE_LAYER > 0
struct k_timer automouse_layer_timer;
static bool automouse_triggered = false;

static void activate_automouse_layer() {
    automouse_triggered = true;
    zmk_keymap_layer_activate(AUTOMOUSE_LAYER);
    k_timer_start(&automouse_layer_timer, K_MSEC(CONFIG_PMW3610_AUTOMOUSE_TIMEOUT_MS), K_NO_WAIT);
}

static void deactivate_automouse_layer(struct k_timer *timer) {
    automouse_triggered = false;
    zmk_keymap_layer_deactivate(AUTOMOUSE_LAYER);
}

K_TIMER_DEFINE(automouse_layer_timer, deactivate_automouse_layer, NULL);
#endif

static enum pixart_input_mode get_input_mode_for_current_layer(const struct device *dev) {
    const struct pixart_config *config = dev->config;
    uint8_t curr_layer = zmk_keymap_highest_layer_active();
    for (size_t i = 0; i < config->scroll_layers_len; i++) {
        if (curr_layer == config->scroll_layers[i]) {
            return SCROLL;
        }
    }
    for (size_t i = 0; i < config->snipe_layers_len; i++) {
        if (curr_layer == config->snipe_layers[i]) {
            return SNIPE;
        }
    }
    return MOVE;
}


// AUTOMOUSE_LAYER有効時にマウスレイヤーの有効を精密に判定するためのグローバル変数
static int16_t g_movement_accumulator = 0;
static int64_t g_last_movement_time = 0;
// センサーの生の値を保存するための変数を追加
static int16_t g_raw_movement_accumulator = 0;

static int pmw3610_report_data(const struct device *dev) {
    struct pixart_data *data = dev->data;
    uint8_t buf[PMW3610_BURST_SIZE];

    if (unlikely(!data->ready)) {
        LOG_WRN("Device is not initialized yet");
        return -EBUSY;
    }

    int32_t dividor;
    enum pixart_input_mode input_mode = get_input_mode_for_current_layer(dev);
    bool input_mode_changed = data->curr_mode != input_mode;
    switch (input_mode) {
    case MOVE:
        set_cpi_if_needed(dev, CONFIG_PMW3610_CPI);
        dividor = CONFIG_PMW3610_CPI_DIVIDOR;
        if (input_mode_changed) {
            // 前のモードからMOVEモードに切り替わった場合は累積値をリセットするが、
            // AMLの状態は維持する（automouse_triggered はリセットしない）
            g_movement_accumulator = 0;
            g_raw_movement_accumulator = 0;
            // g_last_movement_time は更新しない - これにより前のモードでの最後の動きからの時間が計測され、
            // すぐにAMLのチェックが行われる
        }
        break;
    case SCROLL:
        set_cpi_if_needed(dev, CONFIG_PMW3610_CPI);
        if (input_mode_changed) {
            data->scroll_delta_x = 0;
            data->scroll_delta_y = 0;
            data->scroll_acceleration = 1.0f;
            data->scroll_consecutive_movements = 0;
            data->scroll_last_movement_time = 0;
            
            // スクロール補間変数もリセット
            data->scroll_last_direction_x = 0;
            data->scroll_last_direction_y = 0;
            data->scroll_missed_detection_count = 0;
            data->scroll_consistent_direction_count = 0;
            data->scroll_last_real_movement_time = 0;
        }
        dividor = CONFIG_PMW3610_SCROLL_DIVIDOR; // スクロールモード専用のdividor値を使用
        break;
    case SNIPE:
        set_cpi_if_needed(dev, CONFIG_PMW3610_SNIPE_CPI);
        dividor = CONFIG_PMW3610_SNIPE_CPI_DIVIDOR;
        break;
    default:
        return -ENOTSUP;
    }

    data->curr_mode = input_mode;

    int16_t x;
    int16_t y;

    int err = motion_burst_read(dev, buf, sizeof(buf));
    if (err) {
        return err;
    }

    int16_t raw_x =
        TOINT16((buf[PMW3610_X_L_POS] + ((buf[PMW3610_XY_H_POS] & 0xF0) << 4)), 12) / dividor;
    int16_t raw_y =
        TOINT16((buf[PMW3610_Y_L_POS] + ((buf[PMW3610_XY_H_POS] & 0x0F) << 8)), 12) / dividor;

    // センサーからの生の動きの値を計算（変換前）
    const int16_t raw_movement = abs(raw_x) + abs(raw_y);

    if (IS_ENABLED(CONFIG_PMW3610_ORIENTATION_0)) {
        x = -raw_x;
        y = raw_y;
    } else if (IS_ENABLED(CONFIG_PMW3610_ORIENTATION_90)) {
        x = raw_y;
        y = -raw_x;
    } else if (IS_ENABLED(CONFIG_PMW3610_ORIENTATION_180)) {
        x = raw_x;
        y = -raw_y;
    } else if (IS_ENABLED(CONFIG_PMW3610_ORIENTATION_270)) {
        x = -raw_y;
        y = raw_x;
    }

    if (IS_ENABLED(CONFIG_PMW3610_INVERT_X)) {
        x = -x;
    }

    if (IS_ENABLED(CONFIG_PMW3610_INVERT_Y)) {
        y = -y;
    }

#ifdef CONFIG_PMW3610_SMOOTHING_FILTER
    // 平滑化フィルターの適用
    if (data->ready) { // 初期化後のみ適用
        // スムージングの重み付け係数 (0-90%)
        const float weight = data->current_smoothing_weight / 100.0f;
        
        // CPIに基づく調整係数を計算（800dpiを基準）
        float cpi_factor = 800.0f / CONFIG_PMW3610_CPI;
        
        // 移動量の大きさに基づいて適応的に重みを調整
        // 大きな動きには少ない重みを適用し、小さな動きには大きな重みを適用
        int16_t movement_size = abs(x) + abs(y);
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
        if (data->prev_x == 0 && data->prev_y == 0 && (x != 0 || y != 0)) {
            // 値を記録するだけで平滑化はスキップ
            data->prev_x = x;
            data->prev_y = y;
        } else {
            // 指数移動平均を適用
            int16_t smoothed_x = (int16_t)(adaptive_weight * data->prev_x + (1.0f - adaptive_weight) * x);
            int16_t smoothed_y = (int16_t)(adaptive_weight * data->prev_y + (1.0f - adaptive_weight) * y);
            
            // 非常に小さな動きはノイズと見なして抑制
            if (abs(x) <= 1 && abs(data->prev_x) <= 1) {
                smoothed_x = 0;
            }
            if (abs(y) <= 1 && abs(data->prev_y) <= 1) {
                smoothed_y = 0;
            }
            
            // 前回の値を更新
            data->prev_x = x;
            data->prev_y = y;
            
            // 平滑化された値で置き換え
            x = smoothed_x;
            y = smoothed_y;
        }
    }
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
        
        // すでにAMLが有効であり、かつ大きな動きがある場合はタイマーを延長する
        // 絶対値が一定以上あれば継続的にタイマーをリセット
        if (automouse_triggered && (movement_size > 2 || raw_movement > 2)) {
            // タイマーを停止して再開始 = タイマーのリセット
            k_timer_stop(&automouse_layer_timer);
            k_timer_start(&automouse_layer_timer, K_MSEC(CONFIG_PMW3610_AUTOMOUSE_TIMEOUT_MS), K_NO_WAIT);
        }

        // トラックボールの動きに関するデバッグ情報をログに出力
        /* if (raw_movement > 0 || movement_size > 0) {
            LOG_INF("Movement: processed=%d, raw=%d, acc=%d, raw_acc=%d, threshold=%d, raw_mult=%d", 
                    movement_size, raw_movement, g_movement_accumulator, 
                    g_raw_movement_accumulator, MOVEMENT_THRESHOLD, RAW_THRESHOLD_MULTIPLIER);
        } */

        // モード切替直後または指定時間が経過したらチェックとリセットを行う
        bool just_switched_mode = input_mode_changed;
        if (just_switched_mode || time_since_last_movement >= ACCUMULATION_TIME_MS) {
            // 通常の閾値チェックか生の動き値のチェックのどちらかで条件を満たせばレイヤーをアクティブにする
            // SCROLLモードとMOVEモード間の切り替えでもAMLタイマーを維持するため、
            // automouse_triggered が true（AMLがすでにアクティブ）でもレイヤーが違えば再アクティブ化する
            
            // モード切替直後はトラックボールの動き値の閾値を無視して強制的にAMLをアクティブにする
            bool should_activate = just_switched_mode || 
                ((g_movement_accumulator > MOVEMENT_THRESHOLD || 
                 g_raw_movement_accumulator > MOVEMENT_THRESHOLD * RAW_THRESHOLD_MULTIPLIER));
                
            if (should_activate && zmk_keymap_highest_layer_active() != AUTOMOUSE_LAYER) {
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

#ifdef CONFIG_PMW3610_SMART_ALGORITHM
    int16_t shutter =
        ((int16_t)(buf[PMW3610_SHUTTER_H_POS] & 0x01) << 8) + buf[PMW3610_SHUTTER_L_POS];
    if (data->sw_smart_flag && shutter < 45) {
        reg_write(dev, 0x32, 0x00);

        data->sw_smart_flag = false;
    }

    if (!data->sw_smart_flag && shutter > 45) {
        reg_write(dev, 0x32, 0x80);

        data->sw_smart_flag = true;
    }
#endif

#ifdef CONFIG_PMW3610_ADJUSTABLE_MOUSESPEED
    int16_t movement_size = abs(raw_x) + abs(raw_y);

    float speed_multiplier = 1.0; //速度の倍率
    if (movement_size > 60) {
        speed_multiplier = 3.0;
    }else if (movement_size > 30) {
        speed_multiplier = 1.5;
    }else if (movement_size > 5) {
        speed_multiplier = 1.0;
    }else if (movement_size > 4) {
        speed_multiplier = 0.9;
    }else if (movement_size > 3) {
        speed_multiplier = 0.7;
    }else if (movement_size > 2) {
        speed_multiplier = 0.5;
    }else if (movement_size > 1) {
        speed_multiplier = 0.1;
    }

    raw_x = raw_x * speed_multiplier;
    raw_y = raw_y * speed_multiplier;

#endif

#ifdef CONFIG_PMW3610_POLLING_RATE_125_SW
    int64_t curr_time = k_uptime_get();
    if (data->last_poll_time == 0 || curr_time - data->last_poll_time > 128) {
        data->last_poll_time = curr_time;
        data->last_x = x;
        data->last_y = y;
        
        // ポーリングレート制限時にも生の動きを累積するために保存
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
        
        data->last_poll_time = 0;
        data->last_x = 0;
        data->last_y = 0;
        data->last_raw_movement = 0;
    }
#endif

    if (x != 0 || y != 0) {
        // if (input_mode == MOVE) {
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
        // }
        if (input_mode != SCROLL) {
            input_report_rel(dev, INPUT_REL_X, x, false, K_FOREVER);
            input_report_rel(dev, INPUT_REL_Y, y, true, K_FOREVER);
        } else {
            // Calculate scroll values
            int16_t scroll_x = x;
            int16_t scroll_y = y;
            
            // 動きの大きさを計算
            int16_t movement_size = abs(x) + abs(y);
            
            // 加速ロジックはどんな場合でも適用（CONFIG_PMW3610_ADJUSTABLE_SCROLLSPEEDなくても適用）
            // 現在の時間を取得
            int64_t current_time = k_uptime_get();
            
            // 加速度計算のための時間差を計算（ミリ秒）
            int64_t time_diff = current_time - data->scroll_last_movement_time;
            
            // 加速ロジック
            if (movement_size > 0) {
                // 閾値以上の動きがある場合
                if (data->scroll_last_movement_time > 0 && time_diff < 500) {
                    // 前回の動きから500ms以内の場合、加速を徐々に増加
                    data->scroll_consecutive_movements++;
                    
                    // 高度な適応型加速度制御を実装
                    
                    // 1. 動きの速度（速さ）を計算 - スクロール体験の自然さの鍵となる要素
                    int64_t velocity_time_window = 100; // 速度計算の時間窓 (ms)
                    float movement_velocity = 0.0f;
                    
                    // 現在の動きの速度を計算（最近のN回の動きの累積を時間で割る）
                    if (time_diff > 0 && time_diff < velocity_time_window) {
                        // 単位時間あたりの動き量（速度）を計算
                        movement_velocity = (float)movement_size / (float)time_diff * 50.0f;
                    }
                    
                    // 速度の指数平滑化 - 突然の変化を抑え、徐々に追従
                    if (data->scroll_prev_movement_velocity > 0) {
                        // 前回の速度と今回の速度を混合（80%:20%の比率）
                        movement_velocity = data->scroll_prev_movement_velocity * 0.8f + movement_velocity * 0.2f;
                    }
                    data->scroll_prev_movement_velocity = movement_velocity;
                    
                    // 2. 加速度曲線の適用 - 非線形な曲線でより自然な加速を実現
                    float base_acceleration = 1.0f;
                    float max_acceleration = 4.0f;
                    
                    // 非線形シグモイド関数による加速度マッピング
                    // シグモイド関数: 1 / (1 + e^(-x)) は滑らかなS字カーブを生成
                    if (data->scroll_consecutive_movements > 5) {
                        // シグモイド関数のための入力値を計算
                        // 連続動作回数と動きの速度の両方を考慮
                        float sigmoid_input = (data->scroll_consecutive_movements - 5) / 80.0f;
                        sigmoid_input += movement_velocity / 50.0f; // 速度の影響を加える
                        
                        // -6〜6の範囲に制限（シグモイド関数の有効範囲）
                        sigmoid_input = fmaxf(-6.0f, fminf(6.0f, sigmoid_input));
                        
                        // シグモイド関数による0〜1の出力
                        float sigmoid_output = 1.0f / (1.0f + expf(-sigmoid_input));
                        
                        // 目標加速度を基本〜最大の間でマッピング
                        float target_acceleration = base_acceleration + 
                                                  (max_acceleration - base_acceleration) * sigmoid_output;
                        
                        // 現在の加速度から目標加速度へ非常に小さなステップで近づける
                        // ステップサイズは現在の速度に応じて調整（速いほど大きく、遅いほど小さく）
                        float step_size = fmaxf(0.002f, fminf(0.01f, movement_velocity / 200.0f));
                        
                        if (target_acceleration > data->scroll_acceleration) {
                            data->scroll_acceleration += step_size;
                        } else if (target_acceleration < data->scroll_acceleration) {
                            // 減速も許可 - これが高品質なスクロール体験において重要
                            data->scroll_acceleration -= step_size * 0.5f; // 減速は加速よりゆっくり
                        }
                        
                        // 加速度の範囲を制限
                        data->scroll_acceleration = fmaxf(base_acceleration, 
                                                    fminf(max_acceleration, data->scroll_acceleration));
                    } else {
                        // 最初の数回は基本速度（加速なし）
                        data->scroll_acceleration = base_acceleration;
                    }
                } else {
                    // 長時間動きがなかった場合、加速度をリセット
                    data->scroll_acceleration = 1.0f;
                    data->scroll_consecutive_movements = 1;
                }
                
                // 動きの大きさにも加速度を調整
                float size_factor = 1.0f;
                
#ifdef CONFIG_PMW3610_ADJUSTABLE_SCROLLSPEED
                // 動きの大きさに基づいた追加の調整
                if (movement_size > 90) {
                    size_factor = 2.5f; // 非常に大きな動きの場合
                } else if (movement_size > 60) {
                    size_factor = 2.0f; // 大きな動きの場合
                } else if (movement_size > 30) {
                    size_factor = 1.5f; // 中程度の動きの場合
                } else if (movement_size > 15) {
                    size_factor = 1.2f; // 小さめの動きの場合
                }
#endif
                
                // 最終的な加速率を計算（時間ベースの加速と動きの大きさベースの加速を組み合わせる）
                float final_acceleration = data->scroll_acceleration * size_factor;
                
                // スクロール値に加速を適用
                scroll_x = (int16_t)(scroll_x * final_acceleration);
                scroll_y = (int16_t)(scroll_y * final_acceleration);
                
                // LOG_INF("Scroll accel: %d movements, accel=%.2f, size=%d, final=%.2f", 
                //         data->scroll_consecutive_movements, 
                //         (double)data->scroll_acceleration,
                //         movement_size,
                //         (double)final_acceleration);
                
                // 時間と動きのサイズを更新
                data->scroll_last_movement_time = current_time;
                data->scroll_prev_movement_size = movement_size;
            } else if (time_diff > 500) {
                // 500ms以上動きがない場合、加速度を徐々に減少
                data->scroll_acceleration = fmaxf(1.0f, data->scroll_acceleration * 0.95f);
                
                if (time_diff > 1000) {
                    // 1秒以上動きがない場合、完全にリセット
                    data->scroll_acceleration = 1.0f;
                    data->scroll_consecutive_movements = 0;
                }
            }
            
            // 取りこぼし検出のための方向検出と状態更新
            int16_t current_direction_x = (scroll_x > 0) ? 1 : ((scroll_x < 0) ? -1 : 0);
            int16_t current_direction_y = (scroll_y > 0) ? 1 : ((scroll_y < 0) ? -1 : 0);
            
            // スクロール動作がある場合、実際の動き時間を更新し、方向情報を記録
            if (movement_size > 0) {
                data->scroll_last_real_movement_time = current_time;
                
                // 同じ方向への連続スクロールかチェック
                if ((current_direction_x == data->scroll_last_direction_x && current_direction_x != 0) ||
                    (current_direction_y == data->scroll_last_direction_y && current_direction_y != 0)) {
                    data->scroll_consistent_direction_count++;
                } else {
                    // 方向が変わったらリセット
                    data->scroll_consistent_direction_count = 1;
                }
                
                // 方向情報を更新
                data->scroll_last_direction_x = current_direction_x;
                data->scroll_last_direction_y = current_direction_y;
                
                // 取りこぼしカウントをリセット
                data->scroll_missed_detection_count = 0;
            } else {
                // 動きがない場合、取りこぼし検出ロジックを適用
                
                // 前回の実際の動きからの経過時間
                int64_t time_since_real_movement = current_time - data->scroll_last_real_movement_time;
                
                // 取りこぼし条件：
                // 1. 前回の動きから短時間（150ms以内）である
                // 2. 連続した同方向スクロールが一定回数（3回以上）ある
                // 3. 取りこぼしカウントが閾値（5回）以下である
                if (time_since_real_movement < 150 && 
                    data->scroll_consistent_direction_count >= 3 && 
                    data->scroll_missed_detection_count < 5) {
                    
                    // 取りこぼしと判断、前回の方向に基づいて補間値を生成
                    // 補間強度は取りこぼし回数に応じて減衰
                    float interpolation_factor = 1.0f - (data->scroll_missed_detection_count * 0.15f);
                    
                    // 補間値を計算（前回の方向 * 前回の速度の一部 * 減衰係数）
                    if (data->scroll_last_direction_x != 0) {
                        scroll_x = data->scroll_last_direction_x * 
                                  (data->scroll_prev_movement_velocity * 0.3f) * 
                                  interpolation_factor;
                    }
                    
                    if (data->scroll_last_direction_y != 0) {
                        scroll_y = data->scroll_last_direction_y * 
                                  (data->scroll_prev_movement_velocity * 0.3f) * 
                                  interpolation_factor;
                    }
                    
                    // 取りこぼしカウントを増加
                    data->scroll_missed_detection_count++;
                    
                    // ログデータ（デバッグ時のみ有効にする）
                    // LOG_DBG("Interpolated scroll: x=%d, y=%d, factor=%.2f", 
                    //       (int)scroll_x, (int)scroll_y, (double)interpolation_factor);
                } else if (time_since_real_movement >= 150) {
                    // 長時間動きがなければ本当に停止したと判断
                    data->scroll_consistent_direction_count = 0;
                    data->scroll_missed_detection_count = 0;
                }
            }
            
            // Accumulate scroll delta
            data->scroll_delta_x += scroll_x;
            data->scroll_delta_y += scroll_y;
            
            // Generate scroll events when threshold is exceeded
            if (abs(data->scroll_delta_y) > CONFIG_PMW3610_SCROLL_TICK) {
                // Calculate number of scroll events to generate
                int16_t scroll_events = abs(data->scroll_delta_y) / CONFIG_PMW3610_SCROLL_TICK;
                int16_t scroll_direction = data->scroll_delta_y > 0 ? PMW3610_SCROLL_Y_NEGATIVE : PMW3610_SCROLL_Y_POSITIVE;
                
                // Send multiple scroll events if needed
                for (int i = 0; i < scroll_events; i++) {
                    input_report_rel(dev, INPUT_REL_WHEEL, scroll_direction, true, K_FOREVER);
                }
                
                // Keep remainder for next time
                data->scroll_delta_y %= CONFIG_PMW3610_SCROLL_TICK;
                data->scroll_delta_x = 0; // Reset horizontal scrolling after vertical scroll
            } else if (abs(data->scroll_delta_x) > CONFIG_PMW3610_SCROLL_TICK) {
                // Calculate number of scroll events to generate
                int16_t scroll_events = abs(data->scroll_delta_x) / CONFIG_PMW3610_SCROLL_TICK;
                int16_t scroll_direction = data->scroll_delta_x > 0 ? PMW3610_SCROLL_X_NEGATIVE : PMW3610_SCROLL_X_POSITIVE;
                
                // Send multiple scroll events if needed
                for (int i = 0; i < scroll_events; i++) {
                    input_report_rel(dev, INPUT_REL_HWHEEL, scroll_direction, true, K_FOREVER);
                }
                
                // Keep remainder for next time
                data->scroll_delta_x %= CONFIG_PMW3610_SCROLL_TICK;
                data->scroll_delta_y = 0; // Reset vertical scrolling after horizontal scroll
            }
        }
    }

    return err;
}

static void pmw3610_gpio_callback(const struct device *gpiob, struct gpio_callback *cb,
                                  uint32_t pins) {
    struct pixart_data *data = CONTAINER_OF(cb, struct pixart_data, irq_gpio_cb);
    const struct device *dev = data->dev;

    set_interrupt(dev, false);

    // submit the real handler work
    k_work_submit(&data->trigger_work);
}

static void pmw3610_work_callback(struct k_work *work) {
    struct pixart_data *data = CONTAINER_OF(work, struct pixart_data, trigger_work);
    const struct device *dev = data->dev;

    pmw3610_report_data(dev);
    set_interrupt(dev, true);
}

static int pmw3610_init_irq(const struct device *dev) {
    LOG_INF("Configure irq...");

    int err;
    struct pixart_data *data = dev->data;
    const struct pixart_config *config = dev->config;

    // check readiness of irq gpio pin
    if (!device_is_ready(config->irq_gpio.port)) {
        LOG_ERR("IRQ GPIO device not ready");
        return -ENODEV;
    }

    // init the irq pin
    err = gpio_pin_configure_dt(&config->irq_gpio, GPIO_INPUT);
    if (err) {
        LOG_ERR("Cannot configure IRQ GPIO");
        return err;
    }

    // setup and add the irq callback associated
    gpio_init_callback(&data->irq_gpio_cb, pmw3610_gpio_callback, BIT(config->irq_gpio.pin));

    err = gpio_add_callback(config->irq_gpio.port, &data->irq_gpio_cb);
    if (err) {
        LOG_ERR("Cannot add IRQ GPIO callback");
    }

    LOG_INF("Configure irq done");

    return err;
}

static int pmw3610_init(const struct device *dev) {
    LOG_INF("Start initializing...");

    struct pixart_data *data = dev->data;
    const struct pixart_config *config = dev->config;
    int err;

    // init device pointer
    data->dev = dev;

    // init smart algorithm flag;
    data->sw_smart_flag = false;
    
    // スクロール加速変数を初期化
    data->scroll_last_movement_time = 0;
    data->scroll_acceleration = 1.0f;
    data->scroll_consecutive_movements = 0;
    data->scroll_prev_movement_size = 0;
    data->scroll_prev_movement_velocity = 0.0f;
    
    // スクロール補間変数を初期化
    data->scroll_last_direction_x = 0;
    data->scroll_last_direction_y = 0;
    data->scroll_missed_detection_count = 0;
    data->scroll_consistent_direction_count = 0;
    data->scroll_last_real_movement_time = 0;

#ifdef CONFIG_PMW3610_SMOOTHING_FILTER
    // 平滑化フィルター用の変数を初期化
    data->prev_x = 0;
    data->prev_y = 0;
    data->current_smoothing_weight = CONFIG_PMW3610_SMOOTHING_WEIGHT;
#endif

#ifdef CONFIG_PMW3610_PROFILE_SWITCHING
    // プロファイル状態を初期化
    data->precision_profile_active = false;
    data->speed_profile_active = false;
    data->backup_cpi = CONFIG_PMW3610_CPI;
#endif

    // init trigger handler work
    k_work_init(&data->trigger_work, pmw3610_work_callback);

    // check readiness of cs gpio pin and init it to inactive
    if (!device_is_ready(config->cs_gpio.port)) {
        LOG_ERR("SPI CS device not ready");
        return -ENODEV;
    }

    err = gpio_pin_configure_dt(&config->cs_gpio, GPIO_OUTPUT_INACTIVE);
    if (err) {
        LOG_ERR("Cannot configure SPI CS GPIO");
        return err;
    }

    // init irq routine
    err = pmw3610_init_irq(dev);
    if (err) {
        return err;
    }

    // Setup delayable and non-blocking init jobs, including following steps:
    // 1. power reset
    // 2. upload initial settings
    // 3. other configs like cpi, downshift time, sample time etc.
    // The sensor is ready to work (i.e., data->ready=true after the above steps are finished)
    k_work_init_delayable(&data->init_work, pmw3610_async_init);

    k_work_schedule(&data->init_work, K_MSEC(async_init_delay[data->async_init_step]));

    return err;
}

#define PMW3610_DEFINE(n)                                                                          \
    static struct pixart_data data##n;                                                             \
    static int32_t scroll_layers##n[] = DT_PROP(DT_DRV_INST(n), scroll_layers);                    \
    static int32_t snipe_layers##n[] = DT_PROP(DT_DRV_INST(n), snipe_layers);                      \
    static const struct pixart_config config##n = {                                                \
        .irq_gpio = GPIO_DT_SPEC_INST_GET(n, irq_gpios),                                           \
        .bus =                                                                                     \
            {                                                                                      \
                .bus = DEVICE_DT_GET(DT_INST_BUS(n)),                                              \
                .config =                                                                          \
                    {                                                                              \
                        .frequency = DT_INST_PROP(n, spi_max_frequency),                           \
                        .operation =                                                               \
                            SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_MODE_CPOL | SPI_MODE_CPHA,    \
                        .slave = DT_INST_REG_ADDR(n),                                              \
                    },                                                                             \
            },                                                                                     \
        .cs_gpio = SPI_CS_GPIOS_DT_SPEC_GET(DT_DRV_INST(n)),                                       \
        .scroll_layers = scroll_layers##n,                                                         \
        .scroll_layers_len = DT_PROP_LEN(DT_DRV_INST(n), scroll_layers),                           \
        .snipe_layers = snipe_layers##n,                                                           \
        .snipe_layers_len = DT_PROP_LEN(DT_DRV_INST(n), snipe_layers),                             \
    };                                                                                             \
                                                                                                   \
    DEVICE_DT_INST_DEFINE(n, pmw3610_init, NULL, &data##n, &config##n, POST_KERNEL,                \
                          CONFIG_SENSOR_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(PMW3610_DEFINE)

#ifdef CONFIG_PMW3610_PROFILE_SWITCHING
/**
 * @brief 精密作業プロファイルを有効/無効化する関数
 * 
 * @param dev デバイス
 * @param enable 有効化するかどうか
 * @return int 成功時は0、エラー時は負の値
 */
static int set_precision_profile(const struct device *dev, bool enable) {
    struct pixart_data *data = dev->data;
    int err = 0;

    if (enable && !data->precision_profile_active) {
        // 現在のCPI値をバックアップ
        if (!data->speed_profile_active) {
            data->backup_cpi = data->curr_cpi;
        }

        // 精密プロファイル用のCPI値に設定
        err = set_cpi(dev, CONFIG_PMW3610_PRECISION_PROFILE_CPI);
        if (err) {
            LOG_ERR("Failed to set precision profile CPI");
            return err;
        }

#ifdef CONFIG_PMW3610_SMOOTHING_FILTER
        // 精密プロファイル用のスムージング値に変更
        data->current_smoothing_weight = CONFIG_PMW3610_PRECISION_PROFILE_SMOOTHING;
#endif

        data->precision_profile_active = true;
        data->speed_profile_active = false;
        LOG_DBG("Precision profile activated");
    } else if (!enable && data->precision_profile_active) {
        // バックアップしたCPI値に戻す
        err = set_cpi(dev, data->backup_cpi);
        if (err) {
            LOG_ERR("Failed to restore CPI setting");
            return err;
        }

#ifdef CONFIG_PMW3610_SMOOTHING_FILTER
        // 通常のスムージング値に戻す
        data->current_smoothing_weight = CONFIG_PMW3610_SMOOTHING_WEIGHT;
#endif

        data->precision_profile_active = false;
        LOG_DBG("Precision profile deactivated");
    }

    return err;
}

/**
 * @brief 高速移動プロファイルを有効/無効化する関数
 * 
 * @param dev デバイス
 * @param enable 有効化するかどうか
 * @return int 成功時は0、エラー時は負の値
 */
static int set_speed_profile(const struct device *dev, bool enable) {
    struct pixart_data *data = dev->data;
    int err = 0;

    if (enable && !data->speed_profile_active) {
        // 現在のCPI値をバックアップ
        if (!data->precision_profile_active) {
            data->backup_cpi = data->curr_cpi;
        }

        // 高速プロファイル用のCPI値に設定
        err = set_cpi(dev, CONFIG_PMW3610_SPEED_PROFILE_CPI);
        if (err) {
            LOG_ERR("Failed to set speed profile CPI");
            return err;
        }

#ifdef CONFIG_PMW3610_SMOOTHING_FILTER
        // 高速プロファイル用のスムージング値に変更
        data->current_smoothing_weight = CONFIG_PMW3610_SPEED_PROFILE_SMOOTHING;
#endif

        data->speed_profile_active = true;
        data->precision_profile_active = false;
        LOG_DBG("Speed profile activated");
    } else if (!enable && data->speed_profile_active) {
        // バックアップしたCPI値に戻す
        err = set_cpi(dev, data->backup_cpi);
        if (err) {
            LOG_ERR("Failed to restore CPI setting");
            return err;
        }

#ifdef CONFIG_PMW3610_SMOOTHING_FILTER
        // 通常のスムージング値に戻す
        data->current_smoothing_weight = CONFIG_PMW3610_SMOOTHING_WEIGHT;
#endif

        data->speed_profile_active = false;
        LOG_DBG("Speed profile deactivated");
    }

    return err;
}

/**
 * @brief 精密作業プロファイルを有効/無効化する公開API関数
 * 
 * ZMKのビヘイビアからこの関数を呼び出すことで、キーを押している間だけ精密プロファイルに切り替えられる
 * 
 * @param enable 有効化するかどうか
 * @return int 成功時は0、エラー時は負の値
 */
int pmw3610_activate_precision_profile(bool enable) {
    // PMW3610センサーのデバイスを取得
    const struct device *dev = DEVICE_DT_GET(DT_INST(0, pixart_pmw3610));
    
    if (!device_is_ready(dev)) {
        return -ENODEV;
    }
    
    return set_precision_profile(dev, enable);
}

/**
 * @brief 高速移動プロファイルを有効/無効化する公開API関数
 * 
 * ZMKのビヘイビアからこの関数を呼び出すことで、キーを押している間だけ高速プロファイルに切り替えられる
 * 
 * @param enable 有効化するかどうか
 * @return int 成功時は0、エラー時は負の値
 */
int pmw3610_activate_speed_profile(bool enable) {
    // PMW3610センサーのデバイスを取得
    const struct device *dev = DEVICE_DT_GET(DT_INST(0, pixart_pmw3610));
    
    if (!device_is_ready(dev)) {
        return -ENODEV;
    }
    
    return set_speed_profile(dev, enable);
}
#endif
