/*
 * Copyright (c) 2023 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <zephyr/device.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifdef CONFIG_PMW3610_PROFILE_SWITCHING
/**
 * @brief 精密作業プロファイルを有効/無効化する
 * 
 * @param enable 有効化するかどうか
 * @return int 成功時は0、エラー時は負の値
 */
int pmw3610_activate_precision_profile(bool enable);

/**
 * @brief 高速移動プロファイルを有効/無効化する
 * 
 * @param enable 有効化するかどうか
 * @return int 成功時は0、エラー時は負の値
 */
int pmw3610_activate_speed_profile(bool enable);
#endif

#ifdef __cplusplus
}
#endif 