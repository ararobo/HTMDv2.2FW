/**
 * @file app.hpp
 * @author Gento Aiba (aiba-gento)
 * @brief STM32のメイン処理（Arduinoの関数setup,loopを提供）
 * @version 0.2.0
 * @date 2026-03-08
 *
 * @copyright Copyright (c) 2026 ararobo
 * SPDX-License-Identifier: Apache-2.0
 *
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

void setup();
void loop();
#ifdef __cplusplus
}
#endif