#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// Include C headers inside extern "C" to prevent name mangling if they don't have it
#include "can.h"
#include "gpio.h"
#include "main.h"
#include "tim.h"
#include "usart.h"

void setup();
void loop();
#ifdef __cplusplus
}
#endif