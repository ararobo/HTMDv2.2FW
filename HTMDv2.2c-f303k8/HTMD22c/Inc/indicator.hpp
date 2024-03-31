#pragma once
#include <stdint.h>
#include <stdbool.h>

void indicateError(bool state);
void indicateStanby(bool state);
void indicateReady(bool state);
void indicateBusy(bool state);

void setIndicatorColor(bool red_1, bool red_2, bool blue, bool green);