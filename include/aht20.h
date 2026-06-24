#pragma once
#include <stdint.h>
#include <stdbool.h>

bool aht20_begin(void);
bool aht20_read(float *temperature, float *humidity);
