#pragma once
#include <stdint.h>
#include <stdbool.h>

bool bmp280_begin(void);
float bmp280_readPressure(void);
float bmp280_collect(void);
bool bmp280_trigger(void);