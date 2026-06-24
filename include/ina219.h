#pragma once
#include <stdint.h>
#include <stdbool.h>

bool ina219_init(void);
void ina219_configure(void);
void ina219_startSingle(void);
bool ina219_conversionReady(void);
float ina219_getBusVoltage(void);
float ina219_getCurrent(void);
