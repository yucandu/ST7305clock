#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

void i2c_init(void);
void i2c_deinit(void);
bool i2c_write(uint8_t addr, const uint8_t *data, size_t len);
bool i2c_read(uint8_t addr, uint8_t *data, size_t len);
bool i2c_writeread(uint8_t addr, const uint8_t *wdata, size_t wlen, uint8_t *rdata, size_t rlen);
