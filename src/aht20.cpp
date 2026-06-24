#include "aht20.h"
#include "i2c_driver.h"
#include <Arduino.h>

#define AHT20_ADDR  0x38

bool aht20_trigger(void) {
    uint8_t cmd[3] = {0xAC, 0x33, 0x00};
    return i2c_write(AHT20_ADDR, cmd, 3);
}

bool aht20_collect(float *temperature, float *humidity) {
    // Note: No delay here! We already waited in the main loop.
    uint8_t buf[6];
    if (!i2c_read(AHT20_ADDR, buf, 6)) return false;

    if (buf[0] & 0x80) return false;

    uint32_t hum_code = ((uint32_t)buf[1] << 12) | ((uint32_t)buf[2] << 4) | (buf[3] >> 4);
    uint32_t temp_code = (((uint32_t)buf[3] & 0x0F) << 16) | ((uint32_t)buf[4] << 8) | buf[5];

    if (humidity) *humidity = (float)hum_code * 100.0f / 1048576.0f;
    if (temperature) *temperature = (float)temp_code * 200.0f / 1048576.0f - 50.0f;

    return true;
}

bool aht20_begin(void) {
    uint8_t cmd[3] = {0xBE, 0x08, 0x00};
    if (!i2c_write(AHT20_ADDR, cmd, 3)) return false;

    delay(10);

    uint8_t status;
    if (!i2c_read(AHT20_ADDR, &status, 1)) return false;

    return (status & 0x08) != 0;
}

bool aht20_read(float *temperature, float *humidity) {
    uint8_t cmd[3] = {0xAC, 0x33, 0x00};
    if (!i2c_write(AHT20_ADDR, cmd, 3)) return false;

    delay(80);

    uint8_t buf[6];
    if (!i2c_read(AHT20_ADDR, buf, 6)) return false;

    if (buf[0] & 0x80) return false;

    uint32_t hum_code = ((uint32_t)buf[1] << 12) | ((uint32_t)buf[2] << 4) | (buf[3] >> 4);
    uint32_t temp_code = (((uint32_t)buf[3] & 0x0F) << 16) | ((uint32_t)buf[4] << 8) | buf[5];

    if (humidity) *humidity = (float)hum_code * 100.0f / 1048576.0f;
    if (temperature) *temperature = (float)temp_code * 200.0f / 1048576.0f - 50.0f;

    return true;
}
