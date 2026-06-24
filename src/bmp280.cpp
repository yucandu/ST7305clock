#include "bmp280.h"
#include "i2c_driver.h"
#include <Arduino.h>

static uint8_t bmp_addr = 0;

typedef struct {
    uint16_t dig_T1;
    int16_t dig_T2, dig_T3;
    uint16_t dig_P1;
    int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
    int32_t t_fine;
} bmp280_cal_t;

static bmp280_cal_t cal;

bool bmp280_begin(void) {
    uint8_t addrs[2] = {0x77, 0x76};
    uint8_t id;
    uint8_t reg = 0xD0;
    bool found = false;

    for (int i = 0; i < 2; i++) {
        if (i2c_writeread(addrs[i], &reg, 1, &id, 1) && id == 0x58) {
            bmp_addr = addrs[i];
            found = true;
            break;
        }
    }
    if (!found) return false;

    reg = 0x88;
    uint8_t buf[24];
    if (!i2c_writeread(bmp_addr, &reg, 1, buf, 24)) return false;

    cal.dig_T1 = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
    cal.dig_T2 = (int16_t)((uint16_t)buf[2] | ((uint16_t)buf[3] << 8));
    cal.dig_T3 = (int16_t)((uint16_t)buf[4] | ((uint16_t)buf[5] << 8));
    cal.dig_P1 = (uint16_t)buf[6] | ((uint16_t)buf[7] << 8);
    cal.dig_P2 = (int16_t)((uint16_t)buf[8] | ((uint16_t)buf[9] << 8));
    cal.dig_P3 = (int16_t)((uint16_t)buf[10] | ((uint16_t)buf[11] << 8));
    cal.dig_P4 = (int16_t)((uint16_t)buf[12] | ((uint16_t)buf[13] << 8));
    cal.dig_P5 = (int16_t)((uint16_t)buf[14] | ((uint16_t)buf[15] << 8));
    cal.dig_P6 = (int16_t)((uint16_t)buf[16] | ((uint16_t)buf[17] << 8));
    cal.dig_P7 = (int16_t)((uint16_t)buf[18] | ((uint16_t)buf[19] << 8));
    cal.dig_P8 = (int16_t)((uint16_t)buf[20] | ((uint16_t)buf[21] << 8));
    cal.dig_P9 = (int16_t)((uint16_t)buf[22] | ((uint16_t)buf[23] << 8));

    reg = 0xF5;
    uint8_t config = (5 << 5) | (4 << 2);
    uint8_t cfg[2] = {reg, config};
    i2c_write(bmp_addr, cfg, 2);

    return true;
}

static bool try_addr(uint8_t addr) {
    uint8_t reg = 0xF4;
    uint8_t buf[2] = {reg, (2 << 5) | (5 << 2) | 1};
    return i2c_write(addr, buf, 2);
}

float bmp280_readPressure(void) {
    try_addr(bmp_addr);
    delay(80);

    uint8_t reg = 0xFA;
    uint8_t tbuf[3];
    if (!i2c_writeread(bmp_addr, &reg, 1, tbuf, 3)) return 0;

    int32_t adc_T = ((int32_t)tbuf[0] << 12) | ((int32_t)tbuf[1] << 4) | (tbuf[2] >> 4);

    float var1, var2;
    var1 = ((float)adc_T / 16384.0f - (float)cal.dig_T1 / 1024.0f) * (float)cal.dig_T2;
    var2 = (((float)adc_T / 131072.0f - (float)cal.dig_T1 / 8192.0f) *
            ((float)adc_T / 131072.0f - (float)cal.dig_T1 / 8192.0f)) * (float)cal.dig_T3;
    cal.t_fine = (int32_t)(var1 + var2);

    reg = 0xF7;
    uint8_t pbuf[3];
    if (!i2c_writeread(bmp_addr, &reg, 1, pbuf, 3)) return 0;

    int32_t adc_P = ((int32_t)pbuf[0] << 12) | ((int32_t)pbuf[1] << 4) | (pbuf[2] >> 4);
    if (adc_P == 0x800000 || adc_P == 0) return 0;

    var1 = ((float)cal.t_fine / 2.0f) - 64000.0f;
    var2 = var1 * var1 * (float)cal.dig_P6 / 32768.0f;
    var2 = var2 + var1 * (float)cal.dig_P5 * 2.0f;
    var2 = var2 / 4.0f + (float)cal.dig_P4 * 65536.0f;
    var1 = ((float)cal.dig_P3 * var1 * var1 / 524288.0f + (float)cal.dig_P2 * var1) / 524288.0f;
    var1 = (1.0f + var1 / 32768.0f) * (float)cal.dig_P1;
    if (var1 < 1.0f) return 0;

    float p = 1048576.0f - (float)adc_P;
    p = (p - var2 / 4096.0f) * 6250.0f / var1;
    return p;
}
