#include "i2c_driver.h"
#include <Wire.h>

#define I2C1_SCL  PB6
#define I2C1_SDA  PB7

void i2c_init(void) {
    Wire.setSCL(I2C1_SCL);
    Wire.setSDA(I2C1_SDA);
    Wire.begin();
    Wire.setClock(100000);
}

void i2c_deinit(void) {
    Wire.end();
}

bool i2c_write(uint8_t addr, const uint8_t *data, size_t len) {
    Wire.beginTransmission(addr);
    size_t sent = Wire.write(data, len);
    if (sent != len) return false;
    return Wire.endTransmission(true) == 0;
}

bool i2c_read(uint8_t addr, uint8_t *data, size_t len) {
    if (Wire.requestFrom(addr, (uint8_t)len, true) != len) return false;
    for (size_t i = 0; i < len; i++) {
        data[i] = Wire.read();
    }
    return true;
}

bool i2c_writeread(uint8_t addr, const uint8_t *wdata, size_t wlen, uint8_t *rdata, size_t rlen) {
    Wire.beginTransmission(addr);
    if (Wire.write(wdata, wlen) != wlen) return false;
    if (Wire.endTransmission(false) != 0) return false;
    if (rlen && Wire.requestFrom(addr, (uint8_t)rlen, true) != rlen) return false;
    for (size_t i = 0; i < rlen; i++) {
        rdata[i] = Wire.read();
    }
    return true;
}
