#include "ina219.h"
#include "i2c_driver.h"

#define INA219_ADDR      0x40
#define REG_CONFIG       0x00
#define REG_SHUNT        0x01
#define REG_BUS          0x02

#define CONFIG_VALUE     0x07FB

static bool read_reg16(uint8_t reg, uint16_t *val) {
    uint8_t buf[2];
    if (!i2c_writeread(INA219_ADDR, &reg, 1, buf, 2)) return false;
    *val = ((uint16_t)buf[0] << 8) | buf[1];
    return true;
}

static bool write_reg16(uint8_t reg, uint16_t val) {
    uint8_t buf[3] = {reg, (uint8_t)(val >> 8), (uint8_t)(val & 0xFF)};
    return i2c_write(INA219_ADDR, buf, 3);
}

bool ina219_init(void) {
    uint16_t val;
    if (!read_reg16(REG_CONFIG, &val)) return false;
    return true;
}

void ina219_configure(void) {
    write_reg16(REG_CONFIG, CONFIG_VALUE);
}

void ina219_startSingle(void) {
    write_reg16(REG_CONFIG, CONFIG_VALUE);
}

bool ina219_conversionReady(void) {
    uint16_t bus;
    if (!read_reg16(REG_BUS, &bus)) return false;
    return (bus & 0x02) != 0;
}

float ina219_getBusVoltage(void) {
    uint16_t bus;
    if (!read_reg16(REG_BUS, &bus)) return 0.0f;
    return (float)(bus >> 3) * 0.004f;
}

float ina219_getCurrent(void) {
    int16_t shunt;
    uint8_t reg = REG_SHUNT;
    uint8_t buf[2];
    if (!i2c_writeread(INA219_ADDR, &reg, 1, buf, 2)) return 0.0f;
    shunt = (int16_t)((uint16_t)buf[0] << 8 | buf[1]);

    return (float)shunt * 0.01f;
}
