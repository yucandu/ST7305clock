#include "ds3231.h"
#include "i2c_driver.h"

#define DS3231_ADDR  0x68

static const uint8_t dim[] = {31,28,31,30,31,30,31,31,30,31,30,31};

static bool is_leap(uint16_t y) {
    return (y % 4 == 0 && y % 100 != 0) || y % 400 == 0;
}

DateTime::DateTime(uint32_t t) {
    _second = t % 60; t /= 60;
    _minute = t % 60; t /= 60;
    _hour   = t % 24; t /= 24;

    uint16_t y = 1970;
    while (1) {
        uint16_t days = is_leap(y) ? 366 : 365;
        if (t < days) break;
        t -= days; y++;
    }
    _year = y;

    uint8_t m;
    for (m = 0; m < 12; m++) {
        uint8_t d = dim[m];
        if (m == 1 && is_leap(y)) d++;
        if (t < d) break;
        t -= d;
    }
    _month = m + 1;
    _day = t + 1;
}

DateTime::DateTime(uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second) {
    _year = year; _month = month; _day = day;
    _hour = hour; _minute = minute; _second = second;
}

uint32_t DateTime::unixtime(void) const {
    uint32_t t = 0;
    for (uint16_t y = 1970; y < _year; y++)
        t += is_leap(y) ? 366 : 365;
    for (uint8_t m = 1; m < _month; m++) {
        t += dim[m - 1];
        if (m == 2 && is_leap(_year)) t++;
    }
    t += _day - 1;
    t = t * 24 + _hour;
    t = t * 60 + _minute;
    t = t * 60 + _second;
    return t;
}

static uint8_t bcd2bin(uint8_t b) { return (b >> 4) * 10 + (b & 0x0F); }
static uint8_t bin2bcd(uint8_t b) { return ((b / 10) << 4) | (b % 10); }

bool ds3231_begin(void) {
    uint8_t reg = 0x00;
    uint8_t d;
    if (!i2c_writeread(DS3231_ADDR, &reg, 1, &d, 1)) return false;
    return true;
}

DateTime ds3231_now(void) {
    uint8_t reg = 0x00;
    uint8_t buf[7];
    /*if (!*/i2c_writeread(DS3231_ADDR, &reg, 1, buf, 7);//)
        //return DateTime(0);

    uint8_t sec  = bcd2bin(buf[0] & 0x7F);
    uint8_t min  = bcd2bin(buf[1]);
    uint8_t hr   = bcd2bin(buf[2]);
    uint8_t dom  = bcd2bin(buf[4]);
    uint8_t mon  = bcd2bin(buf[5] & 0x7F);
    uint16_t yr  = bcd2bin(buf[6]) + 2000U;

    return DateTime(yr, mon, dom, hr, min, sec);
}

void ds3231_adjust(const DateTime &dt) {
    uint8_t buf[8];
    buf[0] = 0x00;
    buf[1] = bin2bcd(dt._second);
    buf[2] = bin2bcd(dt._minute);
    buf[3] = bin2bcd(dt._hour);
    buf[4] = 0;
    buf[5] = bin2bcd(dt._day);
    buf[6] = bin2bcd(dt._month);
    buf[7] = bin2bcd(dt._year % 100);
    i2c_write(DS3231_ADDR, buf, 8);
}

bool ds3231_lostPower(void) {
    uint8_t reg = 0x0F;
    uint8_t status;
    if (!i2c_writeread(DS3231_ADDR, &reg, 1, &status, 1)) return true;
    return (status & 0x80) != 0;
}
