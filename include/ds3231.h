#pragma once
#include <stdint.h>
#include <stdbool.h>

class DateTime {
public:
    uint16_t _year;
    uint8_t _month;
    uint8_t _day;
    uint8_t _hour;
    uint8_t _minute;
    uint8_t _second;

    DateTime(uint32_t t);
    DateTime(uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second);
    uint32_t unixtime(void) const;
    uint16_t year(void) const { return _year; }
    uint8_t month(void) const { return _month; }
    uint8_t day(void) const { return _day; }
    uint8_t hour(void) const { return _hour; }
    uint8_t minute(void) const { return _minute; }
    uint8_t second(void) const { return _second; }
};

bool ds3231_begin(void);
DateTime ds3231_now(void);
void ds3231_adjust(const DateTime &dt);
bool ds3231_lostPower(void);
