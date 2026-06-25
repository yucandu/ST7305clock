//STM32L412K8T6 

#include <Arduino.h>
#include <U8g2lib.h>
#include <SPI.h>
#include "STM32LowPower.h"
#include "STM32RTC.h"
#include "i2c_driver.h"
#include "ds3231.h"
#include "ina219.h"
#include "aht20.h"
#include "bmp280.h"
#include "monochromebg_rle.h"

extern "C" float expf(float);

STM32RTC& stmRtc = STM32RTC::getInstance();

extern "C" void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
    RCC_OscInitStruct.MSIState = RCC_MSI_ON;
    RCC_OscInitStruct.MSICalibrationValue = 0;
    RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_9;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);
}

#define PWR_SENSORS     PB0
#define PIN_LCD_SCLK    PA5
#define PIN_LCD_MOSI    PA7
#define PIN_LCD_CS      PA4
#define PIN_LCD_DC      PA0
#define PIN_LCD_TE      PA1
#define PIN_LCD_RST     PA2 

#define BTN_LEFT PA3
#define BTN_UP PA6
#define BTN_DOWN PB3
#define BTN_RIGHT PB4

#define SERIAL_BAUD     115200

enum AppState { STATE_MAIN, STATE_MENU, STATE_CHART, STATE_SET_TIME };
AppState currentState = STATE_MAIN;
int8_t menuSelection = 0;
int8_t menuScrollOffset = 0;
const int8_t MENU_VISIBLE = 7;
const char* menuItems[9] = {"Temperature", "Rel Humidity", "Abs Humidity", "Pressure", "Voltage", "Current", "Set Time", "Reset Data", "Reset System"};
const int8_t NUM_MENU_ITEMS = 9;

bool editingMinutes = false;
int8_t tempHour = 0;
int8_t tempMinute = 0;

#define MAX_SAMPLES 1440
// 1. Temperature: -327.68 to +327.67 C (Multiply by 100)
int16_t histTemp[MAX_SAMPLES];      // 2 bytes per sample

// 2. Relative Humidity: 0.00 to 100.00% 
uint16_t histRH[MAX_SAMPLES];        // 2 bytes per sample

// 3. Absolute Humidity: 0.00 to 655.35 g/m3 (Multiply by 100)
uint16_t histAbsHum[MAX_SAMPLES];   // 2 bytes per sample

// 4. Pressure: encoded with bit 15 = hundreds is 10 (vs 9), bits 0-14 = remainder*100 (0.00-99.99)
//    Range: 900.00-1099.99 hPa at 0.01 hPa resolution
uint16_t histPress[MAX_SAMPLES];    // 2 bytes per sample

// 5. Voltage: 0.000V to 65.535V (Multiply by 1000)
uint16_t histVolt[MAX_SAMPLES];     // 2 bytes per sample

// 6. Current: ±32,767 uA 
// (If your current exceeds 32mA, divide by 10 here to store up to 327mA)
int16_t histCur[MAX_SAMPLES];       // 2 bytes per sample

uint16_t histHead = 0;
uint16_t histCount = 0;

uint16_t voltHead = 0;
uint16_t voltCount = 0;
uint8_t voltSampleCounter = 0;

int chartCursor = -1;

void readAllSensors(uint32_t &now);
U8G2_ST7305_168X384_F_4W_HW_SPI u8g2(U8G2_R3, PIN_LCD_CS, PIN_LCD_DC, PIN_LCD_RST);

void drawRleBackground(int16_t x, int16_t y, int16_t w_pixels, int16_t h_pixels, const uint8_t *compData) {
    uint16_t bytesPerRow = w_pixels / 8; 
    uint16_t totalBytes = bytesPerRow * h_pixels;
    uint16_t drawnBytes = 0;
    uint16_t dataIdx = 0;

    uint8_t rowBuf[48]; 
    uint8_t rowIdx = 0;
    int16_t currentY = y;

    u8g2.setDrawColor(1);
    u8g2.setBitmapMode(1);

    while (drawnBytes < totalBytes) {
        uint8_t count = pgm_read_byte(&compData[dataIdx++]);
        uint8_t val = pgm_read_byte(&compData[dataIdx++]);

        for (uint8_t i = 0; i < count; i++) {
            rowBuf[rowIdx++] = val;
            drawnBytes++;

            if (rowIdx >= bytesPerRow) {
                u8g2.drawBitmap(x, currentY, bytesPerRow, 1, rowBuf);
                currentY++;
                rowIdx = 0;
                if (currentY >= y + h_pixels) return;
            }
        }
    }
}

volatile bool wakeFlag = false;
static uint32_t atime = 1000;

float gTemperature = 0.0f;
float gHumidity = 0.0f;
float gPressure = 1013.25f;
float gAbsHumidity = 0.0f;
float gBatteryVoltage = 0.0f;
int32_t gBatteryCurrent_uA = 0;

uint32_t bootEpoch = 0;
uint32_t lastSensorRead = 0;
uint32_t gLastSyncedEpoch = 0;
uint32_t gLastSyncStmEpoch = 0;

static uint8_t convertedBuf[8064];

static const uint8_t map1[16] = {
    0x00,0x02,0x08,0x0A,0x20,0x22,0x28,0x2A,
    0x80,0x82,0x88,0x8A,0xA0,0xA2,0xA8,0xAA
};
static const uint8_t map2[16] = {
    0x00,0x01,0x04,0x05,0x10,0x11,0x14,0x15,
    0x40,0x41,0x44,0x45,0x50,0x51,0x54,0x55
};


void convertBuffer(uint8_t startTileRow = 0, uint8_t endTileRow = 47) {
    uint8_t *src = u8g2.getBufferPtr();
    uint8_t *dst = convertedBuf;

    // dst is sequential output, so it needs fast-forwarding to the start row.
    // src does NOT — tileBase below already computes an absolute offset from
    // the (unshifted) src base using the full tileRow value, so pre-shifting
    // src here would double-count the offset and read from the wrong rows.
    dst += (startTileRow * 168);

    const uint8_t tileWidth = 21;
    const uint8_t tileHeight = 48;
    for (uint8_t tileRow = startTileRow; tileRow <= endTileRow; tileRow++) {
        uint8_t *tileBase = src + tileRow * tileWidth * 8;
        for (uint8_t i = 0; i < 4; i++) {
            uint8_t *ptr = tileBase + tileWidth * i * 2;
            for (uint8_t c = 0; c < 7; c++) {
                uint8_t *p1 = ptr;
                uint8_t *p2 = ptr + tileWidth;
                *dst++ = map1[p1[0]>>4]  | map2[p2[0]>>4];
                *dst++ = map1[p1[0]&0xf] | map2[p2[0]&0xf];
                *dst++ = map1[p1[1]>>4]  | map2[p2[1]>>4];
                *dst++ = map1[p1[1]&0xf] | map2[p2[1]&0xf];
                *dst++ = map1[p1[2]>>4]  | map2[p2[2]>>4];
                *dst++ = map1[p1[2]&0xf] | map2[p2[2]&0xf];
                ptr += 3;
            }
        }
    }
}

void setLcdWindow(uint8_t casetStart, uint8_t casetEnd, uint8_t rasetStart, uint8_t rasetEnd) {
    digitalWrite(PIN_LCD_DC, LOW);  SPI.transfer(0x2A);
    digitalWrite(PIN_LCD_DC, HIGH); SPI.transfer(casetStart); SPI.transfer(casetEnd);
    digitalWrite(PIN_LCD_DC, LOW);  SPI.transfer(0x2B);
    digitalWrite(PIN_LCD_DC, HIGH); SPI.transfer(rasetStart); SPI.transfer(rasetEnd);
    digitalWrite(PIN_LCD_DC, LOW);  SPI.transfer(0x2C);
    digitalWrite(PIN_LCD_DC, HIGH);
}

void prepDisplay() {
    u8g2.clearBuffer();
    u8g2.setFontMode(1);
    u8g2.setBitmapMode(1);
    u8g2.setDrawColor(1);
}

void powerSensors(bool state) {
    if (state) {
        digitalWrite(PWR_SENSORS, HIGH);
        delay(10);
        i2c_init();
    } else {
        i2c_deinit();
        digitalWrite(PWR_SENSORS, LOW);
    }
}

void sendBufferDirect() {
    convertBuffer();
    SPI.beginTransaction(SPISettings(12000000, MSBFIRST, SPI_MODE0));
    
    digitalWrite(PIN_LCD_CS, LOW);
    setLcdWindow(0x17, 0x24, 0x00, 0xBF);
    SPI.transfer(convertedBuf, sizeof(convertedBuf));
    digitalWrite(PIN_LCD_CS, HIGH);
    
    SPI.endTransaction();
}

static uint8_t partialBuf[768];
bool forceMainRedraw = false;
void sendPartialBuffer() {
    uint8_t devXStart = 60, devXEnd = 120;
    uint8_t devYStart = 68, devYEnd = 150;

    uint8_t casetStart = devXStart / 12;
    uint8_t casetEnd   = devXEnd   / 12;
    uint8_t rasetStart = devYStart / 2;  
    uint8_t rasetEnd   = devYEnd   / 2;  

    uint8_t *dst = partialBuf;
    for (uint8_t r = rasetStart; r <= rasetEnd; r++) {
        for (uint8_t g = casetStart; g <= casetEnd; g++) {
            uint16_t off = (uint16_t)(r * 14 + g) * 3;
            *dst++ = convertedBuf[off];
            *dst++ = convertedBuf[off + 1];
            *dst++ = convertedBuf[off + 2];
        }
    }

SPI.beginTransaction(SPISettings(12000000, MSBFIRST, SPI_MODE0));

    digitalWrite(PIN_LCD_CS, LOW);
    setLcdWindow(0x17 + casetStart, 0x17 + casetEnd, rasetStart, rasetEnd);
    SPI.transfer(partialBuf, dst - partialBuf); 
    digitalWrite(PIN_LCD_CS, HIGH);
    
    SPI.endTransaction();
}

void paintMainScreen(uint32_t now) {
    uint32_t uptimeSecs = now - bootEpoch;
    uint32_t seconds;

    if (gLastSyncedEpoch != 0 && gLastSyncStmEpoch != 0) {
        uint32_t elapsed = now - gLastSyncStmEpoch;
        seconds = (gLastSyncedEpoch + elapsed) % 60;
    } else {
        seconds = now % 60;
    }

    if (seconds == 0 || uptimeSecs == 0 || forceMainRedraw) {
        readAllSensors(now);
        forceMainRedraw = false;
        
        uptimeSecs = now - bootEpoch;
        uint32_t uptimeDays  = uptimeSecs / 86400;
        uint32_t uptimeHours = (uptimeSecs % 86400) / 3600;
        uint32_t uptimeMins  = (uptimeSecs % 3600) / 60;
        
        uint32_t hours24 = (now % 86400) / 3600;
        uint32_t minutes = (now % 3600) / 60;
        seconds = now % 60;
        
        uint8_t h12 = hours24 % 12;
        if (h12 == 0) h12 = 12;
        bool pm = hours24 >= 12;

        char timeMainBuf[12], secsBuf[4], tempBuf[16], humBuf[24], pressureBuf[16], voltBuf[16], curBuf[16], uptimeBuf[20];
        snprintf(timeMainBuf, sizeof(timeMainBuf), "%u:%02u:", h12, minutes);
        snprintf(secsBuf, sizeof(secsBuf), "%02u", seconds);

        int t_int = (int)gTemperature;
        int t_frac = abs((int)(gTemperature * 10.0f)) % 10;
        const char* t_sign = (gTemperature < 0.0f && t_int == 0) ? "-" : "";

        int h_int = (int)(gHumidity + 0.5f);
        int ah_int = (int)gAbsHumidity;
        int ah_frac = abs((int)(gAbsHumidity * 10.0f)) % 10;

        int p_int = (int)gPressure;
        int p_frac = abs((int)(gPressure * 100.0f)) % 100;

        int v_int = (int)gBatteryVoltage;
        int v_frac = abs((int)(gBatteryVoltage * 1000.0f)) % 1000;

        snprintf(tempBuf, sizeof(tempBuf), "%s%d.%d\xc2\xb0" "C", t_sign, t_int, t_frac);
        snprintf(humBuf, sizeof(humBuf), "%d%% %d.%dg", h_int, ah_int, ah_frac);
        snprintf(pressureBuf, sizeof(pressureBuf), "%d.%02dmB", p_int, p_frac);
        snprintf(voltBuf, sizeof(voltBuf), "%d.%03dv", v_int, v_frac);
        snprintf(curBuf, sizeof(curBuf), "%lduA", (long)gBatteryCurrent_uA);
        
        if (uptimeDays > 365) {
            uint32_t uptimeYears = uptimeDays / 365;
            uint32_t remDays = uptimeDays % 365;
            snprintf(uptimeBuf, sizeof(uptimeBuf), "%luy %lud %luh",
                    (unsigned long)uptimeYears, (unsigned long)remDays, (unsigned long)uptimeHours);
        } else {
            snprintf(uptimeBuf, sizeof(uptimeBuf), "%lud %luh %lum",
                    (unsigned long)uptimeDays, (unsigned long)uptimeHours, (unsigned long)uptimeMins);
        }
        
        u8g2.clearBuffer();
        u8g2.setFontMode(1);
        u8g2.setBitmapMode(1);
        
        drawRleBackground(0, 0, 384, 164, epd_bitmap_rle);
        
        u8g2.setDrawColor(0); 
        if (pm) {
            u8g2.drawBox(320, 42, 50, 32); 
        } else {
            u8g2.drawBox(320, 74, 50, 31); 
        }

        u8g2.setDrawColor(1); 
        u8g2.setFont(u8g2_font_logisoso62_tn);
        
        int timeWidth = u8g2.getStrWidth(timeMainBuf);
        u8g2.drawStr(234 - timeWidth, 102, timeMainBuf);
        u8g2.drawStr(236, 102, secsBuf); 
        
        u8g2.setFont(u8g2_font_profont17_tr);
        u8g2.drawStr(48, 26, tempBuf);
        u8g2.drawStr(146, 26, humBuf);
        
        int pressWidth = u8g2.getStrWidth(pressureBuf);
        u8g2.drawStr(371 - pressWidth, 26, pressureBuf);
        
        u8g2.drawStr(56, 162, voltBuf);
        u8g2.drawStr(169, 162, curBuf);
        u8g2.drawStr(280, 162, uptimeBuf);
        
        sendBufferDirect();
    } else {
        char secsBuf[4];
        snprintf(secsBuf, sizeof(secsBuf), "%02u", seconds);
        
        u8g2.setDrawColor(0); 
        u8g2.clearBuffer();
        u8g2.setDrawColor(1);
        u8g2.setFont(u8g2_font_logisoso62_tn);
        u8g2.drawStr(236, 126, secsBuf);

        convertBuffer(8, 18);
        sendPartialBuffer();
    }
}

void drawMenu() {
    prepDisplay();
    u8g2.setFont(u8g2_font_profont17_tr);
    
    for(int i = 0; i < MENU_VISIBLE && menuScrollOffset + i < NUM_MENU_ITEMS; i++) {
        int idx = menuScrollOffset + i;
        int y = 22 + i * 22;
        u8g2.drawStr(40, y, menuItems[idx]);
        if(idx == menuSelection) {
            u8g2.drawFrame(30, y - 18, 220, 24);
        }
    }
    sendBufferDirect();
}

void drawSetTime() {
    prepDisplay();
    

    u8g2.setFont(u8g2_font_profont17_tr);
    u8g2.drawStr(100, 30, "Set System Time");
    
    char timeBuf[16];
    snprintf(timeBuf, sizeof(timeBuf), "%02d:%02d", tempHour, tempMinute);
    
    u8g2.setFont(u8g2_font_logisoso62_tn);
    int timeWidth = u8g2.getStrWidth(timeBuf);
    int xOffset = (384 - timeWidth) / 2; 
    u8g2.drawStr(xOffset, 110, timeBuf);
    
    if (!editingMinutes) {
        u8g2.drawBox(xOffset, 120, 75, 5); 
    } else {
        u8g2.drawBox(xOffset + 95, 120, 75, 5); 
    }
    
    sendBufferDirect();
}

float getHistValue(int chartIndex, int rawIdx) {
    // Determine the actual index in the circular buffer
    uint16_t cCount = (chartIndex == 4) ? voltCount : histCount;
    uint16_t cHead  = (chartIndex == 4) ? voltHead : histHead;
    int idx = (cCount < MAX_SAMPLES) ? rawIdx : (cHead + rawIdx) % MAX_SAMPLES;

    // Fetch and un-scale
    switch(chartIndex) {
        case 0: return histTemp[idx] / 100.0f;
        case 1: return histRH[idx] / 100.0f;
        case 2: return histAbsHum[idx] / 100.0f;
        case 3: {
            uint16_t enc = histPress[idx];
            return ((enc & 0x8000) ? 1000.0f : 900.0f) + (enc & 0x7FFF) / 100.0f;
        }
        case 4: return histVolt[idx] / 1000.0f;
        case 5: return (float)histCur[idx];
        default: return 0.0f;
    }
}

void drawChart(int chartIndex) {
    prepDisplay();

    // 1. Get the count. The getHistValue() helper handles the rest.
    uint16_t cCount = (chartIndex == 4) ? voltCount : histCount;

    // Bail out if there is no data yet
    if (cCount == 0) {
        sendBufferDirect();
        return;
    }

    // 2. The Min/Max Loop: 
    // Scan the history to find the bounds for the Y-Axis
    float minV = 99999.0f;
    float maxV = -99999.0f;

    for(int i = 0; i < cCount; i++) {
        // Use our new helper function instead of dataArr!
        float val = getHistValue(chartIndex, i);
        
        if(val < minV) minV = val;
        if(val > maxV) maxV = val;
    }

    // Tightened the threshold so a 0.001v drop won't falsely trigger the padding.
    if(maxV - minV < 0.0001f) { 
        if (chartIndex == 4) {
            maxV += 0.005f;
            minV -= 0.005f;
        } else {
            maxV += 1.0f;
            minV -= 1.0f;
        }
    }
    
    // ... the rest of your chart drawing code continues here ...

    u8g2.setFont(u8g2_font_profont17_tr);
    int titleW = u8g2.getStrWidth(menuItems[chartIndex]);
    u8g2.drawStr((384 - titleW) / 2, 20, menuItems[chartIndex]);

    // Increased xStart to 36 so Y-axis labels (like 1013.2) don't get clipped on the left
    int xStart = 36; 
    int xEnd = 370;
    int yTop = 30;
    int yBottom = 146;

    // --- Draw Axes ---
    u8g2.drawBox(xStart, yTop, 2, yBottom - yTop + 2); 
    u8g2.drawBox(xStart, yBottom, xEnd - xStart, 2);   

    u8g2.setFont(u8g2_font_4x6_tr);
    char buf[32];
    
    // --- Y-Axis Labels & Ticks ---
    int yMid = yTop + (yBottom - yTop) / 2;
    float midV = (maxV + minV) / 2.0f;
    
    // Helper to safely format floats to 1 decimal place without %f
// Helper to safely format floats without %f
    auto fmtFloat = [&](float v) {
        int i = (int)v;
        const char* sign = (v < 0 && i == 0) ? "-" : "";
        if (chartIndex == 4) {
            // 3 decimal places for Voltage
            int f = abs((int)(v * 1000.0f)) % 1000;
            snprintf(buf, sizeof(buf), "%s%d.%03d", sign, i, f);
        } else if (chartIndex == 3) {
            // 2 decimal places for Pressure
            int f = abs((int)(v * 100.0f)) % 100;
            snprintf(buf, sizeof(buf), "%s%d.%02d", sign, i, f);
        } else {
            // 1 decimal place for everything else
            int f = abs((int)(v * 10.0f)) % 10;
            snprintf(buf, sizeof(buf), "%s%d.%d", sign, i, f);
        }
    };

    // Y-Axis Ticks
    u8g2.drawLine(xStart - 4, yTop, xStart, yTop);
    u8g2.drawLine(xStart - 4, yMid, xStart, yMid);
    u8g2.drawLine(xStart - 4, yBottom, xStart, yBottom);

    // Y-Axis Labels (Right-aligned to the tick marks)
    fmtFloat(maxV);
    int tw = u8g2.getStrWidth(buf);
    u8g2.drawStr(xStart - 6 - tw, yTop + 3, buf);
    
    fmtFloat(midV);
    tw = u8g2.getStrWidth(buf);
    u8g2.drawStr(xStart - 6 - tw, yMid + 3, buf);

    fmtFloat(minV);
    tw = u8g2.getStrWidth(buf);
    u8g2.drawStr(xStart - 6 - tw, yBottom, buf);

    // --- X-Axis Labels & Ticks ---
    // Start Tick (0)
    u8g2.drawLine(xStart, yBottom, xStart, yBottom + 4);
    u8g2.drawStr(xStart - 2, yBottom + 12, "0");
    
    // End Tick (cCount)
    u8g2.drawLine(xEnd, yBottom, xEnd, yBottom + 4);
    snprintf(buf, sizeof(buf), "%d", cCount);
    int endW = u8g2.getStrWidth(buf);
    u8g2.drawStr(xEnd - endW/2, yBottom + 12, buf);

    // Intermediate X-Ticks (25%, 50%, 75%)
    for (int i = 1; i <= 3; i++) {
        int tickX = xStart + i * (xEnd - xStart) / 4;
        int tickVal = i * cCount / 4;
        u8g2.drawLine(tickX, yBottom, tickX, yBottom + 4); 
        
        snprintf(buf, sizeof(buf), "%d", tickVal);
        int twX = u8g2.getStrWidth(buf);
        u8g2.drawStr(tickX - twX / 2, yBottom + 12, buf);   
    }

    int prevX = -1, prevY = -1;
    int cursorX = -1, cursorY = -1;
    float cursorVal = 0;

    if (chartCursor >= cCount) chartCursor = cCount - 1;
    if (chartCursor < 0 && cCount > 0) chartCursor = cCount - 1;

    for(int i = 0; i < cCount; i++) {
        // 1. Delete the "int idx = ..." line entirely!
        
        // 2. Just ask our helper function for the value at step 'i'
        float val = getHistValue(chartIndex, i);
        
        // 3. The rest of your X/Y math stays exactly the same
        int x = xStart + 2 + (i * (xEnd - xStart - 4)) / (cCount > 1 ? cCount - 1 : 1);
        int y = yBottom - 2 - ((val - minV) * (yBottom - yTop - 4) / (maxV - minV));
        
        if(prevX != -1) {
            u8g2.drawLine(prevX, prevY, x, y);
        }
        prevX = x;
        prevY = y;

        if (i == chartCursor) {
            cursorX = x;
            cursorY = y;
            cursorVal = val;
        }
    }

    // --- Cursor Drawing & Labeling ---
    if (cursorX != -1) {
        // Vertical dotted crosshair line
        for(int cy = yTop; cy < yBottom; cy += 4) {
            u8g2.drawPixel(cursorX, cy); 
        }
        // Horizontal dotted crosshair line
        for(int cx = xStart; cx < xEnd; cx += 4) {
            u8g2.drawPixel(cx, cursorY); 
        }
        
        u8g2.drawCircle(cursorX, cursorY, 4);
        
        u8g2.setFont(u8g2_font_profont17_tr);
        
        // --- Real Time Timestamp Calculation ---
        int timeAgo = (cCount - 1) - chartCursor;
        uint32_t now = stmRtc.getEpoch();
        uint32_t pointTime = 0;
        
        // chartIndex 4 (Voltage) runs on an hourly tick, others on a minute tick
        if (chartIndex == 4) {
            pointTime = now - (timeAgo * 3600);
        } else {
            pointTime = now - (timeAgo * 60);
        }
        
        uint32_t h24 = (pointTime % 86400) / 3600;
        uint32_t m = (pointTime % 3600) / 60;
        
// Format cursor value manually avoiding %f
        int val_i = (int)cursorVal;
        const char* sign = (cursorVal < 0 && val_i == 0) ? "-" : "";
        
        if (chartIndex == 4) {
            // 3 decimals for Voltage: "4.125 @ 14:27"
            int val_f = abs((int)(cursorVal * 1000.0f)) % 1000;
            snprintf(buf, sizeof(buf), "%s%d.%03d @ %02lu:%02lu", sign, val_i, val_f, (unsigned long)h24, (unsigned long)m);
        } else {
            // 2 decimals for other sensors: "24.53 @ 14:27"
            int val_f = abs((int)(cursorVal * 100.0f)) % 100;
            snprintf(buf, sizeof(buf), "%s%d.%02d @ %02lu:%02lu", sign, val_i, val_f, (unsigned long)h24, (unsigned long)m);
        }
        
        int valW = u8g2.getStrWidth(buf);
        
        // Ensure text stays on opposite side of cursor to prevent overlapping the line
        if (cursorX < 192) {
            u8g2.drawStr(370 - valW, 20, buf); 
        } else {
            u8g2.drawStr(xStart, 20, buf);
        }
    }

    sendBufferDirect();
}

void alarmMatch(void* data) { wakeFlag = true; }

void sleepPins() {
    digitalWrite(PIN_LCD_CS,  HIGH);
    digitalWrite(PIN_LCD_RST, HIGH);
    digitalWrite(PIN_LCD_DC,  LOW);
    digitalWrite(PWR_SENSORS, LOW);
    
    GPIO_InitTypeDef g = {0};
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    g.Pin   = GPIO_PIN_5 | GPIO_PIN_7;   
    HAL_GPIO_Init(GPIOA, &g);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5 | GPIO_PIN_7, GPIO_PIN_RESET);

    g.Mode  = GPIO_MODE_ANALOG;
    g.Pin   = GPIO_PIN_6 | GPIO_PIN_7;   
    HAL_GPIO_Init(GPIOB, &g);

    __HAL_RCC_I2C1_CLK_DISABLE();        
}

void wakePins() {
    GPIO_InitTypeDef g = {0};
    g.Pin       = GPIO_PIN_5 | GPIO_PIN_7;
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_NOPULL;
    g.Speed     = GPIO_SPEED_FREQ_LOW;
    g.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &g);

    __HAL_RCC_I2C1_CLK_ENABLE();         
}

void readAllSensors(uint32_t &now) {
    powerSensors(true); 
    if (ds3231_begin()) {
        DateTime dt = ds3231_now();
        stmRtc.setEpoch(dt.unixtime());
        now = dt.unixtime();
        gLastSyncedEpoch = dt.unixtime();
        gLastSyncStmEpoch = stmRtc.getEpoch();
    }

    ina219_init();
    ina219_configure();
    ina219_startSingle();
    
    aht20_begin();
    bmp280_begin();

    // 1. TRIGGER ALL SENSORS
    aht20_trigger();
    bmp280_trigger();
    
    // 2. WAIT ONCE (80ms covers the slowest sensor, the AHT20)
    delay(80); 

    // 3. COLLECT ALL SENSORS
    aht20_collect(&gTemperature, &gHumidity);
    gPressure = bmp280_collect() / 100.0f;

    float Temp = gTemperature;
    float Humid = gHumidity;
    gAbsHumidity = (6.112f * expf(((17.67f * Temp) / (Temp + 243.5f))) * Humid * 2.1674f) / (273.15f + Temp);
    
    // INA219 max conversion time is ~68ms, so it is almost certainly ready, 
    // but we keep a tiny timeout just in case the I2C bus stalls.
    uint32_t inaTimeout = millis();
    while (!ina219_conversionReady() && (millis() - inaTimeout < 150)) delay(1);
    
    gBatteryVoltage = ina219_getBusVoltage();
    float current_mA = ina219_getCurrent();
    gBatteryCurrent_uA = (int32_t)(current_mA * 1000.0f + (current_mA > 0.0f ? 0.5f : -0.5f));
    
    powerSensors(false);
}

volatile bool btnWakeFlag = false;

void buttonISR() {
    btnWakeFlag = true;
}

bool isPressed(uint32_t pin) {
    return digitalRead(pin) == LOW; 
}

void setup() {
    pinMode(PIN_LCD_TE, INPUT_ANALOG);
    pinMode(PIN_LCD_CS,  OUTPUT); digitalWrite(PIN_LCD_CS,  HIGH);
    pinMode(PIN_LCD_DC,  OUTPUT); digitalWrite(PIN_LCD_DC,  HIGH);
    pinMode(PIN_LCD_RST, OUTPUT); digitalWrite(PIN_LCD_RST, HIGH);

    SPI.setSCLK(PIN_LCD_SCLK);
    SPI.setMOSI(PIN_LCD_MOSI);
    u8g2.begin();
    u8g2.setBusClock(12000000);
    u8g2.sendF("c", 0x39);
    u8g2.sendF("ca", 0xB2, 0x00);
    prepDisplay();
    u8g2.setFont(u8g2_font_profont17_tr);
    u8g2.drawStr(20, 80, "10 second window to flash...");
    sendBufferDirect();

    delay(10000);

    pinMode(PA10, INPUT_ANALOG); 
    pinMode(PA9,  INPUT_ANALOG);  
    pinMode(PA6,  INPUT_ANALOG); 
    pinMode(PA8,  INPUT_ANALOG);
    pinMode(PA11, INPUT_ANALOG);
    pinMode(PA12, INPUT_ANALOG);
    pinMode(PB5,  INPUT_ANALOG);

    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
    GPIOA->MODER |= (GPIO_MODER_MODE13 | GPIO_MODER_MODE14);
    pinMode(PA_15, INPUT_ANALOG); 

    pinMode(PC14, INPUT_ANALOG); 
    pinMode(PC15, INPUT_ANALOG);

    pinMode(BTN_LEFT, INPUT);
    pinMode(BTN_UP, INPUT);
    pinMode(BTN_DOWN, INPUT);
    pinMode(BTN_RIGHT, INPUT);

    pinMode(PWR_SENSORS, OUTPUT); digitalWrite(PWR_SENSORS, LOW);
    stmRtc.begin(); 

// --- SETUP DEDUPLICATION BLOCK ---
    powerSensors(true);
    if (ds3231_begin()) {
       // if (ds3231_lostPower()) {
         //   ds3231_adjust(DateTime((uint32_t)(BUILD_EPOCH - 14340)));
        //}
    }
    powerSensors(false);

    // Call readAllSensors directly to handle the first power-up and global populating
    uint32_t dummyTime = 0;
    readAllSensors(dummyTime); 
    
    bootEpoch = stmRtc.getEpoch();
    lastSensorRead = bootEpoch;
    // ----------------------------------


    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin       = GPIO_PIN_5 | GPIO_PIN_7;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);


    
    forceMainRedraw = true;
    paintMainScreen(stmRtc.getEpoch());
    
    sleepPins();
    HAL_DBGMCU_DisableDBGStopMode();

    // Reconfigure SWDIO (PA13) and SWCLK (PA14) to Analog to prevent floating leakage
  //  pinMode(PA_13, INPUT_ANALOG);
  //  pinMode(PA_14, INPUT_ANALOG);
    LowPower.begin();
    stmRtc.attachInterrupt(alarmMatch); 
}

void loop() {
    uint32_t now = stmRtc.getEpoch();
      static uint32_t lastDisplayTick = 0;  
    auto goMain = [&]() {
        currentState = STATE_MAIN;
        forceMainRedraw = true;  
        now = stmRtc.getEpoch(); 
        paintMainScreen(now);
        lastDisplayTick = now;
    };

    static uint32_t lastInteractionTime = 0;
    
    static bool lastUp = false;
    static bool lastDown = false;
    static bool lastLeft = false;
    static bool lastRight = false;
    
    if (lastDisplayTick == 0) lastDisplayTick = now;

    if (now != lastDisplayTick) {
        lastDisplayTick = now;
        
        if (now - lastSensorRead >= 60) {
            lastSensorRead = now;

            // Scale and cast to integers before saving
            histTemp[histHead]   = (int16_t)(gTemperature * 100.0f);
            histRH[histHead]     = (uint16_t)(gHumidity * 100.0f); 
            histAbsHum[histHead] = (uint16_t)(gAbsHumidity * 100.0f);
            { // Encode: bit 15 = hundreds is 10, bits 0-14 = remainder*100
                float p = gPressure;
                int h = (int)(p / 100.0f);
                uint16_t rem = (uint16_t)((p - h * 100.0f) * 100.0f + 0.5f);
                histPress[histHead] = (h == 10 ? 0x8000 : 0) | (rem & 0x7FFF);
            }
            histCur[histHead]    = (int16_t)gBatteryCurrent_uA;

            histHead = (histHead + 1) % MAX_SAMPLES;
            if (histCount < MAX_SAMPLES) histCount++;

            if (voltSampleCounter == 0) {
                histVolt[voltHead] = (uint16_t)(gBatteryVoltage * 1000.0f);
                voltHead = (voltHead + 1) % MAX_SAMPLES;
                if (voltCount < MAX_SAMPLES) voltCount++;
            }
            voltSampleCounter = (voltSampleCounter + 1) % 60;
        }

        if (currentState == STATE_MAIN) {
            paintMainScreen(now);
        }
    }

bool currUp = isPressed(BTN_UP);
    bool currDown = isPressed(BTN_DOWN);
    bool currLeft = isPressed(BTN_LEFT);
    bool currRight = isPressed(BTN_RIGHT);

    // Timers for auto-repeat behavior
    static uint32_t upHoldStart = 0;
    static uint32_t lastUpRepeat = 0;
    static uint32_t downHoldStart = 0;
    static uint32_t lastDownRepeat = 0;
    
    // Auto-repeat timing thresholds
    const uint32_t REPEAT_DELAY = 400; // milliseconds to hold before scrolling starts
    const uint32_t REPEAT_RATE = 50;   // milliseconds between scroll ticks

    // UP button logic (with auto-repeat)
    bool pressUp = false;
    if (currUp && !lastUp) {
        pressUp = true;
        upHoldStart = millis();
        lastUpRepeat = millis();
    } else if (currUp && lastUp) {
        if (millis() - upHoldStart > REPEAT_DELAY) {
            if (millis() - lastUpRepeat >= REPEAT_RATE) {
                pressUp = true;
                lastUpRepeat = millis();
            }
        }
    }

    // DOWN button logic (with auto-repeat)
    bool pressDown = false;
    if (currDown && !lastDown) {
        pressDown = true;
        downHoldStart = millis();
        lastDownRepeat = millis();
    } else if (currDown && lastDown) {
        if (millis() - downHoldStart > REPEAT_DELAY) {
            if (millis() - lastDownRepeat >= REPEAT_RATE) {
                pressDown = true;
                lastDownRepeat = millis();
            }
        }
    }

    // LEFT and RIGHT remain strictly single-press
    bool pressLeft  = currLeft && !lastLeft;
    bool pressRight = currRight && !lastRight;

    lastUp = currUp;
    lastDown = currDown;
    lastLeft = currLeft;
    lastRight = currRight;

    if (currentState == STATE_MAIN) {
        if (pressRight) {
            currentState = STATE_MENU;
            menuSelection = 0;
            menuScrollOffset = 0;
            lastInteractionTime = millis();
            drawMenu();
        }
    } 
    else if (currentState == STATE_MENU) {
        if (pressUp || pressDown || pressLeft || pressRight) {
            lastInteractionTime = millis(); 
            
            if (pressUp) {
                menuSelection = (menuSelection == 0) ? NUM_MENU_ITEMS - 1 : menuSelection - 1;
                if (menuSelection < menuScrollOffset) menuScrollOffset = menuSelection;
                if (menuSelection > menuScrollOffset + MENU_VISIBLE - 1) menuScrollOffset = menuSelection - MENU_VISIBLE + 1;
                drawMenu();
            } else if (pressDown) {
                menuSelection = (menuSelection == NUM_MENU_ITEMS - 1) ? 0 : menuSelection + 1;
                if (menuSelection < menuScrollOffset) menuScrollOffset = menuSelection;
                if (menuSelection > menuScrollOffset + MENU_VISIBLE - 1) menuScrollOffset = menuSelection - MENU_VISIBLE + 1;
                drawMenu();
            } else if (pressLeft) {
                    goMain();
            } else if (pressRight) {
                if (menuSelection == 6) { 
                    currentState = STATE_SET_TIME;
                    editingMinutes = false;
                    tempHour = (now % 86400) / 3600;
                    tempMinute = (now % 3600) / 60;
                    drawSetTime();
                } else if (menuSelection == 7) {
                    memset(histTemp, 0, sizeof(histTemp));
                    memset(histRH, 0, sizeof(histRH));
                    memset(histAbsHum, 0, sizeof(histAbsHum));
                    memset(histPress, 0, sizeof(histPress));
                    memset(histVolt, 0, sizeof(histVolt));
                    memset(histCur, 0, sizeof(histCur));
                    histHead = 0; histCount = 0;
                    voltHead = 0; voltCount = 0; voltSampleCounter = 0;
                    bootEpoch = stmRtc.getEpoch();
                    goMain();
                } else if (menuSelection == 8) {
                    NVIC_SystemReset();
                } else {
                    currentState = STATE_CHART;
                    chartCursor = -1; 
                    drawChart(menuSelection);
                }
            }
        }
    } 
    else if (currentState == STATE_SET_TIME) {
        if (pressUp || pressDown || pressLeft || pressRight) {
            lastInteractionTime = millis();
            
            if (pressLeft) { 
                currentState = STATE_MENU;
                drawMenu();
            } else if (pressRight) {
                if (!editingMinutes) {
                    editingMinutes = true; 
                    drawSetTime();
                } else {
                    digitalWrite(PWR_SENSORS, HIGH);
                    delay(10);
                    i2c_init();
                    
                    if (ds3231_begin()) {
                        DateTime dt = ds3231_now();
                        DateTime newTime(dt.year(), dt.month(), dt.day(), tempHour, tempMinute, 0);
                        ds3231_adjust(newTime);
                        stmRtc.setEpoch(newTime.unixtime());
                    }
                    
                    i2c_deinit();
                    digitalWrite(PWR_SENSORS, LOW);
                    
                        goMain();
                }
            } else if (pressUp) {
                if (!editingMinutes) {
                    tempHour = (tempHour == 23) ? 0 : tempHour + 1;
                } else {
                    tempMinute = (tempMinute == 59) ? 0 : tempMinute + 1;
                }
                drawSetTime();
            } else if (pressDown) {
                if (!editingMinutes) {
                    tempHour = (tempHour == 0) ? 23 : tempHour - 1;
                } else {
                    tempMinute = (tempMinute == 0) ? 59 : tempMinute - 1;
                }
                drawSetTime();
            }
        }
    }
    else if (currentState == STATE_CHART) {
        if (pressLeft || pressRight || pressUp || pressDown) {
            lastInteractionTime = millis();
            uint16_t cCount = (menuSelection == 4) ? voltCount : histCount;

            if (pressLeft) {
                currentState = STATE_MENU;
                drawMenu();
            } else if (pressRight) {
                        goMain();
            } else if (pressUp) { 
                if (chartCursor == -1) chartCursor = cCount - 1;
                if (chartCursor < cCount - 1) chartCursor++;
                drawChart(menuSelection);
            } else if (pressDown) { 
                if (chartCursor == -1) chartCursor = cCount - 1;
                if (chartCursor > 0) chartCursor--;
                drawChart(menuSelection);
            }
        }
    }

    if (currentState != STATE_MAIN && (millis() - lastInteractionTime > 15000)) {
                goMain();
    }

    if (currentState == STATE_MAIN) {
        SPI.end(); 
        sleepPins(); 
        
        uint32_t targetAlarm = stmRtc.getEpoch() + 1;
        stmRtc.setAlarmEpoch(targetAlarm);
        
        if (stmRtc.getEpoch() >= targetAlarm) {
            targetAlarm = stmRtc.getEpoch() + 1;
            stmRtc.setAlarmEpoch(targetAlarm);
        }
        
        LowPower.deepSleep(); 
        
        wakePins(); 
        SPI.begin(); 
    } else {
        delay(20); 
    }
}