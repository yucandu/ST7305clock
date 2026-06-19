//STM32L412K8T6

#include <Arduino.h>
#include <U8g2lib.h>
#include <SPI.h>
#include <Wire.h>
#include "STM32LowPower.h"
#include "STM32RTC.h"
#include <RTClib.h>
#include <INA219_WE.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_BMP280.h>
#include "monochromebg_rle.h"

extern "C" float expf(float);

STM32RTC& stmRtc = STM32RTC::getInstance();
RTC_DS3231 ds3231;
INA219_WE ina219(0x40);
Adafruit_AHTX0 aht;
Adafruit_BMP280 bmp;

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

#define I2C1_SCL        PB6
#define I2C1_SDA        PB7
#define PWR_SENSORS     PB0
#define PIN_LCD_SCLK    PA5
#define PIN_LCD_MOSI    PA7
#define PIN_LCD_CS      PA4
#define PIN_LCD_DC      PA0
#define PIN_LCD_TE      PA1
#define PIN_LCD_RST     PA2 

// --- NEW BUTTON DEFINES ---
#define BTN_LEFT PA3
#define BTN_UP PA6
#define BTN_DOWN PB3
#define BTN_RIGHT PB4

#define SERIAL_BAUD     115200

// --- NEW APP STATE VARIABLES ---
enum AppState { STATE_MAIN, STATE_MENU, STATE_CHART, STATE_SET_TIME };
AppState currentState = STATE_MAIN;
int8_t menuSelection = 0;
const char* menuItems[7] = {"Temperature", "Rel Humidity", "Abs Humidity", "Pressure", "Voltage", "Current", "Set Time"};
const int8_t NUM_MENU_ITEMS = 7;

// --- TIME SETTING VARIABLES ---
bool editingMinutes = false;
int8_t tempHour = 0;
int8_t tempMinute = 0;

// --- NEW DATA ARRAYS (400 points = ~9.6KB RAM) ---
#define MAX_SAMPLES 400
float histTemp[MAX_SAMPLES];
float histRH[MAX_SAMPLES];
float histAbsHum[MAX_SAMPLES];
float histPress[MAX_SAMPLES];
float histVolt[MAX_SAMPLES];
float histCur[MAX_SAMPLES];
uint16_t histHead = 0;
uint16_t histCount = 0;

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
uint32_t gBatteryCurrent_uA = 0;

uint32_t bootEpoch = 0;
uint32_t lastSensorRead = 0;

static uint8_t convertedBuf[8064];

static const uint8_t map1[16] = {
    0x00,0x02,0x08,0x0A,0x20,0x22,0x28,0x2A,
    0x80,0x82,0x88,0x8A,0xA0,0xA2,0xA8,0xAA
};
static const uint8_t map2[16] = {
    0x00,0x01,0x04,0x05,0x10,0x11,0x14,0x15,
    0x40,0x41,0x44,0x45,0x50,0x51,0x54,0x55
};

void convertBuffer() {
    uint8_t *src = u8g2.getBufferPtr();
    uint8_t *dst = convertedBuf;
    const uint8_t tileWidth = 21;
    const uint8_t tileHeight = 48;
    for (uint8_t tileRow = 0; tileRow < tileHeight; tileRow++) {
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

void sendBufferDirect() {
    convertBuffer();
    SPI.beginTransaction(SPISettings(12000000, MSBFIRST, SPI_MODE0));
    
    digitalWrite(PIN_LCD_CS, LOW);
    digitalWrite(PIN_LCD_DC, LOW);  SPI.transfer(0x2A);
    digitalWrite(PIN_LCD_DC, HIGH); SPI.transfer(0x17); SPI.transfer(0x24);
    digitalWrite(PIN_LCD_DC, LOW);  SPI.transfer(0x2B);
    digitalWrite(PIN_LCD_DC, HIGH); SPI.transfer(0x00); SPI.transfer(0xBF);
    digitalWrite(PIN_LCD_DC, LOW);  SPI.transfer(0x2C);
    digitalWrite(PIN_LCD_DC, HIGH);
    SPI.transfer(convertedBuf, sizeof(convertedBuf));
    digitalWrite(PIN_LCD_CS, HIGH);
    
    SPI.endTransaction();
}

static uint8_t partialBuf[1500];
bool forceMainRedraw = false;
void sendPartialBuffer() {
    uint8_t devXStart = 60, devXEnd = 120;
    uint8_t devYStart = 68, devYEnd = 158;

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
    digitalWrite(PIN_LCD_DC, LOW);  SPI.transfer(0x2A);
    digitalWrite(PIN_LCD_DC, HIGH); SPI.transfer(0x17 + casetStart); SPI.transfer(0x17 + casetEnd);
    digitalWrite(PIN_LCD_DC, LOW);  SPI.transfer(0x2B);
    digitalWrite(PIN_LCD_DC, HIGH); SPI.transfer(rasetStart); SPI.transfer(rasetEnd);
    digitalWrite(PIN_LCD_DC, LOW);  SPI.transfer(0x2C);
    digitalWrite(PIN_LCD_DC, HIGH);
    
    SPI.transfer(partialBuf, dst - partialBuf); 
    
    digitalWrite(PIN_LCD_CS, HIGH);
    SPI.endTransaction();
}

void paintMainScreen(uint32_t now) {
    uint32_t seconds = now % 60;
    char secsBuf[4];
    snprintf(secsBuf, sizeof(secsBuf), "%02u", seconds);
    uint32_t uptimeSecs = now - bootEpoch;

    if (seconds == 0 || uptimeSecs == 0 || forceMainRedraw) {
        readAllSensors(now);
        forceMainRedraw = false;
        uint32_t uptimeDays  = uptimeSecs / 86400;
        uint32_t uptimeHours = (uptimeSecs % 86400) / 3600;
        uint32_t uptimeMins  = (uptimeSecs % 3600) / 60;
        uint32_t hours24 = (now % 86400) / 3600;
        uint32_t minutes = (now % 3600) / 60;
        
        uint8_t h12 = hours24 % 12;
        if (h12 == 0) h12 = 12;
        bool pm = hours24 >= 12;

        char timeMainBuf[12], tempBuf[16], humBuf[24], pressureBuf[16], voltBuf[16], curBuf[16], uptimeBuf[20];
        snprintf(timeMainBuf, sizeof(timeMainBuf), "%u:%02u:", h12, minutes);

        int t_int = (int)gTemperature;
        int t_frac = abs((int)(gTemperature * 10.0f)) % 10;
        const char* t_sign = (gTemperature < 0.0f && t_int == 0) ? "-" : "";

        int h_int = (int)(gHumidity + 0.5f);
        int ah_int = (int)gAbsHumidity;
        int ah_frac = abs((int)(gAbsHumidity * 10.0f)) % 10;

        int p_int = (int)gPressure;
        int p_frac = abs((int)(gPressure * 10.0f)) % 10;

        int v_int = (int)gBatteryVoltage;
        int v_frac = abs((int)(gBatteryVoltage * 1000.0f)) % 1000;

        snprintf(tempBuf, sizeof(tempBuf), "%s%d.%d\xc2\xb0" "C", t_sign, t_int, t_frac);
        snprintf(humBuf, sizeof(humBuf), "%d%% %d.%dg", h_int, ah_int, ah_frac);
        snprintf(pressureBuf, sizeof(pressureBuf), "%d.%dmB", p_int, p_frac);
        snprintf(voltBuf, sizeof(voltBuf), "%d.%03dv", v_int, v_frac);
        snprintf(curBuf, sizeof(curBuf), "%luuA", (unsigned long)gBatteryCurrent_uA);
        
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
        
        u8g2.setFont(u8g2_font_logisoso16_tr);
        u8g2.drawStr(48, 26, tempBuf);
        u8g2.drawStr(146, 26, humBuf);
        
        int pressWidth = u8g2.getStrWidth(pressureBuf);
        u8g2.drawStr(371 - pressWidth, 26, pressureBuf);
        
        u8g2.drawStr(56, 162, voltBuf);
        u8g2.drawStr(169, 162, curBuf);
        u8g2.drawStr(280, 162, uptimeBuf);
        
        sendBufferDirect();
    } else {
        u8g2.setDrawColor(0); 
        u8g2.clearBuffer();
        u8g2.setDrawColor(1);
        u8g2.setFont(u8g2_font_logisoso62_tn);
        u8g2.drawStr(236, 126, secsBuf);

        convertBuffer();
        sendPartialBuffer();
    }
}

// --- UPDATED MENU DRAWING FUNCTION ---
void drawMenu() {
    u8g2.clearBuffer();
    u8g2.setFontMode(1);
    u8g2.setBitmapMode(1);
    u8g2.setDrawColor(1);
    u8g2.setFont(u8g2_font_logisoso16_tr);
    
    // Adjusted spacing to 22px so all 7 items fit properly on the 164px screen height
    for(int i = 0; i < NUM_MENU_ITEMS; i++) {
        int y = 22 + i * 22;
        u8g2.drawStr(40, y, menuItems[i]);
        if(i == menuSelection) {
            u8g2.drawFrame(30, y - 18, 220, 24);
        }
    }
    sendBufferDirect();
}

// --- NEW SET TIME DRAWING FUNCTION ---
void drawSetTime() {
    u8g2.clearBuffer();
    u8g2.setFontMode(1);
    u8g2.setBitmapMode(1);
    u8g2.setDrawColor(1);
    
    // Header
    u8g2.setFont(u8g2_font_logisoso16_tr);
    u8g2.drawStr(100, 30, "Set System Time");
    
    // Time Output
    char timeBuf[16];
    snprintf(timeBuf, sizeof(timeBuf), "%02d:%02d", tempHour, tempMinute);
    
    u8g2.setFont(u8g2_font_logisoso62_tn);
    int timeWidth = u8g2.getStrWidth(timeBuf);
    int xOffset = (384 - timeWidth) / 2; // Center it on the screen
    u8g2.drawStr(xOffset, 110, timeBuf);
    
    // Draw underline indicator based on active selection (Hour vs Minute)
    // Approximate coordinate logic for typical logisoso font spacing
    if (!editingMinutes) {
        u8g2.drawBox(xOffset, 120, 75, 5); // Under Hours
    } else {
        u8g2.drawBox(xOffset + 95, 120, 75, 5); // Under Minutes
    }
    
    sendBufferDirect();
}

void drawChart(int chartIndex) {
    u8g2.clearBuffer();
    u8g2.setFontMode(1);
    u8g2.setBitmapMode(1);
    u8g2.setDrawColor(1);
    u8g2.setFont(u8g2_font_logisoso16_tr);

    float minV = 999999.0f;
    float maxV = -999999.0f;
    float* dataArr = histTemp;
    
    if(chartIndex == 1) dataArr = histRH;
    else if(chartIndex == 2) dataArr = histAbsHum;
    else if(chartIndex == 3) dataArr = histPress;
    else if(chartIndex == 4) dataArr = histVolt;
    else if(chartIndex == 5) dataArr = histCur;

    if (histCount == 0) {
        u8g2.drawStr(100, 80, "No Data Yet");
        sendBufferDirect();
        return;
    }

    for(int i = 0; i < histCount; i++) {
        if(dataArr[i] < minV) minV = dataArr[i];
        if(dataArr[i] > maxV) maxV = dataArr[i];
    }
    if(maxV - minV < 0.001f) { 
        maxV += 1.0f;
        minV -= 1.0f;
    }

    u8g2.drawBox(65, 10, 2, 132); 
    u8g2.drawBox(65, 140, 305, 2); 

    char buf[16];
    snprintf(buf, sizeof(buf), "%.1f", maxV);
    u8g2.drawStr(0, 20, buf);
    snprintf(buf, sizeof(buf), "%.1f", minV);
    u8g2.drawStr(0, 140, buf);
    
    u8g2.drawStr(100, 162, menuItems[chartIndex]);
    snprintf(buf, sizeof(buf), "Samples:%d", histCount);
    u8g2.drawStr(240, 162, buf);

    int prevX = -1, prevY = -1;
    for(int i = 0; i < histCount; i++) {
        int idx = (histCount < MAX_SAMPLES) ? i : (histHead + i) % MAX_SAMPLES;
        float val = dataArr[idx];
        
        int x = 69 + (i * 297) / (histCount > 1 ? histCount - 1 : 1);
        int y = 138 - ((val - minV) * 126 / (maxV - minV));
        
        if(prevX != -1) {
            u8g2.drawLine(prevX, prevY, x, y);
        }
        prevX = x;
        prevY = y;
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
    digitalWrite(PWR_SENSORS, HIGH);
    delay(100);
    Wire.setSCL(I2C1_SCL); 
    Wire.setSDA(I2C1_SDA); 
    Wire.begin();
    Wire.setClock(100000);
    if (ds3231.begin(&Wire)) {
        DateTime dt = ds3231.now();
        stmRtc.setEpoch(dt.unixtime());
        now = dt.unixtime();
    }

    if(!ina219.init()){ Serial.println("INA219 not connected!"); }
    ina219.setShuntSizeInOhms(1.0);
    ina219.setBusRange(INA219_BRNG_16);
    ina219.setPGain(INA219_PG_40);
    ina219.setADCMode(INA219_SAMPLE_MODE_128);
    ina219.setMeasureMode(INA219_TRIGGERED);
    ina219.startSingleMeasurementNoWait();
    
    aht.begin();
    bmp.begin();
    bmp.setSampling(Adafruit_BMP280::MODE_FORCED, Adafruit_BMP280::SAMPLING_X2, Adafruit_BMP280::SAMPLING_X16, Adafruit_BMP280::FILTER_X16, Adafruit_BMP280::STANDBY_MS_500);

    sensors_event_t humEvent, tempEvent;
    aht.getEvent(&humEvent, &tempEvent);
    gTemperature = tempEvent.temperature;
    gHumidity = humEvent.relative_humidity;
    bmp.takeForcedMeasurement();
    gPressure = bmp.readPressure() / 100.0f;

    float Temp = gTemperature;
    float Humid = gHumidity;
    gAbsHumidity = (6.112f * expf(((17.67f * Temp) / (Temp + 243.5f))) * Humid * 2.1674f) / (273.15f + Temp);
    
    uint32_t inaTimeout = millis();
    while (!ina219.getConversionReady() && (millis() - inaTimeout < 100)) delay(1);
    
    gBatteryVoltage = ina219.getBusVoltage_V();
    float current_mA = ina219.getCurrent_mA();
    if (current_mA < 0.0f) current_mA = 0.0f;
    gBatteryCurrent_uA = (uint32_t)(current_mA * 1000.0f + 0.5f);
    
    Wire.end();
    delay(50); 
    digitalWrite(PWR_SENSORS, LOW);
}

volatile bool btnWakeFlag = false;

void buttonISR() {
    btnWakeFlag = true;
}

bool isPressed(uint32_t pin) {
    return digitalRead(pin) == LOW; 
}

void setup() {
    Wire.setSCL(I2C1_SCL);
    Wire.setSDA(I2C1_SDA);

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

    digitalWrite(PWR_SENSORS, HIGH);
    delay(10); 
    Wire.begin();
    Wire.setClock(100000);
    if (ds3231.begin(&Wire)) {
        if (ds3231.lostPower()) {
            ds3231.adjust(DateTime(F(__DATE__), F(__TIME__)) + TimeSpan(22)); 
        }
        uint32_t dsEpoch = ds3231.now().unixtime();
        stmRtc.setEpoch(dsEpoch);
    }

    if(!ina219.init()){ Serial.println("INA219 not connected!"); }
    ina219.setShuntSizeInOhms(1.0);
    ina219.setBusRange(INA219_BRNG_16);
    ina219.setPGain(INA219_PG_40);
    ina219.setADCMode(INA219_SAMPLE_MODE_128);
    ina219.setMeasureMode(INA219_TRIGGERED);
    ina219.startSingleMeasurementNoWait();
    
    aht.begin();
    bmp.begin();
    bmp.setSampling(Adafruit_BMP280::MODE_FORCED, Adafruit_BMP280::SAMPLING_X2, Adafruit_BMP280::SAMPLING_X16, Adafruit_BMP280::FILTER_X16, Adafruit_BMP280::STANDBY_MS_500);

    sensors_event_t humEvent, tempEvent;
    aht.getEvent(&humEvent, &tempEvent);
    gTemperature = tempEvent.temperature;
    gHumidity = humEvent.relative_humidity;
    bmp.takeForcedMeasurement();
    gPressure = bmp.readPressure() / 100.0f;

    float Temp = gTemperature;
    float Humid = gHumidity;
    gAbsHumidity = (6.112f * expf(((17.67f * Temp) / (Temp + 243.5f))) * Humid * 2.1674f) / (273.15f + Temp);
    
    uint32_t inaTimeout = millis();
    while (!ina219.getConversionReady() && (millis() - inaTimeout < 100)) delay(1);
    
    gBatteryVoltage = ina219.getBusVoltage_V();
    float current_mA = ina219.getCurrent_mA();
    if (current_mA < 0.0f) current_mA = 0.0f;
    gBatteryCurrent_uA = (uint32_t)(current_mA * 1000.0f + 0.5f);
    
    Wire.end();
    delay(50); 
    digitalWrite(PWR_SENSORS, LOW);

    bootEpoch = stmRtc.getEpoch();
    lastSensorRead = bootEpoch;

    pinMode(PIN_LCD_TE, INPUT_ANALOG);
    pinMode(PIN_LCD_CS,  OUTPUT); digitalWrite(PIN_LCD_CS,  HIGH);
    pinMode(PIN_LCD_DC,  OUTPUT); digitalWrite(PIN_LCD_DC,  HIGH);
    pinMode(PIN_LCD_RST, OUTPUT); digitalWrite(PIN_LCD_RST, HIGH);

    SPI.setSCLK(PIN_LCD_SCLK);
    SPI.setMOSI(PIN_LCD_MOSI);
    u8g2.begin();
    u8g2.setBusClock(12000000);

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin       = GPIO_PIN_5 | GPIO_PIN_7;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    GPIO_InitStruct.Pin       = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    u8g2.sendF("c", 0x39);
    u8g2.sendF("ca", 0xB2, 0x00);
    
    forceMainRedraw = true;
    paintMainScreen(stmRtc.getEpoch());
    
    sleepPins();
    HAL_DBGMCU_DisableDBGStopMode();
    LowPower.begin();
    stmRtc.attachInterrupt(alarmMatch); 
}

void loop() {
    uint32_t now = stmRtc.getEpoch();
    
    static uint32_t lastDisplayTick = 0;
    static uint32_t lastInteractionTime = 0;
    
    static bool lastUp = false;
    static bool lastDown = false;
    static bool lastLeft = false;
    static bool lastRight = false;
    
    if (lastDisplayTick == 0) lastDisplayTick = now;

    // --- 1. Process 1-Second Updates ---
    if (now != lastDisplayTick) {
        lastDisplayTick = now;
        
        if (now - lastSensorRead >= 60) {
            lastSensorRead = now;

            // Push the current global readings into the circular buffer
            histTemp[histHead]   = gTemperature;
            histRH[histHead]     = gHumidity;
            histAbsHum[histHead] = gAbsHumidity;
            histPress[histHead]  = gPressure;
            histVolt[histHead]   = gBatteryVoltage;
            histCur[histHead]    = (float)gBatteryCurrent_uA;

            // Advance the head index and wrap around if necessary
            histHead = (histHead + 1) % MAX_SAMPLES;
            
            // Increment the total sample count until it fills the buffer
            if (histCount < MAX_SAMPLES) {
                histCount++;
            }
        }

        if (currentState == STATE_MAIN) {
            paintMainScreen(now);
        }
    }

    // --- 2. Button Edge Detection ---
    bool currUp = isPressed(BTN_UP);
    bool currDown = isPressed(BTN_DOWN);
    bool currLeft = isPressed(BTN_LEFT);
    bool currRight = isPressed(BTN_RIGHT);

    bool pressUp    = currUp && !lastUp;
    bool pressDown  = currDown && !lastDown;
    bool pressLeft  = currLeft && !lastLeft;
    bool pressRight = currRight && !lastRight;

    lastUp = currUp;
    lastDown = currDown;
    lastLeft = currLeft;
    lastRight = currRight;

    // --- 3. UI Navigation ---
    if (currentState == STATE_MAIN) {
        if (pressRight) {
            currentState = STATE_MENU;
            menuSelection = 0;
            lastInteractionTime = millis();
            drawMenu();
        }
    } 
    else if (currentState == STATE_MENU) {
        if (pressUp || pressDown || pressLeft || pressRight) {
            lastInteractionTime = millis(); 
            
            if (pressUp) {
                menuSelection = (menuSelection == 0) ? NUM_MENU_ITEMS - 1 : menuSelection - 1;
                drawMenu();
            } else if (pressDown) {
                menuSelection = (menuSelection == NUM_MENU_ITEMS - 1) ? 0 : menuSelection + 1;
                drawMenu();
            } else if (pressLeft) {
                currentState = STATE_MAIN;
                forceMainRedraw = true;  
                paintMainScreen(now);
                lastDisplayTick = now;
            } else if (pressRight) {
                if (menuSelection == 6) { // "Set Time" Sub-menu
                    currentState = STATE_SET_TIME;
                    editingMinutes = false;
                    tempHour = (now % 86400) / 3600;
                    tempMinute = (now % 3600) / 60;
                    drawSetTime();
                } else {
                    currentState = STATE_CHART;
                    drawChart(menuSelection);
                }
            }
        }
    } 
    // --- TIME SETTING STATE CONTROLS ---
    else if (currentState == STATE_SET_TIME) {
        if (pressUp || pressDown || pressLeft || pressRight) {
            lastInteractionTime = millis();
            
            if (pressLeft) { // Cancel and go back
                currentState = STATE_MENU;
                drawMenu();
            } else if (pressRight) {
                if (!editingMinutes) {
                    editingMinutes = true; // Switch context to editing minutes
                    drawSetTime();
                } else {
                    // Right arrow again: Save time and return to Main
                    digitalWrite(PWR_SENSORS, HIGH);
                    delay(100);
                    Wire.setSCL(I2C1_SCL); 
                    Wire.setSDA(I2C1_SDA); 
                    Wire.begin();
                    Wire.setClock(100000);
                    
                    if (ds3231.begin(&Wire)) {
                        DateTime dt = ds3231.now();
                        DateTime newTime(dt.year(), dt.month(), dt.day(), tempHour, tempMinute, 0);
                        ds3231.adjust(newTime);
                        stmRtc.setEpoch(newTime.unixtime());
                    }
                    
                    Wire.end();
                    digitalWrite(PWR_SENSORS, LOW);
                    
                    currentState = STATE_MAIN;
                    forceMainRedraw = true;  
                    now = stmRtc.getEpoch(); // Get immediately updated time
                    paintMainScreen(now);
                    lastDisplayTick = now;
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
        if (pressLeft) {
            lastInteractionTime = millis();
            currentState = STATE_MENU;
            drawMenu();
        } else if (pressUp || pressDown || pressRight) {
            currentState = STATE_MAIN;
            forceMainRedraw = true;  
            paintMainScreen(now);
            lastDisplayTick = now;
        }
    }

    // --- 4. Auto-Exit Menu (Battery Protection) ---
    if (currentState != STATE_MAIN && (millis() - lastInteractionTime > 15000)) {
        currentState = STATE_MAIN;
        forceMainRedraw = true;      
        paintMainScreen(now);
        lastDisplayTick = now;
    }

    // --- 5. Power Management ---
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
        delay(50); 
    }
}