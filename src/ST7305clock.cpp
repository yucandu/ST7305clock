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
#define BTN_UP PB1
#define BTN_DOWN PB3
#define BTN_RIGHT PB4

#define SERIAL_BAUD     115200

// --- NEW APP STATE VARIABLES ---
enum AppState { STATE_MAIN, STATE_MENU, STATE_CHART };
AppState currentState = STATE_MAIN;
int8_t menuSelection = 0;
const char* menuItems[6] = {"Temperature", "Rel Humidity", "Abs Humidity", "Pressure", "Voltage", "Current"};

// --- NEW DATA ARRAYS (400 points = ~9.6KB RAM) ---
#define MAX_SAMPLES 200
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
    // Adjusted bounding box perfectly aligned to the ST7305 12x2 grid.
    // This perfectly envelops the new drawBox(226, 36, 90, 72).
    uint8_t devXStart = 60, devXEnd = 120; // Safely covers U8g2 Y: 36 to 119
    uint8_t devYStart = 68, devYEnd = 158; // Safely covers U8g2 X: 226 to 316

    uint8_t casetStart = devXStart / 12; // Maps to hardware block 3
    uint8_t casetEnd   = devXEnd   / 12; // Maps to hardware block 9
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
    
    // Send only the exact number of bytes framed by the new window (~966 bytes)
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
        // --- Full Screen Refresh ---
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

        // Dedicated buffers to handle split layout logic
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

        // Formatting raw strings
        snprintf(tempBuf, sizeof(tempBuf), "%s%d.%d\xc2\xb0" "C", t_sign, t_int, t_frac);
        snprintf(humBuf, sizeof(humBuf), "%d%% %d.%dg", h_int, ah_int, ah_frac);
        snprintf(pressureBuf, sizeof(pressureBuf), "%d.%dmB", p_int, p_frac);
        snprintf(voltBuf, sizeof(voltBuf), "%d.%03dv", v_int, v_frac);
        snprintf(curBuf, sizeof(curBuf), "%luuA", (unsigned long)gBatteryCurrent_uA);
        
        // Uptime (Removed the "Up>" prefix so it aligns cleanly with your image's hourglass icon)
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
        
        // Draw primary layout
        drawRleBackground(0, 0, 384, 164, epd_bitmap_rle);
        
        // Dynamic AM/PM masking - Draw a white box over the incorrect permanent text
        u8g2.setDrawColor(0); // Assuming 0 is the white background color in your display logic
        if (pm) {
            u8g2.drawBox(320, 42, 50, 32); // Cover AM Text
        } else {
            u8g2.drawBox(320, 74, 50, 31); // Cover PM Text
        }

        // Return to standard text drawing color
        u8g2.setDrawColor(1); 
        
        // --- Large Clock Font ---
        u8g2.setFont(u8g2_font_logisoso62_tn);
        
        // Hours/Minutes/Colons: Right-justified flush with X=223
        int timeWidth = u8g2.getStrWidth(timeMainBuf);
        u8g2.drawStr(223 - timeWidth, 102, timeMainBuf);
        
        // Seconds
        u8g2.drawStr(236, 102, secsBuf); 
        
        // --- Small Details Font ---
        u8g2.setFont(u8g2_font_logisoso16_tr);
        
        // Top Banner Sensors
        u8g2.drawStr(48, 26, tempBuf);
        u8g2.drawStr(146, 26, humBuf);
        
        // Pressure: Right-justified flush with X=371
        int pressWidth = u8g2.getStrWidth(pressureBuf);
        u8g2.drawStr(371 - pressWidth, 26, pressureBuf);
        
        // Bottom Banner System Info
        u8g2.drawStr(56, 162, voltBuf);
        u8g2.drawStr(169, 162, curBuf);
        u8g2.drawStr(280, 162, uptimeBuf);
        
        sendBufferDirect();
    } else {
// --- Partial Screen Refresh (Seconds Only) ---
        // Generous bounding box to wipe all variable-width/height characters
        // Logical X covers 226 to 316. Logical Y covers 36 to 108.
        u8g2.setDrawColor(0); 
        u8g2.clearBuffer();
        //u8g2.drawBox(226, 36, 90, 72); 
        
        u8g2.setDrawColor(1);
        u8g2.setFont(u8g2_font_logisoso62_tn);
        u8g2.drawStr(236, 126, secsBuf);

        convertBuffer();
        sendPartialBuffer();
    }
}

// --- NEW MENU DRAWING FUNCTION ---
void drawMenu() {
    u8g2.clearBuffer();
    u8g2.setFontMode(1);
    u8g2.setBitmapMode(1);
    u8g2.setDrawColor(1);
    u8g2.setFont(u8g2_font_logisoso16_tr);
    for(int i = 0; i < 6; i++) {
        int y = 24 + i * 26;
        u8g2.drawStr(40, y, menuItems[i]);
        if(i == menuSelection) {
            u8g2.drawFrame(30, y - 20, 220, 26);
        }
    }
    sendBufferDirect();
}

// --- NEW CHART DRAWING FUNCTION ---
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
    if(maxV - minV < 0.001f) { // Prevent division by zero
        maxV += 1.0f;
        minV -= 1.0f;
    }

    // 2px thick axes
    u8g2.drawBox(65, 10, 2, 132); // Y axis
    u8g2.drawBox(65, 140, 305, 2); // X axis

    // Labels
    char buf[16];
    snprintf(buf, sizeof(buf), "%.1f", maxV);
    u8g2.drawStr(0, 20, buf);
    snprintf(buf, sizeof(buf), "%.1f", minV);
    u8g2.drawStr(0, 140, buf);
    
    u8g2.drawStr(100, 162, menuItems[chartIndex]);
    snprintf(buf, sizeof(buf), "Samples:%d", histCount);
    u8g2.drawStr(240, 162, buf);

    // 1px thick data line
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
    // Keep CS and RST HIGH to prevent display resets/triggers
    digitalWrite(PIN_LCD_CS,  HIGH);
    digitalWrite(PIN_LCD_RST, HIGH);
    
    // Keep DC LOW
    digitalWrite(PIN_LCD_DC, LOW);

    // Drive SCLK and MOSI LOW to prevent shoot-through on the ST7305
    GPIO_InitTypeDef g = {0};
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    
    g.Pin   = GPIO_PIN_5 | GPIO_PIN_7; 
    HAL_GPIO_Init(GPIOA, &g);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5 | GPIO_PIN_7, GPIO_PIN_RESET);

    // I2C Pins to Analog (Safe since pull-ups are on the switched rail)
    g.Mode  = GPIO_MODE_ANALOG;
    g.Pin   = GPIO_PIN_6 | GPIO_PIN_7;
    HAL_GPIO_Init(GPIOB, &g);
    
}

void wakePins() {
    GPIO_InitTypeDef g = {0};
    g.Pin       = GPIO_PIN_5 | GPIO_PIN_7;
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_NOPULL;
    g.Speed     = GPIO_SPEED_FREQ_LOW;
    g.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &g);
    
    // I2C pin config removed to prevent mapping pins to a disabled peripheral clock
}

//HardwareSerial DebugSerial(PA10, PA9);

void readAllSensors(uint32_t &now) {
    digitalWrite(PWR_SENSORS, HIGH);
    delay(100);
    Wire.setSCL(I2C1_SCL); // Re-map SCL
    Wire.setSDA(I2C1_SDA); // Re-map SDA
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
    delay(50); // Short delay to ensure all I2C transactions are fully completed before cutting power
    digitalWrite(PWR_SENSORS, LOW);
}

volatile bool btnWakeFlag = false;

void buttonISR() {
    btnWakeFlag = true;
}

// Helper to poll button states cleanly
bool isPressed(uint32_t pin) {
    return digitalRead(pin) == LOW; // Assumes internal pull-ups pulling to GND when pressed
}

void setup() {
    //DebugSerial.begin(115200);
    //DebugSerial.println("Setup start");
    Wire.setSCL(I2C1_SCL);
    Wire.setSDA(I2C1_SDA);

// --- FIX FLOATING PINS (Prevents 200uA -> 400uA leak) ---
    pinMode(PA10, INPUT_ANALOG); // UART RX (Floating when unplugged)
    pinMode(PA9,  INPUT_ANALOG); // UART TX 
    pinMode(PA6,  INPUT_ANALOG); // MISO (Unused)
    // --- REMAINING UNUSED PINS -> ANALOG ---
    pinMode(PA8,  INPUT_ANALOG);
    pinMode(PA11, INPUT_ANALOG);
    pinMode(PA12, INPUT_ANALOG);
    pinMode(PB5,  INPUT_ANALOG);

    // 1. Enable the GPIOA clock (it must be on to configure the pins)
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
    
    // 2. Set the MODER bits for PA13 (bits 26,27) and PA14 (bits 28,29) to 0b11 (Analog Mode)
    // 0b11 is the bit pattern for Analog mode on STM32
    GPIOA->MODER |= (GPIO_MODER_MODE13 | GPIO_MODER_MODE14);
    pinMode(PA_15, INPUT_ANALOG); // JTDI

    // --- LSE OSCILLATOR PINS ---
    // Since you are using the internal MSI clock and an external DS3231 RTC, 
    // you likely do not have a 32.768kHz crystal physically soldered to these pins.
    // If they are bare pads, set them to analog to prevent floating inputs.
    pinMode(PC14, INPUT_ANALOG); 
    pinMode(PC15, INPUT_ANALOG);
    // --- SETUP BUTTON INPUTS ---
    // Changed to INPUT since you have external pullups (saves wasting parallel resistance)
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
            ds3231.adjust(DateTime(F(__DATE__), F(__TIME__)) + TimeSpan(23)); //add 23 seconds to compensate for compile time to upload time delay
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
    delay(50); // Short delay to ensure all I2C transactions are fully completed before cutting power
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
    uint32_t seconds = bootEpoch % 60;
    char secsBuf[4];
    snprintf(secsBuf, sizeof(secsBuf), "%02u", seconds);
    uint32_t uptimeSecs = 0;
        uint32_t uptimeDays  = uptimeSecs / 86400;
        uint32_t uptimeHours = (uptimeSecs % 86400) / 3600;
        uint32_t uptimeMins  = (uptimeSecs % 3600) / 60;
        uint32_t hours24 = (bootEpoch % 86400) / 3600;
        uint32_t minutes = (bootEpoch % 3600) / 60;

        
        uint8_t h12 = hours24 % 12;
        if (h12 == 0) h12 = 12;
        bool pm = hours24 >= 12;

        // Dedicated buffers to handle split layout logic
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

        // Formatting raw strings
        snprintf(tempBuf, sizeof(tempBuf), "%s%d.%d\xc2\xb0" "C", t_sign, t_int, t_frac);
        snprintf(humBuf, sizeof(humBuf), "%d%% %d.%dg", h_int, ah_int, ah_frac);
        snprintf(pressureBuf, sizeof(pressureBuf), "%d.%dmB", p_int, p_frac);
        snprintf(voltBuf, sizeof(voltBuf), "%d.%03dv", v_int, v_frac);
        snprintf(curBuf, sizeof(curBuf), "%luuA", (unsigned long)gBatteryCurrent_uA);
        
        // Uptime (Removed the "Up>" prefix so it aligns cleanly with your image's hourglass icon)
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
        
        // Draw primary layout
        drawRleBackground(0, 0, 384, 164, epd_bitmap_rle);
        
        // Dynamic AM/PM masking - Draw a white box over the incorrect permanent text
        u8g2.setDrawColor(0); // Assuming 0 is the white background color in your display logic
        if (pm) {
            u8g2.drawBox(320, 42, 50, 32); // Cover AM Text
        } else {
            u8g2.drawBox(320, 74, 50, 31); // Cover PM Text
        }

        // Return to standard text drawing color
        u8g2.setDrawColor(1); 
        
        // --- Large Clock Font ---
        u8g2.setFont(u8g2_font_logisoso62_tn);
        
        // Hours/Minutes/Colons: Right-justified flush with X=223
        int timeWidth = u8g2.getStrWidth(timeMainBuf);
        u8g2.drawStr(223 - timeWidth, 102, timeMainBuf);
        
        // Seconds
        u8g2.drawStr(236, 102, secsBuf); 
        
        // --- Small Details Font ---
        u8g2.setFont(u8g2_font_logisoso16_tr);
        
        // Top Banner Sensors
        u8g2.drawStr(48, 26, tempBuf);
        u8g2.drawStr(146, 26, humBuf);
        
        // Pressure: Right-justified flush with X=371
        int pressWidth = u8g2.getStrWidth(pressureBuf);
        u8g2.drawStr(371 - pressWidth, 26, pressureBuf);
        
        // Bottom Banner System Info
        u8g2.drawStr(56, 162, voltBuf);
        u8g2.drawStr(169, 162, curBuf);
        u8g2.drawStr(280, 162, uptimeBuf);
        
        sendBufferDirect();
    sleepPins();
    HAL_DBGMCU_DisableDBGStopMode();
    LowPower.begin();
    stmRtc.attachInterrupt(alarmMatch); 

}

void loop() {
   // wakePins();
    uint32_t now = stmRtc.getEpoch();
    
    static uint32_t lastDisplayTick = 0;
    static uint32_t lastInteractionTime = 0;
    
    // State trackers for edge detection
    static bool lastUp = false;
    static bool lastDown = false;
    static bool lastLeft = false;
    static bool lastRight = false;
    
    if (lastDisplayTick == 0) lastDisplayTick = now;

    // --- 1. Process 1-Second Updates ---
    if (now != lastDisplayTick) {
        lastDisplayTick = now;
        
        if (now - lastSensorRead >= 60) {
            //readAllSensors(now);
            lastSensorRead = now;
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
                menuSelection = (menuSelection == 0) ? 5 : menuSelection - 1;
                drawMenu();
            } else if (pressDown) {
                menuSelection = (menuSelection == 5) ? 0 : menuSelection + 1;
                drawMenu();
            } else if (pressLeft) {
                currentState = STATE_MAIN;
                forceMainRedraw = true;  // <-- Force a complete screen wipe
                paintMainScreen(now);
                lastDisplayTick = now;
            } else if (pressRight) {
                currentState = STATE_CHART;
                drawChart(menuSelection);
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
            forceMainRedraw = true;  // <-- Force a complete screen wipe
            paintMainScreen(now);
            lastDisplayTick = now;
        }
    }

    // --- 4. Auto-Exit Menu (Battery Protection) ---
    if (currentState != STATE_MAIN && (millis() - lastInteractionTime > 15000)) {
        currentState = STATE_MAIN;
        forceMainRedraw = true;      // <-- Force a complete screen wipe
        paintMainScreen(now);
        lastDisplayTick = now;
    }

    // --- 5. Power Management ---
// --- 5. Power Management ---
    if (currentState == STATE_MAIN) {
        SPI.end(); // Unhook the SPI peripheral 
        sleepPins(); // Clamp the pins manually
        
        uint32_t targetAlarm = stmRtc.getEpoch() + 1;
        stmRtc.setAlarmEpoch(targetAlarm);
        
        if (stmRtc.getEpoch() >= targetAlarm) {
            targetAlarm = stmRtc.getEpoch() + 1;
            stmRtc.setAlarmEpoch(targetAlarm);
        }
        
        LowPower.deepSleep(); 
        
        wakePins(); // Reassign pins to Alternate Function
        SPI.begin(); // Reinitialize SPI block
    } else {
        delay(50); 
    }
}