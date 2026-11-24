#pragma once
#include <U8g2lib.h>

static const unsigned char image_bottomLine_bits[] U8X8_PROGMEM = {0x19, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x26, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x1f};
static const unsigned char image_bticon1_bits[] U8X8_PROGMEM = {0x80, 0x00, 0x40, 0x01, 0x40, 0x02, 0x44, 0x04, 0x48, 0x04, 0x52, 0x12, 0x64, 0x09, 0xcc, 0x0c, 0x64, 0x09, 0x52, 0x12, 0x48, 0x04, 0x44, 0x04, 0x40, 0x02, 0x40, 0x01, 0x80, 0x00, 0x00, 0x00};
static const unsigned char image_download__copy___copy___copy__bits[] U8X8_PROGMEM = {0x03, 0x02, 0x00, 0x02, 0x06, 0x0e, 0x1e, 0x0e, 0x06, 0x02, 0x00, 0x02, 0x03};
static const unsigned char image_left_pilar_bits[] U8X8_PROGMEM = {0x3c, 0x02, 0x19, 0x25, 0x25, 0x19, 0x01, 0x01, 0x05, 0x09, 0x11, 0x25, 0x09, 0x11, 0x25, 0x09, 0x11, 0x25, 0x09, 0x11, 0x25, 0x09, 0x11, 0x25, 0x09, 0x11, 0x21, 0x01, 0x02, 0x06, 0x0e, 0x1e, 0x3e, 0x3a, 0x32, 0x32, 0x32, 0x32, 0x32, 0x32, 0x32, 0x32, 0x32, 0x32, 0x32, 0x32};
static const unsigned char image_right_pilar_bits[] U8X8_PROGMEM = {0x0f, 0x10, 0x26, 0x29, 0x29, 0x26, 0x20, 0x20, 0x28, 0x24, 0x22, 0x29, 0x24, 0x22, 0x29, 0x24, 0x22, 0x29, 0x24, 0x22, 0x29, 0x24, 0x22, 0x29, 0x24, 0x22, 0x21, 0x20, 0x10, 0x18, 0x1c, 0x1e, 0x1f, 0x17, 0x13, 0x13, 0x13, 0x13, 0x13, 0x13, 0x13, 0x13, 0x13, 0x13, 0x13, 0x13};
static const unsigned char image_wifiicon_bits[] U8X8_PROGMEM = {0x80, 0x0f, 0x00, 0x60, 0x30, 0x00, 0x18, 0xc0, 0x00, 0x84, 0x0f, 0x01, 0x62, 0x30, 0x02, 0x11, 0x40, 0x04, 0x08, 0x87, 0x00, 0xc4, 0x18, 0x01, 0x20, 0x20, 0x00, 0x10, 0x42, 0x00, 0x80, 0x0d, 0x00, 0x40, 0x10, 0x00, 0x00, 0x02, 0x00, 0x00, 0x05, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00};
// U8G2_NULL u8g2(U8G2_R0);	// null device, a 8x8 pixel display which does nothing
// U8G2_SSD1306_128X64_NONAME_F_4W_SW_SPI u8g2(U8G2_R0, /* clock=*/ 13, /* data=*/ 11, /* cs=*/ 10, /* dc=*/ 9, /* reset=*/ 8);
// U8G2_SSD1306_128X64_NONAME_F_4W_HW_SPI u8g2(U8G2_R0, /* cs=*/ 12, /* dc=*/ 4, /* reset=*/ 6);	// Arduboy (Production, Kickstarter Edition)
// U8G2_SSD1306_128X64_NONAME_F_4W_HW_SPI u8g2(U8G2_R0, /* cs=*/ 10, /* dc=*/ 9, /* reset=*/ 8);
// U8G2_SSD1306_128X64_NONAME_F_3W_SW_SPI u8g2(U8G2_R0, /* clock=*/ 13, /* data=*/ 11, /* cs=*/ 10, /* reset=*/ 8);
// U8G2_SSD1306_128X64_NONAME_F_3W_HW_SPI u8g2(U8G2_R0, /* cs=*/ 10, /* reset=*/ 8);
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE);

struct
{
    void bootscreen(void)
    {
        u8g2.setFontMode(1);
        u8g2.setBitmapMode(1);
        // Layer 1
        u8g2.setFont(u8g2_font_t0_18b_tr);
        u8g2.drawStr(10, 36, "SolarTracker");
        // Layer 2
        u8g2.setFont(u8g2_font_profont10_tr);
        u8g2.drawStr(49, 47, "v1.0.0");
    }
    static void drawSolar(void)
    {
        u8g2.setFontMode(1);
        u8g2.setBitmapMode(1);
        // Layer 2
        u8g2.setDrawColor(2);
        u8g2.setFont(u8g2_font_profont17_tr);
        u8g2.drawStr(44, 61, "12.345A");
        // wifiicon
        u8g2.setDrawColor(1);
        u8g2.drawXBMP(109, 0, 19, 16, image_wifiicon_bits);
        // bticon1
        u8g2.drawXBMP(94, 0, 14, 16, image_bticon1_bits);
        // left-pilar
        u8g2.drawXBMP(0, 16, 6, 46, image_left_pilar_bits);
        // Layer 2 (copy)
        u8g2.setDrawColor(2);
        u8g2.drawStr(53, 46, "0.980V");
        // right-pilar
        u8g2.setDrawColor(1);
        u8g2.drawXBMP(122, 16, 6, 46, image_right_pilar_bits);
        // Layer 9
        u8g2.drawLine(6, 16, 123, 16);
        // Layer 2 (copy)
        u8g2.setDrawColor(2);
        u8g2.drawStr(44, 31, "966.6W");
        // Layer 11
        u8g2.setFont(u8g2_font_profont22_tr);
        u8g2.drawStr(0, 14, "SOLAR");
        // bottomLine
        u8g2.setDrawColor(1);
        u8g2.drawXBMP(1, 62, 126, 2, image_bottomLine_bits);
        // download (copy) (copy) (copy)
        u8g2.drawXBMP(35, 19, 5, 13, image_download__copy___copy___copy__bits);
        // Layer 17 (copy) (copy)
        u8g2.setDrawColor(2);
        u8g2.drawBox(7, 19, 28, 13);
        // Layer 16 (copy) (copy) (copy) (copy) (copy)
        u8g2.setFont(u8g2_font_profont17_tr);
        u8g2.drawStr(8, 61, "CUR");
        // Layer 16 (copy) (copy) (copy) (copy) (copy)
        u8g2.drawStr(8, 46, "VOLT");
        // Layer 17 (copy) (copy)
        u8g2.drawBox(7, 34, 37, 13);
        // download (copy) (copy) (copy)
        u8g2.setDrawColor(1);
        u8g2.drawXBMP(35, 49, 5, 13, image_download__copy___copy___copy__bits);
        // arrowxd2
        u8g2.setDrawColor(2);
        u8g2.drawStr(8, 31, "PWR");
        // download (copy) (copy)
        u8g2.setDrawColor(1);
        u8g2.drawXBMP(44, 34, 5, 13, image_download__copy___copy___copy__bits);
        // Layer 21 (copy)
        u8g2.drawLine(119, 19, 119, 61);
        // Layer 17 (copy)
        u8g2.setDrawColor(2);
        u8g2.drawBox(7, 49, 28, 13);
    }

    void begin()
    {
        // Inisialisasi  I2C
        Serial.println("Initializing I2C buses...");
        Wire.begin(21, 22); // SDA, SCL
        delay(100);
        u8g2.begin();
        delay(100);
        u8g2.clearBuffer();                  // clear the internal memory
        u8g2.setFont(u8g2_font_ncenB08_tr);  // choose a suitable font
        u8g2.drawStr(0, 10, "Hello World!"); // write something to the internal memory
        u8g2.sendBuffer();                   // transfer internal memory to the display
        bootscreen();
        delay(100);
    }
} oled;