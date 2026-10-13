#ifndef CLIMATE_TFT_USER_SETUP_H
#define CLIMATE_TFT_USER_SETUP_H
#define USER_SETUP_INFO "ClimateCamera_ST7789_240_v1.2.0"
#define ST7789_DRIVER
#define TFT_WIDTH 240
#define TFT_HEIGHT 240
#define TFT_MOSI 23
#define TFT_SCLK 18
#define TFT_CS -1
#define TFT_DC 25
#define TFT_RST 13
#define TFT_MISO -1
#define SPI_FREQUENCY 20000000
// Если панель требует другой инверсии, настройте только один параметр ниже.
// #define TFT_INVERSION_ON
// #define TFT_RGB_ORDER TFT_BGR
#define LOAD_GLCD
#define LOAD_FONT2
#define LOAD_FONT4
#define LOAD_FONT6
#define LOAD_FONT7
#define LOAD_FONT8
#define LOAD_GFXFF
#define SMOOTH_FONT
#endif
