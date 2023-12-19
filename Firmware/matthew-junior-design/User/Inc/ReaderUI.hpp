#pragma once        // Include guard

#include "ST7789.hpp"
#include "logger.hpp"

extern "C" SPI_HandleTypeDef hspi1;

#define LCD_SPI         &hspi1
#define LCD_CS_PORT     TFTCS_GPIO_Port
#define LCD_CS_PIN      TFTCS_Pin
#define LCD_DC_PORT     TFTDC_GPIO_Port
#define LCD_DC_PIN      TFTDC_Pin
#define LCD_RST_PORT    0
#define LCD_RST_PIN     0
#define LCD_BL_PORT     0
#define LCD_BL_PIN      0

#define WIDTH           320
#define HEIGHT          240


class ReaderUI : public ST7789 {
public:
    ReaderUI();
    ~ReaderUI();
    void DisplayBanner(const char* text);
    void Clear();
    void DrawFastRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, bool white);
protected:
    inline void Select();
    inline void Deselect();
    void Flood(uint16_t color, uint32_t count);
};