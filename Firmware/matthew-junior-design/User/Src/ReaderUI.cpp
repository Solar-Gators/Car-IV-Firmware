#include "ReaderUI.hpp"

ReaderUI::ReaderUI() : ST7789(LCD_SPI, 
                            LCD_CS_PORT, LCD_CS_PIN,
                            LCD_DC_PORT, LCD_DC_PIN,
                            LCD_RST_PORT, LCD_RST_PIN,
                            LCD_BL_PORT, LCD_BL_PIN) {
}

ReaderUI::~ReaderUI() {}

void ReaderUI::DisplayBanner(const char* text) {
    DrawFastRectangle(0, 210, 320, 30, false);
    DrawText(&FontStyle_Emulogic, text, 50, 215, ST7789_WHITE);
}

void ReaderUI::Clear() {
    DrawFastRectangle(0, 0, 320, 210, true);
}

void ReaderUI::DrawFastRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, bool white) {
    Select();

    SetWindow(x, y, w, h);

    uint16_t color = white ? ST7789_WHITE : ST7789_BLACK;

    uint32_t num_p = (uint32_t)w * (uint32_t)h;
    uint32_t loops = (num_p*2) / 32768;
    uint32_t remainder = (num_p*2) % 32768;

    while (loops--) {
        HAL_SPI_Transmit_DMA(LCD_SPI, (uint8_t*)&color, 32768);
        while (HAL_DMA_GetState((LCD_SPI)->hdmatx) != HAL_DMA_STATE_READY);
    }
    HAL_SPI_Transmit_DMA(LCD_SPI, (uint8_t*)&color, remainder);
    while (HAL_DMA_GetState((LCD_SPI)->hdmatx) != HAL_DMA_STATE_READY);

    Deselect();
}

void ReaderUI::Select() {
    HAL_GPIO_WritePin(LCD_CS_PORT, LCD_CS_PIN, GPIO_PIN_RESET);
}

void ReaderUI::Deselect() {
    HAL_GPIO_WritePin(LCD_CS_PORT, LCD_CS_PIN, GPIO_PIN_SET);
}