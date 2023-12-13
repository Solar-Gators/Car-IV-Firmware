#include "ST7789.hpp"

ST7789::ST7789(SPI_HandleTypeDef* phspi, 
    GPIO_TypeDef* CS_Port, uint16_t CS_Pin,
    GPIO_TypeDef* DC_Port, uint16_t DC_Pin,
    GPIO_TypeDef* RST_Port, uint16_t RST_Pin,
    GPIO_TypeDef* BL_Port, uint16_t BL_Pin)
{
    phspi_ = phspi;
    TFTCS_GPIO_Port_ = CS_Port;
    TFTCS_Pin_ = CS_Pin;
    TFTDC_GPIO_Port_ = DC_Port;
    TFTDC_Pin_ = DC_Pin;
    TFTRST_GPIO_Port_ = RST_Port;
    TFTRST_Pin_ = RST_Pin;
    TFTBL_GPIO_Port_ = BL_Port;
    TFTBL_Pin_ = BL_Pin;
}

ST7789::~ST7789()
{
}

void ST7789::Init() {
		// Pulse CS and set clock high
		uint8_t dummy_data = 0x41;
		Deselect();
		HAL_SPI_Transmit(phspi_, &dummy_data, 1, HAL_MAX_DELAY);

    // Software reset
    Select();

    WriteCommand(ST7789_SWRESET_ADDR);
    HAL_Delay(60);
    WriteCommand(ST7789_SLPOUT_ADDR);
    HAL_Delay(250);
    WriteCommand(ST7789_COLMOD_ADDR);
    WriteData(0x55);

    // Set rotation
    WriteCommand(ST7789_CASET_ADDR);
    WriteData(0x00);
    WriteData(0x00);
    WriteData((X_MAX >> 8 & 0xFF));
    WriteData((X_MAX >> 0 & 0xFF));
    

    WriteCommand(ST7789_RASET_ADDR);
    WriteData(0x00);
    WriteData(0x00);
    WriteData((Y_MAX >> 8 & 0xFF));
    WriteData((Y_MAX >> 0 & 0xFF));
    
    // set column address order right to left, line address order bottom to top
    WriteCommand(ST7789_MADCTL_ADDR);
    //WriteData(0b01001000);
    WriteData(0x40 | 0x20 | 0x80);

    WriteCommand(ST7789_NORON_ADDR);
    HAL_Delay(60);

    WriteCommand(ST7789_INVON_ADDR);
    HAL_Delay(60);

    WriteCommand(ST7789_DISPON_ADDR);
    //HAL_Delay(60);
    //Fill(0x0000);
    
    Deselect();
}

void ST7789::Fill(uint16_t color) {
    DrawRectangle(0, 0, X_MAX, Y_MAX, color);
}

void ST7789::DrawPixel(uint16_t x, uint16_t y, uint16_t color) {
    // Check boundary conditions
    if ( x > X_MAX || y > Y_MAX) {
        // Invalid parameters, handle or return an error
        return;
    }

    Select();
    // Set window
    SetWindow(x, y, 1, 1);
    // Set color
    WriteData16(color);
    //WriteData(color >> 8);   // Send high byte of color
    //WriteData(color & 0xFF); // Send low byte of color
    Deselect();
}

void ST7789::DrawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color) {
    // TODO
}

void ST7789::DrawRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) {
    Select();
    // Set window
    SetWindow(x, y, w, h);

    uint32_t num_p = (uint32_t)w * (uint32_t)h;
    Flood(color, num_p);

    Deselect();
}

void ST7789::DrawText(const fontStyle_t *font, const char *text, uint16_t x, uint16_t y, uint16_t color) {
    while (*text){
        const uint8_t *glyph = GetCharGlyph(font, *text);
        if (glyph) {
            for (uint16_t row = 0; row < font->GlyphHeight; row++) {
                uint8_t rowData = *glyph;

                for (uint8_t col = 0; col < font->FixedWidth; col++) {
                    if (rowData & (0x80 >> col))
                        DrawPixel(x + col, y + row, color);
                }
                glyph -= font->GlyphBytesWidth;
            }
            x += font->FixedWidth; 
        }
        text++; 
    }
}

inline void ST7789::Select() {
    HAL_GPIO_WritePin(TFTCS_GPIO_Port_, TFTCS_Pin_, GPIO_PIN_RESET);
}

inline void ST7789::Deselect() {
    HAL_GPIO_WritePin(TFTCS_GPIO_Port_, TFTCS_Pin_, GPIO_PIN_SET);
}

inline void ST7789::SetData() {
    HAL_GPIO_WritePin(TFTDC_GPIO_Port_, TFTDC_Pin_, GPIO_PIN_SET);
}

inline void ST7789::SetCommand() {
    HAL_GPIO_WritePin(TFTDC_GPIO_Port_, TFTDC_Pin_, GPIO_PIN_RESET);
}

void ST7789::WriteCommand(uint8_t cmd) {
    SetCommand();
    HAL_SPI_Transmit(phspi_, &cmd, 1, HAL_MAX_DELAY);
    SetData();
}

void ST7789::WriteData(uint8_t data) {
    HAL_SPI_Transmit(phspi_, &data, 1, HAL_MAX_DELAY);
}

void ST7789::WriteData16(uint16_t data) {
    HAL_SPI_Transmit(phspi_, ((uint8_t*)&data) + 1, 1, HAL_MAX_DELAY);
    HAL_SPI_Transmit(phspi_, (uint8_t*)&data, 1, HAL_MAX_DELAY);
}

void ST7789::Flood(uint16_t color, uint32_t count) {
    while (count--) {
        WriteData16(color);
    }
}

uint8_t ST7789::ReadRegister(uint8_t addr) {
    return HAL_SPI_Receive(phspi_, &addr, 1, HAL_MAX_DELAY);
}

void ST7789::SetWindow(uint16_t x, uint16_t y, uint16_t w, uint16_t h) {
    // Check boundary conditions
    if (x > X_MAX || y > Y_MAX || w == 0 || h == 0)
        {
            // Invalid parameters, handle or return an error
            return;
        }
    //Select();

    // Set column address
    WriteCommand(ST7789_CASET_ADDR); // Column Address Set command
    WriteData(x >> 8);               // Start column high byte
    WriteData(x & 0xFF);             // Start column low byte
    WriteData((x + w - 1) >> 8);     // End column high byte
    WriteData((x + w - 1) & 0xFF);   // End column low byte

    // Set row address
    WriteCommand(ST7789_RASET_ADDR); // Row Address Set command
    WriteData(y >> 8);               // Start row high byte
    WriteData(y & 0xFF);             // Start row low byte
    WriteData((y + h - 1) >> 8);     // End row high byte
    WriteData((y + h - 1) & 0xFF);   // End row low byte

    // Set register to write to as memory
    WriteCommand(ST7789_RAMWR_ADDR);

    //Deselect();
}

const uint8_t* ST7789::GetCharGlyph(const fontStyle_t *font, char c) {
    int low = 0;
    int high = font->GlyphCount - 1;

    while (low <= high) {
        int mid = low + (high - low) / 2;
        char midChar = font->FirstAsciiCode + mid;

        if (midChar < c)
            low = mid + 1;
        else if (midChar > c)
            high = mid - 1;
        else {
            // Found the glyph
            return &(font->GlyphBitmaps[(mid + 1) * font->GlyphBytesWidth * font->GlyphHeight - font->GlyphBytesWidth]);
        }
    }

    // Glyph not found
    return NULL;
}