/*
 * ILI9341.cpp
 *
 *  Created on: Apr 21, 2022
 *      Author: John Carr
 *      Source: Adafruit GFX Library      https://github.com/adafruit/Adafruit-GFX-Library
 *              Adafruit TFT-LCD Library  https://github.com/adafruit/TFTLCD-Library
 */

#include <ILI9341.hpp>
#include "ILI9341_CMD.hpp"
#include "cmsis_os.h"
#include "GfxFont.h"

namespace SolarGators {
namespace Drivers {

static const uint8_t initcmd[] = {
  ILI9341_DISPOFF,   1, 0x00,
  ILI9341_PWCTR1  ,  1, 0x23,             // Power control VRH[5:0]
  ILI9341_PWCTR2  ,  1, 0x10,             // Power control SAP[2:0];BT[3:0]
  ILI9341_VMCTR1  ,  2, 0x2b, 0x2b,       // VCM control
  ILI9341_VMCTR2  ,  1, 0xC0,             // VCM control2
  ILI9341_MADCTL  ,  1, ILI9341_MADCTL_MX | ILI9341_MADCTL_BGR,             // Memory Access Control
  ILI9341_PIXFMT  ,  1, 0x55,
  ILI9341_FRMCTR1 ,  2, 0x00, 0x1B,
  ILI9341_ENTRYMODE, 1, 0x07,
  ILI9341_SLPOUT  , 0x80,                // Exit Sleep
  ILI9341_DISPON  , 0x80,                // Display on
  0x00                                   // End of list
};

ILI9341::ILI9341(int16_t w, int16_t h):WIDTH(w),HEIGHT(h)
{
  text_size_ = 1;
  width_ = WIDTH;
  height_ = HEIGHT;
  rotation = 0;
  cursor_y = cursor_x = 0;
  textsize_x = textsize_y = 1;
  textcolor = textbgcolor = 0xFFFF;
  wrap = true;
}

ILI9341::~ILI9341()
{
  // TODO Auto-generated destructor stub
}

void ILI9341::Reset() {
	HAL_GPIO_WritePin(LCD_CD_GPIO_Port_, LCD_CD_Pin_, GPIO_PIN_RESET);
}

void ILI9341::Resume() {
	HAL_GPIO_WritePin(LCD_CD_GPIO_Port_, LCD_CD_Pin_, GPIO_PIN_SET);
}

void ILI9341::Init()
{
  // Write all control signals high
  HAL_GPIO_WritePin(Backlight_PWM_GPIO_Port, Backlight_PWM_Pin, GPIO_PIN_SET);

  HAL_GPIO_WritePin(LCD_CS_GPIO_Port_, LCD_CS_Pin_, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LCD_CD_GPIO_Port_, LCD_CD_Pin_, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LCD_WRITE_GPIO_Port_, LCD_WRITE_Pin_, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LCD_READ_GPIO_Port_, LCD_READ_Pin_, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LCD_RST_GPIO_Port_, LCD_RST_Pin_, GPIO_PIN_SET);

  HAL_GPIO_WritePin(LCD_RST_GPIO_Port_, LCD_RST_Pin_, GPIO_PIN_RESET);
  osDelay(10);
  HAL_GPIO_WritePin(LCD_RST_GPIO_Port_, LCD_RST_Pin_, GPIO_PIN_SET);

  // enable chip select -> LOW
  HAL_GPIO_WritePin(LCD_CS_GPIO_Port_, LCD_CS_Pin_, GPIO_PIN_RESET);

  TransmitCmd(ILI9341_SWRESET);
  Transmit8bitData(0x00);
  osDelay(50);


  uint8_t cmd, x, numArgs;
  const uint8_t *addr = initcmd;
  while((cmd = *(addr++)) > 0)
  {
    x = *(addr++);
    numArgs = x & 0x7F;
    TransmitCmd(cmd);
    while(numArgs--)
    {
      Transmit8bitData(*(addr++));
    }
    if(x & 0x80)
    {
      osDelay(150);
    }
  }

  SetWindow(0, 0, width_-1, height_-1);
}

inline void ILI9341::Write(uint8_t data)
{
  // Put data on Bus
#if UI_USE_HAL
  HAL_GPIO_WritePin(LCD_DATA0_GPIO_Port_, LCD_DATA0_Pin_, static_cast<GPIO_PinState>(data & 0x01U) );
  HAL_GPIO_WritePin(LCD_DATA1_GPIO_Port_, LCD_DATA1_Pin_, static_cast<GPIO_PinState>(data & 0x02U) );
  HAL_GPIO_WritePin(LCD_DATA2_GPIO_Port_, LCD_DATA2_Pin_, static_cast<GPIO_PinState>(data & 0x04U) );
  HAL_GPIO_WritePin(LCD_DATA3_GPIO_Port_, LCD_DATA3_Pin_, static_cast<GPIO_PinState>(data & 0x08U) );
  HAL_GPIO_WritePin(LCD_DATA4_GPIO_Port_, LCD_DATA4_Pin_, static_cast<GPIO_PinState>(data & 0x10U) );
  HAL_GPIO_WritePin(LCD_DATA5_GPIO_Port_, LCD_DATA5_Pin_, static_cast<GPIO_PinState>(data & 0x20U) );
  HAL_GPIO_WritePin(LCD_DATA6_GPIO_Port_, LCD_DATA6_Pin_, static_cast<GPIO_PinState>(data & 0x40U) );
  HAL_GPIO_WritePin(LCD_DATA7_GPIO_Port_, LCD_DATA7_Pin_, static_cast<GPIO_PinState>(data & 0x80U) );
  // Pulse Write
  HAL_GPIO_WritePin(LCD_WRITE_GPIO_Port_, LCD_WRITE_Pin_, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LCD_WRITE_GPIO_Port_, LCD_WRITE_Pin_, GPIO_PIN_SET);
#else
  // Setup data
  LCD_DATA0_GPIO_Port_->ODR ^= ( -((data & 0x01U) >> 0)^LCD_DATA0_GPIO_Port_->ODR ) & (1 << LCD_DATA0_Pos_);
  LCD_DATA1_GPIO_Port_->ODR ^= ( -((data & 0x02U) >> 1)^LCD_DATA1_GPIO_Port_->ODR ) & (1 << LCD_DATA1_Pos_);
  LCD_DATA2_GPIO_Port_->ODR ^= ( -((data & 0x04U) >> 2)^LCD_DATA2_GPIO_Port_->ODR ) & (1 << LCD_DATA2_Pos_);
  LCD_DATA3_GPIO_Port_->ODR ^= ( -((data & 0x08U) >> 3)^LCD_DATA3_GPIO_Port_->ODR ) & (1 << LCD_DATA3_Pos_);
  LCD_DATA4_GPIO_Port_->ODR ^= ( -((data & 0x10U) >> 4)^LCD_DATA4_GPIO_Port_->ODR ) & (1 << LCD_DATA4_Pos_);
  LCD_DATA5_GPIO_Port_->ODR ^= ( -((data & 0x20U) >> 5)^LCD_DATA5_GPIO_Port_->ODR ) & (1 << LCD_DATA5_Pos_);
  LCD_DATA6_GPIO_Port_->ODR ^= ( -((data & 0x40U) >> 6)^LCD_DATA6_GPIO_Port_->ODR ) & (1 << LCD_DATA6_Pos_);
  LCD_DATA7_GPIO_Port_->ODR ^= ( -((data & 0x80U) >> 7)^LCD_DATA7_GPIO_Port_->ODR ) & (1 << LCD_DATA7_Pos_);
  // Pulse Write
  LCD_WRITE_GPIO_Port_->BSRR = LCD_WRITE_Pin_; // changed from brr to bsrr... ok?
  LCD_WRITE_GPIO_Port_->BSRR = (uint32_t)LCD_WRITE_Pin_ << 16U;
#endif

}

void ILI9341::TransmitCmd(uint8_t cmd)
{
  // D/C -> LOW
  HAL_GPIO_WritePin(LCD_CD_GPIO_Port_, LCD_CD_Pin_, GPIO_PIN_RESET);

  Write(cmd);

  // D/C -> HIGH
  HAL_GPIO_WritePin(LCD_CD_GPIO_Port_, LCD_CD_Pin_, GPIO_PIN_SET);
}

void ILI9341::Transmit8bitData(uint8_t data)
{
  // D/C -> High
  HAL_GPIO_WritePin(LCD_CD_GPIO_Port_, LCD_CD_Pin_, GPIO_PIN_SET);

  Write(data);
}

void ILI9341::Transmit16bitData(uint16_t data)
{
  // D/C -> High
  HAL_GPIO_WritePin(LCD_CD_GPIO_Port_, LCD_CD_Pin_, GPIO_PIN_SET);

  Write(static_cast<uint8_t>(data >> 8));
  Write(static_cast<uint8_t>(data >> 0));

}

void ILI9341::Transmit32bitData(uint32_t data)
{
  // D/C -> High
  HAL_GPIO_WritePin(LCD_CD_GPIO_Port_, LCD_CD_Pin_, GPIO_PIN_SET);

  Write(static_cast<uint8_t>(data >> 24));
  Write(static_cast<uint8_t>(data >> 16));
  Write(static_cast<uint8_t>(data >> 8 ));
  Write(static_cast<uint8_t>(data >> 0 ));

}

char ILI9341::SetWindow (uint16_t xs, uint16_t ys, uint16_t xe, uint16_t ye)
{
  // enable chip select -> LOW
  HAL_GPIO_WritePin(LCD_CS_GPIO_Port_, LCD_CS_Pin_, GPIO_PIN_RESET);

  // check if coordinates is out of range
  if ((xs > xe) || (xe > width_) ||
      (ys > ye) || (ye > height_))
  {
    // out of range
    return ILI9341_ERROR;
  }

  // set column
  TransmitCmd(ILI9341_CASET);
  // set column -> set column
  Transmit32bitData(((uint32_t) xs << 16) | xe);
  // set page
  TransmitCmd(ILI9341_PASET);
  // set page -> high byte first
  Transmit32bitData(((uint32_t) ys << 16) | ye);

  // disable chip select -> HIGH
  HAL_GPIO_WritePin(LCD_CS_GPIO_Port_, LCD_CS_Pin_, GPIO_PIN_SET);
  // success
  return ILI9341_SUCCESS;
}

char ILI9341::DrawPixel (uint16_t x, uint16_t y, uint16_t color)
{
  // check dimension
  if ((x > width_) || (y > height_)) {
    // error
    return ILI9341_ERROR;
  }
  // set window
  SetWindow(x, y, x, y);
  // draw pixel by 565 mode
  SendColor565(color, 1);
  // success
  return ILI9341_SUCCESS;
}

void ILI9341::SendColor565(uint16_t color, uint32_t count)
{
  // enable chip select -> LOW
  HAL_GPIO_WritePin(LCD_CS_GPIO_Port_, LCD_CS_Pin_, GPIO_PIN_RESET);
  // access to RAM
  TransmitCmd(ILI9341_RAMWR);
  // counter
  while (count--) {
    // write color - first colors byte
    Transmit16bitData(color);
  }
  // disable chip select -> HIGH
  HAL_GPIO_WritePin(LCD_CS_GPIO_Port_, LCD_CS_Pin_, GPIO_PIN_SET);
}

void ILI9341::Flood(uint16_t color, uint32_t count)
{
  // enable chip select -> LOW
  HAL_GPIO_WritePin(LCD_CS_GPIO_Port_, LCD_CS_Pin_, GPIO_PIN_RESET);
  // access to RAM
  TransmitCmd(ILI9341_RAMWR);
  // counter
  // Upper and lower are the same
  if( (color & 0xFF) == (color >> 8))
  {
    count--;
    Transmit16bitData(color);
    while(count--)
    {
#if UI_USE_HAL
  // Pulse Write
  HAL_GPIO_WritePin(LCD_WRITE_GPIO_Port_, LCD_WRITE_Pin_, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LCD_WRITE_GPIO_Port_, LCD_WRITE_Pin_, GPIO_PIN_SET);
  // Pulse Write
  HAL_GPIO_WritePin(LCD_WRITE_GPIO_Port_, LCD_WRITE_Pin_, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LCD_WRITE_GPIO_Port_, LCD_WRITE_Pin_, GPIO_PIN_SET);
#else
  // Pulse Write for high byte
  LCD_WRITE_GPIO_Port_->BSRR = LCD_WRITE_Pin_; // changed from brr to bsrr
  LCD_WRITE_GPIO_Port_->BSRR = (uint32_t)LCD_WRITE_Pin_ << 16U;;
  // Pulse Write for low byte
  LCD_WRITE_GPIO_Port_->BSRR = LCD_WRITE_Pin_; // changed from brr to bsrr
  LCD_WRITE_GPIO_Port_->BSRR = (uint32_t)LCD_WRITE_Pin_ << 16U;;

#endif
    }
  }
  else
  {
    while (count--)
    {
      Transmit16bitData(color);
    }
  }
  // disable chip select -> HIGH
  HAL_GPIO_WritePin(LCD_CS_GPIO_Port_, LCD_CS_Pin_, GPIO_PIN_SET);
}

void ILI9341::ClearScreen(uint16_t color)
{
  // set whole window
  SetWindow(0, 0, width_, height_);
  // draw individual pixels
  Flood(color, ILI9341_CACHE_MEM);
  // Update background color
  textbgcolor = color;
}

void ILI9341::DrawFastHLine(uint16_t x, uint16_t y, uint16_t length,
                                    uint16_t color) {
  int16_t x2;

  // Initial off-screen clipping
  if ((length <= 0) || (y < 0) || (y >= height_) || (x >= width_) ||
      ((x2 = (x + length - 1)) < 0))
    return;

  if (x < 0) { // Clip left
    length += x;
    x = 0;
  }
  if (x2 >= width_) { // Clip right
    x2 = width_ - 1;
    length = x2 - x + 1;
  }

  SetWindow(x, y, x2, y);
  Flood(color, length);
}

void ILI9341::DrawFastVLine(uint16_t x, uint16_t y, uint16_t length,
                                    uint16_t color) {
  int16_t y2;

  // Initial off-screen clipping
  if ((length <= 0) || (x < 0) || (x >= width_) || (y >= height_) ||
      ((y2 = (y + length - 1)) < 0))
    return;
  if (y < 0) { // Clip top
    length += y;
    y = 0;
  }
  if (y2 >= height_) { // Clip bottom
    y2 = height_ - 1;
    length = y2 - y + 1;
  }

  SetWindow(x, y, x, y2);
  Flood(color, length);
}

void ILI9341::SetRotation(uint8_t x) {
  // enable chip select -> LOW
  HAL_GPIO_WritePin(LCD_CS_GPIO_Port_, LCD_CS_Pin_, GPIO_PIN_RESET);
  rotation = x % 4; // can't be higher than 3
  switch (rotation) {
  case 0:
    x = (ILI9341_MADCTL_MX | ILI9341_MADCTL_BGR);
    width_ = ILI9341_TFTWIDTH;
    height_ = ILI9341_TFTHEIGHT;
    break;
  case 1:
    x = (ILI9341_MADCTL_MV | ILI9341_MADCTL_BGR);
    width_ = ILI9341_TFTHEIGHT;
    height_ = ILI9341_TFTWIDTH;
    break;
  case 2:
    x = (ILI9341_MADCTL_MY | ILI9341_MADCTL_BGR);
    width_ = ILI9341_TFTWIDTH;
    height_ = ILI9341_TFTHEIGHT;
    break;
  case 3:
    x = (ILI9341_MADCTL_MX | ILI9341_MADCTL_MY | ILI9341_MADCTL_MV | ILI9341_MADCTL_BGR);
    width_ = ILI9341_TFTHEIGHT;
    height_ = ILI9341_TFTWIDTH;
    break;
  }
  TransmitCmd(ILI9341_MADCTL); // MADCTL
  Transmit8bitData(x);
  // For 9341, init default full-screen address window:
  SetWindow(0, 0, width_ - 1, height_ - 1); // CS_IDLE happens here
}

void ILI9341::SetTextSize(uint8_t size)
{
  text_size_ = size;
}

void ILI9341::DrawChar(int16_t x, int16_t y, unsigned char c,
    uint16_t color, uint8_t size)
{
  DrawChar(x, y, c, color, textbgcolor, size, size);
}

void ILI9341::DrawChar(int16_t x, int16_t y, unsigned char c,
    uint16_t color, uint16_t bg, uint8_t size_x,
    uint8_t size_y)
{
  for (int8_t i = 0; i < 5; i++) { // Char bitmap = 5 columns
      uint8_t line = font[c * 5 + i];
      for (int8_t j = 0; j < 8; j++, line >>= 1) {
        if (line & 1) {
          if (size_x == 1 && size_y == 1)
            DrawPixel(x + i, y + j, color);
          else
            FillRect(x + i * size_x, y + j * size_y, size_x, size_y,
                          color);
        } else if (bg != color) {
          if (size_x == 1 && size_y == 1)
            DrawPixel(x + i, y + j, bg);
          else
            FillRect(x + i * size_x, y + j * size_y, size_x, size_y, bg);
        }
      }
    }
    if (bg != color) { // If opaque, draw vertical line for last column
      if (size_x == 1 && size_y == 1)
        DrawFastVLine(x + 5, y, 8, bg);
      else
        FillRect(x + 5 * size_x, y, size_x, 8 * size_y, bg);
    }
}

void ILI9341::DrawText(uint16_t x, uint16_t y, const char *str, uint16_t color)
{
  // NOTE: Characters are 6x8 (wxh)
    uint8_t TempChar;

    /* Set area back to span the entire LCD */
    SetWindow(0, 0, width_ - 1, height_ - 1);
    do
    {
        TempChar = *str++;
        DrawChar( x, y, TempChar, color, text_size_);
        if( x < width_ - 1 - 8)
        {
            x += (6 * text_size_);
        }
        else if ( y < height_ - 1 - 16)
        {
            x = 0;
            y += (8 * text_size_);
        }
        else
        {
            x = 0;
            y = 0;
        }
    }
    while ( *str != 0 );
}

void ILI9341::DrawRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
  DrawFastHLine(x, y, w, color);
  DrawFastHLine(x, y + h - 1, w, color);
  DrawFastVLine(x, y, h, color);
  DrawFastVLine(x + w - 1, y, h, color);
}

void ILI9341::FillRect(uint16_t x1, uint16_t y1, uint16_t w, uint16_t h, uint16_t color)
{
  int16_t x2, y2;

  // Initial off-screen clipping
  if ((w <= 0) || (h <= 0) || (x1 >= width_) || (y1 >= height_) ||
      ((x2 = x1 + w - 1) < 0) || ((y2 = y1 + h - 1) < 0))
    return;
  if (x1 < 0) { // Clip left
    w += x1;
    x1 = 0;
  }
  if (y1 < 0) { // Clip top
    h += y1;
    y1 = 0;
  }
  if (x2 >= width_) { // Clip right
    x2 = width_ - 1;
    w = x2 - x1 + 1;
  }
  if (y2 >= height_) { // Clip bottom
    y2 = height_ - 1;
    h = y2 - y1 + 1;
  }

  SetWindow(x1, y1, x2, y2);
  Flood(color, (uint32_t)w * (uint32_t)h);
}

} /* namespace Drivers */
} /* namespace SolarGators */
