/*
 * ILI9341.h
 *
 *  Created on: Apr 21, 2022
 *      Author: John Carr
 */

#ifndef SOLARGATORSBSP_DRIVERS_INC_ILI9341_HPP_
#define SOLARGATORSBSP_DRIVERS_INC_ILI9341_HPP_

#include "main.h"
#include "etl/string.h"

#define UI_USE_HAL 1

namespace SolarGators {
namespace Drivers {

#define ILI9341_SUCCESS 1
#define ILI9341_ERROR 0

class ILI9341 {
public:
  ILI9341(int16_t w, int16_t h);
  virtual ~ILI9341();

  void Init();
  void Reset();
  void Resume();

  void DrawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color);
  void DrawFastVLine(uint16_t x, uint16_t y, uint16_t length, uint16_t color);
  void DrawFastHLine(uint16_t x, uint16_t y, uint16_t length, uint16_t color);
  void DrawRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
  void FillRect(uint16_t x0, uint16_t y0, uint16_t w, uint16_t h, uint16_t color);
  void DrawCircle(uint16_t x0, uint16_t y0, uint16_t r, uint16_t color);
  void FillCircle(uint16_t x0, uint16_t y0, uint16_t r, uint16_t color);
  void DrawRoundRect(uint16_t x0, uint16_t y0, uint16_t w, uint16_t h, uint16_t radius, uint16_t color);
  void FillRoundRect(uint16_t x0, uint16_t y0, uint16_t w, uint16_t h, uint16_t radius, uint16_t color);
  void DrawTriangle(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
  void FillTriangle(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
  void DrawChar(int16_t x, int16_t y, unsigned char c,
      uint16_t color, uint8_t size);
  void DrawChar(int16_t x, int16_t y, unsigned char c,
      uint16_t color, uint16_t bg, uint8_t size_x,
      uint8_t size_y);
  void DrawText(uint16_t x, uint16_t y, const char *str, uint16_t color);
  void SetCursor(int16_t x0, int16_t y0);
  void SetTextColor(uint16_t color);
  void SetTextColor(uint16_t color, uint16_t backgroundcolor);
  void SetTextSize(uint8_t size);
  void SetTextWrap(bool w);
  void FillScreen(uint16_t color);
  char DrawPixel (uint16_t x, uint16_t y, uint16_t color);
  void ClearScreen(uint16_t color);
  void SetRotation(uint8_t x);
protected:
  inline void Write(uint8_t data);
  void Read(uint8_t* data);
  void TransmitCmd(uint8_t cmd);
  void Transmit8bitData(uint8_t data);
  void Transmit16bitData(uint16_t data);
  void Transmit32bitData(uint32_t data);
  void SendColor565(uint16_t color, uint32_t count);
  void Flood(uint16_t color, uint32_t count);
  char SetWindow(uint16_t xs, uint16_t ys, uint16_t xe, uint16_t ye);
  int16_t WIDTH;
  int16_t HEIGHT;
  int16_t width_;
  int16_t height_;
  int16_t text_size_;
  /** @var array Chache memory char index row */
  unsigned short int ili9341_cache_index_row_ = 0;
  /** @var array Chache memory char index column */
  unsigned short int ili9341_cache_index_col_ = 0;
  int16_t cursor_x;
  int16_t cursor_y;
  uint16_t textcolor;
  uint16_t textbgcolor;
  uint8_t textsize_x;
  uint8_t textsize_y;
  uint8_t rotation;
  bool wrap;

private:
  // Data
  static constexpr uint16_t LCD_DATA7_Pin_  = GPIO_PIN_13;
  static constexpr uint16_t LCD_DATA7_Pos_  = 7;
  GPIO_TypeDef* LCD_DATA7_GPIO_Port_        = GPIOC;
  static constexpr uint16_t LCD_DATA6_Pin_  = GPIO_PIN_14;
  static constexpr uint16_t LCD_DATA6_Pos_  = 6;
  GPIO_TypeDef* LCD_DATA6_GPIO_Port_        = GPIOC;
  static constexpr uint16_t LCD_DATA5_Pin_  = GPIO_PIN_15;
  static constexpr uint16_t LCD_DATA5_Pos_  = 5;
  GPIO_TypeDef* LCD_DATA5_GPIO_Port_        = GPIOC;
  static constexpr uint16_t LCD_DATA4_Pin_  = GPIO_PIN_0;
  static constexpr uint16_t LCD_DATA4_Pos_  = 4;
  GPIO_TypeDef* LCD_DATA4_GPIO_Port_        = GPIOH;
  static constexpr uint16_t LCD_DATA3_Pin_  = GPIO_PIN_1;
  static constexpr uint16_t LCD_DATA3_Pos_  = 3;
  GPIO_TypeDef* LCD_DATA3_GPIO_Port_        = GPIOH;
  static constexpr uint16_t LCD_DATA2_Pin_  = GPIO_PIN_0;
  static constexpr uint16_t LCD_DATA2_Pos_  = 2;
  GPIO_TypeDef* LCD_DATA2_GPIO_Port_        = GPIOC;
  static constexpr uint16_t LCD_DATA1_Pin_  = GPIO_PIN_1;
  static constexpr uint16_t LCD_DATA1_Pos_  = 1;
  GPIO_TypeDef* LCD_DATA1_GPIO_Port_        = GPIOC;
  static constexpr uint16_t LCD_DATA0_Pin_  = GPIO_PIN_2;
  static constexpr uint16_t LCD_DATA0_Pos_  = 0;
  GPIO_TypeDef* LCD_DATA0_GPIO_Port_        = GPIOC;
  // Read / Write
  uint16_t LCD_READ_Pin_              = GPIO_PIN_1;
  GPIO_TypeDef* LCD_READ_GPIO_Port_   = GPIOA;
  uint16_t LCD_WRITE_Pin_             = GPIO_PIN_2;
  GPIO_TypeDef* LCD_WRITE_GPIO_Port_  = GPIOA;
  // Command / Data
  uint16_t LCD_CD_Pin_                = GPIO_PIN_3;
  GPIO_TypeDef* LCD_CD_GPIO_Port_     = GPIOA;
  // Chip Select
  uint16_t LCD_CS_Pin_                = GPIO_PIN_4;
  GPIO_TypeDef* LCD_CS_GPIO_Port_     = GPIOC;
  // Reset
  uint16_t LCD_RST_Pin_               = GPIO_PIN_0;
  GPIO_TypeDef* LCD_RST_GPIO_Port_    = GPIOA;
};

} /* namespace Drivers */
} /* namespace SolarGators */

#endif /* SOLARGATORSBSP_DRIVERS_INC_ILI9341_HPP_ */
