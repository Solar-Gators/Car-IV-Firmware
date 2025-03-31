
/** Put this in the src folder **/

#include "i2c-lcd.h"
//extern I2C_HandleTypeDef hi2c2;  // change your handler here accordingly

// #define SLAVE_ADDRESS_LCD 0x4E // change this according to ur setup

// void lcd_send_cmd (char cmd)
// {
//   char data_u, data_l;
// 	uint8_t data_t[4];
// 	data_u = (cmd&0xf0);
// 	data_l = ((cmd<<4)&0xf0);
// 	data_t[0] = data_u|0x0C;  //en=1, rs=0 -> bxxxx1100
// 	data_t[1] = data_u|0x08;  //en=0, rs=0 -> bxxxx1000
// 	data_t[2] = data_l|0x0C;  //en=1, rs=0 -> bxxxx1100
// 	data_t[3] = data_l|0x08;  //en=0, rs=0 -> bxxxx1000
// 	HAL_I2C_Master_Transmit (&hi2c2, SLAVE_ADDRESS_LCD,(uint8_t *) data_t, 4, 100);
// }

// void lcd_send_data (char data)
// {
// 	char data_u, data_l;
// 	uint8_t data_t[4];
// 	data_u = (data&0xf0);
// 	data_l = ((data<<4)&0xf0);
// 	data_t[0] = data_u|0x0D;  //en=1, rs=0 -> bxxxx1101
// 	data_t[1] = data_u|0x09;  //en=0, rs=0 -> bxxxx1001
// 	data_t[2] = data_l|0x0D;  //en=1, rs=0 -> bxxxx1101
// 	data_t[3] = data_l|0x09;  //en=0, rs=0 -> bxxxx1001
// 	HAL_I2C_Master_Transmit (&hi2c2, SLAVE_ADDRESS_LCD,(uint8_t *) data_t, 4, 100);
// }

// void lcd_clear (void)
// {
// 	lcd_send_cmd (0x80);
// 	for (int i=0; i<70; i++)
// 	{
// 		lcd_send_data (' ');
// 	}
// }

// void lcd_put_cur(int row, int col)
// {
//     switch (row)
//     {
//         case 0:
//             col |= 0x80;
//             break;
//         case 1:
//             col |= 0xC0;
//             break;
//     }

//     lcd_send_cmd (col);
// }


// void lcd_init (void)
// {
// 	// 4 bit initialisation
// 	HAL_Delay(50);  // wait for >40ms
// 	lcd_send_cmd (0x30);
// 	HAL_Delay(5);  // wait for >4.1ms
// 	lcd_send_cmd (0x30);
// 	HAL_Delay(1);  // wait for >100us
// 	lcd_send_cmd (0x30);
// 	HAL_Delay(10);
// 	lcd_send_cmd (0x20);  // 4bit mode
// 	HAL_Delay(10);

//   // dislay initialisation
// 	lcd_send_cmd (0x28); // Function set --> DL=0 (4 bit mode), N = 1 (2 line display) F = 0 (5x8 characters)
// 	HAL_Delay(1);
// 	lcd_send_cmd (0x08); //Display on/off control --> D=0,C=0, B=0  ---> display off
// 	HAL_Delay(1);
// 	lcd_send_cmd (0x01);  // clear display
// 	HAL_Delay(1);
// 	HAL_Delay(1);
// 	lcd_send_cmd (0x06); //Entry mode set --> I/D = 1 (increment cursor) & S = 0 (no shift)
// 	HAL_Delay(1);
// 	lcd_send_cmd (0x0C); //Display on/off control --> D = 1, C and B = 0. (Cursor and blink, last two bits)
// }

// void lcd_send_string (const char *str)
// {
// 	while (*str) lcd_send_data (*str++);
// }

// void I2C_Write_USB_PD(uint16_t Register, uint8_t *DataW, uint16_t Length) {
//     // The I2C address of the USB PD device
//     uint16_t DevAddress = 0x28; // Replace with the actual I2C address of your USB PD device

//     // Transmit the data to the specified register
//     HAL_StatusTypeDef status = HAL_I2C_Mem_Write(&hi2c2, DevAddress, Register, I2C_MEMADD_SIZE_8BIT, DataW, Length, HAL_MAX_DELAY);

//     // Check the status of the transmission
//     if (status != HAL_OK) {
//         // Handle the error
//         // You can add error handling code here
//     }
// }



