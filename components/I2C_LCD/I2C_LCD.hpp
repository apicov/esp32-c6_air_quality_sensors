#pragma once

#include "PCF8574.hpp"

// FreeRTOS
#include <freertos/FreeRTOS.h>

#include "rom/ets_sys.h"

// LCD module defines
#define LCD_LINEONE             0x00        // start of line 1
#define LCD_LINETWO             0x40        // start of line 2
#define LCD_LINETHREE           0x14        // start of line 3
#define LCD_LINEFOUR            0x54        // start of line 4

#define LCD_BACKLIGHT           0x08
#define LCD_ENABLE              0x04
#define LCD_COMMAND             0x00
#define LCD_WRITE               0x01

#define LCD_SET_DDRAM_ADDR      0x80
#define LCD_READ_BF             0x40

// LCD instructions
#define LCD_CLEAR               0x01        // replace all characters with ASCII 'space'
#define LCD_HOME                0x02        // return cursor to first position on first line
#define LCD_ENTRY_MODE          0x06        // shift cursor from left to right on read/write
#define LCD_DISPLAY_OFF         0x08        // turn display off
#define LCD_DISPLAY_ON          0x0C        // display on, cursor off, don't blink character
#define LCD_FUNCTION_RESET      0x30        // reset the LCD
#define LCD_FUNCTION_SET_4BIT   0x20        // 4-bit data, 2-line display, 5 x 7 font
#define LCD_SET_CURSOR          0x80        // set cursor position

#define LCD_RS 0
#define LCD_RW 1
#define LCD_E 2

// Pin mappings
// P0 -> RS
// P1 -> RW
// P2 -> E
// P3 -> Backlight
// P4 -> D4
// P5 -> D5
// P6 -> D6
// P7 -> D7

class I2C_LCD
{
    public:
        I2C_LCD(i2c_master_bus_handle_t bus_handle, uint8_t address);
        void write_char(uint8_t data);
        void write_string(char *data);
        void set_cursor(uint8_t row, uint8_t col);
        ~I2C_LCD();
    private:
        PCF8574 *o_port_;
        void lcd_send_cmd(uint8_t cmd);
        void lcd_send_char(uint8_t data);
        void lcd_send_enable_pulse();
};
