#include "I2C_LCD.hpp"


I2C_LCD::I2C_LCD(i2c_master_bus_handle_t bus_handle, uint8_t address)
{
    o_port_ = new PCF8574(bus_handle, address);

    // write mode
    o_port_->set_pin(LCD_RW, 0);

    //send reset 3 times (init sequence)
    vTaskDelay(pdMS_TO_TICKS(50));
	lcd_send_cmd(LCD_FUNCTION_RESET);
	vTaskDelay(pdMS_TO_TICKS(10));
	lcd_send_cmd(LCD_FUNCTION_RESET);
    ets_delay_us(200);
	lcd_send_cmd(LCD_FUNCTION_RESET);
    ets_delay_us(80);

    //configure to 4 bit databus
	lcd_send_cmd(LCD_FUNCTION_SET_4BIT);
    ets_delay_us(100);
    // clear display ram
    lcd_send_cmd(LCD_CLEAR);
    ets_delay_us(100);
    //turn display on
    lcd_send_cmd(LCD_DISPLAY_ON);
    ets_delay_us(100);

	lcd_send_cmd(0x0E);
    ets_delay_us(100);

    ESP_LOGI("I2C_LCD", "LCD initialized");
    set_cursor(1, 15);
    lcd_send_char('F');
}

I2C_LCD::~I2C_LCD()
{
    delete o_port_;
}

void I2C_LCD::lcd_send_cmd(uint8_t  cmd)
{
    o_port_->set_pin(LCD_RS, 0);

    o_port_->write_msb_nibble(cmd >> 4);
    lcd_send_enable_pulse();

    o_port_->write_msb_nibble(cmd & 0x0F);
    lcd_send_enable_pulse();
}

void I2C_LCD::lcd_send_char(uint8_t data)
{
    o_port_->set_pin(LCD_RS, 1);

    o_port_->write_msb_nibble(data >> 4);
    lcd_send_enable_pulse();

    o_port_->write_msb_nibble(data & 0x0F);
    lcd_send_enable_pulse();
}

void I2C_LCD::lcd_send_enable_pulse()
{
    o_port_->set_pin(LCD_E, 1);
    ets_delay_us(1);
    o_port_->set_pin(LCD_E, 0);
    ets_delay_us(500);
}
void I2C_LCD::set_cursor(uint8_t row, uint8_t col)
{
    uint8_t row_offsets[] = {LCD_LINEONE, LCD_LINETWO, LCD_LINETHREE, LCD_LINEFOUR};
    lcd_send_cmd(LCD_SET_DDRAM_ADDR | (col + row_offsets[row]));
}
