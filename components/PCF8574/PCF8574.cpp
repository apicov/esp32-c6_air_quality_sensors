#include "PCF8574.hpp"

PCF8574::PCF8574(i2c_master_bus_handle_t bus_handle, uint8_t address)
    :address_(address), bus_handle_(bus_handle)
{
	i2c_device_config_t dev_cfg = {
		.dev_addr_length = I2C_ADDR_BIT_LEN_7,
		.device_address = address_,
		.scl_speed_hz = 100000,
	};

	ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle_, &dev_cfg, &dev_handle_));

    //activate pull-ups as initial state
    //uint8_t data = 0xFF;
    //set pins to input
	//ESP_ERROR_CHECK(i2c_master_transmit(dev_handle_, &data, 1, -1));
}

void PCF8574::write_byte(uint8_t data)
{
	ESP_ERROR_CHECK(i2c_master_transmit(dev_handle_, &data, 1, -1));
    last_written_ = data;
}

uint8_t PCF8574::read_byte()
{
    uint8_t data = 0xFF;
    //set pins to input
	ESP_ERROR_CHECK(i2c_master_transmit(dev_handle_, &data, 1, -1));
    last_written_ = data;
    // read port
	ESP_ERROR_CHECK(i2c_master_receive(dev_handle_, &data, 1, -1));
    return data;
}

void PCF8574::write_lsb_nibble(uint8_t nibble)
{
    uint8_t data = last_written_ & 0xF0;
    nibble &= 0x0F;
    data |= nibble;
    ESP_ERROR_CHECK(i2c_master_transmit(dev_handle_, &data, 1, -1));
    last_written_ = data;
}

void PCF8574::write_msb_nibble(uint8_t nibble)
{
    uint8_t data = last_written_ & 0x0F;
    nibble &= 0x0F;
    data |= nibble << 4;
    ESP_ERROR_CHECK(i2c_master_transmit(dev_handle_, &data, 1, -1));
    last_written_ = data;
}

void PCF8574::set_pin(uint8_t pin, bool value)
{
    uint8_t data = last_written_;
    if (value) {
        data |= 1 << pin;
    }
    else {
        data &= ~(1 << pin);
    }
    ESP_ERROR_CHECK(i2c_master_transmit(dev_handle_, &data, 1, -1));
    last_written_ = data;
}

bool PCF8574::get_pin(uint8_t pin)
{
    uint8_t data = read_byte();
    return data & (1 << pin);
}
