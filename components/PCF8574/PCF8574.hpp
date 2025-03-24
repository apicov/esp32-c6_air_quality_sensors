#pragma once
#include "driver/i2c_master.h"
// ESP
#include <esp_log.h>
#include <esp_check.h>
#include <esp_system.h>



class PCF8574
{
    public:
        PCF8574(i2c_master_bus_handle_t bus_handle, uint8_t address);
        void write_byte(uint8_t data);
        uint8_t read_byte();
        void write_lsb_nibble(uint8_t nibble);
        void write_msb_nibble(uint8_t nibble);
        void set_pin(uint8_t pin, bool value);
        bool get_pin(uint8_t pin);
    private:
        uint8_t address_;
        uint8_t data_;
        uint8_t last_written_ = 0xFF;
        i2c_master_bus_handle_t bus_handle_;
        i2c_master_dev_handle_t dev_handle_;
        i2c_port_num_t port_num_;
};
