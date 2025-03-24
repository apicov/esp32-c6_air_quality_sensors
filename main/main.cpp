#include <cstdint>
#include <cstdio>

// FreeRTOS
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// ESP-IDF
#include "esp_system.h"
#include <esp_log.h>
#include "driver/i2c_master.h"

// components
#include <PCF8574.hpp>
#include <I2C_LCD.hpp>

//#include "HD44780.h"

#define LCD_ADDR 0x27
#define SDA_PIN  (gpio_num_t) 6
#define SCL_PIN  (gpio_num_t) 7
#define I2C_PORT (gpio_num_t)0
#define LCD_COLS 16
#define LCD_ROWS 2

i2c_master_bus_handle_t bus_handle;

i2c_master_bus_config_t i2c_mst_config = {
    .i2c_port = 0,
    .sda_io_num = SDA_PIN,
    .scl_io_num = SCL_PIN,
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .glitch_ignore_cnt = 7,
    .intr_priority = 0,
    .trans_queue_depth = 0,
    .flags = { .enable_internal_pullup  = true },
};

void test_task(void *p);

extern "C" void app_main(void)
{
	ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));

	ESP_LOGI(__func__,"start...");


 xTaskCreate(&test_task, "Test Task", 2048, NULL, 5, NULL);

}


void test_task(void *p)
{
    I2C_LCD lcd(bus_handle, 0x27);

    while(1) {
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
