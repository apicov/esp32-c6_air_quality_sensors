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
#include <SCD30.hpp>

#define LCD_ADDR 0x27
#define SCD30_ADDR 0x61

#define SDA_PIN  (gpio_num_t) 6
#define SCL_PIN  (gpio_num_t) 7
#define I2C_PORT (gpio_num_t) 0

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

void scd30_task(void *p);

extern "C" void app_main(void)
{
	ESP_LOGI(__func__,"starting app_main");
    // configure I2C bus
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));

    xTaskCreate(&scd30_task, "Test Task", 2048, NULL, 5, NULL);

}

void scd30_task(void *p)
{
    I2C_LCD lcd(bus_handle, 0x27);
    SCD30 scd30(bus_handle, SCD30_ADDR);

    uint16_t version, major_ver, minor_ver;

    ESP_ERROR_CHECK(scd30.read_firmware_version(&version));

    major_ver = (version >> 8) & 0xf;
    minor_ver = version & 0xf;

    ESP_LOGI(__func__, "SCD30 Firmware Version: %d.%d", major_ver, minor_ver);
    vTaskDelay(2000 / portTICK_PERIOD_MS);

    uint16_t m_interval = 60;
    ESP_ERROR_CHECK(scd30.set_measurement_interval(m_interval));
    vTaskDelay(10 / portTICK_PERIOD_MS);
    ESP_LOGI(__func__, "measurement interval set to %d seconds", m_interval);

    ESP_ERROR_CHECK(scd30.trigger_continuous_measurement(0));
    ESP_LOGI(__func__, "Starting continuous measurement");

    char lcd_buf[17];
    float co2 = 0, temperature = 0, humidity = 0;
    bool data_ready;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(30000));

        scd30.get_data_ready_status(&data_ready);

        if (data_ready) {
            esp_err_t res = scd30.read_measurement(&co2, &temperature, &humidity);
            if (res != ESP_OK) {
                ESP_LOGE(__func__, "Error reading results %d (%s)", res, esp_err_to_name(res));
                continue;
            }

            if (co2 == 0) {
                ESP_LOGW(__func__, "Invalid sample detected, skipping");
                continue;
            }

            ESP_LOGI(__func__, "CO2: %d ppm",(unsigned int) co2);
            ESP_LOGI(__func__, "Temperature: %.2f °C", temperature);
            ESP_LOGI(__func__, "Humidity: %.2f %%", humidity);

            // display measurements on LCD
            sprintf(lcd_buf,"CO2:%d ppm", (unsigned int) co2);
            lcd.set_cursor(0, 0);
            lcd.write_string(lcd_buf);

            sprintf(lcd_buf,"Temp:%dC", (unsigned int) (temperature + 0.5));
            lcd.set_cursor(1, 0);
            lcd.write_string(lcd_buf);

            sprintf(lcd_buf,"RH:%d%%", (unsigned int) (humidity + 0.5));
            lcd.set_cursor(1, 9);
            lcd.write_string(lcd_buf);
            }
        }
    }
