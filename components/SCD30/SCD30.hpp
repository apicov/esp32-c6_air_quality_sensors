#pragma once

#include <esp_err.h>
#include <stdbool.h>

#include "driver/i2c_master.h"

class SCD30 {
public:
    void SCD30(i2c_master_bus_handle_t bus_handle, uint8_t address);
    /**
     * @brief Initialize device descriptor.
     *
     * @param port     I2C port
     * @param sda_gpio SDA GPIO
     * @param scl_gpio SCL GPIO
     * @return         `ESP_OK` on success
     */
    esp_err_t init_desc(i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio);

    /**
     * @brief Free device descriptor.
     *
     * @return    `ESP_OK` on success
     */
    esp_err_t free_desc();

    /**
     * @brief Trigger continuous measurement.
     *
     * Signal update interval default is 2 seconds.
     *
     * @param p_comp Optional ambient pressure compensation in mBar, 0 to deactivate
     * @return      `ESP_OK` on success
     */
    esp_err_t trigger_continuous_measurement(uint16_t p_comp);

    /**
     * @brief Stop continuous measurement.
     *
     * Stops the continuous measurement of the SCD30.
     *
     * @return    `ESP_OK` on success
     */
    esp_err_t stop_continuous_measurement();

    /**
     * @brief Get measurement interval
     *
     * Gets the interval used by the SCD30 sensor to measure in continuous measurement mode.
     * Saved in non-volatile memory.
     *
     * @param interval_seconds Measurement interval in seconds
     * @return                `ESP_OK` on success
     */
    esp_err_t get_measurement_interval(uint16_t *interval_seconds);

    /**
     * @brief Set measurement interval
     *
     * Sets the interval used by the SCD30 sensor to measure in continuous measurement mode.
     * Saved in non-volatile memory.
     *
     * @param interval_seconds Measurement interval in seconds
     * @return                `ESP_OK` on success
     */
    esp_err_t set_measurement_interval(uint16_t interval_seconds);

    /**
     * @brief Check whether new measurement data is available for read-out.
     *
     * @param data_ready true if data is ready, false otherwise
     * @return          `ESP_OK` on success
     */
    esp_err_t get_data_ready_status(bool *data_ready);

    /**
     * @brief Read sensor output and convert.
     *
     * When new measurement data is available it can be read out with the following command.
     * Make sure that the measurement is completed by calling get_data_ready_status()
     *
     * @param co2         CO₂ concentration in ppm
     * @param temperature Temperature in degrees Celsius (°C)
     * @param humidity    Relative humidity in percent RH
     * @return           `ESP_OK` on success
     */
    esp_err_t read_measurement(float *co2, float *temperature, float *humidity);

    /**
     * @brief Get automatic self calibration (ASC) state.
     *
     * By default, the ASC is disabled.
     *
     * @param enabled true if ASC is enabled, false otherwise
     * @return       `ESP_OK` on success
     */
    esp_err_t get_automatic_self_calibration(bool *enabled);

    /**
     * @brief Enable or disable automatic self calibration (ASC).
     *
     * By default, the ASC is disabled.
     *
     * @param enabled true to enable ASC, false to disable ASC
     * @return       `ESP_OK` on success
     */
    esp_err_t set_automatic_self_calibration(bool enabled);

    /**
     * @brief Get Forced Recalibration Value
     *
     * @param correction_value FRC correction value in CO₂ ppm
     * @return                `ESP_OK` on success
     */
    esp_err_t get_forced_recalibration_value(uint16_t *correction_value);

    /**
     * @brief Set Forced Recalibration Value.
     *
     * @param target_co2_concentration Target CO₂ concentration in ppm. (400 <= val <= 2000)
     * @return                          `ESP_OK` on success
     */
    esp_err_t set_forced_recalibration_value(uint16_t target_co2_concentration);

    /**
     * @brief Get temperature offset in ticks.
     *
     * @param t_offset Temperature offset.
     * @return         `ESP_OK` on success
     */
    esp_err_t get_temperature_offset_ticks(uint16_t *t_offset);

    /**
     * @brief Get temperature offset in °C.
     *
     * @param t_offset Temperature offset in degrees Celsius (°C)
     * @return         `ESP_OK` on success
     */
    esp_err_t get_temperature_offset(float *t_offset);

    /**
     * @brief Set temperature offset in ticks.
     *
     * @param t_offset Temperature offset.
     * @return         `ESP_OK` on success
     */
    esp_err_t set_temperature_offset_ticks(uint16_t t_offset);

    /**
     * @brief Set temperature offset in °C.
     *
     * @param t_offset Temperature offset in degrees Celsius (°C)
     * @return         `ESP_OK` on success
     */
    esp_err_t set_temperature_offset(float t_offset);

    /**
     * @brief Get configured sensor altitude.
     *
     * @param altitude Sensor altitude in meters.
     * @return        `ESP_OK` on success
     */
    esp_err_t get_sensor_altitude(uint16_t *altitude);

    /**
     * @brief Set sensor altitude in meters above sea level.
     *
     * @param altitude Sensor altitude in meters.
     * @return        `ESP_OK` on success
     */
    esp_err_t set_sensor_altitude(uint16_t altitude);

    /**
     * @brief Get firmware version.
     *
     * @param firmware_version Firmware version
     * @return                `ESP_OK` on success
     */
    esp_err_t read_firmware_version(uint16_t *firmware_version);

    /**
     * @brief Reset the sensor
     *
     * Soft reset mechanism that forces the sensor into the same state as after powering up.
     * After soft reset, the sensor will reload all calibrated data.
     *
     * @return    `ESP_OK` on success
     */
    esp_err_t soft_reset();

private:
    i2c_dev_t dev; // Device descriptor
};
