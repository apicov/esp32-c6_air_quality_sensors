#pragma once

#include <esp_err.h>
#include <stdbool.h>
#include <esp_log.h>
#include "rom/ets_sys.h"

// FreeRTOS
#include <freertos/FreeRTOS.h>
#include "driver/i2c_master.h"

#define CMD_TRIGGER_CONTINUOUS_MEASUREMENT (0x0010)
#define CMD_STOP_CONTINUOUS_MEASUREMENT (0x0104)
#define CMD_SET_MEASUREMENT_INTERVAL (0x4600)
#define CMD_GET_DATA_READY_STATUS (0x0202)
#define CMD_READ_MEASUREMENT (0x0300)
#define CMD_ACTIVATE_AUTOMATIC_SELF_CALIBRATION (0x5306)
#define CMD_SET_FORCED_RECALIBRATION_VALUE (0x5204)
#define CMD_SET_TEMPERATURE_OFFSET (0x5403)
#define CMD_ALTITUDE_COMPENSATION (0x5102)
#define CMD_READ_FIRMWARE_VERSION (0xD100)
#define CMD_SOFT_RESET (0XD304)

#define CHECK(x)                \
    do                          \
    {                           \
        esp_err_t __;           \
        if ((__ = x) != ESP_OK) \
            return __;          \
    } while (0)
#define CHECK_ARG(VAL)                  \
    do                                  \
    {                                   \
        if (!(VAL))                     \
            return ESP_ERR_INVALID_ARG; \
    } while (0)

class SCD30 {
public:
    SCD30(i2c_master_bus_handle_t bus_handle, uint8_t address);
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
    uint8_t address_;
    i2c_master_bus_handle_t bus_handle_;
    i2c_master_dev_handle_t dev_handle_;

    uint8_t crc8(const uint8_t *data, size_t count);
    uint16_t swap(uint16_t v);
    esp_err_t send_cmd(uint16_t cmd, uint16_t *data, size_t words);
    esp_err_t read_resp(uint16_t *data, size_t words);
    esp_err_t execute_cmd(uint16_t cmd, uint32_t timeout_ms,
            uint16_t *out_data, size_t out_words, uint16_t *in_data, size_t in_words);

};
