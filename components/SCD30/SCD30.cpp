#include "SCD30.hpp"

SCD30::SCD30(i2c_master_bus_handle_t bus_handle, uint8_t address)
    :address_(address), bus_handle_(bus_handle)
{
	i2c_device_config_t dev_cfg = {
		.dev_addr_length = I2C_ADDR_BIT_LEN_7,
		.device_address = address_,
		.scl_speed_hz = 100000,
	};

	ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle_, &dev_cfg, &dev_handle_));
}

esp_err_t SCD30::trigger_continuous_measurement(uint16_t p_comp)
{
    CHECK_ARG(p_comp == 0 || (p_comp >= 700 && p_comp <= 1400));

    return execute_cmd(CMD_TRIGGER_CONTINUOUS_MEASUREMENT, 0, &p_comp, 1, NULL, 0);
}

esp_err_t SCD30::stop_continuous_measurement()
{
    return execute_cmd(CMD_STOP_CONTINUOUS_MEASUREMENT, 1, NULL, 0, NULL, 0);
}

esp_err_t SCD30::get_measurement_interval(uint16_t *interval_seconds)
{
    CHECK_ARG(interval_seconds);
    return execute_cmd(CMD_SET_MEASUREMENT_INTERVAL, 1, NULL, 0, interval_seconds, 1);
}

esp_err_t SCD30::set_measurement_interval(uint16_t interval_seconds)
{
    CHECK_ARG(interval_seconds > 2 && interval_seconds < 1800);
    return execute_cmd(CMD_SET_MEASUREMENT_INTERVAL, 1, &interval_seconds, 1, NULL, 0);
}

esp_err_t SCD30::get_data_ready_status(bool *data_ready)
{
    CHECK_ARG(data_ready);

    uint16_t status;
    CHECK(execute_cmd(CMD_GET_DATA_READY_STATUS, 1, NULL, 0, &status, 1));
    *data_ready = status != 0;

    return ESP_OK;
}

esp_err_t SCD30::read_measurement(float *co2, float *temperature, float *humidity)
{
    CHECK_ARG(co2 || temperature || humidity);

    union {
        uint32_t u32;
        float f;
    } tmp;
    uint16_t buf[6];
    CHECK(execute_cmd(CMD_READ_MEASUREMENT, 3, NULL, 0, buf, 6));
    if (co2) {
        tmp.u32 = ((uint32_t)buf[0] << 16) | buf[1];
        *co2 = tmp.f;
    }
    if (temperature) {
        tmp.u32 = ((uint32_t)buf[2] << 16) | buf[3];
        *temperature = tmp.f;
    }
    if (humidity) {
        tmp.u32 = ((uint32_t)buf[4] << 16) | buf[5];
        *humidity = tmp.f;
    }
    return ESP_OK;
}

esp_err_t SCD30::get_automatic_self_calibration(bool *enabled)
{
    CHECK_ARG(enabled);

    return execute_cmd(CMD_ACTIVATE_AUTOMATIC_SELF_CALIBRATION, 1, NULL, 0, (uint16_t *)enabled, 1);
}

esp_err_t SCD30::set_automatic_self_calibration(bool enabled)
{
    return execute_cmd(CMD_ACTIVATE_AUTOMATIC_SELF_CALIBRATION, 1, (uint16_t *)&enabled, 1, NULL, 0);
}

esp_err_t SCD30::get_forced_recalibration_value(uint16_t *correction_value)
{
    CHECK_ARG(correction_value);

    return execute_cmd(CMD_SET_FORCED_RECALIBRATION_VALUE, 1,
                       NULL, 0, correction_value, 1);
}

esp_err_t SCD30::set_forced_recalibration_value(uint16_t target_co2_concentration)
{
    CHECK_ARG(target_co2_concentration);

    return execute_cmd(CMD_SET_FORCED_RECALIBRATION_VALUE, 1,
                       &target_co2_concentration, 1, NULL, 0);
}

esp_err_t SCD30::get_temperature_offset_ticks(uint16_t *t_offset)
{
    CHECK_ARG(t_offset);

    return execute_cmd(CMD_SET_TEMPERATURE_OFFSET, 1, NULL, 0, t_offset, 1);
}

esp_err_t SCD30::get_temperature_offset(float *t_offset)
{
    CHECK_ARG(t_offset);
    uint16_t raw;

    CHECK(get_temperature_offset_ticks(&raw));

    *t_offset = (float)raw / 100;
    return ESP_OK;
}

esp_err_t SCD30::set_temperature_offset_ticks(uint16_t t_offset)
{
    return execute_cmd(CMD_SET_TEMPERATURE_OFFSET, 1, &t_offset, 1, NULL, 0);
}

esp_err_t SCD30::set_temperature_offset(float t_offset)
{
    uint16_t raw = (uint16_t)(t_offset * 100);
    return set_temperature_offset_ticks(raw);
}

esp_err_t SCD30::get_sensor_altitude(uint16_t *altitude)
{
    CHECK_ARG(altitude);

    return execute_cmd(CMD_ALTITUDE_COMPENSATION, 1, NULL, 0, altitude, 1);
}

esp_err_t SCD30::set_sensor_altitude(uint16_t altitude)
{
    return execute_cmd(CMD_ALTITUDE_COMPENSATION, 1, &altitude, 1, NULL, 0);
}

esp_err_t SCD30::read_firmware_version(uint16_t *firmware_version)
{
    CHECK_ARG(firmware_version);
    return execute_cmd(CMD_READ_FIRMWARE_VERSION, 1, NULL, 0, firmware_version, 1);
}

esp_err_t SCD30::soft_reset()
{
    return execute_cmd(CMD_SOFT_RESET, 0, NULL, 0, NULL, 0);
}

uint8_t SCD30::crc8(const uint8_t *data, size_t count)
{
    uint8_t res = 0xff;

    for (size_t i = 0; i < count; ++i) {
        res ^= data[i];
        for (uint8_t bit = 8; bit > 0; --bit) {
            if (res & 0x80)
                res = (res << 1) ^ 0x31;
            else
                res = (res << 1);
        }
    }
    return res;
}

uint16_t SCD30::swap(uint16_t v)
{
    return (v << 8) | (v >> 8);
}

esp_err_t SCD30::send_cmd(uint16_t cmd, uint16_t *data, size_t words)
{
    uint8_t buf[2 + words * 3];
    // add command
    *(uint16_t *)buf = swap(cmd);
    if (data && words) {
        // add arguments
        for (size_t i = 0; i < words; i++) {
            uint8_t *p = buf + 2 + i * 3;
            *(uint16_t *)p = swap(data[i]);
            *(p + 2) = crc8(p, 2);
        }
    }
    ESP_LOGV(__func__, "Sending buffer:");
    ESP_LOG_BUFFER_HEX_LEVEL(__func__, buf, sizeof(buf), ESP_LOG_VERBOSE);

    return i2c_master_transmit(dev_handle_, buf, sizeof(buf), -1);
}

esp_err_t SCD30::read_resp(uint16_t *data, size_t words)
{
    uint8_t buf[words * 3];
    CHECK(i2c_master_receive(dev_handle_, buf, sizeof(buf), -1));

    ESP_LOGV(__func__, "Received buffer:");
    ESP_LOG_BUFFER_HEX_LEVEL(__func__, buf, sizeof(buf), ESP_LOG_VERBOSE);

    for (size_t i = 0; i < words; i++) {
        uint8_t *p = buf + i * 3;
        uint8_t crc = crc8(p, 2);
        if (crc != *(p + 2)) {
            ESP_LOGE(__func__, "Invalid CRC 0x%02x, expected 0x%02x", crc, *(p + 2));
            return ESP_ERR_INVALID_CRC;
        }
        data[i] = swap(*(uint16_t *)p);
    }
    return ESP_OK;
}

esp_err_t SCD30::execute_cmd(uint16_t cmd, uint32_t timeout_ms,
                             uint16_t *out_data, size_t out_words, uint16_t *in_data, size_t in_words)
{
    esp_err_t ret;
    ret = send_cmd(cmd, out_data, out_words);
    if (ret != ESP_OK)
        return ret;

    if (timeout_ms) {
        if (timeout_ms > 10)
            vTaskDelay(pdMS_TO_TICKS(timeout_ms));
        else
            ets_delay_us(timeout_ms * 1000);
    }
    if (in_data && in_words) {
        ret = read_resp(in_data, in_words);
        if (ret != ESP_OK)
            return ret;
    }

    return ESP_OK;
}
