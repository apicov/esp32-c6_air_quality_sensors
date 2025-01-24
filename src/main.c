
#include <stdatomic.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <esp_log.h>
#include <esp_err.h>

#include "driver/gpio.h"
#include "private_data.h"

//static const char *TAG = "cam-example";


#include <stdio.h>
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_http_client.h"
#include "nvs_flash.h"

#include "mqtt_client.h"
#include "bmp280.h"
#include "scd30.h"

static const char *TAG = "WIFI_STA"; // Tag for logging

static atomic_bool is_wifi_connected = false; // Atomic flag for Wi-Fi connection status

static esp_mqtt_client_handle_t mqtt_client = NULL; // Global MQTT client handle

static i2c_dev_t i2c_dev;

void scd30_task(void *p);
void bme280_task(void *p);


// Wi-Fi event handler
void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI(TAG, "Wi-Fi disconnected, retrying...");
        atomic_store(&is_wifi_connected, false); // Update flag atomically
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        atomic_store(&is_wifi_connected, true); // Update flag atomically
    }
}



// Initialize Wi-Fi in Station (STA) mode
void wifi_init_sta() {
    // Initialize the TCP/IP stack
    esp_netif_init();

    // Create the default event loop
    esp_event_loop_create_default();

    // Create a default network interface for Wi-Fi station
    esp_netif_create_default_wifi_sta();

    // Initialize Wi-Fi driver with default configuration
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    // Register event handlers for Wi-Fi and IP events
    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, &instance_any_id);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, &instance_got_ip);

    // Configure Wi-Fi connection settings
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = SSID,         // Replace with your Wi-Fi SSID
            .password = PASSWORD, // Replace with your Wi-Fi password
            .threshold.authmode = WIFI_AUTH_WPA2_PSK, // Minimum security mode
        },
    };

    // Set Wi-Fi mode to Station
    esp_wifi_set_mode(WIFI_MODE_STA);

    // Apply the Wi-Fi configuration
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);

    // Start the Wi-Fi driver
    esp_wifi_start();

    ESP_LOGI(TAG, "Wi-Fi initialized in Station mode");
}


// MQTT event handler
static void mqtt_event_handler_cb(void *handler_args, esp_event_base_t event_base,
                                  int32_t event_id, void *event_data) {

    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t) event_data;
    esp_mqtt_client_handle_t client = event->client;

    switch (event->event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        // Publish a message after connection
        esp_mqtt_client_publish(client, "/topic/test", "Hello from ESP32!", 0, 1, 0);
        break;

    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        break;

    case MQTT_EVENT_ERROR:
        ESP_LOGE(TAG, "MQTT_EVENT_ERROR");
        break;

    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;

    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        break;

    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;

    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        printf("Received topic: %.*s\n", event->topic_len, event->topic);
        printf("Received data: %.*s\n", event->data_len, event->data);
        break;

    default:
        ESP_LOGI(TAG, "Unhandled event id: %d", event->event_id);
        break;
    }
}

// Initialize and start the MQTT client
void mqtt_app_start(void) {

    // MQTT configuration
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = "mqtt://192.168.0.49:1883",  // Correct URI assignment
        .session.keepalive = 10,  // Set the keep-alive interval
    };

    // Initialize MQTT client
    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);

    // Register the event handler
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler_cb, NULL);

    // Start the MQTT client
    esp_mqtt_client_start(mqtt_client);

    ESP_LOGI(TAG, "MQTT client initialized and started");
}

void print_task_list(void) {
    // Get the number of tasks
    UBaseType_t task_count = uxTaskGetNumberOfTasks();

    // Allocate enough space to hold the task list info
    char *task_list = (char *) malloc(1024);

    if (task_list == NULL) {
        ESP_LOGE("Task List", "Memory allocation failed");
        return;
    }

    // Get the task list (task name, state, priority, etc.)
    vTaskList(task_list);

    // Print the list of tasks to the log
    ESP_LOGI("Task List", "\n%s", task_list);

    // Free the allocated memory
    free(task_list);
}


void publish_mqtt(void *p)
{
    ESP_LOGI(TAG, "hola");
    while(1)
    {
        
        vTaskDelay(pdMS_TO_TICKS(12000));
        // Publish a message from main
        if (mqtt_client != NULL) {
            ESP_LOGI(TAG, "Publishing message to topic /topic/main...");
            int msg_id = esp_mqtt_client_publish(mqtt_client, "/topic/main", "Message from app_main", 0, 1, 0);
            if (msg_id >= 0) {
                ESP_LOGI(TAG, "Message published successfully, msg_id=%d", msg_id);
            } else {
                ESP_LOGE(TAG, "Failed to publish message");
            }

            print_task_list();
        }
    }
}



void app_main() {
    //nvs_flash_init(); // Initialize NVS
    //wifi_init_sta();  // Initialize Wi-Fi as Station

    ESP_ERROR_CHECK(i2cdev_init()); // Initialize I2C bus

    // Wait for Wi-Fi connection
    //while (!atomic_load(&is_wifi_connected)) {
    //    ESP_LOGI(TAG, "Waiting for Wi-Fi connection...");
    //    vTaskDelay(pdMS_TO_TICKS(1000));
    //}

    ESP_LOGI(TAG, "Wi-Fi connected.");

    // Initialize MQTT
    //mqtt_app_start();

    // Wait until the MQTT client is connected
    //vTaskDelay(pdMS_TO_TICKS(10000)); // Delay for 2 seconds to ensure connection

    // Create a task to publish MQTT messages
    //xTaskCreate(publish_mqtt, "publish_mqtt", 4096, NULL, 5, NULL);
    xTaskCreate(scd30_task, "scd30_task", 4096, NULL, 5, NULL);
    //xTaskCreate(bme280_task, "scd30_task", 4096, NULL, 5, NULL);


}


void bme280_task(void *p)
{
    bmp280_params_t params;
    bmp280_init_default_params(&params);
    bmp280_t dev;
    memset(&dev, 0, sizeof(bmp280_t));

    ESP_ERROR_CHECK(bmp280_init_desc(&dev, BMP280_I2C_ADDRESS_0, I2C_NUM_0, GPIO_NUM_6, GPIO_NUM_7));
    ESP_ERROR_CHECK(bmp280_init(&dev, &params));

    bool bme280p = dev.id == BME280_CHIP_ID;
    printf("BMP280: found %s\n", bme280p ? "BME280" : "BMP280");

    float pressure, temperature, humidity;

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(500));
        if (bmp280_read_float(&dev, &temperature, &pressure, &humidity) != ESP_OK)
        {
            printf("Temperature/pressure reading failed\n");
            continue;
        }

        /* float is used in printf(). you need non-default configuration in
         * sdkconfig for ESP8266, which is enabled by default for this
         * example. see sdkconfig.defaults.esp8266
         */
        printf("Pressure: %.2f Pa, Temperature: %.2f C", pressure, temperature);
        if (bme280p)
            printf(", Humidity: %.2f\n", humidity);
        else
            printf("\n");
    }
}



void scd30_task(void *p)
    {
        i2c_dev_t dev;
        ESP_ERROR_CHECK(scd30_init_desc(&dev, I2C_NUM_0, GPIO_NUM_6, GPIO_NUM_7));
        

        uint16_t version, major_ver, minor_ver;

        ESP_ERROR_CHECK(scd30_read_firmware_version(&dev, &version));

        major_ver = (version >> 8) & 0xf;
        minor_ver = version & 0xf;

        ESP_LOGI(TAG, "SCD30 Firmware Version: %d.%d", major_ver, minor_ver);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        
        ESP_LOGI(TAG, "Starting continuous measurement");
        ESP_ERROR_CHECK(scd30_trigger_continuous_measurement(&dev, 0));

        float co2, temperature, humidity;
        bool data_ready;
        while (1)
        {
            vTaskDelay(pdMS_TO_TICKS(2000));

            scd30_get_data_ready_status(&dev, &data_ready);

            if (data_ready)
            {
                esp_err_t res = scd30_read_measurement(&dev, &co2, &temperature, &humidity);
                if (res != ESP_OK)
                {
                    ESP_LOGE(TAG, "Error reading results %d (%s)", res, esp_err_to_name(res));
                    continue;
                }

                if (co2 == 0)
                {
                    ESP_LOGW(TAG, "Invalid sample detected, skipping");
                    continue;
                }

                ESP_LOGI(TAG, "CO2: %.0f ppm", co2);
                ESP_LOGI(TAG, "Temperature: %.2f °C", temperature);
                ESP_LOGI(TAG, "Humidity: %.2f %%", humidity);
            }
        }
    } // end of producer






