#include "i2c_test.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"

static const char *TAG = "i2c_scanner";

static int s_i2c_num = I2C_NUM_0;
static TaskHandle_t s_scanner_task_handle = NULL;
static uint32_t s_scan_interval_ms = 5000; // Default scan interval
static bool s_is_initialized = false;
static void i2c_scanner_task(void *pvParameters);

esp_err_t i2c_scanner_init(int sda_pin, int scl_pin, uint32_t freq_hz, int i2c_num)
{
    if (s_is_initialized)
    {
        ESP_LOGI(TAG, "I2C scanner already initialized");
        return ESP_OK;
    }

    s_i2c_num = i2c_num;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda_pin,
        .scl_io_num = scl_pin,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = freq_hz,

    };

    esp_err_t ret = i2c_param_config(s_i2c_num, &conf);

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "I2C parameter configuration failed");
        return ret;
    }

    ret = i2c_driver_install(s_i2c_num, I2C_MODE_MASTER, 0, 0, 0);

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "I2C driver installation failed");
        return ret;
    }

    s_is_initialized = true;
    ESP_LOGI(TAG, "I2C scanner initialized successfully (SDA: %d, SCL: %d, Freq: %lu Hz)",
             sda_pin, scl_pin, freq_hz);
    return ESP_OK;
}

esp_err_t i2c_scanner_scan_once(void)
{
    if (!s_is_initialized)
    {
        ESP_LOGE(TAG, "I2C scanner not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t devices_found = 0;
    ESP_LOGI(TAG, "Scanning I2C bus...");

    for (uint8_t i = 1; i < 128; i++)
    {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (i << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(s_i2c_num, cmd, 10 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);

        if (ret == ESP_OK)
        {
            ESP_LOGI(TAG, "Device found at address 0x%02X", i);
            devices_found++;
        }
    }

    if (devices_found == 0)
    {
        ESP_LOGI(TAG, "No I2C devices found");
    }
    else
    {
        ESP_LOGI(TAG, "Scan complete, %d device(s) found", devices_found);
    }

    return ESP_OK;
}

static void i2c_scanner_task(void *pvParameters)
{
    while (1)
    {
        i2c_scanner_scan_once();
        vTaskDelay(s_scan_interval_ms / portTICK_PERIOD_MS);
    }

    // This should never be reached

    vTaskDelete(NULL);
}

esp_err_t i2c_scanner_start_task(uint32_t scan_interval_ms, int task_priority, uint32_t stack_size)
{
    if (!s_is_initialized)
    {
        ESP_LOGE(TAG, "I2C scanner not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (s_scanner_task_handle != NULL)
    {
        ESP_LOGI(TAG, "I2C scanner task already running");
        return ESP_OK;
    }

    s_scan_interval_ms = scan_interval_ms;

    BaseType_t task_created = xTaskCreate(
        i2c_scanner_task,
        "i2c_scanner_task",
        stack_size,
        NULL,
        task_priority,
        &s_scanner_task_handle);

    if (task_created != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create I2C scanner task");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "I2C scanner task started with interval %lu ms", scan_interval_ms);

    return ESP_OK;
}

esp_err_t i2c_scanner_stop_task(void)
{
    if (s_scanner_task_handle == NULL)
    {
        ESP_LOGI(TAG, "I2C scanner task not running");
        return ESP_OK;
    }

    vTaskDelete(s_scanner_task_handle);

    s_scanner_task_handle = NULL;

    ESP_LOGI(TAG, "I2C scanner task stopped");

    return ESP_OK;
}