#include <time.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_rom_sys.h"

#include "task.h"
#include "signalling.h"
#include "i2c_user.h"
#include "system_settings.h"
#include "sender.h"

#include "bmm350_hal.h"
#include "bmm350.h"

static sender_mag_sample_t bufferA[SENDER_SAMPLES_PER_BUFFER];
static sender_mag_sample_t bufferB[SENDER_SAMPLES_PER_BUFFER];
static TaskHandle_t task_handle = NULL;

static const char *TAG = "APP_TASK";

void IRAM_ATTR sensor_gpio_isr_handler(void *arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(task_handle, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken)
    {
        portYIELD_FROM_ISR();
    }
}

static void task_example(void *pvParameters)
{
    /* Initialize Interrupts */

    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_POSEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = 1ULL << SYSTEM_SETTINGS_INTERRUPT,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE};
    gpio_config(&io_conf);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(SYSTEM_SETTINGS_INTERRUPT, sensor_gpio_isr_handler, NULL);

    /* Status of api are returned to this variable */
    int8_t rslt;

    /* Sensor initialization configuration */
    struct bmm350_dev dev = {0};

    uint8_t int_ctrl, err_reg_data = 0;
    uint8_t set_int_ctrl;

    struct bmm350_mag_temp_data mag_temp_data;
    struct bmm350_pmu_cmd_status_0 pmu_cmd_stat_0;

    /* Update device structure */
    rslt = bmm350_interface_init(&dev);
    bmm350_error_codes_print_result("bmm350_interface_selection", rslt);

    /* Initialize BMM350 */
    rslt = bmm350_init(&dev);
    bmm350_error_codes_print_result("bmm350_init", rslt);

    ESP_LOGI(TAG, "Read : 0x00 : BMM350 Chip ID : 0x%X\n", dev.chip_id);

    /* Check PMU busy */
    rslt = bmm350_get_pmu_cmd_status_0(&pmu_cmd_stat_0, &dev);
    bmm350_error_codes_print_result("bmm350_get_pmu_cmd_status_0", rslt);

    ESP_LOGI(TAG, "Expected : 0x07 : PMU cmd busy : 0x0\n");
    ESP_LOGI(TAG, "Read : 0x07 : PMU cmd busy : 0x%X\n", pmu_cmd_stat_0.pmu_cmd_busy);

    /* Get error data */
    rslt = bmm350_get_regs(BMM350_REG_ERR_REG, &err_reg_data, 1, &dev);
    bmm350_error_codes_print_result("bmm350_get_error_reg_data", rslt);

    ESP_LOGI(TAG, "Expected : 0x02 : Error Register : 0x0\n");
    ESP_LOGI(TAG, "Read : 0x02 : Error Register : 0x%X\n", err_reg_data);

    /* Configure interrupt settings */
    rslt =
        bmm350_configure_interrupt(BMM350_PULSED, BMM350_ACTIVE_HIGH, BMM350_INTR_OPEN_DRAIN, BMM350_MAP_TO_PIN, &dev);
    bmm350_error_codes_print_result("bmm350_configure_interrupt", rslt);

    rslt = bmm350_delay_us(10000, &dev);
    bmm350_error_codes_print_result("bmm350_delay_us", rslt);

    /* Enable data ready interrupt */
    rslt = bmm350_enable_interrupt(BMM350_ENABLE_INTERRUPT, &dev);
    bmm350_error_codes_print_result("bmm350_enable_interrupt", rslt);

    rslt = bmm350_delay_us(10000, &dev);
    bmm350_error_codes_print_result("bmm350_delay_us", rslt);

    /* Get interrupt settings */
    rslt = bmm350_get_regs(BMM350_REG_INT_CTRL, &int_ctrl, 1, &dev);
    bmm350_error_codes_print_result("bmm350_get_regs", rslt);

    rslt = bmm350_delay_us(10000, &dev);
    bmm350_error_codes_print_result("bmm350_delay_us", rslt);

    set_int_ctrl = ((BMM350_INT_POL_ACTIVE_HIGH << 1) | (BMM350_INT_OD_PUSHPULL << 2) | (BMM350_ENABLE << 3) | BMM350_ENABLE << 7);

    ESP_LOGI(TAG, "Expected : 0x2E : Interrupt control : 0x%X\n", set_int_ctrl);
    ESP_LOGI(TAG, "Read : 0x2E : Interrupt control : 0x%X\n", int_ctrl);

    if (int_ctrl & BMM350_DRDY_DATA_REG_EN_MSK)
    {
        ESP_LOGI(TAG, "Data ready enabled\r\n");
    }

    /* Set ODR and performance */
    rslt = bmm350_set_odr_performance(BMM350_DATA_RATE_400HZ, BMM350_AVG_NO_AVG, &dev);
    bmm350_error_codes_print_result("bmm350_set_odr_performance", rslt);

    rslt = bmm350_delay_us(10000, &dev);
    bmm350_error_codes_print_result("bmm350_delay_us", rslt);

    /* Enable all axis */
    rslt = bmm350_enable_axes(BMM350_X_EN, BMM350_Y_EN, BMM350_Z_EN, &dev);
    bmm350_error_codes_print_result("bmm350_enable_axes", rslt);

    if (rslt == BMM350_OK)
    {
        rslt = bmm350_set_pad_drive(BMM350_PAD_DRIVE_STRONGEST, &dev);
        bmm350_error_codes_print_result("bmm350_set_pad_drive", rslt);

        rslt = bmm350_set_powermode(BMM350_NORMAL_MODE, &dev);
        bmm350_error_codes_print_result("bmm350_set_powermode", rslt);

        ESP_LOGI(TAG, "\nPower mode is set to normal mode\n");
        ESP_LOGI(TAG, "Compensated Magnetometer and temperature data with delay\n");
    }
    else
    {
        ESP_LOGE(TAG, "Error while initializing BMM350\r\n");
        while (1)
            vTaskDelay(100);
    }

    sender_mag_sample_t *active_buffer = bufferA;
    size_t sample_index = 0;
    struct timeval tv;
    sender_mag_sample_t sample;

    while (1)
    {
        EventBits_t bits = xEventGroupWaitBits(wifi_event_group,
                                               WIFI_CONNECTED_BIT | TIME_SET_BIT,
                                               pdFALSE,
                                               pdTRUE,
                                               portMAX_DELAY);

#
        if (bits & WIFI_CONNECTED_BIT)
        {
            while (xEventGroupGetBits(wifi_event_group) & WIFI_CONNECTED_BIT)
            {

                /* Wait for Interrupt */
                ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

                gettimeofday(&tv, NULL);
                sample.timestamp = (uint64_t)tv.tv_sec * 1000 + tv.tv_usec / 1000;

                int rslt = bmm350_get_compensated_mag_xyz_temp_data((void *)&mag_temp_data, &dev);
                bmm350_error_codes_print_result("bmm350_get_compensated_mag_xyz_temp_data", rslt);

                sample.x = mag_temp_data.x;
                sample.y = mag_temp_data.y;
                sample.z = mag_temp_data.z;
                sample.temperature = mag_temp_data.temperature;

                active_buffer[sample_index++] = sample;

                if (sample_index >= SENDER_SAMPLES_PER_BUFFER)
                {
                    ESP_LOGI(TAG, "Full");
                    sender_send_buffer(active_buffer, sample_index);

                    if (active_buffer == bufferA)
                        active_buffer = bufferB;
                    else
                        active_buffer = bufferA;

                    sample_index = 0;
                }
            }
            ESP_LOGI(TAG, "Lost Wifi. Waiting.");
        }
    }
}

void task_example_create(void)
{
    xTaskCreate(task_example, "example_task", 4096, NULL, 5, &task_handle);
}