#include "sntp.h"

#include <time.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_sntp.h"

#include "signalling.h"

#define SNTP_RETRY_COUNT 15

static const char *TAG = "SNTP";

static void sntp_task(void *pvParameters)
{
    xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdFALSE, portMAX_DELAY);

    ESP_LOGI(TAG, "Initializing SNTP");
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "ptbtime1.ptb.de");
    esp_sntp_set_sync_mode(SNTP_SYNC_MODE_SMOOTH);
    esp_sntp_init();

    time_t now = 0;
    struct tm timeinfo = {0};
    int retry = 0;

    while (retry++ < SNTP_RETRY_COUNT)
    {
        sntp_sync_status_t status = sntp_get_sync_status();

        if (status == SNTP_SYNC_STATUS_COMPLETED)
        {
            time(&now);
            localtime_r(&now, &timeinfo);
            char strftime_buf[64];
            strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
            ESP_LOGI(TAG, "Time sync successful. Time: %s", strftime_buf);
            setenv("TZ", "CET-1CEST,M3.5.0,M10.5.0/3", 1);
            tzset();
            signalling_set_time_set_event();
            break;
        }
        else if (status == SNTP_SYNC_STATUS_IN_PROGRESS)
        {
            ESP_LOGI(TAG, "Time sync in progress (%d/%d)...", retry, SNTP_RETRY_COUNT);
        }
        else
        {
            ESP_LOGI(TAG, "Waiting for SNTP sync (%d/%d)...", retry, SNTP_RETRY_COUNT);
        }

        vTaskDelay(pdMS_TO_TICKS(2000));
    } 

    if(retry >= SNTP_RETRY_COUNT) {
        esp_restart();
    }

    esp_sntp_stop();
    vTaskDelete(NULL);
}

void sntp_init_user(void)
{
    xTaskCreate(sntp_task, "sntp_task", 4096, NULL, 6, NULL);
}
