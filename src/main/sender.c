#include "sender.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "lwip/sockets.h"

#include "esp_log.h"

#include "system_settings.h"

static const char *TAG = "sender";

static int sender_udp_sock = -1;
static struct sockaddr_in sender_dest_addr;

static QueueHandle_t sender_send_queue = NULL;

typedef struct {
    sender_mag_sample_t *buffer;
    size_t length;
} sender_send_packet_t;

static void sender_udp_send_task(void *param)
{
    sender_send_packet_t packet;

    while (1)
    {
        if (xQueueReceive(sender_send_queue, &packet, portMAX_DELAY) == pdTRUE)
        {
            size_t byte_len = packet.length * sizeof(sender_mag_sample_t);

            int ret = sendto(sender_udp_sock, packet.buffer, byte_len, 0,
                             (struct sockaddr *)&sender_dest_addr, sizeof(sender_dest_addr));
            if (ret < 0)
            {
                ESP_LOGE(TAG, "UDP send failed");
            }
            else
            {
                //ESP_LOGI(TAG, "UDP sent %d bytes", ret);
            }
        }
    }
}

void sender_init(void)
{
    sender_udp_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sender_udp_sock < 0)
    {
        ESP_LOGE(TAG, "Failed to create UDP socket");
        return;
    }

    memset(&sender_dest_addr, 0, sizeof(sender_dest_addr));
    sender_dest_addr.sin_family = AF_INET;
    sender_dest_addr.sin_port = htons(SYSTEM_SETTINGS_UDP_PORT);
    sender_dest_addr.sin_addr.s_addr = inet_addr(SYSTEM_SETTINGS_UDP_IP);

    sender_send_queue = xQueueCreate(4, sizeof(sender_send_packet_t));
    if (!sender_send_queue)
    {
        ESP_LOGE(TAG, "Failed to create send queue");
        return;
    }

    xTaskCreate(sender_udp_send_task, "sender_udp_send_task", 16384, NULL, 5, NULL);

    ESP_LOGI(TAG, "Sender initialized");
}

void sender_send_buffer(sender_mag_sample_t *buffer, size_t length)
{
    sender_send_packet_t packet = {
        .buffer = buffer,
        .length = length,
    };

    if (xQueueSend(sender_send_queue, &packet, 0) != pdTRUE)
    {
        ESP_LOGW(TAG, "Send queue full, dropping packet");
    }
}
