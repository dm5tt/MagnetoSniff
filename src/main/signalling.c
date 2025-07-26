#include "signalling.h"

EventGroupHandle_t wifi_event_group;

void signalling_init(void)
{
    wifi_event_group = xEventGroupCreate();
}

void signalling_set_ip_acquired_event(void)
{
    xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
}

void signalling_clear_ip_acquired_event(void)
{
    xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
}

void signalling_set_time_set_event(void)
{
    xEventGroupSetBits(wifi_event_group, TIME_SET_BIT);
}

void signalling_clear_time_set_event(void)
{
    xEventGroupClearBits(wifi_event_group, TIME_SET_BIT);
}