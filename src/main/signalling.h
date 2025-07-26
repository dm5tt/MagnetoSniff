
#ifndef SIGNALLING_H
#define SIGNALLING_H

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#define WIFI_CONNECTED_BIT BIT0
#define TIME_SET_BIT BIT1

extern EventGroupHandle_t wifi_event_group;

void signalling_init(void);
void signalling_set_ip_acquired_event(void);
void signalling_clear_ip_acquired_event(void);
void signalling_set_time_set_event(void);
void signalling_clear_time_set_event(void);

#endif // SIGNALLING_H
