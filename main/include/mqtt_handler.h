#if !defined(MQTT_HANDLER_H)
#define MQTT_HANDLER_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

void mqtt_app_start(QueueHandle_t mqtt_queue);

#endif // MQTT_HANDLER_H
