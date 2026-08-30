#if !defined(PI18_PARSER_H)
#define PI18_PARSER_H

#include "inverter_uart.h"

typedef struct {
    QueueHandle_t raw_uart_queue; /* From Polling Task */
    QueueHandle_t mqtt_queue;     /* To MQTT Task */
} parser_task_args_t;

char *pi18_parse_to_json(const inverter_payload_t *payload);

void inverter_parser_task(void *pvParameters);


#endif // PI18_PARSER_H
