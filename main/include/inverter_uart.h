#if !defined(INVERTER_UART_H)
#define INVERTER_UART_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

typedef enum {
    CMD_PI18_UNKNOWN = 0,
    CMD_PI18_ID,
    CMD_PI18_GS,
    CMD_PI18_MOD,
    CMD_PI18_FWS,
    CMD_PI18_FLAG,
    CMD_PI18_PIRI,
    CMD_PI18_ET,
    CMD_PI18_PRI0,
    CMD_PI18_PGS0,
} pi18_cmd_id_t;

/* Define shared data structure for the queue */
typedef struct
{
    pi18_cmd_id_t cmd_id;
    char raw_data[256]; /* Safely scaled to 256 bytes for worst-case GS frames */
    uint16_t length;
} inverter_payload_t;

/* Shared inter-task queue handle for incoming MQTT on-demand commands */
extern QueueHandle_t g_cmd_queue;

/**
 * @brief Initializes UART1 with the configured protocol parameters.
 */
esp_err_t inverter_uart_init(void);

/**
 * @brief The FreeRTOS hardware polling task loop.
 */
void inverter_polling_task(void *pvParameters);

#endif // INVERTER_UART_H
