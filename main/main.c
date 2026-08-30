#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "wifi_sta.h"
#include "inverter_uart.h"
#include "pi18_parser.h"
#include "mqtt_handler.h"

static const char *TAG = "main_app";

/* Allocate globally or statically so memory remains valid across tasks */
static parser_task_args_t parser_args;

static QueueHandle_t inverter_queue = NULL; /* Raw UART frames */
static QueueHandle_t mqtt_queue = NULL;     /* Outgoing JSON strings */
QueueHandle_t g_cmd_queue = NULL;           /* Incoming MQTT commands */

void app_main(void)
{
    /* Start the isolated Wi-Fi manager stack */
    /* Spawns background lwIP and Wi-Fi driver tasks automatically with NTP sync */
    wifi_manager_init();

    /* Queue for UART -> Parser */
    inverter_queue = xQueueCreate(10, sizeof(inverter_payload_t));
    if (inverter_queue == NULL) {
        ESP_LOGE(TAG, "UART Queue allocation failed!");
        return;
    }

    /* Queue for Parser -> MQTT (holds char* heap pointers) */
    mqtt_queue = xQueueCreate(10, sizeof(char *));
    if (mqtt_queue == NULL) {
        ESP_LOGE(TAG, "MQTT Queue allocation failed!");
        return;
    }

    /* Queue for handling on-demand commands from MQTT */
    g_cmd_queue = xQueueCreate(10, sizeof(char *));
    if (g_cmd_queue == NULL) {
        ESP_LOGE("MAIN", "Failed to allocate g_cmd_queue");
        return;
    }
    
    /* Start MQTT Client (passes mqtt_queue handle so publisher task can consume from it) */
    mqtt_app_start(mqtt_queue);
    
    /* Prepare argument container for parser task */
    parser_args.raw_uart_queue = inverter_queue;
    parser_args.mqtt_queue     = mqtt_queue;
    
    /* Create and launch inverter polling via UART task */
    xTaskCreate(inverter_polling_task, "inverter_uart", 4096, (void *)inverter_queue, 5, NULL);
    /* Creat and launch PI18 parser task*/
    xTaskCreate(inverter_parser_task, "pi18_parser", 4096, (void *)&parser_args, 6, NULL);
}
