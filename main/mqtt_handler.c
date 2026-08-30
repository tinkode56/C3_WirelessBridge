#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "mqtt_client.h"
#include "esp_log.h"
#include "inverter_uart.h"
#include "pi18_parser.h"

static const char *TAG = "MQTT_HANDLER";

static esp_mqtt_client_handle_t s_mqtt_client = NULL;
static bool s_is_connected = false;

/* Global command queue shared across files for incoming MQTT commands */
extern QueueHandle_t g_cmd_queue;

/**
 * @brief MQTT Event Handler Loop
 */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;

    switch ((esp_mqtt_event_id_t)event_id)
    {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED to broker");
        s_is_connected = true;

        /* Publish online LWT status */
        esp_mqtt_client_publish(s_mqtt_client, CONFIG_C3WB_MQTT_TOPIC_STATUS, "online", 0, 1, 1);

        /* Subscribe to incoming on-demand command topic */
        esp_mqtt_client_subscribe(s_mqtt_client, CONFIG_C3WB_MQTT_TOPIC_COMMAND, 1);
        ESP_LOGI(TAG, "Subscribed to command topic: %s", CONFIG_C3WB_MQTT_TOPIC_COMMAND);
        break;

    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "MQTT_EVENT_DISCONNECTED from broker");
        s_is_connected = false;
        break;

    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT_EVENT_DATA received from topic: %.*s", event->topic_len, event->topic);

        if (event->data_len > 0)
        {
            /* Allocate memory for null-terminated command payload */
            char *cmd_payload = malloc(event->data_len + 1);
            if (cmd_payload != NULL)
            {
                memcpy(cmd_payload, event->data, event->data_len);
                cmd_payload[event->data_len] = '\0';

                /* Strip any trailing newlines or whitespace from CLI testing */
                size_t len = strlen(cmd_payload);
                while (len > 0 && (cmd_payload[len - 1] == '\r' || cmd_payload[len - 1] == '\n' || cmd_payload[len - 1] == ' '))
                {
                    cmd_payload[--len] = '\0';
                }

                ESP_LOGI(TAG, "Enqueuing command for UART execution: %s", cmd_payload);

                /* Send pointer to queue (non-blocking) */
                if (g_cmd_queue != NULL && xQueueSend(g_cmd_queue, &cmd_payload, 0) != pdTRUE)
                {
                    ESP_LOGW(TAG, "Command queue full, dropping command: %s", cmd_payload);
                    free(cmd_payload);
                }
            }
        }
        break;

    case MQTT_EVENT_ERROR:
        ESP_LOGE(TAG, "MQTT_EVENT_ERROR: type %d", event->error_handle->error_type);
        break;

    default:
        break;
    }
}

/**
 * @brief FreeRTOS Task: Dequeues JSON telemetry and publishes to MQTT
 */
void mqtt_publisher_task(void *pvParameters)
{
    QueueHandle_t output_queue = (QueueHandle_t)pvParameters;
    char *json_payload = NULL;

    if (output_queue == NULL)
    {
        ESP_LOGE(TAG, "mqtt_publisher_task created with NULL queue handle!");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "MQTT Publisher Task active and waiting for JSON payloads...");

    while (1)
    {
        /* Queue holds (char *) pointers allocated by cJSON_PrintUnformatted */
        if (xQueueReceive(output_queue, &json_payload, portMAX_DELAY) == pdTRUE)
        {
            if (json_payload != NULL)
            {
                const char *target_topic = CONFIG_C3WB_MQTT_TOPIC_TELEMETRY;

                if (strstr(json_payload, "SETTING_RESPONSE") != NULL)
                {
                    target_topic = CONFIG_C3WB_MQTT_TOPIC_COMMAND_RESPONSE;
                }

                if (s_is_connected)
                {
                    int msg_id = esp_mqtt_client_publish(s_mqtt_client, target_topic, json_payload, 0, 1, 0);
                    if (msg_id >= 0)
                    {
                        ESP_LOGI(TAG, "Published (msg_id=%d) to %s: %s", msg_id, target_topic, json_payload);
                    }
                    else
                    {
                        ESP_LOGE(TAG, "Failed publishing to %s", target_topic);
                    }
                }

                /* Free heap space allocated by parser */
                free(json_payload);
            }
        }
    }
}

/**
 * @brief Initializes MQTT client and starts the publisher task
 */
void mqtt_app_start(QueueHandle_t mqtt_queue)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = CONFIG_C3WB_MQTT_BROKER_URL,
        /* Optional Last Will and Testament (LWT) setup */
        .session.last_will = {
            .topic = CONFIG_C3WB_MQTT_TOPIC_STATUS,
            .msg = "offline",
            .msg_len = 7,
            .qos = 1,
            .retain = 1,
        },
    };

    s_mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(s_mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(s_mqtt_client);

    /* Create the publishing task, passing the QueueHandle_t via pvParameters */
    xTaskCreate(mqtt_publisher_task, "mqtt_publisher_task", 4096, (void *)mqtt_queue, 4, NULL);
}
