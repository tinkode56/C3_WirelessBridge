#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "inverter_uart.h"

#define UART_PORT_NUM UART_NUM_1
#define MAX_PI18_LEN 128

#define CMD_COUNT (sizeof(query_table) / sizeof(query_table[0]))

#define UART_PORT_NUM UART_NUM_1
#define UART_TX_PIN (CONFIG_C3WB_TTL_TX_PIN)
#define UART_RX_PIN (CONFIG_C3WB_TTL_RX_PIN)
#define UART_BUF_SIZE (256)

static const char *TAG = "inv_uart";

typedef struct
{
    const char *cmd_str;
    pi18_cmd_id_t cmd_id;
} pi18_query_t;

/* Command tracking definitions */
static const pi18_query_t query_table[] = {
    {"^P005ID", CMD_PI18_ID},     /* Protocol ID Query */
    {"^P005GS", CMD_PI18_GS},     /* General Status Query */
    {"^P006MOD", CMD_PI18_MOD},   /* Working Mode Query */
    {"^P005FWS", CMD_PI18_FWS},   /* Fault and Warning Status Query */
    {"^P007FLAG", CMD_PI18_FLAG}, /* Enable/Disable Flag Status Query */
    {"^P007PIRI", CMD_PI18_PIRI}, /* Rated information Query */
    {"^P005ET", CMD_PI18_ET},     /* Total generated energy Query */
    {"^P007PRI0", CMD_PI18_PRI0}, /* Rated information of parallel system Query */
    {"^P007PGS0", CMD_PI18_PGS0}, /* General status of parallel system Query */
};

/**
 * @brief Calculates CRC-16/XMODEM checksum for PI18 protocol commands and frames.
 *        Polynomial: 0x1021, Init: 0x0000
 *
 * @param data Pointer to the buffer string/bytes
 * @param len Length of data to include in checksum
 * @return uint16_t The calculated 16-bit CRC value
 */
uint16_t pi18_calc_crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0x0000;

    for (size_t i = 0; i < len; i++)
    {
        crc ^= ((uint16_t)data[i] << 8);
        for (uint8_t bit = 0; bit < 8; bit++)
        {
            if (crc & 0x8000)
            {
                crc = (crc << 1) ^ 0x1021;
            }
            else
            {
                crc = (crc << 1);
            }
        }
    }
    return crc;
}

esp_err_t inverter_uart_init(void)
{

    uart_config_t uart_config = {
        .baud_rate = CONFIG_C3WB_TTL_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT, // Uses APB clock automatically
    };

    esp_err_t err = uart_driver_install(
        UART_PORT_NUM,
        UART_BUF_SIZE * 2, // RX ring buffer size (512 bytes)
        0,                 // TX ring buffer size (0 because we write synchronously)
        0,                 // UART event queue handle (not using event loop here)
        NULL,              // Event queue pointer
        0                  // Interrupt allocation flags
    );

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to install UART driver: %s", esp_err_to_name(err));
        return err;
    }

    err = uart_param_config(UART_PORT_NUM, &uart_config);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to configure UART parameters: %s", esp_err_to_name(err));
        return err;
    }

    err = uart_set_pin(
        UART_PORT_NUM,
        UART_TX_PIN,
        UART_RX_PIN,
        UART_PIN_NO_CHANGE, // RTS pin not connected
        UART_PIN_NO_CHANGE  // CTS pin not connected
    );

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to route UART pins: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "UART1 successfully initialized on TX:%d, RX:%d", UART_TX_PIN, UART_RX_PIN);
    return ESP_OK;
}

void inverter_polling_task(void *pvParameters)
{
    QueueHandle_t output_queue = (QueueHandle_t)pvParameters;
    inverter_payload_t tx_frame;
    int current_cmd_idx = 0;

    if (inverter_uart_init() != ESP_OK)
    {
        ESP_LOGE(TAG, "Critical hardware initialization failure. Halting task.");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Inverter Polling Task active and ready.");

    while (1)
    {
        char *custom_cmd = NULL;
        const char *cmd_to_send = NULL;
        uint8_t current_cmd_id = 0;
        bool is_custom_cmd = false;

        /* Priority Check: Is there an on-demand command waiting from MQTT? */
        if (g_cmd_queue && (xQueueReceive(g_cmd_queue, &custom_cmd, 0) == pdTRUE))
        {
            /* Sanitize: Strip trailing \r or \n if present in raw MQTT message */
            size_t len = strlen(custom_cmd);
            while ((len > 0) && (custom_cmd[len - 1] == '\r' || custom_cmd[len - 1] == '\n'))
            {
                custom_cmd[--len] = '\0';
            }

            cmd_to_send = custom_cmd;
            is_custom_cmd = true;
            current_cmd_id = 0xFF; /* Fallback ID for dynamic / setting commands */

            /* Attempt to match custom string against query_table to resolve command ID */
            for (int i = 0; i < CMD_COUNT; i++)
            {
                if (strcmp(query_table[i].cmd_str, custom_cmd) == 0)
                {
                    current_cmd_id = query_table[i].cmd_id;
                    break;
                }
            }

            ESP_LOGI(TAG, "Executing priority on-demand command: %s", cmd_to_send);
        }
        else
        {
            /* Routine Path: Grab next scheduled poll command from query_table */
            cmd_to_send = query_table[current_cmd_idx].cmd_str;
            current_cmd_id = query_table[current_cmd_idx].cmd_id;
        }

        /* Build the PI18 UART transmit frame for ANY command string */
        char send_buf[64];
        int cmd_text_len = snprintf(send_buf, sizeof(send_buf), "%s", cmd_to_send);

        /* Dynamically compute CRC over the input string (e.g. "^S008V2020") */
        uint16_t crc = pi18_calc_crc16((const uint8_t *)send_buf, cmd_text_len);

        send_buf[cmd_text_len] = (uint8_t)((crc >> 8) & 0xFF); /* CRC High Byte */
        send_buf[cmd_text_len + 1] = (uint8_t)(crc & 0xFF);    /* CRC Low Byte */
        send_buf[cmd_text_len + 2] = '\r';                     /* Trailing carriage return */

        int cmd_len = cmd_text_len + 3;

        /* Flush rx buffer and send request over UART */
        uart_flush(UART_PORT_NUM);
        uart_write_bytes(UART_PORT_NUM, send_buf, cmd_len);

        /* Read response from inverter (1000ms timeout) */
        int len = uart_read_bytes(UART_PORT_NUM, (uint8_t *)tx_frame.raw_data, MAX_PI18_LEN - 1, pdMS_TO_TICKS(1000));
        if (len > 0)
        {
            tx_frame.raw_data[len] = '\0';
            tx_frame.length = len;
            tx_frame.cmd_id = current_cmd_id;

            ESP_LOGW(TAG, "Received raw response (%d bytes): %s", len, tx_frame.raw_data);

            /* Accepts Data (^D), ACK (^1), or NACK (^0) headers */
            if (tx_frame.raw_data[0] == '^' &&
                (tx_frame.raw_data[1] == 'D' || tx_frame.raw_data[1] == '1' || tx_frame.raw_data[1] == '0'))
            {
                xQueueSend(output_queue, &tx_frame, 0);
            }
            else
            {
                ESP_LOGE(TAG, "Malformed PI18 response header: 0x%02X 0x%02X",
                         (uint8_t)tx_frame.raw_data[0], (uint8_t)tx_frame.raw_data[1]);
            }
        }
        else
        {
            ESP_LOGE(TAG, "UART read timeout for command: %s", cmd_to_send);
        }

        /* Cleanup custom dynamic memory and manage cycle pacing */
        if (is_custom_cmd)
        {
            if (custom_cmd)
            {
                free(custom_cmd);
            }
        }
        else
        {
            current_cmd_idx = (current_cmd_idx + 1) % CMD_COUNT;
        }

        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
