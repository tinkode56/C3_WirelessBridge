#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "esp_log.h"
#include "cJSON.h"
#include "pi18_parser.h"
#include "inverter_uart.h"
#include <sys/time.h>

static const char *TAG = "PI18_PARSER";

static const char* get_pi18_cmd_string(uint8_t cmd_id)
{
    switch (cmd_id) {
        case 0x01: return "ID";    
        case 0x02: return "GS";    
        case 0x03: return "MOD";    
        case 0x04: return "FWS";    
        case 0x05: return "FLAG";    
        case 0x06: return "PIRI";    
        case 0x07: return "ET";   
        case 0x08: return "PRI0";  
        case 0x09: return "PGS0";  
        default:   return "UNKNOWN";
    }
}

static void parse_serial_number(cJSON *root, char *data_start)
{
    size_t len = strlen(data_start);
    if (len > 3)
    {
        data_start[len - 3] = '\0';
    }
    cJSON_AddStringToObject(root, "serial_number", data_start);
}

static void parse_general_status(cJSON *root, char *data_start)
{
    char *running_str = data_start;
    char *token = NULL;
    int idx = 0;

    /* strsep returns "" (empty string) for double commas ",," instead of skipping */
    while ((token = strsep(&running_str, ",")) != NULL)
    {
        /* Strip CRC + '\r' on the final token (Index 27) */
        if (idx == 27)
        {
            size_t tlen = strlen(token);
            if (tlen > 3)
            {
                token[tlen - 3] = '\0';
            }
        }

        /* If field is empty (double comma encountered), skip adding to JSON */
        if (strlen(token) == 0)
        {
            idx++;
            continue; /* Leaves the index aligned while skipping missing data */
        }

        switch (idx)
        {
        /* AC Input & Output Parameters */
        case 0:
            cJSON_AddNumberToObject(root, "grid_voltage", atof(token) * 0.1);
            break;
        case 1:
            cJSON_AddNumberToObject(root, "grid_frequency", atof(token) * 0.1);
            break;
        case 2:
            cJSON_AddNumberToObject(root, "ac_output_voltage", atof(token) * 0.1);
            break;
        case 3:
            cJSON_AddNumberToObject(root, "ac_output_frequency", atof(token) * 0.1);
            break;
        case 4:
            cJSON_AddNumberToObject(root, "ac_output_apparent_power", atoi(token));
            break;
        case 5:
            cJSON_AddNumberToObject(root, "ac_output_active_power", atoi(token));
            break;
        case 6:
            cJSON_AddNumberToObject(root, "output_load_percent", atoi(token));
            break;

        /* Battery Parameters */
        case 7:
            cJSON_AddNumberToObject(root, "battery_voltage", atof(token) * 0.1);
            break;
        case 8:
            cJSON_AddNumberToObject(root, "battery_voltage_scc1", atof(token) * 0.1);
            break;
        case 9:
            cJSON_AddNumberToObject(root, "battery_voltage_scc2", atof(token) * 0.1);
            break;
        case 10:
            cJSON_AddNumberToObject(root, "battery_discharge_current", atoi(token));
            break;
        case 11:
            cJSON_AddNumberToObject(root, "battery_charging_current", atoi(token));
            break;
        case 12:
            cJSON_AddNumberToObject(root, "battery_capacity", atoi(token));
            break;

        /* Thermal Parameters */
        case 13:
            cJSON_AddNumberToObject(root, "inverter_heatsink_temp", atoi(token));
            break;
        case 14:
            cJSON_AddNumberToObject(root, "mppt1_charger_temp", atoi(token));
            break;
        case 15:
            cJSON_AddNumberToObject(root, "mppt2_charger_temp", atoi(token));
            break;

        /* Solar (PV) Input Parameters */
        case 16:
            cJSON_AddNumberToObject(root, "pv1_input_power", atoi(token));
            break;
        case 17:
            cJSON_AddNumberToObject(root, "pv2_input_power", atoi(token));
            break;
        case 18:
            cJSON_AddNumberToObject(root, "pv1_input_voltage", atof(token) * 0.1);
            break;
        case 19:
            cJSON_AddNumberToObject(root, "pv2_input_voltage", atof(token) * 0.1);
            break;

        /* Status & Charger Flags */
        case 20:
            cJSON_AddBoolToObject(root, "configuration_changed", atoi(token));
            break;
        case 21:
            cJSON_AddNumberToObject(root, "mppt1_charger_status", atoi(token));
            break;
        case 22:
            cJSON_AddNumberToObject(root, "mppt2_charger_status", atoi(token));
            break;
        case 23:
            cJSON_AddBoolToObject(root, "load_connection", atoi(token));
            break;

        /* Power Directions */
        case 24:
            cJSON_AddNumberToObject(root, "battery_power_direction", atoi(token));
            break;
        case 25:
            cJSON_AddNumberToObject(root, "dc_ac_power_direction", atoi(token));
            break;
        case 26:
            cJSON_AddNumberToObject(root, "line_power_direction", atoi(token));
            break;

        /* System Info */
        case 27:
            cJSON_AddNumberToObject(root, "local_parallel_id", atoi(token));
            break;

        default:
            break;
        }

        idx++;
    }
}

static void parse_working_mode(cJSON *root, char *data_start)
{
    /* Mode structure variant: ^D005XX<CRC><cr> */
    size_t len = strlen(data_start);
    if (len > 3)
    {
        data_start[len - 3] = '\0';
    }

    int mode = atoi(data_start);
    cJSON_AddNumberToObject(root, "working_mode_raw", mode);

    switch (mode)
    {
    case 0:
        cJSON_AddStringToObject(root, "working_mode", "Power On");
        break;
    case 3:
        cJSON_AddStringToObject(root, "working_mode", "Battery Mode");
        break;
    case 5:
        cJSON_AddStringToObject(root, "working_mode", "Hybrid Mode");
        break;
    default:
        cJSON_AddStringToObject(root, "working_mode", "Unknown/Fault");
        break;
    }
}

static void parse_fault_warnings(cJSON *root, char *data_start)
{
    char *running_str = data_start;
    char *token = NULL;
    int idx = 0;

    while ((token = strsep(&running_str, ",")) != NULL)
    {
        /* Strip 2-byte CRC + '\r' on the final token (Index 16) */
        if (idx == 16)
        {
            size_t tlen = strlen(token);
            if (tlen > 3)
            {
                token[tlen - 3] = '\0';
            }
        }

        if (strlen(token) == 0)
        {
            idx++;
            continue;
        }

        int val = atoi(token);

        switch (idx)
        {
        case 0:
            cJSON_AddNumberToObject(root, "fault_code", val);
            break;
        case 1:
            cJSON_AddBoolToObject(root, "line_fail", val);
            break;
        case 2:
            cJSON_AddBoolToObject(root, "output_circuit_short", val);
            break;
        case 3:
            cJSON_AddBoolToObject(root, "inverter_over_temp", val);
            break;
        case 4:
            cJSON_AddBoolToObject(root, "fan_lock", val);
            break;
        case 5:
            cJSON_AddBoolToObject(root, "battery_voltage_high", val);
            break;
        case 6:
            cJSON_AddBoolToObject(root, "battery_low", val);
            break;
        case 7:
            cJSON_AddBoolToObject(root, "battery_under", val);
            break;
        case 8:
            cJSON_AddBoolToObject(root, "over_load", val);
            break;
        case 9:
            cJSON_AddBoolToObject(root, "eeprom_fail", val);
            break;
        case 10:
            cJSON_AddBoolToObject(root, "power_limit", val);
            break;
        case 11:
            cJSON_AddBoolToObject(root, "pv1_voltage_high", val);
            break;
        case 12:
            cJSON_AddBoolToObject(root, "pv2_voltage_high", val);
            break;
        case 13:
            cJSON_AddBoolToObject(root, "mppt1_overload_warning", val);
            break;
        case 14:
            cJSON_AddBoolToObject(root, "mppt2_overload_warning", val);
            break;
        case 15:
            cJSON_AddBoolToObject(root, "scc1_battery_too_low", val);
            break;
        case 16:
            cJSON_AddBoolToObject(root, "scc2_battery_too_low", val);
            break;

        default:
            break;
        }

        idx++;
    }
}

static void parse_flag_status(cJSON *root, char *data_start)
{
    char *running_str = data_start;
    char *token = NULL;
    int idx = 0;

    while ((token = strsep(&running_str, ",")) != NULL)
    {
        if (idx == 8)
        {
            size_t tlen = strlen(token);
            if (tlen > 3)
            {
                token[tlen - 3] = '\0';
            }
        }

        if (strlen(token) == 0)
        {
            idx++;
            continue;
        }

        int val = atoi(token);

        switch (idx)
        {
        case 0:
            cJSON_AddBoolToObject(root, "buzzer_enabled", val);
            break;
        case 1:
            cJSON_AddBoolToObject(root, "overload_bypass_enabled", val);
            break;
        case 2:
            cJSON_AddBoolToObject(root, "lcd_escape_to_default_enabled", val);
            break;
        case 3:
            cJSON_AddBoolToObject(root, "overload_restart_enabled", val);
            break;
        case 4:
            cJSON_AddBoolToObject(root, "over_temp_restart_enabled", val);
            break;
        case 5:
            cJSON_AddBoolToObject(root, "backlight_enabled", val);
            break;
        case 6:
            cJSON_AddBoolToObject(root, "alarm_on_primary_source_interrupt", val);
            break;
        case 7:
            cJSON_AddBoolToObject(root, "fault_code_record_enabled", val);
            break;
        case 8:
            cJSON_AddNumberToObject(root, "reserved_flag", val);
            break;

        default:
            break;
        }

        idx++;
    }
}

static void parse_rated_information(cJSON *root, char *data_start)
{
    char *running_str = data_start;
    char *token = NULL;
    int idx = 0;

    while ((token = strsep(&running_str, ",")) != NULL)
    {
        if (idx == 24)
        {
            size_t tlen = strlen(token);
            if (tlen > 3)
            {
                token[tlen - 3] = '\0';
            }
        }

        if (strlen(token) == 0)
        {
            idx++;
            continue;
        }

        int val = atoi(token);

        switch (idx)
        {
        case 0:
            cJSON_AddNumberToObject(root, "ac_input_rating_voltage", atof(token) * 0.1);
            break;
        case 1:
            cJSON_AddNumberToObject(root, "ac_input_rating_current", atof(token) * 0.1);
            break;
        case 2:
            cJSON_AddNumberToObject(root, "ac_output_rating_voltage", atof(token) * 0.1);
            break;
        case 3:
            cJSON_AddNumberToObject(root, "ac_output_rating_frequency", atof(token) * 0.1);
            break;
        case 4:
            cJSON_AddNumberToObject(root, "ac_output_rating_current", atof(token) * 0.1);
            break;
        case 5:
            cJSON_AddNumberToObject(root, "ac_output_rating_apparent_power", val);
            break;
        case 6:
            cJSON_AddNumberToObject(root, "ac_output_rating_active_power", val);
            break;
        case 7:
            cJSON_AddNumberToObject(root, "battery_rating_voltage", atof(token) * 0.1);
            break;
        case 8:
            cJSON_AddNumberToObject(root, "battery_recharge_voltage", atof(token) * 0.1);
            break;
        case 9:
            cJSON_AddNumberToObject(root, "battery_redischarge_voltage", atof(token) * 0.1);
            break;
        case 10:
            cJSON_AddNumberToObject(root, "battery_under_voltage", atof(token) * 0.1);
            break;
        case 11:
            cJSON_AddNumberToObject(root, "battery_bulk_voltage", atof(token) * 0.1);
            break;
        case 12:
            cJSON_AddNumberToObject(root, "battery_float_voltage", atof(token) * 0.1);
            break;
        case 13:
            cJSON_AddNumberToObject(root, "battery_type", val);
            break;
        case 14:
            cJSON_AddNumberToObject(root, "max_ac_charging_current", val);
            break;
        case 15:
            cJSON_AddNumberToObject(root, "max_charging_current", val);
            break;
        case 16:
            cJSON_AddNumberToObject(root, "input_voltage_range", val);
            break;
        case 17:
            cJSON_AddNumberToObject(root, "output_source_priority", val);
            break;
        case 18:
            cJSON_AddNumberToObject(root, "charger_source_priority", val);
            break;
        case 19:
            cJSON_AddNumberToObject(root, "parallel_max_num", val);
            break;
        case 20:
            cJSON_AddNumberToObject(root, "machine_type", val);
            break;
        case 21:
            cJSON_AddNumberToObject(root, "topology", val);
            break;
        case 22:
            cJSON_AddNumberToObject(root, "output_model_setting", val);
            break;
        case 23:
            cJSON_AddNumberToObject(root, "solar_power_priority", val);
            break;
        case 24:
            cJSON_AddNumberToObject(root, "mppt_string_count", val);
            break;

        default:
            break;
        }

        idx++;
    }
}

static void parse_total_energy(cJSON *root, char *data_start)
{
    size_t len = strlen(data_start);
    if (len > 3)
    {
        data_start[len - 3] = '\0';
    }

    if (strlen(data_start) > 0)
    {
        cJSON_AddNumberToObject(root, "total_generated_energy_kwh", atol(data_start));
    }
}

static void parse_parallel_rated_info(cJSON *root, char *data_start)
{
    char *running_str = data_start;
    char *token = NULL;
    int idx = 0;

    while ((token = strsep(&running_str, ",")) != NULL)
    {
        if (idx == 6)
        {
            size_t tlen = strlen(token);
            if (tlen > 3)
            {
                token[tlen - 3] = '\0';
            }
        }

        if (strlen(token) == 0)
        {
            idx++;
            continue;
        }

        int val = atoi(token);

        switch (idx)
        {
        case 0:
            cJSON_AddBoolToObject(root, "connection_status", val);
            break;
        case 1:
            cJSON_AddNumberToObject(root, "sn_valid_length", val);
            break;
        case 2:
            cJSON_AddStringToObject(root, "serial_number", token);
            break;
        case 3:
            cJSON_AddNumberToObject(root, "charger_source_priority", val);
            break;
        case 4:
            cJSON_AddNumberToObject(root, "max_charging_current", val);
            break;
        case 5:
            cJSON_AddNumberToObject(root, "max_ac_charging_current", val);
            break;
        case 6:
            cJSON_AddNumberToObject(root, "output_model_setting", val);
            break;

        default:
            break;
        }

        idx++;
    }
}

static void parse_parallel_general_status(cJSON *root, char *data_start)
{
    char *running_str = data_start;
    char *token = NULL;
    int idx = 0;

    while ((token = strsep(&running_str, ",")) != NULL)
    {
        if (idx == 28)
        {
            size_t tlen = strlen(token);
            if (tlen > 3)
            {
                token[tlen - 3] = '\0';
            }
        }

        if (strlen(token) == 0)
        {
            idx++;
            continue;
        }

        int val = atoi(token);

        switch (idx)
        {
        case 0:
            cJSON_AddBoolToObject(root, "parallel_id_connected", val);
            break;
        case 1:
            cJSON_AddNumberToObject(root, "work_mode", val);
            break;
        case 2:
            cJSON_AddNumberToObject(root, "fault_code", val);
            break;
        case 3:
            cJSON_AddNumberToObject(root, "grid_voltage", atof(token) * 0.1);
            break;
        case 4:
            cJSON_AddNumberToObject(root, "grid_frequency", atof(token) * 0.1);
            break;
        case 5:
            cJSON_AddNumberToObject(root, "ac_output_voltage", atof(token) * 0.1);
            break;
        case 6:
            cJSON_AddNumberToObject(root, "ac_output_frequency", atof(token) * 0.1);
            break;
        case 7:
            cJSON_AddNumberToObject(root, "ac_output_apparent_power", val);
            break;
        case 8:
            cJSON_AddNumberToObject(root, "ac_output_active_power", val);
            break;
        case 9:
            cJSON_AddNumberToObject(root, "total_ac_output_apparent_power", val);
            break;
        case 10:
            cJSON_AddNumberToObject(root, "total_ac_output_active_power", val);
            break;
        case 11:
            cJSON_AddNumberToObject(root, "output_load_percent", val);
            break;
        case 12:
            cJSON_AddNumberToObject(root, "total_output_load_percent", val);
            break;
        case 13:
            cJSON_AddNumberToObject(root, "battery_voltage", atof(token) * 0.1);
            break;
        case 14:
            cJSON_AddNumberToObject(root, "battery_discharge_current", val);
            break;
        case 15:
            cJSON_AddNumberToObject(root, "battery_charging_current", val);
            break;
        case 16:
            cJSON_AddNumberToObject(root, "total_battery_charging_current", val);
            break;
        case 17:
            cJSON_AddNumberToObject(root, "battery_capacity", val);
            break;
        case 18:
            cJSON_AddNumberToObject(root, "pv1_input_power", val);
            break;
        case 19:
            cJSON_AddNumberToObject(root, "pv2_input_power", val);
            break;
        case 20:
            cJSON_AddNumberToObject(root, "pv1_input_voltage", atof(token) * 0.1);
            break;
        case 21:
            cJSON_AddNumberToObject(root, "pv2_input_voltage", atof(token) * 0.1);
            break;
        case 22:
            cJSON_AddNumberToObject(root, "mppt1_charger_status", val);
            break;
        case 23:
            cJSON_AddNumberToObject(root, "mppt2_charger_status", val);
            break;
        case 24:
            cJSON_AddBoolToObject(root, "load_connection", val);
            break;
        case 25:
            cJSON_AddNumberToObject(root, "battery_power_direction", val);
            break;
        case 26:
            cJSON_AddNumberToObject(root, "dc_ac_power_direction", val);
            break;
        case 27:
            cJSON_AddNumberToObject(root, "line_power_direction", val);
            break;
        case 28:
            cJSON_AddNumberToObject(root, "max_temperature", val);
            break;

        default:
            break;
        }

        idx++;
    }
}

char *pi18_parse_to_json(const inverter_payload_t *payload)
{
    /* Quick architectural boundaries check - accept Data (^D), ACK (^1), or NACK (^0) */
    if (!payload || payload->raw_data[0] != '^' || 
       (payload->raw_data[1] != 'D' && payload->raw_data[1] != '1' && payload->raw_data[1] != '0'))
    {
        return NULL;
    }

    cJSON *root = cJSON_CreateObject();
    if (!root)
        return NULL;

    /* Add InfluxDB Epoch Timestamp in milliseconds */
    struct timeval tv;
    gettimeofday(&tv, NULL);
    int64_t timestamp_ms = ((int64_t)tv.tv_sec * 1000LL) + (tv.tv_usec / 1000);
    cJSON_AddNumberToObject(root, "time", (double)timestamp_ms);

    /* Create sub-object to hold all telemetry/setting values */
    cJSON *fields = cJSON_CreateObject();
    if (!fields) {
        cJSON_Delete(root);
        return NULL;
    }

    /* Create a local thread-safe working string copy */
    char buf[256];
    size_t copy_len = (payload->length < sizeof(buf) - 1) ? payload->length : sizeof(buf) - 1;
    memcpy(buf, payload->raw_data, copy_len);
    buf[copy_len] = '\0';

    /* Handle Setting Commands / Acknowledgments (^1 = ACK, ^0 = NACK) */
    if (buf[1] == '1' || buf[1] == '0')
    {
        cJSON_AddStringToObject(root, "measurement", "SETTING_RESPONSE");
        cJSON_AddBoolToObject(fields, "success", (buf[1] == '1'));
        cJSON_AddStringToObject(fields, "raw", buf);

        cJSON_AddItemToObject(root, "fields", fields);

        char *json_out = cJSON_PrintUnformatted(root);
        cJSON_Delete(root);
        return json_out;
    }

    /* Assign InfluxDB Measurement Name for Data Queries */
    const char *meas_name = get_pi18_cmd_string(payload->cmd_id);
    cJSON_AddStringToObject(root, "measurement", (meas_name != NULL) ? meas_name : "CUSTOM_COMMAND");

    char *data_start = buf + 5; /* Move pointer past header */

    switch (payload->cmd_id)
    {
    case CMD_PI18_ID:
        parse_serial_number(fields, data_start);
        break;
    case CMD_PI18_GS:
        parse_general_status(fields, data_start);
        break;
    case CMD_PI18_MOD:
        parse_working_mode(fields, data_start);
        break;
    case CMD_PI18_FWS:
        parse_fault_warnings(fields, data_start);
        break;
    case CMD_PI18_FLAG:
        parse_flag_status(fields, data_start);
        break;
    case CMD_PI18_PIRI:
        parse_rated_information(fields, data_start);
        break;
    case CMD_PI18_ET:
        parse_total_energy(fields, data_start);
        break;
    case CMD_PI18_PRI0:
        parse_parallel_rated_info(fields, data_start);
        break;
    case CMD_PI18_PGS0:
        parse_parallel_general_status(fields, data_start);
        break;

    default:
        cJSON_AddStringToObject(fields, "raw_data", buf);
        break;
    }

    cJSON_AddItemToObject(root, "fields", fields);

    char *json_out = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);

    return json_out;
}

void inverter_parser_task(void *pvParameters)
{
    parser_task_args_t *args = (parser_task_args_t *)pvParameters;
    QueueHandle_t raw_queue  = args->raw_uart_queue;
    QueueHandle_t mqtt_q     = args->mqtt_queue;

    inverter_payload_t payload;

    ESP_LOGI(TAG, "PI18 Parser Task running...");

    while (1)
    {
        /* Receive raw UART payload from Polling Task */
        if (xQueueReceive(raw_queue, &payload, portMAX_DELAY) == pdTRUE)
        {
            /* Parse payload into heap-allocated JSON string */
            char *json_str = pi18_parse_to_json(&payload);

            if (json_str != NULL)
            {
                ESP_LOGI("PARSER", "[CMD 0x%02X] %s", payload.cmd_id, json_str);

                /* Forward heap string pointer to MQTT Queue */
                if (xQueueSend(mqtt_q, &json_str, pdMS_TO_TICKS(100)) != pdTRUE)
                {
                    ESP_LOGE("PARSER", "MQTT queue full! Freeing memory to prevent leak.");
                    free(json_str);
                }
            }
            else
            {
                ESP_LOGE("PARSER", "Failed to parse frame for CMD 0x%02X", payload.cmd_id);
            }
        }
    }
}