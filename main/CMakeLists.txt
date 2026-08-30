idf_component_register(SRCS "main.c" "wifi_sta.c" "pi18_parser.c" "inverter_uart.c" "mqtt_handler.c"
                       PRIV_REQUIRES spi_flash esp_wifi nvs_flash esp_driver_uart mqtt
                       INCLUDE_DIRS "include")
