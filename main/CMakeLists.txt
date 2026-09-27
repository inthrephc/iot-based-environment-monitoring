idf_component_register(
    SRCS "main.c" "dht22.c" "ssd1306.c"
    INCLUDE_DIRS "."
    REQUIRES freertos esp_timer esp_event esp_netif esp_wifi nvs_flash esp_http_server esp_adc esp_driver_gpio esp_driver_i2c esp_driver_ledc
)
