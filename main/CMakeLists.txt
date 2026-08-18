idf_component_register(SRCS "freddie2_main.c"
                       PRIV_REQUIRES spi_flash esp_driver_gpio esp_driver_i2c esp_driver_ledc esp_driver_uart esp_timer esp_wifi esp_netif esp_event nvs_flash)
