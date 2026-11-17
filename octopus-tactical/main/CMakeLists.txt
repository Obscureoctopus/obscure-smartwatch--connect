idf_component_register(SRCS "main.c" "sunmoon.c" "ble.c" "chat.c"
                       INCLUDE_DIRS "."
                       REQUIRES esp_driver_gpio esp_driver_i2c nvs_flash bt esp_wifi esp_event esp_netif json esp_websocket_client)
target_link_libraries(${COMPONENT_LIB} PRIVATE m)
