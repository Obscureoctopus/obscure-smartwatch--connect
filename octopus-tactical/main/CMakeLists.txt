idf_component_register(SRCS "main.c" "sunmoon.c"
                       INCLUDE_DIRS "."
                       REQUIRES esp_driver_gpio esp_driver_i2c nvs_flash)
target_link_libraries(${COMPONENT_LIB} PRIVATE m)
