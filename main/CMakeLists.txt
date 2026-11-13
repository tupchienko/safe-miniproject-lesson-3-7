# main/CMakeLists.txt
# ESP-IDF v5.3+ and v6. For v5.0-5.2 replace REQUIRES with: REQUIRES driver
idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS "."
                       REQUIRES esp_driver_pcnt esp_driver_gpio esp_driver_ledc)
 