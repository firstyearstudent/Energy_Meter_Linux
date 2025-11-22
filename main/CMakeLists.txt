idf_component_register(SRCS "Energy_Meter.c"
                       INCLUDE_DIRS "."
                       REQUIRES driver esp_ina226 display ssd1306)
