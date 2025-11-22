idf_component_register(SRCS "oled_init.c"
                       INCLUDE_DIRS "."
                       # REQUIRES:
                       # driver: Cho GPIO, SPI
                       # esp_lcd: Thư viện hiển thị lõi
                       # ssd1306: Component driver bạn vừa tạo ở bước trước
                       REQUIRES driver esp_lcd ssd1306)
