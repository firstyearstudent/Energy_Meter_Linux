# Đăng ký component với ESP-IDF
idf_component_register(
    SRCS "ina226.c"
    INCLUDE_DIRS "include"
    # REQUIRES: Các thư viện hệ thống mà INA226 cần dùng
    # driver: Dùng cho I2C (bao gồm cả i2c_master mới trong IDF 5.x)
    # esp_timer: Dùng cho các hàm delay/timeout
    # log: Dùng cho ESP_LOGI, ESP_LOGE
    REQUIRES driver esp_timer log
)
