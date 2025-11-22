#ifndef OLED_INIT_H
#define OLED_INIT_H

#include "esp_err.h"
#include "driver/spi_master.h"
#include "ssd1306.h"  // giả sử bạn dùng driver ssd1306 của component

typedef struct {
    spi_device_handle_t spi_dev;    // handle SPI
    ssd1306_handle_t oled_handle;   // handle của driver SSD1306
} oled_t;

/**
 * @brief Khởi tạo OLED SPI
 *
 * @param oled Con trỏ tới struct oled_t để lưu handle SPI + OLED
 * @param host SPI host (ví dụ SPI2_HOST)
 * @param cs_io chân CS
 * @param dc_io chân DC (data/command)
 * @param reset_io chân reset
 * @param clock_speed_hz tốc độ SPI
 * @return esp_err_t
 */
esp_err_t oled_init(oled_t *oled, spi_host_device_t host, int cs_io, int dc_io, int reset_io, int clock_speed_hz);

/**
 * @brief Hiển thị các thông số đo lên OLED
 *
 * @param oled Con trỏ tới struct oled_t
 * @param bus_voltage Volts
 * @param current Amps
 * @param power Watts
 * @return esp_err_t
 */
esp_err_t oled_display_measure(oled_t *oled, float bus_voltage, float current, float power);

#endif // OLED_INIT_H
