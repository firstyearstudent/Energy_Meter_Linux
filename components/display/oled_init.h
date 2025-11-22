#ifndef OLED_INIT_H
#define OLED_INIT_H

#include "esp_err.h"
#include "driver/spi_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"

// Struct quản lý màn hình OLED
typedef struct {
    // Handle giao tiếp IO (SPI)
    esp_lcd_panel_io_handle_t io_handle;
    // Handle điều khiển màn hình (Gửi lệnh Init, Reset, Draw...)
    esp_lcd_panel_handle_t panel_handle;
} oled_t;

/**
 * @brief Khởi tạo màn hình OLED SSD1306 qua giao tiếp SPI
 * * @param oled Con trỏ tới struct oled_t để lưu các handle
 * @param host SPI Host ID (ví dụ SPI2_HOST)
 * @param cs_io GPIO chân CS
 * @param dc_io GPIO chân DC (Data/Command)
 * @param reset_io GPIO chân RST (Reset)
 * @param clock_speed_hz Tốc độ SPI (thường là 4MHz - 10MHz)
 * @return esp_err_t ESP_OK nếu thành công
 */
esp_err_t oled_init(oled_t *oled, spi_host_device_t host, int cs_io, int dc_io, int reset_io, int clock_speed_hz);

/**
 * @brief Hiển thị thông số đo (Placeholder)
 * Lưu ý: Cần tích hợp thêm thư viện Font hoặc LVGL để vẽ chữ
 */
esp_err_t oled_display_measure(oled_t *oled, float bus_voltage, float current, float power);

#endif // OLED_INIT_H
