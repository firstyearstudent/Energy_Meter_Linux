#include "oled_init.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"

// Include driver SSD1306 từ component 'ssd1306'
#include "esp_lcd_panel_ssd1306.h"

static const char *TAG = "oled_init";

esp_err_t oled_init(oled_t *oled, spi_host_device_t host, int cs_io, int dc_io, int reset_io, int clock_speed_hz) {
    if (!oled) return ESP_ERR_INVALID_ARG;

    ESP_LOGI(TAG, "Khoi tao OLED (esp_lcd + ssd1306)...");

    // 1. Cấu hình GPIO
    // Mặc dù driver có thể tự config, ta config trước để đảm bảo mức logic an toàn
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << dc_io) | (1ULL << reset_io) | (1ULL << cs_io),
        .pull_down_en = 0,
        .pull_up_en = 1,
    };
    gpio_config(&io_conf);

    // 2. Cấu hình Panel IO (Giao tiếp SPI dành cho LCD)
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = dc_io,
        .cs_gpio_num = cs_io,
        .pclk_hz = clock_speed_hz,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .spi_mode = 0,
        .trans_queue_depth = 10,
    };

    // Gắn Panel IO vào Bus SPI (đã init ở main)
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)host, &io_config, &oled->io_handle));

    // 3. Cấu hình Driver SSD1306
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = reset_io,
        .bits_per_pixel = 1,
        .flags.reset_active_high = 0, // Reset kích hoạt ở mức thấp (Low)
    };

    // Tạo đối tượng Panel
    // Hàm esp_lcd_new_panel_ssd1306 nằm trong component 'ssd1306' bạn vừa tạo
    ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1306(oled->io_handle, &panel_config, &oled->panel_handle));

    // 4. Khởi động quy trình hiển thị
    ESP_ERROR_CHECK(esp_lcd_panel_reset(oled->panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(oled->panel_handle));

    // Lật màn hình (nếu bị ngược)
    // ESP_ERROR_CHECK(esp_lcd_panel_mirror(oled->panel_handle, true, true));

    // Bật màn hình
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(oled->panel_handle, true));

    ESP_LOGI(TAG, "OLED init done!");
    return ESP_OK;
}

esp_err_t oled_display_measure(oled_t *oled, float bus_voltage, float current, float power) {
    if (!oled || !oled->panel_handle) return ESP_FAIL;

    // LƯU Ý: Driver esp_lcd chuẩn chỉ hỗ trợ đẩy điểm ảnh (bitmap).
    // Nó KHÔNG có sẵn hàm vẽ chữ (Draw String).
    // Bạn cần dùng thư viện LVGL hoặc tự viết hàm vẽ font bitmap để hiển thị số.

    // Ví dụ xóa màn hình (gửi toàn màu đen):
    // uint8_t *black_screen = calloc(128 * 64 / 8, 1);
    // esp_lcd_panel_draw_bitmap(oled->panel_handle, 0, 0, 128, 64, black_screen);
    // free(black_screen);

    return ESP_OK;
}
