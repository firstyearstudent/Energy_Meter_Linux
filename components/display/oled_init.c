#include "oled_init.h"
#include "esp_log.h"
#include "ssd1306.h"

static const char *TAG = "oled_init";

esp_err_t oled_init(oled_t *oled, spi_host_device_t host, int cs_io, int dc_io, int reset_io, int clock_speed_hz) {
    if (!oled) return ESP_ERR_INVALID_ARG;

    // Tạo cấu hình thiết bị SPI OLED
    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = clock_speed_hz,
        .mode = 0,  // SSD1306 thường mode 0
        .spics_io_num = cs_io,
        .queue_size = 1,
        .flags = SPI_DEVICE_HALFDUPLEX,
    };

    esp_err_t err = spi_bus_add_device(host, &dev_cfg, &oled->spi_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add SPI device for OLED: %s", esp_err_to_name(err));
        return err;
    }

    // Cấu hình driver SSD1306
    ssd1306_config_t cfg = {
        .iface = SSD1306_SPI,       // giao tiếp SPI
        .width = 128,
        .height = 64,
        .mirrored = false,
        .rotate = false,
        .bus = {
            .spi = {
                .host = host,
                .dc_io = dc_io,
                .reset_io = reset_io,
                .cs_io = cs_io,
                .clock_speed_hz = clock_speed_hz,
            }
        }
    };

    err = ssd1306_new(&cfg, &oled->oled_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ssd1306_new failed: %s", esp_err_to_name(err));
        return err;
    }

    // Xoá màn hình ban đầu
    ssd1306_clear(oled->oled_handle);
    ssd1306_display(oled->oled_handle);

    ESP_LOGI(TAG, "OLED initialized (SPI)");

    return ESP_OK;
}

esp_err_t oled_display_measure(oled_t *oled, float bus_voltage, float current, float power) {
    if (!oled) return ESP_ERR_INVALID_ARG;

    // Clear buffer
    ssd1306_clear(oled->oled_handle);

    // Vẽ text (bạn có thể tối ưu font)
    char buf[32];
    // Hiển thị điện áp
    snprintf(buf, sizeof(buf), "V: %.2f V", bus_voltage);
    ssd1306_draw_string(oled->oled_handle, 0, 0, buf, 12, 1);

    // Hiển thị dòng
    snprintf(buf, sizeof(buf), "I: %.3f A", current);
    ssd1306_draw_string(oled->oled_handle, 0, 16, buf, 12, 1);

    // Hiển thị công suất
    snprintf(buf, sizeof(buf), "P: %.3f W", power);
    ssd1306_draw_string(oled->oled_handle, 0, 32, buf, 12, 1);

    // Cập nhật lên màn hình
    ssd1306_display(oled->oled_handle);

    return ESP_OK;
}
