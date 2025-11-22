#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/i2c_master.h"
#include "driver/spi_master.h" // Dùng trực tiếp driver SPI chuẩn của ESP-IDF

// Header custom của dự án
#include "oled_init.h" // Header mới đã sửa cho esp_lcd
#include "ina226.h"

static const char *TAG = "EnergyMeter";

// --- CẤU HÌNH CHÂN (PINOUT) ---
// I2C cho INA226
#define I2C_MASTER_SCL_IO           22
#define I2C_MASTER_SDA_IO           21
#define I2C_PORT_NUM                I2C_NUM_0

// SPI cho OLED (SSD1306)
#define PIN_NUM_MISO                -1  // OLED không cần MISO
#define PIN_NUM_MOSI                23  // D1
#define PIN_NUM_CLK                 18  // D0
#define PIN_NUM_CS                  5   // Chip Select
#define PIN_NUM_DC                  17  // Data/Command
#define PIN_NUM_RST                 16  // Reset

#define SAMPLE_PERIOD_MS            1000

void app_main(void)
{
    esp_err_t err;

    // ---------------------------------------------------------
    // 1. Khởi tạo SPI Bus (Trực tiếp, không qua spi_init wrapper cũ)
    // ---------------------------------------------------------
    ESP_LOGI(TAG, "Initializing SPI Bus...");
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096, // Kích thước buffer đủ cho màn hình
    };
    // Sử dụng SPI2_HOST
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO));

    // ---------------------------------------------------------
    // 2. Khởi tạo OLED (Sử dụng driver esp_lcd mới)
    // ---------------------------------------------------------
    oled_t oled;
    // Hàm này (trong oled_init.c mới) sẽ tạo Panel IO và gắn vào Bus SPI2
    ESP_ERROR_CHECK(oled_init(&oled, SPI2_HOST, PIN_NUM_CS, PIN_NUM_DC, PIN_NUM_RST, 10 * 1000 * 1000));

    // ---------------------------------------------------------
    // 3. Khởi tạo I2C Master Bus (Cho INA226)
    // ---------------------------------------------------------
    ESP_LOGI(TAG, "Initializing I2C Bus...");
    i2c_master_bus_config_t i2c_bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_PORT_NUM,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, &bus_handle));

    // ---------------------------------------------------------
    // 4. Khởi tạo INA226
    // ---------------------------------------------------------
    ESP_LOGI(TAG, "Initializing INA226...");
    ina226_handle_t ina_dev;
    ina226_config_t ina_cfg = INA226_CONFIG_DEFAULT;

    // QUAN TRỌNG: Kiểm tra lại module thực tế của bạn
    // Nếu module ghi R002 -> 0.002 Ohm (Phổ biến)
    // Nếu module ghi R100 -> 0.1 Ohm
    ina_cfg.shunt_resistance = 0.002;
    ina_cfg.max_current = 10.0;

    err = ina226_init(bus_handle, &ina_cfg, &ina_dev);
    if (err != ESP_OK) {
       ESP_LOGE(TAG, "INA226 init failed: %s", esp_err_to_name(err));
       // Có thể thêm logic retry hoặc return tại đây
    } else {
       ESP_ERROR_CHECK(ina226_calibrate(ina_dev, ina_cfg.max_current, ina_cfg.shunt_resistance));
    }

    ESP_LOGI(TAG, "System Initialized. Starting Loop...");

    // In Header cho Logger Python (nếu cần)
    // printf("TIMESTAMP,VOLTAGE,CURRENT,POWER\n");

    // ---------------------------------------------------------
    // 5. Vòng lặp chính
    // ---------------------------------------------------------
    while (1) {
        float bus_v = 0.0f;
        float current = 0.0f;
        float power = 0.0f;

        // Đọc cảm biến
        err = ina226_get_bus_voltage(ina_dev, &bus_v);
        if (err == ESP_OK) ina226_get_current(ina_dev, &current);
        if (err == ESP_OK) ina226_get_power(ina_dev, &power);

        if (err == ESP_OK) {
            // 1. Hiển thị OLED (Hiện tại chỉ là placeholder giữ chỗ)
            oled_display_measure(&oled, bus_v, current, power);

            // 2. Gửi Log qua UART cho máy tính
            int64_t ts = esp_timer_get_time() / 1000;
            // Format chuẩn CSV: timestamp, V, I, P
            printf("%lld,%.3f,%.6f,%.3f\n", ts, bus_v, current, power);
        } else {
            ESP_LOGE(TAG, "Sensor Read Error: %s", esp_err_to_name(err));
        }

        vTaskDelay(pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
    }
}
