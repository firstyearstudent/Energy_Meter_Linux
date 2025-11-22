#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/i2c_master.h"

// Include các file header của bạn
#include "spi_init.h"
#include "oled_init.h"
#include "ina226.h"

static const char *TAG = "EnergyMeter";

#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_SDA_IO 21
#define I2C_PORT_NUM I2C_NUM_0
#define SAMPLE_PERIOD_MS 1000

void app_main(void)
{
    // 1. Khởi tạo SPI & OLED (Như cũ)
    spi_bus_init_config_t spi_bus_cfg = { .host = SPI2_HOST, .sclk_io = 18, .mosi_io = 23, .miso_io = -1, .max_transfer_sz = 0 };
    ESP_ERROR_CHECK(spi_bus_init(&spi_bus_cfg));
    oled_t oled;
    ESP_ERROR_CHECK(oled_init(&oled, SPI2_HOST, 5, 17, 16, 10*1000*1000));

    // 2. Khởi tạo I2C (Driver mới)
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

    // 3. Init INA226
    ina226_handle_t ina_dev;
    ina226_config_t ina_cfg = INA226_CONFIG_DEFAULT;
    ina_cfg.shunt_resistance = 0.1; // Chỉnh lại đúng R shunt của bạn
    ina_cfg.max_current = 10.0;
    ESP_ERROR_CHECK(ina226_init(bus_handle, &ina_cfg, &ina_dev));
    ESP_ERROR_CHECK(ina226_calibrate(ina_dev, ina_cfg.max_current, ina_cfg.shunt_resistance));

    ESP_LOGI(TAG, "System Started. Sending data to UART...");

    while (1) {
        float bus_v = 0, current = 0, power = 0;

        esp_err_t err = ina226_get_bus_voltage(ina_dev, &bus_v);
        if (err == ESP_OK) ina226_get_current(ina_dev, &current);
        if (err == ESP_OK) ina226_get_power(ina_dev, &power);

        if (err == ESP_OK) {
            // Hiển thị OLED
            oled_display_measure(&oled, bus_v, current, power);

            // Gửi dữ liệu về Laptop (Python sẽ bắt dòng này)
            // Định dạng: timestamp, voltage, current, power
            int64_t ts = esp_timer_get_time() / 1000;
            printf("%lld,%.3f,%.6f,%.3f\n", ts, bus_v, current, power);
        } else {
            // Gửi log lỗi (Python sẽ bỏ qua dòng này vì không có dấu phẩy đúng format)
            ESP_LOGE(TAG, "Read Error");
        }

        vTaskDelay(pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
    }
}
