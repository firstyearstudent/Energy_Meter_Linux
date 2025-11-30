#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/i2c_master.h"
#include "driver/spi_master.h"

// Header custom của dự án
#include "oled_init.h" 
#include "ina226.h"

static const char *TAG = "EnergyMeter";

// --- CẤU HÌNH CHÂN (PINOUT) ---
// I2C cho INA226
#define I2C_MASTER_SCL_IO           22
#define I2C_MASTER_SDA_IO           21
#define I2C_PORT_NUM                I2C_NUM_0

// SPI cho OLED (SSD1306)
#define PIN_NUM_MISO                -1  
#define PIN_NUM_MOSI                23  
#define PIN_NUM_CLK                 18  
#define PIN_NUM_CS                  5   
#define PIN_NUM_DC                  17  
#define PIN_NUM_RST                 16  

#define SAMPLE_PERIOD_MS            1000

void app_main(void)
{
    esp_err_t err;

    // ---------------------------------------------------------
    // 1. Khởi tạo SPI Bus
    // ---------------------------------------------------------
    ESP_LOGI(TAG, "Initializing SPI Bus...");
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096, 
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO));

    // ---------------------------------------------------------
    // 2. Khởi tạo OLED
    // ---------------------------------------------------------
    oled_t oled;
    ESP_ERROR_CHECK(oled_init(&oled, SPI2_HOST, PIN_NUM_CS, PIN_NUM_DC, PIN_NUM_RST, 10 * 1000 * 1000));

    // ---------------------------------------------------------
    // 3. Khởi tạo I2C Bus
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
    ina_cfg.i2c_address = 0x40; 
    ina_cfg.i2c_clock_speed = 100000; 
    
    // --- QUAN TRỌNG: KIỂM TRA LẠI MODULE ---
    // Nếu trên lưng trở ghi "R100" -> Điền 0.1
    ina_cfg.shunt_resistance = 0.1; 
    ina_cfg.max_current = 10.0;

    err = ina226_init(bus_handle, &ina_cfg, &ina_dev);
    if (err != ESP_OK) {
       // Nếu init thất bại, nghĩa là I2C lỗi hoặc sai địa chỉ
       ESP_LOGE(TAG, "INA226 init failed: %s (Kiem tra day SDA/SCL hoac dia chi)", esp_err_to_name(err));
    } else {
       // Nếu init thành công, nghĩa là chip ĐÃ TRẢ LỜI (ACK)
       ESP_LOGI(TAG, "=> CHIP INA226 KET NOI TOT!"); 
       ESP_ERROR_CHECK(ina226_calibrate(ina_dev, ina_cfg.max_current, ina_cfg.shunt_resistance));
    }

    ESP_LOGI(TAG, "System Initialized. Starting Loop...");

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
            // 1. Hiển thị OLED 
            oled_display_measure(&oled, bus_v, current, power);

            // 2. Gửi Log qua UART cho máy tính
            int64_t ts = esp_timer_get_time() / 1000;
            // Format chuẩn CSV: timestamp, V, I, P
            printf("%lld,%.3f,%.6f,%.3f\n", ts, bus_v, current, power);
        } else {
            // Chỉ báo lỗi nếu trước đó Init thành công mà giờ đọc lỗi
            // Giúp tránh spam log nếu phần cứng hỏng từ đầu
            ESP_LOGE(TAG, "Sensor Read Error: %s", esp_err_to_name(err));
        }

        vTaskDelay(pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
    }
}