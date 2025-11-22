#include "ina226.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <esp_log.h>
#include <esp_check.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Định nghĩa các thanh ghi
#define INA226_REG_CONFIG               (0x00)
#define INA226_REG_SHUNT_V              (0x01)
#define INA226_REG_BUS_V                (0x02)
#define INA226_REG_POWER                (0x03)
#define INA226_REG_CURRENT              (0x04)
#define INA226_REG_CALIBRATION          (0x05)
#define INA226_REG_MSK_ENA              (0x06)
#define INA226_REG_ALERT_LIMIT          (0x07)
#define INA226_REG_MANU_ID              (0xfe)
#define INA226_REG_DIE_ID               (0xff)

#define INA226_MIN_SHUNT_RESISTANCE     (0.001)
#define INA226_MAX_SHUNT_VOLTAGE        (0.08192)

#define I2C_XFR_TIMEOUT_MS      (500)
#define INA226_CMD_DELAY_MS     (10)

static const char *TAG = "ina226";

// Macro kiểm tra timeout
#define ESP_TIMEOUT_CHECK(start, len) ((uint64_t)(esp_timer_get_time() - (start)) >= (len))
#define ESP_ARG_CHECK(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

/**
 * @brief Cấu trúc thiết bị
 */
typedef struct ina226_device_s {
    ina226_config_t                 config;
    i2c_master_dev_handle_t         i2c_handle;
    float                           current_lsb;
} ina226_device_t;

/**
 * @brief Hàm đọc 16-bit từ thanh ghi (Sửa lại dùng uint8_t thuần túy)
 */
static inline esp_err_t ina226_i2c_read_word_from(ina226_device_t *const device, const uint8_t reg_addr, uint16_t *const word) {
    ESP_ARG_CHECK(device);

    // Buffer truyền địa chỉ thanh ghi (1 byte)
    uint8_t tx_buf[1] = { reg_addr };
    // Buffer nhận dữ liệu (2 byte: MSB, LSB)
    uint8_t rx_buf[2] = { 0 };

    // Gọi hàm truyền nhận của driver I2C Master mới
    ESP_RETURN_ON_ERROR(i2c_master_transmit_receive(device->i2c_handle, tx_buf, sizeof(tx_buf), rx_buf, sizeof(rx_buf), I2C_XFR_TIMEOUT_MS), TAG, "i2c read failed");

    // Ghép 2 byte thành 1 word 16-bit (Big Endian: MSB trước)
    *word = ((uint16_t)rx_buf[0] << 8) | (uint16_t)rx_buf[1];

    return ESP_OK;
}

/**
 * @brief Hàm ghi 16-bit vào thanh ghi (Sửa lại dùng uint8_t thuần túy)
 */
static inline esp_err_t ina226_i2c_write_word_to(ina226_device_t *const device, const uint8_t reg_addr, const uint16_t word) {
    ESP_ARG_CHECK(device);

    // Buffer chứa: [Địa chỉ Register] + [MSB] + [LSB]
    uint8_t tx_buf[3];
    tx_buf[0] = reg_addr;
    tx_buf[1] = (uint8_t)((word >> 8) & 0xFF); // MSB
    tx_buf[2] = (uint8_t)(word & 0xFF);        // LSB

    // Gọi hàm truyền
    ESP_RETURN_ON_ERROR(i2c_master_transmit(device->i2c_handle, tx_buf, sizeof(tx_buf), I2C_XFR_TIMEOUT_MS), TAG, "i2c write failed");

    return ESP_OK;
}

// --- Các hàm API giữ nguyên logic nhưng gọi hàm đọc/ghi đã sửa ở trên ---

esp_err_t ina226_init(i2c_master_bus_handle_t master_handle, const ina226_config_t *ina226_config, ina226_handle_t *ina226_handle) {
    ESP_ARG_CHECK(master_handle && ina226_config);

    // Cấp phát bộ nhớ cho device struct
    ina226_device_t* dev = (ina226_device_t*)calloc(1, sizeof(ina226_device_t));
    if (!dev) return ESP_ERR_NO_MEM;

    dev->config = *ina226_config;

    // Cấu hình thiết bị I2C
    i2c_device_config_t i2c_dev_conf = {
        .dev_addr_length    = I2C_ADDR_BIT_LEN_7,
        .device_address     = dev->config.i2c_address,
        .scl_speed_hz       = dev->config.i2c_clock_speed,
    };

    // Thêm thiết bị vào bus
    if (i2c_master_bus_add_device(master_handle, &i2c_dev_conf, &dev->i2c_handle) != ESP_OK) {
        free(dev);
        return ESP_FAIL;
    }

    *ina226_handle = (ina226_handle_t)dev;

    // Reset thiết bị
    ina226_reset((ina226_handle_t)dev);

    return ESP_OK;
}

esp_err_t ina226_calibrate(ina226_handle_t handle, const float max_current, const float shunt_resistance) {
    ina226_device_t* dev = (ina226_device_t*)handle;
    ESP_ARG_CHECK(dev);

    if (max_current < 0.001 || shunt_resistance < INA226_MIN_SHUNT_RESISTANCE) {
        return ESP_ERR_INVALID_ARG;
    }

    // Tính toán Current_LSB
    float current_lsb = max_current / 32768.0f;
    // Làm tròn lên số đẹp gần nhất (ví dụ logic đơn giản hóa)
    // Hoặc giữ nguyên tính toán chính xác:
    dev->current_lsb = current_lsb;

    // Tính toán giá trị thanh ghi Calibration (CAL)
    // CAL = 0.00512 / (Current_LSB * R_shunt)
    uint16_t cal = (uint16_t)(0.00512f / (dev->current_lsb * shunt_resistance));

    return ina226_i2c_write_word_to(dev, INA226_REG_CALIBRATION, cal);
}

esp_err_t ina226_get_bus_voltage(ina226_handle_t handle, float *const voltage) {
    ina226_device_t* dev = (ina226_device_t*)handle;
    uint16_t raw_val;

    ESP_RETURN_ON_ERROR(ina226_i2c_read_word_from(dev, INA226_REG_BUS_V, &raw_val), TAG, "read bus voltage failed");

    // LSB = 1.25 mV
    *voltage = (float)raw_val * 0.00125f;
    return ESP_OK;
}

esp_err_t ina226_get_shunt_voltage(ina226_handle_t handle, float *const voltage) {
    ina226_device_t* dev = (ina226_device_t*)handle;
    uint16_t raw_val;

    ESP_RETURN_ON_ERROR(ina226_i2c_read_word_from(dev, INA226_REG_SHUNT_V, &raw_val), TAG, "read shunt voltage failed");

    // LSB = 2.5 uV. Dữ liệu là số có dấu (signed 16-bit)
    int16_t signed_val = (int16_t)raw_val;
    *voltage = (float)signed_val * 0.0000025f;
    return ESP_OK;
}

esp_err_t ina226_get_current(ina226_handle_t handle, float *const current) {
    ina226_device_t* dev = (ina226_device_t*)handle;
    uint16_t raw_val;

    ESP_RETURN_ON_ERROR(ina226_i2c_read_word_from(dev, INA226_REG_CURRENT, &raw_val), TAG, "read current failed");

    int16_t signed_val = (int16_t)raw_val;
    *current = (float)signed_val * dev->current_lsb;
    return ESP_OK;
}

esp_err_t ina226_get_power(ina226_handle_t handle, float *const power) {
    ina226_device_t* dev = (ina226_device_t*)handle;
    uint16_t raw_val;

    ESP_RETURN_ON_ERROR(ina226_i2c_read_word_from(dev, INA226_REG_POWER, &raw_val), TAG, "read power failed");

    // Power LSB = 25 * Current_LSB
    *power = (float)raw_val * (dev->current_lsb * 25.0f);
    return ESP_OK;
}

esp_err_t ina226_reset(ina226_handle_t handle) {
    ina226_device_t* dev = (ina226_device_t*)handle;
    // Bit 15 của thanh ghi Config là Reset
    ESP_RETURN_ON_ERROR(ina226_i2c_write_word_to(dev, INA226_REG_CONFIG, 0x8000), TAG, "reset failed");
    vTaskDelay(pdMS_TO_TICKS(10)); // Đợi chip reset

    // Cấu hình lại mặc định (Ví dụ: AVG=1, VBus Time=1.1ms, Shunt Time=1.1ms, Mode=Cont.Shunt+Bus)
    // Config = 0x4127 (Mặc định)
    return ina226_i2c_write_word_to(dev, INA226_REG_CONFIG, 0x4127);
}

esp_err_t ina226_delete(ina226_handle_t handle) {
    ina226_device_t* dev = (ina226_device_t*)handle;
    if (dev && dev->i2c_handle) {
        i2c_master_bus_rm_device(dev->i2c_handle);
    }
    free(dev);
    return ESP_OK;
}
