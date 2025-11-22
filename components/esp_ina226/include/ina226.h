#ifndef __INA226_H__
#define __INA226_H__

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>
#include <driver/i2c_master.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Các địa chỉ I2C mặc định của INA226
 * (Dựa trên cách nối chân A0, A1)
*/
#define I2C_INA226_ADDR_GND_GND         0x40 // A1=GND, A0=GND (Phổ biến nhất)
#define I2C_INA226_ADDR_GND_VS          0x41 // A1=GND, A0=VCC
#define I2C_INA226_ADDR_GND_SDA         0x42 // A1=GND, A0=SDA
#define I2C_INA226_ADDR_GND_SCL         0x43 // A1=GND, A0=SCL
// ... Các địa chỉ khác nếu cần thiết có thể xem datasheet

/*
 * Cấu hình mặc định
*/
#define INA226_CONFIG_DEFAULT {                 \
    .i2c_address        = I2C_INA226_ADDR_GND_GND, \
    .i2c_clock_speed    = 100000,               \
    .shunt_resistance   = 0.002,                \
    .max_current        = 10.0                  \
}

/**
 * @brief Cấu trúc cấu hình thiết bị
 */
typedef struct {
    uint16_t    i2c_address;        /*!< Địa chỉ I2C (0x40 - 0x4F) */
    uint32_t    i2c_clock_speed;    /*!< Tốc độ I2C (Hz) */
    float       shunt_resistance;   /*!< Giá trị điện trở Shunt (Ohm) */
    float       max_current;        /*!< Dòng điện tối đa dự kiến (Ampe) */
} ina226_config_t;

/**
 * @brief Handle quản lý thiết bị
 */
typedef void* ina226_handle_t;

/**
 * @brief Khởi tạo thiết bị INA226 và thêm vào I2C Bus
 * * @param[in] master_handle Handle của I2C Bus (đã init ở main)
 * @param[in] ina226_config Cấu hình thiết bị
 * @param[out] ina226_handle Con trỏ để nhận về handle thiết bị mới
 * @return esp_err_t
 */
esp_err_t ina226_init(i2c_master_bus_handle_t master_handle, const ina226_config_t *ina226_config, ina226_handle_t *ina226_handle);

/**
 * @brief Calibrate (Hiệu chỉnh) thiết bị để đo dòng điện và công suất
 * * @param handle Handle thiết bị
 * @param max_current Dòng điện tối đa (A)
 * @param shunt_resistance Giá trị trở Shunt (Ohm)
 * @return esp_err_t
 */
esp_err_t ina226_calibrate(ina226_handle_t handle, const float max_current, const float shunt_resistance);

/**
 * @brief Đọc điện áp Bus (V)
 * @param voltage Pointer nhận giá trị Volt
 */
esp_err_t ina226_get_bus_voltage(ina226_handle_t handle, float *const voltage);

/**
 * @brief Đọc điện áp rơi trên trở Shunt (V)
 * @param voltage Pointer nhận giá trị Volt (thường rất nhỏ)
 */
esp_err_t ina226_get_shunt_voltage(ina226_handle_t handle, float *const voltage);

/**
 * @brief Đọc dòng điện (A)
 * @note Chỉ hoạt động đúng nếu đã gọi ina226_calibrate
 * @param current Pointer nhận giá trị Ampe
 */
esp_err_t ina226_get_current(ina226_handle_t handle, float *const current);

/**
 * @brief Đọc công suất (W)
 * @note Chỉ hoạt động đúng nếu đã gọi ina226_calibrate
 * @param power Pointer nhận giá trị Watt
 */
esp_err_t ina226_get_power(ina226_handle_t handle, float *const power);

/**
 * @brief Reset thiết bị về mặc định (Soft Reset)
 */
esp_err_t ina226_reset(ina226_handle_t handle);

/**
 * @brief Xóa thiết bị khỏi bus và giải phóng bộ nhớ
 */
esp_err_t ina226_delete(ina226_handle_t handle);

#ifdef __cplusplus
}
#endif

#endif // __INA226_H__
