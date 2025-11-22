
#ifndef SPI_INIT_H
#define SPI_INIT_H

#include "esp_err.h"
#include "driver/spi_master.h"

typedef struct {
    spi_host_device_t host;     // SPI2_HOST, SPI3_HOST, v.v.
    int sclk_io;
    int mosi_io;
    int miso_io;
    int max_transfer_sz;
} spi_bus_init_config_t;

typedef struct {
    int cs_io;
    int dc_io;
    int reset_io;
    int clock_speed_hz;
    spi_mode_t mode;
    int queue_size;
} spi_device_config_t;

esp_err_t spi_bus_init(const spi_bus_init_config_t *bus_cfg);
esp_err_t spi_device_init(spi_host_device_t host, const spi_device_config_t *dev_cfg, spi_device_handle_t *out_handle);

#endif // SPI_INIT_H
