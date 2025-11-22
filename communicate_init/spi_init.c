#include "spi_init.h"
#include "driver/spi_master.h"
#include "esp_log.h"

static const char *TAG = "spi_init";

esp_err_t spi_bus_init(const spi_bus_init_config_t *bus_cfg) {
    if (!bus_cfg) return ESP_ERR_INVALID_ARG;

    spi_bus_config_t bus_conf = {
        .miso_io_num = bus_cfg->miso_io,
        .mosi_io_num = bus_cfg->mosi_io,
        .sclk_io_num = bus_cfg->sclk_io,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = bus_cfg->max_transfer_sz,
    };

    esp_err_t err = spi_bus_initialize(bus_cfg->host, &bus_conf, SPI_DMA_CH_AUTO);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_initialize failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "SPI bus initialized: host %d, SCLK %d, MOSI %d, MISO %d",
             bus_cfg->host, bus_cfg->sclk_io, bus_cfg->mosi_io, bus_cfg->miso_io);
    return ESP_OK;
}

esp_err_t spi_device_init(spi_host_device_t host, const spi_device_config_t *dev_cfg, spi_device_handle_t *out_handle) {
    if (!dev_cfg || !out_handle) return ESP_ERR_INVALID_ARG;

    spi_device_interface_config_t dev_intf = {
        .clock_speed_hz = dev_cfg->clock_speed_hz,
        .mode = dev_cfg->mode,
        .spics_io_num = dev_cfg->cs_io,
        .queue_size = dev_cfg->queue_size,
        .flags = SPI_DEVICE_HALFDUPLEX,  // SSD1306 SPI thường là half-duplex
    };

    esp_err_t err = spi_bus_add_device(host, &dev_intf, out_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_add_device failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "SPI device added: CS %d, DC %d, RST %d, speed %d, mode %d",
             dev_cfg->cs_io, dev_cfg->dc_io, dev_cfg->reset_io, dev_cfg->clock_speed_hz, dev_cfg->mode);

    return ESP_OK;
}
