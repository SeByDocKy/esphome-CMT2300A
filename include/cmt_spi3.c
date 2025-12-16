#include "cmt_spi3.h"
#include <Arduino.h>
#include <driver/spi_master.h>
#include <driver/gpio.h>
#include <esp_rom_gpio.h>

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
  #include <soc/spi_periph.h>
#endif

SemaphoreHandle_t paramLock = NULL;
#define SPI_PARAM_LOCK() \
    do {                 \
    } while (xSemaphoreTake(paramLock, portMAX_DELAY) != pdPASS)
#define SPI_PARAM_UNLOCK() xSemaphoreGive(paramLock)

// Use SPI2_HOST consistently for all ESP32 variants
// (HSPI is just an alias for SPI2_HOST on original ESP32, but may not be
// defined for newer variants like S3/P4)
#define SPI_CMT SPI2_HOST

spi_device_handle_t spi_reg, spi_fifo;

void cmt_spi3_init(const int8_t pin_sdio, const int8_t pin_clk, const int8_t pin_cs, const int8_t pin_fcs, const uint32_t spi_speed)
{
    paramLock = xSemaphoreCreateMutex();

    spi_bus_config_t buscfg = {
        .mosi_io_num = pin_sdio,
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
        .miso_io_num = pin_sdio,  // Same pin for bidirectional 3-wire SPI
#else
        .miso_io_num = -1,  // Single wire MOSI/MISO
#endif
        .sclk_io_num = pin_clk,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32,
    };

    spi_device_interface_config_t devcfg = {
        .command_bits = 1,
        .address_bits = 7,
        .dummy_bits = 0,
        .mode = 0,  // SPI mode 0
        .cs_ena_pretrans = 1,
        .cs_ena_posttrans = 1,
        .clock_speed_hz = spi_speed,
        .spics_io_num = pin_cs,
        .flags = SPI_DEVICE_HALFDUPLEX | SPI_DEVICE_3WIRE,
        .queue_size = 1,
        .pre_cb = NULL,
        .post_cb = NULL,
    };

    ESP_ERROR_CHECK(spi_bus_initialize(SPI_CMT, &buscfg, SPI_DMA_DISABLED));
    ESP_ERROR_CHECK(spi_bus_add_device(SPI_CMT, &devcfg, &spi_reg));

    // FIFO device
    spi_device_interface_config_t devcfg2 = {
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .mode = 0,  // SPI mode 0
        .cs_ena_pretrans = 2,
        .cs_ena_posttrans = (uint8_t)(1 / (spi_speed * 10e6 * 2) + 2),  // >2 us
        .clock_speed_hz = spi_speed,
        .spics_io_num = pin_fcs,
        .flags = SPI_DEVICE_HALFDUPLEX | SPI_DEVICE_3WIRE,
        .queue_size = 1,
        .pre_cb = NULL,
        .post_cb = NULL,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(SPI_CMT, &devcfg2, &spi_fifo));

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    // ESP-IDF 5.x: Configure GPIO for bidirectional operation
    // This is needed because 3-wire SPI mode in ESP-IDF 5.x requires
    // explicit GPIO configuration for the shared MOSI/MISO pin
    gpio_set_direction((gpio_num_t)pin_sdio, GPIO_MODE_INPUT_OUTPUT);
    esp_rom_gpio_connect_out_signal(pin_sdio, spi_periph_signal[SPI_CMT].spid_out, false, false);
    esp_rom_gpio_connect_in_signal(pin_sdio, spi_periph_signal[SPI_CMT].spiq_in, false);
#else
    esp_rom_gpio_connect_out_signal(pin_sdio, spi_periph_signal[SPI_CMT].spid_out, true, false);
#endif

    delay(100);
}

void cmt_spi3_write(const uint8_t addr, const uint8_t dat)
{
    uint8_t tx_data = ~dat;
    spi_transaction_t t = {
        .cmd = 1,
        .addr = ~addr,
        .length = 8,
        .rxlength = 0,
        .tx_buffer = &tx_data,
        .rx_buffer = NULL
    };
    SPI_PARAM_LOCK();
    ESP_ERROR_CHECK(spi_device_polling_transmit(spi_reg, &t));
    SPI_PARAM_UNLOCK();
    delayMicroseconds(100);
}

uint8_t cmt_spi3_read(const uint8_t addr)
{
    uint8_t rx_data = 0;
    spi_transaction_t t = {
        .cmd = 0,
        .addr = ~addr,
        // ESP-IDF 5.x requires length=0 for read-only transactions in half-duplex mode
        // ESP-IDF 4.x used length=8 for both TX and RX phases
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
        .length = 0,
#else
        .length = 8,
#endif
        .rxlength = 8,
        .tx_buffer = NULL,
        .rx_buffer = &rx_data
    };
    SPI_PARAM_LOCK();
    ESP_ERROR_CHECK(spi_device_polling_transmit(spi_reg, &t));
    SPI_PARAM_UNLOCK();
    delayMicroseconds(100);
    return rx_data;
}

void cmt_spi3_write_fifo(const uint8_t* buf, const uint16_t len)
{
    uint8_t tx_data;

    spi_transaction_t t = {
        .length = 8,
        .rxlength = 0,
        .tx_buffer = &tx_data,
        .rx_buffer = NULL
    };

    SPI_PARAM_LOCK();
    for (uint8_t i = 0; i < len; i++) {
        tx_data = ~buf[i];  // Negate buffer contents
        ESP_ERROR_CHECK(spi_device_polling_transmit(spi_fifo, &t));
        delayMicroseconds(4);  // > 4 us
    }
    SPI_PARAM_UNLOCK();
}

void cmt_spi3_read_fifo(uint8_t* buf, const uint16_t len)
{
    uint8_t rx_data;

    spi_transaction_t t = {
        // ESP-IDF 5.x requires length=0 for read-only transactions
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
        .length = 0,
#else
        .length = 8,
#endif
        .rxlength = 8,
        .tx_buffer = NULL,
        .rx_buffer = &rx_data
    };

    SPI_PARAM_LOCK();
    for (uint8_t i = 0; i < len; i++) {
        ESP_ERROR_CHECK(spi_device_polling_transmit(spi_fifo, &t));
        delayMicroseconds(4);  // > 4 us
        buf[i] = rx_data;
    }
    SPI_PARAM_UNLOCK();
}
