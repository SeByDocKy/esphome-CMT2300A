#include "cmt_spi3.h"
#include <Arduino.h>
#include <driver/spi_master.h>
#include <driver/gpio.h>

SemaphoreHandle_t paramLock = NULL;
#define SPI_PARAM_LOCK() \
    do {                 \
    } while (xSemaphoreTake(paramLock, portMAX_DELAY) != pdPASS)
#define SPI_PARAM_UNLOCK() xSemaphoreGive(paramLock)

#define SPI_CMT SPI2_HOST

// CS pin callbacks for manual chip select control
static void IRAM_ATTR pre_cb(spi_transaction_t* trans)
{
    gpio_set_level(*(gpio_num_t*)trans->user, 0);
}

static void IRAM_ATTR post_cb(spi_transaction_t* trans)
{
    gpio_set_level(*(gpio_num_t*)trans->user, 1);
}

spi_device_handle_t spi;
gpio_num_t cs_reg, cs_fifo;

void cmt_spi3_init(const int8_t pin_sdio, const int8_t pin_clk, const int8_t pin_cs, const int8_t pin_fcs, const uint32_t spi_speed)
{
    paramLock = xSemaphoreCreateMutex();

    spi_bus_config_t buscfg = {
        .mosi_io_num = pin_sdio,
        .miso_io_num = -1,  // 3-wire mode uses MOSI bidirectionally
        .sclk_io_num = pin_clk,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32,
    };

    spi_device_interface_config_t devcfg = {
        .command_bits = 0,  // Set per-transaction
        .address_bits = 0,  // Set per-transaction
        .dummy_bits = 0,
        .mode = 0,  // SPI mode 0
        .duty_cycle_pos = 0,
        .cs_ena_pretrans = 2,
        .cs_ena_posttrans = (uint8_t)(2 * spi_speed / 1000000),  // >2 us
        .clock_speed_hz = spi_speed,
        .input_delay_ns = 0,
        .spics_io_num = -1,  // CS handled manually via callbacks
        .flags = SPI_DEVICE_HALFDUPLEX | SPI_DEVICE_3WIRE,
        .queue_size = 1,
        .pre_cb = pre_cb,
        .post_cb = post_cb,
    };

    ESP_ERROR_CHECK(spi_bus_initialize(SPI_CMT, &buscfg, SPI_DMA_DISABLED));
    ESP_ERROR_CHECK(spi_bus_add_device(SPI_CMT, &devcfg, &spi));

    // Configure CS pins manually
    cs_reg = (gpio_num_t)pin_cs;
    ESP_ERROR_CHECK(gpio_reset_pin(cs_reg));
    ESP_ERROR_CHECK(gpio_set_level(cs_reg, 1));
    ESP_ERROR_CHECK(gpio_set_direction(cs_reg, GPIO_MODE_OUTPUT));

    cs_fifo = (gpio_num_t)pin_fcs;
    ESP_ERROR_CHECK(gpio_reset_pin(cs_fifo));
    ESP_ERROR_CHECK(gpio_set_level(cs_fifo, 1));
    ESP_ERROR_CHECK(gpio_set_direction(cs_fifo, GPIO_MODE_OUTPUT));

    delay(100);
}

void cmt_spi3_write(const uint8_t addr, const uint8_t dat)
{
    spi_transaction_ext_t trans = {
        .base = {
            .flags = SPI_TRANS_VARIABLE_CMD | SPI_TRANS_VARIABLE_ADDR,
            .cmd = 0,       // Write command
            .addr = addr,
            .length = 8,
            .rxlength = 0,
            .user = &cs_reg,
            .tx_buffer = &dat,
            .rx_buffer = NULL,
        },
        .command_bits = 1,
        .address_bits = 7,
        .dummy_bits = 0,
    };
    SPI_PARAM_LOCK();
    ESP_ERROR_CHECK(spi_device_polling_transmit(spi, (spi_transaction_t*)&trans));
    SPI_PARAM_UNLOCK();
}

uint8_t cmt_spi3_read(const uint8_t addr)
{
    uint8_t data = 0;
    spi_transaction_ext_t trans = {
        .base = {
            .flags = SPI_TRANS_VARIABLE_CMD | SPI_TRANS_VARIABLE_ADDR,
            .cmd = 1,       // Read command
            .addr = addr,
            .length = 0,
            .rxlength = 8,
            .user = &cs_reg,
            .tx_buffer = NULL,
            .rx_buffer = &data,
        },
        .command_bits = 1,
        .address_bits = 7,
        .dummy_bits = 0,
    };
    SPI_PARAM_LOCK();
    ESP_ERROR_CHECK(spi_device_polling_transmit(spi, (spi_transaction_t*)&trans));
    SPI_PARAM_UNLOCK();
    return data;
}

void cmt_spi3_write_fifo(const uint8_t* buf, const uint16_t len)
{
    spi_transaction_t trans = {
        .flags = 0,
        .cmd = 0,
        .addr = 0,
        .length = 8,
        .rxlength = 0,
        .user = &cs_fifo,
        .tx_buffer = NULL,
        .rx_buffer = NULL,
    };

    SPI_PARAM_LOCK();
    spi_device_acquire_bus(spi, portMAX_DELAY);
    for (uint16_t i = 0; i < len; i++) {
        trans.tx_buffer = buf + i;
        ESP_ERROR_CHECK(spi_device_polling_transmit(spi, &trans));
    }
    spi_device_release_bus(spi);
    SPI_PARAM_UNLOCK();
}

void cmt_spi3_read_fifo(uint8_t* buf, const uint16_t len)
{
    spi_transaction_t trans = {
        .flags = 0,
        .cmd = 0,
        .addr = 0,
        .length = 0,
        .rxlength = 8,
        .user = &cs_fifo,
        .tx_buffer = NULL,
        .rx_buffer = NULL,
    };

    SPI_PARAM_LOCK();
    spi_device_acquire_bus(spi, portMAX_DELAY);
    for (uint16_t i = 0; i < len; i++) {
        trans.rx_buffer = buf + i;
        ESP_ERROR_CHECK(spi_device_polling_transmit(spi, &trans));
    }
    spi_device_release_bus(spi);
    SPI_PARAM_UNLOCK();
}
