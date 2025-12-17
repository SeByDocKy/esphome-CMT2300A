#include "cmt_spi3.h"
#include <Arduino.h>
#include <driver/spi_master.h>

#if CONFIG_IDF_TARGET_ESP32
  #include <esp_rom_gpio.h> // for esp_rom_gpio_connect_out_signal
#elif (CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32P4)
  #include <driver/gpio.h> 
#endif  

#if CONFIG_IDF_TARGET_ESP32    
  #include <soc/spi_periph.h>  // AND ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
#endif

SemaphoreHandle_t paramLock = NULL;
#define SPI_PARAM_LOCK() \
    do {                 \
    } while (xSemaphoreTake(paramLock, portMAX_DELAY) != pdPASS)
#define SPI_PARAM_UNLOCK() xSemaphoreGive(paramLock)

#define SPI_CMT SPI2_HOST

spi_device_handle_t spi_reg;

#if CONFIG_IDF_TARGET_ESP32
  spi_device_handle_t spi_fifo;
  
#elif (CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32P4) && ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
  static void IRAM_ATTR pre_cb(spi_transaction_t* trans)
  {
    gpio_set_level(*(gpio_num_t*)trans->user, 0);
  }

  static void IRAM_ATTR post_cb(spi_transaction_t* trans)
  {
    gpio_set_level(*(gpio_num_t*)trans->user, 1);
  }

  gpio_num_t cs_reg, cs_fifo;

#endif

void cmt_spi3_init(const int8_t pin_sdio, const int8_t pin_clk, const int8_t pin_cs, const int8_t pin_fcs, const uint32_t spi_speed)
{
   paramLock = xSemaphoreCreateMutex();	
   
   spi_bus_config_t buscfg = {
        .mosi_io_num = pin_sdio,
        .miso_io_num = -1, // single wire MOSI/MISO
        .sclk_io_num = pin_clk,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32,
    };
	
#if CONFIG_IDF_TARGET_ESP32 && ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
	
	spi_device_interface_config_t devcfg = {
        .command_bits = 1,
        .address_bits = 7,
        .dummy_bits = 0,
        .mode = 0, // SPI mode 0
        .cs_ena_pretrans = 1,
        .cs_ena_posttrans = 1,
        .clock_speed_hz = spi_speed,
        .spics_io_num = pin_cs,
        .flags = SPI_DEVICE_HALFDUPLEX | SPI_DEVICE_3WIRE,
        .queue_size = 1,
        .pre_cb = NULL,
        .post_cb = NULL,
    };
	
#elif (CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32P4) && ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
	  
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
#endif

	ESP_ERROR_CHECK(spi_bus_initialize(SPI_CMT, &buscfg, SPI_DMA_DISABLED));
    ESP_ERROR_CHECK(spi_bus_add_device(SPI_CMT, &devcfg, &spi_reg));
		
#if CONFIG_IDF_TARGET_ESP32 && ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)

    spi_device_interface_config_t devcfg2 = {
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .mode = 0, // SPI mode 0
        .cs_ena_pretrans = 2,
        .cs_ena_posttrans = (uint8_t)(1 / (spi_speed * 10e6 * 2) + 2), // >2 us
        .clock_speed_hz = spi_speed,
        .spics_io_num = pin_fcs,
        .flags = SPI_DEVICE_HALFDUPLEX | SPI_DEVICE_3WIRE,
        .queue_size = 1,
        .pre_cb = NULL,
        .post_cb = NULL,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(SPI_CMT, &devcfg2, &spi_fifo));
	esp_rom_gpio_connect_out_signal(pin_sdio, spi_periph_signal[SPI_CMT].spid_out, true, false);
	
#elif (CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32P4) && ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)

    cs_reg = (gpio_num_t)pin_cs;
    ESP_ERROR_CHECK(gpio_reset_pin(cs_reg));
    ESP_ERROR_CHECK(gpio_set_level(cs_reg, 1));
    ESP_ERROR_CHECK(gpio_set_direction(cs_reg, GPIO_MODE_OUTPUT));

    cs_fifo = (gpio_num_t)pin_fcs;
    ESP_ERROR_CHECK(gpio_reset_pin(cs_fifo));
    ESP_ERROR_CHECK(gpio_set_level(cs_fifo, 1));
    ESP_ERROR_CHECK(gpio_set_direction(cs_fifo, GPIO_MODE_OUTPUT));	

#endif

    delay(100);
	
}


void cmt_spi3_write(const uint8_t addr, const uint8_t dat)
{
#if CONFIG_IDF_TARGET_ESP32 && ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)	
    uint8_t tx_data;
    tx_data = ~dat;
    spi_transaction_t t = {
        .cmd = 1,
        .addr = ~addr,
        .length = 8,
        .tx_buffer = &tx_data,
        .rx_buffer = NULL
    };
    SPI_PARAM_LOCK();
    ESP_ERROR_CHECK(spi_device_polling_transmit(spi_reg, &t));
    SPI_PARAM_UNLOCK();

#elif (CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32P4) && ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)

    spi_transaction_ext_t t = {
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
    ESP_ERROR_CHECK(spi_device_polling_transmit(spi_reg, (spi_transaction_t*)&t));
    SPI_PARAM_UNLOCK();

#endif
	
	delayMicroseconds(100);
}


uint8_t cmt_spi3_read(const uint8_t addr)
{
	uint8_t rx_data;	
#if CONFIG_IDF_TARGET_ESP32 && ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)    
    spi_transaction_t t = {
        .cmd = 0,
        .addr = ~addr,
        .length = 8,
        .rxlength = 8,
        .tx_buffer = NULL,
        .rx_buffer = &rx_data
    };
    SPI_PARAM_LOCK();
    ESP_ERROR_CHECK(spi_device_polling_transmit(spi_reg, &t));
    SPI_PARAM_UNLOCK(); 
    
	
#elif (CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32P4) && ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    spi_transaction_ext_t t = {
        .base = {
            .flags = SPI_TRANS_VARIABLE_CMD | SPI_TRANS_VARIABLE_ADDR,
            .cmd = 1,       // Read command
            .addr = addr,
            .length = 0,
            .rxlength = 8,
            .user = &cs_reg,
            .tx_buffer = NULL,
            .rx_buffer = &rx_data,
        },
        .command_bits = 1,
        .address_bits = 7,
        .dummy_bits = 0,
    };
    SPI_PARAM_LOCK();
    ESP_ERROR_CHECK(spi_device_polling_transmit(spi_reg, (spi_transaction_t*)&t));
    SPI_PARAM_UNLOCK();

#endif
    delayMicroseconds(100);
    return rx_data;
}

void cmt_spi3_write_fifo(const uint8_t* buf, const uint16_t len)
{
    
#if CONFIG_IDF_TARGET_ESP32 && ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)	

    uint8_t tx_data;
	spi_transaction_t t = {
        .length = 8,
        .tx_buffer = &tx_data, // reference to write data
        .rx_buffer = NULL
    };

    SPI_PARAM_LOCK();
    for (uint8_t i = 0; i < len; i++) {
        tx_data = ~buf[i]; // negate buffer contents
        ESP_ERROR_CHECK(spi_device_polling_transmit(spi_fifo, &t));
        delayMicroseconds(4); // > 4 us
    }
    SPI_PARAM_UNLOCK();
	
#elif (CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32P4) && ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)

   spi_transaction_t t = {
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
        t.tx_buffer = buf + i;
        ESP_ERROR_CHECK(spi_device_polling_transmit(spi_reg, &t));
    }
    spi_device_release_bus(spi_reg);
    SPI_PARAM_UNLOCK();	
	
#endif	
}

void cmt_spi3_read_fifo(uint8_t* buf, const uint16_t len)
{
   
	
#if CONFIG_IDF_TARGET_ESP32 && ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)	

    uint8_t rx_data;
    spi_transaction_t t = {
        .length = 8,
        .rxlength = 8,
        .tx_buffer = NULL,
        .rx_buffer = &rx_data
    };

    SPI_PARAM_LOCK();
    for (uint8_t i = 0; i < len; i++) {
        ESP_ERROR_CHECK(spi_device_polling_transmit(spi_fifo, &t));
        delayMicroseconds(4); // > 4 us
        buf[i] = rx_data;
    }
    SPI_PARAM_UNLOCK();
	
#elif (CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32P4) && ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    
    spi_transaction_t t = {
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
        t.rx_buffer = buf + i;
        ESP_ERROR_CHECK(spi_device_polling_transmit(spi_reg, &t));
    }
    spi_device_release_bus(spi_reg);
    SPI_PARAM_UNLOCK();

#endif	
}
