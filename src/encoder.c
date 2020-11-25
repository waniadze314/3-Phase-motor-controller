#include "encoder.h"
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "soc/rtc_periph.h"
#include "driver/spi_slave.h"
#include "driver/gpio.h"

#define GPIO_MOSI 13
#define GPIO_MISO 12
#define GPIO_SCLK 14
#define GPIO_CS 15

#define RCV_HOST    HSPI_HOST
#define DMA_CHAN    0

float position = 0.0;

void init_spi(){
    esp_err_t ret;
    spi_bus_config_t spi_bus_cfg = {
        .mosi_io_num = GPIO_MOSI,
        .miso_io_num = GPIO_MISO,
        .sclk_io_num = GPIO_SCLK,
        .quadhd_io_num = -1,
        .quadwp_io_num = -1,
    };

    spi_slave_interface_config_t spi_slave_cfg = {
        .mode = 0,
        .spics_io_num = GPIO_CS,
        .queue_size = 3,
        .flags = 0,
        .post_setup_cb = NULL,
        .post_trans_cb = NULL,
    };
    gpio_set_pull_mode(GPIO_MOSI, GPIO_PULLUP_ENABLE);
    gpio_set_pull_mode(GPIO_SCLK, GPIO_PULLUP_ENABLE);
    gpio_set_pull_mode(GPIO_CS, GPIO_PULLUP_ENABLE);

    ret = spi_slave_initialize(RCV_HOST, &spi_bus_cfg, &spi_slave_cfg, DMA_CHAN);
    assert(ret == ESP_OK);
}

void encoder_task(){
    esp_err_t status;
    spi_slave_transaction_t spi_slv_transaction;
    init_spi();
    WORD_ALIGNED_ATTR uint8_t recieve_buffer[16];
    WORD_ALIGNED_ATTR uint8_t send_buffer[80];
    memset(recieve_buffer,0, 16);
    memset(&spi_slv_transaction, 0, sizeof(spi_slv_transaction));

    while(1){
        memset(recieve_buffer, 0, 16);
        spi_slv_transaction.length = 16;
        spi_slv_transaction.rx_buffer = recieve_buffer;
        spi_slv_transaction.tx_buffer = send_buffer;
        status = spi_slave_transmit(RCV_HOST, &spi_slv_transaction, portMAX_DELAY);
        assert(status = ESP_OK);
    }
}