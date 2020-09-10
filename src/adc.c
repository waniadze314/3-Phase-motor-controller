#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "adc.h"
#include "driver/adc.h"
#include "driver/i2s.h"
#include "soc/syscon_reg.h"
#include "esp_event_loop.h"

#define ZERO_CURRENT 0
#define CURRENT_SCALE_FACTOR 1.0

QueueHandle_t i2s_evt_queue;
float actual_current[3];

void init_adc_dma(){

    i2s_config_t i2s_conf={
        .mode = I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN,
        .sample_rate = 16000,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,
        .communication_format = I2S_COMM_FORMAT_I2S ,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 2,
        .dma_buf_len = 32,
    };
    adc_gpio_init(ADC_UNIT_1, ADC_CHANNEL_6);
    adc_gpio_init(ADC_UNIT_1, ADC_CHANNEL_7);
    i2s_driver_install(I2S_NUM_0, &i2s_conf, 1, &i2s_evt_queue);
    i2s_set_adc_mode(I2S_NUM_0, ADC_CHANNEL_6);
    SET_PERI_REG_BITS(SYSCON_SARADC_CTRL_REG, SYSCON_SARADC_SAR1_PATT_LEN, 1, SYSCON_SARADC_SAR1_PATT_LEN_S);
    WRITE_PERI_REG(SYSCON_SARADC_SAR1_PATT_TAB1_REG, 0x6E7E0000);
    SET_PERI_REG_MASK(SYSCON_SARADC_CTRL2_REG, SYSCON_SARADC_SAR1_INV);
    vTaskDelay(5000/portTICK_RATE_MS);
}

void calculate_current_task(){
    uint16_t adc_read[2];
    const uint16_t buffer_len = 32;
    size_t bytes_read;
    system_event_t evt;
    uint16_t i2s_buffer[buffer_len];
    init_adc_dma();
    i2s_adc_enable(I2S_NUM_0);
    while(1){ 
        if(xQueueReceive(i2s_evt_queue, &evt, portMAX_DELAY)==pdPASS){
            adc_read[0]=0;
            adc_read[1]=0;
            uint8_t ch_6_index = 0, ch_7_index = 0;
            if(evt.event_id == 2){
                i2s_read(I2S_NUM_0, (char*)i2s_buffer,buffer_len*2, &bytes_read, portMAX_DELAY );
                for(uint8_t i=0; i<16; i++){
                    if(((i2s_buffer[i]&0xF000)>>12 == 6) && (ch_6_index < 4)){
                        adc_read[0]+=i2s_buffer[i]&0x0FFF;
                        ch_6_index++;
                    }
                    else if(((i2s_buffer[i]&0xF000)>>12 == 7) && (ch_7_index < 4)){
                        adc_read[1]+=i2s_buffer[i]&0x0FFF;
                        ch_7_index++;
                    }
                }
            }            
            actual_current[0] = (float)((adc_read[0]>>2) - ZERO_CURRENT)*CURRENT_SCALE_FACTOR;
            actual_current[1] = (float)((adc_read[1]>>2) - ZERO_CURRENT)*CURRENT_SCALE_FACTOR;
            actual_current[2] = 0.0 - actual_current[0] - actual_current[1];
            
            printf("U: %f V: %f W: %f\n", actual_current[0], actual_current[1], actual_current[2]);
            vTaskDelay(200/portTICK_RATE_MS);
        }   
    }
}