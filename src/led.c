#include "led.h"


void init_led_gpio(){
    gpio_config_t gpio_conf;
    gpio_conf.mode = GPIO_MODE_OUTPUT;
    gpio_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_conf.pin_bit_mask = (1<<GPIO_LED);
    gpio_config(&gpio_conf);
}

void blink_led_task(){ 
    init_led_gpio();
    while(1){
        gpio_set_level(GPIO_LED, 1);
        vTaskDelay(500/portTICK_RATE_MS);
        gpio_set_level(GPIO_LED,0);
        vTaskDelay(500/portTICK_RATE_MS);
    }
}