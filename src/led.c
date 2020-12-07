#include "led.h"
#include "soc/timer_group_reg.h"
#include "soc/timer_group_struct.h"

uint8_t blink_on = 0;

void init_led_gpio(){
    gpio_config_t gpio_conf;
    gpio_conf.mode = GPIO_MODE_OUTPUT;
    gpio_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_conf.pin_bit_mask = (1<<GPIO_LED);
    gpio_config(&gpio_conf);
}

void toggle_blink(int value){
    if(value==0) blink_on = 0;
    else blink_on = 1;
}

void blink_led_task(){ 
    init_led_gpio();
    float blink_time_sec = 0.2;
    int blink_time = (int)(blink_time_sec*1000/portTICK_RATE_MS);
    while(1){
        if(blink_on){
            gpio_set_level(GPIO_LED, 1);
            vTaskDelay(blink_time);
            gpio_set_level(GPIO_LED,0);
            vTaskDelay(blink_time);
        }
        else{            
            vTaskDelay(10/portTICK_RATE_MS);
            TIMERG0.wdt_wprotect = TIMG_WDT_WKEY_VALUE;
            TIMERG0.wdt_feed = 1;
            TIMERG0.wdt_wprotect = 0;
        }
    }
}