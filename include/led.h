#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#define GPIO_LED 4
void init_led_gpio();
void blink_led();