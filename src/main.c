#include "adc.h"
#include "pwm.h"
#include "communiaction.h"
#include "encoder.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led.h"
void app_main() {
    // xTaskCreatePinnedToCore(calculate_current_task, "CURRENT", 1024, NULL, 2, NULL, 0);
    // xTaskCreatePinnedToCore(communication_task, "COMMUNICATION", 1024, NULL, 1, NULL, 0);
    // xTaskCreatePinnedToCore(encoder_task, "ENCODER", 1024, NULL, 0, NULL, 1);
    xTaskCreatePinnedToCore(blink_led_task, "LED", 2048, NULL, 0, NULL, 0);
}