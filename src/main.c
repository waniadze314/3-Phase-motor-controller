#include "adc.h"
#include "pwm.h"
#include "communiaction.h"
#include "encoder.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led.h"
void app_main() {
    xTaskCreatePinnedToCore(calculate_current_task, "CURRENT", 4096, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(communication_task, "COMMUNICATION", 4096, NULL, 0, NULL, 0);
    // xTaskCreatePinnedToCore(encoder_task, "ENCODER", 4096, NULL, 0, NULL, 1);
}