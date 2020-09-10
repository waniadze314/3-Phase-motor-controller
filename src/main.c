#include "adc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
void app_main() {
    xTaskCreatePinnedToCore(calculate_current_task, "CURRENT", 8192, NULL, 0, NULL, 0);
}