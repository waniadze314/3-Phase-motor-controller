#define PWM_U 12
#define PWM_V 13
#define PWM_W 14
#define CTRL_FREQ_MS 10

#include "driver/mcpwm.h"
#include "pwm.h"
#include "foc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/mcpwm_periph.h"

extern float angle_error;
extern float commanded_phase_angle;
extern float actual_phase_angle;
extern uint8_t commutation_established;


void init_pwm(){
    mcpwm_config_t pwm_config ={
        .frequency = 20000,
        .cmpr_a = 50.0,
        .counter_mode = MCPWM_UP_COUNTER,
        .duty_mode = MCPWM_DUTY_MODE_0,
    };
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, PWM_U);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, PWM_V);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM2A, PWM_W);
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config);
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_2, &pwm_config);
    mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_0); 
    mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_1);
    mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_2);
}

void current_to_pwm_transform(float* current, float max_amplitude){
    float pwm_values[3];
    for(uint8_t i = 0; i<3; i++){
        pwm_values[i] = ((current[i] + (max_amplitude/2.0))/max_amplitude) * 100.0;
        if(pwm_values[i] > 100.0) pwm_values[i] = 100.0;
        else if(pwm_values[i] < 0.0) pwm_values[i] = 0.0;
    }
}

void test_pwm(){
    init_pwm();
    while(1){
        for(int i = 25; i<75; i++){
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, i-20);
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, i);
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, i+20);
            vTaskDelay(20/portTICK_RATE_MS);
        }
    }
}

void pwm_task(){
    if(commutation_established == 1){
        angle_error = commanded_phase_angle - actual_phase_angle;
        regulate_angle();
        current_to_pwm_transform(regulate_DQ(),2.0);
    }
    vTaskDelay(CTRL_FREQ_MS/portTICK_RATE_MS);
    
}