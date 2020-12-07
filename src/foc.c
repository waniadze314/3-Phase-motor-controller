#include "foc.h"
#include "adc.h"
#include "pwm.h"
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define MAGNETIC_PITCH 60.0
#define INTEGRATION_LIMIT 1.0
#define CURRENT_INTEGRATION_LIMIT 0.5
#define MAX_STEP 0.5
extern float actual_current[3];
uint8_t commutation_established;
float position_to_angle_correction;
float position;
float actual_phase_angle;
float angle_error;
float commanded_phase_angle;
float temp_phase_angle;
float angle_step;

float KP = 0.05;
float KI = 0.0;
float KD = 0.0;

float D_KP = 0.6;
float D_KI = 0.0;

float Q_KP = 0.6;
float Q_KI = 0.0;

void establish_commutation(){
	float phase_current[3];
	float c = 0.1,p = 90.0;

	for(c = 0.1; c <= 0.7; c+=0.1){
		phase_current[0] = sin(angle_to_rad(90.0))*c;
		phase_current[1] = sin(angle_to_rad(330.0))*c;
		phase_current[2] = sin(angle_to_rad(210.0))*c;
		current_to_pwm_transform(phase_current, 2.0);
		vTaskDelay(250/portTICK_RATE_MS);
	}
	vTaskDelay(1000/portTICK_RATE_MS);
	for(p = 90.0; p <= 180.0; p+=1.0){
		phase_current[0] = sin(angle_to_rad(p))*c;
		phase_current[1] = sin(angle_to_rad(p - 120.0))*c;
		phase_current[2] = sin(angle_to_rad(p - 240.0))*c;
		current_to_pwm_transform(phase_current, 2.0);
		vTaskDelay(5/portTICK_RATE_MS);
	}
	for(p = 180.0; p >= 0.0; p-=1.0){
		phase_current[0] = sin(angle_to_rad(p))*c;
		phase_current[1] = sin(angle_to_rad(p - 120.0))*c;
		phase_current[2] = sin(angle_to_rad(p - 240.0))*c;
		current_to_pwm_transform(phase_current, 2.0);
		vTaskDelay(5/portTICK_RATE_MS);
	}
	position_to_angle_correction = position;
	commanded_phase_angle = 0.0;
	temp_phase_angle = actual_phase_angle;
	commutation_established = 1;
	return;
}

float* clarke_park_transform(float* current, float angle){
    static float DQ[2];
	DQ[0] = 0.667*(cos(angle)*current[0] + cos(angle-0.667*M_PI)*current[1] + cos(angle+0.667*M_PI)*current[2]);
	DQ[1] = 0.667*(sin(angle)*current[0] + sin(angle-0.667*M_PI)*current[1] + sin(angle+0.667*M_PI)*current[2]);
    return DQ;
}

float* inverse_clarke_park_transform(float* DQ, float angle){
	static float desired_current[3];
	desired_current[0] = cos(angle)*DQ[0] - sin(angle)*DQ[1]; //phase A
	desired_current[1] = cos(angle-0.667*M_PI)*DQ[0] - sin(angle-0.667*M_PI)*DQ[1]; //phase B
	desired_current[2] = cos(angle+0.667*M_PI)*DQ[0] - sin(angle+0.667*M_PI)*DQ[1]; //phase C
	return desired_current;
}

float estimate_angle(){
	actual_phase_angle = (position - position_to_angle_correction) * (360.0/MAGNETIC_PITCH);
	return actual_phase_angle;
}

void regulate_angle(){

	static float integral_angle_error = 0.0;
	static float previous_proportional_angle_error = 0.0;
    float P,I,D;
	float proportional_angle_error = angle_error;
	float derrivative_angle_error = (proportional_angle_error - previous_proportional_angle_error);
	integral_angle_error += proportional_angle_error;
	P = KP*proportional_angle_error;
	I = KI*integral_angle_error;
	D = KD*derrivative_angle_error;
	if(I > INTEGRATION_LIMIT){
		integral_angle_error = INTEGRATION_LIMIT/KI;
		I = INTEGRATION_LIMIT;
	}
	else if(I < -INTEGRATION_LIMIT){
		integral_angle_error = INTEGRATION_LIMIT/KI;
		I = (-1.0)*INTEGRATION_LIMIT;
	}
	previous_proportional_angle_error = proportional_angle_error;
	angle_step = P + I + D;
	if(angle_step > MAX_STEP) angle_step = MAX_STEP;
	else if(angle_step < -MAX_STEP) angle_step = -MAX_STEP;
	temp_phase_angle += angle_step;
	return;
}


float* regulate_DQ(){
	static float integral_DQ_error[2] = {0.0, 0.0};
	float DQ_error[2];
	const float desired_D = 0.0;
//	float desired_Q = (-1)*abs(angle_error)/5.0;
	float desired_Q = 0.5;
	float* current_command;
	// float* actual_current = calculate_current();
	float DQ_command[2];
	estimate_angle();
	float* DQ = clarke_park_transform(actual_current,angle_to_rad(actual_phase_angle));
	DQ_error[0] = desired_D - DQ[0];
	DQ_error[1] = desired_Q - DQ[1];
	integral_DQ_error[0] += DQ_error[0];
	integral_DQ_error[1] += DQ_error[1];
	if(integral_DQ_error[0] > CURRENT_INTEGRATION_LIMIT) integral_DQ_error[0] = CURRENT_INTEGRATION_LIMIT;
	else if(integral_DQ_error[0] > -CURRENT_INTEGRATION_LIMIT) integral_DQ_error[0] = -CURRENT_INTEGRATION_LIMIT;
	if(integral_DQ_error[1] > CURRENT_INTEGRATION_LIMIT) integral_DQ_error[1] = CURRENT_INTEGRATION_LIMIT;
	else if(integral_DQ_error[1] > -CURRENT_INTEGRATION_LIMIT) integral_DQ_error[1] = -CURRENT_INTEGRATION_LIMIT;
	DQ_command[0] = D_KP * DQ_error[0] + D_KI * integral_DQ_error[0];
	DQ_command[1] = Q_KP * DQ_error[1] + Q_KI * integral_DQ_error[1];
	current_command = inverse_clarke_park_transform(DQ_command, angle_to_rad(temp_phase_angle));
	return current_command;
}

float angle_to_rad(float angle){
	return (angle/180.0)*M_PI;
}


void set_KP(uint16_t value){
	KP = (float)value/100.0;
}

void set_KI(uint16_t value){
	KI = (float)value/100.0;
}

void set_KD(uint16_t value){
	KD = (float)value/100.0;
}

void set_Q_KP(uint16_t value){
	Q_KP = (float)value/100.0;
}

void set_Q_KI(uint16_t value){
	Q_KI = (float)value/100.0;
}
void set_D_KP(uint16_t value){
	D_KP = (float)value/100.0;
}

void set_D_KI(uint16_t value){
	D_KI = (float)value/100.0;
}