#include <stdio.h>
#include <string.h>
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "communiaction.h"

#define UART_NUMBER UART_NUM_0
#define RD_BUF_SIZE (80)
static QueueHandle_t uart_queue;

extern float D_KP;
extern float D_KI;

extern float Q_KP;
extern float Q_KI;

extern float KP;
extern float KI;
extern float KD;

void init_uart(){
    uart_config_t uart_cfg = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE, 
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, 
        .rx_flow_ctrl_thresh = 122,        
    };

    uart_driver_install(UART_NUMBER, RD_BUF_SIZE*2, RD_BUF_SIZE*2, 20, &uart_queue, 0);
    uart_param_config(UART_NUMBER, &uart_cfg);
    uart_set_pin(UART_NUMBER, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

}

void parse_command(char* command){
	char function[2];
	char value[50];
	int32_t num_value = 0;
	uint8_t command_index = 0;
    
    operation op;
	function[0]=command[0];
	function[1]=command[1];
	if(command[2]!='=') op = read;
	else {
		op = write;
		while (command[command_index] != ';') {
			value[command_index] = command[command_index + 3];
			command_index++;
		}
		num_value = atoi(value);
	}
	// execute_command(function, num_value, op);
    // uart_write_bytes(UART_NUMBER, (const char*) function, 2);
    printf("%c%c %d\n", function[0], function[1], num_value);
	return;
}

void execute_command(char* function, int value, operation op){
}

void communication_task(){
    const uint8_t buffer_size = 80;
    uint8_t char_counter = 0;
    char command[buffer_size];
    uart_event_t uart_evt;
    // size_t buffered_size;
    uint8_t* dtmp = (uint8_t*) malloc(RD_BUF_SIZE);
    init_uart();
    while(1){
        if(xQueueReceive(uart_queue, (void*)&uart_evt, (portTickType)portMAX_DELAY)){
            bzero(dtmp, RD_BUF_SIZE);
            if(uart_evt.type == UART_DATA){
                uart_read_bytes(UART_NUMBER, dtmp, uart_evt.size, portMAX_DELAY);
                if(char_counter == 0) memset(command, 0, buffer_size);
                command[char_counter] = dtmp[0];
                char_counter++;
                if(dtmp[0] == ';'){
                    // uart_write_bytes(UART_NUMBER, (const char*)command, char_counter);
                    parse_command(command);
                    char_counter = 0;
                    memset(command, buffer_size, sizeof(char));
                }
            }
        }        
    }
}