
typedef enum{
    write, read
}operation;

void init_uart();
void parse_command(char* command);
void communication_task();
void execute_command(char* command, int num_value, operation op);