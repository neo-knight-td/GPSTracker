#ifndef SIM800_H
#define SIM800_H

#include "string.h"
#include "stdlib.h"
#include "stdio.h"
#include "stm32g0xx_hal.h"

int sim800_AT_OK(uint8_t debug_on, UART_HandleTypeDef huart2, UART_HandleTypeDef huart3);
int sim800_setup(uint8_t debug_on, UART_HandleTypeDef huart2, UART_HandleTypeDef huart3);
int sim800_send_sms(char str_to_send[], uint8_t debug_on, UART_HandleTypeDef huart2, UART_HandleTypeDef huart3, char mobileNumber[]);
int sim800_read_sms(char** str_to_read, uint8_t debug_on, UART_HandleTypeDef huart2, UART_HandleTypeDef huart3);
int sim800_delete_all_sms(uint8_t debug_on, UART_HandleTypeDef huart2, UART_HandleTypeDef huart3);
int sim800_originate_call(uint8_t debug_on, UART_HandleTypeDef huart2, UART_HandleTypeDef huart3, char mobileNumber[]);

#endif
