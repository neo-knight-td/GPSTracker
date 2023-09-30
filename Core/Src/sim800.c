/*
 * sim800.c
 *
 *  Created on: Sep 1, 2023
 *      Author: thomas
 */

#ifndef SRC_SIM800_C_
#define SRC_SIM800_C_

#include "sim800.h"

//this function checks that SIM800 GSM module responds OK to the AT command "AT" after a maximum of 10 attempts.
int sim800_AT_OK(uint8_t debug_on, UART_HandleTypeDef huart2, UART_HandleTypeDef huart3){

	//char ATcommand[80];
	char ATcommand[] = "AT\r\n";
	uint8_t buffer[30] = {0};
	uint8_t ATisOK = 0;
	uint8_t count = 0;
	uint8_t timeout = 2;

	while(!ATisOK){

		if (debug_on){HAL_UART_Transmit(&huart2,(uint8_t *)ATcommand,strlen(ATcommand),1000);}
		HAL_UART_Transmit(&huart3,(uint8_t *)ATcommand,strlen(ATcommand),1000);
		HAL_UART_Receive (&huart3, buffer, 30, 100);
		HAL_Delay(1000);

		if(strstr((char *)buffer,"OK")){
			char ok_sim800[] = "SIM800 : OK\r\n";
			if (debug_on){HAL_UART_Transmit(&huart2,(uint8_t *)ok_sim800,strlen(ok_sim800),1000);}
			ATisOK = 1;
			return 1;
		}

		HAL_Delay(1000);
		memset(buffer,0,sizeof(buffer));

		count++;
		if (count >= timeout){
			break;
		}
	}

	return 0;

}

int sim800_setup(uint8_t debug_on, UART_HandleTypeDef huart2, UART_HandleTypeDef huart3){

	char ATcommand[80];
	uint8_t buffer[30] = {0};

	if (!sim800_AT_OK(1, huart2, huart3)){
		return -1;
	}

	//going text mode
	sprintf(ATcommand,"AT+CMGF=1\r\n");
	if (debug_on){HAL_UART_Transmit(&huart2,(uint8_t *)ATcommand,strlen(ATcommand),1000);}
	HAL_UART_Transmit(&huart3,(uint8_t *)ATcommand,strlen(ATcommand),1000);
	HAL_UART_Receive (&huart3, buffer, 256, 100);
	HAL_Delay(10);
	memset(buffer,0,sizeof(buffer));

	//save on sim card only
	sprintf(ATcommand,"AT+CPMS=\"SM\",\"SM\",\"SM\"\r\n");
	if (debug_on){HAL_UART_Transmit(&huart2,(uint8_t *)ATcommand,strlen(ATcommand),1000);}
	HAL_UART_Transmit(&huart3,(uint8_t *)ATcommand,strlen(ATcommand),1000);
	HAL_UART_Receive (&huart3, buffer, 256, 100);
	HAL_Delay(10);
	memset(buffer,0,sizeof(buffer));

	/*
	sprintf(ATcommand,"AT+CNMI=1,2,0,0,0\r\n");
	if (debug_on){HAL_UART_Transmit(&huart2,(uint8_t *)ATcommand,strlen(ATcommand),1000);}
	HAL_UART_Transmit(&huart3,(uint8_t *)ATcommand,strlen(ATcommand),1000);
	HAL_Delay(10);
	*/
	return 1;
}

//
//IMPORTANT : remember to call MX_USART3_UART_Init() after calling this function.
int sim800_send_sms(char str_to_send[], uint8_t debug_on, UART_HandleTypeDef huart2, UART_HandleTypeDef huart3, char mobileNumber[]){

	char ATcommand[80];
	uint8_t buffer[30] = {0};

	if (!sim800_AT_OK(1, huart2, huart3)){
		return -1;
	}

	sprintf(ATcommand,"AT+CMGS=\"%s\"\r\n",mobileNumber);
	if (debug_on){HAL_UART_Transmit(&huart2,(uint8_t *)ATcommand,strlen(ATcommand),1000);}
	HAL_UART_Transmit(&huart3,(uint8_t *)ATcommand,strlen(ATcommand),1000);
	HAL_Delay(10);

	sprintf(ATcommand,"%s%c",str_to_send,26);//,0x1a);
	if (debug_on){HAL_UART_Transmit(&huart2,(uint8_t *)ATcommand,strlen(ATcommand),1000);}
	HAL_UART_Transmit(&huart3,(uint8_t *)ATcommand,strlen(ATcommand),1000);
	HAL_UART_Receive (&huart3, buffer, 30, 100);
	memset(buffer,0,sizeof(buffer));
	HAL_Delay(10);

	/*
	//reset the UART, as per note 40
	HAL_Delay(3000);
	MX_USART3_UART_Init();
	*/
	return 1;
}

//IMPORTANT : remember to call MX_USART3_UART_Init() before calling this function.
int sim800_read_sms(char** str_to_read, uint8_t debug_on, UART_HandleTypeDef huart2, UART_HandleTypeDef huart3){

	/*
	//reset the UART, as per note 40
	HAL_Delay(3000);
	MX_USART3_UART_Init();
	*/

	char ATcommand[80];
	uint8_t buffer[1024] = {0};

	if (!sim800_AT_OK(1, huart2, huart3)){
		return -1;
	}

	//list all sms
	sprintf(ATcommand,"AT+CMGL=\"ALL\"\r\n");//REC UNREAD
	if (debug_on){HAL_UART_Transmit(&huart2,(uint8_t *)ATcommand,strlen(ATcommand),1000);}
	HAL_UART_Transmit(&huart3,(uint8_t *)ATcommand,strlen(ATcommand),100);
	HAL_UART_Receive (&huart3, buffer, 1024, 5000);
	if (debug_on){HAL_UART_Transmit(&huart2, buffer, 1024, 1000);}

	HAL_Delay(1000);

	//allocate memory to the pointer
	*str_to_read = malloc((6+1)*sizeof(char));
	//copy start of sms from buffer into the pointer
	strcpy(*str_to_read,strstr((char*)buffer, "\"\r\n"));
	//reset buffer
	memset(buffer,0,sizeof(buffer));
    //NOTE : https://koor.fr/C/cstring/memmove.wp
	//cut start of sms (remove the \"\r\n" characters)
    memmove(*str_to_read,(*str_to_read)+3,6);

    if (debug_on){HAL_UART_Transmit(&huart2, (uint8_t*) *str_to_read, 6, 1000);}

	return 1;
}

int sim800_delete_all_sms(uint8_t debug_on, UART_HandleTypeDef huart2, UART_HandleTypeDef huart3){

	char ATcommand[80];
	uint8_t buffer[30] = {0};

	if (!sim800_AT_OK(1, huart2, huart3)){
		return -1;
	}

	//list all sms
	sprintf(ATcommand,"AT+CMGDA=\"DEL ALL\"\r\n");
	if (debug_on){HAL_UART_Transmit(&huart2,(uint8_t *)ATcommand,strlen(ATcommand),1000);}
	HAL_UART_Transmit(&huart3,(uint8_t *)ATcommand,strlen(ATcommand),1000);
	//sim800 does not always answer to the command (deletes memory with no ack)

	//TODO : use interrupts & avoid waiting for 25 seconds here
	HAL_UART_Receive (&huart3, buffer, 30, 25000);
	//use interrupts to receives this command
	//HAL_UART_Receive_IT(&huart3, buffer, 30);
	if (debug_on){HAL_UART_Transmit(&huart2, buffer, 30, 1000);}

	HAL_Delay(1000);
	memset(buffer,0,sizeof(buffer));

	return 1;
}

int sim800_originate_call(uint8_t debug_on, UART_HandleTypeDef huart2, UART_HandleTypeDef huart3, char mobileNumber[]){

	char ATcommand[80];

	if (!sim800_AT_OK(1, huart2, huart3)){
		return -1;
	}

	sprintf(ATcommand,"ATD%s;\r\n",mobileNumber);
	if (debug_on){HAL_UART_Transmit(&huart2,(uint8_t *)ATcommand,strlen(ATcommand),1000);}
	HAL_UART_Transmit(&huart3,(uint8_t *)ATcommand,strlen(ATcommand),1000);
	HAL_Delay(20000);

	sprintf(ATcommand,"ATH\r\n");
	if (debug_on){HAL_UART_Transmit(&huart2,(uint8_t *)ATcommand,strlen(ATcommand),1000);}
	HAL_UART_Transmit(&huart3,(uint8_t *)ATcommand,strlen(ATcommand),1000);
	HAL_Delay(10);

	return 1;
}

#endif /* SRC_SIM800_C_ */

