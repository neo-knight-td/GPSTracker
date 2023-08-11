/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdlib.h"
#include "stdio.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define BUFF_SIZE 700

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint8_t UART1_rxBuffer[BUFF_SIZE] = {0};
uint8_t UART3_rxBuffer[BUFF_SIZE] = {0};
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
char nmea_raw_data[BUFF_SIZE];
char loc_str[50] = " ";
int gps_lock = 0;
int gps_lock_th = 20;
int flag = 0;
char mobileNumber[] = "+32456413932";

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

float convert_DDmm_to_DDD(float DDmm, char *sign){

    //convert to DDD format
    int DD = ((int)DDmm)/100;
    float mm = DDmm - DD*100;
    float DDD = DD + mm/60;

    //add negative sign if south or west
    if (strcmp(sign,"S") == 0 || strcmp(sign,"W") == 0){
        DDD = -DDD;
    }

    return DDD;
}

void m8n_read_location(char nmea_raw_data[], char *loc_str[]){

    const __uint8_t GLL_MSG_LEN = 47;

    float latitude, longitude;
    char lat_sign, long_sign;
    char *start_GLL_msg;
    int len_GLL_msg;

    start_GLL_msg = strstr(nmea_raw_data, "GLL");
    len_GLL_msg = strlen(start_GLL_msg);

    *loc_str = "";

    //message is not valid as does not have minimum length
    if (len_GLL_msg < GLL_MSG_LEN){
        return;
    }

    //retreive location parameters from nmea sentence
    sscanf(start_GLL_msg, "GLL,%f,%c,%f,%c,",&latitude, &lat_sign, &longitude, &long_sign);

    //if one of the lat or long is equal to 0, reset lock value and return
    if (latitude == 0 || longitude == 0){
        gps_lock = 0;
        //*loc_str = "";

        char sentence[50] = "";
        sprintf(sentence,"%Inconsistent, lock = %i\r\n",gps_lock);
        HAL_UART_Transmit(&huart2, (uint8_t*) sentence, strlen(sentence), 100);
        //HAL_UART_Transmit(&huart2, (uint8_t*) *loc_str, strlen(*loc_str), 100);

        return;
    }

    //if one of the lat or long is nan (https://stackoverflow.com/a/570694), reset lock value and return
    else if (latitude != latitude || longitude != longitude){
        gps_lock = 0;
        //*loc_str = "";

        char sentence[50] = "";
        sprintf(sentence,"%NaN, lock = %i\r\n",gps_lock);
        HAL_UART_Transmit(&huart2, (uint8_t*) sentence, strlen(sentence), 100);
        //HAL_UART_Transmit(&huart2, (uint8_t*) *loc_str, strlen(*loc_str), 100);

        return;
    }

	//copy location parameters into loc_str under desired format
	sprintf(loc_str, "%f,%f\r\n",convert_DDmm_to_DDD(latitude,&lat_sign),convert_DDmm_to_DDD(longitude,&long_sign));

    //if number of gps lock is still below threshold, increment lock value and return
    if (gps_lock < gps_lock_th){
        gps_lock++;

        char sentence[50] = "";
        sprintf(sentence,"%OK, below threshold, lock = %i\r\n",gps_lock);
        HAL_UART_Transmit(&huart2, (uint8_t*) sentence, strlen(sentence), 100);
        //HAL_UART_Transmit(&huart2, (uint8_t*) *loc_str, strlen(*loc_str), 100);

        return;
    }

    //if nb of consecutive locks is above threshold
    else if (gps_lock >= gps_lock_th){
    	gps_lock = gps_lock_th;

        char sentence[50] = "";
        sprintf(sentence,"All OK, lock = %i\r\n",gps_lock);
        HAL_UART_Transmit(&huart2, (uint8_t*) sentence, strlen(sentence), 100);
        //HAL_UART_Transmit(&huart2, (uint8_t*) *loc_str, strlen(*loc_str), 100);

        return;
    }

}

//this function checks that SIM800 GSM module responds OK to the AT command "AT" after a maximum of 10 attempts.
int sim800_AT_OK(uint8_t debug_on){

	//char ATcommand[80];
	char ATcommand[] = "AT\r\n";
	uint8_t buffer[30] = {0};
	uint8_t ATisOK = 0;
	uint8_t count = 0;
	uint8_t timeout = 3;

	while(!ATisOK){

		if (debug_on){HAL_UART_Transmit(&huart2,(uint8_t *)ATcommand,strlen(ATcommand),1000);}
		HAL_UART_Transmit(&huart3,(uint8_t *)ATcommand,strlen(ATcommand),100);
		HAL_UART_Receive (&huart3, buffer, 30, 5000);
		HAL_Delay(10);

		if(strstr((char *)buffer,"OK")){
			ATisOK = 1;
		}

		HAL_Delay(10);
		memset(buffer,0,sizeof(buffer));

		count++;
		if (count >= timeout){
			return -1;
		}
	}

	return 1;
}


int sim800_setup(uint8_t debug_on){

	char ATcommand[80];
	uint8_t buffer[30] = {0};

	if (!sim800_AT_OK(1)){
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

int sim800_send_sms(char str_to_send[], uint8_t debug_on){

	char ATcommand[80];
	uint8_t buffer[30] = {0};

	if (!sim800_AT_OK(1)){
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

	return 1;
}

int sim800_read_sms(uint8_t debug_on){

	char ATcommand[80];
	uint8_t buffer[BUFF_SIZE] = {0};

	if (!sim800_AT_OK(1)){
		return -1;
	}

	//list all sms
	sprintf(ATcommand,"AT+CMGL=\"REC UNREAD\"\r\n");
	if (debug_on){HAL_UART_Transmit(&huart2,(uint8_t *)ATcommand,strlen(ATcommand),1000);}
	HAL_UART_Transmit(&huart3,(uint8_t *)ATcommand,strlen(ATcommand),1000);
	HAL_UART_Receive (&huart3, buffer, BUFF_SIZE, 1000);
	if (debug_on){HAL_UART_Transmit(&huart2, buffer, BUFF_SIZE, 1000);}
	HAL_Delay(10);
	memset(buffer,0,sizeof(buffer));

	return 1;
}

int sim800_delete_all_sms(uint8_t debug_on){

	char ATcommand[80];
	uint8_t buffer[30] = {0};

	if (!sim800_AT_OK(1)){
		return -1;
	}

	//list all sms
	sprintf(ATcommand,"AT+CMGDA=\"DEL ALL\"\r\n");
	if (debug_on){HAL_UART_Transmit(&huart2,(uint8_t *)ATcommand,strlen(ATcommand),1000);}
	HAL_UART_Transmit(&huart3,(uint8_t *)ATcommand,strlen(ATcommand),1000);
	HAL_UART_Receive (&huart3, buffer, 30, 1000);
	if (debug_on){HAL_UART_Transmit(&huart2, buffer, 30, 1000);}
	HAL_Delay(10);
	memset(buffer,0,sizeof(buffer));

	return 1;
}

int sim800_originate_call(uint8_t debug_on){

	char ATcommand[80];

	if (!sim800_AT_OK(1)){
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

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  //setup gsm module
  sim800_setup(1);

  HAL_UART_Receive_DMA (&huart1, (uint8_t*)UART1_rxBuffer, 700);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	//copy buffer content into char array
	int i = 0;
    for (i=0;i<BUFF_SIZE;i++){
        nmea_raw_data[i] = (char) UART1_rxBuffer[i];
    }

    //extract location under "DDD,DDD" format
    m8n_read_location(nmea_raw_data, &loc_str);

	//transmit formatted location to PC if gps is locked
	//HAL_UART_Transmit(&huart2, (uint8_t*) gps_lock, sizeof(gps_lock), 100);
	//HAL_UART_Transmit(&huart2, (uint8_t*) loc_str, sizeof(loc_str), 100);
	//HAL_UART_Transmit(&huart2, (uint8_t*) gps_lock, sizeof(gps_lock), 100);


	//check if flag is on (user button pressed)
	if (flag == 1 && gps_lock >= gps_lock_th){

		//read sms
		//sim800_read_sms(1);

		//send sms
		sim800_send_sms(loc_str, 1);

		//delete all sms
		//sim800_delete_all_sms(1);

		//give a call
		//originate_call(0);

		//reset flag
		flag = 0;
	}

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 38400;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_GREEN_Pin|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin : RING_Pin */
  GPIO_InitStruct.Pin = RING_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RING_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_GREEN_Pin */
  GPIO_InitStruct.Pin = LED_GREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LED_GREEN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	/*
	//if (huart == &huart3){
		char debug_str[] = "In call back DMA 3. Forwarding from SIM800 :\r\n";
	    HAL_UART_Transmit(&huart2, (uint8_t*) debug_str, strlen(debug_str), 100);
	    HAL_UART_Transmit(&huart2, UART3_rxBuffer, sizeof(debug_str), 1000);
	//}
	 * */

}


void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{

	if(GPIO_Pin == GPIO_PIN_13) {
	/*
		//switch led on
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
		//Wait 100 ms
		//HAL_Delay(100);
		//switch led off
		//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
		//turn gps module on
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);

		//set flag on
		flag = 1;
	*/

  } else {
      __NOP();
  }
}

void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == GPIO_PIN_12) {

		//switch led on
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
		//Wait 100 ms
		//HAL_Delay(100);
		//switch led off
		//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
		//turn gps module on
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);

		//set flag on
		flag = 1;


  } else {
      __NOP();
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
