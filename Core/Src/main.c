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
#include "sim800.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define BUFF_SIZE 700
#define SMS_SIZE 6
#define STD_SMS_SEND_LOCATION "LOCATE"
#define NO_GPS_LOCK "NO GPS LOCK"

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint8_t UART1_rxBuffer[BUFF_SIZE] = {0};
uint8_t UART3_rxBuffer[BUFF_SIZE] = {0};
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
char nmea_raw_data[BUFF_SIZE];
char loc_str[50] = "NO GPS LOCK";
int gps_lock = 0;
int gps_lock_th = 20;
int flag = 0;
uint8_t stay_awake_flag = 1;
uint8_t wkp_flag = 0;
uint8_t send_loc_flag = 0;
char mobileNumber[] = "+32456413932";

//accelero
//ref documentation
static const uint8_t MMA8452Q_ADDR = 0x1D << 1;
static const uint8_t CTRL_REG1_ADDR = 0x2A;
static const uint8_t WHO_AM_I_ADDR = 0x0D;
static const uint8_t SYS_MOD_ADDR = 0x0B;
static const uint8_t FF_MT_CFG_ADDR = 0x15;
static const uint8_t FF_MT_SRC_ADDR = 0x16;
static const uint8_t FF_MT_THS_ADDR = 0x17;

//float temp_c;


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 I2C_HandleTypeDef hi2c1;

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
static void MX_I2C1_Init(void);
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


//this function writes data into a register of the mma8452q accelerometer.
int mma8452q_write_into_register(uint8_t debug_on, uint8_t reg_addr, uint8_t data){
	HAL_StatusTypeDef ret;
	uint8_t i2c_buf[12];

	//first byte sent is register address
	i2c_buf[0] = reg_addr;
    //second byte sent is data to write
	i2c_buf[1] = data;

    // Write into the register of MMA8452Q
    ret = HAL_I2C_Master_Transmit(&hi2c1, MMA8452Q_ADDR, i2c_buf, 2, HAL_MAX_DELAY);
    if ( ret != HAL_OK ) {
		  strcpy((char*)i2c_buf, "Error Tx\r\n");
		  if (debug_on){HAL_UART_Transmit(&huart2,i2c_buf,strlen((char*)i2c_buf),HAL_MAX_DELAY);}
		  return 0;
    } else {

    	return 1;

    }
}

//this function reads data from a register of the mma8452q accelerometer.
int mma8452q_read_from_register(uint8_t debug_on, uint8_t reg_addr, uint8_t* data){
	HAL_StatusTypeDef ret;
	uint8_t i2c_buf[12];

    // Tell MMA8452Q from which register we want to read
	i2c_buf[0] = reg_addr;
	ret = HAL_I2C_Master_Transmit(&hi2c1, MMA8452Q_ADDR, i2c_buf, 1, HAL_MAX_DELAY);
	if ( ret != HAL_OK ) {
		strcpy((char*)i2c_buf, "Error Tx\r\n");
		if (debug_on){HAL_UART_Transmit(&huart2,i2c_buf,strlen((char*)i2c_buf),HAL_MAX_DELAY);}
		return 0;
	} else {

	  // read from the register
	  ret = HAL_I2C_Master_Receive(&hi2c1, MMA8452Q_ADDR, i2c_buf, 1, HAL_MAX_DELAY);
	  if ( ret != HAL_OK ) {
		  strcpy((char*)i2c_buf, "Error Rx\r\n");
		  if (debug_on){HAL_UART_Transmit(&huart2,i2c_buf,strlen((char*)i2c_buf),HAL_MAX_DELAY);}
		  return 0;
	  } else {
		  *data = i2c_buf[0];
		  return 1;
	  }
	}
}

//this function checks the system mode of the mma8452q accelerometer. Returns 0 if standby (or error) and 1 if active.
int mma8452q_whoami(uint8_t debug_on){

	uint8_t sysmod;
	uint8_t default_id = 0x2A;

	HAL_I2C_Mem_Read(&hi2c1, MMA8452Q_ADDR, WHO_AM_I_ADDR, 1, &sysmod, 1, HAL_MAX_DELAY);

	return (memcmp(&sysmod, &default_id, 1) == 0);

}

//this function checks the system mode of the mma8452q accelerometer. Returns 0 if standby (or error) and 1 if active.
int mma8452q_read_sysmod(uint8_t debug_on){

	uint8_t sysmod;

	//mma8452q_write_into_register(1, SYS_MOD_ADDR, sysmod);

	if(!mma8452q_read_from_register(1, CTRL_REG1_ADDR, &sysmod)){
		return 0;
	}

	return (sysmod & 0x01);

}

//this function writes the system mode of the mma8452q accelerometer. 0 for standby and 1 for active
int mma8452q_write_sysmod(uint8_t debug_on, uint8_t sysmod_cmd){

	uint8_t sysmod;

	mma8452q_read_from_register(1, CTRL_REG1_ADDR, &sysmod);

	sysmod_cmd = sysmod & ~(0x01);

	if(!mma8452q_write_into_register(1, CTRL_REG1_ADDR, sysmod)){
		return 0;
	}

	return 1;

}


void mma8452q_standby(uint8_t debug_on){

	uint8_t sysmod;
	uint8_t buf[256];

	if(debug_on){
		strcpy((char*)buf, "Request to go standby.\r\n");
		HAL_UART_Transmit(&huart2, buf, strlen((char*)buf), HAL_MAX_DELAY);

		HAL_Delay(500);
	}

	//read the active bit
	HAL_I2C_Mem_Read(&hi2c1, MMA8452Q_ADDR, CTRL_REG1_ADDR, 1, &sysmod, 1, HAL_MAX_DELAY);

	if (debug_on){
		if (!(sysmod & 0x01)){
			strcpy((char*)buf, "I was standby already.\r\n");
			HAL_UART_Transmit(&huart2, buf, strlen((char*)buf), HAL_MAX_DELAY);
		}
		else{
			strcpy((char*)buf, "I was active.\r\n");
			HAL_UART_Transmit(&huart2, buf, strlen((char*)buf), HAL_MAX_DELAY);
		}

		HAL_Delay(500);
	}

	//set active bit to 0 (standby mode)
	sysmod &= ~(0x01);

	//write the active bit into mm4852q
	HAL_I2C_Mem_Write(&hi2c1, MMA8452Q_ADDR, CTRL_REG1_ADDR, 1, &sysmod, 1, HAL_MAX_DELAY);

	if (debug_on){
		if (!(sysmod & 0x01)){
			strcpy((char*)buf, "Going standby.\r\n");
			HAL_UART_Transmit(&huart2, buf, strlen((char*)buf), HAL_MAX_DELAY);
		}
	}

}

void mma8452q_active(uint8_t debug_on){

	uint8_t sysmod;
	uint8_t buf[256];

	if(debug_on){
		strcpy((char*)buf, "Request to go active.\r\n");
		HAL_UART_Transmit(&huart2, buf, strlen((char*)buf), HAL_MAX_DELAY);

		HAL_Delay(500);
	}

	//read the active bit
	HAL_I2C_Mem_Read(&hi2c1, MMA8452Q_ADDR, CTRL_REG1_ADDR, 1, &sysmod, 1, HAL_MAX_DELAY);

	if (debug_on){
		if ((sysmod & 0x01)){
			strcpy((char*)buf, "I was active already.\r\n");
			HAL_UART_Transmit(&huart2, buf, strlen((char*)buf), HAL_MAX_DELAY);
		}
		else{
			strcpy((char*)buf, "I was standby.\r\n");
			HAL_UART_Transmit(&huart2, buf, strlen((char*)buf), HAL_MAX_DELAY);
		}

		HAL_Delay(500);
	}

	//set active bit to 1 (active mode)
	sysmod |= (0x01);

	//write the active bit into mm4852q
	HAL_I2C_Mem_Write(&hi2c1, MMA8452Q_ADDR, CTRL_REG1_ADDR, 1, &sysmod, 1, HAL_MAX_DELAY);

	if (debug_on){
		if ((sysmod & 0x01)){
			strcpy((char*)buf, "Going active.\r\n");
			HAL_UART_Transmit(&huart2, buf, strlen((char*)buf), HAL_MAX_DELAY);
		}
	}
}

//this function reads the ELE & OAE bits from the configuration register and returns true if both of them are 1
int mma8452q_configure(uint8_t debug_on){

	uint8_t ffmtcfg = 0xF8;

	mma8452q_write_into_register(1, FF_MT_CFG_ADDR, ffmtcfg);

	mma8452q_read_from_register(1, FF_MT_CFG_ADDR, &ffmtcfg);

	//check that accelero is configured for motion detection
	if (ffmtcfg == 0xF8){
		return 1;
	}
	else{
		return 0;
	}
}

//this function will check various conditions
uint8_t watch_conditions(){

	//if we just woke up
	if (wkp_flag == 1){

		//reseting the huart3 >> note 40
		HAL_Delay(3000);
		MX_USART3_UART_Init();

		//read sms
		char* sms;
		sim800_read_sms(&sms, 1, huart2, huart3);
		//if sms content corresponds to a location request
		if(strstr(sms,STD_SMS_SEND_LOCATION)){
			//set send_loc_flag
			send_loc_flag = 1;
		}
		//reset flag
		wkp_flag = 0;
	}
	//if we need to stay awake
	else if (stay_awake_flag == 1){

		uint8_t buf[12];

		strcpy((char*)buf, "WhoAmI?\r\n");
		HAL_UART_Transmit(&huart2, buf, strlen((char*)buf), HAL_MAX_DELAY);

		if(mma8452q_whoami(1)){
			strcpy((char*)buf, "Me!\r\n");
		}
		else{
			strcpy((char*)buf, "NoOne\r\n");
		}

		HAL_UART_Transmit(&huart2, buf, strlen((char*)buf), HAL_MAX_DELAY);

		HAL_Delay(500);

		/*
		strcpy((char*)buf, "Standby?\r\n");
		HAL_UART_Transmit(&huart2, buf, strlen((char*)buf), HAL_MAX_DELAY);

		if(!mma8452q_read_sysmod(1)){
			strcpy((char*)buf, "Yes\r\n");
		}
		else{
			strcpy((char*)buf, "No\r\n");
		}

		HAL_UART_Transmit(&huart2, buf, strlen((char*)buf), HAL_MAX_DELAY);

		HAL_Delay(500);



		strcpy((char*)buf, "Active!\r\n");
		HAL_UART_Transmit(&huart2, buf, strlen((char*)buf), HAL_MAX_DELAY);

		*/


		/*

		strcpy((char*)buf, "Active?\r\n");
		HAL_UART_Transmit(&huart2, buf, strlen((char*)buf), HAL_MAX_DELAY);

		if(mma8452q_read_sysmod(1)){
			strcpy((char*)buf, "Yes\r\n");
		}
		else{
			strcpy((char*)buf, "No\r\n");
		}

		HAL_UART_Transmit(&huart2, buf, strlen((char*)buf), HAL_MAX_DELAY);

		HAL_Delay(500);


		strcpy((char*)buf, "Standby!\r\n");
		HAL_UART_Transmit(&huart2, buf, strlen((char*)buf), HAL_MAX_DELAY);

		*/
		mma8452q_standby(1);

		HAL_Delay(500);

		mma8452q_active(1);

		HAL_Delay(500);

	}
	//if we have a location request
	else if (send_loc_flag == 1){
		//check that we have a gps lock
		if (gps_lock >= gps_lock_th){
			//send sms back with our location
			sim800_send_sms(loc_str, 1, huart2, huart3, mobileNumber);
			//reseting the huart3 >> note 40
			HAL_Delay(3000);
			MX_USART3_UART_Init();
			//switch led off
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
			//switch gps off
			//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
			//reset flags
			send_loc_flag = 0;
		}
	}
	//if we have no purpose to stay awake (no flag is on)
	else {
		//delete all sms
		sim800_delete_all_sms(1, huart2, huart3);
		//go low power mode
	    //NOTE : code for low power mode testing ...
	    char sentence[50] = "";
	    sprintf(sentence,"Entering stop mode...\r\n");
	    HAL_UART_Transmit(&huart2, (uint8_t*) sentence, strlen(sentence), 100);

	    //suspend the DMA (avoid wake up from UART)
	    HAL_UART_DMAPause(&huart1);

	    HAL_SuspendTick();
	    //HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
	    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
	}
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
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  //setup gsm module
  sim800_setup(1, huart2, huart3);

  //switch GPS module on
  //init DMA
  HAL_UART_Receive_DMA (&huart1, (uint8_t*)UART1_rxBuffer, 700);
  //turn gps module on
  //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	//TODO : write lines below with strcpy and (char*)
	//copy buffer content into char array
	int i = 0;
    for (i=0;i<BUFF_SIZE;i++){
        nmea_raw_data[i] = (char) UART1_rxBuffer[i];
    }

    //extract location under "DDD,DDD" format
    m8n_read_location(nmea_raw_data, &loc_str, huart2, gps_lock);

    watch_conditions();

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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00303D5B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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

  	//NOTE : code for low power mode testing ...
  	//HAL_ResumeTick();

    //char sentence[50] = "";
    //sprintf(sentence,"Wake up from sleep mode by UART...\r\n");
    //HAL_UART_Transmit(&huart2, (uint8_t*) sentence, strlen(sentence), 100);
}


void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{

	if(GPIO_Pin == GPIO_PIN_13) {

		//switch led on
		//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
		//Wait 100 ms
		//HAL_Delay(100);
		//switch led off
		//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
		//turn gps module on
		//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);

		//set flag on
		//flag = 1;


  } else {
      __NOP();
  }
}

void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == GPIO_PIN_12) {

	  	//NOTE : code for low power mode testing ...
	  	SystemClock_Config ();
	  	HAL_ResumeTick();
	  	//turn DMA on again
	  	HAL_UART_DMAResume(&huart1);

	    char sentence[50] = "";
	    sprintf(sentence,"Wake up from stop mode by EXTI...\r\n");
	    HAL_UART_Transmit(&huart2, (uint8_t*) sentence, strlen(sentence), 100);

		//switch led on
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
		//turn gps module on ==> should be done before waking the stm up
		//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
		//set flag on
		wkp_flag = 1;


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
