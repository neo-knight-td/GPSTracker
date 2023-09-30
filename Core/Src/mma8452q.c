/*
 * mma8452q.c
 *
 *  Created on: Sep 2, 2023
 *      Author: thomas
 */

#ifndef SRC_MMA8452Q_C_
#define SRC_MMA8452Q_C_

#include "mma8452q.h"

//this function checks that the accelero is online
int mma8452q_whoami(uint8_t debug_on, UART_HandleTypeDef *phuart2, I2C_HandleTypeDef *phi2c1){

	uint8_t id;
	uint8_t default_id = 0x2A;

	uint8_t buf[256];

	if(debug_on){
		strcpy((char*)buf, "Who am I ?\r\n");
		HAL_UART_Transmit(phuart2, buf, strlen((char*)buf), HAL_MAX_DELAY);

		HAL_Delay(500);
	}

	HAL_I2C_Mem_Read(phi2c1, MMA8452Q_ADDR, WHO_AM_I_ADDR, 1, &id, 1, HAL_MAX_DELAY);

	uint8_t comp = (memcmp(&id, &default_id, 1) == 0);

	if (debug_on){
		if (comp){
			strcpy((char*)buf, "MMA8452Q is online !\r\n");
			HAL_UART_Transmit(phuart2, buf, strlen((char*)buf), HAL_MAX_DELAY);
		}

		else{
			strcpy((char*)buf, "ERROR : MMA8452Q is offline.\r\n");
			HAL_UART_Transmit(phuart2, buf, strlen((char*)buf), HAL_MAX_DELAY);
		}

		HAL_Delay(500);
	}

	return comp;

}

//sets the accelero active bit to 0
void mma8452q_standby(uint8_t debug_on, UART_HandleTypeDef *phuart2, I2C_HandleTypeDef *phi2c1){

	uint8_t sysmod;
	uint8_t buf[256];

	if(debug_on){
		strcpy((char*)buf, "Request to go standby.\r\n");
		HAL_UART_Transmit(phuart2, buf, strlen((char*)buf), HAL_MAX_DELAY);

		HAL_Delay(500);
	}

	//read the active bit
	HAL_I2C_Mem_Read(phi2c1, MMA8452Q_ADDR, CTRL_REG1_ADDR, 1, &sysmod, 1, HAL_MAX_DELAY);

	if (debug_on){
		if (!(sysmod & 0x01)){
			strcpy((char*)buf, "I was standby already.\r\n");
			HAL_UART_Transmit(phuart2, buf, strlen((char*)buf), HAL_MAX_DELAY);
		}
		else{
			strcpy((char*)buf, "I was active.\r\n");
			HAL_UART_Transmit(phuart2, buf, strlen((char*)buf), HAL_MAX_DELAY);
		}

		HAL_Delay(500);
	}

	//set active bit to 0 (standby mode)
	sysmod &= ~(0x01);

	//write the active bit into mm4852q
	HAL_I2C_Mem_Write(phi2c1, MMA8452Q_ADDR, CTRL_REG1_ADDR, 1, &sysmod, 1, HAL_MAX_DELAY);

	if (debug_on){
		if (!(sysmod & 0x01)){
			strcpy((char*)buf, "Going standby.\r\n");
			HAL_UART_Transmit(phuart2, buf, strlen((char*)buf), HAL_MAX_DELAY);
		}
	}

}

//sets the accelero active bit to 1
void mma8452q_active(uint8_t debug_on, UART_HandleTypeDef *phuart2, I2C_HandleTypeDef *phi2c1){

	uint8_t sysmod;
	uint8_t buf[256];

	if(debug_on){
		strcpy((char*)buf, "Request to go active.\r\n");
		HAL_UART_Transmit(phuart2, buf, strlen((char*)buf), HAL_MAX_DELAY);

		HAL_Delay(500);
	}

	//read the active bit
	HAL_I2C_Mem_Read(phi2c1, MMA8452Q_ADDR, CTRL_REG1_ADDR, 1, &sysmod, 1, HAL_MAX_DELAY);

	if (debug_on){
		if ((sysmod & 0x01)){
			strcpy((char*)buf, "I was active already.\r\n");
			HAL_UART_Transmit(phuart2, buf, strlen((char*)buf), HAL_MAX_DELAY);
		}
		else{
			strcpy((char*)buf, "I was standby.\r\n");
			HAL_UART_Transmit(phuart2, buf, strlen((char*)buf), HAL_MAX_DELAY);
		}

		HAL_Delay(500);
	}

	//set active bit to 1 (active mode)
	sysmod |= (0x01);

	//write the active bit into mm4852q
	HAL_I2C_Mem_Write(phi2c1, MMA8452Q_ADDR, CTRL_REG1_ADDR, 1, &sysmod, 1, HAL_MAX_DELAY);

	if (debug_on){
		if ((sysmod & 0x01)){
			strcpy((char*)buf, "Going active.\r\n");
			HAL_UART_Transmit(phuart2, buf, strlen((char*)buf), HAL_MAX_DELAY);
		}
	}
}

//this function reads the ELE & OAE bits from the configuration register and returns true if both of them are 1
void mma8452q_setup(uint8_t debug_on, UART_HandleTypeDef *phuart2, I2C_HandleTypeDef *phi2c1){

	uint8_t desired_freefall_motion_config = 0xF8;

	uint8_t buf[256];

	//make sure the accelerometer is up
	//TODO : trigger error if not !
	mma8452q_whoami(debug_on, phuart2, phi2c1);

	//going standby in order to be able to modify the configuration register
	mma8452q_standby(debug_on, phuart2, phi2c1);

	if(debug_on){
		strcpy((char*)buf, "Setting up MMA8452Q...\r\n");
		HAL_UART_Transmit(phuart2, buf, strlen((char*)buf), HAL_MAX_DELAY);

		HAL_Delay(500);
	}

	//write desired config
	HAL_I2C_Mem_Write(phi2c1, MMA8452Q_ADDR, FF_MT_CFG_ADDR, 1, &desired_freefall_motion_config, 1, HAL_MAX_DELAY);

	//set desired threshold acceleration to 0.5g (DBCNTM = 0 for the moment)
	uint8_t desired_freefall_motion_threshold = 0x0c;

	//write desired acceleration threshold
	HAL_I2C_Mem_Write(phi2c1, MMA8452Q_ADDR, FF_MT_THS_ADDR, 1, &desired_freefall_motion_threshold, 1, HAL_MAX_DELAY);

	//set desired debounce counter to 14
	uint8_t desired_freefall_motion_count = 0x0d;

	//write desired debounce counter
	HAL_I2C_Mem_Write(phi2c1, MMA8452Q_ADDR, FF_MT_COUNT_ADDR, 1, &desired_freefall_motion_count, 1, HAL_MAX_DELAY);

	//putting accelero active
	mma8452q_active(1,phuart2, phi2c1);

	/*
	//read it back, for control
	HAL_I2C_Mem_Read(phi2c1, MMA8452Q_ADDR, FF_MT_CFG_ADDR, 1, &freefall_motion_config, 1, HAL_MAX_DELAY);

	//check that current config is the one desired
	uint8_t comp = (memcmp(&freefall_motion_config, &freefall_motion_config,1) == 0);

	if (debug_on){
		if (comp){
			strcpy((char*)buf, "MMA8452Q is all set !\r\n");
			HAL_UART_Transmit(phuart2, buf, strlen((char*)buf), HAL_MAX_DELAY);
		}

		else{
			strcpy((char*)buf, "Error when setting up MMA8452Q.\r\n");
			HAL_UART_Transmit(phuart2, buf, strlen((char*)buf), HAL_MAX_DELAY);
		}

		HAL_Delay(500);
	}

	return(comp);
	*/
}

int mma8452q_motion_detection(uint8_t debug_on, UART_HandleTypeDef *phuart2, I2C_HandleTypeDef *phi2c1){

	uint8_t freefall_motion_source;
	uint8_t buf[256];

	//read content of freefall motion source register
	HAL_I2C_Mem_Read(phi2c1, MMA8452Q_ADDR, FF_MT_SRC_ADDR, 1, &freefall_motion_source, 1, HAL_MAX_DELAY);

	//if event active flag (EA) is 1
	if (freefall_motion_source & 0x80){
		if(debug_on){

			strcpy((char*)buf, "Motion detected !!! \r\n");
			HAL_UART_Transmit(phuart2, buf, strlen((char*)buf), HAL_MAX_DELAY);

			HAL_Delay(500);
		}

		return 1;
	}
	else{
		return 0;
	}

}

#endif /* SRC_MMA8452Q_C_ */
