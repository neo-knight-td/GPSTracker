/*
 * mma8452q.h
 *
 *  Created on: Sep 2, 2023
 *      Author: thomas
 */

#ifndef INC_MMA8452Q_H_
#define INC_MMA8452Q_H_

#include "string.h"
#include "stdlib.h"
#include "stdio.h"
#include "stm32g0xx_hal.h"

//ref documentation
static const uint8_t MMA8452Q_ADDR = 0x1D << 1;
static const uint8_t CTRL_REG1_ADDR = 0x2A;
static const uint8_t WHO_AM_I_ADDR = 0x0D;
//static const uint8_t SYS_MOD_ADDR = 0x0B;
static const uint8_t FF_MT_CFG_ADDR = 0x15;
static const uint8_t FF_MT_SRC_ADDR = 0x16;
static const uint8_t FF_MT_THS_ADDR = 0x17;
static const uint8_t FF_MT_COUNT_ADDR = 0x17;

int mma8452q_whoami(uint8_t debug_on, UART_HandleTypeDef *phuart2, I2C_HandleTypeDef *phi2c1);
void mma8452q_standby(uint8_t debug_on, UART_HandleTypeDef *phuart2, I2C_HandleTypeDef *phi2c1);
void mma8452q_active(uint8_t debug_on, UART_HandleTypeDef *phuart2, I2C_HandleTypeDef *phi2c1);
void mma8452q_setup(uint8_t debug_on, UART_HandleTypeDef *phuart2, I2C_HandleTypeDef *phi2c1);
int mma8452q_motion_detection(uint8_t debug_on, UART_HandleTypeDef *phuart2, I2C_HandleTypeDef *phi2c1);

#endif /* INC_MMA8452Q_H_ */
