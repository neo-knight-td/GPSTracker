/*
 * m8n.h
 *
 *  Created on: Sep 1, 2023
 *      Author: thomas
 */

#ifndef INC_M8N_H_
#define INC_M8N_H_

#include "string.h"
#include "stdlib.h"
#include "stdio.h"
#include "stm32g0xx_hal.h"

void m8n_read_location(char nmea_raw_data[], char *loc_str[], UART_HandleTypeDef huart2, int gps_lock);

#endif /* INC_M8N_H_ */
