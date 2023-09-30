/*
 * m8n.c
 *
 *  Created on: Sep 1, 2023
 *      Author: thomas
 */

#ifndef SRC_M8N_C_
#define SRC_M8N_C_

#include "m8n.h"

//returns a location under DDD format
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

uint8_t m8n_read_location(char nmea_raw_data[], char *loc_str[], UART_HandleTypeDef huart2, int *gps_lock_count){

    const __uint8_t GLL_MSG_LEN = 47;
    const int gps_lock_th = 20;

    float latitude = 0;
    float longitude = 0;
    char lat_sign, long_sign;
    char *start_GLL_msg;
    int len_GLL_msg;

    start_GLL_msg = strstr(nmea_raw_data, "GLL");
    len_GLL_msg = strlen(start_GLL_msg);

    //TODO : check to send **loc_str, like in function sim800_read_sms
    *loc_str = "NO GPS LOCK";

    //message is not valid as does not have minimum length
    if (len_GLL_msg < GLL_MSG_LEN){
        char sentence[50] = "";
        sprintf(sentence,"NMEA message not valid\r\n");
        //HAL_UART_Transmit(&huart2, (uint8_t*) sentence, strlen(sentence), 100);

        return 0;
    }

    //retreive location parameters from nmea sentence
    sscanf(start_GLL_msg, "GLL,%f,%c,%f,%c,",&latitude, &lat_sign, &longitude, &long_sign);

    //if one of the lat or long is equal to 0, reset lock count and return 0
    if (latitude == 0 || longitude == 0){
        *gps_lock_count = 0;
        //*loc_str = "";

        char sentence[50] = "";
        sprintf(sentence,"Inconsistent, lock = %i\r\n",*gps_lock_count);
        HAL_UART_Transmit(&huart2, (uint8_t*) sentence, strlen(sentence), 100);
        //HAL_UART_Transmit(&huart2, (uint8_t*) *loc_str, strlen(*loc_str), 100);

        return 0;
    }

    //if one of the lat or long is nan (https://stackoverflow.com/a/570694), reset lock value and return
    else if (latitude != latitude || longitude != longitude){
        *gps_lock_count = 0;
        //*loc_str = "";

        char sentence[50] = "";
        sprintf(sentence,"NaN, lock = %i\r\n",*gps_lock_count);
        HAL_UART_Transmit(&huart2, (uint8_t*) sentence, strlen(sentence), 100);
        //HAL_UART_Transmit(&huart2, (uint8_t*) *loc_str, strlen(*loc_str), 100);

        return 0;
    }

	//copy location parameters into loc_str under desired format
	sprintf(loc_str, "%f,%f\r\n",convert_DDmm_to_DDD(latitude,&lat_sign),convert_DDmm_to_DDD(longitude,&long_sign));

    //if number of gps lock is still below threshold, increment lock value and return
    if (*gps_lock_count < gps_lock_th){
    	//ref here on how to increment a var using a pointer
        ++(*gps_lock_count);

        char sentence[50] = "";
        sprintf(sentence,"OK, below threshold, lock = %i\r\n",*gps_lock_count);
        HAL_UART_Transmit(&huart2, (uint8_t*) sentence, strlen(sentence), 100);
        //HAL_UART_Transmit(&huart2, (uint8_t*) *loc_str, strlen(*loc_str), 100);

        return 0;
    }

    //if nb of consecutive locks is above threshold
    else if (*gps_lock_count >= gps_lock_th){
    	*gps_lock_count = gps_lock_th;

        char sentence[50] = "";
        sprintf(sentence,"All OK, lock = %i\r\n",*gps_lock_count);
        HAL_UART_Transmit(&huart2, (uint8_t*) sentence, strlen(sentence), 100);
        //HAL_UART_Transmit(&huart2, (uint8_t*) *loc_str, strlen(*loc_str), 100);

        return 1;
    }

}


#endif /* SRC_M8N_C_ */
