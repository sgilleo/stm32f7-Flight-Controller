/*
 * Sbus.c
 *
 *  Created on: Aug 13, 2025
 *      Author: sergi
 */


#include "Sbus.h"

HAL_StatusTypeDef Sbus_Begin(UART_HandleTypeDef *huart, Sbus *receiver){
	HAL_StatusTypeDef status;

	HAL_UART_AbortReceive(huart); //Sbus is already sending before initialising, cancel reception to avoid HAL_ERROR

	status = HAL_UARTEx_ReceiveToIdle_DMA(huart, receiver->buffer, 25);
	if(status != HAL_OK) {
		return status;
	}

	return status;
}

int sbus_raw_to_us(uint16_t v) {
	float us = ((v - 192) * 1000.0f / 1600.0f + 1000.0f);
	if (us < 1000.0f) us = 1000.0f;
	if (us > 2000.0f) us = 2000.0f;
	return us;
}

void Sbus_decode(uint8_t *buffer, float *channels){

	uint16_t raw_channels[18];

	raw_channels[0]  = ((buffer[1]    |buffer[2]<<8)                 & 0x07FF);
	raw_channels[1]  = ((buffer[2]>>3 |buffer[3]<<5)                 & 0x07FF);
	raw_channels[2]  = ((buffer[3]>>6 |buffer[4]<<2 |buffer[5]<<10)  & 0x07FF);
	raw_channels[3]  = ((buffer[5]>>1 |buffer[6]<<7)                 & 0x07FF);
	raw_channels[4]  = ((buffer[6]>>4 |buffer[7]<<4)                 & 0x07FF);
	raw_channels[5]  = ((buffer[7]>>7 |buffer[8]<<1 |buffer[9]<<9)   & 0x07FF);
	raw_channels[6]  = ((buffer[9]>>2 |buffer[10]<<6)                & 0x07FF);
	raw_channels[7]  = ((buffer[10]>>5|buffer[11]<<3)                & 0x07FF);
	raw_channels[8]  = ((buffer[12]   |buffer[13]<<8)                & 0x07FF);
	raw_channels[9]  = ((buffer[13]>>3|buffer[14]<<5)                & 0x07FF);
	raw_channels[10] = ((buffer[14]>>6|buffer[15]<<2|buffer[16]<<10) & 0x07FF);
	raw_channels[11] = ((buffer[16]>>1|buffer[17]<<7)                & 0x07FF);
	raw_channels[12] = ((buffer[17]>>4|buffer[18]<<4)                & 0x07FF);
	raw_channels[13] = ((buffer[18]>>7|buffer[19]<<1|buffer[20]<<9)  & 0x07FF);
	raw_channels[14] = ((buffer[20]>>2|buffer[21]<<6)                & 0x07FF);
	raw_channels[15] = ((buffer[21]>>5|buffer[22]<<3)                & 0x07FF);

    if (buffer[23] & (1 << 0)) {
    	raw_channels[16] = 1;
    }
    else {
    	raw_channels[16] = 0;
    }

    if (buffer[23] & (1 << 1)) {
    	raw_channels[17] = 1;
    }
    else {
    	raw_channels[17] = 0;
    }

    for(int i = 0; i < 18; i++){
    	channels[i] = sbus_raw_to_us(raw_channels[i]);
    }
}



