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

void Sbus_decode(Sbus *receiver){

	uint16_t raw_channels[18];

	raw_channels[0]  = ((receiver->buffer[1]    |receiver->buffer[2]<<8)                           & 0x07FF);
	raw_channels[1]  = ((receiver->buffer[2]>>3 |receiver->buffer[3]<<5)                           & 0x07FF);
	raw_channels[2]  = ((receiver->buffer[3]>>6 |receiver->buffer[4]<<2 |receiver->buffer[5]<<10)  & 0x07FF);
	raw_channels[3]  = ((receiver->buffer[5]>>1 |receiver->buffer[6]<<7)                           & 0x07FF);
	raw_channels[4]  = ((receiver->buffer[6]>>4 |receiver->buffer[7]<<4)                           & 0x07FF);
	raw_channels[5]  = ((receiver->buffer[7]>>7 |receiver->buffer[8]<<1 |receiver->buffer[9]<<9)   & 0x07FF);
	raw_channels[6]  = ((receiver->buffer[9]>>2 |receiver->buffer[10]<<6)                          & 0x07FF);
	raw_channels[7]  = ((receiver->buffer[10]>>5|receiver->buffer[11]<<3)                          & 0x07FF);
	raw_channels[8]  = ((receiver->buffer[12]   |receiver->buffer[13]<<8)                          & 0x07FF);
	raw_channels[9]  = ((receiver->buffer[13]>>3|receiver->buffer[14]<<5)                          & 0x07FF);
	raw_channels[10] = ((receiver->buffer[14]>>6|receiver->buffer[15]<<2|receiver->buffer[16]<<10) & 0x07FF);
	raw_channels[11] = ((receiver->buffer[16]>>1|receiver->buffer[17]<<7)                          & 0x07FF);
	raw_channels[12] = ((receiver->buffer[17]>>4|receiver->buffer[18]<<4)                          & 0x07FF);
	raw_channels[13] = ((receiver->buffer[18]>>7|receiver->buffer[19]<<1|receiver->buffer[20]<<9)  & 0x07FF);
	raw_channels[14] = ((receiver->buffer[20]>>2|receiver->buffer[21]<<6)                          & 0x07FF);
	raw_channels[15] = ((receiver->buffer[21]>>5|receiver->buffer[22]<<3)                          & 0x07FF);

    if (receiver->buffer[23] & (1 << 0)) {
    	raw_channels[16] = 1;
    }
    else {
    	raw_channels[16] = 0;
    }

    if (receiver->buffer[23] & (1 << 1)) {
    	raw_channels[17] = 1;
    }
    else {
    	raw_channels[17] = 0;
    }

    // Failsafe
	receiver->failsafe_status = SBUS_SIGNAL_OK;
	if (receiver->buffer[23] & (1 << 2)) {
		receiver->failsafe_status = SBUS_SIGNAL_LOST;
	}

	if (receiver->buffer[23] & (1 << 3)) {
		receiver->failsafe_status = SBUS_SIGNAL_FAILSAFE;
	}

    for(int i = 0; i < 18; i++){
    	receiver->channels[i] = sbus_raw_to_us(raw_channels[i]);
    }
}



