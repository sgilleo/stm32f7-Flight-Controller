/*
 * icm42688p.c
 *
 *  Created on: Aug 11, 2025
 *      Author: sergi
 */


#include "ICM42688P.h"
#include "main.h"

HAL_StatusTypeDef ICM42688_Begin(ICM42688 *device, SPI_HandleTypeDef *spiHandle){
	device->spiHandle = spiHandle;

	device->accel.x = 0.0f;
	device->accel.y = 0.0f;
	device->accel.z = 0.0f;

	device->gyro.x = 0.0f;
	device->gyro.y = 0.0f;
	device->gyro.z = 0.0f;

	device->ready = 0;
	device->dataRdy = 0;

	HAL_Delay(10);
	HAL_StatusTypeDef status;
	uint8_t data;

	status = ICM42688_Read(device, WHO_AM_I, &data);
	if(status != HAL_OK) return status;
	if(data != 0x47) return HAL_ERROR;

	data = (GYRO_FS_SEL_1000_DPS << 5) | GYRO_ODR_1_KHZ;
	status = ICM42688_Write(device, GYRO_CONFIG0, &data);
	if(status != HAL_OK) return status;
	device->gyro_fs = GYRO_FS_SEL_1000_DPS;
	device->gyro_odr = GYRO_ODR_1_KHZ;

	data = (ACCEL_FS_SEL_4_G << 5) | ACCEL_ODR_1_KHZ;
	status = ICM42688_Write(device, ACCEL_CONFIG0, &data);
	if(status != HAL_OK) return status;
	device->accel_fs = ACCEL_FS_SEL_4_G;
	device->accel_odr = ACCEL_ODR_1_KHZ;

	status = ICM42688_Set_Interrupts(device);
	if(status != HAL_OK) return status;

	status = ICM42688_Set_Filters(device);
	if(status != HAL_OK) return status;

	data = 0x0F; //0b00001111 Gyro & Acc in Low Noise Mode
	status = ICM42688_Write(device, PWR_MGMT0, &data);
	if(status != HAL_OK) return status;

	//status = ICM42688_Calibrate_Gyro(device);
	//if(status != HAL_OK) return status;

	//status = ICM42688_Calibrate_Accel(device);
	//if(status != HAL_OK) return status;

	device->ready = 1;

	return status;
}


HAL_StatusTypeDef ICM42688_Set_Gyro_FS(ICM42688 *device, GYRO_FS_SEL range){

	uint8_t data;

	HAL_StatusTypeDef status = ICM42688_Read(device, GYRO_CONFIG0, &data);
	if(status != HAL_OK) return status;

	data = (range << 5) | (data & 0x1F);

	status = ICM42688_Write(device, GYRO_CONFIG0, &data);
	if(status != HAL_OK) return status;

	device->gyro_fs = range;

	return status;
}

HAL_StatusTypeDef ICM42688_Set_Gyro_ODR(ICM42688 *device, GYRO_ODR rate){

	uint8_t data;

	HAL_StatusTypeDef status = ICM42688_Read(device, GYRO_CONFIG0, &data);
	if(status != HAL_OK) return status;

	data = (data & 0xF0) | rate;

	status = ICM42688_Write(device, GYRO_CONFIG0, &data);
	if(status != HAL_OK) return status;

	device->gyro_odr = rate;

	return status;
}


HAL_StatusTypeDef ICM42688_Set_Accel_FS(ICM42688 *device, ACCEL_FS_SEL range){

	uint8_t data;

	HAL_StatusTypeDef status = ICM42688_Read(device, ACCEL_CONFIG0, &data);
	if(status != HAL_OK) return status;

	data = (range << 5) | (data & 0x1F);

	status = ICM42688_Write(device, ACCEL_CONFIG0, &data);
	if(status != HAL_OK) return status;

	device->accel_fs = range;

	return status;
}

HAL_StatusTypeDef ICM42688_Set_Accel_ODR(ICM42688 *device, ACCEL_ODR rate){

	uint8_t data;

	HAL_StatusTypeDef status = ICM42688_Read(device, ACCEL_CONFIG0, &data);
	if(status != HAL_OK) return status;

	data = (data & 0xF0) | rate;

	status = ICM42688_Write(device, ACCEL_CONFIG0, &data);
	if(status != HAL_OK) return status;

	device->accel_odr = rate;

	return status;
}

HAL_StatusTypeDef ICM42688_Set_Filters(ICM42688 *device){
	//Antialiasing and Notch Filters enabled by default

	uint8_t data = (UI_FILT_ORD_1_ORD << 2); //Temperature filter: 4000Hz (Default), 1st order Gyro Low Pass Filter
	HAL_StatusTypeDef status = ICM42688_Write(device, GYRO_CONFIG1, &data);
	if(status != HAL_OK) return status;

	data = (UI_FILT_ORD_1_ORD << 3); //Accelerometer Low Pass Filter 1st order
	status = ICM42688_Write(device, ACCEL_CONFIG1, &data);
	if(status != HAL_OK) return status;

	data = (UI_FILT_BW_125_HZ << 4) | UI_FILT_BW_125_HZ; //Accelerometer BW (Left) & Gyro BW (Right)
	status = ICM42688_Write(device, GYRO_ACCEL_CONFIG0, &data);
	if(status != HAL_OK) return status;

	return status;
}

HAL_StatusTypeDef ICM42688_Set_Interrupts(ICM42688 *device){
	uint8_t data = 0x03; //0b00000011 INT1: Active High, Push pull, Pulsed mode
	HAL_StatusTypeDef status = ICM42688_Write(device, INT_CONFIG, &data);
	if(status != HAL_OK) return status;

	// need to clear bit 4 to allow proper INT1 and INT2 operation
	status = ICM42688_Read(device, INT_CONFIG1, &data);
	if(status != HAL_OK) return status;
	data &= ~0x10;
	status = ICM42688_Write(device, INT_CONFIG1, &data);
	if(status != HAL_OK) return status;

	// route UI data ready interrupt to INT1
	data = 0x08; //0b00001000: UI data ready interrupt routed to INT1
	status = ICM42688_Write(device, INT_SOURCE0, &data);
	if(status != HAL_OK) return status;

	return status;
}



HAL_StatusTypeDef ICM42688_Read_Accel(ICM42688 *device){

	uint8_t data[6];

	HAL_StatusTypeDef status = ICM42688_Read_Multiple(device, ACCEL_DATA_X1, data, 6);
	if(status != HAL_OK) return status;

	int16_t acc[3];

	acc[0] = (int16_t)((data[0] << 8) | data[1]);
	acc[1] = (int16_t)((data[2] << 8) | data[3]);
	acc[2] = (int16_t)((data[4] << 8) | data[5]);

	device->accel.x = (float) acc[1] / 8192.0f; //ACCEL_FS_SEL_4_G:
	device->accel.y = -(float) acc[0] / 8192.0f; //Axis Remapping
	device->accel.z = (float) acc[2] / 8192.0f;

	return status;

}



HAL_StatusTypeDef ICM42688_Read_Gyro(ICM42688 *device){

	uint8_t data[6];

	HAL_StatusTypeDef status = ICM42688_Read_Multiple(device, GYRO_DATA_X1, data, 6);
	if(status != HAL_OK) return status;

	int16_t gyro[3];

	gyro[0] = (int16_t)((data[0] << 8) | data[1]);
	gyro[1] = (int16_t)((data[2] << 8) | data[3]);
	gyro[2] = (int16_t)((data[4] << 8) | data[5]);

	device->gyro.x = (float) gyro[1] / 32.8f; //GYRO_FS_SEL_1000_DPS
	device->gyro.y = -(float) gyro[0] / 32.8f; //Axis Remapping
	device->gyro.z = (float) gyro[2] / 32.8f;

	//device->gyro.x *= DEG_TO_RAD;
	//device->gyro.y *= DEG_TO_RAD;
	//device->gyro.z *= DEG_TO_RAD;

	return status;
}


HAL_StatusTypeDef ICM42688_Read_Temp(ICM42688 *device){

	uint8_t data[2];

	HAL_StatusTypeDef status = ICM42688_Read_Multiple(device, TEMP_DATA1, data, 2);
	if(status != HAL_OK) return status;

	uint16_t tempRaw = (int16_t)((data[0] << 8) | data[1]);

	device->temp = (float) tempRaw / 132.48f + 25.0f;

	return status;
}

void ICM42688_Process_Buffer(ICM42688 *device){

	uint16_t tempRaw = (int16_t)((device->buffer[1] << 8) | device->buffer[2]);
	int16_t acc[3];
	int16_t gyro[3];

	device->temp = (float) tempRaw / 132.48f + 25.0f;

	acc[0] = (int16_t)((device->buffer[3] << 8) | device->buffer[4]);
	acc[1] = (int16_t)((device->buffer[5] << 8) | device->buffer[6]);
	acc[2] = (int16_t)((device->buffer[7] << 8) | device->buffer[8]);

	device->accel.x = (float) acc[1] / 8192.0f; //ACCEL_FS_SEL_4_G
	device->accel.y = -(float) acc[0] / 8192.0f; //Axis remapping
	device->accel.z = (float) acc[2] / 8192.0f;

	gyro[0] = (int16_t)((device->buffer[9] << 8) | device->buffer[10]);
	gyro[1] = (int16_t)((device->buffer[11] << 8) | device->buffer[12]);
	gyro[2] = (int16_t)((device->buffer[13] << 8) | device->buffer[14]);

	device->gyro.x = (float) gyro[1] / 32.8f; //GYRO_FS_SEL_1000_DPS
	device->gyro.y = -(float) gyro[0] / 32.8f; //Axis remapping
	device->gyro.z = (float) gyro[2] / 32.8f;

	device->gyro.x -= -3.13812327; //Calibration
	device->gyro.y -= 0.0449692011;
	device->gyro.z -= 0.46484378;

	//device->gyro.x *= DEG_TO_RAD;
	//device->gyro.y *= DEG_TO_RAD;
	//device->gyro.z *= DEG_TO_RAD;

}

HAL_StatusTypeDef ICM42688_Calibrate_Gyro(ICM42688 *device){
	Vec3 calibration = {0.0f, 0.0f, 0.0f};

	uint8_t data[6];

	for (int i = 0; i < 1000; i++) {
		HAL_StatusTypeDef status = ICM42688_Read_Multiple(device, GYRO_DATA_X1, data, 6);
		if(status != HAL_OK) return status;

		int16_t gyro[3];

		gyro[0] = (int16_t)((data[0] << 8) | data[1]);
		gyro[1] = (int16_t)((data[2] << 8) | data[3]);
		gyro[2] = (int16_t)((data[4] << 8) | data[5]);

		calibration.x += (float) gyro[1] / 32.8f; //GYRO_FS_SEL_1000_DPS
		calibration.y += -(float) gyro[0] / 32.8f; //Axis Remapping
		calibration.z += (float) gyro[2] / 32.8f;
	}

	calibration.x /= 1000;
	calibration.y /= 1000;
	calibration.z /= 1000;

	__NOP();

	return HAL_OK;

}

HAL_StatusTypeDef ICM42688_Change_Bank(ICM42688 *device, BANK_SEL reg){

	uint8_t data = reg & 0x07;

	HAL_StatusTypeDef status = ICM42688_Write(device, REG_BANK_SEL, &data);

	return status;
}


//===============================    LOW LEVEL FUNCTIONS ===================================

HAL_StatusTypeDef ICM42688_Write(ICM42688 *device, uint8_t address, uint8_t *data){

	uint8_t tx_Buff[2] = {address, *data};
	HAL_StatusTypeDef status;

	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);

	status = HAL_SPI_Transmit(device->spiHandle, tx_Buff, 2, HAL_MAX_DELAY);

	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

	return status;
}

HAL_StatusTypeDef ICM42688_Write_Multiple(ICM42688 *device, uint8_t address, uint8_t *data, uint8_t length){

	HAL_StatusTypeDef status;

	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);

	status = HAL_SPI_Transmit(device->spiHandle, &address, 1, HAL_MAX_DELAY);
	if(status != HAL_OK) return status;

	status = HAL_SPI_Transmit(device->spiHandle, data, length, HAL_MAX_DELAY);
	if(status != HAL_OK) return status;

	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

	return status;
}


HAL_StatusTypeDef ICM42688_Read(ICM42688 *device, uint8_t address, uint8_t *data){

	uint8_t txBuff = (address | 0x80);
	HAL_StatusTypeDef status;

	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);

	status = HAL_SPI_Transmit(device->spiHandle, &txBuff, 1, HAL_MAX_DELAY);
	if(status != HAL_OK) return status;

	status = HAL_SPI_Receive(device->spiHandle, data, 1, HAL_MAX_DELAY);
	if(status != HAL_OK) return status;

	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

	return status;
}

HAL_StatusTypeDef ICM42688_Read_Multiple(ICM42688 *device, uint8_t address, uint8_t *data, uint8_t length){

	uint8_t txBuff = (address | 0x80);
	HAL_StatusTypeDef status;

	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);

	status = HAL_SPI_Transmit(device->spiHandle, &txBuff, 1, HAL_MAX_DELAY);
	if(status != HAL_OK) return status;

	status = HAL_SPI_Receive(device->spiHandle, data, length, HAL_MAX_DELAY);
	if(status != HAL_OK) return status;

	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

	return status;

}

HAL_StatusTypeDef ICM42688_Read_DMA(ICM42688 *device){

	uint8_t txBuff[15];
	txBuff[0] = (TEMP_DATA1 | 0x80);
	HAL_StatusTypeDef status;

	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);

	status = HAL_SPI_TransmitReceive_DMA(device->spiHandle, txBuff, device->buffer, 15);
	if(status != HAL_OK) return status;

	//HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

	return status;
}





