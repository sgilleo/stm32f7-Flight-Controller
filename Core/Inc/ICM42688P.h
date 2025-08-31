/*
 * icm42688p.h
 *
 *  Created on: Aug 11, 2025
 *      Author: sergi
 */

#ifndef INC_ICM42688P_H_
#define INC_ICM42688P_H_

#include "stm32f7xx_hal.h"

typedef enum{
	GYRO_FS_SEL_2000_DPS = 0x00,  //+-2000º/s
	GYRO_FS_SEL_1000_DPS = 0x01,  //+-1000º/s
	GYRO_FS_SEL_500_DPS = 0x02,   //+-500º/s
	GYRO_FS_SEL_250_DPS = 0x03,   //+-250º/s
	GYRO_FS_SEL_125_DPS = 0x04,   //+-125º/s
	GYRO_FS_SEL_62P5_DPS = 0x05,  //+-62.5º/s
	GYRO_FS_SEL_31P25_DPS = 0x06, //+-31.25º/s
	GYRO_FS_SEL_15P625_DPS = 0x07 //+-15.625º/s
} GYRO_FS_SEL;

typedef enum{
	GYRO_ODR_32_KHZ = 0x01,   //32 kHz
	GYRO_ODR_16_KHZ = 0x02,   //16 kHz
	GYRO_ODR_8_KHZ = 0x03,    //8 kHz
	GYRO_ODR_4_KHZ = 0x04,    //4 kHz
	GYRO_ODR_2_KHZ = 0x05,    //2 kHz
	GYRO_ODR_1_KHZ = 0x06,    //1 kHz
	GYRO_ODR_200_HZ = 0x07,   //200 Hz
	GYRO_ODR_100_HZ = 0x08,   //100 Hz
	GYRO_ODR_50_HZ = 0x09,    //50 Hz
	GYRO_ODR_25_HZ = 0x0A,    //25 Hz
	GYRO_ODR_12P5_HZ = 0x0B,  //12.5 Hz
	GYRO_ODR_500_HZ = 0x0F    //500 Hz
} GYRO_ODR;

typedef enum{
	ACCEL_FS_SEL_16_G = 0x00, //+-16 G
	ACCEL_FS_SEL_8_G = 0x01,  // +-8 G
	ACCEL_FS_SEL_4_G = 0x02,  // +-4 G
	ACCEL_FS_SEL_2_G = 0x03   // +-2 G
} ACCEL_FS_SEL;

typedef enum{
	ACCEL_ODR_32_KHZ = 0x01,   //32 kHz
	ACCEL_ODR_16_KHZ = 0x02,   //16 kHz
	ACCEL_ODR_8_KHZ = 0x03,    //8 kHz
	ACCEL_ODR_4_KHZ = 0x04,    //4 kHz
	ACCEL_ODR_2_KHZ = 0x05,    //2 kHz
	ACCEL_ODR_1_KHZ = 0x06,    //1 kHz
	ACCEL_ODR_200_HZ = 0x07,   //200 Hz
	ACCEL_ODR_100_HZ = 0x08,   //100 Hz
	ACCEL_ODR_50_HZ = 0x09,    //50 Hz
	ACCEL_ODR_25_HZ = 0x0A,    //25 Hz
	ACCEL_ODR_12P5_HZ = 0x0B,  //12.5 Hz
	ACCEL_ODR_6P25_HZ = 0x0C,  //6.25 Hz
	ACCEL_ODR_3P125_HZ = 0x0D, //3.125 Hz
	ACCEL_ODR_1P5625_HZ = 0x0E,//1.5625 Hz
	ACCEL_ODR_500_HZ = 0x0F    //500 Hz
} ACCEL_ODR;

typedef enum{
	BANK_0 = 0x00,
	BANK_1 = 0x01,
	BANK_2 = 0x02,
	BANK_3 = 0x03,
	BANK_4 = 0x04
} BANK_SEL;

typedef enum{
	UI_FILT_ORD_1_ORD = 0x00,
	UI_FILT_ORD_2_ORD = 0x01,
	UI_FILT_ORD_3_ORD = 0x02
} UI_FILT_ORD;

typedef enum{
	UI_FILT_BW_500_HZ = 0x00, //Equivalent to 1KHz ODR
	UI_FILT_BW_250_HZ = 0x01, //Equivalent to 1KHz ODR
	UI_FILT_BW_200_HZ = 0x02, //Equivalent to 1KHz ODR
	UI_FILT_BW_125_HZ = 0x03, //Equivalent to 1KHz ODR
	UI_FILT_BW_100_HZ = 0x04, //Equivalent to 1KHz ODR
	UI_FILT_BW_62P5_HZ = 0x05,//Equivalent to 1KHz ODR
	UI_FILT_BW_50_HZ = 0x06,  //Equivalent to 1KHz ODR
	UI_FILT_BW_25_HZ = 0x07,  //Equivalent to 1KHz ODR
	UI_FILT_BW_1000_HZ = 0x0E, //Low Latency
	UI_FILT_BW_8000_HZ = 0x0F, //Low Latency
} UI_FILT_BW;

#define DEG_TO_RAD 0.01745329252
#define RAD_TO_DEG 57.29577951

#define INT_CONFIG 0x14

#define TEMP_DATA1 0x1D
#define TEMP_DATA0 0x1E

#define ACCEL_DATA_X1 0x1F
#define ACCEL_DATA_X0 0x20
#define ACCEL_DATA_Y1 0x21
#define ACCEL_DATA_Y0 0x22
#define ACCEL_DATA_Z1 0x23
#define ACCEL_DATA_Z0 0x24

#define GYRO_DATA_X1 0x25
#define GYRO_DATA_X0 0x26
#define GYRO_DATA_Y1 0x27
#define GYRO_DATA_Y0 0x28
#define GYRO_DATA_Z1 0x29
#define GYRO_DATA_Z0 0x2A

#define PWR_MGMT0 0x4E
#define GYRO_CONFIG0 0x4F
#define ACCEL_CONFIG0 0x50
#define GYRO_CONFIG1 0x51
#define GYRO_ACCEL_CONFIG0 0x52
#define ACCEL_CONFIG1 0x53

#define INT_CONFIG0 0x63
#define INT_CONFIG1 0x64
#define INT_SOURCE0 0x65
#define INT_SOURCE1 0x66

#define INT_SOURCE3 0x68
#define INT_SOURCE4 0x69

#define WHO_AM_I 0x75

#define REG_BANK_SEL 0x76

#define OFFSET_USER0 0x77
#define OFFSET_USER1 0x78
#define OFFSET_USER2 0x79
#define OFFSET_USER3 0x7A
#define OFFSET_USER4 0x7B
#define OFFSET_USER5 0x7C
#define OFFSET_USER6 0x7D
#define OFFSET_USER7 0x7E
#define OFFSET_USER8 0x7F

typedef struct {
	float x;
	float y;
	float z;
} Vec3;

typedef struct {
	SPI_HandleTypeDef *spiHandle;

	Vec3 accel;
	Vec3 gyro;

	float temp;

	GYRO_FS_SEL gyro_fs;
	GYRO_ODR gyro_odr;

	ACCEL_FS_SEL accel_fs;
	ACCEL_ODR accel_odr;

	uint8_t buffer[15]; //DMA Buffer: {Trash, Temp1, Temp0, AccX1, AccX0, AccY1, AccY0, AccZ1, AccZ0, GyroX1, GyroX0, GyroY1, GyroY0, GyroZ1, GyroZ0}

	uint8_t ready;
	uint8_t dataRdy;

} ICM42688;

HAL_StatusTypeDef ICM42688_Begin(ICM42688 *device, SPI_HandleTypeDef *spiHandle);




HAL_StatusTypeDef ICM42688_Set_Gyro_FS(ICM42688 *device, GYRO_FS_SEL range);
HAL_StatusTypeDef ICM42688_Set_Gyro_ODR(ICM42688 *device, GYRO_ODR rate);

HAL_StatusTypeDef ICM42688_Set_Accel_FS(ICM42688 *device, ACCEL_FS_SEL range);
HAL_StatusTypeDef ICM42688_Set_Accel_ODR(ICM42688 *device, ACCEL_ODR rate);

HAL_StatusTypeDef ICM42688_Set_Filters(ICM42688 *device);

HAL_StatusTypeDef ICM42688_Set_Interrupts(ICM42688 *device);

HAL_StatusTypeDef ICM42688_Read_Accel(ICM42688 *device);

HAL_StatusTypeDef ICM42688_Read_Gyro(ICM42688 *device);

HAL_StatusTypeDef ICM42688_Read_Temp(ICM42688 *device);

void ICM42688_Process_Buffer(ICM42688 *device);

HAL_StatusTypeDef ICM42688_Change_Bank(ICM42688 *device, BANK_SEL reg);


//===============================    LOW LEVEL FUNCTIONS ===================================

HAL_StatusTypeDef ICM42688_Write(ICM42688 *device, uint8_t address, uint8_t *data);

HAL_StatusTypeDef ICM42688_Write_Multiple(ICM42688 *device, uint8_t address, uint8_t *data, uint8_t length);

HAL_StatusTypeDef ICM42688_Read(ICM42688 *device, uint8_t address, uint8_t *data);

HAL_StatusTypeDef ICM42688_Read_Multiple(ICM42688 *device, uint8_t address, uint8_t *data, uint8_t length);

HAL_StatusTypeDef ICM42688_Read_DMA(ICM42688 *device);





#endif /* INC_ICM42688P_H_ */
