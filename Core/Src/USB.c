/*
 * USB.c
 *
 *  Created on: Sep 6, 2025
 *      Author: sergi
 */

#include "USB.h"

uint8_t usbRxBuffer[100];
uint8_t usbTxBuffer[20];
extern ParameterTable parameters;

Output_Functions atoFunction(char *str){
	if(strcmp(str, "NONE") == 0) return NONE;
	else if(strcmp(str, "AILERONS") == 0) return AILERONS;
	else if(strcmp(str, "ELEVATOR") == 0) return ELEVATOR;
	else if(strcmp(str, "THROTTLE") == 0) return THROTTLE;
	else if(strcmp(str, "RUDDER") == 0) return RUDDER;
	else if(strcmp(str, "FLAP") == 0) return FLAP;

	return NONE;
}

void Set_Param_By_Name(char *name, char *value){
	if(strcmp(name, "STABILIZED_MAX_ROLL") == 0) parameters.STABILIZED_MAX_ROLL = atof(value);
	else if(strcmp(name, "STABILIZED_MAX_PITCH") == 0) parameters.STABILIZED_MAX_PITCH = atof(value);

	else if(strcmp(name, "ACRO_ROLL_RATE") == 0) parameters.ACRO_ROLL_RATE = atof(value);
	else if(strcmp(name, "ACRO_PITCH_RATE") == 0) parameters.ACRO_PITCH_RATE = atof(value);
	else if(strcmp(name, "ACRO_YAW_RATE") == 0) parameters.ACRO_YAW_RATE = atof(value);

	else if(strcmp(name, "ROLL_KD_STABILIZED") == 0) parameters.ROLL_KD_STABILIZED = atof(value);
	else if(strcmp(name, "ROLL_KP_STABILIZED") == 0) parameters.ROLL_KP_STABILIZED = atof(value);
	else if(strcmp(name, "ROLL_KI_STABILIZED") == 0) parameters.ROLL_KI_STABILIZED = atof(value);

	else if(strcmp(name, "PITCH_KD_STABILIZED") == 0) parameters.PITCH_KD_STABILIZED = atof(value);
	else if(strcmp(name, "PITCH_KP_STABILIZED") == 0) parameters.PITCH_KP_STABILIZED = atof(value);
	else if(strcmp(name, "PITCH_KI_STABILIZED") == 0) parameters.PITCH_KI_STABILIZED = atof(value);

	else if(strcmp(name, "ROLL_KD_ACRO") == 0) parameters.ROLL_KD_ACRO = atof(value);
	else if(strcmp(name, "ROLL_KP_ACRO") == 0) parameters.ROLL_KP_ACRO = atof(value);
	else if(strcmp(name, "ROLL_KI_ACRO") == 0) parameters.ROLL_KI_ACRO = atof(value);

	else if(strcmp(name, "PITCH_KD_ACRO") == 0) parameters.PITCH_KD_ACRO = atof(value);
	else if(strcmp(name, "PITCH_KP_ACRO") == 0) parameters.PITCH_KP_ACRO = atof(value);
	else if(strcmp(name, "PITCH_KI_ACRO") == 0) parameters.PITCH_KI_ACRO = atof(value);

	else if(strcmp(name, "YAW_KD_ACRO") == 0) parameters.YAW_KD_ACRO = atof(value);
	else if(strcmp(name, "YAW_KP_ACRO") == 0) parameters.YAW_KP_ACRO = atof(value);

	else if(strcmp(name, "OUT1_FUNCTION") == 0) parameters.OUT1_FUNCTION = atoFunction(value);
	else if(strcmp(name, "OUT2_FUNCTION") == 0) parameters.OUT2_FUNCTION = atoFunction(value);
	else if(strcmp(name, "OUT3_FUNCTION") == 0) parameters.OUT3_FUNCTION = atoFunction(value);
	else if(strcmp(name, "OUT4_FUNCTION") == 0) parameters.OUT4_FUNCTION = atoFunction(value);
	else if(strcmp(name, "OUT5_FUNCTION") == 0) parameters.OUT5_FUNCTION = atoFunction(value);
	else if(strcmp(name, "OUT6_FUNCTION") == 0) parameters.OUT6_FUNCTION = atoFunction(value);
	else if(strcmp(name, "OUT7_FUNCTION") == 0) parameters.OUT7_FUNCTION = atoFunction(value);
	else if(strcmp(name, "OUT8_FUNCTION") == 0) parameters.OUT8_FUNCTION = atoFunction(value);

	else if(strcmp(name, "OUT1_REVERSE") == 0) parameters.OUT1_REVERSE = atoi(value);
	else if(strcmp(name, "OUT2_REVERSE") == 0) parameters.OUT2_REVERSE = atoi(value);
	else if(strcmp(name, "OUT3_REVERSE") == 0) parameters.OUT3_REVERSE = atoi(value);
	else if(strcmp(name, "OUT4_REVERSE") == 0) parameters.OUT4_REVERSE = atoi(value);
	else if(strcmp(name, "OUT5_REVERSE") == 0) parameters.OUT5_REVERSE = atoi(value);
	else if(strcmp(name, "OUT6_REVERSE") == 0) parameters.OUT6_REVERSE = atoi(value);
	else if(strcmp(name, "OUT7_REVERSE") == 0) parameters.OUT7_REVERSE = atoi(value);
	else if(strcmp(name, "OUT8_REVERSE") == 0) parameters.OUT8_REVERSE = atoi(value);

	else if(strcmp(name, "OUT1_TRIM") == 0) parameters.OUT1_TRIM = atoi(value);
	else if(strcmp(name, "OUT2_TRIM") == 0) parameters.OUT2_TRIM = atoi(value);
	else if(strcmp(name, "OUT3_TRIM") == 0) parameters.OUT3_TRIM = atoi(value);
	else if(strcmp(name, "OUT4_TRIM") == 0) parameters.OUT4_TRIM = atoi(value);
	else if(strcmp(name, "OUT5_TRIM") == 0) parameters.OUT5_TRIM = atoi(value);
	else if(strcmp(name, "OUT6_TRIM") == 0) parameters.OUT6_TRIM = atoi(value);
	else if(strcmp(name, "OUT7_TRIM") == 0) parameters.OUT7_TRIM = atoi(value);
	else if(strcmp(name, "OUT8_TRIM") == 0) parameters.OUT8_TRIM = atoi(value);

	else if(strcmp(name, "OUT1_MIN") == 0) parameters.OUT1_MIN = atoi(value);
	else if(strcmp(name, "OUT2_MIN") == 0) parameters.OUT2_MIN = atoi(value);
	else if(strcmp(name, "OUT3_MIN") == 0) parameters.OUT3_MIN = atoi(value);
	else if(strcmp(name, "OUT4_MIN") == 0) parameters.OUT4_MIN = atoi(value);
	else if(strcmp(name, "OUT5_MIN") == 0) parameters.OUT5_MIN = atoi(value);
	else if(strcmp(name, "OUT6_MIN") == 0) parameters.OUT6_MIN = atoi(value);
	else if(strcmp(name, "OUT7_MIN") == 0) parameters.OUT7_MIN = atoi(value);
	else if(strcmp(name, "OUT8_MIN") == 0) parameters.OUT8_MIN = atoi(value);

	else if(strcmp(name, "OUT1_MAX") == 0) parameters.OUT1_MAX = atoi(value);
	else if(strcmp(name, "OUT2_MAX") == 0) parameters.OUT2_MAX = atoi(value);
	else if(strcmp(name, "OUT3_MAX") == 0) parameters.OUT3_MAX = atoi(value);
	else if(strcmp(name, "OUT4_MAX") == 0) parameters.OUT4_MAX = atoi(value);
	else if(strcmp(name, "OUT5_MAX") == 0) parameters.OUT5_MAX = atoi(value);
	else if(strcmp(name, "OUT6_MAX") == 0) parameters.OUT6_MAX = atoi(value);
	else if(strcmp(name, "OUT7_MAX") == 0) parameters.OUT7_MAX = atoi(value);
	else if(strcmp(name, "OUT8_MAX") == 0) parameters.OUT8_MAX = atoi(value);

	else if(strcmp(name, "THROTTLE_CHANNEL") == 0) parameters.THROTTLE_CHANNEL = atoi(value);
	else if(strcmp(name, "ROLL_CHANNEL") == 0) parameters.ROLL_CHANNEL = atoi(value);
	else if(strcmp(name, "PITCH_CHANNEL") == 0) parameters.PITCH_CHANNEL = atoi(value);
	else if(strcmp(name, "YAW_CHANNEL") == 0) parameters.YAW_CHANNEL = atoi(value);
	else if(strcmp(name, "ARMING_CHANNEL") == 0) parameters.ARMING_CHANNEL = atoi(value);
	else if(strcmp(name, "FLIGHT_MODE_CHANNEL") == 0) parameters.FLIGHT_MODE_CHANNEL = atoi(value);

}

float Get_Param_By_Name(char *name){
	if(strcmp(name, "STABILIZED_MAX_ROLL") == 0) return parameters.STABILIZED_MAX_ROLL;
	else if(strcmp(name, "STABILIZED_MAX_PITCH") == 0) return parameters.STABILIZED_MAX_PITCH;

	else if(strcmp(name, "ACRO_ROLL_RATE") == 0) return parameters.ACRO_ROLL_RATE;
	else if(strcmp(name, "ACRO_PITCH_RATE") == 0) return parameters.ACRO_PITCH_RATE;
	else if(strcmp(name, "ACRO_YAW_RATE") == 0) return parameters.ACRO_YAW_RATE;

	else if(strcmp(name, "ROLL_KD_STABILIZED") == 0) return parameters.ROLL_KD_STABILIZED;
	else if(strcmp(name, "ROLL_KP_STABILIZED") == 0) return parameters.ROLL_KP_STABILIZED;
	else if(strcmp(name, "ROLL_KI_STABILIZED") == 0) return parameters.ROLL_KI_STABILIZED;

	else if(strcmp(name, "PITCH_KD_STABILIZED") == 0) return parameters.PITCH_KD_STABILIZED;
	else if(strcmp(name, "PITCH_KP_STABILIZED") == 0) return parameters.PITCH_KP_STABILIZED;
	else if(strcmp(name, "PITCH_KI_STABILIZED") == 0) return parameters.PITCH_KI_STABILIZED;

	else if(strcmp(name, "ROLL_KD_ACRO") == 0) return parameters.ROLL_KD_ACRO;
	else if(strcmp(name, "ROLL_KP_ACRO") == 0) return parameters.ROLL_KP_ACRO;
	else if(strcmp(name, "ROLL_KI_ACRO") == 0) return parameters.ROLL_KI_ACRO;

	else if(strcmp(name, "PITCH_KD_ACRO") == 0) return parameters.PITCH_KD_ACRO;
	else if(strcmp(name, "PITCH_KP_ACRO") == 0) return parameters.PITCH_KP_ACRO;
	else if(strcmp(name, "PITCH_KI_ACRO") == 0) return parameters.PITCH_KI_ACRO;

	else if(strcmp(name, "YAW_KD_ACRO") == 0) return parameters.YAW_KD_ACRO;
	else if(strcmp(name, "YAW_KP_ACRO") == 0) return parameters.YAW_KP_ACRO;

	else if(strcmp(name, "OUT1_FUNCTION") == 0) return parameters.OUT1_FUNCTION;
	else if(strcmp(name, "OUT2_FUNCTION") == 0) return parameters.OUT2_FUNCTION;
	else if(strcmp(name, "OUT3_FUNCTION") == 0) return parameters.OUT3_FUNCTION;
	else if(strcmp(name, "OUT4_FUNCTION") == 0) return parameters.OUT4_FUNCTION;
	else if(strcmp(name, "OUT5_FUNCTION") == 0) return parameters.OUT5_FUNCTION;
	else if(strcmp(name, "OUT6_FUNCTION") == 0) return parameters.OUT6_FUNCTION;
	else if(strcmp(name, "OUT7_FUNCTION") == 0) return parameters.OUT7_FUNCTION;
	else if(strcmp(name, "OUT8_FUNCTION") == 0) return parameters.OUT8_FUNCTION;

	else if(strcmp(name, "OUT1_REVERSE") == 0) return parameters.OUT1_REVERSE;
	else if(strcmp(name, "OUT2_REVERSE") == 0) return parameters.OUT2_REVERSE;
	else if(strcmp(name, "OUT3_REVERSE") == 0) return parameters.OUT3_REVERSE;
	else if(strcmp(name, "OUT4_REVERSE") == 0) return parameters.OUT4_REVERSE;
	else if(strcmp(name, "OUT5_REVERSE") == 0) return parameters.OUT5_REVERSE;
	else if(strcmp(name, "OUT6_REVERSE") == 0) return parameters.OUT6_REVERSE;
	else if(strcmp(name, "OUT7_REVERSE") == 0) return parameters.OUT7_REVERSE;
	else if(strcmp(name, "OUT8_REVERSE") == 0) return parameters.OUT8_REVERSE;

	else if(strcmp(name, "OUT1_TRIM") == 0) return parameters.OUT1_TRIM;
	else if(strcmp(name, "OUT2_TRIM") == 0) return parameters.OUT2_TRIM;
	else if(strcmp(name, "OUT3_TRIM") == 0) return parameters.OUT3_TRIM;
	else if(strcmp(name, "OUT4_TRIM") == 0) return parameters.OUT4_TRIM;
	else if(strcmp(name, "OUT5_TRIM") == 0) return parameters.OUT5_TRIM;
	else if(strcmp(name, "OUT6_TRIM") == 0) return parameters.OUT6_TRIM;
	else if(strcmp(name, "OUT7_TRIM") == 0) return parameters.OUT7_TRIM;
	else if(strcmp(name, "OUT8_TRIM") == 0) return parameters.OUT8_TRIM;

	else if(strcmp(name, "OUT1_MIN") == 0) return parameters.OUT1_MIN;
	else if(strcmp(name, "OUT2_MIN") == 0) return parameters.OUT2_MIN;
	else if(strcmp(name, "OUT3_MIN") == 0) return parameters.OUT3_MIN;
	else if(strcmp(name, "OUT4_MIN") == 0) return parameters.OUT4_MIN;
	else if(strcmp(name, "OUT5_MIN") == 0) return parameters.OUT5_MIN;
	else if(strcmp(name, "OUT6_MIN") == 0) return parameters.OUT6_MIN;
	else if(strcmp(name, "OUT7_MIN") == 0) return parameters.OUT7_MIN;
	else if(strcmp(name, "OUT8_MIN") == 0) return parameters.OUT8_MIN;

	else if(strcmp(name, "OUT1_MAX") == 0) return parameters.OUT1_MAX;
	else if(strcmp(name, "OUT2_MAX") == 0) return parameters.OUT2_MAX;
	else if(strcmp(name, "OUT3_MAX") == 0) return parameters.OUT3_MAX;
	else if(strcmp(name, "OUT4_MAX") == 0) return parameters.OUT4_MAX;
	else if(strcmp(name, "OUT5_MAX") == 0) return parameters.OUT5_MAX;
	else if(strcmp(name, "OUT6_MAX") == 0) return parameters.OUT6_MAX;
	else if(strcmp(name, "OUT7_MAX") == 0) return parameters.OUT7_MAX;
	else if(strcmp(name, "OUT8_MAX") == 0) return parameters.OUT8_MAX;

	else if(strcmp(name, "THROTTLE_CHANNEL") == 0) return parameters.THROTTLE_CHANNEL;
	else if(strcmp(name, "ROLL_CHANNEL") == 0) return parameters.ROLL_CHANNEL;
	else if(strcmp(name, "PITCH_CHANNEL") == 0) return parameters.PITCH_CHANNEL;
	else if(strcmp(name, "YAW_CHANNEL") == 0) return parameters.YAW_CHANNEL;
	else if(strcmp(name, "ARMING_CHANNEL") == 0) return parameters.ARMING_CHANNEL;
	else if(strcmp(name, "FLIGHT_MODE_CHANNEL") == 0) return parameters.FLIGHT_MODE_CHANNEL;

	else return 0;
}

void Process_Command(char *data){
	char *token = strtok(data, " \r\n");

	if(strcmp(token, "set") == 0){
		char *param = strtok(NULL, " \r\n");
		char *value = strtok(NULL, " \r\n");

		if (param && value) {
			Set_Param_By_Name(param, value);
		}
	}

	else if(strcmp(token, "get") == 0) {
		char *param = strtok(NULL, " \r\n");

		if(param) {
			float value = Get_Param_By_Name(param);
			sprintf((char*) usbTxBuffer, "%s = %f\r\n", param, value);
			CDC_Transmit_FS(usbTxBuffer, sizeof(usbTxBuffer));
		}
	}

	else if(strcmp(token, "save") == 0) {
		Save_Parameters(&parameters);
	}

	else if(strcmp(token, "load") == 0) {
		Load_Parameters(&parameters);
	}

	else{
		sprintf((char*) usbTxBuffer, "Unknown command\r\n");
		CDC_Transmit_FS(usbTxBuffer, sizeof(usbTxBuffer));
	}
}

void USB_RXCallback(uint8_t *Buf, uint32_t *Len){
	memcpy(usbRxBuffer, Buf, *Len);

	Process_Command((char*) usbRxBuffer);

	memset(usbRxBuffer, '\0', *Len);
}


