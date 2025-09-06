/*
 * USB.h
 *
 *  Created on: Sep 6, 2025
 *      Author: sergi
 */

#ifndef INC_USB_H_
#define INC_USB_H_

#include "stm32f7xx_hal.h"
#include "string.h"
#include "Parameters.h"
#include <stdlib.h>
#include <stdio.h>
#include "usbd_cdc_if.h"


void USB_RXCallback(uint8_t *Buf, uint32_t *Len);



#endif /* INC_USB_H_ */
