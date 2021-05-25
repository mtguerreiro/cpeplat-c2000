/*
 * blink.h
 *
 *  Created on: 01.12.2020
 *      Author: mguerreiro
 */

#ifndef TASKS_BLINK_H_
#define TASKS_BLINK_H_

//=============================================================================
/*-------------------------------- Includes ---------------------------------*/
//=============================================================================
#include <xdc/std.h>

#include "device.h"
//=============================================================================

//=============================================================================
/*--------------------------------- Defines ---------------------------------*/
//=============================================================================
#define BLINK_CONFIG_LED            DEVICE_GPIO_PIN_LED1

#define BLINK_CMD_UPDATE_PERIOD     0x01
//=============================================================================

//=============================================================================
/*---------------------------------- Task -----------------------------------*/
//=============================================================================
void blink(UArg a0, UArg a1);
//=============================================================================

#endif /* TASKS_BLINK_H_ */