/*
 * adc.h
 *
 *  Created on: 20.05.2021
 *      Author: mguerreiro
 */

#ifndef CLIBS_ADC_H_
#define CLIBS_ADC_H_

//=============================================================================
/*-------------------------------- Includes ---------------------------------*/
//=============================================================================
//#include <xdc/std.h>

#include "device.h"
//#include <stdint.h>
//=============================================================================

//=============================================================================
/*--------------------------------- Defines ---------------------------------*/
//=============================================================================
#define ADC_CONFIG_BUFFER_SIZE      200
//=============================================================================

//=============================================================================
/*-------------------------------- Functions --------------------------------*/
//=============================================================================
//-----------------------------------------------------------------------------
void adcInitialize(void);
//-----------------------------------------------------------------------------
void adcStartAcq(void);
//-----------------------------------------------------------------------------
uint16_t* adcGetBuffer(void);
//-----------------------------------------------------------------------------
int32_t adcRead(uint16_t *buffer, uint32_t nbytes);
//-----------------------------------------------------------------------------
//=============================================================================

#endif /* CLIBS_ADC_H_ */