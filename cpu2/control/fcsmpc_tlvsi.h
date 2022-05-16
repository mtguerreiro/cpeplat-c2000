/*
 * fcsmpc_tlvsi.h
 *
 *  Created on: 03.01.2022
 *      Author: mguerreiro
 */

#ifndef CONTROL_FCSMPC_TLVSI_H_
#define CONTROL_FCSMPC_TLVSI_H_

//===========================================================================
/*-------------------------------- Includes -------------------------------*/
//===========================================================================
#include <stdint.h>

#include "plat_defs.h"

#include "tlvsi.h"
#include "tppll.h"
//===========================================================================

//===========================================================================
/*------------------------------- Functions -------------------------------*/
//===========================================================================
void fcsmpctlvsiInitialize(void *fcsmpctlvsit, uint32_t *p);
float fcsmpctlvsiControl(void *fcsmpctlvsit, uint16_t ref, platCPU2ControlData_t *data);
uint32_t fcsmpcswRead(void);
//===========================================================================

#endif /* CONTROL_FCSMPC_TLVSI_H_ */
