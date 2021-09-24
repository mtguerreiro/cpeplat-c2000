/*
 * MatlabInterface.h
 *
 *  Created on: 04.08.2021
 *      Author: Moritz Frantz
 */

#ifndef CONTROL_MATLABINTERFACE_H_
#define CONTROL_MATLABINTERFACE_H_

//===========================================================================
/*-------------------------------- Includes -------------------------------*/
//===========================================================================
#include <stdint.h>

#include "plat_defs.h"
//===========================================================================

//===========================================================================
/*------------------------------- Data types ------------------------------*/
//===========================================================================
typedef struct{
    float Vout;
    float Vin_buck;
    float IL;
    float Vin;
    float IL_avg;
    float Vout_buck;
}matlab_t;
//===========================================================================

//===========================================================================
/*------------------------------- Functions -------------------------------*/
//===========================================================================
void matlabInitialize(void *matlabt, uint32_t *p);
float matlabControl(void *matlabt, uint16_t ref, platCPU2ControlData_t *data);
//===========================================================================

#endif /* CONTROL_MATLABINTERFACE_H_ */
