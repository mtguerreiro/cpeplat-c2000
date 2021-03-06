/*
 * pid.h
 *
 *  Created on: 21.07.2021
 *      Author: LRS
 */

#ifndef CONTROL_PID_H_
#define CONTROL_PID_H_


//===========================================================================
/*-------------------------------- Includes -------------------------------*/
//===========================================================================
#include <stdint.h>

#include "plat_defs.h"
//===========================================================================

//===========================================================================
/*-------------------------------- Defines --------------------------------*/
//===========================================================================

//===========================================================================

//===========================================================================
/*------------------------------- Data types ------------------------------*/
//===========================================================================
typedef struct{
    float a1;
    float a2;
    float b0;
    float b1;
    float b2;

    float u_1;
    float u_2;
    float e_1;
    float e_2;
}pid_t;
//===========================================================================

//===========================================================================
/*------------------------------- Functions -------------------------------*/
//===========================================================================
void pidInitialize(void *pidt, uint32_t *p);
float pidControl(void *pidt, uint16_t ref, platCPU2ControlData_t *data);
//===========================================================================

#endif /* CONTROL_PID_H_ */
