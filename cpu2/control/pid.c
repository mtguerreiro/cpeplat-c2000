/*
 * pid.c
 *
 *  Created on: 21.07.2021
 *      Author: LRS
 */

//===========================================================================
/*------------------------------- Includes --------------------------------*/
//===========================================================================
#include "pid.h"

//===========================================================================

//===========================================================================
/*------------------------------- Functions -------------------------------*/
//===========================================================================
//---------------------------------------------------------------------------
void pidInitialize(pid_t *pid, float a1, float a2, float b0, float b1, float b2){

    pid->a1 = a1;
    pid->a2 = a2;
    pid->b0 = b0;
    pid->b1 = b1;
    pid->b2 = b2;

    pid->e_1 = 0;
    pid->e_2 = 0;
    pid->u_1 = 0;
    pid->u_2 = 0;
}
//---------------------------------------------------------------------------
float pidControl(void *pidt, uint16_t ref, platCPU2ControlData_t *data){

    float r;
    float y;
    float e;
    float u;

    pid_t *pid;

    pid = (pid_t *)pidt;

    r = ((float)ref) * ((float)PLAT_CONFIG_GAIN_REF);
    y = ((float)(*data->adc[5])) * ((float)PLAT_CONFIG_BUCK_V_OUT_BUCK_GAIN);

    e = r - y;

    u = -pid->a1 * pid->u_1 - pid->a2 * pid->u_2 + \
            pid->b0 * e + pid->b1 * pid->e_1 + pid->b2 * pid->e_2;

    if( u > 1 ) u = 1;
    else if( u < 0 ) u = 0;

    pid->u_2 = pid->u_1;
    pid->u_1 = u;

    pid->e_2 = pid->e_1;
    pid->e_1 = e;

    return u;
}
//---------------------------------------------------------------------------
//===========================================================================
