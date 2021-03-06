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
void pidInitialize(void *pidt, uint32_t *p){

    pid_t *pid;

    pid = (pid_t *)pidt;

    pid->a1 = *((float *)(&p[0]));
    pid->a2 = *((float *)(&p[1]));
    pid->b0 = *((float *)(&p[2]));
    pid->b1 = *((float *)(&p[3]));
    pid->b2 = *((float *)(&p[4]));

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
    y = ((float)(*data->adc[PLAT_CONFIG_BUCK_V_OUT_BUCK_BUFFER])) * ((float)PLAT_CONFIG_BUCK_V_OUT_BUCK_GAIN);

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
