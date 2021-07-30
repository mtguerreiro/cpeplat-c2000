/*
 * sfb.c
 *
 *  Created on: 30.07.2021
 *      Author: mguerreiro
 */

//===========================================================================
/*------------------------------- Includes --------------------------------*/
//===========================================================================
#include "sfb.h"

//===========================================================================

//===========================================================================
/*------------------------------- Functions -------------------------------*/
//===========================================================================
//---------------------------------------------------------------------------
void sfbInitialize(sfb_t *sfb, uint32_t *p){

    sfb->k_il = *((float *)(&p[0]));
    sfb->k_vc = *((float *)(&p[1]));

    sfb->k_z = *((float *)(&p[2]));

    sfb->dt = *((float *)(&p[3]));

    sfb->e_1 = 0;
    sfb->zeta_1 = 0;
}
//---------------------------------------------------------------------------
float sfbControl(void *sfbt, uint16_t ref, platCPU2ControlData_t *data){

    float r;
    float vc;
    float il;
    float e;
    float zeta;
    float u;

    sfb_t *sfb;

    sfb = (sfb_t *)sfbt;

    r = ((float)ref) * ((float)0.007326007326007326);
    vc = ((float)(*data->adc[3])) * ((float)0.007326007326007326);
    il = ((float)(*data->adc[4])) * ((float)0.022165868319714472) + ((float)-50.0);

    e = r - vc;
    zeta = sfb->zeta_1 + sfb->dt * ((float)0.50) * (e + sfb->e_1);

    u = -sfb->k_il * il - sfb->k_vc * vc + sfb->k_z * zeta;

    if( u > 1 ) u = 1;
    else if ( u < 0 ) u = 0;

    sfb->e_1 = e;
    sfb->zeta_1 = zeta;

    return u;
}
//---------------------------------------------------------------------------
//===========================================================================
