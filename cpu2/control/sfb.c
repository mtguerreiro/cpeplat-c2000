/*
 * sfb.c
 *
 *  Created on: 30.07.2021
 *      Author: LRS
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
void sfbInitialize(void *sfbt, uint32_t *p){

    sfb_t *sfb;

    sfb = (sfb_t *)sfbt;

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

    if( data->observer != 0 ){
        il = data->observer->states[0];
        vc = data->observer->states[1];
    }
    else{
        vc = ((float)(*data->adc[5])) * ((float)PLAT_CONFIG_BUCK_V_OUT_BUCK_GAIN);
        il = (((float)*data->adc[2]) * ((float)PLAT_CONFIG_BUCK_IL_GAIN)) + ((float)PLAT_CONFIG_BUCK_IL_OFFS);
    }

    r = ((float)ref) * ((float)PLAT_CONFIG_GAIN_REF);

    e = r - vc;
    zeta = sfb->zeta_1 + sfb->dt * ((float)0.50) * (e + sfb->e_1);

    u = -sfb->k_il * il - sfb->k_vc * vc + sfb->k_z * zeta;

    if( u > 1 ) u = 1.0;
    else if ( u < 0 ) u = 0.0;

    sfb->e_1 = e;
    sfb->zeta_1 = zeta;

    return u;
}
//---------------------------------------------------------------------------
//===========================================================================
