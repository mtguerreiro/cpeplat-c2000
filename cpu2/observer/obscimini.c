/*
 * obscimini.c
 *
 *  Created on: 30.07.2021
 *      Author: mguerreiro
 */

//===========================================================================
/*------------------------------- Includes --------------------------------*/
//===========================================================================
#include "obscimini.h"

//===========================================================================

//===========================================================================
/*------------------------------- Functions -------------------------------*/
//===========================================================================
//---------------------------------------------------------------------------
void obsciminiInitialize(void *ciminit, uint32_t *p){

    obscimini_t *cimini = (obscimini_t *)ciminit;

    cimini->a11 = *((float *)(&p[0]));
    cimini->a12 = *((float *)(&p[1]));
    cimini->b11 = *((float *)(&p[2]));

    cimini->a21 = *((float *)(&p[3]));
    cimini->a22 = *((float *)(&p[4]));
    cimini->a23 = *((float *)(&p[5]));
    cimini->a24 = *((float *)(&p[6]));
    cimini->a25 = *((float *)(&p[7]));
    cimini->a26 = *((float *)(&p[8]));

    cimini->il_h = 0;
    cimini->il_h_1 = 0;

    cimini->vc_h = 0;
    cimini->vc_h_1 = 0;
}
//---------------------------------------------------------------------------
void obsciminiObserve(void *ciminit, platCPU2ObserverData_t *data){

    float vin, u, vc;
    float aux1, aux2;
    obscimini_t *cimini;

    cimini = (obscimini_t *)ciminit;

    vc = ((float)(*data->adc[5])) * ((float)PLAT_GAIN_ADC_5);
    vin = ((float)(*data->adc[1])) * ((float)PLAT_GAIN_ADC_1);
    u = ((float)(*data->u)) * ((float)PLAT_GAIN_CTL);

    aux1 = cimini->vc_h - vc;

    if( aux1 > 0 ){
        if( aux1 > ((float)100e-3) ) aux1 = (float)1.0;
    }
    else{
        if( aux1 < ((float)-100e-3) ) aux1 = (float)-1.0;
    }

    if( vc > 0 ) aux2 = vc;
    else aux2 = -vc;

    cimini->il_h_1 = cimini->a11 * cimini->il_h + cimini->a12 * cimini->vc_h \
            + cimini->b11 * vin * u;

    cimini->vc_h_1 = cimini->a21 * cimini->il_h + cimini->a22 * cimini->vc_h +\
            cimini->a23 * vc + cimini->a24 * aux1 * (cimini->a25 * aux2 + cimini->a26);

    cimini->il_h = cimini->il_h_1;
    cimini->vc_h = cimini->vc_h_1;

    data->states[0] = cimini->il_h_1;
    data->states[1] = cimini->vc_h_1;
}
//---------------------------------------------------------------------------
//===========================================================================
