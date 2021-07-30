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
void obsciminiInitialize(obscimini_t *cimini, uint32_t *p){

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
void obsciminiObserve(void *ciminit, float *measurements, observerStates_t *states){

    float vin, u, vc;
    float aux1, aux2;
    obscimini_t *cimini;

    vc = measurements[0];
    vin = measurements[1];
    u = measurements[2];

    cimini = (obscimini_t *)ciminit;

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

    states->il = cimini->il_h_1;
    states->vc = cimini->vc_h_1;
}
//---------------------------------------------------------------------------
//===========================================================================

