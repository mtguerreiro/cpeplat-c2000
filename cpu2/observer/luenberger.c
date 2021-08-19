/*
 * luenberger.c
 *
 *  Created on: 19.08.2021
 *      Author: mguerreiro
 */

//===========================================================================
/*------------------------------- Includes --------------------------------*/
//===========================================================================
#include "luenberger.h"

//===========================================================================

//===========================================================================
/*------------------------------- Functions -------------------------------*/
//===========================================================================
//---------------------------------------------------------------------------
void luenbergerInitialize(luenberger_t *luenberger, uint32_t *p){

    luenberger->a11 = *((float *)(&p[0]));
    luenberger->a12 = *((float *)(&p[1]));
    luenberger->a21 = *((float *)(&p[2]));
    luenberger->a22 = *((float *)(&p[3]));

    luenberger->b11 = *((float *)(&p[4]));
    luenberger->b12 = *((float *)(&p[5]));
    luenberger->b21 = *((float *)(&p[6]));
    luenberger->b22 = *((float *)(&p[7]));

    luenberger->il_h = 0;
    luenberger->il_h_1 = 0;

    luenberger->vo_h = 0;
    luenberger->vo_h_1 = 0;
}
//---------------------------------------------------------------------------
void luenbergerObserve(void *luenbergert, platCPU2ObserverData_t *data){

    float u, vo;
    luenberger_t *luenberger;

    vo = ((float)(*data->adc[5])) * ((float)0.007326007326007326);
    u = ((float)(*data->u)) * ((float)0.002004008016032064);

    luenberger = (luenberger_t *)luenbergert;

    luenberger->il_h_1 = luenberger->a11 * luenberger->il_h + luenberger->a12 * luenberger->vo_h \
            + luenberger->b11 * u + luenberger->b12 * vo;

    luenberger->vo_h_1 = luenberger->a21 * luenberger->il_h + luenberger->a22 * luenberger->vo_h \
            + luenberger->b21 * u + luenberger->b22 * vo;

    luenberger->il_h = luenberger->il_h_1;
    luenberger->vo_h = luenberger->vo_h_1;

    data->states[0] = luenberger->il_h_1;
    data->states[1] = luenberger->vo_h_1;
}
//---------------------------------------------------------------------------
//===========================================================================
