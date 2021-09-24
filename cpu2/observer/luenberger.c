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
void luenbergerInitialize(void *luenbergert, uint32_t *p){

    luenberger_t *luenberger = (luenberger_t *)luenbergert;

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

    vo = ((float)(*data->adc[PLAT_CONFIG_BUCK_V_OUT_BUCK_BUFFER])) * ((float)PLAT_CONFIG_BUCK_V_OUT_BUCK_GAIN);
    u = ((float)(*data->u)) * ((float)PLAT_CONFIG_GAIN_CTL);

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
