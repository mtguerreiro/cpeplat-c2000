/*
 * predictive.c
 *
 *  Created on: 28.08.2021
 *      Author: mguerreiro
 */

//===========================================================================
/*------------------------------- Includes --------------------------------*/
//===========================================================================
#include "predictive.h"

//===========================================================================

//===========================================================================
/*------------------------------- Functions -------------------------------*/
//===========================================================================
//---------------------------------------------------------------------------
void predictiveInitialize(void *predictivet, uint32_t *p){

    predictive_t *predictive = (predictive_t *)predictivet;

    predictive->a11 = *((float *)(&p[0]));
    predictive->a12 = *((float *)(&p[1]));
    predictive->a21 = *((float *)(&p[2]));
    predictive->a22 = *((float *)(&p[3]));

    predictive->b11 = *((float *)(&p[4]));
    predictive->b21 = *((float *)(&p[5]));

    predictive->il_h_1 = 0.0;
    predictive->vc_h_1 = 0.0;
}
//---------------------------------------------------------------------------
void predictiveObserve(void *predictivet, platCPU2ObserverData_t *data){

    float u, vo, il;
    predictive_t *predictive;

    vo = ((float)(*data->adc[PLAT_CONFIG_BUCK_V_OUT_BUCK_BUFFER])) * ((float)PLAT_CONFIG_BUCK_V_OUT_BUCK_GAIN);
    il = (((float)*data->adc[PLAT_CONFIG_BUCK_IL_BUFFER]) * ((float)PLAT_CONFIG_BUCK_IL_GAIN)) + ((float)PLAT_CONFIG_BUCK_IL_OFFS);
    u = ((float)(*data->u)) * ((float)PLAT_CONFIG_GAIN_CTL);

    predictive = (predictive_t *)predictivet;

    predictive->il_h_1 = predictive->a11 * il + predictive->a12 * vo + predictive->b11 * u;
    predictive->vc_h_1 = predictive->a21 * il + predictive->a22 * vo + predictive->b21 * u;

    data->states[0] = predictive->il_h_1;
    data->states[1] = predictive->vc_h_1;
}
//---------------------------------------------------------------------------
//===========================================================================
