/*
 * dmpc.c
 *
 *  Created on: 02.08.2021
 *      Author: LRS
 */

//===========================================================================
/*------------------------------- Includes --------------------------------*/
//===========================================================================
#include "mpc.h"

#include "dmpc.h"
//===========================================================================

//===========================================================================
/*------------------------------- Functions -------------------------------*/
//===========================================================================
//---------------------------------------------------------------------------
void mpcInitialize(void *mpct, uint32_t *p){

    mpc_t *mpc;

    mpc = (mpc_t *)mpct;

    mpc->x[0] = 0;
    mpc->x[1] = 0;

    mpc->x_1[0] = 0;
    mpc->x_1[1] = 0;

    mpc->u = 0;
    mpc->u_1 = 0;
    mpc->du = 0;

    mpc->iters = 0;
}
//---------------------------------------------------------------------------
float mpcControl(void *mpct, uint16_t ref, platCPU2ControlData_t *data){

    float r;

    mpc_t *mpc;

    mpc = (mpc_t *)mpct;

    r = ((float)ref) * ((float)PLAT_CONFIG_GAIN_REF);

    mpc->x[0] = (((float)*data->adc[PLAT_CONFIG_BUCK_IL_AVG_BUCK_BUFFER]) * ((float)PLAT_CONFIG_BUCK_IL_AVG_GAIN)) + ((float)PLAT_CONFIG_BUCK_IL_AVG_OFFS);
    mpc->x[1] = ((float)(*data->adc[PLAT_CONFIG_BUCK_V_OUT_BUCK_BUFFER])) * ((float)PLAT_CONFIG_BUCK_V_OUT_BUCK_GAIN);

    /* Delay compensation */
    dmpcDelayComp(mpc->x_1, mpc->x, &mpc->u);

    /* Optimization */
    dmpcOpt(mpc->x_1, mpc->x, &r, &mpc->u, 0, &mpc->du);

    /* Computes u = du + u_1 */
    mpc->u = mpc->u + mpc->du;

    if( mpc->u > 25.0f ) mpc->u = 25.0f;
    else if (mpc->u < 0 ) mpc->u = 0;

    //mpc->u_1 = mpc->u;

    return mpc->u / 25.0f;
}
//---------------------------------------------------------------------------
//===========================================================================
