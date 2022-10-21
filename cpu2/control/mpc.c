/*
 * mpc.c
 *
 *  Created on: 02.08.2021
 *      Author: LRS
 */

//===========================================================================
/*------------------------------- Includes --------------------------------*/
//===========================================================================
#include "mpc.h"
#include "dmpc.h"
#include "dmpc_defs.h"
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

    static float r[DMPC_CONFIG_NY] = {0.0};

    static float xm[DMPC_CONFIG_NXM] = {0.0};
    static float xm_1[DMPC_CONFIG_NXM] = {0.0};

    static float u[DMPC_CONFIG_NU + DMPC_CONFIG_ND] = {0.0};
    //static float u_1[1] = {0.0};
    static float du[DMPC_CONFIG_NU + DMPC_CONFIG_ND] = {0.0};

    static uint32_t iters;

    /* Reference */
    r[0] =((float)ref) * ((float)PLAT_CONFIG_GAIN_REF);

    /* Output current */
    u[1] = (((float)*data->adc[PLAT_CONFIG_BUCK_IO_BUFFER]) * ((float)PLAT_CONFIG_BUCK_IO_GAIN)) + ((float)PLAT_CONFIG_BUCK_IO_OFFS);

    /* Assembles state vector */
    xm[0] = (((float)*data->adc[PLAT_CONFIG_BUCK_IL_AVG_BUCK_BUFFER]) * ((float)PLAT_CONFIG_BUCK_IL_AVG_GAIN)) + ((float)PLAT_CONFIG_BUCK_IL_AVG_OFFS);
    xm[1] = ((float)(*data->adc[PLAT_CONFIG_BUCK_V_OUT_BUCK_BUFFER])) * ((float)PLAT_CONFIG_BUCK_V_OUT_BUCK_GAIN);

    xm_1[0] = xm[0];
    xm_1[1] = xm[1];

    /* Delay compensation */
    dmpcDelayComp(xm, xm, u);

    /* Optimization */
    dmpcOpt(xm, xm_1, r, u, &iters, du);

    /* Computes u = du + u_1 */
    u[0] = du[0] + u[0];

    return u[0] / 20.0f;
}
//---------------------------------------------------------------------------
//===========================================================================
