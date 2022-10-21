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
#include "dmpc_buck.h"
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

    static float v_ref_p = 0, v_ref_p_1 = 0;
    static float v_ref_mpc = 0;
    static float vc = 0;
    static float ts = 1/50e3;

    static float kp = 0.0, ki = 5000;
    static float e = 0, e_1 = 0;
    static float int_sat_h = 0.35, int_sat_l = -0.35;

    mpc_t *mpc;

    mpc = (mpc_t *)mpct;

    mpc->x[0] =  data->observer->states[0];
    mpc->x[1] =  data->observer->states[1];
    mpc->u_1 = ((float)(*data->u)) * ((float)PLAT_CONFIG_GAIN_CTL);

    r = ((float)ref) * ((float)PLAT_CONFIG_GAIN_REF);

    vc = ((float)(*data->adc[PLAT_CONFIG_BUCK_V_OUT_BUCK_BUFFER])) * ((float)PLAT_CONFIG_BUCK_V_OUT_BUCK_GAIN);

    e = (((float)r) - vc);
    v_ref_p_1 = v_ref_p + kp * e + (ki*ts-kp)*e_1;
    if( v_ref_p_1 > int_sat_h ) v_ref_p_1 = int_sat_h;
    else if( v_ref_p_1 < int_sat_l ) v_ref_p_1 = int_sat_l;

    r = r + v_ref_p_1;

    mpc->du = dmpcBuckOpt(mpc->x, mpc->x_1, r, mpc->u_1, &mpc->iters);

    mpc->x_1[0] = mpc->x[0];
    mpc->x_1[1] = mpc->x[1];

    mpc->u = mpc->u_1 + mpc->du;

    if( mpc->u > 1 ) mpc->u = 1;
    else if ( mpc->u < 0 ) mpc->u = 0;

    mpc->u_1 = mpc->u;

    v_ref_p = v_ref_p_1;
    e_1 = e;

    return mpc->u;
}
//---------------------------------------------------------------------------
//===========================================================================
