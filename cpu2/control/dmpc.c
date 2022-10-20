/*
 * dmpc.c
 *
 *  Created on: 02.08.2021
 *      Author: LRS
 */

//===========================================================================
/*------------------------------- Includes --------------------------------*/
//===========================================================================
#include "dmpc.h"

#include "dmpc_buck.h"
//===========================================================================

//===========================================================================
/*------------------------------- Functions -------------------------------*/
//===========================================================================
//---------------------------------------------------------------------------
void dmpcInitialize(void *dmpct, uint32_t *p){

    dmpc_t *dmpc;

    dmpc = (dmpc_t *)dmpct;

    dmpc->x[0] = 0;
    dmpc->x[1] = 0;

    dmpc->x_1[0] = 0;
    dmpc->x_1[1] = 0;

    dmpc->u = 0;
    dmpc->u_1 = 0;
    dmpc->du = 0;

    dmpc->iters = 0;
}
//---------------------------------------------------------------------------
float dmpcControl(void *dmpct, uint16_t ref, platCPU2ControlData_t *data){

    float r;

    static float v_ref_p = 0, v_ref_p_1 = 0;
    static float v_ref_mpc = 0;
    static float vc = 0;
    static float ts = 1/50e3;

    static float kp = 0.0, ki = 5000;
    static float e = 0, e_1 = 0;
    static float int_sat_h = 0.35, int_sat_l = -0.35;

    dmpc_t *dmpc;

    dmpc = (dmpc_t *)dmpct;

    dmpc->x[0] =  data->observer->states[0];
    dmpc->x[1] =  data->observer->states[1];
    dmpc->u_1 = ((float)(*data->u)) * ((float)PLAT_CONFIG_GAIN_CTL);

    r = ((float)ref) * ((float)PLAT_CONFIG_GAIN_REF);

    vc = ((float)(*data->adc[PLAT_CONFIG_BUCK_V_OUT_BUCK_BUFFER])) * ((float)PLAT_CONFIG_BUCK_V_OUT_BUCK_GAIN);

    e = (((float)r) - vc);
    v_ref_p_1 = v_ref_p + kp * e + (ki*ts-kp)*e_1;
    if( v_ref_p_1 > int_sat_h ) v_ref_p_1 = int_sat_h;
    else if( v_ref_p_1 < int_sat_l ) v_ref_p_1 = int_sat_l;

    r = r + v_ref_p_1;

    dmpc->du = dmpcBuckOpt(dmpc->x, dmpc->x_1, r, dmpc->u_1, &dmpc->iters);

    dmpc->x_1[0] = dmpc->x[0];
    dmpc->x_1[1] = dmpc->x[1];

    dmpc->u = dmpc->u_1 + dmpc->du;

    if( dmpc->u > 1 ) dmpc->u = 1;
    else if (dmpc->u < 0 ) dmpc->u = 0;

    dmpc->u_1 = dmpc->u;

    v_ref_p = v_ref_p_1;
    e_1 = e;

    return dmpc->u;
}
//---------------------------------------------------------------------------
//===========================================================================
