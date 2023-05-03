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

    dmpc_t *dmpc;

    dmpc = (dmpc_t *)dmpct;

    r = ((float)ref) * ((float)PLAT_CONFIG_GAIN_REF);

    dmpc->x[0] =  data->observer->states[0];
    dmpc->x[1] =  data->observer->states[1];
    dmpc->u_1 = ((float)(*data->u)) * ((float)PLAT_CONFIG_GAIN_CTL);

    //dmpc->du = dmpcBuckOpt(dmpc->x, dmpc->x_1, r, dmpc->u_1, &dmpc->iters);

    dmpc->x_1[0] = dmpc->x[0];
    dmpc->x_1[1] = dmpc->x[1];

    dmpc->u = dmpc->u_1 + dmpc->du;

    if( dmpc->u > 1 ) dmpc->u = 1;
    else if (dmpc->u < 0 ) dmpc->u = 0;

    dmpc->u_1 = dmpc->u;

    return dmpc->u;
}
//---------------------------------------------------------------------------
//===========================================================================
