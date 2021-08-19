/*
 * control.c
 *
 *  Created on: 22.07.2021
 *      Author: mguerreiro
 */

//===========================================================================
/*-------------------------------- Includes -------------------------------*/
//===========================================================================
#include "control.h"

#include "openloop.h"
#include "pid.h"
#include "sfb.h"

// Matlab Interface
#include "MatlabInterface.h"

#include "dmpc.h"
//===========================================================================

//===========================================================================
/*------------------------------- Prototypes ------------------------------*/
//===========================================================================

//===========================================================================

//===========================================================================
/*-------------------------------- Globals --------------------------------*/
//===========================================================================
controlMode_t controlMode[CONTROL_MODE_END];

openloop_t openloop;
pid_t pid;
sfb_t sfb;
dmpc_t dmpc;
//===========================================================================

//===========================================================================
/*------------------------------- Functions -------------------------------*/
//===========================================================================
//---------------------------------------------------------------------------
void controlInitialize(void){

    controlMode[CONTROL_MODE_OL].run = openloopControl;
    controlMode[CONTROL_MODE_OL].controller = (void *)&openloop;

    controlMode[CONTROL_MODE_PID].run = pidControl;
    controlMode[CONTROL_MODE_PID].controller = (void *)&pid;

    controlMode[CONTROL_MODE_SFB].run = sfbControl;
    controlMode[CONTROL_MODE_SFB].controller = (void *)&sfb;

    controlMode[CONTROL_MODE_MATLAB].run = matlabControl;
    controlMode[CONTROL_MODE_MATLAB].controller = 0;
    
    controlMode[CONTROL_MODE_DMPC].run = dmpcControl;
    controlMode[CONTROL_MODE_DMPC].controller = (void *)&dmpc;
}
//---------------------------------------------------------------------------
uint32_t controlSet(controlModeEnum_t mode, uint32_t *p){

    if( mode == CONTROL_MODE_OL ){
        float dc;

        dc = *((float *)(p));

        openloopInitialize(&openloop, dc);
    }

    else if( mode == CONTROL_MODE_PID ){
        float a1, a2, b0, b1, b2;

        a1 = *((float *)(&p[0]));
        a2 = *((float *)(&p[1]));
        b0 = *((float *)(&p[2]));
        b1 = *((float *)(&p[3]));
        b2 = *((float *)(&p[4]));

        pidInitialize(&pid, a1, a2, b0, b1, b2);
    }

    else if( mode == CONTROL_MODE_SFB ){
        sfbInitialize(&sfb, p);
    }

    else if( mode == CONTROL_MODE_MATLAB){
        matlabInitialize();
    }

    else if( mode == CONTROL_MODE_DMPC ){
        dmpcInitialize(&dmpc, p);
    }
    
    else{
        return 1;
    }

    return 0;
}
//---------------------------------------------------------------------------
float controlControl(controlModeEnum_t mode, uint16_t ref, platCPU2ControlData_t *data){

    float u;

    u = controlMode[mode].run(controlMode[mode].controller, ref, data);

    return u;
}
//---------------------------------------------------------------------------
uint32_t controlDMPCIters(void){

    return dmpc.iters;
}
//---------------------------------------------------------------------------
//===========================================================================

//===========================================================================
/*--------------------------- Static functions ----------------------------*/
//===========================================================================
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
//===========================================================================
