/*
 * control.c
 *
 *  Created on: 22.07.2021
 *      Author: LRS
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

    controlMode[CONTROL_MODE_OL].controller = (void *)&openloop;
    controlMode[CONTROL_MODE_OL].run = openloopControl;
    controlMode[CONTROL_MODE_OL].set = openloopInitialize;

    controlMode[CONTROL_MODE_PID].controller = (void *)&pid;
    controlMode[CONTROL_MODE_PID].run = pidControl;
    controlMode[CONTROL_MODE_PID].set = pidInitialize;

    controlMode[CONTROL_MODE_SFB].controller = (void *)&sfb;
    controlMode[CONTROL_MODE_SFB].run = sfbControl;
    controlMode[CONTROL_MODE_SFB].set = sfbInitialize;

    controlMode[CONTROL_MODE_MATLAB].controller = 0;
    controlMode[CONTROL_MODE_MATLAB].run = matlabControl;
    controlMode[CONTROL_MODE_MATLAB].set = matlabInitialize;
    
    controlMode[CONTROL_MODE_DMPC].controller = (void *)&dmpc;
    controlMode[CONTROL_MODE_DMPC].run = dmpcControl;
    controlMode[CONTROL_MODE_DMPC].set = dmpcInitialize;
}
//---------------------------------------------------------------------------
uint32_t controlSet(controlModeEnum_t mode, uint32_t *p){

    controlMode[mode].set(controlMode[mode].controller, p);

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
