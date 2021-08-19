/*
 * control.h
 *
 *  Created on: 22.07.2021
 *      Author: LRS
 */

#ifndef CONTROL_H_
#define CONTROL_H_

//===========================================================================
/*-------------------------------- Includes -------------------------------*/
//===========================================================================
#include <stdint.h>

#include "plat_defs.h"
//===========================================================================

//===========================================================================
/*------------------------------- Data types ------------------------------*/
//===========================================================================
typedef enum{
    CONTROL_MODE_OL = PLAT_CPU2_CONTROL_MODE_OL,
    CONTROL_MODE_PID = PLAT_CPU2_CONTROL_MODE_PID,
    CONTROL_MODE_SFB = PLAT_CPU2_CONTROL_MODE_SFB,
    CONTROL_MODE_MATLAB = PLAT_CPU2_CONTROL_MODE_MATLAB,
    CONTROL_MODE_DMPC = PLAT_CPU2_CONTROL_MODE_DMPC,
    CONTROL_MODE_END = PLAT_CPU2_CONTROL_MODE_END
}controlModeEnum_t;

//---------------------------------------------------------------------------
//typedef void (*controlSet)(void *controller, void *p);
//---------------------------------------------------------------------------
typedef float (*controlRun)(void *controller, uint16_t ref, platCPU2ControlData_t *data);
//---------------------------------------------------------------------------
typedef struct{

    void *controller;
//
//    controlSet set;

    controlRun run;

}controlMode_t;
//---------------------------------------------------------------------------
//===========================================================================

//===========================================================================
/*------------------------------- Functions -------------------------------*/
//===========================================================================
void controlInitialize(void);
float controlControl(controlModeEnum_t mode, uint16_t r, platCPU2ControlData_t *data);
uint32_t controlSet(controlModeEnum_t mode, uint32_t *p);
uint32_t controlDMPCIters(void);
//===========================================================================

#endif /* CONTROL_H_ */
