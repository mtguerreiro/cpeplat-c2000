/*
 * observer.h
 *
 *  Created on: 30.07.2021
 *      Author: LRS
 */

#ifndef OBSERVER_OBSERVER_H_
#define OBSERVER_OBSERVER_H_

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
    OBSERVER_MODE_CIMINI = PLAT_CPU2_OBSERVER_MODE_CIMINI,
    OBSERVER_MODE_LUENBERGER = PLAT_CPU2_OBSERVER_MODE_LUENBERGER,
    OBSERVER_MODE_END = PLAT_CPU2_OBSERVER_MODE_END
}observerModeEnum_t;

typedef struct{
    float il;
    float vc;
}observerStates_t;
//---------------------------------------------------------------------------
//typedef void (*controlSet)(void *controller, void *p);
//---------------------------------------------------------------------------
typedef void (*observerRun)(void *observer, platCPU2ObserverData_t *data);
//---------------------------------------------------------------------------
typedef struct{

    void *observer;

    observerRun run;

}observerMode_t;
//---------------------------------------------------------------------------
//===========================================================================

//===========================================================================
/*------------------------------- Functions -------------------------------*/
//===========================================================================
void observerInitialize(void);
void observerObserve(observerModeEnum_t mode, platCPU2ObserverData_t *data);
uint32_t observerSet(observerModeEnum_t mode, uint32_t *p);
//===========================================================================

#endif /* OBSERVER_OBSERVER_H_ */
