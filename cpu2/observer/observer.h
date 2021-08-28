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
    OBSERVER_MODE_LUENBERGER = PLAT_CPU2_OBSERVER_MODE_LUENBERGER,
    OBSERVER_MODE_CIMINI = PLAT_CPU2_OBSERVER_MODE_CIMINI,
    OBSERVER_MODE_PREDICTIVE = PLAT_CPU2_OBSERVER_PREDICTIVE,
    OBSERVER_MODE_END = PLAT_CPU2_OBSERVER_MODE_END
}observerModeEnum_t;

typedef struct{
    float il;
    float vc;
}observerStates_t;
//---------------------------------------------------------------------------
typedef void (*observerSet_t)(void *observer, uint32_t *p);
//---------------------------------------------------------------------------
typedef void (*observerRun_t)(void *observer, platCPU2ObserverData_t *data);
//---------------------------------------------------------------------------
typedef struct{

    void *observer;

    observerSet_t set;
    observerRun_t run;

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
