/*
 * observer.c
 *
 *  Created on: 30.07.2021
 *      Author: LRS
 */

//===========================================================================
/*-------------------------------- Includes -------------------------------*/
//===========================================================================
#include "observer.h"

#include "obscimini.h"
//===========================================================================

//===========================================================================
/*-------------------------------- Globals --------------------------------*/
//===========================================================================
observerMode_t observerMode[OBSERVER_MODE_END];

obscimini_t cimini;
//===========================================================================

//===========================================================================
/*------------------------------- Functions -------------------------------*/
//===========================================================================
//---------------------------------------------------------------------------
void observerInitialize(void){

    observerMode[OBSERVER_MODE_CIMINI].run = obsciminiObserve;
    observerMode[OBSERVER_MODE_CIMINI].observer = (void *)&cimini;
}
//---------------------------------------------------------------------------
uint32_t observerSet(observerModeEnum_t mode, uint32_t *p){

    if( mode == OBSERVER_MODE_CIMINI ){

        obsciminiInitialize(&cimini, p);
    }

    else{
        return 1;
    }


    return 0;
}
//---------------------------------------------------------------------------
void observerObserve(observerModeEnum_t mode, platCPU2ObserverData_t *data){

    observerMode[mode].run(observerMode[mode].observer, data);
}
//---------------------------------------------------------------------------
//===========================================================================
