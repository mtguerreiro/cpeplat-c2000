/*
 * MatlabInterface.c
 *
 *  Created on: 04.08.2021
 *      Author: Moritz Frantz
*/
//===========================================================================
/*------------------------------- Includes --------------------------------*/
//===========================================================================
#include "MatlabInterface.h"

// MATLAB EXTERNAL INCLUDES (from Main file)
#include <stddef.h>
#include <stdio.h>              /* This ert_main.c example uses printf/fflush */
#include "ExternalControl.h"    /* Model's header file */
#include "rtwtypes.h"
//===========================================================================

//===========================================================================
/*------------------------------- Functions -------------------------------*/
//===========================================================================
//---------------------------------------------------------------------------
void matlabInitialize(void *matlabt, uint32_t *p){

    // Initializes the Delay-Values to 0
    ExternalControl_initialize();
}
//---------------------------------------------------------------------------
float matlabControl(void *matlabt, uint16_t ref, platCPU2ControlData_t *data){

    float u;

    // Update Saved ADC Values to ExternalControl Variables
    rtU.V_ref = ((float)ref) * ((float)PLAT_CONFIG_GAIN_REF);

    rtU.Vin = ((float)(*data->adc[PLAT_CONFIG_BUCK_V_IN_BUFFER])) * ((float)PLAT_CONFIG_BUCK_V_IN_GAIN);
    rtU.Vin_buck = ((float)(*data->adc[PLAT_CONFIG_BUCK_V_IN_BUCK_BUFFER])) * ((float)PLAT_CONFIG_BUCK_V_IN_BUCK_GAIN);
    rtU.Vout = ((float)(*data->adc[PLAT_CONFIG_BUCK_V_OUT_BUFFER])) * ((float)PLAT_CONFIG_BUCK_V_OUT_GAIN);
    rtU.Vout_buck = ((float)(*data->adc[PLAT_CONFIG_BUCK_V_OUT_BUCK_BUFFER])) * ((float)PLAT_CONFIG_BUCK_V_OUT_BUCK_GAIN);
    rtU.IL = ((float)(*data->adc[PLAT_CONFIG_BUCK_IL_BUFFER])) * ((float)PLAT_CONFIG_BUCK_IL_GAIN) + ((float)PLAT_CONFIG_BUCK_IL_OFFS);
    rtU.IL_avg = ((float)(*data->adc[PLAT_CONFIG_BUCK_IL_AVG_BUCK_BUFFER])) * ((float)PLAT_CONFIG_BUCK_IL_AVG_GAIN) + ((float)PLAT_CONFIG_BUCK_IL_AVG_OFFS);

    // Call Embedded Coder C-Code Control Function
    ExternalControl_step();

    u = rtY.ControlSignal;

    return u;
}
//---------------------------------------------------------------------------
//===========================================================================
