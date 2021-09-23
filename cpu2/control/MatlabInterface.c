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
#include "ExternalControl.h"           /* Model's header file */
#include "rtwtypes.h"
//===========================================================================

//===========================================================================
/*------------------------------- Functions -------------------------------*/
//===========================================================================
//---------------------------------------------------------------------------
void matlabInitialize(void){

    // Initializes the Delay-Values to 0
    ExternalControl_initialize();

}
//---------------------------------------------------------------------------
float matlabControl(void *matlab, uint16_t ref, platCPU2ControlData_t *data){

    float r;
    float u;

 //   sfb_t *sfb;
 //   sfb = (sfb_t *)sfbt;


    // Update Saved ADC Values to ExternalControl Variables
    rtU.V_ref = ((float)ref) * ((float)PLAT_CONFIG_GAIN_REF);

    rtU.Vin = ((float)(*data->adc[0])) * ((float)PLAT_CONFIG_BUCK_V_IN_GAIN);
    rtU.Vin_buck = ((float)(*data->adc[1])) * ((float)PLAT_CONFIG_BUCK_V_IN_BUCK_GAIN);
    rtU.IL = ((float)(*data->adc[2])) * ((float)PLAT_CONFIG_BUCK_IL_GAIN) + ((float)PLAT_CONFIG_BUCK_IL_OFFS);
    rtU.Vout = ((float)(*data->adc[3])) * ((float)PLAT_CONFIG_BUCK_V_OUT_GAIN);
    rtU.IL_avg = ((float)(*data->adc[4])) * ((float)PLAT_CONFIG_BUCK_IL_AVG_GAIN) + ((float)PLAT_CONFIG_BUCK_IL_AVG_OFFS);
    rtU.Vout_buck = ((float)(*data->adc[5])) * ((float)PLAT_CONFIG_BUCK_V_OUT_BUCK_GAIN);


    // Call Embedded Coder C-Code Control Function
    ExternalControl_step();

    u = rtY.ControlSignal;
    return u;
}
//---------------------------------------------------------------------------
//===========================================================================



