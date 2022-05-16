/*
 * fcsmpc_tlvsi.c
 *
 *  Created on: 03.01.2022
 *      Author: mguerreiro
 */

//===========================================================================
/*-------------------------------- Includes -------------------------------*/
//===========================================================================
#include "fcsmpc_tlvsi.h"

#include "tptransforms.h"
//===========================================================================

#define FCSMPC_TLVSI_II_GAIN    40.0f / 4095.0f
#define FCSMPC_TLVSI_II_OFFS    -20.0f

#define FCSMPC_TLVSI_IG_GAIN    40.0f / 4095.0f
#define FCSMPC_TLVSI_IG_OFFS    -20.0f

#define FCSMPC_TLVSI_VC_GAIN    400.0f / 4095.0f
#define FCSMPC_TLVSI_VC_OFFS    -200.0f

#define FCSMPC_TLVSI_VG_GAIN    400.0f / 4095.0f
#define FCSMPC_TLVSI_VG_OFFS    -200.0f

uint32_t fcsmpcsw;
psdtypesABC_t ii, ig, vc, vg;
//psdtypesABCint_t ii, ig, vc, vg;

//===========================================================================
/*------------------------------- Functions -------------------------------*/
//===========================================================================
//---------------------------------------------------------------------------
void fcsmpctlvsiInitialize(void *fcsmpctlvsit, uint32_t *p){

    fcsmpcsw = 0;
}
//---------------------------------------------------------------------------
float fcsmpctlvsiControl(void *fcsmpctlvsit, uint16_t ref, platCPU2ControlData_t *data){

    float J = 0;

    ii.a = FCSMPC_TLVSI_II_GAIN * ((float)*data->adc[0]) + FCSMPC_TLVSI_II_OFFS;
    ii.b = FCSMPC_TLVSI_II_GAIN * ((float)*data->adc[1]) + FCSMPC_TLVSI_II_OFFS;
    ii.c = FCSMPC_TLVSI_II_GAIN * ((float)*data->adc[2]) + FCSMPC_TLVSI_II_OFFS;

    ig.a = FCSMPC_TLVSI_IG_GAIN * ((float)*data->adc[3]) + FCSMPC_TLVSI_IG_OFFS;
    ig.b = FCSMPC_TLVSI_IG_GAIN * ((float)*data->adc[4]) + FCSMPC_TLVSI_IG_OFFS;
    ig.c = FCSMPC_TLVSI_IG_GAIN * ((float)*data->adc[5]) + FCSMPC_TLVSI_IG_OFFS;

    vc.a = FCSMPC_TLVSI_VC_GAIN * ((float)*data->adc[0]) + FCSMPC_TLVSI_VC_OFFS;
    vc.b = FCSMPC_TLVSI_VC_GAIN * ((float)*data->adc[1]) + FCSMPC_TLVSI_VC_OFFS;
    vc.c = FCSMPC_TLVSI_VC_GAIN * ((float)*data->adc[2]) + FCSMPC_TLVSI_VC_OFFS;

    vg.a = FCSMPC_TLVSI_VG_GAIN * ((float)*data->adc[3]) + FCSMPC_TLVSI_VG_OFFS;
    vg.b = FCSMPC_TLVSI_VG_GAIN * ((float)*data->adc[4]) + FCSMPC_TLVSI_VG_OFFS;
    vg.c = FCSMPC_TLVSI_VG_GAIN * ((float)*data->adc[5]) + FCSMPC_TLVSI_VG_OFFS;

    fcsmpcsw = tlvsiOpt(&ii, &ig, &vc, &vg, &J);

    return J;

//    fmint_t J;
//     ii.a = *data->adc[0];
//     ii.b = *data->adc[1];
//     ii.c = *data->adc[2];
//
//     ig.a = *data->adc[3];
//     ig.b = *data->adc[4];
//     ig.c = *data->adc[5];
//
//     vc.a = *data->adc[0];
//     vc.b = *data->adc[1];
//     vc.c = *data->adc[2];
//
//     vg.a = *data->adc[3];
//     vg.b = *data->adc[4];
//     vg.c = *data->adc[5];
//
//    fcsmpcsw = tlvsiOptFixed(&ii, &ig, &vc, &vg, &J);
//    return fixedmathitof(J);
}
//---------------------------------------------------------------------------
uint32_t fcsmpcswRead(void){

    return fcsmpcsw;
}
//---------------------------------------------------------------------------
//===========================================================================
