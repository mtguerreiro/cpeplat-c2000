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

tlvsiLCLPredictData_t fcsControl;

//===========================================================================
/*------------------------------- Functions -------------------------------*/
//===========================================================================
//---------------------------------------------------------------------------
void fcsmpctlvsiInitialize(void *fcsmpctlvsit, uint32_t *p){

    float V_dc = 650.0f;

    float ts = 1.0f/40e3;

    float w = 314.1592653589793;

    float Lg = 1.8e-3;
    float Li = 3.4e-3;
    float Cf = 20e-6;

    float w_ii = 1.0f, w_ig = 400.0f, w_vc = 0.49f;

    tlvsiInitializeParams(&fcsControl, Li, Lg, Cf, V_dc, w, ts, w_ii, w_ig, w_vc);
}
//---------------------------------------------------------------------------
float fcsmpctlvsiControl(void *fcsmpctlvsit, uint16_t ref, platCPU2ControlData_t *data){

    float J = 0;

    fcsControl.ii_abc.a = FCSMPC_TLVSI_II_GAIN * ((float)*data->adc[0]) + FCSMPC_TLVSI_II_OFFS;
    fcsControl.ii_abc.b = FCSMPC_TLVSI_II_GAIN * ((float)*data->adc[1]) + FCSMPC_TLVSI_II_OFFS;
    fcsControl.ii_abc.c = FCSMPC_TLVSI_II_GAIN * ((float)*data->adc[2]) + FCSMPC_TLVSI_II_OFFS;

    fcsControl.ig_abc.a = FCSMPC_TLVSI_IG_GAIN * ((float)*data->adc[3]) + FCSMPC_TLVSI_IG_OFFS;
    fcsControl.ig_abc.b = FCSMPC_TLVSI_IG_GAIN * ((float)*data->adc[4]) + FCSMPC_TLVSI_IG_OFFS;
    fcsControl.ig_abc.c = FCSMPC_TLVSI_IG_GAIN * ((float)*data->adc[5]) + FCSMPC_TLVSI_IG_OFFS;

    fcsControl.vc_abc.a = FCSMPC_TLVSI_VC_GAIN * ((float)*data->adc[0]) + FCSMPC_TLVSI_VC_OFFS;
    fcsControl.vc_abc.b = FCSMPC_TLVSI_VC_GAIN * ((float)*data->adc[1]) + FCSMPC_TLVSI_VC_OFFS;
    fcsControl.vc_abc.c = FCSMPC_TLVSI_VC_GAIN * ((float)*data->adc[2]) + FCSMPC_TLVSI_VC_OFFS;

    fcsControl.vg_abc.a = FCSMPC_TLVSI_VG_GAIN * ((float)*data->adc[3]) + FCSMPC_TLVSI_VG_OFFS;
    fcsControl.vg_abc.b = FCSMPC_TLVSI_VG_GAIN * ((float)*data->adc[4]) + FCSMPC_TLVSI_VG_OFFS;
    fcsControl.vg_abc.c = FCSMPC_TLVSI_VG_GAIN * ((float)*data->adc[5]) + FCSMPC_TLVSI_VG_OFFS;

//    fcsControl.ii_abc.a = 0.00488471985f;
//    fcsControl.ii_abc.b = 0.00488471985f;
//    fcsControl.ii_abc.c = 0.00488471985f;
//
//    fcsControl.ig_abc.a = 0.00488471985f;
//    fcsControl.ig_abc.b = 0.00488471985f;
//    fcsControl.ig_abc.c = 0.00488471985f;
//
//    fcsControl.vc_abc.a = 0.0488433838f;
//    fcsControl.vc_abc.b = 0.0488433838f;
//    fcsControl.vc_abc.c = 0.0488433838f;
//
//    fcsControl.vg_abc.a = 162.490845f;
//    fcsControl.vg_abc.b = -81.2210007f;
//    fcsControl.vg_abc.c = -81.2210007f;

    J = tlvsiOpt(&fcsControl, &fcsControl.ii_abc, &fcsControl.ig_abc,
                 &fcsControl.vc_abc, &fcsControl.vg_abc);

    return J;
}
//---------------------------------------------------------------------------
float fcsmpctlvsiTheta(void){

    return fcsControl.theta;
}
//---------------------------------------------------------------------------
//===========================================================================
