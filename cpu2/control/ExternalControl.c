/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: ExternalControl.c
 *
 * Code generated for Simulink model 'ExternalControl'.
 *
 * Model version                  : 1.10
 * Simulink Coder version         : 9.4 (R2020b) 29-Jul-2020
 * C/C++ source code generated on : Mon Aug  9 15:32:52 2021
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Texas Instruments->C2000
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#include "ExternalControl.h"

/* Block signals and states (default storage) */
DW rtDW;

/* External inputs (root inport signals with default storage) */
ExtU rtU;

/* External outputs (root outports fed by signals with default storage) */
ExtY rtY;

/* Model step function */
void ExternalControl_step(void)
{
  real32_T rtb_Sum1;
  real32_T rtb_Sum4;
  real32_T u0;

  /* Sum: '<Root>/Sum1' incorporates:
   *  Inport: '<Root>/V_ref'
   *  Inport: '<Root>/Vout'
   */
  rtb_Sum1 = rtU.V_ref - rtU.Vout;

  /* Sum: '<Root>/Sum4' incorporates:
   *  Delay: '<Root>/Delay'
   *  Delay: '<Root>/Delay2'
   *  Gain: '<Root>/Gain'
   *  Gain: '<Root>/Gain1'
   *  Gain: '<Root>/Gain2'
   *  Sum: '<Root>/Sum3'
   */
  rtb_Sum4 = (rtP.b0 * rtb_Sum1 + rtP.b1 * rtDW.Delay_DSTATE) + rtP.Gain1_Gain *
    rtDW.Delay2_DSTATE;

  /* Gain: '<Root>/Gain3' incorporates:
   *  Inport: '<Root>/IL_avg'
   *  Sum: '<Root>/Sum2'
   */
  u0 = (rtb_Sum4 - rtU.IL_avg) * rtP.P_in;

  /* Saturate: '<Root>/Saturation' */
  if (u0 > rtP.Saturation_UpperSat) {
    /* Outport: '<Root>/ControlSignal' */
    rtY.ControlSignal = rtP.Saturation_UpperSat;
  } else if (u0 < rtP.Saturation_LowerSat) {
    /* Outport: '<Root>/ControlSignal' */
    rtY.ControlSignal = rtP.Saturation_LowerSat;
  } else {
    /* Outport: '<Root>/ControlSignal' */
    rtY.ControlSignal = u0;
  }

  /* End of Saturate: '<Root>/Saturation' */

  /* Update for Delay: '<Root>/Delay' */
  rtDW.Delay_DSTATE = rtb_Sum1;

  /* Update for Delay: '<Root>/Delay2' */
  rtDW.Delay2_DSTATE = rtb_Sum4;
}

/* Model initialize function */
void ExternalControl_initialize(void)
{
  /* InitializeConditions for Delay: '<Root>/Delay' */
  rtDW.Delay_DSTATE = rtP.Delay_InitialCondition;

  /* InitializeConditions for Delay: '<Root>/Delay2' */
  rtDW.Delay2_DSTATE = rtP.Delay2_InitialCondition;
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
