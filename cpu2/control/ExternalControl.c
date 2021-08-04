/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: ExternalControl.c
 *
 * Code generated for Simulink model 'ExternalControl'.
 *
 * Model version                  : 1.9
 * Simulink Coder version         : 9.4 (R2020b) 29-Jul-2020
 * C/C++ source code generated on : Wed Aug  4 13:47:55 2021
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
  real32_T rtb_Delay;
  real32_T rtb_Delay2;
  real32_T rtb_Saturation1;
  real32_T rtb_Sum1;

  /* Sum: '<Root>/Sum1' incorporates:
   *  Inport: '<Root>/V_ref'
   *  Inport: '<Root>/Vout'
   */
  rtb_Sum1 = rtU.V_ref - rtU.Vout;

  /* Delay: '<Root>/Delay' */
  rtb_Delay = rtDW.Delay_DSTATE;

  /* Delay: '<Root>/Delay2' */
  rtb_Delay2 = rtDW.Delay2_DSTATE;

  /* Sum: '<Root>/Sum4' incorporates:
   *  Delay: '<Root>/Delay'
   *  Delay: '<Root>/Delay1'
   *  Delay: '<Root>/Delay3'
   *  Gain: '<Root>/Gain_a1'
   *  Gain: '<Root>/Gain_a2'
   *  Gain: '<Root>/Gain_b0'
   *  Gain: '<Root>/Gain_b1'
   *  Gain: '<Root>/Gain_b2'
   *  Sum: '<Root>/Sum'
   *  Sum: '<Root>/Sum3'
   *  Sum: '<Root>/Sum5'
   */
  rtb_Saturation1 = ((rtP.b1 * rtDW.Delay_DSTATE + rtP.b2 * rtDW.Delay1_DSTATE)
                     + rtP.b0 * rtb_Sum1) + (rtP.Gain_a2_Gain *
    rtDW.Delay3_DSTATE + rtP.Gain_a1_Gain * rtb_Delay2);

  /* Saturate: '<Root>/Saturation1' */
  if (rtb_Saturation1 > rtP.Saturation1_UpperSat) {
    rtb_Saturation1 = rtP.Saturation1_UpperSat;
  } else {
    if (rtb_Saturation1 < rtP.Saturation1_LowerSat) {
      rtb_Saturation1 = rtP.Saturation1_LowerSat;
    }
  }

  /* End of Saturate: '<Root>/Saturation1' */

  /* Outport: '<Root>/ControlSignal' */
  rtY.ControlSignal = rtb_Saturation1;

  /* Update for Delay: '<Root>/Delay' */
  rtDW.Delay_DSTATE = rtb_Sum1;

  /* Update for Delay: '<Root>/Delay1' */
  rtDW.Delay1_DSTATE = rtb_Delay;

  /* Update for Delay: '<Root>/Delay3' */
  rtDW.Delay3_DSTATE = rtb_Delay2;

  /* Update for Delay: '<Root>/Delay2' */
  rtDW.Delay2_DSTATE = rtb_Saturation1;
}

/* Model initialize function */
void ExternalControl_initialize(void)
{
  /* InitializeConditions for Delay: '<Root>/Delay' */
  rtDW.Delay_DSTATE = rtP.Delay_InitialCondition;

  /* InitializeConditions for Delay: '<Root>/Delay1' */
  rtDW.Delay1_DSTATE = rtP.Delay1_InitialCondition;

  /* InitializeConditions for Delay: '<Root>/Delay3' */
  rtDW.Delay3_DSTATE = rtP.Delay3_InitialCondition;

  /* InitializeConditions for Delay: '<Root>/Delay2' */
  rtDW.Delay2_DSTATE = rtP.Delay2_InitialCondition;
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
