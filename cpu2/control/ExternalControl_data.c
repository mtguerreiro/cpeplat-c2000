/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: ExternalControl_data.c
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

/* Block parameters (default storage) */
P rtP = {
  /* Variable: P_in
   * Referenced by: '<Root>/Gain3'
   */
  0.05F,

  /* Variable: b0
   * Referenced by: '<Root>/Gain'
   */
  1.4137F,

  /* Variable: b1
   * Referenced by: '<Root>/Gain2'
   */
  -1.3862F,

  /* Computed Parameter: Delay_InitialCondition
   * Referenced by: '<Root>/Delay'
   */
  0.0F,

  /* Computed Parameter: Delay2_InitialCondition
   * Referenced by: '<Root>/Delay2'
   */
  0.0F,

  /* Computed Parameter: Gain1_Gain
   * Referenced by: '<Root>/Gain1'
   */
  1.0F,

  /* Computed Parameter: Saturation_UpperSat
   * Referenced by: '<Root>/Saturation'
   */
  1.0F,

  /* Computed Parameter: Saturation_LowerSat
   * Referenced by: '<Root>/Saturation'
   */
  0.0F
};

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
