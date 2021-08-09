/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: ExternalControl.h
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

#ifndef RTW_HEADER_ExternalControl_h_
#define RTW_HEADER_ExternalControl_h_
#include "rtwtypes.h"
#ifndef ExternalControl_COMMON_INCLUDES_
#define ExternalControl_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* ExternalControl_COMMON_INCLUDES_ */

/* Model Code Variants */

/* Macros for accessing real-time model data structure */

/* Block signals and states (default storage) for system '<Root>' */
typedef struct {
  real32_T Delay_DSTATE;               /* '<Root>/Delay' */
  real32_T Delay2_DSTATE;              /* '<Root>/Delay2' */
} DW;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real32_T V_ref;                      /* '<Root>/V_ref' */
  real32_T Vout;                       /* '<Root>/Vout' */
  real32_T Vin;                        /* '<Root>/Vin' */
  real32_T Vin_buck;                   /* '<Root>/Vin_buck' */
  real32_T IL;                         /* '<Root>/IL' */
  real32_T IL_avg;                     /* '<Root>/IL_avg' */
  real32_T Vout_buck;                  /* '<Root>/Vout_buck' */
} ExtU;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real32_T ControlSignal;              /* '<Root>/ControlSignal' */
} ExtY;

/* Parameters (default storage) */
struct P_ {
  real32_T P_in;                       /* Variable: P_in
                                        * Referenced by: '<Root>/Gain3'
                                        */
  real32_T b0;                         /* Variable: b0
                                        * Referenced by: '<Root>/Gain'
                                        */
  real32_T b1;                         /* Variable: b1
                                        * Referenced by: '<Root>/Gain2'
                                        */
  real32_T Delay_InitialCondition; /* Computed Parameter: Delay_InitialCondition
                                    * Referenced by: '<Root>/Delay'
                                    */
  real32_T Delay2_InitialCondition;
                                  /* Computed Parameter: Delay2_InitialCondition
                                   * Referenced by: '<Root>/Delay2'
                                   */
  real32_T Gain1_Gain;                 /* Computed Parameter: Gain1_Gain
                                        * Referenced by: '<Root>/Gain1'
                                        */
  real32_T Saturation_UpperSat;       /* Computed Parameter: Saturation_UpperSat
                                       * Referenced by: '<Root>/Saturation'
                                       */
  real32_T Saturation_LowerSat;       /* Computed Parameter: Saturation_LowerSat
                                       * Referenced by: '<Root>/Saturation'
                                       */
};

/* Parameters (default storage) */
typedef struct P_ P;

/* Block parameters (default storage) */
extern P rtP;

/* Block signals and states (default storage) */
extern DW rtDW;

/* External inputs (root inport signals with default storage) */
extern ExtU rtU;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY rtY;

/* Model entry point functions */
extern void ExternalControl_initialize(void);
extern void ExternalControl_step(void);

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'ExternalControl'
 */
#endif                                 /* RTW_HEADER_ExternalControl_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
