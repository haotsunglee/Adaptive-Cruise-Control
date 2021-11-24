/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: Final_project_template_private.h
 *
 * Code generated for Simulink model 'Final_project_template'.
 *
 * Model version                  : 1.263
 * Simulink Coder version         : 9.3 (R2020a) 18-Nov-2019
 * C/C++ source code generated on : Thu Apr 15 12:35:10 2021
 *
 * Target selection: mbd_s32k.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef RTW_HEADER_Final_project_template_private_h_
#define RTW_HEADER_Final_project_template_private_h_
#include "rtwtypes.h"
#include "zero_crossing_types.h"
#include "path_data.h"
#include "path_data.h"
#include "Final_project_template.h"
#ifdef __cplusplus

extern "C" {

#endif

  extern void calc_us_Start_wrapper(void);
  extern void calc_us_Outputs_wrapper(const real32_T *fx,
    const real32_T *fy,
    const real32_T *u,
    const real32_T *delta,
    const real32_T *psi,
    real32_T *us);
  extern void calc_us_Terminate_wrapper(void);

#ifdef __cplusplus

}
#endif

#ifdef __cplusplus

extern "C" {

#endif

  extern void Pick_lead_Start_wrapper(void);
  extern void Pick_lead_Outputs_wrapper(const real32_T *s,
    const real32_T *us,
    real32_T *lead_s,
    real32_T *lead_us);
  extern void Pick_lead_Terminate_wrapper(void);

#ifdef __cplusplus

}
#endif

#ifdef __cplusplus

extern "C" {

#endif

  extern void bicycle_model_Start_wrapper(void);
  extern void bicycle_model_Outputs_wrapper(const real32_T *u,
    const real32_T *delta,
    const real32_T *psi,
    const real32_T *L1,
    const real32_T *L2,
    real32_T *x_dot,
    real32_T *y_dot,
    real32_T *psi_dot);
  extern void bicycle_model_Terminate_wrapper(void);

#ifdef __cplusplus

}
#endif

extern flexcan_state_t canCom0_State;
extern flexcan_msgbuff_t canCom0_recvBuff16;
extern flexcan_msgbuff_t canCom0_recvBuff17;
extern flexcan_msgbuff_t canCom0_recvBuff12;
extern flexcan_msgbuff_t canCom0_recvBuff13;
extern flexcan_msgbuff_t canCom0_recvBuff2;
extern flexcan_msgbuff_t canCom0_recvBuff3;
extern flexcan_msgbuff_t canCom0_recvBuff14;
extern flexcan_msgbuff_t canCom0_recvBuff15;
extern flexcan_msgbuff_t canCom0_recvBuff4;
extern flexcan_msgbuff_t canCom0_recvBuff5;
extern flexcan_msgbuff_t canCom0_recvBuff6;
extern flexcan_msgbuff_t canCom0_recvBuff7;
extern flexcan_msgbuff_t canCom0_recvBuff8;
extern flexcan_msgbuff_t canCom0_recvBuff9;
extern flexcan_msgbuff_t canCom0_recvBuff10;
extern flexcan_msgbuff_t canCom0_recvBuff11;

#if defined(__MWERKS__)

double fmod (double x, double y);
double fabs (double);

#endif

extern void SwitchCaseActionSubsystem2(const uint8_T rtu_sn[8], real32_T *rty_s,
  real32_T *rty_n);
extern void SwitchCaseActionSubsystem3(const uint8_T rtu_psius[8], real32_T
  *rty_psi, real32_T *rty_us);
extern void Final_project_template_step0(void);
extern void Final_project_template_step1(void);
extern void Final_project_template_step2(void);

#endif                        /* RTW_HEADER_Final_project_template_private_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
