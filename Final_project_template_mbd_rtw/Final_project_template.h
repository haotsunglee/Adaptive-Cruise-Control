/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: Final_project_template.h
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

#ifndef RTW_HEADER_Final_project_template_h_
#define RTW_HEADER_Final_project_template_h_
#include <string.h>
#ifndef Final_project_template_COMMON_INCLUDES_
# define Final_project_template_COMMON_INCLUDES_
#include <string.h>
#include "rtwtypes.h"
#include "zero_crossing_types.h"
#include "pcc_hw_access.h"
#include "pins_driver.h"
#include "ftm_qd_driver.h"
#include "clock_manager.h"
#include "ftm_hw_access.h"
#include "pins_port_hw_access.h"
#include "adc_driver.h"
#include "flexcan_hw_access.h"
#include "ftm_pwm_driver.h"
#include "ftm3_pwm_params.h"
#include "ftm0_pwm_params.h"
#include "lpuart_driver.h"
#include "device_registers.h"
#include "flexcan_driver.h"
#include "lin_lpuart_driver.h"
#include "lpuart_hw_access.h"
#include "interrupt_manager.h"
#include "fcan0_s32k_rx_isr.h"
#include "ftm2_qd_params_config.h"
#endif                             /* Final_project_template_COMMON_INCLUDES_ */

#include "Final_project_template_types.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

#ifndef rtmStepTask
# define rtmStepTask(rtm, idx)         ((rtm)->Timing.TaskCounters.TID[(idx)] == 0)
#endif

#ifndef rtmTaskCounter
# define rtmTaskCounter(rtm, idx)      ((rtm)->Timing.TaskCounters.TID[(idx)])
#endif

#define Final_project_template_M       (rtM)

/* user code (top of header file) */
#include <math.h>

/* Block states (default storage) for system '<S6>/Switch Case Action Subsystem2' */
typedef struct {
  int_T ByteUnpacking_IWORK[4];        /* '<S12>/Byte Unpacking ' */
} DW_SwitchCaseActionSubsystem2;

/* Block states (default storage) for system '<S6>/Switch Case Action Subsystem3' */
typedef struct {
  int_T ByteUnpacking_IWORK[4];        /* '<S13>/Byte Unpacking ' */
} DW_SwitchCaseActionSubsystem3;

/* Block signals (default storage) */
typedef struct {
  uint32_T FCAN_Isr_o3;                /* '<S1>/FCAN_Isr' */
  uint32_T ADC_Start;                  /* '<S39>/ADC_Start' */
  real32_T FTM_PWM_Config;             /* '<Root>/FTM_PWM_Config' */
  real32_T RateTransition[22];         /* '<Root>/Rate Transition' */
  real32_T UnitDelay;                  /* '<S65>/Unit Delay' */
  real32_T LookupP_o1;                 /* '<S62>/Look up P' */
  real32_T LookupP_o2;                 /* '<S62>/Look up P' */
  real32_T LookupRightVector_o1;       /* '<S62>/Look up Right Vector' */
  real32_T LookupRightVector_o2;       /* '<S62>/Look up Right Vector' */
  real32_T DiscreteTimeIntegrator3;    /* '<S28>/Discrete-Time Integrator3' */
  real32_T LookupRightVector_o1_l;     /* '<S60>/Look up Right Vector' */
  real32_T LookupRightVector_o2_n;     /* '<S60>/Look up Right Vector' */
  real32_T Gain;                       /* '<S60>/Gain' */
  real32_T u;                          /* '<S28>/Discrete-Time Integrator' */
  real32_T Gain_c;                     /* '<S26>/Gain' */
  real32_T calculateus;                /* '<S28>/calculate us' */
  real32_T snpsiusCars16[24];          /* '<S2>/s,n,psi,us (Cars 1-6)' */
  real32_T TmpSignalConversionAtPick_LeadI[7];/* '<S29>/Subsystem' */
  real32_T TmpSignalConversionAtPick_Lea_o[7];/* '<S29>/Subsystem2' */
  real32_T Pick_Lead_o1;               /* '<S29>/Pick_Lead' */
  real32_T Pick_Lead_o2;               /* '<S29>/Pick_Lead' */
  real32_T Merge;                      /* '<S24>/Merge' */
  real32_T LookupRightVector1_o1;      /* '<S46>/Look up Right Vector1' */
  real32_T LookupRightVector1_o2;      /* '<S46>/Look up Right Vector1' */
  real32_T LookupP1_o1;                /* '<S46>/Look up P1' */
  real32_T LookupP1_o2;                /* '<S46>/Look up P1' */
  real32_T LookupRightVector1_o1_j;    /* '<S53>/Look up Right Vector1' */
  real32_T LookupRightVector1_o2_m;    /* '<S53>/Look up Right Vector1' */
  real32_T LookupP1_o1_j;              /* '<S53>/Look up P1' */
  real32_T LookupP1_o2_m;              /* '<S53>/Look up P1' */
  real32_T LookupRightVector1_o1_i;    /* '<S54>/Look up Right Vector1' */
  real32_T LookupRightVector1_o2_d;    /* '<S54>/Look up Right Vector1' */
  real32_T LookupP1_o1_f;              /* '<S54>/Look up P1' */
  real32_T LookupP1_o2_p;              /* '<S54>/Look up P1' */
  real32_T LookupRightVector1_o1_a;    /* '<S55>/Look up Right Vector1' */
  real32_T LookupRightVector1_o2_j;    /* '<S55>/Look up Right Vector1' */
  real32_T LookupP1_o1_jz;             /* '<S55>/Look up P1' */
  real32_T LookupP1_o2_pi;             /* '<S55>/Look up P1' */
  real32_T LookupRightVector1_o1_c;    /* '<S56>/Look up Right Vector1' */
  real32_T LookupRightVector1_o2_e;    /* '<S56>/Look up Right Vector1' */
  real32_T LookupP1_o1_a;              /* '<S56>/Look up P1' */
  real32_T LookupP1_o2_l;              /* '<S56>/Look up P1' */
  real32_T LookupRightVector1_o1_iv;   /* '<S57>/Look up Right Vector1' */
  real32_T LookupRightVector1_o2_f;    /* '<S57>/Look up Right Vector1' */
  real32_T LookupP1_o1_c;              /* '<S57>/Look up P1' */
  real32_T LookupP1_o2_e;              /* '<S57>/Look up P1' */
  real32_T LookupRightVector1_o1_je;   /* '<S58>/Look up Right Vector1' */
  real32_T LookupRightVector1_o2_b;    /* '<S58>/Look up Right Vector1' */
  real32_T LookupP1_o1_g;              /* '<S58>/Look up P1' */
  real32_T LookupP1_o2_n;              /* '<S58>/Look up P1' */
  real32_T TmpSignalConversionAtBytePackin[2];
  real32_T TmpSignalConversionAtBytePack_m[2];
  real32_T Saturation;                 /* '<S59>/Saturation' */
  real32_T FTM_PWM_Config_o1;          /* '<S45>/FTM_PWM_Config' */
  real32_T FTM_PWM_Config_o2;          /* '<S45>/FTM_PWM_Config' */
  real32_T SFunctionBuilder_o1;        /* '<S28>/S-Function Builder' */
  real32_T SFunctionBuilder_o2;        /* '<S28>/S-Function Builder' */
  real32_T SFunctionBuilder_o3;        /* '<S28>/S-Function Builder' */
  real32_T LookupRightVector_o1_c;     /* '<S64>/Look up Right Vector' */
  real32_T LookupRightVector_o2_j;     /* '<S64>/Look up Right Vector' */
  real32_T Sum5;                       /* '<S25>/Sum5' */
  real32_T ByteUnpacking_o1;           /* '<S23>/Byte Unpacking ' */
  real32_T ByteUnpacking_o2;           /* '<S23>/Byte Unpacking ' */
  real32_T ByteUnpacking_o1_f;         /* '<S22>/Byte Unpacking ' */
  real32_T ByteUnpacking_o2_n;         /* '<S22>/Byte Unpacking ' */
  real32_T ByteUnpacking_o1_f4;        /* '<S21>/Byte Unpacking ' */
  real32_T ByteUnpacking_o2_m;         /* '<S21>/Byte Unpacking ' */
  real32_T ByteUnpacking_o1_o;         /* '<S20>/Byte Unpacking ' */
  real32_T ByteUnpacking_o2_j;         /* '<S20>/Byte Unpacking ' */
  real32_T ByteUnpacking_o1_c;         /* '<S19>/Byte Unpacking ' */
  real32_T ByteUnpacking_o2_c;         /* '<S19>/Byte Unpacking ' */
  real32_T ByteUnpacking_o1_o0;        /* '<S18>/Byte Unpacking ' */
  real32_T ByteUnpacking_o2_p;         /* '<S18>/Byte Unpacking ' */
  real32_T ByteUnpacking_o1_a;         /* '<S17>/Byte Unpacking ' */
  real32_T ByteUnpacking_o2_j1;        /* '<S17>/Byte Unpacking ' */
  real32_T ByteUnpacking_o1_i;         /* '<S16>/Byte Unpacking ' */
  real32_T ByteUnpacking_o2_i;         /* '<S16>/Byte Unpacking ' */
  real32_T ByteUnpacking_o1_az;        /* '<S15>/Byte Unpacking ' */
  real32_T ByteUnpacking_o2_b;         /* '<S15>/Byte Unpacking ' */
  real32_T ByteUnpacking_o1_o3;        /* '<S14>/Byte Unpacking ' */
  real32_T ByteUnpacking_o2_h;         /* '<S14>/Byte Unpacking ' */
  real32_T ByteUnpacking_o1_b;         /* '<S13>/Byte Unpacking ' */
  real32_T ByteUnpacking_o2_nz;        /* '<S13>/Byte Unpacking ' */
  real32_T ByteUnpacking_o1_m;         /* '<S12>/Byte Unpacking ' */
  real32_T ByteUnpacking_o2_k;         /* '<S12>/Byte Unpacking ' */
  uint16_T FCAN_Isr_o6;                /* '<S1>/FCAN_Isr' */
  uint16_T Quadrature_Decoder_o1;      /* '<S40>/Quadrature_Decoder' */
  uint8_T RT1[90];                     /* '<S3>/RT1' */
  uint8_T FCAN_Isr_o2;                 /* '<S1>/FCAN_Isr' */
  uint8_T FCAN_Isr_o4[8];              /* '<S1>/FCAN_Isr' */
  uint8_T FCAN_Isr_o5;                 /* '<S1>/FCAN_Isr' */
  uint8_T IndexVector;                 /* '<S69>/Index Vector' */
  uint8_T BytePacking[88];             /* '<S68>/Byte Packing ' */
  uint8_T BytePacking_b[8];            /* '<S44>/Byte Packing ' */
  uint8_T BytePacking1[8];             /* '<S44>/Byte Packing 1' */
  boolean_T RT_k;                      /* '<S3>/RT' */
  boolean_T Digital_Input;             /* '<S26>/Digital_Input' */
  boolean_T Quadrature_Decoder_o2;     /* '<S40>/Quadrature_Decoder' */
  boolean_T Quadrature_Decoder_o3;     /* '<S40>/Quadrature_Decoder' */
  boolean_T Quadrature_Decoder_o4;     /* '<S40>/Quadrature_Decoder' */
  boolean_T Digital_Input_g;           /* '<S38>/Digital_Input' */
  boolean_T Digital_Input1;            /* '<S38>/Digital_Input1' */
  boolean_T Digital_Input2;            /* '<S38>/Digital_Input2' */
  boolean_T Digital_Input3;            /* '<S38>/Digital_Input3' */
  boolean_T Digital_Input4;            /* '<S38>/Digital_Input4' */
  boolean_T Digital_Input5;            /* '<S38>/Digital_Input5' */
  boolean_T Digital_Input6;            /* '<S38>/Digital_Input6' */
  boolean_T Digital_Input7;            /* '<S38>/Digital_Input7' */
  boolean_T Digital_Input1_b;          /* '<S26>/Digital_Input1' */
} B;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real_T Delay_DSTATE;                 /* '<S70>/Delay' */
  real_T Controller_DSTATE;            /* '<S62>/Controller' */
  real32_T UnitDelay_DSTATE;           /* '<S65>/Unit Delay' */
  real32_T DiscreteTimeIntegrator1_DSTATE;/* '<S28>/Discrete-Time Integrator1' */
  real32_T DiscreteTimeIntegrator2_DSTATE;/* '<S28>/Discrete-Time Integrator2' */
  real32_T DiscreteTimeIntegrator3_DSTATE;/* '<S28>/Discrete-Time Integrator3' */
  real32_T DiscreteTimeIntegrator_DSTATE;/* '<S28>/Discrete-Time Integrator' */
  real32_T Delay1_DSTATE;              /* '<S41>/Delay1' */
  real32_T UnitDelay_DSTATE_j;         /* '<S25>/Unit Delay' */
  real32_T UnitDelay1_DSTATE;          /* '<S25>/Unit Delay1' */
  real32_T UnitDelay_DSTATE_n;         /* '<S37>/Unit Delay' */
  real32_T RateTransition_Buffer[22];  /* '<Root>/Rate Transition' */
  real32_T DiscreteTimeIntegrator1_PREV_U;/* '<S28>/Discrete-Time Integrator1' */
  real32_T DiscreteTimeIntegrator2_PREV_U;/* '<S28>/Discrete-Time Integrator2' */
  real32_T DiscreteTimeIntegrator3_PREV_U;/* '<S28>/Discrete-Time Integrator3' */
  real32_T DiscreteTimeIntegrator_PREV_U;/* '<S28>/Discrete-Time Integrator' */
  uint32_T HighLevelDesign_PREV_T;     /* '<Root>/High Level Design' */
  int_T BytePacking_IWORK[2];          /* '<S68>/Byte Packing ' */
  int_T BytePacking_IWORK_m[2];        /* '<S44>/Byte Packing ' */
  int_T BytePacking1_IWORK[2];         /* '<S44>/Byte Packing 1' */
  uint16_T Delay_DSTATE_h;             /* '<S41>/Delay' */
  boolean_T Delay_DSTATE_j;            /* '<S68>/Delay' */
  uint8_T RT1_Buffer0[90];             /* '<S3>/RT1' */
  uint8_T DiscreteTimeIntegrator1_SYSTEM_;/* '<S28>/Discrete-Time Integrator1' */
  uint8_T DiscreteTimeIntegrator2_SYSTEM_;/* '<S28>/Discrete-Time Integrator2' */
  uint8_T DiscreteTimeIntegrator3_SYSTEM_;/* '<S28>/Discrete-Time Integrator3' */
  uint8_T DiscreteTimeIntegrator_SYSTEM_E;/* '<S28>/Discrete-Time Integrator' */
  uint8_T is_active_c1_Final_project_temp;/* '<S29>/Choose_Mode' */
  uint8_T is_c1_Final_project_template;/* '<S29>/Choose_Mode' */
  boolean_T RT_Buffer0;                /* '<S3>/RT' */
  boolean_T HighLevelDesign_RESET_ELAPS_T;/* '<Root>/High Level Design' */
  DW_SwitchCaseActionSubsystem3 SwitchCaseActionSubsystem3_f;
                                     /* '<S11>/Switch Case Action Subsystem3' */
  DW_SwitchCaseActionSubsystem2 SwitchCaseActionSubsystem2_p;
                                     /* '<S11>/Switch Case Action Subsystem2' */
  DW_SwitchCaseActionSubsystem3 SwitchCaseActionSubsystem3_h;
                                     /* '<S10>/Switch Case Action Subsystem3' */
  DW_SwitchCaseActionSubsystem2 SwitchCaseActionSubsystem2_m0;
                                     /* '<S10>/Switch Case Action Subsystem2' */
  DW_SwitchCaseActionSubsystem3 SwitchCaseActionSubsystem3_o;
                                      /* '<S9>/Switch Case Action Subsystem3' */
  DW_SwitchCaseActionSubsystem2 SwitchCaseActionSubsystem2_e;
                                      /* '<S9>/Switch Case Action Subsystem2' */
  DW_SwitchCaseActionSubsystem3 SwitchCaseActionSubsystem3_l;
                                      /* '<S8>/Switch Case Action Subsystem3' */
  DW_SwitchCaseActionSubsystem2 SwitchCaseActionSubsystem2_c;
                                      /* '<S8>/Switch Case Action Subsystem2' */
  DW_SwitchCaseActionSubsystem3 SwitchCaseActionSubsystem3_n;
                                      /* '<S7>/Switch Case Action Subsystem3' */
  DW_SwitchCaseActionSubsystem2 SwitchCaseActionSubsystem2_m;
                                      /* '<S7>/Switch Case Action Subsystem2' */
  DW_SwitchCaseActionSubsystem3 SwitchCaseActionSubsystem3_d;
                                      /* '<S6>/Switch Case Action Subsystem3' */
  DW_SwitchCaseActionSubsystem2 SwitchCaseActionSubsystem2_a;
                                      /* '<S6>/Switch Case Action Subsystem2' */
} DW;

/* Zero-crossing (trigger) state */
typedef struct {
  ZCSigState ResettableSubsystem_Reset_ZCE;/* '<S69>/Resettable Subsystem' */
} PrevZCX;

/* Constant parameters (default storage) */
typedef struct {
  /* Computed Parameter: Constant_Value
   * Referenced by: '<S28>/Constant'
   */
  real32_T Constant_Value;

  /* Computed Parameter: Constant1_Value
   * Referenced by: '<S28>/Constant1'
   */
  real32_T Constant1_Value;
} ConstP;

/* Real-time Model Data Structure */
struct tag_RTM {
  const char_T *errorStatus;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    uint32_T clockTick1;
    struct {
      uint8_T TID[3];
    } TaskCounters;

    struct {
      boolean_T TID0_2;
      boolean_T TID1_2;
    } RateInteraction;
  } Timing;
};

/* Block signals (default storage) */
extern B rtB;

/* Block states (default storage) */
extern DW rtDW;

/* Zero-crossing (trigger) state */
extern PrevZCX rtPrevZCX;

/* Constant parameters (default storage) */
extern const ConstP rtConstP;

/* External function called from main */
extern void Final_project_template_SetEventsForThisBaseStep(boolean_T
  *eventFlags);

/* Model entry point functions */
extern void Final_project_template_SetEventsForThisBaseStep(boolean_T
  *eventFlags);
extern void Final_project_template_initialize(void);
extern void Final_project_template_step(int_T tid);
extern void Final_project_template_terminate(void);

/* Real-time Model object */
extern RT_MODEL *const rtM;

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S63>/Data Type Duplicate' : Unused code path elimination
 * Block '<S38>/Shift Arithmetic' : Eliminated trivial shift
 * Block '<S39>/pot_gain' : Eliminated nontunable gain of 1
 * Block '<S41>/Data Type Conversion' : Eliminate redundant data type conversion
 * Block '<S45>/Data Type Conversion' : Eliminate redundant data type conversion
 */

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
 * '<Root>' : 'Final_project_template'
 * '<S1>'   : 'Final_project_template/Can Network Inputs'
 * '<S2>'   : 'Final_project_template/High Level Design'
 * '<S3>'   : 'Final_project_template/Serial Sim Out to Virtual Sim'
 * '<S4>'   : 'Final_project_template/Can Network Inputs/Can Message Mailbox Setup'
 * '<S5>'   : 'Final_project_template/Can Network Inputs/Extract CAN data'
 * '<S6>'   : 'Final_project_template/Can Network Inputs/Extract CAN data/Data to s,n,psi,us '
 * '<S7>'   : 'Final_project_template/Can Network Inputs/Extract CAN data/Data to s,n,psi,us 1'
 * '<S8>'   : 'Final_project_template/Can Network Inputs/Extract CAN data/Data to s,n,psi,us 2'
 * '<S9>'   : 'Final_project_template/Can Network Inputs/Extract CAN data/Data to s,n,psi,us 3'
 * '<S10>'  : 'Final_project_template/Can Network Inputs/Extract CAN data/Data to s,n,psi,us 4'
 * '<S11>'  : 'Final_project_template/Can Network Inputs/Extract CAN data/Data to s,n,psi,us 5'
 * '<S12>'  : 'Final_project_template/Can Network Inputs/Extract CAN data/Data to s,n,psi,us /Switch Case Action Subsystem2'
 * '<S13>'  : 'Final_project_template/Can Network Inputs/Extract CAN data/Data to s,n,psi,us /Switch Case Action Subsystem3'
 * '<S14>'  : 'Final_project_template/Can Network Inputs/Extract CAN data/Data to s,n,psi,us 1/Switch Case Action Subsystem2'
 * '<S15>'  : 'Final_project_template/Can Network Inputs/Extract CAN data/Data to s,n,psi,us 1/Switch Case Action Subsystem3'
 * '<S16>'  : 'Final_project_template/Can Network Inputs/Extract CAN data/Data to s,n,psi,us 2/Switch Case Action Subsystem2'
 * '<S17>'  : 'Final_project_template/Can Network Inputs/Extract CAN data/Data to s,n,psi,us 2/Switch Case Action Subsystem3'
 * '<S18>'  : 'Final_project_template/Can Network Inputs/Extract CAN data/Data to s,n,psi,us 3/Switch Case Action Subsystem2'
 * '<S19>'  : 'Final_project_template/Can Network Inputs/Extract CAN data/Data to s,n,psi,us 3/Switch Case Action Subsystem3'
 * '<S20>'  : 'Final_project_template/Can Network Inputs/Extract CAN data/Data to s,n,psi,us 4/Switch Case Action Subsystem2'
 * '<S21>'  : 'Final_project_template/Can Network Inputs/Extract CAN data/Data to s,n,psi,us 4/Switch Case Action Subsystem3'
 * '<S22>'  : 'Final_project_template/Can Network Inputs/Extract CAN data/Data to s,n,psi,us 5/Switch Case Action Subsystem2'
 * '<S23>'  : 'Final_project_template/Can Network Inputs/Extract CAN data/Data to s,n,psi,us 5/Switch Case Action Subsystem3'
 * '<S24>'  : 'Final_project_template/High Level Design/ACC'
 * '<S25>'  : 'Final_project_template/High Level Design/Auto Steering'
 * '<S26>'  : 'Final_project_template/High Level Design/Inputs'
 * '<S27>'  : 'Final_project_template/High Level Design/Outputs'
 * '<S28>'  : 'Final_project_template/High Level Design/Vehicle Dynamics'
 * '<S29>'  : 'Final_project_template/High Level Design/ACC/ACC State Logic'
 * '<S30>'  : 'Final_project_template/High Level Design/ACC/Manual Ctrl'
 * '<S31>'  : 'Final_project_template/High Level Design/ACC/Position Ctrl'
 * '<S32>'  : 'Final_project_template/High Level Design/ACC/Speed Ctrl'
 * '<S33>'  : 'Final_project_template/High Level Design/ACC/ACC State Logic/Choose_Mode'
 * '<S34>'  : 'Final_project_template/High Level Design/ACC/ACC State Logic/Subsystem'
 * '<S35>'  : 'Final_project_template/High Level Design/ACC/ACC State Logic/Subsystem2'
 * '<S36>'  : 'Final_project_template/High Level Design/ACC/Speed Ctrl/PI'
 * '<S37>'  : 'Final_project_template/High Level Design/ACC/Speed Ctrl/PI/My Discrete Integrator'
 * '<S38>'  : 'Final_project_template/High Level Design/Inputs/Select Speed (GPI 122-129)'
 * '<S39>'  : 'Final_project_template/High Level Design/Inputs/Throttle (Potentiometer)'
 * '<S40>'  : 'Final_project_template/High Level Design/Inputs/get wheel position'
 * '<S41>'  : 'Final_project_template/High Level Design/Inputs/get wheel position/Subsystem'
 * '<S42>'  : 'Final_project_template/High Level Design/Outputs/Extract x,y,psi'
 * '<S43>'  : 'Final_project_template/High Level Design/Outputs/Extract x,y,psi (Cars 1 - 6)'
 * '<S44>'  : 'Final_project_template/High Level Design/Outputs/Tx CAN'
 * '<S45>'  : 'Final_project_template/High Level Design/Outputs/write torque'
 * '<S46>'  : 'Final_project_template/High Level Design/Outputs/Extract x,y,psi/s,n to x,y'
 * '<S47>'  : 'Final_project_template/High Level Design/Outputs/Extract x,y,psi (Cars 1 - 6)/Extract x,y,psi'
 * '<S48>'  : 'Final_project_template/High Level Design/Outputs/Extract x,y,psi (Cars 1 - 6)/Extract x,y,psi1'
 * '<S49>'  : 'Final_project_template/High Level Design/Outputs/Extract x,y,psi (Cars 1 - 6)/Extract x,y,psi2'
 * '<S50>'  : 'Final_project_template/High Level Design/Outputs/Extract x,y,psi (Cars 1 - 6)/Extract x,y,psi3'
 * '<S51>'  : 'Final_project_template/High Level Design/Outputs/Extract x,y,psi (Cars 1 - 6)/Extract x,y,psi4'
 * '<S52>'  : 'Final_project_template/High Level Design/Outputs/Extract x,y,psi (Cars 1 - 6)/Extract x,y,psi5'
 * '<S53>'  : 'Final_project_template/High Level Design/Outputs/Extract x,y,psi (Cars 1 - 6)/Extract x,y,psi/s,n to x,y'
 * '<S54>'  : 'Final_project_template/High Level Design/Outputs/Extract x,y,psi (Cars 1 - 6)/Extract x,y,psi1/s,n to x,y'
 * '<S55>'  : 'Final_project_template/High Level Design/Outputs/Extract x,y,psi (Cars 1 - 6)/Extract x,y,psi2/s,n to x,y'
 * '<S56>'  : 'Final_project_template/High Level Design/Outputs/Extract x,y,psi (Cars 1 - 6)/Extract x,y,psi3/s,n to x,y'
 * '<S57>'  : 'Final_project_template/High Level Design/Outputs/Extract x,y,psi (Cars 1 - 6)/Extract x,y,psi4/s,n to x,y'
 * '<S58>'  : 'Final_project_template/High Level Design/Outputs/Extract x,y,psi (Cars 1 - 6)/Extract x,y,psi5/s,n to x,y'
 * '<S59>'  : 'Final_project_template/High Level Design/Outputs/write torque/Subsystem1'
 * '<S60>'  : 'Final_project_template/High Level Design/Vehicle Dynamics/Look up Forward Vector'
 * '<S61>'  : 'Final_project_template/High Level Design/Vehicle Dynamics/Subsystem'
 * '<S62>'  : 'Final_project_template/High Level Design/Vehicle Dynamics/x,y to s,n'
 * '<S63>'  : 'Final_project_template/High Level Design/Vehicle Dynamics/x,y to s,n/Data Type Conversion Inherited'
 * '<S64>'  : 'Final_project_template/High Level Design/Vehicle Dynamics/x,y to s,n/Look up the Forward Vector'
 * '<S65>'  : 'Final_project_template/High Level Design/Vehicle Dynamics/x,y to s,n/My Discrete Integrator'
 * '<S66>'  : 'Final_project_template/High Level Design/Vehicle Dynamics/x,y to s,n/My Dot Product1'
 * '<S67>'  : 'Final_project_template/High Level Design/Vehicle Dynamics/x,y to s,n/My Dot Product2'
 * '<S68>'  : 'Final_project_template/Serial Sim Out to Virtual Sim/Serial'
 * '<S69>'  : 'Final_project_template/Serial Sim Out to Virtual Sim/UART Iteration'
 * '<S70>'  : 'Final_project_template/Serial Sim Out to Virtual Sim/UART Iteration/Resettable Subsystem'
 */
#endif                                /* RTW_HEADER_Final_project_template_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
