/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: Final_project_template.c
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

#include "Final_project_template.h"
#include "Final_project_template_private.h"

/* Named constants for Chart: '<S29>/Choose_Mode' */
#define IN_Manual                      ((uint8_T)1U)
#define IN_Position                    ((uint8_T)2U)
#define IN_Velocity                    ((uint8_T)3U)

/* Block signals (default storage) */
B rtB;

/* Block states (default storage) */
DW rtDW;

/* Previous zero-crossings (trigger) states */
PrevZCX rtPrevZCX;

/* Real-time model */
RT_MODEL rtM_;
RT_MODEL *const rtM = &rtM_;
static void rate_monotonic_scheduler(void);
flexcan_state_t canCom0_State;
flexcan_msgbuff_t canCom0_recvBuff16;
flexcan_msgbuff_t canCom0_recvBuff17;
flexcan_msgbuff_t canCom0_recvBuff12;
flexcan_msgbuff_t canCom0_recvBuff13;
flexcan_msgbuff_t canCom0_recvBuff2;
flexcan_msgbuff_t canCom0_recvBuff3;
flexcan_msgbuff_t canCom0_recvBuff14;
flexcan_msgbuff_t canCom0_recvBuff15;
flexcan_msgbuff_t canCom0_recvBuff4;
flexcan_msgbuff_t canCom0_recvBuff5;
flexcan_msgbuff_t canCom0_recvBuff6;
flexcan_msgbuff_t canCom0_recvBuff7;
flexcan_msgbuff_t canCom0_recvBuff8;
flexcan_msgbuff_t canCom0_recvBuff9;
flexcan_msgbuff_t canCom0_recvBuff10;
flexcan_msgbuff_t canCom0_recvBuff11;

/*
 * Set which subrates need to run this base step (base rate always runs).
 * This function must be called prior to calling the model step function
 * in order to "remember" which rates need to run this base step.  The
 * buffering of events allows for overlapping preemption.
 */
void Final_project_template_SetEventsForThisBaseStep(boolean_T *eventFlags)
{
  /* Task runs when its counter is zero, computed via rtmStepTask macro */
  eventFlags[1] = ((boolean_T)rtmStepTask(rtM, 1));
  eventFlags[2] = ((boolean_T)rtmStepTask(rtM, 2));
}

/*
 *   This function updates active task flag for each subrate
 * and rate transition flags for tasks that exchange data.
 * The function assumes rate-monotonic multitasking scheduler.
 * The function must be called at model base rate so that
 * the generated code self-manages all its subrates and rate
 * transition flags.
 */
static void rate_monotonic_scheduler(void)
{
  /* To ensure a deterministic data transfer between two rates,
   * data is transferred at the priority of a fast task and the frequency
   * of the slow task.  The following flags indicate when the data transfer
   * happens.  That is, a rate interaction flag is set true when both rates
   * will run, and false otherwise.
   */

  /* tid 0 shares data with slower tid rate: 2 */
  rtM->Timing.RateInteraction.TID0_2 = (rtM->Timing.TaskCounters.TID[2] == 0);

  /* tid 1 shares data with slower tid rate: 2 */
  if (rtM->Timing.TaskCounters.TID[1] == 0) {
    rtM->Timing.RateInteraction.TID1_2 = (rtM->Timing.TaskCounters.TID[2] == 0);
  }

  /* Compute which subrates run during the next base time step.  Subrates
   * are an integer multiple of the base rate counter.  Therefore, the subtask
   * counter is reset when it reaches its limit (zero means run).
   */
  (rtM->Timing.TaskCounters.TID[1])++;
  if ((rtM->Timing.TaskCounters.TID[1]) > 19) {/* Sample time: [0.01s, 0.0s] */
    rtM->Timing.TaskCounters.TID[1] = 0;
  }

  (rtM->Timing.TaskCounters.TID[2])++;
  if ((rtM->Timing.TaskCounters.TID[2]) > 99) {/* Sample time: [0.05s, 0.0s] */
    rtM->Timing.TaskCounters.TID[2] = 0;
  }
}

/*
 * Output and update for action system:
 *    '<S6>/Switch Case Action Subsystem2'
 *    '<S7>/Switch Case Action Subsystem2'
 *    '<S8>/Switch Case Action Subsystem2'
 *    '<S9>/Switch Case Action Subsystem2'
 *    '<S10>/Switch Case Action Subsystem2'
 *    '<S11>/Switch Case Action Subsystem2'
 */
void SwitchCaseActionSubsystem2(const uint8_T rtu_sn[8], real32_T *rty_s,
  real32_T *rty_n)
{
  /* S-Function (xpcbytepacking): '<S12>/Byte Unpacking ' */

  /* Byte Unpacking: <S12>/Byte Unpacking  */
  (void)memcpy((uint8_T*)rty_s, (uint8_T*)&rtu_sn[0] + 0, 4);
  (void)memcpy((uint8_T*)rty_n, (uint8_T*)&rtu_sn[0] + 4, 4);
}

/*
 * Output and update for action system:
 *    '<S6>/Switch Case Action Subsystem3'
 *    '<S7>/Switch Case Action Subsystem3'
 *    '<S8>/Switch Case Action Subsystem3'
 *    '<S9>/Switch Case Action Subsystem3'
 *    '<S10>/Switch Case Action Subsystem3'
 *    '<S11>/Switch Case Action Subsystem3'
 */
void SwitchCaseActionSubsystem3(const uint8_T rtu_psius[8], real32_T *rty_psi,
  real32_T *rty_us)
{
  /* S-Function (xpcbytepacking): '<S13>/Byte Unpacking ' */

  /* Byte Unpacking: <S13>/Byte Unpacking  */
  (void)memcpy((uint8_T*)rty_psi, (uint8_T*)&rtu_psius[0] + 0, 4);
  (void)memcpy((uint8_T*)rty_us, (uint8_T*)&rtu_psius[0] + 4, 4);
}

/* Model step function for TID0 */
void Final_project_template_step0(void) /* Sample time: [0.0005s, 0.0s] */
{
  {                                    /* Sample time: [0.0005s, 0.0s] */
    rate_monotonic_scheduler();
  }

  /* RateTransition: '<S3>/RT' */
  if (rtM->Timing.RateInteraction.TID0_2) {
    rtB.RT_k = rtDW.RT_Buffer0;

    /* RateTransition: '<S3>/RT1' */
    memcpy(&rtB.RT1[0], &rtDW.RT1_Buffer0[0], 90U * sizeof(uint8_T));
  }

  /* End of RateTransition: '<S3>/RT' */

  /* S-Function (fcncallgen): '<S3>/Function-Call Generator2' incorporates:
   *  SubSystem: '<S3>/UART Iteration'
   */
  /* Outputs for Resettable SubSystem: '<S69>/Resettable Subsystem' incorporates:
   *  ResetPort: '<S70>/Reset'
   */
  if (((rtPrevZCX.ResettableSubsystem_Reset_ZCE == POS_ZCSIG) != (int32_T)
       rtB.RT_k) && (rtPrevZCX.ResettableSubsystem_Reset_ZCE !=
                     UNINITIALIZED_ZCSIG)) {
    /* InitializeConditions for Delay: '<S70>/Delay' */
    rtDW.Delay_DSTATE = 0.0;
  }

  rtPrevZCX.ResettableSubsystem_Reset_ZCE = rtB.RT_k;

  /* Sum: '<S70>/Sum' incorporates:
   *  Constant: '<S70>/Constant'
   *  Delay: '<S70>/Delay'
   */
  rtDW.Delay_DSTATE++;

  /* End of Outputs for SubSystem: '<S69>/Resettable Subsystem' */

  /* MultiPortSwitch: '<S69>/Index Vector' incorporates:
   *  Delay: '<S70>/Delay'
   */
  rtB.IndexVector = rtB.RT1[(int32_T)rtDW.Delay_DSTATE - 1];

  /* S-Function (lpuart_s32k_transmit): '<S69>/LPUART_Transmit' */
  {
    LPUART_DRV_SendData(1, &rtB.IndexVector, 1);
  }

  /* End of Outputs for S-Function (fcncallgen): '<S3>/Function-Call Generator2' */

  /* S-Function (fcan_s32k_receive): '<S4>/FCAN_Receive' */
  FLEXCAN_DRV_Receive(0, 16, &canCom0_recvBuff16);

  /* S-Function (fcan_s32k_receive): '<S4>/FCAN_Receive1' */
  FLEXCAN_DRV_Receive(0, 17, &canCom0_recvBuff17);

  /* S-Function (fcan_s32k_receive): '<S4>/FCAN_Receive10' */
  FLEXCAN_DRV_Receive(0, 12, &canCom0_recvBuff12);

  /* S-Function (fcan_s32k_receive): '<S4>/FCAN_Receive11' */
  FLEXCAN_DRV_Receive(0, 13, &canCom0_recvBuff13);

  /* S-Function (fcan_s32k_receive): '<S4>/FCAN_Receive12' */
  FLEXCAN_DRV_Receive(0, 2, &canCom0_recvBuff2);

  /* S-Function (fcan_s32k_receive): '<S4>/FCAN_Receive13' */
  FLEXCAN_DRV_Receive(0, 3, &canCom0_recvBuff3);

  /* S-Function (fcan_s32k_receive): '<S4>/FCAN_Receive14' */
  FLEXCAN_DRV_Receive(0, 14, &canCom0_recvBuff14);

  /* S-Function (fcan_s32k_receive): '<S4>/FCAN_Receive15' */
  FLEXCAN_DRV_Receive(0, 15, &canCom0_recvBuff15);

  /* S-Function (fcan_s32k_receive): '<S4>/FCAN_Receive2' */
  FLEXCAN_DRV_Receive(0, 4, &canCom0_recvBuff4);

  /* S-Function (fcan_s32k_receive): '<S4>/FCAN_Receive3' */
  FLEXCAN_DRV_Receive(0, 5, &canCom0_recvBuff5);

  /* S-Function (fcan_s32k_receive): '<S4>/FCAN_Receive4' */
  FLEXCAN_DRV_Receive(0, 6, &canCom0_recvBuff6);

  /* S-Function (fcan_s32k_receive): '<S4>/FCAN_Receive5' */
  FLEXCAN_DRV_Receive(0, 7, &canCom0_recvBuff7);

  /* S-Function (fcan_s32k_receive): '<S4>/FCAN_Receive6' */
  FLEXCAN_DRV_Receive(0, 8, &canCom0_recvBuff8);

  /* S-Function (fcan_s32k_receive): '<S4>/FCAN_Receive7' */
  FLEXCAN_DRV_Receive(0, 9, &canCom0_recvBuff9);

  /* S-Function (fcan_s32k_receive): '<S4>/FCAN_Receive8' */
  FLEXCAN_DRV_Receive(0, 10, &canCom0_recvBuff10);

  /* S-Function (fcan_s32k_receive): '<S4>/FCAN_Receive9' */
  FLEXCAN_DRV_Receive(0, 11, &canCom0_recvBuff11);

  /* S-Function (ftm_s32k_pwm_config): '<Root>/FTM_PWM_Config' incorporates:
   *  Constant: '<Root>/Constant'
   */
  {
    uint16_t dutyA = FTM_MAX_DUTY_CYCLE * 0.5F;
    FTM_DRV_UpdatePwmChannel(FTM_PWM3, 0U, FTM_PWM_UPDATE_IN_DUTY_CYCLE, dutyA,
      0, true);
  }

  /* Simulation Outputs */
  rtB.FTM_PWM_Config = (float) 0.5F;
}

/* Model step function for TID1 */
void Final_project_template_step1(void) /* Sample time: [0.01s, 0.0s] */
{
  /* local block i/o variables */
  real_T rtb_DataTypeConversion;
  real_T rtb_dsdt;
  boolean_T rtb_LogicalOperator;
  real32_T rtb_Gain_ie;
  boolean_T rtb_en_man;
  boolean_T rtb_en_pos;
  boolean_T rtb_en_vel;
  real32_T rtb_Sum1_m;
  real32_T rtb_Sum1;
  uint32_T HighLevelDesign_ELAPS_T;
  real32_T rtb_Product_o_idx_0;
  real32_T rtb_Product_o_idx_1;

  /* End of Outputs for S-Function (fcan_s32k_isr): '<S1>/FCAN_Isr' */

  /* S-Function (fcncallgen): '<Root>/Function-Call Top' incorporates:
   *  SubSystem: '<Root>/High Level Design'
   */
  if (rtDW.HighLevelDesign_RESET_ELAPS_T) {
    HighLevelDesign_ELAPS_T = 0U;
  } else {
    HighLevelDesign_ELAPS_T = rtM->Timing.clockTick1 -
      rtDW.HighLevelDesign_PREV_T;
  }

  rtDW.HighLevelDesign_PREV_T = rtM->Timing.clockTick1;
  rtDW.HighLevelDesign_RESET_ELAPS_T = false;

  /* S-Function (gpio_s32k_input): '<S26>/Digital_Input' */

  /* GPIPORTE14 signal update */
  rtB.Digital_Input = (PINS_DRV_ReadPins(PTE) >> 14) & 0x01;

  /* Logic: '<S29>/Logical Operator' */
  rtb_LogicalOperator = !rtB.Digital_Input;

  /* UnitDelay: '<S65>/Unit Delay' */
  rtB.UnitDelay = rtDW.UnitDelay_DSTATE;

  /* DiscreteIntegrator: '<S28>/Discrete-Time Integrator1' */
  if (rtDW.DiscreteTimeIntegrator1_SYSTEM_ == 0) {
    rtDW.DiscreteTimeIntegrator1_DSTATE += 0.01F * (real32_T)
      HighLevelDesign_ELAPS_T * rtDW.DiscreteTimeIntegrator1_PREV_U;
  }

  /* End of DiscreteIntegrator: '<S28>/Discrete-Time Integrator1' */

  /* DiscreteIntegrator: '<S28>/Discrete-Time Integrator2' */
  if (rtDW.DiscreteTimeIntegrator2_SYSTEM_ == 0) {
    rtDW.DiscreteTimeIntegrator2_DSTATE += 0.01F * (real32_T)
      HighLevelDesign_ELAPS_T * rtDW.DiscreteTimeIntegrator2_PREV_U;
  }

  /* End of DiscreteIntegrator: '<S28>/Discrete-Time Integrator2' */

  /* S-Function (get_path1): '<S62>/Look up P' */
  {
    extern struct road_seg_type rs[MAX_RS];
    real_T p, u_alt;
    int i;

    /* keep u within the range [0, PATH_LENGTH) */
    u_alt = fmod( (rtB.UnitDelay), PATH_LENGTH);

    /* Find the segment in rs[] that the path distance u_alt lies within */
    for (i=0; i<MAX_RS; i++) {
      if (rs[i].p > u_alt) {
        i--;
        break;
      }
    }

    /* i is the segment in which u lies; p is the path length from the
     * beginning of segment i to u_alt */
    p = u_alt - rs[i].p;
    switch ( rs[i].type)
    {
     case STRAIGHT:
      /* If the path is straight, interpolate between start and stop */
      rtB.LookupP_o1 = rs[i].data.straight.start[0] +
        (p/rs[i].data.straight.length)*(rs[i].data.straight.stop[0]-rs[i].
        data.straight.start[0]);
      rtB.LookupP_o2 = rs[i].data.straight.start[1] +
        (p/rs[i].data.straight.length)*(rs[i].data.straight.stop[1]-rs[i].
        data.straight.start[1]);
      break;

     case CONST_CURVE:
      rtB.LookupP_o1 = rs[i].data.const_curve.center[0] + rs[i].
        data.const_curve.r
        *cos( (double)(rs[i].data.const_curve.dir)*p/(rs[i].data.const_curve.r)
             + rs[i].data.const_curve.ts );
      rtB.LookupP_o2 = rs[i].data.const_curve.center[1] + rs[i].
        data.const_curve.r
        *sin( (double)(rs[i].data.const_curve.dir)*p/(rs[i].data.const_curve.r)
             + rs[i].data.const_curve.ts );
      break;

     default :
      /* assign no value */
      break;
    }
  }

  /* Sum: '<S62>/Sum' */
  rtb_Product_o_idx_0 = rtDW.DiscreteTimeIntegrator1_DSTATE - rtB.LookupP_o1;
  rtb_Product_o_idx_1 = rtDW.DiscreteTimeIntegrator2_DSTATE - rtB.LookupP_o2;

  /* S-Function (get_rvec1): '<S62>/Look up Right Vector' */
  {
    extern struct road_seg_type rs[MAX_RS];
    real_T p, u_alt;
    int i;

    /* keep u within the range [0, PATH_LENGTH) */
    u_alt = fmod( (rtB.UnitDelay), PATH_LENGTH);

    /* Find the segment in rs[] that the path distance u_alt lies within */
    for (i=0; i<MAX_RS; i++) {
      if (rs[i].p > u_alt) {
        i--;
        break;
      }
    }

    /* i is the segment in which u lies; p is the path length from the
     * beginning of segment i to u_alt */
    p = u_alt - rs[i].p;
    switch ( rs[i].type)
    {
     case STRAIGHT:
      /* if path is straight, then the "right" vector is normal to
       * the vector that points straight ahead */
      {
        double fx, fy;                 /* unit vector straight ahead */
        double length;                 /* length of vector from start to stop */
        fx = rs[i].data.straight.stop[0] - rs[i].data.straight.start[0];
        fy = rs[i].data.straight.stop[1] - rs[i].data.straight.start[1];
        length = sqrt(fx*fx + fy*fy);
        fx /= length;
        fy /= length;

        /* <x,y,0> =  <fx,fy,0> x <0,0,1> */
        rtB.LookupRightVector_o1 = fy;
        rtB.LookupRightVector_o2 = -fx;
        break;
      }

     case CONST_CURVE:
      {
        double r;                      /* right angle */
        if (rs[i].data.const_curve.dir > 0.0) {
          r = p/(rs[i].data.const_curve.r) + rs[i].data.const_curve.ts;
        } else {
          r = -p/(rs[i].data.const_curve.r) + rs[i].data.const_curve.ts
            + 3.14159265358979;
        }

        rtB.LookupRightVector_o1 = cos(r);
        rtB.LookupRightVector_o2 = sin(r);
        break;
      }

     default :
      /* assign no value */
      break;
    }
  }

  /* Sum: '<S67>/Sum1' incorporates:
   *  Product: '<S67>/Product'
   */
  rtb_Sum1 = rtb_Product_o_idx_0 * rtB.LookupRightVector_o1 +
    rtb_Product_o_idx_1 * rtB.LookupRightVector_o2;

  /* DiscreteIntegrator: '<S28>/Discrete-Time Integrator3' */
  if (rtDW.DiscreteTimeIntegrator3_SYSTEM_ != 0) {
    rtB.DiscreteTimeIntegrator3 = rtDW.DiscreteTimeIntegrator3_DSTATE;
  } else {
    rtB.DiscreteTimeIntegrator3 = 0.01F * (real32_T)HighLevelDesign_ELAPS_T
      * rtDW.DiscreteTimeIntegrator3_PREV_U +
      rtDW.DiscreteTimeIntegrator3_DSTATE;
  }

  /* End of DiscreteIntegrator: '<S28>/Discrete-Time Integrator3' */

  /* S-Function (get_rvec1): '<S60>/Look up Right Vector' */
  {
    extern struct road_seg_type rs[MAX_RS];
    real_T p, u_alt;
    int i;

    /* keep u within the range [0, PATH_LENGTH) */
    u_alt = fmod( (rtB.UnitDelay), PATH_LENGTH);

    /* Find the segment in rs[] that the path distance u_alt lies within */
    for (i=0; i<MAX_RS; i++) {
      if (rs[i].p > u_alt) {
        i--;
        break;
      }
    }

    /* i is the segment in which u lies; p is the path length from the
     * beginning of segment i to u_alt */
    p = u_alt - rs[i].p;
    switch ( rs[i].type)
    {
     case STRAIGHT:
      /* if path is straight, then the "right" vector is normal to
       * the vector that points straight ahead */
      {
        double fx, fy;                 /* unit vector straight ahead */
        double length;                 /* length of vector from start to stop */
        fx = rs[i].data.straight.stop[0] - rs[i].data.straight.start[0];
        fy = rs[i].data.straight.stop[1] - rs[i].data.straight.start[1];
        length = sqrt(fx*fx + fy*fy);
        fx /= length;
        fy /= length;

        /* <x,y,0> =  <fx,fy,0> x <0,0,1> */
        rtB.LookupRightVector_o1_l = fy;
        rtB.LookupRightVector_o2_n = -fx;
        break;
      }

     case CONST_CURVE:
      {
        double r;                      /* right angle */
        if (rs[i].data.const_curve.dir > 0.0) {
          r = p/(rs[i].data.const_curve.r) + rs[i].data.const_curve.ts;
        } else {
          r = -p/(rs[i].data.const_curve.r) + rs[i].data.const_curve.ts
            + 3.14159265358979;
        }

        rtB.LookupRightVector_o1_l = cos(r);
        rtB.LookupRightVector_o2_n = sin(r);
        break;
      }

     default :
      /* assign no value */
      break;
    }
  }

  /* Gain: '<S60>/Gain' */
  rtB.Gain = -rtB.LookupRightVector_o2_n;

  /* DiscreteIntegrator: '<S28>/Discrete-Time Integrator' */
  if (rtDW.DiscreteTimeIntegrator_SYSTEM_E != 0) {
    rtB.u = rtDW.DiscreteTimeIntegrator_DSTATE;
  } else {
    rtB.u = 0.01F * (real32_T)HighLevelDesign_ELAPS_T
      * rtDW.DiscreteTimeIntegrator_PREV_U + rtDW.DiscreteTimeIntegrator_DSTATE;
  }

  /* End of DiscreteIntegrator: '<S28>/Discrete-Time Integrator' */

  /* S-Function (ftm_s32k_quadrature_decoder): '<S40>/Quadrature_Decoder' */

  /* FTM2: get counter value */
  rtB.Quadrature_Decoder_o2 = FTM_DRV_GetQuadDir(FTM2);
  rtB.Quadrature_Decoder_o4 = FTM_DRV_GetQuadTimerOverflowDir(FTM2);
  rtB.Quadrature_Decoder_o3 = FTM_DRV_HasTimerOverflowed(FTM2);
  rtB.Quadrature_Decoder_o1 = FTM_DRV_GetCounter(FTM2);
  FTM_DRV_ClearTimerOverflow(FTM2);

  /* Sum: '<S41>/Sum' incorporates:
   *  DataTypeConversion: '<S41>/Data Type Conversion1'
   *  DataTypeConversion: '<S41>/Data Type Conversion2'
   *  Delay: '<S41>/Delay'
   *  Delay: '<S41>/Delay1'
   *  Gain: '<S41>/Gain'
   *  Sum: '<S41>/Sum1'
   */
  rtDW.Delay1_DSTATE += (real32_T)(int16_T)(rtB.Quadrature_Decoder_o1 -
    rtDW.Delay_DSTATE_h) * 0.09F;

  /* Gain: '<S26>/Gain' incorporates:
   *  Delay: '<S41>/Delay1'
   *  Gain: '<S41>/Gain1'
   */
  rtB.Gain_c = -0.0174532924F * rtDW.Delay1_DSTATE * 0.2F;

  /* S-Function (calc_us): '<S28>/calculate us' */
  calc_us_Outputs_wrapper(&rtB.Gain, &rtB.LookupRightVector_o1_l, &rtB.u,
    &rtB.Gain_c, &rtB.DiscreteTimeIntegrator3, &rtB.calculateus);

  /* Inport: '<S2>/s,n,psi,us (Cars 1-6)' */
  rtB.snpsiusCars16[0] = rtB.ByteUnpacking_o1_m;
  rtB.snpsiusCars16[1] = rtB.ByteUnpacking_o2_k;
  rtB.snpsiusCars16[2] = rtB.ByteUnpacking_o1_b;
  rtB.snpsiusCars16[3] = rtB.ByteUnpacking_o2_nz;
  rtB.snpsiusCars16[4] = rtB.ByteUnpacking_o1_o3;
  rtB.snpsiusCars16[5] = rtB.ByteUnpacking_o2_h;
  rtB.snpsiusCars16[6] = rtB.ByteUnpacking_o1_az;
  rtB.snpsiusCars16[7] = rtB.ByteUnpacking_o2_b;
  rtB.snpsiusCars16[8] = rtB.ByteUnpacking_o1_i;
  rtB.snpsiusCars16[9] = rtB.ByteUnpacking_o2_i;
  rtB.snpsiusCars16[10] = rtB.ByteUnpacking_o1_a;
  rtB.snpsiusCars16[11] = rtB.ByteUnpacking_o2_j1;
  rtB.snpsiusCars16[12] = rtB.ByteUnpacking_o1_o0;
  rtB.snpsiusCars16[13] = rtB.ByteUnpacking_o2_p;
  rtB.snpsiusCars16[14] = rtB.ByteUnpacking_o1_c;
  rtB.snpsiusCars16[15] = rtB.ByteUnpacking_o2_c;
  rtB.snpsiusCars16[16] = rtB.ByteUnpacking_o1_o;
  rtB.snpsiusCars16[17] = rtB.ByteUnpacking_o2_j;
  rtB.snpsiusCars16[18] = rtB.ByteUnpacking_o1_f4;
  rtB.snpsiusCars16[19] = rtB.ByteUnpacking_o2_m;
  rtB.snpsiusCars16[20] = rtB.ByteUnpacking_o1_f;
  rtB.snpsiusCars16[21] = rtB.ByteUnpacking_o2_n;
  rtB.snpsiusCars16[22] = rtB.ByteUnpacking_o1;
  rtB.snpsiusCars16[23] = rtB.ByteUnpacking_o2;

  /* SignalConversion generated from: '<S29>/Pick_Lead' incorporates:
   *  Fcn: '<S34>/Fcn'
   *  Fcn: '<S34>/Fcn1'
   *  Fcn: '<S34>/Fcn2'
   *  Fcn: '<S34>/Fcn3'
   *  Fcn: '<S34>/Fcn4'
   *  Fcn: '<S34>/Fcn5'
   *  Fcn: '<S34>/Fcn7'
   */
  rtB.TmpSignalConversionAtPick_LeadI[0] = rtB.UnitDelay;
  rtB.TmpSignalConversionAtPick_LeadI[1] = rtB.snpsiusCars16[0];
  rtB.TmpSignalConversionAtPick_LeadI[2] = rtB.snpsiusCars16[4];
  rtB.TmpSignalConversionAtPick_LeadI[3] = rtB.snpsiusCars16[8];
  rtB.TmpSignalConversionAtPick_LeadI[4] = rtB.snpsiusCars16[12];
  rtB.TmpSignalConversionAtPick_LeadI[5] = rtB.snpsiusCars16[16];
  rtB.TmpSignalConversionAtPick_LeadI[6] = rtB.snpsiusCars16[20];

  /* SignalConversion generated from: '<S29>/Pick_Lead' incorporates:
   *  Fcn: '<S35>/Fcn'
   *  Fcn: '<S35>/Fcn1'
   *  Fcn: '<S35>/Fcn2'
   *  Fcn: '<S35>/Fcn3'
   *  Fcn: '<S35>/Fcn4'
   *  Fcn: '<S35>/Fcn5'
   *  Fcn: '<S35>/Fcn7'
   */
  rtB.TmpSignalConversionAtPick_Lea_o[0] = rtB.calculateus;
  rtB.TmpSignalConversionAtPick_Lea_o[1] = rtB.snpsiusCars16[3];
  rtB.TmpSignalConversionAtPick_Lea_o[2] = rtB.snpsiusCars16[7];
  rtB.TmpSignalConversionAtPick_Lea_o[3] = rtB.snpsiusCars16[11];
  rtB.TmpSignalConversionAtPick_Lea_o[4] = rtB.snpsiusCars16[15];
  rtB.TmpSignalConversionAtPick_Lea_o[5] = rtB.snpsiusCars16[19];
  rtB.TmpSignalConversionAtPick_Lea_o[6] = rtB.snpsiusCars16[23];

  /* S-Function (Pick_lead): '<S29>/Pick_Lead' */
  Pick_lead_Outputs_wrapper(&rtB.TmpSignalConversionAtPick_LeadI[0],
    &rtB.TmpSignalConversionAtPick_Lea_o[0], &rtB.Pick_Lead_o1,
    &rtB.Pick_Lead_o2);

  /* Chart: '<S29>/Choose_Mode' incorporates:
   *  Constant: '<S29>/Constant'
   */
  if (rtDW.is_active_c1_Final_project_temp == 0U) {
    rtDW.is_active_c1_Final_project_temp = 1U;
    rtDW.is_c1_Final_project_template = IN_Manual;
    rtb_en_man = true;
    rtb_en_vel = false;
    rtb_en_pos = false;
  } else {
    switch (rtDW.is_c1_Final_project_template) {
     case IN_Manual:
      rtb_en_man = true;
      rtb_en_vel = false;
      rtb_en_pos = false;
      rtb_Gain_ie = rtB.Pick_Lead_o1 - rtB.UnitDelay;
      rtb_LogicalOperator = !rtb_LogicalOperator;
      if (rtb_LogicalOperator && ((rtB.Pick_Lead_o2 <= rtB.calculateus) &&
           (rtb_Gain_ie <= 21.0F)) && (rtB.Pick_Lead_o1 != rtB.UnitDelay)) {
        rtDW.is_c1_Final_project_template = IN_Position;
        rtb_en_man = false;
        rtb_en_pos = true;
      } else {
        if (rtb_LogicalOperator && ((rtB.Pick_Lead_o2 > rtB.calculateus) ||
             (rtb_Gain_ie > 21.0F) || (rtB.Pick_Lead_o1 == rtB.UnitDelay))) {
          rtDW.is_c1_Final_project_template = IN_Velocity;
          rtb_en_man = false;
          rtb_en_vel = true;
        }
      }
      break;

     case IN_Position:
      rtb_en_man = false;
      rtb_en_vel = false;
      rtb_en_pos = true;
      if ((!rtb_LogicalOperator) && ((rtB.Pick_Lead_o2 > rtB.calculateus) ||
           (rtB.Pick_Lead_o1 - rtB.UnitDelay > 21.0F) || (rtB.Pick_Lead_o1 ==
            rtB.UnitDelay))) {
        rtDW.is_c1_Final_project_template = IN_Velocity;
        rtb_en_vel = true;
        rtb_en_pos = false;
      } else {
        if (rtb_LogicalOperator) {
          rtDW.is_c1_Final_project_template = IN_Manual;
          rtb_en_man = true;
          rtb_en_pos = false;
        }
      }
      break;

     default:
      /* case IN_Velocity: */
      rtb_en_man = false;
      rtb_en_vel = true;
      rtb_en_pos = false;
      if (rtb_LogicalOperator) {
        rtDW.is_c1_Final_project_template = IN_Manual;
        rtb_en_man = true;
        rtb_en_vel = false;
      } else {
        if ((rtB.Pick_Lead_o2 <= rtB.calculateus) && (rtB.Pick_Lead_o1 -
             rtB.UnitDelay <= 21.0F) && (rtB.Pick_Lead_o1 != rtB.UnitDelay)) {
          rtDW.is_c1_Final_project_template = IN_Position;
          rtb_en_vel = false;
          rtb_en_pos = true;
        }
      }
      break;
    }
  }

  /* End of Chart: '<S29>/Choose_Mode' */

  /* S-Function (adc_s32k_start): '<S39>/ADC_Start' */
  {
    adc_chan_config_t adc0_chan_cfg = {
      .interruptEnable = false,
      .channel = ADC_INPUTCHAN_EXT0
    };

    /* Initialize channel configuration of ADC0. */
    ADC_DRV_ConfigChan(0, 0, &adc0_chan_cfg);
    uint16_t result;

    /* Get conversion result of ADC0 */
    ADC_DRV_WaitConvDone(0);
    ADC_DRV_GetChanResult(0, 0, &result);
    rtB.ADC_Start = result;
  }

  /* Outputs for Enabled SubSystem: '<S24>/Manual Ctrl' incorporates:
   *  EnablePort: '<S30>/Enable'
   */
  if (rtb_en_man) {
    /* Inport: '<S30>/throttle' incorporates:
     *  Constant: '<S39>/pot_offset'
     *  DataTypeConversion: '<S39>/Data Type Conversion'
     *  Sum: '<S39>/Sum'
     */
    rtB.Merge = (real32_T)rtB.ADC_Start + 512.0F;
  }

  /* End of Outputs for SubSystem: '<S24>/Manual Ctrl' */

  /* Outputs for Enabled SubSystem: '<S24>/Position Ctrl' incorporates:
   *  EnablePort: '<S31>/Enable'
   */
  if (rtb_en_pos) {
    /* Sum: '<S31>/Sum5' incorporates:
     *  Constant: '<S24>/Constant'
     *  Gain: '<S31>/D-gain'
     *  Gain: '<S31>/Feed-forward'
     *  Gain: '<S31>/P-gain'
     *  Sum: '<S31>/Sum1'
     *  Sum: '<S31>/Sum2'
     *  Sum: '<S31>/Sum3'
     *  Sum: '<S31>/Sum4'
     */
    rtB.Merge = ((20.0F - (rtB.Pick_Lead_o1 - rtB.UnitDelay)) * -3000.0F +
                 (rtB.Pick_Lead_o2 - rtB.calculateus) * 4900.0F) + 100.0F *
      rtB.Pick_Lead_o2;
  }

  /* End of Outputs for SubSystem: '<S24>/Position Ctrl' */

  /* S-Function (gpio_s32k_input): '<S38>/Digital_Input' */

  /* GPIPORTE6 signal update */
  rtB.Digital_Input_g = (PINS_DRV_ReadPins(PTE) >> 6) & 0x01;

  /* S-Function (gpio_s32k_input): '<S38>/Digital_Input1' */

  /* GPIPORTE7 signal update */
  rtB.Digital_Input1 = (PINS_DRV_ReadPins(PTE) >> 7) & 0x01;

  /* S-Function (gpio_s32k_input): '<S38>/Digital_Input2' */

  /* GPIPORTE8 signal update */
  rtB.Digital_Input2 = (PINS_DRV_ReadPins(PTE) >> 8) & 0x01;

  /* S-Function (gpio_s32k_input): '<S38>/Digital_Input3' */

  /* GPIPORTE9 signal update */
  rtB.Digital_Input3 = (PINS_DRV_ReadPins(PTE) >> 9) & 0x01;

  /* S-Function (gpio_s32k_input): '<S38>/Digital_Input4' */

  /* GPIPORTE10 signal update */
  rtB.Digital_Input4 = (PINS_DRV_ReadPins(PTE) >> 10) & 0x01;

  /* S-Function (gpio_s32k_input): '<S38>/Digital_Input5' */

  /* GPIPORTE11 signal update */
  rtB.Digital_Input5 = (PINS_DRV_ReadPins(PTE) >> 11) & 0x01;

  /* S-Function (gpio_s32k_input): '<S38>/Digital_Input6' */

  /* GPIPORTE12 signal update */
  rtB.Digital_Input6 = (PINS_DRV_ReadPins(PTE) >> 12) & 0x01;

  /* S-Function (gpio_s32k_input): '<S38>/Digital_Input7' */

  /* GPIPORTE13 signal update */
  rtB.Digital_Input7 = (PINS_DRV_ReadPins(PTE) >> 13) & 0x01;

  /* Gain: '<S38>/Gain' incorporates:
   *  ArithShift: '<S38>/Shift Arithmetic1'
   *  ArithShift: '<S38>/Shift Arithmetic2'
   *  ArithShift: '<S38>/Shift Arithmetic3'
   *  ArithShift: '<S38>/Shift Arithmetic4'
   *  ArithShift: '<S38>/Shift Arithmetic5'
   *  ArithShift: '<S38>/Shift Arithmetic6'
   *  ArithShift: '<S38>/Shift Arithmetic7'
   *  DataTypeConversion: '<S38>/Data Type Conversion'
   *  DataTypeConversion: '<S38>/Data Type Conversion1'
   *  DataTypeConversion: '<S38>/Data Type Conversion2'
   *  DataTypeConversion: '<S38>/Data Type Conversion3'
   *  DataTypeConversion: '<S38>/Data Type Conversion4'
   *  DataTypeConversion: '<S38>/Data Type Conversion5'
   *  DataTypeConversion: '<S38>/Data Type Conversion6'
   *  DataTypeConversion: '<S38>/Data Type Conversion7'
   *  DataTypeConversion: '<S38>/Data Type Conversion8'
   *  Sum: '<S38>/Add'
   */
  rtb_Gain_ie = (real32_T)((int32_T)(((((((uint32_T)(rtB.Digital_Input1 << 1) +
    rtB.Digital_Input_g) + (rtB.Digital_Input2 << 2)) + (rtB.Digital_Input3 << 3))
    + (rtB.Digital_Input4 << 4)) + (rtB.Digital_Input5 << 5)) +
    (rtB.Digital_Input6 << 6)) + (rtB.Digital_Input7 << 7)) * 0.25F;

  /* Outputs for Enabled SubSystem: '<S24>/Speed Ctrl' incorporates:
   *  EnablePort: '<S32>/Enable'
   */
  if (rtb_en_vel) {
    /* Sum: '<S32>/Sum1' */
    rtb_Sum1_m = rtb_Gain_ie - rtB.calculateus;

    /* Sum: '<S32>/Sum' incorporates:
     *  Gain: '<S32>/Gain'
     *  Gain: '<S36>/Gain1'
     *  Sum: '<S36>/Sum'
     *  UnitDelay: '<S37>/Unit Delay'
     */
    rtB.Merge = (5000.0F * rtb_Sum1_m + rtDW.UnitDelay_DSTATE_n) + 100.0F *
      rtb_Gain_ie;

    /* Sum: '<S37>/Sum' incorporates:
     *  Gain: '<S36>/Gain'
     *  Gain: '<S37>/Gain'
     *  UnitDelay: '<S37>/Unit Delay'
     */
    rtDW.UnitDelay_DSTATE_n += 50.0F * rtb_Sum1_m * 0.01F;
  }

  /* End of Outputs for SubSystem: '<S24>/Speed Ctrl' */

  /* S-Function (gpio_s32k_input): '<S26>/Digital_Input1' */

  /* GPIPORTE15 signal update */
  rtB.Digital_Input1_b = (PINS_DRV_ReadPins(PTE) >> 15) & 0x01;

  /* Outputs for Enabled SubSystem: '<S2>/Auto Steering' incorporates:
   *  EnablePort: '<S25>/Enable'
   */
  if (rtB.Digital_Input1_b) {
    /* Sum: '<S25>/Sum1' incorporates:
     *  Gain: '<S25>/Gain1'
     *  Gain: '<S25>/Gain6'
     *  Gain: '<S25>/Gain7'
     *  Sum: '<S25>/Sum'
     *  Sum: '<S25>/Sum2'
     *  Sum: '<S25>/Sum3'
     *  UnitDelay: '<S25>/Unit Delay'
     */
    rtb_Gain_ie = ((rtb_Sum1 - rtDW.UnitDelay_DSTATE_j) * 100.0F * 0.05F + 0.5F *
                   rtb_Sum1) - rtB.Gain_c;

    /* Sum: '<S25>/Sum5' incorporates:
     *  Gain: '<S25>/Gain2'
     *  Gain: '<S25>/Gain4'
     *  Gain: '<S25>/Gain8'
     *  Sum: '<S25>/Sum4'
     *  UnitDelay: '<S25>/Unit Delay1'
     */
    rtB.Sum5 = (rtb_Gain_ie - rtDW.UnitDelay1_DSTATE) * 100.0F * 100.0F +
      1000.0F * rtb_Gain_ie;

    /* Update for UnitDelay: '<S25>/Unit Delay' incorporates:
     *  Sum: '<S25>/Sum'
     */
    rtDW.UnitDelay_DSTATE_j = rtb_Sum1;

    /* Update for UnitDelay: '<S25>/Unit Delay1' */
    rtDW.UnitDelay1_DSTATE = rtb_Gain_ie;
  }

  /* End of Outputs for SubSystem: '<S2>/Auto Steering' */

  /* S-Function (get_rvec1): '<S46>/Look up Right Vector1' */
  {
    extern struct road_seg_type rs[MAX_RS];
    real_T p, u_alt;
    int i;

    /* keep u within the range [0, PATH_LENGTH) */
    u_alt = fmod( (rtB.UnitDelay), PATH_LENGTH);

    /* Find the segment in rs[] that the path distance u_alt lies within */
    for (i=0; i<MAX_RS; i++) {
      if (rs[i].p > u_alt) {
        i--;
        break;
      }
    }

    /* i is the segment in which u lies; p is the path length from the
     * beginning of segment i to u_alt */
    p = u_alt - rs[i].p;
    switch ( rs[i].type)
    {
     case STRAIGHT:
      /* if path is straight, then the "right" vector is normal to
       * the vector that points straight ahead */
      {
        double fx, fy;                 /* unit vector straight ahead */
        double length;                 /* length of vector from start to stop */
        fx = rs[i].data.straight.stop[0] - rs[i].data.straight.start[0];
        fy = rs[i].data.straight.stop[1] - rs[i].data.straight.start[1];
        length = sqrt(fx*fx + fy*fy);
        fx /= length;
        fy /= length;

        /* <x,y,0> =  <fx,fy,0> x <0,0,1> */
        rtB.LookupRightVector1_o1 = fy;
        rtB.LookupRightVector1_o2 = -fx;
        break;
      }

     case CONST_CURVE:
      {
        double r;                      /* right angle */
        if (rs[i].data.const_curve.dir > 0.0) {
          r = p/(rs[i].data.const_curve.r) + rs[i].data.const_curve.ts;
        } else {
          r = -p/(rs[i].data.const_curve.r) + rs[i].data.const_curve.ts
            + 3.14159265358979;
        }

        rtB.LookupRightVector1_o1 = cos(r);
        rtB.LookupRightVector1_o2 = sin(r);
        break;
      }

     default :
      /* assign no value */
      break;
    }
  }

  /* S-Function (get_path1): '<S46>/Look up P1' */
  {
    extern struct road_seg_type rs[MAX_RS];
    real_T p, u_alt;
    int i;

    /* keep u within the range [0, PATH_LENGTH) */
    u_alt = fmod( (rtB.UnitDelay), PATH_LENGTH);

    /* Find the segment in rs[] that the path distance u_alt lies within */
    for (i=0; i<MAX_RS; i++) {
      if (rs[i].p > u_alt) {
        i--;
        break;
      }
    }

    /* i is the segment in which u lies; p is the path length from the
     * beginning of segment i to u_alt */
    p = u_alt - rs[i].p;
    switch ( rs[i].type)
    {
     case STRAIGHT:
      /* If the path is straight, interpolate between start and stop */
      rtB.LookupP1_o1 = rs[i].data.straight.start[0] +
        (p/rs[i].data.straight.length)*(rs[i].data.straight.stop[0]-rs[i].
        data.straight.start[0]);
      rtB.LookupP1_o2 = rs[i].data.straight.start[1] +
        (p/rs[i].data.straight.length)*(rs[i].data.straight.stop[1]-rs[i].
        data.straight.start[1]);
      break;

     case CONST_CURVE:
      rtB.LookupP1_o1 = rs[i].data.const_curve.center[0] + rs[i].
        data.const_curve.r
        *cos( (double)(rs[i].data.const_curve.dir)*p/(rs[i].data.const_curve.r)
             + rs[i].data.const_curve.ts );
      rtB.LookupP1_o2 = rs[i].data.const_curve.center[1] + rs[i].
        data.const_curve.r
        *sin( (double)(rs[i].data.const_curve.dir)*p/(rs[i].data.const_curve.r)
             + rs[i].data.const_curve.ts );
      break;

     default :
      /* assign no value */
      break;
    }
  }

  /* S-Function (get_rvec1): '<S53>/Look up Right Vector1' */
  {
    extern struct road_seg_type rs[MAX_RS];
    real_T p, u_alt;
    int i;

    /* keep u within the range [0, PATH_LENGTH) */
    u_alt = fmod( (rtB.snpsiusCars16[0]), PATH_LENGTH);

    /* Find the segment in rs[] that the path distance u_alt lies within */
    for (i=0; i<MAX_RS; i++) {
      if (rs[i].p > u_alt) {
        i--;
        break;
      }
    }

    /* i is the segment in which u lies; p is the path length from the
     * beginning of segment i to u_alt */
    p = u_alt - rs[i].p;
    switch ( rs[i].type)
    {
     case STRAIGHT:
      /* if path is straight, then the "right" vector is normal to
       * the vector that points straight ahead */
      {
        double fx, fy;                 /* unit vector straight ahead */
        double length;                 /* length of vector from start to stop */
        fx = rs[i].data.straight.stop[0] - rs[i].data.straight.start[0];
        fy = rs[i].data.straight.stop[1] - rs[i].data.straight.start[1];
        length = sqrt(fx*fx + fy*fy);
        fx /= length;
        fy /= length;

        /* <x,y,0> =  <fx,fy,0> x <0,0,1> */
        rtB.LookupRightVector1_o1_j = fy;
        rtB.LookupRightVector1_o2_m = -fx;
        break;
      }

     case CONST_CURVE:
      {
        double r;                      /* right angle */
        if (rs[i].data.const_curve.dir > 0.0) {
          r = p/(rs[i].data.const_curve.r) + rs[i].data.const_curve.ts;
        } else {
          r = -p/(rs[i].data.const_curve.r) + rs[i].data.const_curve.ts
            + 3.14159265358979;
        }

        rtB.LookupRightVector1_o1_j = cos(r);
        rtB.LookupRightVector1_o2_m = sin(r);
        break;
      }

     default :
      /* assign no value */
      break;
    }
  }

  /* S-Function (get_path1): '<S53>/Look up P1' */
  {
    extern struct road_seg_type rs[MAX_RS];
    real_T p, u_alt;
    int i;

    /* keep u within the range [0, PATH_LENGTH) */
    u_alt = fmod( (rtB.snpsiusCars16[0]), PATH_LENGTH);

    /* Find the segment in rs[] that the path distance u_alt lies within */
    for (i=0; i<MAX_RS; i++) {
      if (rs[i].p > u_alt) {
        i--;
        break;
      }
    }

    /* i is the segment in which u lies; p is the path length from the
     * beginning of segment i to u_alt */
    p = u_alt - rs[i].p;
    switch ( rs[i].type)
    {
     case STRAIGHT:
      /* If the path is straight, interpolate between start and stop */
      rtB.LookupP1_o1_j = rs[i].data.straight.start[0] +
        (p/rs[i].data.straight.length)*(rs[i].data.straight.stop[0]-rs[i].
        data.straight.start[0]);
      rtB.LookupP1_o2_m = rs[i].data.straight.start[1] +
        (p/rs[i].data.straight.length)*(rs[i].data.straight.stop[1]-rs[i].
        data.straight.start[1]);
      break;

     case CONST_CURVE:
      rtB.LookupP1_o1_j = rs[i].data.const_curve.center[0] + rs[i].
        data.const_curve.r
        *cos( (double)(rs[i].data.const_curve.dir)*p/(rs[i].data.const_curve.r)
             + rs[i].data.const_curve.ts );
      rtB.LookupP1_o2_m = rs[i].data.const_curve.center[1] + rs[i].
        data.const_curve.r
        *sin( (double)(rs[i].data.const_curve.dir)*p/(rs[i].data.const_curve.r)
             + rs[i].data.const_curve.ts );
      break;

     default :
      /* assign no value */
      break;
    }
  }

  /* S-Function (get_rvec1): '<S54>/Look up Right Vector1' */
  {
    extern struct road_seg_type rs[MAX_RS];
    real_T p, u_alt;
    int i;

    /* keep u within the range [0, PATH_LENGTH) */
    u_alt = fmod( (rtB.snpsiusCars16[4]), PATH_LENGTH);

    /* Find the segment in rs[] that the path distance u_alt lies within */
    for (i=0; i<MAX_RS; i++) {
      if (rs[i].p > u_alt) {
        i--;
        break;
      }
    }

    /* i is the segment in which u lies; p is the path length from the
     * beginning of segment i to u_alt */
    p = u_alt - rs[i].p;
    switch ( rs[i].type)
    {
     case STRAIGHT:
      /* if path is straight, then the "right" vector is normal to
       * the vector that points straight ahead */
      {
        double fx, fy;                 /* unit vector straight ahead */
        double length;                 /* length of vector from start to stop */
        fx = rs[i].data.straight.stop[0] - rs[i].data.straight.start[0];
        fy = rs[i].data.straight.stop[1] - rs[i].data.straight.start[1];
        length = sqrt(fx*fx + fy*fy);
        fx /= length;
        fy /= length;

        /* <x,y,0> =  <fx,fy,0> x <0,0,1> */
        rtB.LookupRightVector1_o1_i = fy;
        rtB.LookupRightVector1_o2_d = -fx;
        break;
      }

     case CONST_CURVE:
      {
        double r;                      /* right angle */
        if (rs[i].data.const_curve.dir > 0.0) {
          r = p/(rs[i].data.const_curve.r) + rs[i].data.const_curve.ts;
        } else {
          r = -p/(rs[i].data.const_curve.r) + rs[i].data.const_curve.ts
            + 3.14159265358979;
        }

        rtB.LookupRightVector1_o1_i = cos(r);
        rtB.LookupRightVector1_o2_d = sin(r);
        break;
      }

     default :
      /* assign no value */
      break;
    }
  }

  /* S-Function (get_path1): '<S54>/Look up P1' */
  {
    extern struct road_seg_type rs[MAX_RS];
    real_T p, u_alt;
    int i;

    /* keep u within the range [0, PATH_LENGTH) */
    u_alt = fmod( (rtB.snpsiusCars16[4]), PATH_LENGTH);

    /* Find the segment in rs[] that the path distance u_alt lies within */
    for (i=0; i<MAX_RS; i++) {
      if (rs[i].p > u_alt) {
        i--;
        break;
      }
    }

    /* i is the segment in which u lies; p is the path length from the
     * beginning of segment i to u_alt */
    p = u_alt - rs[i].p;
    switch ( rs[i].type)
    {
     case STRAIGHT:
      /* If the path is straight, interpolate between start and stop */
      rtB.LookupP1_o1_f = rs[i].data.straight.start[0] +
        (p/rs[i].data.straight.length)*(rs[i].data.straight.stop[0]-rs[i].
        data.straight.start[0]);
      rtB.LookupP1_o2_p = rs[i].data.straight.start[1] +
        (p/rs[i].data.straight.length)*(rs[i].data.straight.stop[1]-rs[i].
        data.straight.start[1]);
      break;

     case CONST_CURVE:
      rtB.LookupP1_o1_f = rs[i].data.const_curve.center[0] + rs[i].
        data.const_curve.r
        *cos( (double)(rs[i].data.const_curve.dir)*p/(rs[i].data.const_curve.r)
             + rs[i].data.const_curve.ts );
      rtB.LookupP1_o2_p = rs[i].data.const_curve.center[1] + rs[i].
        data.const_curve.r
        *sin( (double)(rs[i].data.const_curve.dir)*p/(rs[i].data.const_curve.r)
             + rs[i].data.const_curve.ts );
      break;

     default :
      /* assign no value */
      break;
    }
  }

  /* S-Function (get_rvec1): '<S55>/Look up Right Vector1' */
  {
    extern struct road_seg_type rs[MAX_RS];
    real_T p, u_alt;
    int i;

    /* keep u within the range [0, PATH_LENGTH) */
    u_alt = fmod( (rtB.snpsiusCars16[8]), PATH_LENGTH);

    /* Find the segment in rs[] that the path distance u_alt lies within */
    for (i=0; i<MAX_RS; i++) {
      if (rs[i].p > u_alt) {
        i--;
        break;
      }
    }

    /* i is the segment in which u lies; p is the path length from the
     * beginning of segment i to u_alt */
    p = u_alt - rs[i].p;
    switch ( rs[i].type)
    {
     case STRAIGHT:
      /* if path is straight, then the "right" vector is normal to
       * the vector that points straight ahead */
      {
        double fx, fy;                 /* unit vector straight ahead */
        double length;                 /* length of vector from start to stop */
        fx = rs[i].data.straight.stop[0] - rs[i].data.straight.start[0];
        fy = rs[i].data.straight.stop[1] - rs[i].data.straight.start[1];
        length = sqrt(fx*fx + fy*fy);
        fx /= length;
        fy /= length;

        /* <x,y,0> =  <fx,fy,0> x <0,0,1> */
        rtB.LookupRightVector1_o1_a = fy;
        rtB.LookupRightVector1_o2_j = -fx;
        break;
      }

     case CONST_CURVE:
      {
        double r;                      /* right angle */
        if (rs[i].data.const_curve.dir > 0.0) {
          r = p/(rs[i].data.const_curve.r) + rs[i].data.const_curve.ts;
        } else {
          r = -p/(rs[i].data.const_curve.r) + rs[i].data.const_curve.ts
            + 3.14159265358979;
        }

        rtB.LookupRightVector1_o1_a = cos(r);
        rtB.LookupRightVector1_o2_j = sin(r);
        break;
      }

     default :
      /* assign no value */
      break;
    }
  }

  /* S-Function (get_path1): '<S55>/Look up P1' */
  {
    extern struct road_seg_type rs[MAX_RS];
    real_T p, u_alt;
    int i;

    /* keep u within the range [0, PATH_LENGTH) */
    u_alt = fmod( (rtB.snpsiusCars16[8]), PATH_LENGTH);

    /* Find the segment in rs[] that the path distance u_alt lies within */
    for (i=0; i<MAX_RS; i++) {
      if (rs[i].p > u_alt) {
        i--;
        break;
      }
    }

    /* i is the segment in which u lies; p is the path length from the
     * beginning of segment i to u_alt */
    p = u_alt - rs[i].p;
    switch ( rs[i].type)
    {
     case STRAIGHT:
      /* If the path is straight, interpolate between start and stop */
      rtB.LookupP1_o1_jz = rs[i].data.straight.start[0] +
        (p/rs[i].data.straight.length)*(rs[i].data.straight.stop[0]-rs[i].
        data.straight.start[0]);
      rtB.LookupP1_o2_pi = rs[i].data.straight.start[1] +
        (p/rs[i].data.straight.length)*(rs[i].data.straight.stop[1]-rs[i].
        data.straight.start[1]);
      break;

     case CONST_CURVE:
      rtB.LookupP1_o1_jz = rs[i].data.const_curve.center[0] + rs[i].
        data.const_curve.r
        *cos( (double)(rs[i].data.const_curve.dir)*p/(rs[i].data.const_curve.r)
             + rs[i].data.const_curve.ts );
      rtB.LookupP1_o2_pi = rs[i].data.const_curve.center[1] + rs[i].
        data.const_curve.r
        *sin( (double)(rs[i].data.const_curve.dir)*p/(rs[i].data.const_curve.r)
             + rs[i].data.const_curve.ts );
      break;

     default :
      /* assign no value */
      break;
    }
  }

  /* S-Function (get_rvec1): '<S56>/Look up Right Vector1' */
  {
    extern struct road_seg_type rs[MAX_RS];
    real_T p, u_alt;
    int i;

    /* keep u within the range [0, PATH_LENGTH) */
    u_alt = fmod( (rtB.snpsiusCars16[12]), PATH_LENGTH);

    /* Find the segment in rs[] that the path distance u_alt lies within */
    for (i=0; i<MAX_RS; i++) {
      if (rs[i].p > u_alt) {
        i--;
        break;
      }
    }

    /* i is the segment in which u lies; p is the path length from the
     * beginning of segment i to u_alt */
    p = u_alt - rs[i].p;
    switch ( rs[i].type)
    {
     case STRAIGHT:
      /* if path is straight, then the "right" vector is normal to
       * the vector that points straight ahead */
      {
        double fx, fy;                 /* unit vector straight ahead */
        double length;                 /* length of vector from start to stop */
        fx = rs[i].data.straight.stop[0] - rs[i].data.straight.start[0];
        fy = rs[i].data.straight.stop[1] - rs[i].data.straight.start[1];
        length = sqrt(fx*fx + fy*fy);
        fx /= length;
        fy /= length;

        /* <x,y,0> =  <fx,fy,0> x <0,0,1> */
        rtB.LookupRightVector1_o1_c = fy;
        rtB.LookupRightVector1_o2_e = -fx;
        break;
      }

     case CONST_CURVE:
      {
        double r;                      /* right angle */
        if (rs[i].data.const_curve.dir > 0.0) {
          r = p/(rs[i].data.const_curve.r) + rs[i].data.const_curve.ts;
        } else {
          r = -p/(rs[i].data.const_curve.r) + rs[i].data.const_curve.ts
            + 3.14159265358979;
        }

        rtB.LookupRightVector1_o1_c = cos(r);
        rtB.LookupRightVector1_o2_e = sin(r);
        break;
      }

     default :
      /* assign no value */
      break;
    }
  }

  /* S-Function (get_path1): '<S56>/Look up P1' */
  {
    extern struct road_seg_type rs[MAX_RS];
    real_T p, u_alt;
    int i;

    /* keep u within the range [0, PATH_LENGTH) */
    u_alt = fmod( (rtB.snpsiusCars16[12]), PATH_LENGTH);

    /* Find the segment in rs[] that the path distance u_alt lies within */
    for (i=0; i<MAX_RS; i++) {
      if (rs[i].p > u_alt) {
        i--;
        break;
      }
    }

    /* i is the segment in which u lies; p is the path length from the
     * beginning of segment i to u_alt */
    p = u_alt - rs[i].p;
    switch ( rs[i].type)
    {
     case STRAIGHT:
      /* If the path is straight, interpolate between start and stop */
      rtB.LookupP1_o1_a = rs[i].data.straight.start[0] +
        (p/rs[i].data.straight.length)*(rs[i].data.straight.stop[0]-rs[i].
        data.straight.start[0]);
      rtB.LookupP1_o2_l = rs[i].data.straight.start[1] +
        (p/rs[i].data.straight.length)*(rs[i].data.straight.stop[1]-rs[i].
        data.straight.start[1]);
      break;

     case CONST_CURVE:
      rtB.LookupP1_o1_a = rs[i].data.const_curve.center[0] + rs[i].
        data.const_curve.r
        *cos( (double)(rs[i].data.const_curve.dir)*p/(rs[i].data.const_curve.r)
             + rs[i].data.const_curve.ts );
      rtB.LookupP1_o2_l = rs[i].data.const_curve.center[1] + rs[i].
        data.const_curve.r
        *sin( (double)(rs[i].data.const_curve.dir)*p/(rs[i].data.const_curve.r)
             + rs[i].data.const_curve.ts );
      break;

     default :
      /* assign no value */
      break;
    }
  }

  /* S-Function (get_rvec1): '<S57>/Look up Right Vector1' */
  {
    extern struct road_seg_type rs[MAX_RS];
    real_T p, u_alt;
    int i;

    /* keep u within the range [0, PATH_LENGTH) */
    u_alt = fmod( (rtB.snpsiusCars16[16]), PATH_LENGTH);

    /* Find the segment in rs[] that the path distance u_alt lies within */
    for (i=0; i<MAX_RS; i++) {
      if (rs[i].p > u_alt) {
        i--;
        break;
      }
    }

    /* i is the segment in which u lies; p is the path length from the
     * beginning of segment i to u_alt */
    p = u_alt - rs[i].p;
    switch ( rs[i].type)
    {
     case STRAIGHT:
      /* if path is straight, then the "right" vector is normal to
       * the vector that points straight ahead */
      {
        double fx, fy;                 /* unit vector straight ahead */
        double length;                 /* length of vector from start to stop */
        fx = rs[i].data.straight.stop[0] - rs[i].data.straight.start[0];
        fy = rs[i].data.straight.stop[1] - rs[i].data.straight.start[1];
        length = sqrt(fx*fx + fy*fy);
        fx /= length;
        fy /= length;

        /* <x,y,0> =  <fx,fy,0> x <0,0,1> */
        rtB.LookupRightVector1_o1_iv = fy;
        rtB.LookupRightVector1_o2_f = -fx;
        break;
      }

     case CONST_CURVE:
      {
        double r;                      /* right angle */
        if (rs[i].data.const_curve.dir > 0.0) {
          r = p/(rs[i].data.const_curve.r) + rs[i].data.const_curve.ts;
        } else {
          r = -p/(rs[i].data.const_curve.r) + rs[i].data.const_curve.ts
            + 3.14159265358979;
        }

        rtB.LookupRightVector1_o1_iv = cos(r);
        rtB.LookupRightVector1_o2_f = sin(r);
        break;
      }

     default :
      /* assign no value */
      break;
    }
  }

  /* S-Function (get_path1): '<S57>/Look up P1' */
  {
    extern struct road_seg_type rs[MAX_RS];
    real_T p, u_alt;
    int i;

    /* keep u within the range [0, PATH_LENGTH) */
    u_alt = fmod( (rtB.snpsiusCars16[16]), PATH_LENGTH);

    /* Find the segment in rs[] that the path distance u_alt lies within */
    for (i=0; i<MAX_RS; i++) {
      if (rs[i].p > u_alt) {
        i--;
        break;
      }
    }

    /* i is the segment in which u lies; p is the path length from the
     * beginning of segment i to u_alt */
    p = u_alt - rs[i].p;
    switch ( rs[i].type)
    {
     case STRAIGHT:
      /* If the path is straight, interpolate between start and stop */
      rtB.LookupP1_o1_c = rs[i].data.straight.start[0] +
        (p/rs[i].data.straight.length)*(rs[i].data.straight.stop[0]-rs[i].
        data.straight.start[0]);
      rtB.LookupP1_o2_e = rs[i].data.straight.start[1] +
        (p/rs[i].data.straight.length)*(rs[i].data.straight.stop[1]-rs[i].
        data.straight.start[1]);
      break;

     case CONST_CURVE:
      rtB.LookupP1_o1_c = rs[i].data.const_curve.center[0] + rs[i].
        data.const_curve.r
        *cos( (double)(rs[i].data.const_curve.dir)*p/(rs[i].data.const_curve.r)
             + rs[i].data.const_curve.ts );
      rtB.LookupP1_o2_e = rs[i].data.const_curve.center[1] + rs[i].
        data.const_curve.r
        *sin( (double)(rs[i].data.const_curve.dir)*p/(rs[i].data.const_curve.r)
             + rs[i].data.const_curve.ts );
      break;

     default :
      /* assign no value */
      break;
    }
  }

  /* S-Function (get_rvec1): '<S58>/Look up Right Vector1' */
  {
    extern struct road_seg_type rs[MAX_RS];
    real_T p, u_alt;
    int i;

    /* keep u within the range [0, PATH_LENGTH) */
    u_alt = fmod( (rtB.snpsiusCars16[20]), PATH_LENGTH);

    /* Find the segment in rs[] that the path distance u_alt lies within */
    for (i=0; i<MAX_RS; i++) {
      if (rs[i].p > u_alt) {
        i--;
        break;
      }
    }

    /* i is the segment in which u lies; p is the path length from the
     * beginning of segment i to u_alt */
    p = u_alt - rs[i].p;
    switch ( rs[i].type)
    {
     case STRAIGHT:
      /* if path is straight, then the "right" vector is normal to
       * the vector that points straight ahead */
      {
        double fx, fy;                 /* unit vector straight ahead */
        double length;                 /* length of vector from start to stop */
        fx = rs[i].data.straight.stop[0] - rs[i].data.straight.start[0];
        fy = rs[i].data.straight.stop[1] - rs[i].data.straight.start[1];
        length = sqrt(fx*fx + fy*fy);
        fx /= length;
        fy /= length;

        /* <x,y,0> =  <fx,fy,0> x <0,0,1> */
        rtB.LookupRightVector1_o1_je = fy;
        rtB.LookupRightVector1_o2_b = -fx;
        break;
      }

     case CONST_CURVE:
      {
        double r;                      /* right angle */
        if (rs[i].data.const_curve.dir > 0.0) {
          r = p/(rs[i].data.const_curve.r) + rs[i].data.const_curve.ts;
        } else {
          r = -p/(rs[i].data.const_curve.r) + rs[i].data.const_curve.ts
            + 3.14159265358979;
        }

        rtB.LookupRightVector1_o1_je = cos(r);
        rtB.LookupRightVector1_o2_b = sin(r);
        break;
      }

     default :
      /* assign no value */
      break;
    }
  }

  /* S-Function (get_path1): '<S58>/Look up P1' */
  {
    extern struct road_seg_type rs[MAX_RS];
    real_T p, u_alt;
    int i;

    /* keep u within the range [0, PATH_LENGTH) */
    u_alt = fmod( (rtB.snpsiusCars16[20]), PATH_LENGTH);

    /* Find the segment in rs[] that the path distance u_alt lies within */
    for (i=0; i<MAX_RS; i++) {
      if (rs[i].p > u_alt) {
        i--;
        break;
      }
    }

    /* i is the segment in which u lies; p is the path length from the
     * beginning of segment i to u_alt */
    p = u_alt - rs[i].p;
    switch ( rs[i].type)
    {
     case STRAIGHT:
      /* If the path is straight, interpolate between start and stop */
      rtB.LookupP1_o1_g = rs[i].data.straight.start[0] +
        (p/rs[i].data.straight.length)*(rs[i].data.straight.stop[0]-rs[i].
        data.straight.start[0]);
      rtB.LookupP1_o2_n = rs[i].data.straight.start[1] +
        (p/rs[i].data.straight.length)*(rs[i].data.straight.stop[1]-rs[i].
        data.straight.start[1]);
      break;

     case CONST_CURVE:
      rtB.LookupP1_o1_g = rs[i].data.const_curve.center[0] + rs[i].
        data.const_curve.r
        *cos( (double)(rs[i].data.const_curve.dir)*p/(rs[i].data.const_curve.r)
             + rs[i].data.const_curve.ts );
      rtB.LookupP1_o2_n = rs[i].data.const_curve.center[1] + rs[i].
        data.const_curve.r
        *sin( (double)(rs[i].data.const_curve.dir)*p/(rs[i].data.const_curve.r)
             + rs[i].data.const_curve.ts );
      break;

     default :
      /* assign no value */
      break;
    }
  }

  /* SignalConversion generated from: '<S44>/Byte Packing ' */
  rtB.TmpSignalConversionAtBytePackin[0] = rtB.UnitDelay;
  rtB.TmpSignalConversionAtBytePackin[1] = rtb_Sum1;

  /* S-Function (xpcbytepacking): '<S44>/Byte Packing ' */

  /* Byte Packing: <S44>/Byte Packing  */
  (void)memcpy((uint8_T*)&rtB.BytePacking_b[0] + 0, (uint8_T*)
               &rtB.TmpSignalConversionAtBytePackin[0], 8);

  /* S-Function (fcan_s32k_send): '<S44>/FCAN_Send' incorporates:
   *  Constant: '<S44>/Constant'
   */
  {
    flexcan_data_info_t txInfo = {
      .msg_id_type = FLEXCAN_MSG_ID_STD,
      .data_length = ((uint8_T)8U),
      .fd_enable = false,
      .fd_padding = 0x0,
      .enable_brs = false,
      .is_remote = false
    };

    FLEXCAN_DRV_ConfigTxMb(0, 0, &txInfo, 0x19);
    FLEXCAN_DRV_Send(0, 0, &txInfo, 0x19, &rtB.BytePacking_b[0]);
  }

  /* SignalConversion generated from: '<S44>/Byte Packing 1' */
  rtB.TmpSignalConversionAtBytePack_m[0] = rtB.DiscreteTimeIntegrator3;
  rtB.TmpSignalConversionAtBytePack_m[1] = rtB.calculateus;

  /* S-Function (xpcbytepacking): '<S44>/Byte Packing 1' */

  /* Byte Packing: <S44>/Byte Packing 1 */
  (void)memcpy((uint8_T*)&rtB.BytePacking1[0] + 0, (uint8_T*)
               &rtB.TmpSignalConversionAtBytePack_m[0], 8);

  /* S-Function (fcan_s32k_send): '<S44>/FCAN_Send1' incorporates:
   *  Constant: '<S44>/Constant2'
   */
  {
    flexcan_data_info_t txInfo = {
      .msg_id_type = FLEXCAN_MSG_ID_STD,
      .data_length = ((uint8_T)8U),
      .fd_enable = false,
      .fd_padding = 0x0,
      .enable_brs = false,
      .is_remote = false
    };

    FLEXCAN_DRV_ConfigTxMb(0, 1, &txInfo, 0x1A);
    FLEXCAN_DRV_Send(0, 1, &txInfo, 0x1A, &rtB.BytePacking1[0]);
  }

  /* Sum: '<S59>/Sum' incorporates:
   *  Constant: '<S59>/Constant'
   *  Gain: '<S59>/Gain'
   *  Gain: '<S61>/Gain'
   *  Product: '<S61>/Product'
   *  Sum: '<S27>/Sum1'
   */
  rtB.Saturation = ((0.0F - rtB.u * rtB.Gain_c * 0.0F) - rtB.Sum5) * 0.0003162F
    + 0.5F;

  /* Saturate: '<S59>/Saturation' */
  if (rtB.Saturation > 0.76F) {
    /* Sum: '<S59>/Sum' */
    rtB.Saturation = 0.76F;
  } else {
    if (rtB.Saturation < 0.24F) {
      /* Sum: '<S59>/Sum' */
      rtB.Saturation = 0.24F;
    }
  }

  /* End of Saturate: '<S59>/Saturation' */

  /* S-Function (ftm_s32k_pwm_config): '<S45>/FTM_PWM_Config' incorporates:
   *  Constant: '<S45>/Constant'
   */
  {
    uint16_t dutyA = FTM_MAX_DUTY_CYCLE * rtB.Saturation;
    FTM_DRV_UpdatePwmChannel(FTM_PWM0, 0U, FTM_PWM_UPDATE_IN_DUTY_CYCLE, dutyA,
      0, true);
  }

  /* Simulation Outputs */
  rtB.FTM_PWM_Config_o1 = (float) rtB.Saturation;
  rtB.FTM_PWM_Config_o2 = (float) 0.0F;

  /* S-Function (bicycle_model): '<S28>/S-Function Builder' incorporates:
   *  Constant: '<S28>/Constant'
   *  Constant: '<S28>/Constant1'
   */
  bicycle_model_Outputs_wrapper(&rtB.u, &rtB.Gain_c,
    &rtB.DiscreteTimeIntegrator3, &rtConstP.Constant_Value,
    &rtConstP.Constant1_Value, &rtB.SFunctionBuilder_o1,
    &rtB.SFunctionBuilder_o2, &rtB.SFunctionBuilder_o3);

  /* S-Function (get_rvec1): '<S64>/Look up Right Vector' */
  {
    extern struct road_seg_type rs[MAX_RS];
    real_T p, u_alt;
    int i;

    /* keep u within the range [0, PATH_LENGTH) */
    u_alt = fmod( (rtB.UnitDelay), PATH_LENGTH);

    /* Find the segment in rs[] that the path distance u_alt lies within */
    for (i=0; i<MAX_RS; i++) {
      if (rs[i].p > u_alt) {
        i--;
        break;
      }
    }

    /* i is the segment in which u lies; p is the path length from the
     * beginning of segment i to u_alt */
    p = u_alt - rs[i].p;
    switch ( rs[i].type)
    {
     case STRAIGHT:
      /* if path is straight, then the "right" vector is normal to
       * the vector that points straight ahead */
      {
        double fx, fy;                 /* unit vector straight ahead */
        double length;                 /* length of vector from start to stop */
        fx = rs[i].data.straight.stop[0] - rs[i].data.straight.start[0];
        fy = rs[i].data.straight.stop[1] - rs[i].data.straight.start[1];
        length = sqrt(fx*fx + fy*fy);
        fx /= length;
        fy /= length;

        /* <x,y,0> =  <fx,fy,0> x <0,0,1> */
        rtB.LookupRightVector_o1_c = fy;
        rtB.LookupRightVector_o2_j = -fx;
        break;
      }

     case CONST_CURVE:
      {
        double r;                      /* right angle */
        if (rs[i].data.const_curve.dir > 0.0) {
          r = p/(rs[i].data.const_curve.r) + rs[i].data.const_curve.ts;
        } else {
          r = -p/(rs[i].data.const_curve.r) + rs[i].data.const_curve.ts
            + 3.14159265358979;
        }

        rtB.LookupRightVector_o1_c = cos(r);
        rtB.LookupRightVector_o2_j = sin(r);
        break;
      }

     default :
      /* assign no value */
      break;
    }
  }

  /* Product: '<S66>/Product' incorporates:
   *  Gain: '<S64>/Gain'
   */
  rtb_Product_o_idx_0 *= -rtB.LookupRightVector_o2_j;
  rtb_Product_o_idx_1 *= rtB.LookupRightVector_o1_c;

  /* DataTypeConversion: '<S62>/Data  Type  Conversion' incorporates:
   *  Sum: '<S66>/Sum1'
   */
  rtb_DataTypeConversion = rtb_Product_o_idx_0 + rtb_Product_o_idx_1;

  /* DiscreteStateSpace: '<S62>/Controller' */
  {
    rtb_dsdt = 0.38289220218870362*rtDW.Controller_DSTATE;
    rtb_dsdt += 150.12501387153085*rtb_DataTypeConversion;
  }

  /* Sum: '<S65>/Sum' incorporates:
   *  DataTypeConversion: '<S63>/Conversion'
   *  Gain: '<S65>/Gain'
   *  UnitDelay: '<S65>/Unit Delay'
   */
  rtDW.UnitDelay_DSTATE = 0.01F * (real32_T)rtb_dsdt + rtB.UnitDelay;

  /* Update for DiscreteIntegrator: '<S28>/Discrete-Time Integrator1' */
  rtDW.DiscreteTimeIntegrator1_SYSTEM_ = 0U;
  rtDW.DiscreteTimeIntegrator1_PREV_U = rtB.SFunctionBuilder_o1;

  /* Update for DiscreteIntegrator: '<S28>/Discrete-Time Integrator2' */
  rtDW.DiscreteTimeIntegrator2_SYSTEM_ = 0U;
  rtDW.DiscreteTimeIntegrator2_PREV_U = rtB.SFunctionBuilder_o2;

  /* Update for DiscreteIntegrator: '<S28>/Discrete-Time Integrator3' */
  rtDW.DiscreteTimeIntegrator3_SYSTEM_ = 0U;
  rtDW.DiscreteTimeIntegrator3_DSTATE = rtB.DiscreteTimeIntegrator3;
  rtDW.DiscreteTimeIntegrator3_PREV_U = rtB.SFunctionBuilder_o3;

  /* Update for DiscreteIntegrator: '<S28>/Discrete-Time Integrator' incorporates:
   *  Gain: '<S28>/Gain'
   *  Gain: '<S28>/Gain1'
   *  Sum: '<S28>/Sum'
   */
  rtDW.DiscreteTimeIntegrator_SYSTEM_E = 0U;
  rtDW.DiscreteTimeIntegrator_DSTATE = rtB.u;
  rtDW.DiscreteTimeIntegrator_PREV_U = (rtB.Merge - 100.0F * rtB.u) * 0.0005F;

  /* Update for Delay: '<S41>/Delay' */
  rtDW.Delay_DSTATE_h = rtB.Quadrature_Decoder_o1;

  /* Update for DiscreteStateSpace: '<S62>/Controller' */
  {
    real_T xnew[1];
    xnew[0] = 1.0*rtDW.Controller_DSTATE;
    xnew[0] += 0.65292519369349644*rtb_DataTypeConversion;
    (void) memcpy(&rtDW.Controller_DSTATE, xnew,
                  sizeof(real_T)*1);
  }

  /* End of Outputs for S-Function (fcncallgen): '<Root>/Function-Call Top' */

  /* RateTransition: '<Root>/Rate Transition' incorporates:
   *  Product: '<S46>/Product'
   *  Product: '<S53>/Product'
   *  Product: '<S54>/Product'
   *  Product: '<S55>/Product'
   *  Product: '<S56>/Product'
   *  Product: '<S57>/Product'
   *  Product: '<S58>/Product'
   *  SignalConversion generated from: '<S2>/Serial'
   *  Sum: '<S46>/Sum'
   *  Sum: '<S53>/Sum'
   *  Sum: '<S54>/Sum'
   *  Sum: '<S55>/Sum'
   *  Sum: '<S56>/Sum'
   *  Sum: '<S57>/Sum'
   *  Sum: '<S58>/Sum'
   */
  if (rtM->Timing.RateInteraction.TID1_2) {
    /* S-Function (fcncallgen): '<Root>/Function-Call Top' incorporates:
     *  SubSystem: '<Root>/High Level Design'
     */
    rtDW.RateTransition_Buffer[2] = rtB.DiscreteTimeIntegrator3;
    rtDW.RateTransition_Buffer[3] = rtB.u;
    rtDW.RateTransition_Buffer[6] = rtB.snpsiusCars16[2];
    rtDW.RateTransition_Buffer[9] = rtB.snpsiusCars16[6];
    rtDW.RateTransition_Buffer[12] = rtB.snpsiusCars16[10];
    rtDW.RateTransition_Buffer[15] = rtB.snpsiusCars16[14];
    rtDW.RateTransition_Buffer[18] = rtB.snpsiusCars16[18];
    rtDW.RateTransition_Buffer[0] = rtb_Sum1 * rtB.LookupRightVector1_o1 +
      rtB.LookupP1_o1;
    rtDW.RateTransition_Buffer[4] = rtB.snpsiusCars16[1] *
      rtB.LookupRightVector1_o1_j + rtB.LookupP1_o1_j;
    rtDW.RateTransition_Buffer[7] = rtB.snpsiusCars16[5] *
      rtB.LookupRightVector1_o1_i + rtB.LookupP1_o1_f;
    rtDW.RateTransition_Buffer[10] = rtB.snpsiusCars16[9] *
      rtB.LookupRightVector1_o1_a + rtB.LookupP1_o1_jz;
    rtDW.RateTransition_Buffer[13] = rtB.snpsiusCars16[13] *
      rtB.LookupRightVector1_o1_c + rtB.LookupP1_o1_a;
    rtDW.RateTransition_Buffer[16] = rtB.snpsiusCars16[17] *
      rtB.LookupRightVector1_o1_iv + rtB.LookupP1_o1_c;
    rtDW.RateTransition_Buffer[19] = rtB.snpsiusCars16[21] *
      rtB.LookupRightVector1_o1_je + rtB.LookupP1_o1_g;
    rtDW.RateTransition_Buffer[1] = rtb_Sum1 * rtB.LookupRightVector1_o2 +
      rtB.LookupP1_o2;
    rtDW.RateTransition_Buffer[5] = rtB.snpsiusCars16[1] *
      rtB.LookupRightVector1_o2_m + rtB.LookupP1_o2_m;
    rtDW.RateTransition_Buffer[8] = rtB.snpsiusCars16[5] *
      rtB.LookupRightVector1_o2_d + rtB.LookupP1_o2_p;
    rtDW.RateTransition_Buffer[11] = rtB.snpsiusCars16[9] *
      rtB.LookupRightVector1_o2_j + rtB.LookupP1_o2_pi;
    rtDW.RateTransition_Buffer[14] = rtB.snpsiusCars16[13] *
      rtB.LookupRightVector1_o2_e + rtB.LookupP1_o2_l;
    rtDW.RateTransition_Buffer[17] = rtB.snpsiusCars16[17] *
      rtB.LookupRightVector1_o2_f + rtB.LookupP1_o2_e;
    rtDW.RateTransition_Buffer[20] = rtB.snpsiusCars16[21] *
      rtB.LookupRightVector1_o2_b + rtB.LookupP1_o2_n;
    rtDW.RateTransition_Buffer[21] = rtB.snpsiusCars16[22];

    /* End of Outputs for S-Function (fcncallgen): '<Root>/Function-Call Top' */
  }

  /* End of RateTransition: '<Root>/Rate Transition' */

  /* Update absolute time */
  /* The "clockTick1" counts the number of times the code of this task has
   * been executed. The resolution of this integer timer is 0.01, which is the step size
   * of the task. Size of "clockTick1" ensures timer will not overflow during the
   * application lifespan selected.
   */
  rtM->Timing.clockTick1++;
}

/* Model step function for TID2 */
void Final_project_template_step2(void) /* Sample time: [0.05s, 0.0s] */
{
  /* RateTransition: '<Root>/Rate Transition' */
  memcpy(&rtB.RateTransition[0], &rtDW.RateTransition_Buffer[0], 22U * sizeof
         (real32_T));

  /* S-Function (fcncallgen): '<S3>/Function-Call Generator1' incorporates:
   *  SubSystem: '<S3>/Serial'
   */
  /* S-Function (xpcbytepacking): '<S68>/Byte Packing ' */

  /* Byte Packing: <S68>/Byte Packing  */
  (void)memcpy((uint8_T*)&rtB.BytePacking[0] + 0, (uint8_T*)&rtB.RateTransition
               [0], 88);

  /* Logic: '<S68>/NOT' incorporates:
   *  Delay: '<S68>/Delay'
   */
  rtDW.Delay_DSTATE_j = !rtDW.Delay_DSTATE_j;

  /* End of Outputs for S-Function (fcncallgen): '<S3>/Function-Call Generator1' */

  /* RateTransition: '<S3>/RT' incorporates:
   *  Delay: '<S68>/Delay'
   */
  rtDW.RT_Buffer0 = rtDW.Delay_DSTATE_j;

  /* S-Function (fcncallgen): '<S3>/Function-Call Generator1' incorporates:
   *  SubSystem: '<S3>/Serial'
   */
  /* RateTransition: '<S3>/RT1' incorporates:
   *  Constant: '<S68>/Constant2'
   *  Constant: '<S68>/Constant3'
   *  SignalConversion generated from: '<S68>/data_out'
   */
  rtDW.RT1_Buffer0[0] = 115U;
  memcpy(&rtDW.RT1_Buffer0[1], &rtB.BytePacking[0], 88U * sizeof(uint8_T));
  rtDW.RT1_Buffer0[89] = 101U;

  /* End of Outputs for S-Function (fcncallgen): '<S3>/Function-Call Generator1' */
}

/* Model step wrapper function for compatibility with a static main program */
void Final_project_template_step(int_T tid)
{
  switch (tid) {
   case 0 :
    Final_project_template_step0();
    break;

   case 1 :
    Final_project_template_step1();
    break;

   case 2 :
    Final_project_template_step2();
    break;

   default :
    break;
  }
}

/* Model initialize function */
void Final_project_template_initialize(void)
{
  /* Start for S-Function (fcan_s32k_config): '<Root>/FCAN_Config' */
  {
    const flexcan_user_config_t canCom0_InitConfig = {
      .fd_enable = false,
      .pe_clock = FLEXCAN_CLK_SOURCE_PERIPH,
      .max_num_mb = 32U,
      .num_id_filters = FLEXCAN_RX_FIFO_ID_FILTERS_8,
      .is_rx_fifo_needed = false,
      .flexcanMode = FLEXCAN_NORMAL_MODE,
      .payload = FLEXCAN_PAYLOAD_SIZE_8,
      .bitrate = {
        .propSeg = 7U,
        .phaseSeg1 = 4U,
        .phaseSeg2 = 1U,
        .preDivider = 9U,
        .rJumpwidth = 3U
      },
      .bitrate_cbt = {
        .propSeg = 6U,
        .phaseSeg1 = 7U,
        .phaseSeg2 = 7U,
        .preDivider = 3U,
        .rJumpwidth = 3U
      },
      .transfer_type = FLEXCAN_RXFIFO_USING_INTERRUPTS,
      .rxFifoDMAChannel = 0U
    };

    /* CAN RX pin config */
    PCC_SetClockMode(PCC, PCC_PORTE_CLOCK, true);
    PINS_DRV_SetMuxModeSel(PORTE, 4, PORT_MUX_ALT5);

    /* CAN TX pin config */
    PCC_SetClockMode(PCC, PCC_PORTE_CLOCK, true);
    PINS_DRV_SetMuxModeSel(PORTE, 5, PORT_MUX_ALT5);

    /* Enable CAN0 clock */
    PCC_SetClockMode(PCC, PCC_FlexCAN0_CLOCK, true);
    FLEXCAN_DRV_Init(0, &canCom0_State, &canCom0_InitConfig);
  }

  /* Start for S-Function (lpuart_s32k_config): '<Root>/LPUART_Config' */
  {
    static lpuart_state_t lpuartState;

    /* Enable clock for PORTC */
    PCC_SetClockMode(PCC, PCC_PORTC_CLOCK, true);

    /* Configure pin for RX function */
    PINS_SetMuxModeSel(PORTC, 6, PORT_MUX_ALT2);

    /* Enable clock for PORTC */
    PCC_SetClockMode(PCC, PCC_PORTC_CLOCK, true);

    /* Configure pin for TX function */
    PINS_SetMuxModeSel(PORTC, 7, PORT_MUX_ALT2);

    /* Enable clock for PORTA */
    PCC_SetClockMode(PCC, PCC_PORTA_CLOCK, true);

    /* Configure pin for CTS function */
    PINS_SetMuxModeSel(PORTA, 6, PORT_MUX_ALT6);

    /* Enable clock for PORTA */
    PCC_SetClockMode(PCC, PCC_PORTA_CLOCK, true);

    /* Configure pin for RTS function */
    PINS_SetMuxModeSel(PORTA, 7, PORT_MUX_ALT6);

    /* Set LPUART clock source */
    PCC_SetPeripheralClockControl(PCC, PCC_LPUART1_CLOCK, true,
      CLK_SRC_FIRC_DIV2, 0, 0);

    /* Enable LPUART clock */
    PCC_SetClockMode(PCC, PCC_LPUART1_CLOCK, true);
    const lpuart_user_config_t lpuart1_config = {
      .transferType = LPUART_USING_INTERRUPTS,
      .baudRate = 115200U,
      .parityMode = LPUART_PARITY_DISABLED,
      .stopBitCount = LPUART_ONE_STOP_BIT,
      .bitCountPerChar = LPUART_8_BITS_PER_CHAR,
      .rxDMAChannel = 0U,
      .txDMAChannel = 0U,
    };

    /* Initializes a LPUART instance for operation */
    LPUART_DRV_Init(1, &lpuartState, &lpuart1_config);
  }

  /* Start for S-Function (fcan_s32k_receive): '<S4>/FCAN_Receive' */
  {
    flexcan_msgbuff_id_type_t msg_id_type = FLEXCAN_MSG_ID_STD;
    flexcan_data_info_t rxInfo = {
      .msg_id_type = FLEXCAN_MSG_ID_STD,
      .data_length = 8,
      .fd_enable = false,
      .fd_padding = 0,
      .enable_brs = false,
      .is_remote = false
    };

    FLEXCAN_DRV_ConfigRxMb(0, 16, &rxInfo, 0x0F);
    FLEXCAN_DRV_SetRxMaskType(0, FLEXCAN_RX_MASK_INDIVIDUAL);
    FLEXCAN_DRV_SetRxIndividualMask(0, msg_id_type, 16, 0x0);
  }

  /* Start for S-Function (fcan_s32k_receive): '<S4>/FCAN_Receive1' */
  {
    flexcan_msgbuff_id_type_t msg_id_type = FLEXCAN_MSG_ID_STD;
    flexcan_data_info_t rxInfo = {
      .msg_id_type = FLEXCAN_MSG_ID_STD,
      .data_length = 8,
      .fd_enable = false,
      .fd_padding = 0,
      .enable_brs = false,
      .is_remote = false
    };

    FLEXCAN_DRV_ConfigRxMb(0, 17, &rxInfo, 0x10);
    FLEXCAN_DRV_SetRxMaskType(0, FLEXCAN_RX_MASK_INDIVIDUAL);
    FLEXCAN_DRV_SetRxIndividualMask(0, msg_id_type, 17, 0x0);
  }

  /* Start for S-Function (fcan_s32k_receive): '<S4>/FCAN_Receive10' */
  {
    flexcan_msgbuff_id_type_t msg_id_type = FLEXCAN_MSG_ID_STD;
    flexcan_data_info_t rxInfo = {
      .msg_id_type = FLEXCAN_MSG_ID_STD,
      .data_length = 8,
      .fd_enable = false,
      .fd_padding = 0,
      .enable_brs = false,
      .is_remote = false
    };

    FLEXCAN_DRV_ConfigRxMb(0, 12, &rxInfo, 0x41);
    FLEXCAN_DRV_SetRxMaskType(0, FLEXCAN_RX_MASK_INDIVIDUAL);
    FLEXCAN_DRV_SetRxIndividualMask(0, msg_id_type, 12, 0x0);
  }

  /* Start for S-Function (fcan_s32k_receive): '<S4>/FCAN_Receive11' */
  {
    flexcan_msgbuff_id_type_t msg_id_type = FLEXCAN_MSG_ID_STD;
    flexcan_data_info_t rxInfo = {
      .msg_id_type = FLEXCAN_MSG_ID_STD,
      .data_length = 8,
      .fd_enable = false,
      .fd_padding = 0,
      .enable_brs = false,
      .is_remote = false
    };

    FLEXCAN_DRV_ConfigRxMb(0, 13, &rxInfo, 0x42);
    FLEXCAN_DRV_SetRxMaskType(0, FLEXCAN_RX_MASK_INDIVIDUAL);
    FLEXCAN_DRV_SetRxIndividualMask(0, msg_id_type, 13, 0x0);
  }

  /* Start for S-Function (fcan_s32k_receive): '<S4>/FCAN_Receive12' */
  {
    flexcan_msgbuff_id_type_t msg_id_type = FLEXCAN_MSG_ID_STD;
    flexcan_data_info_t rxInfo = {
      .msg_id_type = FLEXCAN_MSG_ID_STD,
      .data_length = 8,
      .fd_enable = false,
      .fd_padding = 0,
      .enable_brs = false,
      .is_remote = false
    };

    FLEXCAN_DRV_ConfigRxMb(0, 2, &rxInfo, 0x4b);
    FLEXCAN_DRV_SetRxMaskType(0, FLEXCAN_RX_MASK_INDIVIDUAL);
    FLEXCAN_DRV_SetRxIndividualMask(0, msg_id_type, 2, 0x0);
  }

  /* Start for S-Function (fcan_s32k_receive): '<S4>/FCAN_Receive13' */
  {
    flexcan_msgbuff_id_type_t msg_id_type = FLEXCAN_MSG_ID_STD;
    flexcan_data_info_t rxInfo = {
      .msg_id_type = FLEXCAN_MSG_ID_STD,
      .data_length = 8,
      .fd_enable = false,
      .fd_padding = 0,
      .enable_brs = false,
      .is_remote = false
    };

    FLEXCAN_DRV_ConfigRxMb(0, 3, &rxInfo, 0x4c);
    FLEXCAN_DRV_SetRxMaskType(0, FLEXCAN_RX_MASK_INDIVIDUAL);
    FLEXCAN_DRV_SetRxIndividualMask(0, msg_id_type, 3, 0x0);
  }

  /* Start for S-Function (fcan_s32k_receive): '<S4>/FCAN_Receive14' */
  {
    flexcan_msgbuff_id_type_t msg_id_type = FLEXCAN_MSG_ID_STD;
    flexcan_data_info_t rxInfo = {
      .msg_id_type = FLEXCAN_MSG_ID_STD,
      .data_length = 8,
      .fd_enable = false,
      .fd_padding = 0,
      .enable_brs = false,
      .is_remote = false
    };

    FLEXCAN_DRV_ConfigRxMb(0, 14, &rxInfo, 0x69);
    FLEXCAN_DRV_SetRxMaskType(0, FLEXCAN_RX_MASK_INDIVIDUAL);
    FLEXCAN_DRV_SetRxIndividualMask(0, msg_id_type, 14, 0x0);
  }

  /* Start for S-Function (fcan_s32k_receive): '<S4>/FCAN_Receive15' */
  {
    flexcan_msgbuff_id_type_t msg_id_type = FLEXCAN_MSG_ID_STD;
    flexcan_data_info_t rxInfo = {
      .msg_id_type = FLEXCAN_MSG_ID_STD,
      .data_length = 8,
      .fd_enable = false,
      .fd_padding = 0,
      .enable_brs = false,
      .is_remote = false
    };

    FLEXCAN_DRV_ConfigRxMb(0, 15, &rxInfo, 0x6a);
    FLEXCAN_DRV_SetRxMaskType(0, FLEXCAN_RX_MASK_INDIVIDUAL);
    FLEXCAN_DRV_SetRxIndividualMask(0, msg_id_type, 15, 0x0);
  }

  /* Start for S-Function (fcan_s32k_receive): '<S4>/FCAN_Receive2' */
  {
    flexcan_msgbuff_id_type_t msg_id_type = FLEXCAN_MSG_ID_STD;
    flexcan_data_info_t rxInfo = {
      .msg_id_type = FLEXCAN_MSG_ID_STD,
      .data_length = 8,
      .fd_enable = false,
      .fd_padding = 0,
      .enable_brs = false,
      .is_remote = false
    };

    FLEXCAN_DRV_ConfigRxMb(0, 4, &rxInfo, 0x19);
    FLEXCAN_DRV_SetRxMaskType(0, FLEXCAN_RX_MASK_INDIVIDUAL);
    FLEXCAN_DRV_SetRxIndividualMask(0, msg_id_type, 4, 0x0);
  }

  /* Start for S-Function (fcan_s32k_receive): '<S4>/FCAN_Receive3' */
  {
    flexcan_msgbuff_id_type_t msg_id_type = FLEXCAN_MSG_ID_STD;
    flexcan_data_info_t rxInfo = {
      .msg_id_type = FLEXCAN_MSG_ID_STD,
      .data_length = 8,
      .fd_enable = false,
      .fd_padding = 0,
      .enable_brs = false,
      .is_remote = false
    };

    FLEXCAN_DRV_ConfigRxMb(0, 5, &rxInfo, 0x1a);
    FLEXCAN_DRV_SetRxMaskType(0, FLEXCAN_RX_MASK_INDIVIDUAL);
    FLEXCAN_DRV_SetRxIndividualMask(0, msg_id_type, 5, 0x0);
  }

  /* Start for S-Function (fcan_s32k_receive): '<S4>/FCAN_Receive4' */
  {
    flexcan_msgbuff_id_type_t msg_id_type = FLEXCAN_MSG_ID_STD;
    flexcan_data_info_t rxInfo = {
      .msg_id_type = FLEXCAN_MSG_ID_STD,
      .data_length = 8,
      .fd_enable = false,
      .fd_padding = 0,
      .enable_brs = false,
      .is_remote = false
    };

    FLEXCAN_DRV_ConfigRxMb(0, 6, &rxInfo, 0x23);
    FLEXCAN_DRV_SetRxMaskType(0, FLEXCAN_RX_MASK_INDIVIDUAL);
    FLEXCAN_DRV_SetRxIndividualMask(0, msg_id_type, 6, 0x0);
  }

  /* Start for S-Function (fcan_s32k_receive): '<S4>/FCAN_Receive5' */
  {
    flexcan_msgbuff_id_type_t msg_id_type = FLEXCAN_MSG_ID_STD;
    flexcan_data_info_t rxInfo = {
      .msg_id_type = FLEXCAN_MSG_ID_STD,
      .data_length = 8,
      .fd_enable = false,
      .fd_padding = 0,
      .enable_brs = false,
      .is_remote = false
    };

    FLEXCAN_DRV_ConfigRxMb(0, 7, &rxInfo, 0x24);
    FLEXCAN_DRV_SetRxMaskType(0, FLEXCAN_RX_MASK_INDIVIDUAL);
    FLEXCAN_DRV_SetRxIndividualMask(0, msg_id_type, 7, 0x0);
  }

  /* Start for S-Function (fcan_s32k_receive): '<S4>/FCAN_Receive6' */
  {
    flexcan_msgbuff_id_type_t msg_id_type = FLEXCAN_MSG_ID_STD;
    flexcan_data_info_t rxInfo = {
      .msg_id_type = FLEXCAN_MSG_ID_STD,
      .data_length = 8,
      .fd_enable = false,
      .fd_padding = 0,
      .enable_brs = false,
      .is_remote = false
    };

    FLEXCAN_DRV_ConfigRxMb(0, 8, &rxInfo, 0x2d);
    FLEXCAN_DRV_SetRxMaskType(0, FLEXCAN_RX_MASK_INDIVIDUAL);
    FLEXCAN_DRV_SetRxIndividualMask(0, msg_id_type, 8, 0x0);
  }

  /* Start for S-Function (fcan_s32k_receive): '<S4>/FCAN_Receive7' */
  {
    flexcan_msgbuff_id_type_t msg_id_type = FLEXCAN_MSG_ID_STD;
    flexcan_data_info_t rxInfo = {
      .msg_id_type = FLEXCAN_MSG_ID_STD,
      .data_length = 8,
      .fd_enable = false,
      .fd_padding = 0,
      .enable_brs = false,
      .is_remote = false
    };

    FLEXCAN_DRV_ConfigRxMb(0, 9, &rxInfo, 0x2e);
    FLEXCAN_DRV_SetRxMaskType(0, FLEXCAN_RX_MASK_INDIVIDUAL);
    FLEXCAN_DRV_SetRxIndividualMask(0, msg_id_type, 9, 0x0);
  }

  /* Start for S-Function (fcan_s32k_receive): '<S4>/FCAN_Receive8' */
  {
    flexcan_msgbuff_id_type_t msg_id_type = FLEXCAN_MSG_ID_STD;
    flexcan_data_info_t rxInfo = {
      .msg_id_type = FLEXCAN_MSG_ID_STD,
      .data_length = 8,
      .fd_enable = false,
      .fd_padding = 0,
      .enable_brs = false,
      .is_remote = false
    };

    FLEXCAN_DRV_ConfigRxMb(0, 10, &rxInfo, 0x37);
    FLEXCAN_DRV_SetRxMaskType(0, FLEXCAN_RX_MASK_INDIVIDUAL);
    FLEXCAN_DRV_SetRxIndividualMask(0, msg_id_type, 10, 0x0);
  }

  /* Start for S-Function (fcan_s32k_receive): '<S4>/FCAN_Receive9' */
  {
    flexcan_msgbuff_id_type_t msg_id_type = FLEXCAN_MSG_ID_STD;
    flexcan_data_info_t rxInfo = {
      .msg_id_type = FLEXCAN_MSG_ID_STD,
      .data_length = 8,
      .fd_enable = false,
      .fd_padding = 0,
      .enable_brs = false,
      .is_remote = false
    };

    FLEXCAN_DRV_ConfigRxMb(0, 11, &rxInfo, 0x38);
    FLEXCAN_DRV_SetRxMaskType(0, FLEXCAN_RX_MASK_INDIVIDUAL);
    FLEXCAN_DRV_SetRxIndividualMask(0, msg_id_type, 11, 0x0);
  }

  /* Start for S-Function (ftm_s32k_pwm_config): '<Root>/FTM_PWM_Config' incorporates:
   *  Constant: '<Root>/Constant'
   */

  /* Enable clock for PORTB */
  PCC_SetClockMode (PCC, PCC_PORTB_CLOCK, true);

  /* Pin is configured for FTM function */
  PINS_SetMuxModeSel (PORTB, 8, PORT_MUX_ALT2);

  /* Set FTM_3 clock source */
  PCC_SetPeripheralClockControl (PCC, FTM3_CLK, true, CLK_SRC_SPLL, 0, 0);

  /* Enable clock for FTM_3 */
  PCC_SetClockMode (PCC, FTM3_CLK, true);

  /* PWM3 initialization */
  FTM_DRV_Init (FTM_PWM3, &flexTimer_pwm3_InitConfig, &ftmStateStruct3);
  FTM_DRV_InitPwm (FTM_PWM3, &flexTimer_pwm3_PwmConfig);
  FTM_DRV_SetChnTriggerCmd(FTM3, 1, false);

  /* Start for S-Function (adc_s32k_config): '<Root>/ADC_Config' */
  {
    const adc_converter_config_t adc0_cfg = {
      .clockDivide = ADC_CLK_DIVIDE_1,
      .sampleTime = 1.0,
      .resolution = ADC_RESOLUTION_12BIT,
      .inputClock = ADC_CLK_ALT_1,
      .trigger = ADC_TRIGGER_SOFTWARE,
      .pretriggerSel = ADC_PRETRIGGER_SEL_PDB,
      .triggerSel = ADC_TRIGGER_SEL_TRGMUX,
      .dmaEnable = false,
      .voltageRef = ADC_VOLTAGEREF_VREF,
      .continuousConvEnable = false,
      .supplyMonitoringEnable = false
    };

    const adc_compare_config_t adc0_cmp_cfg = {
      .compareEnable = false,
      .compareGreaterThanEnable = false,
      .compareRangeFuncEnable = false,
      .compVal1 = 0,
      .compVal2 = 0
    };

    const adc_average_config_t adc0_avrg_cfg = {
      .hwAvgEnable = false,
      .hwAverage = ADC_AVERAGE_4
    };

    /* Enable ADC0 clock */
    PCC_SetClockMode(PCC, PCC_ADC0_CLOCK, false);

    /* Set ADC0 clock source */
    PCC_SetPeripheralClockControl(PCC, PCC_ADC0_CLOCK, true, CLK_SRC_SPLL, 0, 0);

    /* Enable ADC0 clock */
    PCC_SetClockMode(PCC, PCC_ADC0_CLOCK, true);
    ADC_DRV_Reset(0);

    /* Configure ADC0 */
    ADC_DRV_ConfigConverter(0, &adc0_cfg);
    ADC_DRV_SetSwPretrigger(0,ADC_SW_PRETRIGGER_DISABLED);
    ADC_DRV_ConfigHwCompare(0, &adc0_cmp_cfg);
    ADC_DRV_ConfigHwAverage(0, &adc0_avrg_cfg);

    /* Do calibration before initialize the ADC0. */
    ADC_DRV_AutoCalibration(0);
  }

  /* Start for S-Function (fcan_s32k_isr): '<S1>/FCAN_Isr' */
  FLEXCAN_DRV_InstallEventCallback(0, fcan0_s32k_rx_isr, (void *)0);

  /* End of Start for S-Function (fcan_s32k_isr): '<S1>/FCAN_Isr' */
  rtPrevZCX.ResettableSubsystem_Reset_ZCE = UNINITIALIZED_ZCSIG;

  /* SystemInitialize for S-Function (fcncallgen): '<Root>/Function-Call Top' incorporates:
   *  SubSystem: '<Root>/High Level Design'
   */

  /* Start for S-Function (gpio_s32k_input): '<S26>/Digital_Input' */
  {
    /* Enable clock for PORTE */
    PCC_SetClockMode(PCC, PCC_PORTE_CLOCK, true);

    /* Configure the input port init structure. */
    const pin_settings_config_t gpioPORTEPin14 = {
      .base = PORTE,
      .pinPortIdx = 14,
      .pullConfig = PORT_INTERNAL_PULL_NOT_ENABLED,
      .passiveFilter = false,
      .driveSelect = PORT_LOW_DRIVE_STRENGTH,
      .mux = PORT_MUX_AS_GPIO,
      .pinLock = false,
      .intConfig = PORT_DMA_INT_DISABLED,
      .clearIntFlag = true,
      .gpioBase = PTE,
      .direction = GPIO_INPUT_DIRECTION,
    };

    /* Initialize GPIPORTE14. */
    PINS_DRV_Init(1, &gpioPORTEPin14);
  }

  /* Start for S-Function (ftm_s32k_quadrature_decoder): '<S40>/Quadrature_Decoder' */

  /* Enable clock for PORTA */
  PCC_SetClockMode(PCC, PCC_PORTA_CLOCK, true);

  /* Pin is configured for FTM function */
  PINS_SetMuxModeSel(PORTA, 13, PORT_MUX_ALT6);

  /* Enable clock for PORTA */
  PCC_SetClockMode(PCC, PCC_PORTA_CLOCK, true);

  /* Pin is configured for FTM function */
  PINS_SetMuxModeSel(PORTA, 12, PORT_MUX_ALT6);

  /* Set FTM_2 clock source */
  PCC_SetPeripheralClockControl (PCC, FTM2_CLK, true, CLK_SRC_SPLL, 0, 0);

  /* Enable clock for FTM_2 */
  PCC_SetClockMode (PCC, FTM2_CLK, true);

  /* Initialize FTM instance QD decoder*/
  static ftm_state_t ftm2StateStruct;
  FTM_DRV_Init(2, &ftm2_qd_InitConfig, &ftm2StateStruct);

  /* Start QD decoder*/
  FTM_DRV_QuadDecodeStart(2, &ftm2_qd_Params);

  /* Start for S-Function (gpio_s32k_input): '<S38>/Digital_Input' */
  {
    /* Enable clock for PORTE */
    PCC_SetClockMode(PCC, PCC_PORTE_CLOCK, true);

    /* Configure the input port init structure. */
    const pin_settings_config_t gpioPORTEPin6 = {
      .base = PORTE,
      .pinPortIdx = 6,
      .pullConfig = PORT_INTERNAL_PULL_NOT_ENABLED,
      .passiveFilter = false,
      .driveSelect = PORT_LOW_DRIVE_STRENGTH,
      .mux = PORT_MUX_AS_GPIO,
      .pinLock = false,
      .intConfig = PORT_DMA_INT_DISABLED,
      .clearIntFlag = true,
      .gpioBase = PTE,
      .direction = GPIO_INPUT_DIRECTION,
    };

    /* Initialize GPIPORTE6. */
    PINS_DRV_Init(1, &gpioPORTEPin6);
  }

  /* Start for S-Function (gpio_s32k_input): '<S38>/Digital_Input1' */
  {
    /* Enable clock for PORTE */
    PCC_SetClockMode(PCC, PCC_PORTE_CLOCK, true);

    /* Configure the input port init structure. */
    const pin_settings_config_t gpioPORTEPin7 = {
      .base = PORTE,
      .pinPortIdx = 7,
      .pullConfig = PORT_INTERNAL_PULL_NOT_ENABLED,
      .passiveFilter = false,
      .driveSelect = PORT_LOW_DRIVE_STRENGTH,
      .mux = PORT_MUX_AS_GPIO,
      .pinLock = false,
      .intConfig = PORT_DMA_INT_DISABLED,
      .clearIntFlag = true,
      .gpioBase = PTE,
      .direction = GPIO_INPUT_DIRECTION,
    };

    /* Initialize GPIPORTE7. */
    PINS_DRV_Init(1, &gpioPORTEPin7);
  }

  /* Start for S-Function (gpio_s32k_input): '<S38>/Digital_Input2' */
  {
    /* Enable clock for PORTE */
    PCC_SetClockMode(PCC, PCC_PORTE_CLOCK, true);

    /* Configure the input port init structure. */
    const pin_settings_config_t gpioPORTEPin8 = {
      .base = PORTE,
      .pinPortIdx = 8,
      .pullConfig = PORT_INTERNAL_PULL_NOT_ENABLED,
      .passiveFilter = false,
      .driveSelect = PORT_LOW_DRIVE_STRENGTH,
      .mux = PORT_MUX_AS_GPIO,
      .pinLock = false,
      .intConfig = PORT_DMA_INT_DISABLED,
      .clearIntFlag = true,
      .gpioBase = PTE,
      .direction = GPIO_INPUT_DIRECTION,
    };

    /* Initialize GPIPORTE8. */
    PINS_DRV_Init(1, &gpioPORTEPin8);
  }

  /* Start for S-Function (gpio_s32k_input): '<S38>/Digital_Input3' */
  {
    /* Enable clock for PORTE */
    PCC_SetClockMode(PCC, PCC_PORTE_CLOCK, true);

    /* Configure the input port init structure. */
    const pin_settings_config_t gpioPORTEPin9 = {
      .base = PORTE,
      .pinPortIdx = 9,
      .pullConfig = PORT_INTERNAL_PULL_NOT_ENABLED,
      .passiveFilter = false,
      .driveSelect = PORT_LOW_DRIVE_STRENGTH,
      .mux = PORT_MUX_AS_GPIO,
      .pinLock = false,
      .intConfig = PORT_DMA_INT_DISABLED,
      .clearIntFlag = true,
      .gpioBase = PTE,
      .direction = GPIO_INPUT_DIRECTION,
    };

    /* Initialize GPIPORTE9. */
    PINS_DRV_Init(1, &gpioPORTEPin9);
  }

  /* Start for S-Function (gpio_s32k_input): '<S38>/Digital_Input4' */
  {
    /* Enable clock for PORTE */
    PCC_SetClockMode(PCC, PCC_PORTE_CLOCK, true);

    /* Configure the input port init structure. */
    const pin_settings_config_t gpioPORTEPin10 = {
      .base = PORTE,
      .pinPortIdx = 10,
      .pullConfig = PORT_INTERNAL_PULL_NOT_ENABLED,
      .passiveFilter = false,
      .driveSelect = PORT_LOW_DRIVE_STRENGTH,
      .mux = PORT_MUX_AS_GPIO,
      .pinLock = false,
      .intConfig = PORT_DMA_INT_DISABLED,
      .clearIntFlag = true,
      .gpioBase = PTE,
      .direction = GPIO_INPUT_DIRECTION,
    };

    /* Initialize GPIPORTE10. */
    PINS_DRV_Init(1, &gpioPORTEPin10);
  }

  /* Start for S-Function (gpio_s32k_input): '<S38>/Digital_Input5' */
  {
    /* Enable clock for PORTE */
    PCC_SetClockMode(PCC, PCC_PORTE_CLOCK, true);

    /* Configure the input port init structure. */
    const pin_settings_config_t gpioPORTEPin11 = {
      .base = PORTE,
      .pinPortIdx = 11,
      .pullConfig = PORT_INTERNAL_PULL_NOT_ENABLED,
      .passiveFilter = false,
      .driveSelect = PORT_LOW_DRIVE_STRENGTH,
      .mux = PORT_MUX_AS_GPIO,
      .pinLock = false,
      .intConfig = PORT_DMA_INT_DISABLED,
      .clearIntFlag = true,
      .gpioBase = PTE,
      .direction = GPIO_INPUT_DIRECTION,
    };

    /* Initialize GPIPORTE11. */
    PINS_DRV_Init(1, &gpioPORTEPin11);
  }

  /* Start for S-Function (gpio_s32k_input): '<S38>/Digital_Input6' */
  {
    /* Enable clock for PORTE */
    PCC_SetClockMode(PCC, PCC_PORTE_CLOCK, true);

    /* Configure the input port init structure. */
    const pin_settings_config_t gpioPORTEPin12 = {
      .base = PORTE,
      .pinPortIdx = 12,
      .pullConfig = PORT_INTERNAL_PULL_NOT_ENABLED,
      .passiveFilter = false,
      .driveSelect = PORT_LOW_DRIVE_STRENGTH,
      .mux = PORT_MUX_AS_GPIO,
      .pinLock = false,
      .intConfig = PORT_DMA_INT_DISABLED,
      .clearIntFlag = true,
      .gpioBase = PTE,
      .direction = GPIO_INPUT_DIRECTION,
    };

    /* Initialize GPIPORTE12. */
    PINS_DRV_Init(1, &gpioPORTEPin12);
  }

  /* Start for S-Function (gpio_s32k_input): '<S38>/Digital_Input7' */
  {
    /* Enable clock for PORTE */
    PCC_SetClockMode(PCC, PCC_PORTE_CLOCK, true);

    /* Configure the input port init structure. */
    const pin_settings_config_t gpioPORTEPin13 = {
      .base = PORTE,
      .pinPortIdx = 13,
      .pullConfig = PORT_INTERNAL_PULL_NOT_ENABLED,
      .passiveFilter = false,
      .driveSelect = PORT_LOW_DRIVE_STRENGTH,
      .mux = PORT_MUX_AS_GPIO,
      .pinLock = false,
      .intConfig = PORT_DMA_INT_DISABLED,
      .clearIntFlag = true,
      .gpioBase = PTE,
      .direction = GPIO_INPUT_DIRECTION,
    };

    /* Initialize GPIPORTE13. */
    PINS_DRV_Init(1, &gpioPORTEPin13);
  }

  /* Start for S-Function (gpio_s32k_input): '<S26>/Digital_Input1' */
  {
    /* Enable clock for PORTE */
    PCC_SetClockMode(PCC, PCC_PORTE_CLOCK, true);

    /* Configure the input port init structure. */
    const pin_settings_config_t gpioPORTEPin15 = {
      .base = PORTE,
      .pinPortIdx = 15,
      .pullConfig = PORT_INTERNAL_PULL_NOT_ENABLED,
      .passiveFilter = false,
      .driveSelect = PORT_LOW_DRIVE_STRENGTH,
      .mux = PORT_MUX_AS_GPIO,
      .pinLock = false,
      .intConfig = PORT_DMA_INT_DISABLED,
      .clearIntFlag = true,
      .gpioBase = PTE,
      .direction = GPIO_INPUT_DIRECTION,
    };

    /* Initialize GPIPORTE15. */
    PINS_DRV_Init(1, &gpioPORTEPin15);
  }

  /* Start for S-Function (ftm_s32k_pwm_config): '<S45>/FTM_PWM_Config' incorporates:
   *  Constant: '<S45>/Constant'
   */

  /* Enable clock for PORTB */
  PCC_SetClockMode (PCC, PCC_PORTB_CLOCK, true);

  /* Pin is configured for FTM function */
  PINS_SetMuxModeSel (PORTB, 12, PORT_MUX_ALT2);

  /* Set FTM_0 clock source */
  PCC_SetPeripheralClockControl (PCC, FTM0_CLK, true, CLK_SRC_SPLL, 0, 0);

  /* Enable clock for FTM_0 */
  PCC_SetClockMode (PCC, FTM0_CLK, true);

  /* PWM0 initialization */
  FTM_DRV_Init (FTM_PWM0, &flexTimer_pwm0_InitConfig, &ftmStateStruct0);
  FTM_DRV_InitPwm (FTM_PWM0, &flexTimer_pwm0_PwmConfig);
  FTM_DRV_SetChnTriggerCmd(FTM0, 1, false);

  /* End of SystemInitialize for S-Function (fcncallgen): '<Root>/Function-Call Top' */

  /* Enable for S-Function (fcncallgen): '<Root>/Function-Call Top' incorporates:
   *  SubSystem: '<Root>/High Level Design'
   */
  rtDW.HighLevelDesign_RESET_ELAPS_T = true;

  /* Enable for DiscreteIntegrator: '<S28>/Discrete-Time Integrator1' */
  rtDW.DiscreteTimeIntegrator1_SYSTEM_ = 1U;

  /* Enable for DiscreteIntegrator: '<S28>/Discrete-Time Integrator2' */
  rtDW.DiscreteTimeIntegrator2_SYSTEM_ = 1U;

  /* Enable for DiscreteIntegrator: '<S28>/Discrete-Time Integrator3' */
  rtDW.DiscreteTimeIntegrator3_SYSTEM_ = 1U;

  /* Enable for DiscreteIntegrator: '<S28>/Discrete-Time Integrator' */
  rtDW.DiscreteTimeIntegrator_SYSTEM_E = 1U;

  /* End of Enable for S-Function (fcncallgen): '<Root>/Function-Call Top' */
}

/* Model terminate function */
void Final_project_template_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
