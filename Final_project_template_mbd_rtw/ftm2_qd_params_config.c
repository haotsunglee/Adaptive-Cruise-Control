#include "ftm2_qd_params_config.h"

ftm_user_config_t ftm2_qd_InitConfig = {
  {
    true,                              /* Software trigger state */
    false,                             /* Hardware trigger 1 state */
    false,                             /* Hardware trigger 2 state */
    false,                             /* Hardware trigger 3 state */
    false,                             /* Max loading point state */
    false,                             /* Min loading point state */
    FTM_SYSTEM_CLOCK,                  /* Update mode for INVCTRL register */
    FTM_SYSTEM_CLOCK,                  /* Update mode for SWOCTRL register */
    FTM_SYSTEM_CLOCK,                  /* Update mode for OUTMASK register */
    FTM_SYSTEM_CLOCK,                  /* Update mode for CNTIN register */
    false,                             /* Automatic clear of the trigger*/
    FTM_UPDATE_NOW,                    /* Synchronization point */
  },
  FTM_MODE_QUADRATURE_DECODER,         /* Mode of operation for FTM */
  FTM_CLOCK_DIVID_BY_1,                /* FTM clock prescaler */
  FTM_CLOCK_SOURCE_SYSTEMCLK,          /* FTM clock source */
  FTM_BDM_MODE_00,                     /* FTM debug mode */
  false,                               /* Interrupt state */
  false                                /* Initialization trigger */
};

ftm_quad_decode_config_t ftm2_qd_Params = {
  FTM_QUAD_PHASE_ENCODE,               /* Quadrature decoder mode */
  0,                                   /* Initial counter value */
  65535,                               /* Maximum counter value */

  {
    false,                             /* Filter state */
    0,                                 /* Filter value */
    FTM_QUAD_PHASE_NORMAL              /* Phase polarity */
  },

  {
    false,                             /* Filter state */
    0,                                 /* Filter value */
    FTM_QUAD_PHASE_NORMAL              /* Phase polarity */
  }
};
