
/*
 * Include Files
 *
 */
#if defined(MATLAB_MEX_FILE)
#include "tmwtypes.h"
#include "simstruc_types.h"
#else
#include "rtwtypes.h"
#endif



/* %%%-SFUNWIZ_wrapper_includes_Changes_BEGIN --- EDIT HERE TO _END */
#include <math.h>
/* %%%-SFUNWIZ_wrapper_includes_Changes_END --- EDIT HERE TO _BEGIN */
#define u_width 1
#define y_width 1

/*
 * Create external references here.  
 *
 */
/* %%%-SFUNWIZ_wrapper_externs_Changes_BEGIN --- EDIT HERE TO _END */
/* extern double func(double a); */
/* %%%-SFUNWIZ_wrapper_externs_Changes_END --- EDIT HERE TO _BEGIN */

/*
 * Output function
 *
 */
void bicycle_model_Outputs_wrapper(const real32_T *u,
			const real32_T *delta,
			const real32_T *psi,
			const real32_T *L1,
			const real32_T *L2,
			real32_T *x_dot,
			real32_T *y_dot,
			real32_T *psi_dot)
{
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_BEGIN --- EDIT HERE TO _END */
/* This sample sets the output equal to the input
      y0[0] = u0[0]; 
 For complex signals use: y0[0].re = u0[0].re; 
      y0[0].im = u0[0].im;
      y1[0].re = u1[0].re;
      y1[0].im = u1[0].im;
 */

x_dot[0] = ((-L2[0]/L1[0]) * sin(delta[0])*sin(psi[0]) + cos(delta[0])*cos(psi[0]))*u[0];
y_dot[0] = ((L2[0]/L1[0]) * sin(delta[0])*cos(psi[0]) + cos(delta[0])*sin(psi[0]))*u[0];
psi_dot[0] = sin(delta[0])*u[0]/L1[0];
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_END --- EDIT HERE TO _BEGIN */
}


