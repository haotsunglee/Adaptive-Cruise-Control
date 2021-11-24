
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
void ACClogic_Test_Outputs_wrapper(const boolean_T *en,
			const real32_T *s,
			const real32_T *u,
			const real32_T *H,
			boolean_T *en_vel,
			boolean_T *en_pos,
			real32_T *lead_s,
			real32_T *lead_us)
{
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_BEGIN --- EDIT HERE TO _END */

/* This sample sets the output equal to the input
      y0[0] = u0[0]; 
 For complex signals use: y0[0].re = u0[0].re; 
      y0[0].im = u0[0].im;
      y1[0].re = u1[0].re;
      y1[0].im = u1[0].im;
 */
int lead_car_i = 0;
int i;
int s_temp = 150;

for(i = 1; i < 7; ++i)
{
    if(s[i] > s[0] && s[i] < s_temp)
    {
        lead_car_i = i;  
        s_temp = s[i];
    }
}

if(lead_car_i == 0)
{
    en_vel[0] = true;
    en_pos[0] = false;
}
else
{
    if(u[lead_car_i] <= u[0] &&
            s[lead_car_i] - s[0] <= H[0])
    {
        en_vel[0] = false;
        en_pos[0] = true;
    }
//    else if(s[lead_car_i] != s[0])
//    {
//        en_vel[0] = false;
//        en_pos[0] = true;
//    }
    else
    {
        en_vel[0] = true;
        en_pos[0] = false;
    }
}

lead_s[0] = s[lead_car_i];
lead_us[0] = u[lead_car_i];

if(en[0])
{
    en_vel[0] = false;
    en_pos[0] = false;
}
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_END --- EDIT HERE TO _BEGIN */
}


