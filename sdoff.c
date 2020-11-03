#include "udf.h"
#include "dynamesh_tools.h"

#define nozzle_tid       15    /* Zone ID of nozzle exit */
#define missile_tid      14    /* Zone ID of missile wall */
#define moving_fluid_id  2     /* Zone ID of fluid surrounding the missile */

#define t_liftoff        0.1    /* Time at which to allow lift-off (sec) */
#define g_c              9.81   /* Gravity acceleration */
#define initial_mass     750.0  /* Initial mass of the rocket (kg) */
#define burn_rate        0.0    /* Fuel burn rate (kg/sec) */
#define pi               4.0*atan(1.0)

float U_sum;     /* Sum of inlet U velocity */
float V_sum;     /* Sum of inlet V velocity */
float W_sum;     /* Sum of inlet W velocity */
float A_sum;     /* Sum of facet areas */
float P_sum;     /* Sum of face pressure */
float P_i;       /* Pressure on face i */
float A_i;       /* Area of face i */
float U_i;       /* U velocity on face i */
float V_i;       /* V velocity on face i */
float W_i;       /* W velocity on face i */
float A[3];      /* Area vector of face i */
float VA_sum;    /* Sum of velocity times area */
float VEL_i;     /* Velocity magnitude on face i */
float V_avg;     /* Average velocity */
float V_e;       /* Nozzle exit velocity relative to missile */
float Pthrust;   /* Thrust due to pressure */
float Mthrust;   /* Thrust due to momentum */
float MDOT;      /* Mass flow rate from the missile */
float Vmiss[3];  /* Missile velocity vector */
/*float Umiss;*/     /* Missile x velocity */
/*float Vmiss;*/     /* Missile y or r velocity */
/*float Wmiss;*/     /* Missile z velocity (zero for 2-D) */

DEFINE_SDOF_PROPERTIES(launch, prop, dt, time, dtime)
{

/* Define the mass and moments of inertia  */
   prop[SDOF_MASS] = initial_mass - (burn_rate * time);
   prop[SDOF_IXX]        = 10.84;
   prop[SDOF_IYY]        = 1759.09;
   prop[SDOF_IZZ]        = 1759.09;


/* Define ejector moments  */
/*   prop[SDOF_LOAD_M_X] = 0.; */
/*  prop[SDOF_LOAD_M_Y] = 0.;*/
/*    prop[SDOF_LOAD_M_Z] = 0.;*/

/* Calculate rocket thrust and assign to X ejector force.*/


	
    V_avg = 1000;
	if(time > 0.2)
          MDOT = 33;
      else if(time > 0.15)
          MDOT = 20;
	  else if(time > 0.1)
          MDOT = 10;
	  else if(time > 0.05)
          MDOT = 5;
      else
          MDOT = 1;
        
	V_e = V_avg;
    Mthrust = V_e * MDOT;
		
/*  Set calculated thrust as a body force (ejector force)  */

	
	/*prop[SDOF_LOAD_F_X] = -(cos(DT_THETA(dt)[1])*cos(DT_THETA(dt)[2])*( Pthrust + Mthrust )); /*cos yaw * cos pitch*/
	/*prop[SDOF_LOAD_F_Y] = -(cos(DT_THETA(dt)[1])*sin(DT_THETA(dt)[2])*( Pthrust + Mthrust )); /*cos yaw * sin pitch*/
	/*prop[SDOF_LOAD_F_Z] = -(sin(DT_THETA(dt)[1])*( Pthrust + Mthrust )); /*sin yaw*/
	
	prop[SDOF_LOAD_F_X] = -(  Mthrust );
    
}

DEFINE_PROFILE(jetnozzle,th,i)
 {
   face_t f;
   real flow_time = CURRENT_TIME;
   begin_f_loop(f,th)
     {
       if(flow_time > 0.2)
          F_PROFILE(f,th,i) = 1;
      else if(flow_time > 0.15)
          F_PROFILE(f,th,i) = 0.8;
	  else if(flow_time > 0.1)
          F_PROFILE(f,th,i) = 0.4;
	  else if(flow_time > 0.05)
          F_PROFILE(f,th,i) = 0.2;
      else
          F_PROFILE(f,th,i) = 0.1;
     }
    end_f_loop(f,th);
 }