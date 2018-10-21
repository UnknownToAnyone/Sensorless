/* =================================================================================
File name:       PID_REG3.H  (IQ version)                    
                    
Originator:	Digital Control Systems Group
			Texas Instruments

Description: 
Header file containing constants, data type, and macro definitions for the PIDREG3.
=====================================================================================
 History:
-------------------------------------------------------------------------------------
 10-15-2009	Version 1.0
------------------------------------------------------------------------------*/
#ifndef __PIDREG3_H__
#define __PIDREG3_H__

typedef struct {  float  Ref;   			// Input: Reference input
				  float  Fdb;   			// Input: Feedback input
				  float  Err;				// Variable: Error
				  float  Kp;				// Parameter: Proportional gain
				  float  Up;				// Variable: Proportional output
				  float  Ui;				// Variable: Integral output
				  float  Ud;				// Variable: Derivative output
				  float  OutPreSat; 		// Variable: Pre-saturated output
				  float  OutMax;		    // Parameter: Maximum output
				  float  OutMin;	    	// Parameter: Minimum output
				  float  Out;   			// Output: PID output
				  float  SatErr;			// Variable: Saturated difference
				  float  Ki;			    // Parameter: Integral gain
				  float  Kc;		     	// Parameter: Integral correction gain
				  float  Kd; 		        // Parameter: Derivative gain
				  float  Up1;		   	    // History: Previous proportional output
		 	 	} PIDREG3;	            

typedef PIDREG3 *PIDREG3_handle;
/*-----------------------------------------------------------------------------
速度环，iqid环，ixiy环PI参数默认值
-----------------------------------------------------------------------------*/
#define spd_PI__DEFAULTS { 0, 			\
                           0, 			\
                           0, 			\
                           0.01, 	\
                           0, 			\
                           0, 			\
                           0, 			\
                           0, 			\
                           5, 		\
                           -5, 	\
                           0, 			\
                           0, 			\
                           0.01, 	\
                           0.1, 	\
                           1.05, 	\
                           0, 			\
              			  }

#define iqid_PI__DEFAULTS { 0, 			\
                           0, 			\
                           0, 			\
                           0.5, 	\
                           0, 			\
                           0, 			\
                           0, 			\
                           0, 			\
                           30, 		\
                           -30, 	\
                           0, 			\
                           0, 			\
                           0.05, 	  \
                           0.5, 	\
                           1.05, 	\
                           0, 			\
              			  }

#define ixiy_PI__DEFAULTS { 0, 			\
                           0, 			\
                           0, 			\
                           0.5, 	\
                           0, 			\
                           0, 			\
                           0, 			\
                           0, 			\
                           3, 		\
                           -3, 	\
                           0, 			\
                           0, 			\
                           0.05, 	\
                           0.5, 	\
                           1.05, 	\
                           0, 			\
              			  }
/*------------------------------------------------------------------------------
 	PID Macro Definition
------------------------------------------------------------------------------*/


#define PID_MACRO(v)																					\
	v.Err = v.Ref - v.Fdb; 									/* Compute the error */						\
	v.Up= v.Kp*v.Err;								/* Compute the proportional output */		\
	v.Ui= v.Ui + (v.Ki*v.Up) + (v.Kc*v.SatErr);	/* Compute the integral output */			\
	v.OutPreSat= v.Up + v.Ui;								/* Compute the pre-saturated output */		\
	v.Out = v.OutPreSat;		/* Saturate the output */					\
	if (v.OutPreSat>v.OutMax)							\
		v.Out = v.OutMax;								\
	else if (v.OutPreSat<v.OutMin)						\
		v.Out = v.OutMin;								\
	v.SatErr = v.Out - v.OutPreSat;							/* Compute the saturate difference */		\
	v.Up1 = v.Up;											/* Update the previous proportional output */


#define PID_CLEAR(v)																					\
	v.Err= 0; 					\
	v.Up= 0;					\
	v.Ui= 0;		            \
	v.OutPreSat= 0;	     		\
	v.Out = 0;					\

#endif // __PIDREG3_H__

// Add the lines below if derivative output is needed following the integral update
// v.Ud = _IQmpy(v.Kd,(v.Up - v.Up1)); 
// v.OutPreSat = v.Up + v.Ui + v.Ud; 
