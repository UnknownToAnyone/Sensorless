/* =================================================================================
File name:       CLARKE.H  (IQ version)                  
                    
Originator:	Digital Control Systems Group
			Texas Instruments

Description: 
Header file containing constants, data type, and macro definition for the CLARKE.
=====================================================================================
 History:
-------------------------------------------------------------------------------------
 04-15-2009	Version 1.0                                                  
------------------------------------------------------------------------------*/
#ifndef __CLARKE_H__
#define __CLARKE_H__

typedef struct {  double  As;  		// Input: phase-a stator variable
				  double  Bs;			// Input: phase-b stator variable
				  double  Cs;			// Input: phase-c stator variable
				  double  Ds;			// Input: phase-d stator variable
				  double  Es;			// Input: phase-e stator variable
				  double  Fs;			// Input: phase-f stator variable
				  double  Alpha;		// Output: stationary d-axis stator variable
				  double  Beta;		// Output: stationary q-axis stator variable
				  double  X;
				  double  Y;
		 	 	} CLARKE;	            

typedef CLARKE *CLARKE_handle;
/*-----------------------------------------------------------------------------
	Default initalizer for the CLARKE object.
-----------------------------------------------------------------------------*/                     
#define CLARKE_DEFAULTS { 0, \
                          0, \
                          0, \
                          0, \
                          0, \
						  0, \
						  0, \
						  0, \
						  0, \
						  0, \
              			} 

/*------------------------------------------------------------------------------
	CLARKE Transformation Macro Definition
------------------------------------------------------------------------------*/
	
//sqrt(3)/2 = 0.8860254038
//1/sqrt(3) = 0.57735026918963
#define CLARKE_MACRO(v)											\
																\
v.Alpha = 0.57735026919*(v.As  -  0.5*v.Cs  -  0.5*v.Es  +  0.8860254038*v.Bs  -  0.8860254038*v.Ds);   \
v.Beta = 0.57735026919*(0.8860254038*v.Cs  -  0.8860254038*v.Es  +  0.5*v.Bs  +  0.5*v.Ds  -  v.Fs);    \
v.X = 0.57735026919*(v.As-0.5*v.Cs-0.5*v.Es-0.8860254038*v.Bs+0.8860254038*v.Ds);       \
v.Y = 0.57735026919*(-0.8860254038*v.Cs+0.8860254038*v.Es+0.5*v.Bs+0.5*v.Ds-v.Fs);
#endif // __CLARKE_H__
