/* =================================================================================
File name:       PARK.H (IQ version)                    
                    
Originator:	Digital Control Systems Group
			Texas Instruments

Description: 
Header file containing constants, data type, and macro definitions for the PARK.
=====================================================================================
 History:
-------------------------------------------------------------------------------------
 10-15-2009	Version 1.0                                                  
------------------------------------------------------------------------------*/
#ifndef __PARK_H__
#define __PARK_H__

typedef struct {  float  Alpha;  		// Input: stationary d-axis stator variable
				  float  Beta;	 	// Input: stationary q-axis stator variable
				  float  Angle;		// Input: rotating angle (pu)
				  float  Ds;			// Output: rotating d-axis stator variable
				  float  Qs;			// Output: rotating q-axis stator variable
				  float  Sine;
				  float  Cosine;
		 	 	} PARK;	            

typedef PARK *PARK_handle;
/*-----------------------------------------------------------------------------
Default initalizer for the PARK object.
-----------------------------------------------------------------------------*/                     
#define PARK_DEFAULTS {   0, \
                          0, \
                          0, \
                          0, \
                          0, \
						  0, \
                          0, \
              			  }

/*------------------------------------------------------------------------------
	PARK Transformation Macro Definition
------------------------------------------------------------------------------*/


#define PARK_MACRO(v)											\
																\
	v.Ds = v.Alpha*v.Cosine + v.Beta*v.Sine;	\
    v.Qs = v.Beta*v.Cosine - v.Alpha*v.Sine;

#endif // __PARK_H__
