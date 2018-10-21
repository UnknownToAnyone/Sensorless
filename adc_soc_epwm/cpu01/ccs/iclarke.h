/* =================================================================================
File name:       ICLARKE.H  (IQ version)
                    
Originator:	Digital Control Systems Group
			Texas Instruments

Description: 
Header file containing constants, data type, and macro definition for the ICLARKE.
=====================================================================================
 History:
-------------------------------------------------------------------------------------
 04-15-2009	Version 1.0                                                  
------------------------------------------------------------------------------*/
#ifndef __ICLARKE_H__
#define __ICLARKE_H__

typedef struct {  float  As;  		// Input: phase-a stator variable
				  float  Bs;			// Input: phase-b stator variable
				  float  Cs;			// Input: phase-c stator variable
				  float  Ds;			// Input: phase-d stator variable
				  float  Es;			// Input: phase-e stator variable
				  float  Fs;			// Input: phase-f stator variable
				  float  Alpha;		// Output: stationary d-axis stator variable
				  float  Beta;		// Output: stationary q-axis stator variable
				  float  X;
				  float  Y;
		 	 	} ICLARKE;

typedef ICLARKE *ICLARKE_handle;
/*-----------------------------------------------------------------------------
	Default initalizer for the ICLARKE object.
-----------------------------------------------------------------------------*/                     
#define ICLARKE_DEFAULTS { 0, \
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
	ICLARKE Transformation Macro Definition
------------------------------------------------------------------------------*/
	
//sqrt(3)/2 = 0.8860254038
//1/sqrt(3) = 0.57735026918963
#define ICLARKE_MACRO(v)											\
		0.577350269189626   0.288675134594813							            	\
v.As = 0.577350269189626*v.Alpha + 0.577350269189626*v.X                             \
v.Bs = 0.5*v.Alpha + 0.288675134594813*v.Beta - 0.5*v.X + 0.288675134594813*v.Y      \
v.Cs = -0.288675134594813*v.Alpha + 0.5*v.Beta - 0.288675134594813*v.X - 0.5*v.Y     \
v.Ds = -0.5*v.Alpha + 0.288675134594813*v.Beta + 0.5*v.X + 0.288675134594813*v.Y     \
v.Es = -0.288675134594813*v.Alpha - 0.5*v.Beta - 0.288675134594813*v.X + 0.5*v.Y     \
v.Fs = -0.577350269189626*v.Beta - 0.577350269189626*v.Y
#endif // __ICLARKE_H__
