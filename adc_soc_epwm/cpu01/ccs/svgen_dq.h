/* =================================================================================
File name:       SVGEN_DQ.H  (IQ version)

Originator:	Digital Control Systems Group
			Texas Instruments

Description:
Header file containing constants, data type, and macro  definitions for the SVGEN_DQ module .
=====================================================================================
 History:
-------------------------------------------------------------------------------------
 10-15-2009	Version 1.0
------------------------------------------------------------------------------*/
#ifndef __SVGEN_DQ_H__
#define __SVGEN_DQ_H__
//extern float I_A;
//extern float I_B;
//extern float I_C;
//extern float ceshi1;


typedef struct 	{ double  Ualpha; 			// Input: reference alpha-axis phase voltage
				  double  Ubeta;			// Input: reference beta-axis phase voltage
				  double  Ux;			    // Input: reference x-axis phase voltage
				  double  Uy;			    // Input: reference y-axis phase voltage
				  float  MyTheta;			// Input: reference beta-axis phase voltage
				  float  Ta;				// Output: reference phase-a switching function
				  float  Tb;				// Output: reference phase-b switching function
				  float  Tc;				// Output: reference phase-c switching function
				  float  Td;				// Output: reference phase-d switching function
				  float  Te;				// Output: reference phase-e switching function
				  float  Tf;				// Output: reference phase-f switching function
				  float  Ta1;				// Output: reference phase-a switching function
				  float  Tb1;				// Output: reference phase-b switching function
				  float  Tc1;				// Output: reference phase-c switching function
				  float  Td1;				// Output: reference phase-d switching function
				  float  Te1;				// Output: reference phase-e switching function
				  float  Tf1;				// Output: reference phase-f switching function
				  //int	 tempA;
				  //int    tempB;
				  //int    tempC;
				} SVGENDQ;

typedef SVGENDQ *SVGENDQ_handle;
/*-----------------------------------------------------------------------------
Default initalizer for the SVGENDQ object.
-----------------------------------------------------------------------------*/
#define SVGENDQ_DEFAULTS {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}

/*------------------------------------------------------------------------------
	Space Vector PWM Generator (SVGEN_DQ) Macro Definition
------------------------------------------------------------------------------*/
#endif // __SVGEN_DQ_H__
