
#ifndef __VECTOR_H__
#define __VECTOR_H__

typedef struct {  double real;
                  double imagine;
                  double amplitude;
                  double angle;
		 	 	} VECTOR;

typedef VECTOR *VECTOR_handle;
/*-----------------------------------------------------------------------------
	Default initalizer for the CLARKE object.
-----------------------------------------------------------------------------*/                     
#define VECTOR_DEFAULTS { 0, \
                          0, \
                          0, \
                          0, \
              			} 

#define ab44  { 1.0773503,\
                0.28867513,\
				0,\
				0,\
               }
#define xy44  { 0.077350269,\
		        0.28867513,\
				0,\
				0,\
               }
#define ab64  { 0.78867513,\
	            0.78867513,\
				0,\
				0,\
               }
#define xy64  { -0.21132487,\
	            -0.21132487,\
				0,\
				0,\
               }
#define ab66  { 0.28867513,\
		        1.0773503,\
				0,\
				0,\
               }
#define xy66  { 0.28867513,\
		        0.077350269,\
				0,\
				0,\
               }
#define ab26  { -0.28867513,\
		        1.0773503,\
				0,\
				0,\
               }
#define xy26  { -0.28867513,\
		        0.077350269,\
				0,\
				0,\
               }
#define ab22  { -0.78867513,\
		        0.788675139,\
				0,\
				0,\
               }
#define xy22  { 0.21132487,\
		        -0.21132487,\
				0,\
				0,\
               }
#define ab32  { -1.0773503,\
		        0.28867513,\
				0,\
				0,\
               }
#define xy32  { -0.077350269,\
		        0.28867513,\
				0,\
				0,\
               }
#define ab33  { -1.0773503,\
		        -0.28867513,\
				0,\
				0,\
               }
#define xy33  { -0.077350269,\
                -0.28867513,\
				0,\
				0,\
               }
#define ab13  { -0.78867513,\
		        -0.78867513,\
				0,\
				0,\
               }
#define xy13  { 0.21132487,\
		        0.21132487,\
				0,\
				0,\
               }
#define ab11  { -0.28867513,\
		        -1.0773503,\
				0,\
				0,\
               }
#define xy11  { -0.28867513,\
	            -0.077350269,\
				0,\
				0,\
               }
#define ab51  { 0.28867513,\
		        -1.0773503,\
				0,\
				0,\
               }
#define xy51  { 0.28867513,\
		        -0.077350269,\
				0,\
				0,\
               }
#define ab55  { 0.78867513,\
		        -0.78867513,\
				0,\
				0,\
               }
#define xy55  { -0.21132487,\
		        0.21132487,\
				0,\
				0,\
               }
#define ab45  { 1.0773503,\
		        -0.28867513,\
				0,\
				0,\
               }
#define xy45  { 0.077350269,\
		        -0.28867513,\
				0,\
				0,\
               }
/*------------------------------------------------------------------------------
	VECTOR Transformation Macro Definition
------------------------------------------------------------------------------*/
	

#define VECTOR_MACRO(v)											\
																\
v.amplitude = sqrt(pow(v.real,2)+pow(v.imagine,2));              \
v.angle =atan(v.imagine,v.real);
#endif // __VECTOR_H__
