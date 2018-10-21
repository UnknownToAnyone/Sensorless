
//         Lowpass IIR filter calculated by MATLAB filter designer
//                gain*(1+numerator1*z^(-1)+numerator2*z^(-2))
//      z”Ú¥´µ›∫Ø ˝ :------------------------------------------------
//                  1+denominator1*z^(-1)+denominator2*z^(-2)

//       ±”Ú:y(n)=-denominator1*y(n-1)-denominator2*y(n-2)+gain*(x(n)+numerator1*x(n-1)+numerator2*x(n-2))
#ifndef __DIGITALFLITER_H__
#define __DIGITALFLITER_H__

typedef struct {  double  gain;
				  double  numerator1;
				  double  numerator2;
				  double  denominator1;
				  double  denominator2;
				  double  xn;			// x(n)
				  double  xn1;	       	// x(n-1)
				  double  xn2;		    // x(n-2)
				  double  yn;           // y(n)
				  double  yn1;          // y(n-1)
				  double  yn2;          // y(n-2)
		 	 	} LPFILTER;

typedef LPFILTER *LPFILTER_handle;

typedef struct {  double  gain;
				  double  numerator1;
				  double  numerator2;
				  double  denominator1;
				  double  denominator2;
				  double  xn;			// x(n)
				  double  xn1;	       	// x(n-1)
				  double  xn2;		    // x(n-2)
				  double  yn;           // y(n)
				  double  yn1;          // y(n-1)
				  double  yn2;          // y(n-2)
		 	 	} PRFILTER;

typedef PRFILTER *PRFILTER_MACRO_handle;

typedef struct {  double  gain;
				  double  numerator1;
				  double  numerator2;
				  double  denominator1;
				  double  denominator2;
				  double  xn;			// x(n)
				  double  xn1;	       	// x(n-1)
				  double  xn2;		    // x(n-2)
				  double  yn;           // y(n)
				  double  yn1;          // y(n-1)
				  double  yn2;          // y(n-2)
		 	 	} PLL;

typedef PRFILTER *PLL_MACRO_handle;

/*-----------------------------------------------------------------------------
	Default initalizer for the LPFILTER object.
-----------------------------------------------------------------------------*/
//      15s+1000     -1            0.00075025(1+0.0006664z^(-1)+0.9993336z^(-2))
//s”Ú£∫-----------*------   z”Ú£∫----------------------------------------------------
//         s          s                            1-2z^(-1)+z^(-2)
#define PLL_DEFAULTS { -0.0007525, \
	                 	0.0066445, \
						-0.9933555, \
                          -2, \
                          1, \
						  0, \
						  0, \
						  0, \
						  0, \
						  0, \
						  0, \
              			}
//      15s+1000
//s”Ú£∫-----------
//         s
#define PLL_PI1        {  15.05, \
	                 	 -0.9933555, \
						  0, \
                         -1, \
                          0, \
						  0, \
						  0, \
						  0, \
						  0, \
						  0, \
						  0, \
              			}
//      5s+1000
//s”Ú£∫-----------
//         s
#define PLL_PI2        {  5.05, \
	                 	 -0.98019802, \
						  0, \
                         -1, \
                          0, \
						  0, \
						  0, \
						  0, \
						  0, \
						  0, \
						  0, \
              			}
//      1s+1000
//s”Ú£∫-----------
//         s
#define PLL_PI3        {  1.05, \
	                 	 -0.904762, \
						  0, \
                         -1, \
                          0, \
						  0, \
						  0, \
						  0, \
						  0, \
						  0, \
						  0, \
              			}
//      1s+10000
//s”Ú£∫-----------
//         s
#define PLL_PI4        {  1.5, \
	                 	 -0.33333333333, \
						  0, \
                         -1, \
                          0, \
						  0, \
						  0, \
						  0, \
						  0, \
						  0, \
						  0, \
              			}
//        s+50
//s”Ú£∫-----------
//         s
#define PLL_PI5        {  1.0025, \
	                 	 -0.99501247, \
						  0, \
                         -1, \
                          0, \
						  0, \
						  0, \
						  0, \
						  0, \
						  0, \
						  0, \
              			}
//        s+25
//s”Ú£∫-----------
//         s
#define PLL_PI6        {  1.00125, \
	                 	 -0.99750312, \
						  0, \
                         -1, \
                          0, \
						  0, \
						  0, \
						  0, \
						  0, \
						  0, \
						  0, \
              			}
//        s+1
//s”Ú£∫-----------
//         s
#define PLL_PI7        {  1, \
	                 	 -0.99995, \
						  0, \
                         -1, \
                          0, \
						  0, \
						  0, \
						  0, \
						  0, \
						  0, \
						  0, \
              			}
//        s+0.5
//s”Ú£∫-----------
//         s
#define PLL_PI8        {  1, \
	                 	 -0.999975, \
						  0, \
                         -1, \
                          0, \
						  0, \
						  0, \
						  0, \
						  0, \
						  0, \
						  0, \
              			}
//        s+0.25
//s”Ú£∫-----------
//         s
#define PLL_PI9        {  1, \
	                 	 -0.9999875, \
						  0, \
                         -1, \
                          0, \
						  0, \
						  0, \
						  0, \
						  0, \
						  0, \
						  0, \
              			}
//        s+0.05
//s”Ú£∫-----------
//         s
#define PLL_PI10        {  1, \
	                 	 -0.9999975, \
						  0, \
                         -1, \
                          0, \
						  0, \
						  0, \
						  0, \
						  0, \
						  0, \
						  0, \
              			}
//        s+0.01
//s”Ú£∫-----------
//         s
#define PLL_PI11        {  1, \
	                 	 -0.9999995, \
						  0, \
                         -1, \
                          0, \
						  0, \
						  0, \
						  0, \
						  0, \
						  0, \
						  0, \
              			}
//        s+10
//s”Ú£∫-----------
//         s
#define PLL_PI12        {  1, \
	                 	 -0.9995, \
						  0, \
                         -1, \
                          0, \
						  0, \
						  0, \
						  0, \
						  0, \
						  0, \
						  0, \
              			}
#define PLL_I         {  0.00005, \
	                 	  1, \
						  0, \
                         -1, \
                          0, \
						  0, \
						  0, \
						  0, \
						  0, \
						  0, \
						  0, \
              			}
#define LPFILTER_DEFAULTS { 0.638945, \
                          2, \
                          1, \
                          1.1429, \
                          0.4128, \
						  0, \
						  0, \
						  0, \
						  0, \
						  0, \
						  0, \
              			}

//100rpm: 6felc=50Hz kr=10 wc=5 Fs=100kHz
#define PR_FILTER_100rpm { 0.049962691, \
                    0, \
                    -1, \
                    -1.9980145, \
					0.99900075, \
					0, \
					0, \
				    0, \
				    0, \
					0, \
					0, \
              	  }
//30rpm: 6felc=15Hz kr=100 wc=5 Fs=100kHz
#define PR_FILTER_30rpm { 0.049973903, \
                    0, \
                    -1, \
                    -1.9989117, \
                    0.99900052, \
					0, \
					0, \
				    0, \
				    0, \
					0, \
					0, \
              	  }
#define LPFILTER_Fs10k_Fc100 { 0.000944692, \
                          2, \
                          1, \
						  -1.9111970674260732, \
						  0.91497583480143374, \
						  0, \
						  0, \
						  0, \
						  0, \
						  0, \
						  0, \
              			} 
#define LPFILTER_Fs10k_Fc20 { 0.000039130205399144361, \
                          2, \
                          1, \
						  -1.9822289297925286, \
						  0.9823854506141253, \
						  0, \
						  0, \
						  0, \
						  0, \
						  0, \
						  0, \
              			}
#define LPFILTER_Fs10k_Fc50 { 0.00024135904904198073, \
                          2, \
                          1, \
						  -1.9555782403150355, \
						  0.95654367651120342, \
						  0, \
						  0, \
						  0, \
						  0, \
						  0, \
						  0, \
              			}
#define LPFILTER_Fs10k_Fc40 { 0.00015514842347569903, \
                          2, \
                          1, \
						  -1.964460580205232, \
						  0.96508117389913495, \
						  0, \
						  0, \
						  0, \
						  0, \
						  0, \
						  0, \
              			}
#define LPFILTER_Fs10k_Fc400 { 0.0133592, \
                          2, \
                          1, \
						  -1.64745998, \
						  0.70089678, \
						  0, \
						  0, \
						  0, \
						  0, \
						  0, \
						  0, \
              			}
#define LPFILTER_Fs10k_Fc1k { 0.067455273889071896, \
                          2, \
                          1, \
						  -1.1429805025399011, \
						  0.41280159809618877, \
						  0, \
						  0, \
						  0, \
						  0, \
						  0, \
						  0, \
              			}
#define LPFILTER_Fs10k_Fc2k { 0.0036216815149286421, \
                          2, \
                          1, \
						  -1.8226949251963083, \
						  0.83718165125602262, \
						  0, \
						  0, \
						  0, \
						  0, \
						  0, \
						  0, \
              			}


/*------------------------------------------------------------------------------
	LPFILTER Transformation Macro Definition
------------------------------------------------------------------------------*/
	

#define LPFILTER_MACRO(v)\
\
\
v.yn=-v.denominator1 * v.yn1-v.denominator2 * v.yn2 + v.gain * (v.xn + v.numerator1 * v.xn1 + v.numerator2 * v.xn2);\
v.xn2=v.xn1;\
v.xn1=v.xn;\
v.yn2=v.yn1;\
v.yn1=v.yn;

#define PRFILTER_MACRO(v)\
\
v.yn=-v.denominator1 * v.yn1-v.denominator2 * v.yn2 + v.gain * (v.xn + v.numerator1 * v.xn1 + v.numerator2 * v.xn2);\
v.xn2=v.xn1;\
v.xn1=v.xn;\
v.yn2=v.yn1;\
v.yn1=v.yn;

#define PLL_MACRO(v)\
\
v.yn=-v.denominator1 * v.yn1-v.denominator2 * v.yn2 + v.gain * (v.xn + v.numerator1 * v.xn1 + v.numerator2 * v.xn2);\
v.xn2=v.xn1;\
v.xn1=v.xn;\
v.yn2=v.yn1;\
v.yn1=v.yn;

#endif // __DIGITALFLITER_H__


