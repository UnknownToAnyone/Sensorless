//###########################################################################
// FILE:   adc_soc_epwm_cpu01.c
// TITLE:  ADC triggering via epwm for F2837xD.
//
//! \addtogroup cpu01_example_list
//! <h1> ADC ePWM Triggering (adc_soc_epwm)</h1>
//!
//! This example sets up the ePWM to periodically trigger the ADC.
//!
//! After the program runs, the memory will contain:\n
//! - \b AdcaResults \b: A sequence of analog-to-digital conversion samples from
//! pin A0. The time between samples is determined based on the period
//! of the ePWM timer.
//
//###########################################################################
// $TI Release: F2837xD Support Library v150 $
// $Release Date: Thu Mar  5 14:18:39 CST 2015 $
// $Copyright: Copyright (C) 2013-2015 Texas Instruments Incorporated -
//             http://www.ti.com/ ALL RIGHTS RESERVED $
//###########################################################################
#include "F28x_Project.h"     // Device Headerfile and Examples Include File
#include "F2837xD_input_xbar.h"
#include "math.h"
#include "Example_posspeed.h"   // Example specific Include file
#include "clarke.h"
#include "iclarke.h"
#include "park.h"
#include "ipark.h"
#include "pid_reg3.h"
#include "rampgen.h"
#include "rmp_cntl.h"
#include "stdio.h"
#include "svgen_dq.h"
#include "speed_fr.h"
#include "vector.h"
#include "digitalfilter.h"


void Cal_function(void);

Uint32 ADC_adj_num = 0;
Uint16 ADC_0V_adjust_start = 0;
float AdcResult_Ia_0A=0,AdcResult_Ib_0A=0,AdcResult_Ic_0A=0,AdcResult_Id_0A=0,AdcResult_Ie_0A=0,AdcResult_If_0A=0;
float AdcResult_Ia_even_0A = 0;
float AdcResult_Ib_even_0A = 0;
float AdcResult_Ic_even_0A = 0;
float AdcResult_Id_even_0A = 0;
float AdcResult_Ie_even_0A = 0;
float AdcResult_If_even_0A = 0;
Uint16 ADC_0V_adjust_done=0;
int show_PWM_low=0;
int counter_epwm6=0;
int counter_epwm10=0;
double onekHz=0;
int show_temp=0;
int show_compensation=0;
int show_Ic=0;
double sv_Q=0;
double iq_temp=0;
double id_temp=0;
int current_loop_control_mod=0;//电流闭环控制方式；1代表只加入xy电流环；2代表只加入电压补偿；3代表同时加入xy电流环和电压补偿；default代表原始电流环
double compensation_A=0;
double compensation_B=0;
double compensation_C=0;
double compensation_D=0;
double compensation_E=0;
double compensation_F=0;
int DAC_ratio=100;

#define   pi   3.14159265359
#define two_pi 6.2831853072
#define natual_constant 2.718281828459
// Maximum Dead Band values
#define EPWMx_MIN_DB   50
Uint16 VF_1_and_FOC_2_Ctrl_flag = 0;
Uint16 VF_CTRL = 1;
Uint16 FOC_CTRL = 2;
Uint16 Current_CTRL = 0;

// ePWM variables
Uint16 EPWM_period=2500; //10kHz

Uint16 CompareA_EPWM01_PhaseD_temp=0;
Uint16 CompareA_EPWM02_PhaseE_temp=0;
Uint16 CompareA_EPWM03_PhaseF_temp=0;
Uint16 CompareA_EPWM05_PhaseB_temp=0;
Uint16 CompareA_EPWM11_PhaseC_temp=0;
Uint16 CompareA_EPWM12_PhaseA_temp=0;
Uint16 CompareB_EPWM01_PhaseD_temp=0;
Uint16 CompareB_EPWM02_PhaseE_temp=0;
Uint16 CompareB_EPWM03_PhaseF_temp=0;
Uint16 CompareB_EPWM05_PhaseB_temp=0;
Uint16 CompareB_EPWM11_PhaseC_temp=0;
Uint16 CompareB_EPWM12_PhaseA_temp=0;

Uint16 ADC_CompA_EPWM6[4]={135,210,0,0};//trigger ADC
Uint16 ADC_CompB_EPWM6[4]={139,214,4,0};
Uint16 ADC_CompA_EPWM8[4]={160,235,0,0};//trigger ADC
Uint16 ADC_CompB_EPWM8[4]={164,239,4,0};
Uint16 ADC_CompA_EPWM6_temp[4]={135,210,0,0};//trigger ADC
Uint16 ADC_CompB_EPWM6_temp[4]={134,214,4,0}; //trigger PWM6_INT
Uint16 ADC_CompA_EPWM8_temp[4]={160,235,0,0};//trigger ADC
Uint16 ADC_CompB_EPWM8_temp[4]={169,239,4,0}; //trigger PWM8_INT

Uint16 ADC_EPWM6_index=0;
Uint16 ADC_EPWM8_index=0;

Uint16 EPWM_Eable_Flag=0;
Uint16 Decr_Speed_Flag=0;
Uint16 Decr_Speed_coun=0;
float  Speed_Vol_Decrease_ratio=1;

// To keep track of which way the Dead Band is moving
#define DB_UP   1
#define DB_DOWN 0

Uint16 Xint2Count_overCurrent=0;
//****************************************//
float  SpeedRefRef =0; //25Hz
float  SpeedRef=0;
Uint16 SpeedLoopCount=0;
//****************************************//

//*****************SVPWM*******************//
SVGENDQ svgen_dq1=SVGENDQ_DEFAULTS;
//****************************************//

//*************eQEP***********************//
POSSPEED qep_posspeed=POSSPEED_DEFAULTS;
//****************************************//

//**************PID**********************//
PIDREG3 pid1_spd = spd_PI__DEFAULTS;

PIDREG3 pid1_id = iqid_PI__DEFAULTS;
PIDREG3 pid1_iq = iqid_PI__DEFAULTS;

PIDREG3 pid1_ix = ixiy_PI__DEFAULTS;
PIDREG3 pid1_iy = ixiy_PI__DEFAULTS;   //对xy子空间电流进行闭环控制

Uint16 SpeedLoopPrescaler = 5;    //转速环预分频
//***************************************//

//***********V/F************************//
RMPCNTL rc1 = RMPCNTL_DEFAULTS;
RMPCNTL rc2 = RMPCNTL_DEFAULTS;
RAMPGEN rg1 = RAMPGEN_DEFAULTS;
//**************************************//

//********speed_from_theta***************//
SPEED_MEAS_QEP speed1 = SPEED_MEAS_QEP_DEFAULTS;
//**************************************/

//******Clark & Park, etc***************//
CLARKE clarke1 = CLARKE_DEFAULTS;
CLARKE DT_clarke = CLARKE_DEFAULTS;     //用于电压补偿变换
PARK park1 = PARK_DEFAULTS;
PARK DT_park = PARK_DEFAULTS;           //用于电压补偿变换
IPARK ipark1=   IPARK_DEFAULTS;
//**************************************//

//**********电压矢量，矩阵计算***************//
void matrix_inversion(float A[4][4]);   //求四阶逆矩阵,运行时间在30us以上，故不用
void matrix_multiply(float A[4][4],float B[4]);   //4x4矩阵 x 4x1矩阵 = 4x1矩阵
void vectorI_select(Uint16 Sector_temp);   //为电压矢量矩阵选择电压矢量
void vector_select(Uint16 Sector_old_temp);   //为电压矢量矩阵选择电压矢量
VECTOR AB[15]={ab55,ab45,ab44,ab64,ab66,ab26,ab22,ab32,ab33,ab13,ab11,ab51,ab55,ab45,ab44};
VECTOR XY[15]={xy55,xy45,xy44,xy64,xy66,xy26,xy22,xy32,xy33,xy13,xy11,xy51,xy55,xy45,xy44};
double vector_original_matrix[4][4];   //电压矢量矩阵 4x4
double vector_inversion_matrix[4][4];  //电压矢量逆矩阵4x4
double voltage_reference_matrix[4];    //参考电压矩阵4x4
double time_matrix[4];                 //时间矩阵4x1
//S1-S12为扇区1-12时的逆矩阵
double S1[4][4]={0.1339746,-0.5,-1.866025,0.5,  0.3660254,-0.3660254,1.366025,-1.366026,  0.3660254,0.3660254,1.366025,1.366025,  0.1339746,0.5,-1.866025,-0.5};
double S2[4][4]={0.3660254,-0.3660254,1.366025,-1.366025,  0.5,-0.1339746,-0.5,1.866026,  0.1339746,0.5,-1.866026,-0.5,  -0.1339746,0.5,1.866025,-0.5};
double S3[4][4]={0.5,-0.1339746,-0.5,1.866026,  0.5,0.1339746,-0.5,-1.866026,  -0.1339746,0.5,1.866026,-0.5,  -0.3660254,0.3660254,-1.366025,1.366025};
double S4[4][4]={0.5,0.1339746,-0.5,-1.866025,  0.3660254,0.3660254,1.366026,1.366026,  -0.3660254,0.3660254,-1.366026,1.366025,  -0.5,0.1339746,0.5,-1.866026};
double S5[4][4]={0.3660254,0.3660254,1.366026,1.366026,  0.1339746,0.5,-1.866026,-0.5,  -0.5,0.1339746,0.5,-1.866026,  -0.5,-0.1339746,0.5,1.866025};
double S6[4][4]={0.1339746,0.5,-1.866025,-0.5,  -0.1339746,0.5,1.866026,-0.5,  -0.5,-0.1339746,0.5,1.866025,  -0.3660254,-0.3660254,-1.366026,-1.366026};
double S7[4][4]={-0.1339746,0.5,1.866025,-0.5,  -0.3660254,0.3660254,-1.366025,1.366025,  -0.3660254,-0.3660254,-1.366025,-1.366025,  -0.1339746,-0.5,1.866025,0.5};
double S8[4][4]={-0.3660254,0.3660254,-1.366025,1.366026,  -0.5,0.1339746,0.5,-1.866026,  -0.1339746,-0.5,1.866026,0.5,  0.1339746,-0.5,-1.866025,0.5};
double S9[4][4]={-0.5,0.1339746,0.5,-1.866026,  -0.5,-0.1339746,0.5,1.866026,  0.1339746,-0.5,-1.866026,0.5,  0.3660254,-0.3660254,1.366025,-1.366026};
double S10[4][4]={-0.5,-0.1339746,0.5,1.866025,  -0.3660254,-0.3660254,-1.366026,-1.366026,  0.3660254,-0.3660254,1.366026,-1.366025,  0.5,-0.1339746,-0.5,1.866026};
double S11[4][4]={-0.3660254,-0.3660254,-1.366026,-1.366026,  -0.1339746,-0.5,1.866026,0.5,  0.5,-0.1339746,-0.5,1.866026,  0.5,0.1339746,-0.5,-1.866025};
double S12[4][4]={-0.1339746,-0.5,1.866025,0.5,  0.1339746,-0.5,-1.866026,0.5,  0.5,0.1339746,-0.5,-1.866025,  0.3660254,0.3660254,1.366026,1.366026};
//**************************************//

//**************无速度传感器计算滤波*******************//
LPFILTER Ia_sample_lp=LPFILTER_Fs10k_Fc1k;
LPFILTER Ib_sample_lp=LPFILTER_Fs10k_Fc1k;
LPFILTER Ic_sample_lp=LPFILTER_Fs10k_Fc1k;
LPFILTER Id_sample_lp=LPFILTER_Fs10k_Fc1k;
LPFILTER Ie_sample_lp=LPFILTER_Fs10k_Fc1k;
LPFILTER If_sample_lp=LPFILTER_Fs10k_Fc1k;
double pIalpha0;
double pIbeta0;
double pIalpha1;
double pIbeta1;
double pIalpha2;
double pIbeta2;


double Cos2Theta = 0, Sin2Theta = 0, Tan2Theta=0;

double Sin2Theta_sample;
double Cos2Theta_sample;
double Tan2Theta_sample;

LPFILTER Sin_lp=LPFILTER_Fs10k_Fc20;
LPFILTER Cos_lp=LPFILTER_Fs10k_Fc20;

double Sin2Theta_sample_temp1;
double Sin2Theta_sample_temp2;
double Cos2Theta_sample_temp1;
double Cos2Theta_sample_temp2;

PLL Theta_PLL=PLL_DEFAULTS;
PLL PLL1=PLL_I;
PLL PLL2=PLL_I;


//**************************************//

//**************电流滤波*******************//
LPFILTER Id1_lp=LPFILTER_Fs10k_Fc20;
LPFILTER Iq1_lp=LPFILTER_Fs10k_Fc20;
LPFILTER Id2_lp=LPFILTER_Fs10k_Fc20;
LPFILTER Iq2_lp=LPFILTER_Fs10k_Fc20;

LPFILTER DT_Alpha_lp=LPFILTER_Fs10k_Fc1k;
LPFILTER DT_Beta_lp=LPFILTER_Fs10k_Fc1k;
LPFILTER DT_X_lp =LPFILTER_Fs10k_Fc1k;
LPFILTER DT_Y_lp=LPFILTER_Fs10k_Fc1k;
//**************************************//

//***************xy坐标系下PR*****************//
LPFILTER Ix_lp=LPFILTER_Fs10k_Fc1k;
LPFILTER Iy_lp=LPFILTER_Fs10k_Fc1k;
PRFILTER Ix_PR=PR_FILTER_100rpm;
PRFILTER Iy_PR=PR_FILTER_100rpm;
PARK PR_park=PARK_DEFAULTS;
IPARK PR_ipark=IPARK_DEFAULTS;
PRFILTER setPR(PRFILTER pr,float speed,float wc,float kr);
float wc=2;
float kr=50;
float PR_limitation=5;   //park_Ds,park_Qs 上下限
float SpeedRefRef_temp;
float wc_temp=2;
float kr_temp=50;
//*******************************************//


double T =0.0001;            //IGBT开关周期
double Td=0.000002;         //死区时间3us
//***************三相3s至2r变换，用于双DQ轴变换******************//
double Id1_temp=0;
double Iq1_temp=0;
double Id2_temp=0;
double Iq2_temp=0;//滤波之前

double Id1_filter=0;
double Iq1_filter=0;
double Id2_filter=0;
double Iq2_filter=0;//滤波之后

double Ia_filter=0;
double Ic_filter=0;
double Ie_filter=0;
double Ib_filter=0;
double Id_filter=0;
double If_filter=0;
void double_park(double A,double B,double C,double D,double E,double F,double phase);
void double_anti_park(double Da,double Qa,double Db,double Qb,double phase);
//******************************************************//
void DC_Relay_And_Led_Blink_Ctrl(void);
void InitEPWMGpio_SixPhase(void);
void InitGpio_ExternalInterrupt(void);
void InitEPWM_SOC_ADC();
void SetupADCoftware_SixPhase(void);
void StartADC_One_Conversion_SixPhase(void);
void ConfigureADC_SixPhase(void);
void SVGEN_SIX_PHASE(void);
void DAC_init();
void Enable_Force_PWM_allout_low(void);
void Disable_Force_PWM_allout_Low(void);
int show_current_by_DAC(double current);
void Epwm_CMPAB_assignment(void);
int limit_CMPB(int CMPB);
double pIphase_filter(double pIphase,double pIphase_temp);
//void PID_init(void);
void Deadtime_Compensation(float Ia,float Ib,float Ic,float Id,float Ie,float If,float Td);


__interrupt void epwm6_isr(void);
__interrupt void epwm9_isr(void);
__interrupt void epwm10_isr(void);

__interrupt void adca1_isr(void);
__interrupt void adcb1_isr(void);
__interrupt void adcc1_isr(void);
__interrupt void adcc2_isr(void);
__interrupt void adcd1_isr(void);


//--------------------------------------------------------------------------------------------------//
//----1.For calculating t1,t2,t3,t4,t5 of each Sector,We need 12 Inverse Matrix(5_by_5)-------------//
//----2.Yet of which only the two coloms are useful-------------------------------------------------//
//----3.Using the 1st two coloms(2X5) of the 12 inverse 5X5 matrices, we form the following array---//
//----4.More details, see "SVPWM Control of Dual 3-phase IM Using Vector Space Decomposition"-------//

//----------------------------------------------------------------------------------------------//
float V_alpha_beta[48] ={0.7887,-0.7887,1.0774,-0.2887,\
		 1.0774,-0.2887, 1.0774, 0.2887,\
		 1.0774, 0.2887, 0.7887, 0.7887,\
		 0.7887, 0.7887, 0.2887, 1.0774,\
		 0.2887, 1.0774,-0.2887, 1.0774,\
		-0.2887, 1.0774,-0.7887, 0.7887,\
		-0.7887, 0.7887,-1.0774, 0.2887,\
		-1.0774, 0.2887,-1.0774,-0.2887,\
		-1.0774,-0.2887,-0.7887,-0.7887,\
		-0.7887,-0.7887,-0.2887,-1.0774,\
		-0.2887,-1.0774, 0.2887,-1.0774,\
		 0.2887,-1.0774, 0.7887,-0.7887};

//variables to store conversion results
Uint16 AdcaResult_Ia[6]={0,0,0,0,0,0}; // sample 0, 1, 3
Uint16 AdcaResult_Ib[6]={0,0,0,0,0,0}; // sample 0, 1, 3
Uint16 AdcbResult_Ic[6]={0,0,0,0,0,0}; // sample 0, 1, 3
Uint16 AdcbResult_Id[6]={0,0,0,0,0,0}; // sample 0, 1, 3
Uint16 AdccResult_Ie[6]={0,0,0,0,0,0}; // sample 0, 1, 3
Uint16 AdccResult_If[6]={0,0,0,0,0,0}; // sample 0, 1, 3

float32 I_reference_value=20;

float Ia[6]={0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float Ib[6]={0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float Ic[6]={0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float Id[6]={0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float Ie[6]={0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float If[6]={0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

Uint16 AdcaResult2_Id_even[4]={0,0,0,0}; // sample 0, 2, 4
Uint16 AdcaResult3_Ie_even[4]={0,0,0,0}; // sample 0, 2, 4
Uint16 AdcaResult4_If_even[4]={0,0,0,0}; // sample 0, 2, 4
Uint16 AdcaResult5_Ia_even[4]={0,0,0,0}; // sample 0, 2, 4
Uint16 AdcdResult3_Ic_even[4]={0,0,0,0}; // sample 0, 2, 4
Uint16 AdcdResult4_Ib_even[4]={0,0,0,0}; // sample 0, 2, 4

double Ia_even[4]={0.0,0.0,0.0,0.0};
double Ib_even[4]={0.0,0.0,0.0,0.0};
double Ic_even[4]={0.0,0.0,0.0,0.0};
double Id_even[4]={0.0,0.0,0.0,0.0};
double Ie_even[4]={0.0,0.0,0.0,0.0};
double If_even[4]={0.0,0.0,0.0,0.0};

Uint16 AdcdResult2_VdcLink=0;
float VdcLink=0.0;
float VdcLink_temp=10;

char Failure_phase[1];
long pos_current_max=16;  //current_upper_limitation

float SpeedRpm_Lprprprpr = 0;
//float SpeedRpm_Hfrfrfrfr = 0;
float Speed_from_theta = 0;
float tempSpeed_elec=1;
float QEP_temp=0;
Uint16 Sector = 0;
Uint16 Sector_old = 0;
Uint16 Sector_temp = 0;
Uint16 PWM9_10KHz_flag = 0;



//++++++++++SVG+++++++++++++++++
Uint16 T_zero_flag = 0;
double t1=0,t2=0,t3=0,t4=0,t5=0,temp_sv1=0,temp_sv2=0;
double t_sum=0;

double tx1=0,tx2=0,tx3=0,tx4=0,tx5=0,tx6=0,tx7=0,tx8=0;

float32 Tmin=0.1;        // Vector min周期, 100_PWM_Clock
float32 Tmargin = 0.03;
float32 Tshift = -0.01;
float32 Delta_t = 0.04;   // 1 = 2500_PWM_Clock = 100us, 0.01 = 25 PWM_Clock

Uint16 t_1_not_enough = 0, t_2_not_enough=0;
Uint16 Add_negative_vector_flag = 0;
//------------------------------


void SensorLess_RotorPosition(void);


//###################################################################//
//###############_CPU01<-->CPU02_Communication_######################//
#define Connected_to_CPU1 0
#define Connected_to_CPU2 1

#define COMMAND_synch_PWMs 1
#define COMMAND_stop_PWMs 2
#define COMMAND_start_PWMs 3
#define COMMAND_update_PWMs_compAB 4

#define COMMAND_start_PWMs_done 5
#define COMMAND_stop_PWMs_done 6

Uint16 	CUP1_EXINT_NUM=0;

Uint16 SynchronisePWMs_DONE_flag = 0;
Uint16 SynchronisePWMs_start_flag = 0;
Uint16 NVC_start_flag=0;
Uint16 t1_negtive_vector_flag=0;
Uint16 t2_negtive_vector_flag=0;
double t1_compensation=0;
double t2_compensation=0;

Uint32 CPU1TOCPU2RAM_addr = 0x00003FC00;
Uint32 CPU2TOCPU1RAM_addr = 0x00003F800;
Uint16 *CPU1TOCPU2RAM_addr_pt =(Uint16 *) 0x00003FC00;
Uint16 *CPU2TOCPU1RAM_addr_pt =(Uint16 *) 0x00003F800;

void CPU01_Send_Command_To_CPU02(Uint16 CPUcomd);
void Transfer_EPWMs_compAB_To_CPU01TOCPU02RAM(void);
__interrupt void xint1_isr_synchronous_PWM(void);
__interrupt void xint2_isr_overCurrent_INT(void);
__interrupt void CPU02toCPU01IPC0IntHandler(void);
//###################################################################//

void main(void)
{
      	InitSysCtrl(); // Step 1. Initialize System Control:
    	InitGpio();    // Step 2. Initialize GPIO:

    	//DC_Relay_And_Led_Blink_Ctrl();   //  DC_Realy control and Led_blink control
    	//-------For Led-Blink------------------------------
    	//GPIO_SetupPinMux(6, GPIO_MUX_CPU1, 0);
    	//GPIO_SetupPinOptions(6, GPIO_OUTPUT, GPIO_PUSHPULL);
    	//GPIO_WritePin(6, 0);
    	//GPIO_SetupPinMux(7, GPIO_MUX_CPU1, 0);
    	//GPIO_SetupPinOptions(7, GPIO_OUTPUT, GPIO_PUSHPULL);
    	//GPIO_WritePin(7, 0);
    	GPIO_SetupPinMux(11, GPIO_MUX_CPU1, 0);
    	GPIO_SetupPinOptions(11, GPIO_OUTPUT, GPIO_PUSHPULL);
    	GPIO_WritePin(11, 0);
    	//GPIO_SetupPinMux(69, GPIO_MUX_CPU1, 0);
    	//GPIO_SetupPinOptions(69, GPIO_OUTPUT, GPIO_PUSHPULL);
    	//GPIO_WritePin(69, 0);
        //GPIO_SetupPinMux(70, GPIO_MUX_CPU1, 0);
        //GPIO_SetupPinOptions(70, GPIO_OUTPUT, GPIO_PUSHPULL);
        //GPIO_WritePin(70, 1);

    DINT; // Step 3. Clear all interrupts and initialize PIE vector table: Disable CPU interrupts

		InitPieCtrl(); // Initialize the PIE control registers to their default state.
		IER = 0x0000;  // Disable CPU interrupts and clear all CPU interrupt flags:
		IFR = 0x0000;
		InitPieVectTable();  // Initialize the PIE vector table with pointers to the shell Interrupt. Service Routines (ISR).

		ConfigureADC_SixPhase();     //Configure the ADCs and power them up
		SetupADCoftware_SixPhase();  //Setup the ADCs for software conversions
		DAC_init();
		InitEQep2Gpio();

    EALLOW;
    	CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;  //stop ePWMs' clock to reset state
    EDIS;

		InitEPWMGpio_SixPhase(); //  Initialize PWM_GPIO
	EALLOW;
		InitEPWM_SOC_ADC();      //  Initialize_PWM

	    DevCfgRegs.CPUSEL0.bit.EPWM1  = Connected_to_CPU2;
	    DevCfgRegs.CPUSEL0.bit.EPWM2  = Connected_to_CPU2;
	    DevCfgRegs.CPUSEL0.bit.EPWM3  = Connected_to_CPU2;
	    DevCfgRegs.CPUSEL0.bit.EPWM4  = Connected_to_CPU2;
	    DevCfgRegs.CPUSEL0.bit.EPWM5  = Connected_to_CPU2;
	    DevCfgRegs.CPUSEL0.bit.EPWM11 = Connected_to_CPU2;
	    DevCfgRegs.CPUSEL0.bit.EPWM12 = Connected_to_CPU2;

	    DevCfgRegs.CPUSEL0.bit.EPWM6  = Connected_to_CPU1;
	    DevCfgRegs.CPUSEL0.bit.EPWM7  = Connected_to_CPU1;
	    DevCfgRegs.CPUSEL0.bit.EPWM8  = Connected_to_CPU1;
	    DevCfgRegs.CPUSEL0.bit.EPWM9  = Connected_to_CPU1;
	    DevCfgRegs.CPUSEL0.bit.EPWM10 = Connected_to_CPU1;
	    DevCfgRegs.CPUSEL11.bit.ADC_A = Connected_to_CPU1;
	    DevCfgRegs.CPUSEL11.bit.ADC_B = Connected_to_CPU1;
	    DevCfgRegs.CPUSEL11.bit.ADC_C = Connected_to_CPU1;
	    DevCfgRegs.CPUSEL11.bit.ADC_D = Connected_to_CPU1;
	EDIS;

    EALLOW;
    	PieVectTable.XINT1_INT  = &xint1_isr_synchronous_PWM; //External_Interrupt_for_EpwmSynch,
    	PieVectTable.XINT2_INT  = &xint2_isr_overCurrent_INT; //External_Interrupt_for_overCurrent,

    	PieVectTable.IPC0_INT   = &CPU02toCPU01IPC0IntHandler;   //

    	PieVectTable.EPWM6_INT  = &epwm6_isr;  //function for ePWM interrupt 6
		PieVectTable.EPWM9_INT  = &epwm9_isr;  //function for ePWM interrupt 9
//		PieVectTable.EPWM10_INT  = &epwm10_isr;  //function for ePWM interrupt 9

		PieVectTable.ADCA1_INT  = &adca1_isr;  //function for ADCA interrupt 1
		PieVectTable.ADCB1_INT  = &adcb1_isr;  //function for ADCB interrupt 1
		PieVectTable.ADCC1_INT  = &adcc1_isr;  //function for ADCC interrupt 1
		PieVectTable.ADCC2_INT  = &adcc2_isr;  //function for ADCD interrupt 1
		PieVectTable.ADCD1_INT  = &adcd1_isr;  //function for ADCD interrupt 1
    EDIS;

    	IER |= M_INT1;    // Enable group 1 for ADCAINT1,ADCBINT1,ADCCINT1,ADCDINT1,XINT1,XINT2
		IER |= M_INT3;    // Enable group 3 interrupts for ePWM
		IER |= M_INT10;    // Enable group 3 interrupts for ADCDINT2
		PieCtrlRegs.PIEIER3.bit.INTx6 = 1;     // Enable PIE interrupt_ePWM6
		PieCtrlRegs.PIEIER3.bit.INTx9 = 1;     // Enable PIE interrupt_ePWM9
//		PieCtrlRegs.PIEIER3.bit.INTx10 = 1;    // Enable PIE interrupt_ePWM10

		PieCtrlRegs.PIEIER1.bit.INTx1 = 1;     // Enable PIE Group 1 INT1, interrupt_ ADCA1
		PieCtrlRegs.PIEIER1.bit.INTx2 = 1;     // Enable PIE Group 1 INT2, interrupt_ ADCB1
		PieCtrlRegs.PIEIER1.bit.INTx3 = 1;     // Enable PIE Group 1 INT3, interrupt_ ADCC1
		PieCtrlRegs.PIEIER1.bit.INTx6 = 1;     // Enable PIE Group 1 INT6, interrupt_ ADCD1
		PieCtrlRegs.PIEIER10.bit.INTx14 = 1;     // Enable PIE Group 10 INT14, interrupt_ ADCD2

		PieCtrlRegs.PIEIER1.bit.INTx13 = 1;    // CPU2 to CPU1 INT0

		PieCtrlRegs.PIEIER1.bit.INTx4 = 1;     // Enable PIE Group 1 INT4, interrupt_ XINT1
		PieCtrlRegs.PIEIER1.bit.INTx5 = 1;     // Enable PIE Group 1 INT5, interrupt_ XINT2
		PieCtrlRegs.PIECTRL.bit.ENPIE = 1;     // Enable the PIE block,

		//Tips:
		//1. Connect GPIO69 and GPIO70 to synchronise PWMs of CPU1 and CPU2
		//2. Run cpu1 1st, then CPU2 , and then write 1 and 0 alternatively to GPIO66, for several times
		//3. These pulses will trigger XINT_in CPU1 and CPU2.
		//4. In conrrsponding interrupt functions, we will synchronise PWMs of CPU1 and CPU2
		GPIO_SetupPinMux(69,GPIO_MUX_CPU1,0);
		GPIO_SetupPinOptions(69, GPIO_OUTPUT,1);

		InitGpio_ExternalInterrupt();      // Configure XINT_GPIO
		GPIO_SetupXINT1Gpio(70);           // GPIO70 is XINT1_Synch_PWMs_of CPU1 and CPU2
		GPIO_SetupXINT2Gpio(99);           // GPIO99 is XINT2_OverCurrent_protection_for CPU1 and CPU2
		XintRegs.XINT1CR.bit.POLARITY = 1; // Rising edge interrupt
		XintRegs.XINT1CR.bit.ENABLE = 1;   // Enable XINT1
//		XintRegs.XINT2CR.bit.POLARITY = 1; // Rising edge interrupt
//		XintRegs.XINT2CR.bit.ENABLE = 1;   // Enable XINT2

    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM


    	qep_posspeed.init(&qep_posspeed);


    do{
    	if(SynchronisePWMs_DONE_flag==0) //1st, Synchronise PWMs of CPU1 and CPU2 15 times.
    	{
    		while(SynchronisePWMs_DONE_flag!=15)
    		{
    			if(SynchronisePWMs_start_flag==1)
    			{
    				++SynchronisePWMs_DONE_flag;
    				GPIO_WritePin(69,0);
    				GPIO_WritePin(69,1);
    				GPIO_WritePin(69,1);
    				DELAY_US(1000);
    			}
    		}
    		SynchronisePWMs_start_flag = 0;
    	}

    }while(1);
}

float Rotor_Position = 0;
float delta_Ialpha0 = 0, delta_Ialpha1 = 0, delta_Ialpha2 = 0, delta_Ibeta0 = 0, delta_Ibeta1 = 0, delta_Ibeta2 = 0;
float delta_t0=0.000006, delta_t1=0.000006, delta_t2=0.000006;


float I_temp1=0.0,I_temp2=0.0,I_temp3=0.0,I_temp4=0.0;
double Valpha1=0,Valpha2=0,Vbeta1=0,Vbeta2=0;
//float Valpha1_test=0.0,Valpha2_test=0.0,Vbeta1_test=0.0,Vbeta2_test=0.0;

float dIa_over_dt_test1=0;
float dIa_over_dt_test2=0;
float reciprocal_delta_t = 1;

Uint16 Show_I_SP = 11;
Uint16 speed_ratio =20;
Uint16 I_ratio = 100;
Uint16 Kpki_ratio = 1000;
Uint16 Rotor_P_ratio = 10000;
float SinCos_ration = 5000;
Uint16 Minus_dI0_over_dt_flag = 0;

float my_position=0;
Uint16 my_ps_ration=100;

Uint16 Start_restore_flag = 0;
Uint16 Start_restore_done_flag = 0;
Uint16 sensorless_filter_flag=0;
double tempa;
double pIa0,pIb0,pIc0,pId0,pIe0,pIf0;
double pIa1,pIb1,pIc1,pId1,pIe1,pIf1;
double pIa1_temp,pIb1_temp,pIc1_temp,pId1_temp,pIe1_temp,pIf1_temp;
double pIa2,pIb2,pIc2,pId2,pIe2,pIf2;
double pIa2_temp,pIb2_temp,pIc2_temp,pId2_temp,pIe2_temp,pIf2_temp;
Uint16 Ipahse_filter_flag=0;
double Theta;
double double_Theta;
double speed_from_PLL=0;;
double sin_from_PLL=0;
double cos_from_PLL=0;
double PLL_error=0;
int PLL_flag=0;
double Theta_error;
double temp1,temp2,temp3,temp4,temp5,temp6;
int doubleTheta_adjustment_flag=0;
double double_Theta_temp=0;
double double_tempSpeed_elec=0;
double cos_norm;
double sin_norm;
double filter_delay=0;
void SensorLess_RotorPosition(void)
{

//
//	pIa0=(Ia[1]-Ia[0]);
//	pIc0=(Ic[1]-Ic[0]);
//	pIe0=(Ie[1]-Ie[0]);
//
//	pIb0=(Ib[1]-Ib[0]);
//	pId0=(Id[1]-Id[0]);
//	pIf0=(If[1]-If[0]);
//
//	clarke1.As=pIa0;
//	clarke1.Bs=pIb0;
//	clarke1.Cs=pIc0;
//	clarke1.Ds=pId0;
//	clarke1.Es=pIe0;
//	clarke1.Fs=pIf0;
//
//	CLARKE_MACRO(clarke1);
//	pIalpha0 = clarke1.Alpha ; // 1/dt = 1/3us =1000000/3 = 333333.333
//	pIbeta0 = clarke1.Beta ;  // 1/dt = 1/3us =1000000/3 = 333333.333


//*******************************************************************************//
	pIa1=(Ia[3]-Ia[2]);
	pIc1=(Ic[3]-Ic[2]);
	pIe1=(Ie[3]-Ie[2]);

	pIb1=(Ib[3]-Ib[2]);
	pId1=(Id[3]-Id[2]);
	pIf1=(If[3]-If[2]);

	clarke1.As=pIa1;
	clarke1.Bs=pIb1;
	clarke1.Cs=pIc1;
	clarke1.Ds=pId1;
	clarke1.Es=pIe1;
	clarke1.Fs=pIf1;

	CLARKE_MACRO(clarke1);

	pIalpha1 = clarke1.Alpha ; // 1/dt = 1/3us =1000000/3 = 333333.333
	pIbeta1 = clarke1.Beta ;  // 1/dt = 1/3us =1000000/3 = 333333.333




//*******************************************************************************//

	pIa2=(Ia[5]-Ia[4]);
	pIc2=(Ic[5]-Ic[4]);
	pIe2=(Ie[5]-Ie[4]);

	pIb2=(Ib[5]-Ib[4]);
	pId2=(Id[5]-Id[4]);
	pIf2=(If[5]-If[4]);

	clarke1.As=pIa2;
	clarke1.Bs=pIb2;
	clarke1.Cs=pIc2;
	clarke1.Ds=pId2;
	clarke1.Es=pIe2;
	clarke1.Fs=pIf2;

	CLARKE_MACRO(clarke1);
	pIalpha2 = clarke1.Alpha ; // 1/dt = 1/1us =1000000/1 = 333333.333
	pIbeta2 = clarke1.Beta ;  // 1/dt = 1/1us =1000000/1 = 333333.333

//*******************************************************************************//

	vector_select(Sector_old);
	if((Sector_old==1)||(Sector_old==2)||(Sector_old==5)||(Sector_old==6)||(Sector_old==9)||(Sector_old==10))
	{   //In these Sectors, order: vector2 1st, then vector1 second
		Valpha1 = vector_original_matrix[0][1];
		Vbeta1  = vector_original_matrix[1][1];
		Valpha2 = vector_original_matrix[0][0];
		Vbeta2  = vector_original_matrix[1][0];
	}
	else   //In these Sectors, order: vector1 1st, then vector2 second
	{
		Valpha1 = vector_original_matrix[0][0];
		Vbeta1  = vector_original_matrix[1][0];
		Valpha2 = vector_original_matrix[0][1];
		Vbeta2  = vector_original_matrix[1][1];
	}

//	temp1 = (Valpha2*Valpha2) - (Vbeta2*Vbeta2);
//	temp2 = (Vbeta2*(pIalpha2_sample - pIalpha0_sample)) - (Valpha2*(pIbeta2_sample - pIbeta0_sample));
//	temp3 = (Valpha1*Valpha1) - (Vbeta1*Vbeta1);
//	temp4 = (Vbeta1*(pIalpha1_sample - pIalpha0_sample)) - (Valpha1*(pIbeta1_sample - pIbeta0_sample));
//	temp5 = 2*Valpha1*Vbeta1;
//	temp6 = 2*Valpha2*Vbeta2;

//	temp1 = (Valpha2*Valpha2) - (Vbeta2*Vbeta2);
//	temp2 = (Vbeta2*(delta_Ialpha2/delta_t2 - delta_Ialpha0/delta_t0)) - (Valpha2*(delta_Ibeta2/delta_t2 - delta_Ibeta0/delta_t0));
//	temp3 = (Valpha1*Valpha1) - (Vbeta1*Vbeta1);
//	temp4 = (Vbeta1*(delta_Ialpha1/delta_t1 - delta_Ialpha0/delta_t0)) - (Valpha1*(delta_Ibeta1/delta_t1 - delta_Ibeta0/delta_t0));
//	temp5 = 2*Valpha1*Vbeta1;
//	temp6 = 2*Valpha2*Vbeta2;

//	temp1 = (Valpha2*Valpha2) - (Vbeta2*Vbeta2);
//	temp2 = (Vbeta2*pIalpha2_sample) - (Valpha2*pIbeta2_sample);
//	temp3 = (Valpha1*Valpha1) - (Vbeta1*Vbeta1);
//	temp4 = (Vbeta1*pIalpha1_sample) - (Valpha1*pIbeta1_sample);
//	temp5 = 2*Valpha1*Vbeta1;
//	temp6 = 2*Valpha2*Vbeta2;

//	temp1 = (Valpha2*Valpha2) - (Vbeta2*Vbeta2);
//	temp2 = (Vbeta2*delta_Ialpha2/delta_t2) - (Valpha2*delta_Ibeta2/delta_t2);
//	temp3 = (Valpha1*Valpha1) - (Vbeta1*Vbeta1);
//	temp4 = (Vbeta1*delta_Ialpha1/delta_t1) - (Valpha1*delta_Ibeta1/delta_t1);
//	temp5 = 2*Valpha1*Vbeta1;
//	temp6 = 2*Valpha2*Vbeta2;

	temp1 = (Valpha2*Valpha2) - (Vbeta2*Vbeta2);
	temp2 = (Vbeta2*pIalpha2) - (Valpha2*pIbeta2);
	temp3 = (Valpha1*Valpha1) - (Vbeta1*Vbeta1);
	temp4 = (Vbeta1*pIalpha1) - (Valpha1*pIbeta1);
	temp5 = 2*Valpha1*Vbeta1;
	temp6 = 2*Valpha2*Vbeta2;

//	temp1 = (Valpha2*Valpha2) - (Vbeta2*Vbeta2);
//	temp2 = (Vbeta2*(pIalpha2-pIalpha0)) - (Valpha2*(pIbeta2-pIbeta0));
//	temp3 = (Valpha1*Valpha1) - (Vbeta1*Vbeta1);
//	temp4 = (Vbeta1*(pIalpha1-pIalpha0)) - (Valpha1*(pIbeta1-pIbeta0));
//	temp5 = 2*Valpha1*Vbeta1;
//	temp6 = 2*Valpha2*Vbeta2;

//	temp1 = (Valpha2_sample*Valpha2_sample) - (Vbeta2_sample*Vbeta2_sample);
//	temp2 = (Vbeta2_sample*(pIalpha2-pIalpha0)) - (Valpha2_sample*(pIbeta2-pIbeta0));
//	temp3 = (Valpha1_sample*Valpha1_sample) - (Vbeta1_sample*Vbeta1_sample);
//	temp4 = (Vbeta1_sample*(pIalpha1-pIalpha0)) - (Valpha1_sample*(pIbeta1-pIbeta0));
//	temp5 = 2*Valpha1_sample*Vbeta1_sample;
//	temp6 = 2*Valpha2_sample*Vbeta2_sample;

	Cos2Theta = (temp1*temp4-temp2*temp3)/(temp3*temp6-temp1*temp5);
	Sin2Theta = (temp6*temp4-temp2*temp5)/(temp3*temp6-temp1*temp5);
//	Tan2Theta=(temp6*temp4-temp2*temp5)/(temp1*temp4-temp2*temp3);

	double_tempSpeed_elec=2*tempSpeed_elec;
	while (double_tempSpeed_elec>1) {double_tempSpeed_elec=double_tempSpeed_elec-1;}
	while (double_tempSpeed_elec<0) {double_tempSpeed_elec=double_tempSpeed_elec+1;}

	if (sensorless_filter_flag==1)
	{
	    Cos_lp.xn=Cos2Theta;
	    LPFILTER_MACRO(Cos_lp);
	    Cos2Theta_sample=Cos_lp.yn;

	    Sin_lp.xn=Sin2Theta;
	    LPFILTER_MACRO(Sin_lp);
	    Sin2Theta_sample=Sin_lp.yn;

	}

	if(PLL_flag==0)
	{
		cos_from_PLL=Cos2Theta_sample;
		sin_from_PLL=Sin2Theta_sample;
	}

//	cos_norm=-1+2/(1+exp(-3000*Cos2Theta_sample));
//	sin_norm=-1+2/(1+exp(-3000*Sin2Theta_sample));

	PLL_error=500*(Sin2Theta_sample*cos_from_PLL - Cos2Theta_sample*sin_from_PLL);
//	PLL_error=sin_norm*cos_from_PLL - cos_norm*sin_from_PLL;
	PLL_error=40*PLL_error;

    if(PLL_flag==1)
    {
        PLL1.xn=PLL_error;
        PLL_MACRO(PLL1);
        speed_from_PLL=PLL1.yn;

        PLL2.xn=speed_from_PLL;
        PLL_MACRO(PLL2);
    }

    sin_from_PLL=sin(PLL2.yn);
    cos_from_PLL=cos(PLL2.yn);

    if(Speed_from_theta<=10)
    {
    	filter_delay=0.070885*(Speed_from_theta/6);
    }
    else if(Speed_from_theta>10 && Speed_from_theta<=20)
    {
    	filter_delay=0.071853*(Speed_from_theta/6)-0.0016137;
    }
    else if(Speed_from_theta>20 && Speed_from_theta<=30)
    {
    	filter_delay=0.073663*(Speed_from_theta/6)-0.008247;
    }
    else
    {
    	filter_delay=0.360668;
    }
    double_Theta=PLL2.yn-double_Theta_temp+filter_delay;
    Theta=0.5*double_Theta;

    if(doubleTheta_adjustment_flag==1)
    {
    	double_Theta_temp=PLL2.yn-double_tempSpeed_elec*two_pi;
    	doubleTheta_adjustment_flag=0;
    }

//    double_Theta=(double)((int)(double_Theta*10000)%62832)/10000.0;
//    Theta=(double)((int)(Theta*10000)%62832)/10000.0;

    while (double_Theta>two_pi){double_Theta=double_Theta-two_pi;}
    while (double_Theta<0)     {double_Theta=double_Theta+two_pi;}

    while (Theta>two_pi)       {Theta=Theta-two_pi;}
    while (Theta<0)            {Theta=Theta+two_pi;}

    Theta_error=Theta-tempSpeed_elec*two_pi;
    if(Theta_error>pi)  {Theta_error = Theta_error-two_pi;}
    if(Theta_error<-pi) {Theta_error = Theta_error+two_pi;}

}


float DI_ratio=1;
float I1=0;
float I2=0;
float DI_dt=0;
Uint16 Delta_N=1,num_x=0;
Uint16 MAX=0;
float tempSpeed_mech=0;
double speed_temp,speed_error;
int ssss=0;
int counter=0;
int qep_temp=0;
int qep_i=0;
float My_idq_Kp=3;
float My_idq_Ki=1;
float My_ixy_Kp=1;
float My_ixy_Ki=1;
float My_sp_kp=0.01;
float My_sp_ki=0;
void Cal_function(void)
{
	speed_temp=SpeedRpm_Lprprprpr;
	//*************************Speed_&_Angle************************************//
	qep_posspeed.calc(&qep_posspeed);  // use the QEP call back function to get current elec_theta and speed

	tempSpeed_mech=(_IQ15toF(qep_posspeed.theta_mech));   // current elec_theta
	tempSpeed_elec=(_IQ15toF(qep_posspeed.theta_elec));   // current elec_theta
	QEP_temp=tempSpeed_elec*360;
	speed_error=fabs(speed_temp-qep_posspeed.SpeedRpm_pr);
    if(speed_error>18){SpeedRpm_Lprprprpr=speed_temp;}
    else {SpeedRpm_Lprprprpr=qep_posspeed.SpeedRpm_pr;}// For low speed region (approximately, 0~6000 rpm)
	//SpeedRpm_Hfrfrfrfr=qep_posspeed.SpeedRpm_fr;// For high speed region (approximately, >6000 rpm)

	speed1.ElecTheta = _IQ(tempSpeed_elec);     // Due to EMI, when DClink voltage>60V, our DSP's QEP speed part cann't work well,
	SPEED_FR_MACRO(speed1);                     // namely, "qep_posspeed.SpeedRpm_pr" may be wrong occasionally. Therefore, use the
	Speed_from_theta = speed1.SpeedRpm;         // resultant "qep_posspeed.theta_elec" to derive conrresponding speed.

	//*************************************************************************//

//	if(MyThetaAdd>=0 && MyThetaAdd<=360)  tempSpeed_elec = tempSpeed_elec + MyThetaAdd*0.0027778;
//
//	if(tempSpeed_elec<0) tempSpeed_elec = 1- tempSpeed_elec;
//	if(tempSpeed_elec>1) tempSpeed_elec = tempSpeed_elec - 1;



	//**************Current_sample & overcurrnet_protection*******************//


	StartADC_One_Conversion_SixPhase();  // ADC, sample the currents of each phase


	//********************************************************************//

	//**********switching_flag_for_V/F_ctrl or FOC_ctrl*******************//
	//Current_CTRL = VF_CTRL;    //**run_v/f_control_mode_flag
	//Current_CTRL = FOC_CTRL;     //**run_foc_control_mode_flag

		if(EPWM_Eable_Flag==1)  //using this flag to run or stop the motor online。EPWM_Eable_Flag==1 ; PWMs on, PWM_Eable_Flag==0, PWMs off
		{


			Failure_phase[0]='N'; //clear_mark_for_overphase
//			if(SpeedRpm_Lprprprpr > SpeedRefRef + 100)//已经飞车
//			{
//				Decr_Speed_Flag = 1;
//				EPWM_Eable_Flag = 0;
//			}
			SpeedRef=SpeedRefRef*0.002;//1/500;//标幺值
			rc1.TargetValue = SpeedRef;
		   	//*****************Using_10A_as_basicCurrent***********************************//
			Ia_sample_lp.xn=Ia[0];
			Ib_sample_lp.xn=Ib[0];
			Ic_sample_lp.xn=Ic[0];
			Id_sample_lp.xn=Id[0];
			Ie_sample_lp.xn=Ie[0];
			If_sample_lp.xn=If[0];
			LPFILTER_MACRO(Ia_sample_lp);
			LPFILTER_MACRO(Ib_sample_lp);
			LPFILTER_MACRO(Ic_sample_lp);
			LPFILTER_MACRO(Id_sample_lp);
			LPFILTER_MACRO(Ie_sample_lp);
			LPFILTER_MACRO(If_sample_lp);
			clarke1.As=Ia_sample_lp.yn; //cos(temPWM2_INT_Num*0.001*2*3.1415 - 0*0.5235987756);//Ia[0]*0.1; //
			clarke1.Bs=Ib_sample_lp.yn; //cos(temPWM2_INT_Num*0.001*2*3.1415 - 1*0.5235987756);//Ib[0]*0.1; //
			clarke1.Cs=Ic_sample_lp.yn; //cos(temPWM2_INT_Num*0.001*2*3.1415 - 4*0.5235987756);//Ic[0]*0.1; //
			clarke1.Ds=Id_sample_lp.yn; //cos(temPWM2_INT_Num*0.001*2*3.1415 - 5*0.5235987756);//Id[0]*0.1; //
			clarke1.Es=Ie_sample_lp.yn; //cos(temPWM2_INT_Num*0.001*2*3.1415 - 8*0.5235987756);//Ie[0]*0.1; //
			clarke1.Fs=If_sample_lp.yn; //cos(temPWM2_INT_Num*0.001*2*3.1415 - 9*0.5235987756);//If[0]*0.1; //
//			clarke1.As=Ia[0]; //cos(temPWM2_INT_Num*0.001*2*3.1415 - 0*0.5235987756);//Ia[0]*0.1; //
//			clarke1.Bs=Ib[0]; //cos(temPWM2_INT_Num*0.001*2*3.1415 - 1*0.5235987756);//Ib[0]*0.1; //
//			clarke1.Cs=Ic[0]; //cos(temPWM2_INT_Num*0.001*2*3.1415 - 4*0.5235987756);//Ic[0]*0.1; //
//			clarke1.Ds=Id[0]; //cos(temPWM2_INT_Num*0.001*2*3.1415 - 5*0.5235987756);//Id[0]*0.1; //
//			clarke1.Es=Ie[0]; //cos(temPWM2_INT_Num*0.001*2*3.1415 - 8*0.5235987756);//Ie[0]*0.1; //
//			clarke1.Fs=If[0]; //cos(temPWM2_INT_Num*0.001*2*3.1415 - 9*0.5235987756);//If[0]*0.1; //

			CLARKE_MACRO(clarke1);

			park1.Alpha = clarke1.Alpha;
			park1.Beta = clarke1.Beta;
			park1.Angle = tempSpeed_elec*two_pi; //rg1.Out;
			park1.Sine = sin(park1.Angle);
		 	park1.Cosine = cos(park1.Angle);

			PARK_MACRO(park1);


//*******************************PI参数设定****************************************//
			pid1_iq.Kp = My_idq_Kp;
			pid1_iq.Ki = My_idq_Ki*T;
			pid1_iq.OutMax=0.5*VdcLink;
			pid1_iq.OutMin=-0.5*VdcLink;

			pid1_id.Kp = My_idq_Kp;
			pid1_id.Ki = My_idq_Ki*T;
			pid1_id.OutMax=0.5*VdcLink;
			pid1_id.OutMin=-0.5*VdcLink;

			pid1_ix.Kp = My_ixy_Kp;
			pid1_ix.Ki = My_ixy_Ki*T;
			pid1_ix.OutMax=0.1*VdcLink;
			pid1_ix.OutMin=-0.1*VdcLink;

			pid1_iy.Kp = My_ixy_Kp;
			pid1_iy.Ki = My_ixy_Ki*T;
			pid1_iy.OutMax=0.1*VdcLink;
			pid1_iy.OutMin=-0.1*VdcLink;

			pid1_spd.Kp=My_sp_kp;
			pid1_spd.Ki=My_sp_ki*T*SpeedLoopPrescaler*5;//My_sp_ki*T*SpeedLoopPrescaler/0.2;
			pid1_spd.OutMax=10;
			pid1_spd.OutMin=-10;


//*********************************转速闭环*****************************************//
			RC_MACRO(rc1);
			if (++SpeedLoopCount > SpeedLoopPrescaler) //speed_loop
			{
					pid1_spd.Ref = rc1.SetpointValue*500;
					//pid1_spd.Fdb = (-1)*SpeedRpm_Lprprprpr*0.0005;//2000 ;标幺值
				    pid1_spd.Fdb = -Speed_from_theta;
					PID_MACRO(pid1_spd);
					SpeedLoopCount=1;
			}
//******************************************************************************//


//*******************电流iq,id闭环*********************//
			pid1_id.Ref =0;
			pid1_id.Fdb = park1.Ds;
			PID_MACRO(pid1_id);

//		   	if(Decr_Speed_Flag==1)
//		   		{
//		   		pid1_iq.Ref = 0; //remove speed loop, and let iq_ref=0, to decrease voltage, then to decrease speed
//		   		}
//		   	else
//		   		{
//		   		pid1_iq.Ref = pid1_spd.Out;        //IqRef:iq环参考电流, normal_mode
//		   		}

			pid1_iq.Ref = iq_temp;

            pid1_iq.Fdb = park1.Qs;     //iq环实际电流
			PID_MACRO(pid1_iq);
//**************************************************//



//******************电流ix,iy闭环*********************//
			pid1_ix.Ref =0;
		   	pid1_ix.Fdb=clarke1.X;


		   	pid1_iy.Ref =0;
		   	pid1_iy.Fdb=clarke1.Y;

	   		PID_MACRO(pid1_ix);
	   		PID_MACRO(pid1_iy);
//*************************************************//



//*******************************死区补偿电压计算 ************************************//
//		   	double_park(Ia[0],Ib[0],Ic[0],Id[0],Ie[0],If[0],tempSpeed_elec*two_pi);
//
//		   	Id1_lp.xn=Id1_temp;
//		   	LPFILTER_MACRO(Id1_lp);
//		   	Iq1_lp.xn=Iq1_temp;
//		   	LPFILTER_MACRO(Iq1_lp);
//		   	Id2_lp.xn=Id2_temp;
//		   	LPFILTER_MACRO(Id2_lp);
//		   	Iq2_lp.xn=Iq2_temp;
//		   	LPFILTER_MACRO(Iq2_lp);
//
//		   	double_anti_park(Id1_lp.yn,Iq1_lp.yn,Id2_lp.yn,Iq2_lp.yn,tempSpeed_elec*two_pi);
//
//			Deadtime_Compensation(Ia_filter, Ib_filter, Ic_filter, Id_filter, Ie_filter, If_filter, Td);
//
//			DT_clarke.As=compensation_A; //cos(temPWM2_INT_Num*0.001*2*3.1415 - 0*0.5235987756);//Ia[0]*0.1; //
//			DT_clarke.Bs=compensation_B; //cos(temPWM2_INT_Num*0.001*2*3.1415 - 1*0.5235987756);//Ib[0]*0.1; //
//			DT_clarke.Cs=compensation_C; //cos(temPWM2_INT_Num*0.001*2*3.1415 - 4*0.5235987756);//Ic[0]*0.1; //
//			DT_clarke.Ds=compensation_D; //cos(temPWM2_INT_Num*0.001*2*3.1415 - 5*0.5235987756);//Id[0]*0.1; //
//			DT_clarke.Es=compensation_E; //cos(temPWM2_INT_Num*0.001*2*3.1415 - 8*0.5235987756);//Ie[0]*0.1; //
//			DT_clarke.Fs=compensation_F; //cos(temPWM2_INT_Num*0.001*2*3.1415 - 9*0.5235987756);//If[0]*0.1; //
//
//		   	CLARKE_MACRO(DT_clarke);
//
//		   	DT_Alpha_lp.xn=DT_clarke.Alpha;
//		   	LPFILTER_MACRO(DT_Alpha_lp);
//		   	DT_Beta_lp.xn=DT_clarke.Beta;
//		   	LPFILTER_MACRO(DT_Beta_lp);
//		   	DT_X_lp.xn=DT_clarke.X;
//		   	LPFILTER_MACRO(DT_X_lp);
//		   	DT_Y_lp.xn=DT_clarke.Y;
//		   	LPFILTER_MACRO(DT_Y_lp);
//***********************************************************************//

//***********************PR*********************************************//

		   	if(wc!=wc_temp)//若准PR带宽改变，修改准PR带宽
		   	{
		   		Ix_PR=setPR(Ix_PR,SpeedRefRef,wc,kr);
		   		Iy_PR=setPR(Iy_PR,SpeedRefRef,wc,kr);
		   	}
		   	if(kr!=kr_temp)//若kr改变，修改kr
		   	{
		   		Ix_PR=setPR(Ix_PR,SpeedRefRef,wc,kr);
		   		Iy_PR=setPR(Iy_PR,SpeedRefRef,wc,kr);
		   	}
		   	if(SpeedRefRef!=SpeedRefRef_temp)//若转速基准改变，修改中心频率
		   	{
		   		Ix_PR=setPR(Ix_PR,SpeedRefRef,wc,kr);
		   		Iy_PR=setPR(Iy_PR,SpeedRefRef,wc,kr);
		   	}
		   	wc_temp=wc;
		   	kr_temp=kr;
		   	SpeedRefRef_temp=SpeedRefRef;
		   	//**********xy坐标系PR*********//

		   	Ix_lp.xn=-clarke1.X;
		   	LPFILTER_MACRO(Ix_lp);
		   	Iy_lp.xn=-clarke1.Y;
		   	LPFILTER_MACRO(Iy_lp);

		   	PR_ipark.Ds=Ix_lp.yn;//e^(jwt)
		   	PR_ipark.Qs=Iy_lp.yn;
		   	PR_ipark.Sine=sin(tempSpeed_elec*two_pi);
		   	PR_ipark.Cosine=cos(tempSpeed_elec*two_pi);
		   	IPARK_MACRO(PR_ipark);

		   	Ix_PR.xn=PR_ipark.Alpha;
		   	Iy_PR.xn=PR_ipark.Beta;
		   	PRFILTER_MACRO(Ix_PR);
		   	PRFILTER_MACRO(Iy_PR);

		   	PR_park.Alpha=Ix_PR.yn;//e^(-jwt)
		   	PR_park.Beta=Iy_PR.yn;
		   	PR_park.Sine=sin(tempSpeed_elec*two_pi);
		   	PR_park.Cosine=cos(tempSpeed_elec*two_pi);
		   	PARK_MACRO(PR_park);
		   	//*******************************//


		   	//**********PR输出限幅************//
		   	if (PR_park.Ds>VdcLink*0.5) PR_park.Ds=VdcLink*0.5;
		   	if (PR_park.Ds<-VdcLink*0.5) PR_park.Ds=-VdcLink*0.5;
		   	if (PR_park.Ds>PR_limitation) PR_park.Ds=PR_limitation;
		   	if (PR_park.Ds<-PR_limitation) PR_park.Ds=-PR_limitation;

		   	if (PR_park.Qs>VdcLink*0.5) PR_park.Qs=VdcLink*0.5;
		   	if (PR_park.Qs<-VdcLink*0.5) PR_park.Qs=-VdcLink*0.5;
		   	if (PR_park.Qs>PR_limitation) PR_park.Ds=PR_limitation;
		   	if (PR_park.Qs<-PR_limitation) PR_park.Ds=-PR_limitation;
		   	//*******************************//
//

//*************************反park变换输入至SVPWM************************//

			ipark1.Ds=pid1_id.Out; //VdTesting;
			ipark1.Qs=pid1_iq.Out; //VqTesting;
		   	ipark1.Sine=sin(tempSpeed_elec*two_pi);
		   	ipark1.Cosine=cos(tempSpeed_elec*two_pi);
		   	IPARK_MACRO(ipark1);

//*****************************************************************//


		   	switch (current_loop_control_mod)//电流闭环控制方式；1代表只加入xy电流环；2代表只加入电压补偿；3代表同时加入xy电流环和电压补偿；default代表原始电流环
		   	{
		   	case 1:
		   		svgen_dq1.Ualpha=ipark1.Alpha;
		   		svgen_dq1.Ubeta=ipark1.Beta;
		   		svgen_dq1.Ux=pid1_ix.Out+PR_park.Ds;
		   		svgen_dq1.Uy=pid1_iy.Out+PR_park.Qs;
		   		break;
		   	default:
		   		svgen_dq1.Ualpha=ipark1.Alpha;
		   		svgen_dq1.Ubeta=ipark1.Beta;
		   		svgen_dq1.Ux=pid1_ix.Out;
		   		svgen_dq1.Uy=pid1_iy.Out;
		   		break;
		   	}


			//*********************SVPWM_generation***********************//
		   	Sector_old=Sector;//sensorless算法在每个周期最后，epwm采用影子寄存器方法，在下一周期修改比较值，故利用Sector_old记录本周期扇区

			SVGEN_SIX_PHASE();

			Transfer_EPWMs_compAB_To_CPU01TOCPU02RAM();     // copy data to RAM (acting as the shadow registor)

			//************************************************************//
		}

	//************************************************************//


}


//void PID_init(void)
//{
//	// Initialize the PID_REG3 module for Id
//	pid1_id.Kp = My_iq_Kp;
//	pid1_id.Ki = My_iq_Ki*T;
//	pid1_id.Kd = 0;
//	pid1_id.Kc = 0;
//	pid1_id.OutMax = 150;//限电压
//	pid1_id.OutMin = -150;//
//
//	// Initialize the PID_REG3 module for Iq
//	pid1_iq.Kp = My_iq_Kp;
//	pid1_iq.Ki = My_iq_Ki*T;
//	pid1_iq.Kd = 0;
//	pid1_iq.Kc = 0;
//	pid1_iq.OutMax = 150;
//	pid1_iq.OutMin = -150;
//
//	// Initialize the PID_REG3 module for Ix
//	pid1_ix.Kp = My_ix_Kp;
//	pid1_ix.Ki = My_ix_Ki*T;
//	pid1_ix.Kd = 0;
//	pid1_ix.Kc = 0;
//	pid1_ix.OutMax = 50;
//	pid1_ix.OutMin = -50;
//
//	// Initialize the PID_REG3 module for Iy
//	pid1_iy.Kp = My_iy_Kp;
//	pid1_iy.Ki = My_iy_Ki*T;
//	pid1_iy.Kd = 0;
//	pid1_iy.Kc = 0;
//	pid1_iy.OutMax = 50;
//	pid1_iy.OutMin = -50;
//
//	//下面是两个转速环pi值设置 一个是三相下的转速换 一个是两相模式下的转速环
//	pid1_spd.Kp=My_sp_kp;
//	pid1_spd.Ki=My_sp_ki*T*SpeedLoopPrescaler/0.2;
//	pid1_spd.Kd=0;
//	pid1_spd.Kc=0;
//	pid1_spd.OutMax=0.9;//限电流
//	pid1_spd.OutMin=-0.9;
//}

Uint16 test_flag = 0;




__interrupt void epwm6_isr(void)
{
	//------Start of ADCs(ADCA and ADCD) & EPWM6_INT--------------------//
	EPwm6Regs.ETCLR.bit.INT = 1;            // Clear INT flag for this timer
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP3; // Acknowledge this interrupt to receive more interrupts from group

	Cal_function();

}

Uint16 EPWM_Eable_PWMs_Done_Flag = 0;
Uint16 EPWM_Stop_PWMs_Done_Flag = 0;
Uint32 Transfer_INT_time = 0;
Uint32 Transfer_INT_stop_cpu1=0;

Uint32 receive_data = 0;
Uint32 receive_comd = 0;

__interrupt void CPU02toCPU01IPC0IntHandler(void)
{
    //receive_data = IpcRegs.IPCRECVDATA; // data from CPU1
    receive_comd = IpcRegs.IPCRECVCOM;  // command from CPU1

    switch(receive_comd)
    {
    	case COMMAND_start_PWMs_done: //5
    		EPWM_Eable_PWMs_Done_Flag = 1;
    		break;
    	case COMMAND_stop_PWMs_done:  //6;
    		EPWM_Stop_PWMs_Done_Flag = 1;
    		break;
    	default:
    		break;
    }

    // Acknowledge IPC INT0 Flag and PIE to receive more interrupts
    IpcRegs.IPCACK.bit.IPC0 = 1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

void CPU01_Send_Command_To_CPU02(Uint16 CPUcomd)
{
	IpcRegs.IPCSENDCOM = CPUcomd;  // set comd flag
	IpcRegs.IPCSET.bit.IPC0 = 1;   // trigger a CUP2 Interrupt
}

void Transfer_EPWMs_compAB_To_CPU01TOCPU02RAM(void)
{
	*((Uint16 *)CPU1TOCPU2RAM_addr + 0)  = CompareA_EPWM01_PhaseD_temp;
	*((Uint16 *)CPU1TOCPU2RAM_addr + 1)  = CompareA_EPWM02_PhaseE_temp;
	*((Uint16 *)CPU1TOCPU2RAM_addr + 2)  = CompareA_EPWM03_PhaseF_temp;
	*((Uint16 *)CPU1TOCPU2RAM_addr + 3)  = CompareA_EPWM05_PhaseB_temp;
	*((Uint16 *)CPU1TOCPU2RAM_addr + 4)  = CompareA_EPWM11_PhaseC_temp;
	*((Uint16 *)CPU1TOCPU2RAM_addr + 5)  = CompareA_EPWM12_PhaseA_temp;

	*((Uint16 *)CPU1TOCPU2RAM_addr + 6)  = CompareB_EPWM01_PhaseD_temp;
	*((Uint16 *)CPU1TOCPU2RAM_addr + 7)  = CompareB_EPWM02_PhaseE_temp;
	*((Uint16 *)CPU1TOCPU2RAM_addr + 8)  = CompareB_EPWM03_PhaseF_temp;
	*((Uint16 *)CPU1TOCPU2RAM_addr + 9)  = CompareB_EPWM05_PhaseB_temp;
	*((Uint16 *)CPU1TOCPU2RAM_addr + 10) = CompareB_EPWM11_PhaseC_temp;
	*((Uint16 *)CPU1TOCPU2RAM_addr + 11) = CompareB_EPWM12_PhaseA_temp;

	*((Uint16 *)CPU1TOCPU2RAM_addr + 12) = NVC_start_flag;
	*((Uint16 *)CPU1TOCPU2RAM_addr + 13) = EPWM_Eable_Flag;
//	*((Uint16 *)CPU1TOCPU2RAM_addr + 14) = CompareA_EPWM03_PhaseF_temp[1];
//	*((Uint16 *)CPU1TOCPU2RAM_addr + 15) = CompareA_EPWM05_PhaseB_temp[1];
//	*((Uint16 *)CPU1TOCPU2RAM_addr + 16) = CompareA_EPWM11_PhaseC_temp[1];
//	*((Uint16 *)CPU1TOCPU2RAM_addr + 17) = CompareA_EPWM12_PhaseA_temp[1];
//	*((Uint16 *)CPU1TOCPU2RAM_addr + 18) = CompareB_EPWM01_PhaseD_temp[1];
//	*((Uint16 *)CPU1TOCPU2RAM_addr + 19) = CompareB_EPWM02_PhaseE_temp[1];
//	*((Uint16 *)CPU1TOCPU2RAM_addr + 20) = CompareB_EPWM03_PhaseF_temp[1];
//	*((Uint16 *)CPU1TOCPU2RAM_addr + 21) = CompareB_EPWM05_PhaseB_temp[1];
//	*((Uint16 *)CPU1TOCPU2RAM_addr + 22) = CompareB_EPWM11_PhaseC_temp[1];
//	*((Uint16 *)CPU1TOCPU2RAM_addr + 23) = CompareB_EPWM12_PhaseA_temp[1];

//	*((Uint16 *)CPU1TOCPU2RAM_addr + 24) = NVC_start_flag;
//	*((Uint16 *)CPU1TOCPU2RAM_addr + 25) = t1_negtive_vector_flag;
//	*((Uint16 *)CPU1TOCPU2RAM_addr + 26) = t2_negtive_vector_flag;
//	*((Uint16 *)CPU1TOCPU2RAM_addr + 27) = tx6;
//	*((Uint16 *)CPU1TOCPU2RAM_addr + 28) = CompareA_EPWM11_PhaseC_temp[2];
//	*((Uint16 *)CPU1TOCPU2RAM_addr + 29) = CompareA_EPWM12_PhaseA_temp[2];
//	*((Uint16 *)CPU1TOCPU2RAM_addr + 30) = CompareB_EPWM01_PhaseD_temp[2];
//	*((Uint16 *)CPU1TOCPU2RAM_addr + 31) = CompareB_EPWM02_PhaseE_temp[2];
//	*((Uint16 *)CPU1TOCPU2RAM_addr + 32) = CompareB_EPWM03_PhaseF_temp[2];
//	*((Uint16 *)CPU1TOCPU2RAM_addr + 33) = CompareB_EPWM05_PhaseB_temp[2];
//	*((Uint16 *)CPU1TOCPU2RAM_addr + 34) = CompareB_EPWM11_PhaseC_temp[2];
//	*((Uint16 *)CPU1TOCPU2RAM_addr + 35) = CompareB_EPWM12_PhaseA_temp[2];
}

__interrupt void epwm9_isr(void)
{
	//----With this function, we will upate CompareA&B for ADCs and other flags, at the end of Each PWM Cycle------//
	EPwm9Regs.ETCLR.bit.INT = 1;            // Clear INT flag for this timer
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP3; // Acknowledge this interrupt to receive more interrupts from group
    //int i;
	// Update PWMx_compareA&B in CPU02
	if(EPWM_Eable_Flag==1)
	{
		//For the purpose of starting CPU02_PWMs once.....
		if(EPWM_Eable_PWMs_Done_Flag != 1) CPU01_Send_Command_To_CPU02(COMMAND_start_PWMs);

		EPWM_Stop_PWMs_Done_Flag = 0;
	}
	else
	{
		if(EPWM_Stop_PWMs_Done_Flag!=1) CPU01_Send_Command_To_CPU02(COMMAND_stop_PWMs);

		EPWM_Eable_PWMs_Done_Flag = 0;
	}
	Ia[1] = (AdcaResultRegs.ADCRESULT2 - AdcResult_Ia_0A)*0.0099049;
	Ia[2] = (AdcaResultRegs.ADCRESULT4 - AdcResult_Ia_0A)*0.0099049;
	Ia[3] = (AdcaResultRegs.ADCRESULT6 - AdcResult_Ia_0A)*0.0099049;
	Ia[4] = (AdcaResultRegs.ADCRESULT8 - AdcResult_Ia_0A)*0.0099049;
	Ia[5] = (AdcaResultRegs.ADCRESULT10 - AdcResult_Ia_0A)*0.0099049;

	Ib[1] = (AdcaResultRegs.ADCRESULT3 - AdcResult_Ib_0A)*0.0099107;
	Ib[2] = (AdcaResultRegs.ADCRESULT5 - AdcResult_Ib_0A)*0.0099107;
	Ib[3] = (AdcaResultRegs.ADCRESULT7 - AdcResult_Ib_0A)*0.0099107;
	Ib[4] = (AdcaResultRegs.ADCRESULT9 - AdcResult_Ib_0A)*0.0099107;
	Ib[5] = (AdcaResultRegs.ADCRESULT11 - AdcResult_Ib_0A)*0.0099107;

	Ic[1] = (AdcbResultRegs.ADCRESULT3 - AdcResult_Ic_0A)*0.0098922;
	Ic[2] = (AdcbResultRegs.ADCRESULT5 - AdcResult_Ic_0A)*0.0098922;
	Ic[3] = (AdcbResultRegs.ADCRESULT7 - AdcResult_Ic_0A)*0.0098922;
	Ic[4] = (AdcbResultRegs.ADCRESULT9 - AdcResult_Ic_0A)*0.0098922;
	Ic[5] = (AdcbResultRegs.ADCRESULT11 - AdcResult_Ic_0A)*0.0098922;

	Id[1] = (AdcbResultRegs.ADCRESULT2 - AdcResult_Id_0A)*0.0099147;
	Id[2] = (AdcbResultRegs.ADCRESULT4 - AdcResult_Id_0A)*0.0099147;
	Id[3] = (AdcbResultRegs.ADCRESULT6 - AdcResult_Id_0A)*0.0099147;
	Id[4] = (AdcbResultRegs.ADCRESULT8 - AdcResult_Id_0A)*0.0099147;
	Id[5] = (AdcbResultRegs.ADCRESULT10 - AdcResult_Id_0A)*0.0099147;

	Ie[1] = (AdccResultRegs.ADCRESULT3 - AdcResult_Ie_0A)*0.0099611;
	Ie[2] = (AdccResultRegs.ADCRESULT5 - AdcResult_Ie_0A)*0.0099611;
	Ie[3] = (AdccResultRegs.ADCRESULT7 - AdcResult_Ie_0A)*0.0099611;
	Ie[4] = (AdccResultRegs.ADCRESULT9 - AdcResult_Ie_0A)*0.0099611;
	Ie[5] = (AdccResultRegs.ADCRESULT11 - AdcResult_Ie_0A)*0.0099611;

	If[1] = (AdccResultRegs.ADCRESULT2 - AdcResult_If_0A)*0.0099172;
	If[2] = (AdccResultRegs.ADCRESULT4 - AdcResult_If_0A)*0.0099172;
	If[3] = (AdccResultRegs.ADCRESULT6 - AdcResult_If_0A)*0.0099172;
	If[4] = (AdccResultRegs.ADCRESULT8 - AdcResult_If_0A)*0.0099172;
	If[5] = (AdccResultRegs.ADCRESULT10 - AdcResult_If_0A)*0.0099172;

	SensorLess_RotorPosition();

	EALLOW;
		if(show_compensation==0)
		{
		    DacaRegs.DACVALS.bit.DACVALS=(Uint16)(2048+500*pid1_iq.Fdb);
		    DacbRegs.DACVALS.bit.DACVALS=(Uint16)show_current_by_DAC(100*Sin2Theta);
		    DaccRegs.DACVALS.bit.DACVALS=(Uint16)(VdcLink*20);
		}
		if(show_compensation==1)
		{
			DacaRegs.DACVALS.bit.DACVALS=(Uint16)(2048+500*pid1_id.Fdb);
		    DacbRegs.DACVALS.bit.DACVALS=(Uint16)(Speed_from_theta*50);
		    DaccRegs.DACVALS.bit.DACVALS=(Uint16)show_current_by_DAC(Theta_error);

		}
		if(show_compensation==2)
		{
		    DacaRegs.DACVALS.bit.DACVALS=(Uint16)(Theta*200);
		    DacbRegs.DACVALS.bit.DACVALS=(Uint16)(tempSpeed_elec*200*two_pi);
		    DaccRegs.DACVALS.bit.DACVALS=(Uint16)show_current_by_DAC(Theta_error);
		}
		if(show_compensation==3)
		{
		    DacaRegs.DACVALS.bit.DACVALS=(Uint16)show_current_by_DAC(100*Cos2Theta);
		    DacbRegs.DACVALS.bit.DACVALS=(Uint16)show_current_by_DAC(100*Sin2Theta);
		    DaccRegs.DACVALS.bit.DACVALS=(Uint16)show_current_by_DAC(pIalpha2);
		}
		if(show_compensation==4)
		{
		    DacaRegs.DACVALS.bit.DACVALS=(Uint16)show_current_by_DAC(100*Cos2Theta);
		    DacbRegs.DACVALS.bit.DACVALS=(Uint16)show_current_by_DAC(100*Sin2Theta);
		    DaccRegs.DACVALS.bit.DACVALS=(Uint16)show_current_by_DAC(pIbeta1);
		}
		if(show_compensation==5)
		{
		    DacaRegs.DACVALS.bit.DACVALS=(Uint16)show_current_by_DAC(100*Cos2Theta);
		    DacbRegs.DACVALS.bit.DACVALS=(Uint16)show_current_by_DAC(100*Sin2Theta);
		    DaccRegs.DACVALS.bit.DACVALS=(Uint16)show_current_by_DAC(pIbeta2);
		}
		if(show_compensation==6)
		{
		    DacaRegs.DACVALS.bit.DACVALS=(Uint16)show_current_by_DAC(Ia[5]/5);
		    DacbRegs.DACVALS.bit.DACVALS=(Uint16)show_current_by_DAC(100*Cos2Theta);
		    DaccRegs.DACVALS.bit.DACVALS=(Uint16)show_current_by_DAC(Theta_error);
		}
		EDIS;
}

//__interrupt void epwm10_isr(void)
//{
//	//----With this function, we will upate CompareA&B for ADCs and other flags, at the end of Each PWM Cycle------//
//	EPwm10Regs.ETCLR.bit.INT = 1;            // Clear INT flag for this timer
//	PieCtrlRegs.PIEACK.all = PIEACK_GROUP3; // Acknowledge this interrupt to receive more interrupts from group
//
//}



float t_minus=0.06;



void SVGEN_SIX_PHASE(void)
{

	temp_sv1 = svgen_dq1.Ualpha*0.26794919;  // Ualpha*tan(15)
	temp_sv2 = svgen_dq1.Ubeta*0.26794919;   // Ubeta*tan(15)
//**********************section judgement************************************//
	if( (svgen_dq1.Ualpha>0) && (svgen_dq1.Ubeta>=0) ) //0~90;
	{
		if(temp_sv1 >= svgen_dq1.Ubeta)             Sector = 1;
		else{
			if(svgen_dq1.Ualpha >= svgen_dq1.Ubeta) Sector = 2;
			else {
				if(svgen_dq1.Ualpha < temp_sv2)     Sector = 4;
				else                                Sector = 3;
			}
		}
	}
	else
	{
		if( (svgen_dq1.Ualpha<=0) && (svgen_dq1.Ubeta>0) )//90~180;
		{
			if(temp_sv2 >= -svgen_dq1.Ualpha)            Sector = 4;
			else{
				if(svgen_dq1.Ubeta >= -svgen_dq1.Ualpha) Sector = 5;
				else {
					if(-temp_sv1 >= svgen_dq1.Ubeta)     Sector = 7;
					else                                 Sector = 6;
				}
			}
		}
		else
		{
			if( (svgen_dq1.Ualpha<=0) && (svgen_dq1.Ubeta<=0) )//180~270;
			{
				if(-temp_sv1 >= -svgen_dq1.Ubeta)             Sector = 7;
				else{
					if(-svgen_dq1.Ualpha >= -svgen_dq1.Ubeta) Sector = 8;
					else {
						if(-svgen_dq1.Ualpha < -temp_sv2)     Sector = 10;
						else                                  Sector = 9;
					}
				}
			}
			else   //270~360;
			{
				if(-temp_sv2 >= svgen_dq1.Ualpha)            Sector = 10;
				else{
					if(-svgen_dq1.Ubeta >= svgen_dq1.Ualpha) Sector = 11;
					else {
						if(temp_sv1 >= -svgen_dq1.Ubeta)     Sector = 1;
						else                                 Sector = 12;
					}
				}
			}
		}
	}

//********************************************************************************//


//*******************************time caculation**********************************//

    vectorI_select(Sector);


	t1=vector_inversion_matrix[0][0]*svgen_dq1.Ualpha + vector_inversion_matrix[0][1]*svgen_dq1.Ubeta + vector_inversion_matrix[0][2]*svgen_dq1.Ux + vector_inversion_matrix[0][3]*svgen_dq1.Uy;
	t2=vector_inversion_matrix[1][0]*svgen_dq1.Ualpha + vector_inversion_matrix[1][1]*svgen_dq1.Ubeta + vector_inversion_matrix[1][2]*svgen_dq1.Ux + vector_inversion_matrix[1][3]*svgen_dq1.Uy;
	t3=vector_inversion_matrix[2][0]*svgen_dq1.Ualpha + vector_inversion_matrix[2][1]*svgen_dq1.Ubeta + vector_inversion_matrix[2][2]*svgen_dq1.Ux + vector_inversion_matrix[2][3]*svgen_dq1.Uy;
	t4=vector_inversion_matrix[3][0]*svgen_dq1.Ualpha + vector_inversion_matrix[3][1]*svgen_dq1.Ubeta + vector_inversion_matrix[3][2]*svgen_dq1.Ux + vector_inversion_matrix[3][3]*svgen_dq1.Uy;

	if(t1<0){t1=0;}
	if(t2<0){t2=0;}
	if(t3<0){t3=0;}
	if(t4<0){t4=0;}

    t1=t1/VdcLink;
    t2=t2/VdcLink;
    t3=t3/VdcLink;
	t4=t4/VdcLink;


    if(NVC_start_flag==1)
    {
    	//t1为例，若t1<4.7us,t1=5us,t1_compensation=5us-t1作为负矢量补偿（之所以将t1判断条件设为4.7us，是为了出现小于0.3us的负矢量，负矢量太小没有意义）
    	if(t1<0.16)
    	{
    		t1_compensation=0.16-t1;
    		t1=0.16;
    		t1_negtive_vector_flag=1;
    	}
    	else
    	{
    		t1_compensation=0;
    		t1_negtive_vector_flag=0;
    	}

    	if(t2<0.16)
    	{
    		t2_compensation=0.16-t2;
    		t2=0.16;
    		t2_negtive_vector_flag=1;
    	}
    	else
    	{
    		t2_compensation=0;
    		t2_negtive_vector_flag=0;
    	}
    }
    else
    {
    	t1_negtive_vector_flag=0;
    	t2_negtive_vector_flag=0;
    	t1_compensation=0;
		t2_compensation=0;
    }

    t_sum=t1+t2+t3+t4;

    if(t_sum>0.4)
    {
    	t1=0.4*(t1/t_sum);
    	t2=0.4*(t2/t_sum);
    	t3=0.4*(t3/t_sum);
    	t4=0.4*(t4/t_sum);
    	t_sum=0.4;
    }
	t5 = 1-t_sum;
//********************************************************************************//

	temp_sv1 = t5*0.25;//t5/4;
	temp_sv2 = t5*0.5;//t5/2;


	if((Sector==1)||(Sector==2)||(Sector==5)||(Sector==6)||(Sector==9)||(Sector==10))
	{
		//先t2后t1，先补t1后补t2
		tx1 = temp_sv1;
		tx2 = tx1 + t2;
		tx3 = tx2 + t1;
		tx4 = tx3 + temp_sv2;
		tx5 = tx4 + t4;
		tx6 = tx5 + t3;
	}
	else
	{
		//先t1后t2，先补t2后补t1
		tx1 = temp_sv1;
		tx2 = tx1 + t1;
		tx3 = tx2 + t2;
		tx4 = tx3 + temp_sv2;
		tx5 = tx4 + t3;
		tx6 = tx5 + t4;
	}

	tx1 = EPWM_period*tx1;
	tx2 = EPWM_period*tx2;
	tx3 = EPWM_period*tx3;
	tx4 = EPWM_period*tx4;
	tx5 = EPWM_period*tx5;
	tx6 = EPWM_period*tx6;
	t1_compensation = EPWM_period*t1_compensation;
	t2_compensation = EPWM_period*t2_compensation;

	Epwm_CMPAB_assignment();

    EPwm7Regs.CMPA.bit.CMPA = tx1+195;       //tx2-1.4us trigger sample3
    EPwm7Regs.CMPB.bit.CMPB = tx1+395;       //tx2-0.4us trigger sample4
    EPwm8Regs.CMPA.bit.CMPA = tx2+195;       //tx3-1.4us trigger sample5
    EPwm8Regs.CMPB.bit.CMPB = tx2+395;       //tx3-0.4us trigger sample6


//    delta_t1=(tx2-tx1-105)/25000000.0;
//    delta_t2=(tx3-tx2-105)/25000000.0;
}


void DC_Relay_And_Led_Blink_Ctrl(void)
{
    //------For DC-Relay---------------------------------
    GPIO_SetupPinMux(6, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(6, GPIO_OUTPUT, GPIO_PUSHPULL);
    GPIO_WritePin(6, 0);
    DELAY_US(1000*1000*10);
    GPIO_WritePin(6, 1);

    //-------For Led-Blink------------------------------
    GPIO_SetupPinMux(70, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(70, GPIO_OUTPUT, GPIO_PUSHPULL);
    GPIO_WritePin(70, 1);
    //---------------------------------------------------
}

void InitEPWM_SOC_ADC()
{
	//* Tips:
	//* 1. PWM7, sets a flag( in its Interrupt function ),used to start the cal_function (in the while loop of the main function),
	//     which will get the rotor posion(elec_theta) and speed, and then pauses, waiting for the ending ADCs of Phase_current.
	//* 2. PWM10, clears the flag in Step2( in its Interrupt function ), to let cal_function finish its remain tasks.
	//* 3. PWM6,PWM8, triggers SOC_of_ADCs & EPWM8_INT
	//* 4. PWM9,Load_CMPA & CMPB_for PWM1/2/3/5/11/12/ at the end of each pwm cycle.


    //----PWM10---10KHz_interupt--Bring the waiting while to an end, in case of ADCs failure-----------------//
    EPwm10Regs.TBPRD = EPWM_period;                // Set timer period
    EPwm10Regs.TBPHS.bit.TBPHS = 0x0000;           // Phase is 0
    EPwm10Regs.TBCTR = 0x0000;                     // Clear counter

    // Setup TBCLK
    EPwm10Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; // Count up
    EPwm10Regs.TBCTL.bit.PHSEN = TB_DISABLE;    // Disable phase loading
    EPwm10Regs.TBCTL.bit.HSPCLKDIV = TB_DIV4;   // Clock ratio to SYSCLKOUT
    EPwm10Regs.TBCTL.bit.CLKDIV =0;             // TB_DIV4;

    EPwm10Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;   //CC_IMMEDIATE;//CC_SHADOW;   // Load registers every ZERO
    EPwm10Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;   //CC_IMMEDIATE;//CC_SHADOW;
    EPwm10Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO; //CC_LD_DISABLE;//CC_CTR_ZERO;
    EPwm10Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO; //CC_LD_DISABLE;//CC_CTR_ZERO;

    // Setup compare
    EPwm10Regs.CMPA.bit.CMPA = EPWM_period-50; // CompareA_EPWM8;
    EPwm10Regs.CMPB.bit.CMPB = EPWM_period;

   	// Interrupt where we will change the Deadband
    EPwm10Regs.ETSEL.bit.INTSEL =ET_CTRU_CMPA;    //ET_CTRU_CMPA;   // ET_CTR_ZERO;    // Select INT on Zero event  ET_CTRD_CMPA
    EPwm10Regs.ETSEL.bit.INTEN = 1;               // Enable INT
    EPwm10Regs.ETPS.bit.INTPRD = ET_1ST;          // Generate INT on 3rd event

    //----PWM6---10KHz_interupt--sencorless Calculation_CMPA & CMPB_for PWM1/2/3/5/11/12/---------------------//
    EPwm6Regs.TBPRD = EPWM_period;               // Set timer period
    EPwm6Regs.TBPHS.bit.TBPHS = 0x0000;          // Phase is 0
    EPwm6Regs.TBCTR = 0x0000;                    // Clear counter

    // Setup TBCLK
    EPwm6Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; // Count up
    EPwm6Regs.TBCTL.bit.PHSEN = TB_DISABLE;    // Disable phase loading
    EPwm6Regs.TBCTL.bit.HSPCLKDIV = TB_DIV4;   // Clock ratio to SYSCLKOUT
    EPwm6Regs.TBCTL.bit.CLKDIV =0;             // TB_DIV4;

    EPwm6Regs.CMPCTL.bit.SHDWAMODE = CC_IMMEDIATE; //CC_SHADOW;   // Load registers every ZERO
    EPwm6Regs.CMPCTL.bit.SHDWBMODE = CC_IMMEDIATE; //CC_SHADOW;
    EPwm6Regs.CMPCTL.bit.LOADAMODE = CC_LD_DISABLE;//CC_LD_DISABLE;//CC_CTR_ZERO;
    EPwm6Regs.CMPCTL.bit.LOADBMODE = CC_LD_DISABLE;//CC_LD_DISABLE;//CC_CTR_ZERO;

    // Setup compare
    EPwm6Regs.CMPA.bit.CMPA = 1;          // CompareA_EPWM8;
    EPwm6Regs.CMPB.bit.CMPB = 50;

    // Set actions
    EPwm6Regs.AQCTLA.bit.CAU = AQ_SET;    // Set PWM1A on Zero
    EPwm6Regs.AQCTLA.bit.CBU = AQ_CLEAR;

    EPwm6Regs.ETSEL.bit.SOCASEL = 4;  // 100: Enable event: time-base counter equal to CMPA when the timeris incrementing
	EPwm6Regs.ETPS.bit.SOCAPRD = 1;   // These bits determine how many selected ETSEL[SOCASEL] events need to occur before an EPWMxSOCA pulse is generated.
	EPwm6Regs.ETSEL.bit.SOCAEN = 1;   // Enable the ADC Start of Conversion A (EPWMxSOCA) Pulse. =1: Enable EPWMxSOCA pulse.

	EPwm6Regs.ETSEL.bit.SOCBSEL = 6; // 110: Enable event: time-base counter equal to CMPB when the timeris incrementing
	EPwm6Regs.ETPS.bit.SOCBPRD = 1;  // These bits determine how many selected ETSEL[SOCBSEL] events need to occur before an EPWMxSOCB pulse is generated.
	EPwm6Regs.ETSEL.bit.SOCBEN = 1;  // Enable the ADC Start of Conversion A (EPWMxSOCB) Pulse. =1: Enable EPWMxSOCB pulse.

	// Interrupt where we will change the Deadband
    EPwm6Regs.ETSEL.bit.INTSEL =ET_CTRU_CMPB;  // ET_CTRU_CMPA;   // ET_CTR_ZERO;    // Select INT on Zero event  ET_CTRD_CMPA
    EPwm6Regs.ETSEL.bit.INTEN = 1;             // Enable INT
    EPwm6Regs.ETPS.bit.INTPRD = ET_1ST;        // Generate INT on 3rd event




    //----PWM7---10KHz_interrupt_for_starting_calculation----------------------------------
    EPwm7Regs.TBPRD = EPWM_period;                // Set timer period,10kHz
    EPwm7Regs.TBPHS.bit.TBPHS = 0x0000;           // Phase is 0
    EPwm7Regs.TBCTR = 0x0000;                     // Clear counter

    // Setup TBCLK
    EPwm7Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; // Count up
    EPwm7Regs.TBCTL.bit.PHSEN = TB_DISABLE;    // Disable phase loading
    EPwm7Regs.TBCTL.bit.HSPCLKDIV = TB_DIV4;   // Clock ratio to SYSCLKOUT
    EPwm7Regs.TBCTL.bit.CLKDIV =0;             // TB_DIV4;

    EPwm7Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;   // Load registers every ZERO
    EPwm7Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm7Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm7Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

    // Setup compare
    EPwm7Regs.CMPA.bit.CMPA = 0; // CompareA_EPWM8;
    EPwm7Regs.CMPB.bit.CMPB = 20;

    EPwm7Regs.AQCTLA.bit.CAU = AQ_SET;         // Set PWM1A on Zero
    EPwm7Regs.AQCTLA.bit.CBU = AQ_CLEAR;

    EPwm7Regs.ETSEL.bit.SOCASEL = 4; // 100: Enable event: time-base counter equal to CMPA when the timeris incrementing
	EPwm7Regs.ETPS.bit.SOCAPRD = 1;  // These bits determine how many selected ETSEL[SOCASEL] events need to occur before an EPWMxSOCA pulse is generated.
	EPwm7Regs.ETSEL.bit.SOCAEN = 1;  // Enable the ADC Start of Conversion A (EPWMxSOCA) Pulse. =1: Enable EPWMxSOCA pulse.

    EPwm7Regs.ETSEL.bit.SOCBSEL = 6; // 110: Enable event: time-base counter equal to CMPB when the timeris incrementing
	EPwm7Regs.ETPS.bit.SOCBPRD = 1;  // These bits determine how many selected ETSEL[SOCBSEL] events need to occur before an EPWMxSOCB pulse is generated.
	EPwm7Regs.ETSEL.bit.SOCBEN = 1;  // Enable the ADC Start of Conversion A (EPWMxSOCB) Pulse. =1: Enable EPWMxSOCB pulse.



    //----PWM8---10KHz_interupt--trigger sample5 and sample6---------------------//
    EPwm8Regs.TBPRD = EPWM_period;               // Set timer period
    EPwm8Regs.TBPHS.bit.TBPHS = 0x0000;          // Phase is 0
    EPwm8Regs.TBCTR = 0x0000;                    // Clear counter

    // Setup TBCLK
    EPwm8Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; // Count up
    EPwm8Regs.TBCTL.bit.PHSEN = TB_DISABLE;    // Disable phase loading
    EPwm8Regs.TBCTL.bit.HSPCLKDIV = TB_DIV4;   // Clock ratio to SYSCLKOUT
    EPwm8Regs.TBCTL.bit.CLKDIV =0;             // TB_DIV4;

    EPwm8Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;//CC_SHADOW;   // Load registers every ZERO
    EPwm8Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;//CC_SHADOW;
    EPwm8Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;//CC_LD_DISABLE;//CC_CTR_ZERO;
    EPwm8Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;//CC_LD_DISABLE;//CC_CTR_ZERO;

    // Setup compare
    EPwm8Regs.CMPA.bit.CMPA = 1000;   // CompareA_EPWM8;
    EPwm8Regs.CMPB.bit.CMPB = 1125;

    // Set actions
    EPwm8Regs.AQCTLA.bit.CAU = AQ_SET;         // Set PWM1A on Zero
    EPwm8Regs.AQCTLA.bit.CBU = AQ_CLEAR;

    EPwm8Regs.ETSEL.bit.SOCASEL = 4; // 100: Enable event: time-base counter equal to CMPA when the timeris incrementing
	EPwm8Regs.ETPS.bit.SOCAPRD = 1;  // These bits determine how many selected ETSEL[SOCASEL] events need to occur before an EPWMxSOCA pulse is generated.
	EPwm8Regs.ETSEL.bit.SOCAEN = 1;  // Enable the ADC Start of Conversion A (EPWMxSOCA) Pulse. =1: Enable EPWMxSOCA pulse.

    EPwm8Regs.ETSEL.bit.SOCBSEL = 6; // 110: Enable event: time-base counter equal to CMPB when the timeris incrementing
	EPwm8Regs.ETPS.bit.SOCBPRD = 1;  // These bits determine how many selected ETSEL[SOCBSEL] events need to occur before an EPWMxSOCB pulse is generated.
	EPwm8Regs.ETSEL.bit.SOCBEN = 1;  // Enable the ADC Start of Conversion A (EPWMxSOCB) Pulse. =1: Enable EPWMxSOCB pulse.



    //----PWM9---10KHz_interupt--Load_CMPA & CMPB_for PWM1/2/3/5/11/12/---------------------//
    EPwm9Regs.TBPRD = EPWM_period;                // Set timer period
    EPwm9Regs.TBPHS.bit.TBPHS = 0x0000;           // Phase is 0
    EPwm9Regs.TBCTR = 0x0000;                     // Clear counter

    // Setup TBCLK
    EPwm9Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; // Count up
    EPwm9Regs.TBCTL.bit.PHSEN = TB_DISABLE;    // Disable phase loading
    EPwm9Regs.TBCTL.bit.HSPCLKDIV = TB_DIV4;   // Clock ratio to SYSCLKOUT
    EPwm9Regs.TBCTL.bit.CLKDIV =0;             // TB_DIV4;

    EPwm9Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;   //CC_IMMEDIATE;//CC_SHADOW;   // Load registers every ZERO
    EPwm9Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;   //CC_IMMEDIATE;//CC_SHADOW;
    EPwm9Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO; //CC_LD_DISABLE;//CC_CTR_ZERO;
    EPwm9Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO; //CC_LD_DISABLE;//CC_CTR_ZERO;

    // Setup compare
    EPwm9Regs.CMPA.bit.CMPA = EPWM_period - 500; // CompareA_EPWM8;
    EPwm9Regs.CMPB.bit.CMPB = EPWM_period;

	// Interrupt where we will change the Deadband
    EPwm9Regs.ETSEL.bit.INTSEL =ET_CTRU_CMPA;    //ET_CTRU_CMPA;   // ET_CTR_ZERO;    // Select INT on Zero event  ET_CTRD_CMPA
    EPwm9Regs.ETSEL.bit.INTEN = 1;               // Enable INT
    EPwm9Regs.ETPS.bit.INTPRD = ET_1ST;          // Generate INT on 3rd event

}

void Enable_Force_PWM_allout_low(void)
{

	show_PWM_low++;
	EPWM_Eable_Flag = 0;  // clear EPWM enable flag

	if(EPWM_Stop_PWMs_Done_Flag!=1) CPU01_Send_Command_To_CPU02(COMMAND_stop_PWMs);

	CompareA_EPWM12_PhaseA_temp=0;     //phaseA
	CompareB_EPWM12_PhaseA_temp=0;
	CompareA_EPWM05_PhaseB_temp=0;     //phaseB
	CompareB_EPWM05_PhaseB_temp=0;

	CompareA_EPWM11_PhaseC_temp=0; //phaseC
	CompareB_EPWM11_PhaseC_temp=0;
	CompareA_EPWM01_PhaseD_temp=0; //phaseD
	CompareB_EPWM01_PhaseD_temp=0;

	CompareA_EPWM02_PhaseE_temp=0; //phaseE
	CompareB_EPWM02_PhaseE_temp=0;
	CompareA_EPWM03_PhaseF_temp=0; //phaseF
	CompareB_EPWM03_PhaseF_temp=0;

	//-------------------------------------------------------
}


//-------added by xiaomutong-2016-6-21---------------------
void InitEPWMGpio_SixPhase(void)
{
   EALLOW;
// Disable internal pull-up for the selected output pins
// for reduced power consumption
// Pull-ups can be enabled or disabled by the user.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPAPUD.bit.GPIO0 = 1;    // Disable pull-up on GPIO0 (EPWM1A)
    GpioCtrlRegs.GPAPUD.bit.GPIO1 = 1;    // Disable pull-up on GPIO1 (EPWM1B)
    GpioCtrlRegs.GPAPUD.bit.GPIO2 = 1;    // Disable pull-up on GPIO0 (EPWM2A)
    GpioCtrlRegs.GPAPUD.bit.GPIO3 = 1;    // Disable pull-up on GPIO1 (EPWM2B)
    GpioCtrlRegs.GPAPUD.bit.GPIO4 = 1;    // Disable pull-up on GPIO0 (EPWM3A)
    GpioCtrlRegs.GPAPUD.bit.GPIO5 = 1;    // Disable pull-up on GPIO1 (EPWM3B)

    GpioCtrlRegs.GPAPUD.bit.GPIO6 = 1;    // Disable pull-up on GPIO1 (EPWM4A)
    GpioCtrlRegs.GPAPUD.bit.GPIO7 = 1;    // Disable pull-up on GPIO1 (EPWM4B)

    GpioCtrlRegs.GPAPUD.bit.GPIO8 = 1;    // Disable pull-up on GPIO0 (EPWM5A)
    GpioCtrlRegs.GPAPUD.bit.GPIO9 = 1;    // Disable pull-up on GPIO1 (EPWM5B)

    GpioCtrlRegs.GPAPUD.bit.GPIO10 = 1;    // Disable pull-up on GPIO1 (EPWM6A)

    GpioCtrlRegs.GPAPUD.bit.GPIO12 = 1;    // Disable pull-up on GPIO1 (EPWM8A)
    GpioCtrlRegs.GPAPUD.bit.GPIO14 = 1;    // Disable pull-up on GPIO1 (EPWM8A)

    GpioCtrlRegs.GPAPUD.bit.GPIO20 = 1;    // Disable pull-up on GPIO0 (EPWM11A)
    GpioCtrlRegs.GPAPUD.bit.GPIO21 = 1;    // Disable pull-up on GPIO1 (EPWM11B)
    GpioCtrlRegs.GPAPUD.bit.GPIO22 = 1;    // Disable pull-u p on GPIO0 (EPWM12A)
    GpioCtrlRegs.GPAPUD.bit.GPIO23 = 1;    // Disable pull-up on GPIO1 (EPWM12B)
// Configure EPWM-1 pins using GPIO regs
// This specifies which of the possible GPIO pins will be EPWM1 functional pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;   // Configure GPIO0 as EPWM1A
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;   // Configure GPIO1 as EPWM1B
    GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;   // Configure GPIO2 as EPWM2A
    GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1;   // Configure GPIO3 as EPWM2B
    GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 1;   // Configure GPIO4 as EPWM3A
    GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 1;   // Configure GPIO5 as EPWM3B

    GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 1;   // Configure GPIO6 as EPWM4A
    GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 1;   // Configure GPIO7 as EPWM4B

    GpioCtrlRegs.GPAMUX1.bit.GPIO8 = 1;   // Configure GPIO8 as EPWM5A
    GpioCtrlRegs.GPAMUX1.bit.GPIO9 = 1;   // Configure GPIO9 as EPWM5B

    GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 1;   // Configure GPIO10 as EPWM6A

    GpioCtrlRegs.GPAMUX1.bit.GPIO12 = 1;   // Configure GPIO12 as EPWM7A
    GpioCtrlRegs.GPAMUX1.bit.GPIO14 = 1;   // Configure GPIO14 as EPWM8A

    GpioCtrlRegs.GPAMUX2.bit.GPIO20 = 1;   // Configure GPIO20 as EPWM11A
    GpioCtrlRegs.GPAGMUX2.bit.GPIO20 =1;
    GpioCtrlRegs.GPAMUX2.bit.GPIO21 = 1;   // Configure GPIO21 as EPWM11B
    GpioCtrlRegs.GPAGMUX2.bit.GPIO21 =1;
    GpioCtrlRegs.GPAMUX2.bit.GPIO22 = 1;   // Configure GPIO22 as EPWM12A
    GpioCtrlRegs.GPAGMUX2.bit.GPIO22 =1;
    GpioCtrlRegs.GPAMUX2.bit.GPIO23 = 1;   // Configure GPIO23 as EPWM12B
    GpioCtrlRegs.GPAGMUX2.bit.GPIO23 =1;
    EDIS;
}
//-------------------------------------------------------------------------------


void StartADC_One_Conversion_SixPhase(void)
{

	 //*****实测：1A 变化量<==>ADC采样变化51.269，正反向ADC采样取平均值******//
	if(ADC_0V_adjust_done==0) //只在开机前，校对0电流的ADC值，一旦开机后，不管停止否，不再进行校对
	{
		if(EPWM_Eable_Flag==1) {ADC_0V_adjust_done=1;}

		if(ADC_0V_adjust_start==1) //2秒内，先求10万次的和，然后再求平均
		{
				AdcResult_Ia_0A = AdcResult_Ia_0A + (AdcaResultRegs.ADCRESULT0);
				AdcResult_Ib_0A = AdcResult_Ib_0A + (AdcaResultRegs.ADCRESULT1);
				AdcResult_Ic_0A = AdcResult_Ic_0A + (AdcbResultRegs.ADCRESULT0);
				AdcResult_Id_0A = AdcResult_Id_0A + (AdcbResultRegs.ADCRESULT1);
				AdcResult_Ie_0A = AdcResult_Ie_0A + (AdccResultRegs.ADCRESULT0);
				AdcResult_If_0A = AdcResult_If_0A + (AdccResultRegs.ADCRESULT1);

			if(++ADC_adj_num == 50000) //10秒内，先求10万次的和，然后再求平均
			{
				ADC_0V_adjust_start = 0;
				ADC_0V_adjust_done = 1;
				AdcResult_Ia_0A = 0.00002*AdcResult_Ia_0A;
				AdcResult_Ib_0A = 0.00002*AdcResult_Ib_0A;
				AdcResult_Ic_0A = 0.00002*AdcResult_Ic_0A;
				AdcResult_Id_0A = 0.00002*AdcResult_Id_0A;
				AdcResult_Ie_0A = 0.00002*AdcResult_Ie_0A;
				AdcResult_If_0A = 0.00002*AdcResult_If_0A;

				ADC_adj_num = 0;
			}
		}
	}
	Ia[0] = (AdcaResultRegs.ADCRESULT0 - AdcResult_Ia_0A)*0.0099049; // Ia=0,采样值=2025.32, 1/51.269=0.019504964
    Ib[0] = (AdcaResultRegs.ADCRESULT1 - AdcResult_Ib_0A)*0.0099107; // Ib=0,采样值=2065.40, 1/51.269=0.019504964
	Ic[0] = (AdcbResultRegs.ADCRESULT0 - AdcResult_Ic_0A)*0.0098922 ; // Ic=0,采样值=2066.95, 1/51.269=0.019504964
	Id[0] = (AdcbResultRegs.ADCRESULT1 - AdcResult_Id_0A)*0.0099147; // Id=0,采样值=2064.13, 1/51.269=0.019504964
	Ie[0] = (AdccResultRegs.ADCRESULT0 - AdcResult_Ie_0A)*0.0099611; // Ie=0,采样值=2060.02, 1/51.269=0.019504964
	If[0] = (AdccResultRegs.ADCRESULT1 - AdcResult_If_0A)*0.0099172; // If=0,采样值=2068.47, 1/51.269=0.019504964


	 if (AdcdResult2_VdcLink>0)
		 {
			 VdcLink = 0.0585897 * AdcdResultRegs.ADCRESULT0 + 1.0306929;
		 }
     else if (AdcdResult2_VdcLink==0)
		 {
			 VdcLink = 0;
		 }
}



void ConfigureADC_SixPhase(void)
{
	EALLOW;

	//write configurations
	AdcaRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
	AdcbRegs.ADCCTL2.bit.PRESCALE = 6;   //set ADCCLK divider to /4
	AdccRegs.ADCCTL2.bit.PRESCALE = 6;   //set ADCCLK divider to /4
	AdcdRegs.ADCCTL2.bit.PRESCALE = 6;   //set ADCCLK divider to /4
	//---end of change

    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdcSetMode(ADC_ADCB, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdcSetMode(ADC_ADCC, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdcSetMode(ADC_ADCD, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    //---end of change

	//Set pulse positions to late， generate the EOC at the end of the voltage conversion.
	AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;
	AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1;
	AdccRegs.ADCCTL1.bit.INTPULSEPOS = 1;
	AdcdRegs.ADCCTL1.bit.INTPULSEPOS = 1;
	//---end of change

	//power up the ADCs
	AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;
	AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;
	AdccRegs.ADCCTL1.bit.ADCPWDNZ = 1;
	AdcdRegs.ADCCTL1.bit.ADCPWDNZ = 1;
	//---end of change

	//delay for 1ms to allow ADC time to power up
	DELAY_US(4000);
	EDIS;
}

void SetupADCoftware_SixPhase(void)
{
	Uint16 acqps;

	//determine minimum acquisition window (in SYSCLKS) based on resolution
	if(ADC_RESOLUTION_12BIT == AdcaRegs.ADCCTL2.bit.RESOLUTION){
		acqps = 14; //75ns
	}
	else { //resolution is 16-bit
		acqps = 63; //320ns
	}

    EALLOW;
//***********************************Ia/Ib ADCA2/A3**********************************//
    //*************Ia_ADCA2***********//
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 2;  //SOC0 will convert pin A2 for Ia
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = acqps; //sample window is acqps + 1 SYSCLK cycles
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 0xF;  //TRIGSEL = 0x0Fh ADCTRIG15 - ePWM6, ADCSOCA

    AdcaRegs.ADCSOC2CTL.bit.CHSEL = 2;  //SOC2 will convert pin A2 for Ia
    AdcaRegs.ADCSOC2CTL.bit.ACQPS = acqps; //sample window is acqps + 1 SYSCLK cycles
    AdcaRegs.ADCSOC2CTL.bit.TRIGSEL = 0x10;  //TRIGSEL = 0x10h ADCTRIG16 - ePWM6, ADCSOCB

    AdcaRegs.ADCSOC4CTL.bit.CHSEL = 2;  //SOC4 will convert pin A2 for Ia
    AdcaRegs.ADCSOC4CTL.bit.ACQPS = acqps; //sample window is acqps + 1 SYSCLK cycles
    AdcaRegs.ADCSOC4CTL.bit.TRIGSEL = 0x11;  //TRIGSEL = 0x11h ADCTRIG17 - ePWM7, ADCSOCA

    AdcaRegs.ADCSOC6CTL.bit.CHSEL = 2;  //SOC6 will convert pin A2 for Ia
    AdcaRegs.ADCSOC6CTL.bit.ACQPS = acqps; //sample window is acqps + 1 SYSCLK cycles
    AdcaRegs.ADCSOC6CTL.bit.TRIGSEL = 0x12;  //TRIGSEL = 0x12h ADCTRIG18 - ePWM7, ADCSOCB

    AdcaRegs.ADCSOC8CTL.bit.CHSEL = 2;  //SOC8 will convert pin A2 for Ia
    AdcaRegs.ADCSOC8CTL.bit.ACQPS = acqps; //sample window is acqps + 1 SYSCLK cycles
    AdcaRegs.ADCSOC8CTL.bit.TRIGSEL = 0x13;  //TRIGSEL = 0x13h ADCTRIG19 - ePWM8, ADCSOCA

    AdcaRegs.ADCSOC10CTL.bit.CHSEL = 2;  //SOC10 will convert pin A2 for Ia
    AdcaRegs.ADCSOC10CTL.bit.ACQPS = acqps; //sample window is acqps + 1 SYSCLK cycles
    AdcaRegs.ADCSOC10CTL.bit.TRIGSEL = 0x14;  //TRIGSEL = 0x14h ADCTRIG20 - ePWM8, ADCSOCB
    //**********************************//
    //**************Ib_ADCA3************//
    AdcaRegs.ADCSOC1CTL.bit.CHSEL = 3;  //SOC1 will convert pin A3 for Ib
    AdcaRegs.ADCSOC1CTL.bit.ACQPS = acqps; //sample window is acqps + 1 SYSCLK cycles
    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = 0xF;  //TRIGSEL = 0x0Fh ADCTRIG15 - ePWM6, ADCSOCA

    AdcaRegs.ADCSOC3CTL.bit.CHSEL = 3;  //SOC3 will convert pin A3 for Ib
    AdcaRegs.ADCSOC3CTL.bit.ACQPS = acqps; //sample window is acqps + 1 SYSCLK cycles
    AdcaRegs.ADCSOC3CTL.bit.TRIGSEL = 0x10;  //TRIGSEL = 0x10h ADCTRIG16 - ePWM6, ADCSOCB

    AdcaRegs.ADCSOC5CTL.bit.CHSEL = 3;  //SOC5 will convert pin A3 for Ib
    AdcaRegs.ADCSOC5CTL.bit.ACQPS = acqps; //sample window is acqps + 1 SYSCLK cycles
    AdcaRegs.ADCSOC5CTL.bit.TRIGSEL = 0x11;  //TRIGSEL = 0x11h ADCTRIG17 - ePWM7, ADCSOCA

    AdcaRegs.ADCSOC7CTL.bit.CHSEL = 3;  //SOC7 will convert pin A3 for Ib
    AdcaRegs.ADCSOC7CTL.bit.ACQPS = acqps; //sample window is acqps + 1 SYSCLK cycles
    AdcaRegs.ADCSOC7CTL.bit.TRIGSEL = 0x12;  //TRIGSEL = 0x12h ADCTRIG18 - ePWM7, ADCSOCB

    AdcaRegs.ADCSOC9CTL.bit.CHSEL = 3;  //SOC9 will convert pin A3 for Ib
    AdcaRegs.ADCSOC9CTL.bit.ACQPS = acqps; //sample window is acqps + 1 SYSCLK cycles
    AdcaRegs.ADCSOC9CTL.bit.TRIGSEL = 0x13;  //TRIGSEL = 0x13h ADCTRIG19 - ePWM8, ADCSOCA

    AdcaRegs.ADCSOC11CTL.bit.CHSEL = 3;  //SOC11 will convert pin A3 for Ib
    AdcaRegs.ADCSOC11CTL.bit.ACQPS = acqps; //sample window is acqps + 1 SYSCLK cycles
    AdcaRegs.ADCSOC11CTL.bit.TRIGSEL = 0x14;  //TRIGSEL = 0x14h ADCTRIG20 - ePWM8, ADCSOCB
    //***********************************//


    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 11; //end of SOC11 will set INT1 flag
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;   //enable INT1 flag
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared
//*************************************************************************//

//**********************************Ic/Id ADCB0/B2*********************************************//
    //**************Ic_ADCB0************//
    AdcbRegs.ADCSOC0CTL.bit.CHSEL = 0;  //SOC0 will convert pin B0 for Ic
    AdcbRegs.ADCSOC0CTL.bit.ACQPS = acqps; //sample window is acqps + 1 SYSCLK cycles
    AdcbRegs.ADCSOC0CTL.bit.TRIGSEL = 0xF;  //TRIGSEL = 0x0Fh ADCTRIG15 - ePWM6, ADCSOCA

    AdcbRegs.ADCSOC2CTL.bit.CHSEL = 0;  //SOC2 will convert pin B0 for Ic
    AdcbRegs.ADCSOC2CTL.bit.ACQPS = acqps; //sample window is acqps + 1 SYSCLK cycles
    AdcbRegs.ADCSOC2CTL.bit.TRIGSEL = 0x10;  //TRIGSEL = 0x10h ADCTRIG16 - ePWM6, ADCSOCB

    AdcbRegs.ADCSOC4CTL.bit.CHSEL = 0;  //SOC4 will convert pin B0 for Ic
    AdcbRegs.ADCSOC4CTL.bit.ACQPS = acqps; //sample window is acqps + 1 SYSCLK cycles
    AdcbRegs.ADCSOC4CTL.bit.TRIGSEL = 0x11;  //TRIGSEL = 0x11h ADCTRIG17 - ePWM7, ADCSOCA

    AdcbRegs.ADCSOC6CTL.bit.CHSEL = 0;  //SOC6 will convert pin B0 for Ic
    AdcbRegs.ADCSOC6CTL.bit.ACQPS = acqps; //sample window is acqps + 1 SYSCLK cycles
    AdcbRegs.ADCSOC6CTL.bit.TRIGSEL = 0x12;  //TRIGSEL = 0x12h ADCTRIG18 - ePWM7, ADCSOCB

    AdcbRegs.ADCSOC8CTL.bit.CHSEL = 0;  //SOC8 will convert pin B0 for Ic
    AdcbRegs.ADCSOC8CTL.bit.ACQPS = acqps; //sample window is acqps + 1 SYSCLK cycles
    AdcbRegs.ADCSOC8CTL.bit.TRIGSEL = 0x13;  //TRIGSEL = 0x13h ADCTRIG19 - ePWM8, ADCSOCA

    AdcbRegs.ADCSOC10CTL.bit.CHSEL = 0;  //SOC10 will convert pin B0 for Ic
    AdcbRegs.ADCSOC10CTL.bit.ACQPS = acqps; //sample window is acqps + 1 SYSCLK cycles
    AdcbRegs.ADCSOC10CTL.bit.TRIGSEL = 0x14;  //TRIGSEL = 0x14h ADCTRIG20 - ePWM8, ADCSOCB
    //***********************************//

    //*************Id_ADCB2***************//
    AdcbRegs.ADCSOC1CTL.bit.CHSEL = 2;  //SOC1 will convert pin B2 for Id
    AdcbRegs.ADCSOC1CTL.bit.ACQPS = acqps; //sample window is acqps + 1 SYSCLK cycles
    AdcbRegs.ADCSOC1CTL.bit.TRIGSEL = 0xF;  //TRIGSEL = 0x0Fh ADCTRIG15 - ePWM6, ADCSOCA

    AdcbRegs.ADCSOC3CTL.bit.CHSEL = 2;  //SOC3 will convert pin B2 for Id
    AdcbRegs.ADCSOC3CTL.bit.ACQPS = acqps; //sample window is acqps + 1 SYSCLK cycles
    AdcbRegs.ADCSOC3CTL.bit.TRIGSEL = 0x10;  //TRIGSEL = 0x10h ADCTRIG16 - ePWM6, ADCSOCB

    AdcbRegs.ADCSOC5CTL.bit.CHSEL = 2;  //SOC5 will convert pin B2 for Id
    AdcbRegs.ADCSOC5CTL.bit.ACQPS = acqps; //sample window is acqps + 1 SYSCLK cycles
    AdcbRegs.ADCSOC5CTL.bit.TRIGSEL = 0x11;  //TRIGSEL = 0x11h ADCTRIG17 - ePWM7, ADCSOCA

    AdcbRegs.ADCSOC7CTL.bit.CHSEL = 2;  //SOC7 will convert pin B2 for Id
    AdcbRegs.ADCSOC7CTL.bit.ACQPS = acqps; //sample window is acqps + 1 SYSCLK cycles
    AdcbRegs.ADCSOC7CTL.bit.TRIGSEL = 0x12;  //TRIGSEL = 0x12h ADCTRIG18 - ePWM7, ADCSOCB

    AdcbRegs.ADCSOC9CTL.bit.CHSEL = 2;  //SOC9 will convert pin B2 for Id
    AdcbRegs.ADCSOC9CTL.bit.ACQPS = acqps; //sample window is acqps + 1 SYSCLK cycles
    AdcbRegs.ADCSOC9CTL.bit.TRIGSEL = 0x13;  //TRIGSEL = 0x13h ADCTRIG19 - ePWM8, ADCSOCA

    AdcbRegs.ADCSOC11CTL.bit.CHSEL = 2;  //SOC11 will convert pin B2 for Id
    AdcbRegs.ADCSOC11CTL.bit.ACQPS = acqps; //sample window is acqps + 1 SYSCLK cycles
    AdcbRegs.ADCSOC11CTL.bit.TRIGSEL = 0x14;  //TRIGSEL = 0x14h ADCTRIG20 - ePWM8, ADCSOCB
    //**********************************//

    AdcbRegs.ADCINTSEL1N2.bit.INT1SEL = 11; //end of SOC11 will set INT1 flag
    AdcbRegs.ADCINTSEL1N2.bit.INT1E = 1;   //enable INT1 flag
    AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared
 //***************************************************************************************//



//************************************Ie/If ADCC3/C4************************************************//

    //*************Ie_ADCC3***************//
    AdccRegs.ADCSOC0CTL.bit.CHSEL = 3;  //SOC0 will convert pin C3 for Ie
    AdccRegs.ADCSOC0CTL.bit.ACQPS = acqps; //sample window is acqps + 1 SYSCLK cycles
    AdccRegs.ADCSOC0CTL.bit.TRIGSEL = 0xF;  //TRIGSEL = 0xFh ADCTRIG15 - ePWM6, ADCSOCA

    AdccRegs.ADCSOC2CTL.bit.CHSEL = 3;  //SOC2 will convert pin C3 for Ie
    AdccRegs.ADCSOC2CTL.bit.ACQPS = acqps; //sample window is acqps + 1 SYSCLK cycles
    AdccRegs.ADCSOC2CTL.bit.TRIGSEL = 0x10;  //TRIGSEL = 0x10h ADCTRIG16 - ePWM6, ADCSOCB

    AdccRegs.ADCSOC4CTL.bit.CHSEL = 3;  //SOC4 will convert pin C3 for Ie
    AdccRegs.ADCSOC4CTL.bit.ACQPS = acqps; //sample window is acqps + 1 SYSCLK cycles
    AdccRegs.ADCSOC4CTL.bit.TRIGSEL = 0x11;  //TRIGSEL = 0x11h ADCTRIG17 - ePWM7, ADCSOCA

    AdccRegs.ADCSOC6CTL.bit.CHSEL = 3;  //SOC6 will convert pin C3 for Ie
    AdccRegs.ADCSOC6CTL.bit.ACQPS = acqps; //sample window is acqps + 1 SYSCLK cycles
    AdccRegs.ADCSOC6CTL.bit.TRIGSEL = 0x12;  //TRIGSEL = 0x12h ADCTRIG18 - ePWM7, ADCSOCB

    AdccRegs.ADCSOC8CTL.bit.CHSEL = 3;  //SOC8 will convert pin C3 for Ie
    AdccRegs.ADCSOC8CTL.bit.ACQPS = acqps; //sample window is acqps + 1 SYSCLK cycles
    AdccRegs.ADCSOC8CTL.bit.TRIGSEL = 0x13;  //TRIGSEL = 0x13h ADCTRIG19 - ePWM8, ADCSOCA

    AdccRegs.ADCSOC10CTL.bit.CHSEL = 3;  //SOC10 will convert pin C3 for Ie
    AdccRegs.ADCSOC10CTL.bit.ACQPS = acqps; //sample window is acqps + 1 SYSCLK cycles
    AdccRegs.ADCSOC10CTL.bit.TRIGSEL = 0x14;  //TRIGSEL = 0x14h ADCTRIG20 - ePWM8, ADCSOCB
    //**********************************//

    //*************If_ADCC4***************//
    AdccRegs.ADCSOC1CTL.bit.CHSEL = 4;  //SOC1 will convert pin C4 for If
    AdccRegs.ADCSOC1CTL.bit.ACQPS = acqps; //sample window is acqps + 1 SYSCLK cycles
    AdccRegs.ADCSOC1CTL.bit.TRIGSEL = 0xF;  //TRIGSEL = 0x0Fh ADCTRIG15 - ePWM6, ADCSOCA

    AdccRegs.ADCSOC3CTL.bit.CHSEL = 4;  //SOC3 will convert pin C4 for If
    AdccRegs.ADCSOC3CTL.bit.ACQPS = acqps; //sample window is acqps + 1 SYSCLK cycles
    AdccRegs.ADCSOC3CTL.bit.TRIGSEL = 0x10;  //TRIGSEL = 0x10h ADCTRIG16 - ePWM6, ADCSOCB

    AdccRegs.ADCSOC5CTL.bit.CHSEL = 4;  //SOC5 will convert pin C4 for If
    AdccRegs.ADCSOC5CTL.bit.ACQPS = acqps; //sample window is acqps + 1 SYSCLK cycles
    AdccRegs.ADCSOC5CTL.bit.TRIGSEL = 0x11;  //TRIGSEL = 0x11h ADCTRIG17 - ePWM7, ADCSOCA

    AdccRegs.ADCSOC7CTL.bit.CHSEL = 4;  //SOC7 will convert pin C4 for If
    AdccRegs.ADCSOC7CTL.bit.ACQPS = acqps; //sample window is acqps + 1 SYSCLK cycles
    AdccRegs.ADCSOC7CTL.bit.TRIGSEL = 0x12;  //TRIGSEL = 0x12h ADCTRIG18 - ePWM7, ADCSOCB

    AdccRegs.ADCSOC9CTL.bit.CHSEL = 4;  //SOC9 will convert pin C4 for If
    AdccRegs.ADCSOC9CTL.bit.ACQPS = acqps; //sample window is acqps + 1 SYSCLK cycles
    AdccRegs.ADCSOC9CTL.bit.TRIGSEL = 0x13;  //TRIGSEL = 0x13h ADCTRIG20 - ePWM8, ADCSOCA

    AdccRegs.ADCSOC11CTL.bit.CHSEL = 4;  //SOC11 will convert pin C4 for If
    AdccRegs.ADCSOC11CTL.bit.ACQPS = acqps; //sample window is acqps + 1 SYSCLK cycles
    AdccRegs.ADCSOC11CTL.bit.TRIGSEL = 0x14;  //TRIGSEL = 0x14h ADCTRIG20 - ePWM8, ADCSOCB
    //**********************************//


    AdccRegs.ADCINTSEL1N2.bit.INT1SEL = 11; //end of SOC11 will set INT1 flag
    AdccRegs.ADCINTSEL1N2.bit.INT1E = 1;   //enable INT1 flag
    AdccRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared

//********************************VdcLink ADCD0**********************************//
    //***********VdcLink_ADCD0************//
    AdcdRegs.ADCSOC0CTL.bit.CHSEL = 0;  //SOC0 will convert pin D0 for VdcLink
    AdcdRegs.ADCSOC0CTL.bit.ACQPS = acqps; //sample window is acqps + 1 SYSCLK cycles
    AdcdRegs.ADCSOC0CTL.bit.TRIGSEL = 0xF;//ePWM6

    AdcdRegs.ADCSOC1CTL.bit.CHSEL = 0;  //SOC1 will convert pin D0 for PWM_enalbe
    AdcdRegs.ADCSOC1CTL.bit.ACQPS = acqps; //sample window is acqps + 1 SYSCLK cycles
    AdcdRegs.ADCSOC1CTL.bit.TRIGSEL = 0xF;//ePWM6
    //************************************//

    AdcdRegs.ADCINTSEL1N2.bit.INT1SEL = 1; //end of SOC1 will set INT1 flag
    AdcdRegs.ADCINTSEL1N2.bit.INT1E = 1;   //enable INT1 flag
    AdcdRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared
//*******************************************************************************//
    //---end of change
    EDIS;
}

void DAC_init()
{
	EALLOW;
		DacaRegs.DACCTL.bit.DACREFSEL = 1;//ADC VREFHI/VREFLO are the reference voltages
		DacaRegs.DACCTL.bit.LOADMODE = 0; //Load on next SYSCLK
		DacaRegs.DACVALS.bit.DACVALS=0;//maxmum
		//DacaRegs.DACOUTEN.bit.DACOUTEN=1; // enable DAC

		DacbRegs.DACCTL.bit.DACREFSEL = 1;//ADC VREFHI/VREFLO are the reference voltages
		DacbRegs.DACCTL.bit.LOADMODE = 0; //Load on next SYSCLK
		DacbRegs.DACVALS.bit.DACVALS=0;//maxmum

		DaccRegs.DACCTL.bit.DACREFSEL = 1;//ADC VREFHI/VREFLO are the reference voltages
		DaccRegs.DACCTL.bit.LOADMODE = 0; //Load on next SYSCLK
		DaccRegs.DACVALS.bit.DACVALS=0;//maxmum

		DacaRegs.DACOUTEN.bit.DACOUTEN=1; // enable DAC
		DacbRegs.DACOUTEN.bit.DACOUTEN=1; // enable DAC
		DaccRegs.DACOUTEN.bit.DACOUTEN=1; // enable DAC
	EDIS;
}


void InitGpio_ExternalInterrupt(void)
{
	// GPIO0 and GPIO1 are inputs
	EALLOW;
	   GpioCtrlRegs.GPCMUX1.bit.GPIO70 = 0;         // GPIO70
	   GpioCtrlRegs.GPCDIR.bit.GPIO70 = 0;          // input
	   GpioCtrlRegs.GPCQSEL1.bit.GPIO70 = 0;        // XINT1 Synch to SYSCLKOUT onlys
	   GpioCtrlRegs.GPCCTRL.bit.QUALPRD0 = 0xFF;    // Each sampling window is 510*SYSCLKOUT

	   GpioCtrlRegs.GPDMUX1.bit.GPIO99 = 0;         // GPIO99
	   GpioCtrlRegs.GPDDIR.bit.GPIO99 = 0;          // input
	   GpioCtrlRegs.GPDQSEL1.bit.GPIO99 = 0;        // XINTx Synch to SYSCLKOUT onlys
	   GpioCtrlRegs.GPDCTRL.bit.QUALPRD0 = 0xFF;    // Each sampling window is 510*SYSCLKOUT

	   GpioCtrlRegs.GPCMUX2.bit.GPIO82 = 0;         // GPIO82
	   GpioCtrlRegs.GPCDIR.bit.GPIO82 = 1;          // output

	   GpioCtrlRegs.GPCMUX2.bit.GPIO83 = 0;         // GPIO83
	   GpioCtrlRegs.GPCDIR.bit.GPIO83 = 1;          // output

	   GpioCtrlRegs.GPCMUX2.bit.GPIO85 = 0;         // GPIO84
	   GpioCtrlRegs.GPCDIR.bit.GPIO85 = 1;          // output

	   GpioCtrlRegs.GPCMUX2.bit.GPIO94 = 0;         // GPIO85
	   GpioCtrlRegs.GPCDIR.bit.GPIO94 = 1;          // output
	EDIS;
}

__interrupt void xint1_isr_synchronous_PWM(void)
{
	EALLOW;
		CpuSysRegs.PCLKCR0.bit.TBCLKSYNC =1;
	EDIS;
	++CUP1_EXINT_NUM;
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;// Acknowledge this interrupt to get more from group 1
}

__interrupt void xint2_isr_overCurrent_INT(void)
{
	Xint2Count_overCurrent++;
	Enable_Force_PWM_allout_low();
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;// Acknowledge this interrupt to get more from group 1
}


Uint32 ADCa_int_num = 0;
Uint32 ADCb_int_num = 0;
Uint32 ADCc_int_num = 0;
Uint32 ADCd_int_num = 0;

__interrupt void adca1_isr(void)
{
	AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear INT1 flag
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
	GPIO_WritePin(82,1);
	AdcaResult_Ia[0] = AdcaResultRegs.ADCRESULT0;
	AdcaResult_Ia[1] = AdcaResultRegs.ADCRESULT2;
	AdcaResult_Ia[2] = AdcaResultRegs.ADCRESULT4;
	AdcaResult_Ia[3] = AdcaResultRegs.ADCRESULT6;
	AdcaResult_Ia[4] = AdcaResultRegs.ADCRESULT8;
	AdcaResult_Ia[5] = AdcaResultRegs.ADCRESULT10;

	AdcaResult_Ib[0] = AdcaResultRegs.ADCRESULT1;
	AdcaResult_Ib[1] = AdcaResultRegs.ADCRESULT3;
	AdcaResult_Ib[2] = AdcaResultRegs.ADCRESULT5;
	AdcaResult_Ib[3] = AdcaResultRegs.ADCRESULT7;
	AdcaResult_Ib[4] = AdcaResultRegs.ADCRESULT9;
	AdcaResult_Ib[5] = AdcaResultRegs.ADCRESULT11;
	GPIO_WritePin(82,0);
}

__interrupt void adcd1_isr(void)
{
	AdcdRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear INT1 flag
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;


	AdcdResult2_VdcLink=AdcdResultRegs.ADCRESULT0;

}


__interrupt void adcb1_isr(void)
{
	AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear INT1 flag
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

	AdcbResult_Id[0]=AdcbResultRegs.ADCRESULT0;
	AdcbResult_Id[1]=AdcbResultRegs.ADCRESULT2;
	AdcbResult_Id[2]=AdcbResultRegs.ADCRESULT4;
	AdcbResult_Id[3]=AdcbResultRegs.ADCRESULT6;
	AdcbResult_Id[4]=AdcbResultRegs.ADCRESULT8;
	AdcbResult_Id[5]=AdcbResultRegs.ADCRESULT10;

	AdcbResult_Ic[0]=AdcbResultRegs.ADCRESULT1;
	AdcbResult_Ic[1]=AdcbResultRegs.ADCRESULT3;
	AdcbResult_Ic[2]=AdcbResultRegs.ADCRESULT5;
	AdcbResult_Ic[3]=AdcbResultRegs.ADCRESULT7;
	AdcbResult_Ic[4]=AdcbResultRegs.ADCRESULT9;
	AdcbResult_Ic[5]=AdcbResultRegs.ADCRESULT11;
}

__interrupt void adcc1_isr(void)
{
	AdccRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear INT1 flag
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

	AdccResult_If[0]=AdccResultRegs.ADCRESULT0;
	AdccResult_If[1]=AdccResultRegs.ADCRESULT2;
	AdccResult_If[2]=AdccResultRegs.ADCRESULT4;
	AdccResult_If[3]=AdccResultRegs.ADCRESULT6;
	AdccResult_If[4]=AdccResultRegs.ADCRESULT8;
	AdccResult_If[5]=AdccResultRegs.ADCRESULT10;

	AdccResult_Ie[0]=AdccResultRegs.ADCRESULT1;
	AdccResult_Ie[1]=AdccResultRegs.ADCRESULT3;
	AdccResult_Ie[2]=AdccResultRegs.ADCRESULT5;
	AdccResult_Ie[3]=AdccResultRegs.ADCRESULT7;
	AdccResult_Ie[4]=AdccResultRegs.ADCRESULT9;
	AdccResult_Ie[5]=AdccResultRegs.ADCRESULT11;
}
__interrupt void adcc2_isr(void)
{
	AdccRegs.ADCINTFLGCLR.bit.ADCINT2 = 1; //clear INT1 flag
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP10;

}


void Deadtime_Compensation(float Ia,float Ib,float Ic,float Id,float Ie,float If,float Td)
{
    double temp;
    temp=VdcLink*Td/T;

    	if (Ia>0) compensation_A=temp;
    	else compensation_A=-temp;

    	if (Ib>0) compensation_B=temp;
    	else compensation_B=-temp;

    	if (Ic>0) compensation_C=temp;
    	else compensation_C=-temp;

    	if (Id>0) compensation_D=temp;
    	else compensation_D=-temp;

    	if (Ie>0) compensation_E=temp;
    	else compensation_E=-temp;

    	if (If>0) compensation_F=temp;
    	else compensation_F=-temp;


}

void matrix_inversion(float A[4][4])
{
	float determinant,adjoint[4][4];
	int i,j;
	determinant=A[0][0]*(A[1][1]*A[2][2]*A[3][3] - A[1][1]*A[2][3]*A[3][2] - A[1][2]*A[2][1]*A[3][3] + A[1][2]*A[2][3]*A[3][1] + A[1][3]*A[2][1]*A[3][2] - A[1][3]*A[2][2]*A[3][1])
			   -A[0][1]*(A[1][0]*A[2][2]*A[3][3] - A[1][0]*A[2][3]*A[3][2] - A[1][2]*A[2][0]*A[3][3] + A[1][2]*A[2][3]*A[3][0] + A[1][3]*A[2][0]*A[3][2] - A[1][3]*A[2][2]*A[3][0])
			   +A[0][2]*(A[1][0]*A[2][1]*A[3][3] - A[1][0]*A[2][3]*A[3][1] - A[1][1]*A[2][0]*A[3][3] + A[1][1]*A[2][3]*A[3][0] + A[1][3]*A[2][0]*A[3][1] - A[1][3]*A[2][1]*A[3][0])
			   -A[0][3]*(A[1][0]*A[2][1]*A[3][2] - A[1][0]*A[2][2]*A[3][1] - A[1][1]*A[2][0]*A[3][2] + A[1][1]*A[2][2]*A[3][0] + A[1][2]*A[2][0]*A[3][1] - A[1][2]*A[2][1]*A[3][0]);

	adjoint[0][0]=  A[1][1]*A[2][2]*A[3][3] - A[1][1]*A[2][3]*A[3][2] - A[1][2]*A[2][1]*A[3][3] + A[1][2]*A[2][3]*A[3][1] + A[1][3]*A[2][1]*A[3][2] - A[1][3]*A[2][2]*A[3][1];
	adjoint[0][1]=-(A[0][1]*A[2][2]*A[3][3] - A[0][1]*A[2][3]*A[3][2] - A[0][2]*A[2][1]*A[3][3] + A[0][2]*A[2][3]*A[3][1] + A[0][3]*A[2][1]*A[3][2] - A[0][3]*A[2][2]*A[3][1]);
	adjoint[0][2]=  A[0][1]*A[1][2]*A[3][3] - A[0][1]*A[1][3]*A[3][2] - A[0][2]*A[1][1]*A[3][3] + A[0][2]*A[1][3]*A[3][1] + A[0][3]*A[1][1]*A[3][2] - A[0][3]*A[1][2]*A[3][1];
	adjoint[0][3]=-(A[0][1]*A[1][2]*A[2][3] - A[0][1]*A[1][3]*A[2][2] - A[0][2]*A[1][1]*A[2][3] + A[0][2]*A[1][3]*A[2][1] + A[0][3]*A[1][1]*A[2][2] - A[0][3]*A[1][2]*A[2][1]);
	adjoint[1][0]=-(A[1][0]*A[2][2]*A[3][3] - A[1][0]*A[2][3]*A[3][2] - A[1][2]*A[2][0]*A[3][3] + A[1][2]*A[2][3]*A[3][0] + A[1][3]*A[2][0]*A[3][2] - A[1][3]*A[2][2]*A[3][0]);
	adjoint[1][1]=  A[0][0]*A[2][2]*A[3][3] - A[0][0]*A[2][3]*A[3][2] - A[0][2]*A[2][0]*A[3][3] + A[0][2]*A[2][3]*A[3][0] + A[0][3]*A[2][0]*A[3][2] - A[0][3]*A[2][2]*A[3][0];
	adjoint[1][2]=-(A[0][0]*A[1][2]*A[3][3] - A[0][0]*A[1][3]*A[3][2] - A[0][2]*A[1][0]*A[3][3] + A[0][2]*A[1][3]*A[3][0] + A[0][3]*A[1][0]*A[3][2] - A[0][3]*A[1][2]*A[3][0]);
	adjoint[1][3]=  A[0][0]*A[1][2]*A[2][3] - A[0][0]*A[1][3]*A[2][2] - A[0][2]*A[1][0]*A[2][3] + A[0][2]*A[1][3]*A[2][0] + A[0][3]*A[1][0]*A[2][2] - A[0][3]*A[1][2]*A[2][0];
	adjoint[2][0]=  A[1][0]*A[2][1]*A[3][3] - A[1][0]*A[2][3]*A[3][1] - A[1][1]*A[2][0]*A[3][3] + A[1][1]*A[2][3]*A[3][0] + A[1][3]*A[2][0]*A[3][1] - A[1][3]*A[2][1]*A[3][0];
    adjoint[2][1]=-(A[0][0]*A[2][1]*A[3][3] - A[0][0]*A[2][3]*A[3][1] - A[0][1]*A[2][0]*A[3][3] + A[0][1]*A[2][3]*A[3][0] + A[0][3]*A[2][0]*A[3][1] - A[0][3]*A[2][1]*A[3][0]);//wrong
	adjoint[2][2]=  A[0][0]*A[1][1]*A[3][3] - A[0][0]*A[1][3]*A[3][1] - A[0][1]*A[1][0]*A[3][3] + A[0][1]*A[1][3]*A[3][0] + A[0][3]*A[1][0]*A[3][1] - A[0][3]*A[1][1]*A[3][0];
	adjoint[2][3]=-(A[0][0]*A[1][1]*A[2][3] - A[0][0]*A[1][3]*A[2][1] - A[0][1]*A[1][0]*A[2][3] + A[0][1]*A[1][3]*A[2][0] + A[0][3]*A[1][0]*A[2][1] - A[0][3]*A[1][1]*A[2][0]);
	adjoint[3][0]=-(A[1][0]*A[2][1]*A[3][2] - A[1][0]*A[2][2]*A[3][1] - A[1][1]*A[2][0]*A[3][2] + A[1][1]*A[2][2]*A[3][0] + A[1][2]*A[2][0]*A[3][1] - A[1][2]*A[2][1]*A[3][0]);
	adjoint[3][1]=  A[0][0]*A[2][1]*A[3][2] - A[0][0]*A[2][2]*A[3][1] - A[0][1]*A[2][0]*A[3][2] + A[0][1]*A[2][2]*A[3][0] + A[0][2]*A[2][0]*A[3][1] - A[0][2]*A[2][1]*A[3][0];
	adjoint[3][2]=-(A[0][0]*A[1][1]*A[3][2] - A[0][0]*A[1][2]*A[3][1] - A[0][1]*A[1][0]*A[3][2] + A[0][1]*A[1][2]*A[3][0] + A[0][2]*A[1][0]*A[3][1] - A[0][2]*A[1][1]*A[3][0]);
	adjoint[3][3]=  A[0][0]*A[1][1]*A[2][2] - A[0][0]*A[1][2]*A[2][1] - A[0][1]*A[1][0]*A[2][2] + A[0][1]*A[1][2]*A[2][0] + A[0][2]*A[1][0]*A[2][1] - A[0][2]*A[1][1]*A[2][0];
	for(i=0;i<4;i++)
	{
		for(j=0;j<4;j++)
		{
			vector_inversion_matrix[i][j]=adjoint[i][j]/determinant;
		}
	}
}

void matrix_multiply(float A[4][4],float B[4])
{
	int i;
	for(i=0;i<4;i++)
	{
		time_matrix[i]=A[i][0]*B[0]+A[i][1]*B[1]+A[i][2]*B[2]+A[i][3]*B[3];
		if(time_matrix[i]<0){time_matrix[i]=0;}
	}
}

void vectorI_select(Uint16 Sector_temp)
{
	int i,j;

	switch(Sector_temp){

	case 1:
		for(i=0;i<4;i++)
		{
			for(j=0;j<4;j++)
					vector_inversion_matrix[i][j]=S1[i][j];
		}
	    break;
	case 2:
		for(i=0;i<4;i++)
		{
			for(j=0;j<4;j++)
					vector_inversion_matrix[i][j]=S2[i][j];
		}
	    break;
	case 3:
		for(i=0;i<4;i++)
		{
			for(j=0;j<4;j++)
					vector_inversion_matrix[i][j]=S3[i][j];
		}
	    break;
	case 4:
		for(i=0;i<4;i++)
		{
			for(j=0;j<4;j++)
					vector_inversion_matrix[i][j]=S4[i][j];
		}
	    break;
	case 5:
		for(i=0;i<4;i++)
		{
			for(j=0;j<4;j++)
					vector_inversion_matrix[i][j]=S5[i][j];
		}
	    break;
	case 6:
		for(i=0;i<4;i++)
		{
			for(j=0;j<4;j++)
					vector_inversion_matrix[i][j]=S6[i][j];
		}
	    break;
	case 7:
		for(i=0;i<4;i++)
		{
			for(j=0;j<4;j++)
					vector_inversion_matrix[i][j]=S7[i][j];
		}
	    break;
	case 8:
		for(i=0;i<4;i++)
		{
			for(j=0;j<4;j++)
					vector_inversion_matrix[i][j]=S8[i][j];
		}
	    break;
	case 9:
		for(i=0;i<4;i++)
		{
			for(j=0;j<4;j++)
					vector_inversion_matrix[i][j]=S9[i][j];
		}
	    break;
	case 10:
		for(i=0;i<4;i++)
		{
			for(j=0;j<4;j++)
					vector_inversion_matrix[i][j]=S10[i][j];
		}
	    break;
	case 11:
		for(i=0;i<4;i++)
		{
			for(j=0;j<4;j++)
					vector_inversion_matrix[i][j]=S11[i][j];
		}
	    break;
	case 12:
		for(i=0;i<4;i++)
		{
			for(j=0;j<4;j++)
					vector_inversion_matrix[i][j]=S12[i][j];
		}
	    break;
	default:
		for(i=0;i<4;i++)
		{
			for(j=0;j<4;j++)
					vector_inversion_matrix[i][j]=0;
		}
	    break;
	}
}
void vector_select(Uint16 Sector_old_temp)
{
	int i,j,m,n;

	for(i=0;i<4;i++) {vector_original_matrix[0][i]=VdcLink*AB[Sector_old_temp-1+i].real;}

	for(j=0;j<4;j++) {vector_original_matrix[1][j]=VdcLink*AB[Sector_old_temp-1+j].imagine;}

	for(m=0;m<4;m++) {vector_original_matrix[2][m]=VdcLink*XY[Sector_old_temp-1+m].real;}

	for(n=0;n<4;n++) {vector_original_matrix[3][n]=VdcLink*XY[Sector_old_temp-1+n].imagine;}
}

void double_park(double A,double B,double C,double D,double E,double F,double phase)
{
	Id1_temp = 0.66666667*(A*cos(phase)+C*cos(phase-2.094395067)+E*cos(phase+2.094395067));
	Iq1_temp =-0.66666667*(A*sin(phase)+C*sin(phase-2.094395067)+E*sin(phase+2.094395067));
	Id2_temp = 0.66666667*(B*cos(phase)+D*cos(phase-2.094395067)+F*cos(phase+2.094395067));
	Iq2_temp =-0.66666667*(B*sin(phase)+D*sin(phase-2.094395067)+F*sin(phase+2.094395067));
}
void double_anti_park(double Da,double Qa,double Db,double Qb,double phase)
{
    Ia_filter=Da*cos(phase)-Qa*sin(phase);
    Ic_filter=Da*cos(phase-2.094395067)-Qa*sin(phase-2.094395067);
    Ie_filter=Da*cos(phase+2.094395067)-Qa*sin(phase+2.094395067);
    Ib_filter=Db*cos(phase)-Qb*sin(phase);
    Id_filter=Db*cos(phase-2.094395067)-Qb*sin(phase-2.094395067);
    If_filter=Db*cos(phase+2.094395067)-Qb*sin(phase+2.094395067);
}
int show_current_by_DAC(double current)
{
	//register=f(u)=1256.0913*u-210.0064797
	//当输出1.65V时，计算可得DAC寄存器值应为1863.319（实际中寄存器只能赋值正整数）
	//此函数当电流为0A时，DAC输出1.65V，输出电压变化与输入电流变化等比例，用于矫正电流采样
	double DAC_value_temp;
	int DAC_value;
	DAC_value_temp=1256.560913*current+1863.319;
	DAC_value=(Uint16)DAC_value_temp;
	return DAC_value;
}

void Epwm_CMPAB_assignment(void)
{
	switch (Sector)
	{
		case 1:
			CompareA_EPWM12_PhaseA_temp = tx1;  //PhaseA
			CompareB_EPWM12_PhaseA_temp = tx6;

			CompareA_EPWM05_PhaseB_temp = tx1;  //PhaseB
			CompareB_EPWM05_PhaseB_temp = tx6;

			CompareA_EPWM11_PhaseC_temp = tx3;  //PhaseC
			CompareB_EPWM11_PhaseC_temp = tx5+t1_compensation+t2_compensation;

			CompareA_EPWM01_PhaseD_temp = tx3;  //PhaseD
			CompareB_EPWM01_PhaseD_temp = tx4+t1_compensation+t2_compensation;

			CompareA_EPWM02_PhaseE_temp = tx2;  //PhaseE
			CompareB_EPWM02_PhaseE_temp = tx4+t2_compensation;

			CompareA_EPWM03_PhaseF_temp = tx1;  //PhaseF
			CompareB_EPWM03_PhaseF_temp = tx4;

			break;

		case 2:
		    CompareA_EPWM12_PhaseA_temp = tx1;  //PhaseA
	        CompareB_EPWM12_PhaseA_temp = tx6;

			CompareA_EPWM05_PhaseB_temp = tx1;  //PhaseB
			CompareB_EPWM05_PhaseB_temp = tx6;

			CompareA_EPWM11_PhaseC_temp = tx3;  //PhaseC
			CompareB_EPWM11_PhaseC_temp = tx6+t1_compensation+t2_compensation;

			CompareA_EPWM01_PhaseD_temp = tx3;  //PhaseD
			CompareB_EPWM01_PhaseD_temp = tx5+t1_compensation+t2_compensation;

			CompareA_EPWM02_PhaseE_temp = tx3;  //PhaseE
			CompareB_EPWM02_PhaseE_temp = tx4+t1_compensation+t2_compensation;

			CompareA_EPWM03_PhaseF_temp = tx2;  //PhaseF
			CompareB_EPWM03_PhaseF_temp = tx4+t2_compensation;

			break;

		case 3:
			CompareA_EPWM12_PhaseA_temp = tx1;  //PhaseA
			CompareB_EPWM12_PhaseA_temp = tx5;

			CompareA_EPWM05_PhaseB_temp = tx1;  //PhaseB
			CompareB_EPWM05_PhaseB_temp = tx6;

			CompareA_EPWM11_PhaseC_temp = tx2;  //PhaseC
			CompareB_EPWM11_PhaseC_temp = tx6+t1_compensation;

			CompareA_EPWM01_PhaseD_temp = tx3;  //PhaseD
			CompareB_EPWM01_PhaseD_temp = tx6+t1_compensation+t2_compensation;

			CompareA_EPWM02_PhaseE_temp = tx3;  //PhaseE
			CompareB_EPWM02_PhaseE_temp = tx4+t1_compensation+t2_compensation;

			CompareA_EPWM03_PhaseF_temp = tx3;  //PhaseF
			CompareB_EPWM03_PhaseF_temp = tx4+t1_compensation+t2_compensation;

			break;

		case 4:
			CompareA_EPWM12_PhaseA_temp = tx1;  //PhaseA
			CompareB_EPWM12_PhaseA_temp = tx4;

			CompareA_EPWM05_PhaseB_temp = tx1;  //PhaseB
			CompareB_EPWM05_PhaseB_temp = tx5;

	    	CompareA_EPWM11_PhaseC_temp = tx1;  //PhaseC
			CompareB_EPWM11_PhaseC_temp = tx6;

			CompareA_EPWM01_PhaseD_temp = tx2;  //PhaseD
			CompareB_EPWM01_PhaseD_temp = tx6+t1_compensation;

			CompareA_EPWM02_PhaseE_temp = tx3;  //PhaseE
			CompareB_EPWM02_PhaseE_temp = tx4+t1_compensation+t2_compensation;

			CompareA_EPWM03_PhaseF_temp = tx3;  //PhaseF
			CompareB_EPWM03_PhaseF_temp = tx4+t1_compensation+t2_compensation;

			break;

		case 5:
			CompareA_EPWM12_PhaseA_temp = tx2;  //PhaseA
			CompareB_EPWM12_PhaseA_temp = tx4+t2_compensation;

			CompareA_EPWM05_PhaseB_temp = tx1;  //PhaseB
			CompareB_EPWM05_PhaseB_temp = tx4;

			CompareA_EPWM11_PhaseC_temp = tx1;  //PhaseC
			CompareB_EPWM11_PhaseC_temp = tx6;

	     	CompareA_EPWM01_PhaseD_temp = tx1;  //PhaseD
			CompareB_EPWM01_PhaseD_temp = tx6;

			CompareA_EPWM02_PhaseE_temp = tx3;  //PhaseE
	    	CompareB_EPWM02_PhaseE_temp = tx5+t1_compensation+t2_compensation;

			CompareA_EPWM03_PhaseF_temp = tx3;  //PhaseF
			CompareB_EPWM03_PhaseF_temp = tx4+t1_compensation+t2_compensation;

			break;

		case 6:
			CompareA_EPWM12_PhaseA_temp = tx3;  //PhaseA
			CompareB_EPWM12_PhaseA_temp = tx4+t1_compensation+t2_compensation;

			CompareA_EPWM05_PhaseB_temp = tx2;  //PhaseB
			CompareB_EPWM05_PhaseB_temp = tx4+t2_compensation;

			CompareA_EPWM11_PhaseC_temp = tx1;  //PhaseC
			CompareB_EPWM11_PhaseC_temp = tx6;

			CompareA_EPWM01_PhaseD_temp = tx1;  //PhaseD
			CompareB_EPWM01_PhaseD_temp = tx6;

			CompareA_EPWM02_PhaseE_temp = tx3;  //PhaseE
			CompareB_EPWM02_PhaseE_temp = tx6+t1_compensation+t2_compensation;

			CompareA_EPWM03_PhaseF_temp = tx3;  //PhaseF
			CompareB_EPWM03_PhaseF_temp = tx5+t1_compensation+t2_compensation;

			break;

		case 7:
			CompareA_EPWM12_PhaseA_temp = tx3;  //PhaseA
			CompareB_EPWM12_PhaseA_temp = tx4+t1_compensation+t2_compensation;

			CompareA_EPWM05_PhaseB_temp = tx3;  //PhaseB
			CompareB_EPWM05_PhaseB_temp = tx4+t1_compensation+t2_compensation;

			CompareA_EPWM11_PhaseC_temp = tx1;  //PhaseC
			CompareB_EPWM11_PhaseC_temp = tx5;

			CompareA_EPWM01_PhaseD_temp = tx1;  //PhaseD
			CompareB_EPWM01_PhaseD_temp = tx6;

			CompareA_EPWM02_PhaseE_temp = tx2;  //PhaseE
			CompareB_EPWM02_PhaseE_temp = tx6+t1_compensation;

			CompareA_EPWM03_PhaseF_temp = tx3;  //PhaseF
			CompareB_EPWM03_PhaseF_temp = tx6+t1_compensation+t2_compensation;
			break;

		case 8:
			CompareA_EPWM12_PhaseA_temp = tx3;  //PhaseA
			CompareB_EPWM12_PhaseA_temp = tx4+t1_compensation+t2_compensation;

			CompareA_EPWM05_PhaseB_temp = tx3;  //PhaseB
			CompareB_EPWM05_PhaseB_temp = tx4+t1_compensation+t2_compensation;

			CompareA_EPWM11_PhaseC_temp = tx1;  //PhaseC
			CompareB_EPWM11_PhaseC_temp = tx4;

			CompareA_EPWM01_PhaseD_temp = tx1;  //PhaseD
			CompareB_EPWM01_PhaseD_temp = tx5;

			CompareA_EPWM02_PhaseE_temp = tx1;  //PhaseE
			CompareB_EPWM02_PhaseE_temp = tx6;

			CompareA_EPWM03_PhaseF_temp = tx2;  //PhaseF
			CompareB_EPWM03_PhaseF_temp = tx6+t1_compensation;

			break;

		case 9:
			CompareA_EPWM12_PhaseA_temp = tx3;  //PhaseA
			CompareB_EPWM12_PhaseA_temp = tx5+t1_compensation+t2_compensation;

			CompareA_EPWM05_PhaseB_temp = tx3;  //PhaseB
			CompareB_EPWM05_PhaseB_temp = tx4+t1_compensation+t2_compensation;

			CompareA_EPWM11_PhaseC_temp = tx2;  //PhaseC
		    CompareB_EPWM11_PhaseC_temp = tx4+t2_compensation;

			CompareA_EPWM01_PhaseD_temp = tx1;  //PhaseD
			CompareB_EPWM01_PhaseD_temp = tx4;

			CompareA_EPWM02_PhaseE_temp = tx1;  //PhaseE
			CompareB_EPWM02_PhaseE_temp = tx6;

			CompareA_EPWM03_PhaseF_temp = tx1;  //PhaseF
			CompareB_EPWM03_PhaseF_temp = tx6;

			break;

		case 10:
			CompareA_EPWM12_PhaseA_temp = tx3;  //PhaseA
			CompareB_EPWM12_PhaseA_temp = tx6+t1_compensation+t2_compensation;

			CompareA_EPWM05_PhaseB_temp = tx3;  //PhaseB
			CompareB_EPWM05_PhaseB_temp = tx5+t1_compensation+t2_compensation;

			CompareA_EPWM11_PhaseC_temp = tx3;  //PhaseC
			CompareB_EPWM11_PhaseC_temp = tx4+t1_compensation+t2_compensation;

			CompareA_EPWM01_PhaseD_temp = tx2;  //PhaseD
			CompareB_EPWM01_PhaseD_temp = tx4+t2_compensation;

			CompareA_EPWM02_PhaseE_temp = tx1;  //PhaseE
			CompareB_EPWM02_PhaseE_temp = tx6;

			CompareA_EPWM03_PhaseF_temp = tx1;  //PhaseF
			CompareB_EPWM03_PhaseF_temp = tx6;

			break;

		case 11:
			CompareA_EPWM12_PhaseA_temp = tx2;  //PhaseA
			CompareB_EPWM12_PhaseA_temp = tx6+t1_compensation;

			CompareA_EPWM05_PhaseB_temp = tx3;  //PhaseB
			CompareB_EPWM05_PhaseB_temp = tx6+t1_compensation+t2_compensation;

			CompareA_EPWM11_PhaseC_temp = tx3;  //PhaseC
			CompareB_EPWM11_PhaseC_temp = tx4+t1_compensation+t2_compensation;

			CompareA_EPWM01_PhaseD_temp = tx3;  //PhaseD
			CompareB_EPWM01_PhaseD_temp = tx4+t1_compensation+t2_compensation;

			CompareA_EPWM02_PhaseE_temp = tx1;  //PhaseE
			CompareB_EPWM02_PhaseE_temp = tx5;

			CompareA_EPWM03_PhaseF_temp = tx1;  //PhaseF
			CompareB_EPWM03_PhaseF_temp = tx6;

			break;

		case 12:

			CompareA_EPWM12_PhaseA_temp = tx1;  //PhaseA
			CompareB_EPWM12_PhaseA_temp = tx6;

			CompareA_EPWM05_PhaseB_temp = tx2;  //PhaseB
			CompareB_EPWM05_PhaseB_temp = tx6+t1_compensation;

			CompareA_EPWM11_PhaseC_temp = tx3;  //PhaseC
		    CompareB_EPWM11_PhaseC_temp = tx4+t1_compensation+t2_compensation;

			CompareA_EPWM01_PhaseD_temp = tx3;  //PhaseD
	    	CompareB_EPWM01_PhaseD_temp = tx4+t1_compensation+t2_compensation;

			CompareA_EPWM02_PhaseE_temp = tx1;  //PhaseE
			CompareB_EPWM02_PhaseE_temp = tx4;

			CompareA_EPWM03_PhaseF_temp = tx1;  //PhaseF
			CompareB_EPWM03_PhaseF_temp = tx5;

			break;
	}
	CompareB_EPWM12_PhaseA_temp=limit_CMPB(CompareB_EPWM12_PhaseA_temp);
	CompareB_EPWM05_PhaseB_temp=limit_CMPB(CompareB_EPWM05_PhaseB_temp);
	CompareB_EPWM11_PhaseC_temp=limit_CMPB(CompareB_EPWM11_PhaseC_temp);
	CompareB_EPWM01_PhaseD_temp=limit_CMPB(CompareB_EPWM01_PhaseD_temp);
	CompareB_EPWM02_PhaseE_temp=limit_CMPB(CompareB_EPWM02_PhaseE_temp);
	CompareB_EPWM03_PhaseF_temp=limit_CMPB(CompareB_EPWM03_PhaseF_temp);
}

int limit_CMPB(int CMPB)
{
	if (CMPB>2500) {CMPB=2500;}
	return CMPB;
}
double w, den, b0_num, a1_num, a2_num;

PRFILTER setPR(PRFILTER pr,float speed,float wc,float kr)
{
	PRFILTER pr_new;


	w=speed*pi;
	den = w*w*T*T + 4*wc*T + 4;    //(w0*T)^2+4*wc*T+4
	b0_num = 4*wc*T*kr;                 //4*wc*T*kr
	a1_num = 2*w*w*T*T-8;               //2*(w0*T)^2
	a2_num = w*w*T*T - 4*wc*T + 4; //(w0*T)^2-4*wc*T+4

	pr_new.gain = b0_num/den;
	pr_new.numerator1 = 0;
	pr_new.numerator2 = -1;
	pr_new.denominator1 = a1_num/den;
	pr_new.denominator2 = a2_num/den;

	pr_new.xn=pr.xn;
	pr_new.xn1=pr.xn1;
	pr_new.xn2=pr.xn2;
	pr_new.yn=pr.yn;
	pr_new.yn1=pr.yn1;
	pr_new.yn2=pr.yn2;

    return pr_new;
}

double pIphase_filter(double pIphase,double pIphase_temp)
{
	double tempa;
	tempa=pIphase-pIphase_temp;
	if(Sector==Sector_temp)
	{
		if(tempa>0.2) {pIphase=pIphase_temp+0.1;}
		if(tempa<-0.2){pIphase=pIphase_temp-0.1;}
	}
    return pIphase;
}
