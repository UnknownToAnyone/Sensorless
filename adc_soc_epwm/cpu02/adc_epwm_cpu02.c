//###########################################################################
// FILE:   adc_epwm_cpu02.c
// TITLE:  Example to demonstrate ADCA and EPWM1 on CPU2
//
// This example demonstrates how to make use of the ADC and EPWM peripherals
// from CPU2.  Device clocking (PLL) and GPIO setup are done using CPU1,
// while all other configuration of the peripherals is done using CPU2.
//
// CPU2 configures EPWM1 in up count mode in a similar fashion to what is
// done in the epwm_up_aq example.  The ADC is configured in continuous
// conversion mode similar to the adc_soc_continuous example.  GPIO0 can be
// connected to ADCINA0 and the results buffer AdcaResults graphed in CCS to
// view the duty cycle of the generated waveform.
//
//###########################################################################
// $TI Release: F2837xD Support Library v150 $
// $Release Date: Thu Mar  5 14:18:39 CST 2015 $
// $Copyright: Copyright (C) 2013-2015 Texas Instruments Incorporated -
//             http://www.ti.com/ ALL RIGHTS RESERVED $
//###########################################################################

#include "F28x_Project.h"     // Device Headerfile and Examples Include File
#include "stdio.h"
#include "F2837xD_input_xbar.h"

//###########################################################################//
//####################_EPWMs_var_Definitions_functions_######################//
#define EPWM_period 2500    // Period register
#define EPWMx_MIN_DB   50
#define EPWM_CMP_UP   1
#define EPWM_CMP_DOWN 0

Uint16 EPWM1_INT_Index=0;
Uint16 EPWM2_INT_Index=0;
Uint16 EPWM3_INT_Index=0;
Uint16 EPWM5_INT_Index=0;
Uint16 EPWM11_INT_Index=0;
Uint16 EPWM12_INT_Index=0;
Uint16 NVC_start_flag=0;
Uint16 t1_negtive_vector_flag=0;
Uint16 t2_negtive_vector_flag=0;
int tx6=2500;
int show_int=0;

Uint16 CompareA_EPWM01_PhaseD=0;
Uint16 CompareA_EPWM02_PhaseE=0;
Uint16 CompareA_EPWM03_PhaseF=0;
Uint16 CompareA_EPWM05_PhaseB=0;
Uint16 CompareA_EPWM11_PhaseC=0;
Uint16 CompareA_EPWM12_PhaseA=0;
Uint16 CompareB_EPWM01_PhaseD=0;
Uint16 CompareB_EPWM02_PhaseE=0;
Uint16 CompareB_EPWM03_PhaseF=0;
Uint16 CompareB_EPWM05_PhaseB=0;
Uint16 CompareB_EPWM11_PhaseC=0;
Uint16 CompareB_EPWM12_PhaseA=0;

Uint16 EPWM1_CMPA = 1000;
Uint16 EPWM1_CMPB = 2000;

Uint16 Synchronous_done_flag = 1;
Uint16 EPWM_Eable_Flag=0;
void InitEPWM_SixPhase(void);
void Enable_Force_PWM_allout_low(void);
void Disenable_Force_PWM_allout_low(void);
__interrupt void epwm1_isr(void);
__interrupt void epwm2_isr(void);
__interrupt void epwm3_isr(void);
__interrupt void epwm4_isr(void);
__interrupt void epwm5_isr(void);
__interrupt void epwm11_isr(void);
__interrupt void epwm12_isr(void);
//###################################################################//

Uint16 Xint2Count;

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

Uint16 EPWM_Enable_Flag = 0;

Uint16 CUP2_EXINT_NUM = 0;
Uint16 SynchronisePWMs_DONE_flag = 0;
Uint16 SynchronisePWMs_start_flag = 0;

Uint32 CPU1TOCPU2RAM_addr = 0x00003FC00;
Uint32 CPU2TOCPU1RAM_addr = 0x00003F800;
Uint16 *CPU1TOCPU2RAM_addr_pt =(Uint16 *) 0x00003FC00;
Uint16 *CPU2TOCPU1RAM_addr_pt =(Uint16 *) 0x00003F800;

__interrupt void xint1_isr_synchronous_PWM(void);
__interrupt void xint2_isr_overCurrent_INT(void);
__interrupt void CPU01toCPU02IPC0IntHandler(void);
void Set_EPWMs_compAB_To_Default(void);
void Copy_EPWMs_compAB_From_CPU01TOCPU02RAM(void);
//###################################################################//

void main(void)
{
	                                                                                                                                                                                                                                                                                                                                  	InitSysCtrl(); //Initialize System Control: PLL, WatchDog, enable Peripheral Clocks

    DINT;

    	InitPieCtrl(); // Initialize the PIE control registers to their default state.

    IER = 0x0000;  // Disable CPU interrupts and clear all CPU interrupt flags:
    IFR = 0x0000;

    	InitPieVectTable();

    EALLOW; // Register the EPWMs and other peripherals interrupt handler
    	PieVectTable.XINT1_INT  = &xint1_isr_synchronous_PWM; //External_Interrupt_for_EpwmSynch,
    	PieVectTable.XINT2_INT  = &xint2_isr_overCurrent_INT; //External_Interrupt_for_overCurrent,

		PieVectTable.IPC0_INT   = &CPU01toCPU02IPC0IntHandler;   //

//		PieVectTable.EPWM1_INT  = &epwm1_isr;  //function for ePWM interrupt 1
//		PieVectTable.EPWM2_INT  = &epwm2_isr;  //function for ePWM interrupt 2
//    	PieVectTable.EPWM3_INT  = &epwm3_isr;  //function for ePWM interrupt 3
    	PieVectTable.EPWM4_INT  = &epwm4_isr;  //function for ePWM interrupt 3
//    	PieVectTable.EPWM5_INT  = &epwm5_isr;  //function for ePWM interrupt 5
//		PieVectTable.EPWM11_INT = &epwm11_isr; //function for ePWM interrupt 11
//		PieVectTable.EPWM12_INT = &epwm12_isr; //function for ePWM interrupt 12
	EDIS;

	EALLOW; //Power up the PWMs, again
		CpuSysRegs.PCLKCR2.bit.EPWM1  = 1;
		CpuSysRegs.PCLKCR2.bit.EPWM2  = 1;
		CpuSysRegs.PCLKCR2.bit.EPWM3  = 1;
		CpuSysRegs.PCLKCR2.bit.EPWM4  = 1;
		CpuSysRegs.PCLKCR2.bit.EPWM5  = 1;
		CpuSysRegs.PCLKCR2.bit.EPWM11 = 1;
		CpuSysRegs.PCLKCR2.bit.EPWM12 = 1;
	EDIS;

	EALLOW; // Configure EPWM1. Stop clocks for EPWMs 1st to a reset state
		CpuSysRegs.PCLKCR0.bit.TBCLKSYNC =0;

		InitEPWM_SixPhase();
	EDIS;

		IER |= M_INT1;     // Enable CPU INT1
		IER |= M_INT3;     // Enable CPU INT3

		// Enable EPWM INTn in the PIE: Group 3 interrupt 1
		PieCtrlRegs.PIEIER1.bit.INTx4  = 1;  //XINT_1_sychronize_PWMs
		PieCtrlRegs.PIEIER1.bit.INTx5  = 1;  //XINT_2_overCurrent_protection
		PieCtrlRegs.PIEIER1.bit.INTx13 = 1;  // CPU1 to CPU2 INT0

//		PieCtrlRegs.PIEIER3.bit.INTx1 = 1;    // Enable PIE interrupt_ePWM1
//		PieCtrlRegs.PIEIER3.bit.INTx2 = 1;    // Enable PIE interrupt_ePWM2
//		PieCtrlRegs.PIEIER3.bit.INTx3 = 1;    // Enable PIE interrupt_ePWM3
		PieCtrlRegs.PIEIER3.bit.INTx4 = 1;    // Enable PIE interrupt_ePWM4
//		PieCtrlRegs.PIEIER3.bit.INTx5 = 1;    // Enable PIE interrupt_ePWM5
//		PieCtrlRegs.PIEIER3.bit.INTx11 = 1;   // Enable PIE interrupt_ePWM11
//		PieCtrlRegs.PIEIER3.bit.INTx12 = 1;   // Enable PIE interrupt_ePWM12
		//PieCtrlRegs.PIECTRL.bit.ENPIE = 1;    // Enable the PIE block,

	//Enable global Interrupts and higher priority real-time debug events:
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM

    EALLOW;
    	XintRegs.XINT1CR.bit.POLARITY = 1;  // Rising edge interrupt
    	XintRegs.XINT1CR.bit.ENABLE = 1;    // Enable XINT1
    	XintRegs.XINT2CR.bit.POLARITY = 1;  // Rising edge interrupt
    	XintRegs.XINT2CR.bit.ENABLE = 1;    // Enable XINT2
    EDIS;

    do{

    }while(1);
}

void Copy_EPWMs_compAB_From_CPU01TOCPU02RAM(void)
{
	CompareA_EPWM01_PhaseD = *((Uint16 *)(CPU1TOCPU2RAM_addr_pt + 0));
	CompareA_EPWM02_PhaseE = *((Uint16 *)(CPU1TOCPU2RAM_addr_pt + 1));
	CompareA_EPWM03_PhaseF = *((Uint16 *)(CPU1TOCPU2RAM_addr_pt + 2));
	CompareA_EPWM05_PhaseB = *((Uint16 *)(CPU1TOCPU2RAM_addr_pt + 3));
	CompareA_EPWM11_PhaseC = *((Uint16 *)(CPU1TOCPU2RAM_addr_pt + 4));
	CompareA_EPWM12_PhaseA = *((Uint16 *)(CPU1TOCPU2RAM_addr_pt + 5));

	CompareB_EPWM01_PhaseD = *((Uint16 *)(CPU1TOCPU2RAM_addr_pt + 6));
	CompareB_EPWM02_PhaseE = *((Uint16 *)(CPU1TOCPU2RAM_addr_pt + 7));
	CompareB_EPWM03_PhaseF = *((Uint16 *)(CPU1TOCPU2RAM_addr_pt + 8));
	CompareB_EPWM05_PhaseB = *((Uint16 *)(CPU1TOCPU2RAM_addr_pt + 9));
	CompareB_EPWM11_PhaseC = *((Uint16 *)(CPU1TOCPU2RAM_addr_pt + 10));
	CompareB_EPWM12_PhaseA = *((Uint16 *)(CPU1TOCPU2RAM_addr_pt + 11));

	NVC_start_flag = *((Uint16 *)(CPU1TOCPU2RAM_addr_pt + 12));
	EPWM_Eable_Flag = *((Uint16 *)(CPU1TOCPU2RAM_addr_pt + 13));
//	CompareA_EPWM03_PhaseF[1] = *((Uint16 *)(CPU1TOCPU2RAM_addr_pt + 14));
//	CompareA_EPWM05_PhaseB[1] = *((Uint16 *)(CPU1TOCPU2RAM_addr_pt + 15));
//	CompareA_EPWM11_PhaseC[1] = *((Uint16 *)(CPU1TOCPU2RAM_addr_pt + 16));
//	CompareA_EPWM12_PhaseA[1] = *((Uint16 *)(CPU1TOCPU2RAM_addr_pt + 17));
//	CompareB_EPWM01_PhaseD[1] = *((Uint16 *)(CPU1TOCPU2RAM_addr_pt + 18));
//	CompareB_EPWM02_PhaseE[1] = *((Uint16 *)(CPU1TOCPU2RAM_addr_pt + 19));
//	CompareB_EPWM03_PhaseF[1] = *((Uint16 *)(CPU1TOCPU2RAM_addr_pt + 20));
//	CompareB_EPWM05_PhaseB[1] = *((Uint16 *)(CPU1TOCPU2RAM_addr_pt + 21));
//	CompareB_EPWM11_PhaseC[1] = *((Uint16 *)(CPU1TOCPU2RAM_addr_pt + 22));
//	CompareB_EPWM12_PhaseA[1] = *((Uint16 *)(CPU1TOCPU2RAM_addr_pt + 23));
//
//	NVC_start_flag = *((Uint16 *)(CPU1TOCPU2RAM_addr_pt + 24));
//	t1_negtive_vector_flag = *((Uint16 *)(CPU1TOCPU2RAM_addr_pt + 25));
//	t2_negtive_vector_flag = *((Uint16 *)(CPU1TOCPU2RAM_addr_pt + 26));
//	tx6 = *((Uint16 *)(CPU1TOCPU2RAM_addr_pt + 27));
//	CompareA_EPWM11_PhaseC[2] = *((Uint16 *)(CPU1TOCPU2RAM_addr_pt + 28));
//	CompareA_EPWM12_PhaseA[2] = *((Uint16 *)(CPU1TOCPU2RAM_addr_pt + 29));
//	CompareB_EPWM01_PhaseD[2] = *((Uint16 *)(CPU1TOCPU2RAM_addr_pt + 30));
//	CompareB_EPWM02_PhaseE[2] = *((Uint16 *)(CPU1TOCPU2RAM_addr_pt + 31));
//	CompareB_EPWM03_PhaseF[2] = *((Uint16 *)(CPU1TOCPU2RAM_addr_pt + 32));
//	CompareB_EPWM05_PhaseB[2] = *((Uint16 *)(CPU1TOCPU2RAM_addr_pt + 33));
//	CompareB_EPWM11_PhaseC[2] = *((Uint16 *)(CPU1TOCPU2RAM_addr_pt + 34));
//	CompareB_EPWM12_PhaseA[2] = *((Uint16 *)(CPU1TOCPU2RAM_addr_pt + 35));
}


void Set_EPWMs_compAB_To_Default(void)
{
	CompareA_EPWM01_PhaseD = 100;
	CompareA_EPWM02_PhaseE = 100;
	CompareA_EPWM03_PhaseF = 100;
	CompareA_EPWM05_PhaseB = 100;
	CompareA_EPWM11_PhaseC = 100;
	CompareA_EPWM12_PhaseA = 100;
	CompareB_EPWM01_PhaseD = 200;
	CompareB_EPWM02_PhaseE = 200;
	CompareB_EPWM03_PhaseF = 200;
	CompareB_EPWM05_PhaseB = 200;
	CompareB_EPWM11_PhaseC = 200;
	CompareB_EPWM12_PhaseA = 200;

}


void CPU02_Send_Command_To_CPU01(Uint16 CPUcomd)
{
	IpcRegs.IPCSENDCOM = CPUcomd;  // set comd flag
	IpcRegs.IPCSET.bit.IPC0 = 1;   // trigger a CUP1 Interrupt
}

Uint32 receive_data = 0;
Uint32 receive_comd = 0;

Uint32 Transfer_INT_update_cpu2=0;
Uint32 Transfer_INT_stop_cpu2=0;

__interrupt void CPU01toCPU02IPC0IntHandler (void)
{
    //receive_data = IpcRegs.IPCRECVDATA; // data from CPU1
    receive_comd = IpcRegs.IPCRECVCOM;  // command from CPU1

    switch(receive_comd)
    {
    	case COMMAND_synch_PWMs: //1
    		break;

    	case COMMAND_stop_PWMs: // 2
    		EPWM_Enable_Flag = 0;
    		EALLOW;
    		        EPwm12Regs.DBCTL.bit.OUT_MODE = 0x0;
    		        EPwm5Regs.DBCTL.bit.OUT_MODE = 0x0;
    		        EPwm11Regs.DBCTL.bit.OUT_MODE = 0x0;
    		        EPwm1Regs.DBCTL.bit.OUT_MODE = 0x0;
    		        EPwm2Regs.DBCTL.bit.OUT_MODE = 0x0;
    		        EPwm3Regs.DBCTL.bit.OUT_MODE = 0x0;

    				EPwm12Regs.AQCSFRC.bit.CSFA = 1; //PhaseA
    				EPwm12Regs.AQCSFRC.bit.CSFB = 1;
    				EPwm5Regs.AQCSFRC.bit.CSFA = 1;  //PhaseB
    				EPwm5Regs.AQCSFRC.bit.CSFB = 1;
    				EPwm11Regs.AQCSFRC.bit.CSFA = 1; //PhaseC
    				EPwm11Regs.AQCSFRC.bit.CSFB = 1;
    				EPwm1Regs.AQCSFRC.bit.CSFA = 1;  //PhaseD
    				EPwm1Regs.AQCSFRC.bit.CSFB = 1;
    				EPwm2Regs.AQCSFRC.bit.CSFA = 1;  //PhaseE
    				EPwm2Regs.AQCSFRC.bit.CSFB = 1;
    				EPwm3Regs.AQCSFRC.bit.CSFA = 1;  //PhaseF
    				EPwm3Regs.AQCSFRC.bit.CSFB = 1;
    			EDIS;  // force PWMs output low
    		CPU02_Send_Command_To_CPU01(COMMAND_stop_PWMs_done); // tell its master CPU01, finished everyting
    		break;

    	case COMMAND_start_PWMs: //3
    		EPWM_Enable_Flag = 1;

    		EALLOW;
	            EPwm12Regs.DBCTL.bit.OUT_MODE = 0x3;
	            EPwm5Regs.DBCTL.bit.OUT_MODE = 0x3;
	            EPwm11Regs.DBCTL.bit.OUT_MODE = 0x3;
	            EPwm1Regs.DBCTL.bit.OUT_MODE = 0x3;
	            EPwm2Regs.DBCTL.bit.OUT_MODE = 0x3;
	            EPwm3Regs.DBCTL.bit.OUT_MODE = 0x3;

    			EPwm12Regs.AQCSFRC.bit.CSFA = 0; //PhaseA
    			EPwm12Regs.AQCSFRC.bit.CSFB = 0;
    			EPwm5Regs.AQCSFRC.bit.CSFA = 0;  //PhaseB
    			EPwm5Regs.AQCSFRC.bit.CSFB = 0;
    			EPwm11Regs.AQCSFRC.bit.CSFA = 0; //PhaseC
    			EPwm11Regs.AQCSFRC.bit.CSFB = 0;
    			EPwm1Regs.AQCSFRC.bit.CSFA = 0;  //PhaseD
    			EPwm1Regs.AQCSFRC.bit.CSFB = 0;
    			EPwm2Regs.AQCSFRC.bit.CSFA = 0;  //PhaseE
    			EPwm2Regs.AQCSFRC.bit.CSFB = 0;
    			EPwm3Regs.AQCSFRC.bit.CSFA = 0;  //PhaseF
    			EPwm3Regs.AQCSFRC.bit.CSFB = 0;
    		EDIS; // stop forcing enable PWMs output low

    		CPU02_Send_Command_To_CPU01(COMMAND_start_PWMs_done); // tell its master CPU01, finished everyting
    		break;

    	case COMMAND_update_PWMs_compAB: //4

    		break;
    	default:
    		break;
    }
    // Acknowledge IPC INT0 Flag and PIE to receive more interrupts
    IpcRegs.IPCACK.bit.IPC0 = 1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

Uint16 PWM_ShadowModel_flag=0;
void InitEPWM_SixPhase(void)
{
	//* Tips:
	//* 1. PWM12_AB, PWM5_AB, PWM11_AB, PWM1_AB, PWM2_AB, PWM3_AB, for PhaseA, PhaseB, PhaseC, PhaseD, PhaseE, PhaseF, respectively.
	//--------------------------PMW1-----For-----Driver4------PhaseD----------------------------------
    EPwm1Regs.TBPRD = EPWM_period;//EPWM_period;                       // Set timer period
    EPwm1Regs.TBPHS.bit.TBPHS = 0x0000;           // Phase is 0
    EPwm1Regs.TBCTR = 0x0000;                     // Clear counter

    // Setup TBCLK
    EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; // Count up and down
    EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;        // Disable phase loading
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV4;       // Clock ratio to SYSCLKOUT
    EPwm1Regs.TBCTL.bit.CLKDIV =0; // TB_DIV4;
    //EPwm1Regs.TBCTL.bit.PRDLD =0;  // When CTR=0, Load TBPRD from its ShadowRegistor

    EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;//CC_SHADOW;   // Load registers every ZERO CC_IMMEDIATE;//
    EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;//CC_SHADOW;   // CC_IMMEDIATE; //
    EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO; //CC_CTR_ZERO; // CC_LD_DISABLE; //
    EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO; //CC_CTR_ZERO; // CC_LD_DISABLE;//

    // Setup compare
    EPwm1Regs.CMPA.bit.CMPA = 0;
    EPwm1Regs.CMPB.bit.CMPB = 0;

    // Set actions
    EPwm1Regs.AQCTLA.bit.CAU = AQ_SET;             // Set PWM1A on Zero
    EPwm1Regs.AQCTLA.bit.CBU = AQ_CLEAR;
    EPwm1Regs.AQCTLB.bit.CAU = AQ_CLEAR;         // Set PWM1A on Zero
    EPwm1Regs.AQCTLB.bit.CBU = AQ_SET;

    // Active Low PWMs - Setup Deadband
    EPwm1Regs.DBCTL.bit.OUT_MODE = 0x3;
    EPwm1Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
    EPwm1Regs.DBCTL.bit.IN_MODE = DBA_ALL;
    EPwm1Regs.DBRED = EPWMx_MIN_DB;
    EPwm1Regs.DBFED = EPWMx_MIN_DB;

    // Interrupt where we will change the Deadband
    EPwm1Regs.ETSEL.bit.INTSEL = ET_CTRU_CMPB;//ET_CTR_ZERO;    //ET_CTRU_CMPA; // Select INT on Zero event  ET_CTRD_CMPA
	EPwm1Regs.ETSEL.bit.INTEN = 1;               // Enable INT
	EPwm1Regs.ETPS.bit.INTPRD = ET_1ST;          // Generate INT on 3rd event


    //--------------------------PMW2-----For----Driver5--------PhaseE--------------------------------
    EPwm2Regs.TBPRD = EPWM_period;                // Set timer period
    EPwm2Regs.TBPHS.bit.TBPHS = 0x0000;           // Phase is 0
    EPwm2Regs.TBCTR = 0x0000;                     // Clear counter

    // Setup TBCLK
    EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;    // Count up
    EPwm2Regs.TBCTL.bit.PHSEN = TB_DISABLE;       // Disable phase loading
    EPwm2Regs.TBCTL.bit.HSPCLKDIV = TB_DIV4;      // Clock ratio to SYSCLKOUT
    EPwm2Regs.TBCTL.bit.CLKDIV = 0;//TB_DIV4;

    EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;//CC_SHADOW;   // Load registers every ZERO
    EPwm2Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;//CC_SHADOW;
    EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;//CC_CTR_ZERO;
    EPwm2Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;//CC_CTR_ZERO;

    // Setup compare
    EPwm2Regs.CMPA.bit.CMPA = 0;
    EPwm2Regs.CMPB.bit.CMPB = 0;

    // Set actions
    EPwm2Regs.AQCTLA.bit.CAU = AQ_SET;           // Set PWM1A on Zero
    EPwm2Regs.AQCTLA.bit.CBU = AQ_CLEAR;         // Set PWM1A on Zero
    EPwm2Regs.AQCTLB.bit.CAU = AQ_CLEAR;         // Set PWM1A on Zero
    EPwm2Regs.AQCTLB.bit.CBU = AQ_SET;

    // Active Low PWMs - Setup Deadband
    EPwm2Regs.DBCTL.bit.OUT_MODE = 0x3;
    EPwm2Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
    EPwm2Regs.DBCTL.bit.IN_MODE = DBA_ALL;
    EPwm2Regs.DBRED = EPWMx_MIN_DB;
    EPwm2Regs.DBFED = EPWMx_MIN_DB;

    // Interrupt where we will change the Deadband
    EPwm2Regs.ETSEL.bit.INTSEL =ET_CTRU_CMPB;      //ET_CTRU_CMPA;// ET_CTR_ZERO;    // Select INT on Zero event  ET_CTRD_CMPA
	EPwm2Regs.ETSEL.bit.INTEN = 1;               // Enable INT
	EPwm2Regs.ETPS.bit.INTPRD = ET_1ST;          // Generate INT on 3rd event

   //--------------------------------PWM3-----For------Driver6-------PhaseF------------------------------------------
    EPwm3Regs.TBPRD = EPWM_period;                // Set timer period
    EPwm3Regs.TBPHS.bit.TBPHS = 0x0000;           // Phase is 0
    EPwm3Regs.TBCTR = 0x0000;                     // Clear counter

    // Setup TBCLK
    EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; // Count up
    EPwm3Regs.TBCTL.bit.PHSEN = TB_DISABLE;        // Disable phase loading
    EPwm3Regs.TBCTL.bit.HSPCLKDIV = TB_DIV4;       // Clock ratio to SYSCLKOUT
    EPwm3Regs.TBCTL.bit.CLKDIV =0;  // TB_DIV4;

    EPwm3Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;//CC_SHADOW;   // Load registers every ZERO
    EPwm3Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;//CC_SHADOW;
    EPwm3Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;//CC_CTR_ZERO;
    EPwm3Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;//CC_CTR_ZERO;

    // Setup compare
    EPwm3Regs.CMPA.bit.CMPA = 0;
    EPwm3Regs.CMPB.bit.CMPB = 0;

    // Set actions
    EPwm3Regs.AQCTLA.bit.CAU = AQ_SET;           // Set PWM1A on Zero
    EPwm3Regs.AQCTLA.bit.CBU = AQ_CLEAR;         // Set PWM1A on Zero
    EPwm3Regs.AQCTLB.bit.CAU = AQ_CLEAR;         // Set PWM1A on Zero
    EPwm3Regs.AQCTLB.bit.CBU = AQ_SET;

    // Active Low PWMs - Setup Deadband
    EPwm3Regs.DBCTL.bit.OUT_MODE = 0x3;
    EPwm3Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
    EPwm3Regs.DBCTL.bit.IN_MODE = DBA_ALL;
    EPwm3Regs.DBRED = EPWMx_MIN_DB;
    EPwm3Regs.DBFED = EPWMx_MIN_DB;

    // Interrupt where we will change the Deadband
    EPwm3Regs.ETSEL.bit.INTSEL =ET_CTRU_CMPB;      //ET_CTRU_CMPA;// ET_CTR_ZERO;    // Select INT on Zero event  ET_CTRD_CMPA
	EPwm3Regs.ETSEL.bit.INTEN = 1;               // Enable INT
	EPwm3Regs.ETPS.bit.INTPRD = ET_1ST;          // Generate INT on 3rd event


    //----------------------------------PWM5---For-----Driver2------------PhaseB--------------------------
    EPwm5Regs.TBPRD = EPWM_period;          // Set timer period
    EPwm5Regs.TBPHS.bit.TBPHS = 0x0000;     // Phase is 0
    EPwm5Regs.TBCTR = 0x0000;               // Clear counter

            // Setup TBCLK
    EPwm5Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;     // Count up
    EPwm5Regs.TBCTL.bit.PHSEN = TB_DISABLE;        // Disable phase loading
    EPwm5Regs.TBCTL.bit.HSPCLKDIV = TB_DIV4;       // Clock ratio to SYSCLKOUT
    EPwm5Regs.TBCTL.bit.CLKDIV =0;  // TB_DIV4;

	EPwm5Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;//CC_SHADOW;    // Load registers every ZERO
	EPwm5Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;//CC_SHADOW;
	EPwm5Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;//CC_CTR_ZERO;
	EPwm5Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;//CC_CTR_ZERO;

    // Setup compare
    EPwm5Regs.CMPA.bit.CMPA = 0;
    EPwm5Regs.CMPB.bit.CMPB = 0;

    // Set actions
    EPwm5Regs.AQCTLA.bit.CAU = AQ_SET;           // Set PWM1A on Zero
    EPwm5Regs.AQCTLA.bit.CBU = AQ_CLEAR;         // Set PWM1A on Zero
    EPwm5Regs.AQCTLB.bit.CAU = AQ_CLEAR;         // Set PWM1A on Zero
    EPwm5Regs.AQCTLB.bit.CBU = AQ_SET;

    // Active Low PWMs - Setup Deadband
    EPwm5Regs.DBCTL.bit.OUT_MODE = 0x3;
    EPwm5Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
    EPwm5Regs.DBCTL.bit.IN_MODE = DBA_ALL;
    EPwm5Regs.DBRED = EPWMx_MIN_DB;
    EPwm5Regs.DBFED = EPWMx_MIN_DB;

    // Interrupt where we will change the Deadband
	EPwm5Regs.ETSEL.bit.INTSEL =ET_CTRU_CMPB;      //ET_CTRU_CMPA;// ET_CTR_ZERO;    // Select INT on Zero event  ET_CTRD_CMPA
	EPwm5Regs.ETSEL.bit.INTEN = 1;               // Enable INT
	EPwm5Regs.ETPS.bit.INTPRD = ET_1ST;          // Generate INT on 3rd event


   //-------------------------PWM11------For-------Driver3--------------------PhaseC-------------------------------
    EPwm11Regs.TBPRD = EPWM_period;                // Set timer period
    EPwm11Regs.TBPHS.bit.TBPHS = 0x0000;           // Phase is 0
    EPwm11Regs.TBCTR = 0x0000;                     // Clear counter

    // Setup TBCLK
    EPwm11Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; // Count up
    EPwm11Regs.TBCTL.bit.PHSEN = TB_DISABLE;        // Disable phase loading
    EPwm11Regs.TBCTL.bit.HSPCLKDIV = TB_DIV4;       // Clock ratio to SYSCLKOUT
    EPwm11Regs.TBCTL.bit.CLKDIV =0;  // TB_DIV4;

	EPwm11Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;//CC_SHADOW;   // Load registers every ZERO
	EPwm11Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;//CC_SHADOW;
	EPwm11Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;//CC_CTR_ZERO;
	EPwm11Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;//CC_CTR_ZERO;

    // Setup compare
    EPwm11Regs.CMPA.bit.CMPA = 0;
    EPwm11Regs.CMPB.bit.CMPB = 0;

    // Set actions
    EPwm11Regs.AQCTLA.bit.CAU = AQ_SET;           // Set PWM1A on Zero
    EPwm11Regs.AQCTLA.bit.CBU = AQ_CLEAR;         // Set PWM1A on Zero
    EPwm11Regs.AQCTLB.bit.CAU = AQ_CLEAR;         // Set PWM1A on Zero
    EPwm11Regs.AQCTLB.bit.CBU = AQ_SET;

    // Active Low PWMs - Setup Deadband
    EPwm11Regs.DBCTL.bit.OUT_MODE = 0x3;
    EPwm11Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
    EPwm11Regs.DBCTL.bit.IN_MODE = DBA_ALL;
    EPwm11Regs.DBRED = EPWMx_MIN_DB;
    EPwm11Regs.DBFED = EPWMx_MIN_DB;

    // Interrupt where we will change the Deadband
	EPwm11Regs.ETSEL.bit.INTSEL =ET_CTRU_CMPB;      //ET_CTRU_CMPA;// ET_CTR_ZERO;    // Select INT on Zero event  ET_CTRD_CMPA
	EPwm11Regs.ETSEL.bit.INTEN = 1;               // Enable INT
	EPwm11Regs.ETPS.bit.INTPRD = ET_1ST;          // Generate INT on 3rd event


   //----------------------PWM12----For----Driver1-----------------------------PhaseA-------------------------
    EPwm12Regs.TBPRD = EPWM_period;                // Set timer period
    EPwm12Regs.TBPHS.bit.TBPHS = 0x0000;           // Phase is 0
    EPwm12Regs.TBCTR = 0x0000;                     // Clear counter

    // Setup TBCLK
    EPwm12Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; // Count up
    EPwm12Regs.TBCTL.bit.PHSEN = TB_DISABLE;        // Disable phase loading
    EPwm12Regs.TBCTL.bit.HSPCLKDIV = TB_DIV4;       // Clock ratio to SYSCLKOUT
    EPwm12Regs.TBCTL.bit.CLKDIV =0;  // TB_DIV4;

    EPwm12Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;//CC_SHADOW;   // Load registers every ZERO
    EPwm12Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;//CC_SHADOW;
    EPwm12Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;//CC_CTR_ZERO; //
    EPwm12Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;//CC_CTR_ZERO;

    // Setup compare
    EPwm12Regs.CMPA.bit.CMPA = 0;
    EPwm12Regs.CMPB.bit.CMPB = 0;

    // Set actions
    EPwm12Regs.AQCTLA.bit.CAU = AQ_SET;           // Set PWM1A on Zero
    EPwm12Regs.AQCTLA.bit.CBU = AQ_CLEAR;         // Set PWM1A on Zero
    EPwm12Regs.AQCTLB.bit.CAU = AQ_CLEAR;         // Set PWM1A on Zero
    EPwm12Regs.AQCTLB.bit.CBU = AQ_SET;

    // Active Low PWMs - Setup Deadband
    EPwm12Regs.DBCTL.bit.OUT_MODE = 0x3;
    EPwm12Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
    EPwm12Regs.DBCTL.bit.IN_MODE = DBA_ALL;
    EPwm12Regs.DBRED = EPWMx_MIN_DB;
    EPwm12Regs.DBFED = EPWMx_MIN_DB;

    // Interrupt where we will change the Deadband
    EPwm12Regs.ETSEL.bit.INTSEL =ET_CTRU_CMPB;      //ET_CTRU_CMPA;// ET_CTR_ZERO;    // Select INT on Zero event  ET_CTRD_CMPA
	EPwm12Regs.ETSEL.bit.INTEN = 1;               // Enable INT
	EPwm12Regs.ETPS.bit.INTPRD = ET_1ST;          // Generate INT on 3rd event


	//----------------------PWM4----For----Driver1-----------------------------PhaseA-------------------------
	EPwm4Regs.TBPRD = EPWM_period;                // Set timer period
	EPwm4Regs.TBPHS.bit.TBPHS = 0x0000;           // Phase is 0
	EPwm4Regs.TBCTR = 0x0000;                     // Clear counter

	// Setup TBCLK
	EPwm4Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; // Count up
	EPwm4Regs.TBCTL.bit.PHSEN = TB_DISABLE;        // Disable phase loading
	EPwm4Regs.TBCTL.bit.HSPCLKDIV = TB_DIV4;       // Clock ratio to SYSCLKOUT
	EPwm4Regs.TBCTL.bit.CLKDIV =0;  // TB_DIV4;

	EPwm4Regs.CMPCTL.bit.SHDWAMODE = CC_IMMEDIATE;   // Load registers every ZERO
	EPwm4Regs.CMPCTL.bit.SHDWBMODE = CC_IMMEDIATE;
	EPwm4Regs.CMPCTL.bit.LOADAMODE = CC_LD_DISABLE; //
	EPwm4Regs.CMPCTL.bit.LOADBMODE = CC_LD_DISABLE;

	// Setup compare
	EPwm4Regs.CMPA.bit.CMPA = 2000;
	EPwm4Regs.CMPB.bit.CMPB = EPWM_period;

	// Set actions
	EPwm4Regs.AQCTLA.bit.CAU = AQ_SET;           // Set PWM1A on Zero
	EPwm4Regs.AQCTLA.bit.CBU = AQ_CLEAR;         // Set PWM1A on Zero

	// Interrupt where we will change the Deadband
	EPwm4Regs.ETSEL.bit.INTSEL =ET_CTRU_CMPA;      //ET_CTRU_CMPA;// ET_CTR_ZERO;    // Select INT on Zero event  ET_CTRD_CMPA
	EPwm4Regs.ETSEL.bit.INTEN = 1;               // Enable INT
	EPwm4Regs.ETPS.bit.INTPRD = ET_1ST;          // Generate INT on 3rd event

	EPwm12Regs.AQCSFRC.bit.CSFA = 1; //PhaseA
	EPwm12Regs.AQCSFRC.bit.CSFB = 1;
	EPwm5Regs.AQCSFRC.bit.CSFA = 1;  //PhaseB
	EPwm5Regs.AQCSFRC.bit.CSFB = 1;
	EPwm11Regs.AQCSFRC.bit.CSFA = 1; //PhaseC
	EPwm11Regs.AQCSFRC.bit.CSFB = 1;
	EPwm1Regs.AQCSFRC.bit.CSFA = 1;  //PhaseD
	EPwm1Regs.AQCSFRC.bit.CSFB = 1;
	EPwm2Regs.AQCSFRC.bit.CSFA = 1;  //PhaseE
	EPwm2Regs.AQCSFRC.bit.CSFB = 1;
	EPwm3Regs.AQCSFRC.bit.CSFA = 1;  //PhaseF
	EPwm3Regs.AQCSFRC.bit.CSFB = 1;
}

Uint32 epwm1_int_num =0;
Uint16 CompA_not_zero = 0;
Uint16 CompB_not_zero = 0;

__interrupt void epwm4_isr(void)
{
	//---------------------//
	EPwm4Regs.ETCLR.bit.INT = 1;            // Clear INT flag for this timer
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP3; // Acknowledge this interrupt to receive more interrupts from group
	Copy_EPWMs_compAB_From_CPU01TOCPU02RAM();
	if(EPWM_Enable_Flag==1)
	{
		EALLOW;

	    EPwm1Regs.CMPA.bit.CMPA = CompareA_EPWM01_PhaseD;  //PhaseD
        EPwm1Regs.CMPB.bit.CMPB = CompareB_EPWM01_PhaseD;

	    EPwm2Regs.CMPA.bit.CMPA = CompareA_EPWM02_PhaseE;  //PhaseE
	    EPwm2Regs.CMPB.bit.CMPB = CompareB_EPWM02_PhaseE;

    	EPwm3Regs.CMPA.bit.CMPA = CompareA_EPWM03_PhaseF;  //PhaseF
	    EPwm3Regs.CMPB.bit.CMPB = CompareB_EPWM03_PhaseF;

	    EPwm5Regs.CMPA.bit.CMPA = CompareA_EPWM05_PhaseB;  //PhaseB
	    EPwm5Regs.CMPB.bit.CMPB = CompareB_EPWM05_PhaseB;

	    EPwm11Regs.CMPA.bit.CMPA = CompareA_EPWM11_PhaseC;  //PhaseC
	    EPwm11Regs.CMPB.bit.CMPB = CompareB_EPWM11_PhaseC;

		EPwm12Regs.CMPA.bit.CMPA = CompareA_EPWM12_PhaseA;  //PhaseA
		EPwm12Regs.CMPB.bit.CMPB = CompareB_EPWM12_PhaseA;


		EDIS;
	}
	else
	{
		Set_EPWMs_compAB_To_Default();
	}
}

__interrupt void xint1_isr_synchronous_PWM(void)
{
	EALLOW;
		CpuSysRegs.PCLKCR0.bit.TBCLKSYNC =1;
	EDIS;
	++CUP2_EXINT_NUM;
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;// Acknowledge this interrupt to get more from group 1
}

__interrupt void xint2_isr_overCurrent_INT(void)
{
	Xint2Count++;

	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;// Acknowledge this interrupt to get more from group 1
}




