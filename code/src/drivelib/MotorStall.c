#include "AppBuild.h"
#include "../ActADC.h"
#include "drivelib/GlobalVars.h"
#include "drivelib/MotorDriver.h"
#include "drivelib/MotorDriverTables.h"
#include "drivelib/MotorStall.h"
#include "drivelib/PID_Control.h"
#include "camculib/private_mathlib.h"
#include "commlib/LIN_Communication.h"

#pragma space dp
uint16_t l_u16MotorCurrentStallThrshldxN;
volatile uint8_t l_u8StallCountA = 0U;
static uint8_t l_u8StallWidthA = C_STALL_A_WIDTH;
int16_t l_i16StallThresholdLA;
int16_t l_i16StallThresholdLA_Max;
int16_t l_i16ActLoadAngleLPF = 0;
int32_t l_i32ActLoadAngleLPF = 0;
volatile uint8_t l_u8StallCountLA = 0U;
#pragma space none
#pragma space nodp
#pragma space none
uint8_t StallDetectorEna(void)
{
	uint8_t e8StallDetectorEna = C_STALLDET_ALL;
	if((NV_STALL_A  ==  FALSE)  ||  (NV_STALL_A_WIDTH  ==  0x00U)  ||  (NV_STALL_A_THRSHLD  ==  0x00U)){e8StallDetectorEna  &=  ~C_STALLDET_A;}
	if((NV_STALL_LA_THRSHLD  ==  0x00U)  ||  (NV_STALL_LA_WIDTH  ==  0x00U)){e8StallDetectorEna  &=  ~C_STALLDET_LA;}
	return(e8StallDetectorEna);
}

void MotorStallInitA(void)
{
	uint16_t u16Threshold;
	g_u8StallTypeComm = (uint8_t)C_STALL_NOT_FOUND;
	l_u8StallCountA = 0U;
	l_u8StallWidthA = NV_STALL_A_WIDTH;
	if(NV_STALL_SPEED_DEPENDED  !=  0U){u16Threshold = (NV_STALL_A_THRSHLD + (C_STALL_THRESHOLD_DIV - 8U)) + ((uint16_t)g_u8MotorStatusSpeed  <<  3);}
	else{u16Threshold = (NV_STALL_A_THRSHLD + C_STALL_THRESHOLD_DIV);}
	l_u16MotorCurrentStallThrshldxN =
	(uint16_t)(p_MulU32_U16byU16(Get_ActCurrRunMax_LSB(), u16Threshold)  >>  (C_STALL_THRESHOLD_SDIV - C_MOVAVG_SSZ));
}

uint16_t MotorStallCheckA(void)
{
	uint16_t u16StallResult = C_STALL_NOT_FOUND;
	if((Get_StartupDelay()  ==  0U)  &&  (Get_MotorCurrentMovAvgxN() > (C_MIN_MOTORCURRENT  <<  4)))
	{
		if(l_u16MotorCurrentStallThrshldxN > Get_MotorCurrentMovAvgxN()){l_u8StallCountA = p_DecNzU8(l_u8StallCountA);}
		else
		{
			uint8_t u8StallCountA = l_u8StallCountA + 1U;
			l_u8StallCountA = u8StallCountA;
			if(u8StallCountA  >=  l_u8StallWidthA){u16StallResult = C_STALL_FOUND;}
		}
	}
	return(u16StallResult);
}

void LinDiag_MotorStall(void)
{
	uint16_t u16Value;
#if (C_MOVAVG_SZ == (65536UL / C_GMCURR_DIV))
	u16Value = p_MulU16hi_U16byU16(l_u16MotorCurrentStallThrshldxN,
	Get_MCurrGain());
#elif (C_MOVAVG_SZ < (65536UL / C_GMCURR_DIV))
#if defined (C_GMCURR_SDIV) && defined (C_MOVAVG_SSZ)
	u16Value = (uint16_t) (p_MulU32_U16byU16(l_u16MotorCurrentStallThrshldxN,
	Get_MCurrGain())  >>  (C_GMCURR_SDIV + C_MOVAVG_SSZ));
#else
	u16Value = p_MulDivU16_U16byU16byU16(l_u16MotorCurrentStallThrshldxN,
	Get_MCurrGain(),
	(C_GMCURR_DIV * C_MOVAVG_SZ));
#endif
#else
#error "Error: Stall current"
#endif
	g_DiagResponse.u.SF.byD1 = (uint8_t)(u16Value & 0xFFU);
	g_DiagResponse.u.SF.byD2 = (uint8_t)(u16Value  >>  8);
	u16Value = Get_MotorCurrentMovAvgxN_mA();
	g_DiagResponse.u.SF.byD3 = (uint8_t)(u16Value & 0xFFU);
	g_DiagResponse.u.SF.byD4 = (uint8_t)(u16Value  >>  8);
	u16Value = 0U;
	if(((g_u8StallTypeComm & C_STALL_FOUND_A)  !=  0U)){u16Value = l_u8StallCountA;}
	else{}
	while(u16Value > 7U){u16Value = ((u16Value + 1U)  >>  1);}
	g_DiagResponse.u.SF.byD5 = (g_u8StallTypeComm & M_STALL_MODE) | (uint8_t)u16Value;
	g_u8StallTypeComm = (uint8_t)C_STALL_NOT_FOUND;
}
