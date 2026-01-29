#include "AppBuild.h"
#include "ActADC.h"
#include "drivelib/ErrorCodes.h"
#include "drivelib/NV_UserPage.h"
#include "drivelib/MotorDriver.h"
#include "drivelib/MotorDriverTables.h"
#include "camculib/private_mathlib.h"
#include <sys_tools.h>

#pragma space dp
int16_t g_i16MotorCurrentCoilA = 0;
int16_t g_i16MotorCurrentCoilB = 0;
uint16_t g_u16MotorCurrentPeak = 0U;
int16_t g_i16MotorCurrentAngle = 0;
#pragma space none
#pragma space nodp
volatile ADC_RESULTS l_AdcResult;
#pragma space none
static ADC_SDATA_t const SBASE_MOTOR_RUN[] =
{
	C_ADC_VS_MSTR1_CMP,
	C_ADC_VDDA_EOC,
	C_ADC_VDDD_EOC,
	C_ADC_MCUR_SLV2,
	C_ADC_VSMF_EOC,
	C_ADC_TEMP_EOC,
	C_ADC_MCUR_MSTR2
};
static ADC_SDATA_t const SBASE_MOTOR_IDLE[] =
{
	C_ADC_VS_EOC,
	C_ADC_VDDA_EOC,
	C_ADC_VDDD_EOC,
	C_ADC_MCUR_EOC,
	C_ADC_VSMF_EOC,
	C_ADC_TEMP_EOC,
};
void ADC_Start(uint16_t u16IrqEna)
{
	if(l_u8AdcMode  !=  C_ADC_MODE_RUN_HW)
	{
		HAL_ADC_StopSafe();
		{
			uint16_t *pu16Src = &l_au16AdcSource[0];
			{
				uint16_t *pu16SBase = (uint16_t *)&SBASE_MOTOR_RUN[0];
				*pu16Src++ = (uint16_t)&l_AdcResult;
				pu16Src = p_MemCpyU16(pu16Src, pu16SBase, (sizeof(SBASE_MOTOR_RUN) / sizeof(uint16_t)));
			}
			*pu16Src++ = C_ADC_EOF;
			*pu16Src = (uint16_t)&l_au16AdcSource[0];
		}
		HAL_ADC_Setup(
		C_ADC_ASB_NEVER |
		C_ADC_INT_SCHEME_NOINT |
		B_ADC_SATURATE |
		B_ADC_NO_INTERLEAVE |
		C_ADC_SOC_SOURCE_HARD_CTRIG |
		C_ADC_SOS_SOURCE_2ND_HARD_CTRIG);
		HAL_ADC_ClearErrors();
		(void)u16IrqEna;
		HAL_ADC_Start();
		l_u8AdcMode = (uint8_t)C_ADC_MODE_RUN_HW;
	}
}

__attribute__((interrupt)) void ADC_ISR(void)
{
}

static inline void CalcMotorCurrentAngleAndPeak(void)
{
	g_i16MotorCurrentAngle = (int16_t)p_atan2I16(g_i16MotorCurrentCoilA, g_i16MotorCurrentCoilY);
	uint16_t u16Idx = ((uint16_t)g_i16MotorCurrentAngle / 256);
	int16_t *pi16SinCos = (int16_t *) &c_ai16MicroStepVector3PH_SinCos256[u16Idx];
	g_u16MotorCurrentPeak = p_MulI16_I16bypQ15( g_i16MotorCurrentCoilA, pi16SinCos) +
	p_MulI16_I16bypQ15( g_i16MotorCurrentCoilY, (pi16SinCos + C_COS_OFFSET));
}

uint16_t ADC_GetRawMotorDriverCurrent(uint16_t u16MicroStepIdx)
{
	(void)u16MicroStepIdx;
	g_i16MotorCurrentCoilA = (int16_t)(l_AdcResult.u16AdcCurrA - Get_CurrentZeroOffset());
	if(u16MicroStepIdx  >=  (2U * C_MICROSTEP_PER_FULLSTEP)){g_i16MotorCurrentCoilA = -g_i16MotorCurrentCoilA;}
	g_i16MotorCurrentCoilB = (int16_t)(l_AdcResult.u16AdcCurrB - Get_CurrentZeroOffset());
	if((u16MicroStepIdx  >=  C_MICROSTEP_PER_FULLSTEP)  &&  (u16MicroStepIdx < (3U * C_MICROSTEP_PER_FULLSTEP))){g_i16MotorCurrentCoilB = -g_i16MotorCurrentCoilB;}
	CalcMotorCurrentAngleAndPeak();
	return(g_u16MotorCurrentPeak);
}

void ADC_MeasureVsupplyAndTemperature(void)
{
	HAL_ADC_StopSafe();
	{
		uint16_t *pu16Src = &l_au16AdcSource[0];
		uint16_t *pu16SBase = (uint16_t *)&SBASE_MOTOR_IDLE[0];
		*pu16Src++ = (uint16_t)&l_AdcResult;
		pu16Src = p_MemCpyU16(pu16Src, (uint16_t *)pu16SBase,
		(sizeof(SBASE_MOTOR_IDLE) / sizeof(uint16_t)));
		*pu16Src = C_ADC_EOS;
	}
	(void)HAL_ADC_StartSoftTrig(C_ADC_STATE_IDLE);
	if((IO_PORT_DRV_OUT & B_PORT_DRV_OUT_ENABLE_CSA)  ==  0U){l_AdcResult.u16AdcCurrA = Get_CurrentZeroOffset();}
	l_AdcResult.u16AdcCurrB = Get_CurrentZeroOffset();
}

uint16_t ADC_GetNewSampleVsupply(void)
{
	uint16_t u16Vsupply;
	static const ADC_SDATA_t VS = C_ADC_VS_EOC;
	if(l_u8AdcMode  ==  C_ADC_MODE_RUN_HW){u16Vsupply = l_AdcResult.u16AdcVs;}
	else
	{
		uint16_t *pu16Src = &l_au16AdcSource[0];
		*pu16Src++ = (uint16_t)&l_AdcResult.u16AdcVs;
		*pu16Src++ = VS.u16;
		*pu16Src = C_ADC_EOS;
		(void)HAL_ADC_StartSoftTrig(C_ADC_STATE_IDLE);
		u16Vsupply = l_AdcResult.u16AdcVs;
	}
	return(u16Vsupply);
}
