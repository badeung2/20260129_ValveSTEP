#include "AppBuild.h"
#include "drivelib/ErrorCodes.h"
#include "drivelib/GlobalVars.h"
#include "drivelib/MotorDriver.h"
#include "camculib/private_mathlib.h"
#include "../ActADC.h"
#include "commlib/LIN_Communication.h"
#include <bist_inline_impl.h>
#include <sys_tools.h>
#define ADC_TRIGGER_OSD_SW 0
#define ADC_TRIGGER_OSD_CTIMER1 1
#define ADC_TRIGGER_OSD ADC_TRIGGER_OSD_CTIMER1
#pragma space dp
uint16_t l_u16CurrentZeroOffset = C_OADC_MCUR;
#pragma space none
#pragma space nodp
static uint16_t l_u16VddaADC;
static uint16_t l_u16VdddADC;
#pragma space none
void ADC_Init(void)
{
	static const ADC_SDATA_t SBASE_INIT[] =
	{
		{
			{
				.u2AdcMarker = C_ADC_NO_SIGN,
				.u5AdcChannel = C_ADC_VS_HV,
				.u3AdcVref = C_ADC_VREF_2_50_V,
				.u5AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK,
				.u1AdcReserved = 0U
			}
		},
		{
			{
				.u2AdcMarker = C_ADC_NO_SIGN,
				.u5AdcChannel = C_ADC_VSMF_HV,
				.u3AdcVref = C_ADC_VREF_2_50_V,
				.u5AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK,
				.u1AdcReserved = 0U
			}
		},
		C_ADC_VDDA_EOC,
		C_ADC_VDDD_EOC,
		C_ADC_VAUX_EOC,
		C_ADC_MCUR_EOC,
		C_ADC_MCUR_EOC,
		C_ADC_MCUR_EOC,
		{
			.u16 = C_ADC_EOS
		}
	};
	uint16_t u16LoopCnt;
	uint16_t u16AdcValue;
	ADC_INIT_RESULTS AdcInitResult;
	if(l_u8AdcMode > C_ADC_MODE_IDLE){HAL_ADC_StopSafe();}
	{
		uint16_t *pu16Src = &l_au16AdcSource[0];
		uint16_t *pu16SBase = (uint16_t *)&SBASE_INIT[0];
		*pu16Src++ = (uint16_t)&AdcInitResult.u16AdcVs;
		do
		{
			*pu16Src++ = *pu16SBase++;
		}
		while(pu16SBase < (uint16_t *)&SBASE_INIT[sizeof(SBASE_INIT) / sizeof(uint16_t)]);
	}
	HAL_ADC_PowerOn();
	if((IO_PORT_DRV_OUT & B_PORT_DRV_OUT_ENABLE_DRVSUP)  ==  0U)
	{
		IO_PORT_DRV_OUT  |=  B_PORT_DRV_OUT_ENABLE_DRVSUP;
		DELAY_US(10U);
		ENTER_SECTION(SYSTEM_MODE);
		IO_MLX16_ITC_PEND0_S = B_MLX16_ITC_PEND0_UV_VDDAF;
		IO_MLX16_ITC_MASK0_S  |=  B_MLX16_ITC_MASK0_UV_VDDAF;
		EXIT_SECTION();
	}
	IO_PORT_DRV_OUT  |=  B_PORT_DRV_OUT_ENABLE_CSA;
	IO_PORT_MISC2_OUT  |=  B_PORT_MISC2_OUT_VSM_FILT_ON;
	DELAY_US(1250U);
	HAL_ADC_Stop();
	IO_ADC_SAR_CLK_DIV = ((PLL_FREQ / ADC_FREQ) - 1U);
	u16LoopCnt = C_ADC_SUPPLY_CHECK;
	do
	{
		(void)HAL_ADC_StartSoftTrig(C_ADC_STATE_IDLE);
		if(((AdcInitResult.u16AdcVs - C_OADC) > C_MIN_VS)  &&  ((AdcInitResult.u16AdcVsmF - C_OADC) > C_MIN_VS)){u16LoopCnt--;}
		else
		{
			SetLastError(C_WRN_VS_UV);
			u16LoopCnt = C_ADC_SUPPLY_CHECK;
		}
	}
	while(u16LoopCnt  !=  0U);
	u16AdcValue = (AdcInitResult.u16AdcVdda - C_OADC);
	l_u16VddaADC = u16AdcValue;
	if((u16AdcValue < C_MIN_VDDA)  ||  (u16AdcValue > C_MAX_VDDA))
	{
		g_e8ErrorElectric = (uint8_t)C_ERR_SUP_VDDA;
		if(u16AdcValue < C_MIN_VDDA){SetLastError(C_ERR_VDDA | C_ERR_EXTW | 0x0A00U);}
		else{SetLastError(C_ERR_VDDA | C_ERR_EXTW | 0x0B00U);}
		SetLastError(u16AdcValue);
	}
	u16AdcValue = (AdcInitResult.u16AdcVddd - C_OADC);
	l_u16VdddADC = u16AdcValue;
	if((u16AdcValue < C_MIN_VDDD)  ||  (u16AdcValue > C_MAX_VDDD))
	{
		g_e8ErrorElectric = (uint8_t)C_ERR_SUP_VDDD;
		if(u16AdcValue < C_MIN_VDDD){SetLastError(C_ERR_VDDD | C_ERR_EXTW | 0x0A00U);}
		else{SetLastError(C_ERR_VDDD | C_ERR_EXTW | 0x0B00U);}
		SetLastError(u16AdcValue);
	}
	l_u16CurrentZeroOffset = (AdcInitResult.u16AdcZeroCurrent_1 + AdcInitResult.u16AdcZeroCurrent_2) / 2U;
	HAL_ADC_SetPrio(C_MLX16_ITC_PRIO2_ADC_PRIO4);
}

void ADC_MCurrOffCalib(void)
{
	static const ADC_SDATA_t SBASE_MCURR_OFF_CALIB[] =
	{
		C_ADC_MCUR_EOC,
		C_ADC_MCUR_EOC,
		C_ADC_MCUR_EOC,
		C_ADC_MCUR_EOC,
		C_ADC_MCUR_EOC,
		{
			.u16 = C_ADC_EOS
		}
	};
	ADC_MCURR_RESULTS AdcMCurrResult;
	uint16_t u16LastDrvCtrl;
	if(l_u8AdcMode > C_ADC_MODE_IDLE){HAL_ADC_StopSafe();}
	u16LastDrvCtrl = IO_PORT_DRV_CTRL;
	DRVCFG_GND_TUVW();
	{
		uint16_t *pu16Src = &l_au16AdcSource[0];
		uint16_t *pu16SBase = (uint16_t *)&SBASE_MCURR_OFF_CALIB[0];
		*pu16Src++ = (uint16_t)&AdcMCurrResult.u16AdcZeroCurrent_1;
		do
		{
			*pu16Src++ = *pu16SBase++;
		}
		while(pu16SBase < (uint16_t *)&SBASE_MCURR_OFF_CALIB[sizeof(SBASE_MCURR_OFF_CALIB) / sizeof(uint16_t)]);
	}
	IO_PORT_DRV_OUT  |=  B_PORT_DRV_OUT_ENABLE_CSA;
	DELAY_US(10);
	(void)HAL_ADC_StartSoftTrig(C_ADC_STATE_IDLE);
	l_u16CurrentZeroOffset = ((AdcMCurrResult.u16AdcZeroCurrent_1 + AdcMCurrResult.u16AdcZeroCurrent_2) +
	(AdcMCurrResult.u16AdcZeroCurrent_3 + AdcMCurrResult.u16AdcZeroCurrent_4)) / 4;
	IO_PORT_DRV_CTRL = u16LastDrvCtrl;
}

uint16_t ADC_Conv_Vsupply(void)
{
	return( HAL_ADC_Conv_Vsupply(Get_RawTemperature(), Get_RawVsupplyChip()) );
}

uint16_t ADC_Conv_Vmotor(void)
{
	return( HAL_ADC_Conv_Vmotor(Get_RawTemperature(), Get_RawVmotorF()) );
}

int16_t ADC_Conv_TempJ(uint16_t u16Init)
{
	return( HAL_ADC_Conv_TempJ(Get_RawTemperature(), u16Init) );
}

uint16_t ADC_Conv_Cmotor(void)
{
	return( HAL_ADC_Conv_Cmotor(Get_RawTemperature(), ADC_GetRawMotorDriverCurrent(Get_MicroStepIdx())) );
}

void LinDiag_VddaVddd(void)
{
	uint16_t u16ValueVdda, u16ValueVddd;
	l_u16VddaADC = Get_AdcVdda();
	l_u16VdddADC = Get_AdcVddd();
	u16ValueVdda = (uint16_t) (p_MulU32_U16byU16(l_u16VddaADC, C_GADC_VDDA)  >>  10U);
	u16ValueVddd = (uint16_t) (p_MulU32_U16byU16( (l_u16VdddADC + C_OADC_VDDD), C_GADC_VDDD)  >>  10U);
	StoreD1to4(u16ValueVdda, u16ValueVddd);
}
