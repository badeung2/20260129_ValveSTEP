#include "AppBuild.h"
#include "hal_ADC.h"
#include <sys_tools.h>

#pragma space dp
#pragma space none
#pragma space nodp
uint16_t l_au16AdcSource[C_ADC_SBASE_LEN];
uint8_t l_u8AdcMode = (uint8_t)C_ADC_MODE_OFF;
#pragma space none
void HAL_ADC_StopSafe(void)
{
	HAL_ADC_Stop();
	NOP();
	while((IO_ADC_STATUS & M_ADC_STATE)  !=  C_ADC_STATE_IDLE)
	{
		HAL_ADC_Start();
		NOP();
		HAL_ADC_Stop();
		NOP();
	}
	IO_ADC_STATUS = B_ADC_ABORTED;
	l_u8AdcMode = C_ADC_MODE_IDLE;
	HAL_ADC_DisableIRQ();
}

void HAL_ADC_PowerOff(void)
{
	HAL_ADC_StopSafe();
	ENTER_SECTION(SYSTEM_MODE);
	IO_PORT_ADC_CTRL_S = 0U;
	EXIT_SECTION();
	l_u8AdcMode = (uint8_t)C_ADC_MODE_OFF;
}

void HAL_ADC_PowerOn(void)
{
	ENTER_SECTION(SYSTEM_MODE);
	IO_PORT_ADC_CTRL_S = B_PORT_ADC_CTRL_ADC_EN;
	EXIT_SECTION();
}

uint16_t HAL_ADC_StartSoftTrig(uint16_t u16NextState)
{
	uint16_t u16Result = C_HAL_ERR_NONE;
	if((IO_ADC_CTRL & B_ADC_START)  ==  0U)
	{
		IO_ADC_SBASE = (uint16_t)&l_au16AdcSource[0];
		IO_ADC_CTRL = (
		C_ADC_ASB_NEVER |
		C_ADC_INT_SCHEME_NOINT |
		B_ADC_SATURATE |
		B_ADC_NO_INTERLEAVE |
		C_ADC_SOC_SOURCE_HARD_CTRIG |
		C_ADC_SOS_SOURCE_SOFT_TRIG);
		HAL_ADC_ClearErrors();
		HAL_ADC_Start();
	}
	DELAY_US(C_ADC_SETTLING_TIME);
	IO_ADC_STATUS = B_ADC_SW_TRIG;
	while((IO_ADC_CTRL & B_ADC_STOP)  ==  0U)
	{
		if((IO_ADC_STATUS & (B_ADC_ABORTED | M_ADC_STATE))  ==  u16NextState){break;}
		if((IO_ADC_STATUS & B_ADC_ABORTED)  !=  0U)
		{
			u16Result = C_HAL_ERR_ADC;
			break;
		}
	}
	l_u8AdcMode = (uint8_t)C_ADC_MODE_IDLE;
	return(u16Result);
}
