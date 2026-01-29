#ifndef HAL_LIB_ADC_H
#define HAL_LIB_ADC_H
#include "../AppBuild.h"
#include "hal_ADC_channels.h"
#include "hal_ADC_inline.h"
#define C_ADC_SBASE_LEN 22U
#define C_ADC_MODE_OFF 0U
#define C_ADC_MODE_IDLE 1U
#define C_ADC_MODE_RUN_HW 2U
#define C_ADC_MODE_LINAA 3U
#define C_ADC_MODE_MOV_DET 4U
#define C_ADC_MODE_WINDMILL 5U
#define C_ADC_SETTLING_TIME 5U
#define C_HAL_ERR_NONE 0U
#define C_HAL_ERR_ADC 1U
#define C_HAL_ERR_TO_ADC_STOP 2U
#define C_GTEMP_SDIV 7U
#define C_GTEMP_DIV (1U << C_GTEMP_SDIV)
#define C_VOLTGAIN_SDIV 5U
#define C_LVOLTGAIN_SDIV 6U
#define C_VOLTGAIN_DIV (1U << C_VOLTGAIN_SDIV)
#define C_LVOLTGAIN_DIV (1U << C_LVOLTGAIN_SDIV)
#define C_GMCURR_SDIV 5U
#define C_GMCURR_DIV (1U << C_GMCURR_SDIV)
#pragma space dp
#pragma space none
#pragma space nodp
extern uint8_t l_u8AdcMode;
extern uint16_t l_au16AdcSource[C_ADC_SBASE_LEN];
#pragma space none
extern void HAL_ADC_StopSafe(void);
extern void HAL_ADC_PowerOff(void);
extern void HAL_ADC_PowerOn(void);
extern uint16_t HAL_ADC_StartSoftTrig(uint16_t u16NextState);
extern void HAL_ADC_Conv_Init(void);
extern uint16_t HAL_ADC_Conv_Vsupply(uint16_t u16TempJ, uint16_t u16SupplyVoltage);
extern uint16_t HAL_ADC_Conv_Vmotor(uint16_t u16TempJ, uint16_t u16MotorVoltage);
extern int16_t HAL_ADC_Conv_TempJ(uint16_t u16TempJ, uint16_t u16Init);
extern uint16_t HAL_ADC_Conv_Cmotor(uint16_t u16TempJ, uint16_t u16MotorCurrent);
static inline uint16_t Get_SupplyVoltage(void)
{
	extern volatile uint16_t l_u16SupplyVoltage;
	return(l_u16SupplyVoltage);
}

static inline uint16_t Get_MotorVoltage(void)
{
	extern volatile uint16_t l_u16MotorVoltage;
	return(l_u16MotorVoltage);
}

static inline uint16_t Get_HighVoltGain(void)
{
	extern uint16_t l_u16HighVoltGain;
	return(l_u16HighVoltGain);
}

static inline uint16_t Get_HighVoltOffset(void)
{
	extern uint16_t l_u16HighVoltOffset;
	return(l_u16HighVoltOffset);
}

static inline void Set_HighVoltOffset(uint16_t u16Value)
{
	extern uint16_t l_u16HighVoltOffset;
	l_u16HighVoltOffset = u16Value;
}

static inline uint16_t Get_LowVoltOffset(void)
{
	extern uint16_t l_u16LowVoltOffset;
	return(l_u16LowVoltOffset);
}

static inline uint16_t Get_MotorVoltGainF(void)
{
	extern uint16_t l_u16VsmGain;
	return(l_u16VsmGain);
}

static inline int16_t Get_VsmOffset(void)
{
	extern int16_t l_i16VsmOffset;
	return(l_i16VsmOffset);
}

static inline void Set_VsmOffset(int16_t i16Value)
{
	extern int16_t l_i16VsmOffset;
	l_i16VsmOffset = i16Value;
}

static inline int16_t Get_ChipTemperature(void)
{
	extern volatile int16_t l_i16ChipTemperature;
	return(l_i16ChipTemperature);
}

static inline uint16_t Get_TempMidADC(void)
{
	extern uint16_t l_u16TempMidADC;
	return(l_u16TempMidADC);
}

static inline uint16_t Get_MotorCurrent_mA(void)
{
	extern volatile uint16_t l_u16MotorCurrent_mA;
	return(l_u16MotorCurrent_mA);
}

static inline uint16_t Get_MCurrGain(void)
{
	extern uint16_t l_u16MCurrGain;
	return(l_u16MCurrGain);
}

#endif
