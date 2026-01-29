#ifndef ACT_ADC_H_
#define ACT_ADC_H_
#include "AppBuild.h"
#include "drivelib/ADC.h"

typedef enum __attribute__((packed))
{
	C_ADC_MODE_ON_STEPPER = 3U,
	C_ADC_MODE_ON_BEMF
}

ADC_MODE;
typedef struct _ADC_RESULTS
{
	uint16_t u16AdcVs;
	uint16_t u16AdcVdda;
	uint16_t u16AdcVddd;
	uint16_t u16AdcCurrA;
	uint16_t u16AdcVsmF;
	uint16_t u16AdcTj;
	uint16_t u16AdcCurrB;
	uint16_t u16CRC;
}

ADC_RESULTS;
#pragma space dp
extern int16_t g_i16MotorCurrentCoilA;
extern int16_t g_i16MotorCurrentCoilB;
#define g_i16MotorCurrentCoilY g_i16MotorCurrentCoilB
extern uint16_t g_u16MotorCurrentPeak;
extern int16_t g_i16MotorCurrentAngle;
#pragma space none
#pragma space nodp
extern volatile ADC_RESULTS l_AdcResult;
#pragma space none
extern void ADC_Start(uint16_t u16IrqEna);
extern uint16_t ADC_GetRawMotorDriverCurrent(uint16_t u16MicroStepIdx);
extern void ADC_MeasureVsupplyAndTemperature(void);
extern uint16_t ADC_GetNewSampleVsupply(void);
static inline uint16_t Get_RawVsupplyChip(void)
{
	extern uint16_t l_u16HighVoltOffset;
	{
		return(l_AdcResult.u16AdcVs - l_u16HighVoltOffset);
	}
}

static inline void Set_RawVsupplyChip(uint16_t u16Value)
{
	{
		l_AdcResult.u16AdcVs = u16Value;
	}
}

static inline uint16_t Get_RawVmotorF(void)
{
	{
		uint16_t u16RawVSMF = 0U;
		if((int16_t)l_AdcResult.u16AdcVsmF > Get_VsmOffset()){u16RawVSMF = (uint16_t)(l_AdcResult.u16AdcVsmF - Get_VsmOffset());}
		return(u16RawVSMF);
	}
}

static inline uint16_t Get_RawTemperature(void)
{
	{
		return(l_AdcResult.u16AdcTj);
	}
}

static inline void Set_RawTemperature(uint16_t u16Value)
{
	{
		l_AdcResult.u16AdcTj = u16Value;
	}
}

static inline uint16_t Get_AdcVdda(void)
{
	{
		return(l_AdcResult.u16AdcVdda - C_OADC);
	}
}

static inline uint16_t Get_AdcVddd(void)
{
	{
		return(l_AdcResult.u16AdcVddd - C_OADC);
	}
}

#if (_SUPPORT_IO_DUT_SELECT_HVIO != FALSE) && (HVIO_DUT_SELECT == PIN_FUNC_IO_0)
static inline uint16_t Get_IO0HV(void)
{
	return(l_AdcResult.u16AdcIO0HV);
}

#endif
#endif
