#ifndef DRIVE_LIB_ADC_H_
#define DRIVE_LIB_ADC_H_
#include "../AppBuild.h"
#include "../hal_lib/hal_ADC.h"
#define C_ADC_DNL 1U
#define C_ADC_INL 3U
#define C_ADC_ERROR (C_ADC_DNL + C_ADC_INL)
#define C_MAX_NOISE_VS 10U
#define C_MIN_VS (146U - C_ADC_ERROR)
#define C_MIN_VS_wNOISE (146U - C_ADC_ERROR)
#define C_MIN_VS_EEWRT (146U - C_ADC_ERROR)
#define C_MIN_VDDA (643U - C_ADC_ERROR)
#define C_MAX_VDDA (707U + C_ADC_ERROR)
#define C_ABS_MAX_VDDA (738U + C_ADC_ERROR)
#define C_MIN_VDDD (734U - C_ADC_ERROR)
#define C_MAX_VDDD (800U + C_ADC_ERROR)
#define C_ABS_MAX_VDDD (798U + C_ADC_ERROR)
#define C_MIN_VBGD (457U - C_ADC_ERROR)
#define C_MAX_VBGD (513U + C_ADC_ERROR)
#define C_MAX_IO_LOW (120U + C_ADC_ERROR)
#define C_MIN_IO_HIGH (722U - C_ADC_ERROR)
#define C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK C_ADC_HW_TRIGGER_EOCpls6xADC_CLOCK
typedef struct _ADC_INIT_RESULTS
{
	uint16_t u16AdcVs;
	uint16_t u16AdcVsmF;
	uint16_t u16AdcVdda;
	uint16_t u16AdcVddd;
	uint16_t u16AdcVaux;
	uint16_t u16AdcZeroCurrent_Dummy;
	uint16_t u16AdcZeroCurrent_1;
	uint16_t u16AdcZeroCurrent_2;
	uint16_t u16CRC;
}

ADC_INIT_RESULTS;
typedef struct _ADC_MCURR_RESULTS
{
	uint16_t u16AdcZeroCurrent_Dummy;
	uint16_t u16AdcZeroCurrent_1;
	uint16_t u16AdcZeroCurrent_2;
	uint16_t u16AdcZeroCurrent_3;
	uint16_t u16AdcZeroCurrent_4;
	uint16_t u16CRC;
}

ADC_MCURR_RESULTS;
#pragma space dp
extern uint16_t l_u16CurrentZeroOffset;
#pragma space none
#pragma space nodp
#pragma space none
extern void ADC_Init(void);
extern void ADC_MCurrOffCalib(void);
extern uint16_t ADC_Conv_Vsupply(void);
extern uint16_t ADC_Conv_Vmotor(void);
extern int16_t ADC_Conv_TempJ(uint16_t u16Init);
extern uint16_t ADC_Conv_Cmotor(void);
extern void LinDiag_VddaVddd(void);
static inline uint16_t Get_CurrentZeroOffset(void)
{
	return(l_u16CurrentZeroOffset);
}

#endif
