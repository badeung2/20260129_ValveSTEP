#include "AppBuild.h"
#include "hal_ADC.h"
#include "camculib/private_mathlib.h"
#define C_TEMPCORR_BASE 256UL
#define C_TEMPCORR_HV_DIV ((1UL << (11U + 7U)) / (C_TEMPCORR_BASE * C_VOLTGAIN_DIV))
#define C_TEMPCORR_MC_DIV ((1UL << (10U + 7U)) / (C_TEMPCORR_BASE * C_GMCURR_DIV))
#define C_VOLTGAIN_DIV_TC (C_TEMPCORR_BASE * C_VOLTGAIN_DIV)
#define C_GMCURR_DIV_TC (C_TEMPCORR_BASE * C_GMCURR_DIV)
#define C_TEMP_LPF_COEF 1024
#define C_DEBFLT_ERR_TEMP_SENSOR 0x20U
#define C_TEMPERATURE_JUMP 25
#define C_ERR_OTEMP_ERROR ((uint8_t) 0x80U)
#define _FIX_VSMF_OFFSET FALSE
#define C_VSMF_CSA_OC_CORRECTION 0U
extern volatile uint8_t g_e8ErrorOverTemperature;
#pragma space dp
volatile int16_t l_i16ChipTemperature = (255 - C_TEMPOFF);
volatile uint16_t l_u16MotorVoltage = 1200U;
int16_t l_i16VsmOffset = C_OADC_VSMF;
#pragma space none
#pragma space nodp
volatile uint16_t l_u16SupplyVoltage = 1200U;
volatile uint16_t l_u16MotorCurrent_mA = 0U;
uint16_t l_u16MCurrGain = C_GADC_MCUR;
uint16_t l_u16VsmGain = C_GADC_VSMF;
uint16_t l_u16HighVoltGain = C_GADC_HV;
uint16_t l_u16HighVoltOffset = C_OADC_HV;
uint16_t l_u16LowVoltOffset = C_OADC_LV;
uint16_t l_u16TempMidADC = C_ADC_TEMP_MID;
static uint16_t l_u16TempGainHigh = C_GADC_TEMP;
static uint16_t l_u16TempGainLow = C_GADC_TEMP;
static uint16_t l_u16RoomTemp = 35U;
#pragma space none
void HAL_ADC_Conv_Init(void)
{
	if(CalibrationParams.u8APP_TRIM18_GainVSMF  !=  0U){l_u16VsmGain = CalibrationParams.u8APP_TRIM18_GainVSMF;}
	l_i16VsmOffset = (CalibrationParams.i8APP_TRIM18_OffHV + CalibrationParams.i8APP_TRIM20_OffVSMF);
	if(CalibrationParams.u8APP_TRIM20_GainHV  !=  0U){l_u16HighVoltGain = CalibrationParams.u8APP_TRIM20_GainHV;}
	l_u16HighVoltOffset = CalibrationParams.i8APP_TRIM18_OffHV;
	l_u16LowVoltOffset = CalibrationParams.i8APP_TRIM19_OffLV;
	if(CalibrationParams.u8APP_TRIM22_GainMCur  !=  0U){l_u16MCurrGain = CalibrationParams.u8APP_TRIM22_GainMCur;}
	if(CalibrationParams.u16APP_TRIM02_OTempCal  !=  0U){l_u16TempMidADC = CalibrationParams.u16APP_TRIM02_OTempCal;}
	if(CalibrationParams.u8APP_TRIM04_GainTempHigh  !=  0U){l_u16TempGainHigh = CalibrationParams.u8APP_TRIM04_GainTempHigh;}
	if(CalibrationParams.u8APP_TRIM04_GainTempLow  !=  0U){l_u16TempGainLow = CalibrationParams.u8APP_TRIM04_GainTempLow;}
	if(CalibrationParams.u8APP_TRIM00_TempMid  !=  0U){l_u16RoomTemp = CalibrationParams.u8APP_TRIM00_TempMid;}
}

uint16_t HAL_ADC_Conv_Vsupply(uint16_t u16TempJ, uint16_t u16SupplyVoltage)
{
	int16_t i16TempDiff = (u16TempJ - l_u16TempMidADC);
	uint16_t u16GainTC = (l_u16HighVoltGain * C_TEMPCORR_BASE);
	if(i16TempDiff > 0)
	{
		u16GainTC  +=
		(int16_t)(p_MulI32_I16byI16(i16TempDiff,
		CalibrationParams.i8APP_TRIM25_GainHV_LowT) / (int16_t)C_TEMPCORR_HV_DIV);
	}
	else
	{
		u16GainTC  +=
		(int16_t)(p_MulI32_I16byI16(i16TempDiff,
		CalibrationParams.i8APP_TRIM25_GainHV_HighT) / (int16_t)C_TEMPCORR_HV_DIV);
	}
	l_u16SupplyVoltage =
	(uint16_t)((p_MulU32_U16byU16(u16SupplyVoltage, u16GainTC) + (C_VOLTGAIN_DIV_TC / 2U)) / C_VOLTGAIN_DIV_TC);
	return(l_u16SupplyVoltage);
}

uint16_t HAL_ADC_Conv_Vmotor(uint16_t u16TempJ, uint16_t u16MotorVoltage)
{
	int16_t i16TempDiff = (u16TempJ - l_u16TempMidADC);
	uint16_t u16GainTC = (l_u16VsmGain * C_TEMPCORR_BASE);
	if(i16TempDiff > 0)
	{
		u16GainTC  +=
		(int16_t)(p_MulI32_I16byI16(i16TempDiff,
		CalibrationParams.i8APP_TRIM25_GainHV_LowT) / (int16_t)C_TEMPCORR_HV_DIV);
	}
	else
	{
		u16GainTC  +=
		(int16_t)(p_MulI32_I16byI16(i16TempDiff,
		CalibrationParams.i8APP_TRIM25_GainHV_HighT) / (int16_t)C_TEMPCORR_HV_DIV);
	}
	l_u16MotorVoltage =
	(uint16_t)((p_MulU32_U16byU16(u16MotorVoltage, u16GainTC) + (C_VOLTGAIN_DIV_TC / 2U)) / C_VOLTGAIN_DIV_TC);
	return(l_u16MotorVoltage);
}

int16_t HAL_ADC_Conv_TempJ(uint16_t u16TempJ, uint16_t u16Init)
{
	static int32_t i32TemperatureLPF = 0;
	static uint8_t e8ErrorDebounceFilter = FALSE;
	uint16_t u16ChipTemperatureSensor = u16TempJ;
	if((u16ChipTemperatureSensor  !=  C_ADC_MIN)  &&  (u16ChipTemperatureSensor  !=  C_ADC_MAX))
	{
		int16_t i16NewChipTemperature, i16ChipTempDelta;
		if(u16ChipTemperatureSensor < l_u16TempMidADC)
		{
			i16NewChipTemperature = (int16_t)(l_u16RoomTemp +
			p_MulDivU16_U16byU16byU16( (l_u16TempMidADC - u16ChipTemperatureSensor),
			l_u16TempGainHigh, C_GTEMP_DIV));
		}
		else
		{
			i16NewChipTemperature = (int16_t)(l_u16RoomTemp -
			p_MulDivU16_U16byU16byU16( (u16ChipTemperatureSensor - l_u16TempMidADC),
			l_u16TempGainLow, C_GTEMP_DIV));
		}
		if(u16Init  ==  FALSE)
		{
			i16ChipTempDelta = i16NewChipTemperature - l_i16ChipTemperature;
			if(i16ChipTempDelta < 0){i16ChipTempDelta = -i16ChipTempDelta;}
			if(i16ChipTempDelta > C_TEMPERATURE_JUMP)
			{
				if(i16NewChipTemperature > l_i16ChipTemperature){i16NewChipTemperature = l_i16ChipTemperature + 1;}
				else{i16NewChipTemperature = l_i16ChipTemperature - 1;}
			}
		}
		else
		{
			i32TemperatureLPF = ((int32_t)i16NewChipTemperature)  <<  16;
			l_i16ChipTemperature = i16NewChipTemperature;
		}
		l_i16ChipTemperature = p_LpfI16_I16byI16(&i32TemperatureLPF,
		C_TEMP_LPF_COEF,
		(i16NewChipTemperature - l_i16ChipTemperature));
		e8ErrorDebounceFilter  &=  (uint8_t) ~C_DEBFLT_ERR_TEMP_SENSOR;
		g_e8ErrorOverTemperature  &=  ~C_ERR_OTEMP_ERROR;
	}
	else
	{
		if((e8ErrorDebounceFilter & (uint8_t)C_DEBFLT_ERR_TEMP_SENSOR)  !=  0U){g_e8ErrorOverTemperature  |=  C_ERR_OTEMP_ERROR;}
		else{e8ErrorDebounceFilter  |=  (uint8_t)C_DEBFLT_ERR_TEMP_SENSOR;}
	}
	return(l_i16ChipTemperature);
}

uint16_t HAL_ADC_Conv_Cmotor(uint16_t u16TempJ, uint16_t u16MotorCurrent)
{
	int16_t i16TempDiff = (u16TempJ - l_u16TempMidADC);
	uint16_t u16GainTC = (l_u16MCurrGain * C_TEMPCORR_BASE);
	if(i16TempDiff > 0)
	{
		u16GainTC  +=
		(int16_t)(p_MulI32_I16byI16(i16TempDiff,
		CalibrationParams.i8APP_TRIM24_GainMCur_LowT) / (int16_t)C_TEMPCORR_MC_DIV);
	}
	else
	{
		u16GainTC  +=
		(int16_t)(p_MulI32_I16byI16(i16TempDiff,
		CalibrationParams.i8APP_TRIM24_GainMCur_HighT) / (int16_t)C_TEMPCORR_MC_DIV);
	}
	l_u16MotorCurrent_mA =
	(uint16_t)((p_MulU32_U16byU16(u16MotorCurrent, u16GainTC) + (C_GMCURR_DIV_TC / 2U)) / C_GMCURR_DIV_TC);
	return(l_u16MotorCurrent_mA);
}
