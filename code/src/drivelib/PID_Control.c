#include "AppBuild.h"
#include "../ActADC.h"
#include "drivelib/NV_Functions.h"
#include "drivelib/ErrorCodes.h"
#include "drivelib/GlobalVars.h"
#include "drivelib/MotorDriver.h"
#include "drivelib/PID_Control.h"
#include "drivelib/Timer.h"
#include "camculib/private_mathlib.h"
#define C_REF_VOLTAGE 1200U
#define C_TARGET_LA_CLSU 16384U
#define PID_STATIC static
#pragma space dp
uint16_t l_u16PidCtrlRatio;
PID_STATIC uint16_t l_u16MotorRefVoltageADC = (uint16_t)((12U * 1024U) / (2.5 * C_ADC_HV_DIV));
PID_STATIC uint16_t l_u16MinCorrectionRatio;
PID_STATIC uint16_t l_u16MaxCorrectionRatio;
#pragma space none
#pragma space nodp
uint16_t l_u16PID_CtrlCounter = 0U;
uint16_t l_u16PidHoldingThreshold;
#if ((_SUPPORT_FOC_MODE != FOC_MODE_NONE) && ((_SUPPORT_CLOSED_LOOP_STARTUP == FALSE) || (_SUPPORT_TRIAXIS_MLX9038x != FALSE))) || ((_SUPPORT_FOC_MODE == FOC_MODE_NONE) && ((_SUPPORT_HALL_LATCH == FALSE) || (_SUPPORT_NR_OF_HL == 1) || (_SUPPORT_HALL_LATCH_DIAG != FALSE)))
static uint16_t l_u16PidLosses = 0U;
#endif
static uint16_t l_u16SpeedRampDownLimit;
static uint16_t l_u16SpeedRampUpLimit;
PID_PARAMS_t sPIDpSE2VA =
{
	.i16CoefI = 0,
	.u32SumError = 0U,
	.u32SumErrorMax = (65535UL  <<  C_GN_PID),
	.i16CoefP = 0,
	.i16PrevError = 0,
	.i16CoefD = 0,
	.u16MinOutput = 0U,
	.u32MaxOutput = 65535UL
};
uint16_t l_u16ActCurrRunMax_mA = 0U;
uint16_t l_u16ActCurrRunMax_LSB;
#if (_SUPPORT_FOC_MODE != FOC_MODE_NONE) || ((_SUPPORT_PWM_MODE == SINGLE_COIL_PWM_BIPOLAR) && (_SUPPORT_SPEED_CTRL != FALSE)) || ((_SUPPORT_HALL_LATCH != FALSE) && (_SUPPORT_NR_OF_HL > 1) && (_SUPPORT_HALL_LATCH_DIAG == FALSE))
static uint16_t l_u16ActCurrRunMin_LSB;
#endif
#pragma space none
void PID_SetRunningCurrent(uint16_t u16CurrentLevel)
{
	l_u16ActCurrRunMax_mA = u16CurrentLevel;
	l_u16ActCurrRunMax_LSB = p_MulDivU16_U16byU16byU16(l_u16ActCurrRunMax_mA, C_GMCURR_DIV, Get_MCurrGain());
}

void PID_Init(void)
{
	PID_SetRunningCurrent(NV_RUNNING_CURR_LEVEL);
#if (_SUPPORT_FOC_MODE != FOC_MODE_NONE) || ((_SUPPORT_PWM_MODE == SINGLE_COIL_PWM_BIPOLAR) && (_SUPPORT_SPEED_CTRL != FALSE)) || ((_SUPPORT_HALL_LATCH != FALSE) && (_SUPPORT_NR_OF_HL > 1) && (_SUPPORT_HALL_LATCH_DIAG == FALSE))
	l_u16ActCurrRunMin_LSB = p_MulDivU16_U16byU16byU16(NV_RUNNING_CURR_MIN, C_GMCURR_DIV, Get_MCurrGain());
#endif
	l_u16PidHoldingThreshold = p_MulDivU16_U16byU16byU16(NV_HOLDING_CURR_LEVEL, C_GMCURR_DIV, Get_MCurrGain());
	l_u16MotorRefVoltageADC =
	p_MulDivU16_U16byU16byU16(NV_VSUP_REF, C_VOLTGAIN_DIV, Get_MotorVoltGainF()) + Get_VsmOffset();
	l_u16MinCorrectionRatio = NV_MIN_CORR_RATIO;
	l_u16MaxCorrectionRatio = NV_MAX_CORR_RATIO_PID;
	if(NV_PID_RAMP_DOWN  !=  0U){l_u16SpeedRampDownLimit = (uint16_t)(p_MulU32_U16byU16(g_u16MaxSpeedRPM, NV_PID_RAMP_DOWN)  >>  10);}
	else{l_u16SpeedRampDownLimit = (g_u16MaxSpeedRPM  >>  2);}
	if(NV_PID_RAMP_UP  !=  0U){l_u16SpeedRampUpLimit = (uint16_t)(p_MulU32_U16byU16(g_u16MaxSpeedRPM, NV_PID_RAMP_UP)  >>  8);}
	else{l_u16SpeedRampUpLimit = g_u16MaxSpeedRPM;}
	if(l_u16SpeedRampUpLimit > 32767U){l_u16SpeedRampUpLimit = 32767U;}
	sPIDpSE2VA.i16CoefP = (int16_t)(NV_PID_COEF_P  <<  (C_GN_PID - 10U));
	sPIDpSE2VA.i16CoefI = (int16_t)(NV_PID_COEF_I  <<  (C_GN_PID - 10U));
	sPIDpSE2VA.i16CoefD = (int16_t)(NV_PID_COEF_D  <<  (C_GN_PID - 10U));
	sPIDpSE2VA.u16MinOutput = l_u16MinCorrectionRatio;
	sPIDpSE2VA.u32MaxOutput = (uint32_t)l_u16MaxCorrectionRatio;
	sPIDpSE2VA.u32SumErrorMax = (((uint32_t)l_u16MaxCorrectionRatio)  <<  C_GN_PID);
}

uint16_t PID_Start(uint16_t u16Losses, uint16_t u16Bemf)
{
	uint16_t u16MotorVoltage = Get_MotorVoltage();
	uint16_t u16ReferenceVoltage = NV_VSUP_REF;
	uint16_t u16CorrectionRatio;
	if(u16ReferenceVoltage  ==  0U){u16ReferenceVoltage = C_REF_VOLTAGE;}
#if ((_SUPPORT_FOC_MODE != FOC_MODE_NONE) && ((_SUPPORT_CLOSED_LOOP_STARTUP == FALSE) || (_SUPPORT_TRIAXIS_MLX9038x != FALSE))) || ((_SUPPORT_FOC_MODE == FOC_MODE_NONE) && ((_SUPPORT_HALL_LATCH == FALSE) || (_SUPPORT_NR_OF_HL == 1) || (_SUPPORT_HALL_LATCH_DIAG != FALSE)))
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__)
	l_u16PidLosses =
	p_MulDivU16_U16byU16byU16(u16Losses, (PWM_REG_PERIOD  <<  C_PID_FACTOR),
	u16ReferenceVoltage);
#if (_SUPPORT_PWM_MODE != SINGLE_COIL_PWM_BIPOLAR) && ((_SUPPORT_PWM_MODE != SINGLE_COIL_PWM) || (C_NR_OF_DC_MOTORS != 1))
	if(l_u16PidLosses < ((2U * C_PWM_MIN_DC)  <<  C_PID_FACTOR)){l_u16PidLosses = ((2U * C_PWM_MIN_DC)  <<  C_PID_FACTOR);}
#endif
#elif defined (__MLX81160__) || defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
	l_u16PidLosses = p_MulDivU16_U16byU16byU16(u16Losses, (PWM_REG_PERIOD  <<  C_PID_FACTOR), u16ReferenceVoltage);
#endif
	l_u16PidCtrlRatio = l_u16PidLosses + p_MulDivU16_U16byU16byU16(u16Bemf,
	(PWM_REG_PERIOD  <<  C_PID_FACTOR),
	u16ReferenceVoltage);
#else
	{
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__)
		uint16_t u16PidLosses = p_MulDivU16_U16byU16byU16(u16Losses,
		(PWM_REG_PERIOD  <<  C_PID_FACTOR),
		u16ReferenceVoltage) + ((2U * C_PWM_MIN_DC)  <<  C_PID_FACTOR);
#elif defined (__MLX81160__) || defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
		uint16_t u16PidLosses = p_MulDivU16_U16byU16byU16(u16Losses,
		(PWM_REG_PERIOD  <<  C_PID_FACTOR),
		u16ReferenceVoltage);
#endif
		l_u16PidCtrlRatio = u16PidLosses + p_MulDivU16_U16byU16byU16(u16Bemf,
		(PWM_REG_PERIOD  <<  C_PID_FACTOR),
		u16ReferenceVoltage);
	}
#endif
	if(l_u16PidCtrlRatio > l_u16MaxCorrectionRatio){l_u16PidCtrlRatio = l_u16MaxCorrectionRatio;}
	sPIDpSE2VA.u32SumError = (((uint32_t)l_u16PidCtrlRatio)  <<  C_GN_PID);
	sPIDpSE2VA.i16PrevError = 0;
	l_u16PID_CtrlCounter = 0U;
	if(u16MotorVoltage  !=  0U)
	{
		u16CorrectionRatio = p_MulDivU16_U16byU16byU16(l_u16PidCtrlRatio, u16ReferenceVoltage, u16MotorVoltage);
		if(u16CorrectionRatio > l_u16MaxCorrectionRatio){u16CorrectionRatio = l_u16MaxCorrectionRatio;}
	}
	else{u16CorrectionRatio = l_u16PidCtrlRatio;}
	return(u16CorrectionRatio);
}

#if ((_SUPPORT_FOC_MODE != FOC_MODE_NONE) && ((_SUPPORT_CLOSED_LOOP_STARTUP == FALSE) || (_SUPPORT_TRIAXIS_MLX9038x != FALSE))) || ((_SUPPORT_FOC_MODE == FOC_MODE_NONE) && ((_SUPPORT_HALL_LATCH == FALSE) || (_SUPPORT_NR_OF_HL == 1) || (_SUPPORT_HALL_LATCH_DIAG != FALSE)))
void PID_SpeedCompensate(uint16_t u16NewSpeed, uint16_t u16OldSpeed)
{
	if(l_u16PidCtrlRatio > l_u16PidLosses)
	{
		l_u16PidCtrlRatio = l_u16PidLosses + p_MulDivU16_U16byU16byU16( (l_u16PidCtrlRatio - l_u16PidLosses),
		u16NewSpeed,
		u16OldSpeed);
	}
	else{l_u16PidCtrlRatio = l_u16PidLosses;}
#if ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_IB_IPK)
	sPID_IpkCtrl.u32SumError = (((uint32_t)l_u16PidCtrlRatio)  <<  C_GN_PID);
#elif ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_ID_IQ)
#else
	sPIDpSE2VA.u32SumError = (((uint32_t)l_u16PidCtrlRatio)  <<  C_GN_PID);
#endif
}

#endif
uint16_t VoltageCorrection(void)
{
	register uint16_t u16NewCorrectionRatio = l_u16PidCtrlRatio;
	register uint16_t u16MotorVoltageADC = Get_RawVmotorF();
	if((u16MotorVoltageADC > 0U)  &&  (l_u16MotorRefVoltageADC > 0U))
	{
		u16NewCorrectionRatio = p_MulDivU16_U16byU16byU16(l_u16MotorRefVoltageADC,
		u16NewCorrectionRatio,
		u16MotorVoltageADC);
		if((g_e8MotorStatus & C_MOTOR_STATUS_STOP_MASK)  !=  C_MOTOR_STATUS_STOP)
		{
			if(u16NewCorrectionRatio > l_u16MaxCorrectionRatio){u16NewCorrectionRatio = l_u16MaxCorrectionRatio;}
			else if(u16NewCorrectionRatio < l_u16MinCorrectionRatio){u16NewCorrectionRatio = l_u16MinCorrectionRatio;}
			else{}
		}
		else if(u16NewCorrectionRatio < NV_MIN_HOLDCORR_RATIO){u16NewCorrectionRatio = NV_MIN_HOLDCORR_RATIO;}
		else{}
	}
	return(u16NewCorrectionRatio);
}

static uint16_t PID_Control_Period(void)
{
	uint16_t u16PidPeriod = 0U;
	if((g_e8MotorStatus & C_MOTOR_STATUS_STOP_MASK)  ==  C_MOTOR_STATUS_STOP){u16PidPeriod = (NV_PID_HOLDINGCTRL_PER * PI_TICKS_PER_MILLISECOND);}
	else if((NV_PID_RUNNINGCTRL_PER_UNIT  ==  C_PID_PERIOD_UNIT_TIME)){u16PidPeriod = (NV_PID_RUNNINGCTRL_PER * PI_TICKS_PER_MILLISECOND);}
	else{u16PidPeriod = (uint16_t) (p_MulU32_U16byU16(Get_CommutTimerPeriod(), NV_PID_RUNNINGCTRL_PER)  >>  7U) + 1U;}
	return(u16PidPeriod);
}

static int16_t PID_Control_Error(void)
{
	int16_t i16ControlError = 0;
#if (_SUPPORT_HALL_LATCH != FALSE) && (_SUPPORT_NR_OF_HL > 1) && (_SUPPORT_HALL_LATCH_DIAG == FALSE)
#if (_SUPPORT_PID_SPEED_CURRENT != FALSE)
	if((g_e8MotorStatus & C_MOTOR_STATUS_RUNNING)  !=  0U)
	{
		int16_t i16ErrorI, i16ErrorS, i16Divisor;
		if(Get_NrOfCommut() < 6U){i16ErrorS = (int16_t)g_u16TargetMotorSpeedRPM;}
		else{i16ErrorS = (int16_t)(g_u16TargetMotorSpeedRPM - g_u16ActualMotorSpeedRPM);}
#if (_SUPPORT_BRAKING != FALSE)
		if((g_e8MotorStatus & C_MOTOR_STATUS_STOP_MASK)  ==  C_MOTOR_STATUS_BRAKING){i16ErrorS = -i16ErrorS;}
#endif
		if(i16ErrorS  >=  0){i16ErrorI = (int16_t)(l_u16ActCurrRunMax_LSB - Get_MotorCurrentLPF());}
		else{i16ErrorI = (int16_t)(Get_MotorCurrentLPF() - l_u16ActCurrRunMin_LSB);}
		if(i16ErrorI < 0){i16ErrorI = 0;}
		i16Divisor = (int16_t)l_u16ActCurrRunMax_LSB;
		i16ControlError = p_MulDivI16_I16byI16byI16(i16ErrorI, i16ErrorS, i16Divisor);
	}
#else
	if((g_e8MotorStatus & C_MOTOR_STATUS_RUNNING)  !=  0U)
	{
		if(Get_NrOfCommut() < 6U){i16ControlError = (int16_t)g_u16TargetMotorSpeedRPM;}
		else{i16ControlError = (int16_t)(g_u16TargetMotorSpeedRPM - g_u16ActualMotorSpeedRPM);}
#if (_SUPPORT_BRAKING != FALSE)
		if((g_e8MotorStatus & C_MOTOR_STATUS_STOP_MASK)  ==  C_MOTOR_STATUS_BRAKING){i16ControlError = -i16ControlError;}
#endif
	}
#endif
	else
#endif
	{
		if((g_e8MotorStatus & C_MOTOR_STATUS_STOP_MASK)  ==  C_MOTOR_STATUS_STOP)
		{
			i16ControlError = (int16_t)(l_u16PidHoldingThreshold - Get_MotorCurrentLPF());
			sPIDpSE2VA.u16MinOutput = NV_MIN_HOLDCORR_RATIO;
		}
		else{i16ControlError = (int16_t)(l_u16ActCurrRunMax_LSB - Get_MotorCurrentLPF());}
	}
	return(i16ControlError);
}

void PID_Control(void)
{
	if((g_e8MotorStatus & C_MOTOR_STATUS_MASK)  !=  C_MOTOR_STATUS_STOP)
	{
		uint16_t u16PidPeriod = PID_Control_Period();
		if((u16PidPeriod  !=  0U)  &&  (l_u16PID_CtrlCounter  >=  u16PidPeriod))
		{
			int16_t i16ControlError;
			sPIDpSE2VA.u16MinOutput = l_u16MinCorrectionRatio;
			i16ControlError = PID_Control_Error();
			if(i16ControlError  !=  0){l_u16PidCtrlRatio = (uint16_t)p_PID_Control(i16ControlError, (void *)&sPIDpSE2VA);}
			l_u16PID_CtrlCounter = 0U;
			if((g_e8MotorStatus  ==  C_MOTOR_STATUS_HOLD)  &&  (Get_MotorHoldingCurrState()  !=  FALSE))
			{
				Set_CorrectionRatio(VoltageCorrection());
				MotorDriver_4Phase(Get_MicroStepIdx());
			}
		}
	}
}
