#include "AppBuild.h"
#include "../ActADC.h"
#include "drivelib/Diagnostic.h"
#include "drivelib/NV_UserPage.h"
#include "drivelib/GlobalVars.h"
#include "drivelib/MotorDriver.h"
#include "drivelib/MotorDriverTables.h"
#include "drivelib/MotorStall.h"
#include "drivelib/PID_Control.h"
#include "camculib/private_mathlib.h"
#include "drivelib/Timer.h"
#if ((_SUPPORT_HALL_LATCH != FALSE) && (_SUPPORT_NR_OF_HL > 1)) || (_SUPPORT_HALL_LATCH_DIAG != FALSE)
#include "senselib/HallLatch.h"
#endif
#include <atomic.h>
#include <sys_tools.h>
#define FOC_STATIC static
#define C_MAX_POS 0xFFFEU
#define C_SOFT_START_RAMP_STEPS 16U
#define C_OFF_FULL_STEP 1U
#define C_MAX_STEP_POS C_MAX_POS
#pragma space dp
volatile uint16_t l_u16CorrectionRatio = 0U;
volatile uint16_t l_u16StallDetectorDelay = C_DETECTOR_DELAY;
uint16_t l_u16MotorCurrentMovAvgxN = 0U;
uint16_t l_u16MotorCurrentLPF = 0U;
uint16_t l_u16TargetCommutTimerPeriod;
volatile uint16_t l_u16MicroStepIdx = 0U;
uint16_t l_u16CommutTimerPeriod = 0U;
uint16_t l_u16MotorDriverDisconDelay = 0U;
uint16_t l_u16MotorMicroStepsPerElecRotation;
uint16_t l_u16mR_AT = 0U;
uint16_t l_u16mZ = 0U;
volatile E_MOTOR_STARTUP_MODE_t l_e8MotorStartupMode = E_MSM_STOP;
uint8_t l_u8MotorHoldDelay = 0U;
#if (C_MOTOR_PHASES == 3) && (_SUPPORT_PWM_MODE == TRIPLEPHASE_ALLPWM_MIRROR) && (UART_COMM != FALSE) && ((_SUPPORT_UART_IF_APP == C_UART_IF_SCOPE) || (_SUPPORT_UART2_IF_APP == C_UART_IF_SCOPE))
volatile int16_t l_i16MotorVoltageCoilA = 0;
#endif
#pragma space none
#pragma space nodp
MOTOR_PWM_t MotorPwm =
{
	0
};
uint16_t l_u16StallDetectorThrshld;
uint16_t l_u16MotorFullStepsPerElecRotation;
uint16_t l_u16MotorMicroStepsPerMechRotation;
uint16_t g_u16NrOfMicroStepsPerFullStep;
uint16_t l_u16ShaftRatiox512 = 512U;
uint32_t l_u32MicroStepPeriodOneRPM;
uint16_t l_u16AccelerationConst;
uint16_t g_u16ForcedSpeedRPM;
#if (_SUPPORT_HALL_LATCH != FALSE) && (_SUPPORT_NR_OF_HL > 1) && (_SUPPORT_HALL_LATCH_DIAG == FALSE)
uint16_t l_u16LastHallLatchEvent = TRUE;
uint16_t l_u16LastCommutTimerPeriod;
#endif
uint16_t l_u16LowSpeedPeriod;
FOC_STATIC uint32_t l_u32MotorCurrentLPF = 0U;
#if (C_MOTOR_PHASES != 1) && ((_SUPPORT_HALL_LATCH == FALSE) || (_SUPPORT_NR_OF_HL <= 1))
static uint16_t l_u16SpeedUpdateAcc = (C_MICROSTEP_PER_FULLSTEP - 1U);
static uint16_t l_u16SpeedUpdateDec = (C_MICROSTEP_PER_FULLSTEP - 1U);
#endif
static uint16_t l_u16PhaseCoilResistanceRT = (C_FETS_RTOT + 10U);
static uint16_t l_u16PhaseCoilResistanceAT = (C_FETS_RTOT + 10U);
static uint16_t l_au16MotorSpeedRPM[8];
FOC_STATIC uint16_t l_u16RampDownSteps = 0U;
FOC_STATIC uint16_t l_u16DeltaPosition = 0U;
uint8_t l_u8MotorHoldingCurrState = FALSE;
static uint16_t l_u16RampdownTimeout = 0U;
static uint16_t l_u16MotorRewindSteps;
static uint16_t l_au16MotorCurrentRaw[C_MOVAVG_SZ];
#if !defined (C_MOVAVG_SSZ) || (C_MOVAVG_SZ > ((C_MOTOR_PHASES * C_MOTOR_STEP_PER_PHASE) * C_MICROSTEP_PER_FULLSTEP))
static uint16_t l_u16MotorCurrentRawIdx;
#endif
#if (_SUPPORT_HALL_LATCH != FALSE) && (_SUPPORT_NR_OF_HL > 1U)
volatile uint8_t g_u8ZcHallFound = FALSE;
#if (_SUPPORT_MICRO_STEP_COMMUTATION != FALSE)
uint8_t g_u8HallMicroSteps = 0U;
#endif
#if (_SUPPORT_HALL_LATCH_DIAG == FALSE)
static uint32_t l_u32SumCommutTimerPeriods;
uint16_t l_u16NrOfCommut;
static uint16_t l_au16CommutTime[C_NR_OF_FULLSTEPS];
static uint16_t l_u16CommutTimeIdx = 0U;
#else
uint8_t l_au8MotorMicroStep[C_MOTOR_PHASES * C_MOTOR_STEP_PER_PHASE * C_MICROSTEP_PER_FULLSTEP];
#endif
#endif
#pragma space none
void MotorDriverConfig(uint16_t u16State)
{
	static uint16_t u16DriverState = FALSE;
	if((u16State  !=  FALSE)  &&  (u16DriverState  ==  FALSE))
	{
		if((IO_PORT_DRV_OUT & B_PORT_DRV_OUT_ENABLE_DRVSUP)  ==  0U)
		{
			IO_PORT_DRV_OUT = B_PORT_DRV_OUT_ENABLE_DRVSUP;
			DELAY(C_DELAY_10US);
			ENTER_SECTION(SYSTEM_MODE);
			IO_MLX16_ITC_PEND0_S = B_MLX16_ITC_PEND0_UV_VDDAF;
			IO_MLX16_ITC_MASK0_S  |=  B_MLX16_ITC_MASK0_UV_VDDAF;
			EXIT_SECTION();
		}
		IO_PORT_DRV_OUT = (B_PORT_DRV_OUT_ENABLE_LS_OC |
		B_PORT_DRV_OUT_ENABLE_HS_OC |
#if (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM_BIPOLAR) || ((_SUPPORT_PWM_MODE == SINGLE_COIL_PWM) && (C_NR_OF_DC_MOTORS == 1))
		B_PORT_DRV_OUT_PARALLEL_MODE_DRV |
#endif
		(0U  <<  9) |
		B_PORT_DRV_OUT_ENABLE_CSA |
		B_PORT_DRV_OUT_ENABLE_DRVMOD_CPCLK |
		B_PORT_DRV_OUT_ENABLE_DRVSUP);
		DELAY(C_DELAY_10US);
		IO_PORT_SSCM2_CONF  |=  B_PORT_SSCM2_CONF_SSCM2_EN;
		u16DriverState = TRUE;
	}
	else if(u16State  ==  FALSE)
	{
		DRVCFG_TRI_TUVW();
		DRVCFG_DIS_TUVW();
		IO_PORT_DRV_OUT = B_PORT_DRV_OUT_ENABLE_DRVSUP;
		IO_PORT_SSCM2_CONF  &=  ~B_PORT_SSCM2_CONF_SSCM2_EN;
		u16DriverState = FALSE;
	}
	else{}
}

void MotorDriverInit(uint16_t u16FullInit)
{
	l_u16MotorFullStepsPerElecRotation = C_NR_OF_FULLSTEPS;
	g_u16NrOfMicroStepsPerFullStep = C_MICROSTEP_PER_FULLSTEP;
	g_u16MotorPolePairs = NV_POLE_PAIRS;
	l_u16MotorMicroStepsPerElecRotation = (uint16_t)p_MulU32_U16byU16(g_u16NrOfMicroStepsPerFullStep,
	l_u16MotorFullStepsPerElecRotation);
	l_u16MotorMicroStepsPerMechRotation = (uint16_t)p_MulU32_U16byU16(g_u16MotorPolePairs,
	l_u16MotorMicroStepsPerElecRotation);
	if(l_u16MotorMicroStepsPerMechRotation  >=  (uint16_t)(65536UL  >>  5))
	{
		l_u16ShaftRatiox512 =
		p_MulDivU16_U16byU16byU16( (NV_GEARBOX_RATIO  <<  5),
		(l_u16MotorMicroStepsPerMechRotation  <<  4),
		C_SHAFT_STEPS_PER_ROTATION);
	}
	else
	{
		l_u16ShaftRatiox512 =
		p_MulDivU16_U16byU16byU16( (NV_GEARBOX_RATIO  <<  4),
		(l_u16MotorMicroStepsPerMechRotation  <<  5),
		C_SHAFT_STEPS_PER_ROTATION);
	}
	l_u32ActualPosition = ConvShaftSteps2MicroSteps(g_u16ActualPosition);
	g_u32TargetPosition = ConvShaftSteps2MicroSteps(g_u16TargetPosition);
	l_u32MicroStepPeriodOneRPM = p_DivU32_U32byU16( (TIMER_CLOCK * 60UL), l_u16MotorMicroStepsPerMechRotation);
	g_u16MinSpeedRPM = NV_MIN_SPEED;
	g_u16LowSpeedRPM = NV_ACT_SPEED1;
	{
		g_u16StartupSpeedRPM = g_u16MinSpeedRPM;
	}
	g_u16MaxSpeedRPM = NV_ACT_SPEED4;
	l_u16LowSpeedPeriod = p_DivU16_U32byU16(l_u32MicroStepPeriodOneRPM, g_u16StartupSpeedRPM) - 1U;
#if (C_MOTOR_PHASES != 1) && ((_SUPPORT_HALL_LATCH == FALSE) || (_SUPPORT_NR_OF_HL <= 1))
#if (_SUPPORT_FOC_MODE != FOC_MODE_NONE)
#if (_SUPPORT_CLOSED_LOOP_STARTUP != FALSE)
#if (_SUPPORT_AUTO_SPEED_FOC == FALSE)
	l_u16StartrupThrshldSpeedRPMe = (uint16_t)p_MulU32_U16byU16(g_u16LowSpeedRPM, g_u16MotorPolePairs)  >>  1;
#else
	l_u16StartrupThrshldSpeedRPMe = (uint16_t)p_MulU32_U16byU16(g_u16MinSpeedRPM, g_u16MotorPolePairs);
#endif
#if (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
	l_u16SpeedUpdateAcc = (NV_ACCELERATION_STEPS - 1U);
	l_u16SpeedUpdateDec = (NV_DECELERATION_STEPS - 1U);
#endif
#else
	l_u16CloseLoopCommutTimerPeriod = p_DivU16_U32byU16(l_u32MicroStepPeriodOneRPM, g_u16LowSpeedRPM) - 1U;
	if(NV_ACCELERATION_PWR  ==  7U){l_u16SpeedUpdateAcc = ((8U * C_MICROSTEP_PER_FULLSTEP) - 1U);}
	else{l_u16SpeedUpdateAcc = (NV_ACCELERATION_STEPS - 1U);}
	if(NV_DECELERATION_PWR  ==  7U){l_u16SpeedUpdateDec = ((8U * C_MICROSTEP_PER_FULLSTEP) - 1U);}
	else{l_u16SpeedUpdateDec = (NV_DECELERATION_STEPS - 1U);}
	l_u16AccelerationConst = NV_ACCELERATION_CONST;
#endif
#else
	l_u16SpeedUpdateAcc = (NV_ACCELERATION_STEPS - 1U);
	l_u16SpeedUpdateDec = (NV_DECELERATION_STEPS - 1U);
	l_u16AccelerationConst = NV_ACCELERATION_CONST;
#if (_SUPPORT_ACCELERATION == C_ACCELERATION_COSINE_CURVE)
	l_u16AccelerationMin = g_u16MinSpeedRPM;
	l_u16AccelerationMax = NV_ACCELERATION_CONST;
	l_u16AccelerationStep = 3U;
#endif
#endif
#elif (_SUPPORT_HALL_LATCH == FALSE) || (_SUPPORT_NR_OF_HL != 1)
	l_u16AccelerationConst = NV_ACCELERATION_CONST;
#endif
	l_u16MotorRewindSteps = (uint16_t)mulU32_U16byU16(NV_REWIND_STEPS, g_u16NrOfMicroStepsPerFullStep);
	if(l_u16MotorRewindSteps < C_MOVAVG_SZ){l_u16MotorRewindSteps = C_MOVAVG_SZ;}
	l_u16PhaseCoilResistanceRT = p_MulU16lo_U16byU16(NV_MOTOR_COIL_RTOT, 100U) + C_FETS_RTOT;
	l_u16PhaseCoilResistanceAT = l_u16PhaseCoilResistanceRT;
	l_au16MotorSpeedRPM[0] = g_u16MinSpeedRPM;
	l_au16MotorSpeedRPM[1] = g_u16LowSpeedRPM;
	l_au16MotorSpeedRPM[2] = NV_ACT_SPEED2;
	l_au16MotorSpeedRPM[3] = NV_ACT_SPEED3;
	l_au16MotorSpeedRPM[4] = g_u16MaxSpeedRPM;
	l_au16MotorSpeedRPM[5] = g_u16LowSpeedRPM;
	l_au16MotorSpeedRPM[6] = g_u16MinSpeedRPM;
	l_au16MotorSpeedRPM[7] = g_u16MinSpeedRPM;
	if(u16FullInit  !=  FALSE)
	{
		IO_TRIM2_DRV = (IO_TRIM2_DRV & ~(M_TRIM2_DRV_TRIM_CP_SLWRT_DRV | M_TRIM2_DRV_TRIM_SLWRT)) |
		(C_CP_SLWRT_DRV |
		C_DRV_SLWRT);
		l_u16CorrectionRatio = NV_MIN_CORR_RATIO;
		IO_CTIMER0_CTRL = C_TMRx_CTRL_MODE0;
		ENTER_SECTION(SYSTEM_MODE);
		IO_MLX16_ITC_PRIO0_S = (IO_MLX16_ITC_PRIO0_S & ~M_MLX16_ITC_PRIO0_CTIMER0_3) |
		C_MLX16_ITC_PRIO0_CTIMER0_3_PRIO4;
		IO_MLX16_ITC_PEND1_S = B_MLX16_ITC_PEND1_CTIMER0_3;
		IO_MLX16_ITC_MASK1_S  |=  B_MLX16_ITC_MASK1_CTIMER0_3;
		EXIT_SECTION();
		IO_PWM_MASTER1_CTRL = B_PWM_MASTER1_STOP;
		IO_PWM_SLAVE1_CTRL = B_PWM_SLAVE1_STOP;
		IO_PWM_SLAVE2_CTRL = B_PWM_SLAVE2_STOP;
		IO_PWM_SLAVE3_CTRL = B_PWM_SLAVE3_STOP;
		IO_PWM_MASTER2_CTRL = B_PWM_MASTER2_STOP;
		IO_PWM_MASTER1_CTRL = (PWM_PRESCALER  <<  8) | C_PWM_MASTER1_MODE_MIRROR;
		IO_PWM_SLAVE1_CTRL = B_PWM_SLAVE1_SLAVE | C_PWM_SLAVE1_MODE_MIRROR | B_PWM_SLAVE1_POL;
		IO_PWM_SLAVE2_CTRL = B_PWM_SLAVE2_SLAVE | C_PWM_SLAVE2_MODE_MIRROR;
		IO_PWM_SLAVE3_CTRL = B_PWM_SLAVE3_SLAVE | C_PWM_SLAVE3_MODE_MIRROR | B_PWM_SLAVE3_POL;
		IO_PWM_MASTER2_CTRL = B_PWM_MASTER2_SLAVE | C_PWM_MASTER2_MODE_MIRROR;
		IO_PWM_MASTER1_PER = PWM_REG_PERIOD;
		IO_PWM_MASTER1_CMP = (((1UL * PWM_REG_PERIOD) + 4U) / 8U);
		IO_PWM_SLAVE1_CMP = (((2UL * PWM_REG_PERIOD) + 3U) / 6U);
		IO_PWM_SLAVE2_CMP = (((3UL * PWM_REG_PERIOD) + 3U) / 6U) + C_PWM_DCORR;
		IO_PWM_SLAVE3_CMP = (((4UL * PWM_REG_PERIOD) + 3U) / 6U);
		IO_PWM_MASTER2_CMP = C_PWM_DCORR;
		ENTER_SECTION(SYSTEM_MODE);
		IO_MLX16_ITC_PRIO1_S = (IO_MLX16_ITC_PRIO1_S & ~M_MLX16_ITC_PRIO1_PWM_MASTER1_END) |
		C_MLX16_ITC_PRIO1_PWM_MASTER1_END_PRIO4;
		EXIT_SECTION();
		IO_PWM_MASTER1_CTRL = B_PWM_MASTER1_START;
	}
	if(g_u8MotorHoldingCurrEna  !=  l_u8MotorHoldingCurrState){MotorDriverStop( (uint16_t)C_STOP_RAMPDOWN);}
}

void MotorDriverPermanentError(uint8_t u8ElectricErrorCode)
{
	DRVCFG_DIS_TUVW();
	g_e8ErrorElectric  |=  u8ElectricErrorCode;
}

void MotorDriverPosInit(uint16_t u16InitPosition)
{
	l_u32ActualPosition = ConvShaftSteps2MicroSteps(u16InitPosition);
}

void ConvMicroSteps2ShaftSteps(void)
{
	uint32_t u32ActualPosition = l_u32ActualPosition;
	if(u32ActualPosition > C_ZERO_POS_OFFSET)
	{
		g_u16ActualPosition =
		p_DivU16_U32byU16( ((u32ActualPosition - C_ZERO_POS_OFFSET)  <<  9) + ((uint32_t)l_u16ShaftRatiox512  >>  1),
		l_u16ShaftRatiox512);
	}
	else{g_u16ActualPosition = 0U;}
}

uint32_t ConvShaftSteps2MicroSteps(uint16_t u16Position)
{
	return( ((p_MulU32_U16byU16(u16Position, l_u16ShaftRatiox512) + 256U)  >>  9) + C_ZERO_POS_OFFSET);
}

FOC_STATIC uint16_t DeltaPosition(void)
{
	uint16_t u16Result;
	__asm__ __volatile__ (
	"lod AL, dp:_g_e8MotorDirectionCCW\n\t"
	"jne _DP_10\n\t"
	"lod Y, #_g_u32TargetPosition\n\t"
	"lod X, #_l_u32ActualPosition\n\t"
	"jmp _DP_20\n\t"
	"_DP_10:\n\t"
	"lod X, #_g_u32TargetPosition\n\t"
	"lod Y, #_l_u32ActualPosition\n\t"
	"_DP_20:\n\t"
	"mov YA, [Y]\n\t"
	"sub YA, [X]\n\t"
	"jsge _DP_30\n\t"
	"movu YA, #0\n\t"
	"_DP_30:\n\t"
	"cmp Y, #0x0000\n\t"
	"je _DP_40\n\t"
	"mov A, #0xFFFF\n\t"
	"_DP_40:"
	: "=a" (u16Result)
	:
	: "X", "Y"
	);
	return(u16Result);
}

static void MotorDriver_InitialPwmDutyCycle(uint16_t u16CurrentLevel, uint16_t u16MotorSpeed)
{
	uint16_t u16Losses = p_MulDivU16_U16byU16byU16(l_u16PhaseCoilResistanceAT, u16CurrentLevel, 1414U);
	uint16_t u16Bemf = p_MulDivU16_U16byU16byU16(NV_MOTOR_CONSTANT, u16MotorSpeed, (10U * 60U));
	l_u16CorrectionRatio = PID_Start(u16Losses, u16Bemf);
}

void MotorDriver_4Phase(uint16_t u16MicroStepIdx)
{
	int16_t i16Pwm1, i16Pwm2;
	int16_t *pi16Vector = (int16_t *)&c_ai16MicroStepVector4PH[u16MicroStepIdx];
#if (PWM_REG_PERIOD >= (128U << (4U - PWM_PRESCALER_N)))
	i16Pwm1 = (int16_t)(mulI32_I16byU16(*pi16Vector, l_u16CorrectionRatio)  >>  (16 + C_PID_FACTOR));
	pi16Vector  +=  C_MICROSTEP_PER_FULLSTEP;
	i16Pwm2 = (int16_t)(mulI32_I16byU16(*pi16Vector, l_u16CorrectionRatio)  >>  (16 + C_PID_FACTOR));
#elif (C_PID_FACTOR == 4)
	i16Pwm1 = p_MulI16hi_I16byI16asr4(*pi16Vector, (int16_t)l_u16CorrectionRatio);
	pi16Vector  +=  C_MICROSTEP_PER_FULLSTEP;
	i16Pwm2 = p_MulI16hi_I16byI16asr4(*pi16Vector, (int16_t)l_u16CorrectionRatio);
#else
	i16Pwm1 = (int16_t)(mulI16_I16byI16(*pi16Vector, (int16_t)l_u16CorrectionRatio)  >>  C_PID_FACTOR);
	pi16Vector  +=  C_MICROSTEP_PER_FULLSTEP;
	i16Pwm2 = (int16_t)(mulI16_I16byI16(*pi16Vector, (int16_t)l_u16CorrectionRatio)  >>  C_PID_FACTOR);
#endif
	{
		if((u16MicroStepIdx < C_MICROSTEP_PER_FULLSTEP)  ||  (u16MicroStepIdx  >=  (3U * C_MICROSTEP_PER_FULLSTEP)))
		{
			MotorPwm.u16PwmSlave1 = (uint16_t)i16Pwm2;
			MotorPwm.u16PwmSlave3 = 0U;
		}
		else
		{
			i16Pwm2 = (0 - i16Pwm2);
			MotorPwm.u16PwmSlave3 = (uint16_t)i16Pwm2;
			MotorPwm.u16PwmSlave1 = 0U;
		}
	}
	if((u16MicroStepIdx & (2U * C_MICROSTEP_PER_FULLSTEP))  !=  0U)
	{
		i16Pwm1 = 0 - i16Pwm1;
		MotorPwm.u16PwmSlave2 = (uint16_t)(PWM_SCALE_OFFSET - i16Pwm1);
		i16Pwm1 = PWM_REG_PERIOD;
	}
	else
	{
		MotorPwm.u16PwmSlave2 = PWM_REG_PERIOD;
		i16Pwm1 = (PWM_SCALE_OFFSET - i16Pwm1);
	}
	MotorPwm.u16PwmMaster1 = (uint16_t)i16Pwm1;
	HAL_PWM_MasterIrqEnable();
}

__attribute__((interrupt)) void ISR_PWM_MASTER1_END(void)
{
	IO_PWM_SLAVE2_LT = (uint16_t)MotorPwm.u16PwmSlave2;
	IO_PWM_SLAVE3_LT = (uint16_t)MotorPwm.u16PwmSlave3;
	IO_PWM_SLAVE1_LT = (uint16_t)MotorPwm.u16PwmSlave1;
	IO_PWM_MASTER1_LT = (uint16_t)MotorPwm.u16PwmMaster1;
	HAL_PWM_MasterIrqDisable();
}

void MotorDriverCurrentMeasureInit(void)
{
	ENTER_SECTION(ATOMIC_KEEP_MODE);
	{
		l_u16StallDetectorDelay = NV_STALL_DETECTOR_DELAY;
		if(l_u16StallDetectorDelay < C_MOVAVG_SZ){l_u16StallDetectorDelay = C_MOVAVG_SZ;}
		if(l_u16StallDetectorDelay > (2U * C_MOVAVG_SZ)){l_u16StallDetectorThrshld = (l_u16StallDetectorDelay - (2U * C_MOVAVG_SZ));}
		else{l_u16StallDetectorThrshld = 0U;}
		l_u16MotorCurrentLPF = 0U;
		l_u32MotorCurrentLPF = 0U;
#if !defined (C_MOVAVG_SSZ) || (C_MOVAVG_SZ > ((C_MOTOR_PHASES * C_MOTOR_STEP_PER_PHASE) * C_MICROSTEP_PER_FULLSTEP))
		l_u16MotorCurrentRawIdx = 0U;
#endif
		l_u16MotorCurrentMovAvgxN = 0U;
		l_au16MotorCurrentRaw[0] = 0U;
	}
	EXIT_SECTION();
	p_MemSet( (uint16_t *)&l_au16MotorCurrentRaw[0], 0U, (uint16_t)(C_MOVAVG_SZ * sizeof(l_au16MotorCurrentRaw[0])));
}

static void MotorDriverCurrentMeasure(uint16_t u16MicroStepIdx)
{
	static uint16_t u16LastMicroStepIdx = 0U;
	uint16_t u16MicroStepMotorCurrent = ADC_GetRawMotorDriverCurrent(u16MicroStepIdx);
	{
#if defined (C_MOVAVG_SSZ) && (C_MOVAVG_SZ <= ((C_MOTOR_PHASES * C_MOTOR_STEP_PER_PHASE) * C_MICROSTEP_PER_FULLSTEP))
		uint16_t *pu16MotorCurrentElement = &l_au16MotorCurrentRaw[u16MicroStepIdx & (C_MOVAVG_SZ - 1U)];
		uint16_t u16PrevMotorCurrent = *pu16MotorCurrentElement;
#else
		uint16_t *pu16MotorCurrentElement = &l_au16MotorCurrentRaw[l_u16MotorCurrentRawIdx];
		uint16_t u16PrevMotorCurrent = *pu16MotorCurrentElement;
		l_u16MotorCurrentRawIdx++;
		if(l_u16MotorCurrentRawIdx  >=  C_MOVAVG_SZ){l_u16MotorCurrentRawIdx = 0U;}
#endif
		if((l_u16StallDetectorDelay  !=  0U)  ||  (u16PrevMotorCurrent < C_MIN_MOTORCURRENT)  ||  (u16MicroStepMotorCurrent < (u16PrevMotorCurrent  <<  2)))
		{
			l_u16MotorCurrentMovAvgxN  -=  u16PrevMotorCurrent;
			l_u16MotorCurrentMovAvgxN  +=  u16MicroStepMotorCurrent;
			*pu16MotorCurrentElement = u16MicroStepMotorCurrent;
		}
	}
	{
#if defined (C_MOVAVG_SSZ) && (C_MOVAVG_SZ < ((C_MOTOR_PHASES * C_MOTOR_STEP_PER_PHASE) * C_MICROSTEP_PER_FULLSTEP))
		uint16_t u16MotorCurrentAcc = (l_u16MotorCurrentMovAvgxN  >>  C_MOVAVG_SSZ);
#else
		uint16_t u16MotorCurrentAcc = p_DivU16_U32byU16( (uint32_t)l_u16MotorCurrentMovAvgxN, C_MOVAVG_SZ);
#endif
		if((l_u16StallDetectorDelay  <=  l_u16StallDetectorThrshld)  &&  (l_e8MotorStartupMode  !=  E_MSM_STEPPER_D)  &&  ((l_e8MotorStartupMode  !=  E_MSM_STEPPER_A)  ||  (u16MotorCurrentAcc  >=  l_u16MotorCurrentLPF)))
		{
			l_u16MotorCurrentLPF =
			p_LpfU16_I16byI16(&l_u32MotorCurrentLPF, 1024, (int16_t)(u16MotorCurrentAcc - l_u16MotorCurrentLPF));
		}
		else
		{
			l_u16MotorCurrentLPF = u16MotorCurrentAcc;
			l_u32MotorCurrentLPF = ((uint32_t)l_u16MotorCurrentLPF  <<  16U);
		}
	}
	if((l_u16StallDetectorDelay > 0U)  &&  (l_u16MicroStepIdx  !=  u16LastMicroStepIdx))
	{
		l_u16StallDetectorDelay--;
		u16LastMicroStepIdx = l_u16MicroStepIdx;
	}
}

void MotorDriverHoldCurrentMeasure(void)
{
	uint16_t u16HoldMotorCurrent = ADC_GetRawMotorDriverCurrent(l_u16MicroStepIdx);
	l_u16MotorCurrentLPF =
	p_LpfU16_I16byI16(&l_u32MotorCurrentLPF, 1024, (int16_t)(u16HoldMotorCurrent - l_u16MotorCurrentLPF));
	l_u16MotorCurrentMovAvgxN = (l_u16MotorCurrentLPF  <<  C_MOVAVG_SSZ);
}

uint16_t MotorDriverUpdateMicroStepIndex(uint16_t u16MicroStepIdx)
{
	if(g_e8MotorDirectionCCW  !=  ((uint8_t)C_MOTOR_DIR_CW))
	{
		if(u16MicroStepIdx  ==  0U){u16MicroStepIdx = l_u16MotorMicroStepsPerElecRotation;}
		u16MicroStepIdx--;
	}
	else
	{
		u16MicroStepIdx++;
		if(u16MicroStepIdx  >=  l_u16MotorMicroStepsPerElecRotation){u16MicroStepIdx  -=  l_u16MotorMicroStepsPerElecRotation;}
	}
	return(u16MicroStepIdx);
}

void MotorDriverSpeed(uint16_t u16MotorTargetSpeed)
{
	uint16_t u16DeltaPosition = DeltaPosition();
	if(u16DeltaPosition < l_u16DeltaPosition){}
	else
	{
		uint8_t u8MotorSpeedIdx = (uint8_t)u16MotorTargetSpeed;
		g_u16TargetMotorSpeedRPM = l_au16MotorSpeedRPM[u8MotorSpeedIdx];
		g_u8MotorStatusSpeed = u8MotorSpeedIdx;
		l_u16TargetCommutTimerPeriod = p_DivU16_U32byU16(l_u32MicroStepPeriodOneRPM, g_u16TargetMotorSpeedRPM) - 1U;
		if((g_e8MotorStatus & (uint8_t)C_MOTOR_STATUS_MASK)  ==  (uint8_t)C_MOTOR_STATUS_STOPPING)
		{
			g_e8MotorStatus = (uint8_t)C_MOTOR_STATUS_RUNNING;
			l_e8MotorStartupMode = E_MSM_STEPPER_A;
		}
	}
}

void MotorDriverStart(uint16_t u16MotorTargetSpeed)
{
	if((g_e8ErrorElectric & (uint8_t)C_ERR_PERMANENT)  !=  0U){return;}
	g_e8ErrorElectric = (uint8_t)C_ERR_NONE;
	DiagnosticReset();
	if(g_u8MotorHoldingCurrEna  ==  FALSE){ADC_Init();}
	else{ADC_MCurrOffCalib();}
	g_u16ForcedSpeedRPM = g_u16MinSpeedRPM;
	l_u16CommutTimerPeriod = l_u16LowSpeedPeriod;
	l_e8MotorStartupMode = E_MSM_STEPPER_A;
	MotorDriverSpeed(u16MotorTargetSpeed);
	if(g_u16TargetMotorSpeedRPM < g_u16ForcedSpeedRPM){g_u16ForcedSpeedRPM = g_u16TargetMotorSpeedRPM;}
	g_u32TargetPosition = ConvShaftSteps2MicroSteps(g_u16TargetPosition);
	MotorDriverCurrentMeasureInit();
	MotorStallInitA();
	MotorStallInitLA(u16MotorTargetSpeed);
#if (_SUPPORT_HALL_LATCH != FALSE) && (_SUPPORT_NR_OF_HL > 1)
	l_u32SumCommutTimerPeriods = 0U;
	l_u16NrOfCommut = 0U;
	l_u16CommutTimeIdx = 0U;
	l_u16LastHallLatchEvent = TRUE;
#if (_SUPPORT_MICRO_STEP_COMMUTATION != FALSE) && (_SUPPORT_HALL_LATCH_DIAG == FALSE)
	g_u8HallMicroSteps = 0U;
#endif
#endif
	MotorDriverConfig(TRUE);
	{
#if (_SUPPORT_HALL_LATCH != FALSE) && (_SUPPORT_NR_OF_HL > 1) && (_SUPPORT_HALL_LATCH_DIAG == FALSE)
		g_u16HallLatchIO = IO_HALL_STATE;
		if(g_e8MotorDirectionCCW  !=  ((uint8_t)C_MOTOR_DIR_CW)){l_u16MicroStepIdx = au16HallLatchIdxCCW[g_u16HallLatchIO];}
		else{l_u16MicroStepIdx = au16HallLatchIdxCW[g_u16HallLatchIO];}
#endif
		MotorDriver_InitialPwmDutyCycle(Get_ActCurrRunMax_mA(), g_u16ForcedSpeedRPM);
		{
			l_u16MicroStepIdx = MotorDriverUpdateMicroStepIndex(l_u16MicroStepIdx);
		}
		g_e8MotorStatus = C_MOTOR_STATUS_RUNNING;
	}
	MotorDriver_4Phase(l_u16MicroStepIdx);
	HAL_PWM_MasterPendClear();
	HAL_PWM_MasterPendWait();
	DRVCFG_PWM_TUVW();
	DRVCFG_ENA_TUVW();
	l_u8MotorHoldingCurrState = FALSE;
	l_u16RampDownSteps = 0U;
	ADC_Start(FALSE);
	if(l_u16TargetCommutTimerPeriod < l_u16LowSpeedPeriod){l_u16CommutTimerPeriod = l_u16LowSpeedPeriod;}
	else{l_u16CommutTimerPeriod = l_u16TargetCommutTimerPeriod;}
	IO_CTIMER0_TREGB = l_u16CommutTimerPeriod;
#if (_SUPPORT_HALL_LATCH != FALSE) && (_SUPPORT_NR_OF_HL > 1) && (_SUPPORT_HALL_LATCH_DIAG == FALSE)
	l_u16LastCommutTimerPeriod = l_u16CommutTimerPeriod;
#endif
#if (_SUPPORT_HALL_LATCH != FALSE) && (_SUPPORT_HALL_LATCH_SMOOTHING != FALSE)
	g_i16HL_MicroStepCnt = 0;
#endif
	IO_CTIMER0_CTRL = B_CTIMER0_START;
	l_u16MotorDriverDisconDelay = 0U;
	l_u8MotorHoldDelay = 0U;
}

void MotorDriverStop(uint16_t u16Immediate)
{
	if((u16Immediate  ==  (uint16_t)C_STOP_RAMPDOWN)  &&  ((g_e8MotorStatus & C_MOTOR_STATUS_STOP_MASK)  !=  C_MOTOR_STATUS_STOP))
	{
		ENTER_SECTION(ATOMIC_KEEP_MODE);
		{
			if(((g_e8MotorStatus & C_MOTOR_STATUS_STOP_MASK)  !=  C_MOTOR_STATUS_STOP)  &&  ((g_e8MotorStatus & C_MOTOR_STATUS_MASK)  !=  (uint8_t)C_MOTOR_STATUS_STOPPING))
			{
				if(l_u16DeltaPosition > l_u16RampDownSteps)
				{
					if(g_e8MotorDirectionCCW  !=  ((uint8_t)C_MOTOR_DIR_CW))
					{
						p_SubU32_U32byU16(&g_u32TargetPosition,
						(const uint32_t*)&l_u32ActualPosition,
						l_u16RampDownSteps);
					}
					else
					{
						p_AddU32_U32byU16(&g_u32TargetPosition,
						(const uint32_t*)&l_u32ActualPosition,
						l_u16RampDownSteps);
					}
				}
			}
			else if((g_e8MotorStatus & C_MOTOR_STATUS_STOP_MASK)  !=  C_MOTOR_STATUS_STOP){u16Immediate = (uint16_t)C_STOP_IMMEDIATE;}
		}
		EXIT_SECTION();
	}
	else
	{
		HAL_ADC_StopSafe();
		l_e8MotorStartupMode = E_MSM_STOP;
		p_CpyU32_U32( (const uint32_t *)&l_u32ActualPosition, &g_u32TargetPosition);
		ConvMicroSteps2ShaftSteps();
		l_u16DeltaPosition = 0U;
		l_u16RampdownTimeout = 0U;
		g_u8MotorStatusSpeed = (uint8_t)C_STATUS_SPEED_STOP;
		g_u16ActualMotorSpeedRPM = 0U;
		g_u16ForcedSpeedRPM = 0U;
		if((g_u8MotorHoldingCurrEna  !=  FALSE)  &&  ((g_e8ErrorElectric & (uint8_t)C_ERR_PERMANENT)  ==  0U)  &&  (g_e8ErrorVoltage  ==  (uint8_t)C_ERR_VOLTAGE_IN_RANGE)  &&  (g_e8ErrorOverTemperature  ==  (uint8_t)C_ERR_OTEMP_NO)  &&  (u16Immediate  !=  (uint16_t)C_STOP_WO_HOLDING))
		{
			if(g_e8MotorStatus  ==  C_MOTOR_STATUS_STOP)
			{
				MotorDriver_InitialPwmDutyCycle(NV_HOLDING_CURR_LEVEL, 0U);
				MotorDriverConfig(TRUE);
				l_u16MotorCurrentLPF = Get_HoldingThreshold();
			}
			else
			{
				MotorDriver_InitialPwmDutyCycle(Get_ActCurrRunMax_mA() / 2U, 0U);
				l_u16MotorCurrentLPF  >>=  1;
			}
			l_u32MotorCurrentLPF = ((uint32_t)l_u16MotorCurrentLPF  <<  16);
			MotorDriver_4Phase(l_u16MicroStepIdx);
			DRVCFG_PWM_TUVW();
			DRVCFG_ENA_TUVW();
			l_u8MotorHoldingCurrState = TRUE;
			g_e8MotorStatus = C_MOTOR_STATUS_HOLD;
			ADC_Start(FALSE);
		}
		else
		{
			if(g_e8ErrorElectric  ==  (uint8_t)C_ERR_NONE)
			{
				l_u8MotorHoldDelay = PI_TICKS_PER_MILLISECOND;
				l_u16MotorDriverDisconDelay = C_PI_TICKS_100MS;
			}
			else{MotorDriverConfig(FALSE);}
			l_u8MotorHoldingCurrState = FALSE;
			l_u16MotorCurrentMovAvgxN = 0U;
			l_u16MotorCurrentLPF = 0U;
			l_u32MotorCurrentLPF = 0U;
			if(g_e8MotorStatus  !=  C_MOTOR_STATUS_STOP)
			{
				g_u16MotorStartDelay = C_PI_TICKS_MOTOR_RESTART;
				g_e8MotorStatus = C_MOTOR_STATUS_STOP;
			}
		}
		if((u16Immediate  ==  C_STOP_REVERSE)  &&  (l_u16MotorRewindSteps  !=  0U))
		{
			if(g_e8MotorDirectionCCW  !=  FALSE)
			{
				if(g_u16ActualPosition  <=  (uint16_t)(C_MAX_POS - l_u16MotorRewindSteps))
				{
					g_u16TargetPosition = g_u16ActualPosition + l_u16MotorRewindSteps;
					g_u8RewindFlags = (uint8_t)C_REWIND_ACTIVE | (uint8_t)C_REWIND_STALL_DETECT;
				}
				else{g_u8RewindFlags = 0U;}
			}
			else
			{
				if(g_u16ActualPosition  >=  l_u16MotorRewindSteps)
				{
					g_u16TargetPosition = g_u16ActualPosition - l_u16MotorRewindSteps;
					g_u8RewindFlags = (uint8_t)C_REWIND_ACTIVE | (uint8_t)C_REWIND_STALL_DETECT;
				}
				else{g_u8RewindFlags = 0U;}
			}
			if((g_u8RewindFlags & (uint8_t)C_REWIND_ACTIVE)  !=  0U)
			{
				g_u16MotorStartDelay = 0U;
				g_u16TargetMotorSpeedRPM = g_u16LowSpeedRPM;
				g_e8MotorRequest = (uint8_t)C_MOTOR_REQUEST_START;
			}
		}
		else{g_u8RewindFlags = 0U;}
		{
			IO_CTIMER0_CTRL = B_CTIMER0_STOP;
			ENTER_SECTION(SYSTEM_MODE);
			IO_MLX16_ITC_PEND1_S = B_MLX16_ITC_PEND1_CTIMER0_3;
			EXIT_SECTION();
		}
	}
	(void)u16Immediate;
}

void MotorDriverPeriodicTimer(uint16_t u16Period)
{
	if(l_u8MotorHoldDelay  !=  0U)
	{
		if(l_u8MotorHoldDelay > u16Period){l_u8MotorHoldDelay  -=  u16Period;}
		else
		{
			DRVCFG_GND_TUVW();
			l_u8MotorHoldDelay = 0U;
			l_u8MotorHoldingCurrState = FALSE;
		}
	}
	else if(l_u16MotorDriverDisconDelay  !=  0U)
	{
		if(l_u16MotorDriverDisconDelay > u16Period){l_u16MotorDriverDisconDelay  -=  u16Period;}
		else
		{
			l_u16MotorDriverDisconDelay = 0U;
			{
				MotorDriverConfig(FALSE);
			}
			g_e8MotorStatus = C_MOTOR_STATUS_STOP;
		}
	}
	else{}
	ENTER_SECTION(ATOMIC_KEEP_MODE);
	{
		if(l_u16RampdownTimeout  !=  0U)
		{
			if(l_u16RampdownTimeout > u16Period){l_u16RampdownTimeout  -=  u16Period;}
			else
			{
				l_u16RampdownTimeout = 0U;
				MotorDriverStop( (uint16_t)C_STOP_IMMEDIATE);
			}
		}
	}
	EXIT_SECTION();
}

__attribute__((interrupt)) void ISR_CTIMER0_3(void)
{
	do
	{
		if((g_e8MotorStatus & C_MOTOR_STATUS_STOP_MASK)  ==  C_MOTOR_STATUS_STOP){break;}
#if (_SUPPORT_HALL_LATCH == FALSE) || (_SUPPORT_NR_OF_HL == 1U) || (_SUPPORT_HALL_LATCH_DIAG != FALSE)
		if(g_e8MotorDirectionCCW  !=  FALSE){l_u32ActualPosition--;}
		else{l_u32ActualPosition++;}
#else
#if (_SUPPORT_MICRO_STEP_COMMUTATION != FALSE)
		g_u8HallMicroSteps++;
#endif
#endif
		l_u16DeltaPosition = DeltaPosition();
#if (_SUPPORT_HALL_LATCH != FALSE) && (_SUPPORT_NR_OF_HL > 1U) && (_SUPPORT_MICRO_STEP_COMMUTATION != FALSE)
		if(l_u16DeltaPosition  <=  g_u8HallMicroSteps){l_u16DeltaPosition = 0U;}
#endif
		if(l_u16DeltaPosition  ==  0U)
		{
			MotorDriverStop( (uint16_t)C_STOP_IMMEDIATE);
			break;
		}
		if((l_u16DeltaPosition < l_u16RampDownSteps)  &&  (l_e8MotorStartupMode  !=  E_MSM_STEPPER_D))
		{
			l_u16TargetCommutTimerPeriod = l_u16LowSpeedPeriod;
			{
				l_e8MotorStartupMode = E_MSM_STEPPER_D;
			}
			ENTER_SECTION(ATOMIC_KEEP_MODE);
			{
				if((g_e8MotorStatus & (uint8_t)C_MOTOR_STATUS_STOP_MASK)  !=  (uint8_t)C_MOTOR_STATUS_STOP){g_e8MotorStatus = (uint8_t)C_MOTOR_STATUS_STOPPING;}
			}
			EXIT_SECTION();
		}
		MotorDriverCurrentMeasure(l_u16MicroStepIdx);
		{
			int16_t i16MotorVoltageAngle = (int16_t)p_IdxToAngle(l_u16MicroStepIdx, l_u16MotorMicroStepsPerElecRotation);
			int16_t i16TanIV = (i16MotorVoltageAngle - g_i16MotorCurrentAngle);
			if(g_e8MotorDirectionCCW  !=  FALSE){i16TanIV = -i16TanIV;}
			l_i16ActLoadAngleLPF =
			p_LpfI16_I16byI16(&l_i32ActLoadAngleLPF, C_LA_LPF_COEF_OL, (i16TanIV - l_i16ActLoadAngleLPF));
		}
		{
#if (_SUPPORT_HALL_LATCH != FALSE) && (_SUPPORT_NR_OF_HL > 1) && (_SUPPORT_HALL_LATCH_DIAG == FALSE) && (_SUPPORT_HALL_LATCH_SMOOTHING == FALSE)
			if(g_u8ZcHallFound  !=  FALSE)
			{
				uint16_t u16MicroStepIdx = l_u16MicroStepIdx;
				g_u16HallLatchIO = IO_HALL_STATE;
				if(g_e8MotorDirectionCCW  !=  ((uint8_t)C_MOTOR_DIR_CW))
				{
					u16MicroStepIdx = au16HallLatchIdxCCW[g_u16HallLatchIO];
#if (_SUPPORT_HALL_LATCH_LEADANGLE_COMP != FALSE) && (_SUPPORT_FULLSTEP == FALSE)
					uint16_t u16MicroStepIdxCompensation = p_MulDivU16_U16byU16byU16( g_u16ActualMotorSpeedRPM, C_HL_LA_COMPENSATION_uSTEP, g_u16MaxSpeedRPM);
					if(u16MicroStepIdxCompensation > u16MicroStepIdx){u16MicroStepIdx  +=  l_u16MotorMicroStepsPerElecRotation;}
					u16MicroStepIdx = u16MicroStepIdx - u16MicroStepIdxCompensation;
#endif
				}
				else
				{
					u16MicroStepIdx = au16HallLatchIdxCW[g_u16HallLatchIO];
#if (_SUPPORT_HALL_LATCH_LEADANGLE_COMP != FALSE) && (_SUPPORT_FULLSTEP == FALSE)
					u16MicroStepIdx  +=  p_MulDivU16_U16byU16byU16( g_u16ActualMotorSpeedRPM, C_HL_LA_COMPENSATION_uSTEP, g_u16MaxSpeedRPM);
					if(u16MicroStepIdx  >=  l_u16MotorMicroStepsPerElecRotation){u16MicroStepIdx  -=  l_u16MotorMicroStepsPerElecRotation;}
#endif
				}
				l_u16MicroStepIdx = u16MicroStepIdx;
			}
			else
#endif
			{
				l_u16MicroStepIdx = MotorDriverUpdateMicroStepIndex(l_u16MicroStepIdx);
#if (_SUPPORT_HALL_LATCH != FALSE) && (_SUPPORT_HALL_LATCH_SMOOTHING != FALSE)
				g_i16HL_MicroStepCnt++;
#endif
			}
#if (_SUPPORT_HALL_LATCH != FALSE) && (_SUPPORT_NR_OF_HL > 1) && (_SUPPORT_HALL_LATCH_DIAG == FALSE)
#if (_SUPPORT_MICRO_STEP_COMMUTATION != FALSE)
			if(g_u8ZcHallFound  !=  FALSE)
			{
				uint16_t u16CommutTimerPeriod = g_u16Hall_AvgPeriod;
				g_u8ZcHallFound = FALSE;
				if(l_u16NrOfCommut < C_NR_OF_FULLSTEPS)
				{
					p_AddU32byU16( (uint32_t *)&l_u32SumCommutTimerPeriods, u16CommutTimerPeriod);
					l_u16NrOfCommut  +=  1U;
				}
				else
				{
					uint16_t u16PrevCommutPeriod = l_au16CommutTime[l_u16CommutTimeIdx];
					p_SubU32byU16( (uint32_t *)&l_u32SumCommutTimerPeriods, u16PrevCommutPeriod);
					p_AddU32byU16( (uint32_t *)&l_u32SumCommutTimerPeriods, u16CommutTimerPeriod);
				}
				l_au16CommutTime[l_u16CommutTimeIdx] = u16CommutTimerPeriod;
				l_u16CommutTimeIdx  +=  1U;
				if(l_u16CommutTimeIdx  >=  C_NR_OF_FULLSTEPS){l_u16CommutTimeIdx = 0U;}
				l_u16CommutTimerPeriod =
				p_DivU16_U32byU16(l_u32SumCommutTimerPeriods, (l_u16NrOfCommut * g_u16NrOfMicroStepsPerFullStep));
			}
#else
			{
				uint16_t u16CommutTimerPeriod = IO_CTIMER0_TREGB;
				if(g_u8ZcHallFound  !=  FALSE)
				{
					g_u8ZcHallFound = FALSE;
					if(l_u16LastHallLatchEvent  ==  FALSE)
					{
						u16CommutTimerPeriod  +=  l_u16LastCommutTimerPeriod;
						if(u16CommutTimerPeriod < l_u16LastCommutTimerPeriod){u16CommutTimerPeriod = 65534U;}
					}
					if(l_u16NrOfCommut < C_NR_OF_FULLSTEPS){l_u16NrOfCommut++;}
					else
					{
						uint16_t u16PrevCommutPeriod = l_au16CommutTime[l_u16CommutTimeIdx];
						p_SubU32byU16( (uint32_t *)&l_u32SumCommutTimerPeriods, u16PrevCommutPeriod);
					}
					p_AddU32byU16( (uint32_t *)&l_u32SumCommutTimerPeriods, u16CommutTimerPeriod);
					l_au16CommutTime[l_u16CommutTimeIdx] = u16CommutTimerPeriod;
					l_u16CommutTimeIdx  +=  1U;
					if(l_u16CommutTimeIdx  >=  C_NR_OF_FULLSTEPS){l_u16CommutTimeIdx = 0U;}
					l_u16CommutTimerPeriod =
					p_DivU16_U32byU16(l_u32SumCommutTimerPeriods, l_u16NrOfCommut);
					l_u16LastHallLatchEvent = TRUE;
				}
				else{l_u16LastHallLatchEvent = FALSE;}
				u16CommutTimerPeriod = l_u16CommutTimerPeriod;
				if(u16CommutTimerPeriod < 52427U){u16CommutTimerPeriod  +=  (u16CommutTimerPeriod  >>  2);}
				else{u16CommutTimerPeriod = 65534U;}
				IO_CTIMER0_TREGB = u16CommutTimerPeriod;
				l_u16LastCommutTimerPeriod = u16CommutTimerPeriod;
			}
#endif
#else
			if(l_u16CommutTimerPeriod  !=  l_u16TargetCommutTimerPeriod)
			{
				uint16_t u16Compensation = g_u16ForcedSpeedRPM;
				if(l_u16CommutTimerPeriod < l_u16TargetCommutTimerPeriod)
				{
					if((l_u16MicroStepIdx & l_u16SpeedUpdateDec)  ==  0U)
					{
#if (_DEBUG_COMMUT_ISR != FALSE) && defined(DEBUG_SET_IO_C)
						DEBUG_SET_IO_C();
#endif
						uint16_t u16SpeedDecrease = p_DivU16_U32byU16( (uint32_t)l_u16AccelerationConst,
						g_u16ForcedSpeedRPM);
						l_e8MotorStartupMode = E_MSM_STEPPER_D;
						if(u16SpeedDecrease < g_u16ForcedSpeedRPM)
						{
							g_u16ForcedSpeedRPM  -=  u16SpeedDecrease;
#if (_SUPPORT_ACCELERATION == C_ACCELERATION_COSINE_CURVE)
							if(g_u16ForcedSpeedRPM < l_u16AccelerationMin){g_u16ForcedSpeedRPM = l_u16AccelerationMin;}
#else
							if(g_u16ForcedSpeedRPM < g_u16TargetMotorSpeedRPM){g_u16ForcedSpeedRPM = g_u16TargetMotorSpeedRPM;}
#endif
						}
						l_u16CommutTimerPeriod = p_DivU16_U32byU16(l_u32MicroStepPeriodOneRPM, g_u16ForcedSpeedRPM) - 1U;
						if(l_u16CommutTimerPeriod > l_u16TargetCommutTimerPeriod){l_u16CommutTimerPeriod = l_u16TargetCommutTimerPeriod;}
#if (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE)
						l_u16RampDownSteps  -=  (l_u16SpeedUpdateDec + 1U);
#endif
						IO_CTIMER0_TREGB = l_u16CommutTimerPeriod;
						PID_SpeedCompensate(g_u16ForcedSpeedRPM, u16Compensation);
#if (_DEBUG_COMMUT_ISR != FALSE) && defined(DEBUG_CLR_IO_C)
						DEBUG_CLR_IO_C();
#endif
					}
				}
				else if((l_u16MicroStepIdx & l_u16SpeedUpdateAcc)  ==  0U)
				{
#if (_SUPPORT_FOC_MODE != FOC_MODE_NONE) && (_SUPPORT_CLOSED_LOOP_STARTUP == FALSE)
					if((l_u16CommutTimerPeriod  <=  l_u16CloseLoopCommutTimerPeriod)  &&  (g_u8MotorCtrlSpeed  ==  (uint8_t)C_MOTOR_SPEED_AUTO))
					{
						l_e8MotorStartupMode = E_MSM_FOC_2PWM;
						g_u16ActualMotorSpeedRPM = g_u16LowSpeedRPM;
#if (_SUPPORT_PID_U32 == FALSE)
						g_u16ActualMotorSpeedRPMe = (uint16_t)p_MulU32_U16byU16(g_u16ActualMotorSpeedRPM,
						g_u16MotorPolePairs);
#else
						g_u32ActualMotorSpeedRPMe = p_MulU32_U16byU16(g_u16ActualMotorSpeedRPM,
						g_u16MotorPolePairs);
#endif
						PID_SwitchOpen2Close();
						l_u16MicroStepIdx64k = p_DivU16_U32byU16( ((uint32_t)l_u16MicroStepIdx)  <<  16,
						l_u16MotorMicroStepsPerElecRotation);
						HAL_ADC_EnableIRQ();
						IO_CTIMER0_CTRL = B_CTIMER0_STOP;
					}
					else
#endif
					{
#if (_DEBUG_COMMUT_ISR != FALSE) && defined(DEBUG_SET_IO_C)
						DEBUG_SET_IO_C();
#endif
						l_e8MotorStartupMode = E_MSM_STEPPER_A;
#if (_SUPPORT_ACCELERATION == C_ACCELERATION_COSINE_CURVE)
						l_u16AccelerationConst  +=  l_u16AccelerationStep;
						if(l_u16AccelerationConst > l_u16AccelerationMax){l_u16AccelerationConst = l_u16AccelerationMax;}
#endif
						g_u16ForcedSpeedRPM = g_u16ForcedSpeedRPM +
						p_DivU16_U32byU16( (uint32_t)l_u16AccelerationConst, g_u16ForcedSpeedRPM);
						if(g_u16ForcedSpeedRPM > g_u16TargetMotorSpeedRPM){g_u16ForcedSpeedRPM = g_u16TargetMotorSpeedRPM;}
						l_u16CommutTimerPeriod = p_DivU16_U32byU16(l_u32MicroStepPeriodOneRPM, g_u16ForcedSpeedRPM) - 1U;
#if (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE)
						l_u16RampDownSteps  +=  (l_u16SpeedUpdateDec + 1U);
#endif
						IO_CTIMER0_TREGB = l_u16CommutTimerPeriod;
						PID_SpeedCompensate(g_u16ForcedSpeedRPM, u16Compensation);
#if (_DEBUG_COMMUT_ISR != FALSE) && defined(DEBUG_CLR_IO_C)
						DEBUG_CLR_IO_C();
#endif
					}
				}
				else{}
			}
			else{l_e8MotorStartupMode = E_MSM_STEPPER_C;}
#endif
			l_u16CorrectionRatio = VoltageCorrection();
			MotorDriver_4Phase(l_u16MicroStepIdx);
		}
		if(MotorStallCheckA()  !=  C_STALL_NOT_FOUND)
		{
			g_u8StallTypeComm  |=  (uint8_t)C_STALL_FOUND_A;
			if((g_e8StallDetectorEna & ((uint8_t)C_STALLDET_A | (uint8_t)C_STALLDET_CALIB))  !=  0U)
			{
				ConvMicroSteps2ShaftSteps();
				g_u8StallOcc = TRUE;
				if((g_u8RewindFlags & (uint8_t)C_REWIND_ACTIVE)  ==  0U){MotorDriverStop( (uint16_t)C_STOP_REVERSE);}
				else{MotorDriverStop( (uint16_t)C_STOP_EMERGENCY);}
			}
		}
#if (_SUPPORT_STALLDET_H != FALSE) && (_SUPPORT_MICRO_STEP_COMMUTATION == FALSE)
		else if(MotorStallCheckH(l_u16LastHallLatchEvent)  !=  C_STALL_NOT_FOUND)
		{
			g_u8StallTypeComm  |=  (uint8_t)C_STALL_FOUND_H;
			if((g_e8StallDetectorEna & ((uint8_t)C_STALLDET_H | (uint8_t)C_STALLDET_CALIB))  !=  0U)
			{
#if (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE)
				ConvMicroSteps2ShaftSteps();
#endif
				g_u8StallOcc = TRUE;
#if (_SUPPORT_STALL_REVERSE != FALSE)
				if((g_u8RewindFlags & (uint8_t)C_REWIND_ACTIVE)  ==  0U){MotorDriverStop( (uint16_t)C_STOP_REVERSE);}
				else{MotorDriverStop( (uint16_t)C_STOP_EMERGENCY);}
#else
				MotorDriverStop( (uint16_t)C_STOP_EMERGENCY);
#endif
			}
		}
#endif
		else if(MotorStallCheckLA()  !=  C_STALL_NOT_FOUND)
		{
			g_u8StallTypeComm  |=  (uint8_t)C_STALL_FOUND_LA;
			if((g_e8StallDetectorEna & (C_STALLDET_LA | C_STALLDET_CALIB))  !=  0U)
			{
				g_u8StallOcc = TRUE;
				MotorDriverStop( (uint16_t)C_STOP_EMERGENCY);
			}
		}
	}
	while(FALSE);
}
