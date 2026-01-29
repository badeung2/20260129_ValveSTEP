#ifndef DRIVE_LIB_MOTOR_DRIVER_H
#define DRIVE_LIB_MOTOR_DRIVER_H
#include "../AppBuild.h"
#include "../ActADC.h"
#include "../camculib/private_mathlib.h"
#include "../hal_lib/hal_PWM.h"
#include "fm.h"
#define DRVCFG_DIS_TUVW() {IO_PORT_DRV_OUT = (IO_PORT_DRV_OUT & ~(B_PORT_DRV_OUT_ENABLE_DRV3 | B_PORT_DRV_OUT_ENABLE_DRV2 | B_PORT_DRV_OUT_ENABLE_DRV1 | B_PORT_DRV_OUT_ENABLE_DRV0)); }
#define DRVCFG_ENA_TUVW() {IO_PORT_DRV_OUT = (IO_PORT_DRV_OUT | (B_PORT_DRV_OUT_ENABLE_DRV3 | B_PORT_DRV_OUT_ENABLE_DRV2 | B_PORT_DRV_OUT_ENABLE_DRV1 | B_PORT_DRV_OUT_ENABLE_DRV0)); }
#define DRVCFG_PWM_TUVW() {IO_PORT_DRV_CTRL = (C_PORT_DRV_CTRL_DRV3_SLAVE3 | C_PORT_DRV_CTRL_DRV2_SLAVE2 | C_PORT_DRV_CTRL_DRV1_SLAVE1 | C_PORT_DRV_CTRL_DRV0_MASTER1); }
#define DRVCFG_GND_TUVW() {IO_PORT_DRV_CTRL = (C_PORT_DRV_CTRL_DRV3_L | C_PORT_DRV_CTRL_DRV2_L | C_PORT_DRV_CTRL_DRV1_L | C_PORT_DRV_CTRL_DRV0_L); }
#define DRVCFG_SUP_TUVW() {IO_PORT_DRV_CTRL = (C_PORT_DRV_CTRL_DRV3_H | C_PORT_DRV_CTRL_DRV2_H | C_PORT_DRV_CTRL_DRV1_H | C_PORT_DRV_CTRL_DRV0_H); }
#define DRVCFG_CNFG_TUVW(x) {IO_PORT_DRV_CTRL = x;}
#define DRVCFG_TRI_TUVW() {IO_PORT_DRV_CTRL = (C_PORT_DRV_CTRL_DRV3_TRISTATE | C_PORT_DRV_CTRL_DRV2_TRISTATE | C_PORT_DRV_CTRL_DRV1_TRISTATE | C_PORT_DRV_CTRL_DRV0_TRISTATE); }
#define DRVCFG_PWM_UV_WT() {IO_PORT_DRV_CTRL = (C_PORT_DRV_CTRL_DRV3_SLAVE1 | C_PORT_DRV_CTRL_DRV2_SLAVE1 | C_PORT_DRV_CTRL_DRV1_MASTER1 | C_PORT_DRV_CTRL_DRV0_MASTER1); }
#define C_DELAY_5US (uint16_t)((5UL * FPLL) / C_DELAY_CONST)
#define C_DELAY_10US (uint16_t)((10UL * FPLL) / C_DELAY_CONST)
#define C_FETS_RTOT 80U
#define C_NR_OF_FULLSTEPS 4U
#define C_NR_OF_HALFSTEPS 8U
#define C_ZERO_POS_OFFSET 10000U
#define UNKNOWN_STEP 0xFFFFU
#define C_PWM_DCORR_8 ((uint16_t)(((2.29 * PWM_TIMER_CLK) + 500000UL) / 1000000UL))
#define C_PWM_MIN_DC_8 ((uint16_t)(((2.65 * PWM_TIMER_CLK) + 500000UL) / 1000000UL))
#define C_PWM_DCORR_0 ((uint16_t)(((1.13 * PWM_TIMER_CLK) + 500000UL) / 1000000UL))
#define C_PWM_MIN_DC_0 ((uint16_t)(((1.38 * PWM_TIMER_CLK) + 500000UL) / 1000000UL))
#define C_PWM_DCORR_6 ((uint16_t)(((0.08 * PWM_TIMER_CLK) + 500000UL) / 1000000UL))
#define C_PWM_MIN_DC_6 ((uint16_t)(((0.80 * PWM_TIMER_CLK) + 500000UL) / 1000000UL))
#define C_DRV_SLWRT 6U
#define C_CP_SLWRT_DRV 0U
#define C_PWM_DCORR C_PWM_DCORR_6
#if (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM_BIPOLAR) || ((_SUPPORT_PWM_MODE == SINGLE_COIL_PWM) && (C_NR_OF_DC_MOTORS == 1))
#define C_PWM_MIN_DC 0U
#elif (_SUPPORT_FOC_MODE != FOC_MODE_NONE)
#define C_PWM_MIN_DC (C_PWM_MIN_DC_6 >> 1)
#else
#define C_PWM_MIN_DC C_PWM_MIN_DC_6
#endif
#define Q15(A) (int16_t)((A) * 32768)
#define C_INV_SQRT3 Q15(0.57735026918962576450914878050196)
#define C_PI_TICKS_STABILISE (32U * PI_TICKS_PER_MILLISECOND)
#define C_MOVAVG_SZ (1U << C_MOVAVG_SSZ)
typedef enum __attribute__((packed))
{
	E_MSM_STOP = ((uint8_t) 0x00U),
	E_MSM_ACCELERATE = ((uint8_t) 0x01U),
	E_MSM_DECELERATE = ((uint8_t) 0x02U),
	E_MSM_CONSTANT = ((uint8_t) 0x03U),
	E_MSM_SPEED_MODE_MASK = ((uint8_t) 0x03U),
	E_MSM_ALIGN = ((uint8_t) 0x04U),
	E_MSM_STEPPER = ((uint8_t) 0x00U),
	E_MSM_STEPPER_A = ((uint8_t)E_MSM_STEPPER | (uint8_t)E_MSM_ACCELERATE),
	E_MSM_STEPPER_D = ((uint8_t)E_MSM_STEPPER | (uint8_t)E_MSM_DECELERATE),
	E_MSM_STEPPER_C = ((uint8_t)E_MSM_STEPPER | (uint8_t)E_MSM_CONSTANT),
	E_MSM_STEPPER_ALIGN = ((uint8_t)E_MSM_STEPPER | (uint8_t)E_MSM_ALIGN),
}

E_MOTOR_STARTUP_MODE_t;
typedef enum __attribute__((packed))
{
	E_ALIGN_STEP_SHORT = 0U,
	E_ALIGN_STEP_1,
	E_ALIGN_STEP_2A,
	E_ALIGN_STEP_2B,
	E_ALIGN_STEP_2C,
	E_ALIGN_STEP_DONE
}

E_ALIGNMENT_STEPS_t;
#define C_ALIGN_SOFT_PERIOD C_MICROSTEP_PER_FULLSTEP
#define C_ALIGN_STEP_1_PERIOD (1U * 64U)
#define C_ALIGN_STEP_2A_PERIOD (1U * 64U)
#define C_ALIGN_STEP_2B_PERIOD (2U * 64U)
#define C_ALIGN_STEP_2C_PERIOD (4U * 64U)
typedef enum __attribute__((packed))
{
	C_STOP_RAMPDOWN = 0U,
	C_STOP_IMMEDIATE,
	C_STOP_EMERGENCY,
	C_STOP_REVERSE,
	C_STOP_WO_HOLDING
}

MOTOR_STOP_MODE;
#define C_DELTA_POS_MOTOR_STOP 1U
#define C_STATUS_SPEED_STOP 0U
typedef struct _MOTOR_PWM_t
{
	uint16_t u16PwmMaster1;
	uint16_t u16PwmSlave1;
	uint16_t u16PwmSlave2;
	uint16_t u16PwmSlave3;
}

MOTOR_PWM_t;
static inline int16_t* p_SinDirectLutInlined_2_5k(uint16_t theta)
{
	int16_t *pSinLUT;
	__asm__ __volatile__ (
	"lsr %w1, #2\n\t"
	"lsr %w1, #2\n\t"
	"lsr %w1, #2\n\t"
	"lsl %w1\n\t"
	"add %w1, #(_fm_SinCosTable_2_5k+2)\n\t"
	: "=x" (pSinLUT)
	: "x" (theta)
	);
	return( pSinLUT );
}

#pragma space dp
extern uint16_t l_u16MotorMicroStepsPerElecRotation;
#pragma space none
#pragma space nodp
extern MOTOR_PWM_t MotorPwm;
extern uint16_t g_u16NrOfMicroStepsPerFullStep;
extern uint16_t g_u16ForcedSpeedRPM;
#if (_SUPPORT_HALL_LATCH != FALSE) && (_SUPPORT_NR_OF_HL > 1)
extern volatile uint8_t g_u8ZcHallFound;
#if (_SUPPORT_MICRO_STEP_COMMUTATION != FALSE)
extern uint8_t g_u8HallMicroSteps;
#endif
#endif
extern uint16_t l_u16CDI;
#pragma space none
extern void MotorDriverConfig(uint16_t u16State);
extern void MotorDriverInit(uint16_t u16FullInit);
extern void MotorDriverPermanentError(uint8_t u8ElectricErrorCode);
extern void MotorDriverPosInit(uint16_t u16InitPosition);
extern void ConvMicroSteps2ShaftSteps(void);
extern uint32_t ConvShaftSteps2MicroSteps(uint16_t u16Position);
extern void MotorDriver_4Phase(uint16_t u16MicroStepIdx);
extern void MotorDriverCurrentMeasureInit(void);
extern void MotorDriverHoldCurrentMeasure(void);
extern uint16_t MotorDriverUpdateMicroStepIndex(uint16_t u16MicroStepIdx);
extern void MotorDriverSpeed(uint16_t u16MotorTargetSpeed);
extern void MotorDriverStart(uint16_t u16MotorTargetSpeed);
extern void MotorDriverStop(uint16_t u16Immediate);
extern void MotorDriverPeriodicTimer(uint16_t u16Period);
#pragma space dp
extern volatile uint16_t l_u16CorrectionRatio;
extern volatile uint16_t l_u16StallDetectorDelay;
extern uint16_t l_u16MotorCurrentMovAvgxN;
extern uint16_t l_u16MotorCurrentLPF;
extern volatile uint16_t l_u16MicroStepIdx;
extern uint16_t l_u16MotorDriverDisconDelay;
extern volatile E_MOTOR_STARTUP_MODE_t l_e8MotorStartupMode;
#pragma space none
static inline uint16_t Get_CorrectionRatio(void)
{
	return(l_u16CorrectionRatio);
}

static inline void Set_CorrectionRatio(uint16_t u16Value)
{
	l_u16CorrectionRatio = u16Value;
}

static inline uint16_t Get_MicroStepIdx(void)
{
	return(l_u16MicroStepIdx);
}

static inline uint16_t Get_MotorDriverDisconDelay(void)
{
	return(l_u16MotorDriverDisconDelay);
}

static inline uint16_t Get_StartupDelay(void)
{
	return(l_u16StallDetectorDelay);
}

static inline E_MOTOR_STARTUP_MODE_t Get_MotorStartupMode(void)
{
	return(l_e8MotorStartupMode);
}

static inline uint8_t Get_MotorHoldingCurrState(void)
{
	extern uint8_t l_u8MotorHoldingCurrState;
	return(l_u8MotorHoldingCurrState);
}

static inline uint16_t Get_MotorCurrentMovAvgxN(void)
{
	return(l_u16MotorCurrentMovAvgxN);
}

static inline uint16_t Get_MotorCurrentMovAvgxN_mA(void)
{
#if (C_MOVAVG_SZ == (65536UL / C_GMCURR_DIV))
	uint16_t u16Value = p_MulU16hi_U16byU16(Get_MotorCurrentMovAvgxN(),
	Get_MCurrGain());
#elif defined (C_GMCURR_SDIV) && defined (C_MOVAVG_SSZ)
	uint16_t u16Value =
	(uint16_t) (p_MulU32_U16byU16(l_u16MotorCurrentMovAvgxN,
	Get_MCurrGain())  >>  (C_GMCURR_SDIV + C_MOVAVG_SSZ));
#else
	uint16_t u16Value =
	p_MulDivU16_U16byU16byU16(Get_MotorCurrentMovAvgxN(),
	Get_MCurrGain(),
	(C_GMCURR_DIV * C_MOVAVG_SZ));
#endif
	return(u16Value);
}

static inline void Set_MotorCurrentMovAvgxN(uint16_t u16Value)
{
	l_u16MotorCurrentMovAvgxN = u16Value;
}

static inline uint16_t Get_MotorCurrentLPF(void)
{
	return(l_u16MotorCurrentLPF);
}

static inline uint16_t Get_ShaftRatiox512(void)
{
	extern uint16_t l_u16ShaftRatiox512;
	return(l_u16ShaftRatiox512);
}

static inline void Set_CommutTimerPeriod(uint16_t u16Value)
{
	extern uint16_t l_u16CommutTimerPeriod;
	l_u16CommutTimerPeriod = u16Value;
}

#if (_SUPPORT_HALL_LATCH != FALSE) && (_SUPPORT_NR_OF_HL > 1U) && (_SUPPORT_HALL_LATCH_DIAG == FALSE)
static inline uint16_t Get_NrOfCommut(void)
{
	extern uint16_t l_u16NrOfCommut;
	return(l_u16NrOfCommut);
}

#endif
static inline uint16_t Get_MotorMicroStepsPerMechRotation(void)
{
	extern uint16_t l_u16MotorMicroStepsPerMechRotation;
	return(l_u16MotorMicroStepsPerMechRotation);
}

static inline uint16_t Get_MotorMicroStepsPerElecRotation(void)
{
	extern uint16_t l_u16MotorMicroStepsPerElecRotation;
	return(l_u16MotorMicroStepsPerElecRotation);
}

static inline uint16_t Get_CommutTimerPeriod(void)
{
	extern uint16_t l_u16CommutTimerPeriod;
	return(l_u16CommutTimerPeriod);
}

static inline uint32_t Get_MicroStepPeriodOneRPM(void)
{
	extern uint32_t l_u32MicroStepPeriodOneRPM;
	return(l_u32MicroStepPeriodOneRPM);
}

#endif
