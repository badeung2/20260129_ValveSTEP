/*!*************************************************************************** *
 * \file        MotorDriver.c
 * \brief       MLX813xx Motor Driver (Stepper) handling
 *
 * \note        project MLX813xx
 *
 * \author      Marcel Braat
 *
 * \date        2017-08-15
 *
 * \version     2.0
 *
 * A list of functions:
 *  - Global Functions:
 *           -# MotorDriverConfig()
 *           -# MotorDriverInit()
 *           -# MotorDriverPermanentError()
 *           -# MotorDriverPosInit()
 *           -# ConvMicroSteps2ShaftSteps()
 *           -# ConvShaftSteps2MicroSteps()
 *           -# DeltaPosition()
 *           -# MotorDriver_3Phase()
 *           -# MotorDriver_3PhaseBEMF()
 *           -# MotorDriver_4Phase()
 *           -# ISR_PWM_MASTER1_END();
 *           -# MotorDriverCurrentMeasureInit()
 *           -# MotorDriverHoldCurrentMeasure()
 *           -# MotorDriverSpeed()
 *           -# MotorDriverStart()
 *           -# MotorDriverStop()
 *           -# MotorDriverPeriodicTimer()
 *           -# ISR_CTIMER0_3()
 *           -# ISR_CDI()
 *  - Internal Functions:
 *           -# MotorDriver_InitialPwmDutyCycle()
 *           -# MotorDriverCurrentMeasure()
 *           -# MotorDriverUpdateMicroStepIndex()
 *
 * \copyright   MELEXIS Microelectronic Integrated Systems
 *
 * Copyright (C) 2017-2022 Melexis N.V.
 * The Software is being delivered 'AS IS' and Melexis, whether explicitly or
 * implicitly, makes no warranty as to its Use or performance.
 * The user accepts the Melexis Firmware License Agreement.
 *
 * Melexis confidential & proprietary
 * *************************************************************************** */

/*!*************************************************************************** */
/*                           INCLUDES                                          */
/* *************************************************************************** */
#include "AppBuild.h"                                                           /* Application Build */

#if (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT)

#include "../ActADC.h"                                                          /* Application ADC support */

#include "drivelib/Diagnostic.h"                                                /* Diagnostic support */
#include "drivelib/NV_UserPage.h"                                               /* Non Volatile Memory User Application Page Layout */
#if (_SUPPORT_LOG_ERRORS != FALSE) && (((_SUPPORT_TRIAXIS_MLX9038x != FALSE) && (_DEBUG_MLX90381 != FALSE)) || (_SUPPORT_WINDMILL != FALSE))
#include "drivelib/ErrorCodes.h"                                                /* Error logging support */
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) && (((_SUPPORT_TRIAXIS_MLX9038x != FALSE) && (_DEBUG_MLX90381 != FALSE)) || (_SUPPORT_WINDMILL != FALSE)) */
#include "drivelib/GlobalVars.h"                                                /* Global Variables support */
#include "drivelib/MotorDriver.h"                                               /* Motor driver support */
#include "drivelib/MotorDriverTables.h"                                         /* Wave-form vector tables */
#include "drivelib/MotorStall.h"                                                /* Motor Stall Detectors */
#if (_SUPPORT_WINDMILL != FALSE)
#include "drivelib/MotorWindmill.h"                                             /* Motor Wind-mill support */
#endif /* (_SUPPORT_WINDMILL != FALSE) */
#include "drivelib/PID_Control.h"                                               /* PID support */
#include "camculib/private_mathlib.h"                                           /* Private math-library */
#include "drivelib/Timer.h"                                                     /* Simple Timer support */
#if (_SUPPORT_TRIAXIS_MLX90363 != FALSE)
#include "senselib/Triaxis_MLX90363.h"                                          /* (SPI) Triaxis MLX90363 support */
#elif (_SUPPORT_TRIAXIS_MLX90367 != FALSE) || (_SUPPORT_TRIAXIS_MLX90372 != FALSE)
#include "senselib/Triaxis_MLX90367_372.h"                                      /* (SENT) Triaxis MLX90367/372 support */
#elif (_SUPPORT_TRIAXIS_MLX90377 != FALSE)
#include "senselib/Triaxis_MLX90377.h"                                          /* (PWM/SENT/SPC) Triaxis MLX90377 support */
#elif (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
#include "senselib/Triaxis_MLX9038x.h"                                          /* Resolver MLX90380 support */
#elif (_SUPPORT_TRIAXIS_MLX90395 != FALSE)
#include "senselib/Triaxis_MLX90395.h"                                          /* (SPI) Triaxis MLX90395 support */
#elif (_SUPPORT_TRIAXIS_MLX90421 != FALSE) || (_SUPPORT_TRIAXIS_MLX90425 != FALSE)
#include "senselib/Triaxis_MLX90421_425.h"                                      /* (PWM) Triaxis MLX90421 or MLX90425 support */
#elif (_SUPPORT_TRIAXIS_MLX90422 != FALSE) || (_SUPPORT_TRIAXIS_MLX90426 != FALSE)
#include "senselib/Triaxis_MLX90422_426.h"                                      /* (SENT) Triaxis MLX90422 or MLX90426 support */
#elif (_SUPPORT_TRIAXIS_MLX90427 != FALSE)
#include "senselib/Triaxis_MLX90427.h"                                          /* (SPI) Triaxis MLX90427 support */
#elif (_SUPPORT_DUAL_HALL_LATCH_MLX92255 != FALSE)
#include "senselib/DualHallLatch_MLX92255.h"                                    /* Dual Hall-Latch MLX92255 support */
#endif /* (_SUPPORT_DUAL_HALL_LATCH_MLX92255 != FALSE) */
#if (_SUPPORT_HALL_LATCH_MLX9227x != FALSE)
#include "senselib/HallLatch_MLX9227x.h"                                        /* Hall Latch MLX9227x support */
#endif /* (_SUPPORT_HALL_LATCH_MLX9227x != FALSE) */
#if ((_SUPPORT_HALL_LATCH != FALSE) && (_SUPPORT_NR_OF_HL > 1)) || (_SUPPORT_HALL_LATCH_DIAG != FALSE)
#include "senselib/HallLatch.h"                                                 /* Hall-Latch support */
#endif /* ((_SUPPORT_HALL_LATCH != FALSE)  && (_SUPPORT_NR_OF_HL > 1)) || (_SUPPORT_HALL_LATCH_DIAG != FALSE) */

#include <atomic.h>
#include <sys_tools.h>

#if (_SUPPORT_FOC_MODE == FOC_MODE_NONE)
#define FOC_STATIC static
#else  /* (_SUPPORT_FOC_MODE == FOC_MODE_NONE) */
#define FOC_STATIC
#endif /* (_SUPPORT_FOC_MODE == FOC_MODE_NONE) */

#ifndef _SUPPORT_ACT_SPEED_BY_LIN
#define _SUPPORT_ACT_SPEED_BY_LIN       FALSE                                   /*!< FALSE: Actuator speed by MotroDriver; TRUE: Actuator speed by LIN Command */
#endif /* _SUPPORT_ACT_SPEED_BY_LIN */

#ifndef C_MAX_POS
#define C_MAX_POS               0xFFFEU                                         /*!< Maximum position */
#endif /* C_MAX_POS */

/*!*************************************************************************** */
/*                           DEFINITIONS                                       */
/* *************************************************************************** */
#define C_SOFT_START_RAMP_STEPS 16U                                             /*!< Soft start-up ramp in micro-steps */
#define C_OFF_FULL_STEP         1U                                              /*!< Full-step compensation */

#if (_SUPPORT_FOC_MODE != FOC_MODE_NONE) && defined(C_COIL_L)                  /* FOC for Positioning Device */
#if (C_COIL_L > 64000)
#error "Coil inductance too large"
#elif (C_COIL_L > 32000)
#define C_L_SR                  8U                                              /*!< Coil-inductance shift-divider */
#define C_Z_SR                  8U                                              /*!< Z_Const shift-divider factor (Must be greater-equal C_L_SR, and C_Z_CONST must be below 64k) */
#elif (C_COIL_L > 16000)
#define C_L_SR                  7U                                              /*!< Coil-inductance shift-divider */
#define C_Z_SR                  7U                                              /*!< Z_Const shift-divider factor (Must be greater-equal C_L_SR, and C_Z_CONST must be below 64k) */
#elif (C_COIL_L > 8000)
#define C_L_SR                  6U                                              /*!< Coil-inductance shift-divider */
#define C_Z_SR                  6U                                              /*!< Z_Const shift-divider factor (Must be greater-equal C_L_SR, and C_Z_CONST must be below 64k) */
#elif (C_COIL_L > 4000)
#define C_L_SR                  5U                                              /*!< Coil-inductance shift-divider */
#define C_Z_SR                  5U                                              /*!< Z_Const shift-divider factor (Must be greater-equal C_L_SR, and C_Z_CONST must be below 64k) */
#elif (C_COIL_L > 2000)
#define C_L_SR                  4U                                              /*!< Coil-inductance shift-divider */
#define C_Z_SR                  4U                                              /*!< Z_Const shift-divider factor (Must be greater-equal C_L_SR, and C_Z_CONST must be below 64k) */
#elif (C_COIL_L > 1000)
#define C_L_SR                  3U                                              /*!< Coil-inductance shift-divider */
#define C_Z_SR                  4U                                              /*!< Z_Const shift-divider factor (Must be greater-equal C_L_SR, and C_Z_CONST must be below 64k) */
#elif (C_COIL_L > 500)
#define C_L_SR                  2U                                              /*!< Coil-inductance shift-divider */
#define C_Z_SR                  4U                                              /*!< Z_Const shift-divider factor (Must be greater-equal C_L_SR, and C_Z_CONST must be below 64k) */
#elif (C_COIL_L > 250)
#define C_L_SR                  1U                                              /*!< Coil-inductance shift-divider */
#define C_Z_SR                  4U                                              /*!< Z_Const shift-divider factor (Must be greater-equal C_L_SR, and C_Z_CONST must be below 64k) */
#else
#define C_L_SR                  0U                                              /*!< Coil-inductance shift-divider */
#define C_Z_SR                  4U                                              /*!< Z_Const shift-divider factor (Must be greater-equal C_L_SR, and C_Z_CONST must be below 64k) */
#endif
#define C_Z_CONST (uint16_t)(((26484375UL * (C_GMCURR_DIV / C_VOLTGAIN_DIV)) / 71U) >> C_Z_SR) /*!< * CONST = (C_Z_SR=8) 1457.1069
                                                                                                *             (C_Z_SR=7) 2914.2138
                                                                                                *             (C_Z_SR=6) 5828.4276
                                                                                                *             (C_Z_SR=5) 11656.8552
                                                                                                *             (C_Z_SR=4) 23313.7104 */
#endif /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) && defined(C_COIL_L) */

#if !defined (C_MAX_STEP_POS)
#define C_MAX_STEP_POS                      C_MAX_POS                           /*!< Maximum Step Position */
#endif /* !defined (C_MAX_STEP_POS) */

/*!*************************************************************************** */
/*                           GLOBAL & LOCAL VARIABLES                          */
/* *************************************************************************** */
#pragma space dp
volatile uint16_t l_u16CorrectionRatio = 0U;                                    /*!< Motor correction ratio, depend on temperature and voltage */
volatile uint16_t l_u16StallDetectorDelay = C_DETECTOR_DELAY;                   /*!< Stall detector delay (micro-steps) */
uint16_t l_u16MotorCurrentMovAvgxN = 0U;                                        /*!< Moving average current (4..16 samples) [ADC-LSB] */
uint16_t l_u16MotorCurrentLPF = 0U;                                             /*!< Low-pass filter (IIR-1) motor-current [ADC-LSB] */
uint16_t l_u16TargetCommutTimerPeriod;                                          /*!< Target commutation timer period (target speed) */
volatile uint16_t l_u16MicroStepIdx = 0U;                                       /*!< (Micro)step index */
uint16_t l_u16CommutTimerPeriod = 0U;                                           /*!< Commutation timer period */
uint16_t l_u16MotorDriverDisconDelay = 0U;                                      /*!< Motor Driver Disconnect Delay: From LS/TRI-state to OFF */
uint16_t l_u16MotorMicroStepsPerElecRotation;                                   /*!< Number of (micro-) steps per electric rotation */
#if (C_MOTOR_PHASES != 1)
uint16_t l_u16mR_AT = 0U;                                                       /*!< Coil resistance at ambient temperature */
uint16_t l_u16mZ = 0U;                                                          /*!< Coil Inductance */
#endif /* (C_MOTOR_PHASES != 1) */
#if (_SUPPORT_FOC_MODE != FOC_MODE_NONE) && (C_MOTOR_PHASES != 1)
uint16_t l_u16Ipk = 0U;                                                         /*!< Ipk value */
uint16_t l_u16Ipk_Prev = 0U;                                                    /*!< Previous Ipk value (filter) */
#if (_SUPPORT_PID_U32 == FALSE)
uint16_t g_u16ActualMotorSpeedRPMe = 0U;                                        /*!< Actual motor-speed (electric-rotations) */
#else  /* (_SUPPORT_PID_U32 == FALSE) */
uint32_t g_u32ActualMotorSpeedRPMe = 0UL;                                       /*!< Actual motor-speed (electric-rotations) */
#endif /* (_SUPPORT_PID_U32 == FALSE) */
#if (_SUPPORT_CLOSED_LOOP_STARTUP == FALSE)
static uint16_t l_u16CloseLoopCommutTimerPeriod;                                /*!< Close-loop Commutation Timer Period */
#endif /* (_SUPPORT_CLOSED_LOOP_STARTUP == FALSE) */
#endif /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) && (C_MOTOR_PHASES != 1) */
#if (_SUPPORT_PWM_MODE == TRIPLEPHASE_TWOPWM_INDEPENDENT_GND) && (_SUPPORT_PWM_POL_CORR == FALSE)
static uint16_t l_u16PwmState = FALSE;                                          /*!< PWM State */
#endif /* (_SUPPORT_PWM_MODE == TRIPLEPHASE_TWOPWM_INDEPENDENT_GND) && (_SUPPORT_PWM_POL_CORR == FALSE) */
volatile E_MOTOR_STARTUP_MODE_t l_e8MotorStartupMode = E_MSM_STOP;              /*!< Motor driver state */
#if (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE)
uint8_t l_u8MotorHoldDelay = 0U;                                                /*!< Delay between drive stage frozen to LS */
#endif /* (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) */
#if (_SUPPORT_VARIABLE_PWM != FALSE) && (_SUPPORT_VAR_PWM_MODE != C_VAR_PWM_MODE_OFF)
uint8_t g_u8TriplePWM = FALSE;                                                  /*!< FALSE: 2-coil/phase current; TRUE: 3-coil/phase current (MMP220815-1) */
#endif /* (_SUPPORT_VARIABLE_PWM != FALSE) && (_SUPPORT_VAR_PWM_MODE != C_VAR_PWM_MODE_OFF) */
#if (_SUPPORT_STALLDET_BZC != FALSE)
volatile uint8_t g_e8ZcDetectorState = (uint8_t)ZC_RESET;                       /*!< [BEMF] Zero-crossing detector state */
#endif /* (_SUPPORT_STALLDET_BZC != FALSE) */
#if (C_MOTOR_PHASES == 3) && (_SUPPORT_PWM_MODE == TRIPLEPHASE_ALLPWM_MIRROR) && \
    (UART_COMM != FALSE) && ((_SUPPORT_UART_IF_APP == C_UART_IF_SCOPE) || (_SUPPORT_UART2_IF_APP == C_UART_IF_SCOPE))
volatile int16_t l_i16MotorVoltageCoilA = 0;                                    /*!< Motor Voltage Coil 'A' (used by UART-Scope) */
#endif
#if (_SUPPORT_POTI != FALSE) && (_SUPPORT_POTI_MODE == C_POTI_MODE_OPEN_LOOP)
volatile uint16_t g_u16ActualPotiPos __attribute__ ((section(".dp.noinit")));   /*!< (Motor-driver) Actual motor-rotor position */
#endif /* (_SUPPORT_POTI != FALSE) && (_SUPPORT_POTI_MODE == C_POTI_MODE_OPEN_LOOP) */
#pragma space none

#pragma space nodp                                                              /* __NEAR_SECTION__ */
#if (_SUPPORT_PWM_SYNC == FALSE)
MOTOR_PWM_t MotorPwm = {0};                                                     /*!< PWM LT Values */
#endif /* (_SUPPORT_PWM_SYNC == FALSE) */
uint16_t l_u16StallDetectorThrshld;                                             /*!< Stall detector delay threshold (micro-steps) */
uint16_t l_u16MotorFullStepsPerElecRotation;                                    /*!< Number of (full-) steps per electric rotation */
uint16_t l_u16MotorMicroStepsPerMechRotation;                                   /*!< Number of (micro-) steps per mechanical rotation */
uint16_t g_u16NrOfMicroStepsPerFullStep;                                        /*!< Number of Micro-steps per full-step */
uint16_t l_u16ShaftRatiox512 = 512U;                                            /*!< Shaft ration (x512) */
#if (_SUPPORT_PWM_MODE != SINGLE_COIL_PWM)
uint32_t l_u32MicroStepPeriodOneRPM;                                            /*!< Temporary variable speed control */
#endif /* (_SUPPORT_PWM_MODE != SINGLE_COIL_PWM) */
#if (C_MOTOR_PHASES != 1) || (_SUPPORT_HALL_LATCH == FALSE)
uint16_t l_u16AccelerationConst;                                                /*!< Acceleration (and deceleration) constant */
#if (_SUPPORT_ACCELERATION == C_ACCELERATION_COSINE_CURVE)
uint16_t  l_u16AccelerationMin = 10U;                                           /*!< Initial/minimum acceleration */
uint16_t  l_u16AccelerationMax = 10U;                                           /*!< Maximum acceleration */
uint16_t  l_u16AccelerationStep = 3U;                                           /*!< Acceleration increase: 3 */
#endif /* (_SUPPORT_ACCELERATION == C_ACCELERATION_COSINE_CURVE) */
uint16_t g_u16ForcedSpeedRPM;                                                   /*!< Forced (Feed Forward) Speed [RPM] */
#endif /* (C_MOTOR_PHASES != 1) || (_SUPPORT_HALL_LATCH == FALSE) */
#if (_SUPPORT_HALL_LATCH != FALSE) && (_SUPPORT_NR_OF_HL > 1) && (_SUPPORT_HALL_LATCH_DIAG == FALSE)
uint16_t l_u16LastHallLatchEvent = TRUE;                                        /*!< Last Hall-Latch Event Found (MMP230720-1) */
uint16_t l_u16LastCommutTimerPeriod;                                            /*!< Last Commutation Timer Period (MMP230720-1) */
#endif /* (_SUPPORT_HALL_LATCH != FALSE) && (_SUPPORT_NR_OF_HL > 1) && (_SUPPORT_HALL_LATCH_DIAG == FALSE) */
#if (_SUPPORT_PWM_MODE != SINGLE_COIL_PWM)
uint16_t l_u16LowSpeedPeriod;                                                   /*!< Minimum speed (period) */
#endif /* (_SUPPORT_PWM_MODE != SINGLE_COIL_PWM) */
FOC_STATIC uint32_t l_u32MotorCurrentLPF = 0U;                                  /*!< Motor current LPF internal variable */
#if (_SUPPORT_FOC_MODE != FOC_MODE_NONE)
#if ((_SUPPORT_CLOSED_LOOP_STARTUP == FALSE) || (_SUPPORT_TRIAXIS_MLX9038x != FALSE)) && (C_MOTOR_PHASES != 1)
uint16_t l_u16SpeedUpdateAcc = (C_MICROSTEP_PER_FULLSTEP - 1U);                 /*!< Speed acceleration update (micro-steps) */
uint16_t l_u16SpeedUpdateDec = (C_MICROSTEP_PER_FULLSTEP - 1U);                 /*!< Speed deceleration update (micro-steps) */
#endif /* ((_SUPPORT_CLOSED_LOOP_STARTUP == FALSE) || (_SUPPORT_TRIAXIS_MLX9038x != FALSE)) && (C_MOTOR_PHASES != 1) */
#if (C_MOTOR_PHASES != 1)
uint16_t l_u16mR_RT = 0U;                                                       /*!< Coil resistance [mR] converted to ADC_LSB_V at Reference (e.g. Room) Temperature */
#endif /* (C_MOTOR_PHASES != 1) */
#if (_SUPPORT_CLOSED_LOOP_STARTUP != FALSE)
uint16_t l_u16StartrupThrshldSpeedRPMe;                                         /*!< FOC Close-loop Start-up max. speed threshold */
#endif /* (_SUPPORT_CLOSED_LOOP_STARTUP != FALSE) */
#else  /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) */
#if (_SUPPORT_PWM_MODE != SINGLE_COIL_PWM)
#endif /* (_SUPPORT_PWM_MODE != SINGLE_COIL_PWM) */
#if (C_MOTOR_PHASES != 1) && ((_SUPPORT_HALL_LATCH == FALSE) || (_SUPPORT_NR_OF_HL <= 1))
static uint16_t l_u16SpeedUpdateAcc = (C_MICROSTEP_PER_FULLSTEP - 1U);          /*!< Speed acceleration update (micro-steps) */
static uint16_t l_u16SpeedUpdateDec = (C_MICROSTEP_PER_FULLSTEP - 1U);          /*!< Speed deceleration update (micro-steps) */
#endif /* (C_MOTOR_PHASES != 1) && ((_SUPPORT_HALL_LATCH == FALSE) || (_SUPPORT_NR_OF_HL <= 1)) */
#endif /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) */
#if (_SUPPORT_STALLDET_BZC != FALSE) || (_SUPPORT_CDI != FALSE) || \
    (_SUPPORT_TACHO_OUT != FALSE)                                               /* MMP230607-1 */
static uint16_t l_u16MotorFullStepsPerMechRotation;                             /*!< [BEMF|CDI|TACHO] Number of full-steps per mechanical rotation */
uint32_t l_u32FullStepPeriodOneRPM;                                             /*!< [CDI] Temporary variable speed control */
#endif /* (_SUPPORT_STALLDET_BZC != FALSE) || (_SUPPORT_CDI != FALSE) */
#if (_SUPPORT_TACHO_OUT != FALSE)
uint16_t l_u16TachoThreshold = 0U;                                              /*!< Tacho threshold (disabled) */
uint16_t l_u16TachoCount = 0U;                                                  /*!< Tacho count */
#endif /* (_SUPPORT_TACHO_OUT != FALSE) */
#if (_SUPPORT_PWM_MODE != SINGLE_COIL_PWM)
static uint16_t l_u16PhaseCoilResistanceRT = (C_FETS_RTOT + 10U);               /*!< Phase + Motor-coil Resistance at RT [10mR] (MMP240515-1) */
static uint16_t l_u16PhaseCoilResistanceAT = (C_FETS_RTOT + 10U);               /*!< Phase + Motor-coil Resistance at AT [10mR] (MMP240515-1) */
#endif /* (_SUPPORT_PWM_MODE != SINGLE_COIL_PWM) */
#if (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE)
#if (_SUPPORT_ACT_SPEED_BY_LIN == FALSE)
#if (LINPROT == LIN22_SIMPLE_PCT) || (LINPROT == NOLIN)
static uint16_t l_au16MotorSpeedRPM[4];                                         /*!< Motor speed table (4 speeds) */
#elif (LINPROT == LIN2X_HVAC49) || (LINPROT == LIN2X_HVAC52) || (LINPROT == LIN2X_AIRVENT12) || (LINPROT == LIN2X_AGS)
static uint16_t l_au16MotorSpeedRPM[8];                                         /*!< Motor speed table (8 speeds) */
#endif /* (LINPROT == LIN2X_HVAC49) || (LINPROT == LIN2X_HVAC52) || (LINPROT == LIN2X_AIRVENT12) */
#endif /* (_SUPPORT_ACT_SPEED_BY_LIN == FALSE) */
#if (C_MOTOR_PHASES != 1)
FOC_STATIC uint16_t l_u16RampDownSteps = 0U;                                    /*!< Ramp-down steps */
FOC_STATIC uint16_t l_u16DeltaPosition = 0U;                                    /*!< Difference in target and actual position */
#endif /* (C_MOTOR_PHASES != 1) */
uint8_t l_u8MotorHoldingCurrState = FALSE;                                      /*!< Motor Holding Current State */
#endif /* (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) */
#if (PWM_LIMIT == PWM_LIMIT_BY_PWM)
uint16_t l_u16MaxPwmRatio = PWM_REG_PERIOD;                                     /*!< Maximum Motor-PWM ratio (MMP181114-2) */
#if (_SUPPORT_BOOST_MODE == BOOST_MODE_WAVEFORM)
uint16_t l_u16MaxPwmCorrRatio = (PWM_REG_PERIOD << 4);                          /*!< Max PWM Correction Ratio */
uint16_t l_u16MaxPwmCorrRatioBoost = (PWM_REG_PERIOD << 4) + PWM_REG_PERIOD;    /*!< Max PWM Correction Ratio Boost */
#endif /* (_SUPPORT_BOOST_MODE == BOOST_MODE_WAVEFORM) */
#endif /* (PWM_LIMIT == PWM_LIMIT_BY_PWM) */
#if (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) && (_SUPPORT_PWM_MODE != SINGLE_COIL_PWM)
static uint16_t l_u16RampdownTimeout = 0U;                                      /*!< Ramp-down timeout period (MMP170806-1) */
#endif /* (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) && (_SUPPORT_PWM_MODE != SINGLE_COIL_PWM) */

#if (_SUPPORT_REWIND != FALSE) || (_SUPPORT_STALL_REVERSE != FALSE)             /* MMP230531-2 */
#if (_SUPPORT_REWIND != FALSE)
static uint16_t l_u16TargetPositionRewind;                                      /*!< Memorised Target Position after Rewind */
#endif /* (_SUPPORT_REWIND != FALSE) */
static uint16_t l_u16MotorRewindSteps;
#endif /* (_SUPPORT_REWIND != FALSE) || (_SUPPORT_STALL_REVERSE != FALSE) */

static uint16_t l_au16MotorCurrentRaw[C_MOVAVG_SZ];                             /*!< Motor Current Moving-average samples */
#if !defined (C_MOVAVG_SSZ) || (C_MOVAVG_SZ > ((C_MOTOR_PHASES * C_MOTOR_STEP_PER_PHASE) * C_MICROSTEP_PER_FULLSTEP))
static uint16_t l_u16MotorCurrentRawIdx;                                        /*!< Motor Current Moving-average filter sample-index */
#endif /* !defined (C_MOVAVG_SSZ) || (C_MOVAVG_SZ > ((C_MOTOR_PHASES * C_MOTOR_STEP_PER_PHASE) * C_MICROSTEP_PER_FULLSTEP)) */

#if (_SUPPORT_STALLDET_BRI != FALSE)
static uint8_t l_u8StallCountB = 0U;                                            /*!< [BEMF] Stall Detector "B" */
#endif /* (_SUPPORT_STALLDET_BRI != FALSE) */

#if (_SUPPORT_STALLDET_BZC != FALSE)
static uint16_t l_u16MaxBemfCommutTimerPeriod = 0U;                             /*!< [BEMF] */
static uint16_t l_u16NrOfFullStepCommut = 0U;                                   /*!< [BEMF] */
static uint16_t l_au16CommutTime[6];                                            /*!< [BEMF] */
static uint16_t l_u16CommutTimeIdx = 0U;                                        /*!< [BEMF] */
static uint32_t l_u32SumCommutTimerPeriods = 0U;                                /*!< [BEMF] */
static uint8_t l_u8StallCountB = 0U;                                            /*!< [BEMF] Stall Detector "B" */
#endif /* (_SUPPORT_STALLDET_BZC != FALSE) */

#if (_SUPPORT_FOC_MODE == FOC_MODE_NONE) && (_SUPPORT_SOFT_START != FALSE)
static uint16_t l_u16StartCorrectionRatio = 0U;                                 /*!< Soft-ramp correction ratio */
static uint16_t l_u16RampStep = C_SOFT_START_RAMP_STEPS;                        /*!< Soft-ramp steps */
#endif /* (_SUPPORT_FOC_MODE == FOC_MODE_NONE) && (_SUPPORT_SOFT_START != FALSE) */

#if (_DEBUG_MOTOR_CURRENT_FLT != FALSE)
uint8_t g_au8DebugBuf[C_DEBUG_BUF_SZ];                                          /*!< Debug Buffer; e.g. Raw (unfiltered) motor current measurement */
uint16_t g_u16DebugBufWrIdx;                                                    /*!< Debug Buffer Write Index */
#if (_DEBUG_MCUR_SUB_SAMPLED != FALSE)
uint16_t g_u16SubSamplingIdx;                                                   /*!< Modulo index */
#endif /* (_DEBUG_MCUR_SUB_SAMPLED != FALSE) */
#if (UART_COMM != FALSE) && (_SUPPORT_UART_IF_APP == C_UART_IF_SCOPE) && (_SUPPORT_UART_SCOPE_MODE == CH12_SCOPE)
uint16_t g_u16DebufBufUartTxIdx;                                                /*!< Debug Buffer UART Transmission index */
#endif /* (UART_COMM != FALSE) && (_SUPPORT_UART_IF_APP == C_UART_IF_SCOPE) && (_SUPPORT_UART_SCOPE_MODE == CH12_SCOPE) */
#endif /* (_DEBUG_MOTOR_CURRENT_FLT != FALSE) */

#if ((_SUPPORT_FOC_MODE & FOC_OBSERVER_MASK) == FOC_OBSERVER_SENSORED) || (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
FOC_STATIC uint16_t l_u16ResolverAngle;                                         /*!< Resolver Angle */
#endif /* ((_SUPPORT_FOC_MODE & FOC_OBSERVER_MASK) == FOC_OBSERVER_SENSORED) || (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */

#if (_SUPPORT_HALL_LATCH != FALSE) && (_SUPPORT_NR_OF_HL > 1U)
volatile uint8_t g_u8ZcHallFound = FALSE;                                       /*!< Zero-crossing Hall-Latch Found flag */
#if (_SUPPORT_MICRO_STEP_COMMUTATION != FALSE)
uint8_t g_u8HallMicroSteps = 0U;                                                /*!< Number of micro-steps between hall-latch edges */
#endif /* (_SUPPORT_MICRO_STEP_COMMUTATION != FALSE) */
#if (_SUPPORT_HALL_LATCH_DIAG == FALSE)
static uint32_t l_u32SumCommutTimerPeriods;                                     /*!< Sum of commutation periods */
uint16_t l_u16NrOfCommut;                                                       /*!< Number of commutation periods */
static uint16_t l_au16CommutTime[C_NR_OF_FULLSTEPS];                            /*!< Array of 6 last commutation periods */
static uint16_t l_u16CommutTimeIdx = 0U;                                        /*!< Array index for commutation periods */
#else  /* (_SUPPORT_HALL_LATCH_DIAG == FALSE) */
uint8_t l_au8MotorMicroStep[C_MOTOR_PHASES * C_MOTOR_STEP_PER_PHASE * C_MICROSTEP_PER_FULLSTEP];
#endif /* (_SUPPORT_HALL_LATCH_DIAG != FALSE) */
#endif /*(_SUPPORT_HALL_LATCH != FALSE) && (_SUPPORT_NR_OF_HL > 1U) */

#if (_SUPPORT_TRIAXIS_MLX9038x != FALSE) && (_DEBUG_MLX90381 != FALSE)
uint16_t l_u16CommutAngle;
uint16_t l_u16ActualAngle;
uint16_t l_u16PrevAngle;
uint16_t l_u16MinAngle;
uint16_t l_u16MaxAngle;
uint16_t l_u16AvgAngle;
uint32_t l_u32SumAngle;
uint16_t l_u16MicroStepAngle[C_MOTOR_PHASES * C_MOTOR_STEP_PER_PHASE * C_MICROSTEP_PER_FULLSTEP];
uint16_t l_u16PrevResolverAmpl;
#endif /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) && (_DEBUG_MLX90381 != FALSE) */

#if (_SUPPORT_CDI != FALSE)
uint16_t l_u16CDI = C_IO_ACTIVE_CDI_ACTIVE_CDI_PHASE_DIS;                       /*!< CDI State */
uint16_t l_u16MotorCurrentMovAvgxFS = 0U;                                       /*!< Full-step rotational current */
static uint16_t l_au16MotorCurrentRawFS[C_NR_OF_FULLSTEPS];                     /*!< Motor Current Moving-average samples (Full-step mode) */
static uint16_t l_u16MotorCurrentRawIdxFS;                                      /*!< Motor Current Moving-average filter sample-index */
#endif /* (_SUPPORT_CDI != FALSE) */
#pragma space none                                                              /* __NEAR_SECTION__ */

#if (_SUPPORT_PWM_SPREAD_SPECTRUM != FALSE)
#define C_PWM_STEP  8U                                                          /*!< PWM Spread Spectrum of 8 units of PWM_FREQ (24kHz)/RCO32_FREQ (28MHz), ~163 Hz */
/*! PWM Period for Spread Spectrum */
static const uint16_t au16PwmPeriod[] =
{
    (PWM_REG_PERIOD + (0U * C_PWM_STEP)),
    (PWM_REG_PERIOD + (1U * C_PWM_STEP)),
    (PWM_REG_PERIOD + (2U * C_PWM_STEP)),
    (PWM_REG_PERIOD + (3U * C_PWM_STEP)),
    (PWM_REG_PERIOD + (4U * C_PWM_STEP)),
    (PWM_REG_PERIOD + (3U * C_PWM_STEP)),
    (PWM_REG_PERIOD + (2U * C_PWM_STEP)),
    (PWM_REG_PERIOD + (1U * C_PWM_STEP)),
    (PWM_REG_PERIOD - (0U * C_PWM_STEP)),
    (PWM_REG_PERIOD - (1U * C_PWM_STEP)),
    (PWM_REG_PERIOD - (2U * C_PWM_STEP)),
    (PWM_REG_PERIOD - (3U * C_PWM_STEP)),
    (PWM_REG_PERIOD - (4U * C_PWM_STEP)),
    (PWM_REG_PERIOD - (3U * C_PWM_STEP)),
    (PWM_REG_PERIOD - (2U * C_PWM_STEP)),
    (PWM_REG_PERIOD - (1U * C_PWM_STEP))
};
#endif /* (_SUPPORT_PWM_SPREAD_SPECTRUM != FALSE) */

/*!*************************************************************************** *
 * MotorDriverConfig
 * \brief   Configure Motor Driver
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16State: FALSE: Disable driver
 *                         TRUE: Enable driver
 * \return  -
 * *************************************************************************** *
 * \details  1. Enable power and clock of driver stage
 *           2. (Optional) enable driver charge pump spread-spectrum (after
 *              10us delay)
 *           3. Enable VDDAF UV Diagnostic protection
 * *************************************************************************** *
 * - Call Hierarchy: MotorDriverStart(), MotorDriverStop(),
 *                  MotorDriverPeriodicTimer()
 * - Cyclomatic Complexity: 2+1
 * - Nesting: 2
 * - Function calling: 0
 * *************************************************************************** */
void MotorDriverConfig(uint16_t u16State)
{
    static uint16_t u16DriverState = FALSE;                                     /* Driver state by default is OFF */

    if ( (u16State != FALSE) && (u16DriverState == FALSE) ) /*lint !e845 */
    {
        /* Enable driver */
#if defined (__MLX81160__)
        if ( (IO_PORT_DRV_OUT & B_PORT_DRV_OUT_ENABLE_DRVSUP) == 0U)
        {
            IO_PORT_DRV_OUT = B_PORT_DRV_OUT_ENABLE_DRVSUP;                     /* Enable driver supply */
            DELAY(C_DELAY_10US);
#if (_SUPPORT_APP_USER_MODE != FALSE)
            ENTER_SECTION(SYSTEM_MODE); /*lint !e534 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
            IO_MLX16_ITC_PEND0_S = B_MLX16_ITC_PEND0_UV_VDDAF;
            IO_MLX16_ITC_MASK0_S |= B_MLX16_ITC_MASK0_UV_VDDAF;                 /* Enable VDDAF UV */
#if (_SUPPORT_APP_USER_MODE != FALSE)
            EXIT_SECTION(); /*lint !e438 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
        }
        IO_PORT_DRV_OUT = ((0U << 8) |                                          /* Drive-mode 0b00: No division for CPCLK */
/*                         (1U << 8) | */                                       /* Drive-mode 0b01: CPCLK is constantly divided by 4 */
/*                         (2U << 8) | */                                       /* Drive-mode 0b10: CPCLK starts at full speed, then is divided by 4, ~3us after enabling a HS FET */
                           B_PORT_DRV_OUT_ENABLE_DRVMOD_CPCLK |                 /* Enable driver clock */
                           B_PORT_DRV_OUT_ENABLE_DRVSUP);                       /* Enable driver supply */
        DELAY(C_DELAY_10US);
#if (_SUPPORT_CP_SSCM != FALSE)
        /* Driver SSCM2: Enable the spread spectrum modulation */
        IO_PORT_SSCM2_CONF |= B_PORT_SSCM2_CONF_SSCM2_EN;                       /* Enable the spread spectrum modulation */
#endif /* (_SUPPORT_CP_SSCM != FALSE) */
#elif defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__)
        /* MMP200602-1: Enable driver supply prior to the CP-Clock */
        if ( (IO_PORT_DRV_OUT & B_PORT_DRV_OUT_ENABLE_DRVSUP) == 0U)
        {
            IO_PORT_DRV_OUT = B_PORT_DRV_OUT_ENABLE_DRVSUP;                     /* Enable driver supply */
            DELAY(C_DELAY_10US);
#if (_SUPPORT_APP_USER_MODE != FALSE)
            ENTER_SECTION(SYSTEM_MODE); /*lint !e534 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
            IO_MLX16_ITC_PEND0_S = B_MLX16_ITC_PEND0_UV_VDDAF;
            IO_MLX16_ITC_MASK0_S |= B_MLX16_ITC_MASK0_UV_VDDAF;                 /* Enable VDDAF UV */
#if (_SUPPORT_APP_USER_MODE != FALSE)
            EXIT_SECTION(); /*lint !e438 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
        }
        IO_PORT_DRV_OUT = (B_PORT_DRV_OUT_ENABLE_LS_OC |                        /* Enable low-side FET VDS over-voltage / over-current detection */
                           B_PORT_DRV_OUT_ENABLE_HS_OC |                        /* Enable high-side FET VDS over-voltage / over-current detection */
#if (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM_BIPOLAR) || \
    ((_SUPPORT_PWM_MODE == SINGLE_COIL_PWM) && (C_NR_OF_DC_MOTORS == 1))
                           B_PORT_DRV_OUT_PARALLEL_MODE_DRV |                   /* Enable parallel driver mode */
#endif /* (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM_BIPOLAR) */
                           (0U << 9) |                                          /* Drive-mode 0b00: No division for CPCLK */
/*                         (1U << 9) | */                                       /* Drive-mode 0b01: CPCLK is constantly divided by 4 */
/*                         (2U << 9) | */                                       /* Drive-mode 0b10: CPCLK starts at full speed, then is divided by 4, ~3us after enabling a HS FET */
                           B_PORT_DRV_OUT_ENABLE_CSA |                          /* Enable current sense amplifier */
                           B_PORT_DRV_OUT_ENABLE_DRVMOD_CPCLK |                 /* Enable driver clock */
                           B_PORT_DRV_OUT_ENABLE_DRVSUP);                       /* Enable driver supply */
        DELAY(C_DELAY_10US);
#if (_SUPPORT_CP_SSCM != FALSE)
        /* Driver SSCM2: Enable the spread spectrum modulation */
        IO_PORT_SSCM2_CONF |= B_PORT_SSCM2_CONF_SSCM2_EN;                       /* Enable the spread spectrum modulation */
#endif /* (_SUPPORT_CP_SSCM != FALSE) */
#elif defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
#if (_DEBUG_CP != FALSE)
        DEBUG_SET_IO_A();
#endif /* (_DEBUG_CP != FALSE) */
        DRVCFG_DIS();
        IO_PORT_SUPP_CFG &= ~B_PORT_SUPP_CFG_UV_BOOST_FILT_SEL;                 /* Temporary reduce UV BOOST filter time (MMP219001-2) */
        IO_PORT_CP |= B_PORT_CP_EN_CP;
        DELAY(C_DELAY_10US);                                                    /* Wait BOOST UV filter delay time */
#if (_SUPPORT_APP_USER_MODE != FALSE)
        ENTER_SECTION(SYSTEM_MODE); /*lint !e534 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
        do
        {
            IO_MLX16_ITC_PEND0_S = B_MLX16_ITC_PEND0_UV_BOOST;
            DELAY(C_DELAY_10US);                                                /* Wait BOOST UV filter delay time */
        } while ( (IO_MLX16_ITC_PEND0_S & B_MLX16_ITC_PEND0_UV_BOOST) != 0U);
        IO_PORT_SUPP_CFG |= B_PORT_SUPP_CFG_UV_BOOST_FILT_SEL;                  /* Restore UV BOOST filter time (MMP219001-2) */
#if (_DEBUG_CP != FALSE)
        DEBUG_CLR_IO_A();
        DEBUG_SET_IO_A();
#endif /* (_DEBUG_CP != FALSE) */
        IO_MLX16_ITC_MASK0_S |= B_MLX16_ITC_MASK0_UV_BOOST;                     /* Enable BOOST UV */
        IO_MLX16_ITC_PEND3_S = B_MLX16_ITC_PEND3_OV_BOOST;
        IO_MLX16_ITC_MASK3_S |= B_MLX16_ITC_MASK3_OV_BOOST;                     /* Enable BOOST OV */
#if (_SUPPORT_APP_USER_MODE != FALSE)
        EXIT_SECTION(); /*lint !e438 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
#if (_DEBUG_CP != FALSE)
        DEBUG_CLR_IO_A();
#endif /* (_DEBUG_CP != FALSE) */
#endif
        u16DriverState = TRUE;
    }
    else if (u16State == FALSE)
    {
        /* Disable driver */
#if defined (__MLX81160__)
#if (_SUPPORT_EVB == MLX81160EVB_3P_3N)
        DRVCFG_DIS_3P_3N();
#else  /* (_SUPPORT_EVB == MLX81160EVB_3P_3N) */
        DRVCFG_DIS_RSTUVW();
#endif /* (_SUPPORT_EVB == MLX81160EVB_3P_3N) */
#if (_SUPPORT_DRVSUP_ALWAYS_ENA != FALSE)
        IO_PORT_DRV_OUT = B_PORT_DRV_OUT_ENABLE_DRVSUP;                         /* 6.2mA */
#else /* (_SUPPORT_DRVSUP_ALWAYS_ENA != FALSE) */
#if (_SUPPORT_APP_USER_MODE != FALSE)
        ENTER_SECTION(SYSTEM_MODE); /*lint !e534 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
        IO_MLX16_ITC_MASK0_S &= ~B_MLX16_ITC_MASK0_UV_VDDAF;                    /* Disable VDDAF UV */
#if (_SUPPORT_APP_USER_MODE != FALSE)
        EXIT_SECTION(); /*lint !e438 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
        IO_PORT_DRV_OUT = 0U;                                                   /* 5.8mA */
#endif /* (_SUPPORT_DRVSUP_ALWAYS_ENA != FALSE) */
        IO_PORT_SSCM2_CONF &= ~B_PORT_SSCM2_CONF_SSCM2_EN;                      /* Disable the spread spectrum modulation */
#elif defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__)
#if (C_MOTOR_PHASES == 3)
        DRVCFG_TRI_UVW();
        DRVCFG_DIS_UVW();
#else  /* (C_MOTOR_PHASES == 3) */
        DRVCFG_TRI_TUVW();
        DRVCFG_DIS_TUVW();
#endif /* (C_MOTOR_PHASES == 3) */
#if (_SUPPORT_DRVSUP_ALWAYS_ENA != FALSE)
        IO_PORT_DRV_OUT = B_PORT_DRV_OUT_ENABLE_DRVSUP;                         /* 6.2mA */
#else /* (_SUPPORT_DRVSUP_ALWAYS_ENA != FALSE) */
#if (_SUPPORT_APP_USER_MODE != FALSE)
        ENTER_SECTION(SYSTEM_MODE); /*lint !e534 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
        IO_MLX16_ITC_MASK0_S &= ~B_MLX16_ITC_MASK0_UV_VDDAF;                    /* Disable VDDAF UV */
#if (_SUPPORT_APP_USER_MODE != FALSE)
        EXIT_SECTION(); /*lint !e438 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
        IO_PORT_DRV_OUT = 0U;                                                   /* 5.8mA */
#endif /* (_SUPPORT_DRVSUP_ALWAYS_ENA != FALSE) */
        IO_PORT_SSCM2_CONF &= ~B_PORT_SSCM2_CONF_SSCM2_EN;                      /* Disable the spread spectrum modulation */
#elif defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
#if (_DEBUG_CP != FALSE)
        DEBUG_SET_IO_A();
        DEBUG_CLR_IO_A();
        DEBUG_SET_IO_A();
#endif /* (_DEBUG_CP != FALSE) */
        DRVCFG_DIS();
#if (_SUPPORT_APP_USER_MODE != FALSE)
        ENTER_SECTION(SYSTEM_MODE); /*lint !e534 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
        IO_MLX16_ITC_MASK0_S &= ~B_MLX16_ITC_MASK0_UV_BOOST;                    /* Disable BOOST UV */
        IO_MLX16_ITC_MASK3_S &= ~B_MLX16_ITC_MASK3_OV_BOOST;                    /* Disable BOOST OV */
#if (_SUPPORT_APP_USER_MODE != FALSE)
        EXIT_SECTION(); /*lint !e438 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
        IO_PORT_CP &= ~B_PORT_CP_EN_CP;                                         /* Disable Change pump (BOOST) */
#if (_DEBUG_CP != FALSE)
        DEBUG_CLR_IO_A();
#endif /* (_DEBUG_CP != FALSE) */
        /* IO_PORT_CURR_SENS &= ~B_PORT_CURR_SENS_EN_CSA; */                    /* Disable Current Sense amplifier (this will also disable OC comparator!) */
#endif
        u16DriverState = FALSE;
    }
    else
    {
        /* Nothing; Leave driver in same state */
    }
} /* End of MotorDriverConfig() */

/*!*************************************************************************** *
 * MotorDriverInit
 * \brief   Initialise Motor Driver
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16FullInit: FALSE: Setup only Actual-position
 *                             TRUE: Full initialisation
 * \return  -
 * *************************************************************************** *
 * \details Note: Function can only be called when Non Volatile Memory Write is inactive!
 * *************************************************************************** *
 * - Call Hierarchy: main_Init(), HandleMotorRequest()
 * - Cyclomatic Complexity: 2+1
 * - Nesting: 1
 * - Function calling: 1 (ConvShaftSteps2MicroSteps())
 * *************************************************************************** */
void MotorDriverInit(uint16_t u16FullInit)
{
#if (_SUPPORT_PWM_MODE != SINGLE_COIL_PWM)
#if (C_MICROSTEP_PER_FULLSTEP != 0)
    l_u16MotorFullStepsPerElecRotation = C_NR_OF_FULLSTEPS;
#else  /* (C_MICROSTEP_PER_FULLSTEP != 0) */
    l_u16MotorFullStepsPerElecRotation = 1U;
#endif /* (C_MICROSTEP_PER_FULLSTEP != 0) */
    g_u16NrOfMicroStepsPerFullStep = C_MICROSTEP_PER_FULLSTEP;
    g_u16MotorPolePairs = NV_POLE_PAIRS;
#if (_SUPPORT_STALLDET_BZC != FALSE) || (_SUPPORT_CDI != FALSE) || \
    (_SUPPORT_TACHO_OUT != FALSE)
    /* [BEMF|CDI|TACHO] */
    l_u16MotorFullStepsPerMechRotation = (uint16_t)p_MulU32_U16byU16(g_u16MotorPolePairs,
                                                                     l_u16MotorFullStepsPerElecRotation);
#endif /* (_SUPPORT_STALLDET_BZC != FALSE) || (_SUPPORT_CDI != FALSE) */
    l_u16MotorMicroStepsPerElecRotation = (uint16_t)p_MulU32_U16byU16(g_u16NrOfMicroStepsPerFullStep,
                                                                      l_u16MotorFullStepsPerElecRotation);
    l_u16MotorMicroStepsPerMechRotation = (uint16_t)p_MulU32_U16byU16(g_u16MotorPolePairs,
                                                                      l_u16MotorMicroStepsPerElecRotation);
#if (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE)
    if (l_u16MotorMicroStepsPerMechRotation >= (uint16_t)(65536UL >> 5) )       /* MMP211223-1: Overflow fix (e.g. >10 Pole-Pairs) */
    {
        l_u16ShaftRatiox512 =
            p_MulDivU16_U16byU16byU16( (NV_GEARBOX_RATIO << 5),
                                       (l_u16MotorMicroStepsPerMechRotation << 4),
                                       C_SHAFT_STEPS_PER_ROTATION);
    }
    else
    {
        l_u16ShaftRatiox512 =
            p_MulDivU16_U16byU16byU16( (NV_GEARBOX_RATIO << 4),
                                       (l_u16MotorMicroStepsPerMechRotation << 5),
                                       C_SHAFT_STEPS_PER_ROTATION);
    }
    l_u32ActualPosition = ConvShaftSteps2MicroSteps(g_u16ActualPosition);
    g_u32TargetPosition = ConvShaftSteps2MicroSteps(g_u16TargetPosition);
#endif /* (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) */
    l_u32MicroStepPeriodOneRPM = p_DivU32_U32byU16( (TIMER_CLOCK * 60UL), l_u16MotorMicroStepsPerMechRotation);
#if (_SUPPORT_CDI != FALSE)
    l_u32FullStepPeriodOneRPM = p_DivU32_U32byU16( (TIMER_CLOCK * 60UL), l_u16MotorFullStepsPerMechRotation);
#endif /* (_SUPPORT_CDI != FALSE) */
#endif /* (_SUPPORT_PWM_MODE != SINGLE_COIL_PWM) */
#if (_SUPPORT_CURRSPIKE_POSITION != FALSE)
    l_u16ShaftRatiox512 = 512U;
#elif (_SUPPORT_DUAL_HALL_LATCH_MLX92255 != FALSE)
    extern uint16_t l_u16HallLatchRatiox512;
    l_u16HallLatchRatiox512 = p_MulDivU16_U16byU16byU16( (NV_GEARBOX_RATIO << 3),
                                                         ((NV_SENSE_POLE_PAIRS * 2) << 6),
                                                         C_SHAFT_STEPS_PER_ROTATION); /* MMP181128-4 */
#endif
    g_u16MinSpeedRPM = NV_MIN_SPEED;
    g_u16LowSpeedRPM = NV_ACT_SPEED1;
#if (_SUPPORT_MOTOR_POSITION == C_MOTOR_POS_NONE)
    if ( (NV_ALIGNMENT_SPEED != 0) && (NV_ALIGNMENT_SPEED < g_u16MinSpeedRPM) )
    {
        g_u16StartupSpeedRPM = NV_ALIGNMENT_SPEED;
    }
    else
#endif /* (_SUPPORT_MOTOR_POSITION == C_MOTOR_POS_NONE) */
    {
        g_u16StartupSpeedRPM = g_u16MinSpeedRPM;
    }
#if (LINPROT == LIN22_SIMPLE_PCT) || (LINPROT == NOLIN) || (LINPROT == LIN2X_AIRVENT12)
    g_u16MaxSpeedRPM = NV_ACT_SPEED3;
#else  /* (LINPROT == LIN22_SIMPLE_PCT) || (LINPROT == NOLIN) || (LINPROT == LIN2X_AIRVENT12) */
    g_u16MaxSpeedRPM = NV_ACT_SPEED4;
#endif /* (LINPROT == LIN22_SIMPLE_PCT) || (LINPROT == NOLIN) || (LINPROT == LIN2X_AIRVENT12) */
#if (LINPROT == LIN2X_AIRVENT12)
    g_u16TorqueSpeedRPM = NV_ACT_TORQUE_SPEED;
#endif /* (LINPROT == LIN2X_AIRVENT12) */
#if (_SUPPORT_PWM_MODE != SINGLE_COIL_PWM)
    l_u16LowSpeedPeriod = p_DivU16_U32byU16(l_u32MicroStepPeriodOneRPM, g_u16StartupSpeedRPM) - 1U;
#endif /* (_SUPPORT_PWM_MODE != SINGLE_COIL_PWM) */
#if (C_MOTOR_PHASES != 1) && ((_SUPPORT_HALL_LATCH == FALSE) || (_SUPPORT_NR_OF_HL <= 1))
#if (_SUPPORT_FOC_MODE != FOC_MODE_NONE)
#if (_SUPPORT_CLOSED_LOOP_STARTUP != FALSE)
#if (_SUPPORT_AUTO_SPEED_FOC == FALSE)
    l_u16StartrupThrshldSpeedRPMe = (uint16_t)p_MulU32_U16byU16(g_u16LowSpeedRPM, g_u16MotorPolePairs) >> 1;  /* 50% of Speed-mode #1 */
#else  /* (_SUPPORT_AUTO_SPEED_FOC == FALSE) */
    l_u16StartrupThrshldSpeedRPMe = (uint16_t)p_MulU32_U16byU16(g_u16MinSpeedRPM, g_u16MotorPolePairs);  /* Min-Speed */
#endif /* (_SUPPORT_AUTO_SPEED_FOC == FALSE) */
#if (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
    l_u16SpeedUpdateAcc = (NV_ACCELERATION_STEPS - 1U);
    l_u16SpeedUpdateDec = (NV_DECELERATION_STEPS - 1U);
#endif /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */
#else  /* (_SUPPORT_CLOSED_LOOP_STARTUP != FALSE) */
    l_u16CloseLoopCommutTimerPeriod = p_DivU16_U32byU16(l_u32MicroStepPeriodOneRPM, g_u16LowSpeedRPM) - 1U;
    if (NV_ACCELERATION_PWR == 7U)
    {
        l_u16SpeedUpdateAcc = ((8U * C_MICROSTEP_PER_FULLSTEP) - 1U);
    }
    else
    {
        l_u16SpeedUpdateAcc = (NV_ACCELERATION_STEPS - 1U);
    }
    if (NV_DECELERATION_PWR == 7U)
    {
        l_u16SpeedUpdateDec = ((8U * C_MICROSTEP_PER_FULLSTEP) - 1U);
    }
    else
    {
        l_u16SpeedUpdateDec = (NV_DECELERATION_STEPS - 1U);
    }
    l_u16AccelerationConst = NV_ACCELERATION_CONST;
#endif /* (_SUPPORT_CLOSED_LOOP_STARTUP != FALSE) */
#else  /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) */
    l_u16SpeedUpdateAcc = (NV_ACCELERATION_STEPS - 1U);
    l_u16SpeedUpdateDec = (NV_DECELERATION_STEPS - 1U);
    l_u16AccelerationConst = NV_ACCELERATION_CONST;
#if (_SUPPORT_ACCELERATION == C_ACCELERATION_COSINE_CURVE)
    l_u16AccelerationMin = g_u16MinSpeedRPM;                                    /* Initial/minimum acceleration */
    l_u16AccelerationMax = NV_ACCELERATION_CONST;                               /* Maximum acceleration */
    l_u16AccelerationStep = 3U;                                                 /* Acceleration increase: 3 */
#endif /* (_SUPPORT_ACCELERATION == C_ACCELERATION_COSINE_CURVE) */
#endif /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) */
#elif (_SUPPORT_HALL_LATCH == FALSE) || (_SUPPORT_NR_OF_HL != 1)
    l_u16AccelerationConst = NV_ACCELERATION_CONST;
#endif /* (C_MOTOR_PHASES != 1) */
#if (_SUPPORT_REWIND != FALSE) || (_SUPPORT_STALL_REVERSE != FALSE)
    l_u16MotorRewindSteps = (uint16_t)mulU32_U16byU16(NV_REWIND_STEPS, g_u16NrOfMicroStepsPerFullStep);
    if (l_u16MotorRewindSteps < C_MOVAVG_SZ)
    {
        l_u16MotorRewindSteps = C_MOVAVG_SZ;
    }
#endif /* (_SUPPORT_REWIND != FALSE) || (_SUPPORT_STALL_REVERSE != FALSE) */
#if (_SUPPORT_PWM_MODE != SINGLE_COIL_PWM)
    /* Phase coil (including driver/FET RDS-on) in [10mR] (MMP240515-1) */
#if (_SUPPORT_COIL_UNIT_10mR != FALSE)
    l_u16PhaseCoilResistanceRT = NV_MOTOR_COIL_RTOT + C_FETS_RTOT;              /* Coil Resistance [10mR], including driver/FET RDS-on */
#elif (_SUPPORT_COIL_UNIT_100mR != FALSE)
    l_u16PhaseCoilResistanceRT = p_MulU16lo_U16byU16(NV_MOTOR_COIL_RTOT, 10U) + C_FETS_RTOT;   /* Coil Resistance [10mR], including driver/FET RDS-on */
#else  /* (_SUPPORT_COIL_UNIT_100mR != FALSE) */
    l_u16PhaseCoilResistanceRT = p_MulU16lo_U16byU16(NV_MOTOR_COIL_RTOT, 100U) + C_FETS_RTOT;  /* Coil Resistance [10mR], including driver/FET RDS-on */
#endif /* (_SUPPORT_COIL_UNIT_100mR != FALSE) */
    l_u16PhaseCoilResistanceAT = l_u16PhaseCoilResistanceRT;
#endif /* (_SUPPORT_PWM_MODE != SINGLE_COIL_PWM) */
#if (_SUPPORT_FOC_MODE != FOC_MODE_NONE) && (C_MOTOR_PHASES != 1)              /* FOC for Positioning Device */
    {
        uint16_t u16Value;
        /* u16mR = ((NV_MOTOR_COIL_RTOT * 1000[mR/R]) / 2) * (Get_MCurrGain()/(C_GMCURR_DIV * 10)) * (C_VOLTGAIN_DIV/Get_MotorVoltGainF())
         * NV_MOTOR_COIL_RTOT = Resistance between two phases in ohm[R]; Single coil for star-configuration is NV_MOTOR_COIL_RTOT/2 */
#if (_SUPPORT_COIL_UNIT_10mR != FALSE)
        u16Value = NV_MOTOR_COIL_RTOT;                                          /* Max 2.55 [R] = 255 [10mR] --> [mR] */
#elif (_SUPPORT_COIL_UNIT_100mR != FALSE)
        u16Value = p_MulU16lo_U16byU16(NV_MOTOR_COIL_RTOT, 10U);                /* Max 25.5 [R] = 255 [100mR] --> [mR] */
#else
        u16Value = p_MulU16lo_U16byU16(NV_MOTOR_COIL_RTOT, 100U);               /* Max 255 [R] = 255 [R] --> [mR] */
#endif
        /* l_u16mR_RT = (NV_MOTOR_COIL_RTOT/2 [10mR] * Get_MCurrGain()) / C_GMCURR_DIV;
         * l_u16mR_RT = (l_u16mR_RT * C_VOLTGAIN_DIV) / Get_MotorVoltGainF();
         * Multiply by 1000/1024 to replace the divide in the FOC-algorithm from 1000 to 1024; 1024/1000 = 128/125 (MMP210129-1) */
        u16Value = p_MulDivU16_U16byU16byU16(u16Value, Get_MCurrGain(), Get_MotorVoltGainF());
        /* l_u16mR_RT = p_MulDivU16_U16byU16byU16( u16Value, C_VOLTGAIN_DIV, (C_GMCURR_DIV * 2U)); */
        /* l_u16mR_RT = p_MulDivU16_U16byU16byU16( u16Value, (128U * C_VOLTGAIN_DIV), (125U * (C_GMCURR_DIV * 2U))); */ /* MMP210129-1 (Overflow for C_GMCURR_DIV > 256) */
        l_u16mR_RT = p_MulDivU16_U16byU16byU16(u16Value, ((128U / 2U) * C_VOLTGAIN_DIV), (125U * C_GMCURR_DIV));  /* MMP210129-1 */
        l_u16mR_AT = l_u16mR_RT;
        /* Vadc = (Iadc * Gcurr/(GmA * 1000) * X) * (G10mV*100/Gvolt)
         * X = 2 * pi * L[uH] * speed[eRPM] / (60[eRPM/eRPS] * 1000000[uH/H])
         * Vadc = (Iadc * Gcurr/(GmA * 1000) * (2 * 355 * L) * S / (60 * 1000000 * 113)) * ((32 * G10mV)/Gvolt)
         * Vadc = (Iadc * Gcurr * (2 * 355 * L) * S * (G10mV * 100) / ((GmA * 1000) * 60 * 1000000 * 113 * Gvolt)
         * Vadc = (Iadc * S * Gcurr * (2 * 355 * L) * (G10mV * 100) / ((GmA * 1000) * 60 * 1000000 * 113 * Gvolt)
         * X = (Gcurr * (2 * 355 * L) * (G10mV * 100) / ((GmA * 1000) * 60 * 1000000 * 113 * Gvolt)
         * 1/X = ((GmA * 1000) * 60 * 1000000 * 113 * Gvolt) / (Gcurr * (2 * 355 * L) * (G10mV * 100))
         * 1/(256 * X) = ((GmA * 1000) * 60 * 1000000 * 113 * Gvolt) / (Gcurr * (2 * 355 * L) * (G10mV * 100) * 256), C_L_SR = 8
         *
         * 1/(256 * X) = ((GmA * 5^3*2^3) * 3*5^1*2^2 * 5^6*2^6 * 113 * Gvolt) / (Gcurr * (2^1 * 5^1*71 * L) * (G10mV * 5^2*2^2) * 2^8)
         * 1/(256 * X) = ((GmA * 5^3*1) * 3*5^1*1 * 5^6*1 * 113 * Gvolt) / (Gcurr * (1 * 5^1*71 * L) * (G10mV * 5^2*1) * 1)
         * 1/(256 * X) = ((GmA * 1*1) * 3*5^1*1 * 5^6*1 * 113 * Gvolt) / (Gcurr * (1 * 1*71 * L) * (G10mV * 1*1) * 1)
         * 1/(256 * X) = (GmA * 3*5^1 * 5^6 * 113 * Gvolt) / (Gcurr * (71 * L) * G10mV)
         * 1/(256 * X) = (GmA * 26484375 * Gvolt) / (Gcurr * (71 * L) * G10mV)
         * CONST = ((26484375 * GmA) / (71 * G10mV)) >> C_Z_SR, C_Z_SR = 8
         * CONST = ((26484375 * 32) / (71 * 32)) >> C_Z_SR, C_Z_SR = 8 --> 1457.1069
         * 1/(256 * X) = ((CONST << C_Z_SR) * Gvolt) / (Gcurr * L)
         */
        u16Value = (uint16_t)(p_MulU32_U16byU16(Get_MCurrGain(), NV_MOTOR_COIL_LTOT) >> C_L_SR);
        l_u16mZ = p_MulDivU16_U16byU16byU16(C_Z_CONST, (Get_MotorVoltGainF() << (C_Z_SR - C_L_SR)), u16Value);
    }
#endif /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) && (C_MOTOR_PHASES != 1) */     /* FOC */

#if (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) && (_SUPPORT_ACT_SPEED_BY_LIN == FALSE)
#if (LINPROT == LIN22_SIMPLE_PCT) || (LINPROT == NOLIN)
    l_au16MotorSpeedRPM[0] = g_u16MinSpeedRPM;
    l_au16MotorSpeedRPM[1] = g_u16LowSpeedRPM;
    l_au16MotorSpeedRPM[2] = NV_ACT_SPEED2;
    l_au16MotorSpeedRPM[3] = NV_ACT_SPEED3;
#elif (LINPROT == LIN2X_HVAC49) || (LINPROT == LIN2X_HVAC52) || (LINPROT == LIN2X_AGS)
    l_au16MotorSpeedRPM[0] = g_u16MinSpeedRPM;
    l_au16MotorSpeedRPM[1] = g_u16LowSpeedRPM;
    l_au16MotorSpeedRPM[2] = NV_ACT_SPEED2;
    l_au16MotorSpeedRPM[3] = NV_ACT_SPEED3;
    l_au16MotorSpeedRPM[4] = g_u16MaxSpeedRPM;
#if (_SUPPORT_FOC_MODE == FOC_MODE_NONE) || !defined (C_SPEED_AUTO)
    l_au16MotorSpeedRPM[5] = g_u16LowSpeedRPM;                                  /* Auto-speed */
#else  /* (_SUPPORT_FOC_MODE == FOC_MODE_NONE) || !defined (C_SPEED_AUTO) */
    l_au16MotorSpeedRPM[5] = C_SPEED_AUTO;                                      /* Auto-speed */
#endif /* (_SUPPORT_FOC_MODE == FOC_MODE_NONE) || !defined (C_SPEED_AUTO) */
    l_au16MotorSpeedRPM[6] = g_u16MinSpeedRPM;
    l_au16MotorSpeedRPM[7] = g_u16MinSpeedRPM;
#elif (LINPROT == LIN2X_AIRVENT12)
    l_au16MotorSpeedRPM[0] = g_u16MinSpeedRPM;
    l_au16MotorSpeedRPM[1] = g_u16LowSpeedRPM;
    l_au16MotorSpeedRPM[2] = NV_ACT_SPEED2;
    l_au16MotorSpeedRPM[3] = g_u16MaxSpeedRPM;
    l_au16MotorSpeedRPM[4] = g_u16MinSpeedRPM;
    l_au16MotorSpeedRPM[5] = g_u16MinSpeedRPM;
    l_au16MotorSpeedRPM[6] = g_u16TorqueSpeedRPM;
    l_au16MotorSpeedRPM[7] = g_u16MinSpeedRPM;
#endif /* (LINPROT) */
#endif /* (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) && (_SUPPORT_ACT_SPEED_BY_LIN == FALSE) */

#if (PWM_LIMIT == PWM_LIMIT_BY_PWM)
#if (_SUPPORT_STALLDET_BZC != FALSE) || (_SUPPORT_PWM_MODE == BIPOLAR_TRIPHASE_ALLPWM_MIRROR) || \
    (_SUPPORT_PWM_MODE == TRIPLEPHASE_ALLPWM_MIRROR)
    l_u16MaxPwmRatio =
        (uint16_t)(p_MulU32_U16byU16(PWM_REG_PERIOD, (UserParams.act.u8PidUpperLimit + 1U)) >> (8U + 2U));  /* MMP181114-2: BEMF, Push-pull */
    if (l_u16MaxPwmRatio >= (PWM_REG_PERIOD >> 2) )
    {
        l_u16MaxPwmRatio = (PWM_REG_PERIOD >> 2);
    }
#else  /* (_SUPPORT_STALLDET_BZC != FALSE) */
    l_u16MaxPwmRatio =
        (uint16_t)(p_MulU32_U16byU16(PWM_REG_PERIOD, (UserParams.act.u8PidUpperLimit + 1U)) >> (8U + 1U));  /* MMP181114-2: Stepper */
#endif /* (_SUPPORT_STALLDET_BZC != FALSE) */
#if (_SUPPORT_BOOST_MODE == BOOST_MODE_WAVEFORM)
    l_u16MaxPwmCorrRatio =
        (uint16_t)(p_MulU32_U16byU16( (UserParams.act.u8PidUpperLimit + 1U),    /* 100% */
                                      PWM_REG_PERIOD) >> (8U - C_PID_FACTOR));
    l_u16MaxPwmCorrRatioBoost =
        (uint16_t)(p_MulU32_U16byU16( (UserParams.act.u8PidUpperLimit + 1U),    /* 125% */
                                      (PWM_REG_PERIOD + (PWM_REG_PERIOD >> 2))) >> (8U - C_PID_FACTOR));
#endif /* (_SUPPORT_BOOST_MODE == BOOST_MODE_WAVEFORM) */
#endif /* (PWM_LIMIT == PWM_LIMIT_BY_PWM) */

#if (_SUPPORT_TACHO_OUT != FALSE)
    MotorDriverTachoInit();
#endif /* (_SUPPORT_TACHO_OUT != FALSE) */

#if (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM)
    l_u16CorrectionRatio = NV_STARTUP_CORR_RATIO;
#endif /* (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM) */

    if (u16FullInit != FALSE)
    {
#if defined (__MLX81160__) || defined (__MLX81330__) || defined (__MLX81350__)
        IO_TRIM2_DRV = (IO_TRIM2_DRV & ~M_TRIM2_DRV_TRIM_SLWRT) | C_DRV_SLWRT;  /* Fastest (7) / Medium (0) / Slowest (8) */
#elif defined (__MLX81332__) || defined (__MLX81334__)
        IO_TRIM2_DRV = (IO_TRIM2_DRV & ~(M_TRIM2_DRV_TRIM_CP_SLWRT_DRV | M_TRIM2_DRV_TRIM_SLWRT)) |
                       (C_CP_SLWRT_DRV |                                        /* Fastest (7) / Medium (0) / Slowest (8) */
                        C_DRV_SLWRT);                                           /* Fastest (7) / Medium (0) / Slowest (8) */
#elif defined (__MLX81339__)
        IO_TRIM2_DRV = (IO_TRIM2_DRV & ~(M_TRIM2_DRV_TRIM_SLWRT)) |
                       (C_DRV_SLWRT << 2);                                      /* Fastest (7) / Medium (0) / Slowest (8) */
#endif /* defined (__MLX81160__) || defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81339__) */

#if (_DEBUG_PWM_TO_DRV_DELAY != FALSE)
        IO_PORT_IO_CFG1 = C_PORT_IO_CFG1_IO4_OUT_SEL_PWM_MSTR1;
        IO_PORT_IO_ENABLE |= B_PORT_IO_ENABLE_IO_ENABLE_4;
#endif /* (_DEBUG_PWM_TO_DRV_DELAY != FALSE) */

        l_u16CorrectionRatio = NV_MIN_CORR_RATIO;

        /* BLDC motor Commutation/Stepper timer */
        IO_CTIMER0_CTRL = C_TMRx_CTRL_MODE0;                                    /* Timer mode */
#if (_SUPPORT_APP_USER_MODE != FALSE)
        ENTER_SECTION(SYSTEM_MODE); /*lint !e534 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
        IO_MLX16_ITC_PRIO0_S = (IO_MLX16_ITC_PRIO0_S & ~M_MLX16_ITC_PRIO0_CTIMER0_3) |
                               C_MLX16_ITC_PRIO0_CTIMER0_3_PRIO4;               /* Set Timer0 priority to 4 (3..6); Same as PWM_SYNC_ISR */
        IO_MLX16_ITC_PEND1_S = B_MLX16_ITC_PEND1_CTIMER0_3;
        IO_MLX16_ITC_MASK1_S |= B_MLX16_ITC_MASK1_CTIMER0_3;                    /* Enable Timer interrupt */
#if (_SUPPORT_APP_USER_MODE != FALSE)
        EXIT_SECTION(); /*lint !e438 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */

        /* Setup Motor PWM */
        IO_PWM_MASTER1_CTRL = B_PWM_MASTER1_STOP;                               /* Disable Master 1 */
        IO_PWM_SLAVE1_CTRL = B_PWM_SLAVE1_STOP;                                 /* Disable Slave 1 */
        IO_PWM_SLAVE2_CTRL = B_PWM_SLAVE2_STOP;                                 /* Disable Slave 2 */
        IO_PWM_SLAVE3_CTRL = B_PWM_SLAVE3_STOP;                                 /* Disable Slave 3 */
#if defined (__MLX81160__)
        IO_PWM_SLAVE4_CTRL = B_PWM_SLAVE4_STOP;                                 /* Disable Slave 4 */
#endif /* defined (__MLX81160__) */
        IO_PWM_MASTER2_CTRL = B_PWM_MASTER2_STOP;                               /* Disable Master 2 */

#if (_SUPPORT_PWM_MODE == BIPOLAR_PWM_SINGLE_INDEPENDENT_GND) || (_SUPPORT_PWM_MODE == BIPOLAR_FULL_STEP_BEMF) || (_SUPPORT_PWM_MODE == BIPOLAR_HALF_STEP_BEMF)
        /* PWM in Mirror (M1 + S1) and Inverse Mirror (S2 + S3) (Dual Coil) */
        IO_PWM_MASTER1_CTRL = (PWM_PRESCALER << 8) | C_PWM_MASTER1_MODE_MIRROR;  /* U (R): Initialise the master pre-scaler ratio (Fck/8) */
#if defined (__MLX81160__)
        IO_PWM_SLAVE1_CTRL = B_PWM_SLAVE1_SLAVE | C_PWM_SLAVE1_MODE_MIRROR;     /* S */
        IO_PWM_MASTER2_CTRL = B_PWM_MASTER2_SLAVE | C_PWM_MASTER2_MODE_MIRROR | B_PWM_MASTER2_POL;  /* T */
        IO_PWM_SLAVE3_CTRL = B_PWM_SLAVE3_SLAVE | C_PWM_SLAVE3_MODE_MIRROR | B_PWM_SLAVE3_POL;  /* U */
#else  /* defined (__MLX81160__) */
#if (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UV_WT) || (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UV_TW)
        IO_PWM_SLAVE1_CTRL = B_PWM_SLAVE1_SLAVE | C_PWM_SLAVE1_MODE_MIRROR;     /* V */
        IO_PWM_SLAVE2_CTRL = B_PWM_SLAVE2_SLAVE | C_PWM_SLAVE2_MODE_MIRROR | B_PWM_SLAVE2_POL;  /* W */
        IO_PWM_SLAVE3_CTRL = B_PWM_SLAVE3_SLAVE | C_PWM_SLAVE3_MODE_MIRROR | B_PWM_SLAVE3_POL;  /* T */
#elif (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UT_VW)
        IO_PWM_SLAVE1_CTRL = B_PWM_SLAVE1_SLAVE | C_PWM_SLAVE1_MODE_MIRROR | B_PWM_SLAVE1_POL;
        IO_PWM_SLAVE2_CTRL = B_PWM_SLAVE2_SLAVE | C_PWM_SLAVE2_MODE_MIRROR | B_PWM_SLAVE2_POL;
        IO_PWM_SLAVE3_CTRL = B_PWM_SLAVE3_SLAVE | C_PWM_SLAVE3_MODE_MIRROR;
#elif (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UW_VT)
        IO_PWM_SLAVE1_CTRL = B_PWM_SLAVE1_SLAVE | C_PWM_SLAVE1_MODE_MIRROR | B_PWM_SLAVE1_POL;
        IO_PWM_SLAVE2_CTRL = B_PWM_SLAVE2_SLAVE | C_PWM_SLAVE2_MODE_MIRROR;
        IO_PWM_SLAVE3_CTRL = B_PWM_SLAVE3_SLAVE | C_PWM_SLAVE3_MODE_MIRROR | B_PWM_SLAVE3_POL;
#endif /* (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UV_WT) */
        IO_PWM_MASTER2_CTRL = B_PWM_MASTER2_SLAVE | C_PWM_MASTER2_MODE_MIRROR;  /* Used for ADC trigger */
#endif /* defined (__MLX81160__) */
        IO_PWM_MASTER1_PER = PWM_REG_PERIOD;
        /*                                      12.5%         33.3%   50.0%+D            66.7%   0%+D
         *                                      MASTER1       SLAVE1  SLAVE2             SLAVE3  MASTER2
         * BIPOLAR_PWM_SINGLE_INDEPENDENT_GND:  Vs,VDDA,VDDD  -       M-CurrA,VSMF,Tj    -       M-CurrB
         *                                      VS,IO-X,IO-Y  -       M-CurrA,VSMF,Tj    -       M-CurrB
         * BIPOLAR_FULL_STEP_BEMF:              Vs,VDDA,VDDD  -       MCurr,VSMF,Tj,U,V  -       -
         * BIPOLAR_HALF_STEP_BEMF:              Vs,VDDA,VDDD  -       MCurr,VSMF,Tj,U,V  -       -
         */
        IO_PWM_MASTER1_CMP = (((1UL * PWM_REG_PERIOD) + 4U) / 8U);              /* 12.5% of period */
        IO_PWM_SLAVE1_CMP = (((2UL * PWM_REG_PERIOD) + 3U) / 6U);               /* 33.3% of period */
        IO_PWM_SLAVE2_CMP = (((3UL * PWM_REG_PERIOD) + 3U) / 6U) + C_PWM_DCORR;  /* 50.0%+D of period */
        IO_PWM_SLAVE3_CMP = (((4UL * PWM_REG_PERIOD) + 3U) / 6U);               /* 66.7% of period */
        IO_PWM_MASTER2_CMP = C_PWM_DCORR;                                       /*  0.0%+D */
#elif (_SUPPORT_PWM_MODE == TRIPLEPHASE_ALLPWM_MIRROR) || (_SUPPORT_PWM_MODE == TRIPLEPHASE_FULL_STEP_BEMF) || \
        (_SUPPORT_PWM_MODE == BIPOLAR_TRIPHASE_ALLPWM_MIRROR) || (_SUPPORT_PWM_MODE == TRIPLEPHASE_FULL_STEP)
        /* All PWM in Mirror-mode (M1, S1 & S2; S3 not used) */
        IO_PWM_MASTER1_CTRL = (PWM_PRESCALER << 8) | C_PWM_MASTER1_MODE_MIRROR;  /* Initialise the master pre-scaler ratio (Fck/8) */
        IO_PWM_SLAVE1_CTRL = B_PWM_SLAVE1_SLAVE | C_PWM_SLAVE1_MODE_MIRROR;
        IO_PWM_SLAVE2_CTRL = B_PWM_SLAVE2_SLAVE | C_PWM_SLAVE2_MODE_MIRROR;
        IO_PWM_SLAVE3_CTRL = B_PWM_SLAVE3_SLAVE | C_PWM_SLAVE3_MODE_MIRROR;     /* Used for ADC trigger */
        IO_PWM_MASTER1_PER = PWM_REG_PERIOD;
        /*                                 12.5%    25.0%                      50.0%   75.0%
         *                                 MASTER1  SLAVE1                     SLAVE2  SLAVE3_CMP
         * TRIPLEPHASE_ALLPWM_MIRROR:
         * 1) _SUPPORT_STALLDET_BZC:       VS,VDDA,VDDD,VSMF,Tj                -       M-Curr
         * 2) FOC_MODE:                    VS       M-CurrA,VDDA,VSMF,Tj       -       M-CurrB
         *    or (SW-Trig)                          M-CurrA,VSMF,Tj            -       M-CurrB,VS,VDDA,VDDD
         * 3) Non-FOC/BSZ:                 VS,      M-CurrA,VDDA,VSMF,Tj       -       M-CurrB
         *    or (Resolver)                VS,      M-CurrA,IO-X,IO-Y,VSMF,Tj  -       M-CurrB
         *    or (SW-Trig)                          M-CurrA,VSMF,Tj            -       M-CurrB,VDDA,VDDD
         *    or (Resolver)                         M-CurrA,IO-X,IO-Y,VSMF,Tj  -       M-CurrB,VDDA,VDDD
         * TRIPLEPHASE_FULL_STEP_BEMF:     VS,VDDA,VDDD                        M-Curr,U|V|W,VSMF,Tj
         * TRIPLEPHASE_FULL_STEP:          VS,VDDA,VDDD                        M-Curr,VSMF,Tj
         * BIPOLAR_TRIPHASE_ALLPWM_MIRROR: VS       M-CurrA,VDDA,VSMF,Tj       -       M-CurrB
         * or (Resolver):                  VS       M-CurrA,IO-X,IO-Y,VSMF,Tj  -       M-CurrB
         * or (Middle)  :                  -        M-CurrA,VSMF,Tj            -       M-CurrB,VS
         * or (Min-Max) :                  -        M-CurrA,VSMF,Tj,           -       M-CurrB,VS,VDDA,VDDD
         */
#if (_SUPPORT_PWM_ADC_TRIGGER_MODE == ADC_TRIGGER_PWM_MIN_MAX)
        IO_PWM_MASTER1_CMP = (((1UL * PWM_REG_PERIOD) + 4U) / 8U);              /* 12.5% of period */
#elif (_SUPPORT_PWM_ADC_TRIGGER_MODE == ADC_TRIGGER_PWM_MID)
        IO_PWM_MASTER1_CMP = (((0UL * PWM_REG_PERIOD) + 4U) / 8U) + C_PWM_DCORR;  /* 0.0% of period */
#endif
        IO_PWM_SLAVE1_CMP = (((2UL * PWM_REG_PERIOD) + 4U) / 8U) + C_PWM_DCORR;  /* 25.0% of period */
        IO_PWM_SLAVE2_CMP = (((4UL * PWM_REG_PERIOD) + 4U) / 8U);               /* 50.0% of period */
        IO_PWM_SLAVE3_CMP = (((6UL * PWM_REG_PERIOD) + 4U) / 8U) + C_PWM_DCORR;  /* 75.0%+D of period */
#elif (_SUPPORT_PWM_MODE == TRIPLEPHASE_TWOPWM_MIRROR_SUP)
        IO_PWM_MASTER1_CTRL = (PWM_PRESCALER << 8) | C_PWM_MASTER1_MODE_MIRROR;  /* Initialise the master pre-scaler ratio (Fck/8) */
        IO_PWM_SLAVE1_CTRL = B_PWM_SLAVE1_SLAVE | C_PWM_SLAVE1_MODE_MIRROR;
        IO_PWM_SLAVE2_CTRL = B_PWM_SLAVE2_SLAVE | C_PWM_SLAVE2_MODE_MIRROR;
        IO_PWM_SLAVE3_CTRL = B_PWM_SLAVE3_SLAVE | C_PWM_SLAVE3_MODE_MIRROR;     /* Used for ADC trigger */
        IO_PWM_MASTER2_CTRL = B_PWM_MASTER2_SLAVE | C_PWM_MASTER2_MODE_MIRROR;  /* Used for ADC trigger */
        IO_PWM_MASTER1_PER = PWM_REG_PERIOD;
        /*              12.5%        33.33%      50.00%      66.67%      100%
         *              MASTER1_CMP  SLAVE1_CMP  SLAVE2_CMP  SLAVE3_CMP  MASTER2_CMP
         * TRIPLEPHASE_TWOPWM_MIRROR_SUP:
         *              VS,VDDA,VDDD,VSMF,Tj                             M-Curr
         * (Resolver)   VS,VDDA,VDDD,VSMF,Tj.IO-X,IO-Y                   M-Curr
         */
        IO_PWM_MASTER1_CMP = (((1UL * PWM_REG_PERIOD) + 4U) / 8U);              /* 12.5% of period */
        IO_PWM_SLAVE1_CMP = (((2UL * PWM_REG_PERIOD) + 3U) / 6U);               /* 33.3% of period */
        IO_PWM_SLAVE2_CMP = (((3UL * PWM_REG_PERIOD) + 3U) / 6U);               /* 50.0% of period */
        IO_PWM_SLAVE3_CMP = (((3UL * PWM_REG_PERIOD) + 2U) / 4U);               /* 75.0% of period */
        IO_PWM_MASTER2_CMP = C_PWM_DCORR;                                       /*  0.0%+D */
#elif (_SUPPORT_PWM_MODE == TRIPLEPHASE_TWOPWM_MIRROR_GND)
        IO_PWM_MASTER1_CTRL = (PWM_PRESCALER << 8) | C_PWM_MASTER1_MODE_MIRROR;  /* Initialise the master pre-scaler ratio (Fck/8) */
        IO_PWM_SLAVE1_CTRL = B_PWM_SLAVE1_SLAVE | C_PWM_SLAVE1_MODE_MIRROR;
        IO_PWM_SLAVE2_CTRL = B_PWM_SLAVE2_SLAVE | C_PWM_SLAVE2_MODE_MIRROR;
        IO_PWM_SLAVE3_CTRL = B_PWM_SLAVE3_SLAVE | C_PWM_SLAVE3_MODE_MIRROR;     /* Used for ADC trigger */
        IO_PWM_MASTER1_PER = PWM_REG_PERIOD;
        /*              12.5%         33.33%      50.00%                    66.7%
         *              MASTER1_CMP   SLAVE1_CMP  SLAVE2_CMP                SLAVE3_CMP
         * TRIPLEPHASE_TWOPWM_MIRROR_GND:
         *              VS,VDDA,VDDD  -           M-Curr,VSMF,TJ            -
         * (Resolver)   VS,VDDA,VDDD  -           M-Curr,VSMF,Tj,X-IO,Y-IO  -
         */
        IO_PWM_MASTER1_CMP = (((1UL * PWM_REG_PERIOD) + 4U) / 8U);              /* 12.5% of period */
        IO_PWM_SLAVE1_CMP = (((2UL * PWM_REG_PERIOD) + 3U) / 6U);               /* 33.3% of period */
        IO_PWM_SLAVE2_CMP = (((3UL * PWM_REG_PERIOD) + 3U) / 6U) + C_PWM_DCORR;  /* 50.0%+D of period */
        IO_PWM_SLAVE3_CMP = (((4UL * PWM_REG_PERIOD) + 3U) / 6U);               /* 66.7% of period */
#elif (_SUPPORT_PWM_MODE == TRIPLEPHASE_TWOPWM_INDEPENDENT_GND) || \
      (_SUPPORT_PWM_MODE == BIPOLAR_TRIPHASE_TWOPWM_INDEPENDENT_GND)
#if (_SUPPORT_PWM_MIRROR != FALSE)
        IO_PWM_MASTER1_CTRL = (PWM_PRESCALER << 8) | C_PWM_MASTER1_MODE_MIRROR;  /* Initialise the master pre-scaler ratio (Fck/8) */
        IO_PWM_SLAVE1_CTRL = B_PWM_SLAVE1_SLAVE | C_PWM_SLAVE1_MODE_MIRROR;
        IO_PWM_SLAVE2_CTRL = B_PWM_SLAVE2_SLAVE | C_PWM_SLAVE2_MODE_MIRROR | B_PWM_SLAVE2_POL;
#if defined (__MLX81160__) && (_SUPPORT_EVB == MLX81160EVB_3P_3N)
        IO_PWM_MASTER2_CTRL = (PWM_PRESCALER << 8) | C_PWM_MASTER2_MODE_MIRROR | B_PWM_MASTER2_SLAVE;
        IO_PWM_SLAVE3_CTRL = B_PWM_SLAVE3_SLAVE | C_PWM_SLAVE3_MODE_MIRROR;     /* Used for ADC trigger */
        IO_PWM_SLAVE4_CTRL = B_PWM_SLAVE4_SLAVE | C_PWM_SLAVE4_MODE_MIRROR | B_PWM_SLAVE4_POL;  /* Used for ADC trigger */
#else  /* defined (__MLX81160__) && (_SUPPORT_EVB == MLX81160EVB_3P_3N) */
        IO_PWM_SLAVE3_CTRL = B_PWM_SLAVE3_SLAVE | C_PWM_SLAVE3_MODE_MIRROR;     /* Used for ADC trigger */
        IO_PWM_MASTER2_CTRL = B_PWM_MASTER2_SLAVE | C_PWM_MASTER2_MODE_MIRROR;  /* Used for ADC trigger */
#endif /* defined (__MLX81160__) && (_SUPPORT_EVB == MLX81160EVB_3P_3N) */
#else  /* (_SUPPORT_PWM_MIRROR != FALSE) */
        IO_PWM_MASTER1_CTRL = (PWM_PRESCALER << 8) | C_PWM_MASTER1_MODE_INDEPENDENT;  /* Initialise the master pre-scaler ratio (Fck/8) */
        IO_PWM_SLAVE1_CTRL = B_PWM_SLAVE1_SLAVE | C_PWM_SLAVE1_MODE_INDEPENDENT;
        IO_PWM_SLAVE2_CTRL = B_PWM_SLAVE2_SLAVE | C_PWM_SLAVE2_MODE_INDEPENDENT | B_PWM_SLAVE2_POL;
#if defined (__MLX81160__) && (_SUPPORT_EVB == MLX81160EVB_3P_3N)
        IO_PWM_MASTER2_CTRL = (PWM_PRESCALER << 8) | C_PWM_MASTER2_MODE_INDEPENDENT | B_PWM_MASTER2_SLAVE;
        IO_PWM_SLAVE3_CTRL = B_PWM_SLAVE3_SLAVE | C_PWM_SLAVE3_MODE_INDEPENDENT;     /* Used for ADC trigger */
        IO_PWM_SLAVE4_CTRL = B_PWM_SLAVE4_SLAVE | C_PWM_SLAVE4_MODE_INDEPENDENT | B_PWM_SLAVE4_POL;  /* Used for ADC trigger */
#else  /* defined (__MLX81160__) && (_SUPPORT_EVB == MLX81160EVB_3P_3N) */
        IO_PWM_SLAVE3_CTRL = B_PWM_SLAVE3_SLAVE | C_PWM_SLAVE3_MODE_INDEPENDENT;     /* Used for ADC trigger */
        IO_PWM_MASTER2_CTRL = B_PWM_MASTER2_SLAVE | C_PWM_MASTER2_MODE_INDEPENDENT;  /* Used for ADC trigger */
#endif /* defined (__MLX81160__) && (_SUPPORT_EVB == MLX81160EVB_3P_3N) */
#endif /* (_SUPPORT_PWM_MIRROR != FALSE) */
        IO_PWM_MASTER1_PER = PWM_REG_PERIOD;
#if (_DEBUG_FOC_PERF != FALSE)
        /* IO[5]: Narrow pulse around PWM-END */
        IO_PWM_SLAVE3_CTRL = B_PWM_SLAVE3_SLAVE | C_PWM_SLAVE3_MODE_MIRROR | B_PWM_SLAVE3_POL;  /* Used for ADC trigger */
        IO_PORT_IO_CFG2 = (IO_PORT_IO_CFG2 & ~M_PORT_IO_CFG2_IO8_OUT_SEL) | C_PORT_IO_CFG2_IO8_OUT_SEL_PWM_SLV3;
        IO_PORT_IO_ENABLE1 |= B_PORT_IO_ENABLE1_IO_ENABLE_8;
        IO_PWM_SLAVE3_LT = (uint16_t)(C_PWM_MIN_DC >> 2);                       /* Narrow pulse at PWM_END */
#endif /* (_DEBUG_FOC_PERF != FALSE) */
#if (_DEBUG_FOC_IB_IQ != FALSE)
#if defined (__MLX81330__) || defined (__MLX81350__)
        /* Use PWM_Slave3 to set I/O[0] PWM signal with Duty-cycle based on Iq amplitude */
        IO_PORT_IO_CFG0 = (IO_PORT_IO_CFG0 & ~M_PORT_IO_CFG0_IO0_OUT_SEL) | C_PORT_IO_CFG0_IO0_OUT_SEL_PWM_SLV3;
        IO_PORT_IO_ENABLE |= B_PORT_IO_ENABLE_IO_ENABLE_0;
#elif defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81339__)
        /* Use PWM_Slave3 to set I/O[4] PWM signal with Duty-cycle based on Iq amplitude */
        IO_PORT_IO_CFG1 = (IO_PORT_IO_CFG1 & ~M_PORT_IO_CFG1_IO4_OUT_SEL) | C_PORT_IO_CFG1_IO4_OUT_SEL_PWM_SLV3;
        IO_PORT_IO_ENABLE |= B_PORT_IO_ENABLE_IO_ENABLE_4;
        IO_PORT_IO_CFG1 = (IO_PORT_IO_CFG1 & ~M_PORT_IO_CFG1_IO5_OUT_SEL) | C_PORT_IO_CFG1_IO5_OUT_SEL_PWM_MSTR2;
        IO_PORT_IO_ENABLE |= B_PORT_IO_ENABLE_IO_ENABLE_5;
#endif
#endif /* (_DEBUG_FOC_IB_IQ != FALSE) */
#if (_SUPPORT_ADC_PWM_DELAY != FALSE)
        /*              16.7%        33.3%(+D)  50.0%+D                    66.7%(+D)  0%+D
         *              MASTER1      SLAVE1     SLAVE2                     SLAVE3     MASTER2
         * TRIPLEPHASE_TWOPWM_INDEPENDENT_GND
         *              VS,VDDA,VDDD  -         M-CurrA,VSMF,Tj            -          M-CurrB
         * (Resolver)   VS,IO-X,IO-Y  -         M-CurrA,VSMF,Tj,VDDA,VDDA  -          M-CurrB
         * BIPOLAR_TRIPHASE_TWOPWM_INDEPENDENT_GND
         *              VS,VDDA,VDDD  -         M-CurrA,VSMF,Tj            -          M-CurrB
         * (Resolver)   VS,IO-X,IO-Y  -         M-CurrA,VSMF,Tj,VDDA,VDDD  -          M-CurrB
         * (2xResolver) VS,IO-X,IO-Y  -         M-CurrA,VSMF,Tj,IO-X,IO-Y  -          M-CurrB
         */
        IO_PWM_MASTER1_CMP = (((1UL * PWM_REG_PERIOD) + 4U) / 8U);              /* 12.5% of period */
        IO_PWM_SLAVE1_CMP = (((2UL * PWM_REG_PERIOD) + 3U) / 6U) + C_PWM_DCORR;  /* 33.3%+D of period */
        IO_PWM_SLAVE2_CMP = (((3UL * PWM_REG_PERIOD) + 3U) / 6U) + C_PWM_DCORR;  /* 50.0%+D of period */
        IO_PWM_SLAVE3_CMP = (((4UL * PWM_REG_PERIOD) + 3U) / 6U) + C_PWM_DCORR;  /* 66.7%+D of period */
#if defined (__MLX81160__)
        IO_PWM_SLAVE4_CMP = (((5UL * PWM_REG_PERIOD) + 3U) / 6U) + C_PWM_DCORR;  /* 83.3%+D of period */
#endif /* defined (__MLX81160__) */
        IO_PWM_MASTER2_CMP = C_PWM_DCORR;                                       /*  0.0%+D */
#else  /* (_SUPPORT_ADC_PWM_DELAY != FALSE) */
        /*              16.7%         33.3%   50.0%                      66.7%   100%
         *              MASTER1       SLAVE1  SLAVE2                     SLAVE3  MASTER2
         * TRIPLEPHASE_TWOPWM_INDEPENDENT_GND
         *              VS,VDDA,VDDD  -       M-CurrA,VSMF,Tj            -       M-CurrB
         * (Resolver)   VS,IO-X,IO-Y  -       M-CurrA,VSMF,Tj,VDDA,VDDA  -       M-CurrB
         * BIPOLAR_TRIPHASE_TWOPWM_INDEPENDENT_GND
         *              VS,VDDA,VDDD  -       M-CurrA,VSMF,Tj            -       M-CurrB
         * (Resolver)   VS,IO-X,IO-Y  -       M-CurrA,VSMF,Tj,VDDA,VDDD  -       M-CurrB
         * (2xResolver) VS,IO-X,IO-Y  -       M-CurrA,VSMF,Tj,IO-X,IO-Y  -       M-CurrB
         */
        IO_PWM_MASTER1_CMP = (((1UL * PWM_REG_PERIOD) + 4U) / 8U);              /* 12.5% of period */
        IO_PWM_SLAVE1_CMP = (((2UL * PWM_REG_PERIOD) + 3U) / 6U);               /* 33.3% of period */
        IO_PWM_SLAVE2_CMP = (((3UL * PWM_REG_PERIOD) + 3U) / 6U);               /* 50.0% of period */
        IO_PWM_SLAVE3_CMP = (((4UL * PWM_REG_PERIOD) + 3U) / 6U);               /* 66.7% of period */
        IO_PWM_MASTER2_CMP = (PWM_REG_PERIOD - 1U);                             /* 100.0% of period */
#endif /* (_SUPPORT_ADC_PWM_DELAY != FALSE) */
#elif (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM)
        IO_PWM_MASTER1_CTRL = (PWM_PRESCALER << 8) | C_PWM_MASTER1_MODE_MIRROR;  /* Initialise the master pre-scaler ratio (Fck/8) */
        IO_PWM_SLAVE1_CTRL = B_PWM_SLAVE1_SLAVE | C_PWM_SLAVE1_MODE_MIRROR;
        IO_PWM_SLAVE2_CTRL = B_PWM_SLAVE2_SLAVE | C_PWM_SLAVE2_MODE_MIRROR;
        IO_PWM_SLAVE3_CTRL = B_PWM_SLAVE3_SLAVE | C_PWM_SLAVE3_MODE_MIRROR;
        /*IO_PWM_MASTER2_CTRL = B_PWM_MASTER2_SLAVE | C_PWM_MASTER2_MODE_MIRROR;*/  /* Used for ADC trigger */
        IO_PWM_MASTER1_PER = PWM_REG_PERIOD;
        /*                 15.0%         30.0%   45.0%    55.0%                  80.0%
         *                 MASTER1       SLAVE1  SLAVE2   SLAVE3                 MASTER2
         * SINGLE_COIL_PWM
         *             (1) VS,VDDA,VDDD  -       M-CurrF  M-CurrF,VSMF,Tj,IO3    -       (IO3 = Potentiometer)
         *             (2) VS            -       M-CurrF  M-CurrF,VSMF,Tj,IO0HV  -       (IO0 = Analogue input)
         */
        IO_PWM_MASTER1_CMP = (((15UL * PWM_REG_PERIOD) + 50U) / 100U);          /* 15% of period */
        IO_PWM_SLAVE1_CMP = (((30UL * PWM_REG_PERIOD) + 50U) / 100U);           /* 30% of period */
        IO_PWM_SLAVE2_CMP = (((45UL * PWM_REG_PERIOD) + 50U) / 100U) + C_PWM_DCORR;  /* 45% of period */
        IO_PWM_SLAVE3_CMP = (((55UL * PWM_REG_PERIOD) + 50U) / 100U) + C_PWM_DCORR;  /* 55% of period */
        /*IO_PWM_MASTER2_CMP = (((80UL * PWM_REG_PERIOD) + 50U)/100U);*/        /* 80% of period */
#elif (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM_BIPOLAR)
        IO_PWM_MASTER1_CTRL = (PWM_PRESCALER << 8) | C_PWM_MASTER1_MODE_MIRROR;  /* Initialise the master pre-scaler ratio (Fck/8) */
        IO_PWM_SLAVE1_CTRL = B_PWM_SLAVE1_SLAVE | C_PWM_SLAVE1_MODE_MIRROR;
        IO_PWM_SLAVE2_CTRL = B_PWM_SLAVE2_SLAVE | C_PWM_SLAVE2_MODE_MIRROR;
        IO_PWM_SLAVE3_CTRL = B_PWM_SLAVE3_SLAVE | C_PWM_SLAVE3_MODE_MIRROR;
        /*IO_PWM_MASTER2_CTRL = B_PWM_MASTER2_SLAVE | C_PWM_MASTER2_MODE_MIRROR;*/  /* Used for ADC trigger */
        IO_PWM_MASTER1_PER = PWM_REG_PERIOD;
        /*                          12.5%         33.3%(+D)  50.0%(+D)       66.7%(+D)
         *                          MASTER1       SLAVE1     SLAVE2          SLAVE3_CMP
         * SINGLE_COIL_PWM_BIPOLAR  VS,VDDA,VDDD  -          M-Curr,VSMF,Tj  -
         */
        IO_PWM_MASTER1_CMP = (((1UL * PWM_REG_PERIOD) + 4U) / 8U);              /* 12.5% of period */
        IO_PWM_SLAVE1_CMP = (((2UL * PWM_REG_PERIOD) + 3U) / 6U) + C_PWM_DCORR;  /* 33.3%+D of period (unused) */
        IO_PWM_SLAVE2_CMP = (((2UL * PWM_REG_PERIOD) + 2U) / 4U) + C_PWM_DCORR;  /* 50.0%+D of period */
        IO_PWM_SLAVE3_CMP = (((4UL * PWM_REG_PERIOD) + 3U) / 6U) + C_PWM_DCORR;  /* 66.7%+D of period (unused) */
#else
#error "ERROR: Unsupported Motor-PWM Mode */"
#endif /* (_SUPPORT_PWM_MODE) */

#if (_SUPPORT_PWM_SYNC == FALSE)
#if (_SUPPORT_APP_USER_MODE != FALSE)
        ENTER_SECTION(SYSTEM_MODE); /*lint !e534 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
        IO_MLX16_ITC_PRIO1_S = (IO_MLX16_ITC_PRIO1_S & ~M_MLX16_ITC_PRIO1_PWM_MASTER1_END) |
                               C_MLX16_ITC_PRIO1_PWM_MASTER1_END_PRIO4;         /* Same priority as commutation timer */
#if (_SUPPORT_APP_USER_MODE != FALSE)
        EXIT_SECTION(); /*lint !e438 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
#endif /* (_SUPPORT_PWM_SYNC == FALSE) */

        IO_PWM_MASTER1_CTRL = B_PWM_MASTER1_START;                              /* Start PWM in application mode */
    }

#if (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE)
    if (g_u8MotorHoldingCurrEna != l_u8MotorHoldingCurrState)                   /* MMP190123-3 */
    {
        MotorDriverStop( (uint16_t)C_STOP_RAMPDOWN);
    }
#endif /* (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) */
} /* End of MotorDriverInit */

#if FALSE /* TODO[MMP] To be implemented */
void MotorDriverCheck(void)
{
    /* Check IO_PWM_MASTER1_CTRL */
    if ( (IO_PWM_MASTER1_CTRL & B_PWM_MASTER1_START) == 0U)
    {
        /* Master PWM is stopped */
    }
} /* End of MotorDriverCheck() */
#endif /* FALSE */

/*!*************************************************************************** *
 * MotorDriverPermanentError
 * \brief   Switch-off internal NN-Driver
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u8ElectricErrorCode: Electric Error code
 * \return  -
 * *************************************************************************** *
 * \details Disable Motor driver and report electric error.
 * *************************************************************************** *
 * - Call Hierarchy: main_Init(), MotorDriverStart()
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 0
 * *************************************************************************** */
void MotorDriverPermanentError(uint8_t u8ElectricErrorCode)
{
#if defined (__MLX81160__)
#if (_SUPPORT_EVB == MLX81160EVB_3P_3N)
    DRVCFG_DIS_3P_3N();                                                         /* Turn off motor-driver */
#else  /* (_SUPPORT_EVB == MLX81160EVB_3P_3N) */
    DRVCFG_DIS_RSTUVW();                                                        /* Turn off motor-driver */
#endif /* (_SUPPORT_EVB == MLX81160EVB_3P_3N) */
#elif defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__)
#if (C_MOTOR_PHASES == 3)
    DRVCFG_DIS_UVW();                                                           /* Turn off motor-driver */
#else  /* (C_MOTOR_PHASES == 3) */
    DRVCFG_DIS_TUVW();                                                          /* Turn off motor-driver */
#endif /* (C_MOTOR_PHASES == 3) */
#elif defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
    DRVCFG_DIS();                                                               /* Turn off motor-driver */
#endif
    g_e8ErrorElectric |= u8ElectricErrorCode;
} /* End of MotorDriverPermanentError() */

#if (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE)
#if (_SUPPORT_POS_INIT != FALSE)
/*!*************************************************************************** *
 * MotorDriverPosInit
 * \brief   Set Motor internal initial position
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16InitPosition: Set Initial Position
 * \return  -
 * *************************************************************************** *
 * \details -
 * *************************************************************************** *
 * - Call Hierarchy: HandleMotorRequest()
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 1 (ConvShaftSteps2MicroSteps())
 * *************************************************************************** */
void MotorDriverPosInit(uint16_t u16InitPosition)
{
#if (C_MOTOR_PHASES != 1)
    l_u32ActualPosition = ConvShaftSteps2MicroSteps(u16InitPosition);
#else  /* (C_MOTOR_PHASES != 1) */
#if (_SUPPORT_CURRSPIKE_POSITION != FALSE)
    extern uint16_t ConvShaftSteps2HallLatchSteps(uint16_t u16Position);
    g_u16ActualSpikePos = ConvShaftSteps2HallLatchSteps(u16InitPosition);
#elif (_SUPPORT_DUAL_HALL_LATCH_MLX92255 != FALSE) || (_SUPPORT_HALL_LATCH_MLX9227x != FALSE)
    extern uint16_t ConvShaftSteps2HallLatchSteps(uint16_t u16Position);
    g_u16ActualHallLatchPos = ConvShaftSteps2HallLatchSteps(u16InitPosition);
#endif
#endif /* (C_MOTOR_PHASES != 1) */
} /* End of MotorDriverPosInit() */
#endif /* (_SUPPORT_POS_INIT != FALSE) */

#if (C_MOTOR_PHASES != 1)
/*!*************************************************************************** *
 * ConvMicroSteps2ShaftSteps
 * \brief   Convert LIN communication steps to actuator steps
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details Shaft-step = Micro-step / Shaft-ratio.
 *          Shaft-ratio = (Nr_of_uSteps/e-rotation * Pole_Pairs *
 *          Gearbox-ratio) / Nr_of_shaft_steps/rotation
 * *************************************************************************** *
 * - Call Hierarchy: main(), MotorDriverStop()
 * - Cyclomatic Complexity: 1+1
 * - Nesting: 1
 * - Function calling: 0
 * *************************************************************************** */
void ConvMicroSteps2ShaftSteps(void)
{
    /* Convert 32-bit Micro-step into 16-bit shaft-step (cooling-step), including rounding at 0.5 */
    uint32_t u32ActualPosition = l_u32ActualPosition;                           /* take a only time copy */
    if (u32ActualPosition > C_ZERO_POS_OFFSET)
    {
        g_u16ActualPosition =
            p_DivU16_U32byU16( ((u32ActualPosition - C_ZERO_POS_OFFSET) << 9) + ((uint32_t)l_u16ShaftRatiox512 >> 1),
                               l_u16ShaftRatiox512);
    }
    else
    {
        g_u16ActualPosition = 0U;
    }
} /* End of ConvMicroSteps2ShaftSteps() */

/*!*************************************************************************** *
 * ConvShaftSteps2MicroSteps
 * \brief   Convert actuator steps to LIN communication steps
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16Position: 16-bit position
 * \return  uint32_t 32-bit (internal) position
 * *************************************************************************** *
 * \details Micro-step = Shaft-step * Shaft-ratio.
 *            Shaft-ratio = (Nr_of_uSteps/e-rotation * Pole_Pairs *
 *            Gearbox-ratio) / Nr_of_shaft_steps/rotation
 * *************************************************************************** *
 * - Call Hierarchy: main(), MotorDriverInit(), MotorDriverPosInit,
 *                   MotorDriverStart()
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 0
 * *************************************************************************** */
uint32_t ConvShaftSteps2MicroSteps(uint16_t u16Position)
{
    return ( ((p_MulU32_U16byU16(u16Position, l_u16ShaftRatiox512) + 256U) >> 9) + C_ZERO_POS_OFFSET);  /* MMP160902-2: Fix rounding; MMP161109-1 */
} /* End of ConvShaftSteps2MicroSteps() */

/*!*************************************************************************** *
 * DeltaPosition
 * \brief   Absolute distance between actual-position and target-position
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  uint16_t
 * *************************************************************************** *
 * \details Truncated to 65535
 *            The assembly-routine is 1.8us faster than the C-code
 * *************************************************************************** *
 * - Call Hierarchy: ISR_CTIMER0_3(), MotorDriverStop()
 * - Cyclomatic Complexity: 2+1
 * - Nesting: 1
 * - Function calling: 0
 * *************************************************************************** */
FOC_STATIC uint16_t DeltaPosition(void)
{
#if TRUE
    /* 1.8us faster */
    uint16_t u16Result;  /*lint -e530 */

    __asm__ __volatile__ (
        "lod  AL, dp:_g_e8MotorDirectionCCW\n\t"
        "jne  _DP_10\n\t"
        "lod  Y, #_g_u32TargetPosition\n\t"                                     /* Y = Address of g_u32TargetPosition */
        "lod  X, #_l_u32ActualPosition\n\t"                                     /* X = Address of l_u32ActualPosition */
        "jmp  _DP_20\n\t"                                                       /* CW: (g_u32TargetPosition - l_u32ActualPosition) */
        "_DP_10:\n\t"                                                           /* CCW: (l_u32ActualPosition - g_u32TargetPosition) */
        "lod  X, #_g_u32TargetPosition\n\t"                                     /* X = Address of g_u32TargetPosition */
        "lod  Y, #_l_u32ActualPosition\n\t"                                     /* Y = Address of l_u32ActualPosition */
        "_DP_20:\n\t"
        "mov  YA, [Y]\n\t"
        "sub  YA, [X]\n\t"
        "jsge _DP_30\n\t"                                                       /* if ( (iResult = YA) < 0 */
        "movu YA, #0\n\t"                                                       /* YA = 0 */
        "_DP_30:\n\t"
        "cmp  Y, #0x0000\n\t"                                                   /* if ( (iResult = YA) > 0x0000FFFF ) */
        "je   _DP_40\n\t"
        "mov  A, #0xFFFF\n\t"                                                   /* iResult = 0xFFFF */
        "_DP_40:"
        : "=a" (u16Result)
        :
        : "X", "Y"
        );
    return (u16Result);
#else
    int32_t i32Delta;
    if (g_e8MotorDirectionCCW != FALSE)
    {
        /* Counter Clock Wise */
        i32Delta = (int32_t)(l_u32ActualPosition - g_u32TargetPosition);
    }
    else
    {
        /* Clock Wise */
        i32Delta = (int32_t)(g_u32TargetPosition - l_u32ActualPosition);
    }
    if (i32Delta < 0)
    {
        i32Delta = 0;
    }
    else if (i32Delta > 0xFFFF)
    {
        i32Delta = 0xFFFF
    }
    return ( (uint16_t)i32Result);
#endif
} /* End of DeltaPosition() */
#endif /* (C_MOTOR_PHASES != 1) */
#endif /* (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) */

#if ((_SUPPORT_FOC_MODE & FOC_OBSERVER_MASK) == FOC_OBSERVER_SENSORED) || (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
/*!*************************************************************************** *
 * MotorDriverResolverAngleToMicroStepIndex
 * \brief   -
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16ResolverAngle: Resolver angle
 * \return  uint16_t u16MicroStepIdx: Micro-step index
 * *************************************************************************** *
 * \details -
 * *************************************************************************** *
 * - Call Hierarchy: MotorDriverStart()
 * - Cyclomatic Complexity: 7+1
 * - Nesting: 2
 * - Function calling: 0
 * *************************************************************************** */
FOC_STATIC uint16_t MotorDriverResolverAngleToMicroStepIndex(uint16_t u16ResolverAngle)
{
    uint16_t u16MicroStepIdx;
    uint16_t u16MicroStepIdxResolver;                                           /*!< Micro-step index resolver */

    if (NV_SENSE_POLE_PAIRS == g_u16MotorPolePairs)
    {
        u16MicroStepIdxResolver =
            (uint16_t)((p_MulU32_U16byU16( (u16ResolverAngle - g_i16ResolverAngleOffset),
                                           l_u16MotorMicroStepsPerElecRotation) + 32768U) >> 16);
        if (u16MicroStepIdxResolver >= l_u16MotorMicroStepsPerElecRotation)
        {
            u16MicroStepIdxResolver -= l_u16MotorMicroStepsPerElecRotation;
        }
    }
    else if (NV_SENSE_POLE_PAIRS == 1U)
    {
        u16MicroStepIdxResolver =
            (uint16_t)((p_MulU32_U16byU16( (u16ResolverAngle - g_i16ResolverAngleOffset),
                                           l_u16MotorMicroStepsPerMechRotation) + 32768U) >> 16);
        while (u16MicroStepIdxResolver >= l_u16MotorMicroStepsPerElecRotation)
        {
            u16MicroStepIdxResolver -= l_u16MotorMicroStepsPerElecRotation;
        }
    }
    else
    {
        u16MicroStepIdxResolver = 0U;
    }
#if (_SUPPORT_FOC_MODE != FOC_MODE_NONE)
    uint16_t u16Step = (l_u16MotorMicroStepsPerElecRotation >> 2);              /* 90 degrees shift */
#else  /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) */
    uint16_t u16Step = (l_u16MotorMicroStepsPerElecRotation >> 6);              /* 5.6 degrees shift */
#endif /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) */
    if (g_e8MotorDirectionCCW != FALSE)
    {
        /* Counter clock wise */
        if (u16MicroStepIdxResolver > u16Step)
        {
            u16MicroStepIdx = u16MicroStepIdxResolver - u16Step;
        }
        else
        {
            u16MicroStepIdx = (u16MicroStepIdxResolver + l_u16MotorMicroStepsPerElecRotation) - u16Step;
        }
#if (_SUPPORT_PWM_MODE == TRIPLEPHASE_TWOPWM_INDEPENDENT_GND) && (_SUPPORT_PWM_POL_CORR == FALSE)
        if ( (l_u16MicroStepIdx <= 176U) && (u16MicroStepIdx > 176U) )
        {
            l_u16PwmState = l_u16PwmState ^ TRUE;
        }
#endif /* (_SUPPORT_PWM_MODE == TRIPLEPHASE_TWOPWM_INDEPENDENT_GND) && (_SUPPORT_PWM_POL_CORR == FALSE) */
    }
    else
    {
        /* Clock wise */
        u16MicroStepIdx = u16MicroStepIdxResolver + u16Step;
        if (u16MicroStepIdx >= l_u16MotorMicroStepsPerElecRotation)
        {
            u16MicroStepIdx = u16MicroStepIdx - l_u16MotorMicroStepsPerElecRotation;
        }
#if (_SUPPORT_PWM_MODE == TRIPLEPHASE_TWOPWM_INDEPENDENT_GND) && (_SUPPORT_PWM_POL_CORR == FALSE)
        if ( (l_u16MicroStepIdx < 176U) && (u16MicroStepIdx >= 176U) )
        {
            l_u16PwmState = l_u16PwmState ^ TRUE;
        }
#endif /* (_SUPPORT_PWM_MODE == TRIPLEPHASE_TWOPWM_INDEPENDENT_GND) && (_SUPPORT_PWM_POL_CORR == FALSE) */
    }
    return (u16MicroStepIdx);
} /* End of MotorDriverUpdateMicroStepIndexResolver() */
#endif /* ((_SUPPORT_FOC_MODE & FOC_OBSERVER_MASK) == FOC_OBSERVER_SENSORED) || (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */

#if (_SUPPORT_PWM_MODE != SINGLE_COIL_PWM)
/*!*************************************************************************** *
 * MotorDriver_InitialPwmDutyCycle
 * \brief   Calculate initial motor PWM duty-cycle
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16CurrentLevel: Motor current level
 * \param   [in] u16MotorSpeed: Motor speed in RPM-mechanical
 * \return  -
 * *************************************************************************** *
 * \details Calculate Motor PWM (initial) Duty-cycle, based on motor current
 *          level and speed (BEMF)
 *          u16Losses is the (constant) ohmic losses in [10mV]
 *          u16Losses [10mV] = u16CurentLevel [mA] * l_u16PhaseCoilResistanceAT [10mR] / 1000
 *          u16Bemf is the speed dependent losses in [10mV]
 *          u16Bemf [10mV] = NV_MOTOR_CONSTANT [mV/rps] or [10mV/rps] * u16MotorSpeed [rpm] / 60 [sec/min]
 * *************************************************************************** *
 * - Call Hierarchy: MotorDriverStart(), MotorDriverStop()
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 1 (PID_Start())
 * *************************************************************************** */
#if (_SUPPORT_FOC_MODE != FOC_MODE_NONE) || (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM_BIPOLAR)
void MotorDriver_InitialPwmDutyCycle(uint16_t u16CurrentLevel, uint16_t u16MotorSpeed)
#else  /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) || (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM_BIPOLAR) */
static void MotorDriver_InitialPwmDutyCycle(uint16_t u16CurrentLevel, uint16_t u16MotorSpeed)
#endif /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) */
{
#if (C_MOTOR_PHASES == 3) || (C_MOTOR_PHASES == 1)
    /* BLDC */
    uint16_t u16Losses = p_MulDivU16_U16byU16byU16(l_u16PhaseCoilResistanceAT, u16CurrentLevel, 1000U);
#else  /* (C_MOTOR_PHASES == 3) || (C_MOTOR_PHASES == 1) */
       /* Bi-polar steppers */
    uint16_t u16Losses = p_MulDivU16_U16byU16byU16(l_u16PhaseCoilResistanceAT, u16CurrentLevel, 1414U);
#endif /* (C_MOTOR_PHASES == 3) || (C_MOTOR_PHASES == 1) */
#if (_SUPPORT_NV_NON_BLOCK_WRITE != FALSE)
#if defined (C_MOTOR_CONST_MV_PER_RPS)                                          /* MMP240726-2 */
    uint16_t u16Bemf = p_MulDivU16_U16byU16byU16(g_u16MotorConst_NV, u16MotorSpeed, (10U * 60U));  /* BEMF[10mV] = Kmotor[mV/RPSm] * Speed[RPM] / (10 [mV/10mV] * 60 [sec/min]) */
#else  /* defined (C_MOTOR_CONST_MV_PER_RPS) */
    uint16_t u16Bemf = p_MulDivU16_U16byU16byU16(g_u16MotorConst_NV, u16MotorSpeed, 60U);  /* BEMF[10mV] = Kmotor[10mV/RPSm] * Speed[RPM] / 60 [sec/min] */
#endif /* defined (C_MOTOR_CONST_MV_PER_RPS) */
#else  /* (_SUPPORT_NV_NON_BLOCK_WRITE != FALSE) */
#if defined (C_MOTOR_CONST_MV_PER_RPS)                                          /* MMP240726-2 */
    uint16_t u16Bemf = p_MulDivU16_U16byU16byU16(NV_MOTOR_CONSTANT, u16MotorSpeed, (10U * 60U));  /* BEMF[10mV] = Kmotor[mV/RPSm] * Speed[RPM] / (10 [mV/10mV] * 60 [sec/min]) */
#else  /* defined (C_MOTOR_CONST_MV_PER_RPS) */
    uint16_t u16Bemf = p_MulDivU16_U16byU16byU16(NV_MOTOR_CONSTANT, u16MotorSpeed, 60U);  /* BEMF[10mV] = Kmotor[10mV/RPSm] * Speed[RPM] / 60 [sec/min] */
#endif /* defined (C_MOTOR_CONST_MV_PER_RPS) */
#endif /* (_SUPPORT_NV_NON_BLOCK_WRITE != FALSE) */
    l_u16CorrectionRatio = PID_Start(u16Losses, u16Bemf);
} /* End of MotorDriver_InitialPwmDutyCycle() */
#endif /* (_SUPPORT_PWM_MODE != SINGLE_COIL_PWM) */

#if (C_MOTOR_PHASES == 3)

#if (_SUPPORT_PWM_MODE == BIPOLAR_TRIPHASE_TWOPWM_INDEPENDENT_GND)

#if (_SUPPORT_OPTIMIZE_FOR_SPEED != FALSE)
void MotorDriver_3Phase(uint16_t u16MicroStepIdx) __attribute__((aligned(8)));
#endif /* (_SUPPORT_OPTIMIZE_FOR_SPEED != FALSE) */

/*!*************************************************************************** *
 * MotorDriver_3Phase
 * \brief   Drive motor in micro-stepper mode (48/96/192-steps)
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16MicroStepIdx: Micro-step index
 * \return  -
 * *************************************************************************** *
 * \details Bipolar actuator using 3-phase with 2x PWM (Mirror & Inverse-Mirror)
 *          and GND (VOLTAGE_SHAPE_DSVM_GND)
 * *************************************************************************** *
 * - Call Hierarchy: ISR_CTIMER0_3(), MotorDriverStart(), MotorDriverStop(),
 *                   PID_COntrol()
 * - Cyclomatic Complexity: 2+1
 * - Nesting: 2
 * - Function calling: 1 (p_MulI16_I16byI16asr5())
 * *************************************************************************** */
void MotorDriver_3Phase(uint16_t u16MicroStepIdx)
{
    int16_t i16PwmU, i16PwmV, i16PwmW;
    int16_t *pi16Vector = (int16_t *)&c_au16MicroStepVector3PH_DualCoil_A[u16MicroStepIdx];
    i16PwmU = p_MulU16hi_U16byU16lsr4(*pi16Vector, l_u16CorrectionRatio);       /* U */
#if (PWM_LIMIT == PWM_LIMIT_BY_PWM)
    i16PwmU = p_ClipMinMaxI16(i16PwmU, 0, l_u16MaxPwmRatio);
#endif /* (PWM_LIMIT == PWM_LIMIT_BY_PWM) */
    uint16_t u16MicroStepIdxW = (SZ_MICRO_VECTOR_TABLE_3PH_2COIL - u16MicroStepIdx);
    pi16Vector = (int16_t *)&c_au16MicroStepVector3PH_DualCoil_A[u16MicroStepIdxW];
    i16PwmW = p_MulU16hi_U16byU16lsr4(*pi16Vector, l_u16CorrectionRatio);       /* W */
#if (PWM_LIMIT == PWM_LIMIT_BY_PWM)
    i16PwmW = p_ClipMinMaxI16(i16PwmW, 0, l_u16MaxPwmRatio);
#endif /* (PWM_LIMIT == PWM_LIMIT_BY_PWM) */
    pi16Vector = (int16_t *)&c_au16MicroStepVector3PH_DualCoil_B[u16MicroStepIdx];
    i16PwmV = p_MulU16hi_U16byU16lsr4(*pi16Vector, l_u16CorrectionRatio);       /* V */
#if (PWM_LIMIT == PWM_LIMIT_BY_PWM)
    i16PwmV = p_ClipMinMaxI16(i16PwmV, 0, l_u16MaxPwmRatio);
#endif /* (PWM_LIMIT == PWM_LIMIT_BY_PWM) */
    if (u16MicroStepIdx < ((9U * C_MICROSTEP_PER_FULLSTEP) / 4U) )
    {
#if (_SUPPORT_PWM_SYNC != FALSE)
        IO_PWM_SLAVE1_CTRL = B_PWM_SLAVE1_SLAVE | C_PWM_SLAVE1_MODE_MIRROR | B_PWM_SLAVE1_POL;
        IO_PWM_SLAVE2_LT = 0U;
        IO_PWM_SLAVE1_LT = (uint16_t)i16PwmV;
#else  /* (_SUPPORT_PWM_SYNC != FALSE) */
        MotorPwm.u16PwmSlave2 = 0U;
        MotorPwm.u16PwmSlave1 = (uint16_t)i16PwmV;
        MotorPwm.u16PwmCtrl = B_PWM_SLAVE1_SLAVE | C_PWM_SLAVE1_MODE_MIRROR | B_PWM_SLAVE1_POL;
#endif /* (_SUPPORT_PWM_SYNC != FALSE) */
        i16PwmU = (PWM_SCALE_OFFSET - i16PwmU);
    }
    else if (u16MicroStepIdx >= ((15U * C_MICROSTEP_PER_FULLSTEP) / 4U) )
    {
        i16PwmV = (PWM_SCALE_OFFSET - i16PwmV);
#if (_SUPPORT_PWM_SYNC != FALSE)
        IO_PWM_SLAVE2_LT = (uint16_t)i16PwmW;
        IO_PWM_SLAVE1_CTRL = B_PWM_SLAVE1_SLAVE | C_PWM_SLAVE1_MODE_MIRROR;
        IO_PWM_SLAVE1_LT = (uint16_t)i16PwmV;
#else  /* (_SUPPORT_PWM_SYNC != FALSE) */
        MotorPwm.u16PwmSlave2 = (uint16_t)i16PwmW;
        MotorPwm.u16PwmSlave1 = (uint16_t)i16PwmV;
        MotorPwm.u16PwmCtrl = B_PWM_SLAVE1_SLAVE | C_PWM_SLAVE1_MODE_MIRROR;
#endif /* (_SUPPORT_PWM_SYNC != FALSE) */
        i16PwmU = PWM_REG_PERIOD;
    }
    else
    {
#if (_SUPPORT_PWM_SYNC != FALSE)
        IO_PWM_SLAVE2_LT = (uint16_t)i16PwmW;
        IO_PWM_SLAVE1_CTRL = B_PWM_SLAVE1_SLAVE | C_PWM_SLAVE1_MODE_MIRROR | B_PWM_SLAVE1_POL;
        IO_PWM_SLAVE1_LT = 0U;
#else  /* (_SUPPORT_PWM_SYNC != FALSE) */
        MotorPwm.u16PwmSlave2 = (uint16_t)i16PwmW;
        MotorPwm.u16PwmSlave1 = 0U;
        MotorPwm.u16PwmCtrl = B_PWM_SLAVE1_SLAVE | C_PWM_SLAVE1_MODE_MIRROR | B_PWM_SLAVE1_POL;
#endif /* (_SUPPORT_PWM_SYNC != FALSE) */
        i16PwmU = (PWM_SCALE_OFFSET - i16PwmU);
    }

#if (_SUPPORT_PWM_SYNC != FALSE)
    IO_PWM_MASTER1_LT = (uint16_t)i16PwmU;                                        /* Master must be modified at last (value is not important) */
#else  /* (_SUPPORT_PWM_SYNC != FALSE) */
    MotorPwm.u16PwmMaster1 = (uint16_t)i16PwmU;
    HAL_PWM_MasterIrqEnable();
#endif /* (_SUPPORT_PWM_SYNC != FALSE) */
} /* End of MotorDriver_3Phase() */

#elif (_SUPPORT_PWM_MODE == BIPOLAR_TRIPHASE_ALLPWM_MIRROR)

#if (_SUPPORT_OPTIMIZE_FOR_SPEED != FALSE)
void MotorDriver_3Phase(uint16_t u16MicroStepIdx) __attribute__((aligned(8)));
#endif /* (_SUPPORT_OPTIMIZE_FOR_SPEED != FALSE) */

/*!*************************************************************************** *
 * MotorDriver_3Phase
 * \brief   Drive motor in micro-stepper mode (48/96/192-steps)
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16MicroStepIdx: Micro-step index
 * \return  -
 * *************************************************************************** *
 * \details Bipolar actuator using 3-phase with all 3 phase in mirror-mode
 *          (VOLTAGE_SHAPE_SINE or VOLTAGE_SHAPE_SVM)
 * *************************************************************************** *
 * - Call Hierarchy: ISR_CTIMER0_3(), MotorDriverStart(), MotorDriverStop(),
 *                   PID_COntrol()
 * - Cyclomatic Complexity: 2+1
 * - Nesting: 2
 * - Function calling: 1 (p_MulI16_I16byI16asr5())
 * *************************************************************************** */
void MotorDriver_3Phase(uint16_t u16MicroStepIdx)
{
    int16_t i16PwmU, i16PwmV, i16PwmW;
    int16_t *pi16Vector = (int16_t *)&c_ai16MicroStepVector3PH_DualCoil_A[u16MicroStepIdx];
    i16PwmW = p_MulI16hi_I16byI16asr5(*pi16Vector, l_u16CorrectionRatio);
    i16PwmU = PWM_OFFSET_25 - i16PwmW;                                          /* U */
#if (PWM_LIMIT == PWM_LIMIT_BY_PWM)
    i16PwmU = p_ClipMinMaxI16(i16PwmU, 0, l_u16MaxPwmRatio);
#endif /* (PWM_LIMIT == PWM_LIMIT_BY_PWM) */
    i16PwmW = PWM_OFFSET_25 + i16PwmW;                                          /* W */
#if (PWM_LIMIT == PWM_LIMIT_BY_PWM)
    i16PwmW = p_ClipMinMaxI16(i16PwmW, 0, l_u16MaxPwmRatio);
#endif /* (PWM_LIMIT == PWM_LIMIT_BY_PWM) */
    pi16Vector = (int16_t *)&c_ai16MicroStepVector3PH_DualCoil_B[u16MicroStepIdx];
    i16PwmV = PWM_OFFSET_25 - p_MulI16hi_I16byI16asr5(*pi16Vector, l_u16CorrectionRatio);   /* V */
#if (PWM_LIMIT == PWM_LIMIT_BY_PWM)
    i16PwmV = p_ClipMinMaxI16(i16PwmV, 0, l_u16MaxPwmRatio);
#endif /* (PWM_LIMIT == PWM_LIMIT_BY_PWM) */
#if (_SUPPORT_PWM_ADC_TRIGGER_MODE == ADC_TRIGGER_PWM_MIN_MAX)
    {
        /* Use PWM with DC between MIN and MAX; PWM Raising edge (LT) sample current of MAX-PWM to MIN-PWM; PWM Falling edge (HT = PER-LT) sample current of two PWM's to MIN-PWM */
        /*
         * @           *                       #           @           *
         * |   @   *       *               #       #   @       @   *       * U
         * |     *           *           #           #           *           *        ^
         * |   *   @           *       #           @   #       *   @           *      | Max
         * | *       @           *   #           @       #   *       @           *    |
         * |*         @           * #           @         # *         @           *   |
         * *---+---+---@---+---+---*---+---+---@---+---+---*---+---+---@---+---+---*----
         * 0#  4   8  12@ 16  20  #4* 28  32  @6  40  44  *0#  4   8  12@ 16  20  #4  |
         * | #           @       #   *       @           *   #           @       #    |
         * |   #           @   #       *   @           *       #           @   #      | Min
         * |     #           #           *           *           #           #        V
         * |       #       #   @       @   *       *               #       # W  @
         * |           #           @           *                       #           @ V
         *
         *  0..18: Phase W: Min PWM-DC --> CurrA: -W
         * 18..30: Phase V: Min PWM-DC --> CurrA: -V
         * 30..48: Phase U: Min PWM-DC --> CurrA: -U
         *  0.. 6: Phase V: Max PWM-DC --> CurrB: V
         *  6..24: Phase U: Max PWM-DC --> CurrB: U
         * 24..42: Phase W: Max PWM-DC --> CurrB: W
         * 42..48: Phase V: Max PWM-DC --> CurrB: V
         * Trigger CurrA: 25% .. 50%
         * Trigger CurrB: 75% .. 100%
         *
         * * * U-V   # # # W-V                           * * *       # # #
         * |   *   #       #                           *       *   #       #
         * |     *           #           @           *           *           #
         * |    # *      #    #  #   @       @   *  *    * U    # *      #    #  #
         * *   #   *   #       #   #           *   *       *   #   *   #       #   # ^
         * | *#     *#          #@   #       *   @*          *#     *#          #@    | Max
         * | # *   # *         @ #     #   *     * @         # *   # *         @ #    |
         * |#   * #   *       @   #     # *     *   @       #   * #   *       @   #   |
         * #---+-*-+---*---+-@-+---#---+-*-+---*---+-@-+---#---+-*-+---*---+-@-+---#----
         * 0   4# *8  12* 16@ 20  24# 28* 32  36  40  44  #0   4# *8  12* 16@ 20  24  |
         * |   #   *     * @         # *   # *         @ #     #   *     * @          |
         * | #       *   @*          *#     *#          #@   #       *   @*           | Min
         * #           *   *       *   #   *   #       #   #           *   *       *  V
         * | @       @   *  *    *      # *      #    #  # W  @      @   *  *    *
         * |     @           *           *           #           @ V         *
         * |                   *       *   #       #                           *
         * |                     * * *       # # #                               * *
         *
         *  0..12: Phase V: Min PWM-DC --> CurrA: -V
         * 12..30: Phase U: Min PWM-DC --> CurrA: -U
         * 30..48: Phase W: Min PWM-DC --> CurrA: -W
         *  0.. 6: Phase U: Max PWM-DC --> CurrB: U
         *  6..24: Phase W: Max PWM-DC --> CurrB: W
         * 24..36: Phase V: Max PWM-DC --> CurrB: V
         * 36..48: Phase U: Max PWM-DC --> CurrB: U
         */
        uint16_t u16Time;
        if (u16MicroStepIdx < ((9U * C_MICROSTEP_PER_FULLSTEP) / 4U) )
        {
            /* W-phase is PWM-MIN (Sink); CurrA = -W */
            u16Time = (uint16_t)i16PwmW;
        }
        else if (u16MicroStepIdx < ((15U * C_MICROSTEP_PER_FULLSTEP) / 4U) )
        {
            /* V-phase is PWM-MIN (Sink); CurrA = -V */
            u16Time = (uint16_t)i16PwmV;
        }
        else
        {
            /* U-phase is PWM-MIN (Sink); CurrA = -U */
            u16Time = (uint16_t)i16PwmU;
        }
        IO_PWM_SLAVE1_CMP = u16Time;

        if ( (u16MicroStepIdx < ((3U * C_MICROSTEP_PER_FULLSTEP) / 4U)) ||
             (u16MicroStepIdx >= ((21U * C_MICROSTEP_PER_FULLSTEP) / 4U)) )
        {
            /* V-phase is PWM-MAX (Source); CurrB = V */
            u16Time = (uint16_t)i16PwmV;
        }
        else if (u16MicroStepIdx < ((12U * C_MICROSTEP_PER_FULLSTEP) / 4U) )
        {
            /* U-phase is PWM-MAX (Source); CurrB = U */
            u16Time = (uint16_t)i16PwmU;
        }
        else /* if ( u16MicroStepIdx < ((21U * C_MICROSTEP_PER_FULLSTEP) / 4U) ) */
        {
            /* W-phase is PWM-MAX (Source); CurrB = W */
            u16Time = (uint16_t)i16PwmW;
        }
        IO_PWM_SLAVE2_CMP = (PWM_REG_PERIOD - u16Time);                         /* Use SLV2 (instead of SLV3), as MST1, SLV1 and SLV2 are updated (LT) */
    }
#elif (_SUPPORT_PWM_ADC_TRIGGER_MODE == ADC_TRIGGER_PWM_MID)
    /* Use PWM with DC between MIN and MAX; PWM Raising edge (LT) sample current of MAX-PWM to MIN-PWM; PWM Falling edge (HT = PER-LT) sample current of two PWM's to MIN-PWM */
    /*
     * @           *                       #           @           *
     * |   @   *       *               #       #   @       @   *       *
     * |     *           *           #           #           *           *
     * |   *   @           *       #           @   #       *   @           *
     * | *       @           *   #           @       #   *       @           *
     * |*         @           * #           @         # *         @           *  U
     * *---+---+---@---+---+---*---+---+---@---+---+---*---+---+---@---+---+---*
     * 0#  4   8  12@ 16  20  #4* 28  32  @6  40  44  *0#  4   8  12@ 16  20  #4
     * | #           @       #   *       @           *   #           @       #   W
     * |   #           @   #       *   @           *       #           @   #
     * |     #           #           *           *           #           #
     * |       #       #   @       @   *       *               #       #    @
     * |           #           @           *                       #           @ V
     *
     *  0.. 6: Phase U: Medium PWM-DC, Phase V: Max PWM-DC, Phase W: Min PWM-DC --> CurrA: V, CurrB: -W
     *  6..18: Phase V: Medium PWM-DC, Phase U: Max PWM-DC, Phase W: Min PWM-DC --> CurrA: U, CurrB: -W
     * 18..24: Phase W: Medium PWM-DC, Phase U: Max PWM-DC, Phase V: Min PWM-DC --> CurrA: U, CurrB: -V
     * 20..28: Phase U: Medium PWM-DC, Phase W: Max PWM-DC, Phase V: Min PWM-DC --> CurrA: W, CurrB: -V
     * 28..36: Phase V: Medium PWM-DC, Phase W: Max PWM-DC, Phase U: Min PWM-DC --> CurrA: W, CurrB: -U
     * 36..44: Phase W: Medium PWM-DC, Phase V: Max PWM-DC, Phase U: Min PWM-DC --> CurrA: V, CurrB: -U
     * 44..48: Phase U: Medium PWM-DC, Phase V: Max PWM-DC, Phase W: Min PWM-DC --> CurrA: V, CurrB: -W
     * Trigger CurrA: 12.5% .. 37.5%
     * Trigger CurrB: 62.5% .. 87.5%
     */
    if (u16MicroStepIdx < ((3U * C_MICROSTEP_PER_FULLSTEP) / 4U) )
    {
        IO_PWM_SLAVE1_CMP = i16PwmU;                                            /* ~25.0% of period */
        IO_PWM_SLAVE2_CMP = (PWM_REG_PERIOD - i16PwmU);                         /* ~75.0% of period */
    }
    else if (u16MicroStepIdx < ((9U * C_MICROSTEP_PER_FULLSTEP) / 4U) )
    {
        IO_PWM_SLAVE1_CMP = i16PwmV;                                            /* ~25.0% of period */
        IO_PWM_SLAVE2_CMP = (PWM_REG_PERIOD - i16PwmV);                         /* ~75.0% of period */
    }
    else if (u16MicroStepIdx < ((12U * C_MICROSTEP_PER_FULLSTEP) / 4U) )
    {
        IO_PWM_SLAVE1_CMP = i16PwmW;                                            /* ~25.0% of period */
        IO_PWM_SLAVE2_CMP = (PWM_REG_PERIOD - i16PwmW);                         /* ~75.0% of period */
    }
    else if (u16MicroStepIdx < ((15U * C_MICROSTEP_PER_FULLSTEP) / 4U) )
    {
        IO_PWM_SLAVE1_CMP = i16PwmU;                                            /* ~25.0% of period */
        IO_PWM_SLAVE2_CMP = (PWM_REG_PERIOD - i16PwmU);                         /* ~75.0% of period */
    }
    else if (u16MicroStepIdx < ((21U * C_MICROSTEP_PER_FULLSTEP) / 4U) )
    {
        IO_PWM_SLAVE1_CMP = i16PwmV;                                            /* ~25.0% of period */
        IO_PWM_SLAVE2_CMP = (PWM_REG_PERIOD - i16PwmV);                         /* ~75.0% of period */
    }
    else
    {
        IO_PWM_SLAVE1_CMP = i16PwmW;                                            /* ~25.0% of period */
        IO_PWM_SLAVE2_CMP = (PWM_REG_PERIOD - i16PwmW);                         /* ~75.0% of period */
    }
#else
#error "Error: Unsupported ADC Current triggers"
#endif

#if (_SUPPORT_PWM_SYNC != FALSE)
    IO_PWM_SLAVE1_LT = (uint16_t)i16PwmV;
    IO_PWM_SLAVE2_LT = (uint16_t)i16PwmW;
    IO_PWM_MASTER1_LT = (uint16_t)i16PwmU;                                        /* Master must be modified at last (value is not important) */
#else  /* (_SUPPORT_PWM_SYNC != FALSE) */
    MotorPwm.u16PwmSlave2 = (uint16_t)i16PwmW;
    MotorPwm.u16PwmSlave1 = (uint16_t)i16PwmV;
    MotorPwm.u16PwmMaster1 = (uint16_t)i16PwmU;
    HAL_PWM_MasterIrqEnable();
#endif /* (_SUPPORT_PWM_SYNC != FALSE) */
} /* End of MotorDriver_3Phase() */

#elif (_SUPPORT_PWM_MODE == TRIPLEPHASE_ALLPWM_MIRROR)

#if (_SUPPORT_OPTIMIZE_FOR_SPEED != FALSE)
void MotorDriver_3Phase(uint16_t u16MicroStepIdx) __attribute__((aligned(8)));
#endif /* (_SUPPORT_OPTIMIZE_FOR_SPEED != FALSE) */

/*!*************************************************************************** *
 * MotorDriver_3Phase
 * \brief   Drive motor in micro-stepper mode (48-steps)
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16MicroStepIdx: Micro-step index
 * \return  -
 * *************************************************************************** *
 * \details BLDC 3-Phase motor using 3-phase with:
 *          All three-phases in mirror mode (VOLTAGE_SHAPE_SINE or VOLTAGE_SHAPE_SVM)
 * *************************************************************************** *
 * - Call Hierarchy: ISR_CTIMER0_3(), MotorDriverStart(), MotorDriverStop(),
 *                   PID_COntrol()
 * - Cyclomatic Complexity: 2+1
 * - Nesting: 2
 * - Function calling: 1 (p_MulI16_I16byI16asr5())
 * *************************************************************************** */
void MotorDriver_3Phase(uint16_t u16MicroStepIdx)
{
#if (_SUPPORT_BRAKING != FALSE)
    if (g_e8MotorStatus == (uint8_t)C_MOTOR_STATUS_BRAKING)
    {
        /* Breaking-mode: Use 180 degrees shifted vector */
        u16MicroStepIdx += (3U * C_MICROSTEP_PER_FULLSTEP);
        if (u16MicroStepIdx >= (6U * C_MICROSTEP_PER_FULLSTEP) )
        {
            u16MicroStepIdx -= (6U * C_MICROSTEP_PER_FULLSTEP);
        }
    }
#endif /* (_SUPPORT_BRAKING != FALSE) */
    int16_t i16PwmU, i16PwmV, i16PwmW;
    int16_t *pi16Vector = (int16_t *)&c_ai16MicroStepVector3PH[u16MicroStepIdx];
#if (UART_COMM != FALSE) && ((_SUPPORT_UART_IF_APP == C_UART_IF_SCOPE) || (_SUPPORT_UART2_IF_APP == C_UART_IF_SCOPE))
    l_i16MotorVoltageCoilA = *pi16Vector;
#endif /* (UART_COMM != FALSE) && ((_SUPPORT_UART_IF_APP == C_UART_IF_SCOPE) || (_SUPPORT_UART2_IF_APP == C_UART_IF_SCOPE)) */
#if (PWM_REG_PERIOD >= (128U << (4U - PWM_PRESCALER_N)))                        /* (((PWM_REG_PERIOD * 256U) >> (4U - PWM_PRESCALER_N)) > 32767U) */
    i16PwmU = PWM_SCALE_OFFSET -
              (int16_t)(mulI32_I16byU16(*pi16Vector, l_u16CorrectionRatio) >> (20U + PWM_PRESCALER_N));                       /* U */
    pi16Vector += (2U * C_MICROSTEP_PER_FULLSTEP);
    i16PwmV = PWM_SCALE_OFFSET -
              (int16_t)(mulI32_I16byU16(*pi16Vector, l_u16CorrectionRatio) >> (20U + PWM_PRESCALER_N));                       /* V */
    pi16Vector += (2U * C_MICROSTEP_PER_FULLSTEP);
    i16PwmW = PWM_SCALE_OFFSET -
              (int16_t)(mulI32_I16byU16(*pi16Vector, l_u16CorrectionRatio) >> (20U + PWM_PRESCALER_N));                       /* W */
#elif (PWM_PRESCALER_N == 0)
    i16PwmU = PWM_OFFSET_25 - p_MulI16hi_I16byI16asr5(*pi16Vector, l_u16CorrectionRatio);        /* U */
    pi16Vector += (2U * C_MICROSTEP_PER_FULLSTEP);
    i16PwmV = PWM_OFFSET_25 - p_MulI16hi_I16byI16asr5(*pi16Vector, l_u16CorrectionRatio);        /* V */
    pi16Vector += (2U * C_MICROSTEP_PER_FULLSTEP);
    i16PwmW = PWM_OFFSET_25 - p_MulI16hi_I16byI16asr5(*pi16Vector, l_u16CorrectionRatio);        /* W */
#else  /* (PWM_PRESCALER_N == 0) */
    i16PwmU = PWM_SCALE_OFFSET -
              (int16_t)(mulI16_I16byI16(*pi16Vector, l_u16CorrectionRatio) >> (4U + PWM_PRESCALER_N));                        /* U */
    pi16Vector += (2U * C_MICROSTEP_PER_FULLSTEP);
    i16PwmV = PWM_SCALE_OFFSET -
              (int16_t)(mulI16_I16byI16(*pi16Vector, l_u16CorrectionRatio) >> (4U + PWM_PRESCALER_N));                        /* V */
    pi16Vector += (2U * C_MICROSTEP_PER_FULLSTEP);
    i16PwmW = PWM_SCALE_OFFSET -
              (int16_t)(mulI16_I16byI16(*pi16Vector, l_u16CorrectionRatio) >> (4U + PWM_PRESCALER_N));                        /* W */
#endif /* (PWM_PRESCALER_N == 0) */
#if (_SUPPORT_PWM_ADC_TRIGGER_MODE == ADC_TRIGGER_PWM_MIN_MAX)
    {
        /* Use PWM with DC between MIN and MAX; PWM Raising edge (LT) sample current of MAX-PWM to MIN-PWM; PWM Falling edge (HT = PER-LT) sample current of two PWM's to MIN-PWM */
        /*
         * |           *               #               @               *
         * @       *       *       #       #       @       @       *       *       # W
         * | @   *           *   #           #   @           @   *           *   #
         * |   *               *               #               *               *
         * | *   @           #   *           @   #           *   @           #   *
         * |*     @         #     *         @     #         *     @         #     *
         * *---+---@---+---#---+---*---+---@---+---#---+---*---+---@---+---#---+---* U
         * 0   4   8@ 12  #6  20  24* 28  @2  36  40# 44  *0   4   8@ 12  #6  20  24
         * |         @   #           *   @           #   *           @   #
         * |           #               *               *               #
         * |         #   @           @   *           *   #           #   @
         * #       #       @       @       *       *       #       #       @       @ V
         * |   #               @               *               #               @
         *
         *  0..12: Phase V: Min PWM-DC --> CurrA: -W
         * 12..28: Phase U: Min PWM-DC --> CurrA: -V
         * 28..44: Phase W: Min PWM-DC --> CurrA: -U
         * 44..48: Phase V: Min PWM-DC --> CurrA: -W
         *  0.. 4: Phase V: Max PWM-DC --> CurrB: V
         *  4..20: Phase U: Max PWM-DC --> CurrB: U
         * 20..36: Phase W: Max PWM-DC --> CurrB: W
         * 36..48: Phase V: Max PWM-DC --> CurrB: V
         * Trigger CurrA: 25% .. 50%
         * Trigger CurrB: 75% .. 100%
         */
        uint16_t u16Idx = (u16MicroStepIdx + (C_MICROSTEP_PER_FULLSTEP / 2U)) / (2U * C_MICROSTEP_PER_FULLSTEP);
        uint16_t u16Time;
        if (u16Idx == 1)
        {
            /* V-phase is PWM-MIN (Sink); CurrA = -V */
            u16Time = i16PwmV;
        }
        else if (u16Idx == 2)
        {
            /* U-phase is PWM-MIN (Sink); CurrA = -U */
            u16Time = i16PwmU;
        }
        else
        {
            /* W-phase is PWM-MIN (Sink); CurrA = -W */
            u16Time = i16PwmW;
        }
        IO_PWM_SLAVE1_CMP = u16Time;

        u16Idx = (u16MicroStepIdx + (C_MICROSTEP_PER_FULLSTEP + (C_MICROSTEP_PER_FULLSTEP / 2U))) /
                 (2U * C_MICROSTEP_PER_FULLSTEP);
        if (u16Idx == 1)
        {
            /* U-phase is PWM-MAX (Source); CurrB = U */
            u16Time = i16PwmU;
        }
        else if (u16Idx == 2)
        {
            /* W-phase is PWM-MAX (Source); CurrB = W */
            u16Time = i16PwmW;
        }
        else
        {
            /* V-phase is PWM-MAX (Source); CurrB = V */
            u16Time = i16PwmV;
        }
        IO_PWM_SLAVE2_CMP = (PWM_REG_PERIOD - u16Time);                         /* Use SLV2 (instead of SLV3), as MST1, SLV1 and SLV2 are updated (LT) */
    }
#elif (_SUPPORT_PWM_ADC_TRIGGER_MODE == ADC_TRIGGER_PWM_MID)
    /* Use PWM with DC between MIN and MAX; PWM Raising edge (LT) sample current of MAX-PWM to MIN-PWM; PWM Falling edge (HT = PER-LT) sample current of two PWM's to MIN-PWM */
    /*
     * |           *               #               @               *
     * @       *       *       #       #       @       @       *       *       # W
     * | @   *           *   #           #   @           @   *           *   #
     * |   *               *               #               *               *
     * | *   @           #   *           @   #           *   @           #   *
     * |*     @         #     *         @     #         *     @         #     *
     * *---+---@---+---#---+---*---+---@---+---#---+---*---+---@---+---#---+---* U
     * 0   4   8@ 12  #6  20  24* 28  @2  36  40# 44  *0   4   8@ 12  #6  20  24
     * |         @   #           *   @           #   *           @   #
     * |           #               *               *               #
     * |         #   @           @   *           *   #           #   @
     * #       #       @       @       *       *       #       #       @       @ V
     * |   #               @               *               #               @
     *
     *  0.. 4: Phase U: Medium PWM-DC, Phase V: Max PWM-DC, Phase W: Min PWM-DC --> CurrA: V, CurrB: -W
     *  4..12: Phase V: Medium PWM-DC, Phase U: Max PWM-DC, Phase W: Min PWM-DC --> CurrA: U, CurrB: -W
     * 12..20: Phase W: Medium PWM-DC, Phase U: Max PWM-DC, Phase V: Min PWM-DC --> CurrA: U, CurrB: -V
     * 20..28: Phase U: Medium PWM-DC, Phase W: Max PWM-DC, Phase V: Min PWM-DC --> CurrA: W, CurrB: -V
     * 28..36: Phase V: Medium PWM-DC, Phase W: Max PWM-DC, Phase U: Min PWM-DC --> CurrA: W, CurrB: -U
     * 36..44: Phase W: Medium PWM-DC, Phase V: Max PWM-DC, Phase U: Min PWM-DC --> CurrA: V, CurrB: -U
     * 44..48: Phase U: Medium PWM-DC, Phase V: Max PWM-DC, Phase W: Min PWM-DC --> CurrA: V, CurrB: -W
     * Trigger CurrA: 12.5% .. 37.5%
     * Trigger CurrB: 62.5% .. 87.5%
     */
    if (u16MicroStepIdx < (C_MICROSTEP_PER_FULLSTEP / 2U) )
    {
        IO_PWM_SLAVE1_CMP = i16PwmU;                                            /* ~25.0% of period */
        IO_PWM_SLAVE2_CMP = (PWM_REG_PERIOD - i16PwmU);                         /* ~75.0% of period */
    }
    else if (u16MicroStepIdx < (C_MICROSTEP_PER_FULLSTEP + (C_MICROSTEP_PER_FULLSTEP / 2U)) )
    {
        IO_PWM_SLAVE1_CMP = i16PwmV;                                            /* ~25.0% of period */
        IO_PWM_SLAVE2_CMP = (PWM_REG_PERIOD - i16PwmV);                         /* ~75.0% of period */
    }
    else if (u16MicroStepIdx < ((2U * C_MICROSTEP_PER_FULLSTEP) + (C_MICROSTEP_PER_FULLSTEP / 2U)) )
    {
        IO_PWM_SLAVE1_CMP = i16PwmW;                                            /* ~25.0% of period */
        IO_PWM_SLAVE2_CMP = (PWM_REG_PERIOD - i16PwmW);                         /* ~75.0% of period */
    }
    else if (u16MicroStepIdx < ((3U * C_MICROSTEP_PER_FULLSTEP) + (C_MICROSTEP_PER_FULLSTEP / 2U)) )
    {
        IO_PWM_SLAVE1_CMP = i16PwmU;                                            /* ~25.0% of period */
        IO_PWM_SLAVE2_CMP = (PWM_REG_PERIOD - i16PwmU);                         /* ~75.0% of period */
    }
    else if (u16MicroStepIdx < ((4U * C_MICROSTEP_PER_FULLSTEP) + (C_MICROSTEP_PER_FULLSTEP / 2U)) )
    {
        IO_PWM_SLAVE1_CMP = i16PwmV;                                            /* ~25.0% of period */
        IO_PWM_SLAVE2_CMP = (PWM_REG_PERIOD - i16PwmV);                         /* ~75.0% of period */
    }
    else if (u16MicroStepIdx < ((5U * C_MICROSTEP_PER_FULLSTEP) + (C_MICROSTEP_PER_FULLSTEP / 2U)) )
    {
        IO_PWM_SLAVE1_CMP = i16PwmW;                                            /* ~25.0% of period */
        IO_PWM_SLAVE2_CMP = (PWM_REG_PERIOD - i16PwmW);                         /* ~75.0% of period */
    }
    else
    {
        IO_PWM_SLAVE1_CMP = i16PwmU;                                            /* ~25.0% of period */
        IO_PWM_SLAVE2_CMP = (PWM_REG_PERIOD - i16PwmU);                         /* ~75.0% of period */
    }
#else
#error "Error: Unsupported ADC Current triggers"
#endif
#if (_SUPPORT_PWM_SYNC != FALSE)
    IO_PWM_SLAVE1_LT = (uint16_t)i16PwmV;
    IO_PWM_SLAVE2_LT = (uint16_t)i16PwmW;
    IO_PWM_MASTER1_LT = (uint16_t)i16PwmU;                                        /* Master must be modified at last (value is not important) */
#else  /* (_SUPPORT_PWM_SYNC != FALSE) */
    MotorPwm.u16PwmSlave2 = (uint16_t)i16PwmW;
    MotorPwm.u16PwmSlave1 = i16PwmV;
    MotorPwm.u16PwmMaster1 = (uint16_t)i16PwmU;
    HAL_PWM_MasterIrqEnable();
#endif /* (_SUPPORT_PWM_SYNC != FALSE) */
} /* End of MotorDriver_3Phase() */

#elif (_SUPPORT_PWM_MODE == TRIPLEPHASE_TWOPWM_MIRROR_SUP)

#if (_SUPPORT_OPTIMIZE_FOR_SPEED != FALSE)
void MotorDriver_3Phase(uint16_t u16MicroStepIdx) __attribute__((aligned(8)));
#endif /* (_SUPPORT_OPTIMIZE_FOR_SPEED != FALSE) */

/*!*************************************************************************** *
 * MotorDriver_3Phase
 * \brief   Drive motor in micro-stepper mode (48-steps)
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16MicroStepIdx: Micro-step index
 * \return  -
 * *************************************************************************** *
 * \details BLDC 3-Phase motor using 3-phase with:
 *          Two-phases in mirror mode and SUP (TRIPLEPHASE_TWOPWM_MIRROR_SUP & VOLTAGE_SHAPE_DSVM_SUP)
 * *************************************************************************** *
 * - Call Hierarchy: ISR_CTIMER0_3(), MotorDriverStart(), MotorDriverStop(),
 *                   PID_COntrol()
 * - Cyclomatic Complexity: 2+1
 * - Nesting: 2
 * - Function calling: 1 (p_MulI16_I16byI16asr5())
 * *************************************************************************** */
void MotorDriver_3Phase(uint16_t u16MicroStepIdx)
{
#if (_SUPPORT_BRAKING != FALSE)
    if (g_e8MotorStatus == (uint8_t)C_MOTOR_STATUS_BRAKING)
    {
        /* Breaking-mode: Use 180 degrees shifted vector */
        u16MicroStepIdx += (3U * C_MICROSTEP_PER_FULLSTEP);
        if (u16MicroStepIdx >= (6U * C_MICROSTEP_PER_FULLSTEP) )
        {
            u16MicroStepIdx -= (6U * C_MICROSTEP_PER_FULLSTEP);
        }
    }
#endif /* (_SUPPORT_BRAKING != FALSE) */
    int16_t i16PwmU, i16PwmV, i16PwmW;
    int16_t *pi16Vector = (int16_t *)&c_ai16MicroStepVector3PH[u16MicroStepIdx];
#if (PWM_REG_PERIOD >= (128U << (4U - PWM_PRESCALER_N)))                        /* (((PWM_REG_PERIOD * 256U) >> (4U - PWM_PRESCALER_N)) > 32767U) */
    i16PwmU = (int16_t)(mulI32_I16byU16(*pi16Vector, l_u16CorrectionRatio) >> (20U + PWM_PRESCALER_N));    /* U */
    pi16Vector += (2U * C_MICROSTEP_PER_FULLSTEP);
    i16PwmV = (int16_t)(mulI32_I16byU16(*pi16Vector, l_u16CorrectionRatio) >> (20U + PWM_PRESCALER_N));    /* V */
    pi16Vector += (2U * C_MICROSTEP_PER_FULLSTEP);
    i16PwmW = (int16_t)(mulI32_I16byU16(*pi16Vector, l_u16CorrectionRatio) >> (20U + PWM_PRESCALER_N));    /* W */
#elif (PWM_PRESCALER_N == 0)
    i16PwmU = p_MulI16hi_I16byI16asr5(*pi16Vector, l_u16CorrectionRatio);        /* U */
    pi16Vector += (2U * C_MICROSTEP_PER_FULLSTEP);
    i16PwmV = p_MulI16hi_I16byI16asr5(*pi16Vector, l_u16CorrectionRatio);        /* V */
    pi16Vector += (2U * C_MICROSTEP_PER_FULLSTEP);
    i16PwmW = p_MulI16hi_I16byI16asr5(*pi16Vector, l_u16CorrectionRatio);        /* W */
#else  /* (PWM_PRESCALER_N == 0) */
    i16PwmU = (int16_t)(mulI16_I16byI16(*pi16Vector, l_u16CorrectionRatio) >> (4U + PWM_PRESCALER_N));     /* U */
    pi16Vector += (2U * C_MICROSTEP_PER_FULLSTEP);
    i16PwmV = (int16_t)(mulI16_I16byI16(*pi16Vector, l_u16CorrectionRatio) >> (4U + PWM_PRESCALER_N));     /* V */
    pi16Vector += (2U * C_MICROSTEP_PER_FULLSTEP);
    i16PwmW = (int16_t)(mulI16_I16byI16(*pi16Vector, l_u16CorrectionRatio) >> (4U + PWM_PRESCALER_N));     /* W */
#endif /* (PWM_PRESCALER_N == 0) */
    IO_PWM_SLAVE1_LT = (uint16_t)i16PwmV;
    IO_PWM_SLAVE2_LT = (uint16_t)i16PwmW;
    IO_PWM_MASTER1_LT = (uint16_t)i16PwmU;                                        /* Master must be modified at last (value is not important) */
} /* End of MotorDriver_3Phase() */

#elif (_SUPPORT_PWM_MODE == TRIPLEPHASE_TWOPWM_MIRROR_GND)

#if (_SUPPORT_OPTIMIZE_FOR_SPEED != FALSE)
void MotorDriver_3Phase(uint16_t u16MicroStepIdx) __attribute__((aligned(8)));
#endif /* (_SUPPORT_OPTIMIZE_FOR_SPEED != FALSE) */

/*!*************************************************************************** *
 * MotorDriver_3Phase
 * \brief   Drive motor in micro-stepper mode (48-steps)
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16MicroStepIdx: Micro-step index
 * \return  -
 * *************************************************************************** *
 * \details BLDC 3-Phase motor using 3-phase with:
 *          Two-phases in mirror mode and GND (TRIPLEPHASE_TWOPWM_MIRROR_GND & VOLTAGE_SHAPE_DSVM_GND)
 * *************************************************************************** *
 * - Call Hierarchy: ISR_CTIMER0_3(), MotorDriverStart(), MotorDriverStop(),
 *                   PID_COntrol()
 * - Cyclomatic Complexity: 2+1
 * - Nesting: 2
 * - Function calling: 1 (p_MulI16_I16byI16asr5())
 * *************************************************************************** */
void MotorDriver_3Phase(uint16_t u16MicroStepIdx)
{
#if (_SUPPORT_BRAKING != FALSE)
    if (g_e8MotorStatus == (uint8_t)C_MOTOR_STATUS_BRAKING)
    {
        /* Breaking-mode: Use 180 degrees shifted vector */
        u16MicroStepIdx += (3U * C_MICROSTEP_PER_FULLSTEP);
        if (u16MicroStepIdx >= (6U * C_MICROSTEP_PER_FULLSTEP) )
        {
            u16MicroStepIdx -= (6U * C_MICROSTEP_PER_FULLSTEP);
        }
    }
#endif /* (_SUPPORT_BRAKING != FALSE) */
    uint16_t u16PwmU, u16PwmV, u16PwmW;
    uint16_t *pu16Vector = (uint16_t *)&c_au16MicroStepVector3PH[u16MicroStepIdx];
#if (PWM_REG_PERIOD >= (128U << (4U - PWM_PRESCALER_N)))                        /* (((PWM_REG_PERIOD * 256U) >> (4U - PWM_PRESCALER_N)) > 32767U) */
    u16PwmU = PWM_SCALE_OFFSET -
              (int16_t)(mulI32_I16byU16(*pu16Vector, l_u16CorrectionRatio) >> (20U + PWM_PRESCALER_N));                       /* U */
    pu16Vector += (2U * C_MICROSTEP_PER_FULLSTEP);
    u16PwmV = PWM_SCALE_OFFSET -
              (int16_t)(mulI32_I16byU16(*pu16Vector, l_u16CorrectionRatio) >> (20U + PWM_PRESCALER_N));                       /* V */
    pu16Vector += (2U * C_MICROSTEP_PER_FULLSTEP);
    u16PwmW = PWM_SCALE_OFFSET -
              (int16_t)(mulI32_I16byU16(*pu16Vector, l_u16CorrectionRatio) >> (20U + PWM_PRESCALER_N));                       /* W */
#elif (PWM_PRESCALER_N == 0)
    u16PwmU = (uint16_t)(PWM_SCALE_OFFSET - p_MulU16hi_U16byU16lsr4(*pu16Vector, l_u16CorrectionRatio));    /* U */
    pu16Vector += (2U * C_MICROSTEP_PER_FULLSTEP);
    u16PwmV = (uint16_t)(PWM_SCALE_OFFSET - p_MulU16hi_U16byU16lsr4(*pu16Vector, l_u16CorrectionRatio));    /* V */
    pu16Vector += (2U * C_MICROSTEP_PER_FULLSTEP);
    u16PwmW = (uint16_t)(PWM_SCALE_OFFSET - p_MulU16hi_U16byU16lsr4(*pu16Vector, l_u16CorrectionRatio));    /* W */
#else  /* (PWM_PRESCALER_N == 0) */
    u16PwmU = PWM_SCALE_OFFSET -
              (int16_t)(mulI16_I16byI16(*pu16Vector, l_u16CorrectionRatio) >> (4U + PWM_PRESCALER_N));                        /* U */
    pu16Vector += (2U * C_MICROSTEP_PER_FULLSTEP);
    u16PwmV = PWM_SCALE_OFFSET -
              (int16_t)(mulI16_I16byI16(*pu16Vector, l_u16CorrectionRatio) >> (4U + PWM_PRESCALER_N));                        /* V */
    pu16Vector += (2U * C_MICROSTEP_PER_FULLSTEP);
    u16PwmW = PWM_SCALE_OFFSET -
              (int16_t)(mulI16_I16byI16(*pu16Vector, l_u16CorrectionRatio) >> (4U + PWM_PRESCALER_N));                        /* W */
#endif /* (PWM_PRESCALER_N == 0) */
#if (_SUPPORT_PWM_SYNC != FALSE)
    IO_PWM_SLAVE1_LT = u16PwmV;
    IO_PWM_SLAVE2_LT = u16PwmW;
    IO_PWM_MASTER1_LT = u16PwmU;                                                /* Master must be modified at last (value is not important) */
#else  /* (_SUPPORT_PWM_SYNC != FALSE) */
    MotorPwm.u16PwmSlave2 = u16PwmW;
    MotorPwm.u16PwmSlave1 = u16PwmV;
    MotorPwm.u16PwmMaster1 = u16PwmU;
    HAL_PWM_MasterIrqEnable();
#endif /* _SUPPORT_PWM_SYNC */
} /* End of MotorDriver_3Phase() */

#elif (_SUPPORT_PWM_MODE == TRIPLEPHASE_TWOPWM_INDEPENDENT_GND) && (_SUPPORT_FOC_MODE == FOC_MODE_NONE)

#if (_SUPPORT_OPTIMIZE_FOR_SPEED != FALSE)
void MotorDriver_3Phase(uint16_t u16MicroStepIdx) __attribute__((aligned(8)));
#endif /* (_SUPPORT_OPTIMIZE_FOR_SPEED != FALSE) */

/*!*************************************************************************** *
 * MotorDriver_3Phase
 * \brief   Drive motor in micro-stepper mode (48-steps)
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16MicroStepIdx: Micro-step index
 * \return  -
 * *************************************************************************** *
 * \details BLDC 3-Phase motor using 3-phase with:
 *          Two-phases in mirror and inverse mirror mode and GND (TRIPLEPHASE_TWOPWM_INDEPENDENT_GND & VOLTAGE_SHAPE_DSVM_GND)
 *          Two-phases in mirror and inverse mirror mode and GND/SUP (TRIPLEPHASE_TWOPWM_INDEPENDENT_GND & VOLTAGE_SHAPE_DSVM_ALT)
 * *************************************************************************** *
 * - Call Hierarchy: ISR_CTIMER0_3(), MotorDriverStart(), MotorDriverStop(),
 *                   PID_COntrol()
 * - Cyclomatic Complexity: 2+1
 * - Nesting: 2
 * - Function calling: 1 (p_MulI16_I16byI16asr5())
 * *************************************************************************** */
void MotorDriver_3Phase(uint16_t u16MicroStepIdx)
{
#if (_SUPPORT_BRAKING != FALSE)
    if (g_e8MotorStatus == (uint8_t)C_MOTOR_STATUS_BRAKING)
    {
        /* Breaking-mode: Use 180 degrees shifted vector */
        u16MicroStepIdx += (3U * C_MICROSTEP_PER_FULLSTEP);
        if (u16MicroStepIdx >= (6U * C_MICROSTEP_PER_FULLSTEP) )
        {
            u16MicroStepIdx -= (6U * C_MICROSTEP_PER_FULLSTEP);
        }
    }
#endif /* (_SUPPORT_BRAKING != FALSE) */
    int16_t i16PwmU, i16PwmV, i16PwmW;
    uint16_t *pu16Vector = (uint16_t *)&c_au16MicroStepVector3PH[u16MicroStepIdx];
#if (_SUPPORT_ANTICOGGING != FALSE)
    int16_t i16AntiCoggingAmplitude = C_ANTI_COGGING_AMPLITUDE;                 /* Anti-cogging Amplitude */
    uint16_t u16AntiCoggingHarmonic =
        (u16MicroStepIdx * C_ANTI_COGGING_HARMONIC) + C_ANTI_COGGING_PHASE_SHIFT;  /* Anti-cogging 6th harmonic with phase-shift */
    if (g_e8MotorDirectionCCW != FALSE)
    {
        u16AntiCoggingHarmonic += (_SUPPORT_SINCOS_TABLE_SZ / 2U);
    }
#if (_SUPPORT_SPECIAL_COMM_FIELD != FALSE)
    if (g_u8Special[0] != 0U)
    {
        u16AntiCoggingHarmonic = u16MicroStepIdx * g_u8Special[0];              /* N-harmonic */
    }
    if (g_u8Special[1] != 0U)
    {
        i16AntiCoggingAmplitude = g_u8Special[1] * 8U;
    }
    if ((g_u8Special[2] != 0U) && (g_u8Special[2] < 192U))
    {
        u16AntiCoggingHarmonic += g_u8Special[2];
    }
#endif /* (_SUPPORT_SPECIAL_COMM_FIELD != FALSE) */
    while (u16AntiCoggingHarmonic >= (C_NR_OF_FULLSTEPS * C_MICROSTEP_PER_FULLSTEP))
    {
        u16AntiCoggingHarmonic -= (C_NR_OF_FULLSTEPS * C_MICROSTEP_PER_FULLSTEP);
    }
    i16AntiCoggingAmplitude = p_MulI16hi_I16byI16asr4(c_ai16MicroStepVector3PH_SinCos192[u16AntiCoggingHarmonic],
                                                      i16AntiCoggingAmplitude);
#endif /* (_SUPPORT_ANTICOGGING != FALSE) */
#if (PWM_REG_PERIOD >= (128U << (C_PID_FACTOR - PWM_PRESCALER_N)))              /* (((PWM_REG_PERIOD * 256U) >> (4U - PWM_PRESCALER_N)) > 32767U) */
    i16PwmU = (int16_t)(mulI32_I16byU16(*pu16Vector, l_u16CorrectionRatio) >> (20U + PWM_PRESCALER_N));    /* U */
    pi16Vector += (2 * C_MICROSTEP_PER_FULLSTEP);
    i16PwmV = (int16_t)(mulI32_I16byU16(*pu16Vector, l_u16CorrectionRatio) >> (20U + PWM_PRESCALER_N));    /* V */
    pi16Vector += (2 * C_MICROSTEP_PER_FULLSTEP);
    i16PwmW = (int16_t)(mulI32_I16byU16(*pu16Vector, l_u16CorrectionRatio) >> (20U + PWM_PRESCALER_N));    /* W */
#elif (PWM_PRESCALER_N == 0)
    i16PwmU = p_MulU16hi_U16byU16lsr4(*pu16Vector, l_u16CorrectionRatio);       /* U */
#if (_SUPPORT_ANTICOGGING != FALSE)
    i16PwmU += i16AntiCoggingAmplitude;
    if (i16PwmU < 0)
    {
        i16PwmU = 0;
    }
#endif /* (_SUPPORT_ANTICOGGING != FALSE) */
#if (PWM_LIMIT == PWM_LIMIT_BY_PWM)
    if (i16PwmU > (int16_t)l_u16MaxPwmRatio)                                    /* MMP181114-2 */
    {
        i16PwmU = l_u16MaxPwmRatio;
    }
#endif /* (PWM_LIMIT == PWM_LIMIT_BY_PWM) */
    pu16Vector += (2U * C_MICROSTEP_PER_FULLSTEP);
    i16PwmV = p_MulU16hi_U16byU16lsr4(*pu16Vector, l_u16CorrectionRatio);       /* V */
#if (_SUPPORT_ANTICOGGING != FALSE)
    i16PwmV += i16AntiCoggingAmplitude;
    if (i16PwmV < 0)
    {
        i16PwmV = 0;
    }
#endif /* (_SUPPORT_ANTICOGGING != FALSE) */
#if (PWM_LIMIT == PWM_LIMIT_BY_PWM)
    if (i16PwmV > (int16_t)l_u16MaxPwmRatio)                                    /* MMP181114-2 */
    {
        i16PwmV = l_u16MaxPwmRatio;
    }
#endif /* (PWM_LIMIT == PWM_LIMIT_BY_PWM) */
    pu16Vector += (2U * C_MICROSTEP_PER_FULLSTEP);
    i16PwmW = p_MulU16hi_U16byU16lsr4(*pu16Vector, l_u16CorrectionRatio);       /* W */
#if (_SUPPORT_ANTICOGGING != FALSE)
    i16PwmW += i16AntiCoggingAmplitude;
    if (i16PwmW < 0)
    {
        i16PwmW = 0;
    }
#endif /* (_SUPPORT_ANTICOGGING != FALSE) */
#if (PWM_LIMIT == PWM_LIMIT_BY_PWM)
    if (i16PwmW > (int16_t)l_u16MaxPwmRatio)                                    /* MMP181114-2 */
    {
        i16PwmW = l_u16MaxPwmRatio;
    }
#endif /* (PWM_LIMIT == PWM_LIMIT_BY_PWM) */
#else  /* (PWM_PRESCALER_N == 0) */
    i16PwmU = (int16_t)(mulI16_I16byI16(*pi16Vector, l_u16CorrectionRatio) >> (4U + PWM_PRESCALER_N));     /* U */
    pi16Vector += (2 * C_MICROSTEP_PER_FULLSTEP);
    i16PwmV = (int16_t)(mulI16_I16byI16(*pi16Vector, l_u16CorrectionRatio) >> (4U + PWM_PRESCALER_N));     /* V */
    pi16Vector += (2 * C_MICROSTEP_PER_FULLSTEP);
    i16PwmW = (int16_t)(mulI16_I16byI16(*pi16Vector, l_u16CorrectionRatio) >> (4U + PWM_PRESCALER_N));     /* W */
#endif /* (PWM_PRESCALER_N == 0) */

#if (_SUPPORT_VOLTAGE_SHAPE == VOLTAGE_SHAPE_DSVM_GND)
    u16MicroStepIdx = (u16MicroStepIdx + (C_MICROSTEP_PER_FULLSTEP / 2U)) / (2U * C_MICROSTEP_PER_FULLSTEP);
    if (u16MicroStepIdx == 1U)
    {
        /* Small V-phase low period --> V-phase to Low */
#if (SPACE_VECTOR_SINE == FALSE)
        i16PwmU = (i16PwmU - i16PwmV);
        i16PwmW = (i16PwmW - i16PwmV);
#endif /* (SPACE_VECTOR_SINE == FALSE) */
#if (_SUPPORT_PWM_POL_CORR == FALSE)
        if (l_u16PwmState != FALSE)
        {
            IO_PWM_MASTER1_CTRL = (PWM_PRESCALER << 8) | C_PWM_MASTER1_MODE_MIRROR | B_PWM_MASTER1_POL;
            IO_PWM_SLAVE2_CTRL = B_PWM_SLAVE2_SLAVE | C_PWM_SLAVE2_MODE_MIRROR;
            IO_PWM_SLAVE2_LT = (uint16_t)(PWM_SCALE_OFFSET - i16PwmW);
        }
        else
        {
            IO_PWM_MASTER1_CTRL = (PWM_PRESCALER << 8) | C_PWM_MASTER1_MODE_MIRROR;
            i16PwmU = (PWM_SCALE_OFFSET - i16PwmU);
            IO_PWM_SLAVE2_CTRL = B_PWM_SLAVE2_SLAVE | C_PWM_SLAVE2_MODE_MIRROR | B_PWM_SLAVE2_POL;
            IO_PWM_SLAVE2_LT = (uint16_t)i16PwmW;
        }
        IO_PWM_SLAVE1_CTRL = B_PWM_SLAVE1_SLAVE | C_PWM_SLAVE1_MODE_MIRROR | B_PWM_SLAVE1_POL;
        IO_PWM_SLAVE1_LT = 0U;
#else  /* (_SUPPORT_PWM_POL_CORR == FALSE) */
#if (_SUPPORT_EVB == MLX81160EVB_3P_3N)
        /* i16PwmU = i16PwmU; */
#if (_SUPPORT_PWM_SYNC != FALSE)
        IO_PWM_SLAVE2_LT = (uint16_t)(PWM_SCALE_OFFSET - i16PwmW);
        IO_PWM_SLAVE1_CTRL = B_PWM_SLAVE1_SLAVE | C_PWM_SLAVE1_MODE_MIRROR | B_PWM_SLAVE1_POL;
        IO_PWM_SLAVE1_LT = PWM_SCALE_OFFSET;
#else  /* (_SUPPORT_PWM_SYNC != FALSE) */
        MotorPwm.u16PwmSlave2 = (uint16_t)(PWM_SCALE_OFFSET - i16PwmW);
        MotorPwm.u16PwmCtrl = B_PWM_SLAVE1_SLAVE | C_PWM_SLAVE1_MODE_MIRROR | B_PWM_SLAVE1_POL;
        MotorPwm.u16PwmSlave1 = PWM_SCALE_OFFSET;
#endif /* (_SUPPORT_PWM_SYNC != FALSE) */
#else  /* (_SUPPORT_EVB == MLX81160EVB_3P_3N) */
        i16PwmU = (PWM_SCALE_OFFSET - i16PwmU);
#if (_SUPPORT_PWM_SYNC != FALSE)
        IO_PWM_SLAVE2_LT = (uint16_t)i16PwmW;
        IO_PWM_SLAVE1_CTRL = B_PWM_SLAVE1_SLAVE | C_PWM_SLAVE1_MODE_MIRROR | B_PWM_SLAVE1_POL;
        IO_PWM_SLAVE1_LT = 0U;
#else  /* (_SUPPORT_PWM_SYNC != FALSE) */
        MotorPwm.u16PwmSlave2 = (uint16_t)i16PwmW;
        MotorPwm.u16PwmSlave1 = 0U;
        MotorPwm.u16PwmCtrl = B_PWM_SLAVE1_SLAVE | C_PWM_SLAVE1_MODE_MIRROR | B_PWM_SLAVE1_POL;
#endif /* (_SUPPORT_PWM_SYNC != FALSE) */
#endif /* (_SUPPORT_EVB == MLX81160EVB_3P_3N) */
#endif /* (_SUPPORT_PWM_POL_CORR == FALSE) */
    }
    else if (u16MicroStepIdx == 2U)
    {
        /* Small U-phase low period --> U-phase to Low */
#if (SPACE_VECTOR_SINE == FALSE)
        i16PwmV = (i16PwmV - i16PwmU);
        i16PwmW = (i16PwmW - i16PwmU);
#endif /* (SPACE_VECTOR_SINE == FALSE) */
#if (_SUPPORT_PWM_POL_CORR == FALSE)
        if (l_u16PwmState != FALSE)
        {
            IO_PWM_SLAVE2_CTRL = B_PWM_SLAVE2_SLAVE | C_PWM_SLAVE2_MODE_MIRROR;
            IO_PWM_SLAVE2_LT = (uint16_t)(PWM_SCALE_OFFSET - i16PwmW);
            IO_PWM_SLAVE1_CTRL = B_PWM_SLAVE1_SLAVE | C_PWM_SLAVE1_MODE_MIRROR | B_PWM_SLAVE1_POL;
            IO_PWM_SLAVE1_LT = (uint16_t)i16PwmV;
            IO_PWM_MASTER1_CTRL = (PWM_PRESCALER << 8) | C_PWM_MASTER1_MODE_MIRROR | B_PWM_MASTER1_POL;
            i16PwmU = 0U;
        }
        else
        {
            IO_PWM_SLAVE2_CTRL = B_PWM_SLAVE2_SLAVE | C_PWM_SLAVE2_MODE_MIRROR | B_PWM_SLAVE2_POL;
            IO_PWM_SLAVE2_LT = (uint16_t)i16PwmW;
            IO_PWM_SLAVE1_CTRL = B_PWM_SLAVE1_SLAVE | C_PWM_SLAVE1_MODE_MIRROR;
            IO_PWM_SLAVE1_LT = (uint16_t)(PWM_SCALE_OFFSET - i16PwmV);
            IO_PWM_MASTER1_CTRL = (PWM_PRESCALER << 8) | C_PWM_MASTER1_MODE_MIRROR | B_PWM_MASTER1_POL;
            i16PwmU = 0U;
        }
#else  /* (_SUPPORT_PWM_POL_CORR == FALSE) */
#if (_SUPPORT_EVB == MLX81160EVB_3P_3N)
#if (_SUPPORT_PWM_SYNC != FALSE)
        IO_PWM_SLAVE2_LT = (uint16_t)(PWM_SCALE_OFFSET - i16PwmW);
        IO_PWM_SLAVE1_CTRL = B_PWM_SLAVE1_SLAVE | C_PWM_SLAVE1_MODE_MIRROR;
        IO_PWM_SLAVE1_LT = (uint16_t)i16PwmV;
        i16PwmU = 0;
#else  /* (_SUPPORT_PWM_SYNC != FALSE) */
        MotorPwm.u16PwmSlave2 = (uint16_t)(PWM_SCALE_OFFSET - i16PwmW);
        MotorPwm.u16PwmCtrl = B_PWM_SLAVE1_SLAVE | C_PWM_SLAVE1_MODE_MIRROR;
        MotorPwm.u16PwmSlave1 = (uint16_t)i16PwmV;
        i16PwmU = 0;
#endif /* (_SUPPORT_PWM_SYNC != FALSE) */
#else  /* (_SUPPORT_EVB == MLX81160EVB_3P_3N) */
#if (_SUPPORT_PWM_SYNC != FALSE)
        IO_PWM_SLAVE2_LT = (uint16_t)i16PwmW;
        IO_PWM_SLAVE1_CTRL = B_PWM_SLAVE1_SLAVE | C_PWM_SLAVE1_MODE_MIRROR;
        IO_PWM_SLAVE1_LT = (uint16_t)(PWM_SCALE_OFFSET - i16PwmV);
#else  /* (_SUPPORT_PWM_SYNC != FALSE) */
        MotorPwm.u16PwmSlave2 = (uint16_t)i16PwmW;
        MotorPwm.u16PwmCtrl = B_PWM_SLAVE1_SLAVE | C_PWM_SLAVE1_MODE_MIRROR;
        MotorPwm.u16PwmSlave1 = (uint16_t)(PWM_SCALE_OFFSET - i16PwmV);
#endif /* (_SUPPORT_PWM_SYNC != FALSE) */
        i16PwmU = PWM_REG_PERIOD;
#endif /* (_SUPPORT_EVB == MLX81160EVB_3P_3N) */
#endif /* (_SUPPORT_PWM_POL_CORR == FALSE) */
    }
    else
    {
        /* Small W-phase low period --> W-phase to Low */
#if (SPACE_VECTOR_SINE == FALSE)
        i16PwmU = (i16PwmU - i16PwmW);
        i16PwmV = (i16PwmV - i16PwmW);
#endif /* (SPACE_VECTOR_SINE == FALSE) */
#if (_SUPPORT_PWM_POL_CORR == FALSE)
        if (l_u16PwmState != FALSE)
        {
            IO_PWM_MASTER1_CTRL = (PWM_PRESCALER << 8) | C_PWM_MASTER1_MODE_MIRROR | B_PWM_MASTER1_POL;
            IO_PWM_SLAVE1_CTRL = B_PWM_SLAVE1_SLAVE | C_PWM_SLAVE1_MODE_MIRROR;
            IO_PWM_SLAVE1_LT = (uint16_t)(PWM_SCALE_OFFSET - i16PwmV);
            IO_PWM_SLAVE2_CTRL = B_PWM_SLAVE2_SLAVE | C_PWM_SLAVE2_MODE_MIRROR | B_PWM_SLAVE2_POL;
            IO_PWM_SLAVE2_LT = 0U;
        }
        else
        {
            IO_PWM_MASTER1_CTRL = (PWM_PRESCALER << 8) | C_PWM_MASTER1_MODE_MIRROR;
            i16PwmU = (PWM_SCALE_OFFSET - i16PwmU);
            IO_PWM_SLAVE1_CTRL = B_PWM_SLAVE1_SLAVE | C_PWM_SLAVE1_MODE_MIRROR | B_PWM_SLAVE1_POL;
            IO_PWM_SLAVE1_LT = (uint16_t)i16PwmV;
            IO_PWM_SLAVE2_CTRL = B_PWM_SLAVE2_SLAVE | C_PWM_SLAVE2_MODE_MIRROR | B_PWM_SLAVE2_POL;
            IO_PWM_SLAVE2_LT = 0U;
        }
#else  /* (_SUPPORT_PWM_POL_CORR == FALSE) */
#if (_SUPPORT_EVB == MLX81160EVB_3P_3N)
        /* i16PwmU = i16PwmU; */
#if (_SUPPORT_PWM_SYNC != FALSE)
        IO_PWM_SLAVE1_CTRL = B_PWM_SLAVE1_SLAVE | C_PWM_SLAVE1_MODE_MIRROR | B_PWM_SLAVE1_POL;
        IO_PWM_SLAVE1_LT = (uint16_t)(PWM_SCALE_OFFSET - i16PwmV);
        IO_PWM_SLAVE2_LT = PWM_SCALE_OFFSET;
#else  /* (_SUPPORT_PWM_SYNC != FALSE) */
        MotorPwm.u16PwmCtrl = B_PWM_SLAVE1_SLAVE | C_PWM_SLAVE1_MODE_MIRROR | B_PWM_SLAVE1_POL;
        MotorPwm.u16PwmSlave1 = (uint16_t)(PWM_SCALE_OFFSET - i16PwmV);
        MotorPwm.u16PwmSlave2 = PWM_SCALE_OFFSET;
#endif /* (_SUPPORT_PWM_SYNC != FALSE) */
#else  /* (_SUPPORT_EVB == MLX81160EVB_3P_3N) */
        i16PwmU = (PWM_SCALE_OFFSET - i16PwmU);
#if (_SUPPORT_PWM_SYNC != FALSE)
        IO_PWM_SLAVE1_CTRL = B_PWM_SLAVE1_SLAVE | C_PWM_SLAVE1_MODE_MIRROR | B_PWM_SLAVE1_POL;
        IO_PWM_SLAVE1_LT = (uint16_t)i16PwmV;
        IO_PWM_SLAVE2_LT = 0U;
#else  /* (_SUPPORT_PWM_SYNC != FALSE) */
        MotorPwm.u16PwmCtrl = B_PWM_SLAVE1_SLAVE | C_PWM_SLAVE1_MODE_MIRROR | B_PWM_SLAVE1_POL;
        MotorPwm.u16PwmSlave1 = (uint16_t)i16PwmV;
        MotorPwm.u16PwmSlave2 = 0U;
#endif /* (_SUPPORT_PWM_SYNC != FALSE) */
#endif /* (_SUPPORT_EVB == MLX81160EVB_3P_3N) */
#endif /* (_SUPPORT_PWM_POL_CORR == FALSE) */
    }
#elif (_SUPPORT_VOLTAGE_SHAPE == VOLTAGE_SHAPE_DSVM_SUP)
    u16MicroStepIdx = (u16MicroStepIdx + (C_MICROSTEP_PER_FULLSTEP / 2U)) / (2U * C_MICROSTEP_PER_FULLSTEP);
    if (u16MicroStepIdx == 1U)
    {
        /* Small V-phase low period --> V-phase to Low */
#if (SPACE_VECTOR_SINE == FALSE)
        i16PwmU = (i16PwmU - i16PwmV);
        i16PwmW = (i16PwmW - i16PwmV);
#endif /* (SPACE_VECTOR_SINE == FALSE) */
        IO_PWM_MASTER1_HT = (PWM_SCALE_OFFSET + (uint16_t)i16PwmU);
        i16PwmU = (PWM_SCALE_OFFSET - (uint16_t)i16PwmU);
        IO_PWM_SLAVE2_HT = (uint16_t)i16PwmW;
        IO_PWM_SLAVE2_LT = (PWM_REG_PERIOD - (uint16_t)i16PwmW);
        IO_PWM_SLAVE1_LT = PWM_REG_PERIOD + 1U;
        IO_PWM_SLAVE1_HT = 0;
    }
    else if (u16MicroStepIdx == 2U)
    {
        /* Small U-phase low period --> U-phase to Low */
#if (SPACE_VECTOR_SINE == FALSE)
        i16PwmV = (i16PwmV - i16PwmU);
        i16PwmW = (i16PwmW - i16PwmU);
#endif /* (SPACE_VECTOR_SINE == FALSE) */
        IO_PWM_SLAVE1_HT = (PWM_SCALE_OFFSET + (uint16_t)i16PwmV);
        IO_PWM_SLAVE1_LT = (PWM_SCALE_OFFSET - (uint16_t)i16PwmV);
        IO_PWM_SLAVE2_HT = i16PwmW;
        IO_PWM_SLAVE2_LT = (PWM_REG_PERIOD - (uint16_t)i16PwmW);
        i16PwmU = PWM_REG_PERIOD + 1U;
        IO_PWM_MASTER1_HT = 0U;
    }
    else
    {
        /* Small W-phase low period --> W-phase to Low */
#if (SPACE_VECTOR_SINE == FALSE)
        i16PwmU = (i16PwmU - i16PwmW);
        i16PwmV = (i16PwmV - i16PwmW);
#endif /* (SPACE_VECTOR_SINE == FALSE) */
        IO_PWM_MASTER1_HT = (PWM_SCALE_OFFSET + (uint16_t)i16PwmU);
        i16PwmU = (PWM_SCALE_OFFSET - (uint16_t)i16PwmU);
        IO_PWM_SLAVE1_HT = (uint16_t)i16PwmV;
        IO_PWM_SLAVE1_LT = (PWM_REG_PERIOD - (uint16_t)i16PwmV);
        IO_PWM_SLAVE2_LT = PWM_REG_PERIOD + 1U;
        IO_PWM_SLAVE2_HT = 0U;
    }
#elif (_SUPPORT_VOLTAGE_SHAPE == VOLTAGE_SHAPE_DSVM_ALT) /* MMP210524-1 */
    /* Third-phase alternate GND/SUP */
    u16MicroStepIdx = (u16MicroStepIdx / C_MICROSTEP_PER_FULLSTEP);
    if (u16MicroStepIdx == 0U)
    {
#if (_SUPPORT_PWM_SYNC != FALSE)
        IO_PWM_SLAVE2_CTRL = B_PWM_SLAVE2_SLAVE | C_PWM_SLAVE2_MODE_MIRROR | B_PWM_SLAVE2_POL;
        IO_PWM_SLAVE2_LT = 0U;                                                  /* Low */
        IO_PWM_SLAVE1_CTRL = B_PWM_SLAVE1_SLAVE | C_PWM_SLAVE1_MODE_MIRROR;
        IO_PWM_SLAVE1_LT = (uint16_t)(PWM_SCALE_OFFSET - i16PwmV);
        IO_PWM_MASTER1_CTRL = (PWM_PRESCALER << 8) | C_PWM_MASTER1_MODE_MIRROR | B_PWM_MASTER1_POL;
#else  /* (_SUPPORT_PWM_SYNC != FALSE) */
        MotorPwm.u16PwmCtrlS2 = B_PWM_SLAVE2_SLAVE | C_PWM_SLAVE2_MODE_MIRROR | B_PWM_SLAVE2_POL;
        MotorPwm.u16PwmSlave2 = 0U;                                             /* Low */
        MotorPwm.u16PwmCtrlS1 = B_PWM_SLAVE1_SLAVE | C_PWM_SLAVE1_MODE_MIRROR;
        MotorPwm.u16PwmSlave1 = (uint16_t)(PWM_SCALE_OFFSET - i16PwmV);
        MotorPwm.u16PwmCtrlM1 = (PWM_PRESCALER << 8) | C_PWM_MASTER1_MODE_MIRROR | B_PWM_MASTER1_POL;
#endif /* (_SUPPORT_PWM_SYNC != FALSE) */
    }
    else if (u16MicroStepIdx == 1U)
    {
#if (_SUPPORT_PWM_SYNC != FALSE)
        IO_PWM_SLAVE2_CTRL = B_PWM_SLAVE2_SLAVE | C_PWM_SLAVE2_MODE_MIRROR | B_PWM_SLAVE2_POL;
        IO_PWM_SLAVE2_LT = (uint16_t)(PWM_SCALE_OFFSET - i16PwmW);
        IO_PWM_SLAVE1_CTRL = B_PWM_SLAVE1_SLAVE | C_PWM_SLAVE1_MODE_MIRROR;
        IO_PWM_SLAVE1_LT = (uint16_t)i16PwmV;
        IO_PWM_MASTER1_CTRL = (PWM_PRESCALER << 8) | C_PWM_MASTER1_MODE_MIRROR;
#else  /* (_SUPPORT_PWM_SYNC != FALSE) */
        MotorPwm.u16PwmCtrlS2 = B_PWM_SLAVE2_SLAVE | C_PWM_SLAVE2_MODE_MIRROR | B_PWM_SLAVE2_POL;
        MotorPwm.u16PwmSlave2 = (uint16_t)(PWM_SCALE_OFFSET - i16PwmW);
        MotorPwm.u16PwmCtrlS1 = B_PWM_SLAVE1_SLAVE | C_PWM_SLAVE1_MODE_MIRROR;
        MotorPwm.u16PwmSlave1 = (uint16_t)i16PwmV;
        MotorPwm.u16PwmCtrlM1 = (PWM_PRESCALER << 8) | C_PWM_MASTER1_MODE_MIRROR;
#endif /* (_SUPPORT_PWM_SYNC != FALSE) */
        i16PwmU = 0U;                                                           /* High */
    }
    else if (u16MicroStepIdx == 2U)
    {
#if (_SUPPORT_PWM_SYNC != FALSE)
        IO_PWM_SLAVE2_CTRL = B_PWM_SLAVE2_SLAVE | C_PWM_SLAVE2_MODE_MIRROR | B_PWM_SLAVE2_POL;
        IO_PWM_SLAVE2_LT = (uint16_t)i16PwmW;
        IO_PWM_SLAVE1_CTRL = B_PWM_SLAVE1_SLAVE | C_PWM_SLAVE1_MODE_MIRROR | B_PWM_SLAVE1_POL;
        IO_PWM_SLAVE1_LT = 0U;                                                   /* Low */
        IO_PWM_MASTER1_CTRL = (PWM_PRESCALER << 8) | C_PWM_MASTER1_MODE_MIRROR;
#else  /* (_SUPPORT_PWM_SYNC != FALSE) */
        MotorPwm.u16PwmCtrlS2 = B_PWM_SLAVE2_SLAVE | C_PWM_SLAVE2_MODE_MIRROR | B_PWM_SLAVE2_POL;
        MotorPwm.u16PwmSlave2 = (uint16_t)i16PwmW;
        MotorPwm.u16PwmCtrlS1 = B_PWM_SLAVE1_SLAVE | C_PWM_SLAVE1_MODE_MIRROR | B_PWM_SLAVE1_POL;
        MotorPwm.u16PwmSlave1 = 0U;                                             /* Low */
        MotorPwm.u16PwmCtrlM1 = (PWM_PRESCALER << 8) | C_PWM_MASTER1_MODE_MIRROR;
#endif /* (_SUPPORT_PWM_SYNC != FALSE) */
        i16PwmU = (PWM_SCALE_OFFSET - i16PwmU);
    }
    else if (u16MicroStepIdx == 3U)
    {
#if (_SUPPORT_PWM_SYNC != FALSE)
        IO_PWM_SLAVE2_CTRL = B_PWM_SLAVE2_SLAVE | C_PWM_SLAVE2_MODE_MIRROR;
        IO_PWM_SLAVE2_LT = 0U;                                                  /* High */
        IO_PWM_SLAVE1_CTRL = B_PWM_SLAVE1_SLAVE | C_PWM_SLAVE1_MODE_MIRROR | B_PWM_SLAVE1_POL;
        IO_PWM_SLAVE1_LT = (uint16_t)(PWM_SCALE_OFFSET - i16PwmV);
        IO_PWM_MASTER1_CTRL = (PWM_PRESCALER << 8) | C_PWM_MASTER1_MODE_MIRROR;
#else  /* (_SUPPORT_PWM_SYNC != FALSE) */
        MotorPwm.u16PwmCtrlS2 = B_PWM_SLAVE2_SLAVE | C_PWM_SLAVE2_MODE_MIRROR;
        MotorPwm.u16PwmSlave2 = 0U;                                             /* High */
        MotorPwm.u16PwmCtrlS1 = B_PWM_SLAVE1_SLAVE | C_PWM_SLAVE1_MODE_MIRROR | B_PWM_SLAVE1_POL;
        MotorPwm.u16PwmSlave1 = (uint16_t)(PWM_SCALE_OFFSET - i16PwmV);
        MotorPwm.u16PwmCtrlM1 = (PWM_PRESCALER << 8) | C_PWM_MASTER1_MODE_MIRROR;
#endif /* (_SUPPORT_PWM_SYNC != FALSE) */
    }
    else if (u16MicroStepIdx == 4U)
    {
#if (_SUPPORT_PWM_SYNC != FALSE)
        IO_PWM_SLAVE2_CTRL = B_PWM_SLAVE2_SLAVE | C_PWM_SLAVE2_MODE_MIRROR;
        IO_PWM_SLAVE2_LT = (uint16_t)(PWM_SCALE_OFFSET - i16PwmW);
        IO_PWM_SLAVE1_CTRL = B_PWM_SLAVE1_SLAVE | C_PWM_SLAVE1_MODE_MIRROR | B_PWM_SLAVE1_POL;
        IO_PWM_SLAVE1_LT = (uint16_t)i16PwmV;
        IO_PWM_MASTER1_CTRL = (PWM_PRESCALER << 8) | C_PWM_MASTER1_MODE_MIRROR | B_PWM_MASTER1_POL;
#else  /* (_SUPPORT_PWM_SYNC != FALSE) */
        MotorPwm.u16PwmCtrlS2 = B_PWM_SLAVE2_SLAVE | C_PWM_SLAVE2_MODE_MIRROR;
        MotorPwm.u16PwmSlave2 = (uint16_t)(PWM_SCALE_OFFSET - i16PwmW);
        MotorPwm.u16PwmCtrlS1 = B_PWM_SLAVE1_SLAVE | C_PWM_SLAVE1_MODE_MIRROR | B_PWM_SLAVE1_POL;
        MotorPwm.u16PwmSlave1 = (uint16_t)i16PwmV;
        MotorPwm.u16PwmCtrlM1 = (PWM_PRESCALER << 8) | C_PWM_MASTER1_MODE_MIRROR | B_PWM_MASTER1_POL;
#endif /* (_SUPPORT_PWM_SYNC != FALSE) */
        i16PwmU = 0U;                                                           /* Low */
    }
    else /* if ( u16MicroStepIdx == 5U ) */
    {
#if (_SUPPORT_PWM_SYNC != FALSE)
        IO_PWM_SLAVE2_CTRL = B_PWM_SLAVE2_SLAVE | C_PWM_SLAVE2_MODE_MIRROR;
        IO_PWM_SLAVE2_LT = (uint16_t)i16PwmW;
        IO_PWM_SLAVE1_CTRL = B_PWM_SLAVE1_SLAVE | C_PWM_SLAVE1_MODE_MIRROR;
        IO_PWM_SLAVE1_LT = 0U;                                                  /* High */
        IO_PWM_MASTER1_CTRL = (PWM_PRESCALER << 8) | C_PWM_MASTER1_MODE_MIRROR | B_PWM_MASTER1_POL;
#else  /* (_SUPPORT_PWM_SYNC != FALSE) */
        MotorPwm.u16PwmCtrlS2 = B_PWM_SLAVE2_SLAVE | C_PWM_SLAVE2_MODE_MIRROR;
        MotorPwm.u16PwmSlave2 = (uint16_t)i16PwmW;
        MotorPwm.u16PwmCtrlS1 = B_PWM_SLAVE1_SLAVE | C_PWM_SLAVE1_MODE_MIRROR;
        MotorPwm.u16PwmSlave1 = 0U;                                             /* High */
        MotorPwm.u16PwmCtrlM1 = (PWM_PRESCALER << 8) | C_PWM_MASTER1_MODE_MIRROR | B_PWM_MASTER1_POL;
#endif /* (_SUPPORT_PWM_SYNC != FALSE) */
        i16PwmU = (PWM_SCALE_OFFSET - i16PwmU);
    }
#else
#error "Error: _SUPPORT_VOLTAGE_SHAPE not supported"
#endif /* (_SUPPORT_VOLTAGE_SHAPE) */

#if (_SUPPORT_PWM_SYNC != FALSE)
    IO_PWM_MASTER1_LT = (uint16_t)i16PwmU;                                        /* Master must be modified at last (value is not important) */
#else  /* (_SUPPORT_PWM_SYNC != FALSE) */
    MotorPwm.u16PwmMaster1 = (uint16_t)i16PwmU;
    HAL_PWM_MasterIrqEnable();
#endif /* (_SUPPORT_PWM_SYNC != FALSE) */
} /* End of MotorDriver_3Phase() */

#elif (_SUPPORT_PWM_MODE == TRIPLEPHASE_FULL_STEP_BEMF) || (_SUPPORT_PWM_MODE == TRIPLEPHASE_FULL_STEP)

uint16_t const au16DriverBldcFullStep[C_MOTOR_PHASES * C_MOTOR_STEP_PER_PHASE] =
{
    (C_PORT_DRV_CTRL_DRV2_L | C_PORT_DRV_CTRL_DRV1_MASTER1 | C_PORT_DRV_CTRL_DRV0_TRISTATE),  /* PhC = L, PhB = PWM, PhA = Z (index++: raising) */
    (C_PORT_DRV_CTRL_DRV2_L | C_PORT_DRV_CTRL_DRV1_TRISTATE | C_PORT_DRV_CTRL_DRV0_MASTER1),  /* PhC = L, PhB = Z, PHA = PWM (index++: falling) */
    (C_PORT_DRV_CTRL_DRV2_TRISTATE | C_PORT_DRV_CTRL_DRV1_L | C_PORT_DRV_CTRL_DRV0_MASTER1),  /* PhC = Z, PhB = L, PhA = PWM (index++: raising) */
    (C_PORT_DRV_CTRL_DRV2_MASTER1 | C_PORT_DRV_CTRL_DRV1_L | C_PORT_DRV_CTRL_DRV0_TRISTATE),  /* PhC = PWM, PhB = L, PhA = Z (index++: falling) */
    (C_PORT_DRV_CTRL_DRV2_MASTER1 | C_PORT_DRV_CTRL_DRV1_TRISTATE | C_PORT_DRV_CTRL_DRV0_L),  /* PhC = PWM, PhB = Z, PhA = L (index++: raising) */
    (C_PORT_DRV_CTRL_DRV2_TRISTATE | C_PORT_DRV_CTRL_DRV1_MASTER1 | C_PORT_DRV_CTRL_DRV0_L)   /* PhC = Z, PhB = PWM, PhA = L (index++: falling) */
};

/*!*************************************************************************** *
 * MotorDriver_3Phase
 * \brief   Drive motor in full-step mode (6 full-steps)
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16MicroStepIdx: Micro-step index
 * \return  -
 * *************************************************************************** *
 * \details BLDC 3-Phase motor using 3-phase with:
 * *************************************************************************** *
 * - Call Hierarchy: ISR_CTIMER0_3(), MotorDriverStart(), MotorDriverStop(),
 *                   PID_COntrol()
 * - Cyclomatic Complexity: 2+1
 * - Nesting: 2
 * - Function calling: 1 (p_MulI16_I16byI16asr5())
 * *************************************************************************** */
void MotorDriver_3Phase(uint16_t u16FullStepIdx)
{
#if (_SUPPORT_BRAKING != FALSE)
    if (g_e8MotorStatus == (uint8_t)C_MOTOR_STATUS_BRAKING)
    {
        /* Breaking-mode: Use 180 degrees shifted vector */
        u16FullStepIdx += 3U;
        if (u16FullStepIdx >= 6U)
        {
            u16FullStepIdx -= 6U;
        }
    }
#endif /* (_SUPPORT_BRAKING != FALSE) */
    int16_t i16PwmU = l_u16CorrectionRatio >> 5U;
#if (PWM_LIMIT == PWM_LIMIT_BY_PWM)
#ifdef C_PWM_MIN_DC
    i16PwmU = p_ClipMinMaxU16(i16PwmU, C_PWM_MIN_DC, l_u16MaxPwmRatio);         /* MMP190926-2 */
#else
    if (i16PwmU > (int16_t)l_u16MaxPwmRatio)                                    /* MMP181114-2 */
    {
        i16PwmU = l_u16MaxPwmRatio;
    }
#endif
#endif /* (PWM_LIMIT == PWM_LIMIT_BY_PWM) */
#if (_SUPPORT_PWM_SYNC != FALSE)
    IO_PWM_MASTER1_LT = (uint16_t)(PWM_SCALE_OFFSET - i16PwmU);                 /* Master must be modified at last (value is not important) */
    HAL_PWM_MasterPendClear();                                                  /* Clear PWM-module Master1-End IRQ's */
    HAL_PWM_MasterPendWait();                                                   /* Wait for PWM Master PEND-flag */
    IO_PORT_DRV_CTRL = au16DriverBldcFullStep[u16FullStepIdx];
#else  /* (_SUPPORT_PWM_SYNC != FALSE) */
    MotorPwm.u16PwmMaster1 = (uint16_t)i16PwmU;
    HAL_PWM_MasterIrqEnable();
#endif /* (_SUPPORT_PWM_SYNC != FALSE) */
} /* End of MotorDriver_3Phase() */

#endif /* (_SUPPORT_PWM_MODE) */

#if (_SUPPORT_STALLDET_BZC != FALSE)
/*!*************************************************************************** *
 * MotorDriver_3PhaseBEMF
 * \brief   Drive motor in Full-step mode (6-steps)
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16MicroStepIdx: Full-step index
 * \return  -
 * *************************************************************************** *
 * \details
 * *************************************************************************** *
 * - Call Hierarchy: ISR_CTIMER0_3(), MotorDriverStart(), MotorDriverStop(),
 *                   PID_COntrol()
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 1 (p_MulI16_I16byI16asr5())
 * *************************************************************************** */
void MotorDriver_3PhaseBEMF(uint16_t u16MicroStepIdx)
{
    uint16_t u16DC1 = (l_u16CorrectionRatio >> 6U);
#if (PWM_LIMIT == PWM_LIMIT_BY_PWM)
    u16DC1 = p_ClipMinMaxU16(u16DC1, C_PWM_MIN_DC, l_u16MaxPwmRatio);
#endif /* (PWM_LIMIT == PWM_LIMIT_BY_PWM) */
    u16DC1 = (PWM_OFFSET_25 - u16DC1);

    if (u16MicroStepIdx <= 1U)
    {
        if (u16MicroStepIdx == 1U)
        {
            /* u16MicroStepIdx = 1: (DRV_CFG_W_PWM | DRV_CFG_V_TRISTATE | DRV_CFG_U_PWM); PhC = PWM, PhB = Z, PHA = PWM (index++: falling) */
            /* U */
            IO_PWM_MASTER1_CTRL = (PWM_PRESCALER << 8) | C_PWM_MASTER1_MODE_MIRROR;
            /* V */
            if (g_e8MotorDirectionCCW != FALSE)
            {
                IO_PWM_SLAVE1_CTRL = B_PWM_SLAVE1_SLAVE | C_PWM_SLAVE1_MODE_MIRROR;
            }
            else
            {
                IO_PWM_SLAVE1_CTRL = B_PWM_SLAVE1_SLAVE | C_PWM_SLAVE1_MODE_MIRROR | B_PWM_SLAVE1_POL;
            }
        }
        else
        {
            /* u16MicroStepIdx = 0: (DRV_CFG_W_PWM | DRV_CFG_V_PWM | DRV_CFG_U_TRISTATE); PhC = PWM, PhB = PWM, PhA = Z (index++: raising) */
            /* V */
            IO_PWM_SLAVE1_CTRL = B_PWM_SLAVE1_SLAVE | C_PWM_SLAVE1_MODE_MIRROR;
            /* U */
            if (g_e8MotorDirectionCCW != FALSE)
            {
                IO_PWM_MASTER1_CTRL = (PWM_PRESCALER << 8) | C_PWM_MASTER1_MODE_MIRROR | B_PWM_MASTER1_POL;
            }
            else
            {
                IO_PWM_MASTER1_CTRL = (PWM_PRESCALER << 8) | C_PWM_MASTER1_MODE_MIRROR;
            }
        }
        /* W */
        IO_PWM_SLAVE2_CTRL = B_PWM_SLAVE2_SLAVE | C_PWM_SLAVE2_MODE_MIRROR | B_PWM_SLAVE2_POL;
    }
    else if (u16MicroStepIdx <= 3U)
    {
        if (u16MicroStepIdx == 3U)
        {
            /* u16MicroStepIdx = 3: (DRV_CFG_W_PWM | DRV_CFG_V_PWM | DRV_CFG_U_TRISTATE); PhC = PWM, PhB = PWM, PhA = Z (index++: falling) */
            /* W */
            IO_PWM_SLAVE2_CTRL = B_PWM_SLAVE2_SLAVE | C_PWM_SLAVE2_MODE_MIRROR;
            /* U */
            if (g_e8MotorDirectionCCW != FALSE)
            {
                IO_PWM_MASTER1_CTRL = (PWM_PRESCALER << 8) | C_PWM_MASTER1_MODE_MIRROR;
            }
            else
            {
                IO_PWM_MASTER1_CTRL = (PWM_PRESCALER << 8) | C_PWM_MASTER1_MODE_MIRROR | B_PWM_MASTER1_POL;
            }
        }
        else
        {
            /* u16MicroStepIdx = 2: (DRV_CFG_W_TRISTATE | DRV_CFG_V_PWM | DRV_CFG_U_PWM); PhC = Z, PhB = PWM, PhA = PWM (index++: raising) */
            /* U */
            IO_PWM_MASTER1_CTRL = (PWM_PRESCALER << 8) | C_PWM_MASTER1_MODE_MIRROR;
            IO_PWM_MASTER1_LT = u16DC1;
            /* W */
            if (g_e8MotorDirectionCCW != FALSE)
            {
                IO_PWM_SLAVE2_CTRL = B_PWM_SLAVE2_SLAVE | C_PWM_SLAVE2_MODE_MIRROR | B_PWM_SLAVE2_POL;
            }
            else
            {
                IO_PWM_SLAVE2_CTRL = B_PWM_SLAVE2_SLAVE | C_PWM_SLAVE2_MODE_MIRROR;
            }
        }
        /* V */
        IO_PWM_SLAVE1_CTRL = B_PWM_SLAVE1_SLAVE | C_PWM_SLAVE1_MODE_MIRROR | B_PWM_SLAVE1_POL;
    }
    else
    {
        if (u16MicroStepIdx == 4U)
        {
            /* u16MicroStepIdx = 4: (DRV_CFG_W_PWM | DRV_CFG_V_TRISTATE | DRV_CFG_U_PWM); PhC = PWM, PhB = Z, PhA = PWM (index++: raising) */
            /* W */
            IO_PWM_SLAVE2_CTRL = B_PWM_SLAVE2_SLAVE | C_PWM_SLAVE2_MODE_MIRROR;
            /* V */
            if (g_e8MotorDirectionCCW != FALSE)
            {
                IO_PWM_SLAVE1_CTRL = B_PWM_SLAVE1_SLAVE | C_PWM_SLAVE1_MODE_MIRROR | B_PWM_SLAVE1_POL;
            }
            else
            {
                IO_PWM_SLAVE1_CTRL = B_PWM_SLAVE1_SLAVE | C_PWM_SLAVE1_MODE_MIRROR;
            }
        }
        else
        {
            /* u16MicroStepIdx = 5: (DRV_CFG_W_TRISTATE | DRV_CFG_V_PWM | DRV_CFG_U_PWM); PhC = Z, PhB = PWM, PhA = PWM (index++: falling) */
            /* V */
            IO_PWM_SLAVE1_CTRL = B_PWM_SLAVE1_SLAVE | C_PWM_SLAVE1_MODE_MIRROR;
            /* W */
            if (g_e8MotorDirectionCCW != FALSE)
            {
                IO_PWM_SLAVE2_CTRL = B_PWM_SLAVE2_SLAVE | C_PWM_SLAVE2_MODE_MIRROR;
            }
            else
            {
                IO_PWM_SLAVE2_CTRL = B_PWM_SLAVE2_SLAVE | C_PWM_SLAVE2_MODE_MIRROR | B_PWM_SLAVE2_POL;
            }
        }
        /* U */
        IO_PWM_MASTER1_CTRL = (PWM_PRESCALER << 8) | C_PWM_MASTER1_MODE_MIRROR | B_PWM_MASTER1_POL;
    }
    IO_PWM_SLAVE1_LT = u16DC1;
    IO_PWM_SLAVE2_LT = u16DC1;
    /* Change Drive configuration synchronised with PWM-update */
    {
#if defined (__MLX81160__)
#error "ERROR: BEMF Implementation not done"
#elif defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__)
        uint16_t u16NewDrvCfg = ((IO_PORT_DRV_CTRL & ~(M_PORT_DRV_CTRL_DRV2_CTRL |
                                                       M_PORT_DRV_CTRL_DRV1_CTRL |
                                                       M_PORT_DRV_CTRL_DRV0_CTRL)) |
                                 c_au16DrvCfg[u16MicroStepIdx]);
#else
        uint16_t u16NewDrvCfg = c_au16DrvCfg[u16MicroStepIdx];
#endif /* defined (__MLX81160__) */
        HAL_PWM_MasterPendClear();                                              /* Clear PWM-module Master1-End IRQ's */
        IO_PWM_MASTER1_LT = u16DC1;                                             /* Master must be modified at last (value is not important) */
        HAL_PWM_MasterPendWait();                                               /* Wait for PWM Master PEND-flag */
        IO_PORT_DRV_CTRL = u16NewDrvCfg;
#if (_DEBUG_COMMUT_ISR != FALSE)
        DEBUG_SET_IO_A();
#endif /* (_DEBUG_COMMUT_ISR != FALSE) */
    }
#if (_DEBUG_COMMUT_ISR != FALSE)
    DEBUG_CLR_IO_A();
#endif /* (_DEBUG_COMMUT_ISR != FALSE) */
} /* End of MotorDriver_3PhaseBEMF() */
#endif /* (_SUPPORT_STALLDET_BZC != FALSE) */

#elif (C_MOTOR_PHASES != 1)

#if (C_MICROSTEP_PER_FULLSTEP > 2)                                              /* MMP231017-1 */
/*!*************************************************************************** *
 * MotorDriver_4Phase
 * \brief   Drive motor in micro-stepper mode (32, 64 or 128-steps)
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16MicroStepIdx: Micro-step index
 * \return  -
 * *************************************************************************** *
 * \details -
 * *************************************************************************** *
 * - Call Hierarchy: ISR_CTIMER0_3(), MotorDriverStart(), MotorDriverStop(),
 *                   PID_COntrol()
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 1 (p_MulI16_I16byI16Shft4())
 * *************************************************************************** */
void MotorDriver_4Phase(uint16_t u16MicroStepIdx)
{
    /* EMC CE/RE reduction */
#if (_SUPPORT_PWM_SPREAD_SPECTRUM != FALSE)
    static uint16_t u16PwmSpreadSpectrumIdx = 0U;
#endif /* (_SUPPORT_PWM_SPREAD_SPECTRUM != FALSE) */
    int16_t i16Pwm1, i16Pwm2;

#if (_SUPPORT_PWM_SPREAD_SPECTRUM != FALSE)
    u16PwmSpreadSpectrumIdx = (u16PwmSpreadSpectrumIdx + 1U) & 15U;
    IO_PWM_MASTER1_PER = au16PwmPeriod[u16PwmSpreadSpectrumIdx];
#endif /* (_SUPPORT_PWM_SPREAD_SPECTRUM != FALSE) */

#if (_SUPPORT_MULTI_VECTOR_WAVEFORM != FALSE)
    int16_t *pi16Vector = (int16_t *)&c_ai16MicroStepVector4PH[C_SPACE_VECTOR_FIFTH_SINE_ID][u16MicroStepIdx];
#else  /* (_SUPPORT_MULTI_VECTOR_WAVEFORM != FALSE) */
    int16_t *pi16Vector = (int16_t *)&c_ai16MicroStepVector4PH[u16MicroStepIdx];
#endif /* (_SUPPORT_MULTI_VECTOR_WAVEFORM != FALSE) */
#if (PWM_REG_PERIOD >= (128U << (4U - PWM_PRESCALER_N)))                        /* (((PWM_REG_PERIOD * 256U) >> (4U - PWM_PRESCALER_N)) > 32767U) */
    i16Pwm1 = (int16_t)(mulI32_I16byU16(*pi16Vector, l_u16CorrectionRatio) >> (16 + C_PID_FACTOR));      /* Coil-1 */
    pi16Vector += C_MICROSTEP_PER_FULLSTEP;
    i16Pwm2 = (int16_t)(mulI32_I16byU16(*pi16Vector, l_u16CorrectionRatio) >> (16 + C_PID_FACTOR));      /* Coil-2 */
#elif (C_PID_FACTOR == 4)
    i16Pwm1 = p_MulI16hi_I16byI16asr4(*pi16Vector, (int16_t)l_u16CorrectionRatio);    /* Coil-1 */
    pi16Vector += C_MICROSTEP_PER_FULLSTEP;
    i16Pwm2 = p_MulI16hi_I16byI16asr4(*pi16Vector, (int16_t)l_u16CorrectionRatio);    /* Coil-2 */
#else
    i16Pwm1 = (int16_t)(mulI16_I16byI16(*pi16Vector, (int16_t)l_u16CorrectionRatio) >> C_PID_FACTOR);     /* Coil-1 */
    pi16Vector += C_MICROSTEP_PER_FULLSTEP;
    i16Pwm2 = (int16_t)(mulI16_I16byI16(*pi16Vector, (int16_t)l_u16CorrectionRatio) >> C_PID_FACTOR);     /* Coil-2 */
#endif
    /* Inverse Mirror-mode */
    {
        if ( (u16MicroStepIdx < C_MICROSTEP_PER_FULLSTEP) || (u16MicroStepIdx >= (3U * C_MICROSTEP_PER_FULLSTEP)) )
        {
            /* 1st and 4th Quadrant (Pwm2) */
#if (PWM_LIMIT == PWM_LIMIT_BY_PWM)
#ifdef C_PWM_MIN_DC
            i16Pwm2 = p_ClipMinMaxU16(i16Pwm2, C_PWM_MIN_DC, l_u16MaxPwmRatio);   /* MMP190926-2 */
#else
            if (i16Pwm2 > (int16_t)l_u16MaxPwmRatio)                            /* MMP181114-2 */
            {
                i16Pwm2 = l_u16MaxPwmRatio;
            }
#endif
#endif /* (PWM_LIMIT == PWM_LIMIT_BY_PWM) */
#if defined (__MLX81160__)
#if (_SUPPORT_PWM_SYNC != FALSE)
            IO_PWM_MASTER2_LT = (uint16_t)i16Pwm2;                              /* T = PWM */
            IO_PWM_SLAVE3_LT = 0U;                                              /* U = LOW */
#else  /* (_SUPPORT_PWM_SYNC != FALSE) */
            MotorPwm.u16PwmSlave2 = (uint16_t)i16Pwm2;                          /* T = PWM */
            MotorPwm.u16PwmSlave3 = 0U;                                         /* U = LOW */
#endif /* (_SUPPORT_PWM_SYNC != FALSE) */
#else  /* defined (__MLX81160__) */
#if (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UV_WT)
#if (_SUPPORT_PWM_SYNC != FALSE)
            IO_PWM_SLAVE2_LT = (uint16_t)i16Pwm2;                               /* W = PWM */
            IO_PWM_SLAVE3_LT = 0U;                                              /* T = LOW */
#else  /* (_SUPPORT_PWM_SYNC != FALSE) */
            MotorPwm.u16PwmSlave2 = (uint16_t)i16Pwm2;                          /* W = PWM */
            MotorPwm.u16PwmSlave3 = 0U;                                         /* T = LOW */
#endif /* (_SUPPORT_PWM_SYNC != FALSE) */
#endif /* (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UV_WT) */
#if (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UT_VW)
#if (_SUPPORT_PWM_SYNC != FALSE)
            IO_PWM_SLAVE1_LT = (uint16_t)i16Pwm2;                               /* V = PWM */
            IO_PWM_SLAVE2_LT = 0U;                                              /* W = LOW */
#else  /* (_SUPPORT_PWM_SYNC != FALSE) */
            MotorPwm.u16PwmSlave1 = (uint16_t)i16Pwm2;                          /* V = PWM */
            MotorPwm.u16PwmSlave2 = 0U;                                         /* W = LOW */
#endif /* (_SUPPORT_PWM_SYNC != FALSE) */
#endif /* (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UT_VW) */
#if (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UW_VT)
#if (_SUPPORT_PWM_SYNC != FALSE)
            IO_PWM_SLAVE1_LT = (uint16_t)i16Pwm2;                               /* V = PWM */
            IO_PWM_SLAVE3_LT = 0U;                                              /* T = LOW */
#else  /* (_SUPPORT_PWM_SYNC != FALSE) */
            MotorPwm.u16PwmSlave1 = (uint16_t)i16Pwm2;                          /* V = PWM */
            MotorPwm.u16PwmSlave3 = 0U;                                         /* T = LOW */
#endif /* (_SUPPORT_PWM_SYNC != FALSE) */
#endif /* (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UW_VT) */
#if (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UV_TW)
#if (_SUPPORT_PWM_SYNC != FALSE)
            IO_PWM_SLAVE3_LT = (uint16_t)i16Pwm2;                               /* T = PWM */
            IO_PWM_SLAVE2_LT = 0U;                                              /* W = LOW */
#else  /* (_SUPPORT_PWM_SYNC != FALSE) */
            MotorPwm.u16PwmSlave3 = (uint16_t)i16Pwm2;                          /* T = PWM */
            MotorPwm.u16PwmSlave2 = 0U;                                         /* W = LOW */
#endif /* (_SUPPORT_PWM_SYNC != FALSE) */
#endif /* (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UV_TW) */
#endif /* defined (__MLX81160__) */
        }
        else
        {
            /* 2nd and 3rd Quadrant (Pwm2) */
            i16Pwm2 = (0 - i16Pwm2);
#if (PWM_LIMIT == PWM_LIMIT_BY_PWM)
#ifdef C_PWM_MIN_DC
            i16Pwm2 = p_ClipMinMaxU16(i16Pwm2, C_PWM_MIN_DC, l_u16MaxPwmRatio);   /* MMP190926-2 */
#else
            if (i16Pwm2 > (int16_t)l_u16MaxPwmRatio)                            /* MMP181114-2 */
            {
                i16Pwm2 = l_u16MaxPwmRatio;
            }
#endif
#endif /* (PWM_LIMIT == PWM_LIMIT_BY_PWM) */
#if defined (__MLX81160__)
#if (_SUPPORT_PWM_SYNC != FALSE)
            IO_PWM_SLAVE3_LT = (uint16_t)i16Pwm2;                               /* U = PWM */
            IO_PWM_MASTER2_LT = 0U;                                             /* T = LOW */
#else  /* (_SUPPORT_PWM_SYNC != FALSE) */
            MotorPwm.u16PwmSlave3 = (uint16_t)i16Pwm2;                          /* U = PWM */
            MotorPwm.u16PwmSlave2 = 0U;                                         /* T = LOW */
#endif /* (_SUPPORT_PWM_SYNC != FALSE) */
#else  /* defined (__MLX81160__) */
#if (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UV_WT)
#if (_SUPPORT_PWM_SYNC != FALSE)
            IO_PWM_SLAVE3_LT = (uint16_t)i16Pwm2;                               /* T = PWM */
            IO_PWM_SLAVE2_LT = 0U;                                              /* W = LOW */
#else  /* (_SUPPORT_PWM_SYNC != FALSE) */
            MotorPwm.u16PwmSlave3 = (uint16_t)i16Pwm2;                          /* T = PWM */
            MotorPwm.u16PwmSlave2 = 0U;                                         /* W = LOW */
#endif /* (_SUPPORT_PWM_SYNC != FALSE) */
#endif /* (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UV_WT) */
#if (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UT_VW)
#if (_SUPPORT_PWM_SYNC != FALSE)
            IO_PWM_SLAVE2_LT = (uint16_t)i16Pwm2;                               /* W = PWM */
            IO_PWM_SLAVE1_LT = 0U;                                              /* V = LOW */
#else  /* (_SUPPORT_PWM_SYNC != FALSE) */
            MotorPwm.u16PwmSlave2 = (uint16_t)i16Pwm2;                          /* W = PWM */
            MotorPwm.u16PwmSlave1 = 0U;                                         /* V = LOW */
#endif /* (_SUPPORT_PWM_SYNC != FALSE) */
#endif /* (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UT_VW) */
#if (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UW_VT)
#if (_SUPPORT_PWM_SYNC != FALSE)
            IO_PWM_SLAVE3_LT = (uint16_t)i16Pwm2;                               /* T = PWM */
            IO_PWM_SLAVE1_LT = 0U;                                              /* V = LOW */
#else  /* (_SUPPORT_PWM_SYNC != FALSE) */
            MotorPwm.u16PwmSlave3 = (uint16_t)i16Pwm2;                          /* T = PWM */
            MotorPwm.u16PwmSlave1 = 0U;                                         /* V = LOW */
#endif /* (_SUPPORT_PWM_SYNC != FALSE) */
#endif /* (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UW_VT) */
#if (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UV_TW)
#if (_SUPPORT_PWM_SYNC != FALSE)
            IO_PWM_SLAVE2_LT = (uint16_t)i16Pwm2;                               /* W = PWM */
            IO_PWM_SLAVE3_LT = 0U;                                              /* T = LOW */
#else  /* (_SUPPORT_PWM_SYNC != FALSE) */
            MotorPwm.u16PwmSlave2 = (uint16_t)i16Pwm2;                          /* W = PWM */
            MotorPwm.u16PwmSlave3 = 0U;                                         /* T = LOW */
#endif /* (_SUPPORT_PWM_SYNC != FALSE) */
#endif /* (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UV_TW) */
#endif /* defined (__MLX81160__) */
        }
    }

    if ( (u16MicroStepIdx & (2U * C_MICROSTEP_PER_FULLSTEP)) != 0U)
    {
        /* 3rd and 4th Quadrant (Pwm1) */
        i16Pwm1 = 0 - i16Pwm1;
#if (PWM_LIMIT == PWM_LIMIT_BY_PWM)
#ifdef C_PWM_MIN_DC
        i16Pwm1 = p_ClipMinMaxU16(i16Pwm1, C_PWM_MIN_DC, l_u16MaxPwmRatio);     /* MMP190926-2 */
#else
        if (i16Pwm1 > (int16_t)l_u16MaxPwmRatio)                                /* MMP181114-2 */
        {
            i16Pwm1 = l_u16MaxPwmRatio;
        }
#endif
#endif /* (PWM_LIMIT == PWM_LIMIT_BY_PWM) */
#if defined (__MLX81160__)
#if (_SUPPORT_PWM_SYNC != FALSE)
        IO_PWM_SLAVE1_LT = (uint16_t)(PWM_SCALE_OFFSET - i16Pwm1);              /* S = PWM */
#else  /* (_SUPPORT_PWM_SYNC != FALSE) */
        MotorPwm.u16PwmSlave1 = (uint16_t)(PWM_SCALE_OFFSET - i16Pwm1);         /* S = PWM */
#endif /* (_SUPPORT_PWM_SYNC != FALSE) */
#else  /* defined (__MLX81160__) */
#if (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UW_VT)
#if (_SUPPORT_PWM_SYNC != FALSE)
        IO_PWM_SLAVE2_LT = (uint16_t)(PWM_SCALE_OFFSET - i16Pwm1);              /* W = PWM */
#else  /* (_SUPPORT_PWM_SYNC != FALSE) */
        MotorPwm.u16PwmSlave2 = (uint16_t)(PWM_SCALE_OFFSET - i16Pwm1);         /* W = PWM */
#endif /* (_SUPPORT_PWM_SYNC != FALSE) */
#endif /* (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UW_VT) */
#if (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UV_WT) || (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UV_TW)
#if (_SUPPORT_PWM_SYNC != FALSE)
        IO_PWM_SLAVE1_LT = (uint16_t)(PWM_SCALE_OFFSET - i16Pwm1);              /* V = PWM */
#else  /* (_SUPPORT_PWM_SYNC != FALSE) */
        MotorPwm.u16PwmSlave1 = (uint16_t)(PWM_SCALE_OFFSET - i16Pwm1);         /* V = PWM */
#endif /* (_SUPPORT_PWM_SYNC != FALSE) */
#endif /* (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UV_WT) || (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UV_TW) */
#if (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UT_VW)
#if (_SUPPORT_PWM_SYNC != FALSE)
        IO_PWM_SLAVE3_LT = (uint16_t)(PWM_SCALE_OFFSET - i16Pwm1);              /* T = PWM */
#else  /* (_SUPPORT_PWM_SYNC != FALSE) */
        MotorPwm.u16PwmSlave3 = (uint16_t)(PWM_SCALE_OFFSET - i16Pwm1);         /* T = PWM */
#endif /* (_SUPPORT_PWM_SYNC != FALSE) */
#endif /* (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UT_VW) */
#endif /* defined (__MLX81160__) */
        /* U = LOW */
        i16Pwm1 = PWM_REG_PERIOD;
    }
    else
    {
        /* 1st and 2nd Quadrant (Pwm1) */
#if (PWM_LIMIT == PWM_LIMIT_BY_PWM)
#ifdef C_PWM_MIN_DC
        i16Pwm1 = p_ClipMinMaxU16(i16Pwm1, C_PWM_MIN_DC, l_u16MaxPwmRatio);     /* MMP190926-2 */
#else
        if (i16Pwm1 > (int16_t)l_u16MaxPwmRatio)                                /* MMP181114-2 */
        {
            i16Pwm1 = l_u16MaxPwmRatio;
        }
#endif
#endif /* (PWM_LIMIT == PWM_LIMIT_BY_PWM) */
#if defined (__MLX81160__)
#if (_SUPPORT_PWM_SYNC != FALSE)
        IO_PWM_SLAVE1_LT = PWM_REG_PERIOD;                                      /* S = LOW */
#else  /* (_SUPPORT_PWM_SYNC != FALSE) */
        MotorPwm.u16PwmSlave1 = PWM_REG_PERIOD;                                 /* S = LOW */
#endif /* (_SUPPORT_PWM_SYNC != FALSE) */
#else  /* defined (__MLX81160__) */
#if (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UW_VT)
#if (_SUPPORT_PWM_SYNC != FALSE)
        IO_PWM_SLAVE2_LT = PWM_REG_PERIOD;                                      /* W = LOW */
#else  /* (_SUPPORT_PWM_SYNC != FALSE) */
        MotorPwm.u16PwmSlave2 = PWM_REG_PERIOD;                                 /* W = LOW */
#endif /* (_SUPPORT_PWM_SYNC != FALSE) */
#endif /* (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UW_VT) */
#if (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UV_WT) || (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UV_TW)
#if (_SUPPORT_PWM_SYNC != FALSE)
        IO_PWM_SLAVE1_LT = PWM_REG_PERIOD;                                      /* V = LOW */
#else  /* (_SUPPORT_PWM_SYNC != FALSE) */
        MotorPwm.u16PwmSlave1 = PWM_REG_PERIOD;                                 /* V = LOW */
#endif /* (_SUPPORT_PWM_SYNC != FALSE) */
#endif /* (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UV_WT) || (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UV_TW) */
#if (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UT_VW)
#if (_SUPPORT_PWM_SYNC != FALSE)
        IO_PWM_SLAVE3_LT = PWM_REG_PERIOD;                                      /* T = LOW */
#else  /* (_SUPPORT_PWM_SYNC != FALSE) */
        MotorPwm.u16PwmSlave3 = PWM_REG_PERIOD;                                 /* T = LOW */
#endif /* (_SUPPORT_PWM_SYNC != FALSE) */
#endif /* (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UT_VW) */
#endif /* defined (__MLX81160__) */
        /* U = PWM */
        i16Pwm1 = (PWM_SCALE_OFFSET - i16Pwm1);
    }
#if (_SUPPORT_PWM_SYNC != FALSE)
    IO_PWM_MASTER1_LT = (uint16_t)i16Pwm1;                                      /* Master must be modified at last (value is not important) */
#else  /* (_SUPPORT_PWM_SYNC != FALSE) */
    MotorPwm.u16PwmMaster1 = (uint16_t)i16Pwm1;
    HAL_PWM_MasterIrqEnable();
#endif /* (_SUPPORT_PWM_SYNC != FALSE) */
} /* End of MotorDriver_4Phase() */
#else  /* (C_MICROSTEP_PER_FULLSTEP > 2) */                                     /* MMP231017-1 */

#if (C_MICROSTEP_PER_FULLSTEP == 1)
const uint16_t au16DriverBiPolarFullStep[C_NR_OF_FULLSTEPS] =
{
#if (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UV_WT) || (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UV_TW)
    (C_PORT_DRV_CTRL_DRV0_MASTER1 | C_PORT_DRV_CTRL_DRV1_L | C_PORT_DRV_CTRL_DRV2_TRISTATE |
     C_PORT_DRV_CTRL_DRV3_TRISTATE),
    (C_PORT_DRV_CTRL_DRV0_TRISTATE | C_PORT_DRV_CTRL_DRV1_TRISTATE | C_PORT_DRV_CTRL_DRV2_MASTER1 |
     C_PORT_DRV_CTRL_DRV3_L),
    (C_PORT_DRV_CTRL_DRV0_L | C_PORT_DRV_CTRL_DRV1_MASTER1 | C_PORT_DRV_CTRL_DRV2_TRISTATE |
     C_PORT_DRV_CTRL_DRV3_TRISTATE),
    (C_PORT_DRV_CTRL_DRV0_TRISTATE | C_PORT_DRV_CTRL_DRV1_TRISTATE | C_PORT_DRV_CTRL_DRV2_L |
     C_PORT_DRV_CTRL_DRV3_MASTER1)
#elif (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UW_VT)
    (C_PORT_DRV_CTRL_DRV0_MASTER1 | C_PORT_DRV_CTRL_DRV1_TRISTATE | C_PORT_DRV_CTRL_DRV2_L |
     C_PORT_DRV_CTRL_DRV3_TRISTATE),
    (C_PORT_DRV_CTRL_DRV0_TRISTATE | C_PORT_DRV_CTRL_DRV1_MASTER1 | C_PORT_DRV_CTRL_DRV2_TRISTATE |
     C_PORT_DRV_CTRL_DRV3_L),
    (C_PORT_DRV_CTRL_DRV0_L | C_PORT_DRV_CTRL_DRV1_TRISTATE | C_PORT_DRV_CTRL_DRV2_MASTER1 |
     C_PORT_DRV_CTRL_DRV3_TRISTATE),
    (C_PORT_DRV_CTRL_DRV0_TRISTATE | C_PORT_DRV_CTRL_DRV1_L | C_PORT_DRV_CTRL_DRV2_TRISTATE |
     C_PORT_DRV_CTRL_DRV3_MASTER1)
#elif (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UT_VW)
    (C_PORT_DRV_CTRL_DRV0_MASTER1 | C_PORT_DRV_CTRL_DRV1_TRISTATE | C_PORT_DRV_CTRL_DRV2_TRISTATE |
     C_PORT_DRV_CTRL_DRV3_L),
    (C_PORT_DRV_CTRL_DRV0_TRISTATE | C_PORT_DRV_CTRL_DRV1_MASTER1 | C_PORT_DRV_CTRL_DRV2_L |
     C_PORT_DRV_CTRL_DRV3_TRISTATE),
    (C_PORT_DRV_CTRL_DRV0_L | C_PORT_DRV_CTRL_DRV1_TRISTATE | C_PORT_DRV_CTRL_DRV2_TRISTATE |
     C_PORT_DRV_CTRL_DRV3_MASTER1),
    (C_PORT_DRV_CTRL_DRV0_TRISTATE | C_PORT_DRV_CTRL_DRV1_L | C_PORT_DRV_CTRL_DRV2_MASTER1 |
     C_PORT_DRV_CTRL_DRV3_TRISTATE)
#else
#error "ERROR: Bi-Polar Full-step not implemented"
#endif
};

/*!*************************************************************************** *
 * MotorDriver_4Phase
 * \brief   Drive motor in micro-stepper mode (32, 64 or 128-steps)
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16FullStepIdx: Full-step index
 * \return  -
 * *************************************************************************** *
 * \details -
 * *************************************************************************** *
 * - Call Hierarchy: ISR_CTIMER0_3(), MotorDriverStart(), MotorDriverStop(),
 *                   PID_COntrol()
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 1 (p_MulI16_I16byI16Shft4())
 * *************************************************************************** */
void MotorDriver_4Phase(uint16_t u16FullStepIdx)
#elif (C_MICROSTEP_PER_FULLSTEP == 2)

const uint16_t au16DriverBiPolarHalfStep[C_NR_OF_HALFSTEPS] =
{
#if (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UV_WT) || (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UV_TW)
    (C_PORT_DRV_CTRL_DRV0_MASTER1 | C_PORT_DRV_CTRL_DRV1_L | C_PORT_DRV_CTRL_DRV2_TRISTATE |
     C_PORT_DRV_CTRL_DRV3_TRISTATE),
    (C_PORT_DRV_CTRL_DRV0_MASTER1 | C_PORT_DRV_CTRL_DRV1_L | C_PORT_DRV_CTRL_DRV2_MASTER1 |
     C_PORT_DRV_CTRL_DRV3_L),
    (C_PORT_DRV_CTRL_DRV0_TRISTATE | C_PORT_DRV_CTRL_DRV1_TRISTATE | C_PORT_DRV_CTRL_DRV2_MASTER1 |
     C_PORT_DRV_CTRL_DRV3_L),
    (C_PORT_DRV_CTRL_DRV0_L | C_PORT_DRV_CTRL_DRV1_MASTER1 | C_PORT_DRV_CTRL_DRV2_MASTER1 |
     C_PORT_DRV_CTRL_DRV3_L),
    (C_PORT_DRV_CTRL_DRV0_L | C_PORT_DRV_CTRL_DRV1_MASTER1 | C_PORT_DRV_CTRL_DRV2_TRISTATE |
     C_PORT_DRV_CTRL_DRV3_TRISTATE),
    (C_PORT_DRV_CTRL_DRV0_L | C_PORT_DRV_CTRL_DRV1_MASTER1 | C_PORT_DRV_CTRL_DRV2_L |
     C_PORT_DRV_CTRL_DRV3_MASTER1),
    (C_PORT_DRV_CTRL_DRV0_TRISTATE | C_PORT_DRV_CTRL_DRV1_TRISTATE | C_PORT_DRV_CTRL_DRV2_L |
     C_PORT_DRV_CTRL_DRV3_MASTER1),
    (C_PORT_DRV_CTRL_DRV0_MASTER1 | C_PORT_DRV_CTRL_DRV1_L | C_PORT_DRV_CTRL_DRV2_L |
     C_PORT_DRV_CTRL_DRV3_MASTER1)
#elif (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UW_VT)
    (C_PORT_DRV_CTRL_DRV0_MASTER1 | C_PORT_DRV_CTRL_DRV1_TRISTATE | C_PORT_DRV_CTRL_DRV2_L |
     C_PORT_DRV_CTRL_DRV3_TRISTATE),
    (C_PORT_DRV_CTRL_DRV0_MASTER1 | C_PORT_DRV_CTRL_DRV1_MASTER1 | C_PORT_DRV_CTRL_DRV2_L |
     C_PORT_DRV_CTRL_DRV3_L),
    (C_PORT_DRV_CTRL_DRV0_TRISTATE | C_PORT_DRV_CTRL_DRV1_MASTER1 | C_PORT_DRV_CTRL_DRV2_TRISTATE |
     C_PORT_DRV_CTRL_DRV3_L),
    (C_PORT_DRV_CTRL_DRV0_L | C_PORT_DRV_CTRL_DRV1_MASTER1 | C_PORT_DRV_CTRL_DRV2_MASTER1 |
     C_PORT_DRV_CTRL_DRV3_L),
    (C_PORT_DRV_CTRL_DRV0_L | C_PORT_DRV_CTRL_DRV1_TRISTATE | C_PORT_DRV_CTRL_DRV2_MASTER1 |
     C_PORT_DRV_CTRL_DRV3_TRISTATE),
    (C_PORT_DRV_CTRL_DRV0_L | C_PORT_DRV_CTRL_DRV1_L | C_PORT_DRV_CTRL_DRV2_MASTER1 |
     C_PORT_DRV_CTRL_DRV3_MASTER1),
    (C_PORT_DRV_CTRL_DRV0_TRISTATE | C_PORT_DRV_CTRL_DRV1_L | C_PORT_DRV_CTRL_DRV2_TRISTATE |
     C_PORT_DRV_CTRL_DRV3_MASTER1),
    (C_PORT_DRV_CTRL_DRV0_MASTER1 | C_PORT_DRV_CTRL_DRV1_L | C_PORT_DRV_CTRL_DRV2_L |
     C_PORT_DRV_CTRL_DRV3_MASTER1)
#elif (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UT_VW)
    (C_PORT_DRV_CTRL_DRV0_MASTER1 | C_PORT_DRV_CTRL_DRV1_TRISTATE | C_PORT_DRV_CTRL_DRV2_TRISTATE |
     C_PORT_DRV_CTRL_DRV3_L),
    (C_PORT_DRV_CTRL_DRV0_MASTER1 | C_PORT_DRV_CTRL_DRV1_MASTER1 | C_PORT_DRV_CTRL_DRV2_L |
     C_PORT_DRV_CTRL_DRV3_L),
    (C_PORT_DRV_CTRL_DRV0_TRISTATE | C_PORT_DRV_CTRL_DRV1_MASTER1 | C_PORT_DRV_CTRL_DRV2_L |
     C_PORT_DRV_CTRL_DRV3_TRISTATE),
    (C_PORT_DRV_CTRL_DRV0_L | C_PORT_DRV_CTRL_DRV1_MASTER1 | C_PORT_DRV_CTRL_DRV2_L |
     C_PORT_DRV_CTRL_DRV3_MASTER1),
    (C_PORT_DRV_CTRL_DRV0_L | C_PORT_DRV_CTRL_DRV1_TRISTATE | C_PORT_DRV_CTRL_DRV2_TRISTATE |
     C_PORT_DRV_CTRL_DRV3_MASTER1),
    (C_PORT_DRV_CTRL_DRV0_L | C_PORT_DRV_CTRL_DRV1_L | C_PORT_DRV_CTRL_DRV2_MASTER1 |
     C_PORT_DRV_CTRL_DRV3_MASTER1),
    (C_PORT_DRV_CTRL_DRV0_TRISTATE | C_PORT_DRV_CTRL_DRV1_L | C_PORT_DRV_CTRL_DRV2_MASTER1 |
     C_PORT_DRV_CTRL_DRV3_TRISTATE),
    (C_PORT_DRV_CTRL_DRV0_MASTER1 | C_PORT_DRV_CTRL_DRV1_L | C_PORT_DRV_CTRL_DRV2_MASTER1 |
     C_PORT_DRV_CTRL_DRV3_L)
#else
#error "ERROR: Bi-Polar Full-step not implemented"
#endif
};

/*!*************************************************************************** *
 * MotorDriver_4Phase
 * \brief   Drive motor in micro-stepper mode (32, 64 or 128-steps)
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16HalfStepIdx: Full-step index
 * \return  -
 * *************************************************************************** *
 * \details -
 * *************************************************************************** *
 * - Call Hierarchy: ISR_CTIMER0_3(), MotorDriverStart(), MotorDriverStop(),
 *                   PID_COntrol()
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 1 (p_MulI16_I16byI16Shft4())
 * *************************************************************************** */
void MotorDriver_4Phase(uint16_t u16HalfStepIdx)
#endif
{
    /* EMC CE/RE reduction */
#if (_SUPPORT_PWM_SPREAD_SPECTRUM != FALSE)
    static uint16_t u16PwmSpreadSpectrumIdx = 0U;
#endif /* (_SUPPORT_PWM_SPREAD_SPECTRUM != FALSE) */
    int16_t i16Pwm1;

#if (_SUPPORT_PWM_SPREAD_SPECTRUM != FALSE)
    u16PwmSpreadSpectrumIdx = (u16PwmSpreadSpectrumIdx + 1U) & 15U;
    IO_PWM_MASTER1_PER = au16PwmPeriod[u16PwmSpreadSpectrumIdx];
#endif /* (_SUPPORT_PWM_SPREAD_SPECTRUM != FALSE) */

#if (_SUPPORT_FULLSTEP != FALSE)
    i16Pwm1 = l_u16CorrectionRatio >> 5U;                                       /* MMP230601-1 */
#elif (_SUPPORT_HALFSTEP != FALSE)
    if ( (u16HalfStepIdx & 1U) == 0U)
    {
        i16Pwm1 = l_u16CorrectionRatio >> 5U;                                   /* MMP230601-1 */
    }
    else
    {
        i16Pwm1 = p_DivU16_U32byU16( (uint32_t)l_u16CorrectionRatio, 45U);      /* Approximate 32*SQRT(2) */
    }
#endif

#if (PWM_LIMIT == PWM_LIMIT_BY_PWM)
#ifdef C_PWM_MIN_DC
    i16Pwm1 = p_ClipMinMaxU16(i16Pwm1, C_PWM_MIN_DC, l_u16MaxPwmRatio);         /* MMP190926-2 */
#else
    if (i16Pwm1 > (int16_t)l_u16MaxPwmRatio)                                    /* MMP181114-2 */
    {
        i16Pwm1 = l_u16MaxPwmRatio;
    }
#endif
#endif /* (PWM_LIMIT == PWM_LIMIT_BY_PWM) */
#if (_SUPPORT_PWM_SYNC != FALSE)
    IO_PWM_MASTER1_LT = (uint16_t)(PWM_SCALE_OFFSET - i16Pwm1);                 /* Master must be modified at last (value is not important) */
    HAL_PWM_MasterPendClear();                                                  /* Clear PWM-module Master1-End IRQ's */
    HAL_PWM_MasterPendWait();                                                   /* Wait for PWM Master PEND-flag */
#if (_SUPPORT_FULLSTEP != FALSE)
    IO_PORT_DRV_CTRL = au16DriverBiPolarFullStep[u16FullStepIdx];
#elif (_SUPPORT_HALFSTEP != FALSE)
    IO_PORT_DRV_CTRL = au16DriverBiPolarHalfStep[u16HalfStepIdx];
#else
#error "ERROR: DRV_CTRL"
#endif
#else  /* (_SUPPORT_PWM_SYNC != FALSE) */
    MotorPwm.u16PwmMaster1 = (uint16_t)i16Pwm1;
    HAL_PWM_MasterIrqEnable();
#endif /* (_SUPPORT_PWM_SYNC != FALSE) */
} /* End of MotorDriver_4Phase() */
#endif /* (C_MICROSTEP_PER_FULLSTEP > 2) */                                     /* MMP231017-1 */
#endif /* (C_MOTOR_PHASES == 3) */

#if (_SUPPORT_PWM_SYNC == FALSE)
/*!*************************************************************************** *
 * ISR_PWM_MASTER1_END
 * \brief   Update Motor PWM LT registers just after PWM period ends
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details Update Motor PWM LT registers directly after PWM period ends
 * *************************************************************************** *
 * - Call Hierarchy: PWM_MASTER1_END_IT
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 0
 * *************************************************************************** */
__attribute__((interrupt)) void ISR_PWM_MASTER1_END(void)
{
    /* Update PWM LT registers, just after a PWM Master1 End event */
#if (_SUPPORT_PWM_MODE != SINGLE_COIL_PWM_BIPOLAR)
#if defined (__MLX81160__)
#if (_SUPPORT_EVB == MLX81160EVB_3P_3N)
    IO_PWM_SLAVE4_LT = (uint16_t)(MotorPwm.u16PwmSlave2 - C_EXT_FET_DEADTIME);   /* DRV5 (W), POL */
    IO_PWM_SLAVE2_LT = (uint16_t)(MotorPwm.u16PwmSlave2 + C_EXT_FET_DEADTIME);   /* DRV2 (T), POL */
#elif (C_MOTOR_PHASES > 3)
    IO_PWM_MASTER2_LT = (uint16_t)MotorPwm.u16PwmSlave2;
    IO_PWM_SLAVE3_LT = (uint16_t)MotorPwm.u16PwmSlave3;
#else /* (C_MOTOR_PHASES > 3) */
    IO_PWM_SLAVE2_LT = (uint16_t)MotorPwm.u16PwmSlave2;
#endif /* (C_MOTOR_PHASES > 3) */
#else  /* defined (__MLX81160__) */
#if (_SUPPORT_VOLTAGE_SHAPE == VOLTAGE_SHAPE_DSVM_ALT)
    IO_PWM_SLAVE2_CTRL = MotorPwm.u16PwmCtrlS2;
#endif /* (_SUPPORT_VOLTAGE_SHAPE == VOLTAGE_SHAPE_DSVM_ALT) */
    IO_PWM_SLAVE2_LT = (uint16_t)MotorPwm.u16PwmSlave2;
#if (C_MOTOR_PHASES > 3)
    IO_PWM_SLAVE3_LT = (uint16_t)MotorPwm.u16PwmSlave3;
    /* IO_PWM_MASTER2_LT = (uint16_t) MotorPwm.u16PwmMaster2; */
#endif /* (C_MOTOR_PHASES > 3) */
#endif /* defined (__MLX81160__) */
#endif /* (_SUPPORT_PWM_MODE != SINGLE_COIL_PWM_BIPOLAR) */
#if defined (__MLX81160__) && (_SUPPORT_EVB == MLX81160EVB_3P_3N)
    IO_PWM_SLAVE1_CTRL = MotorPwm.u16PwmCtrl;
    IO_PWM_SLAVE3_CTRL = MotorPwm.u16PwmCtrl;
    if ( (MotorPwm.u16PwmCtrl & B_PWM_SLAVE1_POL) != 0U)
    {
        IO_PWM_SLAVE3_LT = (uint16_t)(MotorPwm.u16PwmSlave1 - C_EXT_FET_DEADTIME);   /* DRV4 (V), POL */
        IO_PWM_SLAVE1_LT = (uint16_t)(MotorPwm.u16PwmSlave1 + C_EXT_FET_DEADTIME);   /* DRV1 (S), POL */
    }
    else
    {
        IO_PWM_SLAVE3_LT = (uint16_t)(MotorPwm.u16PwmSlave1 + C_EXT_FET_DEADTIME);   /* DRV4 (V) */
        IO_PWM_SLAVE1_LT = (uint16_t)(MotorPwm.u16PwmSlave1 - C_EXT_FET_DEADTIME);   /* DRV1 (S) */
    }
    if (MotorPwm.u16PwmMaster1 != 0U)
    {
        IO_PWM_MASTER2_LT = (uint16_t)(MotorPwm.u16PwmMaster1 + C_EXT_FET_DEADTIME);   /* DRV3 (U) */
        IO_PWM_MASTER1_LT = (uint16_t)(MotorPwm.u16PwmMaster1 - C_EXT_FET_DEADTIME);   /* DRV0 (R) */
    }
    else
    {
        IO_PWM_MASTER2_LT = (uint16_t)(MotorPwm.u16PwmMaster1);                 /* DRV3 (U) */
        IO_PWM_MASTER1_LT = (uint16_t)(MotorPwm.u16PwmMaster1);                 /* DRV0 (R) */
    }
#else  /* defined (__MLX81160__) && (_SUPPORT_EVB == MLX81160EVB_3P_3N) */
#if (_SUPPORT_VOLTAGE_SHAPE == VOLTAGE_SHAPE_DSVM_ALT)
    IO_PWM_SLAVE1_CTRL = MotorPwm.u16PwmCtrlS1;
#elif (_SUPPORT_PWM_MODE == BIPOLAR_TRIPHASE_TWOPWM_INDEPENDENT_GND) || \
    ((_SUPPORT_PWM_MODE == TRIPLEPHASE_TWOPWM_INDEPENDENT_GND) && (_SUPPORT_FOC_MODE == FOC_MODE_NONE))
    IO_PWM_SLAVE1_CTRL = MotorPwm.u16PwmCtrl;
#endif /* (_SUPPORT_VOLTAGE_SHAPE == VOLTAGE_SHAPE_DSVM_ALT) */
    IO_PWM_SLAVE1_LT = (uint16_t)MotorPwm.u16PwmSlave1;
#if (_SUPPORT_VOLTAGE_SHAPE == VOLTAGE_SHAPE_DSVM_ALT)
    IO_PWM_MASTER1_CTRL = MotorPwm.u16PwmCtrlM1;
#endif /* (_SUPPORT_VOLTAGE_SHAPE == VOLTAGE_SHAPE_DSVM_ALT) */
    IO_PWM_MASTER1_LT = (uint16_t)MotorPwm.u16PwmMaster1;
#endif /* defined (__MLX81160__) && (_SUPPORT_EVB == MLX81160EVB_3P_3N) */

    HAL_PWM_MasterIrqDisable();                                                 /* Disable PWM Master1 End IRQ */

#if (_SUPPORT_CDI != FALSE)
    if ( (l_u16MicroStepIdx >= ((3U * C_MICROSTEP_PER_FULLSTEP) + (C_MICROSTEP_PER_FULLSTEP >> 3))) &&
         (l_u16MicroStepIdx < ((4U * C_MICROSTEP_PER_FULLSTEP) - (C_MICROSTEP_PER_FULLSTEP >> 3))) &&
         (l_u16CDI != C_IO_ACTIVE_CDI_ACTIVE_CDI_PHASE_U) )
    {
        IO_ACTIVE_CDI = (IO_ACTIVE_CDI & ~M_IO_ACTIVE_CDI_ACTIVE_CDI_PHASE) |
                        (C_IO_ACTIVE_CDI_ACTIVE_CDI_EDGE_SEL_FALL | C_IO_ACTIVE_CDI_ACTIVE_CDI_PHASE_U);
    }
    else if ( (l_u16MicroStepIdx >= ((5U * C_MICROSTEP_PER_FULLSTEP) + (C_MICROSTEP_PER_FULLSTEP >> 3))) &&
              (l_u16MicroStepIdx < ((6U * C_MICROSTEP_PER_FULLSTEP) - (C_MICROSTEP_PER_FULLSTEP >> 3))) &&
              (l_u16CDI != C_IO_ACTIVE_CDI_ACTIVE_CDI_PHASE_W) )
    {
        IO_ACTIVE_CDI = (IO_ACTIVE_CDI & ~M_IO_ACTIVE_CDI_ACTIVE_CDI_PHASE) |
                        (C_IO_ACTIVE_CDI_ACTIVE_CDI_EDGE_SEL_FALL | C_IO_ACTIVE_CDI_ACTIVE_CDI_PHASE_W);
    }
    else if ( (l_u16MicroStepIdx >= ((1U * C_MICROSTEP_PER_FULLSTEP) + (C_MICROSTEP_PER_FULLSTEP >> 3))) &&
              (l_u16MicroStepIdx < ((2U * C_MICROSTEP_PER_FULLSTEP) - (C_MICROSTEP_PER_FULLSTEP >> 3))) &&
              (l_u16CDI != C_IO_ACTIVE_CDI_ACTIVE_CDI_PHASE_V) )
    {
        IO_ACTIVE_CDI = (IO_ACTIVE_CDI & ~M_IO_ACTIVE_CDI_ACTIVE_CDI_PHASE) |
                        (C_IO_ACTIVE_CDI_ACTIVE_CDI_EDGE_SEL_FALL | C_IO_ACTIVE_CDI_ACTIVE_CDI_PHASE_V);
    }
#endif /* (_SUPPORT_CDI != FALSE) */
} /* End of ISR_PWM_MASTER1_END() */
#endif /* (_SUPPORT_PWM_SYNC == FALSE) */

/*!*************************************************************************** *
 * MotorDriverCurrentMeasureInit
 * \brief   Initialise for motor driver current measurement
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details Initialise the motor current filters
 * *************************************************************************** *
 * - Call Hierarchy: MotorDriverStart()
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 1 (p_MemSet())
 * *************************************************************************** */
void MotorDriverCurrentMeasureInit(void)
{
#if (_SUPPORT_CURRSPIKE_POSITION != FALSE)
    static uint8_t l_e8SpikePulseMotorDirCCW_Prev = (uint8_t)C_MOTOR_DIR_UNKNOWN;

    ENTER_SECTION(ATOMIC_KEEP_MODE);  /*lint !e534 */
    {
#if (_SUPPORT_NV_NON_BLOCK_WRITE != FALSE)
        l_u16StallDetectorDelay = g_u16StallDetectorDelay_NV;
#else  /* (_SUPPORT_NV_NON_BLOCK_WRITE != FALSE) */
        l_u16StallDetectorDelay = NV_STALL_DETECTOR_DELAY; /* C_MOVAVG_SZ; */
#endif /* (_SUPPORT_NV_NON_BLOCK_WRITE != FALSE) */

#if !defined (C_MOVAVG_SSZ) || (C_MOVAVG_SZ > ((C_MOTOR_PHASES * C_MOTOR_STEP_PER_PHASE) * C_MICROSTEP_PER_FULLSTEP))
        l_u16MotorCurrentRawIdx = 0U;                                           /* Raw current moving average index */
#endif /* !defined (C_MOVAVG_SSZ) || (C_MOVAVG_SZ > ((C_MOTOR_PHASES * C_MOTOR_STEP_PER_PHASE) * C_MICROSTEP_PER_FULLSTEP)) */

        l_au16MotorCurrentRaw[0] = 0U;
        l_u16MotorCurrentMovAvgxN = 0U;                                         /* Moving average motor-current (x 4..16) */
        l_u16MotorCurrentLPF = 0U;                                              /* Low-pass Filtered motor-current [ADC-LSB] */
        l_u32MotorCurrentLPF = 0U;
        l_u16SpikeEdge = C_SPIKE_ANYEDGE;
        l_u16SpikePulseSkipCount = 0U;
        l_u16PulsePeriodCount = 0U;
        l_u16LastPulsePeriod = 0xFFFFU;
        l_u16StartPulseWidth = 0U;
        l_u8PulseMode = 0U;
        if (g_e8MotorDirectionCCW != l_e8SpikePulseMotorDirCCW_Prev)
        {
            l_e8SpikePulseMotorDirCCW_Prev = g_e8MotorDirectionCCW;
            l_u16SpikePulseSkipCount = C_EDGE_SPIKE_SKIP;
        }
        g_u16SpikePeriodSum = 0U;                                               /* Total period (in commutation-ISR periods) */
        g_u16SpikeCount = 0U;                                                   /* Number of spikes found in sum-period */
    }
    EXIT_SECTION(); /*lint !e438 */
#else  /* (_SUPPORT_CURRSPIKE_POSITION != FALSE) */
    ENTER_SECTION(ATOMIC_KEEP_MODE);  /*lint !e534 */
    {
#if (_SUPPORT_NV_NON_BLOCK_WRITE != FALSE)
        l_u16StallDetectorDelay = g_u16StallDetectorDelay_NV;
#else  /* (_SUPPORT_NV_NON_BLOCK_WRITE != FALSE) */
        l_u16StallDetectorDelay = NV_STALL_DETECTOR_DELAY;
#endif /* (_SUPPORT_NV_NON_BLOCK_WRITE != FALSE) */
        if (l_u16StallDetectorDelay < C_MOVAVG_SZ)
        {
            l_u16StallDetectorDelay = C_MOVAVG_SZ;
        }
        if (l_u16StallDetectorDelay > (2U * C_MOVAVG_SZ) )
        {
            l_u16StallDetectorThrshld = (l_u16StallDetectorDelay - (2U * C_MOVAVG_SZ));
        }
        else
        {
            l_u16StallDetectorThrshld = 0U;
        }
        l_u16MotorCurrentLPF = 0U;                                              /* Low-pass Filtered motor-current [ADC-LSB] */
        l_u32MotorCurrentLPF = 0U;
#if !defined (C_MOVAVG_SSZ) || (C_MOVAVG_SZ > ((C_MOTOR_PHASES * C_MOTOR_STEP_PER_PHASE) * C_MICROSTEP_PER_FULLSTEP))
        l_u16MotorCurrentRawIdx = 0U;                                           /* Raw current moving average index */
#endif /* !defined (C_MOVAVG_SSZ) || (C_MOVAVG_SZ > ((C_MOTOR_PHASES * C_MOTOR_STEP_PER_PHASE) * C_MICROSTEP_PER_FULLSTEP)) */
        l_u16MotorCurrentMovAvgxN = 0U;                                         /* Moving average motor-current (x 4..16) */
        l_au16MotorCurrentRaw[0] = 0U;
#if (_SUPPORT_CDI != FALSE)
        l_u16MotorCurrentMovAvgxFS = 0U;
        l_u16MotorCurrentRawIdxFS = 0U;                                        /* Raw current moving average index */
#endif /* (_SUPPORT_CDI != FALSE) */
    }
    EXIT_SECTION(); /*lint !e438 */
#endif /* (_SUPPORT_CURRSPIKE_POSITION != FALSE) */
    p_MemSet( (uint16_t *)&l_au16MotorCurrentRaw[0], 0U, (uint16_t)(C_MOVAVG_SZ * sizeof(l_au16MotorCurrentRaw[0])));
#if (_SUPPORT_CDI != FALSE)
    p_MemSet( (uint16_t *)&l_au16MotorCurrentRawFS[0], 0U,
              (uint16_t)(C_NR_OF_FULLSTEPS * sizeof(l_au16MotorCurrentRawFS[0])));
#endif /* (_SUPPORT_CDI != FALSE) */
#if (_DEBUG_MOTOR_CURRENT_FLT != FALSE)
#if (_SUPPORT_STALL_REVERSE != FALSE)
    if ( (g_u8RewindFlags & (uint8_t)C_REWIND_ACTIVE) == 0U)
#endif /* (_SUPPORT_STALL_REVERSE != FALSE) */
    {
        g_u16DebugBufWrIdx = 0U;
#if (_DEBUG_MCUR_SUB_SAMPLED != FALSE)
        g_u16SubSamplingIdx = 0U;
#endif /* (_DEBUG_MCUR_SUB_SAMPLED != FALSE) */
#if (UART_COMM != FALSE) && (_SUPPORT_UART_IF_APP == C_UART_IF_SCOPE) && (_SUPPORT_UART_SCOPE_MODE == CH12_SCOPE)
        g_u16DebufBufUartTxIdx = 0U;
#endif /* (UART_COMM != FALSE) && (_SUPPORT_UART_IF_APP == C_UART_IF_SCOPE) && (_SUPPORT_UART_SCOPE_MODE == CH12_SCOPE) */
    }
#endif /* (_DEBUG_MOTOR_CURRENT_FLT != FALSE) */
} /* End of MotorDriverCurrentMeasureInit() */

#if  (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM_BIPOLAR)
/*!*************************************************************************** *
 * MotorDriverCurrentMeasure
 * \brief   Measure a average motor current, based on ADC current's
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16MicroStepIdx: Micro-step index
 * \return  (uint16_t) u16MicroStepMotorCurrent: Micro-step motor coil current.
 * *************************************************************************** *
 * \details The current measurement is taken every micro-step an actuator
 *          current (ADC in Auto-sequence mode), and add it to the moving-
 *          average filter to get a filtered actuator current. This moving-
 *          average current is used by a second low-pass-filter to get a
 *          long-term average current.
 * *************************************************************************** *
 * - Call Hierarchy: ISR_CTIMER0_3()
 * - Cyclomatic Complexity: 3+1
 * - Nesting: 1
 * - Function calling: 1 (ADC_GetRawMotorDriverCurrent())
 * *************************************************************************** */
uint16_t MotorDriverCurrentMeasure(uint16_t u16MicroStepIdx)
#elif (_SUPPORT_FOC_MODE != FOC_MODE_NONE) || (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM)
#if (_SUPPORT_FOC_MODE != FOC_MODE_NONE) && _SUPPORT_OPTIMIZE_FOR_SPEED
void MotorDriverCurrentMeasure(uint16_t u16MicroStepIdx) __attribute__((aligned(8)));
#endif /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) */
/*!*************************************************************************** *
 * MotorDriverCurrentMeasure
 * \brief   Measure a average motor current, based on ADC current's
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16MicroStepIdx
 * \return  -
 * *************************************************************************** *
 * \details The current measurement is taken every micro-step an actuator
 *          current (ADC in Auto-sequence mode), and add it to the moving-
 *          average filter to get a filtered actuator current. This moving-
 *          average current is used by a second low-pass-filter to get a
 *          long-term average current.
 * *************************************************************************** *
 * - Call Hierarchy: ISR_CTIMER0_3()
 * - Cyclomatic Complexity: 3+1
 * - Nesting: 1
 * - Function calling: 1 (ADC_GetRawMotorDriverCurrent())
 * *************************************************************************** */
void MotorDriverCurrentMeasure(uint16_t u16MicroStepIdx)
#else  /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) || (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM) || (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM_BIPOLAR) */
/*!*************************************************************************** *
 * MotorDriverCurrentMeasure
 * \brief   Measure a average motor current, based on ADC current's
 * \author  mmp
 * *************************************************************************** *
 * \param  [in] u16MicroStepIdx
 * \return  -
 * *************************************************************************** *
 * \details The current measurement is taken every micro-step an actuator
 *          current (ADC in Auto-sequence mode), and add it to the moving-
 *          average filter to get a filtered actuator current. This moving-
 *          average current is used by a second low-pass-filter to get a
 *          long-term average current.
 * *************************************************************************** *
 * - Call Hierarchy: ISR_CTIMER0_3()
 * - Cyclomatic Complexity: 2+1
 * - Nesting: 1
 * - Function calling: 2 (ADC_GetRawMotorDriverCurrent(), p_DecNzU16())
 * *************************************************************************** */
static void MotorDriverCurrentMeasure(uint16_t u16MicroStepIdx)
#endif /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) || (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM) || (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM_BIPOLAR) */
{
#if (C_MOTOR_PHASES == 4)
    static uint16_t u16LastMicroStepIdx = 0U;
#endif /* (C_MOTOR_PHASES == 4) */

#if (_SUPPORT_PWM_POL_CORR == FALSE)
    uint16_t u16MicroStepMotorCurrent = ADC_GetRawMotorDriverCurrent(l_u16MicroStepIdxPrev, l_u16PwmStatePrev);
#else  /* (_SUPPORT_PWM_POL_CORR == FALSE) */
    uint16_t u16MicroStepMotorCurrent = ADC_GetRawMotorDriverCurrent(u16MicroStepIdx);
#endif /* (_SUPPORT_PWM_POL_CORR == FALSE) */
#if (_SUPPORT_FOC_MODE != FOC_MODE_NONE) && (C_MOTOR_PHASES != 1)
    l_u16Ipk = (u16MicroStepMotorCurrent + l_u16Ipk_Prev) >> 1;
    l_u16Ipk_Prev = u16MicroStepMotorCurrent;
#endif /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) && (C_MOTOR_PHASES != 1) */

#if (_DEBUG_MOTOR_CURRENT_FLT != FALSE) && (_DEBUG_MCUR_SENSE != FALSE)
#if FALSE && (C_DEBUG_BUF_SZ > (C_MICROSTEP_PER_FULLSTEP * 6U * 3U))
    {
        uint8_t *pBfr = &g_au8DebugBuf[u16MicroStepIdx * 3U];
        /* 12-bit ADC */
        pBfr[0] = (uint8_t)(g_i16MotorCurrentCoilA >> 3);
        pBfr[1] = (uint8_t)(g_i16MotorCurrentCoilB >> 3);
        pBfr[2] = (uint8_t)(g_i16MotorCurrentCoilC >> 3);
    }
#else  /* (C_DEBUG_BUF_SZ > (C_MICROSTEP_PER_FULLSTEP * 6U * 3U)) */
#if (_DEBUG_MCUR_CYCLIC == FALSE)
    if (g_u16DebugBufWrIdx < C_DEBUG_BUF_SZ)
#endif /* (_DEBUG_MCUR_CYCLIC == FALSE) */
#if (_DEBUG_MCUR_SUB_SAMPLED != FALSE)
    if ( (g_u16SubSamplingIdx & C_SUB_SAMPLE_MASK) == 0)
#endif /* (_DEBUG_MCUR_SUB_SAMPLED != FALSE) */
    {
        uint8_t *pBfr = &g_au8DebugBuf[g_u16DebugBufWrIdx];
#define C_MCUR_SHR 0
        /* 10-bit ADC */
        pBfr[0] = (uint8_t)(g_i16MotorCurrentCoilA >> C_MCUR_SHR);
        pBfr[1] = (uint8_t)(g_i16MotorCurrentCoilB >> C_MCUR_SHR);
        pBfr[2] = (uint8_t)(g_i16MotorCurrentCoilC >> C_MCUR_SHR);
        g_u16DebugBufWrIdx += 3U;
#if (_DEBUG_MCUR_CYCLIC != FALSE)
        if (g_u16DebugBufWrIdx >= C_DEBUG_BUF_SZ)
        {
            g_u16DebugBufWrIdx = 0U;
        }
#endif /* (_DEBUG_MCUR_CYCLIC != FALSE) */
    }
#endif /* (C_DEBUG_BUF_SZ > (C_MICROSTEP_PER_FULLSTEP * 6U * 3U)) */
#endif /* _DEBUG_MOTOR_CURRENT_FLT && (_DEBUG_MCUR_SENSE != FALSE) */

    /* Moving average (sum) of motor-driver current */
    {
#if defined (C_MOVAVG_SSZ) && (C_MOVAVG_SZ <= ((C_MOTOR_PHASES * C_MOTOR_STEP_PER_PHASE) * C_MICROSTEP_PER_FULLSTEP))
        uint16_t *pu16MotorCurrentElement = &l_au16MotorCurrentRaw[u16MicroStepIdx & (C_MOVAVG_SZ - 1U)];
        uint16_t u16PrevMotorCurrent = *pu16MotorCurrentElement;
#else  /* defined (C_MOVAVG_SSZ) */
        uint16_t *pu16MotorCurrentElement = &l_au16MotorCurrentRaw[l_u16MotorCurrentRawIdx];
        uint16_t u16PrevMotorCurrent = *pu16MotorCurrentElement;
        l_u16MotorCurrentRawIdx++;
        if (l_u16MotorCurrentRawIdx >= C_MOVAVG_SZ)
        {
            l_u16MotorCurrentRawIdx = 0U;
        }
#endif /* defined (C_MOVAVG_SSZ) */
#if (_SUPPORT_FOC_MODE == FOC_MODE_NONE) && (_SUPPORT_PWM_MODE != SINGLE_COIL_PWM)
        if ( (l_u16StallDetectorDelay != 0U) || (u16PrevMotorCurrent < C_MIN_MOTORCURRENT) ||
             (u16MicroStepMotorCurrent < (u16PrevMotorCurrent << 2)) )          /* Check for valid motor-driver current (at least smaller than 4x previous current)  */
#endif /* (_SUPPORT_FOC_MODE == FOC_MODE_NONE) && (_SUPPORT_PWM_MODE != SINGLE_COIL_PWM) */
        {
            l_u16MotorCurrentMovAvgxN -= u16PrevMotorCurrent;                   /* Subtract oldest raw motor-driver current */
            l_u16MotorCurrentMovAvgxN += u16MicroStepMotorCurrent;              /* Add newest raw motor-driver current */
            *pu16MotorCurrentElement = u16MicroStepMotorCurrent;                /* Overwrite oldest with newest motor-driver current */
        }
    }

    /* During twice the moving-average-buffer size and during acceleration of the motor, LPF should follow
     * lowest value of LPF or Motor-current. As the speed is increasing so also is the BEMF also increasing,
     * which causes the current to decrease. Otherwise a first order (IIR-1) LPF is used. */
    {
#if defined (C_MOVAVG_SSZ) && (C_MOVAVG_SZ < ((C_MOTOR_PHASES * C_MOTOR_STEP_PER_PHASE) * C_MICROSTEP_PER_FULLSTEP))
        uint16_t u16MotorCurrentAcc = (l_u16MotorCurrentMovAvgxN >> C_MOVAVG_SSZ);
#else  /* defined (C_MOVAVG_SSZ) */
        uint16_t u16MotorCurrentAcc = p_DivU16_U32byU16( (uint32_t)l_u16MotorCurrentMovAvgxN, C_MOVAVG_SZ);
#endif /* defined (C_MOVAVG_SSZ) */
#if (_SUPPORT_PWM_MODE != SINGLE_COIL_PWM)
        if ( (l_u16StallDetectorDelay <= l_u16StallDetectorThrshld) &&
             (l_e8MotorStartupMode != E_MSM_STEPPER_D) &&
             ((l_e8MotorStartupMode != E_MSM_STEPPER_A) ||
              (u16MotorCurrentAcc >= l_u16MotorCurrentLPF)) )
#else  /* (_SUPPORT_PWM_MODE != SINGLE_COIL_PWM) */
        if (l_u16StallDetectorDelay <= l_u16StallDetectorThrshld)
#endif /* (_SUPPORT_PWM_MODE != SINGLE_COIL_PWM) */
        {
#if (_DEBUG_FOC_PERF != FALSE)
            DEBUG_SET_IO_D();
#endif /* (_DEBUG_FOC_PERF != FALSE) */
            l_u16MotorCurrentLPF =
                p_LpfU16_I16byI16(&l_u32MotorCurrentLPF, 1024, (int16_t)(u16MotorCurrentAcc - l_u16MotorCurrentLPF));  /* MMP171026-1 */
#if (_DEBUG_FOC_PERF != FALSE)
            DEBUG_CLR_IO_D();
#endif /* (_DEBUG_FOC_PERF != FALSE) */
        }
        else
        {
            l_u16MotorCurrentLPF = u16MotorCurrentAcc;
            l_u32MotorCurrentLPF = ((uint32_t)l_u16MotorCurrentLPF << 16U);
        }
    }

#if (_SUPPORT_CURRSPIKE_POSITION != FALSE)
    l_u16PulsePeriodCount++;

    /* Motor A */
    uint16_t u16SpikeAmplitude = 0;
    if (l_u16SpikePulseSkipCount == 0U)
    {
        if (l_u16MotorCurrentLPF > (C_MIN_MCUR_ADCLSB << C_MOVAVG_SSZ) )
        {
            if ( (l_u16SpikeEdge & C_SPIKE_RAISING) != 0U)
            {
                u16SpikeAmplitude = p_MulU16hi_U16byU16(l_u16MotorCurrentLPF, C_MCUR_RAISING_SPIKE_THRSHLD);
                if (u16MicroStepMotorCurrent >= u16SpikeAmplitude)
                {
#if (_DEBUG_SPIKE_DETECTED != FALSE)
                    DEBUG_SET_IO_D();
#endif /* (_DEBUG_SPIKE_DETECTED != FALSE) */
                    l_u16SpikeEdge = C_SPIKE_FALLING;
                    l_u16SpikePulseSkipCount = C_EDGE_SPIKE_SKIP;
                    if (g_e8MotorDirectionCCW != FALSE)
                    {
                        g_u16ActualSpikePos--;                                  /* CCW-direction */
                    }
                    else
                    {
                        g_u16ActualSpikePos++;                                  /* CW-direction */
                    }
                    g_u16SpikePeriodSum += l_u16PulsePeriodCount;               /* Number of Communication period's */
                    g_u16SpikeCount++;                                          /* Number of spikes */
                    l_u16LastPulsePeriod = l_u16PulsePeriodCount;
                    l_u16PulsePeriodCount = 0U;
                    if (l_u8PulseMode <= C_START_PULSE_COUNT)
                    {
                        /* 0: Start of First pulse, 1: End of first pulse/Start-of-second, 2: End of Second */
                        if (l_u8PulseMode != 0U)
                        {
                            l_u16StartPulseWidth += l_u16LastPulsePeriod;
                            if (l_u8PulseMode == C_START_PULSE_COUNT)
                            {
                                /* Average time of C_START_PULSE_COUNT start pulses */
                                l_u16StartPulseWidth >>= C_START_PULSE_SCNT;
                            }
                        }
                        else
                        {
                            /* First pulse(s) will be missed */
                            if (g_e8MotorDirectionCCW != FALSE)
                            {
                                g_u16ActualSpikePos -= C_START_PULSE_LOST;      /* CCW-direction */
                            }
                            else
                            {
                                g_u16ActualSpikePos += C_START_PULSE_LOST;      /* CW-direction */
                            }
                        }
                        l_u8PulseMode++;
                    }
                }
            }
            if ( (l_u16SpikeEdge & C_SPIKE_FALLING) != 0U)
            {
                u16SpikeAmplitude = p_MulU16hi_U16byU16(l_u16MotorCurrentLPF, C_MCUR_FALLING_SPIKE_THRSHLD);
                if (u16MicroStepMotorCurrent <= u16SpikeAmplitude)
                {
#if (_DEBUG_SPIKE_DETECTED != FALSE)
                    DEBUG_CLR_IO_D();
#endif /* (_DEBUG_SPIKE_DETECTED != FALSE) */
                    l_u16SpikeEdge = C_SPIKE_RAISING;
                    l_u16SpikePulseSkipCount = C_EDGE_SPIKE_SKIP;
                }
            }
        }
    }
    else
    {
        l_u16SpikePulseSkipCount--;
    }
#endif /* (_SUPPORT_CURRSPIKE_POSITION != FALSE) */

#if (C_MOTOR_PHASES == 4)
    /* Stall delay is based in micro-steps; Count down each new micro-step index */
    if ( (l_u16StallDetectorDelay > 0U) && (l_u16MicroStepIdx != u16LastMicroStepIdx) )
    {
        l_u16StallDetectorDelay--;
        u16LastMicroStepIdx = l_u16MicroStepIdx;
    }
#else  /* (C_MOTOR_PHASES == 4) */
    l_u16StallDetectorDelay = p_DecNzU16(l_u16StallDetectorDelay);
#endif /* (C_MOTOR_PHASES == 4) */

#if  (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM_BIPOLAR)
    return (u16MicroStepMotorCurrent);
#endif /* (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM_BIPOLAR) */
} /* End of MotorDriverCurrentMeasure() */

#if (_SUPPORT_CDI != FALSE)
/*!*************************************************************************** *
 * MotorDriverCurrentMeasureCDI
 * \brief   Measure a average motor current, based on ADC current's
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details The current measurement is taken every full-step an actuator
 *          current (ADC in Auto-sequence mode), and add it to the moving-
 *          average filter to get a filtered actuator current. This moving-
 *          average current is used by a second low-pass-filter to get a
 *          long-term average current.
 * *************************************************************************** *
 * - Call Hierarchy: ISR_CTIMER0_3()
 * - Cyclomatic Complexity: 2+1
 * - Nesting: 1
 * - Function calling: 2 (ADC_GetRawMotorDriverCurrent(), p_DecNzU16())
 * *************************************************************************** */
void MotorDriverCurrentMeasureCDI(void)
{
    uint16_t u16FullStepMotorCurrent = ADC_GetRawMotorDriverCurrentCDI();

    /* Moving average (sum) of motor-driver current */
    {
        uint16_t *pu16MotorCurrentElement = &l_au16MotorCurrentRawFS[l_u16MotorCurrentRawIdxFS];
        uint16_t u16PrevMotorCurrent = *pu16MotorCurrentElement;
        l_u16MotorCurrentRawIdxFS++;
        if (l_u16MotorCurrentRawIdxFS >= C_NR_OF_FULLSTEPS)
        {
            l_u16MotorCurrentRawIdxFS = 0U;
        }
        l_u16MotorCurrentMovAvgxFS -= u16PrevMotorCurrent;                       /* Subtract oldest raw motor-driver current */
        l_u16MotorCurrentMovAvgxFS += u16FullStepMotorCurrent;                   /* Add newest raw motor-driver current */
        *pu16MotorCurrentElement = u16FullStepMotorCurrent;                     /* Overwrite oldest with newest motor-driver current */
        l_u16MotorCurrentMovAvgxN =
            p_MulDivU16_U16byU16byU16(l_u16MotorCurrentMovAvgxFS, C_MOVAVG_SZ, C_NR_OF_FULLSTEPS);
    }

    /* During twice the moving-average-buffer size and during acceleration of the motor, LPF should follow
     * lowest value of LPF or Motor-current. As the speed is increasing so also is the BEMF also increasing,
     * which causes the current to decrease. Otherwise a first order (IIR-1) LPF is used. */
    {
        uint16_t u16MotorCurrentAcc = p_DivU16_U32byU16( (uint32_t)l_u16MotorCurrentMovAvgxFS, C_NR_OF_FULLSTEPS);
#if (_SUPPORT_PWM_MODE != SINGLE_COIL_PWM)
        if ( (l_u16StallDetectorDelay <= l_u16StallDetectorThrshld) &&
             (l_e8MotorStartupMode != E_MSM_STEPPER_D) &&
             ((l_e8MotorStartupMode != E_MSM_STEPPER_A) ||
              (u16MotorCurrentAcc >= l_u16MotorCurrentLPF)) )
#else  /* (_SUPPORT_PWM_MODE != SINGLE_COIL_PWM) */
        if (l_u16StallDetectorDelay <=  l_u16StallDetectorThrshld)
#endif /* (_SUPPORT_PWM_MODE != SINGLE_COIL_PWM) */
        {
            l_u16MotorCurrentLPF =
                p_LpfU16_I16byI16(&l_u32MotorCurrentLPF, 1024, (int16_t)(u16MotorCurrentAcc - l_u16MotorCurrentLPF));                      /* MMP171026-1 */
        }
        else
        {
            l_u16MotorCurrentLPF = u16MotorCurrentAcc;
            l_u32MotorCurrentLPF = ((uint32_t)l_u16MotorCurrentLPF << 16U);
        }
    }
} /* End of MotorDriverCurrentMeasureCDI() */
#endif /* (_SUPPORT_CDI != FALSE) */

/*!*************************************************************************** *
 * MotorDriverHoldCurrentMeasure
 * \brief   Measure a motor holding current, based on ADC current's
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details The current measurement is taken in holding mode.
 * *************************************************************************** *
 * - Call Hierarchy: AppCurrentCheck()
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 2 (ADC_GetRawMotorDriverCurrent(), p_LpfU16_I16byI16())
 * *************************************************************************** */
void MotorDriverHoldCurrentMeasure(void)
{
    uint16_t u16HoldMotorCurrent = ADC_GetRawMotorDriverCurrent(l_u16MicroStepIdx);
    l_u16MotorCurrentLPF =
        p_LpfU16_I16byI16(&l_u32MotorCurrentLPF, 1024, (int16_t)(u16HoldMotorCurrent - l_u16MotorCurrentLPF));

#if defined (C_MOVAVG_SSZ)
    l_u16MotorCurrentMovAvgxN = (l_u16MotorCurrentLPF << C_MOVAVG_SSZ);
#else  /* defined (C_MOVAVG_SSZ) */
    l_u16MotorCurrentMovAvgxN = (uint16_t)p_MulU32_U16byU16(l_u16MotorCurrentLPF, C_MOVAVG_SZ);
#endif /* defined (C_MOVAVG_SSZ) */
} /* End of MotorDriverHoldCurrentMeasure() */

#if (_SUPPORT_PWM_MODE != SINGLE_COIL_PWM) && ((_SUPPORT_PWM_MODE != SINGLE_COIL_PWM_BIPOLAR) || _SUPPORT_HALL_LATCH) && \
    (C_MICROSTEP_PER_FULLSTEP != 0)
/*!*************************************************************************** *
 * MotorDriverUpdateMicroStepIndex
 * \brief   Update the actuator micro-step index
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16MicroStepIdx: Actual index
 * \return  (uint16_t) updated index
 * *************************************************************************** *
 * \details -
 * *************************************************************************** *
 * - Call Hierarchy: ISR_CTIMER0_3(), MotorDriverStart()
 * - Cyclomatic Complexity: 3+1
 * - Nesting: 2
 * - Function calling: 0
 * *************************************************************************** */
#if (_SUPPORT_FOC_MODE != FOC_MODE_NONE) || ((_SUPPORT_PWM_MODE != SINGLE_COIL_PWM_BIPOLAR) || _SUPPORT_HALL_LATCH)
uint16_t MotorDriverUpdateMicroStepIndex(uint16_t u16MicroStepIdx)
#else  /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) || ((_SUPPORT_PWM_MODE != SINGLE_COIL_PWM_BIPOLAR) || _SUPPORT_HALL_LATCH) */
static uint16_t MotorDriverUpdateMicroStepIndex(uint16_t u16MicroStepIdx)
#endif /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) || ((_SUPPORT_PWM_MODE != SINGLE_COIL_PWM_BIPOLAR) || _SUPPORT_HALL_LATCH) */
{
    if (g_e8MotorDirectionCCW != ((uint8_t)C_MOTOR_DIR_CW) )
    {
        /* Counter Clock-wise (Closing) */
        if (u16MicroStepIdx == 0U)
        {
            u16MicroStepIdx = l_u16MotorMicroStepsPerElecRotation;
        }
#if (_SUPPORT_PWM_MODE == TRIPLEPHASE_TWOPWM_INDEPENDENT_GND) && (_SUPPORT_PWM_POL_CORR == FALSE)
        else if (u16MicroStepIdx == ((6U * C_MICROSTEP_PER_FULLSTEP) - (C_MICROSTEP_PER_FULLSTEP / 2U)) )
        {
            l_u16PwmState = l_u16PwmState ^ TRUE;
        }
        else
        {
            /* Nothing */
        }
#endif /* (_SUPPORT_PWM_MODE == TRIPLEPHASE_TWOPWM_INDEPENDENT_GND) && (_SUPPORT_PWM_POL_CORR == FALSE) */
        u16MicroStepIdx--;
    }
    else
    {
        /* Clock-wise (Opening) */
        u16MicroStepIdx++;
        if (u16MicroStepIdx >= l_u16MotorMicroStepsPerElecRotation)             /* Check the PWM vectors pointer: 48 usteps per electrical period */
        {
            u16MicroStepIdx -= l_u16MotorMicroStepsPerElecRotation;             /* Re-initialise the PWM vectors pointer to 0 */
        }
#if (_SUPPORT_PWM_MODE == TRIPLEPHASE_TWOPWM_INDEPENDENT_GND) && (_SUPPORT_PWM_POL_CORR == FALSE)
        else if (u16MicroStepIdx == ((6U * C_MICROSTEP_PER_FULLSTEP) - (C_MICROSTEP_PER_FULLSTEP / 2U)) )
        {
            l_u16PwmState = l_u16PwmState ^ TRUE;
        }
        else
        {
            /* Nothing */
        }
#endif /* (_SUPPORT_PWM_MODE == TRIPLEPHASE_TWOPWM_INDEPENDENT_GND) && (_SUPPORT_PWM_POL_CORR == FALSE) */
    }
    return (u16MicroStepIdx);
} /* End of MotorDriverUpdateMicroStepIndex() */
#endif /* (_SUPPORT_PWM_MODE != SINGLE_COIL_PWM) && ((_SUPPORT_PWM_MODE != SINGLE_COIL_PWM_BIPOLAR) || _SUPPORT_HALL_LATCH) && (C_MICROSTEP_PER_FULLSTEP != 0) */

#if (_SUPPORT_CDI != FALSE)
/*!*************************************************************************** *
 * MotorDriverUpdateFullStepIndex
 * \brief   Update the actuator Full-step index
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16FullStepIdx: Actual index
 * \return  (uint16_t) updated index
 * *************************************************************************** *
 * \details -
 * *************************************************************************** *
 * - Call Hierarchy: ISR_CTIMER0_3(), MotorDriverStart()
 * - Cyclomatic Complexity: 3+1
 * - Nesting: 2
 * - Function calling: 0
 * *************************************************************************** */
uint16_t MotorDriverUpdateFullStepIndex(uint16_t u16FullStepIdx)
{
    if (g_e8MotorDirectionCCW != ((uint8_t)C_MOTOR_DIR_CW) )
    {
        /* Counter Clock-wise (Closing) */
        if (u16FullStepIdx == 0U)
        {
            u16FullStepIdx = l_u16MotorFullStepsPerElecRotation;
        }
        u16FullStepIdx--;
    }
    else
    {
        /* Clock-wise (Opening) */
        u16FullStepIdx++;
        if (u16FullStepIdx >= l_u16MotorFullStepsPerElecRotation)              /* Check the PWM vectors pointer: 48 usteps per electrical period */
        {
            u16FullStepIdx -= l_u16MotorFullStepsPerElecRotation;              /* Re-initialise the PWM vectors pointer to 0 */
        }
    }
    return (u16FullStepIdx);
} /* End of MotorDriverUpdateFullStepIndex() */
#endif /* (_SUPPORT_CDI != FALSE) */

#if (_SUPPORT_SPEED_CTRL != FALSE)
/*!*************************************************************************** *
 * MotorDriverSpeed
 * \brief   Motor Driver Speed depended parameters
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16MotorTargetSpeed: Motor Speed Index (mode)
 * \return  -
 * *************************************************************************** *
 * \details -
 * *************************************************************************** *
 * - Call Hierarchy: MotorDriverStart()
 * - Cyclomatic Complexity: 2+1
 * - Nesting: 2
 * - Function calling: 1 (DeltaPosition())
 * *************************************************************************** */
void MotorDriverSpeed(uint16_t u16MotorTargetSpeed)
{
#if (_SUPPORT_PWM_MODE != SINGLE_COIL_PWM)
#if (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE)
    uint16_t u16DeltaPosition = DeltaPosition();                                /* Actual delta between actual and target position */
    if (u16DeltaPosition < l_u16DeltaPosition)                                  /* Check against last known delta position */
    {
        /* Don't change any thing; ramp-down is ongoing or will start at the next commutation event */
    }
    else
#endif /* (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) */
    {
        /* New target is further away */
        /* (Micro-) step commutation-time [Timer] */
#if (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) && (_SUPPORT_ACT_SPEED_BY_LIN == FALSE)
        uint8_t u8MotorSpeedIdx = (uint8_t)u16MotorTargetSpeed;

        g_u16TargetMotorSpeedRPM = l_au16MotorSpeedRPM[u8MotorSpeedIdx];
        g_u8MotorStatusSpeed = u8MotorSpeedIdx;
#else /* (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) && (_SUPPORT_ACT_SPEED_BY_LIN == FALSE) */
        g_u16TargetMotorSpeedRPM = u16MotorTargetSpeed;
#endif /* (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) && (_SUPPORT_ACT_SPEED_BY_LIN == FALSE) */
        l_u16TargetCommutTimerPeriod = p_DivU16_U32byU16(l_u32MicroStepPeriodOneRPM, g_u16TargetMotorSpeedRPM) - 1U;  /* -1 for at 'least'-speed */

        if ( (g_e8MotorStatus & (uint8_t)C_MOTOR_STATUS_MASK) == (uint8_t)C_MOTOR_STATUS_STOPPING)
        {
            /* Busy with rampdown/stopping; Cancel it */
            g_e8MotorStatus = (uint8_t)C_MOTOR_STATUS_RUNNING;                  /* Remove stopping flag */
            l_e8MotorStartupMode = E_MSM_STEPPER_A;                             /* Change to acceleration */
#if FALSE
            if ( ((l_u16RampDownSteps + l_u16DeltaPosition) * 2) < u16DeltaPosition)
            {
                l_u16RampDownSteps = (l_u16DeltaPosition + u16DeltaPosition) >> 1;
            }
#endif /* FALSE */
        }
    }
#else  /* (_SUPPORT_PWM_MODE != SINGLE_COIL_PWM) */
#if (_SUPPORT_ACT_SPEED_BY_LIN == FALSE)
    uint8_t u8MotorSpeedIdx = (uint8_t)u16MotorTargetSpeed;

    g_u16TargetMotorSpeedRPM = l_au16MotorSpeedRPM[u8MotorSpeedIdx];
    g_u8MotorStatusSpeed = u8MotorSpeedIdx;
#else  /* (_SUPPORT_ACT_SPEED_BY_LIN != FALSE) */
    g_u16TargetMotorSpeedRPM = u16MotorTargetSpeed;
#endif /* (_SUPPORT_ACT_SPEED_BY_LIN != FALSE) */
#endif /* (_SUPPORT_PWM_MODE != SINGLE_COIL_PWM) */
} /* End of MotorDriverSpeed() */
#endif /* (_SUPPORT_SPEED_CTRL != FALSE) */

#if (_SUPPORT_PWM_MODE != SINGLE_COIL_PWM) && (_SUPPORT_PWM_MODE != SINGLE_COIL_PWM_BIPOLAR)
#if (_SUPPORT_FOC_MODE == FOC_MODE_NONE)
/*!*************************************************************************** *
 * MotorDriverStart
 * \brief   Start Motor Driver
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16MotorTargetSpeed: Target Speed [RPM]
 * \return  -
 * *************************************************************************** *
 * \details -
 * *************************************************************************** *
 * - Call Hierarchy: HandleStartMotorRequest()
 * - Cyclomatic Complexity: 6+1
 * - Nesting: 3
 * - Function calling: 11 (MotorDriverSelfTest(), ADC_Init(),
 *                         MotorDriverSpeed(), ConvShaftSteps2MicroSteps(),
 *                         MotorDriverCurrentMeasureInit(), MotorStallInitA(),
 *                         MotorDriver_InitialPwmDutyCycle(),
 *                         MotorDriverUpdateMicroStepIndex(),
 *                         MotorDriver_4Phase(), ADC_Start(),
 *                         Get_ActCurrRunMax_mA())
 * *************************************************************************** */
void MotorDriverStart(uint16_t u16MotorTargetSpeed)
{
    if ( (g_e8ErrorElectric & (uint8_t)C_ERR_PERMANENT) != 0U)                  /* Don't start motor in case of permanent electric failure */
    {
        return;
    }
#if (_SUPPORT_MOTOR_SELFTEST != FALSE)
    else if ( (g_u8ForceMotorDriverSelfTest != FALSE) ||                        /* MMP180917-1 */
              ((g_e8ErrorElectric & (uint8_t)C_ERR_MOTOR) != 0U) )
    {
        uint8_t u8MemorizedElectricError = g_e8ErrorElectric;
        g_e8ErrorElectric = (uint8_t)C_ERR_NONE;
        MotorDriverSelfTest();
        if (g_e8ErrorElectric != (uint8_t)C_ERR_NONE)
        {
            g_e8ErrorElectric = (g_e8ErrorElectric & ~C_ERR_PERMANENT) | u8MemorizedElectricError;
            return;
        }
        g_u8ForceMotorDriverSelfTest = FALSE;
    }
#else  /* (_SUPPORT_MOTOR_SELFTEST != FALSE) */
    g_e8ErrorElectric = (uint8_t)C_ERR_NONE;
#endif /* (_SUPPORT_MOTOR_SELFTEST != FALSE) */

    DiagnosticReset();

#if (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE)
    if (g_u8MotorHoldingCurrEna == FALSE)
    {
        ADC_Init();
    }
    else
#endif /* (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) */
    {
        ADC_MCurrOffCalib();
    }

#if (_SUPPORT_TACHO_OUT != FALSE)
    MotorDriverTachoInit();
#endif /* (_SUPPORT_TACHO_OUT != FALSE) */

    /* Normal operation */
    g_u16ForcedSpeedRPM = g_u16MinSpeedRPM;
#if (_SUPPORT_ACCELERATION == C_ACCELERATION_COSINE_CURVE)
    l_u16AccelerationConst = l_u16AccelerationMin;                               /* Initial/minimum acceleration */
#endif /* (_SUPPORT_ACCELERATION == C_ACCELERATION_COSINE_CURVE) */
#if (_SUPPORT_STALLDET_BZC != FALSE)
    /* [BEMF] */
    l_u32MicroStepPeriodOneRPM = p_DivU32_U32byU16( (TIMER_CLOCK * 60UL), l_u16MotorMicroStepsPerMechRotation);
    l_u16LowSpeedPeriod = p_DivU16_U32byU16(l_u32MicroStepPeriodOneRPM, g_u16ForcedSpeedRPM) - 1U;
    l_u16MaxBemfCommutTimerPeriod = p_DivU16_U32byU16(l_u32MicroStepPeriodOneRPM, (g_u16LowSpeedRPM / 2U)) - 1U; /* Minimum speed to switch from stepper to BEMF */
#endif /* (_SUPPORT_STALLDET_BZC != FALSE) */
    l_u16CommutTimerPeriod = l_u16LowSpeedPeriod;
    l_e8MotorStartupMode = E_MSM_STEPPER_A;                                     /* Start-up in Acceleration stepper mode */
#if (_SUPPORT_STALLDET_BZC != FALSE) || (_SUPPORT_STALLDET_BRI != FALSE)
    /* [BEMF] */
    IO_PWM_MASTER1_CTRL = (PWM_PRESCALER << 8) | C_PWM_MASTER1_MODE_MIRROR;     /* Initialise the master pre-scaler ratio (Fck/8) */
    IO_PWM_SLAVE1_CTRL = B_PWM_SLAVE1_SLAVE | C_PWM_SLAVE1_MODE_MIRROR;
    IO_PWM_SLAVE2_CTRL = B_PWM_SLAVE2_SLAVE | C_PWM_SLAVE2_MODE_MIRROR;
#endif /* (_SUPPORT_STALLDET_BZC != FALSE) || (_SUPPORT_STALLDET_BRI != FALSE) */
    MotorDriverSpeed(u16MotorTargetSpeed);
    if (g_u16TargetMotorSpeedRPM < g_u16ForcedSpeedRPM)
    {
        g_u16ForcedSpeedRPM = g_u16TargetMotorSpeedRPM;
    }

#if (_SUPPORT_REWIND != FALSE)
    if ( (g_u8RewindFlags & (uint8_t)C_REWIND_STALL_DETECT) != 0U)              /* Stall has been detected during previous movement */
    {
        if ( (l_u16MotorRewindSteps != 0U) &&                                   /* Rewind steps non-zero */
             (((g_u8RewindFlags & (uint8_t)C_REWIND_DIRECTION_CCW) == g_e8MotorDirectionCCW) ||   /* Same direction as last detected stall */
              ((g_u8RewindFlags & (uint8_t)C_REWIND_DIRECTION_ANY) != 0U)) )    /* or Any direction do rewind (POR) */
        {
            /* Start rewind-function, with "rewinding" */
            g_u8RewindFlags = (uint8_t)(C_REWIND_ACTIVE | C_REWIND_REWIND);     /* Start rewind-process */
            l_u16TargetPositionRewind = g_u16TargetPosition;                    /* Memorise Target-position */
            if (g_e8MotorDirectionCCW != FALSE)
            {
                /* Targeted movement is Counter Clock Wise rotational direction; Rewind to CW direction (higher position) */
                if (g_u16ActualPosition <= (uint16_t)(C_MAX_STEP_POS - l_u16MotorRewindSteps) )
                {
                    g_u16TargetPosition = g_u16ActualPosition + l_u16MotorRewindSteps;  /* Change Target-position to rewind-position */
                    g_e8MotorDirectionCCW = FALSE;                              /* Change direction */
                }
                else
                {
                    g_u8RewindFlags = 0U;                                       /* No rewind possible */
                }
            }
            else
            {
                /* Targeted movement is Clock Wise rotational direction; Rewind to CCW direction (lower position) */
                if (g_u16ActualPosition >= l_u16MotorRewindSteps)
                {
                    g_u16TargetPosition = g_u16ActualPosition - l_u16MotorRewindSteps;  /* Change Target-position to rewind-position */
                    g_e8MotorDirectionCCW = TRUE;                               /* Change direction */
                }
                else
                {
                    g_u8RewindFlags = 0U;                                       /* No rewind possible */
                }
            }
        }
        else if ( (g_u8RewindFlags & (uint8_t)C_REWIND_DIRECTION_CCW) != g_e8MotorDirectionCCW)
        {
            g_u8RewindFlags = 0U;                                               /* Clear previous detected stall flags */
        }
        else
        {
            /* Nothing */
        }
    }
#endif /* (_SUPPORT_REWIND != FALSE) */

#if (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE)
    g_u32TargetPosition = ConvShaftSteps2MicroSteps(g_u16TargetPosition);
#endif /* (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) */

    /* Clear motor-driver current measurement */
    MotorDriverCurrentMeasureInit();

#if (_SUPPORT_STALLDET_A != FALSE)
    MotorStallInitA();
#endif /* (_SUPPORT_STALLDET_A != FALSE) */
#if (_SUPPORT_STALLDET_LA != FALSE)
    MotorStallInitLA(u16MotorTargetSpeed);
#endif /* (_SUPPORT_STALLDET_LA != FALSE) */
#if (_SUPPORT_STALLDET_O != FALSE)
    MotorStallInitO();
#endif /* (_SUPPORT_STALLDET_O != FALSE) */
#if (_SUPPORT_STALLDET_P != FALSE)
    MotorStallInitP();
#endif /* (_SUPPORT_STALLDET_P != FALSE) */
#if (_SUPPORT_STALLDET_S != FALSE)
    MotorStallInitS();
#endif /* (_SUPPORT_STALLDET_S != FALSE) */
#if (_SUPPORT_STALLDET_BRI != FALSE)
    l_u8StallCountB = 0U;
#endif /* (_SUPPORT_STALLDET_BRI != FALSE) */
#if (_SUPPORT_STALLDET_BZC != FALSE)
    /* [BEMF] Speed control */
    l_u16CommutTimeIdx = 0U;
    l_u32SumCommutTimerPeriods = 0U;
    l_u16NrOfFullStepCommut = 0U;
    l_u8StallCountB = 0U;
#endif /* (_SUPPORT_STALLDET_BZC != FALSE) */

#if (_SUPPORT_HALL_LATCH != FALSE) && (_SUPPORT_NR_OF_HL > 1)
    l_u32SumCommutTimerPeriods = 0U;
    l_u16NrOfCommut = 0U;
    l_u16CommutTimeIdx = 0U;
    l_u16LastHallLatchEvent = TRUE;                                             /* Assume last Hall-Latch event was found (MMP230720-1) */
#if (_SUPPORT_MICRO_STEP_COMMUTATION != FALSE) && (_SUPPORT_HALL_LATCH_DIAG == FALSE)
    g_u8HallMicroSteps = 0U;
#endif /* (_SUPPORT_MICRO_STEP_COMMUTATION != FALSE) && (_SUPPORT_HALL_LATCH_DIAG == FALSE) */
#endif /* (_SUPPORT_HALL_LATCH != FALSE) && (_SUPPORT_NR_OF_HL > 1) */

    /* Connect drivers - BLDC */
    MotorDriverConfig(TRUE);

#if (_SUPPORT_WINDMILL != FALSE)
    l_u16MicroStepIdx = MotorWindMillCheck();                                   /* TODO: [MMP] Assume motor rotate in right direction!! Otherwise: BREAK and start-up from zero */
    if ( (l_u16MicroStepIdx & 0x8000U) == 0x0000U)
    {
        g_u16ActualMotorSpeedRPM = p_DivU16_U32byU16(l_u32MicroStepPeriodOneRPM, l_u16CommutTimerPeriod);
#if (_SUPPORT_LOG_ERRORS != FALSE)
        SetLastError(C_INF_WINDMILL | C_ERR_EXTW);
        SetLastError(g_u16ActualMotorSpeedRPM);
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
        MotorDriver_InitialPwmDutyCycle(Get_ActCurrRunMax_mA(), g_u16ActualMotorSpeedRPM);
        g_e8MotorStatus = C_MOTOR_STATUS_RUNNING;
    }
    else
#endif /* (_SUPPORT_WINDMILL != FALSE) */
    {
#if (_SUPPORT_HALL_LATCH != FALSE) && (_SUPPORT_NR_OF_HL > 1) && (_SUPPORT_HALL_LATCH_DIAG == FALSE)
        /* Full/Micro-step */
        g_u16HallLatchIO = IO_HALL_STATE;
        if (g_e8MotorDirectionCCW != ((uint8_t)C_MOTOR_DIR_CW) )
        {
            l_u16MicroStepIdx = au16HallLatchIdxCCW[g_u16HallLatchIO];
        }
        else
        {
            l_u16MicroStepIdx = au16HallLatchIdxCW[g_u16HallLatchIO];
        }
#endif /* (_SUPPORT_HALL_LATCH != FALSE) && (_SUPPORT_NR_OF_HL > 1) && (_SUPPORT_HALL_LATCH_DIAG == FALSE) */

        MotorDriver_InitialPwmDutyCycle(Get_ActCurrRunMax_mA(), g_u16ForcedSpeedRPM);

#if ((_SUPPORT_FOC_MODE & FOC_OBSERVER_MASK) == FOC_OBSERVER_SENSORED) || (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
        MeasureResolverPos();
        l_u16ResolverAngle = Triaxis_Angle();
        if (u16MotorTargetSpeed != 0U)
        {
            /* Convert Resolver-angle to Micro-step index */
            l_u16MicroStepIdx = MotorDriverResolverAngleToMicroStepIndex(l_u16ResolverAngle);
        }
        else
#endif /* ((_SUPPORT_FOC_MODE & FOC_OBSERVER_MASK) == FOC_OBSERVER_SENSORED) */
#if (_SUPPORT_HALL_LATCH == FALSE)
        {
            l_u16MicroStepIdx = MotorDriverUpdateMicroStepIndex(l_u16MicroStepIdx);
        }
#endif /* (_SUPPORT_HALL_LATCH == FALSE) */

        /* Micro-stepping: 48-steps (3-phase) */
#if (_SUPPORT_SOFT_START != FALSE)
#if (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE)
        if (g_u8MotorHoldingCurrEna == FALSE)
#endif /* (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) */
        {
            l_u16StartCorrectionRatio = l_u16CorrectionRatio;
            l_u16RampStep = (C_SOFT_START_RAMP_STEPS - 4U);
            l_u16CorrectionRatio = l_u16StartCorrectionRatio - p_MulDivU16_U16byU16byU16(l_u16StartCorrectionRatio,
                                                                                         l_u16RampStep,
                                                                                         C_SOFT_START_RAMP_STEPS);
            g_e8MotorStatus = (uint8_t)C_MOTOR_STATUS_SOFT_START;
        }
#if (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE)
        else
        {
            g_e8MotorStatus = (uint8_t)C_MOTOR_STATUS_RUNNING;
        }
#endif /* (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) */
#else  /* (_SUPPORT_SOFT_START != FALSE) */
        g_e8MotorStatus = C_MOTOR_STATUS_RUNNING;
#endif /* (_SUPPORT_SOFT_START != FALSE) */
    }
#if (_SUPPORT_CDI != FALSE)
    IO_ACTIVE_CDI = (B_IO_ACTIVE_CDI_ACTIVE_CDI_CLK_PREDIV2 |
                     C_IO_ACTIVE_CDI_ACTIVE_CDI_EDGE_SEL_FALL |
                     (4U << 8) | (2U << 6) | (7U << 2));                        /* Maximum CDI-Clock: 2MHz */
    l_u16CDI = 0U;
#if (_SUPPORT_APP_USER_MODE != FALSE)
    ENTER_SECTION(SYSTEM_MODE); /*lint !e534 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
    IO_MLX16_ITC_PRIO7_S = (IO_MLX16_ITC_PRIO7_S & ~M_MLX16_ITC_PRIO7_ACTIVE_CDI) | C_MLX16_ITC_PRIO7_ACTIVE_CDI_PRIO4;
    IO_MLX16_ITC_PEND5_S = B_MLX16_ITC_PEND5_ACTIVE_CDI;
    IO_MLX16_ITC_MASK5_S |= B_MLX16_ITC_MASK5_ACTIVE_CDI;
#if (_SUPPORT_APP_USER_MODE != FALSE)
    EXIT_SECTION(); /*lint !e438 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
#endif /* (_SUPPORT_CDI != FALSE) */
#if (C_MOTOR_PHASES == 3)
    MotorDriver_3Phase(l_u16MicroStepIdx);
#if (_SUPPORT_PWM_MODE != TRIPLEPHASE_FULL_STEP_BEMF) && (_SUPPORT_PWM_MODE != TRIPLEPHASE_FULL_STEP)
    /* Switch Drive to active synchronous with PWM-period */
#if (_SUPPORT_APP_USER_MODE != FALSE)
    ENTER_SECTION(SYSTEM_MODE); /*lint !e534 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
#if defined (__MLX81160__)
#if (_SUPPORT_EVB == MLX81160EVB_3P_3N)
    DRVCFG_DIS_3P_3N();
#elif (_SUPPORT_DUAL_BLDC != FALSE)
    DRVCFG_DIS_RST_UVW();
#elif (_SUPPORT_1ST_BLDC != FALSE)
    DRVCFG_DIS_RST();
#else
    DRVCFG_DIS_UVW();
#endif
    IO_MLX16_ITC_PEND1_S = B_MLX16_ITC_PEND1_PWM_MASTER1_END;
#elif defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__)
    DRVCFG_DIS_TUVW();
    IO_MLX16_ITC_PEND1_S = B_MLX16_ITC_PEND1_PWM_MASTER1_END;
#elif defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
    DRVCFG_DIS();
    IO_MLX16_ITC_PEND2_S = B_MLX16_ITC_PEND2_PWM_MASTER1_END;
#endif
#if (_SUPPORT_APP_USER_MODE != FALSE)
    EXIT_SECTION(); /*lint !e438 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
    HAL_PWM_MasterPendWait();                                                   /* Wait for End-of-PWM period */
#if defined (__MLX81160__)
#if (_SUPPORT_EVB == MLX81160EVB_3P_3N)
    DRVCFG_PWM_3P_3N();
    DRVCFG_ENA_3P_3N();                                                         /* Enable the driver and the PWM phase U, V and W */
#elif (_SUPPORT_DUAL_BLDC != FALSE)
    DRVCFG_PWM_RST_UVW();
    DRVCFG_ENA_RST_UVW();                                                       /* Enable the driver and the PWM phase R, S and T and U, V and W */
#elif (_SUPPORT_1ST_BLDC != FALSE)
    DRVCFG_PWM_RST();
    DRVCFG_ENA_RST();                                                           /* Enable the driver and the PWM phase R, S and T */
#else
    DRVCFG_PWM_UVW();
    DRVCFG_ENA_UVW();                                                           /* Enable the driver and the PWM phase U, V and W */
#endif
#elif defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__)
    DRVCFG_PWM_UVW();
    DRVCFG_ENA_UVW();                                                           /* Enable the driver and the PWM phase W, V and U */
#elif defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
    DRVCFG_PWM_UVW();
    DRVCFG_ENA();                                                               /* Enable the driver and the PWM phase W, V and U */
#endif
#else  /* (_SUPPORT_PWM_MODE != TRIPLEPHASE_FULL_STEP_BEMF) && (_SUPPORT_PWM_MODE != TRIPLEPHASE_FULL_STEP) */
#if defined (__MLX81160__)
#elif defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__)
    DRVCFG_ENA_UVW();                                                           /* Enable the driver and the PWM phase W, V and U */
#elif defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
    DRVCFG_ENA();                                                               /* Enable the driver and the PWM phase W, V and U */
#endif
#endif /* (_SUPPORT_PWM_MODE != TRIPLEPHASE_FULL_STEP_BEMF) && (_SUPPORT_PWM_MODE != TRIPLEPHASE_FULL_STEP) */
#else  /* (C_MOTOR_PHASES == 3) */
    MotorDriver_4Phase(l_u16MicroStepIdx);
#if (_SUPPORT_PWM_MODE != BIPOLAR_FULL_STEP_BEMF) && (_SUPPORT_PWM_MODE != BIPOLAR_HALF_STEP_BEMF)
    /* Switch Drive to active synchronous with PWM-period */
    HAL_PWM_MasterPendClear();                                                  /* Clear PWM-module Master1-End IRQ's */
    HAL_PWM_MasterPendWait();                                                   /* Wait for PWM Master PEND-flag */
#if defined (__MLX81160__)
    DRVCFG_PWM_RSTU();
    DRVCFG_ENA_RSTU();                                                          /* Enable the driver and the PWM phase W, V, U and T */
#else  /* defined (__MLX81160__) */
    DRVCFG_PWM_TUVW();
    DRVCFG_ENA_TUVW();                                                          /* Enable the driver and the PWM phase W, V, U and T */
#endif /* defined (__MLX81160__) */
#else  /* (_SUPPORT_PWM_MODE != BIPOLAR_FULL_STEP_BEMF) && (_SUPPORT_PWM_MODE != BIPOLAR_HALF_STEP_BEMF) */
#if defined (__MLX81160__)
    DRVCFG_ENA_RSTU();                                                          /* Enable the driver and the PWM phase W, V, U and T */
#else  /* defined (__MLX81160__) */
    DRVCFG_ENA_TUVW();                                                          /* Enable the driver and the PWM phase W, V, U and T */
#endif /* defined (__MLX81160__) */
#endif /* (_SUPPORT_PWM_MODE != BIPOLAR_FULL_STEP_BEMF) && (_SUPPORT_PWM_MODE != BIPOLAR_HALF_STEP_BEMF) */
#endif /* (C_MOTOR_PHASES == 3) */
#if (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE)
    l_u8MotorHoldingCurrState = FALSE;
    l_u16RampDownSteps = 0U;
#endif /* (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) */

    /* Setup ADC for Motor Current/Voltage measurements */
#if (_SUPPORT_PWM_MODE == BIPOLAR_FULL_STEP_BEMF) || (_SUPPORT_PWM_MODE == BIPOLAR_HALF_STEP_BEMF) || (_SUPPORT_PWM_MODE == TRIPLEPHASE_FULL_STEP_BEMF)
    ADC_Start(l_u16MicroStepIdx, TRUE);                                         /* BEMF-sensing requires ADC IRQ */
#else  /* (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM_BIPOLAR) || (_SUPPORT_PWM_MODE == BIPOLAR_HALF_STEP_BEMF) */
    ADC_Start(FALSE);                                                           /* Stepper-mode doesn't need ADC IRQ (CTimer based) */
#endif /* (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM_BIPOLAR) || (_SUPPORT_PWM_MODE == BIPOLAR_HALF_STEP_BEMF) */

    /* (Micro-) stepping */
    if (l_u16TargetCommutTimerPeriod < l_u16LowSpeedPeriod)
    {
        /* Target speed too fast for motor to start-up with */
        l_u16CommutTimerPeriod = l_u16LowSpeedPeriod;                           /* Initial start-up speed */
    }
    else
    {
        /* Target speed is slower than maximum motor start-up speed */
        l_u16CommutTimerPeriod = l_u16TargetCommutTimerPeriod;
    }
#if (_SUPPORT_SOFT_START != FALSE)
#if (UART_COMM != FALSE) && (_SUPPORT_UART_IF_APP == C_UART_IF_QUAD_WHEEL_CAR)
    IO_CTIMER0_TREGB = l_u16CommutTimerPeriod;
#else  /* (UART_COMM != FALSE) && (_SUPPORT_UART_IF_APP == C_UART_IF_QUAD_WHEEL_CAR) */
    IO_CTIMER0_TREGB = l_u16CommutTimerPeriod >> 1;
#endif /* (UART_COMM != FALSE) && (_SUPPORT_UART_IF_APP == C_UART_IF_QUAD_WHEEL_CAR) */
#else  /* (_SUPPORT_SOFT_START != FALSE) */
    IO_CTIMER0_TREGB = l_u16CommutTimerPeriod;
#if (_SUPPORT_HALL_LATCH != FALSE) && (_SUPPORT_NR_OF_HL > 1) && (_SUPPORT_HALL_LATCH_DIAG == FALSE)
    l_u16LastCommutTimerPeriod = l_u16CommutTimerPeriod;                        /* Store Commutation Timer period as it will be modified by Hall-Latch event */
#endif /* (_SUPPORT_HALL_LATCH != FALSE) && (_SUPPORT_NR_OF_HL > 1) && (_SUPPORT_HALL_LATCH_DIAG == FALSE) */
#endif /* (_SUPPORT_SOFT_START != FALSE) */
#if (_SUPPORT_HALL_LATCH != FALSE) && (_SUPPORT_HALL_LATCH_SMOOTHING != FALSE)
    g_i16HL_MicroStepCnt = 0;
#endif /* (_SUPPORT_HALL_LATCH != FALSE) && (_SUPPORT_HALL_LATCH_SMOOTHING != FALSE) */
    IO_CTIMER0_CTRL = B_CTIMER0_START;

#if (_SUPPORT_TRIAXIS_MLX9038x != FALSE) && (_DEBUG_MLX90381 != FALSE)
    l_u16ActualAngle = Triaxis_Angle();
    l_u16PrevAngle = l_u16ActualAngle;
    l_u16MinAngle = 0xFFFFU;
    l_u16MaxAngle = 0x0000U;
    l_u16AvgAngle = 0x0000U;
    l_u32SumAngle = 0x0000U;
    p_MemSet( (uint16_t *)&l_u16MicroStepAngle[0], 0U,
              (uint16_t)(C_MOTOR_PHASES * C_MOTOR_STEP_PER_PHASE * C_MICROSTEP_PER_FULLSTEP *
                         sizeof(l_u16MicroStepAngle[0])));
#endif /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) && (_DEBUG_MLX90381 != FALSE) */

    l_u16MotorDriverDisconDelay = 0U;
#if (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE)
    l_u8MotorHoldDelay = 0U;
#endif /* (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) */

#if (_SUPPORT_REWIND != FALSE)
    if ( (g_u8RewindFlags & (C_REWIND_ACTIVE | C_REWIND_REWIND)) == C_REWIND_ACTIVE)
    {
        g_u8RewindFlags &= ~C_REWIND_ACTIVE;                                    /* Rewind-function is finished */
    }
#endif /* (_SUPPORT_REWIND != FALSE) */

} /* End of MotorDriverStart */
#endif /* (_SUPPORT_FOC_MODE == FOC_MODE_NONE) */
#endif /* (_SUPPORT_PWM_MODE != SINGLE_COIL_PWM) && (_SUPPORT_PWM_MODE != SINGLE_COIL_PWM_BIPOLAR) */

#if (_SUPPORT_PWM_MODE != SINGLE_COIL_PWM)
/*!*************************************************************************** *
 * MotorDriverStop
 * \brief   Stop Motor Driver
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16Immediate: Stop-modus
 *              C_STOP_RAMPDOWN  : Ramp-down
 *              C_STOP_IMMEDIATE : Immediate stop (without ramp-down)
 *              C_STOP_EMERGENCY : Immediate stop (without ramp-down) + delay
 *              C_STOP_REVERSE   : Reverse some steps to release gear-stress
 * \return  -
 * *************************************************************************** *
 * \details Stop the actuator. In case stopping with ramp-down, the actuator
 *          will slow-down to avoid step-loss. A new target-position is set
 *          and the target-speed is set to minimum.
 *          Stopping without ramp-down, will immediate stop the actuator.
 * *************************************************************************** *
 * - Call Hierarchy: AppDegradedCheck(), DfrDiagAssignNAD(),
 *                   HandleDiagnosticsOC(), HandleDiagnosticsOT(),
 *                   HandleDiagnosticsUVOV(), HandleDiagnosticsVDS(),
 *                   ISR_CTIMER0_3(), main_Init(), main(),
 *                   mlu_ApplicationStop(), MotorDriverPeriodicTimer()
 * - Cyclomatic Complexity: 9+1
 * - Nesting: 5
 * - Function calling: 8 (DeltaPosition(),
 *                        HAL_ADC_StopSafe(), ConvMicroSteps2ShaftSteps(),
 *                        MotorDriver_InitialPwmDutyCycle(),
 *                        MotorDriver_4Phase(), Get_HoldingThreshold(),
 *                        ADC_Start())
 * *************************************************************************** */
void MotorDriverStop(uint16_t u16Immediate)
{
#if (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE)
    if ( (u16Immediate == (uint16_t)C_STOP_RAMPDOWN) &&
         ((g_e8MotorStatus & C_MOTOR_STATUS_STOP_MASK) != C_MOTOR_STATUS_STOP) ) /*lint !e845 */  /* MMP190123-2 */
    {
        /* Request to ramp-down */
        ENTER_SECTION(ATOMIC_KEEP_MODE);  /*lint !e534 */
        {
            if ( ((g_e8MotorStatus & C_MOTOR_STATUS_STOP_MASK) != C_MOTOR_STATUS_STOP) &&
                 ((g_e8MotorStatus & C_MOTOR_STATUS_MASK) != (uint8_t)C_MOTOR_STATUS_STOPPING) )
            {
                /* Not stopping; Request to stop by setting Target-position near to Actual-position and slow-down */
                if (l_u16DeltaPosition > l_u16RampDownSteps)
                {
                    if (g_e8MotorDirectionCCW != ((uint8_t)C_MOTOR_DIR_CW) )
                    {
                        /* Closing */
                        p_SubU32_U32byU16(&g_u32TargetPosition,
                                          (const uint32_t*)&l_u32ActualPosition,
                                          l_u16RampDownSteps);
                    }
                    else
                    {
                        /* Opening */
                        p_AddU32_U32byU16(&g_u32TargetPosition,
                                          (const uint32_t*)&l_u32ActualPosition,
                                          l_u16RampDownSteps);
                    }
                }
            }
            else if ( (g_e8MotorStatus & C_MOTOR_STATUS_STOP_MASK) != C_MOTOR_STATUS_STOP)
            {
                u16Immediate = (uint16_t)C_STOP_IMMEDIATE;
            }
        }
        EXIT_SECTION(); /*lint !e438 */
    }
    else
#endif /* (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) */
    {
        /* First stop ADC, before stopping motor (trigger-event) */
#if (_DEBUG_MOTOR_STOP != FALSE)
        DEBUG_SET_IO_B();
#endif /* (_DEBUG_MOTOR_STOP != FALSE) */

#if (_SUPPORT_REWIND != FALSE)
        if ( (g_u8StallOcc != FALSE) && ((g_u8RewindFlags & (uint8_t)C_REWIND_ACTIVE) == 0U) )
        {
            /* Real stall; Activate re-wind for future movement (in case in same direction); Memorise STALL + Direction */
            if (g_e8MotorDirectionCCW != FALSE)
            {
                g_u8RewindFlags = (g_u8RewindFlags | (uint8_t)C_REWIND_DIRECTION_CCW) | (uint8_t)C_REWIND_STALL_DETECT;
            }
            else
            {
                g_u8RewindFlags = (g_u8RewindFlags & (uint8_t) ~C_REWIND_DIRECTION_CCW) |
                                  (uint8_t)C_REWIND_STALL_DETECT;
            }
        }
        else
        {
            /* Ignore stall during rewind-action */
        }
#endif /* (_SUPPORT_REWIND != FALSE) */

        HAL_ADC_StopSafe();
#if (_SUPPORT_STALLDET_BZC != FALSE)
        /* [BEMF] */
        if ( (l_e8MotorStartupMode & E_MSM_BEMF) != 0U)
        {
            /* Before changing Startup-mode, convert full-step to micro-step */
            l_u16MicroStepIdx =
                (uint16_t)p_MulU32_U16byU16(l_u16MicroStepIdx,
                                            g_u16NrOfMicroStepsPerFullStep) + (g_u16NrOfMicroStepsPerFullStep >> 1);
        }
#endif /* (_SUPPORT_STALLDET_BZC != FALSE) */
        l_e8MotorStartupMode = E_MSM_STOP;                                      /* Stop mode */
        /* Make target-position same as actual position */
#if (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE)
        p_CpyU32_U32( (const uint32_t *)&l_u32ActualPosition, &g_u32TargetPosition);  /* Stop: Target = Actual */
        ConvMicroSteps2ShaftSteps();
        l_u16DeltaPosition = 0U;
        l_u16RampdownTimeout = 0U;
        g_u8MotorStatusSpeed = (uint8_t)C_STATUS_SPEED_STOP;
#endif /* (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) */
        g_u16ActualMotorSpeedRPM = 0U;
#if (_SUPPORT_FOC_MODE != FOC_MODE_NONE) && (C_MOTOR_PHASES != 1)               /* FOC for Positioning Device */
#if (_SUPPORT_PID_U32 == FALSE)
        g_u16ActualMotorSpeedRPMe = 0U;
#else  /* (_SUPPORT_PID_U32 == FALSE) */
        g_u32ActualMotorSpeedRPMe = 0UL;
#endif /* (_SUPPORT_PID_U32 == FALSE) */
#else  /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) && (C_MOTOR_PHASES != 1) */
#if (_SUPPORT_HALL_LATCH == FALSE)
        g_u16ForcedSpeedRPM = 0U;
#endif /* (_SUPPORT_HALL_LATCH == FALSE) */
#endif /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) && (C_MOTOR_PHASES != 1) */

#if (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE)
        if ( (g_u8MotorHoldingCurrEna != FALSE) &&                              /* Holding mode enabled */
             ((g_e8ErrorElectric & (uint8_t)C_ERR_PERMANENT) == 0U) &&
             (g_e8ErrorVoltage == (uint8_t)C_ERR_VOLTAGE_IN_RANGE) &&
             (g_e8ErrorOverTemperature == (uint8_t)C_ERR_OTEMP_NO) &&
             (u16Immediate != (uint16_t)C_STOP_WO_HOLDING) )   /*lint !e845 */
        {
            /* Keep Motor driver active with a specified amount of current (unless permanent electric error) */
            if (g_e8MotorStatus == C_MOTOR_STATUS_STOP)                         /* MMP231121-1: Skip in case motor driver is active */
            {
#if (_SUPPORT_NV_NON_BLOCK_WRITE != FALSE)
                MotorDriver_InitialPwmDutyCycle(g_u16HoldingCurrentLevel_NV, 0U);
#else  /* (_SUPPORT_NV_NON_BLOCK_WRITE != FALSE) */
                MotorDriver_InitialPwmDutyCycle(NV_HOLDING_CURR_LEVEL, 0U);
#endif /* (_SUPPORT_NV_NON_BLOCK_WRITE != FALSE) */
                MotorDriverConfig(TRUE);
                l_u16MotorCurrentLPF = Get_HoldingThreshold();                  /* Low-pass Filtered motor-current [ADC-LSB] */
            }
            else
            {
                /* Reduce Motor PWM Duty cycle by 50% (MMP240709-1) */
                MotorDriver_InitialPwmDutyCycle(Get_ActCurrRunMax_mA() / 2U, 0U);
                l_u16MotorCurrentLPF >>= 1;                                     /* 50% of the Low-pass Filtered motor-current [ADC-LSB] */
            }
            l_u32MotorCurrentLPF = ((uint32_t)l_u16MotorCurrentLPF << 16);

#if (C_MOTOR_PHASES == 3)
            MotorDriver_3Phase(l_u16MicroStepIdx);
#if defined (__MLX81160__)
#if (_SUPPORT_EVB == MLX81160EVB_3P_3N)
            DRVCFG_PWM_3P_3N();
            DRVCFG_ENA_3P_3N();                                                 /* Enable the driver and the PWM phase U, V and W */
#elif (_SUPPORT_DUAL_BLDC != FALSE)
            DRVCFG_PWM_RST_UVW();
            DRVCFG_ENA_RST_UVW();                                               /* Enable the driver and the PWM phase R, S and T and U, V and W*/
#elif (_SUPPORT_1ST_BLDC != FALSE)
            DRVCFG_PWM_RST();
            DRVCFG_ENA_RST();                                                   /* Enable the driver and the PWM phase R, S and T */
#else
            DRVCFG_PWM_UVW();
            DRVCFG_ENA_UVW();                                                   /* Enable the driver and the PWM phase U, V and W */
#endif
#else  /* defined (__MLX81160__) */
            DRVCFG_PWM_UVW();
#if defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
            DRVCFG_ENA();
#else
            DRVCFG_ENA_UVW();                                                   /* Enable the driver and the PWM phase W, V and U */
#endif
#endif /* defined (__MLX81160__) */
#else  /* (C_MOTOR_PHASES == 3) */
            MotorDriver_4Phase(l_u16MicroStepIdx);
#if defined (__MLX81160__)
            DRVCFG_PWM_RSTU();
            DRVCFG_ENA_RSTU();                                                  /* Enable the driver and the PWM phase W, V and U */
#else  /* defined (__MLX81160__) */
            DRVCFG_PWM_TUVW();
            DRVCFG_ENA_TUVW();                                                  /* Enable the driver and the PWM phase W, V and U */
#endif /* defined (__MLX81160__) */
#endif /* (C_MOTOR_PHASES == 3) */
            l_u8MotorHoldingCurrState = TRUE;
            g_e8MotorStatus = C_MOTOR_STATUS_HOLD;
#if (_SUPPORT_PWM_MODE == BIPOLAR_FULL_STEP_BEMF) || (_SUPPORT_PWM_MODE == BIPOLAR_HALF_STEP_BEMF) || (_SUPPORT_PWM_MODE == TRIPLEPHASE_FULL_STEP_BEMF)
            ADC_Start(l_u16MicroStepIdx, FALSE);                                /* Holding mode doesn't require ADC-IRQ */
#else  /* (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM_BIPOLAR) */
            ADC_Start(FALSE);                                                   /* Holding mode doesn't require ADC-IRQ */
#endif /* (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM_BIPOLAR) */
        }
        else
        {
            /* Disconnect drivers */
            if (g_e8ErrorElectric == (uint8_t)C_ERR_NONE)
            {
                /* With motor-stop short-time hold */
                l_u8MotorHoldDelay = PI_TICKS_PER_MILLISECOND;                  /*   1ms delay before driver is Grounded */
                /*l_u8MotorHoldDelay = C_PI_TICKS_50MS;*/                       /*  50ms delay before driver is Grounded */
                l_u16MotorDriverDisconDelay = C_PI_TICKS_100MS;                 /* 100ms delay before driver is disconnected */
            }
            else
            {
                /* In case of a permanent error, don't connect drivers anymore */
                MotorDriverConfig(FALSE);
            }
            l_u8MotorHoldingCurrState = FALSE;
            l_u16MotorCurrentMovAvgxN = 0U;
            l_u16MotorCurrentLPF = 0U;                                          /* Low-pass Filtered motor-current */
            l_u32MotorCurrentLPF = 0U;
            if (g_e8MotorStatus != C_MOTOR_STATUS_STOP)
            {
                g_u16MotorStartDelay = C_PI_TICKS_MOTOR_RESTART;                /* Wait 250ms before continue */
                g_e8MotorStatus = C_MOTOR_STATUS_STOP;
            }
        }
#else  /* (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) */
#if (UART_COMM != FALSE) && (_SUPPORT_UART_IF_APP == C_UART_IF_QUAD_WHEEL_CAR)
        if (g_e8ErrorElectric == (uint8_t)C_ERR_NONE)
        {
            /* Break the rotating Wheels for 500ms, by putting phases to GND */
            l_u16MotorDriverDisconDelay = C_PI_TICKS_500MS;                     /* 500ms delay before driver is disconnected */
            DRVCFG_GND_UVW();                                                   /* Make Low-side active, for a short time (recycle current) */
        }
        else
#endif /* (UART_COMM != FALSE) && (_SUPPORT_UART_IF_APP == C_UART_IF_QUAD_WHEEL_CAR) */
#if (_SUPPORT_FAST_STOP != FALSE)
        if (u16Immediate == (uint16_t)C_STOP_FAST_STOP)                         /* MMP231121-1 */
        {
            /* (Keep) drivers connected */
            ADC_Start(FALSE);                                                   /* Active stopping mode doesn't require ADC-IRQ */
            MotorDriverConfig(TRUE);
#if (C_MOTOR_PHASES == 3)
            DRVCFG_TRI_UVW();
#else  /* (C_MOTOR_PHASES == 3) */
#if defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
            DRVCFG_TRI_UVW();
#else  /* defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__) */
            DRVCFG_TRI_TUVW();
#endif /* defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__) */
#endif /* (C_MOTOR_PHASES == 3) */
            DRVCFG_ENA();
            l_u16MotorDriverDisconDelay = C_MOTOR_START_DELAY_1SEC;             /* Ground driver after 1 seconds */
            g_e8MotorStatus = C_MOTOR_STATUS_FAST_STOP;
            IO_CTIMER0_CTRL = B_CTIMER0_STOP;                                   /* Stop "commutation timer" */
            IO_CTIMER0_TREGB = (3U * C_MIN_PWM_PERIOD);                         /* Three PWM Periods */
            IO_CTIMER0_CTRL = B_CTIMER0_START;                                  /* Start "commutation timer" */
        }
        else
#endif /* (_SUPPORT_FAST_STOP != FALSE) */
        {
            /* Disconnect drivers */
#if (C_MOTOR_PHASES == 3)
#if defined (__MLX81160__)
#if (_SUPPORT_DUAL_BLDC != FALSE)
            DRVCFG_TRI_RST_UVW();
#elif (_SUPPORT_1ST_BLDC != FALSE)
            DRVCFG_TRI_RST();
#else
            DRVCFG_TRI_UVW();
#endif
#else  /* defined (__MLX81160__) */
            DRVCFG_TRI_UVW();
#if defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
            DRVCFG_DIS();
#endif /* defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__) */
#endif /* defined (__MLX81160__) */
#else  /* (C_MOTOR_PHASES == 3) */
#if defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
            DRVCFG_TRI_UVW();
            DRVCFG_DIS();
#else  /* defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__) */
            DRVCFG_TRI_TUVW();
#endif /* defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__) */
#endif /* (C_MOTOR_PHASES == 3) */
            l_u16MotorDriverDisconDelay = C_MOTOR_START_DELAY_10SEC;            /* Disconnect driver after 10 seconds */
        }
        l_u16MotorCurrentMovAvgxN = 0U;
        l_u16MotorCurrentLPF = 0U;                                              /* Low-pass Filtered motor-current */
        l_u32MotorCurrentLPF = 0U;
        if ( (g_e8MotorStatus != C_MOTOR_STATUS_STOP)
#if (_SUPPORT_FAST_STOP != FALSE)
            && (g_e8MotorStatus != C_MOTOR_STATUS_FAST_STOP)
#endif /* (_SUPPORT_FAST_STOP != FALSE) */
           )
        {
            if (l_u16TargetCommutTimerPeriod == l_u16LowSpeedPeriod)
            {
                /* Decelerated to Low-speed */
                g_u16MotorStartDelay = C_PI_TICKS_20MS;                         /* Wait 20ms */
            }
            else
            {
                g_u16MotorStartDelay = C_PI_TICKS_MOTOR_RESTART;                /* Wait 250ms before continue */
            }
            g_e8MotorStatus = C_MOTOR_STATUS_STOP;
        }
#endif /* (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) */
#if (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) && (_SUPPORT_STALL_REVERSE != FALSE)
        if ( (u16Immediate == C_STOP_REVERSE) && (l_u16MotorRewindSteps != 0U) )
        {
            if (g_e8MotorDirectionCCW != FALSE)
            {
                if (g_u16ActualPosition <= (uint16_t)(C_MAX_POS - l_u16MotorRewindSteps) )
                {
                    g_u16TargetPosition = g_u16ActualPosition + l_u16MotorRewindSteps;
                    g_u8RewindFlags = (uint8_t)C_REWIND_ACTIVE | (uint8_t)C_REWIND_STALL_DETECT;
                }
                else
                {
                    g_u8RewindFlags = 0U;                                       /* No rewind possible */
                }
            }
            else
            {
                if (g_u16ActualPosition >= l_u16MotorRewindSteps)
                {
                    g_u16TargetPosition = g_u16ActualPosition - l_u16MotorRewindSteps;
                    g_u8RewindFlags = (uint8_t)C_REWIND_ACTIVE | (uint8_t)C_REWIND_STALL_DETECT;
                }
                else
                {
                    g_u8RewindFlags = 0U;                                       /* No rewind possible */
                }
            }
            if ( (g_u8RewindFlags & (uint8_t)C_REWIND_ACTIVE) != 0U)
            {
                g_u16MotorStartDelay = 0U;
                g_u16TargetMotorSpeedRPM = g_u16LowSpeedRPM;
                g_e8MotorRequest = (uint8_t)C_MOTOR_REQUEST_START;
            }
        }
        else
        {
            g_u8RewindFlags = 0U;
        }
#endif /* (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) && (_SUPPORT_STALL_REVERSE != FALSE) */

#if (_SUPPORT_FAST_STOP != FALSE)
        if (u16Immediate != (uint16_t)C_STOP_FAST_STOP)                         /* MMP231121-1 */
#endif /* (_SUPPORT_FAST_STOP != FALSE) */
        {
            IO_CTIMER0_CTRL = B_CTIMER0_STOP;                                   /* Stop "commutation timer" */
#if (_SUPPORT_APP_USER_MODE != FALSE)
            ENTER_SECTION(SYSTEM_MODE); /*lint !e534 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
            IO_MLX16_ITC_PEND1_S = B_MLX16_ITC_PEND1_CTIMER0_3;                 /* Clear (potentially pending) Timer0 interrupts (T0_INT3) */
#if (_SUPPORT_CDI != FALSE)
            IO_ACTIVE_CDI = C_IO_ACTIVE_CDI_ACTIVE_CDI_PHASE_DIS;               /* Disable CDI */
            IO_MLX16_ITC_PEND5_S = B_MLX16_ITC_PEND5_ACTIVE_CDI;                /* Clear pending CDI interrupts */
            IO_MLX16_ITC_MASK5_S &= ~B_MLX16_ITC_MASK5_ACTIVE_CDI;              /* Disable CDI IRQ */
#endif /* (_SUPPORT_CDI != FALSE) */
#if (_SUPPORT_APP_USER_MODE != FALSE)
            EXIT_SECTION(); /*lint !e438 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
        }
#if (_DEBUG_MOTOR_STOP != FALSE)
        DEBUG_CLR_IO_B();
#endif /* (_DEBUG_MOTOR_STOP != FALSE) */

#if (_SUPPORT_REWIND != FALSE)
        /* Re-stall code */
        if ( (g_u8RewindFlags & (uint8_t)(C_REWIND_ACTIVE | C_REWIND_REWIND)) ==
             (uint8_t)(C_REWIND_ACTIVE | C_REWIND_REWIND) )
        {
            /* Rewind activated; First part of the rewind-function has been done (the rewinding) */
            g_u8RewindFlags &= (uint8_t) ~C_REWIND_REWIND;                     /* Rewinding done */
            g_u16TargetPosition = l_u16TargetPositionRewind;                   /* Restore original targeted position; Note: During rewind no new position is accepted */
            l_u16MotorDriverDisconDelay = 0U;                                  /* Cancel stop delay */
            g_u16MotorStartDelay = C_PI_TICKS_STABILISE;                       /* Allow actuator to shortly settle, before the wanted movement starts */
            g_e8MotorRequest = (uint8_t)C_MOTOR_REQUEST_START;
        }
        else if ( (g_u8RewindFlags & (uint8_t)C_REWIND_STALL_DETECT) == 0U)
        {
            g_u8RewindFlags = 0U;                                              /* Clear all other flags in case no STALL have been detected */
        }
        else
        {
            /* Nothing */
        }
#endif /* (_SUPPORT_REWIND != FALSE) */

#if (LIN_COMM != FALSE) && (_SUPPORT_BUSTIMEOUT_SLEEP != FALSE) && (_SUPPORT_LIN_SLEEP != FALSE)
#if (_SUPPORT_NV_NON_BLOCK_WRITE != FALSE)
        if ( (g_u8BusTimeOutSleep_NV != 0U) &&
#else  /* (_SUPPORT_NV_NON_BLOCK_WRITE != FALSE) */
        if ( (NV_BUSTIMEOUT_SLEEP != 0U) &&
#endif /* (_SUPPORT_NV_NON_BLOCK_WRITE != FALSE) */
             (g_e8EmergencyRunOcc != (uint8_t)C_SAFETY_RUN_NO) &&
             (g_e8MotorRequest == (uint8_t)C_MOTOR_REQUEST_NONE) )
        {
            g_e8MotorRequest = C_MOTOR_REQUEST_SLEEP;
        }
#endif /* (LIN_COMM != FALSE) && (_SUPPORT_BUSTIMEOUT_SLEEP != FALSE) && (_SUPPORT_LIN_SLEEP != FALSE) */

    }
    (void)u16Immediate;
} /* End of MotorDriverStop */
#endif /* (_SUPPORT_PWM_MODE != SINGLE_COIL_PWM) */

/*!*************************************************************************** *
 * MotorDriverPeriodicTimer
 * \brief   Periodic Motor Driver updater
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16Period: Period in CoreTimer units
 * \return  -
 * *************************************************************************** *
 * \details -
 * *************************************************************************** *
 * - Call Hierarchy: TIMER_IT
 * - Cyclomatic Complexity: 6+1
 * - Nesting: 3
 * - Function calling: 2 (MotorDriverStop(), MotorDriverConfig())
 * *************************************************************************** */
void MotorDriverPeriodicTimer(uint16_t u16Period)
{
#if (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) && (_SUPPORT_PWM_MODE != SINGLE_COIL_PWM)
    if (l_u8MotorHoldDelay != 0U)
    {
        if (l_u8MotorHoldDelay > u16Period)
        {
            l_u8MotorHoldDelay -= u16Period;
        }
        else
        {
#if (_SUPPORT_VOLTAGE_SHAPE == VOLTAGE_SHAPE_SINE) || (_SUPPORT_VOLTAGE_SHAPE == VOLTAGE_SHAPE_SVM) || \
            (_SUPPORT_VOLTAGE_SHAPE == VOLTAGE_SHAPE_DSVM_GND) || (_SUPPORT_VOLTAGE_SHAPE == VOLTAGE_SHAPE_NONE)
#if (C_MOTOR_PHASES == 3)
#if defined (__MLX81160__)
#if (_SUPPORT_EVB == MLX81160EVB_3P_3N)
            DRVCFG_GND_3P_3N();
#elif (_SUPPORT_DUAL_BLDC != FALSE)
            DRVCFG_GND_RST_UVW();                                               /* Make Low-side active, for a short time (recycle current) */
#elif (_SUPPORT_1ST_BLDC != FALSE)
            DRVCFG_GND_RST();                                                   /* Make Low-side active, for a short time (recycle current) */
#else
            DRVCFG_GND_UVW();                                                   /* Make Low-side active, for a short time (recycle current) */
#endif
#else  /* defined (__MLX81160__) */
            DRVCFG_GND_UVW();                                                   /* Make Low-side active, for a short time (recycle current) */
#endif /* defined (__MLX81160__) */
#else  /* (C_MOTOR_PHASES == 3) */
#if defined (__MLX81160__)
            DRVCFG_GND_RSTU();                                                  /* Make Low-side active, for a short time (recycle current) */
#else  /* defined (__MLX81160__) */
            DRVCFG_GND_TUVW();                                                  /* Make Low-side active, for a short time (recycle current) */
#endif /* defined (__MLX81160__) */
#endif /* (C_MOTOR_PHASES == 3) */
#else  /* (_SUPPORT_VOLTAGE_SHAPE == VOLTAGE_SHAPE_DSVM_GND) */
#if (C_MOTOR_PHASES == 3)
#if defined (__MLX81160__)
#if (_SUPPORT_DUAL_BLDC != FALSE)
            DRVCFG_VSUP_RST_UVW();
#elif (_SUPPORT_1ST_BLDC != FALSE)
            DRVCFG_SUP_RST();
#else
            DRVCFG_SUP_UVW();
#endif
#else  /* defined (__MLX81160__) */
            DRVCFG_SUP_UVW();
#endif /* defined (__MLX81160__) */
#else  /* (C_MOTOR_PHASES == 3) */
#if defined (__MLX81160__)
            DRVCFG_SUP_RSTU();
#else  /* defined (__MLX81160__) */
            DRVCFG_SUP_TUVW();
#endif /* defined (__MLX81160__) */
#endif /* (C_MOTOR_PHASES == 3) */
#endif /* (_SUPPORT_VOLTAGE_SHAPE == VOLTAGE_SHAPE_DSVM_GND) */
            l_u8MotorHoldDelay = 0U;
            l_u8MotorHoldingCurrState = FALSE;
        }
    }
    else
#endif /* (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) && (_SUPPORT_PWM_MODE != SINGLE_COIL_PWM) */
    if (l_u16MotorDriverDisconDelay != 0U)
    {
        if (l_u16MotorDriverDisconDelay > u16Period)
        {
            l_u16MotorDriverDisconDelay -= u16Period;
        }
        else
        {
            l_u16MotorDriverDisconDelay = 0U;
#if (_SUPPORT_FAST_STOP != FALSE)
            if (g_e8MotorStatus == C_MOTOR_STATUS_FAST_STOP)
            {
                DRVCFG_GND_UVW();                                               /* Ground drivers */
            }
            else
#endif /* (_SUPPORT_FAST_STOP != FALSE) */
            {
                MotorDriverConfig(FALSE);                                       /* Disable the driver and the PWM phase W, V and U */
            }
            g_e8MotorStatus = C_MOTOR_STATUS_STOP;
        }
    }
    else
    {
        /* Nothing */
    }

#if (_SUPPORT_COIL_TEMP_COMP != FALSE)
    /* Temperature compensated Phase & Motor-Coil resistance */
    {
        int16_t i16deltaTemperature = (Get_ChipTemperature() - 25);             /* Reference temperature: 25C */
        /* Copper temperature coefficient: 0.00393 = ~1/256 */
        int16_t i16deltaResistance =
            (int16_t)(p_MulI32_I16byI16( (int16_t)l_u16PhaseCoilResistanceRT, i16deltaTemperature) >> 8);
        l_u16PhaseCoilResistanceAT = l_u16PhaseCoilResistanceRT + i16deltaResistance;
    }
#endif /* (_SUPPORT_COIL_TEMP_COMP != FALSE) */

#if (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) && (_SUPPORT_PWM_MODE != SINGLE_COIL_PWM)
    ENTER_SECTION(ATOMIC_KEEP_MODE);  /*lint !e534 */
    {
        if (l_u16RampdownTimeout != 0U)                                         /* MMP170806-1 */
        {
            if (l_u16RampdownTimeout > u16Period)
            {
                l_u16RampdownTimeout -= u16Period;
            }
            else
            {
                l_u16RampdownTimeout = 0U;
                MotorDriverStop( (uint16_t)C_STOP_IMMEDIATE);
            }
        }
    }
    EXIT_SECTION(); /*lint !e438 */
#endif /* (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) && (_SUPPORT_PWM_MODE != SINGLE_COIL_PWM) */
} /* End of MotorDriverPeriodicTimer() */

#if (C_MOTOR_PHASES != 1) && (_SUPPORT_FOC_MODE == FOC_MODE_NONE)
/*!*************************************************************************** *
 * ISR_CTIMER0_3
 * \brief   MotorDriver Commutation Interrupt
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details Perform next motor step. If motor speed not reached, accelerate.
 *          IRQ-Priority: 4
 * *************************************************************************** *
 * - Call Hierarchy: IRQ
 * - Cyclomatic Complexity: 14+1
 * - Nesting: 5
 * - Function calling: 7 (DeltaPosition(), MotorDriverStop(),
 *                        MotorDriverCurrentMeasure(),
 *                        MotorDriverUpdateMicroStepIndex(),
 *                        VoltageCorrection(), MotorDriver_4Phase(),
 *                        MotorStallCheckA())
 * ************************************************************************** */
__attribute__((interrupt)) void ISR_CTIMER0_3(void)
{
#if (_DEBUG_CPU_LOAD != FALSE)
    DEBUG_SET_IO_B();                                                           /* IRQ-Priority: 4 */
#endif /* (_DEBUG_CPU_LOAD != FALSE) */
#if (_DEBUG_COMMUT_ISR != FALSE)
    DEBUG_SET_IO_B();
#endif /* (_DEBUG_COMMUT_ISR != FALSE) */

#if (_DEBUG_MCUR_SUB_SAMPLED != FALSE)
    ++g_u16SubSamplingIdx;
#endif /* (_DEBUG_MCUR_SUB_SAMPLED != FALSE) */

    do
    {
        if ( (g_e8MotorStatus & C_MOTOR_STATUS_STOP_MASK) == C_MOTOR_STATUS_STOP)
        {
            /* Resume */
            break;
        }
#if (_SUPPORT_SOFT_START != FALSE)
        /* Soft Start-up */
        if (g_e8MotorStatus == (uint8_t)C_MOTOR_STATUS_SOFT_START)
        {
            /* Soft Alignment */
            if (l_u16RampStep == 0U)
            {
                /* Soft Alignment is done; Switch to Running */
                IO_CTIMER0_TREGB = l_u16CommutTimerPeriod;
                g_e8MotorStatus = (uint8_t)C_MOTOR_STATUS_RUNNING;
            }
            else
            {
                l_u16RampStep--;
                l_u16CorrectionRatio = l_u16StartCorrectionRatio - p_MulDivU16_U16byU16byU16(l_u16StartCorrectionRatio,
                                                                                             l_u16RampStep,
                                                                                             C_SOFT_START_RAMP_STEPS);
#if (UART_COMM != FALSE) && (_SUPPORT_UART_IF_APP == C_UART_IF_QUAD_WHEEL_CAR)
                l_u16MicroStepIdx = MotorDriverUpdateMicroStepIndex(l_u16MicroStepIdx);  /* MMP221024-1 */
#endif /* (UART_COMM != FALSE) && (_SUPPORT_UART_IF_APP == C_UART_IF_QUAD_WHEEL_CAR) */
#if (C_MOTOR_PHASES == 3)
                MotorDriver_3Phase(l_u16MicroStepIdx);
#else  /* (C_MOTOR_PHASES == 3) */
                MotorDriver_4Phase(l_u16MicroStepIdx);
#endif /* (C_MOTOR_PHASES == 3) */
                break;
            }
        }
#endif /* (_SUPPORT_SOFT_START != FALSE) */

#if (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
        /* Triaxis MLX9038x calibration */
        if (g_e8MotorRequest == (uint8_t)C_MOTOR_REQUEST_CALIB_FACTORY)
        {
            Triaxis_Calibrate(C_TRIAXIS_CALIB_STORE);
        }
#if (_DEBUG_MLX90381 != FALSE)
        else
        {
            /* Test purpose */
            l_u16ActualAngle = Triaxis_Angle();
            if (g_e8MotorDirectionCCW != FALSE)
            {
                /* Counter Clock Wise */
                l_u16CommutAngle = l_u16ActualAngle - l_u16PrevAngle;
            }
            else
            {
                /* Clock Wise */
                l_u16CommutAngle = l_u16PrevAngle - l_u16ActualAngle;
            }
            l_u16PrevAngle = l_u16ActualAngle;
            if (l_u16MinAngle > l_u16CommutAngle)
            {
                l_u16MinAngle = l_u16CommutAngle;
            }
            if (l_u16MaxAngle < l_u16CommutAngle)
            {
                l_u16MaxAngle = l_u16CommutAngle;
            }
            l_u32SumAngle -= l_u16MicroStepAngle[l_u16MicroStepIdx];
            l_u16MicroStepAngle[l_u16MicroStepIdx] = l_u16CommutAngle;
            l_u32SumAngle += l_u16CommutAngle;
            l_u16AvgAngle =
                p_DivU16_U32byU16(l_u32SumAngle, (C_MOTOR_PHASES * C_MOTOR_STEP_PER_PHASE * C_MICROSTEP_PER_FULLSTEP));
            if (l_e8MotorStartupMode == E_MSM_STEPPER_C)
            {
                if ( (l_u16CommutAngle < (l_u16AvgAngle >> 2)) || (l_u16CommutAngle > (l_u16AvgAngle << 2)) )
                {
                    g_e8ErrorElectric = (uint8_t)C_ERR_SENSOR;
#if (_SUPPORT_LOG_ERRORS != FALSE)
                    SetLastError(C_ERR_TRIAXIS_CALIB | C_ERR_EXT | 0x0F00U);
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
                }
            }
#if (_DEBUG_MOTOR_CURRENT_FLT != FALSE) && (_DEBUG_MLX90381 != FALSE)
            uint8_t *pBfr = &g_au8DebugBuf[g_u16DebugBufWrIdx];
            pBfr[0] = (uint8_t)((l_u16CommutAngle + (1U << 5)) >> 6);
/*            pBfr[1] = (uint8_t) (Get_ResolverPosX() >> 2); */
            pBfr[1] = (uint8_t)((Get_ResolverPosX() - g_u16ResolverOffX) >> 2) + 128;
/*            pBfr[2] = (uint8_t) (Get_ResolverPosY() >> 2); */
            pBfr[2] =
                (uint8_t)((p_MulI32_I16byI16( (int16_t)(Get_ResolverPosY() - g_u16ResolverOffY),
                                              (int16_t)g_u16ResolverAmplXY) >> 10) >> 2) + 128;
            g_u16DebugBufWrIdx += 3U;
            if (g_u16DebugBufWrIdx >= C_DEBUG_BUF_SZ)
            {
                g_u16DebugBufWrIdx = 0U;
            }
#endif /* (_DEBUG_MOTOR_CURRENT_FLT != FALSE) && (_DEBUG_MLX90381 != FALSE) */
        }
#endif /* (_DEBUG_MLX90381 != FALSE) */
#endif /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */

#if (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE)
        /* Actual-position update */
#if (_SUPPORT_STALLDET_BZC != FALSE)
        /* [BEMF] */
        if (g_e8MotorDirectionCCW != FALSE)
        {
            /* Closing */
            if ( (l_e8MotorStartupMode & E_MSM_BEMF) != 0U)
            {
                p_SubU32byU16( (uint32_t *)&l_u32ActualPosition, g_u16NrOfMicroStepsPerFullStep);
            }
            else
            {
                p_SubU32byU16( (uint32_t *)&l_u32ActualPosition, 1U);
            }
        }
        else
        {
            /* Opening */
            if ( (l_e8MotorStartupMode & E_MSM_BEMF) != 0U)
            {
                p_AddU32byU16( (uint32_t *)&l_u32ActualPosition, g_u16NrOfMicroStepsPerFullStep);
            }
            else
            {
                p_AddU32byU16( (uint32_t *)&l_u32ActualPosition, 1U);
            }
        }
#elif (_SUPPORT_HALL_LATCH == FALSE) || (_SUPPORT_NR_OF_HL == 1U) || (_SUPPORT_HALL_LATCH_DIAG != FALSE)
        if (g_e8MotorDirectionCCW != FALSE)
        {
            l_u32ActualPosition--;                                              /* Closing */
        }
        else
        {
            l_u32ActualPosition++;                                              /* Opening */
        }
#else  /* (_SUPPORT_HALL_LATCH == FALSE) || (_SUPPORT_NR_OF_HL == 1U) || (_SUPPORT_HALL_LATCH_DIAG != FALSE) */
       /* Sensor (micro-)stepping */
#if (_SUPPORT_MICRO_STEP_COMMUTATION != FALSE)
        g_u8HallMicroSteps++;                                                   /* Number of micro-steps since last Hall-Latch ISR */
#endif /* (_SUPPORT_MICRO_STEP_COMMUTATION != FALSE) */
#endif /* (_SUPPORT_STALLDET_BZC != FALSE) */

        l_u16DeltaPosition = DeltaPosition();
#if (_SUPPORT_STALLDET_BZC != FALSE)
        /* [BEMF] */
        if ( (l_e8MotorStartupMode & E_MSM_BEMF) != 0U)
        {
            /* BEMF-mode; If actual-position is within micro-steps per full-steps
             * range of target-position, then stop. */
            if (l_u16DeltaPosition < g_u16NrOfMicroStepsPerFullStep)
            {
                g_u32TargetPosition = l_u32ActualPosition;
                l_u16DeltaPosition = 0U;
            }
        }
#elif (_SUPPORT_HALL_LATCH != FALSE) && (_SUPPORT_NR_OF_HL > 1U) && (_SUPPORT_MICRO_STEP_COMMUTATION != FALSE)
        if (l_u16DeltaPosition <= g_u8HallMicroSteps)
        {
            l_u16DeltaPosition = 0U;
        }
#endif /* _SUPPORT_STALLDET_BZC */
        if (l_u16DeltaPosition == 0U)
        {
            MotorDriverStop( (uint16_t)C_STOP_IMMEDIATE);                       /* CPOS = FPOS */
            break;
        }

        if ( (l_u16DeltaPosition < l_u16RampDownSteps) && (l_e8MotorStartupMode != E_MSM_STEPPER_D) )
        {
            /* Decelerate motor speed (almost at target-position) */
            l_u16TargetCommutTimerPeriod = l_u16LowSpeedPeriod;
#if (_SUPPORT_STALLDET_BZC != FALSE)
            /* [BEMF] */
            if ( (l_e8MotorStartupMode & E_MSM_BEMF) != 0U)
            {
                /* BEMF-mode; TargetCommutTimerPeriod is a full-step period */
                l_u16TargetCommutTimerPeriod *= g_u16NrOfMicroStepsPerFullStep;
                l_e8MotorStartupMode = E_MSM_BEMF_D;
            }
            else
#endif /* (_SUPPORT_STALLDET_BZC != FALSE) */
            {
                l_e8MotorStartupMode = E_MSM_STEPPER_D;
            }
            /* l_u16StallDetectorDelay = l_u16DeltaPosition; */
            ENTER_SECTION(ATOMIC_KEEP_MODE);  /*lint !e534 */
            {
                if ( (g_e8MotorStatus & (uint8_t)C_MOTOR_STATUS_STOP_MASK) != (uint8_t)C_MOTOR_STATUS_STOP)
                {
                    g_e8MotorStatus = (uint8_t)C_MOTOR_STATUS_STOPPING;
                }
            }
            EXIT_SECTION(); /*lint !e438 */
        }
#elif (UART_COMM != FALSE) && (_SUPPORT_UART_IF_APP == C_UART_IF_QUAD_WHEEL_CAR)
        extern void QuadWheelCarPosUpdate(void);                                /* (MMP221014-2) */
        QuadWheelCarPosUpdate();
#endif /* (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) */

#if (_SUPPORT_TACHO_OUT != FALSE)
        if ( (++l_u16TachoCount >= l_u16TachoThreshold) && (l_u16TachoThreshold != 0U) )
        {
            /* Tacho is enabled */
            l_u16TachoCount = 0U;
#if (TACHO_OUT_IO == PIN_FUNC_IO_0)
            IO_PORT_IO_OUT_SOFT ^= C_PORT_IO_OUT_SOFT_IO0_OUT;
#elif (TACHO_OUT_IO == PIN_FUNC_IO_1)
            IO_PORT_IO_OUT_SOFT ^= C_PORT_IO_OUT_SOFT_IO1_OUT;
#elif (TACHO_OUT_IO == PIN_FUNC_IO_2)
            IO_PORT_IO_OUT_SOFT ^= C_PORT_IO_OUT_SOFT_IO2_OUT;
#elif (TACHO_OUT_IO == PIN_FUNC_IO_3)
            IO_PORT_IO_OUT_SOFT ^= C_PORT_IO_OUT_SOFT_IO3_OUT;
#elif (TACHO_OUT_IO == PIN_FUNC_IO_4)
            IO_PORT_IO_OUT_SOFT ^= C_PORT_IO_OUT_SOFT_IO4_OUT;                  /* Toggle IO[4] */
#else
#error "ERROR: Tacho-out I/O port not supported"
#endif
        }
#endif /* (_SUPPORT_TACHO_OUT != FALSE) */

        /* Current measurement used for Stall-detector "A" and current control (PID) */
        MotorDriverCurrentMeasure(l_u16MicroStepIdx);
#if (_SUPPORT_STALLDET_LA != FALSE)                                             /* MMP230824-1 */
        {
            int16_t i16MotorVoltageAngle = (int16_t)p_IdxToAngle(l_u16MicroStepIdx, l_u16MotorMicroStepsPerElecRotation);
#if (_SUPPORT_MOTOR_DIRECTION == C_MOTOR_DIRECTION_CW)
            int16_t i16TanIV = (i16MotorVoltageAngle - g_i16MotorCurrentAngle);
#elif (_SUPPORT_MOTOR_DIRECTION == C_MOTOR_DIRECTION_CCW)
            int16_t i16TanIV = (g_i16MotorCurrentAngle - i16MotorVoltageAngle);
#elif (_SUPPORT_MOTOR_DIRECTION == C_MOTOR_DIRECTION_CMD)
            int16_t i16TanIV = (i16MotorVoltageAngle - g_i16MotorCurrentAngle);
            if (g_e8MotorDirectionCCW != FALSE)
            {
                i16TanIV = -i16TanIV;
            }
#endif /* _SUPPORT_MOTOR_DIRECTION */

#if (_DEBUG_MOTOR_CURRENT_FLT != FALSE) && (_DEBUG_IV_STALL != FALSE)
#if (_DEBUG_MCUR_CYCLIC == FALSE)
            if (((g_u16SubSamplingIdx & C_SUB_SAMPLE_MASK) == 0) && (g_u16DebugBufWrIdx < C_DEBUG_BUF_SZ))
#endif /* (_DEBUG_MCUR_CYCLIC == FALSE) */
            {
                uint8_t *pBfr = &g_au8DebugBuf[g_u16DebugBufWrIdx];
                pBfr[0] = (uint8_t)(g_i16MotorCurrentCoilA >> 0);
                pBfr[1] = (uint8_t)(g_i16MotorCurrentCoilB >> 0);
                pBfr[2] = (uint8_t)(i16MotorVoltageAngle >> 8);
                pBfr[3] = (uint8_t)(g_i16MotorCurrentAngle >> 8);
                g_u16DebugBufWrIdx += 4U;
#if (_DEBUG_MCUR_CYCLIC != FALSE)
                if (g_u16DebugBufWrIdx >= C_DEBUG_BUF_SZ)
                {
                    g_u16DebugBufWrIdx = 0U;
                }
#endif /* (_DEBUG_MCUR_CYCLIC != FALSE) */
            }
#endif /* (_DEBUG_MOTOR_CURRENT_FLT != FALSE) && (_DEBUG_IV_STALL != FALSE) */

            l_i16ActLoadAngleLPF =
                p_LpfI16_I16byI16(&l_i32ActLoadAngleLPF, C_LA_LPF_COEF_OL, (i16TanIV - l_i16ActLoadAngleLPF));  /* MMP240607-2 */
        }
#endif /* (_SUPPORT_STALLDET_LA != FALSE) */

#if (_SUPPORT_STALLDET_BZC != FALSE)
        /* BEMF Zero Crossing */
        if ( (l_e8MotorStartupMode & E_MSM_BEMF) == 0U)
        {
            /* Stepper */
            /* Update (micro-)step index */
            l_u16MicroStepIdx = MotorDriverUpdateMicroStepIndex(l_u16MicroStepIdx);

            if ( (l_u16MicroStepIdx & (g_u16NrOfMicroStepsPerFullStep - 1U)) == 0U)
            {
                /* Full-step micro-step index */
                /* Check if needed to switch to BEMF-mode */
                if (l_u16CommutTimerPeriod < l_u16MaxBemfCommutTimerPeriod)
                {
                    /* Minimum speed reached */
#if (_DEBUG_COMMUT_ISR != FALSE)
                    DEBUG_SET_IO_A();
#endif /* (_DEBUG_COMMUT_ISR != FALSE) */
                    /* Convert micro-step commutation-period into full-step commutation-period */
                    l_u16CommutTimerPeriod *= g_u16NrOfMicroStepsPerFullStep;
                    IO_CTIMER0_TREGB = l_u16CommutTimerPeriod;
                    l_u16TargetCommutTimerPeriod *= g_u16NrOfMicroStepsPerFullStep;
                    l_u16LowSpeedPeriod *= g_u16NrOfMicroStepsPerFullStep;

                    l_u32MicroStepPeriodOneRPM = p_DivU32_U32byU16( (TIMER_CLOCK * 60U),
                                                                    l_u16MotorFullStepsPerMechRotation);
                    l_u16MicroStepIdx = p_DivU16_U32byU16( (uint32_t)l_u16MicroStepIdx, g_u16NrOfMicroStepsPerFullStep);  /* Convert micro-step index into a full-step-index */
#if (C_OFF_FULL_STEP != 0U)
                    l_u16MicroStepIdx += C_OFF_FULL_STEP;
                    if (l_u16MicroStepIdx >= l_u16MotorFullStepsPerElecRotation)
                    {
                        l_u16MicroStepIdx -= l_u16MotorFullStepsPerElecRotation;
                    }
#endif
                    g_e8ZcDetectorState = (uint8_t)ZC_RESET;
                    Set_ZcTime(l_u16CommutTimerPeriod >> 1);                    /* ZC at 50% of commutation period */
                    PID_SwitchStepper2Bemf();
                    l_e8MotorStartupMode |= E_MSM_BEMF;                         /* Switch to BEMF-mode */
#if (_DEBUG_COMMUT_ISR != FALSE)
                    DEBUG_CLR_IO_A();
#endif /* (_DEBUG_COMMUT_ISR != FALSE) */
                }
            }
        }
        else
        {
            /* BEMF-mode: Full-step */
            uint16_t u16MicroStepIdx = l_u16MicroStepIdx;
            if (g_e8MotorDirectionCCW != FALSE)
            {
                /* Counter Clock-wise */
                if (u16MicroStepIdx == 0U)
                {
                    u16MicroStepIdx = l_u16MotorFullStepsPerElecRotation;
                }
                u16MicroStepIdx--;                                              /* Decrement the PWM vector pointer */
            }
            else
            {
                /* Clock-wise */
                if (++u16MicroStepIdx >= l_u16MotorFullStepsPerElecRotation)    /* Test the PWM vectors pointer: 48 usteps per electrical period */
                {
                    u16MicroStepIdx = 0U;                                       /* Re-initialise the PWM vectors pointer to 0 */
                }
            }
            l_u16MicroStepIdx = u16MicroStepIdx;
        }

        if ( (l_e8MotorStartupMode & E_MSM_BEMF) != 0U)
        {
            /* BEMF-mode */
            uint8_t u8CpyZcDetectorState = g_e8ZcDetectorState;
            uint16_t u16ActualCommutTimerPeriod = IO_CTIMER0_TREGB;
            g_e8ZcDetectorState = (uint8_t)ZC_RESET;
            if ( (u8CpyZcDetectorState & (uint8_t)ZC_FOUND) != 0U)
            {
                /* ZC-found */
#if FALSE
                /* TODO[MMP]: Current version reach speed0 with minimum PWM DC */
                if ( (l_u16CorrectionRatio == NVRAM_MIN_CORR_RATIO) &&
                     (u16ActualCommutTimerPeriod < l_u16TargetCommutTimerPeriod) )
                {
                    /* Strange combination of Motor PWM DC and speed; Motor stalled */
                    l_u8StallCountB++;
                    if (l_u8StallCountB > 14U)                                  /* Minor issue; */
                    {
                        g_u8StallTypeComm |= C_STALL_FOUND_P;                   /* Used for communication (PWM) */
                        if ( (g_e8StallDetectorEna & ((uint8_t)C_STALLDET_B | (uint8_t)C_STALLDET_CALIB)) != 0U)
                        {
                            g_u8StallOcc = TRUE;                                /* Report stall and ...  */
                            MotorDriverStop( (uint16)C_STOP_EMERGENCY);         /* ... stop motor (Stall) */
                        }
                    }
                }
                else
#endif
                if (l_u8StallCountB != 0U)
                {
                    l_u8StallCountB--;
                }
                else
                {
                    /* Nothing */
                }
            }
            else
            {
                /* ZC not-found */
                Set_ZcTime(u16ActualCommutTimerPeriod >> 1);                    /* ZC at 50% of commutation period */

                l_u8StallCountB++;
                if (l_u8StallCountB > 12U)                                      /* Serious problem; After 12x zero-crossing not found, stop (Stall) */
                {
                    g_u8StallTypeComm |= C_STALL_FOUND_B;                       /* Used for communication (BEMF-ZC) */
                    if ( (g_e8StallDetectorEna & ((uint8_t)C_STALLDET_B | (uint8_t)C_STALLDET_CALIB)) != 0U)
                    {
                        g_u8StallOcc = TRUE;                                    /* Report stall and ...  */
                        MotorDriverStop( (uint16_t)C_STOP_EMERGENCY);           /* ... stop motor (Stall) */
                    }
                }
            }
            if (u16ActualCommutTimerPeriod <= 32767U)
            {
                /* Set next commutation period at +100% */
                IO_CTIMER0_TREGB = (u16ActualCommutTimerPeriod << 1);
            }
            else if (u16ActualCommutTimerPeriod <= 43690U)
            {
                /* Set next commutation period at +50% */
                IO_CTIMER0_TREGB = u16ActualCommutTimerPeriod + (u16ActualCommutTimerPeriod >> 1);
            }
            else if (u16ActualCommutTimerPeriod <= 52428U)
            {
                /* Set next commutation period at +25% */
                IO_CTIMER0_TREGB = u16ActualCommutTimerPeriod + (u16ActualCommutTimerPeriod >> 2);
            }
            else if (u16ActualCommutTimerPeriod <= 58253U)
            {
                /* Set next commutation period at +12.5% */
                IO_CTIMER0_TREGB = u16ActualCommutTimerPeriod + (u16ActualCommutTimerPeriod >> 3);
            }
            else
            {
                /* Set next commutation period at maximum */
                IO_CTIMER0_TREGB = 0xFFFEU;
            }

            ADC_BEMF_Start(l_u16MicroStepIdx);
            l_u16CorrectionRatio = VoltageCorrection();
            MotorDriver_3PhaseBEMF(l_u16MicroStepIdx);

            if (l_u16NrOfFullStepCommut < C_NR_OF_FULLSTEPS)
            {
                p_AddU32byU16( (uint32_t *)&l_u32SumCommutTimerPeriods, u16ActualCommutTimerPeriod);
                l_u16NrOfFullStepCommut++;
            }
            else
            {
                uint16_t u16PrevCommutPeriod = l_au16CommutTime[l_u16CommutTimeIdx];
                p_SubU32byU16( (uint32_t *)&l_u32SumCommutTimerPeriods, u16PrevCommutPeriod);
                p_AddU32byU16( (uint32_t *)&l_u32SumCommutTimerPeriods, u16ActualCommutTimerPeriod);
            }
            l_au16CommutTime[l_u16CommutTimeIdx] = u16ActualCommutTimerPeriod;
            l_u16CommutTimeIdx += 1;
            if (l_u16CommutTimeIdx >= C_NR_OF_FULLSTEPS)
            {
                l_u16CommutTimeIdx = 0U;
            }
            l_u16CommutTimerPeriod = p_DivU16_U32byU16(l_u32SumCommutTimerPeriods, l_u16NrOfFullStepCommut);      /* Average full-step commutation time */
            g_u16ActualMotorSpeedRPM = p_DivU16_U32byU16(l_u32MicroStepPeriodOneRPM, l_u16CommutTimerPeriod);
        }
        else
#endif /* (_SUPPORT_STALLDET_BZC != FALSE) */
        {
            /* Stepper */
#if (_SUPPORT_STALLDET_BZC == FALSE) || (C_MOTOR_PHASES != 3)
            /* Update (micro-)step index */
#if (_SUPPORT_HALL_LATCH != FALSE) && (_SUPPORT_NR_OF_HL > 1) && (_SUPPORT_HALL_LATCH_DIAG == FALSE) && (_SUPPORT_HALL_LATCH_SMOOTHING == FALSE)
            if (g_u8ZcHallFound != FALSE)
            {
                /* Force full/micro-step index */
                uint16_t u16MicroStepIdx = l_u16MicroStepIdx;
                g_u16HallLatchIO = IO_HALL_STATE;
                if (g_e8MotorDirectionCCW != ((uint8_t)C_MOTOR_DIR_CW) )
                {
                    u16MicroStepIdx = au16HallLatchIdxCCW[g_u16HallLatchIO];
#if (_SUPPORT_HALL_LATCH_LEADANGLE_COMP != FALSE) && (_SUPPORT_FULLSTEP == FALSE)
                    /* Speed-dependend Lead-angle compensation */
                    /*uint16_t u16MicroStepIdxCompensation = p_DivU16_U32byU16( (uint32_t)g_u16ActualMotorSpeedRPM, C_HL_LA_COMPENSATION_RPM);*/
                    uint16_t u16MicroStepIdxCompensation = p_MulDivU16_U16byU16byU16( g_u16ActualMotorSpeedRPM, C_HL_LA_COMPENSATION_uSTEP, g_u16MaxSpeedRPM);
                    if (u16MicroStepIdxCompensation > u16MicroStepIdx)
                    {
                        u16MicroStepIdx += l_u16MotorMicroStepsPerElecRotation;
                    }
                    u16MicroStepIdx = u16MicroStepIdx - u16MicroStepIdxCompensation;
#endif /* (_SUPPORT_HALL_LATCH_LEADANGLE_COMP != FALSE) && (_SUPPORT_FULLSTEP == FALSE) */
                }
                else
                {
                    u16MicroStepIdx = au16HallLatchIdxCW[g_u16HallLatchIO];
#if (_SUPPORT_HALL_LATCH_LEADANGLE_COMP != FALSE) && (_SUPPORT_FULLSTEP == FALSE)
                    /* Speed-dependend Lead-angle compensation */
                    /*u16MicroStepIdx += p_DivU16_U32byU16( (uint32_t)g_u16ActualMotorSpeedRPM, C_HL_LA_COMPENSATION_RPM);*/
                    u16MicroStepIdx += p_MulDivU16_U16byU16byU16( g_u16ActualMotorSpeedRPM, C_HL_LA_COMPENSATION_uSTEP, g_u16MaxSpeedRPM);
                    if (u16MicroStepIdx >= l_u16MotorMicroStepsPerElecRotation)
                    {
                        u16MicroStepIdx -= l_u16MotorMicroStepsPerElecRotation;
                    }
#endif /* (_SUPPORT_HALL_LATCH_LEADANGLE_COMP != FALSE) && (_SUPPORT_FULLSTEP == FALSE) */
                }
                l_u16MicroStepIdx = u16MicroStepIdx;
            }
            else
#endif /* (_SUPPORT_HALL_LATCH != FALSE) && (_SUPPORT_NR_OF_HL > 1) && (_SUPPORT_HALL_LATCH_DIAG == FALSE) && (_SUPPORT_HALL_LATCH_SMOOTHING == FALSE) */
            {
#if (_SUPPORT_HALL_LATCH_DIAG != FALSE)
                /* Log for each micro-step the hall-state */
                l_au8MotorMicroStep[l_u16MicroStepIdx] = (uint8_t)IO_HALL_STATE;
#endif /* (_SUPPORT_HALL_LATCH_DIAG != FALSE) */
                l_u16MicroStepIdx = MotorDriverUpdateMicroStepIndex(l_u16MicroStepIdx);
#if (_SUPPORT_HALL_LATCH != FALSE) && (_SUPPORT_HALL_LATCH_SMOOTHING != FALSE)
                g_i16HL_MicroStepCnt++;
#endif /* (_SUPPORT_HALL_LATCH != FALSE) && (_SUPPORT_HALL_LATCH_SMOOTHING != FALSE) */
            }
#endif /* (_SUPPORT_STALLDET_BZC == FALSE) || (C_MOTOR_PHASES != 3) */

#if (_SUPPORT_HALL_LATCH != FALSE) && (_SUPPORT_NR_OF_HL > 1) && (_SUPPORT_HALL_LATCH_DIAG == FALSE)
            /* Calculate actual speed; Used by PID to compensate Motor PWM-DutyCycle */
#if (_SUPPORT_MICRO_STEP_COMMUTATION != FALSE)
            if (g_u8ZcHallFound != FALSE)
            {
                /* Set commutation time to twice the hall-commutation */
                uint16_t u16CommutTimerPeriod = g_u16Hall_AvgPeriod;
                g_u8ZcHallFound = FALSE;
                if (l_u16NrOfCommut < C_NR_OF_FULLSTEPS)
                {
                    p_AddU32byU16( (uint32_t *)&l_u32SumCommutTimerPeriods, u16CommutTimerPeriod);
                    l_u16NrOfCommut += 1U;
                }
                else
                {
                    uint16_t u16PrevCommutPeriod = l_au16CommutTime[l_u16CommutTimeIdx];
                    p_SubU32byU16( (uint32_t *)&l_u32SumCommutTimerPeriods, u16PrevCommutPeriod);
                    p_AddU32byU16( (uint32_t *)&l_u32SumCommutTimerPeriods, u16CommutTimerPeriod);
                }
                /* Save full-step commutation time of the last 6 FULL-steps */
                l_au16CommutTime[l_u16CommutTimeIdx] = u16CommutTimerPeriod;
                l_u16CommutTimeIdx += 1U;
                if (l_u16CommutTimeIdx >= C_NR_OF_FULLSTEPS)
                {
                    l_u16CommutTimeIdx = 0U;
                }
                l_u16CommutTimerPeriod =
                    p_DivU16_U32byU16(l_u32SumCommutTimerPeriods, (l_u16NrOfCommut * g_u16NrOfMicroStepsPerFullStep));
            }
#else  /* (_SUPPORT_MICRO_STEP_COMMUTATION != FALSE) */
            {
                uint16_t u16CommutTimerPeriod = IO_CTIMER0_TREGB;
                if (g_u8ZcHallFound != FALSE)
                {
                    g_u8ZcHallFound = FALSE;
                    if (l_u16LastHallLatchEvent == FALSE)
                    {
                        /* Last/previous Hall Latch event was not found (MMP230720-1) */
                        u16CommutTimerPeriod += l_u16LastCommutTimerPeriod;     /* Compensate commutation period for last missed Hall-Latch event (MMP230720-1) */
                        if (u16CommutTimerPeriod < l_u16LastCommutTimerPeriod)
                        {
                            u16CommutTimerPeriod = 65534U;                      /* Truncate at maximum period */
                        }
                    }
                    if (l_u16NrOfCommut < C_NR_OF_FULLSTEPS)
                    {
                        l_u16NrOfCommut++;
                    }
                    else
                    {
                        uint16_t u16PrevCommutPeriod = l_au16CommutTime[l_u16CommutTimeIdx];
                        p_SubU32byU16( (uint32_t *)&l_u32SumCommutTimerPeriods, u16PrevCommutPeriod);
                    }
                    p_AddU32byU16( (uint32_t *)&l_u32SumCommutTimerPeriods, u16CommutTimerPeriod);

                    /* Save full-step commutation time of the last 6 FULL-steps */
                    l_au16CommutTime[l_u16CommutTimeIdx] = u16CommutTimerPeriod;
                    l_u16CommutTimeIdx += 1U;
                    if (l_u16CommutTimeIdx >= C_NR_OF_FULLSTEPS)
                    {
                        l_u16CommutTimeIdx = 0U;
                    }
                    l_u16CommutTimerPeriod =
                        p_DivU16_U32byU16(l_u32SumCommutTimerPeriods, l_u16NrOfCommut);
                    l_u16LastHallLatchEvent = TRUE;                             /* During this commutation period, a Hall-Latch event also occurred (MMP230720-1) */
                }
                else
                {
                    l_u16LastHallLatchEvent = FALSE;                            /* During this commutation period, no hall-latch event found (MMP230720-1) */
                }
                /* Update Commutation period; Increase commutation time by 25% */
                u16CommutTimerPeriod = l_u16CommutTimerPeriod;
                if (u16CommutTimerPeriod < 52427U)
                {
                    u16CommutTimerPeriod += (u16CommutTimerPeriod >> 2);        /* Increase commutation time by 25% */
                }
                else
                {
                    u16CommutTimerPeriod = 65534U;                              /* Set maximum commutation time */
                }
                IO_CTIMER0_TREGB = u16CommutTimerPeriod;
                l_u16LastCommutTimerPeriod = u16CommutTimerPeriod;
            }
#endif /* (_SUPPORT_MICRO_STEP_COMMUTATION != FALSE) */
#else  /* (_SUPPORT_HALL_LATCH != FALSE) && (_SUPPORT_NR_OF_HL > 1) */
            if (l_u16CommutTimerPeriod != l_u16TargetCommutTimerPeriod)
            {
                uint16_t u16Compensation = g_u16ForcedSpeedRPM;
                if (l_u16CommutTimerPeriod < l_u16TargetCommutTimerPeriod)
                {
                    if ( (l_u16MicroStepIdx & l_u16SpeedUpdateDec) == 0U)
                    {
                        /* Deceleration (each or multiple of micro-step) */
#if (_DEBUG_COMMUT_ISR != FALSE) && defined(DEBUG_SET_IO_C)
                        DEBUG_SET_IO_C();
#endif /* (_DEBUG_COMMUT_ISR != FALSE) */
                        uint16_t u16SpeedDecrease = p_DivU16_U32byU16( (uint32_t)l_u16AccelerationConst,
                                                                       g_u16ForcedSpeedRPM);
                        l_e8MotorStartupMode = E_MSM_STEPPER_D;                 /* Too fast, decelerate */
                        if (u16SpeedDecrease < g_u16ForcedSpeedRPM)
                        {
                            g_u16ForcedSpeedRPM -= u16SpeedDecrease;
#if (_SUPPORT_ACCELERATION == C_ACCELERATION_COSINE_CURVE)
                            if (g_u16ForcedSpeedRPM < l_u16AccelerationMin)
                            {
                                g_u16ForcedSpeedRPM = l_u16AccelerationMin;
                            }
#else
                            if (g_u16ForcedSpeedRPM < g_u16TargetMotorSpeedRPM)
                            {
                                g_u16ForcedSpeedRPM = g_u16TargetMotorSpeedRPM;
                            }
#endif /* (_SUPPORT_ACCELERATION == C_ACCELERATION_COSINE_CURVE) */
                        }
                        l_u16CommutTimerPeriod = p_DivU16_U32byU16(l_u32MicroStepPeriodOneRPM, g_u16ForcedSpeedRPM) - 1U;
                        if (l_u16CommutTimerPeriod > l_u16TargetCommutTimerPeriod)
                        {
                            l_u16CommutTimerPeriod = l_u16TargetCommutTimerPeriod;
                        }
#if (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE)
                        l_u16RampDownSteps -= (l_u16SpeedUpdateDec + 1U);
#endif /* (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) */
                        IO_CTIMER0_TREGB = l_u16CommutTimerPeriod;
                        PID_SpeedCompensate(g_u16ForcedSpeedRPM, u16Compensation);
#if (_DEBUG_COMMUT_ISR != FALSE) && defined(DEBUG_CLR_IO_C)
                        DEBUG_CLR_IO_C();
#endif /* (_DEBUG_COMMUT_ISR != FALSE) */
                    }
                }
                else if ( (l_u16MicroStepIdx & l_u16SpeedUpdateAcc) == 0U)
                {
                    /* Acceleration per acceleration_points ((multiple) micro-step) */
#if (_SUPPORT_FOC_MODE != FOC_MODE_NONE) && (_SUPPORT_CLOSED_LOOP_STARTUP == FALSE)
                    if ( (l_u16CommutTimerPeriod <= l_u16CloseLoopCommutTimerPeriod) &&
                         (g_u8MotorCtrlSpeed == (uint8_t)C_MOTOR_SPEED_AUTO) )
                    {
                        l_e8MotorStartupMode = E_MSM_FOC_2PWM;
                        g_u16ActualMotorSpeedRPM = g_u16LowSpeedRPM;
#if (_SUPPORT_PID_U32 == FALSE)
                        g_u16ActualMotorSpeedRPMe = (uint16_t)p_MulU32_U16byU16(g_u16ActualMotorSpeedRPM,
                                                                                g_u16MotorPolePairs);
#else  /* (_SUPPORT_PID_U32 == FALSE) */
                        g_u32ActualMotorSpeedRPMe = p_MulU32_U16byU16(g_u16ActualMotorSpeedRPM,
                                                                      g_u16MotorPolePairs);
#endif /* (_SUPPORT_PID_U32 == FALSE) */
                        PID_SwitchOpen2Close();
                        l_u16MicroStepIdx64k = p_DivU16_U32byU16( ((uint32_t)l_u16MicroStepIdx) << 16,
                                                                  l_u16MotorMicroStepsPerElecRotation);
                        /* Switch from CTimer to ADC IRQ */
                        HAL_ADC_EnableIRQ();                                    /* Enable ADC Interrupt */
                        IO_CTIMER0_CTRL = B_CTIMER0_STOP;                       /* Stop "commutation timer" */
                    }
                    else
#endif /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) && (_SUPPORT_CLOSED_LOOP_STARTUP == FALSE) */
                    {
#if (_DEBUG_COMMUT_ISR != FALSE) && defined(DEBUG_SET_IO_C)
                        DEBUG_SET_IO_C();
#endif /* (_DEBUG_COMMUT_ISR != FALSE) */
                        l_e8MotorStartupMode = E_MSM_STEPPER_A;                 /* Too slow, accelerate */
#if (_SUPPORT_ACCELERATION == C_ACCELERATION_COSINE_CURVE)
                        /* Cosine-Curve acceleration */
                        l_u16AccelerationConst += l_u16AccelerationStep;
                        if (l_u16AccelerationConst > l_u16AccelerationMax)
                        {
                            l_u16AccelerationConst = l_u16AccelerationMax;
                        }
#endif /* (_SUPPORT_ACCELERATION) */
                        g_u16ForcedSpeedRPM = g_u16ForcedSpeedRPM +
                                              p_DivU16_U32byU16( (uint32_t)l_u16AccelerationConst, g_u16ForcedSpeedRPM);
                        if (g_u16ForcedSpeedRPM > g_u16TargetMotorSpeedRPM)
                        {
                            g_u16ForcedSpeedRPM = g_u16TargetMotorSpeedRPM;
                        }
                        l_u16CommutTimerPeriod = p_DivU16_U32byU16(l_u32MicroStepPeriodOneRPM, g_u16ForcedSpeedRPM) - 1U;
#if (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE)
                        l_u16RampDownSteps += (l_u16SpeedUpdateDec + 1U);
#endif /* (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) */
                        IO_CTIMER0_TREGB = l_u16CommutTimerPeriod;
                        PID_SpeedCompensate(g_u16ForcedSpeedRPM, u16Compensation);
#if (_DEBUG_COMMUT_ISR != FALSE) && defined(DEBUG_CLR_IO_C)
                        DEBUG_CLR_IO_C();
#endif /* (_DEBUG_COMMUT_ISR != FALSE) */
                    }
                }
                else
                {
                    /* Nothing */
                }
            }
            else
            {
                l_e8MotorStartupMode = E_MSM_STEPPER_C;
            }
#endif /* (_SUPPORT_HALL_LATCH != FALSE) && (_SUPPORT_NR_OF_HL > 1) */

#if (_SUPPORT_PWM_MODE == BIPOLAR_FULL_STEP_BEMF) || (_SUPPORT_PWM_MODE == BIPOLAR_HALF_STEP_BEMF) || (_SUPPORT_PWM_MODE == TRIPLEPHASE_FULL_STEP_BEMF)
            ADC_Start(l_u16MicroStepIdx, TRUE);                                 /* BEMF-sensing requires ADC IRQ */
#endif /* (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM_BIPOLAR) */

            l_u16CorrectionRatio = VoltageCorrection();
#if (C_MOTOR_PHASES == 3)
            MotorDriver_3Phase(l_u16MicroStepIdx);
#else  /* (C_MOTOR_PHASES == 3) */
            MotorDriver_4Phase(l_u16MicroStepIdx);
#endif /* (C_MOTOR_PHASES == 3) */
        }

#if (_SUPPORT_STALLDET_A != FALSE)
        /* Stall detection based on current increase */
        if (MotorStallCheckA() != C_STALL_NOT_FOUND)                            /* Stall-detector "A" */
        {
            g_u8StallTypeComm |= (uint8_t)C_STALL_FOUND_A;
            if ( (g_e8StallDetectorEna & ((uint8_t)C_STALLDET_A | (uint8_t)C_STALLDET_CALIB)) != 0U)
            {
#if (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE)
                ConvMicroSteps2ShaftSteps();
#endif /* (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) */
                g_u8StallOcc = TRUE;                                            /* Report stall and ...  */
#if (_SUPPORT_STALL_REVERSE != FALSE)
                if ( (g_u8RewindFlags & (uint8_t)C_REWIND_ACTIVE) == 0U)
                {
                    MotorDriverStop( (uint16_t)C_STOP_REVERSE);                 /* ... stop motor (Stall) */
                }
                else
                {
                    MotorDriverStop( (uint16_t)C_STOP_EMERGENCY);               /* ... stop motor (Rewind Stall) */
                }
#else  /* (_SUPPORT_STALL_REVERSE != FALSE) */
                MotorDriverStop( (uint16_t)C_STOP_EMERGENCY);                   /* ... stop motor (Stall) */
#endif /* (_SUPPORT_STALL_REVERSE != FALSE) */
            }
        }
#endif /* (_SUPPORT_STALLDET_A != FALSE) */
#if (_SUPPORT_STALLDET_BRI != FALSE)
#if (_SUPPORT_FULLSTEP != FALSE)
        else if (l_u16StallDetectorDelay <= l_u16StallDetectorThrshld)
#elif (_SUPPORT_HALFSTEP != FALSE)
        else if ( ((l_u16MicroStepIdx & 1U) != 0U) && (l_u16StallDetectorDelay <= l_u16StallDetectorThrshld) )  /* Note: l_u16MicroStepIdx is already updated */
#else
#error "ERROR: Stall-detector BRI implementation"
#endif
        {
#if (_DEBUG_MOTOR_CURRENT_FLT != FALSE) && (_DEBUG_BEMF_SENSE_STALL != FALSE)
#if (_DEBUG_MCUR_CYCLIC == FALSE)
            if (((g_u16SubSamplingIdx & C_SUB_SAMPLE_MASK) == 0) && (g_u16DebugBufWrIdx < C_DEBUG_BUF_SZ))
#endif /* (_DEBUG_MCUR_CYCLIC == FALSE) */
            {
                uint8_t *pBfr = &g_au8DebugBuf[g_u16DebugBufWrIdx];
                pBfr[0] = (uint8_t)g_u16MotorBemfVoltage;
                g_u16DebugBufWrIdx += 1U;
#if (_DEBUG_MCUR_CYCLIC != FALSE)
                if (g_u16DebugBufWrIdx >= C_DEBUG_BUF_SZ)
                {
                    g_u16DebugBufWrIdx = 0U;
                }
#endif /* (_DEBUG_MCUR_CYCLIC != FALSE) */
            }
#endif /* (_DEBUG_MOTOR_CURRENT_FLT != FALSE) && (_DEBUG_BEMF_SENSE_STALL != FALSE) */
            if (g_u16MotorBemfVoltage < C_STALL_B_THRESHOLD)
            {
#if (_DEBUG_BEMF_SENSE != FALSE)
                DEBUG_SET_IO_C();
#endif /* (_DEBUG_BEMF_SENSE != FALSE) */
                l_u8StallCountB++;
                if (l_u8StallCountB >= C_STALL_B_WIDTH)
                {
                    g_u8StallTypeComm |= C_STALL_FOUND_B;                       /* Stall due to absent BEMF  */
                    if ( (g_e8StallDetectorEna & ((uint8_t)C_STALLDET_B | (uint8_t)C_STALLDET_CALIB)) != 0U)
                    {
                        g_u8StallOcc = TRUE;                                    /* Report stall and ...  */
                        if (g_e8MotorDirectionCCW != FALSE)
                        {
                            l_u32ActualPosition += C_STALL_B_WIDTH;             /* Closing */
                        }
                        else
                        {
                            l_u32ActualPosition -= C_STALL_B_WIDTH;             /* Opening */
                        }
                        MotorDriverStop( (uint16_t)C_STOP_EMERGENCY);           /* ... stop motor (Stall) */
                    }
                }
#if (_DEBUG_BEMF_SENSE != FALSE)
                DEBUG_CLR_IO_C();
#endif /* (_DEBUG_BEMF_SENSE != FALSE) */
            }
            else
            {
                l_u8StallCountB = 0U;
            }
        }
#endif /* (_SUPPORT_STALLDET_BRI != FALSE) */
#if (_SUPPORT_STALLDET_H != FALSE) && (_SUPPORT_MICRO_STEP_COMMUTATION == FALSE)
        else if (MotorStallCheckH(l_u16LastHallLatchEvent) != C_STALL_NOT_FOUND)      /* Stall-detector "H" */
        {
            g_u8StallTypeComm |= (uint8_t)C_STALL_FOUND_H;
            if ( (g_e8StallDetectorEna & ((uint8_t)C_STALLDET_H | (uint8_t)C_STALLDET_CALIB)) != 0U)
            {
#if (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE)
                ConvMicroSteps2ShaftSteps();
#endif /* (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) */
                g_u8StallOcc = TRUE;                                            /* Report stall and ...  */
#if (_SUPPORT_STALL_REVERSE != FALSE)
                if ( (g_u8RewindFlags & (uint8_t)C_REWIND_ACTIVE) == 0U)
                {
                    MotorDriverStop( (uint16_t)C_STOP_REVERSE);                 /* ... stop motor (Stall) */
                }
                else
                {
                    MotorDriverStop( (uint16_t)C_STOP_EMERGENCY);               /* ... stop motor (Rewind Stall) */
                }
#else  /* (_SUPPORT_STALL_REVERSE != FALSE) */
                MotorDriverStop( (uint16_t)C_STOP_EMERGENCY);                   /* ... stop motor (Stall) */
#endif /* (_SUPPORT_STALL_REVERSE != FALSE) */
            }
        }
#endif /* (_SUPPORT_STALLDET_H != FALSE) && (_SUPPORT_MICRO_STEP_COMMUTATION == FALSE) */
#if (_SUPPORT_STALLDET_LA != FALSE)                                             /* MMP230824-1 */
        else if (MotorStallCheckLA() != C_STALL_NOT_FOUND)
        {
            /* Invalid Load-angle stall detected */
            g_u8StallTypeComm |= (uint8_t)C_STALL_FOUND_LA;                     /* StallCheckIV */
            if ( (g_e8StallDetectorEna & (C_STALLDET_LA | C_STALLDET_CALIB)) != 0U)
            {
                g_u8StallOcc = TRUE;                                            /* Report stall and ...  */
                MotorDriverStop( (uint16_t)C_STOP_EMERGENCY);                   /* ... stop motor (Stall) */
            }
        }
#endif /* (_SUPPORT_STALLDET_LA != FALSE) */
#if (_SUPPORT_STALLDET_O != FALSE)
#if (_SUPPORT_STALLDET_O_SPEED3 != FALSE)
        /* Stall "O" below ACT_SPEED4 */
        else if ( (g_u16TargetMotorSpeedRPM < g_u16MaxSpeedRPM) && (MotorStallCheckO() != C_STALL_NOT_FOUND) )
#elif (_SUPPORT_STALLDET_O_SPEED2 != FALSE)
        /* Stall "O" below ACT_SPEED3 */
        else if ( (g_u16TargetMotorSpeedRPM < NV_ACT_SPEED3) && (MotorStallCheckO() != C_STALL_NOT_FOUND) )
#elif (_SUPPORT_STALLDET_O_SPEED1 != FALSE)
        /* Stall "O" below ACT_SPEED2 */
        else if ( (g_u16TargetMotorSpeedRPM < NV_ACT_SPEED2) && (MotorStallCheckO() != C_STALL_NOT_FOUND) )
#else
        /* Stall "O" always */
        else if ( (NV_STALL_O != 0U) && (MotorStallCheckO() != C_STALL_NOT_FOUND) )
#endif
        {
            g_u8StallTypeComm |= (uint8_t)C_STALL_FOUND_O;
            if ( (g_e8StallDetectorEna & ((uint8_t)C_STALLDET_O | (uint8_t)C_STALLDET_CALIB)) != 0U)
            {
#if (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE)
                ConvMicroSteps2ShaftSteps();
#endif /* (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) */
                g_u8StallOcc = TRUE;                                            /* Report stall and ... */
                MotorDriverStop( (uint16_t)C_STOP_EMERGENCY);                   /* ... stop motor (Stall) */
            }
        }
#endif /* (_SUPPORT_STALLDET_O != FALSE) */
#if (_SUPPORT_STALLDET_P != FALSE)
        else if (MotorStallCheckP() != C_STALL_NOT_FOUND)
        {
            g_u8StallTypeComm |= (uint8_t)C_STALL_FOUND_P;
            if ( (g_e8StallDetectorEna & ((uint8_t)C_STALLDET_P | (uint8_t)C_STALLDET_CALIB)) != 0U)
            {
                g_u8StallOcc = TRUE;                                            /* Report stall and ...  */
                MotorDriverStop( (uint16_t)C_STOP_EMERGENCY);                   /* ... stop motor (Stall) */
            }
        }
#endif /* (_SUPPORT_STALLDET_P != FALSE) */
#if FALSE
        else if ( (l_u8MotorCurrentMovAvgValid != FALSE) && (g_u16MotorCurrentMovAvgxN < (5U * C_MOVAVG_SZ)) )  /* Below 5 LSB's */
        {
            MotorDriverStop( (uint16_t)C_STOP_EMERGENCY);                       /* ... stop motor (open coil) */
            g_e8ErrorElectric = (uint8_t)C_ERR_MOTOR_ZERO_CURRENT;
            SetLastError(C_ERR_COIL_ZERO_CURRENT);
        }
#endif
    } while (FALSE);

#if (_DEBUG_COMMUT_ISR != FALSE)
    DEBUG_CLR_IO_B();
#endif /* (_DEBUG_COMMUT_ISR != FALSE) */
#if (_DEBUG_CPU_LOAD != FALSE)
    DEBUG_CLR_IO_B();                                                           /* IRQ-Priority: 4 */
#endif /* (_DEBUG_CPU_LOAD != FALSE) */
} /* End of ISR_CTIMER0_3() */
#endif /* (C_MOTOR_PHASES != 1) && (_SUPPORT_FOC_MODE == FOC_MODE_NONE) */

#if (_SUPPORT_TACHO_OUT != FALSE)
/*!*************************************************************************** *
 * MotorDriverTachoInit
 * \brief   Initialise Motor Tacho Output
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details Note: Function can only be called when Non Volatile Memory Write is inactive!
 * *************************************************************************** *
 * - Call Hierarchy: main()
 * - Cyclomatic Complexity: 3+1
 * - Nesting: 3
 * - Function calling: 0
 * *************************************************************************** */
void MotorDriverTachoInit(void)
{
    if (NV_TACHO_MODE != (uint16_t)C_TACHO_NONE)    /*lint !e506 */
    {
#if (TACHO_OUT_IO == PIN_FUNC_IO_0)
        /* Tacho at IO[0] */
        IO_PORT_IO_CFG0 = ((IO_PORT_IO_CFG0 & ~M_PORT_IO_CFG0_IO0_OUT_SEL) | C_PORT_IO_CFG0_IO0_OUT_SEL_SOFT);  /* Configure IO[0] */
#ifdef __MLX81330A01__
        IO_PORT_IO_OUT_EN |= (B_PORT_IO_OUT_EN_IO0_LV_ENABLE | C_PORT_IO_OUT_EN_IO0_EN);  /* Enable LV of IO[0] when used as Output */
#else
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__)
#if (_SUPPORT_TACHO_OUT_OD != FALSE)
        IO_PORT_IO_OUT_EN = (IO_PORT_IO_OUT_EN & ~M_PORT_IO_OUT_EN_IO_CH_SEL) |
                            B_PORT_IO_OUT_EN_IO0_LV_ENABLE |
                            B_PORT_IO_OUT_EN_IO0_OD_ENABLE;
#else  /* (_SUPPORT_TACHO_OUT_OD != FALSE) */
        IO_PORT_IO_OUT_EN = (IO_PORT_IO_OUT_EN & ~(M_PORT_IO_OUT_EN_IO_CH_SEL | B_PORT_IO_OUT_EN_IO0_OD_ENABLE)) |
                            B_PORT_IO_OUT_EN_IO0_LV_ENABLE;
#endif /* (_SUPPORT_TACHO_OUT_OD != FALSE) */
#else  /* defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__) */
#if (_SUPPORT_TACHO_OUT_OD != FALSE)
        IO_PORT_IO_OUT_EN = (IO_PORT_IO_OUT_EN & ~B_PORT_IO_OUT_EN_IO_LV_ENABLE_0) |
                            B_PORT_IO_OUT_EN_IO_OD_ENABLE_0;
#else  /* (_SUPPORT_TACHO_OUT_OD != FALSE) */
        IO_PORT_IO_OUT_EN = (IO_PORT_IO_OUT_EN & ~B_PORT_IO_OUT_EN_IO_OD_ENABLE_0) | B_PORT_IO_OUT_EN_IO_LV_ENABLE_0;
#endif /* (_SUPPORT_TACHO_OUT_OD != FALSE) */
#endif /* defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__) */
        IO_PORT_IO_ENABLE |= B_PORT_IO_ENABLE_IO_ENABLE_0;
#endif
#elif (TACHO_OUT_IO == PIN_FUNC_IO_1)
        /* Tacho at IO[1] */
        IO_PORT_IO_CFG0 = ((IO_PORT_IO_CFG0 & ~M_PORT_IO_CFG0_IO1_OUT_SEL) | C_PORT_IO_CFG0_IO1_OUT_SEL_SOFT);  /* Configure IO[1] */
#ifdef __MLX81330A01__
        IO_PORT_IO_OUT_EN |= C_PORT_IO_OUT_EN_IO1_EN;                           /* Enable IO[1] when used as Output */
#else
#if defined (__MLX81160__) || defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
#if (_SUPPORT_TACHO_OUT_OD != FALSE)
        IO_PORT_IO_OUT_EN = (IO_PORT_IO_OUT_EN & ~B_PORT_IO_OUT_EN_IO_LV_ENABLE_1) |
                            B_PORT_IO_OUT_EN_IO_OD_ENABLE_1;
#else  /* (_SUPPORT_TACHO_OUT_OD != FALSE) */
        IO_PORT_IO_OUT_EN = (IO_PORT_IO_OUT_EN & ~(B_PORT_IO_OUT_EN_IO_LV_ENABLE_1 | B_PORT_IO_OUT_EN_IO_OD_ENABLE_1));
#endif /* (_SUPPORT_TACHO_OUT_OD != FALSE) */
#endif /* defined (__MLX81160__) || defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__) */
        IO_PORT_IO_ENABLE |= B_PORT_IO_ENABLE_IO_ENABLE_1;
#endif
#elif (TACHO_OUT_IO == PIN_FUNC_IO_2)
        /* Tacho at IO[1] */
        IO_PORT_IO_CFG0 = ((IO_PORT_IO_CFG0 & ~M_PORT_IO_CFG0_IO1_OUT_SEL) | C_PORT_IO_CFG0_IO2_OUT_SEL_SOFT);  /* Configure IO[2] */
#ifdef __MLX81330A01__
        IO_PORT_IO_OUT_EN |= C_PORT_IO_OUT_EN_IO2_EN;                           /* Enable IO[2] when used as Output */
#else
#if defined (__MLX81160__) || defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
#if (_SUPPORT_TACHO_OUT_OD != FALSE)
        IO_PORT_IO_OUT_EN = (IO_PORT_IO_OUT_EN & ~B_PORT_IO_OUT_EN_IO_LV_ENABLE_2) |
                            B_PORT_IO_OUT_EN_IO_OD_ENABLE_2;
#else  /* (_SUPPORT_TACHO_OUT_OD != FALSE) */
        IO_PORT_IO_OUT_EN = (IO_PORT_IO_OUT_EN & ~(B_PORT_IO_OUT_EN_IO_LV_ENABLE_2 | B_PORT_IO_OUT_EN_IO_OD_ENABLE_2));
#endif /* (_SUPPORT_TACHO_OUT_OD != FALSE) */
#endif /* defined (__MLX81160__) || defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__) */
        IO_PORT_IO_ENABLE |= B_PORT_IO_ENABLE_IO_ENABLE_2;
#endif
#elif (TACHO_OUT_IO == PIN_FUNC_IO_3)
        /* Tacho at IO[3] */
        IO_PORT_IO_CFG0 = ((IO_PORT_IO_CFG0 & ~M_PORT_IO_CFG0_IO1_OUT_SEL) | C_PORT_IO_CFG0_IO3_OUT_SEL_SOFT);  /* Configure IO[3] */
#ifdef __MLX81330A01__
        IO_PORT_IO_OUT_EN |= C_PORT_IO_OUT_EN_IO3_EN;                           /* Enable IO[3] when used as Output */
#else
#if defined (__MLX81344__) || defined (__MLX81346__)
#if (_SUPPORT_TACHO_OUT_OD != FALSE)
        IO_PORT_IO_OUT_EN = (IO_PORT_IO_OUT_EN & ~B_PORT_IO_OUT_EN_IO_LV_ENABLE_3) |
                            B_PORT_IO_OUT_EN_IO_OD_ENABLE_3;
#else  /* (_SUPPORT_TACHO_OUT_OD != FALSE) */
        IO_PORT_IO_OUT_EN = (IO_PORT_IO_OUT_EN & ~(B_PORT_IO_OUT_EN_IO_LV_ENABLE_3 | B_PORT_IO_OUT_EN_IO_OD_ENABLE_3));
#endif /* (_SUPPORT_TACHO_OUT_OD != FALSE) */
#endif /* defined (__MLX81344__) || defined (__MLX81346__) */
        IO_PORT_IO_ENABLE |= B_PORT_IO_ENABLE_IO_ENABLE_3;
#endif
#elif (TACHO_OUT_IO == PIN_FUNC_IO_4)
        /* Tacho at IO[4] */
        IO_PORT_IO_CFG1 = (IO_PORT_IO_CFG1 & ~M_PORT_IO_CFG1_IO4_OUT_SEL) | C_PORT_IO_CFG1_IO4_OUT_SEL_SOFT;
#if defined (__MLX81344__) || defined (__MLX81346__)
#if (_SUPPORT_TACHO_OUT_OD != FALSE)
        IO_PORT_IO_OUT_EN = (IO_PORT_IO_OUT_EN & ~B_PORT_IO_OUT_EN_IO_LV_ENABLE_4) |
                            B_PORT_IO_OUT_EN_IO_OD_ENABLE_4;
#else  /* (_SUPPORT_TACHO_OUT_OD != FALSE) */
        IO_PORT_IO_OUT_EN = (IO_PORT_IO_OUT_EN & ~(B_PORT_IO_OUT_EN_IO_LV_ENABLE_4 | B_PORT_IO_OUT_EN_IO_OD_ENABLE_4));
#endif /* (_SUPPORT_TACHO_OUT_OD != FALSE) */
#endif /* defined (__MLX81344__) || defined (__MLX81346__) */
        IO_PORT_IO_ENABLE |= B_PORT_IO_ENABLE_IO_ENABLE_4;
#elif (TACHO_OUT_IO == PIN_FUNC_IO_5)
        /* Tacho at IO[4] */
        IO_PORT_IO_CFG1 = (IO_PORT_IO_CFG1 & ~M_PORT_IO_CFG1_IO5_OUT_SEL) | C_PORT_IO_CFG1_IO5_OUT_SEL_SOFT;
        IO_PORT_IO_ENABLE |= B_PORT_IO_ENABLE_IO_ENABLE_5;
#else
#error "ERROR: Tacho-out I/O port not supported"
#endif

        l_u16TachoCount = 0U;
#if (_SUPPORT_HALL_LATCH == FALSE)
        if (NV_TACHO_MODE == (uint16_t)C_TACHO_60DEG_ELECTRIC)    /*lint !e506 */
        {
            l_u16TachoThreshold = g_u16NrOfMicroStepsPerFullStep;
        }
        else
        {
            l_u16TachoThreshold = (l_u16MotorMicroStepsPerElecRotation >> 1);
            if (NV_TACHO_MODE == (uint16_t)C_TACHO_180DEG_MECHANICAL)    /*lint !e506 */
            {
                l_u16TachoThreshold = (l_u16MotorMicroStepsPerMechRotation >> 1);
            }
        }
#else  /* (_SUPPORT_HALL_LATCH == FALSE) */
        if (NV_TACHO_MODE == (uint16_t)C_TACHO_60DEG_ELECTRIC)    /*lint !e506 */
        {
            l_u16TachoThreshold = 1U;
        }
        else
        {
            l_u16TachoThreshold = (l_u16MotorFullStepsPerElecRotation >> 1);
            if (NV_TACHO_MODE == (uint16_t)C_TACHO_180DEG_MECHANICAL)    /*lint !e506 */
            {
                l_u16TachoThreshold = (l_u16MotorFullStepsPerMechRotation >> 1);
            }
        }
#endif /* (_SUPPORT_HALL_LATCH == FALSE) */
    }
} /* End of MotorDriverTachoInit() */
#endif /* (_SUPPORT_TACHO_OUT != FALSE) */

#if (_SUPPORT_MOTOR_POSITION == C_MOTOR_POS_ROTOR)
#if (_SUPPORT_TRIAXIS_MLX90363 != FALSE) || (_SUPPORT_TRIAXIS_MLX90367 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90372 != FALSE) || (_SUPPORT_TRIAXIS_MLX90377 != FALSE) || \
    /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) || */ (_SUPPORT_TRIAXIS_MLX90395 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90421 != FALSE) || (_SUPPORT_TRIAXIS_MLX90422 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90425 != FALSE) || (_SUPPORT_TRIAXIS_MLX90426 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90427 != FALSE)
/*!*************************************************************************** *
 * ConvTriaxisPos2ShaftSteps
 * \brief   Convert Triaxis position to (internal) shaft-position
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details
 * *************************************************************************** *
 * - Call Hierarchy: -
 * - Cyclomatic Complexity: 1+1
 * - Nesting: 1
 * - Function calling: 0
 * *************************************************************************** */
void ConvTriaxisPos2ShaftSteps(void)
{
    int16_t i16Position = p_MulI16hi_I16byI16( (int16_t)(g_u16ShaftAngle - C_TRIAXIS_APP_BGN),
                                               C_SHAFT_STEPS_PER_ROTATION);
    if ( (i16Position < 0) && ((i16Position + (int16_t)C_TRAVEL_OFFSET) < 0) )
    {
        g_u16ActualPosition = 0U;
    }
    else
    {
        g_u16ActualPosition = (i16Position + C_TRAVEL_OFFSET);
    }
} /* End of ConvTriaxisPos2ShaftSteps() */

/*!*************************************************************************** *
 * ConvShaftSteps2TriaxisPos
 * \brief   Convert (internal) shaft-position to Triaxis position
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16Position: "Actuator steps" based on 6400 per 360 degrees
 * \return  uint16_t: "Triaxis Position"
 * *************************************************************************** *
 * \details
 * *************************************************************************** *
 * - Call Hierarchy: -
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 *  -Function calling: 0
 * *************************************************************************** */
uint16_t ConvShaftSteps2TriaxisPos(uint16_t u16Position)
{
    return ( (uint16_t)(p_DivI16_I32byI16( ((int32_t)(u16Position - C_TRAVEL_OFFSET) << 16),
                                           C_SHAFT_STEPS_PER_ROTATION) + C_TRIAXIS_APP_BGN) );
} /* End of ConvShaftSteps2TriaxisPos() */
#endif /* (_SUPPORT_TRIAXIS_MLX90363 != FALSE) || (_SUPPORT_TRIAXIS_MLX90367 != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90372 != FALSE) || (_SUPPORT_TRIAXIS_MLX90377 != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX9038x != FALSE) || (_SUPPORT_TRIAXIS_MLX90395 != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90421 != FALSE) || (_SUPPORT_TRIAXIS_MLX90422 != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90425 != FALSE) || (_SUPPORT_TRIAXIS_MLX90426 != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90427 != FALSE) */
#endif /* (_SUPPORT_MOTOR_POSITION == C_MOTOR_POS_ROTOR) */

#endif /* (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT) */

/* EOF */
