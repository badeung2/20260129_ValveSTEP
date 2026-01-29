/*!*************************************************************************** *
 * \file        MotorDriverFOC.c
 * \brief       MLX8133x Motor Driver FOC handling
 *
 * \note        project MLX8133x
 *
 * \author      Marcel Braat
 *
 * \date        2017-08-15
 *
 * \version     2.0
 *
 * A list of functions:
 *  - Global Functions:
 *           -# MotorDriver_3Phase()
 *           -# MotorDriver_3PhaseCDI()
 *           -# MotorDriverStart()
 *           -# ISR_CTIMER0_3()
 *           -# ADC_ISR()
 *           -# CheckActivationCDI()
 *           -# ISR_CDI()
 *  - Internal Functions:
 *           -# RampUp()
 *           -# MotorDriverFOC_IDIQ()
 *           -# MotorDriverFOC_IB()
 *           -# MotorDriverFOC_IV()
 *           -# SwitchOpen2Close()
 *           -# SwitchClose2Open()
 *           -# SwitchFOC2CDI()
 *           -# SwitchCDI2FOC()
 *
 * \copyright   MELEXIS Microelectronic Integrated Systems
 *
 * Copyright (C) 2017-2022 Melexis N.V.
 * The Software is being delivered 'AS IS' and Melexis, whether explicitly or
 * implicitly, makes no warranty as to its Use or performance.
 * The user accepts the Melexis Firmware License Agreement.
 *
 * Melexis confidential & proprietary
 * *************************************************************************** *
 * Resources:
 *  Timer1 as Commutation-timer.
 *  PWM 1 through 4 (Master + 3x Slave for U, V & W phase)
 *
 * *************************************************************************** */

/*!*************************************************************************** */
/*                           INCLUDES                                          */
/* *************************************************************************** */
#include "AppBuild.h"                                                           /* Application Build */

#if (_SUPPORT_FOC_MODE != FOC_MODE_NONE) && (C_MOTOR_PHASES > 1)

#include "../ActADC.h"                                                          /* Application ADC support */

#include "drivelib/Diagnostic.h"                                                /* Diagnostics support */
#if (_SUPPORT_LOG_ERRORS != FALSE)
#include "drivelib/ErrorCodes.h"                                                /* Error logging support */
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
#include "drivelib/GlobalVars.h"                                                /* Global Variables support */
#include "drivelib/MotorDriver.h"                                               /* Motor driver support */
#include "drivelib/MotorDriverTables.h"                                         /* Wave-form vector tables */
#if (_SUPPORT_SRP != FALSE)
#include "drivelib/MotorSRP.h"                                                  /* Motor Static Rotor Position support */
#endif /* (_SUPPORT_SRP != FALSE) */
#include "drivelib/MotorStall.h"                                                /* Motor Stall Detectors */
#if (_SUPPORT_WINDMILL != FALSE)
#include "drivelib/MotorWindmill.h"                                             /* Motor Wind-mill support */
#endif /* (_SUPPORT_WINDMILL != FALSE) */
#if (_SUPPORT_MOTION_DET != C_MOTION_DET_NONE)
#include "drivelib/MotionDetector.h"                                            /* Motion Detector support */
#endif /* (_SUPPORT_MOTION_DET != C_MOTION_DET_NONE) */
#include "drivelib/PID_Control.h"                                               /* PID support */
#include "drivelib/Timer.h"                                                     /* Simple Timer support */
#include "camculib/private_mathlib.h"                                           /* Private math-library */

#if PWM_COMM && (_SUPPORT_WINDMILL || _SUPPORT_SRP)
#include "commlib/PWM_Communication.h"                                          /* PWM Communication support */
#endif /* PWM_COMM && (_SUPPORT_WINDMILL || _SUPPORT_SRP) */

#if (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
#include "senselib/Triaxis_MLX9038x.h"                                          /* Triaxis MLX9038x support (Rotor) */
#endif /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */

#include <atomic.h>
#include <sys_tools.h>

/*!*************************************************************************** */
/*                           DEFINITIONS                                       */
/* *************************************************************************** */
/*#define FOC_FREQ              ((PWM_FREQ + 1U) / 2U) */                       /*!< FOC Update Frequency: Each second PWM period */
#define FOC_FREQ                PWM_FREQ                                        /*!< FOC Update Frequency: Each PWM period */

#define C_ZERO_POS_OFFSET       10000U                                          /*!< Actuator Position Offset */

#define C_SOFT_START_RAMP_STEPS 16U                                             /*!< Soft start-up ramp in micro-steps */

#if (_SUPPORT_STALLDET_LA != FALSE)
#define C_OPEN_TO_CLOSE_IV      28                                              /*!< 2.5 degrees; 2.5/180 * 2048 (PI = 2048) */
#endif /* (_SUPPORT_STALLDET_LA != FALSE) */

#if (_SUPPORT_VARIABLE_PWM != FALSE) && (_SUPPORT_VAR_PWM_MODE == C_VAR_PWM_MODE_ON_DUTY_CYCLE)  /* MMP220815-1 */
#define C_PWM_DC_SWITCH2PWM     ((uint16_t)((((uint32_t)PWM_REG_PERIOD) * 96U) >> (8U - C_PID_FACTOR)))  /*!< PWM Duty-cycle threshold to switch to 2-PWM: 37.5% of 256 */
#define C_PWM_DC_SWITCH3PWM     ((uint16_t)((((uint32_t)PWM_REG_PERIOD) * 64U) >> (8U - C_PID_FACTOR)))  /*!< PWM Duty-cycle threshold to switch to 3-PWM: 25.0% of 256 */
#endif /* (_SUPPORT_VARIABLE_PWM != FALSE) && (_SUPPORT_VAR_PWM_MODE == C_VAR_PWM_MODE_ON_DUTY_CYCLE) */

#if (_SUPPORT_VOLTAGE_SHAPE == VOLTAGE_SHAPE_DSVM_GND) || (_SUPPORT_VOLTAGE_SHAPE == VOLTAGE_SHAPE_DSVM_ALT) || \
    (_SUPPORT_VOLTAGE_SHAPE == VOLTAGE_SHAPE_SVM)
#define C_PWM_DC_MAX            (uint16_t)((PWM_REG_PERIOD << C_PID_FACTOR) * 1.7320508075688772935274463415059)  /*!< Peak-Voltage factor (SVM) */
#else  /* (_SUPPORT_VOLTAGE_SHAPE == VOLTAGE_SHAPE_SINE) */
#define C_PWM_DC_MAX            (uint16_t)((PWM_REG_PERIOD << C_PID_FACTOR) * 2)  /*!< Peak-Voltage factor (Sine) */
#endif

#define C_ANGLE_90DEG           (uint16_t)16384U                                /*!< Angle of 90-degrees */
#define C_ANGLE_360DEG          (uint32_t)65536UL                               /*!< Angle of 360-degrees */
#define C_SPEED_RPMe_TO_ANGLE   (uint16_t)(((C_ANGLE_360DEG * 1092.2667) + (FOC_FREQ / 2U)) / FOC_FREQ)  /*!< Speed [RPMe] to angle conversion (C_ANGLE_360DEG / 60 [sec]/[min]) */

#define Q15(A)                  (int16_t)((A) * 32768)                          /*!< Q15 definition */

#define CLIP_PWM                TRUE                                            /*!< PWM Clipping enabled */

#define C_RAMP_SSZ              4U                                              /*!< Division by 16 (MMP180108-1) */

#if ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_ID_IQ)
#define C_IDIQ_FACTOR           4096U                                           /*!< Id/Iq speed scaling factor; Max 4096 electric rotations per second (or 245kRPMe) NOTE: Divided by 2 due to cast to 'int16_t' */
#define C_IDIQ_BANDWITDH        1000.0f                                         /*!< Id/Iq filter bandwidth */
#define C_IDIQ_ALPHA            Q15((2.0f * C_IDIQ_BANDWITDH) / FOC_FREQ)       /*!< Filter alpha-coefficient */
#define C_IDIQ_BETA             (uint16_t)((0.25f * (2.0f * C_IDIQ_BANDWITDH) * (2.0f * C_IDIQ_BANDWITDH)) / FOC_FREQ)  /*!< Filter beta-coefficient */
#endif /* ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_ID_IQ) */

#if (_SUPPORT_CDI != FALSE)
/*#define C_FOC2CDI_AMPL_CORR     62585U                                          / *!< FOC-to-CDI PWM-Duty Cycle (amplitude) correction (WP)* / */
#define C_FOC2CDI_AMPL_CORR     59600U                                          /*!< FOC-to-CDI PWM-Duty Cycle (amplitude) correction (HVAC FAN: 14900/16384) */
#define C_CDI_COMMUT_LPF_COEF   4096                                            /*!< CPI Commutation-period LPF Coefficient */
#define C_CDI_OFFSET_LPF_COEF   4096                                            /*!< CDI Offset LDF Coefficient */
#define C_CDI_COMMUT_OFFSET     8576U                                           /*!< CDI Commutation Offset (ISR): 7.5 Deg = 8192/65536 * 60 Deg (9V-Max PWM DC) */
#define C_CDI_PID_COEF_P        256U                                            /*!< CDI PID P-Coefficient: 125/4096 = 0.0625 */
#define C_CDI_PID_COEF_I        16U                                             /*!< CDI PID P-Coefficient: 16/4096 = 0.00390625*/
#define C_CDI_PID_COEF_D        0U                                              /*!< CDI PID P-Coefficient:  0/4096 = 0.0 */

/*! CDI Mode */
typedef enum __attribute__((packed))
{
    C_CDI_DISABLED = 0U,                                                        /*!< CDI Disabled */
    C_CDI_SEARCH,                                                               /*!< CDI Detector enabled */
    C_CDI_FOUND,                                                                /*!< CDI Found */
    C_CDI_BLIND                                                                 /*!< CDI Blind (commutation step) */
} CDI_MODE;
#define NV_STALL_CDI_WIDTH      3U                                              /*!< CDI Stall Width: 3 samples */
#endif /* (_SUPPORT_CDI != FALSE) */

/*!************************************************************************** */
/*                          GLOBAL & LOCAL VARIABLES                          */
/* ************************************************************************** */
#pragma space dp
static uint16_t l_u16MicroStepIdxPrev = 0U;                                     /*!< Previous (before update) Micro-step index */
/*static*/ uint16_t l_u16MotorVoltageAngle = 0U;                                    /*!< Motor Voltage Angle (0..2pi :: 0..0xFFFF) */
static uint16_t l_u16PrevMotorVoltageAngle = 0U;                                /*!< Previous Motor Voltage Angle (0..2pi :: 0..0xFFFF) (MMP231112-1) */
#if ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_ID_IQ)
uint16_t l_u16VoltageAngle = 0U;                                                /*!< Vector Angle (0..2pi :: 0..0xFFFF) */
#endif /* ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_ID_IQ) */
#if (_SUPPORT_CDI != FALSE)
static uint8_t l_e8CDI_Mode = (uint8_t)C_CDI_DISABLED;
static uint8_t l_u8StallCountCDI = 0U;
static uint16_t l_au16CDI_TCNT[3] = {0U};
static uint16_t l_au16CDI_TPER[3] = {0U};
#endif /* (_SUPPORT_CDI != FALSE) */
#if (_SUPPORT_FOC_MODE == (FOC_OBSERVER_SENSORLESS | FOC_MODE_ID_IQ))
volatile int16_t l_i16Valpha;                                                   /*!< Valpha (Inverse Park Transformation) */
volatile int16_t l_i16Vbeta;                                                    /*!< Vbeta (Inverse Park Transformation) */
#endif /* (_SUPPORT_FOC_MODE == (FOC_OBSERVER_SENSORLESS | FOC_MODE_ID_IQ)) */
#if (C_MOTOR_PHASES == 3) && \
    (_SUPPORT_PWM_MODE != TRIPLEPHASE_ALLPWM_MIRROR) && \
    (_SUPPORT_PWM_MODE != BIPOLAR_TRIPHASE_TWOPWM_INDEPENDENT_GND) && \
    (_SUPPORT_PWM_MODE != BIPOLAR_TRIPHASE_ALLPWM_MIRROR) && \
    (UART_COMM != FALSE) && ((_SUPPORT_UART_IF_APP == C_UART_IF_SCOPE) || (_SUPPORT_UART2_IF_APP == C_UART_IF_SCOPE))
uint16_t l_u16MotorVoltageCoilA = 0U;                                           /*!< Motor Voltage Coil 'A' (used by UART-Scope) */
#endif
#pragma space none

#pragma space nodp                                                              /* __NEAR_SECTION__ */
#if (_SUPPORT_SOFT_START != FALSE)
static uint16_t l_u16StartCorrectionRatio = 0U;                                 /*!< Soft-ramp correction ratio */
static uint16_t l_u16RampStep = 0U;                                             /*!< Soft-ramp steps */
static E_ALIGNMENT_STEPS_t l_e8AlignmentStep = E_ALIGN_STEP_SHORT;              /*!< Alignment step */
uint8_t g_u8SkipSRP = FALSE;
#endif /* (_SUPPORT_SOFT_START != FALSE) */
static uint32_t l_u32CommutTimerPeriodLPF = 0U;                                 /*!< Commutation Timer period (LPF-Internal) */
uint16_t g_u16ActualCommutTimerPeriod = 0U;                                     /*!< Commutation Timer Period */
int16_t l_i16ActLoadAngle = 0;                                                  /*!< Actual Load-angle */
#if (_SUPPORT_PWM_POL_CORR == FALSE)
static uint16_t l_u16PwmState = FALSE;                                          /*!< PWM actual State */
static uint16_t l_u16PwmStatePrev = FALSE;                                      /*!< PWM previous state */
static uint16_t l_u16PwmStateStepIdx;                                           /*!< PWM state step-index */
#endif /* (_SUPPORT_PWM_POL_CORR == FALSE) */

#if ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_ID_IQ)
static uint32_t l_u32MotorSpeedRadPSe = 0U;                                     /*!< Rotor speed in rad/sec * 65536 */
#endif /* ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_ID_IQ) */

#if (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE)
static uint32_t l_u32StartPosition = 0U;                                        /*!< Motor start-position */
#endif /* (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) */
#if (_SUPPORT_CDI != FALSE)
static uint16_t l_u16FullStepIdx;
static uint16_t l_u16FullStepIdxPrev;
#if (_SUPPORT_CDI_PID != FALSE)
PID_PARAMS_t sPIDpCDI =                                                         /*!< PID parameter structure for CDI */
{
    .i16CoefI = 0,
    .u32SumError = 0U,
    .u32SumErrorMax = (65535UL << C_GN_PID),
    .i16CoefP = 0,
#if (_SUPPORT_PID_D_COEF != FALSE)
    .i16PrevError = 0,
    .i16CoefD = 0,
#endif /* (_SUPPORT_PID_D_COEF != FALSE) */
    .u16MinOutput = 0U,
    .u32MaxOutput = 65535UL
};
#else  /* (_SUPPORT_CDI_PID != FALSE) */
static int32_t l_i32CDIOffset;                                                  /*!< Commutation-time LPF CDI-Offset */
#endif /* (_SUPPORT_CDI_PID != FALSE) */
#endif /* (_SUPPORT_CDI != FALSE) */
#if (_SUPPORT_STALLDET_FLUX != FALSE)
/*static*/ uint16_t l_u16Flux;                                                  /*!< FOC Id/Iq Flux estimation */
#endif /* (_SUPPORT_STALLDET_FLUX != FALSE) */
uint16_t g_u16MemorizedTargetMotorSpeedRPM = 0U;
#pragma space none                                                              /* __NEAR_SECTION__ */

/* Externals from MotorDriver.c */
#if (_SUPPORT_WINDMILL != FALSE) || (_SUPPORT_CLOSED_LOOP_STARTUP == FALSE) || (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
extern uint32_t l_u32MicroStepPeriodOneRPM;                                     /*!< Temporary variable speed control */
#endif /* (_SUPPORT_WINDMILL != FALSE) || (_SUPPORT_CLOSED_LOOP_STARTUP == FALSE) || (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */
#if (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE)
extern uint16_t l_u16DeltaPosition;                                             /*!< Difference in target and actual position */
extern uint16_t l_u16RampDownSteps;                                             /*!< Ramp-down steps */
extern uint8_t l_u8MotorHoldingCurrState;                                       /*!< Motor Holding Current State */
extern uint8_t l_u8MotorHoldDelay;                                              /*!< Delay between drive stage frozen to LS */
#endif /* (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) */
extern uint16_t l_u16LowSpeedPeriod;                                            /*!< Minimum speed (period) */
extern uint16_t l_u16TargetCommutTimerPeriod;                                   /*!< Target commutation timer period (target speed) */
#if (_SUPPORT_CDI != FALSE)
extern uint32_t l_u32FullStepPeriodOneRPM;                                      /*!< [CDI] Temporary variable speed control */
#endif /* (_SUPPORT_CDI != FALSE) */
#if ((_SUPPORT_FOC_MODE & FOC_OBSERVER_MASK) == FOC_OBSERVER_SENSORED)
extern uint16_t l_u16ResolverAngle;                                             /*!< Resolver Angle */
static uint16_t l_u16PrevResolverAngle;
static uint16_t l_u16RotorAngleSpeed;
static uint32_t l_u32RotorAngleSpeedLPF;
#endif /* ((_SUPPORT_FOC_MODE & FOC_OBSERVER_MASK) == FOC_OBSERVER_SENSORED) */

#if (_SUPPORT_SOFT_START != FALSE)
extern uint16_t l_u16PID_CtrlCounter;                                           /* MMP230912-3 */
#endif /* (_SUPPORT_SOFT_START != FALSE) */

#if (_SUPPORT_STALLDET_LA != FALSE) && (_SUPPORT_CLOSED_LOOP_STARTUP == FALSE)
extern int16_t l_i16StallThresholdLA;                                           /*!< Stall-detector Load-angle threshold x 5 degrees */
extern volatile uint8_t l_u8StallCountLA;                                       /*!< Stall detector "LA" Counter */
#endif /* (_SUPPORT_STALLDET_LA != FALSE) && (_SUPPORT_CLOSED_LOOP_STARTUP == FALSE) */

#if (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE)
extern uint16_t DeltaPosition(void);

#if ((_SUPPORT_FOC_MODE & FOC_OBSERVER_MASK) == FOC_OBSERVER_SENSORLESS) && (_SUPPORT_CLOSED_LOOP_STARTUP != FALSE)
static uint16_t RampUp(void)
{
    /* 1.8us faster */
    uint16_t u16Result;  /*lint -e530 */

    __asm__ __volatile__ (
        "lod  Y, #_l_u32StartPosition\n\t"                                      /* Y = Address of l_u32StartPosition */
        "lod  X, #_l_u32ActualPosition\n\t"                                     /* X = Address of l_u32ActualPosition */
        "lod  AL, dp:_g_e8MotorDirectionCCW\n\t"
        "jne  _RU_10\n\t"                                                       /* CCW: (l_u32StartPosition - l_u32ActualPosition) */
        "lod  X, #_l_u32StartPosition\n\t"                                      /* X = Address of l_u32StartPosition */
        "lod  Y, #_l_u32ActualPosition\n\t"                                     /* Y = Address of l_u32ActualPosition */
        "_RU_10:\n\t"                                                           /* CW: (l_u32ActualPosition - l_u32StartPosition) */
        "mov  YA, [Y]\n\t"
        "sub  YA, [X]\n\t"
        "jsge _RU_20\n\t"                                                       /* if ( (iResult = YA) < 0 */
        "mov  YA, #0\n\t"                                                       /* YA = 0 */
        "_RU_20:\n\t"
        "cmp  Y, #0x0000\n\t"                                                   /* if ( (iResult = YA) > 0x0000FFFF ) */
        "je   _RU_30\n\t"
        "mov  A, #0xFFFF\n\t"                                                   /* iResult = 0xFFFF */
        "_RU_30:"
        : "=a" (u16Result)
        :
        : "X", "Y"
        );
    return (u16Result);
} /* End of RampUp() */
#endif /* ((_SUPPORT_FOC_MODE & FOC_OBSERVER_MASK) == FOC_OBSERVER_SENSORLESS) */
#endif /* (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) */

#if (_SUPPORT_VARIABLE_PWM != FALSE) && (_SUPPORT_VAR_PWM_MODE != C_VAR_PWM_MODE_OFF)
/*!*************************************************************************** *
 * MotorDriverReconfigure
 * \brief   Reconfigure motor driver controller
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u8PwmMode : FALSE : 2-coil/phase current measurement; PWM in mirror-mode
 *                           TRUE : 3-coil/phase current measurement; PWM in independent mode
 * \return  -
 * *************************************************************************** *
 * \details MMP220815-1
 * *************************************************************************** *
 * - Call Hierarchy:
 * - Cyclomatic Complexity: 1+1
 * - Nesting: 1
 * - Function calling: 0
 * *************************************************************************** */
void MotorDriverReconfigure(uint8_t u8PwmMode)
{
#if (_SUPPORT_PWM_MIRROR != FALSE)
    IO_PWM_MASTER1_CTRL = B_PWM_MASTER1_STOP;                                   /* Disable Master 1 */
    IO_PWM_SLAVE1_CTRL = B_PWM_SLAVE1_STOP;                                     /* Disable Slave 1 */
    IO_PWM_SLAVE2_CTRL = B_PWM_SLAVE2_STOP;                                     /* Disable Slave 2 */

    if (u8PwmMode == FALSE)
    {
        /* 2-coil/phase PWM-mode */
        IO_PWM_MASTER1_CTRL = (PWM_PRESCALER << 8) | C_PWM_MASTER1_MODE_MIRROR;  /* Initialise the master pre-scaler ratio (Fck/8); PWM at 50% centred */
        IO_PWM_SLAVE1_CTRL = B_PWM_SLAVE1_SLAVE | C_PWM_SLAVE1_MODE_MIRROR;     /* PWM at 50% centred */
        IO_PWM_SLAVE2_CTRL = B_PWM_SLAVE2_SLAVE | C_PWM_SLAVE2_MODE_MIRROR | B_PWM_SLAVE2_POL;  /* PWM at 100% centred */
#if (_DEBUG_PWM_VARIABLE_PWM != FALSE)
        DEBUG_CLR_IO_A();
#endif /* (_DEBUG_PWM_VARIABLE_PWM != FALSE) */
    }
    else
    {
        /* 3-coil/phase PWM-mode */
        IO_PWM_MASTER1_CTRL = (PWM_PRESCALER << 8) | C_PWM_MASTER1_MODE_INDEPENDENT;  /* Initialise the master pre-scaler ratio (Fck/8); PWM at 33% centred */
        IO_PWM_SLAVE1_CTRL = B_PWM_SLAVE1_SLAVE | C_PWM_SLAVE1_MODE_INDEPENDENT;  /* PWM at 67% centred */
        IO_PWM_SLAVE2_CTRL = B_PWM_SLAVE2_SLAVE | C_PWM_SLAVE2_MODE_MIRROR | B_PWM_SLAVE2_POL;  /* PWM at 100% centred */
#if (_DEBUG_PWM_VARIABLE_PWM != FALSE)
        DEBUG_SET_IO_A();
#endif /* (_DEBUG_PWM_VARIABLE_PWM != FALSE) */
    }
    g_u8TriplePWM = u8PwmMode;
    MotorDriver_3Phase(l_u16MicroStepIdx);
    IO_PWM_MASTER1_CTRL = B_PWM_MASTER1_START;                                  /* Start PWM in application mode */
#else  /* (_SUPPORT_PWM_MIRROR != FALSE) */
    g_u8TriplePWM = u8PwmMode;
#endif /* (_SUPPORT_PWM_MIRROR != FALSE) */
} /* End of MotorDriverReconfigure() */
#endif /* (_SUPPORT_VARIABLE_PWM != FALSE) && (_SUPPORT_VAR_PWM_MODE != C_VAR_PWM_MODE_OFF) */

#if (C_MOTOR_PHASES == 3) && \
    (_SUPPORT_PWM_MODE != TRIPLEPHASE_ALLPWM_MIRROR) && \
    (_SUPPORT_PWM_MODE != BIPOLAR_TRIPHASE_TWOPWM_INDEPENDENT_GND) && \
    (_SUPPORT_PWM_MODE != BIPOLAR_TRIPHASE_ALLPWM_MIRROR)
#if (_SUPPORT_RAM_FUNC != FALSE)
void MotorDriver_3Phase(uint16_t u16MicroStepIdx) __attribute__ ((section(".ramfunc"))) __attribute__ ((aligned(8)));
#elif (_SUPPORT_OPTIMIZE_FOR_SPEED != FALSE)
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
 * \details MMP181206: Tuned for MLX-GNU V3.0.39                        [RPM]
 * Total Performance: Normal:   typ. 25.7 [us], max. 26.45[us]           8400
 *                    Clipping: typ. 26.0 [us], max. 27.0 [us]           9250
 *                    Blockwave:typ. 26.1 [us], max. 27.25[us] (< 100%)
 *                              typ. 31.95[us], max. 33.65[us] (> 100%)  9200
 * *************************************************************************** *
 * - Call Hierarchy: ISR_CTIMER0_3(), MotorDriverStart(), MotorDriverStop(),
 *                   PID_COntrol()
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 1 (p_MulI16_I16byI16Shft4())
 * *************************************************************************** */
void MotorDriver_3Phase(uint16_t u16MicroStepIdx)
{
#if (_SUPPORT_VOLTAGE_SHAPE == VOLTAGE_SHAPE_DSVM_ALT)
    static uint16_t l_u16PrevMicroStepIdx = 0U;
#endif /* (_SUPPORT_VOLTAGE_SHAPE == VOLTAGE_SHAPE_DSVM_ALT) */

    uint16_t u16Pwm1;
#if (_SUPPORT_BOOST_MODE == BOOST_MODE_WAVEFORM)
    uint16_t *pu16BoostVector;
    pu16BoostVector = (uint16_t *)&c_au16MicroStepVector3PH_Boost[u16MicroStepIdx];
#endif /* (_SUPPORT_BOOST_MODE == BOOST_MODE_WAVEFORM) */

#if (_SUPPORT_VARIABLE_PWM != FALSE) && (_SUPPORT_VAR_PWM_MODE != C_VAR_PWM_MODE_OFF)
    if (g_u8TriplePWM == FALSE)                                                 /* MMP220815-1 */
#endif /* (_SUPPORT_VARIABLE_PWM != FALSE) && (_SUPPORT_VAR_PWM_MODE != C_VAR_PWM_MODE_OFF) */
    {
        register uint16_t *pu16Vector;
#if (_SUPPORT_ANTICOGGING != FALSE)
        int16_t i16AntiCoggingAmplitude = 0;
        uint16_t u16AntiCoggingHarmonic;

        if (g_u16ActualMotorSpeedRPM < (C_SPEED_3 / 16U) )                      /* Use Anti-cogging when speed < 6.25% of Max-speed */
        {
            i16AntiCoggingAmplitude = (int16_t)(C_ANTI_COGGING_AMPLITUDE * 8U);  /* Anti-cogging Amplitude */
            u16AntiCoggingHarmonic = (u16MicroStepIdx * C_ANTI_COGGING_HARMONIC);  /* Anti-cogging 6th harmonic with phase-shift */
            if (g_e8MotorDirectionCCW != FALSE)
            {
                u16AntiCoggingHarmonic += C_ANTI_COGGING_PHASE_SHIFT_CCW;       /* Harmonic Phase Shift CCW */
            }
            else
            {
                u16AntiCoggingHarmonic += C_ANTI_COGGING_PHASE_SHIFT_CW;        /* Harmonic Phase Shift CW */
            }
#if (_SUPPORT_SPECIAL_COMM_FIELD != FALSE)
            if (g_u8Special[1] != 0U)
            {
                u16AntiCoggingHarmonic = u16MicroStepIdx * g_u8Special[1];      /* N-harmonic */
                u16AntiCoggingHarmonic += g_u8Special[3];                       /* Harmonic Phase Shift */
                i16AntiCoggingAmplitude = (int16_t)(g_u8Special[2] * 8U);       /* Harmonic amplitude */
            }
#endif /* (_SUPPORT_SPECIAL_COMM_FIELD != FALSE) */
            u16AntiCoggingHarmonic = p_ModU16_U32byU16((uint32_t)u16AntiCoggingHarmonic, 192U);
            i16AntiCoggingAmplitude = p_MulI16hi_I16byI16asr4(
                c_ai16MicroStepVector3PH_SinCos192[u16AntiCoggingHarmonic],
                i16AntiCoggingAmplitude);
        }
#endif /* (_SUPPORT_ANTICOGGING != FALSE) */
        pu16Vector = p_X(u16MicroStepIdx); /* (uint16_t *) &c_au16MicroStepVector3PH[u16MicroStepIdx]; */
#if (UART_COMM != FALSE) && ((_SUPPORT_UART_IF_APP == C_UART_IF_SCOPE) || (_SUPPORT_UART2_IF_APP == C_UART_IF_SCOPE))
        l_u16MotorVoltageCoilA = *pu16Vector;
#endif /* (UART_COMM != FALSE) && ((_SUPPORT_UART_IF_APP == C_UART_IF_SCOPE) || (_SUPPORT_UART2_IF_APP == C_UART_IF_SCOPE)) */
#if (_SUPPORT_VOLTAGE_SHAPE != VOLTAGE_SHAPE_DSVM_ALT) /* MMP210524-1 */
        uint16_t u16Phase = (u16MicroStepIdx + (C_MICROSTEP_PER_FULLSTEP / 2U)) &
                            ~((2U * C_MICROSTEP_PER_FULLSTEP) - 1U);
#if (_SUPPORT_BOOST_MODE == BOOST_MODE_WAVEFORM)
        if (l_u16CorrectionRatio <= l_u16MaxPwmCorrRatio)
#endif /* (_SUPPORT_BOOST_MODE == BOOST_MODE_WAVEFORM) */
        {
            switch (u16Phase)
            {
                default:
                {
                    /* Small W-phase low period --> W-phase to Low */
                    /* V */
#if (C_PID_FACTOR == 4U)
                    u16Pwm1 = p_MulU16hi_pU16byU16lsr4((pu16Vector + (2U * C_MICROSTEP_PER_FULLSTEP)),
                                                       l_u16CorrectionRatio);   /* V */
#elif (C_PID_FACTOR == 3U)
                    u16Pwm1 = p_MulU16hi_pU16byU16lsr3((pu16Vector + (2U * C_MICROSTEP_PER_FULLSTEP)),
                                                       l_u16CorrectionRatio);   /* V */
#else
#error "No PWM-W calculation"
#endif
#if (_SUPPORT_ANTICOGGING != FALSE)
                    u16Pwm1 += i16AntiCoggingAmplitude;
                    u16Pwm1 = (uint16_t)p_ClipMinMaxI16((int16_t)u16Pwm1,
                                                        (int16_t)C_PWM_MIN_DC,
                                                        (int16_t)l_u16MaxPwmRatio);
#else /* (_SUPPORT_ANTICOGGING != FALSE) */
                    u16Pwm1 = p_ClipMinMaxU16(u16Pwm1, C_PWM_MIN_DC, l_u16MaxPwmRatio);
#endif /* (_SUPPORT_ANTICOGGING != FALSE) */
#if (_SUPPORT_PWM_POL_CORR == FALSE)
                    if (l_u16PwmState != FALSE)
                    {
                        IO_PWM_SLAVE1_CTRL = B_PWM_SLAVE1_SLAVE | C_PWM_SLAVE1_MODE_MIRROR;
                        IO_PWM_SLAVE1_LT = (PWM_SCALE_OFFSET - u16Pwm1);
                    }
                    else
                    {
                        IO_PWM_SLAVE1_CTRL = B_PWM_SLAVE1_SLAVE | C_PWM_SLAVE1_MODE_MIRROR | B_PWM_SLAVE1_POL;
                        IO_PWM_SLAVE1_LT = u16Pwm1;
                    }
#else  /* (_SUPPORT_PWM_POL_CORR == FALSE) */
                    IO_PWM_SLAVE1_CTRL = B_PWM_SLAVE1_SLAVE | C_PWM_SLAVE1_MODE_MIRROR | B_PWM_SLAVE1_POL;
                    IO_PWM_SLAVE1_LT = u16Pwm1;
#endif /* (_SUPPORT_PWM_POL_CORR == FALSE) */

                    /* W */
#if (_SUPPORT_PWM_POL_CORR == FALSE)
                    IO_PWM_SLAVE2_CTRL = B_PWM_SLAVE2_SLAVE | C_PWM_SLAVE2_MODE_MIRROR | B_PWM_SLAVE2_POL;
#endif /* (_SUPPORT_PWM_POL_CORR == FALSE) */
                    IO_PWM_SLAVE2_LT = 0U;

                    /* U */
#if (C_PID_FACTOR == 4U)
                    u16Pwm1 = p_MulU16hi_pU16byU16lsr4(pu16Vector, l_u16CorrectionRatio);  /* U */
#elif (C_PID_FACTOR == 3U)
                    u16Pwm1 = p_MulU16hi_pU16byU16lsr3(pu16Vector, l_u16CorrectionRatio);  /* U */
#else
#error "No PWM-U calculation"
#endif
#if (_SUPPORT_ANTICOGGING != FALSE)
                    u16Pwm1 += i16AntiCoggingAmplitude;
                    u16Pwm1 = (uint16_t)p_ClipMinMaxI16((int16_t)u16Pwm1,
                                                        (int16_t)C_PWM_MIN_DC,
                                                        (int16_t)l_u16MaxPwmRatio);
#else /* (_SUPPORT_ANTICOGGING != FALSE) */
                    u16Pwm1 = p_ClipMinMaxU16(u16Pwm1, C_PWM_MIN_DC, l_u16MaxPwmRatio);
#endif /* (_SUPPORT_ANTICOGGING != FALSE) */
#if (_SUPPORT_PWM_POL_CORR == FALSE)
                    if (l_u16PwmState != FALSE)
                    {
                        IO_PWM_MASTER1_CTRL = (PWM_PRESCALER << 8) | C_PWM_MASTER1_MODE_MIRROR | B_PWM_MASTER1_POL;
                    }
                    else
                    {
                        IO_PWM_MASTER1_CTRL = (PWM_PRESCALER << 8) | C_PWM_MASTER1_MODE_MIRROR;
                        u16Pwm1 = (PWM_SCALE_OFFSET - u16Pwm1);
                    }
#else  /* (_SUPPORT_PWM_POL_CORR == FALSE) */
                    u16Pwm1 = (PWM_SCALE_OFFSET - u16Pwm1);
#endif /* (_SUPPORT_PWM_POL_CORR == FALSE) */
                }
                break;
                case (2U * C_MICROSTEP_PER_FULLSTEP):
                {
                    /* Small V-phase low period --> V-phase to Low */
                    /* V */
                    IO_PWM_SLAVE1_CTRL = B_PWM_SLAVE1_SLAVE | C_PWM_SLAVE1_MODE_MIRROR | B_PWM_SLAVE1_POL;
                    IO_PWM_SLAVE1_LT = 0U;

                    /* W */
#if (C_PID_FACTOR == 4U)
                    u16Pwm1 = p_MulU16hi_pU16byU16lsr4((pu16Vector + (4U * C_MICROSTEP_PER_FULLSTEP)),
                                                       l_u16CorrectionRatio);   /* W */
#elif (C_PID_FACTOR == 3U)
                    u16Pwm1 = p_MulU16hi_pU16byU16lsr3((pu16Vector + (4U * C_MICROSTEP_PER_FULLSTEP)),
                                                       l_u16CorrectionRatio);   /* W */
#else
#error "No PWM-W calculation"
#endif
#if (_SUPPORT_ANTICOGGING != FALSE)
                    u16Pwm1 += i16AntiCoggingAmplitude;
                    u16Pwm1 = (uint16_t)p_ClipMinMaxI16((int16_t)u16Pwm1,
                                                        (int16_t)C_PWM_MIN_DC,
                                                        (int16_t)l_u16MaxPwmRatio);
#else /* (_SUPPORT_ANTICOGGING != FALSE) */
                    u16Pwm1 = p_ClipMinMaxU16(u16Pwm1, C_PWM_MIN_DC, l_u16MaxPwmRatio);
#endif /* (_SUPPORT_ANTICOGGING != FALSE) */
#if (_SUPPORT_PWM_POL_CORR == FALSE)
                    if (l_u16PwmState != FALSE)
                    {
                        IO_PWM_SLAVE2_CTRL = B_PWM_SLAVE2_SLAVE | C_PWM_SLAVE2_MODE_MIRROR;
                        IO_PWM_SLAVE2_LT = (PWM_SCALE_OFFSET - u16Pwm1);
                    }
                    else
                    {
                        IO_PWM_SLAVE2_CTRL = B_PWM_SLAVE2_SLAVE | C_PWM_SLAVE2_MODE_MIRROR | B_PWM_SLAVE2_POL;
                        IO_PWM_SLAVE2_LT = u16Pwm1;
                    }
#else  /* (_SUPPORT_PWM_POL_CORR == FALSE) */
                    IO_PWM_SLAVE2_LT = u16Pwm1;
#endif /* (_SUPPORT_PWM_POL_CORR == FALSE) */
                    /* U */
#if (C_PID_FACTOR == 4U)
                    u16Pwm1 = p_MulU16hi_pU16byU16lsr4(pu16Vector, l_u16CorrectionRatio);  /* U */
#elif (C_PID_FACTOR == 3U)
                    u16Pwm1 = p_MulU16hi_pU16byU16lsr3(pu16Vector, l_u16CorrectionRatio);  /* U */
#else
#error "No PWM-U calculation"
#endif
#if (_SUPPORT_ANTICOGGING != FALSE)
                    u16Pwm1 += i16AntiCoggingAmplitude;
                    u16Pwm1 = (uint16_t)p_ClipMinMaxI16((int16_t)u16Pwm1,
                                                        (int16_t)C_PWM_MIN_DC,
                                                        (int16_t)l_u16MaxPwmRatio);
#else /* (_SUPPORT_ANTICOGGING != FALSE) */
                    u16Pwm1 = p_ClipMinMaxU16(u16Pwm1, C_PWM_MIN_DC, l_u16MaxPwmRatio);
#endif /* (_SUPPORT_ANTICOGGING != FALSE) */
#if (_SUPPORT_PWM_POL_CORR == FALSE)
                    if (l_u16PwmState != FALSE)
                    {
                        IO_PWM_MASTER1_CTRL = (PWM_PRESCALER << 8) | C_PWM_MASTER1_MODE_MIRROR | B_PWM_MASTER1_POL;
                    }
                    else
                    {
                        u16Pwm1 = (PWM_SCALE_OFFSET - u16Pwm1);
                        IO_PWM_MASTER1_CTRL = (PWM_PRESCALER << 8) | C_PWM_MASTER1_MODE_MIRROR;
                    }
#else  /* (_SUPPORT_PWM_POL_CORR == FALSE) */
                    u16Pwm1 = (PWM_SCALE_OFFSET - u16Pwm1);
#endif /* (_SUPPORT_PWM_POL_CORR == FALSE) */
                }
                break;
                case (4U * C_MICROSTEP_PER_FULLSTEP):
                {
                    /* Small U-phase low period --> U-phase to Low */
                    /* V */
                    pu16Vector += (2U * C_MICROSTEP_PER_FULLSTEP);
#if (C_PID_FACTOR == 4U)
                    u16Pwm1 = p_MulU16hi_pU16byU16lsr4(pu16Vector, l_u16CorrectionRatio);  /* V */
#elif (C_PID_FACTOR == 3U)
                    u16Pwm1 = p_MulU16hi_pU16byU16lsr3(pu16Vector, l_u16CorrectionRatio);  /* V */
#else
#error "No PWM-V calculation"
#endif
#if (_SUPPORT_ANTICOGGING != FALSE)
                    u16Pwm1 += i16AntiCoggingAmplitude;
                    u16Pwm1 = (uint16_t)p_ClipMinMaxI16((int16_t)u16Pwm1,
                                                        (int16_t)C_PWM_MIN_DC,
                                                        (int16_t)l_u16MaxPwmRatio);
#else /* (_SUPPORT_ANTICOGGING != FALSE) */
                    u16Pwm1 = p_ClipMinMaxU16(u16Pwm1, C_PWM_MIN_DC, l_u16MaxPwmRatio);
#endif /* (_SUPPORT_ANTICOGGING != FALSE) */
#if (_SUPPORT_PWM_POL_CORR == FALSE)
                    if (l_u16PwmState != FALSE)
                    {
                        IO_PWM_SLAVE1_CTRL = B_PWM_SLAVE1_SLAVE | C_PWM_SLAVE1_MODE_MIRROR | B_PWM_SLAVE1_POL;
                        IO_PWM_SLAVE1_LT = u16Pwm1;
                    }
                    else
                    {
                        IO_PWM_SLAVE1_CTRL = B_PWM_SLAVE1_SLAVE | C_PWM_SLAVE1_MODE_MIRROR;
                        IO_PWM_SLAVE1_LT = (PWM_SCALE_OFFSET - u16Pwm1);
                    }
#else  /* (_SUPPORT_PWM_POL_CORR == FALSE) */
                    IO_PWM_SLAVE1_CTRL = B_PWM_SLAVE1_SLAVE | C_PWM_SLAVE1_MODE_MIRROR;
                    IO_PWM_SLAVE1_LT = (PWM_SCALE_OFFSET - u16Pwm1);
#endif /* (_SUPPORT_PWM_POL_CORR == FALSE) */

                    /* W */
                    pu16Vector += (2U * C_MICROSTEP_PER_FULLSTEP);
#if (C_PID_FACTOR == 4U)
                    u16Pwm1 = p_MulU16hi_pU16byU16lsr4(pu16Vector, l_u16CorrectionRatio);  /* W */
#elif (C_PID_FACTOR == 3U)
                    u16Pwm1 = p_MulU16hi_pU16byU16lsr3(pu16Vector, l_u16CorrectionRatio);  /* W */
#else
#error "No PWM-W calculation"
#endif
#if (_SUPPORT_ANTICOGGING != FALSE)
                    u16Pwm1 += i16AntiCoggingAmplitude;
                    u16Pwm1 = (uint16_t)p_ClipMinMaxI16((int16_t)u16Pwm1,
                                                        (int16_t)C_PWM_MIN_DC,
                                                        (int16_t)l_u16MaxPwmRatio);
#else /* (_SUPPORT_ANTICOGGING != FALSE) */
                    u16Pwm1 = p_ClipMinMaxU16(u16Pwm1, C_PWM_MIN_DC, l_u16MaxPwmRatio);
#endif /* (_SUPPORT_ANTICOGGING != FALSE) */
#if (_SUPPORT_PWM_POL_CORR == FALSE)
                    if (l_u16PwmState != FALSE)
                    {
                        IO_PWM_SLAVE2_CTRL = B_PWM_SLAVE2_SLAVE | C_PWM_SLAVE2_MODE_MIRROR;
                        IO_PWM_SLAVE2_LT = (PWM_SCALE_OFFSET - u16Pwm1);
                    }
                    else
                    {
                        IO_PWM_SLAVE2_CTRL = B_PWM_SLAVE2_SLAVE | C_PWM_SLAVE2_MODE_MIRROR | B_PWM_SLAVE2_POL;
                        IO_PWM_SLAVE2_LT = u16Pwm1;
                    }
#else  /* (_SUPPORT_PWM_POL_CORR == FALSE) */
                    IO_PWM_SLAVE2_LT = u16Pwm1;
#endif /* (_SUPPORT_PWM_POL_CORR == FALSE) */

                    /* U */
#if (_SUPPORT_PWM_POL_CORR == FALSE)
                    IO_PWM_MASTER1_CTRL = (PWM_PRESCALER << 8) | C_PWM_MASTER1_MODE_MIRROR | B_PWM_MASTER1_POL;
                    u16Pwm1 = 0U;
#else  /* (_SUPPORT_PWM_POL_CORR == FALSE) */
                    u16Pwm1 = PWM_REG_PERIOD;
#endif /* (_SUPPORT_PWM_POL_CORR == FALSE) */
                }
                break;
            }
        }
#if (_SUPPORT_BOOST_MODE == BOOST_MODE_WAVEFORM)
        else
        {
            uint16_t u16Pwm2;
            uint16_t u16CorrectionRatio = l_u16CorrectionRatio;
            switch (u16Phase)
            {
                default:
                {
                    /* Small W-phase low period --> W-phase to Low */
                    if (u16CorrectionRatio >= l_u16MaxPwmCorrRatioBoost)
                    {
                        u16Pwm1 = p_MulU16hi_pU16byU16lsr4(pu16BoostVector, l_u16MaxPwmCorrRatio);  /* U */
                        pu16BoostVector += (2U * C_MICROSTEP_PER_FULLSTEP);
                        u16Pwm2 = p_MulU16hi_pU16byU16lsr4(pu16BoostVector, l_u16MaxPwmCorrRatio);  /* V */
                    }
                    else /* l_u16MaxPwmCorrRatio < u16CorrectionRatio < l_u16MaxPwmCorrRatioBoost */
                    {
                        u16Pwm1 = p_MulDivU16_U16byU16byU16(*pu16Vector,
                                                            (l_u16MaxPwmCorrRatioBoost - u16CorrectionRatio),
                                                            (l_u16MaxPwmCorrRatioBoost - l_u16MaxPwmCorrRatio));
                        u16Pwm1 += p_MulDivU16_U16byU16byU16(*pu16BoostVector,
                                                             (u16CorrectionRatio - l_u16MaxPwmCorrRatio),
                                                             (l_u16MaxPwmCorrRatioBoost - l_u16MaxPwmCorrRatio));
                        u16Pwm1 = p_MulU16hi_U16byU16lsr4(u16Pwm1, l_u16MaxPwmCorrRatio);  /* U */
                        pu16Vector += (2U * C_MICROSTEP_PER_FULLSTEP);
                        pu16BoostVector += (2U * C_MICROSTEP_PER_FULLSTEP);
                        u16Pwm2 = p_MulDivU16_U16byU16byU16(*pu16Vector,
                                                            (l_u16MaxPwmCorrRatioBoost - u16CorrectionRatio),
                                                            (l_u16MaxPwmCorrRatioBoost - l_u16MaxPwmCorrRatio));
                        u16Pwm2 += p_MulDivU16_U16byU16byU16(*pu16BoostVector,
                                                             (u16CorrectionRatio - l_u16MaxPwmCorrRatio),
                                                             (l_u16MaxPwmCorrRatioBoost - l_u16MaxPwmCorrRatio));
                        u16Pwm2 = p_MulU16hi_U16byU16lsr4(u16Pwm2, l_u16MaxPwmCorrRatio);  /* V */
                    }
                    if (u16Pwm1 < C_PWM_MIN_DC)
                    {
                        u16Pwm1 = C_PWM_MIN_DC;
                    }
                    u16Pwm1 = (PWM_SCALE_OFFSET - u16Pwm1);
                    IO_PWM_SLAVE2_LT = 0U;                                      /* W */
                    if (u16Pwm2 < C_PWM_MIN_DC)
                    {
                        u16Pwm2 = C_PWM_MIN_DC;
                    }
                    IO_PWM_SLAVE1_CTRL = B_PWM_SLAVE1_SLAVE | C_PWM_SLAVE1_MODE_MIRROR | B_PWM_SLAVE1_POL;
                    IO_PWM_SLAVE1_LT = u16Pwm2;
                }
                break;
                case (2U * C_MICROSTEP_PER_FULLSTEP):
                {
                    /* Small V-phase low period --> V-phase to Low */
                    if (u16CorrectionRatio >= l_u16MaxPwmCorrRatioBoost)
                    {
                        u16Pwm1 = p_MulU16hi_pU16byU16lsr4(pu16BoostVector, l_u16MaxPwmCorrRatio);  /* U */
                        pu16BoostVector += (4U * C_MICROSTEP_PER_FULLSTEP);
                        u16Pwm2 = p_MulU16hi_pU16byU16lsr4(pu16BoostVector, l_u16MaxPwmCorrRatio);  /* V */
                    }
                    else
                    {
                        u16Pwm1 = p_MulDivU16_U16byU16byU16(*pu16Vector,
                                                            (l_u16MaxPwmCorrRatioBoost - u16CorrectionRatio),
                                                            (l_u16MaxPwmCorrRatioBoost - l_u16MaxPwmCorrRatio));
                        u16Pwm1 += p_MulDivU16_U16byU16byU16(*pu16BoostVector,
                                                             (u16CorrectionRatio - l_u16MaxPwmCorrRatio),
                                                             (l_u16MaxPwmCorrRatioBoost - l_u16MaxPwmCorrRatio));
                        u16Pwm1 = p_MulU16hi_U16byU16lsr4(u16Pwm1, l_u16MaxPwmCorrRatio);  /* U */
                        pu16Vector += (4U * C_MICROSTEP_PER_FULLSTEP);
                        pu16BoostVector += (4U * C_MICROSTEP_PER_FULLSTEP);
                        u16Pwm2 = p_MulDivU16_U16byU16byU16(*pu16Vector,
                                                            (l_u16MaxPwmCorrRatioBoost - u16CorrectionRatio),
                                                            (l_u16MaxPwmCorrRatioBoost - l_u16MaxPwmCorrRatio));
                        u16Pwm2 += p_MulDivU16_U16byU16byU16(*pu16BoostVector,
                                                             (u16CorrectionRatio - l_u16MaxPwmCorrRatio),
                                                             (l_u16MaxPwmCorrRatioBoost - l_u16MaxPwmCorrRatio));
                        u16Pwm2 = p_MulU16hi_U16byU16lsr4(u16Pwm2, l_u16MaxPwmCorrRatio);  /* U */
                    }
                    if (u16Pwm1 < C_PWM_MIN_DC)
                    {
                        u16Pwm1 = C_PWM_MIN_DC;
                    }
                    u16Pwm1 = (PWM_SCALE_OFFSET - u16Pwm1);
                    IO_PWM_SLAVE1_CTRL = B_PWM_SLAVE1_SLAVE | C_PWM_SLAVE1_MODE_MIRROR | B_PWM_SLAVE1_POL;
                    IO_PWM_SLAVE1_LT = 0U;                                      /* V */
                    if (u16Pwm2 < C_PWM_MIN_DC)
                    {
                        u16Pwm2 = C_PWM_MIN_DC;
                    }
                    IO_PWM_SLAVE2_LT = u16Pwm2;
                }
                break;
                case (4U * C_MICROSTEP_PER_FULLSTEP):
                {
                    /* Small U-phase low period --> U-phase to Low */
                    pu16Vector += (2U * C_MICROSTEP_PER_FULLSTEP);
                    pu16BoostVector += (2U * C_MICROSTEP_PER_FULLSTEP);
                    if (u16CorrectionRatio >= l_u16MaxPwmCorrRatioBoost)
                    {
                        u16Pwm1 = p_MulU16hi_pU16byU16lsr4(pu16BoostVector, l_u16MaxPwmCorrRatio);  /* V */
                        pu16BoostVector += (2U * C_MICROSTEP_PER_FULLSTEP);
                        u16Pwm2 = p_MulU16hi_pU16byU16lsr4(pu16BoostVector, l_u16MaxPwmCorrRatio);  /* V */
                    }
                    else
                    {
                        u16Pwm1 = p_MulDivU16_U16byU16byU16(*pu16Vector,
                                                            (l_u16MaxPwmCorrRatioBoost - u16CorrectionRatio),
                                                            (l_u16MaxPwmCorrRatioBoost - l_u16MaxPwmCorrRatio));
                        u16Pwm1 += p_MulDivU16_U16byU16byU16(*pu16BoostVector,
                                                             (u16CorrectionRatio - l_u16MaxPwmCorrRatio),
                                                             (l_u16MaxPwmCorrRatioBoost - l_u16MaxPwmCorrRatio));
                        u16Pwm1 = p_MulU16hi_U16byU16lsr4(u16Pwm1, l_u16MaxPwmCorrRatio);  /* V */
                        pu16Vector += (2U * C_MICROSTEP_PER_FULLSTEP);
                        pu16BoostVector += (2U * C_MICROSTEP_PER_FULLSTEP);
                        u16Pwm2 = p_MulDivU16_U16byU16byU16(*pu16Vector,
                                                            (l_u16MaxPwmCorrRatioBoost - u16CorrectionRatio),
                                                            (l_u16MaxPwmCorrRatioBoost - l_u16MaxPwmCorrRatio));
                        u16Pwm2 += p_MulDivU16_U16byU16byU16(*pu16BoostVector,
                                                             (u16CorrectionRatio - l_u16MaxPwmCorrRatio),
                                                             (l_u16MaxPwmCorrRatioBoost - l_u16MaxPwmCorrRatio));
                        u16Pwm2 = p_MulU16hi_U16byU16lsr4(u16Pwm2, l_u16MaxPwmCorrRatio);  /* W */
                    }
                    if (u16Pwm1 < C_PWM_MIN_DC)
                    {
                        u16Pwm1 = C_PWM_MIN_DC;
                    }
                    IO_PWM_SLAVE1_CTRL = B_PWM_SLAVE1_SLAVE | C_PWM_SLAVE1_MODE_MIRROR;
                    IO_PWM_SLAVE1_LT = (PWM_SCALE_OFFSET - u16Pwm1);
                    u16Pwm1 = PWM_REG_PERIOD;                                   /* U */
                    if (u16Pwm2 < C_PWM_MIN_DC)
                    {
                        u16Pwm2 = C_PWM_MIN_DC;
                    }
                    IO_PWM_SLAVE2_LT = u16Pwm2;
                }
                break;
            }
        }
#endif /* (_SUPPORT_BOOST_MODE == BOOST_MODE_CLIPPING) */
#else  /* (_SUPPORT_VOLTAGE_SHAPE != VOLTAGE_SHAPE_DSVM_ALT) */ /* MMP210524-1 */
        u16MicroStepIdx = (u16MicroStepIdx / C_MICROSTEP_PER_FULLSTEP);

        /* Perform a synchronically PWM update in STEPPER mode, especially for a POLarity change; In FOC mode automatically by 'design' */
        if ( ((l_e8MotorStartupMode & E_MSM_MODE_MASK) == E_MSM_STEPPER) &&
             (u16MicroStepIdx != l_u16PrevMicroStepIdx) )
        {
            HAL_PWM_MasterPendClear();
            l_u16PrevMicroStepIdx = u16MicroStepIdx;
            HAL_PWM_MasterPendWait();
        }

#if (_SUPPORT_PWM_MIRROR != FALSE)
        if ((u16MicroStepIdx == 0U) || (u16MicroStepIdx == 3U))                 /* MMP230804-1 */
        {
            /* MCurA @  50%: V-phase
             * MCurB @ 100%: U-phase
             */
            if (u16MicroStepIdx == 0U)
            {
                IO_PWM_SLAVE2_CTRL = B_PWM_SLAVE2_SLAVE | C_PWM_SLAVE2_MODE_MIRROR | B_PWM_SLAVE2_POL;
                IO_PWM_SLAVE1_CTRL = B_PWM_SLAVE1_SLAVE | C_PWM_SLAVE1_MODE_MIRROR;
                IO_PWM_MASTER1_CTRL = (PWM_PRESCALER << 8) | C_PWM_MASTER1_MODE_MIRROR | B_PWM_MASTER1_POL;
            }
            else
            {
                /* W */
                IO_PWM_SLAVE2_CTRL = B_PWM_SLAVE2_SLAVE | C_PWM_SLAVE2_MODE_MIRROR;
                IO_PWM_SLAVE1_CTRL = B_PWM_SLAVE1_SLAVE | C_PWM_SLAVE1_MODE_MIRROR | B_PWM_SLAVE1_POL;
                IO_PWM_MASTER1_CTRL = (PWM_PRESCALER << 8) | C_PWM_MASTER1_MODE_MIRROR;
            }
            /* Small W-phase low period --> W-phase to Low */
            /* W */
            IO_PWM_SLAVE2_LT = 0U; /* Low */
            /* V */
#if (C_PID_FACTOR == 4U)
            u16Pwm1 = p_MulU16hi_pU16byU16lsr4( (pu16Vector + (2U * C_MICROSTEP_PER_FULLSTEP)), l_u16CorrectionRatio); /* V */
#elif (C_PID_FACTOR == 3U)
            u16Pwm1 = p_MulU16hi_pU16byU16lsr3( (pu16Vector + (2U * C_MICROSTEP_PER_FULLSTEP)), l_u16CorrectionRatio); /* V */
#else
#error "No PWM-W calculation"
#endif
#if (PWM_LIMIT == PWM_LIMIT_BY_PWM)
            u16Pwm1 = p_ClipMinMaxU16(u16Pwm1, C_PWM_MIN_DC, l_u16MaxPwmRatio);
#endif /* (PWM_LIMIT == PWM_LIMIT_BY_PWM) */
            IO_PWM_SLAVE1_LT = (PWM_SCALE_OFFSET - u16Pwm1);                    /* Centred around 50% */
            /* U */
#if (C_PID_FACTOR == 4U)
            u16Pwm1 = p_MulU16hi_pU16byU16lsr4(pu16Vector, l_u16CorrectionRatio);   /* U */
#elif (C_PID_FACTOR == 3U)
            u16Pwm1 = p_MulU16hi_pU16byU16lsr3(pu16Vector, l_u16CorrectionRatio);   /* U */
#else
#error "No PWM-U calculation"
#endif
#if (PWM_LIMIT == PWM_LIMIT_BY_PWM)
            u16Pwm1 = p_ClipMinMaxU16(u16Pwm1, C_PWM_MIN_DC, l_u16MaxPwmRatio);
#endif /* (PWM_LIMIT == PWM_LIMIT_BY_PWM) */
        }
        else if ((u16MicroStepIdx == 1U) || (u16MicroStepIdx == 4U))
        {
            /* MCurA @  50%: W-phase
             * MCurB @ 100%: V-phase
             */
            if (u16MicroStepIdx == 1U)
            {
                IO_PWM_SLAVE1_CTRL = B_PWM_SLAVE1_SLAVE | C_PWM_SLAVE1_MODE_MIRROR;
                IO_PWM_SLAVE2_CTRL = B_PWM_SLAVE2_SLAVE | C_PWM_SLAVE2_MODE_MIRROR | B_PWM_SLAVE2_POL;
                IO_PWM_MASTER1_CTRL = (PWM_PRESCALER << 8) | C_PWM_MASTER1_MODE_MIRROR;
            }
            else
            {
                IO_PWM_SLAVE1_CTRL = B_PWM_SLAVE1_SLAVE | C_PWM_SLAVE1_MODE_MIRROR | B_PWM_SLAVE1_POL;
                IO_PWM_SLAVE2_CTRL = B_PWM_SLAVE2_SLAVE | C_PWM_SLAVE2_MODE_MIRROR;
                IO_PWM_MASTER1_CTRL = (PWM_PRESCALER << 8) | C_PWM_MASTER1_MODE_MIRROR | B_PWM_MASTER1_POL;
            }
            /* V */
            pu16Vector += (2U * C_MICROSTEP_PER_FULLSTEP);
#if (C_PID_FACTOR == 4U)
            u16Pwm1 = p_MulU16hi_pU16byU16lsr4(pu16Vector, l_u16CorrectionRatio);  /* V */
#elif (C_PID_FACTOR == 3U)
            u16Pwm1 = p_MulU16hi_pU16byU16lsr3(pu16Vector, l_u16CorrectionRatio);  /* V */
#else
#error "No PWM-V calculation"
#endif
#if (PWM_LIMIT == PWM_LIMIT_BY_PWM)
            u16Pwm1 = p_ClipMinMaxU16(u16Pwm1, C_PWM_MIN_DC, l_u16MaxPwmRatio);
#endif /* (PWM_LIMIT == PWM_LIMIT_BY_PWM) */
            IO_PWM_SLAVE1_LT = u16Pwm1;
            /* W */
            pu16Vector += (2U * C_MICROSTEP_PER_FULLSTEP);
#if (C_PID_FACTOR == 4U)
            u16Pwm1 = p_MulU16hi_pU16byU16lsr4(pu16Vector, l_u16CorrectionRatio);  /* W */
#elif (C_PID_FACTOR == 3U)
            u16Pwm1 = p_MulU16hi_pU16byU16lsr3(pu16Vector, l_u16CorrectionRatio);  /* W */
#else
#error "No PWM-W calculation"
#endif
#if (PWM_LIMIT == PWM_LIMIT_BY_PWM)
            u16Pwm1 = p_ClipMinMaxU16(u16Pwm1, C_PWM_MIN_DC, l_u16MaxPwmRatio);
#endif /* (PWM_LIMIT == PWM_LIMIT_BY_PWM) */
            IO_PWM_SLAVE2_LT = (uint16_t)(PWM_SCALE_OFFSET - u16Pwm1);
            /* U */
            u16Pwm1 = 0U; /* High */
        }
        else
        {
            /* MCurA @  50%: U-phase
             * MCurB @ 100%: W-phase
             */
            if (u16MicroStepIdx == 2U)
            {
                IO_PWM_SLAVE2_CTRL = B_PWM_SLAVE2_SLAVE | C_PWM_SLAVE2_MODE_MIRROR | B_PWM_SLAVE2_POL;
                IO_PWM_SLAVE1_CTRL = B_PWM_SLAVE1_SLAVE | C_PWM_SLAVE1_MODE_MIRROR | B_PWM_SLAVE1_POL;
                IO_PWM_MASTER1_CTRL = (PWM_PRESCALER << 8) | C_PWM_MASTER1_MODE_MIRROR;
            }
            else
            {
                IO_PWM_SLAVE2_CTRL = B_PWM_SLAVE2_SLAVE | C_PWM_SLAVE2_MODE_MIRROR;
                IO_PWM_SLAVE1_CTRL = B_PWM_SLAVE1_SLAVE | C_PWM_SLAVE1_MODE_MIRROR;
                IO_PWM_MASTER1_CTRL = (PWM_PRESCALER << 8) | C_PWM_MASTER1_MODE_MIRROR | B_PWM_MASTER1_POL;
            }
#if (C_PID_FACTOR == 4U)
            u16Pwm1 = p_MulU16hi_pU16byU16lsr4( (pu16Vector + (4U * C_MICROSTEP_PER_FULLSTEP)), l_u16CorrectionRatio); /* W */
#elif (C_PID_FACTOR == 3U)
            u16Pwm1 = p_MulU16hi_pU16byU16lsr3( (pu16Vector + (4U * C_MICROSTEP_PER_FULLSTEP)), l_u16CorrectionRatio); /* W */
#else
#error "No PWM-W calculation"
#endif
#if (PWM_LIMIT == PWM_LIMIT_BY_PWM)
            u16Pwm1 = p_ClipMinMaxU16(u16Pwm1, C_PWM_MIN_DC, l_u16MaxPwmRatio);
#endif /* (PWM_LIMIT == PWM_LIMIT_BY_PWM) */
            IO_PWM_SLAVE2_LT = u16Pwm1;
            /* V */
            IO_PWM_SLAVE1_LT = 0U; /* Low */
            /* U */
#if (C_PID_FACTOR == 4U)
            u16Pwm1 = p_MulU16hi_pU16byU16lsr4(pu16Vector, l_u16CorrectionRatio);  /* U */
#elif (C_PID_FACTOR == 3U)
            u16Pwm1 = p_MulU16hi_pU16byU16lsr3(pu16Vector, l_u16CorrectionRatio);  /* U */
#else
#error "No PWM-U calculation"
#endif
#if (PWM_LIMIT == PWM_LIMIT_BY_PWM)
            u16Pwm1 = p_ClipMinMaxU16(u16Pwm1, C_PWM_MIN_DC, l_u16MaxPwmRatio);
#endif /* (PWM_LIMIT == PWM_LIMIT_BY_PWM) */
            u16Pwm1 = (PWM_SCALE_OFFSET - u16Pwm1);
        }
#else  /* (_SUPPORT_PWM_MIRROR != FALSE) */
        if ((u16MicroStepIdx == 0U) || (u16MicroStepIdx == 3U))                 /* MMP230804-1 */
        {
            /* MCurA @  50%: V-phase
             * MCurB @ 100%: U-phase
             */
            if (u16MicroStepIdx == 0U)
            {
                IO_PWM_SLAVE2_CTRL = B_PWM_SLAVE2_SLAVE | C_PWM_SLAVE2_MODE_INDEPENDENT | B_PWM_SLAVE2_POL;
                IO_PWM_SLAVE1_CTRL = B_PWM_SLAVE1_SLAVE | C_PWM_SLAVE1_MODE_INDEPENDENT;
                IO_PWM_MASTER1_CTRL = (PWM_PRESCALER << 8) | C_PWM_MASTER1_MODE_INDEPENDENT | B_PWM_MASTER1_POL;
            }
            else
            {
                /* W */
                IO_PWM_SLAVE2_CTRL = B_PWM_SLAVE2_SLAVE | C_PWM_SLAVE2_MODE_INDEPENDENT;
                IO_PWM_SLAVE1_CTRL = B_PWM_SLAVE1_SLAVE | C_PWM_SLAVE1_MODE_INDEPENDENT | B_PWM_SLAVE1_POL;
                IO_PWM_MASTER1_CTRL = (PWM_PRESCALER << 8) | C_PWM_MASTER1_MODE_INDEPENDENT;
            }
            /* Small W-phase low period --> W-phase to Low */
            /* W */
            IO_PWM_SLAVE2_HT = PWM_REG_PERIOD; /* Low */
            IO_PWM_SLAVE2_LT = 0U; /* Low */
            /* V */
#if (C_PID_FACTOR == 4U)
            u16Pwm1 = p_MulU16hi_pU16byU16lsr3( (pu16Vector + (2U * C_MICROSTEP_PER_FULLSTEP)), l_u16CorrectionRatio); /* V */
#elif (C_PID_FACTOR == 3U)
            u16Pwm1 = p_MulU16hi_pU16byU16lsr2( (pu16Vector + (2U * C_MICROSTEP_PER_FULLSTEP)), l_u16CorrectionRatio); /* V */
#else
#error "No PWM-W calculation"
#endif
#if (PWM_LIMIT == PWM_LIMIT_BY_PWM)
            u16Pwm1 = p_ClipMinMaxU16(u16Pwm1, C_PWM_MIN_DC, l_u16MaxPwmRatio);
#endif /* (PWM_LIMIT == PWM_LIMIT_BY_PWM) */
            {
                uint16_t u16HT = (PWM_SCALE_OFFSET + (u16Pwm1 / 2U));
                IO_PWM_SLAVE1_HT = u16HT;
                IO_PWM_SLAVE1_LT = (u16HT - u16Pwm1);
            }
            /* U */
#if (C_PID_FACTOR == 4U)
            u16Pwm1 = p_MulU16hi_pU16byU16lsr3(pu16Vector, l_u16CorrectionRatio);   /* U */
#elif (C_PID_FACTOR == 3U)
            u16Pwm1 = p_MulU16hi_pU16byU16lsr2(pu16Vector, l_u16CorrectionRatio);   /* U */
#else
#error "No PWM-U calculation"
#endif
#if (PWM_LIMIT == PWM_LIMIT_BY_PWM)
            u16Pwm1 = p_ClipMinMaxU16(u16Pwm1, C_PWM_MIN_DC, l_u16MaxPwmRatio);
#endif /* (PWM_LIMIT == PWM_LIMIT_BY_PWM) */
            {
                uint16_t u16HT = (PWM_REG_PERIOD - (u16Pwm1 / 2U));
                IO_PWM_MASTER1_HT = u16HT;
                IO_PWM_MASTER1_LT = (u16HT - (PWM_REG_PERIOD - u16Pwm1));
            }
        }
        else if ((u16MicroStepIdx == 1U) || (u16MicroStepIdx == 4U))
        {
            /* MCurA @  50%: W-phase
             * MCurB @ 100%: V-phase
             */
            if (u16MicroStepIdx == 1U)
            {
                IO_PWM_SLAVE1_CTRL = B_PWM_SLAVE1_SLAVE | C_PWM_SLAVE1_MODE_INDEPENDENT;
                IO_PWM_SLAVE2_CTRL = B_PWM_SLAVE2_SLAVE | C_PWM_SLAVE2_MODE_INDEPENDENT | B_PWM_SLAVE2_POL;
                IO_PWM_MASTER1_CTRL = (PWM_PRESCALER << 8) | C_PWM_MASTER1_MODE_INDEPENDENT;
            }
            else
            {
                IO_PWM_SLAVE1_CTRL = B_PWM_SLAVE1_SLAVE | C_PWM_SLAVE1_MODE_INDEPENDENT | B_PWM_SLAVE1_POL;
                IO_PWM_SLAVE2_CTRL = B_PWM_SLAVE2_SLAVE | C_PWM_SLAVE2_MODE_INDEPENDENT;
                IO_PWM_MASTER1_CTRL = (PWM_PRESCALER << 8) | C_PWM_MASTER1_MODE_INDEPENDENT | B_PWM_MASTER1_POL;
            }
            /* V */
            pu16Vector += (2U * C_MICROSTEP_PER_FULLSTEP);
#if (C_PID_FACTOR == 4U)
            u16Pwm1 = p_MulU16hi_pU16byU16lsr3(pu16Vector, l_u16CorrectionRatio);  /* V */
#elif (C_PID_FACTOR == 3U)
            u16Pwm1 = p_MulU16hi_pU16byU16lsr2(pu16Vector, l_u16CorrectionRatio);  /* V */
#else
#error "No PWM-V calculation"
#endif
#if (PWM_LIMIT == PWM_LIMIT_BY_PWM)
            u16Pwm1 = p_ClipMinMaxU16(u16Pwm1, C_PWM_MIN_DC, l_u16MaxPwmRatio);
#endif /* (PWM_LIMIT == PWM_LIMIT_BY_PWM) */
            {
                uint16_t u16HT = (PWM_REG_PERIOD - (u16Pwm1 / 2U));
                IO_PWM_SLAVE1_HT = u16HT;
                IO_PWM_SLAVE1_LT = (u16HT - (PWM_REG_PERIOD - u16Pwm1));
            }
            /* W */
            pu16Vector += (2U * C_MICROSTEP_PER_FULLSTEP);
#if (C_PID_FACTOR == 4U)
            u16Pwm1 = p_MulU16hi_pU16byU16lsr3(pu16Vector, l_u16CorrectionRatio);  /* W */
#elif (C_PID_FACTOR == 3U)
            u16Pwm1 = p_MulU16hi_pU16byU16lsr2(pu16Vector, l_u16CorrectionRatio);  /* W */
#else
#error "No PWM-W calculation"
#endif
#if (PWM_LIMIT == PWM_LIMIT_BY_PWM)
            u16Pwm1 = p_ClipMinMaxU16(u16Pwm1, C_PWM_MIN_DC, l_u16MaxPwmRatio);
#endif /* (PWM_LIMIT == PWM_LIMIT_BY_PWM) */
            {
                uint16_t u16HT = (PWM_SCALE_OFFSET + (u16Pwm1 / 2U));
                IO_PWM_SLAVE2_HT = u16HT;
                IO_PWM_SLAVE2_LT = (u16HT - u16Pwm1);
            }
            /* U */
            IO_PWM_MASTER1_HT = PWM_REG_PERIOD;
            IO_PWM_MASTER1_LT = 0U;
        }
        else
        {
            /* MCurA @  50%: U-phase
             * MCurB @ 100%: W-phase
             */
            if (u16MicroStepIdx == 2U)
            {
                IO_PWM_SLAVE2_CTRL = B_PWM_SLAVE2_SLAVE | C_PWM_SLAVE2_MODE_INDEPENDENT | B_PWM_SLAVE2_POL;
                IO_PWM_SLAVE1_CTRL = B_PWM_SLAVE1_SLAVE | C_PWM_SLAVE1_MODE_INDEPENDENT | B_PWM_SLAVE1_POL;
                IO_PWM_MASTER1_CTRL = (PWM_PRESCALER << 8) | C_PWM_MASTER1_MODE_INDEPENDENT;
            }
            else
            {
                IO_PWM_SLAVE2_CTRL = B_PWM_SLAVE2_SLAVE | C_PWM_SLAVE2_MODE_INDEPENDENT;
                IO_PWM_SLAVE1_CTRL = B_PWM_SLAVE1_SLAVE | C_PWM_SLAVE1_MODE_INDEPENDENT;
                IO_PWM_MASTER1_CTRL = (PWM_PRESCALER << 8) | C_PWM_MASTER1_MODE_INDEPENDENT | B_PWM_MASTER1_POL;
            }
#if (C_PID_FACTOR == 4U)
            u16Pwm1 = p_MulU16hi_pU16byU16lsr3( (pu16Vector + (4U * C_MICROSTEP_PER_FULLSTEP)), l_u16CorrectionRatio); /* W */
#elif (C_PID_FACTOR == 3U)
            u16Pwm1 = p_MulU16hi_pU16byU16lsr2( (pu16Vector + (4U * C_MICROSTEP_PER_FULLSTEP)), l_u16CorrectionRatio); /* W */
#else
#error "No PWM-W calculation"
#endif
#if (PWM_LIMIT == PWM_LIMIT_BY_PWM)
            u16Pwm1 = p_ClipMinMaxU16(u16Pwm1, C_PWM_MIN_DC, l_u16MaxPwmRatio);
#endif /* (PWM_LIMIT == PWM_LIMIT_BY_PWM) */
            {
                uint16_t u16HT = (PWM_REG_PERIOD - (u16Pwm1 / 2U));
                IO_PWM_SLAVE2_HT = u16HT;
                IO_PWM_SLAVE2_LT = (u16HT - (PWM_REG_PERIOD - u16Pwm1));
            }
            /* V */
            IO_PWM_SLAVE1_HT = PWM_REG_PERIOD;
            IO_PWM_SLAVE1_LT = 0U;
            /* U */
#if (C_PID_FACTOR == 4U)
            u16Pwm1 = p_MulU16hi_pU16byU16lsr3(pu16Vector, l_u16CorrectionRatio);  /* U */
#elif (C_PID_FACTOR == 3U)
            u16Pwm1 = p_MulU16hi_pU16byU16lsr2(pu16Vector, l_u16CorrectionRatio);  /* U */
#else
#error "No PWM-U calculation"
#endif
#if (PWM_LIMIT == PWM_LIMIT_BY_PWM)
            u16Pwm1 = p_ClipMinMaxU16(u16Pwm1, C_PWM_MIN_DC, l_u16MaxPwmRatio);
#endif /* (PWM_LIMIT == PWM_LIMIT_BY_PWM) */
            {
                uint16_t u16HT = (PWM_SCALE_OFFSET + (u16Pwm1 / 2U));
                IO_PWM_MASTER1_HT = u16HT;
                IO_PWM_MASTER1_LT = (u16HT - u16Pwm1);
            }
        }
#endif /* (_SUPPORT_PWM_MIRROR != FALSE) */
#if FALSE
        if (u16MicroStepIdx == 0U)
        {
            /* Small W-phase low period --> W-phase to Low */
            /* W */
            IO_PWM_SLAVE2_CTRL = B_PWM_SLAVE2_SLAVE | C_PWM_SLAVE2_MODE_MIRROR | B_PWM_SLAVE2_POL;
            IO_PWM_SLAVE2_LT = 0U; /* Low */
            /* V */
            IO_PWM_SLAVE1_CTRL = B_PWM_SLAVE1_SLAVE | C_PWM_SLAVE1_MODE_MIRROR;
#if (C_PID_FACTOR == 4U)
            u16Pwm1 = p_MulU16hi_pU16byU16lsr4( (pu16Vector + (2U * C_MICROSTEP_PER_FULLSTEP)), l_u16CorrectionRatio); /* V */
#elif (C_PID_FACTOR == 3U)
            u16Pwm1 = p_MulU16hi_pU16byU16lsr3( (pu16Vector + (2U * C_MICROSTEP_PER_FULLSTEP)), l_u16CorrectionRatio); /* V */
#else
#error "No PWM-W calculation"
#endif
#if (PWM_LIMIT == PWM_LIMIT_BY_PWM)
            u16Pwm1 = p_ClipMinMaxU16(u16Pwm1, C_PWM_MIN_DC, l_u16MaxPwmRatio);
#endif /* (PWM_LIMIT == PWM_LIMIT_BY_PWM) */
            IO_PWM_SLAVE1_LT = (PWM_SCALE_OFFSET - u16Pwm1);
            /* U */
            IO_PWM_MASTER1_CTRL = (PWM_PRESCALER << 8) | C_PWM_MASTER1_MODE_MIRROR | B_PWM_MASTER1_POL;
#if (C_PID_FACTOR == 4U)
            u16Pwm1 = p_MulU16hi_pU16byU16lsr4(pu16Vector, l_u16CorrectionRatio);   /* U */
#elif (C_PID_FACTOR == 3U)
            u16Pwm1 = p_MulU16hi_pU16byU16lsr3(pu16Vector, l_u16CorrectionRatio);   /* U */
#else
#error "No PWM-U calculation"
#endif
#if (PWM_LIMIT == PWM_LIMIT_BY_PWM)
            u16Pwm1 = p_ClipMinMaxU16(u16Pwm1, C_PWM_MIN_DC, l_u16MaxPwmRatio);
#endif /* (PWM_LIMIT == PWM_LIMIT_BY_PWM) */
        }
        else if (u16MicroStepIdx == 1U)
        {
            /* V */
            IO_PWM_SLAVE1_CTRL = B_PWM_SLAVE1_SLAVE | C_PWM_SLAVE1_MODE_MIRROR;
            pu16Vector += (2U * C_MICROSTEP_PER_FULLSTEP);
#if (C_PID_FACTOR == 4U)
            u16Pwm1 = p_MulU16hi_pU16byU16lsr4(pu16Vector, l_u16CorrectionRatio);  /* V */
#elif (C_PID_FACTOR == 3U)
            u16Pwm1 = p_MulU16hi_pU16byU16lsr3(pu16Vector, l_u16CorrectionRatio);  /* V */
#else
#error "No PWM-V calculation"
#endif
#if (PWM_LIMIT == PWM_LIMIT_BY_PWM)
            u16Pwm1 = p_ClipMinMaxU16(u16Pwm1, C_PWM_MIN_DC, l_u16MaxPwmRatio);
#endif /* (PWM_LIMIT == PWM_LIMIT_BY_PWM) */
            IO_PWM_SLAVE1_LT = u16Pwm1;
            /* W */
            IO_PWM_SLAVE2_CTRL = B_PWM_SLAVE2_SLAVE | C_PWM_SLAVE2_MODE_MIRROR | B_PWM_SLAVE2_POL;
            pu16Vector += (2U * C_MICROSTEP_PER_FULLSTEP);
#if (C_PID_FACTOR == 4U)
            u16Pwm1 = p_MulU16hi_pU16byU16lsr4(pu16Vector, l_u16CorrectionRatio);  /* W */
#elif (C_PID_FACTOR == 3U)
            u16Pwm1 = p_MulU16hi_pU16byU16lsr3(pu16Vector, l_u16CorrectionRatio);  /* W */
#else
#error "No PWM-W calculation"
#endif
#if (PWM_LIMIT == PWM_LIMIT_BY_PWM)
            u16Pwm1 = p_ClipMinMaxU16(u16Pwm1, C_PWM_MIN_DC, l_u16MaxPwmRatio);
#endif /* (PWM_LIMIT == PWM_LIMIT_BY_PWM) */
            IO_PWM_SLAVE2_LT = (uint16_t)(PWM_SCALE_OFFSET - u16Pwm1);
            /* U */
            IO_PWM_MASTER1_CTRL = (PWM_PRESCALER << 8) | C_PWM_MASTER1_MODE_MIRROR;
            u16Pwm1 = 0U; /* High */
        }
        else if (u16MicroStepIdx == 2U)
        {
            /* W */
            IO_PWM_SLAVE2_CTRL = B_PWM_SLAVE2_SLAVE | C_PWM_SLAVE2_MODE_MIRROR | B_PWM_SLAVE2_POL;
#if (C_PID_FACTOR == 4U)
            u16Pwm1 = p_MulU16hi_pU16byU16lsr4( (pu16Vector + (4U * C_MICROSTEP_PER_FULLSTEP)), l_u16CorrectionRatio); /* W */
#elif (C_PID_FACTOR == 3U)
            u16Pwm1 = p_MulU16hi_pU16byU16lsr3( (pu16Vector + (4U * C_MICROSTEP_PER_FULLSTEP)), l_u16CorrectionRatio); /* W */
#else
#error "No PWM-W calculation"
#endif
#if (PWM_LIMIT == PWM_LIMIT_BY_PWM)
            u16Pwm1 = p_ClipMinMaxU16(u16Pwm1, C_PWM_MIN_DC, l_u16MaxPwmRatio);
#endif /* (PWM_LIMIT == PWM_LIMIT_BY_PWM) */
            IO_PWM_SLAVE2_LT = u16Pwm1;
            /* V */
            IO_PWM_SLAVE1_CTRL = B_PWM_SLAVE1_SLAVE | C_PWM_SLAVE1_MODE_MIRROR | B_PWM_SLAVE1_POL;
            IO_PWM_SLAVE1_LT = 0U; /* Low */
            /* U */
            IO_PWM_MASTER1_CTRL = (PWM_PRESCALER << 8) | C_PWM_MASTER1_MODE_MIRROR;
#if (C_PID_FACTOR == 4U)
            u16Pwm1 = p_MulU16hi_pU16byU16lsr4(pu16Vector, l_u16CorrectionRatio);  /* U */
#elif (C_PID_FACTOR == 3U)
            u16Pwm1 = p_MulU16hi_pU16byU16lsr3(pu16Vector, l_u16CorrectionRatio);  /* U */
#else
#error "No PWM-U calculation"
#endif
#if (PWM_LIMIT == PWM_LIMIT_BY_PWM)
            u16Pwm1 = p_ClipMinMaxU16(u16Pwm1, C_PWM_MIN_DC, l_u16MaxPwmRatio);
#endif /* (PWM_LIMIT == PWM_LIMIT_BY_PWM) */
            u16Pwm1 = (PWM_SCALE_OFFSET - u16Pwm1);
        }
        else if (u16MicroStepIdx == 3U)
        {
            /* W */
            IO_PWM_SLAVE2_CTRL = B_PWM_SLAVE2_SLAVE | C_PWM_SLAVE2_MODE_MIRROR;
            IO_PWM_SLAVE2_LT = 0U; /* High */
            /* V */
            IO_PWM_SLAVE1_CTRL = B_PWM_SLAVE1_SLAVE | C_PWM_SLAVE1_MODE_MIRROR | B_PWM_SLAVE1_POL;
#if (C_PID_FACTOR == 4U)
            u16Pwm1 = p_MulU16hi_pU16byU16lsr4( (pu16Vector + (2U * C_MICROSTEP_PER_FULLSTEP)), l_u16CorrectionRatio); /* V */
#elif (C_PID_FACTOR == 3U)
            u16Pwm1 = p_MulU16hi_pU16byU16lsr3( (pu16Vector + (2U * C_MICROSTEP_PER_FULLSTEP)), l_u16CorrectionRatio); /* V */
#else
#error "No PWM-W calculation"
#endif
#if (PWM_LIMIT == PWM_LIMIT_BY_PWM)
            u16Pwm1 = p_ClipMinMaxU16(u16Pwm1, C_PWM_MIN_DC, l_u16MaxPwmRatio);
#endif /* (PWM_LIMIT == PWM_LIMIT_BY_PWM) */
            IO_PWM_SLAVE1_LT = (uint16_t)(PWM_SCALE_OFFSET - u16Pwm1);
            /* U */
#if (UART_COMM != FALSE) && ((_SUPPORT_UART_IF_APP == C_UART_IF_SCOPE) || (_SUPPORT_UART2_IF_APP == C_UART_IF_SCOPE))
            l_u16MotorVoltageCoilA = 32767 - l_u16MotorVoltageCoilA;
#endif /* (UART_COMM != FALSE) && ((_SUPPORT_UART_IF_APP == C_UART_IF_SCOPE) || (_SUPPORT_UART2_IF_APP == C_UART_IF_SCOPE)) */
            IO_PWM_MASTER1_CTRL = (PWM_PRESCALER << 8) | C_PWM_MASTER1_MODE_MIRROR;
#if (C_PID_FACTOR == 4U)
            u16Pwm1 = p_MulU16hi_pU16byU16lsr4(pu16Vector, l_u16CorrectionRatio);   /* U */
#elif (C_PID_FACTOR == 3U)
            u16Pwm1 = p_MulU16hi_pU16byU16lsr3(pu16Vector, l_u16CorrectionRatio);   /* U */
#else
#error "No PWM-U calculation"
#endif
#if (PWM_LIMIT == PWM_LIMIT_BY_PWM)
            u16Pwm1 = p_ClipMinMaxU16(u16Pwm1, C_PWM_MIN_DC, l_u16MaxPwmRatio);
#endif /* (PWM_LIMIT == PWM_LIMIT_BY_PWM) */
        }
        else if (u16MicroStepIdx == 4U)
        {
            /* V */
            IO_PWM_SLAVE1_CTRL = B_PWM_SLAVE1_SLAVE | C_PWM_SLAVE1_MODE_MIRROR | B_PWM_SLAVE1_POL;
            pu16Vector += (2U * C_MICROSTEP_PER_FULLSTEP);
#if (C_PID_FACTOR == 4U)
            u16Pwm1 = p_MulU16hi_pU16byU16lsr4(pu16Vector, l_u16CorrectionRatio);  /* V */
#elif (C_PID_FACTOR == 3U)
            u16Pwm1 = p_MulU16hi_pU16byU16lsr3(pu16Vector, l_u16CorrectionRatio);  /* V */
#else
#error "No PWM-V calculation"
#endif
#if (PWM_LIMIT == PWM_LIMIT_BY_PWM)
            u16Pwm1 = p_ClipMinMaxU16(u16Pwm1, C_PWM_MIN_DC, l_u16MaxPwmRatio);
#endif /* (PWM_LIMIT == PWM_LIMIT_BY_PWM) */
            IO_PWM_SLAVE1_LT = u16Pwm1;
            /* W */
            IO_PWM_SLAVE2_CTRL = B_PWM_SLAVE2_SLAVE | C_PWM_SLAVE2_MODE_MIRROR;
            pu16Vector += (2U * C_MICROSTEP_PER_FULLSTEP);
#if (C_PID_FACTOR == 4U)
            u16Pwm1 = p_MulU16hi_pU16byU16lsr4(pu16Vector, l_u16CorrectionRatio);  /* W */
#elif (C_PID_FACTOR == 3U)
            u16Pwm1 = p_MulU16hi_pU16byU16lsr3(pu16Vector, l_u16CorrectionRatio);  /* W */
#else
#error "No PWM-W calculation"
#endif
#if (PWM_LIMIT == PWM_LIMIT_BY_PWM)
            u16Pwm1 = p_ClipMinMaxU16(u16Pwm1, C_PWM_MIN_DC, l_u16MaxPwmRatio);
#endif /* (PWM_LIMIT == PWM_LIMIT_BY_PWM) */
            IO_PWM_SLAVE2_LT = (uint16_t)(PWM_SCALE_OFFSET - u16Pwm1);
            /* U */
            IO_PWM_MASTER1_CTRL = (PWM_PRESCALER << 8) | C_PWM_MASTER1_MODE_MIRROR | B_PWM_MASTER1_POL;
            u16Pwm1 = 0U; /* Low */
        }
        else /* if ( u16MicroStepIdx == 5U ) */
        {
            /* W */
            IO_PWM_SLAVE2_CTRL = B_PWM_SLAVE2_SLAVE | C_PWM_SLAVE2_MODE_MIRROR;
#if (C_PID_FACTOR == 4U)
            u16Pwm1 = p_MulU16hi_pU16byU16lsr4( (pu16Vector + (4U * C_MICROSTEP_PER_FULLSTEP)), l_u16CorrectionRatio); /* W */
#elif (C_PID_FACTOR == 3U)
            u16Pwm1 = p_MulU16hi_pU16byU16lsr3( (pu16Vector + (4U * C_MICROSTEP_PER_FULLSTEP)), l_u16CorrectionRatio); /* W */
#else
#error "No PWM-W calculation"
#endif
#if (PWM_LIMIT == PWM_LIMIT_BY_PWM)
            u16Pwm1 = p_ClipMinMaxU16(u16Pwm1, C_PWM_MIN_DC, l_u16MaxPwmRatio);
#endif /* (PWM_LIMIT == PWM_LIMIT_BY_PWM) */
            IO_PWM_SLAVE2_LT = u16Pwm1;
            /* U */
#if (UART_COMM != FALSE) && ((_SUPPORT_UART_IF_APP == C_UART_IF_SCOPE) || (_SUPPORT_UART2_IF_APP == C_UART_IF_SCOPE))
            l_u16MotorVoltageCoilA = 32767 - l_u16MotorVoltageCoilA;
#endif /* (UART_COMM != FALSE) && ((_SUPPORT_UART_IF_APP == C_UART_IF_SCOPE) || (_SUPPORT_UART2_IF_APP == C_UART_IF_SCOPE)) */
            IO_PWM_MASTER1_CTRL = (PWM_PRESCALER << 8) | C_PWM_MASTER1_MODE_MIRROR | B_PWM_MASTER1_POL;
#if (C_PID_FACTOR == 4U)
            u16Pwm1 = p_MulU16hi_pU16byU16lsr4(pu16Vector, l_u16CorrectionRatio);  /* U */
#elif (C_PID_FACTOR == 3U)
            u16Pwm1 = p_MulU16hi_pU16byU16lsr3(pu16Vector, l_u16CorrectionRatio);  /* U */
#else
#error "No PWM-U calculation"
#endif
#if (PWM_LIMIT == PWM_LIMIT_BY_PWM)
            u16Pwm1 = p_ClipMinMaxU16(u16Pwm1, C_PWM_MIN_DC, l_u16MaxPwmRatio);
#endif /* (PWM_LIMIT == PWM_LIMIT_BY_PWM) */
            u16Pwm1 = (PWM_SCALE_OFFSET - u16Pwm1);
            /* V */
            IO_PWM_SLAVE1_CTRL = B_PWM_SLAVE1_SLAVE | C_PWM_SLAVE1_MODE_MIRROR;
            IO_PWM_SLAVE1_LT = 0U; /* High */
        }
#endif
#endif /* (_SUPPORT_VOLTAGE_SHAPE != VOLTAGE_SHAPE_DSVM_ALT) */ /* MMP210524-1 */
    }
#if (_SUPPORT_VARIABLE_PWM != FALSE) && (_SUPPORT_VAR_PWM_MODE != C_VAR_PWM_MODE_OFF)
    else
    {
        /* 3-coil/phase current measurement requires a 3 PWM at 33%, 67% and 100/0% (MMP220815-1) */
        uint16_t u16Pwm, u16HT;
#if (C_PID_FACTOR == 4U)
        /*uint16_t u16CompPulse = PWM_REG_PERIOD;*/                             /* Opposite Pulse 1/16th: 6.25% Duty-cycle */
        uint16_t u16CompPulse = (PWM_REG_PERIOD >> 1U);                         /* Opposite Pulse 1/32nd: 3.125% Duty-cycle */
        /*uint16_t u16CompPulse = (PWM_REG_PERIOD >> 2U);*/                     /* Opposite Pulse 1/64th: 1.5625% Duty-cycle */
#elif (C_PID_FACTOR == 3U)
        uint16_t u16CompPulse = (PWM_REG_PERIOD >> 1U);
#endif
        uint16_t u16CorrectionRatio = l_u16CorrectionRatio + u16CompPulse;
        register uint16_t *pu16Vector;
        pu16Vector = p_X(u16MicroStepIdx); /* (uint16_t *) &c_au16MicroStepVector3PH[u16MicroStepIdx]; */

        /* U-phase @ 33% */
        u16Pwm = p_MulU16hi_pU16byU16lsr3(pu16Vector, u16CorrectionRatio);      /* U */
        pu16Vector += (3U * C_MICROSTEP_PER_FULLSTEP);
        u16Pwm += p_MulU16hi_pU16byU16lsr3(pu16Vector,  u16CompPulse);          /* Ucomp */
        u16HT = ((uint16_t)((1U * PWM_REG_PERIOD) / 3U) + (u16Pwm / 2U));
        IO_PWM_MASTER1_HT = u16HT;
        u16Pwm1 = (u16HT - u16Pwm);

        /* V-phase @ 67% */
        pu16Vector -= (1U * C_MICROSTEP_PER_FULLSTEP);
        u16Pwm = p_MulU16hi_pU16byU16lsr3(pu16Vector, u16CorrectionRatio);      /* V */
        pu16Vector += (3U * C_MICROSTEP_PER_FULLSTEP);
        u16Pwm += p_MulU16hi_pU16byU16lsr3(pu16Vector,  u16CompPulse);          /* Vcomp */
        u16HT = ((uint16_t)((2U * PWM_REG_PERIOD) / 3U) + (u16Pwm / 2U));
        IO_PWM_SLAVE1_HT = u16HT;
        IO_PWM_SLAVE1_LT = (u16HT - u16Pwm);

        /* W-phase @ 100/0% */
        pu16Vector -= C_MICROSTEP_PER_FULLSTEP;
        u16Pwm = p_MulU16hi_pU16byU16lsr3(pu16Vector, u16CorrectionRatio);      /* W */
        pu16Vector -= (3U * C_MICROSTEP_PER_FULLSTEP);
        u16Pwm += p_MulU16hi_pU16byU16lsr3(pu16Vector, u16CompPulse);           /* Wcomp */
        IO_PWM_SLAVE2_CTRL = C_PWM_SLAVE2_MODE_MIRROR | B_PWM_SLAVE2_POL;
        u16HT = (PWM_REG_PERIOD - (u16Pwm / 2U));
        IO_PWM_SLAVE2_HT = u16HT;
        IO_PWM_SLAVE2_LT = ((u16HT + u16Pwm) - PWM_REG_PERIOD);
    }
#endif /* (_SUPPORT_VARIABLE_PWM != FALSE) && (_SUPPORT_VAR_PWM_MODE != C_VAR_PWM_MODE_OFF) */
#if (_SUPPORT_PWM_MIRROR != FALSE)
    IO_PWM_MASTER1_LT = u16Pwm1;
#else  /* (_SUPPORT_PWM_MIRROR != FALSE) */
#endif /* (_SUPPORT_PWM_MIRROR != FALSE) */

#if (_SUPPORT_CDI != FALSE)
    if (NV_CDI_ENA != 0U)
    {
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
    }
#endif /* (_SUPPORT_CDI != FALSE) */
} /* End of MotorDriver_3Phase() */
#endif /* (C_MOTOR_PHASES == 3) */

#if (_SUPPORT_CDI != FALSE)
#if FALSE /* TODO[MMP-2] */
uint16_t const c_au16DrvCfgCDI[C_MOTOR_PHASES * C_MOTOR_STEP_PER_PHASE] =
{
    (C_PORT_DRV_CTRL_DRV2_L | C_PORT_DRV_CTRL_DRV1_MASTER1 | C_PORT_DRV_CTRL_DRV0_MASTER1),  /* PhC = L, PhB = PWM, PhA = PWM */
    (C_PORT_DRV_CTRL_DRV2_L | C_PORT_DRV_CTRL_DRV1_L | C_PORT_DRV_CTRL_DRV0_MASTER1),        /* PhC = L, PhB = L, PHA = PWM */
    (C_PORT_DRV_CTRL_DRV2_MASTER1 | C_PORT_DRV_CTRL_DRV1_L | C_PORT_DRV_CTRL_DRV0_MASTER1),  /* PhC = PPM, PhB = L, PhA = PWM */
    (C_PORT_DRV_CTRL_DRV2_MASTER1 | C_PORT_DRV_CTRL_DRV1_L | C_PORT_DRV_CTRL_DRV0_L),        /* PhC = PWM, PhB = L, PhA = L */
    (C_PORT_DRV_CTRL_DRV2_MASTER1 | C_PORT_DRV_CTRL_DRV1_MASTER1 | C_PORT_DRV_CTRL_DRV0_L),  /* PhC = PWM, PhB = PWM, PhA = L */
    (C_PORT_DRV_CTRL_DRV2_L | C_PORT_DRV_CTRL_DRV1_MASTER1 | C_PORT_DRV_CTRL_DRV0_L)         /* PhC = L, PhB = PWM, PhA = L  */
};
#else
uint16_t const c_au16DrvCfgCDI[C_MOTOR_PHASES * C_MOTOR_STEP_PER_PHASE] =
{
    (C_PORT_DRV_CTRL_DRV2_L | C_PORT_DRV_CTRL_DRV1_SLAVE2 | C_PORT_DRV_CTRL_DRV0_MASTER1),   /* PhC = L, PhB = PWM, PhA = PWM */
    (C_PORT_DRV_CTRL_DRV2_L | C_PORT_DRV_CTRL_DRV1_L | C_PORT_DRV_CTRL_DRV0_MASTER1),        /* PhC = L, PhB = L, PHA = PWM */
    (C_PORT_DRV_CTRL_DRV2_SLAVE2 | C_PORT_DRV_CTRL_DRV1_L | C_PORT_DRV_CTRL_DRV0_MASTER1),   /* PhC = PPM, PhB = L, PhA = PWM */
    (C_PORT_DRV_CTRL_DRV2_SLAVE2 | C_PORT_DRV_CTRL_DRV1_L | C_PORT_DRV_CTRL_DRV0_L),         /* PhC = PWM, PhB = L, PhA = L */
    (C_PORT_DRV_CTRL_DRV2_SLAVE2 | C_PORT_DRV_CTRL_DRV1_MASTER1 | C_PORT_DRV_CTRL_DRV0_L),   /* PhC = PWM, PhB = PWM, PhA = L */
    (C_PORT_DRV_CTRL_DRV2_L | C_PORT_DRV_CTRL_DRV1_SLAVE2 | C_PORT_DRV_CTRL_DRV0_L)          /* PhC = L, PhB = PWM, PhA = L  */
};
#endif

uint16_t const c_au16CdiCfg[C_MOTOR_PHASES * C_MOTOR_STEP_PER_PHASE] =
{
    C_IO_ACTIVE_CDI_ACTIVE_CDI_PHASE_DIS,
    C_IO_ACTIVE_CDI_ACTIVE_CDI_PHASE_V,
    C_IO_ACTIVE_CDI_ACTIVE_CDI_PHASE_DIS,
    C_IO_ACTIVE_CDI_ACTIVE_CDI_PHASE_U,
    C_IO_ACTIVE_CDI_ACTIVE_CDI_PHASE_DIS,
    C_IO_ACTIVE_CDI_ACTIVE_CDI_PHASE_W
};

#if (_SUPPORT_OPTIMIZE_FOR_SPEED != FALSE)
void MotorDriver_3PhaseCDI(uint16_t u16FullStepIdx) __attribute__((aligned(8)));
#endif /* (_SUPPORT_OPTIMIZE_FOR_SPEED != FALSE) */
/*!*************************************************************************** *
 * MotorDriver_3PhaseCDI
 * \brief   Drive motor in micro-stepper mode (48-steps)
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16StepIdx: Full-step index
 * \return  -
 * *************************************************************************** *
 * \details
 * *************************************************************************** *
 * - Call Hierarchy: ISR_CTIMER0_3(), MotorDriverStart(), MotorDriverStop(),
 *                   PID_Control()
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 1 (p_MulI16_I16byI16Shft4())
 * *************************************************************************** */
#define _SUPPORT_CDI_DRV_CFG                /*FALSE*/ TRUE                      /*!< FALSE: Use PWM-DC only (Fails! Why?); TRUE: Set PWM-DC and DRV-CTRL */

void MotorDriver_3PhaseCDI(uint16_t u16FullStepIdx)
{
#if (_SUPPORT_CDI_DRV_CFG != FALSE)
    uint16_t u16NewDrvCfg = ((IO_PORT_DRV_CTRL & ~(M_PORT_DRV_CTRL_DRV2_CTRL |
                                                   M_PORT_DRV_CTRL_DRV1_CTRL |
                                                   M_PORT_DRV_CTRL_DRV0_CTRL)) |
                             c_au16DrvCfgCDI[u16FullStepIdx]);
#endif /* (_SUPPORT_CDI_DRV_CFG != FALSE) */
    uint16_t u16DC1 = (l_u16CorrectionRatio >> 5U);
#if (PWM_LIMIT == PWM_LIMIT_BY_PWM)
    uint16_t u16DC2 = p_ClipMinMaxU16(u16DC1, C_PWM_MIN_DC, PWM_SCALE_OFFSET);
#else  /* (PWM_LIMIT == PWM_LIMIT_BY_PWM) */
    uint16_t u16DC2 = u16DC1;
#endif /* (PWM_LIMIT == PWM_LIMIT_BY_PWM) */
    u16DC1 = (PWM_SCALE_OFFSET - u16DC2);
#if (_SUPPORT_CDI_DRV_CFG != FALSE)
#if (_SUPPORT_APP_USER_MODE != FALSE)
    ENTER_SECTION(SYSTEM_MODE); /*lint !e534 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
    IO_MLX16_ITC_PEND2_S = B_MLX16_ITC_PEND2_PWM_MASTER1_END;                   /* Clear PWM-module Master1-End IRQ's */
#if (_SUPPORT_APP_USER_MODE != FALSE)
    EXIT_SECTION(); /*lint !e438 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
    IO_PWM_SLAVE2_LT = u16DC2;  /* TODO[MMP-2] */
    IO_PWM_MASTER1_LT = u16DC1;                                                 /* Master must be modified at last */
    while ( (IO_MLX16_ITC_PEND2_S & B_MLX16_ITC_PEND2_PWM_MASTER1_END) == 0x0000U) {}  /* Check on PWM-MASTER1_END */
    ENTER_SECTION(ATOMIC_KEEP_MODE);  /*lint !e534 */
    if (g_e8MotorStatus != C_MOTOR_STATUS_STOP)                                 /* Don't set driver in case MotorDriverStop() has been called just before */
    {
        IO_PORT_DRV_CTRL = u16NewDrvCfg;
    }
    EXIT_SECTION(); /*lint !e438 */
#else  /* (_SUPPORT_CDI_DRV_CFG != FALSE) */
    if (u16FullStepIdx == 0U)
    {
        IO_PWM_SLAVE2_LT = 0U;                                                  /* Slave 2 (W): GND */
        IO_PWM_SLAVE1_CTRL = B_PWM_SLAVE1_SLAVE | C_PWM_SLAVE1_MODE_MIRROR | B_PWM_SLAVE1_POL;
        IO_PWM_SLAVE1_LT = u16DC2;                                              /* Slave 1 (V): PWM (POL) */
        IO_PWM_MASTER1_LT = u16DC1;                                             /* Master1 (U): PWM; Master must be modified at last */
    }
    else if (u16FullStepIdx == 1U)
    {
        IO_PWM_SLAVE2_LT = 0U;                                                  /* Slave 2 (W): GND */
        IO_PWM_SLAVE1_CTRL = B_PWM_SLAVE1_SLAVE | C_PWM_SLAVE1_MODE_MIRROR;
        IO_PWM_SLAVE1_LT = PWM_SCALE_OFFSET;                                    /* Slave 1 (V): GND */
        IO_PWM_MASTER1_LT = u16DC1;                                             /* Master1 (U): PWM; Master must be modified at last */
    }
    else if (u16FullStepIdx == 2U)
    {
        IO_PWM_SLAVE2_LT = u16DC2;                                              /* Slave 2 (W): PWM (POL) */
        IO_PWM_SLAVE1_CTRL = B_PWM_SLAVE1_SLAVE | C_PWM_SLAVE1_MODE_MIRROR;
        IO_PWM_SLAVE1_LT = PWM_SCALE_OFFSET;                                    /* Slave 1 (V): GND */
        IO_PWM_MASTER1_LT = u16DC1;                                             /* Master1 (U): PWM; Master must be modified at last */
    }
    else if (u16FullStepIdx == 3U)
    {
        IO_PWM_SLAVE2_LT = u16DC2;                                              /* Slave 2 (W): PWM (POL) */
        IO_PWM_SLAVE1_CTRL = B_PWM_SLAVE1_SLAVE | C_PWM_SLAVE1_MODE_MIRROR;
        IO_PWM_SLAVE1_LT = PWM_SCALE_OFFSET;                                    /* Slave 1 (V): GND */
        IO_PWM_MASTER1_LT = PWM_SCALE_OFFSET;                                   /* Master1 (U): GND; Master must be modified at last */
    }
    else if (u16FullStepIdx == 4U)
    {
        IO_PWM_SLAVE2_LT = u16DC2;                                              /* Slave 2 (W): PWM (POL) */
        IO_PWM_SLAVE1_CTRL = B_PWM_SLAVE1_SLAVE | C_PWM_SLAVE1_MODE_MIRROR;
        IO_PWM_SLAVE1_LT = u16DC1;                                              /* Slave 1 (V): PWM */
        IO_PWM_MASTER1_LT = PWM_SCALE_OFFSET;                                   /* Master1 (U): GND; Master must be modified at last */
    }
    else
    {
        IO_PWM_SLAVE2_LT = 0U;                                                  /* Slave 2 (W): GND */
        IO_PWM_SLAVE1_CTRL = B_PWM_SLAVE1_SLAVE | C_PWM_SLAVE1_MODE_MIRROR | B_PWM_SLAVE1_POL;
        IO_PWM_SLAVE1_LT = u16DC2;                                              /* Slave 1 (V): PWM (POL) */
        IO_PWM_MASTER1_LT = PWM_SCALE_OFFSET;                                   /* Master1 (U): GND; Master must be modified at last */
    }
#endif /* (_SUPPORT_CDI_DRV_CFG != FALSE) */

    IO_ACTIVE_CDI = (IO_ACTIVE_CDI & ~M_IO_ACTIVE_CDI_ACTIVE_CDI_PHASE) | c_au16CdiCfg[u16FullStepIdx];
    /* Change of CDI configuration may trigger CDI IRQ */
#if (_SUPPORT_APP_USER_MODE != FALSE)
    ENTER_SECTION(SYSTEM_MODE); /*lint !e534 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
    IO_MLX16_ITC_PEND5_S = B_MLX16_ITC_PEND5_ACTIVE_CDI;                        /* Clear CDI IRQ's */
#if (_SUPPORT_APP_USER_MODE != FALSE)
    EXIT_SECTION(); /*lint !e438 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
} /* End of MotorDriver_3PhaseCDI() */
#endif /* (_SUPPORT_CDI != FALSE) */

/*!*************************************************************************** *
 * MotorDriverStart
 * \brief   Start Motor Driver
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16TargetMotorSpeed: Target speed [RPM]
 * \return  -
 * *************************************************************************** *
 * \details
 * *************************************************************************** *
 * - Call Hierarchy: main()
 * - Cyclomatic Complexity: 5+1
 * - Nesting: 3
 * - Function calling: 8 (MotorDriverSelfTest(), ADC_Init(),
 *                       MotorDriverSpeed(),
 *                       MotorDriverCurrentMeasureInit(), MotorStallInitA(),
 *                       MotorDriver_InitialPwmDutyCycle(),
 *                       MotorDriver_3Phase(), ADC_Start())
 * *************************************************************************** */
void MotorDriverStart(uint16_t u16TargetMotorSpeed)
{
    if ( (g_e8ErrorElectric & (uint8_t)C_ERR_PERMANENT) != 0U)                  /* Don't start motor in case of permanent electric failure */
    {
        return;
    }
#if (_SUPPORT_MOTOR_SELFTEST != FALSE)
    else if ( (g_u8ForceMotorDriverSelfTest != FALSE) ||
              ((g_e8ErrorElectric & (uint8_t)C_ERR_MOTOR) != 0U) )
    {
        uint8_t u8MemorizedElectricError = g_e8ErrorElectric;
        g_u8ForceMotorDriverSelfTest = FALSE;
        g_e8ErrorElectric = (uint8_t)C_ERR_NONE;
        MotorDriverSelfTest();
        if (g_e8ErrorElectric != (uint8_t)C_ERR_NONE)
        {
            g_e8ErrorElectric = (g_e8ErrorElectric & ~C_ERR_PERMANENT) | u8MemorizedElectricError;
            return;
        }
    }
    else
    {
        /* Nothing */
    }
#endif /* (_SUPPORT_MOTOR_SELFTEST != FALSE) */

#if (_DEBUG_MOTOR_START != FALSE)
    DEBUG_SET_IO_E();
#endif /* (_DEBUG_MOTOR_START != FALSE) */

    DiagnosticReset();

#if (_SUPPORT_PWM_POL_CORR == FALSE)
    l_u16PwmStateStepIdx = (l_u16MotorMicroStepsPerElecRotation - (C_MICROSTEP_PER_FULLSTEP / 2U));
#endif /* (_SUPPORT_PWM_POL_CORR == FALSE) */
    l_i16ActLoadAngle = 0;

#if (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE)
    if ( (g_u8MotorHoldingCurrEna == FALSE) && (l_u8MotorHoldingCurrState == FALSE) )
#endif /* (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) */
    {
        ADC_Init();
    }
#if (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE)
    else
    {
        ADC_MCurrOffCalib();
    }
#endif /* (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) */

#if (_SUPPORT_CLOSED_LOOP_STARTUP == FALSE) || (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE)
    g_u16ForcedSpeedRPM = g_u16MinSpeedRPM;
#endif /* (_SUPPORT_CLOSED_LOOP_STARTUP == FALSE) || (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) */

#if (_SUPPORT_MOTOR_POSITION == C_MOTOR_POS_NONE)
#if (_SUPPORT_TACHO_OUT != FALSE)
    MotorDriverTachoInit();
#endif /* (_SUPPORT_TACHO_OUT != FALSE) */

    MotorDriverSpeed(u16TargetMotorSpeed);
#else  /* (_SUPPORT_MOTOR_POSITION == C_MOTOR_POS_NONE) */
    MotorDriverSpeed(u16TargetMotorSpeed);
    g_u32TargetPosition = ConvShaftSteps2MicroSteps(g_u16TargetPosition);
#endif /* (_SUPPORT_MOTOR_POSITION == C_MOTOR_POS_NONE) */

    /* Clear motor-driver current measurement */
    MotorDriverCurrentMeasureInit();

    l_e8MotorStartupMode = E_MSM_STEPPER_A;                                     /* Start-up in stepper-mode 'A' */
    l_u16StallDetectorDelay = C_DETECTOR_DELAY;
#if (_SUPPORT_STALLDET_A != FALSE)
    MotorStallInitA();
#endif /* (_SUPPORT_STALLDET_A != FALSE) */
#if (_SUPPORT_STALLDET_O != FALSE)
    MotorStallInitO();
#endif /* (_SUPPORT_STALLDET_O != FALSE) */
#if (_SUPPORT_STALLDET_P != FALSE)
    MotorStallInitP();
#endif /* (_SUPPORT_STALLDET_P != FALSE) */
#if (_SUPPORT_STALLDET_LA != FALSE)
    MotorStallInitLA(u16TargetMotorSpeed);
#endif /* (_SUPPORT_STALLDET_LA != FALSE) */
#if (_SUPPORT_STALLDET_FLUX != FALSE)
    MotorStallInitFlux();
#endif /* (_SUPPORT_STALLDET_FLUX != FALSE) */
#if (_SUPPORT_STALLDET_S != FALSE)
    MotorStallInitS();
#endif /* (_SUPPORT_STALLDET_S != FALSE) */

    /* Connect drivers */
#if (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE)
    l_u8MotorHoldDelay = 0U;
#endif /* (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) */
#if (_SUPPORT_CDI != FALSE)
    if (NV_CDI_ENA != 0U)
    {
        /* Set CDI-Clock at Max. of 2 MHz (Pre-divider = 1, Divider = 3+1); 32MHz / (2 * 2^PRE-DIV * (DIV + 1))
         * Mask-time is 2us (4)
         * Debounce time is 1us (2) */
        IO_ACTIVE_CDI = (B_IO_ACTIVE_CDI_ACTIVE_CDI_CLK_PREDIV2 |               /* Clock pre-divider for Active CDI: 1 */
                         C_IO_ACTIVE_CDI_ACTIVE_CDI_EDGE_SEL_FALL |             /* Edge selection for the ACTIVE CDI interrupt */
                         (4U << 8) | (2U << 6) | (3U << 2) |                    /* Active CDI masking time: 6, Active CDI debounce time in periods: 2..3, Clock divider for Active CDI: 6+1 */
/*TBV                         (6U << 8) | (2U << 6) | (6U << 2) |                    / * Active CDI masking time: 4, Active CDI debounce time in periods: 2..3, Clock divider for Active CDI: 3+1 * / */
                         C_IO_ACTIVE_CDI_ACTIVE_CDI_PHASE_DIS);                 /* Select the phase for the Active CDI measurement: Disabled */
        l_u16CDI = C_IO_ACTIVE_CDI_ACTIVE_CDI_PHASE_DIS;
#if (_SUPPORT_APP_USER_MODE != FALSE)
        ENTER_SECTION(SYSTEM_MODE); /*lint !e534 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
        IO_MLX16_ITC_PRIO7_S = (IO_MLX16_ITC_PRIO7_S & ~M_MLX16_ITC_PRIO7_ACTIVE_CDI) |
                               C_MLX16_ITC_PRIO7_ACTIVE_CDI_PRIO4;
        IO_MLX16_ITC_PEND5_S = B_MLX16_ITC_PEND5_ACTIVE_CDI;
        IO_MLX16_ITC_MASK5_S |= B_MLX16_ITC_MASK5_ACTIVE_CDI;                   /* Activate CDI IRQ */
#if (_SUPPORT_APP_USER_MODE != FALSE)
        EXIT_SECTION(); /*lint !e438 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
        sPIDpCDI.i16CoefP = (int16_t)(C_CDI_PID_COEF_P << (C_GN_PID - 12U));      /* CDI Commutation-period PID P-coefficient */
        sPIDpCDI.i16CoefI = (int16_t)(C_CDI_PID_COEF_I << (C_GN_PID - 12U));      /* CDI Commutation-period PID I-coefficient */
#if (_SUPPORT_PID_D_COEF != FALSE)
        sPIDpCDI.i16CoefD = (int16_t)(C_CDI_PID_COEF_D << (C_GN_PID - 12U));      /* CDI Commutation-period PID D-coefficient */
#endif /* (_SUPPORT_PID_D_COEF != FALSE) */
        sPIDpCDI.u16MinOutput = 200U;                                             /* Minimal Commutation period (100us) */
        sPIDpCDI.u32MaxOutput = 65535UL;                                          /* Maximal Commutation period */
        sPIDpCDI.u32SumErrorMax = (65535UL << C_GN_PID);                          /* Maximal Commutation period */
    }
#endif /* (_SUPPORT_CDI != FALSE) */
    MotorDriverConfig(TRUE);
#if (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT)
    l_u16MicroStepIdx = UNKNOWN_STEP;
#endif /* (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT) */

#if (PWM_COMM != FALSE) && (_SUPPORT_WINDMILL || _SUPPORT_SRP)
    PWMin_Stop();                                                               /* Release CTimer1 for Wind-milling (PWM-Communication has stopped) */
#endif /* (PWM_COMM != FALSE) && (_SUPPORT_WINDMILL || _SUPPORT_SRP) */
#if (_SUPPORT_WINDMILL != FALSE)
    g_e8MotorStatus = C_MOTOR_STATUS_WINDMILLING;
    l_u16MicroStepIdx = MotorWindMillCheck();
    if ( (l_u16MicroStepIdx & 0x8000U) == 0x0000U)
    {
        /* Windmill mode */
#if (_SUPPORT_PID_U32 == FALSE)
        g_u16ActualMotorSpeedRPM = p_DivU16_U32byU16( (uint32_t)g_u16ActualMotorSpeedRPMe, g_u16MotorPolePairs);
#else  /* (_SUPPORT_PID_U32 == FALSE) */
        g_u16ActualMotorSpeedRPM = p_DivU16_U32byU16(g_u32ActualMotorSpeedRPMe, g_u16MotorPolePairs);
#endif /* (_SUPPORT_PID_U32 == FALSE) */
        l_u16CommutTimerPeriod = p_DivU16_U32byU16(l_u32MicroStepPeriodOneRPM, g_u16ActualMotorSpeedRPM);
        g_u16ActualCommutTimerPeriod = l_u16CommutTimerPeriod;

#if (_SUPPORT_BRAKING != FALSE)
        if (g_u8WindMillMotorDirectionCCW != g_e8MotorDirectionCCW)
        {
            /* Windmilling in wrong direction */
#if (_SUPPORT_LOG_ERRORS != FALSE)
            SetLastError(C_INF_WINDMILL | C_ERR_EXTW | (0x01U << 8));           /* Wind-milling in wrong direction */
            SetLastError(g_u16ActualMotorSpeedRPM);
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */

            g_u16MemorizedTargetMotorSpeedRPM = g_u16TargetMotorSpeedRPM;       /* Memorise requested Target speed */
            g_u16TargetMotorSpeedRPM = g_u16MinSpeedRPM;                        /* Set Target speed to minimum speed */
            l_e8MotorStartupMode = E_MSM_FOC_BRAKING;                           /* Start FOC Braking */
        }
        else
#endif /* (_SUPPORT_BRAKING != FALSE) */
        {
            /* Windmilling in right direction */
#if (_SUPPORT_LOG_ERRORS != FALSE)
            SetLastError(C_INF_WINDMILL | C_ERR_EXTW);
            SetLastError(g_u16ActualMotorSpeedRPM);
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
#if (_SUPPORT_FOC_MODE != FOC_MODE_NONE)
            l_e8MotorStartupMode = E_MSM_FOC_2PWM;
#else  /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) */
            l_e8MotorStartupMode = E_MSM_STEPPER_C;
#endif /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) */
        }

#if (_SUPPORT_MOTOR_DIRECTION == C_MOTOR_DIRECTION_CMD)
#if (_SUPPORT_BRAKING != FALSE)
        if ((g_e8MotorDirectionCCW != FALSE) ^ (l_e8MotorStartupMode == E_MSM_FOC_BRAKING))
#else  /* (_SUPPORT_BRAKING != FALSE) */
        if (g_e8MotorDirectionCCW != FALSE)
#endif /* (_SUPPORT_BRAKING != FALSE) */
#endif /* (_SUPPORT_MOTOR_DIRECTION == C_MOTOR_DIRECTION_CMD) */
#if (_SUPPORT_MOTOR_DIRECTION == C_MOTOR_DIRECTION_CCW) || (_SUPPORT_MOTOR_DIRECTION == C_MOTOR_DIRECTION_CMD)
        {
            if (l_u16MicroStepIdx >= g_u16NrOfMicroStepsPerFullStep)
            {
                l_u16MicroStepIdx -= g_u16NrOfMicroStepsPerFullStep;
            }
            else
            {
                l_u16MicroStepIdx += (l_u16MotorMicroStepsPerElecRotation - g_u16NrOfMicroStepsPerFullStep);
            }
        }
#endif /* (_SUPPORT_MOTOR_DIRECTION == C_MOTOR_DIRECTION_CCW) || (_SUPPORT_MOTOR_DIRECTION == C_MOTOR_DIRECTION_CMD) */
#if (_SUPPORT_MOTOR_DIRECTION == C_MOTOR_DIRECTION_CMD)
        else
#endif /* (_SUPPORT_MOTOR_DIRECTION == C_MOTOR_DIRECTION_CMD) */
#if (_SUPPORT_MOTOR_DIRECTION == C_MOTOR_DIRECTION_CW) || (_SUPPORT_MOTOR_DIRECTION == C_MOTOR_DIRECTION_CMD)
        {
            l_u16MicroStepIdx += g_u16NrOfMicroStepsPerFullStep;
            if (l_u16MicroStepIdx >= l_u16MotorMicroStepsPerElecRotation)
            {
                l_u16MicroStepIdx -= l_u16MotorMicroStepsPerElecRotation;
            }
        }
#endif /* (_SUPPORT_MOTOR_DIRECTION == C_MOTOR_DIRECTION_CW) || (_SUPPORT_MOTOR_DIRECTION == C_MOTOR_DIRECTION_CMD) */

#if (_SUPPORT_BRAKING != FALSE)
        if (l_e8MotorStartupMode == E_MSM_FOC_BRAKING)
        {
            /* During braking use 50% of duty cycle, to avoid huge current spike (MMP240625-1) */
            PID_WindmillStart(g_u16ActualMotorSpeedRPM / 2U);
        }
        else
#endif /* (_SUPPORT_BRAKING != FALSE) */
        {
            PID_WindmillStart(g_u16ActualMotorSpeedRPM);
        }
        l_i16ActLoadAngle = p_MulDivU16_U16byU16byU16(NV_TARGET_LA, g_u16ActualMotorSpeedRPM, g_u16MaxSpeedRPM);
        l_u16CorrectionRatio = VoltageCorrection();
#if (_SUPPORT_VARIABLE_PWM != FALSE)                                            /* MMP220815-1 */
#if (_SUPPORT_VAR_PWM_MODE == C_VAR_PWM_MODE_ON_DUTY_CYCLE)
        {
            uint8_t u8TripplePWM = FALSE;
            if (l_u16CorrectionRatio < C_PWM_DC_SWITCH3PWM)
            {
                u8TripplePWM = TRUE;
            }
            MotorDriverReconfigure(u8TripplePWM);
        }
#else  /* (_SUPPORT_VAR_PWM_MODE == C_VAR_PWM_MODE_ON_DUTY_CYCLE) */
        MotorDriverReconfigure(FALSE);                                          /* 2-phase Motor-current */
#endif /* (_SUPPORT_VAR_PWM_MODE == C_VAR_PWM_MODE_ON_DUTY_CYCLE) */
#endif /* (_SUPPORT_VARIABLE_PWM != FALSE) */
    }
    else
#endif /* (_SUPPORT_WINDMILL != FALSE) */
    {
        /* Stepper-mode */
#if (_SUPPORT_LOG_ERRORS != FALSE)
        SetLastError(C_INF_WINDMILL | C_ERR_EXT | (0x0A << 8));                 /* Windmill absent */
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
#if ((_SUPPORT_FOC_MODE & FOC_OBSERVER_MASK) == FOC_OBSERVER_SENSORED)
        l_u16ResolverAngle = Triaxis_Angle();
        l_u16PrevResolverAngle = l_u16ResolverAngle;
        l_u16MicroStepIdx = MotorDriverResolverAngleToMicroStepIndex(l_u16ResolverAngle);
        l_u16MicroStepIdxPrev = l_u16MicroStepIdx;
        l_u16RotorAngleSpeed = 0U;
        l_u32RotorAngleSpeedLPF = 0U;
#elif (_SUPPORT_SRP != FALSE)
#if (_SUPPORT_WINDMILL != FALSE)
        /* Windmill speed was too low, but direction is wrong; Stop motor before starting */
        if ( (g_u8WindMillMotorDirectionCCW != (uint8_t)C_MOTOR_DIR_UNKNOWN) &&
             (g_u8WindMillMotorDirectionCCW != g_e8MotorDirectionCCW) )
        {
            /* Alignment will stop motor, but need higher start-up current */
#if (_SUPPORT_LOG_ERRORS != FALSE)
            SetLastError(C_INF_SRP | C_ERR_EXT | (0x0E << 8));                  /* SRP Error (Reverse windmilling detected); Use Alignment */
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
            l_u16MicroStepIdx = UNKNOWN_STEP;
        }
        else if (g_u8SkipSRP != FALSE)
        {
            l_u16MicroStepIdx = UNKNOWN_STEP;
            g_u8SkipSRP = FALSE;
        }
        else
#endif /* (_SUPPORT_WINDMILL != FALSE) */
        {
            g_e8MotorStatus = C_MOTOR_STATUS_SRP;
            l_u16MicroStepIdx = MotorSRP(C_MOTOR_PHASES * C_MOTOR_STEP_PER_PHASE);
        }

        if (l_u16MicroStepIdx != UNKNOWN_STEP)
        {
            /* Known actual rotor position; Start with soft-ramp */
#if (_SUPPORT_LOG_ERRORS != FALSE)
            SetLastError(C_INF_SRP | C_ERR_EXT | ((l_u16MicroStepIdx & 0x0FU) << 8));
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
#if ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_ID_IQ)
            l_u16MicroStepIdx = (7U - l_u16MicroStepIdx) * C_MICROSTEP_PER_FULLSTEP;  /* Convert Full-step to Micro-step (CCW) */
#if (_SUPPORT_MOTOR_DIRECTION == C_MOTOR_DIRECTION_CW)
            l_u16MicroStepIdx += C_MICROSTEP_PER_FULLSTEP;                      /* Convert Full-step to Micro-step (CW) */
#elif (_SUPPORT_MOTOR_DIRECTION == C_MOTOR_DIRECTION_CMD)
            if (g_e8MotorDirectionCCW == FALSE)
            {
                l_u16MicroStepIdx += C_MICROSTEP_PER_FULLSTEP;                  /* Convert Full-step to Micro-step (CW) */
            }
#endif /* (_SUPPORT_MOTOR_DIRECTION) */
#else  /* ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_ID_IQ) */
            l_u16MicroStepIdx = (5U + l_u16MicroStepIdx) * C_MICROSTEP_PER_FULLSTEP;  /* Convert Full-step to Micro-step (CCW) */
#endif /* ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_ID_IQ) */
            while (l_u16MicroStepIdx >= l_u16MotorMicroStepsPerElecRotation)
            {
                l_u16MicroStepIdx -= l_u16MotorMicroStepsPerElecRotation;
            }
#if (_SUPPORT_SOFT_START != FALSE)
            l_e8AlignmentStep = E_ALIGN_STEP_SHORT;
            l_u16RampStep = C_ALIGN_SOFT_PERIOD;
#endif /* (_SUPPORT_SOFT_START != FALSE) */
        }
        else
#endif /* (_SUPPORT_SRP != FALSE) */
        {
            /* Unknown actual rotor position; Start with very soft ramp; time should be long enough for the fan to settle */
#if (_SUPPORT_SRP != FALSE) && (_SUPPORT_LOG_ERRORS != FALSE)
            SetLastError(C_INF_SRP | C_ERR_EXT | (0x0FU << 8));                 /* SRP Failure */
#endif /* (_SUPPORT_SRP != FALSE) && (_SUPPORT_LOG_ERRORS != FALSE) */

#if (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT)
            l_u16MicroStepIdx = (C_MICROSTEP_PER_FULLSTEP / 2U);
#else  /* (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT) */
            l_u16MicroStepIdx = MotorDriverUpdateMicroStepIndex( l_u16MicroStepIdx);
#endif /* (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT) */
#if (_SUPPORT_SOFT_START != FALSE)
            l_e8AlignmentStep = E_ALIGN_STEP_1;
            l_u16RampStep = C_ALIGN_STEP_1_PERIOD;
#endif /* (_SUPPORT_SOFT_START != FALSE) */
        }

        /* Define initial commutation timer */
        if (l_u16TargetCommutTimerPeriod < l_u16LowSpeedPeriod)
        {
            /* Target speed too fast for motor to start-up with */
            l_u16CommutTimerPeriod = l_u16LowSpeedPeriod;                       /* Initial start-up speed */
            g_u16ActualMotorSpeedRPM = g_u16StartupSpeedRPM;
        }
        else
        {
            /* Target speed is slower than maximum motor start-up speed */
            l_u16CommutTimerPeriod = l_u16TargetCommutTimerPeriod;
            g_u16ActualMotorSpeedRPM = g_u16TargetMotorSpeedRPM;
        }
#if (_SUPPORT_PID_U32 == FALSE)
        g_u16ActualMotorSpeedRPMe = (uint16_t)p_MulU32_U16byU16(g_u16ActualMotorSpeedRPM, g_u16MotorPolePairs);
#else  /* (_SUPPORT_PID_U32 == FALSE) */
        g_u32ActualMotorSpeedRPMe = p_MulU32_U16byU16(g_u16ActualMotorSpeedRPM, g_u16MotorPolePairs);
#endif /* (_SUPPORT_PID_U32 == FALSE) */
#if ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_ID_IQ)
        /* Speed [rad/sec] = Speed [RPMe] * 65536 [2*pi] / 60 [sec/min] */
#if (_SUPPORT_PID_U32 == FALSE)
        l_u32MotorSpeedRadPSe = p_MulU32_U16byU16(g_u16ActualMotorSpeedRPMe, (uint16_t)(65536UL/60U));
#else  /* (_SUPPORT_PID_U32 == FALSE) */
        l_u32MotorSpeedRadPSe = p_MulU32_U16byU16((uint16_t)g_u32ActualMotorSpeedRPMe, (uint16_t)(65536UL/60U));
#endif /* (_SUPPORT_PID_U32 == FALSE) */
#endif /* ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_ID_IQ) */

        /* Set initial l_u16CorrectionRatio */
#if (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) && (_SUPPORT_FOC_MODE == FOC_MODE_NONE)
        MotorDriver_InitialPwmDutyCycle(Get_ActCurrRunMax_mA(), g_u16MinSpeedRPM);
#else  /* (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) */
#if (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
        if (g_e8MotorRequest == (uint8_t)C_MOTOR_REQUEST_CALIB_FACTORY)
        {
            MotorDriver_InitialPwmDutyCycle(NV_STARTUP_CURR_MAX, g_u16MinSpeedRPM);  /* Stepper */
        }
        else
#endif /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */
        {
#if (_SUPPORT_AUTO_SPEED_FOC == FALSE)
            MotorDriver_InitialPwmDutyCycle(NV_STARTUP_CURR_MAX, g_u16LowSpeedRPM);  /* FOC Close-loop */
#else  /* (_SUPPORT_AUTO_SPEED_FOC == FALSE) */
            MotorDriver_InitialPwmDutyCycle(Get_ActCurrRunMax_mA(), g_u16MinSpeedRPM);  /* FOC Open-loop */
#endif /* (_SUPPORT_AUTO_SPEED_FOC == FALSE) */
        }
#endif /* (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) */

#if (_SUPPORT_SOFT_START != FALSE)
        l_e8MotorStartupMode = E_MSM_STEPPER_ALIGN;
        l_u16StartCorrectionRatio = l_u16CorrectionRatio;
#if (C_ALIGN_STEP_1_PERIOD == 64U) && (C_ALIGN_STEP_2A_PERIOD == 64U) && (_SUPPORT_SINCOS_TABLE_SZ == 256)
        l_u16CorrectionRatio = C_PWM_MIN_DC + (l_u16StartCorrectionRatio >> 1) +
            mulI16hi_I16byI16(l_u16StartCorrectionRatio, c_ai16MicroStepVector3PH_SinCos256[l_u16RampStep - C_COS_OFFSET]);
#elif (C_ALIGN_STEP_1_PERIOD == 48U) && (C_ALIGN_STEP_2A_PERIOD == 48U) && (_SUPPORT_SINCOS_TABLE_SZ == 192)
        l_u16CorrectionRatio = C_PWM_MIN_DC + (l_u16StartCorrectionRatio >> 1) +
            mulI16hi_I16byI16(l_u16StartCorrectionRatio, c_ai16MicroStepVector3PH_SinCos192[l_u16RampStep - C_COS_OFFSET]);
#else
        l_u16CorrectionRatio = C_PWM_MIN_DC;
#endif
#else  /* (_SUPPORT_SOFT_START != FALSE) */
        l_e8MotorStartupMode = E_MSM_STEPPER_A;
#endif /* (_SUPPORT_SOFT_START != FALSE) */

#if (_SUPPORT_VARIABLE_PWM != FALSE)                                            /* MMP220815-1 */
#if (_SUPPORT_VAR_PWM_MODE == C_VAR_PWM_MODE_ON_DUTY_CYCLE)
        MotorDriverReconfigure(FALSE);                                          /* Start-up with 2-PWM's (CL) */
#else  /* (_SUPPORT_CLOSED_LOOP_STARTUP != FALSE) */
        MotorDriverReconfigure(TRUE);                                           /* Start-up with 3-PWM's (OL) */
#endif /* (_SUPPORT_CLOSED_LOOP_STARTUP != FALSE) */
#endif /* (_SUPPORT_VARIABLE_PWM != FALSE) */
#if (_SUPPORT_STALLDET_LA != FALSE) && (_SUPPORT_CLOSED_LOOP_STARTUP == FALSE)
        /*l_i16StallThresholdLA = (C_STALL_LA_THRESHOLD_FOC * C_ANGLE_3DEG);*/  /* MMP230912-1 */
        l_i16StallThresholdLA_Max = (NV_STALL_LA_THRSHLD_OL * C_ANGLE_3DEG);    /* MMP240725-2 */
#endif /* (_SUPPORT_STALLDET_LA != FALSE) && (_SUPPORT_CLOSED_LOOP_STARTUP == FALSE) */
    }
    l_u16MotorDriverDisconDelay = 0U;                                           /* Clear Discon-delay after SRP */

#if (C_MOTOR_PHASES == 3)
    MotorDriver_3Phase(l_u16MicroStepIdx);
#else  /* (C_MOTOR_PHASES == 3) */
    MotorDriver_4Phase(l_u16MicroStepIdx);
#endif /* (C_MOTOR_PHASES == 3) */

#if (PWM_COMM != FALSE) && (_SUPPORT_WINDMILL || _SUPPORT_SRP)
#if (C_PWM_PERIOD_DIV == 0)
    PWMin_Start(FALSE);                                                         /* Start single PWM-IN measurement based of timer-divider */
#else  /* (C_PWM_PERIOD_DIV == 0) */
    PWMin_Start(TRUE);                                                          /* Start continuously PWM-IN measurement based of timer-divider */
#endif /* (C_PWM_PERIOD_DIV == 0) */
#endif /* (PWM_COMM != FALSE) && (_SUPPORT_WINDMILL || _SUPPORT_SRP) */

    /* Switch Drive to active synchronous with PWM-period */
    HAL_PWM_MasterPendClear();
    HAL_PWM_MasterPendWait();
    {
        /* Wait for End-of-PWM period */
    }
#if defined (__MLX81160__)
    DRVCFG_PWM_UVW();
    DRVCFG_ENA_RSTUVW();                                                        /* Enable the driver and the PWM phase W, V and U */
#elif defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__)
#if (C_MOTOR_PHASES == 3)
    DRVCFG_PWM_UVW();
    DRVCFG_ENA_UVW();                                                           /* Enable the driver and the PWM phase W, V and U */
#else  /* (C_MOTOR_PHASES == 3) */
    DRVCFG_PWM_TUVW();
    DRVCFG_ENA_TUVW();                                                          /* Enable the driver and the PWM phase W, V, U and T */
#endif /* (C_MOTOR_PHASES == 3) */
#elif defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
    DRVCFG_ENA();                                                               /* Enable the driver and the PWM phase W, V and U */
    DRVCFG_PWM_UVW();                                                           /* Enable driver before setting PWM to DRV_CTRL */
#endif
#if (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE)
    l_u8MotorHoldingCurrState = FALSE;
    l_u16RampDownSteps = 0U;
#endif /* (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) */

    /* Setup ADC for Motor Current/Voltage measurements */
#if (_SUPPORT_MOTION_DET != C_MOTION_DET_NONE)
    HAL_ADC_StopSafe();
#endif /* (_SUPPORT_MOTION_DET != C_MOTION_DET_NONE) */

#if (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE)
    p_CpyU32_U32( (const uint32_t *)&l_u32ActualPosition, &l_u32StartPosition);
#endif /* (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) */

    /* Start commutation timer */
#if (_SUPPORT_WINDMILL != FALSE)
    if ( (l_e8MotorStartupMode == E_MSM_FOC_2PWM)
#if (_SUPPORT_BRAKING != FALSE)
        || (l_e8MotorStartupMode == E_MSM_FOC_BRAKING)
#endif /* (_SUPPORT_BRAKING != FALSE) */
        )
    {
        ADC_Start(TRUE);                                                        /* FOC based need ADC IRQ */
        MotorStallSwitchOpen2CloseA();
        l_u16MotorVoltageAngle = p_DivU16_U16hibyU16(l_u16MicroStepIdx,
                                                     l_u16MotorMicroStepsPerElecRotation);
        /* Switch from CTimer to ADC IRQ */
    }
    else
#endif /* (_SUPPORT_WINDMILL != FALSE) */
    {
        ADC_Start(FALSE);                                                       /* Stepper-mode doesn't need ADC IRQ (CTimer based) */
        g_u16ActualCommutTimerPeriod = l_u16CommutTimerPeriod;
        l_u32CommutTimerPeriodLPF = ((uint32_t)l_u16CommutTimerPeriod << 16);
        IO_CTIMER0_CTRL = C_TMRx_CTRL_MODE0;                                    /* Timer mode */
        if (l_e8MotorStartupMode == E_MSM_STEPPER_ALIGN)
        {
            IO_CTIMER0_TREGB = (l_u16CommutTimerPeriod >> 1U);
        }
        else
        {
            IO_CTIMER0_TREGB = l_u16CommutTimerPeriod;
        }
        IO_CTIMER0_CTRL = B_CTIMER0_START;
    }

#if (_SUPPORT_BRAKING != FALSE)
    if (l_e8MotorStartupMode == E_MSM_FOC_BRAKING)
    {
        g_e8MotorStatus = C_MOTOR_STATUS_BRAKING;
    }
    else
#endif /* (_SUPPORT_BRAKING != FALSE) */
#if (_SUPPORT_SOFT_START != FALSE)
    if (l_e8MotorStartupMode == E_MSM_STEPPER_ALIGN)
    {
        g_e8MotorStatus = C_MOTOR_STATUS_SOFT_START;
    }
    else
#endif /* (_SUPPORT_SOFT_START != FALSE) */
    {
        g_e8MotorStatus = C_MOTOR_STATUS_RUNNING;
    }

#if (_SUPPORT_FAST_STOP != FALSE)
    IO_PORT_MISC_OUT = (IO_PORT_MISC_OUT & ~IO_PORT_MISC_OUT) | (5U * C_PORT_MISC_OUT_PRUV_0);  /* Set UV at 9V (MMP231121-1) */
#endif /* (_SUPPORT_FAST_STOP != FALSE) */

#if (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE)
    l_u8MotorHoldDelay = 0U;
#endif /* (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) */

#if (_DEBUG_COMMUT_ISR != FALSE)
    DEBUG_CLR_IO_B();
#endif /* (_DEBUG_COMMUT_ISR != FALSE) */

#if (_SUPPORT_VARIABLE_PWM != FALSE)
    /* Add delay for one PWM-period, to make sure the ADC has sampled all channels at least once */
    DELAY(4U * C_DELAY_mPWM);  /*lint !e522 */
#endif /* (_SUPPORT_VARIABLE_PWM != FALSE) */

#if (_DEBUG_MOTOR_START != FALSE)
    DEBUG_CLR_IO_E();
#endif /* (_DEBUG_MOTOR_START != FALSE) */
} /* End of MotorDriverStart */

#if (_SUPPORT_FOC_MODE == (FOC_OBSERVER_SENSORLESS | FOC_MODE_ID_IQ))
/*!*************************************************************************** *
 * MotorDriverFOC_IDIQ
 * \brief   Motor Driver ...
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  (uint16_t) l_u16VoltageAngle
 * *************************************************************************** *
 * \details
 * *************************************************************************** *
 * - Call Hierarchy: ISR_CTIMER0_3(), ADC_ISR()
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 0
 * *************************************************************************** */
#if (_SUPPORT_RAM_FUNC != FALSE)
static __inline__ uint16_t MotorDriverFOC_IDIQ(void) __attribute__ ((section(".ramfunc"))) __attribute__ ((aligned(8)));
#elif (_SUPPORT_OPTIMIZE_FOR_SPEED != FALSE)
static __inline__ uint16_t MotorDriverFOC_IDIQ(void) __attribute__((aligned(8)));
#endif /* (_SUPPORT_OPTIMIZE_FOR_SPEED != FALSE) */
static __inline__ uint16_t MotorDriverFOC_IDIQ(void)
{
#if (_DEBUG_MOTOR_CURRENT_FLT != FALSE) && (_DEBUG_MCUR_SENSE != FALSE)
#if (_DEBUG_MCUR_SUB_SAMPLED != FALSE)
    if ( (g_u16SubSamplingIdx & C_SUB_SAMPLE_MASK) == 0U)
#endif /* (_DEBUG_MCUR_SUB_SAMPLED != FALSE) */
    {
        uint8_t *pBfr = &g_au8DebugBuf[g_u16DebugBufWrIdx];
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81350__)
        /* 10-bit ADC */
        pBfr[0] = (uint8_t)(g_i16MotorCurrentCoilA >> 1);                       /* Motor Current Coil A */
        pBfr[1] = (uint8_t)(g_i16MotorCurrentCoilB >> 1);                       /* Motor Current Coil B */
        pBfr[2] = (uint8_t)(g_i16MotorCurrentCoilC >> 1);                       /* Motor Current Coil C */
        /*pBfr[3] = (uint8_t)(l_u16Ipk >> 1);*/
#else
        /* 12-bit ADC */
        pBfr[0] = (uint8_t)(g_i16MotorCurrentCoilA >> 3);                       /* Motor Current Coil A */
        pBfr[1] = (uint8_t)(g_i16MotorCurrentCoilB >> 3);                       /* Motor Current Coil B */
        pBfr[2] = (uint8_t)(g_i16MotorCurrentCoilC >> 3);                       /* Motor Current Coil C */
        /*pBfr[3] = (uint8_t)(l_u16Ipk >> 3);*/
#endif
        g_u16DebugBufWrIdx += 3U;
#if (_DEBUG_MCUR_CYCLIC != FALSE)
        if (g_u16DebugBufWrIdx >= C_DEBUG_BUF_SZ)
        {
            g_u16DebugBufWrIdx = 0U;
        }
#endif /* (_DEBUG_MCUR_CYCLIC != FALSE) */
    }
#endif /* _DEBUG_MOTOR_CURRENT_FLT && (_DEBUG_MCUR_SENSE != FALSE) */

    /* Ialpha = CurrentCoilA * 1.5
     * Ibeta = (CurrentCoilB - CurrentCoilC) * COS(30 deg)
     *       = (CurrentCoilB - CurrentCoilC) * SQRT(3)/2
     * (Divide by 1.5)
     * Ialpha' = CurrentCoilA;
     * Ibeta' = (CurrentCoilB - CurrentCoilC) * 1/SQRT(3) = CurrentCoilY;
     */
    int16_t i16Vpk = (int16_t)p_MulDivU16_U16byU16byU16(Get_RawVmotorF(),       /* Vpk = VSM[ADC_LSB_V] * PWM-DC / PWM_PERIOD; [ADC_LSB_V] */
                                                        l_u16CorrectionRatio,
                                                        C_PWM_DC_MAX);
    uint16_t u16Vr = p_MulU16_U16byU16lsr10(l_u16Ipk, l_u16mR_AT);              /* Vr = Ipk * R'; [ADC_LSB_V] (MMP210129-1) */
    g_u16ActualMotorSpeedRPMe = (uint16_t)(p_MulU32lo_U32byU16(l_u32MotorSpeedRadPSe, 60U) >> 16);
#if (_SUPPORT_PID_U32 == FALSE)
    uint16_t u16Vz = p_DivU16_U32byU16(p_MulU32_U16byU16lsr8(l_u16Ipk, g_u16ActualMotorSpeedRPMe), l_u16mZ);    /* Vz = (Ipk * Speed) / L' */
#else  /* (_SUPPORT_PID_U32 == FALSE) */
    uint16_t u16Vz =
        p_DivU16_U32byU16(p_MulU32_U16byU16lsr8( (l_u16Ipk * C_FOC_PID_MULT),
                                                 (uint16_t)(g_u32ActualMotorSpeedRPMe / C_FOC_PID_MULT)), l_u16mZ);
#endif /* (_SUPPORT_PID_U32 == FALSE) */
    int16_t i16TanV = l_u16VoltageAngle; /* TODO[MMP]: Prev V-angle */
#if (_SUPPORT_MOTOR_DIRECTION == C_MOTOR_DIRECTION_CW)
    int16_t i16TanIV = (i16TanV - g_i16MotorCurrentAngle);
#elif (_SUPPORT_MOTOR_DIRECTION == C_MOTOR_DIRECTION_CCW)
    int16_t i16TanIV = (g_i16MotorCurrentAngle - i16TanV);
#elif (_SUPPORT_MOTOR_DIRECTION == C_MOTOR_DIRECTION_CMD)
    int16_t i16TanIV = (i16TanV - g_i16MotorCurrentAngle);
    if (g_e8MotorDirectionCCW != FALSE)
    {
        i16TanIV = -i16TanIV;
    }
#endif /* (_SUPPORT_MOTOR_DIRECTION) */
    {
#if (_SUPPORT_SINCOS_TABLE_SZ == 256)
        int16_t *pi16SinCos = (int16_t *)&c_ai16MicroStepVector3PH_SinCos256[((uint16_t)i16TanIV) / _SUPPORT_SINCOS_TABLE_SZ];
#elif (_SUPPORT_SINCOS_TABLE_SZ == 192)
        int16_t *pi16SinCos =
            (int16_t *)&c_ai16MicroStepVector3PH_SinCos192[p_MulU16hi_U16byU16( (uint16_t)i16TanIV, _SUPPORT_SINCOS_TABLE_SZ)];
#elif (_SUPPORT_SINCOS_TABLE_SZ == 1024)
        int16_t *pi16SinCos = p_SinDirectLutInlined_2_5k(i16TanIV);
#else
#error "ERROR: Unsupported table"
#endif
        int16_t i16V_Iy = p_MulI16_I16bypQ15(i16Vpk, pi16SinCos);               /* V_Iy = Vpk * SIN(IV); [ADC_LSB_V] */
        int16_t i16V_Ix = p_MulI16_I16bypQ15(i16Vpk, (pi16SinCos + C_COS_OFFSET));  /* V_Ix = Vpk * COS(IV); [ADC_LSB_V] */
        int16_t i16B_Ix = i16V_Ix - u16Vr;                                      /* B_Ix = V_Ix - Vr; [ADC_LSB_V] */
        int16_t i16B_Iy = i16V_Iy - u16Vz;                                      /* B_Iy = V_Iy - Vz; [ADC_LSB_V] */
#if (_SUPPORT_FAST_ATAN2 != FALSE)
        int16_t i16TanIB = (int16_t)p_atan2I16(i16B_Iy, i16B_Ix);               /* Direct Load Angle; 2^16 = 360 deg or 2pi */
#else  /* (_SUPPORT_FAST_ATAN2 != FALSE) */
        int16_t i16TanIB = (int16_t)atan2I16(i16B_Iy, i16B_Ix);                 /* Direct Load Angle; 2^16 = 360 deg or 2pi */
#endif /* (_SUPPORT_FAST_ATAN2 != FALSE) */

#if (_SUPPORT_STALLDET_LA != FALSE)
        l_i16ActLoadAngleLPF =
            p_LpfI16_I16byI16(&l_i32ActLoadAngleLPF, C_LA_LPF_COEF_CL, (i16TanIB - l_i16ActLoadAngleLPF));
#endif /* (_SUPPORT_STALLDET_LA != FALSE) */

#if (_DEBUG_MOTOR_CURRENT_FLT != FALSE) && (_DEBUG_MCUR_FOC != FALSE)
#if (_DEBUG_MCUR_SUB_SAMPLED != FALSE)
    if ( (g_u16SubSamplingIdx & C_SUB_SAMPLE_MASK) == 0U)
#endif /* (_DEBUG_MCUR_SUB_SAMPLED != FALSE) */
    {
        uint8_t *pBfr = &g_au8DebugBuf[g_u16DebugBufWrIdx];
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81350__)
        /* 10-bit ADC */
        pBfr[0] = (uint8_t)(l_u16Ipk >> 1);                                     /* Motor Current Ipk */
        pBfr[1] = (uint8_t)(g_i16MotorCurrentAngle >> 8);                       /* Motor Current Angle Ipk */
        pBfr[2] = (uint8_t)(i16TanIB >> 8);                                     /* BEMF-Ipk angle */
#else
        /* 12-bit ADC */
        pBfr[0] = (uint8_t)(l_u16Ipk >> 3);                                     /* Motor Current Ipk */
        pBfr[1] = (uint8_t)(g_i16MotorCurrentAngle >> 8);                       /* Motor Current Angle Ipk */
        pBfr[2] = (uint8_t)(i16TanIB >> 8);                                     /* BEMF-Ipk angle */
#endif
        g_u16DebugBufWrIdx += 3U;
#if (_DEBUG_MCUR_CYCLIC != FALSE)
        if (g_u16DebugBufWrIdx >= C_DEBUG_BUF_SZ)
        {
            g_u16DebugBufWrIdx = 0U;
        }
#endif /* (_DEBUG_MCUR_CYCLIC != FALSE) */
    }
#endif /* _DEBUG_MOTOR_CURRENT_FLT && (_DEBUG_MCUR_FOC != FALSE) */

        /* Calculate rotor position based on measurement */
        uint16_t u16Temp = l_u32MotorSpeedRadPSe / C_IDIQ_FACTOR;
        /* Calculate predicted Motor Voltage Angle based on the Last Angle and speed found in previous iteration */
        /* u16MotorVoltageAngleUpdate = l_u32MotorSpeedRadPSe / FOC_FREQ
         * uint16_t u16MotorVoltageAngleUpdate = p_DivU16_U32byU16(l_u32MotorSpeedRadPSe, FOC_FREQ);
         */
        uint16_t u16MotorVoltageAngleUpdate =
            p_MulU16hi_U16byU16(u16Temp, (uint16_t)((C_IDIQ_FACTOR * C_ANGLE_360DEG) / FOC_FREQ));
#if (_SUPPORT_MOTOR_DIRECTION == C_MOTOR_DIRECTION_CMD)
        if (g_e8MotorDirectionCCW != FALSE)
#endif /* (_SUPPORT_MOTOR_DIRECTION == C_MOTOR_DIRECTION_CMD) */
#if (_SUPPORT_MOTOR_DIRECTION == C_MOTOR_DIRECTION_CCW) || (_SUPPORT_MOTOR_DIRECTION == C_MOTOR_DIRECTION_CMD)
        {
            /* Closing (TODO[MMP] + 90deg) */
            int16_t i16AngleError;
            l_u16MotorVoltageAngle = l_u16MotorVoltageAngle + u16MotorVoltageAngleUpdate;
            /* Kalman-filter alpha-beta */
            i16AngleError = (int16_t)((i16TanIB - g_i16MotorCurrentAngle) - l_u16MotorVoltageAngle);
            i16AngleError = p_MulI16hi_I16byI16( (int16_t)u16Temp, i16AngleError);
            l_u16MotorVoltageAngle = l_u16MotorVoltageAngle + p_MulI16_I16byQ15(i16AngleError, C_IDIQ_ALPHA);
            l_u32MotorSpeedRadPSe = l_u32MotorSpeedRadPSe + p_MulI32_I16byI16(i16AngleError, C_IDIQ_BETA);
        }
#endif /* (_SUPPORT_MOTOR_DIRECTION == C_MOTOR_DIRECTION_CCW) || (_SUPPORT_MOTOR_DIRECTION == C_MOTOR_DIRECTION_CMD) */
#if (_SUPPORT_MOTOR_DIRECTION == C_MOTOR_DIRECTION_CMD)
        else
#endif /* (_SUPPORT_MOTOR_DIRECTION == C_MOTOR_DIRECTION_CMD) */
#if (_SUPPORT_MOTOR_DIRECTION == C_MOTOR_DIRECTION_CW) || (_SUPPORT_MOTOR_DIRECTION == C_MOTOR_DIRECTION_CMD)
        {
            /* Opening */
            int16_t i16AngleError;
            l_u16MotorVoltageAngle = l_u16MotorVoltageAngle - u16MotorVoltageAngleUpdate;
            /* Kalman-filter alpha-beta */
            i16AngleError = (int16_t)((-i16TanIB - g_i16MotorCurrentAngle) - l_u16MotorVoltageAngle);
            i16AngleError = p_MulI16hi_I16byI16( (int16_t)u16Temp, i16AngleError);
            l_u16MotorVoltageAngle = l_u16MotorVoltageAngle + p_MulI16_I16byQ15(i16AngleError, C_IDIQ_ALPHA);
            l_u32MotorSpeedRadPSe = l_u32MotorSpeedRadPSe + p_MulI32_I16byI16(i16AngleError, -C_IDIQ_BETA);
        }
#endif /* (_SUPPORT_MOTOR_DIRECTION == C_MOTOR_DIRECTION_CW) || (_SUPPORT_MOTOR_DIRECTION == C_MOTOR_DIRECTION_CMD) */

        /*TODO[MMP]: Min.speed check */

        /* Park Transformation of the current */
        {
#if (_SUPPORT_SINCOS_TABLE_SZ == 256)
            int16_t *pi16SinRotorAngle = (int16_t *)&c_ai16MicroStepVector3PH_SinCos256[l_u16MotorVoltageAngle / _SUPPORT_SINCOS_TABLE_SZ];
#elif (_SUPPORT_SINCOS_TABLE_SZ == 192)
            int16_t *pi16SinRotorAngle =
                (int16_t *)&c_ai16MicroStepVector3PH_SinCos192[p_MulU16hi_U16byU16(l_u16MotorVoltageAngle, _SUPPORT_SINCOS_TABLE_SZ)];
#elif (_SUPPORT_SINCOS_TABLE_SZ == 1024)
            int16_t *pi16SinRotorAngle = p_SinDirectLutInlined_2_5k(l_u16MotorVoltageAngle);
#else
#error "ERROR: Unsupported table"
#endif
            /* Id = Ialpha * cos + Ibeta * sin
             * Iq = Ibeta * cos - Ialpha * sin
             */
            int16_t i16Sine = p_GetSine((int16_t)pi16SinRotorAngle);
            int16_t i16Id = p_MulI16_I16byQ15x(g_i16MotorCurrentCoilY, i16Sine);
            int16_t i16Iq = p_MulI16_I16byQ15x(g_i16MotorCurrentCoilA, i16Sine);
            int16_t i16Cosine = p_GetSine((int16_t)(pi16SinRotorAngle + C_COS_OFFSET));
            i16Iq = p_MulI16_I16byQ15x(g_i16MotorCurrentCoilY, i16Cosine) - i16Iq;
            i16Id += p_MulI16_I16byQ15x(g_i16MotorCurrentCoilA, i16Cosine);

#if (_DEBUG_MOTOR_CURRENT_FLT != FALSE) && (_DEBUG_MCUR_FOC_PID != FALSE)
#if (_DEBUG_MCUR_SUB_SAMPLED != FALSE)
            if ( (g_u16SubSamplingIdx & C_SUB_SAMPLE_MASK) == 0U)
#endif /* (_DEBUG_MCUR_SUB_SAMPLED != FALSE) */
            {
                uint8_t *pBfr = &g_au8DebugBuf[g_u16DebugBufWrIdx];
                pBfr[0] = (uint8_t)(l_u16MotorVoltageAngle >> 8);               /* Rotor Angle */
                pBfr[1] = (uint8_t)(i16Id >> 1);                                /* FOC Id Current */
                pBfr[2] = (uint8_t)(i16Iq >> 1);                                /* FOC Iq Current */
                g_u16DebugBufWrIdx += 3U;
#if (_DEBUG_MCUR_CYCLIC != FALSE)
                if (g_u16DebugBufWrIdx >= C_DEBUG_BUF_SZ)
                {
                    g_u16DebugBufWrIdx = 0U;
                }
#endif /* (_DEBUG_MCUR_CYCLIC != FALSE) */
            }
#endif /* _DEBUG_MOTOR_CURRENT_FLT && (_DEBUG_MCUR_FOC_PID != FALSE) */

#if (_DEBUG_MOTOR_CURRENT_FLT != FALSE) && (_DEBUG_MCUR_FOC_PID != FALSE) && (_DEBUG_MCUR_SUB_SAMPLED != FALSE)
            PID_FOC_IDIQ(i16Id, i16Iq, i16Sine, i16Cosine, g_u16SubSamplingIdx);
#else  /* (_DEBUG_MOTOR_CURRENT_FLT != FALSE) && (_DEBUG_MCUR_FOC_PID != FALSE) && (_DEBUG_MCUR_SUB_SAMPLED != FALSE) */
            PID_FOC_IDIQ(i16Id, i16Iq, i16Sine, i16Cosine);
#endif /* (_DEBUG_MOTOR_CURRENT_FLT != FALSE) && (_DEBUG_MCUR_FOC_PID != FALSE) && (_DEBUG_MCUR_SUB_SAMPLED != FALSE) */

#if (_DEBUG_MOTOR_CURRENT_FLT != FALSE) && (_DEBUG_MCUR_FOC_PID != FALSE)
#if (_DEBUG_MCUR_SUB_SAMPLED != FALSE)
            if ( (g_u16SubSamplingIdx & C_SUB_SAMPLE_MASK) == 0U)
#endif /* (_DEBUG_MCUR_SUB_SAMPLED != FALSE) */
            {
                uint8_t *pBfr = &g_au8DebugBuf[g_u16DebugBufWrIdx];
                pBfr[0] = (uint8_t)(l_u16VoltageAngle >> 8);                    /* Voltage Vector Angle */
                pBfr[1] = (uint8_t)(l_i16Valpha >> 1);                          /* V-alpha */
                pBfr[2] = (uint8_t)(l_i16Vbeta >> 1);                           /* V-beta */
                g_u16DebugBufWrIdx += 3U;
#if (_DEBUG_MCUR_CYCLIC != FALSE)
                if (g_u16DebugBufWrIdx >= C_DEBUG_BUF_SZ)
                {
                    g_u16DebugBufWrIdx = 0U;
                }
#endif /* (_DEBUG_MCUR_CYCLIC != FALSE) */
            }
#endif /* _DEBUG_MOTOR_CURRENT_FLT && (_DEBUG_MCUR_FOC_PID != FALSE) */

#if (_SUPPORT_STALLDET_FLUX != FALSE)
            {
                int16_t i16FluxAlpha = (l_i16Valpha - p_MulI16_I16byI16asr10(g_i16MotorCurrentCoilA, l_u16mR_AT)) -
                                       p_DivI16_I32byI16(p_MulI32_I16byI16asr8(g_i16MotorCurrentCoilA, g_u16ActualMotorSpeedRPMe),
                                                         l_u16mZ);
                int16_t i16FluxBeta = l_i16Vbeta - p_MulI16_I16byI16asr10(g_i16MotorCurrentCoilY, l_u16mR_AT) -
                                      p_DivI16_I32byI16(p_MulI32_I16byI16asr8(g_i16MotorCurrentCoilY, g_u16ActualMotorSpeedRPMe),
                                                        l_u16mZ);
                l_u16Flux = p_AproxSqrtU16_I16byI16(i16FluxAlpha, i16FluxBeta);
            }
#endif /* (_SUPPORT_STALLDET_FLUX != FALSE) */

#if (_SUPPORT_FOC_ID_IQ_MODE == FOC_ID_IQ_iCLARKE)
            {
                /* Approximate 4.5us faster (MMP220722-1) */
                /* Inverse Clarke:
                 * Vu = Valpha
                 * Vv = -0.5 * Valpha + SQRT(3)/2 * Vbeta
                 * Vw = -0.5 * Valpha - SQRT(3)/2 * Vbeta */
                int16_t i16Vu = l_i16Valpha;
                int16_t i16Valpha = -l_i16Valpha / 2;
                int16_t i16Vbeta = p_MulI16_I16byQ15(l_i16Vbeta, C_SQRT3_DIV2);
                int16_t i16Vv = i16Valpha + i16Vbeta;
                int16_t i16Vw = i16Valpha - i16Vbeta;
#if (_SUPPORT_VOLTAGE_SHAPE == VOLTAGE_SHAPE_DSVM_GND)
                /* SVM-L */
                int16_t i16Min = i16Vu;
                if (i16Vv < i16Min)
                {
                    i16Min = i16Vv;
                }
                if (i16Vw < i16Min)
                {
                    i16Min = i16Vw;
                }
#else
#error "ERROR: Cartesian with SVM-L is only supported."
#endif /* (_SUPPORT_VOLTAGE_SHAPE == VOLTAGE_SHAPE_DSVM_GND) */
                i16Vu = (i16Vu - i16Min);
                i16Vv = (i16Vv - i16Min);
                i16Vw = (i16Vw - i16Min);
                if (i16Vw == 0)
                {
                    IO_PWM_SLAVE1_CTRL = B_PWM_SLAVE1_SLAVE | C_PWM_SLAVE1_MODE_MIRROR | B_PWM_SLAVE1_POL;
                    i16Vv = p_MulI16_I16byQ15(i16Vv, (PWM_REG_PERIOD << 4));
                    IO_PWM_SLAVE1_LT = p_ClipMinMaxU16((uint16_t)i16Vv, C_PWM_MIN_DC, l_u16MaxPwmRatio);
                    IO_PWM_SLAVE2_LT = 0U;
                    i16Vu = p_MulI16_I16byQ15(i16Vu, (PWM_REG_PERIOD << 4));
                    IO_PWM_MASTER1_LT =
                        (PWM_SCALE_OFFSET - p_ClipMinMaxU16((uint16_t)i16Vu, C_PWM_MIN_DC, l_u16MaxPwmRatio));
                }
                else if (i16Vv == 0)
                {
                    IO_PWM_SLAVE1_CTRL = B_PWM_SLAVE1_SLAVE | C_PWM_SLAVE1_MODE_MIRROR | B_PWM_SLAVE1_POL;
                    IO_PWM_SLAVE1_LT = 0U;
                    i16Vw = p_MulI16_I16byQ15(i16Vw, (PWM_REG_PERIOD << 4));
                    IO_PWM_SLAVE2_LT = p_ClipMinMaxU16((uint16_t)i16Vw, C_PWM_MIN_DC, l_u16MaxPwmRatio);
                    i16Vu = p_MulI16_I16byQ15(i16Vu, (PWM_REG_PERIOD << 4));
                    IO_PWM_MASTER1_LT =
                        (PWM_SCALE_OFFSET - p_ClipMinMaxU16((uint16_t)i16Vu, C_PWM_MIN_DC, l_u16MaxPwmRatio));
                }
                else
                {
                    IO_PWM_SLAVE1_CTRL = B_PWM_SLAVE1_SLAVE | C_PWM_SLAVE1_MODE_MIRROR;
                    i16Vv = p_MulI16_I16byQ15(i16Vv, (PWM_REG_PERIOD << 4));
                    IO_PWM_SLAVE1_LT =
                        (PWM_SCALE_OFFSET - p_ClipMinMaxU16((uint16_t)i16Vv, C_PWM_MIN_DC, l_u16MaxPwmRatio));
                    i16Vw = p_MulI16_I16byQ15(i16Vw, (PWM_REG_PERIOD << 4));
                    IO_PWM_SLAVE2_LT = p_ClipMinMaxU16((uint16_t)i16Vw, C_PWM_MIN_DC, l_u16MaxPwmRatio);
                    IO_PWM_MASTER1_LT = PWM_REG_PERIOD;
                }
            }
#endif /* (_SUPPORT_FOC_ID_IQ_MODE == FOC_ID_IQ_iCLARKE) */
        }
    }

    return (l_u16VoltageAngle);
} /* End of MotorDriverFOC_IDIQ() */

#else  /* ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_ID_IQ) */

/*!*************************************************************************** *
 * MotorDriverFOC_IB
 * \brief   Motor Driver Load-angle calculation
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  Load-angle
 * *************************************************************************** *
 * \details FOC-mode:
 *          IV: Angle between Coil-current and applied coil voltage.
 *          IB: Angle between Rotor and Coil-current
 *          Performance (at 28MHz):
 *          IV-method: typ. 24.0us
 *          IB-Method I: (2x ATAN + SQRT): typ. 36.2us, max. 36.75us
 *          IB-Method II: (2x ATAN): typ. 36.5us, max. 37.25us
 *          IB-Method III: (1x ATAN + SQRT): 36.0us, max. 36.6us
 * *************************************************************************** *
 * - Call Hierarchy: ISR_CTIMER0_3(), ADC_ISR()
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 0
 * *************************************************************************** */
#if (_SUPPORT_OPTIMIZE_FOR_SPEED != FALSE)
static __inline__ int16_t MotorDriverFOC_IB(int16_t i16MotorVoltageAngle) __attribute__((aligned(8)));
#endif /* (_SUPPORT_OPTIMIZE_FOR_SPEED != FALSE) */
static __inline__ int16_t MotorDriverFOC_IB(int16_t i16MotorVoltageAngle)
{
    int16_t i16Result;
#if ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_IV)
    uint16_t u16MotorVoltageAngle = p_IdxToAngle(l_u16MicroStepIdx, l_u16MotorMicroStepsPerElecRotation);  /*p_DivU16_U32byU16( ((uint32_t)((uint16_t)(l_u16MicroStepIdx * 2U) + 1U) << 16U), (l_u16MotorMicroStepsPerElecRotation * 2U));  / * 2^16 = 360 degrees * / */
    /* Load-angle = VI-angle; Convert 2pi (=2^16) into 2pi (=2^12) */
    if (g_e8MotorDirectionCCW != FALSE)
    {
        i16Result = (int16_t)(g_i16MotorCurrentAngle - u16MotorVoltageAngle);
    }
    else
    {
        i16Result = (int16_t)(u16MotorVoltageAngle - g_i16MotorCurrentAngle);
    }
#elif ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_IB) || ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_IB_IPK)
    int16_t i16Vpk = (int16_t)p_MulDivU16_U16byU16byU16(Get_RawVmotorF(),
                                                        l_u16CorrectionRatio,
                                                        C_PWM_DC_MAX);
    uint16_t u16Vr = p_MulU16_U16byU16lsr10(l_u16Ipk, l_u16mR_AT);              /* [ADC_LSB_V] (MMP210129-1) */
#if (_SUPPORT_PID_U32 == FALSE)
    uint16_t u16Vz = p_DivU16_U32byU16(p_MulU32_U16byU16lsr8(l_u16Ipk, g_u16ActualMotorSpeedRPMe), l_u16mZ);
#else  /* (_SUPPORT_PID_U32 == FALSE) */
    uint16_t u16Vz =
        p_DivU16_U32byU16(p_MulU32_U16byU16lsr8( (l_u16Ipk * C_FOC_PID_MULT),
                                                 (uint16_t)(g_u32ActualMotorSpeedRPMe / C_FOC_PID_MULT)), l_u16mZ);
#endif /* (_SUPPORT_PID_U32 == FALSE) */
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
#endif /* (_SUPPORT_MOTOR_DIRECTION) */
    {
#if (_DEBUG_FOC_PERF != FALSE)
        DEBUG_SET_IO_D();
#endif /* (_DEBUG_FOC_PERF != FALSE) */
#if (_SUPPORT_SINCOS_TABLE_SZ == 256)
        int16_t *pSinCos = (int16_t *)&c_ai16MicroStepVector3PH_SinCos256[((uint16_t)i16TanIV) / _SUPPORT_SINCOS_TABLE_SZ];
#elif (_SUPPORT_SINCOS_TABLE_SZ == 192)
        int16_t *pSinCos =
            (int16_t *)&c_ai16MicroStepVector3PH_SinCos192[p_MulU16hi_U16byU16( (uint16_t)i16TanIV, _SUPPORT_SINCOS_TABLE_SZ)];
#elif (_SUPPORT_SINCOS_TABLE_SZ == 1024)
        int16_t *pSinCos = p_SinDirectLutInlined_2_5k(i16TanIV);
#else
#error "ERROR: Unsupported table"
#endif
        int16_t i16Vy = p_MulI16_I16bypQ15(i16Vpk, pSinCos);                    /* [ADC_LSB_V] */
        int16_t i16Vx = p_MulI16_I16bypQ15(i16Vpk, (pSinCos + C_COS_OFFSET));   /* [ADC_LSB_V] */
#if (_DEBUG_FOC_PERF != FALSE)
        DEBUG_CLR_IO_D();
#endif /* (_DEBUG_FOC_PERF != FALSE) */
        int16_t i16Bx = i16Vx - u16Vr;                                          /* [ADC_LSB_V] */
        int16_t i16By = i16Vy - u16Vz;                                          /* [ADC_LSB_V] */
#if (_DEBUG_FOC_PERF != FALSE)
        DEBUG_SET_IO_D();
#endif /* (_DEBUG_FOC_PERF != FALSE) */
#if (_SUPPORT_FAST_ATAN2 != FALSE)
        int16_t i16TanIB = (int16_t)p_atan2I16(i16By, i16Bx);                   /* 2^16 = 360 deg or 2pi */
#else  /* (_SUPPORT_FAST_ATAN2 != FALSE) */
        int16_t i16TanIB = (int16_t)atan2I16(i16By, i16Bx);                     /* 2^16 = 360 deg or 2pi */
#endif /* (_SUPPORT_FAST_ATAN2 != FALSE) */
#if (_DEBUG_FOC_PERF != FALSE)
        DEBUG_CLR_IO_D();
#endif /* (_DEBUG_FOC_PERF != FALSE) */
        i16Result = i16TanIB;
#if (_SUPPORT_STALLDET_LA != FALSE)
        {
#if (_SUPPORT_PID_U32 == FALSE)
            uint16_t u16Coef = g_u16ActualMotorSpeedRPMe >> 3;                  /* MMP230303-2: Adaptable LA-LPF coefficient */
#else  /* (_SUPPORT_PID_U32 == FALSE) */
            uint16_t u16Coef = (uint16_t) (g_u32ActualMotorSpeedRPMe >> 4);     /* MMP230303-2: Adaptable LA-LPF coefficient */
#endif /* (_SUPPORT_PID_U32 == FALSE) */
            if (u16Coef == 0U)
            {
                u16Coef = C_LA_LPF_COEF_CL;
            }
            l_i16ActLoadAngleLPF =
                p_LpfI16_I16byI16(&l_i32ActLoadAngleLPF, u16Coef, (i16TanIB - l_i16ActLoadAngleLPF));
        }
#endif /* (_SUPPORT_STALLDET_LA != FALSE) */
    }
#endif /* (_SUPPORT_FOC_MODE & FOC_MODE_MASK) */

#if (_DEBUG_MOTOR_CURRENT_FLT != FALSE) && (_DEBUG_MCUR_FOC != FALSE)
#if (_DEBUG_MCUR_SUB_SAMPLED != FALSE)
    if ( (g_u16SubSamplingIdx & C_SUB_SAMPLE_MASK) == 0U)
#endif /* (_DEBUG_MCUR_SUB_SAMPLED != FALSE) */
    {
        uint8_t *pBfr = &g_au8DebugBuf[g_u16DebugBufWrIdx];
        pBfr[0] = (uint8_t)(g_i16MotorCurrentAngle >> 8);
        pBfr[1] = (uint8_t)(i16TanV >> 8);
        pBfr[2] = (uint8_t)(l_i16ActLoadAngleLPF >> 8);
        g_u16DebugBufWrIdx += 3U;
#if (_DEBUG_MCUR_CYCLIC != FALSE)
        if (g_u16DebugBufWrIdx >= C_DEBUG_BUF_SZ)
        {
            g_u16DebugBufWrIdx = 0U;
        }
#endif /* (_DEBUG_MCUR_CYCLIC != FALSE) */
    }
#endif /* _DEBUG_MOTOR_CURRENT_FLT && (_DEBUG_MCUR_FOC != FALSE) */

    return (i16Result);
} /* End of MotorDriverFOC_IB() */
#endif /* ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_ID_IQ) */

#if ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_ID_IQ) || \
    (((_SUPPORT_FOC_MODE & FOC_OBSERVER_MASK) == FOC_OBSERVER_SENSORLESS) && _SUPPORT_CLOSED_LOOP_STARTUP)
/*!*************************************************************************** *
 * MotorDriverFOC_IV
 * \brief   FOC IV Implementation
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  (int16_t) Load-angle
 * *************************************************************************** *
 * \details MMP190508-2
 * *************************************************************************** *
 * - Call Hierarchy: EXT0_IT(), ADC_ISR()
 * - Cyclomatic Complexity: 1+1
 * - Nesting: 1
 * - Function calling: 2 (p_atan2I16(), p_IdxToAngle())
 * *************************************************************************** */
static __inline__ int16_t MotorDriverFOC_IV(void)
{
    int16_t i16Result;
    uint16_t u16MotorVoltageAngle = p_IdxToAngle(l_u16MicroStepIdx, l_u16MotorMicroStepsPerElecRotation);  /*p_DivU16_U32byU16( ((uint32_t)((uint16_t)(l_u16MicroStepIdx * 2U) + 1U) << 16U), (l_u16MotorMicroStepsPerElecRotation * 2U));  / * 2^16 = 360 degrees * / */
    /* Load-angle = VI-angle; Convert 2pi (=2^16) into 2pi (=2^12) */
#if (_SUPPORT_MOTOR_DIRECTION == C_MOTOR_DIRECTION_CW)
    i16Result = (int16_t)(u16MotorVoltageAngle - g_i16MotorCurrentAngle);
#elif (_SUPPORT_MOTOR_DIRECTION == C_MOTOR_DIRECTION_CCW)
    i16Result = (int16_t)(g_i16MotorCurrentAngle - u16MotorVoltageAngle);
#elif (_SUPPORT_MOTOR_DIRECTION == C_MOTOR_DIRECTION_CMD)
    if (g_e8MotorDirectionCCW != FALSE)
    {
        /* Counter Clock Wise */
        i16Result = (int16_t)(g_i16MotorCurrentAngle - u16MotorVoltageAngle);
    }
    else
    {
        /* Clock Wise */
        i16Result = (int16_t)(u16MotorVoltageAngle - g_i16MotorCurrentAngle);
    }
#endif /* _SUPPORT_MOTOR_DIRECTION */
    return (i16Result);
} /* End of MotorDriverFOC_IV() */
#endif

#if ((_SUPPORT_FOC_MODE & FOC_OBSERVER_MASK) == FOC_OBSERVER_SENSORLESS)
/*!*************************************************************************** *
 * SwitchOpen2Close
 * \brief   Switch from open-loop stepper-mode to closed loop IV
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details  Also switch from 192, 96 or 48 micro-step per electric commutation
 *           (stepper-mode) to 48 micro-step IV-mode.
 * *************************************************************************** *
 * - Call Hierarchy: ISR_CTIMER0_3()
 * - Cyclomatic Complexity: 2+1
 * - Nesting: 2
 * - Function calling: 3 (MotorDriverStop(), PID_SwitchOpen2Close(),
 *                        VoltageCorrection())
 * *************************************************************************** */
/* static INLINE */ void SwitchOpen2Close(E_MOTOR_STARTUP_MODE_t u8Mode)
{
#if (_DEBUG_COMMUT_ISR != FALSE)
    DEBUG_SET_IO_A();                                                           /* Switch to IV */
#endif /* (_DEBUG_COMMUT_ISR != FALSE) */
    l_e8MotorStartupMode = u8Mode;
    PID_SwitchOpen2Close();
    if (u8Mode == E_MSM_FOC_CL_STARTUP)
    {
        MotorStallSwitchOpen2CloseA();
    }
#if (_SUPPORT_PID_U32 == FALSE)
    g_u16ActualMotorSpeedRPMe = (uint16_t)p_MulU32_U16byU16(g_u16ActualMotorSpeedRPM, g_u16MotorPolePairs);     /* TODO[MMP]: Remove for test */
#else  /* (_SUPPORT_PID_U32 == FALSE) */
    g_u32ActualMotorSpeedRPMe = p_MulU32_U16byU16(g_u16ActualMotorSpeedRPM, g_u16MotorPolePairs);    /* TODO[MMP]: Remove for test */
#endif /* (_SUPPORT_PID_U32 == FALSE) */
    l_u16MotorVoltageAngle = p_DivU16_U16hibyU16(l_u16MicroStepIdx, l_u16MotorMicroStepsPerElecRotation);
    l_u16PrevMotorVoltageAngle = l_u16MotorVoltageAngle;
#if (_SUPPORT_VARIABLE_PWM != FALSE) && (_SUPPORT_VAR_PWM_MODE == C_VAR_PWM_MODE_ON_STARTUP_MODE)
    MotorDriverReconfigure(FALSE);                                              /* Operation with 2-PWM's (CL) */
#endif /* (_SUPPORT_VARIABLE_PWM != FALSE) && (_SUPPORT_VAR_PWM_MODE == C_VAR_PWM_MODE_ON_STARTUP_MODE) */
    /* Switch from CTimer to ADC IRQ */
    HAL_ADC_EnableIRQ();                                                        /* Enable ADC Interrupt */
    IO_CTIMER0_CTRL = B_CTIMER0_STOP;                                           /* Stop "commutation timer" */
    l_u16CorrectionRatio = VoltageCorrection();

#if (_SUPPORT_STALLDET_LA != FALSE)
    MotorStallSwitchOpen2CloseLA();                                             /* MMP240725-3 */
#endif /* (_SUPPORT_STALLDET_LA != FALSE) */

#if (_DEBUG_COMMUT_ISR != FALSE)
    DEBUG_CLR_IO_A();                                                           /* Switch to IV */
#endif /* (_DEBUG_COMMUT_ISR != FALSE) */
} /* End of SwitchOpen2Close() */
#endif /* ((_SUPPORT_FOC_MODE & FOC_OBSERVER_MASK) == FOC_OBSERVER_SENSORLESS) */

#if ((_SUPPORT_FOC_MODE & FOC_OBSERVER_MASK) == FOC_OBSERVER_SENSORLESS) && (_SUPPORT_CLOSED_LOOP_STARTUP == FALSE)
/*!*************************************************************************** *
 * SwitchClose2Open
 * \brief   Switch closed loop to open-loop stepper-mode
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details (MMP230912-2)
 * *************************************************************************** *
 * - Call Hierarchy: ADC_ISR()
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 1
 * - Function calling: 0
 * *************************************************************************** */
void SwitchClose2Open(void)
{
    l_e8MotorStartupMode = E_MSM_STEPPER_C;
    g_u16ActualCommutTimerPeriod = p_DivU16_U32byU16(Get_MicroStepPeriodOneRPM(), g_u16ActualMotorSpeedRPM);
#if (_SUPPORT_VARIABLE_PWM != FALSE) && (_SUPPORT_VAR_PWM_MODE == C_VAR_PWM_MODE_ON_STARTUP_MODE)
    MotorDriverReconfigure(TRUE);                                               /* Operation with 3-PWM's (OL) */
#endif /* (_SUPPORT_VARIABLE_PWM != FALSE) && (_SUPPORT_VAR_PWM_MODE == C_VAR_PWM_MODE_ON_STARTUP_MODE) */
    HAL_ADC_DisableIRQ();                                                       /* Disable ADC Interrupt */
#if (_SUPPORT_APP_USER_MODE != FALSE)
    ENTER_SECTION(SYSTEM_MODE); /*lint !e534 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
    IO_MLX16_ITC_PEND1_S = B_MLX16_ITC_PEND1_CTIMER0_3;                         /* Clear pending CTimer0 periodic interrupt event */
#if (_SUPPORT_APP_USER_MODE != FALSE)
    EXIT_SECTION(); /*lint !e438 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
    IO_CTIMER0_CTRL = B_CTIMER0_START;                                          /* Start "commutation timer" */

#if (_SUPPORT_STALLDET_LA != FALSE)
    MotorStallInitLA(g_u16ActualMotorSpeedRPM);
    l_u16StallDetectorDelay = C_DETECTOR_DELAY;
#endif /* (_SUPPORT_STALLDET_LA != FALSE) */
} /* End of SwitchClose2Open() */
#endif /* ((_SUPPORT_FOC_MODE & FOC_OBSERVER_MASK) == FOC_OBSERVER_SENSORLESS) && (_SUPPORT_CLOSED_LOOP_STARTUP == FALSE) */

#if (_SUPPORT_SOFT_START != FALSE)
/*!*************************************************************************** *
 * MotorDriverAlignment
 * \brief   Motor Alignment
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  (uint16_t) FALSE: Alignment not done; TRUE: Alignment done.
 * *************************************************************************** *
 * \details STEP 1: Increasing alignment current (-Cosine)
 *          STEP 2A: Increasing alignment current (Sine)
 *          STEP 2B: Decreasing alignment current (Cosine)
 *          STEP 2C: Constant
 *          |
 *          |          ***|***
 *          |        **   |   **
 *          |       *     |     **
 *          |      *      |       ***|*********
 *          |     *|      |          |
 *          |   ** |      |          |
 *          |***   |      |          |
 *          +------+------+----------+---------
 *          |STEP 1|STEP2A|  STEP2B  | STEP 2C
 *
 *          STEP_SHORT: Increasing alignment current (Linear)
 *          |
 *          |     *|
 *          |    * |
 *          |   *  |
 *          |  *   |
 *          | *    |
 *          |*     |
 *          +------+-------------
 *          | SHORT| ~100ms
 *
 * *************************************************************************** *
 * - Call Hierarchy: ISR_CTIMER0_3()
 * - Cyclomatic Complexity: 12+1
 * - Nesting: 5
 * - Function calling: 0
 * *************************************************************************** */
static uint16_t MotorDriverAlignment(void)
{
    uint16_t u16AlignmentDone = TRUE;

    /* Soft Alignment */
    if (l_u16RampStep == 0U)
    {
        /* Switch to next Alignment step */
        if (l_e8AlignmentStep == E_ALIGN_STEP_1)
        {
            /* Update micro-step index by 1 full-step */
            uint16_t u16MicroStepIdx = l_u16MicroStepIdx;
            if (g_e8MotorDirectionCCW != ((uint8_t)C_MOTOR_DIR_CW) )
            {
                /* Counter Clock-wise (Closing) */
                if (u16MicroStepIdx < C_MICROSTEP_PER_FULLSTEP)
                {
                    u16MicroStepIdx += l_u16MotorMicroStepsPerElecRotation;     /* Underflow; start from 'end' */
                }
                u16MicroStepIdx -= C_MICROSTEP_PER_FULLSTEP;
            }
            else
            {
                /* Clock-wise (Opening) */
                u16MicroStepIdx += C_MICROSTEP_PER_FULLSTEP;
                if (u16MicroStepIdx >= l_u16MotorMicroStepsPerElecRotation)
                {
                    u16MicroStepIdx -= l_u16MotorMicroStepsPerElecRotation;     /* Overflow; start from 'begin' */
                }
            }
            l_u16MicroStepIdx = u16MicroStepIdx;

            l_u16RampStep = C_ALIGN_STEP_2A_PERIOD;
            l_e8AlignmentStep = E_ALIGN_STEP_2A;
        }
        else if (l_e8AlignmentStep == E_ALIGN_STEP_2A)
        {
            l_u16RampStep = C_ALIGN_STEP_2B_PERIOD;
            l_e8AlignmentStep = E_ALIGN_STEP_2B;
        }
        else if (l_e8AlignmentStep == E_ALIGN_STEP_2B)
        {
            l_u16RampStep = C_ALIGN_STEP_2C_PERIOD;
            l_e8AlignmentStep = E_ALIGN_STEP_2C;
        }
        else /* if ((l_u8AlignmentStep == E_ALIGN_STEP_2C) || (l_e8AlignmentStep == E_ALIGN_STEP_SHORT)) */
        {
            /* Alignment done */
            l_u16CorrectionRatio = l_u16StartCorrectionRatio;
            IO_CTIMER0_TREGB = l_u16CommutTimerPeriod;
            l_e8MotorStartupMode = E_MSM_STEPPER_A;
            g_e8MotorStatus = C_MOTOR_STATUS_RUNNING;
            l_e8AlignmentStep = E_ALIGN_STEP_DONE;
        }
    }

    if (l_e8AlignmentStep != E_ALIGN_STEP_DONE)
    {
        l_u16RampStep--;
        if (l_e8AlignmentStep == E_ALIGN_STEP_SHORT)
        {
            l_u16CorrectionRatio = l_u16StartCorrectionRatio - p_MulDivU16_U16byU16byU16(l_u16StartCorrectionRatio,
                                                                                         l_u16RampStep,
                                                                                         C_ALIGN_SOFT_PERIOD);
        }
        else if (l_e8AlignmentStep == E_ALIGN_STEP_1)
        {
            /* Increasing alignment current (-Cosine) */
#if (C_ALIGN_STEP_1_PERIOD == 64U) && (_SUPPORT_SINCOS_TABLE_SZ == 256)
            l_u16CorrectionRatio = C_PWM_MIN_DC + (l_u16StartCorrectionRatio >> 1) +
                mulI16hi_I16byI16(l_u16StartCorrectionRatio, c_ai16MicroStepVector3PH_SinCos256[(2U * C_COS_OFFSET) + l_u16RampStep]);
#elif (C_ALIGN_STEP_1_PERIOD == 48U) && (_SUPPORT_SINCOS_TABLE_SZ == 192)
            l_u16CorrectionRatio = C_PWM_MIN_DC + (l_u16StartCorrectionRatio >> 1) +
                mulI16hi_I16byI16(l_u16StartCorrectionRatio, c_ai16MicroStepVector3PH_SinCos192[(2U * C_COS_OFFSET) + l_u16RampStep]);
#else
            l_u16CorrectionRatio = C_PWM_MIN_DC +
                p_MulDivU16_U16byU16byU16((l_u16StartCorrectionRatio >> 1), (C_ALIGN_STEP_1_PERIOD - l_u16RampStep), C_ALIGN_STEP_1_PERIOD);
#endif
        }
        else if (l_e8AlignmentStep == E_ALIGN_STEP_2A)
        {
            /* Increasing alignment current (Sine) */
#if (C_ALIGN_STEP_2A_PERIOD == 64U) && (_SUPPORT_SINCOS_TABLE_SZ == 256)
            l_u16CorrectionRatio = C_PWM_MIN_DC + (l_u16StartCorrectionRatio >> 1) +
                mulI16hi_I16byI16(l_u16StartCorrectionRatio, c_ai16MicroStepVector3PH_SinCos256[C_COS_OFFSET + l_u16RampStep]);
#elif (C_ALIGN_STEP_2A_PERIOD == 48U) && (_SUPPORT_SINCOS_TABLE_SZ == 192)
            l_u16CorrectionRatio = C_PWM_MIN_DC + (l_u16StartCorrectionRatio >> 1) +
                mulI16hi_I16byI16(l_u16StartCorrectionRatio, c_ai16MicroStepVector3PH_SinCos192[C_COS_OFFSET + l_u16RampStep]);
#else
            l_u16CorrectionRatio = C_PWM_MIN_DC + (l_u16StartCorrectionRatio >> 1) +
                p_MulDivU16_U16byU16byU16((l_u16StartCorrectionRatio >> 1), (C_ALIGN_STEP_2A_PERIOD - l_u16RampStep), C_ALIGN_STEP_2A_PERIOD);
#endif
        }
        else if (l_e8AlignmentStep == E_ALIGN_STEP_2B)
        {
            /* Decreasing alignment current (Cosine) */
#if (C_ALIGN_STEP_2B_PERIOD == 128U) && (_SUPPORT_SINCOS_TABLE_SZ == 256)
            l_u16CorrectionRatio = C_PWM_MIN_DC + (l_u16StartCorrectionRatio - (l_u16StartCorrectionRatio >> 2)) -
                (mulI16hi_I16byI16(l_u16StartCorrectionRatio, c_ai16MicroStepVector3PH_SinCos256[C_COS_OFFSET + l_u16RampStep]) >> 1);
#elif (C_ALIGN_STEP_2B_PERIOD == 96U) && (_SUPPORT_SINCOS_TABLE_SZ == 192)
            l_u16CorrectionRatio = C_PWM_MIN_DC + (l_u16StartCorrectionRatio - (l_u16StartCorrectionRatio >> 2)) -
                (mulI16hi_I16byI16(l_u16StartCorrectionRatio, c_ai16MicroStepVector3PH_SinCos192[C_COS_OFFSET + l_u16RampStep]) >> 1);
#else
            l_u16CorrectionRatio = C_PWM_MIN_DC + (l_u16StartCorrectionRatio >> 1) +
                p_MulDivU16_U16byU16byU16((l_u16StartCorrectionRatio >> 1), l_u16RampStep, C_ALIGN_STEP_2B_PERIOD);
#endif
        }
        else
        {
           /* Nothing */
        }
        MotorDriver_3Phase(l_u16MicroStepIdx);
        l_u16PID_CtrlCounter = 0U;                                              /* Reset PID control counter during alignment (MMP230912-3) */
        u16AlignmentDone = FALSE;                                               /* Alignment not done */
    }
    return(u16AlignmentDone);
} /* End of MotorDriverAlignment() */
#endif /* (_SUPPORT_SOFT_START != FALSE) */

/*!*************************************************************************** *
 * Commutation_ISR
 * \brief   MotorDriver Commutation Interrupt
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details  Perform next motor step. If motor speed not reached, accelerate.
 *           IRQ-Priority 4
 * *************************************************************************** *
 * - Call Hierarchy: IRQ()
 * - Cyclomatic Complexity: 22+1
 * - Nesting: 7
 * - Function calling: 8 (MotorDriverStop(), MotorDriverCurrentMeasure(),
 *                        MotorDriverUpdateMicroStepIndex(), SwitchOpen2Close(),
 *                        VoltageCorrection(), MotorDriver_3Phase()
 *                        MotorStallCheckA(), PID_LA_Speed_Control(),
 *                        MotorDriverAlignment())
 * *************************************************************************** */
__attribute__((interrupt)) void ISR_CTIMER0_3(void)
{
#if (_DEBUG_CPU_LOAD != FALSE)
    DEBUG_SET_IO_B();                                                           /* IRQ-Priority 4 */
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
            break;                                                              /* Used for CPU wake-up */
        }

#if (_SUPPORT_SOFT_START != FALSE)
        if ( (l_e8MotorStartupMode == E_MSM_STEPPER_ALIGN) &&
             (MotorDriverAlignment() == FALSE) )
        {
            /* Alignment still ongoing */
            break;
        }
#endif /* (_SUPPORT_SOFT_START != FALSE) */

#if (_SUPPORT_FAST_STOP != FALSE)
        if ( (g_e8MotorStatus & C_MOTOR_STATUS_STOP_MASK) == C_MOTOR_STATUS_FAST_STOP)
        {
            static uint8_t u8Count = 0U;
#if defined (__MLX81160__) || defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
            /* UV is based on VSM (MMP220818-1) */
            uint16_t u16SupplyVoltage = ADC_Conv_Vmotor();
#else
            /* UV is based on VS */
            uint16_t u16SupplyVoltage = ADC_Conv_Vsupply();
#endif

            if ( (u16SupplyVoltage > 1000U) ||                                  /* Supply above 10.0V */
                 (u8Count <= 3U) )
            {
                /* Re-circulate Motor coil current (Active Brake) */
#if (C_MOTOR_PHASES == 3)
                DRVCFG_GND_UVW();
#else  /* (C_MOTOR_PHASES == 3) */
#if defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
                DRVCFG_GND_UVW();
#else  /* defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__) */
                DRVCFG_GND_TUVW();
#endif /* defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__) */
#endif /* (C_MOTOR_PHASES == 3) */
                u8Count++;
            }
            else
            {
                /* Charge Supply-cap */
#if (C_MOTOR_PHASES == 3)
                DRVCFG_TRI_UVW();
#else  /* (C_MOTOR_PHASES == 3) */
#if defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
                DRVCFG_TRI_UVW();
#else  /* defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__) */
                DRVCFG_TRI_TUVW();
#endif /* defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__) */
#endif /* (C_MOTOR_PHASES == 3) */
                u8Count = 0U;
            }
            break;
        }
#endif /* (_SUPPORT_FAST_STOP != FALSE) */

#if (_SUPPORT_CDI != FALSE)
        if ( (l_e8MotorStartupMode & E_MSM_CDI) != 0U)
        {
            MotorDriverCurrentMeasureCDI();

            /* CDI mode */
            if (l_e8CDI_Mode == (uint8_t)C_CDI_SEARCH)
            {
                /* No "Falling" Current crossing found */
                l_e8CDI_Mode = (uint8_t)C_CDI_FOUND;                            /* Forced */
                l_u8StallCountCDI++;
                if (l_u8StallCountCDI >= NV_STALL_CDI_WIDTH)
                {
                    g_u8StallTypeComm |= (uint8_t)C_STALL_FOUND_CDI;
                    if ( (g_e8StallDetectorEna & (uint8_t)C_STALLDET_CDI) != 0U)
                    {
                        g_u8StallOcc = TRUE;                                    /* Report stall and ...  */
                        MotorDriverStop( (uint16_t)C_STOP_EMERGENCY);           /* ... stop motor (Stall) */
                        break;
                    }
                }
            }
            if (l_e8CDI_Mode == (uint8_t)C_CDI_FOUND)
            {
                /* "Falling" Current crossing found */
                l_au16CDI_TPER[2] += IO_CTIMER0_TREGB;
                if (l_au16CDI_TPER[0] != 0U)
                {
                    /* Calculate new full-step period */
                    uint16_t u16Rotation = l_au16CDI_TPER[0] + l_au16CDI_TPER[1] + l_au16CDI_TPER[2];
#if (_SUPPORT_CDI_PID != FALSE)
                    g_u16ActualCommutTimerPeriod = p_MulU16hi_U16byU16(u16Rotation, 10923U);   /* 1/6th */
#else /* (_SUPPORT_CDI_PID != FALSE) */
                    uint16_t u16ActualCommutTimerPeriod = p_MulU16hi_U16byU16(u16Rotation, 10923U);   /* 1/6th */
#if (_SUPPORT_SPECIAL_COMM_FIELD != FALSE)
                    g_u16ActualCommutTimerPeriod =
                        p_LpfU16_I16byI16(&l_u32CommutTimerPeriodLPF, ((uint16_t)g_u8Special[1] * 32U),
                                          (int16_t)(u16ActualCommutTimerPeriod - g_u16ActualCommutTimerPeriod));                                                                                             /* MMP171026-1 */
#else  /* (_SUPPORT_SPECIAL_COMM_FIELD != FALSE) */
                    g_u16ActualCommutTimerPeriod = p_LpfU16_I16byI16(&l_u32CommutTimerPeriodLPF,
                                                                     C_CDI_COMMUT_LPF_COEF,
                                                                     (int16_t)(u16ActualCommutTimerPeriod -
                                                                               g_u16ActualCommutTimerPeriod));                                                                                   /* MMP171026-1 */
#endif /* (_SUPPORT_SPECIAL_COMM_FIELD != FALSE) */
#endif /* (_SUPPORT_CDI_PID != FALSE) */
                }
                IO_CTIMER0_TREGB = g_u16ActualCommutTimerPeriod;
                l_e8CDI_Mode = (uint8_t)C_CDI_BLIND;
                l_u8StallCountCDI = 0U;
            }
            else if (l_e8CDI_Mode == (uint8_t)C_CDI_BLIND)
            {
                /* "Raising" Current crossing */
                l_au16CDI_TPER[0] = l_au16CDI_TPER[1];
                l_au16CDI_TPER[1] = l_au16CDI_TPER[2];
                l_au16CDI_TPER[2] = IO_CTIMER0_TREGB;
                l_e8CDI_Mode = (uint8_t)C_CDI_SEARCH;
            }

            l_u16FullStepIdx = MotorDriverUpdateFullStepIndex(l_u16FullStepIdx);
            l_u16MicroStepIdx = l_u16FullStepIdx * C_MICROSTEP_PER_FULLSTEP;
            l_u16CorrectionRatio = VoltageCorrection();
            MotorDriver_3PhaseCDI(l_u16FullStepIdx);
            break;
        }
#endif /* (_SUPPORT_CDI != FALSE) */

#if (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE)
        /* Actual-position update */
        if (g_e8MotorDirectionCCW != FALSE)
        {
            l_u32ActualPosition--;                                              /* Closing */
        }
        else
        {
            l_u32ActualPosition++;                                              /* Opening */
        }

        l_u16DeltaPosition = DeltaPosition();
        if (l_u16DeltaPosition == 0U)
        {
            MotorDriverStop( (uint16_t)C_STOP_IMMEDIATE);                       /* CPOS = FPOS */
            break;
        }

        /* Near end-position? Ramp-down */
        if ( (l_u16DeltaPosition < l_u16RampDownSteps) && (l_e8MotorStartupMode != E_MSM_STEPPER_D) )
        {
            /* Decelerate motor speed (almost at target-position) */
            l_u16TargetCommutTimerPeriod = l_u16LowSpeedPeriod;
            l_u16StallDetectorDelay = l_u16DeltaPosition;
            ENTER_SECTION(ATOMIC_KEEP_MODE);  /*lint !e534 */
            {
                if ( (g_e8MotorStatus & C_MOTOR_STATUS_STOP_MASK) != C_MOTOR_STATUS_STOP)
                {
                    g_e8MotorStatus = C_MOTOR_STATUS_STOPPING;
                }
            }
            EXIT_SECTION(); /*lint !e438 */
            l_e8MotorStartupMode = E_MSM_STEPPER_D;
        }
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

        /* Motor shunt current measurement */
        MotorDriverCurrentMeasure(l_u16MicroStepIdx);
#if (_SUPPORT_STALLDET_LA != FALSE) && (_SUPPORT_CLOSED_LOOP_STARTUP == FALSE)  /* MMP230824-1 */
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
            if (((g_u16SubSamplingIdx & 0x01) == 0) && (g_u16DebugBufWrIdx < C_DEBUG_BUF_SZ))
#endif /* (_DEBUG_MCUR_CYCLIC == FALSE) */
            {
                uint8_t *pBfr = &g_au8DebugBuf[g_u16DebugBufWrIdx];
                pBfr[0] = (uint8_t)(i16TanIV >> 8);
                pBfr[1] = (uint8_t)l_u8StallCountLA; //(l_i16StallThresholdLA >> 8);
                g_u16DebugBufWrIdx += 2U;
#if (_DEBUG_MCUR_CYCLIC != FALSE)
                if (g_u16DebugBufWrIdx >= C_DEBUG_BUF_SZ)
                {
                    g_u16DebugBufWrIdx = 0U;
                }
#endif /* (_DEBUG_MCUR_CYCLIC != FALSE) */
            }
#endif /* (_DEBUG_MOTOR_CURRENT_FLT != FALSE) && (_DEBUG_IV_STALL != FALSE) */

            l_i16ActLoadAngleLPF =
                p_LpfI16_I16byI16(&l_i32ActLoadAngleLPF, C_LA_LPF_COEF_OL, (i16TanIV - l_i16ActLoadAngleLPF));  /* MMP240725-4 */
        }
#endif /* (_SUPPORT_STALLDET_LA != FALSE) && (_SUPPORT_CLOSED_LOOP_STARTUP == FALSE) */
        l_u16MicroStepIdxPrev = l_u16MicroStepIdx;

        l_u16MicroStepIdx = MotorDriverUpdateMicroStepIndex(l_u16MicroStepIdx);

        if ( (l_e8MotorStartupMode & E_MSM_MODE_MASK) == E_MSM_STEPPER)
        {
            /* Start-up: Open-loop feed-forward stepper-mode */
            g_u16ActualCommutTimerPeriod = l_u16CommutTimerPeriod;
            l_u32CommutTimerPeriodLPF = ((uint32_t)l_u16CommutTimerPeriod << 16);

#if (_SUPPORT_CLOSED_LOOP_STARTUP != FALSE)                                     /* MMP190508-2 */
#if (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
            if (g_e8MotorRequest != (uint8_t)C_MOTOR_REQUEST_CALIB_FACTORY)
#endif /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */
            {
                SwitchOpen2Close(E_MSM_FOC_CL_STARTUP);
            }
#endif /*_SUPPORT_CLOSED_LOOP_STARTUP */
#if (_SUPPORT_TRIAXIS_MLX9038x != FALSE) || (_SUPPORT_CLOSED_LOOP_STARTUP == FALSE)
#if (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
            if (g_e8MotorRequest == (uint8_t)C_MOTOR_REQUEST_CALIB_FACTORY)
            {
                Triaxis_Calibrate(C_TRIAXIS_CALIB_STORE);
            }
#endif /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */
        }

        if ( (l_e8MotorStartupMode & E_MSM_MODE_MASK) == E_MSM_STEPPER)
        {
            if (l_u16CommutTimerPeriod == l_u16TargetCommutTimerPeriod)
            {
                l_e8MotorStartupMode = E_MSM_STEPPER_C;
#if ((_SUPPORT_FOC_MODE & FOC_OBSERVER_MASK) == FOC_OBSERVER_SENSORLESS)
#if defined (C_SPEED_OL2CL)
                if ((g_u16ActualMotorSpeedRPM > C_SPEED_OL2CL) && (g_u16TargetMotorSpeedRPM > C_SPEED_OL2CL))
#elif (_SUPPORT_AUTO_SPEED_FOC == FALSE)
                if (g_u16ActualMotorSpeedRPM > g_u16LowSpeedRPM)
#else  /* (_SUPPORT_AUTO_SPEED_FOC == FALSE) */
                if ((g_u16ActualMotorSpeedRPM > g_u16MaxSpeedRPM) && (g_u16TargetMotorSpeedRPM > g_u16MaxSpeedRPM))
#endif /* (_SUPPORT_AUTO_SPEED_FOC == FALSE) */
                {
                    SwitchOpen2Close(E_MSM_FOC_2PWM);
                }
#endif /* ((_SUPPORT_FOC_MODE & FOC_OBSERVER_MASK) == FOC_OBSERVER_SENSORLESS) */
            }
            else if (l_u16CommutTimerPeriod > l_u16TargetCommutTimerPeriod)
            {
                /* Acceleration */
                if ( (l_u16MicroStepIdx & l_u16SpeedUpdateAcc) == 0U)
                {
                    /* Update speed: Acceleration per electric rotation */
#if ((_SUPPORT_FOC_MODE & FOC_OBSERVER_MASK) == FOC_OBSERVER_SENSORLESS)
#if defined (C_SPEED_OL2CL)
                    if ((g_u16ActualMotorSpeedRPM > C_SPEED_OL2CL) && (g_u16TargetMotorSpeedRPM > C_SPEED_OL2CL))
#elif (_SUPPORT_AUTO_SPEED_FOC == FALSE)
                    if (g_u16ActualMotorSpeedRPM > g_u16LowSpeedRPM)
#else  /* (_SUPPORT_AUTO_SPEED_FOC == FALSE) */
                    if ((g_u16ActualMotorSpeedRPM > g_u16MaxSpeedRPM) && (g_u16TargetMotorSpeedRPM > g_u16MaxSpeedRPM))
#endif /* (_SUPPORT_AUTO_SPEED_FOC == FALSE) */
                    {
                        SwitchOpen2Close(E_MSM_FOC_2PWM);
                    }
                    else
#endif /* ((_SUPPORT_FOC_MODE & FOC_OBSERVER_MASK) == FOC_OBSERVER_SENSORLESS) */
                    {
#if (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE)
                        uint16_t u16Compensation = g_u16ForcedSpeedRPM;
#endif /* (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) */
                        l_e8MotorStartupMode = E_MSM_STEPPER_A;                 /* Too slow, accelerate */
                        g_u16ForcedSpeedRPM = g_u16ForcedSpeedRPM +
                                              p_DivU16_U32byU16( (uint32_t)l_u16AccelerationConst,
                                                                 g_u16ForcedSpeedRPM);
                        if (g_u16ForcedSpeedRPM > g_u16TargetMotorSpeedRPM)
                        {
                            g_u16ForcedSpeedRPM = g_u16TargetMotorSpeedRPM;
                        }
                        l_u16CommutTimerPeriod = p_DivU16_U32byU16(l_u32MicroStepPeriodOneRPM, g_u16ForcedSpeedRPM) - 1U;
                        IO_CTIMER0_TREGB = l_u16CommutTimerPeriod;
#if (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE)
                        l_u16RampDownSteps += (l_u16SpeedUpdateDec + 1U);
                        PID_SpeedCompensate(g_u16ForcedSpeedRPM, u16Compensation);
#endif /* (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) */
                    }
                }
            }
            else
            {
                /* Deceleration */
                if ( (l_u16MicroStepIdx & l_u16SpeedUpdateDec) == 0U)
//                if ( ((g_e8MotorStatus & C_MOTOR_STATUS_STOP_MASK) == C_MOTOR_STATUS_STOPPING) ||
//                     ((l_u16MicroStepIdx & l_u16SpeedUpdateDec) == 0U) )
                {
                    /* Update speed: Deceleration per electric rotation */
#if (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE)
                    uint16_t u16Compensation = g_u16ForcedSpeedRPM;
#endif /* (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) */
                    uint16_t u16SpeedDecrease = p_DivU16_U32byU16( (uint32_t)l_u16AccelerationConst, g_u16ForcedSpeedRPM);
                    l_e8MotorStartupMode = E_MSM_STEPPER_D;                     /* Too fast, decelerate */
                    if (u16SpeedDecrease < g_u16ForcedSpeedRPM)
                    {
                        g_u16ForcedSpeedRPM -= u16SpeedDecrease;
                        if (g_u16ForcedSpeedRPM < g_u16MinSpeedRPM)
                        {
                            g_u16ForcedSpeedRPM = g_u16MinSpeedRPM;
                        }
                    }
                    l_u16CommutTimerPeriod = p_DivU16_U32byU16(l_u32MicroStepPeriodOneRPM, g_u16ForcedSpeedRPM) - 1U;
                    if (l_u16CommutTimerPeriod > l_u16TargetCommutTimerPeriod)
                    {
                        l_u16CommutTimerPeriod = l_u16TargetCommutTimerPeriod;
                    }
                    IO_CTIMER0_TREGB = l_u16CommutTimerPeriod;
#if (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE)
                    l_u16RampDownSteps -= (l_u16SpeedUpdateDec + 1U);
                    PID_SpeedCompensate(g_u16ForcedSpeedRPM, u16Compensation);
#endif /* (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) */
                }
            }
#endif /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) || (_SUPPORT_CLOSED_LOOP_STARTUP == FALSE) */
        }

#if (_SUPPORT_VARIABLE_PWM != FALSE) && (_SUPPORT_VAR_PWM_MODE == C_VAR_PWM_MODE_ON_DUTY_CYCLE)  /* MMP220815-1 */
        if (l_u16MicroStepIdx == (2U * g_u16NrOfMicroStepsPerFullStep))
        {
            /* Switch ADC; (Current) samples already taken
             * NOTE: Voltage (VSMF) has not been taken! (TODO) */
            if ( (l_u16CorrectionRatio > C_PWM_DC_SWITCH2PWM) &&                /* Switching based on PWM DC */
                 (g_u8TriplePWM != FALSE) )
            {
                /* Switch to "fast"-mode; switch ADC mode too (2 phase-currents) */
                HAL_ADC_StopSafe();
                l_e8MotorStartupMode = E_MSM_FOC_2PWM;
                MotorDriverReconfigure(FALSE);
                ADC_Start(FALSE);                                               /* Stepper-mode doesn't need ADC IRQ (CTimer based) */
            }
            else if ( (l_u16CorrectionRatio < C_PWM_DC_SWITCH3PWM) &&           /* Switching based on PWM DC */
                      (g_u8TriplePWM == FALSE) )
            {
                /* Switch to "slow"-mode; switch ADC mode too (3 phase-currents) */
                HAL_ADC_StopSafe();
                l_e8MotorStartupMode = E_MSM_FOC_3PWM;
                MotorDriverReconfigure(TRUE);
                ADC_Start(FALSE);                                               /* Stepper-mode doesn't need ADC IRQ (CTimer based) */
            }
        }
#endif /* (_SUPPORT_VARIABLE_PWM != FALSE) && (_SUPPORT_VAR_PWM_MODE == C_VAR_PWM_MODE_ON_DUTY_CYCLE) */

        g_u16ActualCommutTimerPeriod =
            p_LpfU16_I16byI16(&l_u32CommutTimerPeriodLPF, 1024,
                              (int16_t)(l_u16CommutTimerPeriod - g_u16ActualCommutTimerPeriod));                                                                /* MMP171026-1 */

        /* Micro Stepping-mode */
        l_u16CorrectionRatio = VoltageCorrection();
#if (C_MOTOR_PHASES == 3)
        MotorDriver_3Phase(l_u16MicroStepIdx);
#else  /* (C_MOTOR_PHASES == 3) */
        MotorDriver_4Phase(l_u16MicroStepIdx);
#endif /* (C_MOTOR_PHASES == 3) */

#if (_SUPPORT_STALLDET_LA == FALSE)
#if ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_ID_IQ)
        l_i16ActLoadAngle = MotorDriverFOC_IV();
#else  /* ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_ID_IQ) */
        l_i16ActLoadAngle = MotorDriverFOC_IB((int16_t)l_u16MotorVoltageAngle);
        l_u16PrevMotorVoltageAngle = l_u16MotorVoltageAngle;
#endif /* ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_ID_IQ) */
#endif /* (_SUPPORT_STALLDET_LA == FALSE) */

        /* Stall "A" detection */
        if (MotorStallCheckA() != C_STALL_NOT_FOUND)
        {
            /* Max current/torque stall detected */
            g_u8StallTypeComm |= (uint8_t)C_STALL_FOUND_A;
            if ( (g_e8StallDetectorEna & ((uint8_t)C_STALLDET_A | (uint8_t)C_STALLDET_CALIB)) != 0U)
            {
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
#if (_SUPPORT_STALLDET_LA != FALSE) && (_SUPPORT_CLOSED_LOOP_STARTUP == FALSE)
        else if (MotorStallCheckLA() != C_STALL_NOT_FOUND)
        {
            /* Invalid Load-angle stall detected */
            g_u8StallTypeComm |= (uint8_t)C_STALL_FOUND_LA;                     /* StallCheckIV */
            if ( (g_e8StallDetectorEna & (C_STALLDET_LA | C_STALLDET_CALIB)) != 0U)
            {
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
#endif /* (_SUPPORT_STALLDET_LA != FALSE) && (_SUPPORT_CLOSED_LOOP_STARTUP == FALSE) */
    } while (FALSE);

#if (_DEBUG_COMMUT_ISR != FALSE)
    DEBUG_CLR_IO_B();
#endif /* (_DEBUG_COMMUT_ISR != FALSE) */
#if (_DEBUG_CPU_LOAD != FALSE)
    DEBUG_CLR_IO_B();                                                           /* IRQ-Priority 4 */
#endif /* (_DEBUG_CPU_LOAD != FALSE) */
} /* End of ISR_CTIMER0_3() */

/*!*************************************************************************** *
 * ADC_ISR
 * \brief   ADC End-of-frame Interrupt
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details  Perform next motor step. If motor speed not reached, accelerate.
 *           IRQ Priority: 4
 * *************************************************************************** *
 * - Call Hierarchy: IRQ()
 * - Cyclomatic Complexity: 2+1
 * - Nesting: 2
 * - Function calling: 5 (MotorDriverCurrentMeasure(), MotorDriverFOC(), PID_MicroStep(),
 *                       VoltageCorrection(), MotorDriver_3Phase())
 * *************************************************************************** */
#if (_SUPPORT_RAM_FUNC != FALSE)
__attribute__((interrupt)) void ADC_ISR(void) __attribute__ ((section(".ramfunc"))) __attribute__ ((aligned(8)));
#elif (_SUPPORT_OPTIMIZE_FOR_SPEED != FALSE)
__attribute__((interrupt)) void ADC_ISR(void) __attribute__((aligned(8)));
#endif /* (_SUPPORT_OPTIMIZE_FOR_SPEED != FALSE) */
__attribute__((interrupt)) void ADC_ISR(void)
{
#if (FOC_FREQ != PWM_FREQ)
    /* Reduced FOC Update frequency; Not every PWM period */
    static uint16_t u16AdcIsrSkip = 0U;

    if (u16AdcIsrSkip == 0U)
    {
        /* Update FOC now */
        u16AdcIsrSkip = (PWM_FREQ / FOC_FREQ) - 1U;
    }
    else
    {
        /* Skip FOC update */
        u16AdcIsrSkip--;
        return;
    }
#endif /* (FOC_FREQ != PWM_FREQ) */

#if (_DEBUG_CPU_LOAD != FALSE)
    DEBUG_SET_IO_B();                                                           /* IRQ-Priority 4 */
#endif /* (_DEBUG_CPU_LOAD != FALSE) */
#if (_DEBUG_COMMUT_ISR != FALSE)
    DEBUG_SET_IO_B();
#endif /* (_DEBUG_COMMUT_ISR != FALSE) */

#if (_DEBUG_MCUR_SUB_SAMPLED != FALSE)
    ++g_u16SubSamplingIdx;                                                      /* MMP240725-1 */
#endif /* (_DEBUG_MCUR_SUB_SAMPLED != FALSE) */

#if (_SUPPORT_MOTION_DET != C_MOTION_DET_NONE)
    if (l_u8AdcMode == (uint8_t)C_ADC_MODE_MOV_DET)
    {
        /* Movement Detector */
#if (_SUPPORT_MOTION_DET == C_MOTION_DET_BEMF)
        MotionDetectorBEMF();
#elif (_SUPPORT_MOTION_DET == C_MOTION_DET_SENSOR)
        MotionDetectorSensor();
#endif /* (_SUPPORT_MOTION_DET == C_MOTION_DET_SENSOR) */
        return;
    }
#endif /* (_SUPPORT_MOTION_DET != C_MOTION_DET_NONE) */

    MotorDriverCurrentMeasure(l_u16MicroStepIdxPrev);
    l_u16MicroStepIdxPrev = l_u16MicroStepIdx;

#if (_SUPPORT_VARIABLE_PWM != FALSE) && (_SUPPORT_VAR_PWM_MODE == C_VAR_PWM_MODE_ON_DUTY_CYCLE)  /* MMP220815-1 */
    if (l_u16MicroStepIdx == (2U * g_u16NrOfMicroStepsPerFullStep))
    {
        /* Switch ADC; (Current) samples already taken
         * NOTE: Voltage (VSMF) has not been taken! (TODO) */
        if ( (l_u16CorrectionRatio > C_PWM_DC_SWITCH2PWM) &&                    /* Switching based on PWM DC */
             (g_u8TriplePWM != FALSE) )
        {
            /* Switch to "fast"-mode; switch ADC mode too (2 phase-currents) */
            HAL_ADC_StopSafe();
            l_e8MotorStartupMode = E_MSM_FOC_2PWM;
            MotorDriverReconfigure(FALSE);
            ADC_Start(TRUE);                                                    /* FOC based need ADC IRQ */
        }
        else if ( (l_u16CorrectionRatio < C_PWM_DC_SWITCH3PWM) &&               /* Switching based on PWM DC */
                  (g_u8TriplePWM == FALSE) )
        {
            /* Switch to "slow"-mode; switch ADC mode too (3 phase-currents) */
            HAL_ADC_StopSafe();
            l_e8MotorStartupMode = E_MSM_FOC_3PWM;
            MotorDriverReconfigure(TRUE);
            ADC_Start(TRUE);                                                    /* FOC based need ADC IRQ */
        }
    }
#endif /* (_SUPPORT_VARIABLE_PWM != FALSE) && (_SUPPORT_VAR_PWM_MODE == C_VAR_PWM_MODE_ON_DUTY_CYCLE) */

#if ((_SUPPORT_FOC_MODE & FOC_OBSERVER_MASK) == FOC_OBSERVER_SENSORLESS)
#if (_SUPPORT_CLOSED_LOOP_STARTUP != FALSE)
    if (l_e8MotorStartupMode != E_MSM_FOC_CL_STARTUP)                           /* MMP210527-1: Swap order IV (Close-loop start-up) and IB */
#endif /* (_SUPPORT_CLOSED_LOOP_STARTUP != FALSE) */
    {
        /* Closed-Loop motor operation (IB or Id/Iq based) */
#if ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_ID_IQ)
        l_u16VoltageAngle = MotorDriverFOC_IDIQ();
#else  /* ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_ID_IQ) */
        l_i16ActLoadAngle = MotorDriverFOC_IB((int16_t)l_u16PrevMotorVoltageAngle);
        l_u16PrevMotorVoltageAngle = l_u16MotorVoltageAngle;
#if (_SUPPORT_PID_U32 == FALSE)
        g_u16ActualMotorSpeedRPMe = PID_LoadAngleToActSpeed(l_i16ActLoadAngle);
#else  /* (_SUPPORT_PID_U32 == FALSE) */
        g_u32ActualMotorSpeedRPMe = PID_LoadAngleToActSpeed(l_i16ActLoadAngle);
#endif /* (_SUPPORT_PID_U32 == FALSE) */
#if ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_IB_IPK)
        PID_Ipk_Control(l_u16Ipk);
#endif /* ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_IB_IPK) */
#endif /* ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_ID_IQ) */
    }
#if (_SUPPORT_CLOSED_LOOP_STARTUP != FALSE)
    else
    {
        /* Closed-Loop START-UP motor operation (IV-based) */
        l_i16ActLoadAngle = MotorDriverFOC_IV();
#if (_SUPPORT_PID_U32 == FALSE)
        g_u16ActualMotorSpeedRPMe = PID_LoadAngleToActSpeed_IV(l_i16ActLoadAngle);
        if (g_u16ActualMotorSpeedRPMe > l_u16StartrupThrshldSpeedRPMe)
#else  /* (_SUPPORT_PID_U32 == FALSE) */
        g_u32ActualMotorSpeedRPMe = PID_LoadAngleToActSpeed_IV(l_i16ActLoadAngle);
        if (g_u32ActualMotorSpeedRPMe > (uint32_t)l_u16StartrupThrshldSpeedRPMe)
#endif /* (_SUPPORT_PID_U32 == FALSE) */
        {
            /* Switch from Closed-Loop Start-up (FOC-IV) to FOC-IB or Id/Iq */
#if ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_ID_IQ)
            l_u32MotorSpeedRadPSe = divU32_U32byU16( (((uint32_t)l_u16StartrupThrshldSpeedRPMe) << 16), 60U);
            PID_IDIQ_Start(l_u16Ipk);
#elif ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_IB_IPK)
            PID_Ipk_Start(l_u16Ipk);
#endif /* (_SUPPORT_FOC_MODE & FOC_MODE_MASK) */
            l_e8MotorStartupMode = E_MSM_FOC_2PWM;
        }
#if ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_ID_IQ)
        else
        {
#if (_SUPPORT_PID_U32 == FALSE)
            uint16_t u16MotorVoltageAngleUpdate = p_MulU16hi_U16byU16(g_u16ActualMotorSpeedRPMe, C_SPEED_RPMe_TO_ANGLE);
#else  /* (_SUPPORT_PID_U32 == FALSE) */
#if (FOC_FREQ > 2200)
            uint16_t u16MotorVoltageAngleUpdate = p_MulU16hi_U16byU16( (uint16_t)(g_u32ActualMotorSpeedRPMe / C_FOC_PID_MULT),
                                                                       (C_SPEED_RPMe_TO_ANGLE * C_FOC_PID_MULT));
#else
#error "ERROR: Overflow!!"
#endif
#endif /* (_SUPPORT_PID_U32 == FALSE) */
#if (_SUPPORT_MOTOR_DIRECTION == C_MOTOR_DIRECTION_CCW)
            u16MotorVoltageAngleUpdate = (uint16_t)(0U - u16MotorVoltageAngleUpdate);
#elif (_SUPPORT_MOTOR_DIRECTION == C_MOTOR_DIRECTION_CMD)
            if (g_e8MotorDirectionCCW != FALSE)
            {
                /* Closing */
                u16MotorVoltageAngleUpdate = (uint16_t)(0U - u16MotorVoltageAngleUpdate);
            }
            else
            {
                /* Opening */
            }
#endif /* (_SUPPORT_MOTOR_DIRECTION == C_MOTOR_DIRECTION_CMD) */
            l_u16MotorVoltageAngle = l_u16MotorVoltageAngle + u16MotorVoltageAngleUpdate;
            l_u16VoltageAngle = l_u16MotorVoltageAngle;
        }
        l_u16VoltageAngle = l_i16ActLoadAngle;
#endif /* ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_ID_IQ) */
#if (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE)
        l_u16RampDownSteps = RampUp();
#endif /* (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) */
    }
#endif /* (_SUPPORT_CLOSED_LOOP_STARTUP != FALSE) */
#else  /* ((_SUPPORT_FOC_MODE & FOC_OBSERVER_MASK) == FOC_OBSERVER_SENSORLESS) */
    /* Sensor based rotor angle observer */
    l_u16ResolverAngle = Triaxis_Angle();
#if ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_IV)
    {
        /* Resolver Sense Angle change [RAD] */
        int16_t i16RotorAngleUpdate;
        if (g_e8MotorDirectionCCW != FALSE)
        {
            /* Counter Clock Wise */
            i16RotorAngleUpdate = (int16_t)(l_u16PrevResolverAngle - l_u16ResolverAngle);
        }
        else
        {
            /* Clock Wise */
            i16RotorAngleUpdate = (int16_t)l_u16ResolverAngle - l_u16PrevResolverAngle;
        }
        l_u16PrevResolverAngle = l_u16ResolverAngle;
        /* Actual Sense-Magnet Speed [RAD-e] */
        l_u16RotorAngleSpeed =
            p_LpfU16_I16byI16(&l_u32RotorAngleSpeedLPF, 1024, (int16_t)(i16RotorAngleUpdate - l_u16RotorAngleSpeed));
    }
    /* Actual Motor Speed [RPM] */
    if (NV_SENSE_POLE_PAIRS == 1U)
    {
        g_u16ActualMotorSpeedRPM = p_DivU16_U32byU16(l_u32RotorAngleSpeedLPF, C_SPEED_RPMe_TO_ANGLE);
    }
    else
    {
        g_u16ActualMotorSpeedRPM =
            p_DivU16_U32byU16(l_u32RotorAngleSpeedLPF,
                              (uint16_t)p_MulU32_U16byU16(NV_SENSE_POLE_PAIRS, C_SPEED_RPMe_TO_ANGLE));
    }

    /* Vector Angle [table-index] */
    l_u16MicroStepIdx = MotorDriverResolverAngleToMicroStepIndex(l_u16ResolverAngle);   /* Closed-loop, Feed-back */
#elif ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_IB) || ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_IB_IPK)
    {
        /* Sensor/IB Observer */
        /* BEMF Angle */
        uint16_t u16RotorAngle = (uint16_t)p_MulU32_U16byU16( (l_u16ResolverAngle - g_i16ResolverAngleOffset),
                                                              g_u16MotorPolePairs);
        /* Current Angle */
#if ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_IB_IPK)
        PID_Ipk_Control(l_u16Ipk);
#endif /* ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_IB_IPK) */

        /* Load Angle */
        if (g_e8MotorDirectionCCW != FALSE)
        {
            uint16_t u16BemfAngle = u16RotorAngle - C_ANGLE_90DEG;              /* BEMF is at 90 degrees */
            l_i16ActLoadAngle = g_i16MotorCurrentAngle - u16BemfAngle;
        }
        else
        {
            uint16_t u16BemfAngle = u16RotorAngle + C_ANGLE_90DEG;              /* BEMF is at 90 degrees */
            l_i16ActLoadAngle = u16BemfAngle - g_i16MotorCurrentAngle;
        }
#if (_SUPPORT_STALLDET_LA != FALSE)
        l_i16ActLoadAngleLPF =
            p_LpfI16_I16byI16(&l_i32ActLoadAngleLPF, C_LA_LPF_COEF_CL, (l_i16ActLoadAngle - l_i16ActLoadAngleLPF));
#endif /* (_SUPPORT_STALLDET_LA != FALSE) */
#if TRUE /* TODO: Remove it! Just blink stall "O" in GUI when -16384 > LoadAngle > 16384 */
        if ((l_i16ActLoadAngle > 16384) || (l_i16ActLoadAngle < -16384))
        {
            if ( (g_u16ActualMotorSpeedRPM > (g_u16TargetMotorSpeedRPM >> 1)) )  /* Actual-speed above 50% of Target-speed */
            {
                /* Note: This stall-detector also triggers at high-speed with zero-load.
                 * Therefore it can not be used as real stall-detection */
                g_u8StallTypeComm |= (uint8_t)C_STALL_FOUND_LA;
            }
        }
#endif
        /* Actual Motor Speed [RPMe] */
        g_u16ActualMotorSpeedRPMe = PID_LoadAngleToActSpeed(l_i16ActLoadAngle);
        /* Rotor Angle update, based on actual-speed */
        uint16_t u16MotorVoltageAngleUpdate = p_MulU16hi_U16byU16(g_u16ActualMotorSpeedRPMe, C_SPEED_RPMe_TO_ANGLE);
#if (_SUPPORT_MOTOR_DIRECTION == C_MOTOR_DIRECTION_CCW)
        u16MotorVoltageAngleUpdate = (uint16_t)(0U - u16MotorVoltageAngleUpdate);
#elif (_SUPPORT_MOTOR_DIRECTION == C_MOTOR_DIRECTION_CMD)
        if (g_e8MotorDirectionCCW != FALSE)
        {
            /* Closing */
            u16MotorVoltageAngleUpdate = (uint16_t)(0U - u16MotorVoltageAngleUpdate);
        }
        else
        {
            /* Opening */
        }
#endif /* (_SUPPORT_MOTOR_DIRECTION == C_MOTOR_DIRECTION_CMD) */
        l_u16MotorVoltageAngle = l_u16MotorVoltageAngle + u16MotorVoltageAngleUpdate;
        /* Vector Angle [table-index] */
        l_u16MicroStepIdx = (uint16_t)p_MulU16hi_U16byU16(l_u16MotorVoltageAngle, l_u16MotorMicroStepsPerElecRotation);
    }
#elif ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_ID_IQ)
    {
        /* Resolver Sense Angle change [RAD] */
        int16_t i16RotorAngleUpdate;
        if (g_e8MotorDirectionCCW != FALSE)
        {
            /* Counter Clock Wise */
            i16RotorAngleUpdate = (int16_t)(l_u16PrevResolverAngle - l_u16ResolverAngle);
        }
        else
        {
            /* Clock Wise */
            i16RotorAngleUpdate = (int16_t)l_u16ResolverAngle - l_u16PrevResolverAngle;
        }
        l_u16PrevResolverAngle = l_u16ResolverAngle;
        /* Actual Sense-Magnet Speed [RAD-e] */
        l_u16RotorAngleSpeed =
            p_LpfU16_I16byI16(&l_u32RotorAngleSpeedLPF, 1024, (int16_t)(i16RotorAngleUpdate - l_u16RotorAngleSpeed));
    }
    /* Actual Motor Speed [RPM] */
    if (NV_SENSE_POLE_PAIRS == 1U)
    {
        g_u16ActualMotorSpeedRPM = p_DivU16_U32byU16(l_u32RotorAngleSpeedLPF, C_SPEED_RPMe_TO_ANGLE);
    }
    else
    {
        g_u16ActualMotorSpeedRPM =
            p_DivU16_U32byU16(l_u32RotorAngleSpeedLPF,
                              (uint16_t)p_MulU32_U16byU16(NV_SENSE_POLE_PAIRS, C_SPEED_RPMe_TO_ANGLE));
    }

    {
        /* Sensor/Id+Iq Observer */
        /* BEMF Angle */
        l_u16MotorVoltageAngle = (uint16_t)p_MulU32_U16byU16( (l_u16ResolverAngle - g_i16ResolverAngleOffset),
                                                       g_u16MotorPolePairs);
        /* Park Transformation of the current */
#if (_SUPPORT_SINCOS_TABLE_SZ == 256)
        int16_t *pi16SinRotorAngle = (int16_t *)&c_ai16MicroStepVector3PH_SinCos256[l_u16MotorVoltageAngle / _SUPPORT_SINCOS_TABLE_SZ];
#elif (_SUPPORT_SINCOS_TABLE_SZ == 192)
        int16_t *pi16SinRotorAngle =
            (int16_t *)&c_ai16MicroStepVector3PH_SinCos192[p_MulU16hi_U16byU16(l_u16MotorVoltageAngle, _SUPPORT_SINCOS_TABLE_SZ)];
#elif (_SUPPORT_SINCOS_TABLE_SZ == 1024)
        int16_t *pi16SinRotorAngle = p_SinDirectLutInlined_2_5k(l_u16MotorVoltageAngle);
#else
#error "ERROR: Unsupported table"
#endif
        /* Id = Ialpha * cos(theta) + Ibeta * sin(theta)
         * Iq = Ibeta * cos(theta) - Ialpha * sin(theta)
         * (theta = rotor-angle)
         */
        int16_t i16Ialpha = g_i16MotorCurrentCoilA;
        int16_t i16Ibeta = g_i16MotorCurrentCoilY;
        int16_t i16Id = p_MulI16_I16bypQ15(i16Ibeta, pi16SinRotorAngle) +
                        p_MulI16_I16bypQ15(i16Ialpha, (pi16SinRotorAngle + C_COS_OFFSET));
        int16_t i16Iq = p_MulI16_I16bypQ15(i16Ibeta, (pi16SinRotorAngle + C_COS_OFFSET)) -
                        p_MulI16_I16bypQ15(i16Ialpha, pi16SinRotorAngle);
        PID_FOC_IDIQ(i16Id, i16Iq, pi16SinRotorAngle);
        /* Convert Voltage-angle [0..65535] to vector-index [0..uStep-per-rotation] */
        l_u16MicroStepIdx = (uint16_t)p_MulU16hi_U16byU16(l_u16VoltageAngle, l_u16MotorMicroStepsPerElecRotation);
    }
#endif  /* (_SUPPORT_FOC_MODE & FOC_MODE_MASK) */
#endif /* ((_SUPPORT_FOC_MODE & FOC_OBSERVER_MASK) == FOC_OBSERVER_SENSORLESS) */

#if (_DEBUG_FOC_PERF != FALSE)
    DEBUG_TOG_IO_C();
#endif /* (_DEBUG_FOC_PERF != FALSE) */
    do
    {
#if ((_SUPPORT_FOC_MODE & FOC_OBSERVER_MASK) == FOC_OBSERVER_SENSORLESS)
#if ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) != FOC_MODE_ID_IQ)
        /* radial / PWM-period */
        /* PWM-Frequency: 20kHz, PWM-period = 50us
         * Speed [RPM-mechanical] * PP = Speed [RPM-electric]
         * Micro-step-index: 8[16] uSteps/Full-Step --> 6 x 8[16] = 1 electric-rotation = 48[96] uSteps
         * Micro-step update = ((Speed[RPM] / 60 [min/sec]) * PP) * 48[96] [uSteps/e-rot] / PWM-FREQ
         *                   = (Speed[RPM] * PP * 48[96]/60) / PWM-FREQ (48[96]/60 = 4[8]/5)
         * l_u16MotorMicroStepsPerMechRotation = PP * 48[96]
         * RotorAngle = MicroStep * 65536 / l_u16MotorMicroStepsPerElecRotation
         *
         * RotorAngle update = (Speed[RPM] * PP * 65536) / (60 * PWM-FREQ), with Speed[RPM] * PP = Speed[RPMe]
         */
#if (_SUPPORT_PID_U32 == FALSE)
        uint16_t u16MotorVoltageAngleUpdate = p_MulU16hi_U16byU16(g_u16ActualMotorSpeedRPMe, C_SPEED_RPMe_TO_ANGLE);
#else  /* (_SUPPORT_PID_U32 == FALSE) */
#if (FOC_FREQ > 2200)
        uint16_t u16MotorVoltageAngleUpdate = p_MulU16hi_U16byU16( (uint16_t)(g_u32ActualMotorSpeedRPMe / C_FOC_PID_MULT),
                                                                   (uint16_t)(C_FOC_PID_MULT * C_SPEED_RPMe_TO_ANGLE));
#else
#error "ERROR: Overflow!!"
#endif
#endif /* (_SUPPORT_PID_U32 == FALSE) */
#if (_SUPPORT_MOTOR_DIRECTION == C_MOTOR_DIRECTION_CW)
        l_u16MotorVoltageAngle = l_u16MotorVoltageAngle + u16MotorVoltageAngleUpdate;
#elif (_SUPPORT_MOTOR_DIRECTION == C_MOTOR_DIRECTION_CCW)
        l_u16MotorVoltageAngle = l_u16MotorVoltageAngle - u16MotorVoltageAngleUpdate;
#elif (_SUPPORT_MOTOR_DIRECTION == C_MOTOR_DIRECTION_CMD)
#if (_SUPPORT_BRAKING != FALSE)
        if (g_e8MotorStatus != C_MOTOR_STATUS_BRAKING)
#endif /* (_SUPPORT_BRAKING != FALSE) */
        {
            if (g_e8MotorDirectionCCW != FALSE)
            {
                /* Closing (Counter Clock Wise) */
                u16MotorVoltageAngleUpdate = (uint16_t)(0U - u16MotorVoltageAngleUpdate);
            }
            else
            {
                /* Opening (Clock Wise) */
            }
        }
#if (_SUPPORT_BRAKING != FALSE)
        else
        {
#if (_DEBUG_BRAKING != FALSE)
            DEBUG_TOG_IO_C();
#endif /* (_DEBUG_BRAKING != FALSE) */
            /* Braking; main-loop checks if actual speed is getting below minimum speed */
            if (g_e8MotorDirectionCCW == FALSE)
            {
                /* Opening (Clock Wise) */
                u16MotorVoltageAngleUpdate = (uint16_t)(0U - u16MotorVoltageAngleUpdate);
            }
            else
            {
                /* Closing (Counter Clock Wise) */
            }
        }
#endif /* (_SUPPORT_BRAKING != FALSE) */
        l_u16MotorVoltageAngle = l_u16MotorVoltageAngle + u16MotorVoltageAngleUpdate;
#endif /* (_SUPPORT_MOTOR_DIRECTION == C_MOTOR_DIRECTION_CMD) */
#if (_SUPPORT_PWM_POL_CORR == FALSE)
        l_u16PwmStatePrev = l_u16PwmState;
        {
            uint16_t u16MicroStepIdx = (uint16_t)p_MulU16hi_U16byU16(l_u16MotorVoltageAngle,
                                                                     l_u16MotorMicroStepsPerElecRotation);
            if (g_e8MotorDirectionCCW != FALSE)
            {
                /* Counter Clock-wise */
                if ( (u16MicroStepIdx < l_u16PwmStateStepIdx) && (l_u16MicroStepIdx >= l_u16PwmStateStepIdx) )
                {
                    l_u16PwmState = l_u16PwmState ^ TRUE;
                }
            }
            else
            {
                /* Clock-wise */
                if ( (u16MicroStepIdx >= l_u16PwmStateStepIdx) && (l_u16MicroStepIdx < l_u16PwmStateStepIdx) )
                {
                    l_u16PwmState = l_u16PwmState ^ TRUE;
                }
            }
            l_u16MicroStepIdx = u16MicroStepIdx;
        }
#else  /* (_SUPPORT_PWM_POL_CORR == FALSE) */
#if (_SUPPORT_PWM_MODE == BIPOLAR_TRIPHASE_ALLPWM_MIRROR) || \
        (_SUPPORT_PWM_MODE == BIPOLAR_TRIPHASE_TWOPWM_INDEPENDENT_GND)
        if (g_e8MotorDirectionCCW != FALSE)
        {
            l_u16MicroStepIdx = (uint16_t)p_MulU16hi_U16byU16( (l_u16MotorVoltageAngle + 4096U),
                                                               l_u16MotorMicroStepsPerElecRotation);                              /* Shift of +22.5 degrees (45-22.5); Table-shift: 45 degrees */
        }
        else
        {
            l_u16MicroStepIdx = (uint16_t)p_MulU16hi_U16byU16( (l_u16MotorVoltageAngle + 12288U),
                                                               l_u16MotorMicroStepsPerElecRotation);                               /* Shift of +67.5 degrees (45+22.5); Table-shift: 45 degrees */
        }
#else
        l_u16MicroStepIdx = (uint16_t)p_MulU16hi_U16byU16(l_u16MotorVoltageAngle, l_u16MotorMicroStepsPerElecRotation);
#endif /* (_SUPPORT_PWM_MODE == BIPOLAR_TRIPHASE_ALLPWM_MIRROR) */
#endif /* (_SUPPORT_PWM_POL_CORR == FALSE) */
#else  /* ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) != FOC_MODE_ID_IQ) */
        l_u16MicroStepIdx = (uint16_t)p_MulU16hi_U16byU16(l_u16VoltageAngle, l_u16MotorMicroStepsPerElecRotation);
#endif /* ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) != FOC_MODE_ID_IQ) */

#if (_DEBUG_FOC_PERF != FALSE)
        DEBUG_TOG_IO_C();
#endif /* (_DEBUG_FOC_PERF != FALSE) */
#endif /* ((_SUPPORT_FOC_MODE & FOC_OBSERVER_MASK) == FOC_OBSERVER_SENSORLESS) */

#if (_SUPPORT_TACHO_OUT != FALSE)
        if (g_e8MotorDirectionCCW != FALSE)
        {
            /* Counter Clock-wise */
            int16_t i16MicroStepChange = (int16_t) (l_u16MicroStepIdxPrev - l_u16MicroStepIdx);  /* MMP230715-1 */
            if (i16MicroStepChange > (int16_t)(l_u16MotorMicroStepsPerElecRotation/2))
            {
                i16MicroStepChange = i16MicroStepChange - l_u16MotorMicroStepsPerElecRotation;
            }
            else if (i16MicroStepChange < (int16_t)(0 - (l_u16MotorMicroStepsPerElecRotation/2)) )
            {
                i16MicroStepChange = i16MicroStepChange + l_u16MotorMicroStepsPerElecRotation;
            }
            l_u16TachoCount += i16MicroStepChange;
        }
        else
        {
            /* Clock-wise */
            int16_t i16MicroStepChange = (int16_t) (l_u16MicroStepIdx - l_u16MicroStepIdxPrev);  /* MMP230715-1 */
            if (i16MicroStepChange > (int16_t)(l_u16MotorMicroStepsPerElecRotation/2))
            {
                i16MicroStepChange = i16MicroStepChange - l_u16MotorMicroStepsPerElecRotation;
            }
            else if (i16MicroStepChange < (int16_t)(0 - (l_u16MotorMicroStepsPerElecRotation/2)) )
            {
                i16MicroStepChange = i16MicroStepChange + l_u16MotorMicroStepsPerElecRotation;
            }
            l_u16TachoCount += i16MicroStepChange;
        }
        if ( (l_u16TachoCount >= l_u16TachoThreshold) && (l_u16TachoThreshold != 0U) )
        {
            /* Tacho is enabled */
            l_u16TachoCount -= l_u16TachoThreshold;
#if (TACHO_OUT_IO == PIN_FUNC_IO_0)
            IO_PORT_IO_OUT_SOFT ^= C_PORT_IO_OUT_SOFT_IO0_OUT;
#elif (TACHO_OUT_IO == PIN_FUNC_IO_1)
            IO_PORT_IO_OUT_SOFT ^= C_PORT_IO_OUT_SOFT_IO1_OUT;
#elif (TACHO_OUT_IO == PIN_FUNC_IO_2)
            IO_PORT_IO_OUT_SOFT ^= C_PORT_IO_OUT_SOFT_IO2_OUT;
#elif (TACHO_OUT_IO == PIN_FUNC_IO_3)
            IO_PORT_IO_OUT_SOFT ^= C_PORT_IO_OUT_SOFT_IO3_OUT;
#else
#error "ERROR: Tacho-out I/O port not supported"
#endif
        }
#if (_DEBUG_FOC_PERF != FALSE)
        DEBUG_TOG_IO_C();
#endif /* (_DEBUG_FOC_PERF != FALSE) */
#endif /* (_SUPPORT_TACHO_OUT != FALSE) */

#if (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE)
        {
            int16_t i16MicroStepDelta = (int16_t)(l_u16MicroStepIdx - l_u16MicroStepIdxPrev);   /* Opening */
            if (i16MicroStepDelta > (int16_t)(l_u16MotorMicroStepsPerElecRotation / 2U) )
            {
                i16MicroStepDelta -= (int16_t)l_u16MotorMicroStepsPerElecRotation;
            }
            else if (i16MicroStepDelta < (int16_t)-(l_u16MotorMicroStepsPerElecRotation / 2U) )
            {
                i16MicroStepDelta += (int16_t)l_u16MotorMicroStepsPerElecRotation;
            }
            else
            {
                /* In range */
            }
            p_AddU32byI16( (uint32_t*)&l_u32ActualPosition, i16MicroStepDelta);
        }
        l_u16DeltaPosition = DeltaPosition();
        if (l_u16DeltaPosition == 0U)
        {
            MotorDriverStop( (uint16_t)C_STOP_IMMEDIATE);                       /* CPOS = FPOS */
            break;
        }

        if ( (l_u16DeltaPosition < l_u16RampDownSteps) && (l_e8MotorStartupMode != E_MSM_STEPPER_D) )
        {
            /* Decelerate motor speed (almost at target-position) */
            l_u16TargetCommutTimerPeriod = l_u16LowSpeedPeriod;
            l_u16StallDetectorDelay = l_u16DeltaPosition;
            ENTER_SECTION(ATOMIC_KEEP_MODE);  /*lint !e534 */
            {
                if ( (g_e8MotorStatus & C_MOTOR_STATUS_STOP_MASK) != C_MOTOR_STATUS_STOP)
                {
                    g_e8MotorStatus = C_MOTOR_STATUS_STOPPING;
                }
            }
            EXIT_SECTION(); /*lint !e438 */
            l_e8MotorStartupMode = E_MSM_STEPPER_D;
        }
#if (_DEBUG_FOC_PERF != FALSE)
        DEBUG_TOG_IO_C();
#endif /* (_DEBUG_FOC_PERF != FALSE) */
#endif /* (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) */

#if (_SUPPORT_FOC_ID_IQ_MODE == FOC_ID_IQ_iCLARKE)                              /* MMP220722-1 */
        if (l_e8MotorStartupMode == E_MSM_FOC_CL_STARTUP)                       /* MMP210527-1: Swap order IV (Close-loop start-up) and IB */
#endif /* (_SUPPORT_FOC_ID_IQ_MODE == FOC_ID_IQ_iCLARKE) */
        {
            l_u16CorrectionRatio = VoltageCorrection();
#if (_DEBUG_FOC_PERF != FALSE)
            DEBUG_TOG_IO_C();
#endif /* (_DEBUG_FOC_PERF != FALSE) */
#if (C_MOTOR_PHASES == 3)
            MotorDriver_3Phase(l_u16MicroStepIdx);
#else  /* (C_MOTOR_PHASES == 3) */
            MotorDriver_4Phase(l_u16MicroStepIdx);
#endif /* (C_MOTOR_PHASES == 3) */
#if (_DEBUG_FOC_PERF != FALSE)
            DEBUG_TOG_IO_C();
#endif /* (_DEBUG_FOC_PERF != FALSE) */
        }

#if ((_SUPPORT_FOC_MODE & FOC_OBSERVER_MASK) == FOC_OBSERVER_SENSORLESS) && (_SUPPORT_CLOSED_LOOP_STARTUP == FALSE)
#if defined (C_SPEED_CL2OL)
        if (g_u16ActualMotorSpeedRPM < C_SPEED_CL2OL)                           /* MMP231204-1 */
#elif (_SUPPORT_AUTO_SPEED_FOC == FALSE)
        if (g_u16ActualMotorSpeedRPM < g_u16LowSpeedRPM)                        /* MMP230912-2 */
#else  /* (_SUPPORT_AUTO_SPEED_FOC == FALSE) */
        if (g_u16ActualMotorSpeedRPM < (g_u16MaxSpeedRPM - g_u16MinSpeedRPM))   /* MMP230918-1 */
#endif /* (_SUPPORT_AUTO_SPEED_FOC == FALSE) */
        {
            SwitchClose2Open();
        }
#endif /* ((_SUPPORT_FOC_MODE & FOC_OBSERVER_MASK) == FOC_OBSERVER_SENSORLESS) && (_SUPPORT_CLOSED_LOOP_STARTUP == FALSE) */

        if (MotorStallCheckA() != C_STALL_NOT_FOUND)
        {
            /* Max current/torque stall detected */
            g_u8StallTypeComm |= (uint8_t)C_STALL_FOUND_A;
            if ( (g_e8StallDetectorEna & ((uint8_t)C_STALLDET_A | (uint8_t)C_STALLDET_CALIB)) != 0U)
            {
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
#if (_SUPPORT_STALLDET_LA != FALSE)
        else if (MotorStallCheckLA() != C_STALL_NOT_FOUND)
        {
            /* Invalid Load-angle stall detected */
            g_u8StallTypeComm |= (uint8_t)C_STALL_FOUND_LA;                     /* StallCheckIV */
            if ( (g_e8StallDetectorEna & (C_STALLDET_LA | C_STALLDET_CALIB)) != 0U)
            {
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
#endif /* (_SUPPORT_STALLDET_LA != FALSE) */
#if (_SUPPORT_STALLDET_FLUX != FALSE)
        else if (MotorStallCheckFlux(l_u16Flux) != C_STALL_NOT_FOUND)
        {
            /* Invalid Load-angle stall detected */
            g_u8StallTypeComm |= (uint8_t)C_STALL_FOUND_FLUX;                   /* StallCheckFlux */
            if ( (g_e8StallDetectorEna & (C_STALLDET_FLUX | C_STALLDET_CALIB)) != 0U)
            {
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
#endif /* (_SUPPORT_STALLDET_FLUX != FALSE) */
    } while (FALSE);

#if (_DEBUG_FOC_PERF != FALSE)
    DEBUG_CLR_IO_C();
#endif /* (_DEBUG_FOC_PERF != FALSE) */
#if (_DEBUG_COMMUT_ISR != FALSE)
    DEBUG_CLR_IO_B();
#endif /* (_DEBUG_COMMUT_ISR != FALSE) */
#if (_DEBUG_CPU_LOAD != FALSE)
    DEBUG_CLR_IO_B();                                                           /* IRQ-Priority 4 */
#endif /* (_DEBUG_CPU_LOAD != FALSE) */
} /* End of ADC_ISR() */

#if (_SUPPORT_CDI != FALSE)
/*!*************************************************************************** *
 * SwitchFOC2CDI
 * \brief   Switch-over from FOC to CDI
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details
 * *************************************************************************** *
 * - Call Hierarchy: CheckActivationCDI()
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 2 (HAL_ADC_DisableIRQ(), MotorDriver_3PhaseCDI())
 * *************************************************************************** */
static void SwitchFOC2CDI(void)
{
    l_u16PidCtrlRatio = p_MulU16hi_U16byU16(l_u16PidCtrlRatio, C_FOC2CDI_AMPL_CORR);
    l_u16CommutTimerPeriod = p_DivU16_U32byU16(l_u32FullStepPeriodOneRPM, g_u16ActualMotorSpeedRPM) - 1U;
    IO_CTIMER0_TREGB = l_u16CommutTimerPeriod;
    g_u16ActualCommutTimerPeriod = l_u16CommutTimerPeriod;
    l_u32CommutTimerPeriodLPF = ((uint32_t)l_u16CommutTimerPeriod) << 16;
    l_u16FullStepIdx = (l_u16MicroStepIdx + (C_MICROSTEP_PER_FULLSTEP / 2)) / C_MICROSTEP_PER_FULLSTEP;  /* Convert Micro-step to Full-step */
    l_u16FullStepIdxPrev = l_u16FullStepIdx;
    l_au16CDI_TCNT[0] = 0U;
    l_au16CDI_TCNT[1] = 0U;
    l_au16CDI_TCNT[2] = 0U;
    l_au16CDI_TPER[0] = 0U;
    l_au16CDI_TPER[1] = 0U;
    l_au16CDI_TPER[2] = g_u16ActualCommutTimerPeriod;
    l_e8CDI_Mode = (uint8_t)C_CDI_DISABLED;
    l_u8StallCountCDI = 0U;
    extern uint16_t l_u16PID_CtrlCounter;
    l_u16PID_CtrlCounter = 0U;
#if (_SUPPORT_CDI_PID != FALSE)
#if (_SUPPORT_PID_D_COEF != FALSE)
    sPIDpCDI.i16PrevError = 0;
#endif /* (_SUPPORT_PID_D_COEF != FALSE) */
    sPIDpCDI.u32SumError = (((uint32_t)g_u16ActualCommutTimerPeriod) << C_GN_PID);
#else  /* (_SUPPORT_CDI_PID != FALSE) */
    l_i32CDIOffset = 0;
#endif /* (_SUPPORT_CDI_PID != FALSE) */
    HAL_ADC_DisableIRQ();                                                       /* Disable ADC Interrupt */
    IO_CTIMER0_CTRL = B_CTIMER0_START;                                          /* Start "commutation timer" */
    l_u16CorrectionRatio = VoltageCorrection();
    IO_PWM_SLAVE2_CTRL = B_PWM_SLAVE2_SLAVE | C_PWM_SLAVE2_MODE_MIRROR | B_PWM_SLAVE2_POL;
    MotorDriver_3PhaseCDI(l_u16FullStepIdx);
} /* End of SwitchFOC2CDI() */

/*!*************************************************************************** *
 * SwitchCDI2FOC
 * \brief   Switch-over from CDI to FOC
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details
 * *************************************************************************** *
 * - Call Hierarchy: CheckActivationCDI()
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 2 (PID_SwitchCDI2FOC(), HAL_ADC_EnableIRQ())
 * *************************************************************************** */
static void SwitchCDI2FOC(void)
{
    l_u16MicroStepIdx = (l_u16FullStepIdx * C_MICROSTEP_PER_FULLSTEP) + (C_MICROSTEP_PER_FULLSTEP / 2U);
    l_u16MotorVoltageAngle = p_DivU16_U16hibyU16(l_u16MicroStepIdx, l_u16MotorMicroStepsPerElecRotation);
    PID_SwitchCDI2FOC();
    HAL_ADC_EnableIRQ();                                                        /* Enable ADC Interrupt */
    IO_CTIMER0_CTRL = B_CTIMER0_STOP;                                           /* Stop "commutation timer" */
    l_u16CorrectionRatio = VoltageCorrection();
    DRVCFG_PWM_UVW();
    MotorDriver_3Phase(l_u16MicroStepIdx);
} /* End of SwitchCDI2FOC() */

/*!*************************************************************************** *
 * CheckActivationCDI
 * \brief   Check for switch-over from FOC to CDI
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details
 * *************************************************************************** *
 * - Call Hierarchy: main()
 * - Cyclomatic Complexity: 3+1
 * - Nesting: 2
 * - Function calling: 1 (SwitchFOC2CDI())
 * *************************************************************************** */
void CheckActivationCDI(void)
{
    if ( ((l_e8MotorStartupMode & E_MSM_CDI) == 0U) &&
         (l_u16CorrectionRatio > (29U * l_u16MaxPwmRatio)) )                    /* 29/32 of max PWM ratio */
    {
        uint16_t u16MicroStepIdx = l_u16MicroStepIdx;
        if ( (l_u16MicroStepIdxPrev < (uint16_t)(3 * C_MICROSTEP_PER_FULLSTEP)) &&
             (u16MicroStepIdx >= (uint16_t)(3 * C_MICROSTEP_PER_FULLSTEP)) )
        {
#if (_DEBUG_CDI != FALSE)
            DEBUG_SET_IO_A();
#endif /* (_DEBUG_CDI != FALSE) */
            SwitchFOC2CDI();
            l_e8MotorStartupMode |= E_MSM_CDI;                                  /* CDI Mode */
#if (_DEBUG_CDI != FALSE)
            DEBUG_CLR_IO_A();
#endif /* (_DEBUG_CDI != FALSE) */
        }
    }
    else if ( ((l_e8MotorStartupMode & E_MSM_CDI) != 0U) &&
              (l_u16CorrectionRatio < (((27UL * C_FOC2CDI_AMPL_CORR) / C_ANGLE_360DEG) * l_u16MaxPwmRatio)) )  /* 27/32 of max PWM ratio (MMP210921-2) */
    {
        if (l_u16FullStepIdx == 0U)
        {
#if (_DEBUG_CDI != FALSE)
            DEBUG_SET_IO_A();
#endif /* (_DEBUG_CDI != FALSE) */
            SwitchCDI2FOC();
            l_e8MotorStartupMode &= ~E_MSM_CDI;                                 /* FOC Mode */
#if (_DEBUG_CDI != FALSE)
            DEBUG_CLR_IO_A();
#endif /* (_DEBUG_CDI != FALSE) */
        }
    }
    else
    {
        /* Nothing */
    }

} /* End of CheckActivationCDI() */

/*!*************************************************************************** *
 * ISR_CDI
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
 *                        VoltageCorrection(), MotorDriver_4PhaseStepper(),
 *                        MotorStallCheckA())
 * *************************************************************************** */
__attribute__((interrupt)) void ISR_CDI(void)
{
    uint16_t u16CurrentCrossPeriod = IO_CTIMER0_TCNT;

#if (_DEBUG_CDI != FALSE)
    DEBUG_SET_IO_C();
#endif /* (_DEBUG_CDI != FALSE) */
    if ( (IO_ACTIVE_CDI_IN & B_ACTIVE_CDI_IN_VALID) == 0U)                      /* Data-sheet: CDI-VALID should be '0' (MMP210921-1) */
    {
#if (_DEBUG_CDI != FALSE)
        DEBUG_SET_IO_D();
#endif /* (_DEBUG_CDI != FALSE) */
        l_u16CDI = (IO_ACTIVE_CDI & M_IO_ACTIVE_CDI_ACTIVE_CDI_PHASE);
        IO_ACTIVE_CDI = (IO_ACTIVE_CDI & ~M_IO_ACTIVE_CDI_ACTIVE_CDI_PHASE) | C_IO_ACTIVE_CDI_ACTIVE_CDI_PHASE_DIS;
#if (_SUPPORT_APP_USER_MODE != FALSE)
        ENTER_SECTION(SYSTEM_MODE); /*lint !e534 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
        IO_MLX16_ITC_PEND5_S = B_MLX16_ITC_PEND5_ACTIVE_CDI;
#if (_SUPPORT_APP_USER_MODE != FALSE)
        EXIT_SECTION(); /*lint !e438 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
        if ( (l_e8MotorStartupMode & E_MSM_CDI) != 0U)
        {
            if (l_au16CDI_TCNT[0] != 0U)
            {
                uint16_t u16ActualOffset =
                    (l_au16CDI_TCNT[0] + l_au16CDI_TCNT[1] + l_au16CDI_TCNT[2] + u16CurrentCrossPeriod) >> 2;                         /* Average of four CDI-IRQ Offset periods */
#if (_SUPPORT_SPECIAL_COMM_FIELD != FALSE)
                uint16_t u16TargetOffset =
                    p_MulU16hi_U16byU16(g_u16ActualCommutTimerPeriod, ((uint16_t)g_u8Special[0] * 64U));
#else  /* (_SUPPORT_SPECIAL_COMM_FIELD != FALSE) */
                uint16_t u16TargetOffset = p_MulU16hi_U16byU16(g_u16ActualCommutTimerPeriod, C_CDI_COMMUT_OFFSET);
#endif /* (_SUPPORT_SPECIAL_COMM_FIELD != FALSE) */
                int16_t i16OffsetCorrection = (int16_t)(u16TargetOffset - u16ActualOffset);   /* Expected CDI-IRQ Offset period minus measurement */
#if (_SUPPORT_CDI_PID != FALSE)
                uint16_t u16ActualCommutTimerPeriod = (uint16_t)p_PID_Control(i16OffsetCorrection, (void *)&sPIDpCDI);
                IO_CTIMER0_TREGB = u16ActualCommutTimerPeriod;                  /* Update Full-step period */
#else  /* (_SUPPORT_CDI_PID != FALSE) */
#if (_SUPPORT_SPECIAL_COMM_FIELD != FALSE)
                i16OffsetCorrection =
                    p_LpfI16_I16byI16(&l_i32CDIOffset, ((uint16_t)g_u8Special[1] * 32U),
                                      (i16OffsetCorrection - (l_i32CDIOffset >> 16)));
#else  /* (_SUPPORT_SPECIAL_COMM_FIELD != FALSE) */
                i16OffsetCorrection =
                    p_LpfI16_I16byI16(&l_i32CDIOffset, C_CDI_OFFSET_LPF_COEF,
                                      (i16OffsetCorrection - (l_i32CDIOffset >> 16)));
#endif /* (_SUPPORT_SPECIAL_COMM_FIELD != FALSE) */
                IO_CTIMER0_TREGB = g_u16ActualCommutTimerPeriod + i16OffsetCorrection;  /* Update Full-step period */
#endif /* (_SUPPORT_CDI_PID != FALSE) */
            }
            l_au16CDI_TCNT[0] = l_au16CDI_TCNT[1];
            l_au16CDI_TCNT[1] = l_au16CDI_TCNT[2];
            l_au16CDI_TCNT[2] = u16CurrentCrossPeriod;
            l_e8CDI_Mode = (uint8_t)C_CDI_FOUND;                                /* Current Crossing found (falling edge) */
        }
#if (_DEBUG_CDI != FALSE)
        DEBUG_CLR_IO_D();
#endif /* (_DEBUG_CDI != FALSE) */
    }
#if (_DEBUG_CDI != FALSE)
    DEBUG_CLR_IO_C();
#endif /* (_DEBUG_CDI != FALSE) */
} /* End of ISR_CDI() */
#endif /* (_SUPPORT_CDI != FALSE) */

#endif /* (_SUPPORT_FOC != FALSE) && (C_MOTOR_PHASES > 1) */

/* EOF */
