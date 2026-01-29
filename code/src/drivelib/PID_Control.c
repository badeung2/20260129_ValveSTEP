/*!*************************************************************************** *
 * \file        PID_Control.c
 * \brief       MLX8133x/4x PID Controller handling
 *
 * \note        project MLX8133x/4x
 *
 * \author      Marcel Braat
 *
 * \date        2017-08-15
 *
 * \version     2.0
 *
 * A list of functions:
 *  - Global Functions:
 *           -# PID_SetRunningCurrent()
 *           -# PID_Init()
 *           -# PID_Start()
 *           -# PID_WindmillStart()
 *           -# PID_SpeedCompensate()
 *           -# PID_SwitchStepper2Bemf()
 *           -# VoltageCorrection()
 *           -# PID_Control()
 *           -# PID_SwitchOpen2Close()
 *           -# PID_LoadAngleToActSpeed()
 *           -# ThresholdControl()
 *  - Internal Functions:
 *           -# PID_Control_Period()
 *           -# PID_Control_Error()
 *
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

#if (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT)

#include "../ActADC.h"                                                          /* Application ADC support */

#include "drivelib/NV_Functions.h"                                              /* Non Volatile Memory Functions & Layout */
#include "drivelib/ErrorCodes.h"                                                /* Error logging support */
#include "drivelib/GlobalVars.h"                                                /* Global Variables support */
#include "drivelib/MotorDriver.h"                                               /* Motor driver support */
#if (_SUPPORT_STALLDET_S != FALSE)
#include "drivelib/MotorStall.h"                                                /* Motor Stall Detectors */
#endif /* (_SUPPORT_STALLDET_S != FALSE) */
#if ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_ID_IQ)
#include "drivelib/MotorDriverTables.h"                                         /* Wave-form vector tables */
#endif /* ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_ID_IQ) */
#include "drivelib/PID_Control.h"                                               /* PID support */
#include "drivelib/Timer.h"                                                     /* Simple Timer support */
#include "camculib/private_mathlib.h"                                           /* Private Math Library support */

#if (_SUPPORT_CURRSPIKE_POSITION != FALSE)
#include <atomic.h>
#endif /* (_SUPPORT_CURRSPIKE_POSITION != FALSE) */

/*!*************************************************************************** *
 *                            DEFINES                                          *
 * *************************************************************************** */
#if (_APP_SUPPLY_RANGE == C_APP_SUPPLY_RANGE_12V)                               /* MMP230912-1 */
#define C_REF_VOLTAGE               1200U                                       /*!< 12.00V in [10mV]-units */
#elif (_APP_SUPPLY_RANGE == C_APP_SUPPLY_RANGE_24V)
#define C_REF_VOLTAGE               2400U                                       /*!< 24.00V in [10mV]-units */
#elif (_APP_SUPPLY_RANGE == C_APP_SUPPLY_RANGE_48V)
#define C_REF_VOLTAGE               4800U                                       /*!< 48.00V in [10mV]-units */
#endif

#define C_TARGET_LA_CLSU            16384U                                      /*!< Close-loop startup target Lead-Angle (90 degrees) */

#if (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM)
#define C_SENSE_PWMDC (uint16_t)(0.09375 * 256)                                 /*!< 9.375% Sense PWM DC */
#endif /* (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM) */

#if (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM_BIPOLAR)
#if (C_MICROSTEP_PER_FULLSTEP == 8)
#define C_MICRO_STEP_RAD    ((uint16_t)(((4096UL * 240) + 180U) / 360U))        /*!< 7.5 degrees in radial; One update per 7.5 degrees */
#elif (C_MICROSTEP_PER_FULLSTEP == 16)
#define C_MICRO_STEP_RAD    ((uint16_t)(((4096UL * 120) + 180U) / 360U))        /*!< 120 degrees in radial; One update per 7.5 degrees */
#elif (C_MICROSTEP_PER_FULLSTEP == 32)
#define C_MICRO_STEP_RAD    ((uint16_t)(((4096UL * 1.875) + 180U) / 360U))      /*!< 1.875 degrees in radial; One update per 7.5 degrees */
#endif
#endif /* (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM_BIPOLAR) */

#if (_SUPPORT_PID_INLINE == FALSE) && ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) != FOC_MODE_ID_IQ)
/*!< Static variable defined */
#define PID_STATIC static
#else
/*!< Static variable not defined */
#define PID_STATIC
#endif /* (_SUPPORT_PID_INLINE == FALSE) */

/*!*************************************************************************** */
/*                           GLOBAL & LOCAL VARIABLES                          */
/* *************************************************************************** */
#pragma space dp                                                                /* __TINY_SECTION__ */
uint16_t l_u16PidCtrlRatio;                                                     /*!< PID Control Ratio (at ref. voltage) */
#if ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_ID_IQ) || ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_IB_IPK)
uint16_t l_u16TargetIpk;                                                        /*!< Target Ipeak */
#endif
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__)
#if (_SUPPORT_ADC_REF_HV_CALIB != FALSE)
PID_STATIC uint16_t l_u16MotorRefVoltageADC = (uint16_t)((12U * 1024U) / (1.5 * C_ADC_HV_DIV));  /*!< Motor Reference Voltage; 12.00V [ADC-LSB] */
#else  /* (_SUPPORT_ADC_REF_HV_CALIB != FALSE) */
PID_STATIC uint16_t l_u16MotorRefVoltageADC = (uint16_t)((12U * 1024U) / (2.5 * C_ADC_HV_DIV));  /*!< Motor Reference Voltage; 12.00V [ADC-LSB] */
#endif /* (_SUPPORT_ADC_REF_HV_CALIB != FALSE) */
#elif defined (__MLX81339__) || defined (__MLX81350__)
PID_STATIC uint16_t l_u16MotorRefVoltageADC = (uint16_t)((12U * 4096U) / (1.65 * C_ADC_HV_DIV));  /*!< Motor Reference Voltage; 12.00V [ADC-LSB] */
#elif defined (__MLX81160__) || defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
PID_STATIC uint16_t l_u16MotorRefVoltageADC = (uint16_t)((12U * 2048U) / (1.5 * C_ADC_HV_DIV));  /*!< Motor Reference Voltage; 12.00V [ADC-LSB] */
#endif
PID_STATIC uint16_t l_u16MinCorrectionRatio;                                    /*!< PID Minimum correction ratio */
#if (LINPROT == LIN13_HVACTB)
uint16_t l_u16MaxCorrectionRatio;                                               /*!< PID Maximum correction ratio */
#else  /* (LINPROT == LIN13_HVACTB) */
PID_STATIC uint16_t l_u16MaxCorrectionRatio;                                    /*!< PID Maximum correction ratio */
#endif /* (LINPROT == LIN13_HVACTB) */
#if (_SUPPORT_FOC_MODE != FOC_MODE_NONE)                                        /* FOC */
PID_STATIC uint16_t l_u16MaxSpeedTgtLoadAngle = 0U;                             /*!< Target EE Load-angle at Maximum Speed (ACT_SPEED4) */
#endif /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) */                               /* FOC */
#pragma space none                                                              /* __TINY_SECTION__ */

#pragma space nodp                                                              /* __NEAR_SECTION__ */
uint16_t l_u16PID_CtrlCounter = 0U;                                             /*!< PID Control period counter */
#if (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT)
uint16_t l_u16PidHoldingThreshold;                                              /*!<  Motor holding current threshold */
#endif /* (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) */
#if ((_SUPPORT_FOC_MODE != FOC_MODE_NONE) && ((_SUPPORT_CLOSED_LOOP_STARTUP == FALSE) || \
    (_SUPPORT_TRIAXIS_MLX9038x != FALSE))) || \
    ((_SUPPORT_FOC_MODE == FOC_MODE_NONE) && ((_SUPPORT_HALL_LATCH == FALSE) || (_SUPPORT_NR_OF_HL == 1) || (_SUPPORT_HALL_LATCH_DIAG != FALSE)))
static uint16_t l_u16PidLosses = 0U;                                            /*!< PID Losses fraction */
#endif /* (_SUPPORT_CLOSED_LOOP_STARTUP == FALSE) || (_SUPPORT_HALL_LATCH == FALSE) || (_SUPPORT_NR_OF_HL == 1) || (_SUPPORT_HALL_LATCH_DIAG != FALSE) */
#if (_SUPPORT_CURRENT_TEMP_COMPENSATION != FALSE)
uint16_t l_u16PID_ThrshldCtrlCounter = 0U;                                      /*!< Threshold compensation counter */
#endif /* (_SUPPORT_CURRENT_TEMP_COMPENSATION != FALSE) */
#if (_SUPPORT_PID_SPEED_LIMIT != FALSE)
static uint16_t l_u16SpeedRampDownLimit;                                        /*!< PID Speed limit ramp-down */
static uint16_t l_u16SpeedRampUpLimit;                                          /*!< PID Speed limit ramp-up */
#endif /* (_SUPPORT_PID_SPEED_LIMIT != FALSE) */
PID_PARAMS_t sPIDpSE2VA =                                                       /*!< PID parameter structure for Speed-Error to Vector-Amplitude */
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
uint16_t l_u16ActCurrRunMax_mA = 0U;                                            /*!< Motor (maximum) running current [mA] */
uint16_t l_u16ActCurrRunMax_LSB;                                                /*!< Motor (maximum) running current [ADC-LSB] */
#if (_SUPPORT_FOC_MODE != FOC_MODE_NONE) || ((_SUPPORT_PWM_MODE == SINGLE_COIL_PWM_BIPOLAR) && (_SUPPORT_SPEED_CTRL != FALSE)) || \
    ((_SUPPORT_HALL_LATCH != FALSE) && (_SUPPORT_NR_OF_HL > 1) && (_SUPPORT_HALL_LATCH_DIAG == FALSE))
static uint16_t l_u16ActCurrRunMin_LSB;                                         /*!< Motor current Running (minimum) (MMP220204-1) */
#endif /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) || ((_SUPPORT_PWM_MODE == SINGLE_COIL_PWM_BIPOLAR) && (_SUPPORT_SPEED_CTRL != FALSE)) */
#if (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM_BIPOLAR)
static uint16_t l_u16ActCurrAlign_LSB;                                          /*!< Motor Alignment current [ADC-LSB] */
static uint16_t l_u16PID_Period;                                                /*!< PID Controller Period */
#endif /* (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM_BIPOLAR) */
#if (_SUPPORT_FOC_MODE != FOC_MODE_NONE)                                        /* FOC */
#if (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM_BIPOLAR)
#if (_SUPPORT_VOLTAGE_CTRL != FALSE)
static uint16_t l_u16ActualMotorVoltage;                                        /*!< PID Actual Voltage */
#endif /* (_SUPPORT_VOLTAGE_CTRL != FALSE) */
PID_LA_Commut_t sPIDpCommut = { 0, 0, 0, 0U};                                   /*!< PID Commutation period structure */
#else  /* (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM_BIPOLAR) */
static uint16_t l_u16ActCurrStartMax_LSB;                                       /*!< Motor current Starting (Maximum) [ADC-LSB] */
PID_PARAMS_t sPIDpLA2AS =                                                       /*!< PID parameter structure for Load-Angle to Actual Speed */
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
#endif /* (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM_BIPOLAR) */
#if ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_ID_IQ)
PID_PARAMS_t sPID_IdCtrl =                                                      /*!< PID Id Control structure */
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
PID_PARAMS_t sPID_IqCtrl =                                                      /*!< PID Iq Control structure */
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
#elif ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_IB_IPK)
PID_PARAMS_t sPID_IpkCtrl =                                                     /*!< PID I-Peak Control structure */
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
#endif /* (_SUPPORT_FOC_MODE & FOC_MODE_MASK) */
#if ((LIN_COMM != FALSE) && (_SUPPORT_MLX_DEBUG_MODE != FALSE))
uint16_t l_u16ActSpeedTgtLoadAngle = 0U;                                        /*!< Speed dependent Target Load-angle */
#if (_SUPPORT_CLOSED_LOOP_STARTUP != FALSE)
int16_t l_i16ActSpeedTgtLoadAngleIV = 0;                                        /*!< Speed dependent Target Load-angle (IV) */
#endif /* (_SUPPORT_CLOSED_LOOP_STARTUP != FALSE) */
#endif /* ((LIN_COMM != FALSE) && (_SUPPORT_MLX_DEBUG_MODE != FALSE)) */
#endif /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) */
#if (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM)
static uint16_t l_u16SenseCorrectionRatio;                                      /*!< PWM-DC for Sensing ripple count */
#endif /* (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM) */
#if (_SUPPORT_POTI != FALSE) && (_SUPPORT_POTI_MODE == C_POTI_MODE_CLOSED_LOOP)
uint16_t l_au16PID_SpeedRef[8];                                                 /*!< Speed reference table */
uint16_t l_u16PID_LastPotiPos = 0U;                                             /*!< Last Potentiometer Position */
#endif /* (_SUPPORT_POTI != FALSE) && (_SUPPORT_POTI_MODE == C_POTI_MODE_CLOSED_LOOP) */
#if (_SUPPORT_CURRSPIKE_POSITION != FALSE)
uint16_t l_u16PidSpikePeriodSum = 0U;                                           /*!< PID Spike period sum */
uint16_t l_u16PidSpikeCount = 0U;                                               /*!< PID Spike count */
#endif /* (_SUPPORT_CURRSPIKE_POSITION != FALSE) */
#pragma space none                                                              /* __NEAR_SECTION__ */

/*!*************************************************************************** *
 * PID_SetRunningCurrent
 * \brief   Set running current PID level
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16CurrentLevel: Motor current level in mA's
 * \return  -
 * *************************************************************************** *
 * \details -
 * *************************************************************************** *
 * - Call Hierarchy: PID_Init()
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 0 (all inline)
 * *************************************************************************** */
void PID_SetRunningCurrent(uint16_t u16CurrentLevel)
{
    l_u16ActCurrRunMax_mA = u16CurrentLevel;
    l_u16ActCurrRunMax_LSB = p_MulDivU16_U16byU16byU16(l_u16ActCurrRunMax_mA, C_GMCURR_DIV, Get_MCurrGain());
} /* End of PID_SetRunningCurrent() */

/*!*************************************************************************** *
 * PID_Init
 * \brief   Initialise PID
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details
 * Note:    This function is not allowed to be called during Non Volatile Memory Write
 *          (due to Non Volatile Memory reads).
 * *************************************************************************** *
 * - Call Hierarchy: main_Init()
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 1 (PID_SetRunningCurrent())
 * *************************************************************************** */
void PID_Init(void)
{
    /* Convert [mA] to [ADC-LSB] */
#if ((_SUPPORT_PWM_MODE == SINGLE_COIL_PWM_BIPOLAR) && (_SUPPORT_SPEED_CTRL != FALSE))
    l_u16ActCurrAlign_LSB = p_MulDivU16_U16byU16byU16(NV_STARTUP_CURR_MAX, C_GMCURR_DIV, Get_MCurrGain());  /* Alignment current */
#endif /* ((_SUPPORT_PWM_MODE == SINGLE_COIL_PWM_BIPOLAR) && (_SUPPORT_SPEED_CTRL != FALSE)) */
#if (_SUPPORT_FOC_MODE != FOC_MODE_NONE)
#if (_SUPPORT_PWM_MODE != SINGLE_COIL_PWM_BIPOLAR)
    l_u16ActCurrStartMax_LSB = p_MulDivU16_U16byU16byU16(NV_STARTUP_CURR_MAX, C_GMCURR_DIV, Get_MCurrGain());
#endif /* (_SUPPORT_PWM_MODE != SINGLE_COIL_PWM_BIPOLAR) */
    PID_SetRunningCurrent(NV_RUNNING_CURR_MAX);
#else  /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) */
    PID_SetRunningCurrent(NV_RUNNING_CURR_LEVEL);
#endif /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) */
#if (_SUPPORT_FOC_MODE != FOC_MODE_NONE) || ((_SUPPORT_PWM_MODE == SINGLE_COIL_PWM_BIPOLAR) && (_SUPPORT_SPEED_CTRL != FALSE)) || /* MMP220204-1 */ \
    ((_SUPPORT_HALL_LATCH != FALSE) && (_SUPPORT_NR_OF_HL > 1) && (_SUPPORT_HALL_LATCH_DIAG == FALSE))  /* MMP230719-1 */
    l_u16ActCurrRunMin_LSB = p_MulDivU16_U16byU16byU16(NV_RUNNING_CURR_MIN, C_GMCURR_DIV, Get_MCurrGain());
#endif /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) || ((_SUPPORT_PWM_MODE == SINGLE_COIL_PWM_BIPOLAR) && _SUPPORT_SPEED_CTRL) */

#if (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT)
    l_u16PidHoldingThreshold = p_MulDivU16_U16byU16byU16(NV_HOLDING_CURR_LEVEL, C_GMCURR_DIV, Get_MCurrGain());
#endif /* (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) */

#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__)
    l_u16MotorRefVoltageADC =
        p_MulDivU16_U16byU16byU16(NV_VSUP_REF, C_VOLTGAIN_DIV, Get_MotorVoltGainF()) + Get_VsmOffset();
#elif defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
    l_u16MotorRefVoltageADC = p_MulDivU16_U16byU16byU16(NV_VSUP_REF, C_VOLTGAIN_DIV, Get_MotorVoltGainF());
#endif

    l_u16MinCorrectionRatio = NV_MIN_CORR_RATIO;
#if (PWM_LIMIT == PWM_LIMIT_BY_PID)
    l_u16MaxCorrectionRatio = NV_MAX_CORR_RATIO_PID;                            /* PWM duty cycle standard limitation (MMP240604-1) */
#else  /* (PWM_LIMIT == PWM_LIMIT_BY_PID) */
    l_u16MaxCorrectionRatio = NV_MAX_CORR_RATIO_PWM;                            /* PWM duty cycle over-modulation limitation (MMP240604-1) */
#endif /* (PWM_LIMIT == PWM_LIMIT_BY_PID) */
#if (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM)
    l_u16SenseCorrectionRatio =
        (uint16_t)(p_MulU32_U16byU16(C_SENSE_PWMDC, (PWM_REG_PERIOD << (C_PID_FACTOR + PWM_PRESCALER_N))) >> 8);
#endif /* (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM) */

#if (_SUPPORT_PID_SPEED_LIMIT != FALSE)
    if (NV_PID_RAMP_DOWN != 0U)   /*lint !e506 */
    {
        l_u16SpeedRampDownLimit = (uint16_t)(p_MulU32_U16byU16(g_u16MaxSpeedRPM, NV_PID_RAMP_DOWN) >> 10);
    }
    else
    {
        l_u16SpeedRampDownLimit = (g_u16MaxSpeedRPM >> 2);
    }
    if (NV_PID_RAMP_UP != 0U)   /*lint !e506 */
    {
        l_u16SpeedRampUpLimit = (uint16_t)(p_MulU32_U16byU16(g_u16MaxSpeedRPM, NV_PID_RAMP_UP) >> 8);
    }
    else
    {
        l_u16SpeedRampUpLimit = g_u16MaxSpeedRPM;
    }
    if (l_u16SpeedRampUpLimit > 32767U)
    {
        l_u16SpeedRampUpLimit = 32767U;
    }
#endif /* (_SUPPORT_PID_SPEED_LIMIT != FALSE) */

#if (_SUPPORT_FOC_MODE != FOC_MODE_NONE)
    l_u16MaxSpeedTgtLoadAngle = NV_TARGET_LA;
#if ((LIN_COMM != FALSE) && (_SUPPORT_MLX_DEBUG_MODE != FALSE))
    l_u16ActSpeedTgtLoadAngle = l_u16MaxSpeedTgtLoadAngle;
#endif /* ((LIN_COMM != FALSE) && (_SUPPORT_MLX_DEBUG_MODE != FALSE)) */
#if (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM_BIPOLAR)
    sPIDpCommut.i16PidLACoefI = (int16_t)NV_PID_LA_COEF_I;
    sPIDpCommut.i16PidLACoefP = (int16_t)NV_PID_LA_COEF_P;
#else  /* (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM_BIPOLAR) */
    /* PID Parameters for Load-Angle to Actual-Speed (PID Control Gain-factor: (2 ^ C_GN_PID)) */
    sPIDpLA2AS.i16CoefP = (int16_t)(NV_PID_LA_COEF_P << (C_GN_PID - 11U));      /* Lead-Angle PID P-coefficient */
    sPIDpLA2AS.i16CoefI = (int16_t)(NV_PID_LA_COEF_I << (C_GN_PID - 11U));      /* Lead-Angle PID I-coefficient */
#if (_SUPPORT_PID_D_COEF != FALSE)
    sPIDpLA2AS.i16CoefD = (int16_t)(NV_PID_LA_COEF_D << (C_GN_PID - 11U));      /* Lead-Angle PID D-coefficient (MMP240524-1) */
#endif /* (_SUPPORT_PID_D_COEF != FALSE) */
    sPIDpLA2AS.u16MinOutput = (uint16_t)p_MulU16lo_U16byU16(g_u16StartupSpeedRPM, g_u16MotorPolePairs);  /* Minimal Speed [eRPM] */
#if (_SUPPORT_PID_U32 == FALSE)
    sPIDpLA2AS.u32MaxOutput = 65535UL;                                          /* Maximal Speed [eRPM] */
    sPIDpLA2AS.u32SumErrorMax = (65535UL << C_GN_PID);                          /* Maximal Speed [eRPM] */
#else  /* (_SUPPORT_PID_U32 == FALSE) */
    sPIDpLA2AS.u32MaxOutput = (65535UL * C_FOC_PID_MULT);                       /* Maximal Speed [eRPM] */
    sPIDpLA2AS.u32SumErrorMax = ((65535UL * C_FOC_PID_MULT) << C_GN_PID);       /* Maximal Speed [eRPM] */
#endif /* (_SUPPORT_PID_U32 == FALSE) */
#endif /* (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM_BIPOLAR) */

#if ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_ID_IQ)
    /* TODO[MMP]: Id/Iq max output afhankelijk van VSM, slow loop */
    /* PID Parameters for Id control */
    sPID_IdCtrl.i16CoefP = (int16_t)(NV_PID_ID_COEF_P << (C_GN_PID - 12U));     /* Id PID P-coefficient */
    sPID_IdCtrl.i16CoefI = (int16_t)(NV_PID_ID_COEF_I << (C_GN_PID - 12U));     /* Id PID I-coefficient */
#if (_SUPPORT_PID_D_COEF != FALSE)
    sPID_IdCtrl.i16CoefD = (int16_t)(NV_PID_ID_COEF_D << (C_GN_PID - 12U));     /* Id PID D-coefficient */
#endif /* (_SUPPORT_PID_D_COEF != FALSE) */
    sPID_IdCtrl.u16MinOutput = 0U;                                              /* Minimal Motor-PWM Duty-Cycle */
    sPID_IdCtrl.u32MaxOutput = (uint32_t)l_u16MotorRefVoltageADC;               /* Maximal Motor-PWM Duty-Cycle */
    sPID_IdCtrl.u32SumErrorMax = (((uint32_t)l_u16MotorRefVoltageADC) << C_GN_PID);   /* Maximal Motor-PWM Duty-Cycle * (1 << C_GN_PID) */

    /* PID Parameters for Iq control */
    sPID_IqCtrl.i16CoefP = (int16_t)(NV_PID_IQ_COEF_P << (C_GN_PID - 12U));     /* Iq PID P-coefficient */
    sPID_IqCtrl.i16CoefI = (int16_t)(NV_PID_IQ_COEF_I << (C_GN_PID - 12U));     /* Iq PID I-coefficient */
#if (_SUPPORT_PID_D_COEF != FALSE)
    sPID_IqCtrl.i16CoefD = (int16_t)(NV_PID_IQ_COEF_D << (C_GN_PID - 12U));     /* Iq PID D-coefficient */
#endif /* (_SUPPORT_PID_D_COEF != FALSE) */
    sPID_IqCtrl.u16MinOutput = 0U;                                              /* Minimal Motor-PWM Duty-Cycle */
    sPID_IqCtrl.u32MaxOutput = (uint32_t)l_u16MotorRefVoltageADC;               /* Maximal Motor-PWM Duty-Cycle */
    sPID_IqCtrl.u32SumErrorMax = (((uint32_t)l_u16MotorRefVoltageADC) << C_GN_PID);   /* Maximal Motor-PWM Duty-Cycle * (1 << C_GN_PID) */

    /* PID Parameters for Speed-Error to Vector Amplitude (PID Control Gain-factor: (2 ^ C_GN_PID)) */
    sPIDpSE2VA.i16CoefP = (int16_t)(NV_PID_COEF_P << (C_GN_PID - 10U));         /* Speed-Error PID P-coefficient */
    sPIDpSE2VA.i16CoefI = (int16_t)(NV_PID_COEF_I << (C_GN_PID - 10U));         /* Speed-Error PID I-coefficient */
#if (_SUPPORT_PID_D_COEF != FALSE)
    sPIDpSE2VA.i16CoefD = (int16_t)(NV_PID_COEF_D << (C_GN_PID - 10U));         /* Speed-Error PID D-coefficient */
#endif /* (_SUPPORT_PID_D_COEF != FALSE) */
    sPIDpSE2VA.u16MinOutput = l_u16ActCurrRunMin_LSB;                           /* Minimal Ipk_Target */
    sPIDpSE2VA.u32MaxOutput = (uint32_t)l_u16ActCurrRunMax_LSB;                 /* Maximal Ipk_Target */
    sPIDpSE2VA.u32SumErrorMax = (((uint32_t)l_u16ActCurrRunMax_LSB) << C_GN_PID);  /* Maximal Ipk_Target * (1 << C_GN_PID) */
#elif ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_IB_IPK)
    /* PID for Ipeak control */
    sPID_IpkCtrl.i16CoefI = (int16_t)(C_PID_IPK_COEF_I << (C_GN_PID - 12U));    /* Ipk PID I-coefficient */
    sPID_IpkCtrl.i16CoefP = (int16_t)(C_PID_IPK_COEF_P << (C_GN_PID - 12U));    /* Ipk PID P-coefficient */
#if (_SUPPORT_PID_D_COEF != FALSE)
    sPID_IpkCtrl.i16CoefD = (int16_t)(C_PID_IPK_COEF_D << (C_GN_PID - 12U));    /* Ipk PID D-coefficient */
#endif /* (_SUPPORT_PID_D_COEF != FALSE) */
    sPID_IpkCtrl.u16MinOutput = l_u16MinCorrectionRatio;                        /* Minimum Motor-PWM Duty-Cycle */
    sPID_IpkCtrl.u32MaxOutput = (uint32_t)l_u16MaxCorrectionRatio;              /* Maximal Motor-PWM Duty-Cycle */
    sPID_IpkCtrl.u32SumErrorMax = (((uint32_t)l_u16MaxCorrectionRatio) << C_GN_PID);   /* Maximal Motor-PWM Duty-Cycle */

    /* PID Parameters for Speed-Error to Vector Amplitude (PID Control Gain-factor: (2 ^ C_GN_PID)) */
    sPIDpSE2VA.i16CoefP = (int16_t)(NV_PID_COEF_P << (C_GN_PID - 10U));         /* Speed-Error PID P-coefficient */
    sPIDpSE2VA.i16CoefI = (int16_t)(NV_PID_COEF_I << (C_GN_PID - 10U));         /* Speed-Error PID I-coefficient */
#if (_SUPPORT_PID_D_COEF != FALSE)
    sPIDpSE2VA.i16CoefD = (int16_t)(NV_PID_COEF_D << (C_GN_PID - 10U));         /* Speed-Error PID D-coefficient */
#endif /* (_SUPPORT_PID_D_COEF != FALSE) */
    sPIDpSE2VA.u16MinOutput = l_u16ActCurrRunMin_LSB;                           /* Minimal Ipk_Target */
    sPIDpSE2VA.u32MaxOutput = (uint32_t)l_u16ActCurrRunMax_LSB;                 /* Maximal Ipk_Target */
    sPIDpSE2VA.u32SumErrorMax = (((uint32_t)l_u16ActCurrRunMax_LSB) << C_GN_PID);  /* Maximal Ipk_Target * (1 << C_GN_PID) */
#else  /* (_SUPPORT_FOC_MODE & FOC_MODE_MASK) */
    /* PID Parameters for Speed-Error to Vector Amplitude (PID Control Gain-factor: (2 ^ C_GN_PID)) */
    sPIDpSE2VA.i16CoefP = (int16_t)(NV_PID_COEF_P << (C_GN_PID - 9U));         /* Speed-Error PID P-coefficient (MMP240524-2) */
    sPIDpSE2VA.i16CoefI = (int16_t)(NV_PID_COEF_I << (C_GN_PID - 9U));         /* Speed-Error PID I-coefficient (MMP240524-2) */
#if (_SUPPORT_PID_D_COEF != FALSE)
    sPIDpSE2VA.i16CoefD = (int16_t)(NV_PID_COEF_D << (C_GN_PID - 9U));         /* Speed-Error PID D-coefficient (MMP240524-2) */
#endif /* (_SUPPORT_PID_D_COEF != FALSE) */
    sPIDpSE2VA.u16MinOutput = l_u16MinCorrectionRatio;                          /* Minimal Motor-PWM Duty-Cycle */
    sPIDpSE2VA.u32MaxOutput = (uint32_t)l_u16MaxCorrectionRatio;                /* Maximal Motor-PWM Duty-Cycle */
    sPIDpSE2VA.u32SumErrorMax = (((uint32_t)l_u16MaxCorrectionRatio) << C_GN_PID);  /* Maximal Motor-PWM Duty-Cycle * (1 << C_GN_PID) */
#endif /* (_SUPPORT_FOC_MODE & FOC_MODE_MASK) */
#else  /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) */
    /* PID Parameters for Speed-Error to Vector Amplitude (PID Control Gain-factor: (2 ^ C_GN_PID)) */
    sPIDpSE2VA.i16CoefP = (int16_t)(NV_PID_COEF_P << (C_GN_PID - 10U));         /* Speed-Error PID P-coefficient (MMP240524-2) */
    sPIDpSE2VA.i16CoefI = (int16_t)(NV_PID_COEF_I << (C_GN_PID - 10U));         /* Speed-Error PID I-coefficient (MMP240524-2) */
#if (_SUPPORT_PID_D_COEF != FALSE)
    sPIDpSE2VA.i16CoefD = (int16_t)(NV_PID_COEF_D << (C_GN_PID - 10U));         /* Speed-Error PID D-coefficient (MMP240524-2) */
#endif /* (_SUPPORT_PID_D_COEF != FALSE) */
    sPIDpSE2VA.u16MinOutput = l_u16MinCorrectionRatio;                          /* Minimal Motor-PWM Duty-Cycle */
#if (PWM_LIMIT == PWM_LIMIT_BY_PID)
    sPIDpSE2VA.u32MaxOutput = (uint32_t)l_u16MaxCorrectionRatio;                /* Maximal Motor-PWM Duty-Cycle */
    sPIDpSE2VA.u32SumErrorMax = (((uint32_t)l_u16MaxCorrectionRatio) << C_GN_PID);  /* Maximal Motor-PWM Duty-Cycle * (1 << C_GN_PID) */
#else  /* (PWM_LIMIT == PWM_LIMIT_BY_PID) */
    sPIDpSE2VA.u32MaxOutput = (uint32_t)l_u16MaxCorrectionRatio;                /* Maximal Motor-PWM Duty-Cycle */
    sPIDpSE2VA.u32SumErrorMax = (((uint32_t)l_u16MaxCorrectionRatio) << C_GN_PID);  /* Maximal Motor-PWM Duty-Cycle * (1 << C_GN_PID) */
#endif /* (PWM_LIMIT == PWM_LIMIT_BY_PID) */
#endif /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) */

#if (_SUPPORT_POTI != FALSE) && (_SUPPORT_POTI_MODE == C_POTI_MODE_CLOSED_LOOP)
    /* Potentiometer 360-degrees = 8 * (C_POS_TRAVEL * SHAFT_STEPS_PER_ROTATION)/1024 */
    {
        uint16_t u16Temp =
            (uint16_t)p_MulDivU16_U16byU16byU16( ((C_SHAFT_STEPS_PER_ROTATION * C_POS_TRAVEL) / (1024 >> 2)),
                                                 NV_PID_RUNNINGCTRL_PER,
                                                 NV_GEARBOX_RATIO);
        l_au16PID_SpeedRef[0] = p_MulDivU16_U16byU16byU16(u16Temp, g_u16MinSpeedRPM, 60000U);
        l_au16PID_SpeedRef[1] = p_MulDivU16_U16byU16byU16(u16Temp, g_u16LowSpeedRPM, 60000U);
        l_au16PID_SpeedRef[2] = p_MulDivU16_U16byU16byU16(u16Temp, NV_ACT_SPEED2, 60000U);
        l_au16PID_SpeedRef[3] = p_MulDivU16_U16byU16byU16(u16Temp, NV_ACT_SPEED3, 60000U);
        l_au16PID_SpeedRef[4] = p_MulDivU16_U16byU16byU16(u16Temp, g_u16MaxSpeedRPM, 60000U);
        l_au16PID_SpeedRef[5] = l_au16PID_SpeedRef[1];
        l_au16PID_SpeedRef[6] = l_au16PID_SpeedRef[0];
        l_au16PID_SpeedRef[7] = l_au16PID_SpeedRef[0];
    }
#endif /* (_SUPPORT_POTI != FALSE) && (_SUPPORT_POTI_MODE == C_POTI_MODE_CLOSED_LOOP) */
} /* End of PID_Init() */

/*!*************************************************************************** *
 * PID_Start()
 * \brief   Initialise the PID
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16Losses: Ohmic losses part (current & coil impedance)
 * \param   [in] u16Bemf: BEMF voltage part (speed & motor constant)
 * \return  (uint16_t) PWM Duty Cycle amplitude (u16CorrectionRatio)
 * *************************************************************************** *
 * \details -
 * Note:    This function is not allowed to be called during Non Volatile Memory Write
 *          (due to Non Volatile Memory reads).
 * *************************************************************************** *
 * - Call Hierarchy: MotorDriver_InitialPwmDutyCycle()
 * - Cyclomatic Complexity: 3+1
 * - Nesting: 2
 * - Function calling: 0 (all inline)
 * *************************************************************************** */
uint16_t PID_Start(uint16_t u16Losses, uint16_t u16Bemf)
{
    uint16_t u16MotorVoltage = Get_MotorVoltage();
    uint16_t u16ReferenceVoltage = NV_VSUP_REF;
    uint16_t u16CorrectionRatio;

    if (u16ReferenceVoltage == 0U)
    {
        u16ReferenceVoltage = C_REF_VOLTAGE;                                    /* Default reference voltage in [10mV]-units (MMP230912-1) */
    }
#if ((_SUPPORT_FOC_MODE != FOC_MODE_NONE) && ((_SUPPORT_CLOSED_LOOP_STARTUP == FALSE) || \
    (_SUPPORT_TRIAXIS_MLX9038x != FALSE))) || \
    ((_SUPPORT_FOC_MODE == FOC_MODE_NONE) && ((_SUPPORT_HALL_LATCH == FALSE) || (_SUPPORT_NR_OF_HL == 1) || (_SUPPORT_HALL_LATCH_DIAG != FALSE)))
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__)
    l_u16PidLosses =
        p_MulDivU16_U16byU16byU16(u16Losses, (PWM_REG_PERIOD << C_PID_FACTOR),
                                  u16ReferenceVoltage);
#if (_SUPPORT_PWM_MODE != SINGLE_COIL_PWM_BIPOLAR) && ((_SUPPORT_PWM_MODE != SINGLE_COIL_PWM) || (C_NR_OF_DC_MOTORS != 1))
    if (l_u16PidLosses < ((2U * C_PWM_MIN_DC) << C_PID_FACTOR))                 /* MMP240726-1: Check minimum PID-losses is twice minimum PWM Duty-cycle */
    {
        l_u16PidLosses = ((2U * C_PWM_MIN_DC) << C_PID_FACTOR);
    }
#endif /* (_SUPPORT_PWM_MODE != SINGLE_COIL_PWM_BIPOLAR) && ((_SUPPORT_PWM_MODE != SINGLE_COIL_PWM) || (C_NR_OF_DC_MOTORS != 1)) */
#elif defined (__MLX81160__) || defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
    l_u16PidLosses = p_MulDivU16_U16byU16byU16(u16Losses, (PWM_REG_PERIOD << C_PID_FACTOR), u16ReferenceVoltage);
#endif
    l_u16PidCtrlRatio = l_u16PidLosses + p_MulDivU16_U16byU16byU16(u16Bemf,
                                                                   (PWM_REG_PERIOD << C_PID_FACTOR),
                                                                   u16ReferenceVoltage);
#else  /* (_SUPPORT_CLOSED_LOOP_STARTUP == FALSE) || (_SUPPORT_HALL_LATCH == FALSE) || (_SUPPORT_NR_OF_HL == 1) || (_SUPPORT_HALL_LATCH_DIAG != FALSE) */
    {
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__)
        uint16_t u16PidLosses = p_MulDivU16_U16byU16byU16(u16Losses,
                                                          (PWM_REG_PERIOD << C_PID_FACTOR),
                                                          u16ReferenceVoltage) + ((2U * C_PWM_MIN_DC) << C_PID_FACTOR);
#elif defined (__MLX81160__) || defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
        uint16_t u16PidLosses = p_MulDivU16_U16byU16byU16(u16Losses,
                                                          (PWM_REG_PERIOD << C_PID_FACTOR),
                                                          u16ReferenceVoltage);
#endif
        l_u16PidCtrlRatio = u16PidLosses + p_MulDivU16_U16byU16byU16(u16Bemf,
                                                                     (PWM_REG_PERIOD << C_PID_FACTOR),
                                                                     u16ReferenceVoltage);
    }
#endif /* (_SUPPORT_CLOSED_LOOP_STARTUP == FALSE) || (_SUPPORT_HALL_LATCH == FALSE) || (_SUPPORT_NR_OF_HL == 1) || (_SUPPORT_HALL_LATCH_DIAG != FALSE) */
    if (l_u16PidCtrlRatio > l_u16MaxCorrectionRatio)
    {
        /* Overflow */
        l_u16PidCtrlRatio = l_u16MaxCorrectionRatio;
    }
#if (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM)
    else if (l_u16PidCtrlRatio < l_u16SenseCorrectionRatio)
    {
        l_u16PidCtrlRatio = l_u16SenseCorrectionRatio;
    }
#endif /* (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM) */

#if ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_IB_IPK)
    /* l_u16TargetIpk = l_u16ActCurrRunMax_LSB; */                              /* Maximum start-up acceleration; Huge overshoot for low-speed */
    l_u16TargetIpk = l_u16ActCurrStartMax_LSB;
    sPIDpSE2VA.u32SumError = (((uint32_t)l_u16TargetIpk) << C_GN_PID);
    sPID_IpkCtrl.u32SumError = (((uint32_t)l_u16PidCtrlRatio) << C_GN_PID);
#elif ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_ID_IQ)
    sPIDpSE2VA.u32SumError = (((uint32_t)l_u16TargetIpk) << C_GN_PID);
#else  /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) && ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_ID_IQ) || ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_IB_IPK) */
    sPIDpSE2VA.u32SumError = (((uint32_t)l_u16PidCtrlRatio) << C_GN_PID);
#endif /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) && ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_ID_IQ) || ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_IB_IPK) */
#if (_SUPPORT_PID_D_COEF != FALSE)
    sPIDpSE2VA.i16PrevError = 0;
#endif /* (_SUPPORT_PID_D_COEF != FALSE) */
    l_u16PID_CtrlCounter = 0U;                                                  /* Re-start Current-control PID */

    if (u16MotorVoltage != 0U)
    {
        u16CorrectionRatio = p_MulDivU16_U16byU16byU16(l_u16PidCtrlRatio, u16ReferenceVoltage, u16MotorVoltage);
        if (u16CorrectionRatio > l_u16MaxCorrectionRatio)
        {
            /* Overflow */
            u16CorrectionRatio = l_u16MaxCorrectionRatio;
        }
    }
    else
    {
        u16CorrectionRatio = l_u16PidCtrlRatio;
    }

#if (_SUPPORT_POTI != FALSE) && (_SUPPORT_POTI_MODE == C_POTI_MODE_CLOSED_LOOP)
#if (_SUPPORT_MOTOR_POSITION == C_MOTOR_POS_ROTOR)
    l_u16PID_LastPotiPos = g_u16ActualPosition;
#elif (_SUPPORT_MOTOR_POSITION == C_MOTOR_POS_SHAFT)
    l_u16PID_LastPotiPos = g_u16ActualShaftAngle;
#endif
#endif /* (_SUPPORT_POTI != FALSE) && (_SUPPORT_POTI_MODE == C_POTI_MODE_CLOSED_LOOP) */

#if (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM_BIPOLAR)
    if (NV_PID_RUNNINGCTRL_PER_UNIT == C_PID_PERIOD_UNIT_TIME)
    {
        l_u16PID_Period = (NV_PID_RUNNINGCTRL_PER * PI_TICKS_PER_MILLISECOND) * 2U;
    }
    else
    {
        l_u16PID_Period = (Get_CommutTimerPeriod() >> 2);
    }
#endif /* (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM_BIPOLAR) */

    return (u16CorrectionRatio);
} /* End of PID_Start() */

#if (_SUPPORT_FOC_MODE != FOC_MODE_NONE) && _SUPPORT_WINDMILL && (C_MOTOR_COILS == 3U)
/*!*************************************************************************** *
 * PID_WindmillStart
 * \brief   PID for Windmilling
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16ActualMotorSpeed: Actual speed
 * \return  -
 * *************************************************************************** *
 * \details
 * *************************************************************************** *
 * - Call Hierarchy: MotorDriver_InitialPwmDutyCycle()
 * - Cyclomatic Complexity: 3+1
 * - Nesting: 2
 * - Function calling: 0
 * *************************************************************************** */
void PID_WindmillStart(uint16_t u16ActualMotorSpeed)
{
    l_u16PidCtrlRatio = p_MulDivU16_U16byU16byU16(u16ActualMotorSpeed,
                                                  (PWM_REG_PERIOD << C_PID_FACTOR),
                                                  g_u16MaxSpeedRPM);
#if 0
    /* Compensate for ohmic losses (Linear interpolation) */
    uint16_t u16CurrentLevel = p_MulDivU16_U16byU16byU16(NVRAM_RUNNING_CURR_MAX,
                                                         u16ActualMotorSpeed,
                                                         g_u16MaxSpeedRPM);
    l_u16PidCtrlRatio += p_MulDivU16_U16byU16byU16( (NVRAM_MOTOR_COIL_RTOT + 2U * C_FETS_RTOT),
                                                    u16CurrentLevel, (2000U >> (8U - C_PID_FACTOR)));
#endif
    if (l_u16PidCtrlRatio >= l_u16MaxCorrectionRatio)
    {
        /* Overflow */
        l_u16PidCtrlRatio = l_u16MaxCorrectionRatio;
    }
#if ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_ID_IQ) || ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_IB_IPK)
    sPIDpSE2VA.u32SumError = (((uint32_t)l_u16TargetIpk) << C_GN_PID);
#else  /* ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_ID_IQ) || ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_IB_IPK) */
    sPIDpSE2VA.u32SumError = (((uint32_t)l_u16PidCtrlRatio) << C_GN_PID);
#endif /* ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_ID_IQ) || ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_IB_IPK) */
#if (_SUPPORT_PID_D_COEF != FALSE)
    sPIDpSE2VA.i16PrevError = 0;
#endif /* (_SUPPORT_PID_D_COEF != FALSE) */
    l_u16PID_CtrlCounter = 0U;                                                  /* Re-start Speed-control PID */
    /* PID: I-mem = Actual-speed * PP * 256/Coef-I --> (I-mem * Coef-I) / 256 = Actual-speed (electric) */
    sPIDpLA2AS.u32SumError = (p_MulU32_U16byU16(u16ActualMotorSpeed, g_u16MotorPolePairs) << C_GN_PID);
#if (_SUPPORT_PID_D_COEF != FALSE)
    sPIDpLA2AS.i16PrevError = 0;
#endif /* (_SUPPORT_PID_D_COEF != FALSE) */
} /* End of PID_WindmillStart() */
#endif /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) */

#if ((_SUPPORT_FOC_MODE != FOC_MODE_NONE) && ((_SUPPORT_CLOSED_LOOP_STARTUP == FALSE) || \
    (_SUPPORT_TRIAXIS_MLX9038x != FALSE))) || \
    ((_SUPPORT_FOC_MODE == FOC_MODE_NONE) && ((_SUPPORT_HALL_LATCH == FALSE) || (_SUPPORT_NR_OF_HL == 1) || (_SUPPORT_HALL_LATCH_DIAG != FALSE)))
/*!*************************************************************************** *
 * PID_SpeedCompensate
 * \brief   PID Compensation for speed
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16NewSpeed: New Speed (RPM)
 * \param   [in] u16OldSpeed: Old Speed (RPM)
 * \return  -
 * *************************************************************************** *
 * \details -
 * *************************************************************************** *
 * - Call Hierarchy: MotorDriverMicroStepUpdate()
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 0 (all inline)
 * *************************************************************************** */
void PID_SpeedCompensate(uint16_t u16NewSpeed, uint16_t u16OldSpeed)
{
    if (l_u16PidCtrlRatio > l_u16PidLosses)                                     /* MMP210929-1: Avoid PID "overflow" on below calculation. */
    {
        l_u16PidCtrlRatio = l_u16PidLosses + p_MulDivU16_U16byU16byU16( (l_u16PidCtrlRatio - l_u16PidLosses),
                                                                        u16NewSpeed,
                                                                        u16OldSpeed);
    }
    else
    {
        l_u16PidCtrlRatio = l_u16PidLosses;
    }
#if ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_IB_IPK)
    sPID_IpkCtrl.u32SumError = (((uint32_t)l_u16PidCtrlRatio) << C_GN_PID);
#elif ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_ID_IQ)
#else  /* ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_ID_IQ) || ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_IB_IPK) */
    sPIDpSE2VA.u32SumError = (((uint32_t)l_u16PidCtrlRatio) << C_GN_PID);
#endif /* ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_ID_IQ) || ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_IB_IPK) */
} /* End of PID_SpeedCompensate() */
#endif /* ((_SUPPORT_FOC_MODE != FOC_MODE_NONE) && ((_SUPPORT_CLOSED_LOOP_STARTUP == FALSE) || (_SUPPORT_TRIAXIS_MLX9038x != FALSE))) || \
        * ((_SUPPORT_FOC_MODE == FOC_MODE_NONE) && ((_SUPPORT_HALL_LATCH == FALSE) || (_SUPPORT_NR_OF_HL == 1) || (_SUPPORT_HALL_LATCH_DIAG != FALSE))) */

#if (_SUPPORT_STALLDET_BZC != FALSE)
/*!*************************************************************************** *
 * PID_SwitchStepper2Bemf
 * \brief   Switch from stepper-mode to BEMF-mode
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details
 * *************************************************************************** *
 * - Call Hierarchy: Commutation_ISR()
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 0
 * *************************************************************************** */
void PID_SwitchStepper2Bemf(void)
{
    l_u16PidCtrlRatio >>= 1;
    sPIDpSE2VA.u32SumError = (((uint32_t)l_u16PidCtrlRatio) << C_GN_PID);
} /* End of PID_SwitchStepper2Bemf() */
#endif /* (_SUPPORT_STALLDET_BZC != FALSE) */

#if (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM)
/*!*************************************************************************** *
 * PID_RampUp
 * \brief   PID Ramp-up for DC
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16RampUpPeriod : Ramp-up period
 * \return  (uint16_t) FALSE: Not maximum; TRUE : Maximum level
 * *************************************************************************** *
 * \details
 * *************************************************************************** *
 * - Call Hierarchy: RampUp()
 * - Cyclomatic Complexity: 2+1
 * - Nesting: 2
 * - Function calling: 0 (all inline)
 * *************************************************************************** */
uint16_t PID_RampUp(uint16_t u16RampUpPeriod)
{
    uint16_t u16Result = FALSE;
    if ( (u16RampUpPeriod != 0U) && (g_u16ActualMotorSpeedRPM < g_u16TargetMotorSpeedRPM) )
    {
        uint16_t u16DeltaCorrection = p_DivU16_U32byU16( (uint32_t)(l_u16MaxCorrectionRatio - l_u16PidCtrlRatio),
                                                         u16RampUpPeriod);
        l_u16PidCtrlRatio += u16DeltaCorrection;
        if (l_u16PidCtrlRatio >= l_u16MaxCorrectionRatio)
        {
            l_u16PidCtrlRatio = l_u16MaxCorrectionRatio;
            u16Result = TRUE;                                                   /* Max PWM DC */
        }
        sPIDpSE2VA.u32SumError = (((uint32_t)l_u16PidCtrlRatio) << C_GN_PID);
    }
    else
    {
        u16Result = TRUE;                                                       /* Ramp done */
    }
    return (u16Result);
} /* End of PID_RampUp() */

/*!*************************************************************************** *
 * PID_RampDown
 * \brief   PID Ramp-Down for DC
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16RampDownPeriod: Ramp-down period
 * \return  (uint16_t) FALSE: Not maximum; TRUE : Maximum level
 * *************************************************************************** *
 * \details
 * *************************************************************************** *
 * - Call Hierarchy: RampDown()
 * - Cyclomatic Complexity: 2+1
 * - Nesting: 2
 * - -Function calling: 0 (all inline)
 * *************************************************************************** */
uint16_t PID_RampDown(uint16_t u16RampDownPeriod)
{
    uint16_t u16Result = FALSE;
    if (u16RampDownPeriod != 0U)
    {
        uint16_t u16DeltaCorrection = p_DivU16_U32byU16( (uint32_t)(l_u16PidCtrlRatio - l_u16SenseCorrectionRatio),
                                                         u16RampDownPeriod);
        l_u16PidCtrlRatio -= u16DeltaCorrection;
        if ( (l_u16PidCtrlRatio <= l_u16SenseCorrectionRatio) || (u16RampDownPeriod == 1U) )
        {
            /* Truncate PWM duty-cycle till minimum duty-cycle; Ramp-down period is done */
            l_u16PidCtrlRatio = l_u16SenseCorrectionRatio;
            u16Result = TRUE;
        }
    }
    else
    {
        l_u16PidCtrlRatio = l_u16SenseCorrectionRatio;
        u16Result = TRUE;
    }
    sPIDpSE2VA.u32SumError = (((uint32_t)l_u16PidCtrlRatio) << C_GN_PID);
    return (u16Result);
} /* End of PID_RampDown() */
#endif /* (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM) */

#if ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) != FOC_MODE_ID_IQ)                     /* MMP230802-2 */
/*!*************************************************************************** *
 * VoltageCorrection
 * \brief   Compensate Motor PWM Duty Cycle for voltage changes
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details Tuned for MLX-GNU V3.0.31 (MMP181206)
 * Note:    This function is not allowed to be called during Non Volatile Memory Write
 *          (due to Non Volatile Memory reads) (C_APP_POSITIONING_ACT).
 * *************************************************************************** *
 * - Call Hierarchy: ISR_CTIME0_3(), PID_Control()
 * - Cyclomatic Complexity: 5+1
 * - Nesting: 4
 * - Function calling: 2 (Get_RawVmotorF(), Set_CorrectionRatio())
 * *************************************************************************** */
#if (_SUPPORT_FOC_MODE != FOC_MODE_NONE) && (_SUPPORT_RAM_FUNC != FALSE)
uint16_t VoltageCorrection(void) __attribute__ ((section(".ramfunc"))) __attribute__ ((aligned(8)));
#elif (_SUPPORT_FOC_MODE != FOC_MODE_NONE) && (_SUPPORT_OPTIMIZE_FOR_SPEED != FALSE)
uint16_t VoltageCorrection(void) __attribute__((aligned(8)));
#endif /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) && _SUPPORT_OPTIMIZE_FOR_SPEED */
uint16_t VoltageCorrection(void)
{
    register uint16_t u16NewCorrectionRatio = l_u16PidCtrlRatio;
    register uint16_t u16MotorVoltageADC = Get_RawVmotorF();
    if ( (u16MotorVoltageADC > 0U) && (l_u16MotorRefVoltageADC > 0U) )
    {
        /* Correct Motor PWM duty cycle instantly based on change of supply voltage */
        u16NewCorrectionRatio = p_MulDivU16_U16byU16byU16(l_u16MotorRefVoltageADC,
                                                          u16NewCorrectionRatio,
                                                          u16MotorVoltageADC);
#if (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT)
        if ( (g_e8MotorStatus & C_MOTOR_STATUS_STOP_MASK) != C_MOTOR_STATUS_STOP)
#endif /* (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) */
        {
            /* Running-mode */
            if (u16NewCorrectionRatio > l_u16MaxCorrectionRatio)
            {
                /* Overflow */
                u16NewCorrectionRatio = l_u16MaxCorrectionRatio;
            }
            else if (u16NewCorrectionRatio < l_u16MinCorrectionRatio)
            {
                /* Underflow */
                u16NewCorrectionRatio = l_u16MinCorrectionRatio;
            }
            else
            {
                /* Nothing */
            }
        }
#if (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT)
        else if (u16NewCorrectionRatio < NV_MIN_HOLDCORR_RATIO)
        {
            /* Holding-mode: Underflow */
            u16NewCorrectionRatio = NV_MIN_HOLDCORR_RATIO;
        }
        else
        {
            /* Nothing */
        }
#endif /* (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) */
    }
    return (u16NewCorrectionRatio);
} /* End of VoltageCorrection() */
#endif /* ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) != FOC_MODE_ID_IQ) */

/*!*************************************************************************** *
 * PID_Control_Period
 * \brief   determine the PID period
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  (uint16_t) u16PidPeriod: PID period
 * *************************************************************************** *
 * \details -
 * *************************************************************************** *
 * - Call Hierarchy: PID_Control()
 * - Cyclomatic Complexity: 2+1
 * - Nesting: 2
 * - Function calling: 0
 * *************************************************************************** */
static uint16_t PID_Control_Period(void)
{
    uint16_t u16PidPeriod = 0U;
#if (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM)
    u16PidPeriod = NV_PID_RUNNINGCTRL_PER;
#elif (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM_BIPOLAR)
    if (g_e8MotorStatus != C_MOTOR_STATUS_ALIGNMENT)
    {
        u16PidPeriod = l_u16PID_Period;
    }
    else
    {
        /* During Alignment, don't compensate for PID */
        u16PidPeriod = 0U;
    }
#else
#if (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT)
    if ( (g_e8MotorStatus & C_MOTOR_STATUS_STOP_MASK) == C_MOTOR_STATUS_STOP)   /* Running mode */
    {
        /* Stop-mode & holding-current required */
        u16PidPeriod = (NV_PID_HOLDINGCTRL_PER * PI_TICKS_PER_MILLISECOND);     /* MMP240709-2 */
    }
    else
#endif /* (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) */
    if ( (NV_PID_RUNNINGCTRL_PER_UNIT == C_PID_PERIOD_UNIT_TIME)
#if (_SUPPORT_FOC_MODE != FOC_MODE_NONE)
         || ((l_e8MotorStartupMode & E_MSM_MODE_MASK) == E_MSM_STEPPER)
#endif /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) */
         )
    {
        /* Time based period */
        u16PidPeriod = (NV_PID_RUNNINGCTRL_PER * PI_TICKS_PER_MILLISECOND);
#if (_SUPPORT_FOC_MODE != FOC_MODE_NONE) && (_SUPPORT_CLOSED_LOOP_STARTUP == FALSE)
        /* Period is for Open-Loop; Close-Loop use 8-times faster */
        if ( (l_e8MotorStartupMode & E_MSM_MODE_MASK) != E_MSM_STEPPER)
        {
            u16PidPeriod = (u16PidPeriod + 7U) >> 3;
        }
#endif /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) && (_SUPPORT_CLOSED_LOOP_STARTUP == FALSE) */
#if (_SUPPORT_CDI != FALSE)
        /* HVAC FAN needed? WP not */
        if ( (l_e8MotorStartupMode & E_MSM_CDI) != 0U)
        {
            /* Double the period in CDI-mode */
            u16PidPeriod *= 2U;
        }
#endif /* (_SUPPORT_CDI != FALSE) */
    }
    else
    {
        /* Speed based period; NV_PID_RUNNINGCTRL_PER based on 1/128th (>> 7) */
#if (_SUPPORT_CDI != FALSE) || (_SUPPORT_FOC_MODE != FOC_MODE_NONE)
        u16PidPeriod = (uint16_t) (p_MulU32_U16byU16(g_u16ActualCommutTimerPeriod, NV_PID_RUNNINGCTRL_PER) >> 7U) + 1U;
#else  /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) */
        u16PidPeriod = (uint16_t) (p_MulU32_U16byU16(Get_CommutTimerPeriod(), NV_PID_RUNNINGCTRL_PER) >> 7U) + 1U;
#endif /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) */
    }
#endif /* (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM) */
    return (u16PidPeriod);
} /* End of PID_Control_Period() */

/*!*************************************************************************** *
 * PID_Control_Error
 * \brief   Calculate the PID error-input
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  (int16_t) i16ControlError: PID error input
 * *************************************************************************** *
 * \details (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM): DC-motor
 *            (_SUPPORT_POTI != FALSE) && (_SUPPORT_POTI_MODE == C_POTI_MODE_CLOSED_LOOP)
 *              The PID input error is based on the rotor/outer-shaft position change per time-period (speed-error).
 *            (_SUPPORT_CURRSPIKE_POSITION != FALSE)
 *              The PID input error is based on number of current-spikes/fluctuations per time-period (speed-error)
 *          (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM_BIPOLAR): Single Coil fan/Pump
 *              The PID input error is based on actual-speed versus target-speed and actual-current versus maximum-current (speed-error * "current")
 *              -or- voltage level.
 *          (_SUPPORT_FOC_MODE != FOC_MODE_NONE): FOC-based:
 *            Close-loop:
 *              The PID input error is based on actual-speed versus target-speed and actual-current versus maximum-current (speed-error * "current")
 *            Open-Loop:
 *              The PID input error is based on current-control (= torque control).
 * Note:
 * *************************************************************************** *
 * - Call Hierarchy: PID_Control()
 * - Cyclomatic Complexity: 11+1
 * - Nesting: 3
 * - Function calling: 0
 * *************************************************************************** */
static int16_t PID_Control_Error(void)
{
    int16_t i16ControlError = 0;

#if (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM)
#if (_SUPPORT_POTI != FALSE) && (_SUPPORT_POTI_MODE == C_POTI_MODE_CLOSED_LOOP)
    /* Movement during last PID-period */
#if (_SUPPORT_MOTOR_POSITION == C_MOTOR_POS_ROTOR)
    if (g_e8MotorDirectionCCW != FALSE)
    {
        i16ControlError = (int16_t)(l_u16PID_LastPotiPos - g_u16ActualPosition);
    }
    else
    {
        i16ControlError = (int16_t)(g_u16ActualPosition - l_u16PID_LastPotiPos);
    }
    l_u16PID_LastPotiPos = g_u16ActualPosition;
#elif (_SUPPORT_MOTOR_POSITION == C_MOTOR_POS_SHAFT)
    if (g_e8MotorDirectionCCW != FALSE)
    {
        i16ControlError = (int16_t)(l_u16PID_LastPotiPos - g_u16ActualShaftAngle);
    }
    else
    {
        i16ControlError = (int16_t)(g_u16ActualShaftAngle - l_u16PID_LastPotiPos);
    }
    l_u16PID_LastPotiPos = g_u16ActualShaftAngle;
#endif
    if (i16ControlError < 0)
    {
        i16ControlError = 0;
    }
    i16ControlError = (int16_t)(p_MulI32_I16byI16(i16ControlError, (int16_t)Get_ShaftRatiox512()) >> 9);
    i16ControlError = (int16_t)(l_au16PID_SpeedRef[g_u8MotorStatusSpeed] - i16ControlError) * 128;
#endif /* (_SUPPORT_POTI != FALSE) && (_SUPPORT_POTI_MODE == C_POTI_MODE_CLOSED_LOOP) */

#if (_SUPPORT_CURRSPIKE_POSITION != FALSE)
    if ( (g_u16SpikeCount != 0U) && (g_u16SpikePeriodSum != 0U) )
    {
        ENTER_SECTION(ATOMIC_KEEP_MODE);  /*lint !e534 */
        {
            l_u16PidSpikePeriodSum = g_u16SpikePeriodSum;
            l_u16PidSpikeCount = g_u16SpikeCount;
            g_u16SpikePeriodSum = 0U;
            g_u16SpikeCount = 0U;
        }
        EXIT_SECTION(); /*lint !e438 */
#if FALSE
        /* RPM = seconds/minute * PWM-frequency * count / period * commutation-periods * 2 * coils =
         *        60 = (15 * 4) *    PWM_FREQ   * count / period *           2         * 2 *   3   */
        uint16_t u16Helper = p_DivU16_U32byU16( (15UL * PWM_FREQ), (2U * g_u16MotorPolePairs));
        Set_ActualMotorSpeedRPM(p_MulDivU16_U16byU16byU16( (l_u16PidSpikeCount << 2), u16Helper,
                                                           l_u16PidSpikePeriodSum));
#endif
        i16ControlError = (int16_t)(g_u16TargetMotorSpeedRPM - g_u16ActualMotorSpeedRPM);
    }
#else  /* (_SUPPORT_CURRSPIKE_POSITION != FALSE) */
    /* Speed control */
    i16ControlError = (int16_t)(g_u16TargetMotorSpeedRPM - g_u16ActualMotorSpeedRPM);   /* Speed correction is more stable for low-speeds */
#endif /* (_SUPPORT_CURRSPIKE_POSITION != FALSE) */
#elif (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM_BIPOLAR)
#if (_SUPPORT_SPEED_CTRL != FALSE)
#if (_SUPPORT_SPEED_CTRL != FALSE) && (_SUPPORT_VOLTAGE_CTRL != FALSE)
    if (g_e8ControlType == (uint8_t)C_SPEED_CTRL)
#endif /* (_SUPPORT_SPEED_CTRL != FALSE) && (_SUPPORT_VOLTAGE_CTRL != FALSE) */
    {
        /* Speed control */
        int16_t i16ErrorI, i16ErrorS, i16Divisor;

        i16ErrorS = (int16_t)(g_u16TargetMotorSpeedRPM - g_u16ActualMotorSpeedRPM);
#if (_SUPPORT_PID_SPEED_LIMIT != FALSE)
        if (i16ErrorS < 0)
        {
            /* Ramp-down limitation */
            if (i16ErrorS < (int16_t)(0 - l_u16SpeedRampDownLimit) )
            {
                i16ErrorS = (int16_t)(0 - l_u16SpeedRampDownLimit);
            }
        }
        else
        {
            l_u16SpeedRampUpLimit = (g_u16ActualMotorSpeedRPM >> 2U);
            if (i16ErrorS > (int16_t)l_u16SpeedRampUpLimit)
            {
                i16ErrorS = (int16_t)l_u16SpeedRampUpLimit;
            }
        }
#endif /* (_SUPPORT_PID_SPEED_LIMIT != FALSE) */
        if (i16ErrorS >= 0)
        {
            /* Ramp-up: Limit up to maximum current defined by 'l_u16ActCurrRunMax_LSB' */
            if (g_e8MotorStatus != C_MOTOR_STATUS_ALIGNMENT)
            {
                i16ErrorI = (int16_t)(l_u16ActCurrRunMax_LSB - Get_MotorCurrentLPF());
            }
            else
            {
                i16ErrorI = (int16_t)(l_u16ActCurrAlign_LSB - Get_MotorCurrentLPF());
            }
        }
        else
        {
            /* Ramp-down: Limit up to minimum current defined by 'l_u16ActCurrRunMin_LSB' */
            i16ErrorI = (int16_t)(Get_MotorCurrentLPF() - l_u16ActCurrRunMin_LSB);
        }
        if (i16ErrorI < 0)
        {
            i16ErrorI = 0;
        }

        i16Divisor = (int16_t)l_u16ActCurrRunMax_LSB;
        i16ControlError = p_MulDivI16_I16byI16byI16(i16ErrorI, i16ErrorS, i16Divisor);
    }
#endif /* (_SUPPORT_SPEED_CTRL != FALSE) */
#if (_SUPPORT_VOLTAGE_CTRL != FALSE)
#if (_SUPPORT_SPEED_CTRL != FALSE) && (_SUPPORT_VOLTAGE_CTRL != FALSE)
    else
#endif /* (_SUPPORT_SPEED_CTRL != FALSE) && (_SUPPORT_VOLTAGE_CTRL != FALSE) */
    {
        /* Voltage control */
        g_u16ActualMotorVoltage = p_MulDivU16_U16byU16byU16(Get_MotorVoltage(),
                                                            l_u16CorrectionRatio,
                                                            l_u16MaxCorrectionRatio);
        /* Limit to maximum running current */
        /* i16ControlError = (int16_t) (l_u16ActCurrRunMax_LSB - Get_MotorCurrentLPF());
         * if ( i16ControlError > 0 ) */
        {
            /* Below max. current: Constant Voltage */
            i16ControlError = (int16_t)(g_u16TargetMotorVoltage - l_u16ActualMotorVoltage);
        }
    }
#endif /* (_SUPPORT_VOLTAGE_CTRL != FALSE) */
#else  /* (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM) */
#if (_SUPPORT_STALLDET_BZC != FALSE)
    /* BEMF: Speed-control */
    if ( (Get_MotorStartupMode() & E_MSM_BEMF) != 0U)
    {
        /* BEMF-mode: Speed control */
        i16ControlError = (int16_t)(g_u16TargetMotorSpeedRPM - g_u16ActualMotorSpeedRPM);   /* Speed correction is more stable for low-speeds */
    }
    else
#endif /* (_SUPPORT_STALLDET_BZC != FALSE) */
#if (_SUPPORT_HALL_LATCH != FALSE) && (_SUPPORT_NR_OF_HL > 1) && (_SUPPORT_HALL_LATCH_DIAG == FALSE)
    /* Hall Sensor-based: Speed-control */
#if (_SUPPORT_PID_SPEED_CURRENT != FALSE)
    if ( (g_e8MotorStatus & C_MOTOR_STATUS_RUNNING) != 0U)
    {
        /* Speed Control (Running-mode) */
        int16_t i16ErrorI, i16ErrorS, i16Divisor;
        if (Get_NrOfCommut() < 6U)
        {
            /* No movement in expected time; Increase Motor PWM DC */
            i16ErrorS = (int16_t)g_u16TargetMotorSpeedRPM;
        }
        else
        {
            /* Speed Control (at least 6 commutations) */
            i16ErrorS = (int16_t)(g_u16TargetMotorSpeedRPM - g_u16ActualMotorSpeedRPM);   /* Speed correction is more stable for low-speeds */
        }

#if (_SUPPORT_BRAKING != FALSE)                                                 /* MMP230811-1 */
        if ( (g_e8MotorStatus & C_MOTOR_STATUS_STOP_MASK) == C_MOTOR_STATUS_BRAKING)
        {
            /* Invert the error (Too slow: Reduce breaking current) */
            i16ErrorS = -i16ErrorS;
        }
#endif /* (_SUPPORT_BRAKING != FALSE) */

        /* Add Current limitation (MMP230719-1) */
        if (i16ErrorS >= 0)
        {
            /* Ramp-up: Limit up to maximum current defined by 'l_u16ActCurrRunMax_LSB' */
            i16ErrorI = (int16_t)(l_u16ActCurrRunMax_LSB - Get_MotorCurrentLPF());
        }
        else
        {
            /* Ramp-down: Limit up to minimum current defined by 'l_u16ActCurrRunMin_LSB' */
            i16ErrorI = (int16_t)(Get_MotorCurrentLPF() - l_u16ActCurrRunMin_LSB);
        }
        if (i16ErrorI < 0)
        {
            i16ErrorI = 0;
        }

        i16Divisor = (int16_t)l_u16ActCurrRunMax_LSB;
        i16ControlError = p_MulDivI16_I16byI16byI16(i16ErrorI, i16ErrorS, i16Divisor);
    }
#else  /* (_SUPPORT_PID_SPEED_CURRENT != FALSE) */
    if ( (g_e8MotorStatus & C_MOTOR_STATUS_RUNNING) != 0U)
    {
        /* Speed Control (Running-mode) */
        if (Get_NrOfCommut() < 6U)
        {
            /* No movement in expected time; Increase Motor PWM DC */
            i16ControlError = (int16_t)g_u16TargetMotorSpeedRPM;
        }
        else
        {
            /* Speed Control (at least 6 commutations) */
            i16ControlError = (int16_t)(g_u16TargetMotorSpeedRPM - g_u16ActualMotorSpeedRPM);   /* Speed correction is more stable for low-speeds */
        }

#if (_SUPPORT_BRAKING != FALSE)                                                 /* MMP230811-1 */
        if ( (g_e8MotorStatus & C_MOTOR_STATUS_STOP_MASK) == C_MOTOR_STATUS_BRAKING)
        {
            /* Invert the error (Too slow: Reduce breaking current) */
            i16ControlError = -i16ControlError;
        }
#endif /* (_SUPPORT_BRAKING != FALSE) */
    }
#endif /* (_SUPPORT_PID_SPEED_CURRENT != FALSE) */
    else
#endif /* (_SUPPORT_HALL_LATCH != FALSE) && (_SUPPORT_NR_OF_HL > 1) && (_SUPPORT_HALL_LATCH_DIAG == FALSE) */
#if (_SUPPORT_FOC_MODE != FOC_MODE_NONE)
    if ( (l_e8MotorStartupMode & E_MSM_MODE_MASK) != E_MSM_STEPPER)
    {
        /* Speed control */
        int16_t i16ErrorS;
#if (_SUPPORT_PID_SPEED_CURRENT != FALSE)
        int16_t i16ErrorI, i16Divisor;
#endif /* (_SUPPORT_PID_SPEED_CURRENT != FALSE) */

        /* i16ErrorS = (int16_t) (g_u16TargetMotorSpeedRPM - g_u16ActualMotorSpeedRPM);*/
        if (g_u16TargetMotorSpeedRPM > g_u16ActualMotorSpeedRPM)
        {
            /* Acceleration */
            if ( (g_u16TargetMotorSpeedRPM - g_u16ActualMotorSpeedRPM) > 32767)
            {
                i16ErrorS = 32767;
            }
            else
            {
                i16ErrorS = (int16_t)(g_u16TargetMotorSpeedRPM - g_u16ActualMotorSpeedRPM);
            }
        }
        else
        {
            /* Deceleration */
            if ( (g_u16ActualMotorSpeedRPM - g_u16TargetMotorSpeedRPM) > 32767)
            {
                i16ErrorS = -32767;
            }
            else
            {
                i16ErrorS = (int16_t)(g_u16TargetMotorSpeedRPM - g_u16ActualMotorSpeedRPM);
            }
        }
#if (_SUPPORT_PID_SPEED_LIMIT != FALSE)
        if (i16ErrorS < 0)
        {
            /* Ramp-down limitation */
            if ( NV_PID_RAMP_DOWN != 0 )
            {
                l_u16SpeedRampDownLimit = (uint16_t)(p_MulU32_U16byU16(g_u16ActualMotorSpeedRPM, NV_PID_RAMP_DOWN) >> 10);  /* MMP230303-1: Adapt rampdown-limit by actual-speed, not max-speed TEST */
            }
            if (i16ErrorS < (int16_t)(0 - l_u16SpeedRampDownLimit) )
            {
                i16ErrorS = (int16_t)(0 - l_u16SpeedRampDownLimit);
            }
        }
        else
        {
            if (i16ErrorS > (int16_t)l_u16SpeedRampUpLimit)
            {
                i16ErrorS = (int16_t)l_u16SpeedRampUpLimit;
            }
        }
#elif (_SUPPORT_CDI != FALSE)
        if ( ((l_e8MotorStartupMode & E_MSM_CDI) != 0U) && (i16ErrorS < 0) )
        {
            /* Ramp-down limitation, only in CDI mode */
            uint16_t u16SpeedRampDownLimit = p_MulU16hi_U16byU16(g_u16MaxSpeedRPM, 6554U);   /* 10% of Speed4; 6554/65536 */
            if (i16ErrorS < (int16_t)(0 - u16SpeedRampDownLimit) )
            {
                i16ErrorS = (int16_t)(0 - u16SpeedRampDownLimit);
            }
        }
#endif /* (_SUPPORT_PID_SPEED_LIMIT != FALSE) */
#if (_SUPPORT_PID_SPEED_CURRENT != FALSE)
        if (i16ErrorS >= 0)
        {
            /* Ramp-up: Limit up to maximum current defined by 'l_u16ActCurrRunMax_LSB' */
            i16ErrorI = (int16_t)(l_u16ActCurrRunMax_LSB - Get_MotorCurrentLPF());
        }
        else
        {
            /* Ramp-down: Limit up to minimum current defined by 'l_u16ActCurrRunMin_LSB' */
            i16ErrorI = (int16_t)(Get_MotorCurrentLPF() - l_u16ActCurrRunMin_LSB);
        }
        if (i16ErrorI < 0)
        {
            i16ErrorI = 0;
        }
        i16Divisor = (int16_t)l_u16ActCurrRunMax_LSB;
        i16ControlError = p_MulDivI16_I16byI16byI16(i16ErrorI, i16ErrorS, i16Divisor);
#if (_SUPPORT_BRAKING != FALSE)                                                 /* MMP230811-1 */
        if ( (g_e8MotorStatus & C_MOTOR_STATUS_STOP_MASK) == C_MOTOR_STATUS_BRAKING)
        {
            /* Invert the speed-error */
            i16ControlError = -i16ControlError;
        }
#endif /* (_SUPPORT_BRAKING != FALSE) */
#else  /* (_SUPPORT_PID_SPEED_CURRENT != FALSE) */
        i16ControlError = i16ErrorS;
#endif /* (_SUPPORT_PID_SPEED_CURRENT != FALSE) */
    }
    else
#endif /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) */
    {
        /* Current control */
#if (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) && ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) != FOC_MODE_ID_IQ)
        if ( (g_e8MotorStatus & C_MOTOR_STATUS_STOP_MASK) == C_MOTOR_STATUS_STOP)
        {
            /* Holding mode */
            i16ControlError = (int16_t)(l_u16PidHoldingThreshold - Get_MotorCurrentLPF());
#if ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_IB_IPK)
            sPIDpSE2VA.u16MinOutput = l_u16PidHoldingThreshold;
#else  /* ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_IB_IPK) */
            sPIDpSE2VA.u16MinOutput = NV_MIN_HOLDCORR_RATIO;
#endif /* ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_IB_IPK) */
        }
        else
#endif /* (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) && ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) != FOC_MODE_ID_IQ) */
        {
            /* Running-mode */
#if (_SUPPORT_FOC_MODE != FOC_MODE_NONE) && (_SUPPORT_AUTO_SPEED_FOC == FALSE)
            i16ControlError = (int16_t)(l_u16ActCurrStartMax_LSB - Get_MotorCurrentLPF());
#else  /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) && (_SUPPORT_AUTO_SPEED_FOC == FALSE) */
            i16ControlError = (int16_t)(l_u16ActCurrRunMax_LSB - Get_MotorCurrentLPF());
#endif /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) && (_SUPPORT_AUTO_SPEED_FOC == FALSE) */
        }
    }
#endif /* (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM) */

    return (i16ControlError);

} /* End of PID_Control_Error() */

#if (_SUPPORT_FOC_MODE != FOC_MODE_NONE) && (_SUPPORT_PWM_MODE != SINGLE_COIL_PWM_BIPOLAR)
/*!*************************************************************************** *
 * PID_Control
 * \brief   Current & Speed Control
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  (uint16_t) sPIDpSE2VA.u32SumError
 * *************************************************************************** *
 * \details -
 * Note:    This function is not allowed to be called during Non Volatile Memory Write
 *          (due to Non Volatile Memory reads) (C_APP_POSITIONING_ACT or SINGLE_COIL_PWM).
 * *************************************************************************** *
 * - Call Hierarchy: main()
 * - Cyclomatic Complexity: 4+1
 * - Nesting: 4
 * - Function calling: 5 (PID_Control_Period(), PID_Control_Error(),
 *                        VoltageCorrection(), Get_MicroStepIdx(), p_PID_Control())
 * *************************************************************************** */
uint16_t PID_Control(void)
#else  /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) */
/*!*************************************************************************** *
 * PID_Control
 * \brief   Current Control or Speed Control (_SUPPORT_HALL_LATCH)
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details -
 * Note:    This function is not allowed to be called during Non Volatile Memory Write
 *          (due to Non Volatile Memory reads) (C_APP_POSITIONING_ACT or SINGLE_COIL_PWM).
 * *************************************************************************** *
 * - Call Hierarchy: main()
 * - Cyclomatic Complexity: 4+1
 * - Nesting: 4
 * - Function calling: 6 (PID_Control_Period(), PID_Control_Error(),
 *                        VoltageCorrection(), Get_MicroStepIdx(),
 *                        MotorDriver_3Phase(), p_PID_Control())
 * *************************************************************************** */
void PID_Control(void)
#endif /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) */
{
#if (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT)
    if ( (g_e8MotorStatus & C_MOTOR_STATUS_MASK) != C_MOTOR_STATUS_STOP)        /* Stop-mode & holding-current required */
#else  /* (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) */
    if ( (g_e8MotorStatus & C_MOTOR_STATUS_STOP_MASK) != C_MOTOR_STATUS_STOP)   /* Only running mode */
#endif /* (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) */
    {
        /* Running-mode */
        uint16_t u16PidPeriod = PID_Control_Period();

        if ( (u16PidPeriod != 0U) && (l_u16PID_CtrlCounter >= u16PidPeriod) )   /* Period */
        {
            /* Periodic update (PWM-DC/Current/Speed-Control) */
            int16_t i16ControlError;

#if (_DEBUG_PID_CONTROL != FALSE)
            DEBUG_SET_IO_E();
#endif /* (_DEBUG_PID_CONTROL != FALSE) */

#if (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM_BIPOLAR)
            if (NV_PID_RUNNINGCTRL_PER_UNIT == C_PID_PERIOD_UNIT_TIME)
            {
                l_u16PID_Period = (NV_PID_RUNNINGCTRL_PER * PI_TICKS_PER_MILLISECOND) * 2U;
            }
            else
            {
#if (_SUPPORT_SPEED_CTRL != FALSE)
                if (g_u16ActualCommutTimerPeriod != 0U)
                {
#if (TIMER_PRESCALER == 1)
                    l_u16PID_Period = (g_u16ActualCommutTimerPeriod >> 8);
#elif (TIMER_PRESCALER == 16)
                    l_u16PID_Period = (g_u16ActualCommutTimerPeriod >> 4);
#else
                    l_u16PID_Period = (g_u16ActualCommutTimerPeriod >> 2);
#endif
                }
#else  /* (_SUPPORT_SPEED_CTRL != FALSE) */
                l_u16PID_Period = (Get_CommutTimerPeriod() >> 2);
#endif /* (_SUPPORT_SPEED_CTRL != FALSE) */
            }
#endif /* (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM_BIPOLAR) */

#if (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT)
#if ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_IB_IPK)
            sPIDpSE2VA.u16MinOutput = l_u16ActCurrRunMin_LSB;
#elif ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) != FOC_MODE_ID_IQ)
            sPIDpSE2VA.u16MinOutput = l_u16MinCorrectionRatio;
#endif
#endif /* (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) */

            i16ControlError = PID_Control_Error();

            /* PID Motor PWM Duty Cycle correction, based on error and PID-coefficients */
            if (i16ControlError != 0)
            {
#if (_SUPPORT_CDI != FALSE)
                if ( (l_e8MotorStartupMode & E_MSM_CDI) != 0U)
                {
                    i16ControlError *= 2U;
                }
#endif /* (_SUPPORT_CDI != FALSE) */
#if ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_ID_IQ)
                /* FOC ID-IQ: Speed-error --> Target-Ipk */
                l_u16TargetIpk = (uint16_t)p_PID_Control(i16ControlError, (void *)&sPIDpSE2VA);
#elif ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_IB_IPK)
                if ( (l_e8MotorStartupMode & E_MSM_MODE_MASK) != E_MSM_STEPPER)  /* MMP240129-1: CL perform Ipk control (FOC IB+Ipk) */
                {
                    /* FOC IB+Ipk: Speed-error --> Target-Ipk */
                    l_u16TargetIpk = (uint16_t)p_PID_Control(i16ControlError, (void *)&sPIDpSE2VA);
                }
                else  /* MMP240129-1: OL perform current control */
                {
                    /* Open Loop (stepper): Current(Torque)-error --> Motor-Voltage (PWM Duty Cycle) */
                    l_u16PidCtrlRatio = (uint16_t)p_PID_Control(i16ControlError, (void *)&sPID_IpkCtrl);
                }
#else  /* (_SUPPORT_FOC_MODE & FOC_MODE_MASK) */
                /* FOC IB-only & Open Loop (stepper): Speed * Current error --> Motor-Voltage (PWM Duty Cycle) */
                l_u16PidCtrlRatio = (uint16_t)p_PID_Control(i16ControlError, (void *)&sPIDpSE2VA);
#endif /* (_SUPPORT_FOC_MODE & FOC_MODE_MASK) */
            }
            l_u16PID_CtrlCounter = 0U;

#if (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT)
            /* Update motor-driver PWM duty-cycle in case of holding-mode with coil-current */
            /* Stop-mode & holding-current required */
            if ( (g_e8MotorStatus == C_MOTOR_STATUS_HOLD) && (Get_MotorHoldingCurrState() != FALSE) )
            {
                Set_CorrectionRatio(VoltageCorrection());
#if (C_MOTOR_PHASES == 3U)
                MotorDriver_3Phase(Get_MicroStepIdx());
#elif (C_MOTOR_PHASES == 4U)
                MotorDriver_4Phase(Get_MicroStepIdx());
#endif
            }
#endif /* (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) */

#if (_SUPPORT_STALLDET_S != FALSE)
            if (MotorStallCheckS() == C_STALL_FOUND)
            {
                /* Speed low stall detected */
                g_u8StallTypeComm |= (uint8_t)C_STALL_FOUND_S;
                if ( (g_e8StallDetectorEna & (uint8_t)C_STALLDET_S) != 0U)
                {
                    g_u8StallOcc = TRUE;                                        /* Report stall and ...  */
                    MotorDriverStop( (uint16_t)C_STOP_EMERGENCY);               /* ... stop motor (Stall) */
                }
            }
            else
            {
                /* Nothing */
            }
#endif /* (_SUPPORT_STALLDET_S != FALSE) */

#if (_DEBUG_PID_CONTROL != FALSE)
            DEBUG_CLR_IO_E();
#endif /* (_DEBUG_PID_CONTROL != FALSE) */
        }
    }

#if (_SUPPORT_FOC_MODE != FOC_MODE_NONE) && (_SUPPORT_PWM_MODE != SINGLE_COIL_PWM_BIPOLAR)
    return ( (uint16_t)(sPIDpSE2VA.u32SumError >> C_GN_PID) );
#endif /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) && (_SUPPORT_PWM_MODE != SINGLE_COIL_PWM_BIPOLAR) */
} /* End of PID_Control() */

#if (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM_BIPOLAR)
#if ((_SUPPORT_HALL_LATCH == FALSE) && (_SUPPORT_FOC_MODE != FOC_MODE_NONE)) || (_SUPPORT_WINDMILL != FALSE)
/*!*************************************************************************** *
 * PID_Open2Close()
 * \brief   Setup PID for Close-loop operation
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details
 * Note:    This function is not allowed to be called during Non Volatile Memory Write
 *          (due to Non Volatile Memory reads) (_SUPPORT_FOC_MODE or SINGLE_COIL_PWM_BIPOLAR).
 * *************************************************************************** *
 * - Call Hierarchy: MotorDriverStart(), ISR_CTIMER0_3()
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 0
 * *************************************************************************** */
void PID_Open2Close(void)
{
#if (_SUPPORT_FOC_MODE != FOC_MODE_NONE)
    l_u16PidCtrlRatio = (uint16_t)(p_MulU32_U16byU16(l_u16PidCtrlRatio, NV_LA_OPEN_TO_CLOSE_AMPL) >> 3);
#endif /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) */
    sPIDpSE2VA.u32SumError = (((uint32_t)l_u16PidCtrlRatio) << C_GN_PID);
#if (_SUPPORT_FOC_MODE != FOC_MODE_NONE)
    sPIDpCommut.u16PID_LA_I = Get_CommutTimerPeriod();
#endif /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) */
} /* End of PID_Open2Close() */
#endif /* ((_SUPPORT_HALL_LATCH == FALSE) && (_SUPPORT_FOC_MODE != FOC_MODE_NONE)) || (_SUPPORT_WINDMILL != FALSE) */
#endif /* (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM_BIPOLAR) */

#if (_SUPPORT_FOC_MODE != FOC_MODE_NONE)
#if (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM_BIPOLAR)
static __inline__ uint16_t p_PIdpCommut(int16_t i16Error, uint16_t *psPID) __attribute__ ((always_inline));
/*!*************************************************************************** *
 * p_PIdpCommut
 * \brief   PI(D) per commutation
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] i16Error: PI(D) input (error-signal)
 * \param   [in] psPID: Pointer to PID-Structure
 *            [X+0] = CoefI
 *            [X+2] = CoefP
 *            [X+4] = MemE
 *            [X+6] = MemI
 * \return  uint16_t: PI(D) output
 * *************************************************************************** *
 * \details This PI(D) is intentional for angle(error) to micro-step
 *          commutation-period. If set to 0, the minimum period should be used
 * *************************************************************************** *
 * - Call Hierarchy: main()
 * - Cyclomatic Complexity: 13+1
 * - Nesting: 8
 * - Function calling: 2 (VoltageCorrection(), Get_MicroStepIdx())
 * *************************************************************************** */
static __inline__ uint16_t p_PIdpCommut(int16_t i16Error, uint16_t *psPID)
{
    uint16_t u16Result;
    uint16_t u16Garbish;

    __asm__ __volatile__ (
        "mov [X+4], A\n\t"                   /* MemE */
        "muls YA, A, [X++]\n\t"              /* i16Error * CoefI */
        "asr YA, #10\n\t"                    /* Divide 1024 (MMP220113-1) */
        "adc A, #0\n\t"                      /* Round */
        "jsge p_PID_C_10_%=\n\t"
        "mov Y, A\n\t"
        "neg Y\n\t"
        "cmp Y, [X+4]\n\t"                   /* MemI */
        "jule p_PID_C_10_%=\n\t"
        "mov A, #0\n\t"
        "jmp p_PID_C_20_%=\n\t"
        "p_PID_C_10_%=:\n\t"
        "add A, [X+4]\n\t"                   /* MemI */
        "p_PID_C_20_%=:\n\t"
        "mov [X+4], A\n\t"                   /* MemI */
        "lod A, [X+2]\n\t"                   /* MemE */
        "muls YA, A, [X]\n\t"                /* i16Error * CoefP */
        "asr YA, #10\n\t"                    /* Divide 1024 (MMP220113-1) */
        "adc A, #0\n\t"                      /* Round */
        "jsge p_PID_C_30_%=\n\t"
        "mov Y, A\n\t"
        "neg Y\n\t"
        "cmp Y, [X+4]\n\t"                   /* MemI */
        "jule p_PID_C_30_%=\n\t"
        "mov A, #0\n\t"
        "mov [X+4], A\n\t"                   /* MemI */
        "jmp p_PID_C_Exit_%=\n\t"
        "p_PID_C_30_%=:\n\t"
        "add A, [X+4]\n\t"                   /* MemI */
        "p_PID_C_Exit_%=:\n\t"
        : "=a" (u16Result), "=y" (u16Garbish), "=x" (u16Garbish)
        : "a" (i16Error), "x" (psPID)
        );

    return (u16Result);
} /* End of p_PIdpCommut() */

#if (_SUPPORT_HALL_LATCH == FALSE)
/*!*************************************************************************** *
 * PID_LA_Speed_Control
 * \brief   Commutation-time correction
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] i16ActLoadAngle: Actual LA angle in radial (2pi = 2^12)
 * \return  uint16_t l_u16PID_LA_I: PID_I of LA correction
 * *************************************************************************** *
 * \details
 * Note:    This function is not allowed to be called during Non Volatile Memory Write
 *          (due to Non Volatile Memory reads) (_SUPPORT_FOC_MODE or SINGLE_COIL_PWM_BIPOLAR).
 * *************************************************************************** *
 * - Call Hierarchy: SwitchOpen2Close()
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 0
 * *************************************************************************** */
uint16_t PID_LA_Speed_Control(int16_t i16ActLoadAngle)
{
    uint16_t u16CommutPeriod;
    int16_t i16LoadAngleError;

    l_u16ActSpeedTgtLoadAngle = p_MulDivU16_U16byU16byU16(NV_TARGET_LA,
                                                          g_u16ActualMotorSpeedRPM,
                                                          g_u16MaxSpeedRPM);
    /* Convert angle-error into time-error */
    i16LoadAngleError = p_MulDivI16_I16byI16byI16( (int16_t)(l_u16ActSpeedTgtLoadAngle - i16ActLoadAngle),
                                                   Get_CommutTimerPeriod(),
                                                   C_MICRO_STEP_RAD);
    u16CommutPeriod = p_PIdpCommut(i16LoadAngleError, (uint16_t *)&sPIDpCommut);
    if (u16CommutPeriod < C_MIN_PWM_PERIOD)
    {
        u16CommutPeriod = C_MIN_PWM_PERIOD;
    }
    Set_CommutTimerPeriod(u16CommutPeriod);
    return (sPIDpCommut.u16PID_LA_I);
} /* End of PID_LA_Speed_Control() */
#endif /* (_SUPPORT_HALL_LATCH == FALSE) */

#else  /* (_SUPPORT_PWM_MODE != SINGLE_COIL_PWM_BIPOLAR) */
/*!*************************************************************************** *
 * PID_SwitchOpen2Close
 * \brief   Switch PID parameters from Open-loop to Close-loop mode
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details
 * Note:    This function is not allowed to be called during Non Volatile Memory Write
 *          (due to Non Volatile Memory reads) (_SUPPORT_FOC_MODE).
 * *************************************************************************** *
 * - Call Hierarchy: SwitchOpen2Close()
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 0
 * *************************************************************************** */
void PID_SwitchOpen2Close(void)
{
    if (l_u16Ipk > (p_MulU32_U16byU16(l_u16ActCurrRunMax_LSB, NV_LA_OPEN_TO_CLOSE_AMPL) >> 3) )  /* MMP240207-3: Actual-current above Max-current * Open2Close-ratio */
    {
        l_u16PidCtrlRatio = (uint16_t)(p_MulU32_U16byU16(l_u16PidCtrlRatio, NV_LA_OPEN_TO_CLOSE_AMPL) >> 3);
    }
#if ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_ID_IQ)
    /* Ipk Control */
    sPIDpSE2VA.u32SumError = (((uint32_t)l_u16TargetIpk) << C_GN_PID);
#if (_SUPPORT_PID_D_COEF != FALSE)
    sPIDpSE2VA.i16PrevError = 0;
#endif /* (_SUPPORT_PID_D_COEF != FALSE) */
    sPID_IdCtrl.u32SumError = 0U;
#if (_SUPPORT_PID_D_COEF != FALSE)
    sPID_IdCtrl.i16PrevError = 0;
#endif /* (_SUPPORT_PID_D_COEF != FALSE) */
    sPID_IqCtrl.u32SumError =
        p_MulDivU16_U16byU16byU16(l_u16PidCtrlRatio, l_u16MotorRefVoltageADC,
                                  (PWM_REG_PERIOD << (4 + PWM_PRESCALER_N)));
#if (_SUPPORT_PID_D_COEF != FALSE)
    sPID_IqCtrl.i16PrevError = 0;
#endif /* (_SUPPORT_PID_D_COEF != FALSE) */
#else  /* ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_ID_IQ) */
#if ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_IB_IPK)
    PID_Ipk_Start(l_u16Ipk);                                                    /* MMP240207-2 */
#else  /* ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_IB_IPK) */
    sPIDpSE2VA.u32SumError = (((uint32_t)l_u16PidCtrlRatio) << C_GN_PID);
#endif /* ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_IB_IPK) */
    /* PID: I-mem = Actual-speed * PP * 256/Coef-I --> (I-mem * Coef-I) / 256 = Actual-speed (electric) */
    sPIDpLA2AS.u32SumError = (p_MulU32_U16byU16(g_u16ActualMotorSpeedRPM, g_u16MotorPolePairs) << C_GN_PID);
#if (_SUPPORT_PID_D_COEF != FALSE)
    sPIDpLA2AS.i16PrevError = 0;
#endif /* (_SUPPORT_PID_D_COEF != FALSE) */
#endif /* ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_ID_IQ) */
} /* End of PID_SwitchOpen2Close() */

#if (_SUPPORT_CDI != FALSE)
/*!*************************************************************************** *
 * PID_SwitchCDI2FOC
 * \brief   Switch PID parameters from CDI to FOC mode
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details
 * *************************************************************************** *
 * - Call Hierarchy: SwitchCDI2FOC()
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 0
 * *************************************************************************** */
void PID_SwitchCDI2FOC(void)
{
#if ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_ID_IQ) || \
    ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_IB_IPK)
    /* Ipk Control */
    sPIDpSE2VA.u32SumError = (((uint32_t)l_u16TargetIpk) << C_GN_PID);
#else  /* ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_ID_IQ) || ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_IB_IPK) */
    sPIDpSE2VA.u32SumError = (((uint32_t)l_u16PidCtrlRatio) << C_GN_PID);
#endif /* ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_ID_IQ) || ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_IB_IPK) */
    /* PID: I-mem = Actual-speed * PP * 256/Coef-I --> (I-mem * Coef-I) / 256 = Actual-speed (electric) */
    sPIDpLA2AS.u32SumError = (p_MulU32_U16byU16(g_u16ActualMotorSpeedRPM, g_u16MotorPolePairs) << C_GN_PID);
#if (_SUPPORT_PID_D_COEF != FALSE)
    sPIDpLA2AS.i16PrevError = 0;
#endif /* (_SUPPORT_PID_D_COEF != FALSE) */
} /* End of PID_SwitchCDI2FOC() */
#endif /* (_SUPPORT_CDI != FALSE) */

#if (_SUPPORT_PID_INLINE == FALSE)
#if ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_ID_IQ)
#if (_SUPPORT_OPTIMIZE_FOR_SPEED != FALSE)
void PID_FOC_IDIQ(int16_t i16Id, int16_t i16Iq, int16_t i16Sine, int16_t i16Cosine) __attribute__((aligned(8)));
#endif /* (_SUPPORT_OPTIMIZE_FOR_SPEED != FALSE) */
/*!*************************************************************************** *
 * PID_FOC_IDIQ
 * \brief   PID for FOC Id / Iq.
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] i16Id: FOC Id
 *          [in] i16Iq: FOC Iq
 *          [in] i16Sine: Sine of  Rotor-angle
 *          [in] i16Cosine: Cosine of Rotor-angle
 * \return  (uint16_t) Voltage-angle
 * *************************************************************************** *
 * \details
 * *************************************************************************** *
 * - Call Hierarchy:
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 1 (p_PID_Control())
 * *************************************************************************** */
void PID_FOC_IDIQ(int16_t i16Id, int16_t i16Iq, int16_t i16Sine, int16_t i16Cosine)
{
    /* Target I_d = 0. TODO: when field weakening is required, a target I_d current must be added */
    int16_t i16Vd = p_iPID_Control( (0 - i16Id), (void *)&sPID_IdCtrl);

    /* Vector-Amplitude Control (Motor-PWM Duty-Cycle) */
    int16_t i16Vq = (uint16_t)p_PID_Control( (int16_t)(l_u16TargetIpk - i16Iq), (void *)&sPID_IqCtrl);

    /* Inverse Park Transformation */
    /* Valpha = Vd * cos(theta) - Vq * sin(theta)
     * Vbeta  = Vd * sin(theta) + Vq * cos(theta)
     * (theta = rotor-angle)
     */
    int16_t i16Valpha = p_MulI16_I16byQ15x(i16Vq, i16Sine);
    int16_t i16Vbeta = p_MulI16_I16byQ15x(i16Vd, i16Sine);
    l_i16Valpha = p_MulI16_I16byQ15x(i16Vd, i16Cosine) - i16Valpha;
    l_i16Vbeta = i16Vbeta + p_MulI16_I16byQ15x(i16Vq, i16Cosine);

    /* Voltage angle: atan(Valpha/Vbeta) */
#if (_SUPPORT_FAST_ATAN2 != FALSE)
    l_u16VoltageAngle = p_atan2I16(l_i16Valpha, l_i16Vbeta);                    /* 2^16 = 360 deg or 2pi */
#else  /* (_SUPPORT_FAST_ATAN2 != FALSE) */
    l_u16VoltageAngle = atan2I16(l_i16Valpha, l_i16Vbeta);                      /* 2^16 = 360 deg or 2pi */
#endif /* (_SUPPORT_FAST_ATAN2 != FALSE) */

    /* Voltage amplitude */
#if (_SUPPORT_BEMF_SINUSOIDAL != FALSE)
    /* Vpk = (Valpha * Sine + Vbeta * Cosine) (MMP230803-2) */
    uint16_t u16Idx = (uint16_t)p_MulU16hi_U16byU16((uint16_t)l_u16VoltageAngle, l_u16MotorMicroStepsPerElecRotation);
#if (_SUPPORT_SINCOS_TABLE_SZ == 192)
    int16_t *pi16SinCos = (int16_t *) &c_ai16MicroStepVector3PH_SinCos192[u16Idx];
#elif (_SUPPORT_SINCOS_TABLE_SZ == 256)
    int16_t *pi16SinCos = (int16_t *) &c_ai16MicroStepVector3PH_SinCos256[u16Idx];
#else
#error "ERROR: Sine/Cosine table not supported"
#endif
    uint16_t u16Vpk = p_MulI16_I16bypQ15(l_i16Valpha, pi16SinCos) +
                      p_MulI16_I16bypQ15(l_i16Vbeta, (pi16SinCos + C_COS_OFFSET));
#else  /* (_SUPPORT_BEMF_SINUSOIDAL != FALSE) */
    /* Vpk = SQRT(Valpha^2 + Vbeta^2) */
    uint16_t u16Vpk = p_AproxSqrtU16_I16byI16(l_i16Valpha, l_i16Vbeta);
#endif /* (_SUPPORT_BEMF_SINUSOIDAL != FALSE) */

    l_u16PidCtrlRatio = p_MulDivU16_U16byU16byU16(u16Vpk,
                                                  (PWM_REG_PERIOD << (4 + PWM_PRESCALER_N)),
                                                  l_u16MotorRefVoltageADC);

} /* End of PID_FOC_IDIQ() */
#else  /* ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_ID_IQ) */
#if (_SUPPORT_PID_U32 == FALSE)
/*!*************************************************************************** *
 * PID_LoadAngleToActSpeed
 * \brief   Speed correction
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] i16ActualLA: Actual LA angle in radial (2pi = 2^12)
 * \return  (uint16_t) u16ActualSpeed
 * *************************************************************************** *
 * \details
 * *************************************************************************** *
 * - Call Hierarchy: ADC_ISR()
 * - Cyclomatic Complexity: 1+1
 * - Nesting: 1
 * - Function calling: 1 (p_PID_Control())
 * *************************************************************************** */
#if (_SUPPORT_FOC_MODE != FOC_MODE_NONE) && _SUPPORT_OPTIMIZE_FOR_SPEED
uint16_t PID_LoadAngleToActSpeed(int16_t i16ActualLA) __attribute__((aligned(8)));
#endif /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) && _SUPPORT_OPTIMIZE_FOR_SPEED */
uint16_t PID_LoadAngleToActSpeed(int16_t i16ActualLA)
{
    uint16_t u16ActualSpeed;

#if (_SUPPORT_MLX_DEBUG_MODE != FALSE)
    int16_t i16ErrorLA;

    l_u16ActSpeedTgtLoadAngle = p_MulDivU16_U16byU16byU16(l_u16MaxSpeedTgtLoadAngle,
                                                          g_u16ActualMotorSpeedRPM,
                                                          g_u16MaxSpeedRPM);
    i16ErrorLA = (int16_t)(i16ActualLA - l_u16ActSpeedTgtLoadAngle);
#else  /* (_SUPPORT_MLX_DEBUG_MODE != FALSE) */
    int16_t i16TgtLoadAngle = p_MulDivU16_U16byU16byU16(l_u16MaxSpeedTgtLoadAngle,
                                                        g_u16ActualMotorSpeedRPM,
                                                        g_u16MaxSpeedRPM);
    int16_t i16ErrorLA = (int16_t)(i16ActualLA - l_u16ActSpeedTgtLoadAngle);
#endif /* (_SUPPORT_MLX_DEBUG_MODE != FALSE) */

    /* Load-Angle to Actual-Speed Control */
#if (C_PID_ANGLE_ACCURACY > 0)
    i16ErrorLA >>= C_PID_ANGLE_ACCURACY;                                        /* Reduced ATAN angle accuracy (MMP240524-1) */
#endif /* (C_PID_ANGLE_ACCURACY > 0) */
#if (_DEBUG_FOC_PERF != FALSE)
    DEBUG_SET_IO_D();
#endif /* (_DEBUG_FOC_PERF != FALSE) */
    u16ActualSpeed = (uint16_t)p_PID_Control(i16ErrorLA, (void *)&sPIDpLA2AS);
#if (_DEBUG_FOC_PERF != FALSE)
    DEBUG_CLR_IO_D();
#endif /* (_DEBUG_FOC_PERF != FALSE) */

    return (u16ActualSpeed);
} /* End of PID_LoadAngleToActSpeed() */
#else  /* (_SUPPORT_PID_U32 == FALSE) */
/*!*************************************************************************** *
 * PID_LoadAngleToActSpeed
 * \brief   Speed correction
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] i16ActualLA: Actual LA angle in radial (2pi = 2^12)
 * \return  (uint32_t) u32ActualSpeed
 * *************************************************************************** *
 * \details
 * *************************************************************************** *
 * - Call Hierarchy: ADC_ISR()
 * - Cyclomatic Complexity: 1+1
 * - Nesting: 1
 * - Function calling: 1 (p_PID_Control())
 * *************************************************************************** */
#if (_SUPPORT_FOC_MODE != FOC_MODE_NONE) && _SUPPORT_OPTIMIZE_FOR_SPEED
uint32_t PID_LoadAngleToActSpeed(int16_t i16ActualLA) __attribute__((aligned(8)));
#endif /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) && _SUPPORT_OPTIMIZE_FOR_SPEED */
uint32_t PID_LoadAngleToActSpeed(int16_t i16ActualLA)
{
    uint32_t u32ActualSpeed;

#if (_SUPPORT_MLX_DEBUG_MODE != FALSE)
    int16_t i16ErrorLA;

    l_u16ActSpeedTgtLoadAngle = p_MulDivU16_U16byU16byU16(l_u16MaxSpeedTgtLoadAngle,
                                                          g_u16ActualMotorSpeedRPM,
                                                          g_u16MaxSpeedRPM);
    i16ErrorLA = (int16_t)(i16ActualLA - l_u16ActSpeedTgtLoadAngle);
#else  /* (_SUPPORT_MLX_DEBUG_MODE != FALSE) */
    int16_t i16TgtLoadAngle = p_MulDivU16_U16byU16byU16(l_u16MaxSpeedTgtLoadAngle,
                                                        g_u16ActualMotorSpeedRPM,
                                                        g_u16MaxSpeedRPM);
    int16_t i16ErrorLA = (int16_t)(i16ActualLA - l_u16ActSpeedTgtLoadAngle);
#endif /* (_SUPPORT_MLX_DEBUG_MODE != FALSE) */

    /* Load-Angle to Actual-Speed Control */
#if (C_PID_ANGLE_ACCURACY > 0)
    i16ErrorLA >>= C_PID_ANGLE_ACCURACY;                                        /* Reduced ATAN angle accuracy (MMP240524-1) */
#endif /* (C_PID_ANGLE_ACCURACY > 0) */
#if (_DEBUG_FOC_PERF != FALSE)
    DEBUG_SET_IO_D();
#endif /* (_DEBUG_FOC_PERF != FALSE) */
    u32ActualSpeed = p_PID_Control(i16ErrorLA, (void *)&sPIDpLA2AS);
#if (_DEBUG_FOC_PERF != FALSE)
    DEBUG_CLR_IO_D();
#endif /* (_DEBUG_FOC_PERF != FALSE) */

    return (u32ActualSpeed);
} /* End of PID_LoadAngleToActSpeed() */
#endif /* (_SUPPORT_PID_INLINE == FALSE) */
#endif /* ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_ID_IQ) */
#endif /* (_SUPPORT_PID_U32 == FALSE) */

#if ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_IB_IPK)
/*!*************************************************************************** *
 * PID_Ipk_Start
 * \brief   IPeak control
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16Ipk: Ipeak-level
 * \return  -
 * *************************************************************************** *
 * \details Start Ipk controller
 * *************************************************************************** *
 * - Call Hierarchy: ADC_ISR()
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 0
 * *************************************************************************** */
INLINE void PID_Ipk_Start(uint16_t u16Ipk)
{
    l_u16TargetIpk = u16Ipk;
    /* Ipk Control */
    sPIDpSE2VA.u32SumError = (((uint32_t)l_u16TargetIpk) << C_GN_PID);
#if (_SUPPORT_PID_D_COEF != FALSE)
    sPIDpSE2VA.i16PrevError = 0;
#endif /* (_SUPPORT_PID_D_COEF != FALSE) */
    /* Vector-Amplitude Control */
    sPID_IpkCtrl.u32SumError = (((uint32_t)l_u16PidCtrlRatio) << C_GN_PID);
#if (_SUPPORT_PID_D_COEF != FALSE)
    sPID_IpkCtrl.i16PrevError = 0;
#endif /* (_SUPPORT_PID_D_COEF != FALSE) */
} /* End of PID_Ipk_Start */

#if (_SUPPORT_PID_INLINE == FALSE)
/*!*************************************************************************** *
 * PID_Ipk_Control
 * \brief   IPeak control
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16Ipk: Actual Peak-current
 * \return  -
 * *************************************************************************** *
 * \details
 * *************************************************************************** *
 * - Call Hierarchy: ADC_ISR()
 * - Cyclomatic Complexity: 1+1
 * - Nesting: 1
 * - Function calling: 1 (p_PID_Control())
 * *************************************************************************** */
void PID_Ipk_Control(uint16_t u16Ipk)
{
    int16_t i16ErrorIpk = (int16_t)(l_u16TargetIpk - u16Ipk);

    /* Vector-Amplitude Control (Motor-PWM Duty-Cycle) */
    l_u16PidCtrlRatio = (uint16_t)p_PID_Control(i16ErrorIpk, (void *)&sPID_IpkCtrl);
} /* End of PID_Ipk_Control() */
#endif /* (_SUPPORT_PID_INLINE == FALSE) */
#endif /* ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_IB_IPK) */

#if (_SUPPORT_CLOSED_LOOP_STARTUP != FALSE)
#if (_SUPPORT_PID_U32 == FALSE)
/*!*************************************************************************** *
 * PID_LoadAngleToActSpeed_IV
 * \brief   Compensate Motor micro-step timer
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] i16ActLoadAngle: Actual LA angle in radial (2pi = 2^16)
 * \return  (uint16_t) Actual Speed
 * *************************************************************************** *
 * \details
 * Note:    This function is not allowed to be called during Non Volatile Memory Write
 *          (due to Non Volatile Memory reads) (_SUPPORT_CLOSED_LOOP_STARTUP).
 * ************************************************************************** *
 * - Call Hierarchy: ADC_ISR()
 * - Cyclomatic Complexity: 1+1
 * - Nesting: 1
 * - Function calling: 1 (p_PID_Control())
 * *************************************************************************** */
uint16_t PID_LoadAngleToActSpeed_IV(int16_t i16ActLoadAngle)
{
    int16_t i16ErrorLA;

    /* 16384 is equal to 90 degrees; 65536 is equal to 360 degrees */
#if (LIN_COMM != FALSE) && (_SUPPORT_MLX_DEBUG_MODE != FALSE)
    l_u16ActSpeedTgtLoadAngle = p_MulDivU16_U16byU16byU16(l_u16MaxSpeedTgtLoadAngle,
                                                          g_u16ActualMotorSpeedRPM,
                                                          g_u16MaxSpeedRPM);
    l_i16ActSpeedTgtLoadAngleIV = (int16_t)(C_TARGET_LA_CLSU -
                                  p_MulDivU16_U16byU16byU16(C_TARGET_LA_CLSU,
                                                            g_u16ActualMotorSpeedRPM,
                                                            g_u16LowSpeedRPM));
    if (l_i16ActSpeedTgtLoadAngleIV < (int16_t)l_u16ActSpeedTgtLoadAngle)
    {
        l_e8MotorStartupMode = E_MSM_FOC_2PWM;                                  /* Switch from IV to IB */
        i16ErrorLA = (i16ActLoadAngle - (int16_t)l_u16ActSpeedTgtLoadAngle);
    }
    else
    {
        i16ErrorLA = (l_i16ActSpeedTgtLoadAngleIV - i16ActLoadAngle);
    }
#else  /* (LIN_COMM != FALSE) && (_SUPPORT_MLX_DEBUG_MODE != FALSE) */
    uint16_t u16ActSpeedTgtLoadAngle;
    int16_t i16ActSpeedTgtLoadAngleIV;

    u16ActSpeedTgtLoadAngle = p_MulDivU16_U16byU16byU16(l_u16MaxSpeedTgtLoadAngle,
                                                        g_u16ActualMotorSpeedRPM,
                                                        g_u16MaxSpeedRPM);
    i16ActSpeedTgtLoadAngleIV = C_TARGET_LA_CLSU -
                                p_MulDivU16_U16byU16byU16(C_TARGET_LA_CLSU,
                                                          g_u16ActualMotorSpeedRPM,
                                                          g_u16LowSpeedRPM);
    if (i16ActSpeedTgtLoadAngleIV < (int16_t)u16ActSpeedTgtLoadAngle)
    {
        l_e8MotorStartupMode = E_MSM_FOC_2PWM;                                  /* Switch from IV to IB */
        i16ErrorLA = (i16ActLoadAngle - (int16_t)u16ActSpeedTgtLoadAngle);
    }
    else
    {
        i16ErrorLA = (i16ActSpeedTgtLoadAngleIV - i16ActLoadAngle);
    }
#endif /* (LIN_COMM != FALSE) && (_SUPPORT_MLX_DEBUG_MODE != FALSE) */

    /* Load-Angle to Actual-Speed Control */
    i16ErrorLA >>= C_PID_ANGLE_ACCURACY_IV;                                     /* Reduced ATAN angle accuracy */
    return ((uint16_t)p_PID_Control(i16ErrorLA, (void *)&sPIDpLA2AS));
} /* End of PID_LoadAngleToActSpeed_IV() */
#else  /* (_SUPPORT_PID_U32 == FALSE) */
/*!*************************************************************************** *
 * PID_LoadAngleToActSpeed_IV
 * \brief   Compensate Motor micro-step timer
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] i16ActLoadAngle: Actual LA angle in radial (2pi = 2^16)
 * \return  (uint32_t) Actual Speed
 * *************************************************************************** *
 * \details
 * Note:    This function is not allowed to be called during Non Volatile Memory Write
 *          (due to Non Volatile Memory reads) (_SUPPORT_CLOSED_LOOP_STARTUP).
 * ************************************************************************** *
 * - Call Hierarchy: ADC_ISR()
 * - Cyclomatic Complexity: 1+1
 * - Nesting: 1
 * - Function calling: 1 (p_PID_Control())
 * *************************************************************************** */
uint32_t PID_LoadAngleToActSpeed_IV(int16_t i16ActLoadAngle)
{
    int16_t i16ErrorLA;
    uint32_t u32ActualSpeed;

    uint16_t u16TgtLoadAngle = p_MulDivU16_U16byU16byU16(NV_TARGET_LA,
                                                         g_u16ActualMotorSpeedRPM,
                                                         g_u16MaxSpeedRPM);

    /* 16384 is equal to 90 degrees; 65536 is equal to 360 degrees */
#if (_SUPPORT_MLX_DEBUG_MODE != FALSE)
    l_u16ActSpeedTgtLoadAngle = C_TARGET_LA_CLSU -
                                p_MulDivU16_U16byU16byU16(C_TARGET_LA_CLSU,
                                                          g_u16ActualMotorSpeedRPM,
                                                          g_u16LowSpeedRPM);
    if (l_u16ActSpeedTgtLoadAngle < u16TgtLoadAngle)
    {
        l_e8MotorStartupMode = E_MSM_FOC_2PWM;                                  /* Switch from IV to IB */
    }
    i16ErrorLA = (int16_t)(l_u16ActSpeedTgtLoadAngle - i16ActLoadAngle);
#else  /* (_SUPPORT_MLX_DEBUG_MODE != FALSE) */
    int16_t i16TgtLoadAngleStartUp = C_TARGET_LA_CLSU -
                                     p_MulDivU16_U16byU16byU16(C_TARGET_LA_CLSU,
                                                               g_u16ActualMotorSpeedRPM,
                                                               g_u16LowSpeedRPM);
    if (i16TgtLoadAngleStartUp < (int16_t)u16TgtLoadAngle)
    {
        l_e8MotorStartupMode = E_MSM_FOC_2PWM;                                  /* Switch from IV to IB */
    }
    i16ErrorLA = (i16TgtLoadAngleStartUp - i16ActLoadAngle);
#endif /* (_SUPPORT_MLX_DEBUG_MODE != FALSE) */

    /* Load-Angle to Actual-Speed Control */
    i16ErrorLA >>= C_PID_ANGLE_ACCURACY_IV;                                     /* Reduced ATAN angle accuracy */
    u32ActualSpeed = p_PID_Control(i16ErrorLA, (void *)&sPIDpLA2AS);

    return (u32ActualSpeed);
} /* End of PID_LoadAngleToActSpeed_IV() */
#endif /* (_SUPPORT_PID_U32 == FALSE) */
#endif /* (_SUPPORT_CLOSED_LOOP_STARTUP != FALSE) */
#endif /* (_SUPPORT_PWM_MODE != SINGLE_COIL_PWM_BIPOLAR) */
#endif /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) */

#if (_SUPPORT_CURRENT_TEMP_COMPENSATION != FALSE)
/*!*************************************************************************** *
 * ThresholdControl
 * \brief   Current threshold (set-level) Control
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details DC-Motor: Motor Power
 *          Stepper: Current Threshold Control
 *          BEMF: Nothing
 * Note:    This function is not allowed to be called during Non Volatile Memory Write
 *          (due to Non Volatile Memory reads) (_SUPPORT_CURRENT_TEMP_COMPENSATION).
 * *************************************************************************** *
 * - Call Hierarchy: -
 * - Cyclomatic Complexity: 9+1
 * - Nesting: 5
 * - Function calling: 0
 * *************************************************************************** */
void ThresholdControl(void)
{
    if ( ((g_e8MotorStatus & C_MOTOR_STATUS_MASK) != C_MOTOR_STATUS_STOP) &&
         (l_u16PID_ThrshldCtrlCounter >= (NV_PID_THRSHLDCTRL_PER * PI_TICKS_PER_MILLISECOND) )  /* MMP240709-2 */
    {
        uint16_t u16CurrThrshldRatio;
#if (_SUPPORT_AMBIENT_TEMP != FALSE)
        int16_t i16Temperature = Get_AmbientTemperature();
#else  /* (_SUPPORT_AMBIENT_TEMP != FALSE) */
        int16_t i16Temperature = Get_ChipTemperature();
#endif /* (_SUPPORT_AMBIENT_TEMP != FALSE) */
        int16_t i16TemperatureBgn = NV_CURRTHRSHLD_TEMP_1;
        uint16_t u16CurrThrshldRatioBgn = NV_CURRTHRSHLD_RATIO_1;

        if (i16Temperature < (i16TemperatureBgn - C_CURRTHRSHLD_TEMP_HYS) )
        {
            if (NV_CURRTHRSHLD_ZONE_1 != 0U)
            {
                u16CurrThrshldRatio = u16CurrThrshldRatioBgn;                   /* Same as point _1 */
            }
            else
            {
                u16CurrThrshldRatio = 0U;                                       /* Shutdown motor */
            }
        }
        else
        {
            uint8_t u8CurrThrshldCtrlType = NV_CURRTHRSHLD_ZONE_2;              /* Get current threshold compensation-type */
            uint16_t u16CurrThrshldRatioEnd = NV_CURRTHRSHLD_RATIO_2;           /* Get zone end point (_2) */
            int16_t i16TemperatureEnd = NV_CURRTHRSHLD_TEMP_2;
            if (i16Temperature > i16TemperatureEnd)                             /* Temperature above second zone ? */
            {
                i16TemperatureBgn = i16TemperatureEnd;                          /* Next zone; begin point (_2) */
                u16CurrThrshldRatioBgn = u16CurrThrshldRatioEnd;
                u8CurrThrshldCtrlType = NV_CURRTHRSHLD_ZONE_3;
                u16CurrThrshldRatioEnd = NV_CURRTHRSHLD_RATIO_3;                /* Get zone end point (_3) */
                i16TemperatureEnd = NV_CURRTHRSHLD_TEMP_3;
                if (i16Temperature > i16TemperatureEnd)                         /* Temperature above third zone ? */
                {
                    i16TemperatureBgn = i16TemperatureEnd;                      /* Next zone; begin point (_3) */
                    u16CurrThrshldRatioBgn = u16CurrThrshldRatioEnd;
                    u8CurrThrshldCtrlType = NV_CURRTHRSHLD_ZONE_4;
                    u16CurrThrshldRatioEnd = NV_CURRTHRSHLD_RATIO_4;            /* Get zone end point (_3) */
                    i16TemperatureEnd = NV_CURRTHRSHLD_TEMP_4;
                    if (i16Temperature > i16TemperatureEnd)                     /* Temperature above fourth zone ? */
                    {
                        u8CurrThrshldCtrlType = NV_CURRTHRSHLD_ZONE_5;
                        u16CurrThrshldRatioBgn = u16CurrThrshldRatioEnd;        /* MMP201222-2 */
                    }
                }
            }
            if (u8CurrThrshldCtrlType == 1U)
            {
                u16CurrThrshldRatio = u16CurrThrshldRatioBgn;
            }
            else if (u8CurrThrshldCtrlType == 2U)
            {
                u16CurrThrshldRatio = u16CurrThrshldRatioEnd;
            }
            else if (u8CurrThrshldCtrlType == 3U)
            {
                i16TemperatureEnd = i16TemperatureEnd - i16TemperatureBgn;
                u16CurrThrshldRatio =
                    (uint16_t)(p_MulDivI16_I16byI16byI16( ((int16_t)u16CurrThrshldRatioEnd -
                                                           (int16_t)u16CurrThrshldRatioBgn),
                                                          (i16Temperature - i16TemperatureBgn),
                                                          i16TemperatureEnd) + u16CurrThrshldRatioBgn);
            }
            else
            {
                u16CurrThrshldRatio = 0U;                                       /* Shutdown motor */
            }
        }
        {
            l_u16ActCurrRunMax_LSB = p_MulDivU16_U16byU16byU16(NV_RUNNING_CURR_LEVEL,
                                                               u16CurrThrshldRatio,
                                                               g_u16MCurrGain);
#if (MOTOR_POSITIONING != FALSE)
            l_u16PidHoldingThreshold = p_MulDivU16_U16byU16byU16(NV_HOLDING_CURR_LEVEL,
                                                                 u16CurrThrshldRatio,
                                                                 g_u16MCurrGain);
#endif /* (MOTOR_POSITIONING != FALSE) */
        }
        l_u16PID_ThrshldCtrlCounter = 0U;
    }

} /* End of ThresholdControl */
#endif /* (_SUPPORT_CURRENT_TEMP_COMPENSATION != FALSE) */

#if ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_ID_IQ)
/*!*************************************************************************** *
 * PID_IDIQ_Start
 * \brief   Id/Iq control
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16Ipk: Ipeak-level
 * \return  -
 * *************************************************************************** *
 * \details Start Ipk controller
 * *************************************************************************** *
 * - Call Hierarchy: ADC_ISR()
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 0
 * *************************************************************************** */
INLINE void PID_IDIQ_Start(uint16_t u16Ipk)
{
    l_u16TargetIpk = u16Ipk;
    /* Ipk Control */
    sPIDpSE2VA.u32SumError = (((uint32_t)l_u16TargetIpk) << C_GN_PID);
#if (_SUPPORT_PID_D_COEF != FALSE)
    sPIDpSE2VA.i16PrevError = 0;
#endif /* (_SUPPORT_PID_D_COEF != FALSE) */
    sPID_IdCtrl.u32SumError = 0U;
#if (_SUPPORT_PID_D_COEF != FALSE)
    sPID_IdCtrl.i16PrevError = 0;
#endif /* (_SUPPORT_PID_D_COEF != FALSE) */
    sPID_IqCtrl.u32SumError =
        p_MulDivU16_U16byU16byU16(l_u16PidCtrlRatio, l_u16MotorRefVoltageADC,
                                  (PWM_REG_PERIOD << (4 + PWM_PRESCALER_N)));
#if (_SUPPORT_PID_D_COEF != FALSE)
    sPID_IqCtrl.i16PrevError = 0;
#endif /* (_SUPPORT_PID_D_COEF != FALSE) */
} /* End of PID_IDIQ_Start */
#endif /* ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_ID_IQ) */

#elif (_SUPPORT_APP_TYPE == C_APP_SOLENOID)

#include "../ActADC.h"                                                          /* Application ADC support */

#include "drivelib/NV_Functions.h"                                              /* Non Volatile Memory Functions & Layout */
#include "drivelib/GlobalVars.h"                                                /* Global Variables support */
#include "drivelib/PID_Control.h"                                               /* PID support */
#if (_SUPPORT_APP_TYPE == C_APP_RELAY)
#include "drivelib/RelayDriver.h"                                               /* Relay Driver support */
#elif (_SUPPORT_APP_TYPE == C_APP_SOLENOID)
#include "drivelib/SolenoidDriver.h"                                            /* Solenoid Driver support */
#endif
#include "camculib/private_mathlib.h"                                           /* Private Math Library support */

/*!*************************************************************************** *
 *                            DEFINES                                          *
 * *************************************************************************** */
#if (_APP_SUPPLY_RANGE == C_APP_SUPPLY_RANGE_12V)                               /* MMP230912-1 */
#define C_REF_VOLTAGE               1200U                                       /*!< 12.00V in [10mV]-units */
#elif (_APP_SUPPLY_RANGE == C_APP_SUPPLY_RANGE_24V)
#define C_REF_VOLTAGE               2400U                                       /*!< 24.00V in [10mV]-units */
#elif (_APP_SUPPLY_RANGE == C_APP_SUPPLY_RANGE_48V)
#define C_REF_VOLTAGE               4800U                                       /*!< 48.00V in [10mV]-units */
#endif

#if (_SUPPORT_PID_INLINE == FALSE)
/*!< Static variable defined */
#define PID_STATIC static
#else
/*!< Static variable not defined */
#define PID_STATIC
#endif /* (_SUPPORT_PID_INLINE == FALSE) */

/*!*************************************************************************** */
/*                           GLOBAL & LOCAL VARIABLES                          */
/* *************************************************************************** */
#pragma space dp                                                                /* __TINY_SECTION__ */
uint16_t l_u16PidCtrlRatio;                                                     /*!< PID Control Ratio (at ref. voltage) */
#if (_SUPPORT_ADC_REF_HV_CALIB != FALSE)
PID_STATIC uint16_t l_u16MotorRefVoltageADC = (uint16_t)((12U * 1024U) / (1.5 * C_ADC_HV_DIV));  /*!< Motor Reference Voltage; 12.00V [ADC-LSB] */
#else  /* (_SUPPORT_ADC_REF_HV_CALIB != FALSE) */
PID_STATIC uint16_t l_u16MotorRefVoltageADC = (uint16_t)((12U * 1024U) / (2.5 * C_ADC_HV_DIV));  /*!< Motor Reference Voltage; 12.00V [ADC-LSB] */
#endif /* (_SUPPORT_ADC_REF_HV_CALIB != FALSE) */
PID_STATIC uint16_t l_u16MinCorrectionRatio;                                    /*!< PID Minimum correction ratio */
PID_STATIC uint16_t l_u16MaxCorrectionRatio;                                    /*!< PID Maximum correction ratio */
#pragma space none                                                              /* __TINY_SECTION__ */

#pragma space nodp                                                              /* __NEAR_SECTION__ */
uint16_t l_u16PID_CtrlCounter = 0U;                                             /*!< PID Control period counter */
uint16_t l_u16PidHoldingThreshold;                                              /*!<  Motor holding current threshold */
uint16_t l_u16ActCurrRunMax_mA = 0U;                                           /*!< Motor running current */
uint16_t l_u16ActCurrRunMax_LSB;                                              /*!< Motor current threshold (running) */
static uint16_t l_u16PidLosses = 0U;                                            /*!< PID Losses fraction */
PID_PARAMS_t sPIDpSE2VA =                                                       /*!< PID parameter structure for Speed-Error to Vector-Amplitude */
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
#pragma space none                                                              /* __NEAR_SECTION__ */

/*!*************************************************************************** *
 * PID_SetRunningCurrent
 * \brief   Set running current PID level
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16CurrentLevel: Motor current level in mA's
 * \return  -
 * *************************************************************************** *
 * \details -
 * *************************************************************************** *
 * - Call Hierarchy: PID_Init()
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 0 (all inline)
 * *************************************************************************** */
void PID_SetRunningCurrent(uint16_t u16CurrentLevel)
{
    l_u16ActCurrRunMax_mA = u16CurrentLevel;
    l_u16ActCurrRunMax_LSB = p_MulDivU16_U16byU16byU16(l_u16ActCurrRunMax_mA, C_GMCURR_DIV, Get_MCurrGain());
} /* End of PID_SetRunningCurrent() */

/*!*************************************************************************** *
 * PID_Init
 * \brief   Initialise PID
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details
 * Note:    This function is not allowed to be called during Non Volatile Memory Write
 *          (due to Non Volatile Memory reads).
 * *************************************************************************** *
 * - Call Hierarchy: main_Init()
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 1 (PID_SetRunningCurrent())
 * *************************************************************************** */
void PID_Init(void)
{
    /* Convert [mA] to [ADC-LSB] */
    PID_SetRunningCurrent(NV_RUNNING_CURR_LEVEL);

    l_u16PidHoldingThreshold = p_MulDivU16_U16byU16byU16(NV_HOLDING_CURR_LEVEL, C_GMCURR_DIV, Get_MCurrGain());

#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81339__)
    l_u16MotorRefVoltageADC =
        p_MulDivU16_U16byU16byU16(NV_VSUP_REF, C_VOLTGAIN_DIV, Get_MotorVoltGainF()) + Get_VsmOffset();
#elif defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
    l_u16MotorRefVoltageADC = p_MulDivU16_U16byU16byU16(NV_VSUP_REF, C_VOLTGAIN_DIV, Get_MotorVoltGainF());
#endif

    l_u16MinCorrectionRatio = NV_MIN_CORR_RATIO;
    l_u16MaxCorrectionRatio = NV_MAX_CORR_RATIO;

    /* PID Parameters for Speed-Error to Vector Amplitude (PID Control Gain-factor: (2 ^ C_GN_PID)) */
    sPIDpSE2VA.i16CoefP = (int16_t)(NV_PID_COEF_P << (C_GN_PID - 6U));          /* Speed-Error PID P-coefficient */
    sPIDpSE2VA.i16CoefI = (int16_t)(NV_PID_COEF_I << (C_GN_PID - 6U));          /* Speed-Error PID I-coefficient */
#if (_SUPPORT_PID_D_COEF != FALSE)
    sPIDpSE2VA.i16CoefD = (int16_t)(NV_PID_COEF_D << (C_GN_PID - 6U));          /* Speed-Error PID D-coefficient */
#endif /* (_SUPPORT_PID_D_COEF != FALSE) */
    sPIDpSE2VA.u16MinOutput = l_u16MinCorrectionRatio;                          /* Minimal Motor-PWM Duty-Cycle */
    sPIDpSE2VA.u32MaxOutput = (uint32_t)l_u16MaxCorrectionRatio;                /* Maximal Motor-PWM Duty-Cycle */
    sPIDpSE2VA.u32SumErrorMax = (((uint32_t)l_u16MaxCorrectionRatio) << C_GN_PID);   /* Maximal Motor-PWM Duty-Cycle * (1 << C_GN_PID) */
} /* End of PID_Init() */

/*!*************************************************************************** *
 * PID_Start()
 * \brief   Initialise the PID
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16Losses: Ohmic losses part (current & coil impedance) [10mV]
 * \param   [in] u16Bemf: BEMF voltage part (speed & motor constant) [10mV]
 * \return  (uint16_t) PWM Duty Cycle amplitude (u16CorrectionRatio)
 * *************************************************************************** *
 * \details -
 * Note:    This function is not allowed to be called during Non Volatile Memory Write
 *          (due to Non Volatile Memory reads).
 * *************************************************************************** *
 * - Call Hierarchy: MotorDriver_InitialPwmDutyCycle()
 * - Cyclomatic Complexity: 3+1
 * - Nesting: 2
 * - Function calling: 0 (all inline)
 * *************************************************************************** */
uint16_t PID_Start(uint16_t u16Losses)
{
    uint16_t u16MotorVoltage = Get_MotorVoltage();
    uint16_t u16ReferenceVoltage = NV_VSUP_REF;
    uint16_t u16CorrectionRatio;

    if (u16ReferenceVoltage == 0U)
    {
        u16ReferenceVoltage = C_REF_VOLTAGE;                                    /* Default reference voltage in [10mV]-units (MMP230912-1) */
    }
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__)
    l_u16PidLosses =
        p_MulDivU16_U16byU16byU16(u16Losses, (PWM_REG_PERIOD << C_PID_FACTOR),
                                  u16ReferenceVoltage) + ((2U * C_PWM_MIN_DC) << C_PID_FACTOR);
#elif defined (__MLX81160__) || defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
    l_u16PidLosses = p_MulDivU16_U16byU16byU16(u16Losses, (PWM_REG_PERIOD << C_PID_FACTOR), u16ReferenceVoltage);
#endif
    l_u16PidCtrlRatio = l_u16PidLosses;
    if (l_u16PidCtrlRatio > l_u16MaxCorrectionRatio)
    {
        /* Overflow */
        l_u16PidCtrlRatio = l_u16MaxCorrectionRatio;
    }

    sPIDpSE2VA.u32SumError = (((uint32_t)l_u16PidCtrlRatio) << C_GN_PID);
#if (_SUPPORT_PID_D_COEF != FALSE)
    sPIDpSE2VA.i16PrevError = 0;
#endif /* (_SUPPORT_PID_D_COEF != FALSE) */
    l_u16PID_CtrlCounter = 0U;                                                  /* Re-start Current-control PID */

    if (u16MotorVoltage != 0U)
    {
        u16CorrectionRatio = p_MulDivU16_U16byU16byU16(l_u16PidCtrlRatio, u16ReferenceVoltage, u16MotorVoltage);
        if (u16CorrectionRatio > l_u16MaxCorrectionRatio)
        {
            /* Overflow */
            u16CorrectionRatio = l_u16MaxCorrectionRatio;
        }
    }
    else
    {
        u16CorrectionRatio = l_u16PidCtrlRatio;
    }

    return (u16CorrectionRatio);
} /* End of PID_Start() */

#if (_SUPPORT_APP_TYPE == C_APP_RELAY)
/*!*************************************************************************** *
 * VoltageCorrection
 * \brief   Compensate Motor PWM Duty Cycle for voltage changes
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details Tuned for MLX-GNU V3.0.31 (MMP181206)
 * Note:    This function is not allowed to be called during Non Volatile Memory Write
 *          (due to Non Volatile Memory reads) (C_APP_POSITIONING_ACT).
 * *************************************************************************** *
 * - Call Hierarchy: ISR_CTIME0_3(), PID_Control()
 * - Cyclomatic Complexity: 5+1
 * - Nesting: 4
 * - Function calling: 2 (Get_RawVmotorF(), Set_CorrectionRatio())
 * *************************************************************************** */
uint16_t VoltageCorrection(void)
{
    register uint16_t u16NewCorrectionRatio = l_u16PidCtrlRatio;
    register uint16_t u16MotorVoltageADC = Get_RawVmotorF();
    if ( (u16MotorVoltageADC > 0U) && (l_u16MotorRefVoltageADC > 0U) )
    {
        /* Correct Motor PWM duty cycle instantly based on change of supply voltage */
        u16NewCorrectionRatio = p_MulDivU16_U16byU16byU16(l_u16MotorRefVoltageADC,
                                                          u16NewCorrectionRatio,
                                                          u16MotorVoltageADC);
        if ( (g_e8RelayStatus & (uint8_t)C_RELAY1_STATUS_MASK) != (uint8_t)C_RELAY1_STATUS_OFF)
        {
            if (u16NewCorrectionRatio > l_u16MaxCorrectionRatio)
            {
                /* Overflow */
                u16NewCorrectionRatio = l_u16MaxCorrectionRatio;
            }
            else if ( (g_e8RelayStatus & (uint8_t)C_RELAY1_STATUS_MASK) != (uint8_t)C_RELAY1_STATUS_HOLDING)
            {
                if (u16NewCorrectionRatio < l_u16MinCorrectionRatio)
                {
                    /* Underflow */
                    u16NewCorrectionRatio = l_u16MinCorrectionRatio;
                }
            }
            else if (u16NewCorrectionRatio < NV_MIN_HOLDCORR_RATIO)
            {
                /* Holding-mode: Underflow */
                u16NewCorrectionRatio = NV_MIN_HOLDCORR_RATIO;
            }
            else
            {
                /* Nothing */
            }
        }
        else
        {
            /* Nothing */
        }
    }
    return (u16NewCorrectionRatio);
} /* End of VoltageCorrection() */
#else
/*!*************************************************************************** *
 * VoltageCorrection
 * \brief   Compensate Motor PWM Duty Cycle for voltage changes
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details Tuned for MLX-GNU V3.0.31 (MMP181206)
 * Note:    This function is not allowed to be called during Non Volatile Memory Write
 *          (due to Non Volatile Memory reads) (C_APP_POSITIONING_ACT).
 * *************************************************************************** *
 * - Call Hierarchy: ISR_CTIME0_3(), PID_Control()
 * - Cyclomatic Complexity: 5+1
 * - Nesting: 4
 * - Function calling: 2 (Get_RawVmotorF(), Set_CorrectionRatio())
 * *************************************************************************** */
uint16_t VoltageCorrection(void)
{
    register uint16_t u16NewCorrectionRatio = l_u16PidCtrlRatio;
    register uint16_t u16MotorVoltageADC = Get_RawVmotorF();
    if ( (u16MotorVoltageADC > 0U) && (l_u16MotorRefVoltageADC > 0U) )
    {
        /* Correct Motor PWM duty cycle instantly based on change of supply voltage */
        u16NewCorrectionRatio = p_MulDivU16_U16byU16byU16(l_u16MotorRefVoltageADC,
                                                          u16NewCorrectionRatio,
                                                          u16MotorVoltageADC);
        if ( (g_e8SolenoidStatus & (uint8_t)C_SOLENOID_STATUS_MASK) != C_SOLENOID_STATUS_DEACTIVATED)
        {
            if (u16NewCorrectionRatio > l_u16MaxCorrectionRatio)
            {
                /* Overflow */
                u16NewCorrectionRatio = l_u16MaxCorrectionRatio;
            }
            else if ( (g_e8SolenoidStatus & (uint8_t)C_SOLENOID_STATUS_MASK) == C_SOLENOID_STATUS_ACTIVATED)
            {
                if (u16NewCorrectionRatio < l_u16MinCorrectionRatio)
                {
                    /* Underflow */
                    u16NewCorrectionRatio = l_u16MinCorrectionRatio;
                }
            }
            else if (u16NewCorrectionRatio < NV_MIN_HOLDCORR_RATIO)
            {
                /* Holding-mode: Underflow */
                u16NewCorrectionRatio = NV_MIN_HOLDCORR_RATIO;
            }
            else
            {
                /* Nothing */
            }
        }
        else
        {
            /* Nothing */
        }
    }
    return (u16NewCorrectionRatio);
} /* End of VoltageCorrection() */
#endif

/*!*************************************************************************** *
 * PID_Control
 * \brief   Current Control or Speed Control (_SUPPORT_HALL_LATCH)
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details -
 * Note:    This function is not allowed to be called during Non Volatile Memory Write
 *          (due to Non Volatile Memory reads) (C_APP_POSITIONING_ACT or SINGLE_COIL_PWM).
 * *************************************************************************** *
 * - Call Hierarchy: main()
 * - Cyclomatic Complexity: 13+1
 * - Nesting: 8
 * - Function calling: 4 (VoltageCorrection(), Get_MicroStepIdx(),
 *                        MotorDriver_3Phase(), p_PID_Control())
 * *************************************************************************** */
void PID_Control(void)
{
    if ( (g_e8SolenoidStatus & (uint8_t)C_SOLENOID_STATUS_MASK) != C_SOLENOID_STATUS_DEACTIVATED)  /* Stop-mode & holding-current required */
    {
        /* Running-mode */
        uint16_t u16PidPeriod = NV_PID_RUNNINGCTRL_PER;

        if ( (u16PidPeriod != 0U) && (l_u16PID_CtrlCounter >= u16PidPeriod) )   /* Period */
        {
            /* Periodic update (PWM-DC/Current/Speed-Control) */
            int16_t i16ControlError = 0;

#if (_DEBUG_PID_CONTROL != FALSE)
            DEBUG_SET_IO_E();
#endif /* (_DEBUG_PID_CONTROL != FALSE) */

            sPIDpSE2VA.u16MinOutput = l_u16MinCorrectionRatio;

            /* Current control */
            if ( (g_e8SolenoidStatus & (uint8_t)C_SOLENOID_STATUS_MASK) == C_SOLENOID_STATUS_HOLDING)
            {
                /* Holding mode */
                i16ControlError = (int16_t)(l_u16PidHoldingThreshold - Get_MotorCurrentLPF());
                sPIDpSE2VA.u16MinOutput = NV_MIN_HOLDCORR_RATIO;
            }
            else if ( (g_e8SolenoidStatus & (uint8_t)C_SOLENOID_STATUS_MASK) == C_SOLENOID_STATUS_ACTIVATED)
            {
                /* Running-mode */
                i16ControlError = (int16_t)(l_u16ActCurrRunMax_LSB - Get_MotorCurrentLPF());
            }
            else
            {
                /* Nothing */
            }

            /* PID Motor PWM Duty Cycle correction, based on error and PID-coefficients */
            if (i16ControlError != 0)
            {
                l_u16PidCtrlRatio = (uint16_t)p_PID_Control(i16ControlError, (void *)&sPIDpSE2VA);
            }
            l_u16PID_CtrlCounter = 0U;

            /* Update motor-driver PWM duty-cycle in case of holding-mode with coil-current */
            /* Stop-mode & holding-current required */
            if ( (g_e8SolenoidStatus == C_SOLENOID_STATUS_HOLDING) && (Get_MotorHoldingCurrState() != FALSE) )
            {
                Set_CorrectionRatio(VoltageCorrection());
                SolenoidDriverSetPhase();
            }

#if (_DEBUG_PID_CONTROL != FALSE)
            DEBUG_CLR_IO_E();
#endif /* (_DEBUG_PID_CONTROL != FALSE) */
        }
    }

} /* End of PID_Control() */

#endif /* (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT) */

/* EOF */
