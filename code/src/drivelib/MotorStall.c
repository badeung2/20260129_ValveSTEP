/*!************************************************************************** *
 * \file        MotorStall.c
 * \brief       MLX813xx Motor stall detectors handling
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
 *           -# StallDetectorEna()
 *           -# MotorStallInitA()
 *           -# MotorStallSwitchOpen2CloseA()
 *           -# MotorStallCheckA()
 *           -# MotorStallInitO()
 *           -# MotorStallCheckO()
 *           -# MotorStallInitS()
 *           -# MotorStallCheckS()
 *           -# MotorStallInitP()
 *           -# MotorStallCheckP()
 *           -# LinDiag_MotorStall()
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
 * ************************************************************************** */

/*!*************************************************************************** */
/*                           INCLUDES                                          */
/* *************************************************************************** */
#include "AppBuild.h"                                                           /* Application Build */

#include "../ActADC.h"                                                          /* Application ADC support */

#include "drivelib/GlobalVars.h"                                                /* Global Variables support */
#if (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT)
#include "drivelib/MotorDriver.h"                                               /* Motor driver support */
#include "drivelib/MotorDriverTables.h"                                         /* Wave-form vector tables */
#elif (_SUPPORT_APP_TYPE == C_APP_SOLENOID)
#include "drivelib/SolenoidDriver.h"                                            /* Solenoid Driver support */
#endif /* (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT) */
#include "drivelib/MotorStall.h"                                                /* Motor Stall Detectors */
#include "drivelib/PID_Control.h"                                               /* PID support */
#include "camculib/private_mathlib.h"                                           /* Private Math Library support */

#if (LIN_COMM != FALSE) && (_SUPPORT_MLX_DEBUG_MODE != FALSE)
#include "commlib/LIN_Communication.h"                                          /* LIN Communication support */
#endif /* (LIN_COMM != FALSE) && (_SUPPORT_MLX_DEBUG_MODE != FALSE) */

#if (_SUPPORT_STALLDET_P != FALSE) && (_SUPPORT_TRIAXIS_MLX90377 != FALSE)
#include "senselib/Triaxis_MLX90377.h"                                          /* (PWM/SENT/SPC) Triaxis MLX90377 support */
#endif /* (_SUPPORT_STALLDET_P != FALSE) && (_SUPPORT_TRIAXIS_MLX90377 != FALSE) */
#if (_SUPPORT_STALLDET_P != FALSE) && (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
#include "senselib/Triaxis_MLX9038x.h"                                          /* (ANA) Triaxis MLX9038x support */
#endif /* (_SUPPORT_STALLDET_P != FALSE) && (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */

/*!*************************************************************************** */
/*                           DEFINITIONS                                       */
/* *************************************************************************** */
#if (_SUPPORT_STALLDET_O != FALSE)
#ifdef C_MOVAVG_SSZ
#define C_STALL_CURR_THRSHLD    (uint16_t)(p_MulU32_U16byU16(Get_MotorCurrentMovAvgxN(), \
                                                             NV_STALL_O_THRSHLD) >> \
                                           (C_STALL_THRESHOLD_SDIV + C_MOVAVG_SSZ))
#else  /* C_MOVAVG_SSZ */
#define C_STALL_CURR_THRSHLD    (p_MulDivU16_U16byU16byU16(Get_MotorCurrentMovAvgxN(), NV_STALL_O_THRSHLD, \
                                                           (C_STALL_THRESHOLD_DIV * C_MOVAVG_SZ))
#endif /* C_MOVAVG_SSZ */
#endif /* (_SUPPORT_STALLDET_O != FALSE) */

/*!*************************************************************************** */
/*                           GLOBAL & LOCAL VARIABLES                          */
/* *************************************************************************** */
#pragma space dp                                                                /* __TINY_SECTION__ */
#if (_SUPPORT_STALLDET_A != FALSE)
uint16_t l_u16MotorCurrentStallThrshldxN;                                       /*!< Stall-detector current-threshold x 4..16 */
volatile uint8_t l_u8StallCountA = 0U;                                          /*!< Stall detector "A" Counter */
static uint8_t l_u8StallWidthA = C_STALL_A_WIDTH;                               /*!< Stall detector "A" Width */
#endif /* (_SUPPORT_STALLDET_A != FALSE) */
#if (_SUPPORT_STALLDET_H != FALSE)
static uint8_t l_u8StallCountH = 0U;                                            /*!< Stall "H" counter */
#endif /* (_SUPPORT_STALLDET_H != FALSE) */
#if (_SUPPORT_STALLDET_LA != FALSE)
int16_t l_i16StallThresholdLA;                                                  /*!< Stall-detector Load-angle threshold x 3 degrees (Actual Speed) */
int16_t l_i16StallThresholdLA_Max;                                              /*!< Stall-detector Load-angle threshold x 3 degrees (Max Speed) */
int16_t l_i16ActLoadAngleLPF = 0;                                               /*!< Actual Load-angle LPF */
int32_t l_i32ActLoadAngleLPF = 0;                                               /*!< LPF Actual Load-angle (internal variable) */
volatile uint8_t l_u8StallCountLA = 0U;                                         /*!< Stall detector "LA" Counter */
#endif /* (_SUPPORT_STALLDET_LA != FALSE) */
#if (_SUPPORT_FOC_MODE != FOC_MODE_NONE) && (_SUPPORT_STALLDET_FLUX != FALSE)
uint16_t l_u16StallThresholdFlux;                                               /*!< Stall-detector Flux threshold */
volatile uint8_t l_u8StallCountFlux = 0U;                                       /*!< Stall detector "Flux" Counter */
#endif /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) && (_SUPPORT_STALLDET_FLUX != FASLE) */
#if (_SUPPORT_STALLDET_P != FALSE) && ((_SUPPORT_CURRSPIKE_POSITION != FALSE) || ((_SUPPORT_POTI != FALSE) && (_SUPPORT_POTI_MODE == C_POTI_MODE_CLOSED_LOOP)) || (_SUPPORT_TRIAXIS_MLX90377 != FALSE))
static uint8_t l_e8StallMotorDirectionCCW = (uint8_t)C_MOTOR_DIR_UNKNOWN;       /*!< Stall motor direction (position based on outer-shaft) */
#endif /* (_SUPPORT_STALLDET_P != FALSE) && ((_SUPPORT_CURRSPIKE_POSITION != FALSE) || ((_SUPPORT_POTI != FALSE) && (_SUPPORT_POTI_MODE == C_POTI_MODE_CLOSED_LOOP)) || (_SUPPORT_TRIAXIS_MLX90377 != FALSE)) */
#pragma space none                                                              /* __TINY_SECTION__ */

#pragma space nodp                                                              /* __NEAR_SECTION__ */
#if (_SUPPORT_STALLDET_O != FALSE)
uint8_t l_u8StallCountO = 0U;                                                   /*!< Stall detector "O" Counter */
int8_t l_i8StallOscIdx = 0;                                                     /*!< Stall oscillation index */
#if (_SUPPORT_PWM_MODE == BIPOLAR_PWM_SINGLE_INDEPENDENT_GND)
int16_t l_iu16CurrentCoilA[C_CURROSC_SZ];                                       /*!< Stall oscillation current coil-A buffer */
int16_t l_iu16CurrentCoilB[C_CURROSC_SZ];                                       /*!< Stall oscillation current coil-B buffer */
#else  /* (_SUPPORT_PWM_MODE == BIPOLAR_PWM_SINGLE_INDEPENDENT_GND) */
uint16_t l_au16StallOscCurrent[C_MICROSTEP_PER_FULLSTEP];                       /*!< Stall oscillation current buffer */
#endif /* (_SUPPORT_PWM_MODE == BIPOLAR_PWM_SINGLE_INDEPENDENT_GND) */
#endif /* (_SUPPORT_STALLDET_O != FALSE) */

#if (_SUPPORT_STALLDET_S != FALSE)
uint16_t l_u16LastMotorPWMdc = 0U;                                              /*!< Last Motor micro-step PWM Duty Cycle */
uint16_t l_u16LastMotorSpeed = 0U;                                              /*!< Last Motor micro-step Speed */
uint8_t l_u8StallCountS = 0U;                                                   /*!< Stall Detector "S" Counter */
#endif /* (_SUPPORT_STALLDET_S != FALSE) */

#if (_SUPPORT_STALLDET_P != FALSE)
static uint16_t l_u16DetectorDelayP;                                            /*!< Stall detector delay */
static volatile uint16_t l_u16StallCountP = 0U;                                 /*!< Stall "P" Motor Count */
#if (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
uint16_t l_u16LastAngle = 0U;                                                   /*!< Stall "P" Motor last angle */
#else  /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */
uint16_t l_u16LastPosition = 0U;                                                /*!< Stall "P" Motor last position */
#endif /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */
#endif /* (_SUPPORT_STALLDET_P != FALSE) */
#pragma space none                                                              /* __NEAR_SECTION__ */

/*!*************************************************************************** *
 * StallDetectorEna
 * \brief   Get Stall Detector Enable "mask"
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  (uint8_t) Stall-detector enable mask.
 * *************************************************************************** *
 * \details
 * *************************************************************************** *
 * - Call Hierarchy: ()
 * - Cyclomatic Complexity: 1..5+1
 * - Nesting: 1
 * - Function calling: 0
 * *************************************************************************** */
uint8_t StallDetectorEna(void)
{
    uint8_t e8StallDetectorEna = C_STALLDET_ALL;                                /* All Application-code supported stall detectors */
#if (_SUPPORT_STALLDET_A != FALSE)
    /* Stall detector "A" is supported; Check EE if detector "A" is enabled */
    if ((NV_STALL_A == FALSE) || (NV_STALL_A_WIDTH == 0x00U) || (NV_STALL_A_THRSHLD == 0x00U))
    {
        e8StallDetectorEna &= ~C_STALLDET_A;                                    /* Stall detector "A" is disabled */
    }
#endif /* _SUPPORT_STALLDET_O != FALSE) */
#if (_SUPPORT_STALLDET_O != FALSE)
    /* Stall detector "O" is supported; Check EE if detector "O" is enabled */
    if ((NV_STALL_O == FALSE) || (NV_STALL_O_WIDTH == 0x00U) || (NV_STALL_O_THRSHLD == 0x00U))
    {
        e8StallDetectorEna &= ~C_STALLDET_O;                                    /* Stall detector "O" is disabled */
    }
#endif /* _SUPPORT_STALLDET_O != FALSE) */
#if (_SUPPORT_STALLDET_P != FALSE)
    /* Stall detector "P" is supported; Check EE if detector "P" is enabled */
    if ((NV_STALL_P == FALSE) || (NV_STALL_P_THRSHLD == 0x00U) )
    {
        e8StallDetectorEna &= ~C_STALLDET_P;                                    /* Stall detector "P" is disabled */
    }
#endif /* _SUPPORT_STALLDET_P != FALSE) */
#if (_SUPPORT_STALLDET_S != FALSE)
    /* Stall detector "S" is supported; Check EE if detector "S" is enabled */
    if ((NV_STALL_S == FALSE) || (NV_STALL_S_WIDTH == 0x00U) || (NV_STALL_S_THRSHLD == 0x00U))
    {
        e8StallDetectorEna &= ~C_STALLDET_S;                                    /* Stall detector "S" is disabled */
    }
#endif /* _SUPPORT_STALLDET_S != FALSE) */
#if (_SUPPORT_STALLDET_LA != FALSE)
    /* Stall detector "S" is supported; Check EE if detector "S" is enabled */
    if ((NV_STALL_LA_THRSHLD == 0x00U) || (NV_STALL_LA_WIDTH == 0x00U))
    {
        e8StallDetectorEna &= ~C_STALLDET_LA;                                   /* Stall detector "LA" is disabled */
    }
#endif /* _SUPPORT_STALLDET_LA != FALSE) */
    return (e8StallDetectorEna);
} /* End of StallDetectorEna() */

#if (_SUPPORT_STALLDET_A != FALSE)
/*!*************************************************************************** *
 * MotorStallInitA
 * \brief   Initialise Stall detector "A"
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details
 * *************************************************************************** *
 * - Call Hierarchy: MotorDriverStart()
 * - Cyclomatic Complexity: 1+1
 * - Nesting: 1
 * - Function calling: 0
 * *************************************************************************** */
void MotorStallInitA(void)
{
#if (_SUPPORT_STALLDET_A_MODE == C_STALL_THRSHLD_STC) && (_SUPPORT_FOC_MODE == FOC_MODE_NONE)
    uint16_t u16Threshold;
#endif /* (_SUPPORT_FOC_MODE == FOC_MODE_NONE) */

    g_u8StallTypeComm = (uint8_t)C_STALL_NOT_FOUND;                             /* Used for communication */
    l_u8StallCountA = 0U;                                                       /* Stall-counter */
    l_u8StallWidthA = NV_STALL_A_WIDTH;
#if (_SUPPORT_FOC_MODE != FOC_MODE_NONE)
    l_u16MotorCurrentStallThrshldxN = p_MulDivU16_U16byU16byU16(Get_ActCurrRunMax_LSB(),
                                                                (NV_STALL_A_THRSHLD + C_STALL_THRESHOLD_DIV),
                                                                (C_STALL_THRESHOLD_DIV >> C_MOVAVG_SSZ));
#else  /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) */
#if (_SUPPORT_STALLDET_A_MODE == C_STALL_THRSHLD_STC)
#if (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE)
    if (NV_STALL_SPEED_DEPENDED != 0U)
    {
        u16Threshold = (NV_STALL_A_THRSHLD + (C_STALL_THRESHOLD_DIV - 8U)) + ((uint16_t)g_u8MotorStatusSpeed << 3);  /* Speed depended Threshold */
    }
    else
#endif /* (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) */
    {
        u16Threshold = (NV_STALL_A_THRSHLD + C_STALL_THRESHOLD_DIV);            /* Fixed Threshold */
    }
#if defined (C_MOVAVG_SSZ)
#if (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM_BIPOLAR)
    l_u16MotorCurrentStallThrshldxN =
        (uint16_t)(p_MulU32_U16byU16(Get_ActCurrRunMax_LSB(), u16Threshold) >> (C_STALL_THRESHOLD_SDIV - C_MOVAVG_SSZ));  /* Above torque-current */
#else  /* (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM_BIPOLAR) */
    l_u16MotorCurrentStallThrshldxN =
        (uint16_t)(p_MulU32_U16byU16(Get_ActCurrRunMax_LSB(), u16Threshold) >> (C_STALL_THRESHOLD_SDIV - C_MOVAVG_SSZ));  /* Above torque-current */
#endif /* (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM_BIPOLAR) */
#else  /* defined (C_MOVAVG_SSZ) */
    l_u16MotorCurrentStallThrshldxN =
        (uint16_t)(p_MulU32_U16byU16(Get_ActCurrRunMax_LSB(), (u16Threshold * C_MOVAVG_SZ)) >> C_STALL_THRESHOLD_SDIV);  /* Above torque-current */
#endif /* defined (C_MOVAVG_SSZ) */
#endif /* (_SUPPORT_STALLDET_A_MODE == C_STALL_THRSHLD_STC) */
#endif /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) */

} /* End of MotorStallInitA() */

#if (_SUPPORT_FOC_MODE != FOC_MODE_NONE)
/*!*************************************************************************** *
 * MotorStallSwitchOpen2CloseA
 * \brief   Initialise Stall detector "A" for FOC
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details
 * *************************************************************************** *
 * - Call Hierarchy: EXT0_IT(), MotorDriverStart(), SwitchOpen2Close()
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 1 (Get_ActCurrRunMax_LSB())
 * *************************************************************************** */
void MotorStallSwitchOpen2CloseA(void)
{
#if (_SUPPORT_FOC_STALL_A_THRSHLD != FALSE)
    /* Max running current with stall "A" threshold uplift is threshold */
#if defined (C_STALL_THRESHOLD_SDIV)
    l_u16MotorCurrentStallThrshldxN =
        (uint16_t) (p_MulU32_U16byU16(Get_ActCurrRunMax_LSB(),
                                      (NV_STALL_A_THRSHLD + C_STALL_THRESHOLD_DIV)) >> (C_STALL_THRESHOLD_SDIV - C_MOVAVG_SSZ));
#else
    l_u16MotorCurrentStallThrshldxN = p_MulDivU16_U16byU16byU16(Get_ActCurrRunMax_LSB(),
                                                                (NV_STALL_A_THRSHLD + C_STALL_THRESHOLD_DIV),
                                                                (C_STALL_THRESHOLD_DIV >> C_MOVAVG_SSZ));
#endif
#else  /* (_SUPPORT_FOC_STALL_A_THRSHLD != FALSE) */
    /* Max running current is stall "A" threshold */
    l_u16MotorCurrentStallThrshldxN = (Get_ActCurrRunMax_LSB() << C_MOVAVG_SSZ);
#endif /* (_SUPPORT_FOC_STALL_A_THRSHLD != FALSE) */
} /* End of MotorStallSwitchOpen2CloseA() */
#endif /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) */

/*!*************************************************************************** *
 * MotorStallCheckA
 * \brief   Check if motor is stalled
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  (uint16_t) C_STALL_NOT_FOUND: No stall found
 *                     C_STALL_FOUND: Stall have been found
 * *************************************************************************** *
 * \details Stall detector "A" is based on fast current increase
 * *************************************************************************** *
 * - Call Hierarchy: ISR_CTIMER0_3()
 * - Cyclomatic Complexity: 3+1
 * - Nesting: 3
 * - Function calling: 3 (Get_StallDetectorDelay(), Get_MotorCurrentMovAvgxN(),
 *                        Get_ActCurrRunMax_LSB())
 * *************************************************************************** */
#if (_SUPPORT_FOC_MODE != FOC_MODE_NONE) && (_SUPPORT_RAM_FUNC != FALSE)
uint16_t MotorStallCheckA(void) __attribute__ ((section(".ramfunc"))) __attribute__ ((aligned(8)));
#elif (_SUPPORT_FOC_MODE != FOC_MODE_NONE) && (_SUPPORT_OPTIMIZE_FOR_SPEED != FALSE)
uint16_t MotorStallCheckA(void) __attribute__((aligned(8)));
#endif /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) && _SUPPORT_OPTIMIZE_FOR_SPEED */
uint16_t MotorStallCheckA(void)
{
    uint16_t u16StallResult = C_STALL_NOT_FOUND;

    if ( (Get_StartupDelay() == 0U) &&
         (Get_MotorCurrentMovAvgxN() > (C_MIN_MOTORCURRENT << 4)) )
    {
        /* Running stall detection, based on current increase */
        /* A actuator running at it's target speed, generate BEMF. When the rotor
         * blocks, the BEMF drops to zero, and therefore the motor current increases.
         * This increase is monitored by calculating the difference (delta) between
         * a LPF filter (slowly increase) and the actual motor current. If this delta
         * increases above a specified threshold, stall is detected. */
#if (_SUPPORT_STALLDET_A_MODE == C_STALL_THRSHLD_DYN) && (_SUPPORT_FOC_MODE == FOC_MODE_NONE)
        uint16_t u16Threshold;
        if (NV_STALL_SPEED_DEPENDED != 0U)
        {
            u16Threshold = (NV_STALL_A_THRSHLD + (C_STALL_THRESHOLD_DIV - 8U)) + ((uint16_t)g_u8MotorStatusSpeed << 3);   /* Speed depended Threshold */
        }
        else
        {
            u16Threshold = (NV_STALL_A_THRSHLD + C_STALL_THRESHOLD_DIV);        /* Fixed Threshold */
        }
        l_u16MotorCurrentStallThrshldxN =
            (uint16_t)(p_MulU32_U16byU16(Get_MotorCurrentLPF(),
                                         u16Threshold) >> (C_STALL_THRESHOLD_SDIV - C_MOVAVG_SSZ));                                                        /* Above torque-current */
#endif /* (_SUPPORT_STALLDET_A_MODE == C_STALL_THRSHLD_DYN) && (_SUPPORT_FOC_MODE == FOC_MODE_NONE) */
        if (l_u16MotorCurrentStallThrshldxN > Get_MotorCurrentMovAvgxN() )
        {
            l_u8StallCountA = p_DecNzU8(l_u8StallCountA);
        }
        else
        {
            uint8_t u8StallCountA = l_u8StallCountA + 1U;
            l_u8StallCountA = u8StallCountA;
            if (u8StallCountA >= l_u8StallWidthA)
            {
                /* Real stall */
                u16StallResult = C_STALL_FOUND;
            }
        }
    }
    return (u16StallResult);
} /* End of MotorStallCheckA() */
#endif /* (_SUPPORT_STALLDET_A != FALSE) */

#if (_SUPPORT_STALLDET_H != FALSE)
/*!*************************************************************************** *
 * MotorStallInitH
 * \brief   Initialise Stall detector "H"
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details
 * *************************************************************************** *
 * - Call Hierarchy: MotorDriverStart()
 * - Cyclomatic Complexity: 1+1
 * - Nesting: 1
 * - Function calling: 0
 * *************************************************************************** */
void MotorStallInitH(void)
{
    l_u8StallCountH = 0U;                                                       /* Stall-counter */
} /* End of MotorStallInitH() */

#if (_SUPPORT_MICRO_STEP_COMMUTATION != FALSE)
/*!*************************************************************************** *
 * MotorStallCheckH
 * \brief   Check if motor is stalled
 * \author  mmp
 * *************************************************************************** *
 * \param   u16lMicroStepCntPerHallCommut: Number of micro-steps before last Hall-Latch event.
 * \return  (uint16_t) C_STALL_NOT_FOUND: No stall found
 *                     C_STALL_FOUND: Stall have been found
 * *************************************************************************** *
 * \details Stall detector "H" is based on micro-steps between hall-latch commutation
 * *************************************************************************** *
 * - Call Hierarchy: ISR_CTIMER0_3()
 * - Cyclomatic Complexity: 3+1
 * - Nesting: 3
 * - Function calling: 1 (Get_StallDetectorDelay())
 * *************************************************************************** */
uint16_t MotorStallCheckH(uint16_t u16lMicroStepCntPerHallCommut)
{
    uint16_t u16StallResult = C_STALL_NOT_FOUND;

    if (Get_StartupDelay() == 0U)
    {
        if (u16lMicroStepCntPerHallCommut > C_STALL_H_THRESHOLD)
        {
            uint8_t u8StallCountH = l_u8StallCountH + 1U;
            l_u8StallCountH = u8StallCountH;
            if (u8StallCountH >= C_STALL_H_WIDTH)
            {
                /* Real stall */
                u16StallResult = C_STALL_FOUND;
            }
        }
        else if (u16lMicroStepCntPerHallCommut == 0U)
        {
            l_u8StallCountH = p_DecNzU8(l_u8StallCountH);
        }
        else
        {
            /* Nothing */
        }
    }
    return (u16StallResult);
} /* End of MotorStallCheckH() */
#else  /* (_SUPPORT_MICRO_STEP_COMMUTATION != FALSE) */
/*!*************************************************************************** *
 * MotorStallCheckH
 * \brief   Check if motor is stalled
 * \author  mmp
 * *************************************************************************** *
 * \param   u16HallLatchEvent: FALSE: No Hall-Latch event; TRUE: Hall-Latch-event occurred
 * \return  (uint16_t) C_STALL_NOT_FOUND: No stall found
 *                     C_STALL_FOUND: Stall have been found
 * *************************************************************************** *
 * \details Stall detector "H" is based on micro-steps between hall-latch commutation
 * *************************************************************************** *
 * - Call Hierarchy: ISR_CTIMER0_3()
 * - Cyclomatic Complexity: 3+1
 * - Nesting: 3
 * - Function calling: 1 (Get_StallDetectorDelay())
 * *************************************************************************** */
uint16_t MotorStallCheckH(uint16_t u16HallLatchEvent)
{
    uint16_t u16StallResult = C_STALL_NOT_FOUND;

    if (Get_StartupDelay() == 0U)
    {
        if (u16HallLatchEvent == FALSE)
        {
            uint8_t u8StallCountH = l_u8StallCountH + 1U;
            l_u8StallCountH = u8StallCountH;
            if (u8StallCountH >= C_STALL_H_WIDTH)
            {
                /* Real stall */
                u16StallResult = C_STALL_FOUND;
            }
        }
        else
        {
            l_u8StallCountH = p_DecNzU8(l_u8StallCountH);
        }
    }
    return (u16StallResult);
} /* End of MotorStallCheckH() */
#endif /* (_SUPPORT_MICRO_STEP_COMMUTATION != FALSE) */
#endif /* (_SUPPORT_STALLDET_H != FALSE) */

#if (_SUPPORT_STALLDET_O != FALSE)
/*!*************************************************************************** *
 * MotorStallInitO()
 * \brief   Initialise Stall detector "O"
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details -
 * *************************************************************************** *
 * - Call Hierarchy: MotorDriverStart()
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 0
 * *************************************************************************** */
void MotorStallInitO(void)
{
#if (_DEBUG_STALL_O != FALSE)
    DEBUG_CLR_IO_C();
#endif /* (_DEBUG_STALL_O != FALSE) */
    l_u8StallCountO = 0U;
#if (_SUPPORT_PWM_MODE == BIPOLAR_PWM_SINGLE_INDEPENDENT_GND)
    l_i8StallOscIdx = (int8_t)-C_CURROSC_SZ;  /*lint -e501 */
#else
    l_i8StallOscIdx = (int8_t)-C_MICROSTEP_PER_FULLSTEP;  /*lint !e501 */
#endif /* (_SUPPORT_PWM_MODE == BIPOLAR_PWM_SINGLE_INDEPENDENT_VSM) || (_SUPPORT_PWM_MODE == BIPOLAR_PWM_SINGLE_INDEPENDENT_GND) || (_SUPPORT_PWM_MODE == BIPOLAR_PWM_SINGLE_MIRRORSPECIAL) */
} /* End of MotorStallInitO() */

/*!*************************************************************************** *
 * MotorStallCheckO()
 * \brief   Check if motor is stalled
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  (uint16_t) C_STALL_NOT_FOUND: No stall found
 *                     C_STALL_FOUND: Stall have been found
 * *************************************************************************** *
 * \details Stall detector "O" is based on current oscillations
 * *************************************************************************** *
 * - Call Hierarchy: ISR_CTIMER0_3()
 * - Cyclomatic Complexity: 6+1
 * - Nesting: 4
 * - Function calling: 2 (Get_StartupDelay(), Get_MotorCurrentMovAvgxN())
 * *************************************************************************** */
uint16_t MotorStallCheckO(void)
{
    uint16_t u16Result = C_STALL_NOT_FOUND;

#if (_SUPPORT_PWM_MODE == BIPOLAR_PWM_SINGLE_INDEPENDENT_GND)
    /* Dual motor-driver current measurement. Stall based on current per coil (A & B) */
    if (l_i8StallOscIdx < 0)
    {
        /* Fill buffer with current samples, for each coil a separate buffer */
        l_iu16CurrentCoilA[C_CURROSC_SZ + l_i8StallOscIdx] = g_i16MotorCurrentCoilA;
        l_iu16CurrentCoilB[C_CURROSC_SZ + l_i8StallOscIdx] = g_i16MotorCurrentCoilB;
        l_i8StallOscIdx++;
    }
    else
    {
        /* Determine difference between current measurements now and half a (electric) rotation backwards */
#if (_DEBUG_STALL_O != FALSE)
        DEBUG_SET_IO_C();
#endif /* (_DEBUG_STALL_O != FALSE) */
        int16_t i16CurrentCoilA = g_i16MotorCurrentCoilA;                       /* Coil A actual-current */
        int16_t i16CurrentCoilB = g_i16MotorCurrentCoilB;                       /* Coil B actual-current */
        int16_t i16CurrentCoilA_Prev = l_iu16CurrentCoilA[l_i8StallOscIdx];     /* Coil A previous-current */
        l_iu16CurrentCoilA[l_i8StallOscIdx] = i16CurrentCoilA;
        if (i16CurrentCoilA_Prev > i16CurrentCoilA)
        {
            i16CurrentCoilA = i16CurrentCoilA_Prev - i16CurrentCoilA;           /* Delta current Coil-A */
        }
        else
        {
            i16CurrentCoilA = i16CurrentCoilA - i16CurrentCoilA_Prev;
        }
        int16_t i16CurrentCoilB_Prev = l_iu16CurrentCoilB[l_i8StallOscIdx];     /* Coil B previous-current */
        l_iu16CurrentCoilB[l_i8StallOscIdx] = i16CurrentCoilB;
        if (i16CurrentCoilB_Prev > i16CurrentCoilB)
        {
            i16CurrentCoilB = i16CurrentCoilB_Prev - i16CurrentCoilB;           /* Delta current Coil-B */
        }
        else
        {
            i16CurrentCoilB = i16CurrentCoilB - i16CurrentCoilB_Prev;
        }
        l_i8StallOscIdx = ((l_i8StallOscIdx + 1U) & (C_CURROSC_SZ - 1U));
        if (Get_StartupDelay() == 0U)
        {
            if ( ((uint16_t)i16CurrentCoilA > C_STALL_CURR_THRSHLD) ||
                 ((uint16_t)i16CurrentCoilB > C_STALL_CURR_THRSHLD) )
            {
#if (_DEBUG_STALL_O != FALSE)
                DEBUG_TOG_IO_C();
#endif /* (_DEBUG_STALL_O != FALSE) */
                l_u8StallCountO++;                                              /* Suspect current oscillation due to stall */
                if (l_u8StallCountO >= NV_STALL_O_WIDTH)                        /* N uSteps out of Full-step of uSteps */
                {
                    u16Result = C_STALL_FOUND;
                }
            }
            else if (l_u8StallCountO != 0U)
            {
                l_u8StallCountO--;
            }
        }
    }

#else  /* (_SUPPORT_PWM_MODE == BIPOLAR_PWM_SINGLE_INDEPENDENT_VSM) || (_SUPPORT_PWM_MODE == BIPOLAR_PWM_SINGLE_INDEPENDENT_GND) || (_SUPPORT_PWM_MODE == BIPOLAR_PWM_SINGLE_MIRRORSPECIAL) */
    /* Single motor-driver current measurement */
    if (l_i8StallOscIdx < 0)
    {
        /* Fill buffer with motor-current sample */
        l_au16StallOscCurrent[C_MICROSTEP_PER_FULLSTEP + l_i8StallOscIdx] = g_u16MotorCurrentPeak;
        l_i8StallOscIdx++;
    }
    else
    {
        uint16_t u16LastCurr = g_u16MotorCurrentPeak;                           /* Last current measured */
        uint16_t u16CompCurr = l_au16StallOscCurrent[l_i8StallOscIdx];
        l_au16StallOscCurrent[l_i8StallOscIdx] = u16LastCurr;
        l_i8StallOscIdx = ((l_i8StallOscIdx + 1U) & (C_MICROSTEP_PER_FULLSTEP - 1U));
        if (Get_StartupDelay() == 0U)
        {
            uint16_t u16DiffCurr;                                               /* (abs) Difference between last current and one full-step back */
            if (u16LastCurr > u16CompCurr)
            {
                u16DiffCurr = u16LastCurr - u16CompCurr;
            }
            else
            {
                u16DiffCurr = u16CompCurr - u16LastCurr;
            }
            if (u16DiffCurr > C_STALL_CURR_THRSHLD)                             /* x% of MovAvg value */
            {
                l_u8StallCountO++;                                              /* Suspect current oscillation due to stall */
                if (l_u8StallCountO >= NV_STALL_O_WIDTH)                        /* 1-8 uSteps out of 16 uSteps */
                {
                    u16Result = C_STALL_FOUND;
                }
            }
            else if (l_u8StallCountO != 0U)
            {
                l_u8StallCountO--;
            }
        }
    }
#endif /* (_SUPPORT_PWM_MODE == BIPOLAR_PWM_SINGLE_INDEPENDENT_VSM) || (_SUPPORT_PWM_MODE == BIPOLAR_PWM_SINGLE_INDEPENDENT_GND) || (_SUPPORT_PWM_MODE == BIPOLAR_PWM_SINGLE_MIRRORSPECIAL) */
    return (u16Result);

} /* End of MotorStallCheckO() */
#endif /* (_SUPPORT_STALLDET_O != FALSE) */

#if (_SUPPORT_STALLDET_S != FALSE)
/*!*************************************************************************** *
 * MotorStallInitS
 * \brief   Initialise Stall detector "S"
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details
 * *************************************************************************** *
 * - Call Hierarchy: MotordriverStart()
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 0
 * *************************************************************************** */
void MotorStallInitS(void)
{
    l_u16LastMotorPWMdc = 0U;
    l_u16LastMotorSpeed = 0U;
    l_u8StallCountS = 0U;
} /* End of MotorStallInitS */

/*!*************************************************************************** *
 * MotorStallCheckS
 * \brief   Initialise Stall detector "S"
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  (uint16) C_STALL_NOT_FOUND: No stall found
 *                   C_STALL_FOUND: Stall have been found
 * *************************************************************************** *
 * \details Check if motor is stalled
 *          Stall detector "S" is based on increasing motor current (motor PWM-DC)
 *          and decreasing motor speed, below minimum speed.
 * *************************************************************************** *
 * - Call Hierarchy: MotorDriverStart()
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 0
 * *************************************************************************** */
uint16_t MotorStallCheckS(void)
{
    uint16_t u16StallResult = C_STALL_NOT_FOUND;
#if defined (C_STALL_THRESHOLD_SDIV)
    uint16_t u16MinSpeed = (uint16_t) (p_MulU32_U16byU16(g_u16LowSpeedRPM, NV_STALL_S_THRSHLD) >> C_STALL_THRESHOLD_SDIV);  /* MMP190412-1 */
#else
    uint16_t u16MinSpeed = p_MulDivU16_U16byU16byU16(g_u16LowSpeedRPM, NV_STALL_S_THRSHLD, C_STALL_THRESHOLD_DIV);  /* MMP190412-1 */
#endif

    if ( (l_u16LastMotorPWMdc < l_u16PidCtrlRatio) &&                           /* Increasing motor current (PWM-DC) */
         (l_u16LastMotorSpeed > g_u16ActualMotorSpeedRPM) &&                    /* Decreasing speed */
         (g_u16ActualMotorSpeedRPM < g_u16TargetMotorSpeedRPM) &&               /* Actual-speed below Target-speed */
         (g_u16ActualMotorSpeedRPM < u16MinSpeed) )                             /* Below minimum speed */
    {
        /* Increasing motor current and decreasing motor speed */
        l_u8StallCountS++;
        if (l_u8StallCountS > NV_STALL_S_WIDTH)                                 /* MMP190412-1 */
        {
            u16StallResult = C_STALL_FOUND;
        }
    }
    else if (l_u8StallCountS != 0U)
    {
        l_u8StallCountS--;
    }
    l_u16LastMotorPWMdc = l_u16PidCtrlRatio;
    l_u16LastMotorSpeed = g_u16ActualMotorSpeedRPM;
    return (u16StallResult);
} /* End of MotorStallCheckS() */
#endif /* (_SUPPORT_STALLDET_S != FALSE) */

#if (_SUPPORT_STALLDET_P != FALSE)
#if (_SUPPORT_CURRSPIKE_POSITION != FALSE)
/*!*************************************************************************** *
 * MotorStallInitP()
 * \brief   Initialise Stall detector "P"
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details Initialise Stall detector "P". This detector is based on Position
 *          feedback by Current-spike Position
 * *************************************************************************** *
 * - Call Hierarchy: ?
 * - Cyclomatic Complexity: 1+1
 * - Nesting: 1
 * - Function calling: 1 (Get_StartupDelay())
 * *************************************************************************** */
void MotorStallInitP(void)
{
    l_u16StallCountP = 0U;                                                      /* Stall-counter "P" (Motor Left) */
    l_u16DetectorDelayP = Get_StartupDelay() * 1U;                              /* Should this be a NVRAM param? (TODO) */
    if (g_e8MotorDirectionCCW != l_e8StallMotorDirectionCCW)
    {
        /* Increase time as gears are turning in other direction */
        l_u16DetectorDelayP *= 8U;                                              /* Should this be a NVRAM param? (TODO) */
        l_e8StallMotorDirectionCCW = g_e8MotorDirectionCCW;
    }
    l_u16LastPosition = g_u16ActualPosition;
} /* End of MotorStallInitP() */

/* ************************************************************************** *
 * MotorStallCheckP()
 * \brief   Initialise Stall detector "P"
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  (uint16_t) C_STALL_NOT_FOUND: No stall found
 *                     C_STALL_FOUND: Stall have been found
 * *************************************************************************** *
 * \details Check if motor is stalled
 * Stall detector "P" is based on Position feedback by Current-spike Position
 * *************************************************************************** *
 * - Call Hierarchy: ?
 * - Cyclomatic Complexity: 5+1
 * - Nesting: 3
 * - Function calling: 0
 * **************************************************************************** */
uint16_t MotorStallCheckP(void)
{
    uint16_t u16Result = C_STALL_NOT_FOUND;

    if (l_u16DetectorDelayP == 0U)
    {
        if (g_e8MotorDirectionCCW != FALSE)
        {
            /* Closing/Lower */

            /* Motor Left */
            if (l_u16LastPosition <= g_u16ActualPosition)
            {
                l_u16StallCountP++;
            }
            else
            {
                l_u16LastPosition = g_u16ActualPosition;
                l_u16StallCountP = 0U;
            }
        }
        else
        {
            /* Opening/Upper */

            /* Motor Left */
            if (l_u16LastPosition >= g_u16ActualPosition)
            {
                l_u16StallCountP++;
            }
            else
            {
                l_u16LastPosition = g_u16ActualPosition;
                l_u16StallCountP = 0U;
            }
        }

        if (l_u16StallCountP >= NV_STALL_P_THRSHLD)
        {
            u16Result = C_STALL_FOUND;
        }
    }
    else
    {
        l_u16LastPosition = g_u16ActualPosition;
        l_u16DetectorDelayP--;
    }

    return (u16Result);

} /* End of MotorStallCheckP() */

#elif (_SUPPORT_POTI != FALSE) && (_SUPPORT_POTI_MODE == C_POTI_MODE_CLOSED_LOOP)
/*!*************************************************************************** *
 * MotorStallInitP()
 * \brief   Initialise Stall detector "P"
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details Initialise Stall detector "P". This detector is based on Position
 *          feedback by Potentiometer
 * *************************************************************************** *
 * - Call Hierarchy: ?
 * - Cyclomatic Complexity: 1+1
 * - Nesting: 1
 * - Function calling: 1 (Get_StartupDelay())
 * *************************************************************************** */
void MotorStallInitP(void)
{
    l_u16StallCountP = 0U;                                                      /* Stall-counter "P" (Motor Left) */
    l_u16DetectorDelayP = Get_StartupDelay() * 1U;                              /* Should this be a NVRAM param? (TODO) */
    if (g_e8MotorDirectionCCW != l_e8StallMotorDirectionCCW)
    {
        /* Increase time as gears are turning in other direction */
        l_u16DetectorDelayP *= 8U;                                              /* Should this be a NVRAM param? (TODO) */
        l_e8StallMotorDirectionCCW = g_e8MotorDirectionCCW;
    }
    l_u16LastPosition = g_u16ActualPotiPos;
} /* End of MotorStallInitP() */

/* ************************************************************************** *
 * MotorStallCheckP()
 * \brief   Initialise Stall detector "P"
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  (uint16_t) C_STALL_NOT_FOUND: No stall found
 *                     C_STALL_FOUND: Stall have been found
 * *************************************************************************** *
 * \details Check if motor is stalled
 * Stall detector "P" is based on Position feedback by Potentiometer
 * *************************************************************************** *
 * - Call Hierarchy: ?
 * - Cyclomatic Complexity: 12+1
 * - Nesting: 4
 * - Function calling: 0
 * **************************************************************************** */
uint16_t MotorStallCheckP(void)
{
    uint16_t u16Result = C_STALL_NOT_FOUND;

    if (l_u16DetectorDelayP == 0U)
    {
        if (g_e8MotorDirectionCCW != FALSE)
        {
            /* Closing/Lower */

            /* Motor Left */
            if (l_u16LastPosition <= g_u16ActualPotiPos)
            {
                l_u16StallCountP++;
            }
            else
            {
                l_u16LastPosition = g_u16ActualPotiPos;
                l_u16StallCountP = 0U;
            }
        }
        else
        {
            /* Opening/Upper */

            /* Motor Left */
            if (l_u16LastPosition >= g_u16ActualPotiPos)
            {
                l_u16StallCountP++;
            }
            else
            {
                l_u16LastPosition = g_u16ActualPotiPos;
                l_u16StallCountP = 0U;
            }
        }

        if (l_u16StallCountP >= NV_STALL_P_THRSHLD)
        {
            u16Result = C_STALL_FOUND;
        }
    }
    else
    {
        l_u16LastPosition = g_u16ActualPotiPos;
        l_u16DetectorDelayP--;
    }
    return (u16Result);

} /* End of MotorStallCheckP() */

#elif (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
/*!*************************************************************************** *
 * MotorStallInitP()
 * \brief   Initialise Stall detector "P"
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details Initialise Stall detector "P". This detector is based on Position
 *          feedback by Resolver (MLX9038x)
 * *************************************************************************** *
 * - Call Hierarchy: ?
 * - Cyclomatic Complexity: 1+1
 * - Nesting: 1
 * - Function calling: 1 (Get_StartupDelay())
 * *************************************************************************** */
void MotorStallInitP(void)
{
    l_u16StallCountP = 0U;                                                      /* Stall-counter "P" (Motor Left) */
    l_u16DetectorDelayP = Get_StartupDelay() * 1U;                              /* Should this be a NVRAM param? (TODO) */
    l_u16LastAngle = Triaxis_Angle();
} /* End of MotorStallInitP() */

/* ************************************************************************** *
 * MotorStallCheckP()
 * \brief   Initialise Stall detector "P"
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  (uint16_t) C_STALL_NOT_FOUND: No stall found
 *                     C_STALL_FOUND: Stall have been found
 * *************************************************************************** *
 * \details Check if motor is stalled
 * Stall detector "P" is based on Position feedback by Resolver (MLX9038x)
 * *************************************************************************** *
 * - Call Hierarchy: ?
 * - Cyclomatic Complexity: 12+1
 * - Nesting: 4
 * - Function calling: 0
 * **************************************************************************** */
uint16_t MotorStallCheckP(void)
{
    uint16_t u16Result = C_STALL_NOT_FOUND;
    uint16_t u16ActualAngle = Triaxis_Angle();

    if (l_u16DetectorDelayP == 0U)
    {
        int16_t i16DeltaPosition = ((int16_t)l_u16LastAngle - (int16_t)u16ActualAngle);
        if (g_e8MotorDirectionCCW != FALSE)
        {
            /* CCW: Closing/Lower */
            if (i16DeltaPosition < 0)
            {
                l_u16StallCountP++;
            }
            else
            {
                l_u16LastAngle = u16ActualAngle;
                l_u16StallCountP = 0U;
            }
        }
        else
        {
            /* CW: Opening/Upper */
            if (i16DeltaPosition > 0)
            {
                l_u16StallCountP++;
            }
            else
            {
                l_u16LastAngle = u16ActualAngle;
                l_u16StallCountP = 0U;
            }
        }

        if (l_u16StallCountP >= NV_STALL_P_THRSHLD)
        {
            u16Result = C_STALL_FOUND;
        }
    }
    else
    {
        l_u16LastAngle = u16ActualAngle;
        l_u16DetectorDelayP--;
    }
    return (u16Result);

} /* End of MotorStallCheckP() */

#elif (_SUPPORT_TRIAXIS_MLX90377 != FALSE)                                      /* MMP230901-1 */
/*!*************************************************************************** *
 * MotorStallInitP()
 * \brief   Initialise Stall detector "P"
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details Initialise Stall detector "P". This detector is based on Position
 *          feedback by Resolver (MLX9038x)
 * *************************************************************************** *
 * - Call Hierarchy: ?
 * - Cyclomatic Complexity: 1+1
 * - Nesting: 1
 * - Function calling: 1 (Get_StartupDelay())
 * *************************************************************************** */
void MotorStallInitP(void)
{
    l_u16StallCountP = 0U;                                                      /* Stall-counter "P" (Motor Left) */
    l_u16DetectorDelayP = Get_StartupDelay() * 2U;                              /* Should this be a NVRAM param? (TODO) */
    if (g_e8MotorDirectionCCW != l_e8StallMotorDirectionCCW)
    {
        /* Increase time as gears are turning in other direction */
        l_u16DetectorDelayP *= 16U;                                             /* Should this be a NVRAM param? (TODO) */
        l_e8StallMotorDirectionCCW = g_e8MotorDirectionCCW;
    }
} /* End of MotorStallInitP() */

/* ************************************************************************** *
 * MotorStallCheckP()
 * \brief   Initialise Stall detector "P"
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  (uint16_t) C_STALL_NOT_FOUND: No stall found
 *                     C_STALL_FOUND: Stall have been found
 * *************************************************************************** *
 * \details Check if motor is stalled
 * Stall detector "P" is based on Position feedback by Resolver (MLX9038x)
 * *************************************************************************** *
 * - Call Hierarchy: ?
 * - Cyclomatic Complexity: 12+1
 * - Nesting: 4
 * - Function calling: 0
 * **************************************************************************** */
uint16_t MotorStallCheckP(void)
{
    uint16_t u16StallResult = C_STALL_NOT_FOUND;

    if (l_u16DetectorDelayP == 0U)
    {
        if (g_u8TriaxisStatus == STS_NEW_SAMPLE)
        {
            if (g_i16DeltaShaftAngle > (int16_t)C_STALL_P_THRESHOLD)
            {
                l_u16StallCountP = p_DecNzU16(l_u16StallCountP);
            }
            else
            {
                uint16_t u16StallCountP = l_u16StallCountP + 1U;
                l_u16StallCountP = u16StallCountP;
                if (u16StallCountP >= C_STALL_P_WIDTH)
                {
                    /* Real stall */
                    u16StallResult = C_STALL_FOUND;
                }
            }
            g_u8TriaxisStatus = STS_EMPTY;
        }
    }
    else
    {
        l_u16DetectorDelayP--;
    }
    return (u16StallResult);
} /* End of MotorStallCheckP() */

#endif /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */
#endif /* (_SUPPORT_STALLDET_P != FALSE) */

#if (LIN_COMM != FALSE) && (_SUPPORT_MLX_DEBUG_MODE != FALSE)
/*!*************************************************************************** *
 * LinDiag_MotorStall()
 * \brief   LIN Diagnostics Motor stall information
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details -
 * *************************************************************************** *
 * - Call Hierarchy: DfrDiagMlxDebug()
 * - Cyclomatic Complexity: 2+1
 * - Nesting: 2
 * - Function calling: 2 (Get_MCurrGain(), Get_MotorCurrentMovAvgxN())
 * *************************************************************************** */
void LinDiag_MotorStall(void)
{
    uint16_t u16Value;
#if (_SUPPORT_STALLDET_A != FALSE)
#if (C_MOVAVG_SZ == (65536UL / C_GMCURR_DIV))
    u16Value = p_MulU16hi_U16byU16(l_u16MotorCurrentStallThrshldxN,
                                   Get_MCurrGain());                            /* Stall motor-current threshold [mA] */
#elif (C_MOVAVG_SZ < (65536UL / C_GMCURR_DIV))
#if defined (C_GMCURR_SDIV) && defined (C_MOVAVG_SSZ)
    u16Value = (uint16_t) (p_MulU32_U16byU16(l_u16MotorCurrentStallThrshldxN,
                                             Get_MCurrGain()) >> (C_GMCURR_SDIV + C_MOVAVG_SSZ));  /* Stall motor-current threshold [mA] */
#else
    u16Value = p_MulDivU16_U16byU16byU16(l_u16MotorCurrentStallThrshldxN,
                                         Get_MCurrGain(),
                                         (C_GMCURR_DIV * C_MOVAVG_SZ));         /* Stall motor-current threshold [mA] */
#endif
#else
#error "Error: Stall current"
#endif
#else  /* (_SUPPORT_STALLDET_A != FALSE) */
    u16Value = 0xFFFFU;                                                         /* Not supported */
#endif /* (_SUPPORT_STALLDET_A != FALSE) */
    g_DiagResponse.u.SF.byD1 = (uint8_t)(u16Value & 0xFFU);
    g_DiagResponse.u.SF.byD2 = (uint8_t)(u16Value >> 8);

    u16Value = Get_MotorCurrentMovAvgxN_mA();
    g_DiagResponse.u.SF.byD3 = (uint8_t)(u16Value & 0xFFU);
    g_DiagResponse.u.SF.byD4 = (uint8_t)(u16Value >> 8);

    u16Value = 0U;
#if (_SUPPORT_STALLDET_A != FALSE)
    if ( ((g_u8StallTypeComm & C_STALL_FOUND_A) != 0U)
#if (_SUPPORT_STALLDET_O != FALSE)
         || (l_u8StallCountA >= l_u8StallCountO)
#elif (_SUPPORT_STALLDET_P != FALSE)
         || (l_u8StallCountA >= l_u16StallCountP)
#endif /* (_SUPPORT_STALLDET_O != FALSE) */
         )
    {
        u16Value = l_u8StallCountA;                                             /* Stall count "A" */
    }
    else
#endif /* _SUPPORT_STALLDET_A */
#if (_SUPPORT_STALLDET_O != FALSE)
    if ( ((g_u8StallTypeComm & C_STALL_FOUND_O) != 0U)
#if (_SUPPORT_STALLDET_A != FALSE)
         || (l_u8StallCountO > l_u8StallCountA)
#endif /* (_SUPPORT_STALLDET_A != FALSE) */
         )
    {
        u16Value = l_u8StallCountO;                                             /* Stall count "O" */
    }
    else
#endif /* _SUPPORT_STALLDET_O */
#if (_SUPPORT_STALLDET_P != FALSE)
    if ( ((g_u8StallTypeComm & C_STALL_FOUND_P) != 0U)
#if (_SUPPORT_STALLDET_A != FALSE)
         || (l_u16StallCountP > l_u8StallCountA)
#endif /* (_SUPPORT_STALLDET_A != FALSE) */
         )
    {
        u16Value = l_u16StallCountP;                                            /* Stall count "P" */
    }
    else
#endif /* (_SUPPORT_STALLDET_P != FALSE) */
    {
        /* Nothing */
    }
    while (u16Value > 7U)
    {
        u16Value = ((u16Value + 1U) >> 1);
    }
    g_DiagResponse.u.SF.byD5 = (g_u8StallTypeComm & M_STALL_MODE) | (uint8_t)u16Value;
    g_u8StallTypeComm = (uint8_t)C_STALL_NOT_FOUND;
} /* End of LinDiag_MotorStall() */
#endif /* (LIN_COMM != FALSE) && (_SUPPORT_MLX_DEBUG_MODE != FALSE) */

/* EOF */
