/*!************************************************************************** *
 * \file        MotorStall.h
 * \brief       MLX813xx Motor Stall handling
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
 *           -# MotorStallInitLA()
 *           -# MotorStallSwitchOpen2CloseLA()
 *           -# MotorStallCheckLA()
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
 *
 * ************************************************************************** */

#ifndef DRIVE_LIB_MOTOR_STALL_H
#define DRIVE_LIB_MOTOR_STALL_H

/*!************************************************************************** */
/*                          INCLUDES                                          */
/* ************************************************************************** */
#include "AppBuild.h"                                                           /* Application Build */

#include "drivelib/NV_UserPage.h"                                               /* Non Volatile Memory User Application Page Layout */
#include "drivelib/MotorDriver.h"                                               /* Motor driver support */
#include "camculib/private_mathlib.h"                                           /* Private Math Library support */

/*!*************************************************************************** */
/*                           DEFINITIONS                                       */
/* *************************************************************************** */
#define C_STALL_NOT_FOUND           0x00U                                       /*!< Stall Not Found */
#define C_STALL_FOUND               0x01U                                       /*!< Stall Found */
/* Stall detector levels
 * g_u8StallMode
 */
/* #define C_STALL_NOT_FOUND        0x00U */                                    /*!< No stall detected */
#define M_STALL_MODE                0x78U                                       /*!< bit 6:3: Stall-mode */
#if (_SUPPORT_STALLDET_A != FALSE)
#define C_STALL_FOUND_A             0x40U                                       /*!< bit 6: Current Amplitude stall detected (A) */
#endif /* (_SUPPORT_STALLDET_A != FALSE) */
#if (_SUPPORT_STALLDET_BZC != FALSE) || (_SUPPORT_STALLDET_BRI != FALSE)
#define C_STALL_FOUND_B             0x20U                                       /*!< Bit 5: BEMF-based (zero-crossing) (B) */
#endif /* (_SUPPORT_STALLDET_BZC != FALSE) || (_SUPPORT_STALLDET_BRI != FALSE) */
#if (_SUPPORT_STALLDET_FLUX != FALSE)
#define C_STALL_FOUND_FLUX          0x20U                                       /*!< bit 5: Flux Amplitude stall detected (B) */
#endif /* (_SUPPORT_STALLDET_FLUX != FALSE) */
#if (_SUPPORT_STALLDET_S != FALSE)
#define C_STALL_FOUND_UNDER_SPEED   0x10U                                       /*!< bit 4: Under-speed stall detected (P) */
#define C_STALL_FOUND_OVER_SPEED    0x10U                                       /*!< bit 4: Over-speed stall detected (P) */
#define C_STALL_FOUND_S             0x10U                                       /*!< bit 4: Increasing current and decreasing speed (P) */
#endif /* (_SUPPORT_STALLDET_S != FALSE) */
#if (_SUPPORT_CDI != FALSE)
#define C_STALL_FOUND_CDI           0x10U                                       /*!< bit 4: CDI stall detected (CDI) (P) */
#endif /* (_SUPPORT_CDI != FALSE) */
#if (_SUPPORT_STALLDET_H != FALSE)
#define C_STALL_FOUND_H             0x10U                                       /*!< bit 4: Position stall detected (not used) (P) */
#endif /* (_SUPPORT_STALLDET_H != FALSE) */
#if (_SUPPORT_STALLDET_P != FALSE)
#define C_STALL_FOUND_P             0x10U                                       /*!< bit 4: Position stall detected (not used) (P) */
#endif /* (_SUPPORT_STALLDET_P != FALSE) */
#if (_SUPPORT_STALLDET_LA != FALSE)
#define C_STALL_FOUND_PID_LA        0x10U                                       /*!< bit 4: PID-LA control stall detected (P) */
#endif /* (_SUPPORT_STALLDET_LA != FALSE) */
#if (_SUPPORT_STALLDET_O != FALSE)
#define C_STALL_FOUND_O             0x08U                                       /*!< bit 3: Current Oscillation stall detection (O) */
#endif /* (_SUPPORT_STALLDET_O != FALSE) */
#if (_SUPPORT_STALLDET_LA != FALSE)
#define C_STALL_FOUND_LA            0x08U                                       /*!< bit 3: Load-angle stall detected (O) */
#endif /* (_SUPPORT_STALLDET_LA != FALSE) */

/* Stall-O */
#define C_CURROSC_SZ                (2U * C_MICROSTEP_PER_FULLSTEP)             /*!< Stall Oscillation current-sample buffer size (two full-steps of micro-steps) */

/* Stall-LA */
#define C_ANGLE_2DEG                (uint16_t)((65536UL * 2U) / 720U)           /*!< Stall detector: 50% of 2 degrees Load-angle threshold units */
#define C_ANGLE_3DEG                (uint16_t)((65536UL * 3U) / 720U)           /*!< Stall detector: 50% of 3 degrees Load-angle threshold units */
#define C_ANGLE_5DEG                (uint16_t)((65536UL * 5U) / 720U)           /*!< Stall detector: 50% of 5 degrees Load-angle threshold units */

#define C_LA_LPF_COEF_CL            256U                                        /*!< Load-angle LPF Coefficient (Closed Loop) (MMP220720-1/MMP220902-1) */
#define C_LA_LPF_COEF_OL            4096U                                       /*!< Load-angle LPF Coefficient (Open Loop) (MMP240607-2) */

/* Stall-Flux */
#define C_FLUX_UNITS                5U                                          /*!< Flux units */

/*!*************************************************************************** */
/*                           GLOBAL VARIABLES                                  */
/* *************************************************************************** */
#pragma space dp                                                                /* __TINY_SECTION__ */
#pragma space none                                                              /* __TINY_SECTION__ */

#pragma space nodp                                                              /* __NEAR_SECTION__ */
#pragma space none                                                              /* __NEAR_SECTION__ */

/*!*************************************************************************** */
/*                           GLOBAL FUNCTIONS                                  */
/* *************************************************************************** */
extern uint8_t StallDetectorEna(void);                                          /*!< Get Stall Detector Enable "mask" */
#if (_SUPPORT_STALLDET_A != FALSE)
extern void MotorStallInitA(void);                                              /*!< Initialise Stall detector "A" */
#if (_SUPPORT_FOC_MODE != FOC_MODE_NONE)
void MotorStallSwitchOpen2CloseA(void);                                         /*!< Initialise Stall detector "A" for FOC */
#endif /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) */
extern uint16_t MotorStallCheckA(void);                                         /*!< Check if motor is stalled */
#endif /* (_SUPPORT_STALLDET_A != FALSE) */

#if (_SUPPORT_STALLDET_H != FALSE)
extern void MotorStallInitH(void);                                              /*!< Initialise Stall detector "H" */
#if (_SUPPORT_MICRO_STEP_COMMUTATION != FALSE)
extern uint16_t MotorStallCheckH(uint16_t u16lMicroStepCntPerHallCommut);       /*!< Check if motor is stalled */
#else  /* (_SUPPORT_MICRO_STEP_COMMUTATION != FALSE) */
extern uint16_t MotorStallCheckH(uint16_t u16HallLatchEvent);                   /*!< Check if motor is stalled */
#endif /* (_SUPPORT_MICRO_STEP_COMMUTATION != FALSE) */
#endif /* (_SUPPORT_STALLDET_H != FALSE) */

#if (_SUPPORT_STALLDET_O != FALSE)
extern void MotorStallInitO(void);                                              /*!< Initialise Stall detector "O" */
extern uint16_t MotorStallCheckO(void);                                         /*!< Check if motor is stalled */
#endif /* (_SUPPORT_STALLDET_O != FALSE) */

#if (_SUPPORT_STALLDET_S != FALSE)
extern void MotorStallInitS(void);                                              /*!< Initialise Stall detector "S" */
extern uint16_t MotorStallCheckS(void);                                         /*!< Check if motor is stalled */
#endif /* (_SUPPORT_STALLDET_S != FALSE) */

#if (_SUPPORT_STALLDET_P != FALSE)
extern void MotorStallInitP(void);                                              /*!< Initialise Stall detector "P" */
extern uint16_t MotorStallCheckP(void);                                         /*!< Check if motor is stalled */
#endif /* (_SUPPORT_STALLDET_P != FALSE) */

#if (LIN_COMM != FALSE) && (_SUPPORT_MLX_DEBUG_MODE != FALSE)
extern void LinDiag_MotorStall(void);                                           /*!< LIN Diagnostics Motor stall information */
#endif /* (LIN_COMM != FALSE) && (_SUPPORT_MLX_DEBUG_MODE != FALSE) */

#if (_SUPPORT_STALLDET_LA != FALSE)
#pragma space dp
extern int16_t l_i16StallThresholdLA;                                           /*!< Stall-detector Load-angle threshold x 3 degrees (Actual Speed) */
extern int16_t l_i16StallThresholdLA_Max;                                       /*!< Stall-detector Load-angle threshold x 3 degrees (Maximum Speed) */
extern int16_t l_i16ActLoadAngleLPF;                                            /*!< Actual Load-angle LPF */
extern int32_t l_i32ActLoadAngleLPF;                                            /*!< LPF Actual Load-angle (internal variable) */
extern volatile uint8_t l_u8StallCountLA;                                       /*!< Stall detector "LA" Counter */
#pragma space none

/*!*************************************************************************** *
 * MotorStallInitLA
 * \brief   Initialise Stall detector "LA"
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16MotorTargetSpeed: Motor target speed-mode (obsolete)
 * \return  -
 * *************************************************************************** *
 * \details
 * *************************************************************************** *
 * - Call Hierarchy: ?
 * - Cyclomatic Complexity: 0
 * - Nesting: 0
 * - Function calling: 0
 * *************************************************************************** */
static inline void MotorStallInitLA(uint16_t u16MotorTargetSpeed)
{
    l_u8StallCountLA = 0U;
    l_i16ActLoadAngleLPF = 0;
    l_i32ActLoadAngleLPF = 0;
#if ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_NONE)
    if (u16MotorTargetSpeed <= C_MOTOR_SPEED_4)
    {
        /* Open-Loop Stepper-mode */
        l_i16StallThresholdLA_Max = (NV_STALL_LA_THRSHLD_OL * C_ANGLE_3DEG);
    }
    else
#endif /* ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_NONE) */
    {
        /* Closed-Loop FOC-mode */
        l_i16StallThresholdLA_Max = -(NV_STALL_LA_THRSHLD * C_ANGLE_3DEG);      /* MMP230824-1 */
        l_i16StallThresholdLA = l_i16StallThresholdLA_Max;
        (void) u16MotorTargetSpeed;
    }
} /* End of MotorStallInitLA() */

/*!*************************************************************************** *
 * MotorStallSwitchOpen2CloseLA
 * \brief   Initialise Stall detector "LA"
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details Just reset the Stall "LA" count and re-initialise the Threshold for FOC.
 *          Don't clear the actual Load-angle LPF.
 * *************************************************************************** *
 * - Call Hierarchy: ?
 * - Cyclomatic Complexity: 0
 * - Nesting: 0
 * - Function calling: 0
 * *************************************************************************** */
static inline void MotorStallSwitchOpen2CloseLA(void)
{
    /* Closed-Loop FOC-mode */
    l_u8StallCountLA = 0U;
    l_i16StallThresholdLA_Max = -(NV_STALL_LA_THRSHLD * C_ANGLE_3DEG);          /* MMP230824-1 */
    l_i16StallThresholdLA = l_i16StallThresholdLA_Max;
} /* End of MotorStallSwitchOpen2CloseLA() */

/*!*************************************************************************** *
 * MotorStallCheckLA
 * \brief   IInitialise Stall detector "LA"
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  (uint16) C_STALL_NOT_FOUND: No stall found
 *                   C_STALL_FOUND: Stall have been found
 * *************************************************************************** *
 * \details Check if motor is stalled
 *          Stall detector "LA" is based on negative LA-angle
 * *************************************************************************** *
 * - Call Hierarchy: main_Init(), HandleMotorRequest()
 * - Cyclomatic Complexity: 3+0
 * - Nesting: 3
 * - Function calling: 0
 * *************************************************************************** */
static inline uint16_t MotorStallCheckLA(void)
{
    if ( (Get_StartupDelay() == 0U) &&                                          /* Stall start-up delay done */
         (g_u16ActualMotorSpeedRPM >= C_STALL_LA_SPEED_OFFSET) &&               /* Above Stall detector "LA" speed */
         (l_i16StallThresholdLA_Max != 0) )                                     /* Stall LA enabled (MMP220720-1) */
    {
        if (l_i16StallThresholdLA_Max > 0)
        {
#if (_SUPPORT_STALLDET_LA_MODE == C_THRSHLD_SPEED)                              /* MMP240712-1 */
            /* Calculate speed dependent Stall LA threshold */
            l_i16StallThresholdLA =
                (int16_t)p_MulDivU16_U16byU16byU16(g_u16ForcedSpeedRPM,
                                                   (uint16_t)l_i16StallThresholdLA_Max,
                                                   g_u16MaxSpeedRPM);
#elif (_SUPPORT_STALLDET_LA_MODE == C_THRSHLD_SPEED_W_OFFSET)                   /* MMP240715-1 */
            /* Calculate speed dependent Stall LA threshold */
            l_i16StallThresholdLA = (NV_STALL_LA_THRSHLD * C_ANGLE_3DEG) +
                (int16_t)p_MulDivU16_U16byU16byU16(g_u16ForcedSpeedRPM,
                                                   (uint16_t)l_i16StallThresholdLA_Max,
                                                   g_u16MaxSpeedRPM);
#else  /* (_SUPPORT_STALLDET_LA_MODE == C_) */
            l_i16StallThresholdLA = l_i16StallThresholdLA_Max;
#endif /* (_SUPPORT_STALLDET_LA_MODE == C_) */
        }

        if (l_i16ActLoadAngleLPF > l_i16StallThresholdLA)                       /* Stall LA Threshold non-zero (MMP220720-1) */
        {
            /* Lead-angle above threshold; No stall */
            l_u8StallCountLA = p_DecNzU8(l_u8StallCountLA);
        }
        else
        {
            /* Lead-angle below threshold; Nearing stall */
            uint8_t u8StallCountLA = l_u8StallCountLA + 1U;
            l_u8StallCountLA = u8StallCountLA;
#if ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_NONE)
            if (u8StallCountLA >= NV_STALL_LA_WIDTH_OL)
#else  /* ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_NONE) */
            if (u8StallCountLA >= NV_STALL_LA_WIDTH)
#endif /* ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_NONE) */
            {
                /* Real stall */
                return (C_STALL_FOUND);
            }
        }
    }
    return (C_STALL_NOT_FOUND);
} /* End of MotorStallCheckLA() */
#endif /* (_SUPPORT_STALLDET_LA != FALSE) */

#if (_SUPPORT_FOC_MODE != FOC_MODE_NONE) && (_SUPPORT_STALLDET_FLUX != FALSE)
#pragma space dp
extern uint16_t l_u16StallThresholdFlux;                                        /*!< Stall-detector Flux threshold */
extern volatile uint8_t l_u8StallCountFlux;                                     /*!< Stall detector "Flux" Counter */
#pragma space none
/*!*************************************************************************** *
 * MotorStallInitFlux
 * \brief   IInitialise Stall detector "Flux"
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details
 * *************************************************************************** *
 * - Call Hierarchy: ?
 * - Cyclomatic Complexity: 0
 * - Nesting: 0
 * - Function calling: 0
 * *************************************************************************** */
static inline void MotorStallInitFlux(void)
{
    l_u8StallCountFlux = 0U;
    l_u16StallThresholdFlux = (NV_STALL_FLUX_THRSHLD * C_FLUX_UNITS);           /* MMP220725-1 */
} /* End of MotorStallInitLA() */

/*!*************************************************************************** *
 * MotorStallCheckFlux
 * \brief   IInitialise Stall detector "Flux"
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  (uint16) C_STALL_NOT_FOUND: No stall found
 *                   C_STALL_FOUND: Stall have been found
 * *************************************************************************** *
 * \details Check if motor is stalled
 *          Stall detector "Flux" is based on minimum Motor-Flux
 * *************************************************************************** *
 * - Call Hierarchy: main_Init(), HandleMotorRequest()
 * - Cyclomatic Complexity: 3+0
 * - Nesting: 3
 * - Function calling: 0
 * *************************************************************************** */
static inline uint16_t MotorStallCheckFlux(uint16_t u16Flux)
{
    if ( (Get_StartupDelay() == 0U) &&                                          /* Stall start-up delay done */
         (l_u16StallThresholdFlux != 0U))                                       /* Stall Flux enabled (MMP220725-1) */
    {
        if (u16Flux >= l_u16StallThresholdFlux)                                 /* Stall Flux Threshold non-zero (MMP220725-1) */
        {
            /* Flux above threshold; No stall */
            l_u8StallCountFlux = p_DecNzU8(l_u8StallCountFlux);
        }
        else
        {
            /* Flux below threshold; Nearing stall */
            uint8_t u8StallCountFlux = l_u8StallCountFlux + 1U;
            l_u8StallCountFlux = u8StallCountFlux;
            if (u8StallCountFlux >= NV_STALL_FLUX_WIDTH)
            {
                /* Real stall */
                return (C_STALL_FOUND);
            }
        }
    }
    return (C_STALL_NOT_FOUND);
} /* End of MotorStallCheckFlux() */
#endif /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) && (_SUPPORT_STALLDET_FLUX != FALSE) */

#endif /* DRIVE_LIB_MOTOR_STALL_H */

/* EOF */
