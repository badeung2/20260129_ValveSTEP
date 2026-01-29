/*!*************************************************************************** *
 * \file        LIN_2x_HVAC.c
 * \brief       MLX8133x LIN 2.x HVAC-Flap communication handling
 *
 * \note        project MLX8133x
 *
 * \author      Marcel Braat
 *
 * \date        2012-02-11
 *
 * \version     2.0
 *
 * A list of functions:
 *  - Global Functions:
 *           -# LIN_2x_Init()
 *           -# HandleActCtrl()
 *           -# HandleActStatus()
 *           -# HandleBusTimeout()
 *           -# HandleLinError()
 *  - Internal Functions:
 *           -# ClearEventFlags()
 *           -# ModeChange()
 *           -# SaveProgrammingData()
 *           -# SetupStallDetector()
 *           -# SetSpeedMode()
 *           -# SetPosition()
 *           -# SetHoldingCurrent()
 *           -# GetErrorEvents()
 *           -# GetProgrammingData()
 *
 *
 * \copyright   MELEXIS Microelectronic Integrated Systems
 *
 * Copyright (C) 2012-2015 Melexis N.V.
 * The Software is being delivered 'AS IS' and Melexis, whether explicitly or
 * implicitly, makes no warranty as to its Use or performance.
 * The user accepts the Melexis Firmware License Agreement.
 *
 * Melexis confidential & proprietary
 *
 * *************************************************************************** */

/*!*************************************************************************** *
 *                              I N C L U D E S                                *
 * *************************************************************************** */
#include "AppBuild.h"                                                           /* Application Build */

#if (LIN_COMM != FALSE) && ((LINPROT == LIN2X_HVAC49) || (LINPROT == LIN2X_HVAC52))

#include "drivelib/ADC.h"                                                       /* ADC support */
#include "drivelib/AppFunctions.h"                                              /* Application Functions support */
#include "drivelib/ErrorCodes.h"                                                /* Error-logging support */
#include "drivelib/GlobalVars.h"                                                /* Global Variables support */
#include "drivelib/MotorStall.h"                                                /* Motor Stall Detectors */
#include "drivelib/NV_Functions.h"                                              /* Non Volatile Memory Functions & Layout */
#include "drivelib/NV_UserPage.h"                                               /* Non Volatile Memory User Application Page Layout */
#if (_SUPPORT_MLX_DEBUG_MODE != FALSE)
#include "drivelib/PID_Control.h"                                               /* PID support */
#endif /* (_SUPPORT_MLX_DEBUG_MODE != FALSE) */
#include "drivelib/Timer.h"                                                     /* Simple Timer support */
#include "camculib/private_mathlib.h"                                           /* Private Math Library support */

#include "commlib/LIN_AutoAddressing.h"                                         /* LIN Auto-Addressing support */
#include "commlib/LIN_Communication.h"                                          /* LIN Communication support */

#if (_SUPPORT_HALL_LATCH != FALSE) && (_SUPPORT_NR_OF_HL == 1U)
#include "senselib/HallLatch.h"                                                 /* Hall-Latch support */
#endif /* (_SUPPORT_HALL_LATCH != FALSE) && (_SUPPORT_NR_OF_HL == 1U) */

#include <mls_types.h> /*lint !e451 */

#ifndef _SUPPORT_ACT_SPEED_BY_LIN
#define _SUPPORT_ACT_SPEED_BY_LIN       FALSE                                   /*!< FALSE: Actuator speed by MotroDriver; TRUE: Actuator speed by LIN Command */
#endif /* _SUPPORT_ACT_SPEED_BY_LIN */

/*!*************************************************************************** */
/*                           GLOBAL & LOCAL VARIABLES                          */
/* *************************************************************************** */
#pragma space dp                                                                /* __TINY_SECTION__ */
#if (_SUPPORT_LIN_AA != FALSE) && (LIN_AA_TEST_DUT != FALSE)
uint8_t g_u8NAD = C_TEST_DUT_NAD;                                               /*!< Actual NAD */
#else  /* (_SUPPORT_LIN_AA != FALSE) && (LIN_AA_TEST_DUT != FALSE) */
uint8_t g_u8NAD = C_DEFAULT_NAD;                                                /*!< Actual NAD */
#endif /* (_SUPPORT_LIN_AA != FALSE) && (LIN_AA_TEST_DUT != FALSE) */
uint8_t g_u8CtrlPID;                                                            /*!< Control Message Frame ID */
uint8_t g_u8StsPID;                                                             /*!< Status Message Frame ID */
static uint8_t l_u8ActDirection = 0U;                                           /*!< Actuator Rotational Direction flag */
#pragma space none                                                              /* __TINY_SECTION__ */

#pragma space nodp                                                              /* __NEAR_SECTION__ */
#if (_SUPPORT_ACT_SPEED_BY_LIN != FALSE)
static uint16_t l_au16MotorSpeedRPM[8];                                         /*!< Motor speed table (8 speeds) */
static uint8_t l_u8MotorCtrlSpeed = C_CTRL_SPEED_RES;                           /*!< Last selected motor speed-mode */
#endif /* (_SUPPORT_ACT_SPEED_BY_LIN != FALSE) */
static uint8_t l_e8PositionType = (uint8_t)C_POSTYPE_INIT;                      /*!< Status-flag position-type */
static uint8_t l_u8PrevProgramMode = C_CTRL_PROGRAM_INV;                        /*!< Previous Programming mode flags (initial: Invalid) */
#if (_SUPPORT_HVAC_GROUP_ADDRESS != FALSE)
uint8_t l_u8GAD = C_INVALD_GAD;                                                 /*!< Group-address field */
#endif /* (_SUPPORT_HVAC_GROUP_ADDRESS != FALSE) */
#pragma space none                                                              /* __NEAR_SECTION__ */

/*!*************************************************************************** *
 * ActPosition
 * \brief   Convert actuator position into position based on direction
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16Pos: Position
 * \param   [in] u8Direction: Rotational Direction
 * \return  (uint16_t) Convert position based on direction
 * *************************************************************************** *
 * \details
 * *************************************************************************** *
 * - Call Hierarchy: HandleActCtrl(), HandleActStatus()
 * - Cyclomatic Complexity: 1+1
 * - Nesting: 1
 * - Function calling: 0
 * *************************************************************************** */
static INLINE uint16_t ActPosition(uint16_t u16Pos, uint8_t u8Direction)
{
    if ( (u8Direction != FALSE) && (u16Pos <= C_MAX_POS) )
    {
        u16Pos = (C_MAX_POS - u16Pos);
    }
    return (u16Pos);
} /* End of ActPosition() */

/*!*************************************************************************** *
 * LIN_2x_Init
 * \brief   LIN 2.x initialisation for HVAC application
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details Check Non Volatile Memory NAD
 *          Set initial rotational direction
 *          Configure LIN frame-ID's for Control and Status frame.
 * NOTE: As this function is called by main initialisation, it is assumed that no
 *       Non Volatile Memory Write is busy.
 * *************************************************************************** *
 * - Call Hierarchy: LIN_Init()
 * - Cyclomatic Complexity: 1+1
 * - Nesting: 1
 * - Function calling: 0
 * *************************************************************************** */
void LIN_2x_Init(void)
{
    STD_LIN_PARAMS_t *pStdLin = (STD_LIN_PARAMS_t *)ADDR_NV_STD_LIN_1;
#if (LINPROT == LIN2X_HVAC52) && (_SUPPORT_HVAC_GROUP_ADDRESS != FALSE)
    ENH_LIN_PARAMS_t *pEnhLin = (ENH_LIN_PARAMS_t *)ADDR_NV_ENH_LIN_1;
#endif /* (LINPROT == LIN2X_HVAC52) && (_SUPPORT_HVAC_GROUP_ADDRESS != FALSE) */
    APP_EOL_t *pEOL = (APP_EOL_t *)ADDR_NV_EOL;
#if (_SUPPORT_LIN_AA == FALSE) || (LIN_AA_TEST_DUT == FALSE)
    if ( (pStdLin->u8NAD != 0x00U) && ((pStdLin->u8NAD & 0x80U) == 0x00U) )     /* Check NAD-range (0x01-0x7F) */
    {
        /* Valid NAD */
        g_u8NAD = pStdLin->u8NAD;
    }
#if (_SUPPORT_LOG_ERRORS != FALSE)
    else
    {
        /* Keep original default NAD address */
        SetLastError(C_ERR_INV_NAD);
    }
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
#endif /* (_SUPPORT_LIN_AA == FALSE) || (LIN_AA_TEST_DUT == FALSE) */

#if (LINPROT == LIN2X_HVAC52) && (_SUPPORT_HVAC_GROUP_ADDRESS != FALSE)
    /* Group-address */
    l_u8GAD = pEnhLin->u8GAD;
#endif /* (LINPROT == LIN2X_HVAC52) && (_SUPPORT_HVAC_GROUP_ADDRESS != FALSE) */

    l_u8ActDirection = pEOL->u1MotorDirectionCCW;

    g_u8CtrlPID = pStdLin->u8ControlFrameID;
    (void)ml_AssignFrameToMessageID(MSG_CONTROL, g_u8CtrlPID);
    g_u8StsPID = pStdLin->u8StatusFrameID;
    (void)ml_AssignFrameToMessageID(MSG_STATUS, g_u8StsPID);
#if (LINPROT == LIN2X_HVAC52) && (_SUPPORT_HVAC_GROUP_ADDRESS != FALSE)
    if ( (l_u8GAD >= 0x80U) && (l_u8GAD <= 0x9FU) )  /* Group address: 0x80..0x9F (MMP221202-2) */
    {
        (void)ml_AssignFrameToMessageID(MSG_GROUP_CONTROL, pEnhLin->u8GroupControlFrameID);
    }
#endif /* (LINPROT == LIN2X_HVAC52) && (_SUPPORT_HVAC_GROUP_ADDRESS != FALSE) */
#if (_SUPPORT_TRIAXIS_MLX90363 != FALSE) || (_SUPPORT_TRIAXIS_MLX90367 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90372 != FALSE) || (_SUPPORT_TRIAXIS_MLX90377 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX9038x != FALSE) || (_SUPPORT_TRIAXIS_MLX90395 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90421 != FALSE) || (_SUPPORT_TRIAXIS_MLX90422 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90425 != FALSE) || (_SUPPORT_TRIAXIS_MLX90426 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90427 != FALSE) || \
    (_SUPPORT_HUMIDITY_HDC302x != FALSE) || \
    (_SUPPORT_INDUCTIVE_POS_SENSOR_MLX90513 != FALSE) || (_SUPPORT_PRESSURE_MLX90829 != FALSE)
    (void)ml_AssignFrameToMessageID(MSG_SENSOR, 0x03U);
#endif /* (_SUPPORT_TRIAXIS_MLX90363 != FALSE) || (_SUPPORT_TRIAXIS_MLX90367 != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90372 != FALSE) || (_SUPPORT_TRIAXIS_MLX90377 != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX9038x != FALSE) || (_SUPPORT_TRIAXIS_MLX90395 != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90421 != FALSE) || (_SUPPORT_TRIAXIS_MLX90422 != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90425 != FALSE) || (_SUPPORT_TRIAXIS_MLX90426 != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90427 != FALSE) || \
        * (_SUPPORT_INDUCTIVE_POS_SENSOR_MLX90513 != FALSE) || (_SUPPORT_PRESSURE_MLX90829 != FALSE) */
#if (_SUPPORT_MLX_CHIP_STATUS != FALSE)
    (void)ml_AssignFrameToMessageID(MSG_MLX_CHIP_STATUS, mlxCHIP_STATUS);
    (void)ml_AssignFrameToMessageID(MSG_MLX_CHIP_CONTROL, mlxCHIP_CONTROL);
#endif /* (_SUPPORT_MLX_CHIP_STATUS != FALSE) */

#if (_SUPPORT_ACT_SPEED_BY_LIN != FALSE)
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
#endif /* (_SUPPORT_ACT_SPEED_BY_LIN != FALSE) */

} /* End of LIN_2x_Init() */

/*!*************************************************************************** *
 * ClearEventFlags
 * \brief   LIN Control Message Handler
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] pCtrl: Pointer to LIN Control Message
 * \return  (uint8_t) Event summary
 * *************************************************************************** *
 * \details
 * *************************************************************************** *
 * - Call Hierarchy: HandleActCtrl()
 * - Cyclomatic Complexity: 5+1
 * - Nesting: 3
 * - Function calling: 0
 * *************************************************************************** */
static inline uint8_t ClearEventFlags(HVAC_CTRL *pCtrl)
{
    /* Priority 1: Clear Event Flags */
    if ( (pCtrl->u4ClearEventFlags != (uint8_t)C_CTRL_CLREVENT_NONE) &&
         (pCtrl->u4ClearEventFlags != (uint8_t)C_CTRL_CLREVENT_INV) )
    {
        /* Clear one or more event flags */
        if ( (pCtrl->u4ClearEventFlags & (uint8_t)C_CTRL_CLREVENT_RESET) != 0U)
        {
            /* Clear reset flag */
            g_u8ChipResetOcc = FALSE;
        }
        if ( (pCtrl->u4ClearEventFlags & (uint8_t)C_CTRL_CLREVENT_STALL) != 0U)
        {
            /* Clear stall detected flag */
            g_u8StallOcc = FALSE;
        }
        if ( (pCtrl->u4ClearEventFlags & (uint8_t)C_CTRL_CLREVENT_EMRUN) != 0U)
        {
            /* Clear emergency-run occurred flag */
            if (g_e8EmergencyRunOcc != (uint8_t)C_SAFETY_RUN_NO)
            {
                g_e8MotorRequest = (uint8_t)C_MOTOR_REQUEST_STOP;               /* Stop actuator, in case of EmRun; See 4.6.2.1 */
            }
            g_e8EmergencyRunOcc = (uint8_t)C_SAFETY_RUN_NO;
        }
        /* ( pCtrl->byClearEventFlags & C_CTRL_CLREVENT_RES ) is invalid */
    }
    return (g_u8ChipResetOcc | g_u8StallOcc | g_e8EmergencyRunOcc);
} /* End of ClearEventFlags() */

/*!*************************************************************************** *
 * ModeChange
 * \brief   LIN Control Message Handler
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] pCtrl: Pointer to LIN Control Message
 * \return  (uint8_t) Event summary
 * *************************************************************************** *
 * \details
 * *************************************************************************** *
 * - Call Hierarchy: HandleActCtrl()
 * - Cyclomatic Complexity: 3+1
 * - Nesting: 3
 * - Function calling: 0
 * *************************************************************************** */
static inline void ModeChange(HVAC_CTRL *pCtrl)
{
    /* Only handled in case not in Event-mode */
    if (pCtrl->u2StopMode == (uint8_t)C_CTRL_STOPMODE_NORMAL)
    {
        g_e8MotorCtrlMode = (uint8_t)C_MOTOR_CTRL_NORMAL;
    }
    else if (pCtrl->u2StopMode == (uint8_t)C_CTRL_STOPMODE_STOP)
    {
        g_e8MotorRequest = (uint8_t)C_MOTOR_REQUEST_STOP;
        g_e8MotorCtrlMode = (uint8_t)C_MOTOR_CTRL_STOP;
    }
#if (LINPROT == LIN2X_HVAC52)                                                   /* MMP201217-2 */
    else if (pCtrl->u2StopMode == (uint8_t)C_CTRL_STOPMODE_NORMAL_GAD)
    {
        g_e8MotorCtrlMode = (uint8_t)C_MOTOR_CTRL_NORMAL_GAD;
    }
#endif /* (LINPROT == LIN2X_HVAC52) */
    else
    {
        /* Nothing */
    }
} /* End of ModeChange() */

/*!*************************************************************************** *
 * SaveProgrammingData
 * \brief   LIN Control Message Handler
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] pCtrl: Pointer to LIN Control Message
 * \return  -
 * *************************************************************************** *
 * \details Note: Function can only be called when Non Volatile Memory Write is inactive!
 * *************************************************************************** *
 * - Call Hierarchy: HandleActCtrl()
 * - Cyclomatic Complexity: 8+1
 * - Nesting: 2
 * - Function calling: 1 (NV_WriteEOL())
 * *************************************************************************** */
static inline void SaveProgrammingData(HVAC_CTRL *pCtrl)
{
    APP_EOL_t EOL_New;
    uint8_t u8OrgActDirection = l_u8ActDirection;
    p_CopyU16( (sizeof(APP_EOL_t) / sizeof(uint16_t)),
               (uint16_t *)&EOL_New,
               (const uint16_t *)ADDR_NV_EOL);
    if (pCtrl->u2RotationDirection == (uint8_t)C_CTRL_DIR_CW)
    {
        l_u8ActDirection = FALSE;
        EOL_New.u1MotorDirectionCCW = l_u8ActDirection;
    }
    else if (pCtrl->u2RotationDirection == (uint8_t)C_CTRL_DIR_CCW)
    {
        l_u8ActDirection = TRUE;
        EOL_New.u1MotorDirectionCCW = l_u8ActDirection;
    }
    else
    {
        /* Invalid */
    }
    if (pCtrl->u2EmergencyRun == (uint8_t)C_CTRL_EMRUN_ENA)
    {
        EOL_New.u1EmergencyRunPosEna = TRUE;
    }
    else if (pCtrl->u2EmergencyRun == (uint8_t)C_CTRL_EMRUN_DIS)
    {
        EOL_New.u1EmergencyRunPosEna = FALSE;
    }
    else
    {
        /* Invalid */
    }
    if (pCtrl->u2EmergencyEndStop == (uint8_t)C_CTRL_ENRUN_ENDSTOP_HI)
    {
        EOL_New.u7EmergencyRunPos = 100U;
    }
    else if (pCtrl->u2EmergencyEndStop == (uint8_t)C_CTRL_ENRUN_ENDSTOP_LO)
    {
        EOL_New.u7EmergencyRunPos = 0U;
    }
    else
    {
        /* Invalid */
    }

    (void)NV_WriteEOL(&EOL_New);                                                /* Store EOL */
    g_e8MotorDirectionCCW = (uint8_t)C_MOTOR_DIR_UNKNOWN;                       /* Direction is unknown (9.5.3.13) */

    if ( (pCtrl->u2Program == (uint8_t)C_CTRL_PROGRAM_DIS) ||
         (pCtrl->u2Program == (uint8_t)C_CTRL_PROGRAM_ENA) )
    {
        l_u8PrevProgramMode = pCtrl->u2Program;
    }
    if (u8OrgActDirection != l_u8ActDirection)
    {
        g_u16ActualPosition = ActPosition(g_u16ActualPosition, 1);
        g_u16TargetPosition = ActPosition(g_u16TargetPosition, 1);
    }
} /* End of SaveProgrammingData() */

/*!*************************************************************************** *
 * SetupStallDetector
 * \brief   LIN Control Message Handler
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] pCtrl: Pointer to LIN Control Message
 * \return  -
 * *************************************************************************** *
 * \details
 * *************************************************************************** *
 * - Call Hierarchy: HandleActCtrl()
 * - Cyclomatic Complexity: 3+1
 * - Nesting: 3
 * - Function calling: 0
 * *************************************************************************** */
static inline void SetupStallDetector(HVAC_CTRL *pCtrl)
{
    if (g_e8EmergencyRunOcc == (uint8_t)C_SAFETY_RUN_NO)
    {
        if (pCtrl->u2StallDetector == (uint8_t)C_CTRL_STALLDET_DIS)
        {
            g_e8StallDetectorEna = C_STALLDET_NONE;
        }
        else if (pCtrl->u2StallDetector == (uint8_t)C_CTRL_STALLDET_ENA)
        {
            g_e8StallDetectorEna = StallDetectorEna();
        }
        else
        {
            /* Invalid */
        }
    }
} /* End of SetupStallDetector() */

/*!*************************************************************************** *
 * SetSpeedMode
 * \brief   LIN Control Message Handler
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] pCtrl: Pointer to LIN Control Message
 * \return  -
 * *************************************************************************** *
 * \details
 * *************************************************************************** *
 * - Call Hierarchy: HandleActCtrl()
 * - Cyclomatic Complexity: 3+1
 * - Nesting: 3
 * - Function calling: 1 (ADC_Conv_Vmotor())
 * *************************************************************************** */
static inline void SetSpeedMode(HVAC_CTRL *pCtrl)
{
    if ( (pCtrl->u4Speed >= (uint8_t)C_CTRL_SPEED_1) &&
         (pCtrl->u4Speed <= (uint8_t)C_CTRL_SPEED_AUTO) )
    {
#if (_SUPPORT_ACT_SPEED_BY_LIN != FALSE)
        uint8_t u8MotorSpeedIdx = pCtrl->u4Speed;
#if (_SUPPORT_SPEED_AUTO != FALSE)
#if (_SUPPORT_FOC_MODE == FOC_MODE_NONE)
        if (u8MotorSpeedIdx == (uint8_t)C_MOTOR_SPEED_AUTO)
        {
            /* Only allow initial speed "selection" */
            uint16_t u16SupplyVoltage = ADC_Conv_Vmotor();
            if (/*(i16ChipTemperature < C_AUTOSPEED_TEMP_1) ||*/ (u16SupplyVoltage <
                                                                  ((C_AUTOSPEED_VOLT_1 * 25U) / 2U)) )
            {
                /* Temperature below TEMP_1 and/or voltage below VOLT_1 */
                if (u16SupplyVoltage > (((C_AUTOSPEED_VOLT_2 * 25U) / 2U) + 25U) )
                {
                    u8MotorSpeedIdx = (uint8_t)C_MOTOR_SPEED_2;                 /* Speed #2 */
                }
                else
                {
                    u8MotorSpeedIdx = (uint8_t)C_MOTOR_SPEED_1;                 /* Speed #1 */
                }
            }
            else if (/*(i16ChipTemperature > (C_AUTOSPEED_TEMP_1 + 3)) && (i16ChipTemperature < (C_AUTOSPEED_TEMP_2 - 3)) &&*/ (
                         u16SupplyVoltage > (C_AUTOSPEED_VOLT_2 + 25U)) )
            {
                /* temperature above TEMP_1 + 3 degrees and below TEMP_2 - 3 degrees and voltage above VOLT_2 + 25mV */
                u8MotorSpeedIdx = (uint8_t)C_MOTOR_SPEED_4;                     /* Speed #4 */
            }
            else
            {
                u8MotorSpeedIdx = (uint8_t)C_MOTOR_SPEED_2;                     /* Speed #2 */
            }
        }
#endif /* (_SUPPORT_FOC_MODE == FOC_MODE_NONE) */
#endif /* (_SUPPORT_SPEED_AUTO != FALSE) */
        if (g_u16TargetMotorSpeedRPM != l_au16MotorSpeedRPM[u8MotorSpeedIdx])
        {
            /* Speed change */
            g_u16TargetMotorSpeedRPM = l_au16MotorSpeedRPM[u8MotorSpeedIdx];
            if (g_e8MotorRequest != (uint8_t)C_MOTOR_REQUEST_STOP)
            {
                g_e8MotorRequest = (uint8_t)C_MOTOR_REQUEST_SPEED_CHANGE;
            }
            l_u8MotorCtrlSpeed = u8MotorSpeedIdx;
        }
#else  /* (_SUPPORT_ACT_SPEED_BY_LIN != FALSE) */
        if (g_u8MotorCtrlSpeed != pCtrl->u4Speed)
        {
#if (_SUPPORT_SPEED_CHANGE != FALSE)
            /* Speed change */
            if (g_e8MotorRequest != (uint8_t)C_MOTOR_REQUEST_STOP)
            {
                g_e8MotorRequest = (uint8_t)C_MOTOR_REQUEST_SPEED_CHANGE;
            }
#endif /* (_SUPPORT_SPEED_CHANGE != FALSE) */
            g_u8MotorCtrlSpeed = pCtrl->u4Speed;
        }
#endif /* (_SUPPORT_ACT_SPEED_BY_LIN != FALSE) */
    }
    else
    {
        g_e8MotorRequest = (uint8_t)C_MOTOR_REQUEST_STOP;                       /* Invalid speed mode; No reaction */
    }
} /* End of SetSpeedMode() */

/*!*************************************************************************** *
 * SetPosition
 * \brief   LIN Control Message Handler
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] pCtrl: Pointer to LIN Control Message
 * \return  -
 * *************************************************************************** *
 * \details
 * *************************************************************************** *
 * - Call Hierarchy: HandleActCtrl()
 * - Cyclomatic Complexity: 7+1
 * - Nesting: 4
 * - Function calling: 0
 * *************************************************************************** */
static inline void SetPosition(HVAC_CTRL *pCtrl)
{
    /* Only allow change of CPos or FPos in case not in Event-mode */
    if (pCtrl->u2PositionType == (uint8_t)C_CTRL_POSITION_TARGET)
    {
        /* Accept (new) FPOS, in case not in Event-mode */
        uint16_t u16Value = (((uint16_t)pCtrl->u8TargetPositionMSB) << 8) |
                            ((uint16_t)pCtrl->u8TargetPositionLSB);
        if (u16Value != C_INV_POS)                                              /* Check for a valid FPos */
        {
            g_u16TargetPosition = ActPosition(u16Value, l_u8ActDirection);
            l_e8PositionType = (uint8_t)C_POSTYPE_TARGET;
            if (g_e8MotorRequest != (uint8_t)C_MOTOR_REQUEST_STOP)
            {
#if (_SUPPORT_DEGRADED_MODE != FALSE)
                if (g_e8DegradeStatus != FALSE)
                {
                    g_e8DegradedMotorRequest = (uint8_t)C_MOTOR_REQUEST_START;
                }
                else
#endif /* (_SUPPORT_DEGRADED_MODE != FALSE) */
                {
                    g_e8MotorRequest = (uint8_t)C_MOTOR_REQUEST_START;
                }
            }
        }
    }
#if (_SUPPORT_POS_INIT != FALSE)
    else if ( (pCtrl->u2PositionType == (uint8_t)C_CTRL_POSITION_INITIAL) &&
              ((g_e8MotorStatus & C_MOTOR_STATUS_STOP_MASK) == C_MOTOR_STATUS_STOP) ) /* See 9.4.3.6 */
    {
        /* Accept (new) CPOS, in case not normal-running and not in Event-mode */
        uint16_t u16Value = (((uint16_t)pCtrl->u8StartPositionMSB) << 8) |
                            ((uint16_t)pCtrl->u8StartPositionLSB);
        if (u16Value != C_INV_POS)                                              /* Check for a valid CPos */
        {
            g_u16ActualPosition = g_u16TargetPosition = ActPosition(u16Value, l_u8ActDirection);
            l_e8PositionType = (uint8_t)C_POSTYPE_INIT;
            g_e8MotorRequest = (uint8_t)C_MOTOR_REQUEST_INIT;
#if (_SUPPORT_DEGRADED_MODE != FALSE)
            if (g_e8DegradeStatus != FALSE)
            {
                g_e8DegradedMotorRequest = (uint8_t)C_MOTOR_REQUEST_NONE;
            }
#endif /* (_SUPPORT_DEGRADED_MODE != FALSE) */
        }
    }
    else
    {
        /* Nothing */
    }
#endif /* (_SUPPORT_POS_INIT != FALSE) */
} /* End of SetPosition() */

/*!*************************************************************************** *
 * SetHoldingCurrent
 * \brief   LIN Control Message Handler
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] pCtrl: Pointer to LIN Control Message
 * \return  -
 * *************************************************************************** *
 * \details
 * *************************************************************************** *
 * - Call Hierarchy: HandleActCtrl()
 * - Cyclomatic Complexity: 3+1
 * - Nesting: 3
 * - Function calling: 0
 * *************************************************************************** */
static inline void SetHoldingCurrent(HVAC_CTRL *pCtrl)
{
    if ( (pCtrl->u2HoldingCurrent == (uint8_t)C_CTRL_MHOLDCUR_DIS) ||
         (pCtrl->u2HoldingCurrent == (uint8_t)C_CTRL_MHOLDCUR_ENA) )
    {
        uint8_t u8HoldingCurrEna = (pCtrl->u2HoldingCurrent == (uint8_t)C_CTRL_MHOLDCUR_ENA) ? TRUE : FALSE;
        if (g_u8MotorHoldingCurrEna != u8HoldingCurrEna)
        {
            /* Change of motor holding current setting */
            if ( (g_e8MotorRequest == (uint8_t)C_MOTOR_REQUEST_NONE) &&
                 ((g_e8MotorStatus & C_MOTOR_STATUS_STOP_MASK) == C_MOTOR_STATUS_STOP) )
            {
                g_e8MotorRequest = (uint8_t)C_MOTOR_REQUEST_STOP;
            }
            g_u8MotorHoldingCurrEna = u8HoldingCurrEna;
        }
    }
} /* End of SetHoldingCurrent() */

#if (LINPROT == LIN2X_HVAC52) && (_SUPPORT_HVAC_GROUP_ADDRESS != FALSE)
/*!*************************************************************************** *
 * HandleActCtrl
 * \brief   LIN Control Message Handler
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16Group: FALSE: Control Message
 *                         TRUE: Group Message
 * \return  -
 * *************************************************************************** *
 * \details
 *  Message ID: 0x0001
 *  Message size: 8-bytes
 *  Repetitive time: min. 10ms (at 19200 Baud)
 *
 *          | Bit 7 | Bit 6 | Bit 5 | Bit 4 | Bit 3 | Bit 2 | Bit 1 | Bit 0 |
 *          +---------------+---------------+---------------+---------------+
 *  Byte 8  |  Motor modus  | Rot.Direction*| EmRun EndStop*| Emergency Run*|
 *          +---------------+---------------+---------------+---------------+
 *  Byte 7  |                 Start position Actuator (MSB)                 |
 *          +---------------------------------------------------------------+
 *  Byte 6  |                 Start position Actuator (LSB)                 |
 *          +---------------------------------------------------------------+
 *  Byte 5  |                Target position Actuator (MSB)                 |
 *          +---------------------------------------------------------------+
 *  Byte 4  |                Target position Actuator (LSB)                 |
 *          +-------------------------------+---------------+---------------+
 *  Byte 3  |       Speed selection         | PositionType  |Holding current|
 *          +-------------------------------+---------------+---------------+
 *  Byte 2  |       Clear Event flags       | Stall Detector| Program Data* |
 *          +-------------------------------+---------------+---------------+
 *  Byte 1  |                   Address Actuator (NAD)                      |
 *          +---------------------------------------------------------------+
 *
 *  Program Data:   Rotational Direction CW/CCW
 *                  Emergency Run enable/disable
 *                  Emergency Run end-stop Low/High
 *  RAM data:   Stall detection
 *              Speed
 *              Motor holding current
 *              Motor mode
 * *************************************************************************** *
 * - Call Hierarchy: HandleLinInMsg()
 * - Cyclomatic Complexity: 4+1
 * - Nesting: 2
 * - Function calling: 7 (ClearEventFlags(), ModeChange(), SaveProgrammingData(),
 *                        SetupStallDetector(), SetSpeedMode(), SetPosition(),
 *                        SetHoldingCurrent())
 * *************************************************************************** */
void HandleActCtrl(uint16_t u16Group)
#else  /* (LINPROT == LIN2X_HVAC52) && (_SUPPORT_HVAC_GROUP_ADDRESS != FALSE) */
/*!*************************************************************************** *
 * HandleActCtrl
 * \brief   LIN Control Message Handler
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details
 *  Message ID: 0x0001
 *  Message size: 8-bytes
 *  Repetitive time: min. 10ms (at 19200 Baud)
 *
 *          | Bit 7 | Bit 6 | Bit 5 | Bit 4 | Bit 3 | Bit 2 | Bit 1 | Bit 0 |
 *          +---------------+---------------+---------------+---------------+
 *  Byte 8  |  Motor modus  | Rot.Direction*| EmRun EndStop*| Emergency Run*|
 *          +---------------+---------------+---------------+---------------+
 *  Byte 7  |                 Start position Actuator (MSB)                 |
 *          +---------------------------------------------------------------+
 *  Byte 6  |                 Start position Actuator (LSB)                 |
 *          +---------------------------------------------------------------+
 *  Byte 5  |                Target position Actuator (MSB)                 |
 *          +---------------------------------------------------------------+
 *  Byte 4  |                Target position Actuator (LSB)                 |
 *          +-------------------------------+---------------+---------------+
 *  Byte 3  |       Speed selection         | PositionType  |Holding current|
 *          +-------------------------------+---------------+---------------+
 *  Byte 2  |       Clear Event flags       | Stall Detector| Program Data* |
 *          +-------------------------------+---------------+---------------+
 *  Byte 1  |                   Address Actuator (NAD)                      |
 *          +---------------------------------------------------------------+
 *
 *  Program Data:   Rotational Direction CW/CCW
 *                  Emergency Run enable/disable
 *                  Emergency Run end-stop Low/High
 *  RAM data:   Stall detection
 *              Speed
 *              Motor holding current
 *              Motor mode
 * *************************************************************************** *
 * - Call Hierarchy: HandleLinInMsg()
 * - Cyclomatic Complexity: 4+1
 * - Nesting: 2
 * - Function calling: 7 (ClearEventFlags(), ModeChange(), SaveProgrammingData(),
 *                        SetupStallDetector(), SetSpeedMode(), SetPosition(),
 *                        SetHoldingCurrent())
 * *************************************************************************** */
void HandleActCtrl(void)
#endif /* (LINPROT == LIN2X_HVAC52) && (_SUPPORT_HVAC_GROUP_ADDRESS != FALSE) */
{
#if (_SUPPORT_REWIND != FALSE)
    if (g_u8RewindFlags & (uint8_t)C_REWIND_REWIND)
    {
        g_u8LinInFrameBufState = (uint8_t)C_LIN_IN_POSTPONE;
    }
    else
#endif /* (_SUPPORT_REWIND != FALSE) */
    {
        HVAC_CTRL *pCtrl = (HVAC_CTRL *)&(g_LinCmdFrameBuffer.Ctrl);
#if (LINPROT == LIN2X_HVAC52) && (_SUPPORT_HVAC_GROUP_ADDRESS != FALSE)
        /* Check for correct NAD (Broadcast) and not LIN-AA mode */
        if ( (((u16Group == FALSE) && (pCtrl->u8NAD == g_u8NAD)) || /*lint !e845 */
              ((pCtrl->u8NAD == l_u8GAD) && (pCtrl->u8NAD != C_INVALD_GAD)) || /*lint !e845 */
              (pCtrl->u8NAD == (uint8_t)C_BROADCAST_NAD))
#if (_SUPPORT_LIN_AA != FALSE)
             && (g_u8LinAAMode == (uint8_t)C_SNPD_SUBFUNC_INACTIVE)
#endif /* (_SUPPORT_LIN_AA != FALSE) */
             )
#else  /* (LINPROT == LIN2X_HVAC52) && (_SUPPORT_HVAC_GROUP_ADDRESS != FALSE) */
        /* Check for correct NAD (Broadcast) and not LIN-AA mode */
        if ( ((pCtrl->u8NAD == g_u8NAD) || (pCtrl->u8NAD == (uint8_t)C_BROADCAST_NAD))
#if (_SUPPORT_LIN_AA != FALSE)
             && (g_u8LinAAMode == (uint8_t)C_SNPD_SUBFUNC_INACTIVE)
#endif /* (_SUPPORT_LIN_AA != FALSE) */
             )
#endif /* (LINPROT == LIN2X_HVAC52) && (_SUPPORT_HVAC_GROUP_ADDRESS != FALSE) */
        {
            /* Priority 1: Clear Event Flags */
            uint8_t u8EventMode = ClearEventFlags(pCtrl);

            /* Priority 2: Change of mode (STOP, NORMAL) */
            if (u8EventMode == FALSE)
            {
                ModeChange(pCtrl);
            }

            /* Priority 3: Store data into NVRAM */
            if ( (pCtrl->u2Program == (uint8_t)C_CTRL_PROGRAM_ENA) &&
                 (g_e8MotorCtrlMode == (uint8_t)C_MOTOR_CTRL_STOP) &&
                 (u8EventMode == FALSE) && (l_u8PrevProgramMode == (uint8_t)C_CTRL_PROGRAM_DIS) )
            {
                /* Store info into NVRAM (Rotational-direction, Emergency-run info), only in Stop-mode, and not Event-mode */
                SaveProgrammingData(pCtrl);
            }

            /* Priority 4: Stall detection enable/disable */
            SetupStallDetector(pCtrl);

            /* Priority 5: Speed */
            SetSpeedMode(pCtrl);

            /* Priority 6: CPos/FPos update */
            if (u8EventMode == FALSE)                                           /* See 9.4.3.4 */
            {
                SetPosition(pCtrl);
            }

            /* Priority 7: Holding current enable/disable */
            SetHoldingCurrent(pCtrl);
        }
    }
} /* End of HandleActCtrl() */

/*!*************************************************************************** *
 * GetErrorEvents
 * \brief   LIN Status Message Request
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] pStatus: Pointer to LIN Status Message
 * \return  -
 * *************************************************************************** *
 * \details
 * *************************************************************************** *
 * - Call Hierarchy: HandleActStatus()
 * - Cyclomatic Complexity: 9+1
 * - Nesting: 2
 * - Function calling: 0
 * *************************************************************************** */
static inline void GetErrorEvents(volatile HVAC_STATUS *pStatus)
{
    if (g_u8ErrorCommunication != FALSE)
    {
        pStatus->u1ResponseError = TRUE;
    }
    else
    {
        pStatus->u1ResponseError = FALSE;
    }
    g_u8ErrorCommunication = FALSE;                                             /* Data requested; No longer communication error */
    pStatus->u1Reserved1 = 1U;                                                  /* MMP201217-1: Reserved-bit should be '1' */
    if (g_e8ErrorOverTemperature != (uint8_t)C_ERR_OTEMP_NO)
    {
        pStatus->u2OverTemperature = (uint8_t)C_STATUS_OTEMP_YES;
    }
    else
    {
        pStatus->u2OverTemperature = (uint8_t)C_STATUS_OTEMP_NO;
    }
    if (g_e8ErrorElectric == (uint8_t)C_ERR_NONE)
    {
        pStatus->u2ElectricDefect = C_STATUS_ELECDEFECT_NO;
    }
    else if ( (g_e8ErrorElectric & (uint8_t)C_ERR_PERMANENT) != 0U)
    {
        pStatus->u2ElectricDefect = C_STATUS_ELECDEFECT_PERM;
    }
    else
    {
        pStatus->u2ElectricDefect = C_STATUS_ELECDEFECT_YES;
        if ( (g_e8ErrorElectric & (uint8_t)C_ERR_SEMI_PERMANENT) == 0U)
        {
            g_e8ErrorElectric = (uint8_t)C_ERR_NONE;                            /* Only "clear" Electric Error if not (semi)permanent */
        }
    }
    pStatus->u2VoltageError = (uint8_t)(g_e8ErrorVoltage & 0x03);
    if ( (pStatus->u2VoltageError == 0U) && (g_e8ErrorVoltageComm != 0U) )
    {
        pStatus->u2VoltageError = (uint8_t)(g_e8ErrorVoltageComm & 0x03U);      /* At least communicate once the voltage-error */
    }
    g_e8ErrorVoltageComm = g_e8ErrorVoltage;                                    /* 9.5.3.4 */
    /* Byte 2 */
    if (g_e8EmergencyRunOcc != (uint8_t)C_SAFETY_RUN_NO)
    {
        pStatus->u2EmergencyOccurred = (uint8_t)C_STATUS_EMRUNOCC_YES;
    }
    else
    {
        pStatus->u2EmergencyOccurred = (uint8_t)C_STATUS_EMRUNOCC_NO;
    }
    if (g_e8StallDetectorEna != (uint8_t)C_STALLDET_NONE)
    {
        pStatus->u2StallDetector = (uint8_t)C_STATUS_STALLDET_ENA;
    }
    else
    {
        pStatus->u2StallDetector = (uint8_t)C_STATUS_STALLDET_DIS;
    }
    if (g_u8StallOcc != FALSE)
    {
        pStatus->u2StallOccurred = (uint8_t)C_STATUS_STALLOCC_YES;
    }
    else
    {
        pStatus->u2StallOccurred = (uint8_t)C_STATUS_STALLOCC_NO;
    }
    if (g_u8ChipResetOcc != FALSE)
    {
        pStatus->u2Reset = (uint8_t)C_STATUS_RESETOCC_YES;
    }
    else
    {
        pStatus->u2Reset = (uint8_t)C_STATUS_RESETOCC_NO;
    }
} /* End of GetErrorEvents() */

/*!*************************************************************************** *
 * GetProgrammingData
 * \brief   LIN Status Message Request
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] pStatus: Pointer to LIN Status Message
 * \return  -
 * *************************************************************************** *
 * \details
 * *************************************************************************** *
 * - Call Hierarchy: HandleActStatus()
 * - Cyclomatic Complexity: 3+1
 * - Nesting: 1
 * - Function calling: 0
 * *************************************************************************** */
static inline void GetProgrammingData(volatile HVAC_STATUS *pStatus)
{
#if (_SUPPORT_NV_NON_BLOCK_WRITE == FALSE)
    /* NOTE: As this function is called by LIN mlu_DataRequest callback IRQ,
     *       Non Volatile Memory Write should not be busy */
    APP_EOL_t *pEOL = (APP_EOL_t *)ADDR_NV_EOL;
    if (pEOL->u1EmergencyRunPosEna != 0U)                                       /* Non Volatile Memory stored state */
    {
        pStatus->u2EmergencyRun = (uint8_t)C_STATUS_EMRUN_ENA;
    }
    else
    {
        pStatus->u2EmergencyRun = (uint8_t)C_STATUS_EMRUN_DIS;
    }
    if (pEOL->u7EmergencyRunPos != 0U)                                          /* Non Volatile Memory stored state */
    {
        pStatus->u2EmergencyRunEndStop = (uint8_t)C_STATUS_EMRUN_ENDPOS_HI;
    }
    else
    {
        pStatus->u2EmergencyRunEndStop = (uint8_t)C_STATUS_EMRUN_ENDPOS_LO;
    }
    if (pEOL->u1MotorDirectionCCW != 0U)                                        /* Non Volatile Memory stored state */
    {
        pStatus->u2RotationDirection = (uint8_t)C_STATUS_DIRECTION_CCW;
    }
    else
    {
        pStatus->u2RotationDirection = (uint8_t)C_STATUS_DIRECTION_CW;
    }
#else  /* (_SUPPORT_NV_NON_BLOCK_WRITE == FALSE) */
    if (g_u8EmergencyRunPosEna_NV!= 0U)                                        /* Non Volatile Memory stored state */
    {
        pStatus->u2EmergencyRun = (uint8_t)C_STATUS_EMRUN_ENA;
    }
    else
    {
        pStatus->u2EmergencyRun = (uint8_t)C_STATUS_EMRUN_DIS;
    }
    if (g_u8EmergencyRunPos_NV!= 0U)                                           /* Non Volatile Memory stored state */
    {
        pStatus->u2EmergencyRunEndStop = (uint8_t)C_STATUS_EMRUN_ENDPOS_HI;
    }
    else
    {
        pStatus->u2EmergencyRunEndStop = (uint8_t)C_STATUS_EMRUN_ENDPOS_LO;
    }
    if (g_u8MotorDirectionCCW_NV!= 0U)                                         /* Non Volatile Memory stored state */
    {
        pStatus->u2RotationDirection = (uint8_t)C_STATUS_DIRECTION_CCW;
    }
    else
    {
        pStatus->u2RotationDirection = (uint8_t)C_STATUS_DIRECTION_CW;
    }
#endif /* (_SUPPORT_NV_NON_BLOCK_WRITE == FALSE) */
} /* End of GetProgrammingData() */

/*!*************************************************************************** *
 * HandleActStatus
 * \brief   Handle LIN Actuator Status Request
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details
 *  Message ID: 0x0002
 *  Message size: 8-bytes
 *  Repetitive time: min. 10ms (at 19200 Baud)
 *
 *          | Bit 7 | Bit 6 | Bit 5 | Bit 4 | Bit 3 | Bit 2 | Bit 1 | Bit 0 |
 *          +---------------+---------------+---------------+---------------+
 *  Byte 8  |  Motor modus  | Rot.Direction*| EmRun EndStop*| Emergency Run*|
 *          +---------------+---------------+---------------+---------------+
 *  Byte 7  |                       Node Address (NAD)                      | (HVAC Std. Actuator spec V4.4)
 *          +---------------+---------------+---------------+---------------+
 *  Byte 6  |    Reserved   | Special Funct |  Holding Mode |   Direction   | (HVAC Std. Actuator spec V4.4)
 *          +---------------+---------------+---------------+---------------+
 *  Byte 5  |                Actual position Actuator (MSB)                 |
 *          +---------------------------------------------------------------+
 *  Byte 4  |                Actual position Actuator (LSB)                 |
 *          +-------------------------------+---------------+---------------+
 *  Byte 3  |           Speed status        | PositionType  |Holding current|
 *          +---------------+---------------+---------------+---------------+
 *  Byte 2  |   Chip Reset  |Stall occurred | Stall Detector| Emerg.Run Occ |
 *          +---------------+---------------+---------------+-------+-------+
 *  Byte 1  | Voltage Error |Electric Defect|OverTemperature|  Res  | Error |
 *          +---------------+---------------+-----------------------+-------+
 * *************************************************************************** *
 * - Call Hierarchy: mlu_DataRequest()
 * - Cyclomatic Complexity: 11+1
 * - Nesting: 3
 * - Function calling: 6 (ml_DiscardFrame(), ActPosition(), ml_DataReady(),
 *                        GetErrorEvents(), GetProgrammingData(), SetLastError())
 * *************************************************************************** */
void HandleActStatus(void)
{
#if (_SUPPORT_LIN_AA != FALSE)
    if (g_u8LinAAMode != (uint8_t)C_SNPD_SUBFUNC_INACTIVE)
    {
        (void)ml_DiscardFrame();                                                /* Do not respond on ACT_STATUS during LIN-AA */
    }
    else
#endif /* (_SUPPORT_LIN_AA != FALSE) */
    {
#if defined (__MLX81330A01__) || defined (__MLX81332A01__)
        volatile HVAC_STATUS *pStatus = (HVAC_STATUS *)((void *)LinFrameDataBuffer);
#else
        volatile HVAC_STATUS *pStatus = (HVAC_STATUS *)((void *)ML_DATA_LIN_FRAME_DATA_BUFFER);
#endif
        uint16_t i;
        for (i = 0U; i < sizeof(HVAC_STATUS) / sizeof(uint16_t); i++)
        {
            ((uint16_t *)pStatus)[i] = 0xFFFFU;                                 /* Fields set to 0xFFFF or 0b11 are invalid */
        }

        /* Byte 1-2 */
        GetErrorEvents(pStatus);
        /* Byte 3 */
        if (g_u8MotorHoldingCurrEna != FALSE)
        {
            pStatus->u2HoldingCurrent = (uint8_t)C_STATUS_MHOLDCUR_ENA;
        }
        else
        {
            pStatus->u2HoldingCurrent = (uint8_t)C_STATUS_MHOLDCUR_DIS;
        }
        if (l_e8PositionType == (uint8_t)C_POSTYPE_INIT)
        {
            pStatus->u2PositionTypeStatus = (uint8_t)C_STATUS_POSITION_INIT;
        }
        else
        {
            pStatus->u2PositionTypeStatus = (uint8_t)C_STATUS_POSITION_ACTUAL;
        }
        if ( (g_e8MotorStatus & C_MOTOR_STATUS_STOP_MASK) != C_MOTOR_STATUS_STOP)
        {
            /* pStatus->u4SpeedStatus = g_u8MotorStatusSpeed; */                /* Actual Speed */
#if (_SUPPORT_ACT_SPEED_BY_LIN != FALSE)
            pStatus->u4SpeedStatus = l_u8MotorCtrlSpeed;                        /* Requested Speed */
#else  /* (_SUPPORT_ACT_SPEED_BY_LIN != FALSE) */
            pStatus->u4SpeedStatus = g_u8MotorCtrlSpeed;                        /* Requested Speed */
#endif /* (_SUPPORT_ACT_SPEED_BY_LIN != FALSE) */
        }
        else
        {
            pStatus->u4SpeedStatus = 0U;
        }
        /* Byte 4..5 */
        {
            uint16_t u16CopyPosition = ActPosition(g_u16ActualPosition, l_u8ActDirection);
            pStatus->u8ActualPositionLSB = (u16CopyPosition & 0xFFU);
            pStatus->u8ActualPositionMSB = (u16CopyPosition >> 8);
        }
        /* Byte 6 */
        if (g_e8MotorDirectionCCW == (uint8_t)C_MOTOR_DIR_UNKNOWN)
        {
            pStatus->u2ActualRotationalDir = (uint8_t)C_STATUS_ACT_DIR_UNKNOWN;
        }
        else if ( ((g_e8MotorDirectionCCW & 1U) ^ l_u8ActDirection) != 0U)
        {
            pStatus->u2ActualRotationalDir = (uint8_t)C_STATUS_ACT_DIR_CLOSING;
        }
        else
        {
            pStatus->u2ActualRotationalDir = (uint8_t)C_STATUS_ACT_DIR_OPENING;
        }
        pStatus->u2SelfHoldingTorque = (uint8_t)C_STATUS_HOLDING_TORQUE_INV;
#if (_SUPPORT_REWIND != FALSE)
        if (g_u8RewindFlags & (uint8_t)C_REWIND_ACTIVE)
        {
            pStatus->u2SpecialFunctionActive = (uint8_t)C_STATUS_SFUNC1_ACTIVE_YES;
        }
        else
#endif /* (_SUPPORT_REWIND != FALSE) */
        {
            pStatus->u2SpecialFunctionActive = (uint8_t)C_STATUS_SFUNC_ACTIVE_NO;
        }
#if (_SUPPORT_HALL_LATCH != FALSE) && (_SUPPORT_NR_OF_HL == 1U)
        if (g_u16IoState != 0U)
        {
            pStatus->u2Reserved = 1U;
        }
        else
        {
            pStatus->u2Reserved = 0U;
        }
#elif (_SUPPORT_FOC_MODE != FOC_MODE_NONE)
        if ( (l_e8MotorStartupMode & E_MSM_MODE_MASK) == E_MSM_STEPPER)
        {
            pStatus->u2Reserved = 0U;                                           /* Stepper-mode */
        }
        else
        {
            pStatus->u2Reserved = 1U;                                           /* FOC mode */
        }
#else
        pStatus->u2Reserved = 3U;
#endif /* (_SUPPORT_HALL_LATCH != FALSE) && (_SUPPORT_NR_OF_HL == 1U) */
        /* Byte 7 */
#if (LINPROT == LIN2X_HVAC52)                                                   /* MMP201217-2 */
        if ( (g_e8MotorCtrlMode == (uint8_t)C_MOTOR_CTRL_NORMAL_GAD) &&
             (((ENH_LIN_PARAMS_t *)ADDR_NV_ENH_LIN_1)->u16FunctionID & 0x8000U) != 0U)    /* Check EE Function-ID is Group Function-ID */
        {
            pStatus->u8NAD = l_u8GAD;
        }
        else
#endif /* (LINPROT == LIN2X_HVAC52) */
        {
            pStatus->u8NAD = g_u8NAD;
        }
        /* Byte 8 */
        GetProgrammingData(pStatus);
        if (g_e8MotorCtrlMode == (uint8_t)C_MOTOR_CTRL_STOP)
        {
            pStatus->u2StopMode = (uint8_t)C_STATUS_STOPMODE_STOP;
        }
        else
        {
            pStatus->u2StopMode = (uint8_t)C_STATUS_STOPMODE_NORMAL;
        }

        if (COLIN_LINstatus.buffer_used == 0U)                                  /* MMP240408-1 */
        {
            if (ml_DataReady(ML_END_OF_TX_DISABLED) != ML_SUCCESS)
            {
                g_u8ErrorCommunication = TRUE;
#if (_SUPPORT_LOG_ERRORS != FALSE)
                SetLastError(C_ERR_LIN_API);
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
            }
        }
#if (_SUPPORT_LOG_ERRORS != FALSE)
        else
        {
            g_u8ErrorCommunication = TRUE;
            SetLastError(C_ERR_LIN_BUF_NOT_FREE);
        }
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
    }
} /* End of HandleActStatus() */

/*!*************************************************************************** *
 * HandleBusTimeout
 * \brief   Handle LIN Bus Timeout
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details After finishing the emergency-run
 * *************************************************************************** *
 * - Call Hierarchy: mlu_LinSleepMode()
 * - Cyclomatic Complexity: 4+1
 * - Nesting: 3
 * - Function calling: 2 (SetLastError(), LinAATimeoutControl())
 * *************************************************************************** */
void HandleBusTimeout(void)
{
#if (_SUPPORT_LIN_AA != FALSE)
    if (g_u8LinAAMode != (uint8_t)C_SNPD_SUBFUNC_INACTIVE)
    {
        LinAATimeoutControl();
    }
#endif /* (_SUPPORT_LIN_AA != FALSE) */
#if (_SUPPORT_BUSTIMEOUT != FALSE)
    if (g_u8ErrorCommBusTimeout == FALSE)
    {
        /* Emergency run is enabled */
        g_u8ErrorCommBusTimeout = TRUE;
#if (_SUPPORT_LOG_ERRORS != FALSE)
        SetLastError(C_ERR_LIN_BUS_TIMEOUT);
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
        {
            /* NOTE: As this function is called by LIN mlu_LinSleepMode callback IRQ,
             *       Non Volatile Memory Write should not be busy */
            APP_EOL_t *pEOL = (APP_EOL_t *)ADDR_NV_EOL;
            if (pEOL->u1EmergencyRunPosEna != 0U)                               /* Non Volatile Memory stored state */
            {
#if (_SUPPORT_DEGRADED_MODE != FALSE)
                if (g_e8DegradeStatus != FALSE)
                {
                    /* Module is in degraded-mode; Postpone emergency-run till after degraded-mode have been obsoleted */
                    g_e8MotorRequest = (uint8_t)C_MOTOR_REQUEST_EMRUN;
                    g_e8DegradedMotorRequest = g_e8MotorRequest;
                }
                else
#endif /* (_SUPPORT_DEGRADED_MODE != FALSE) */
                {
                    /* Perform emergency-run immediately */
                    g_e8MotorRequest = (uint8_t)C_MOTOR_REQUEST_EMRUN;
                }
            }
#if (_SUPPORT_LIN_SLEEP != FALSE) && (_SUPPORT_BUSTIMEOUT_SLEEP != FALSE)
            else if (NV_BUSTIMEOUT_SLEEP != 0U)
            {
                g_e8MotorRequest = (uint8_t)C_MOTOR_REQUEST_SLEEP;
            }
#endif /* (_SUPPORT_LIN_SLEEP != FALSE) && (_SUPPORT_BUSTIMEOUT_SLEEP != FALSE) */
        }
    }
#endif /* (_SUPPORT_BUSTIMEOUT != FALSE) */
} /* End of HandleBusTimeout() */

/*!*************************************************************************** *
 * HandleDataTransmitted
 * \brief   Handle LIN Data Transmitted
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] Index: Message ID
 * \return  -
 * *************************************************************************** *
 * \details Handle the LIN Data transmitted event.
 *          Clear communication error-flag, if it was send successful.
 * *************************************************************************** *
 * - Call Hierarchy: mlu_DataTransmitted()
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 0
 * *************************************************************************** */
void HandleDataTransmitted(ml_MessageID_t Index)
{
    (void)Index;
} /* End of HandleDataTransmitted() */

/*!*************************************************************************** *
 * HandleLinError
 * \brief   Handle LIN errors
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] Error: LIN error-code
 * \return  -
 * *************************************************************************** *
 * \details LIN2x communication error handling.
 *          Only report an communication error, in case the LIN slave is addressed.
 *          To see if this slave is addressed, check the NAD in the LinFrame. The
 *          position within the LinFrame depends on the Frame-ID.
 * NOTE: As this function is called by LIN mlu_ErrorDetected callback IRQ, Non Volatile Memory
 *       Write should not be busy
 * *************************************************************************** *
 * - Call Hierarchy: mlu_ErrorDetected()
 * - Cyclomatic Complexity: 7+1
 * - Nesting: 5
 * - Function calling: 0
 * *************************************************************************** */
void HandleLinError(ml_LinError_t Error)
{
    STD_LIN_PARAMS_t *pLIN = (STD_LIN_PARAMS_t *)ADDR_NV_STD_LIN_1;
    uint8_t u8FrameID = (uint8_t)(LinProtectedID & 0x3FU);                      /* Get Frame-ID without parity bits (MMP191207-1) */
    if ( (u8FrameID == (uint8_t)ML_MRF_ID) && ((Error == ml_erDataFraming) || (Error == ml_erCheckSum)) )
    {
        /*
         * Abort Diagnostic communication with corrupted Diagnostic request
         * Checked by LIN2.1 CT test case 13.2.2
         */
        g_u8BufferOutID = (uint8_t)QR_INVALID;
    }

    /* ---- ml_erLinModuleReset -------------------------------------------- */
    if (Error == ml_erLinModuleReset)
    {
        /* Non-recoverable failure has occurred in the LIN Module */
        /* switch to System Mode and reinitialise LIN module */
        Set_Mlx4ErrorState(C_MLX4_STATE_IMMEDIATE_RST);                         /* Set the MLX Error state counter */
    }
    /* ---- ml_erIdParity -------------------------------------------------- */
    else if (Error == ml_erIdParity)
    {
        /* Do NOT set response_error bit, because error occurred in a header */
    }
    else if (u8FrameID == (pLIN->u8StatusFrameID & 0x3FU) )
    {
#if defined (__MLX81350__)
        uint8_t u8CommNAD = (uint8_t) (LinFrame[3] & 0x00FFU);                  /* Seventh byte in LIN-frame is NAD (MMP191207-1) */
#else  /* defined (__MLX81350__) */
        uint8_t u8CommNAD = LinFrame[6];                                        /* Seventh byte in LIN-frame is NAD (MMP191207-1) */
#endif /* defined (__MLX81350__) */
        if (u8CommNAD == g_u8NAD)
        {
            g_u8ErrorCommunication = TRUE;
        }
    }
    else if ( (u8FrameID == (pLIN->u8ControlFrameID & 0x3FU)) ||
              (u8FrameID == ML_MRF_ID) ||
              (u8FrameID == ML_SRF_ID) )
    {
#if defined (__MLX81350__)
        uint8_t u8CommNAD = (uint8_t) (LinFrame[0] & 0x00FFU);                  /* First byte in LIN-frame is NAD (MMP191207-1) */
#else  /* defined (__MLX81350__) */
        uint8_t u8CommNAD = LinFrame[0];                                        /* First byte in LIN-frame is NAD (MMP191207-1) */
#endif /* defined (__MLX81350__) */
        if (u8CommNAD == g_u8NAD)
        {
            g_u8ErrorCommunication = TRUE;
        }
    }
} /* End of HandleLinError() */

#endif /* (LIN_COMM != FALSE) && ((LINPROT == LIN2X_HVAC49) || (LINPROT == LIN2X_HVAC52)) */

/* EOF */
