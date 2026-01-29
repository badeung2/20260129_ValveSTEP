/*!*************************************************************************** *
 * \file        main.c
 * \brief       MLX8133x main application
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
 *           -# main()
 *           -# main_PeriodicTimerEvent()
 *  - Internal Functions:
 *           -# main_noinit_section_init()
 *           -# TriaxisPostInit()
 *           -# main_Init()
 *           -# HandleEmergencyRunMotorRequest()
 *           -# HandleStartMotorRequest()
 *           -# HandleCalibrationMotorRequest()
 *           -# HandleCalibrationFactoryRequest()
 *           -# HandleSleepMotorRequest()
 *           -# HandleMotorRequest()
 *           -# AppBackgroundTaskHandler()
 *
 * \copyright   MELEXIS Microelectronic Integrated Systems
 *
 * Copyright (C) 2017-2024 Melexis N.V.
 * The Software is being delivered 'AS IS' and Melexis, whether explicitly or
 * implicitly, makes no warranty as to its Use or performance.
 * The user accepts the Melexis Firmware License Agreement.
 *
 * Melexis confidential & proprietary
 *
 * *************************************************************************** */

/*!*************************************************************************** */
/*                           INCLUDES                                          */
/* *************************************************************************** */
#include "AppBuild.h"                                                           /* Application Build */

/* Application includes */
#include "main.h"                                                               /* Application support */
#include "ActADC.h"                                                             /* Application ADC support */

/* Actuator Library includes */
#include "drivelib/AppFunctions.h"                                              /* Application Functions support */
#include "drivelib/ErrorCodes.h"                                                /* Error-logging support */
#include "drivelib/GlobalVars.h"                                                /* Global Variables support */
#if (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT)
#include "drivelib/MotorDriver.h"                                               /* Motor driver support */
#include "drivelib/MotorStall.h"                                                /* Motor Stall Detectors */
#if (_SUPPORT_MOTION_DET != C_MOTION_DET_NONE)
#include "drivelib/MotionDetector.h"                                            /* Motion Detector support */
#endif /* (_SUPPORT_MOTION_DET != C_MOTION_DET_NONE) */
#elif (_SUPPORT_APP_TYPE == C_APP_SOLENOID)
#include "drivelib/SolenoidDriver.h"                                            /* Solenoid Driver support */
#endif /* (_SUPPORT_APP_TYPE) */
#include "drivelib/PID_Control.h"                                               /* PID support */
#include "camculib/private_mathlib.h"                                           /* Private Math Library support */

/* Actuator Sensor includes */
#if (_SUPPORT_TRIAXIS_MLX90363 != FALSE)
#include "senselib/Triaxis_MLX90363.h"                                          /* (SPI) Triaxis MLX90363 support */
#elif (_SUPPORT_TRIAXIS_MLX90367 != FALSE) || (_SUPPORT_TRIAXIS_MLX90372 != FALSE)
#include "senselib/Triaxis_MLX90367_372.h"                                      /* (SENT) Triaxis MLX90367/372 support */
#elif (_SUPPORT_TRIAXIS_MLX90377 != FALSE)
#include "senselib/Triaxis_MLX90377.h"                                          /* (PWM/SENT/SPC) Triaxis MLX90377 support */
#elif (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
#include "senselib/Triaxis_MLX9038x.h"                                          /* (ANA) Triaxis MLX9038x support */
#elif (_SUPPORT_TRIAXIS_MLX90395 != FALSE)
#include "senselib/Triaxis_MLX90395.h"                                          /* (SPI) Triaxis MLX90395 support */
#elif (_SUPPORT_TRIAXIS_MLX90421 != FALSE) || (_SUPPORT_TRIAXIS_MLX90425 != FALSE)
#include "senselib/Triaxis_MLX90421_425.h"                                      /* (PWM) Triaxis MLX90421 or MLX90425 support */
#elif (_SUPPORT_TRIAXIS_MLX90422 != FALSE) || (_SUPPORT_TRIAXIS_MLX90426 != FALSE)
#include "senselib/Triaxis_MLX90422_426.h"                                      /* (SENT) Triaxis MLX90422 or MLX90426 support */
#elif (_SUPPORT_TRIAXIS_MLX90427 != FALSE)
#include "senselib/Triaxis_MLX90427.h"                                          /* (SPI) Triaxis MLX90427 support */
#elif (_SUPPORT_INDUCTIVE_POS_SENSOR_MLX90513 != FALSE)
#include "senselib/InductivePosSensor_MLX90513.h"                               /* (SENT/SPC/PWM) Inductive Position Sensor MLX90513 */
#elif (_SUPPORT_PRESSURE_MLX90829 != FALSE)
#include "senselib/Pressure_MLX90829.h"                                         /* (SENT) Pressure Sensor MLX90829 support */
#endif /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */
#if (_SUPPORT_DUAL_HALL_LATCH_MLX92251 != FALSE)
#include "senselib/DualHallLatch_MLX92251.h"                                    /* Dual Hall Latch MLX92251 support */
#endif /* (_SUPPORT_DUAL_HALL_LATCH_MLX92251 != FALSE) */
#if (_SUPPORT_DUAL_HALL_LATCH_MLX92255 != FALSE)
#include "senselib/DualHallLatch_MLX92255.h"                                    /* Dual Hall Latch MLX92255 support */
#endif /* (_SUPPORT_DUAL_HALL_LATCH_MLX92255 != FALSE) */
#if (_SUPPORT_HALL_LATCH != FALSE) && (_SUPPORT_NR_OF_HL == 1U)
#include "senselib/HallLatch.h"                                                 /* Hall-Latch support */
#endif /* (_SUPPORT_HALL_LATCH != FALSE) && (_SUPPORT_NR_OF_HL == 1U) */
#if (_SUPPORT_NTC != FALSE)
#include "senselib/NTC.h"                                                       /* NTC Support */
#endif /* (_SUPPORT_NTC != FALSE) */
#if (_SUPPORT_HUMIDITY_HDC302x != FALSE)
#include "senselib/Humidity_TI_HDC302x.h"                                       /* (I2C) Humidity TI HDC302x Sensor support */
#endif /* (_SUPPORT_HUMIDITY_HDC302x != FALSE) */

/* Communication includes */
#if (CAN_COMM != FALSE)
#include "commlib/CAN_Communication.h"                                          /* CAN Communication support */
#endif /* (CAN_COMM != FALSE) */
#if (GPIO_COMM != FALSE)
#include "commlib/GPIO_OnOff.h"                                                 /* GPIO support */
#endif /* (GPIO_COMM != FALSE) */
#if (I2C_COMM != FALSE) && (_SUPPORT_I2C != FALSE) && ((I2C_MASTER_PROT != I2C_MASTER_PROT_NONE) || (I2C_SLAVE_PROT != I2C_SLAVE_PROT_NONE))
#include "commlib/I2C_Generic.h"                                                /* I2C Generic support */
#endif /* (I2C_COMM != FALSE) && (_SUPPORT_I2C != FALSE) && ((I2C_MASTER_PROT != I2C_MASTER_PROT_NONE) || (I2C_SLAVE_PROT != I2C_SLAVE_PROT_NONE)) */
#if (LIN_COMM != FALSE)
#include "commlib/LIN_Communication.h"                                          /* LIN Communication support */
#endif /* (LIN_COMM != FALSE) */
#if (PWM_COMM != FALSE)
#include "commlib/PWM_Communication.h"                                          /* PWM Communication support */
#endif /* (PWM_COMM != FALSE) */
#if (SPI_COMM != FALSE)
#include "commlib/SPI_Communication.h"                                          /* SPI Communication support */
#elif (_SUPPORT_SPI != FALSE) || (_SUPPORT_3WIRE_SPI != FALSE)
#include "commlib/SPI.h"                                                        /* SPI Support */
#endif /* (_SUPPORT_SPI != FALSE) || (_SUPPORT_3WIRE_SPI != FALSE) */

/* CAMCU Platform includes */
#include <atomic.h>
#include <bl_tools.h>                                                           /* Used by MLX16_RESET_SIGNED */
#include <mathlib.h>                                                            /* Use Melexis math-library functions to avoid compiler warnings */
#include <memory_map.h>                                                         /* Memory map defines */
#if (LIN_COMM != FALSE)
#include <mls_api.h>                                                            /* Melexis LIN module (MMP180430-1) */
#endif /* (LIN_COMM != FALSE) */
#include <plib.h>                                                               /* Product libraries */
#include <sys_tools.h>                                                          /* Platform system tools */

#if (_DEBUG_NV_WRITE_BACKGROUND != FALSE)
#include <eeprom_drv.h>
#endif /* (_DEBUG_NV_WRITE_BACKGROUND != FALSE) */

/*!*************************************************************************** */
/*                           DEFINITIONS                                       */
/* *************************************************************************** */

/*!*************************************************************************** */
/*                           GLOBAL & LOCAL VARIABLES                          */
/* *************************************************************************** */
#pragma space dp                                                                /* __TINY_SECTION__ */
#pragma space none                                                              /* __TINY_SECTION__ */

#pragma space nodp                                                              /* __NEAR_SECTION__ */
#if (_SUPPORT_CALIBRATION != FALSE)
static uint16_t l_u16CalibTravel;                                               /*!< Calibration Travel range */
#endif /* (_SUPPORT_CALIBRATION != FALSE) */
#if (_SUPPORT_CALIBRATION != FALSE) || (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
static uint16_t l_u16CalibPauseCounter = 0U;                                    /*!< Calibration end-stop pause counter */
#endif /* (_SUPPORT_CALIBRATION != FALSE) || (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */
#if (_SUPPORT_LIN_UV != FALSE)
static uint16_t l_u16LinUVTimeCounter = 0U;                                     /*!< LIN UV Time-counter */
#endif /* (_SUPPORT_LIN_UV != FALSE) */
#if ((_SUPPORT_TRIAXIS_MLX90363 != FALSE) || (_SUPPORT_TRIAXIS_MLX90427 != FALSE)) && (_SUPPORT_TRIAXIS_STANDBY == FALSE)
static uint16_t l_u16TriaxisPollCounter = 0U;                                   /*!< Triaxis Poll Counter */
#endif /* ((_SUPPORT_TRIAXIS_MLX90363 != FALSE) || (_SUPPORT_TRIAXIS_MLX90427 != FALSE)) && (_SUPPORT_TRIAXIS_STANDBY == FALSE) */
#if (_SUPPORT_NTC != FALSE)
volatile int16_t l_i16NtcTemperature = 0;                                       /* NTC Temperature */
#endif /* (_SUPPORT_NTC != FALSE) */
#if (_SUPPORT_AUTONOMOUS_DEMO != FALSE)
uint8_t l_u8AutonomousMode = TRUE;
#endif /* (_SUPPORT_AUTONOMOUS_DEMO != FALSE) */
#pragma space none                                                              /* __NEAR_SECTION__ */

/*!*************************************************************************** *
 * main_PeriodicTimerEvent
 * \brief   Perform main Periodic Timer Event updates.
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16Period: Time-period in core-timer units [500us]
 * \return  -
 * *************************************************************************** *
 * \details This function reset flags before each actuator operation
 * *************************************************************************** *
 * - Call Hierarchy: TIMER_IT(), Timer_SleepCompensation()
 * - Cyclomatic Complexity: 4+1
 * - Nesting: 2
 * - Function calling: 1 (AppPeriodicTimerEvent())
 * *************************************************************************** */
void main_PeriodicTimerEvent(uint16_t u16Period)
{
    AppPeriodicTimerEvent(u16Period);

#if (_SUPPORT_CALIBRATION != FALSE) || (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
    if (l_u16CalibPauseCounter != 0U)
    {
        if (l_u16CalibPauseCounter > u16Period)
        {
            l_u16CalibPauseCounter -= u16Period;
        }
        else
        {
            l_u16CalibPauseCounter = 0U;
#if (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
            if ( (g_e8MotorStatus & C_MOTOR_STATUS_STOP_MASK) != C_MOTOR_STATUS_STOP)
            {
                MotorDriverStop( (uint16_t)C_STOP_IMMEDIATE);
            }
#endif /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */
        }
    }
#endif /* (_SUPPORT_CALIBRATION != FALSE) || (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */

#if ((_SUPPORT_TRIAXIS_MLX90363 != FALSE) || (_SUPPORT_TRIAXIS_MLX90427 != FALSE)) && (_SUPPORT_TRIAXIS_STANDBY == FALSE)
    l_u16TriaxisPollCounter += u16Period;
#endif /* ((_SUPPORT_TRIAXIS_MLX90363 != FALSE) || (_SUPPORT_TRIAXIS_MLX90427 != FALSE)) && (_SUPPORT_TRIAXIS_STANDBY == FALSE) */

    /* (Re-)start delay */
    if (g_u16MotorStartDelay != 0U)
    {
        if (g_u16MotorStartDelay > u16Period)
        {
            g_u16MotorStartDelay -= u16Period;
        }
        else
        {
            g_u16MotorStartDelay = 0U;
        }
    }

} /* End of main_PeriodicTimerEvent() */

/*!*************************************************************************** *
 * main_noinit_section_init
 * \brief   Initialise the 'no-init' section global variables.
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details Define the initial value for all 'no-init' section global and local
 *          variables.
 * *************************************************************************** *
 * - Call Hierarchy: AppInit()
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 0
 * *************************************************************************** */
void main_noinit_section_init(void)
{
#if (_SUPPORT_CALIBRATION != FALSE)
    l_u16CalibTravel = NV_CALIB_TRAVEL;
    g_u16RealTravel = NV_REAL_TRAVEL;
    if (g_u16RealTravel == 0U)
    {
        g_u16RealTravel = l_u16CalibTravel;
    }
#endif /* (_SUPPORT_CALIBRATION != FALSE) */
} /* End of main_noinit_section_init() */

#if (_SUPPORT_TRIAXIS_MLX90363 != FALSE) || (_SUPPORT_TRIAXIS_MLX90427 != FALSE)
/*!*************************************************************************** *
 * TriaxisPostInit
 * \brief   After Triaxis has been initialised, the initial triaxis angle is retrieved
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details After Triaxis has been initialised, the initial triaxis angle is retrieved
 * *************************************************************************** *
 * - Call Hierarchy: main_Init()
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 0
 * *************************************************************************** */
static void TriaxisPostInit(void)
{
    uint16_t u16TriaxisError = 0U;                                              /* Last Triaxis error */
    uint16_t u16RetryCount = 3U;

    /* Power-on to first SCI message (Start-up time): 23.2ms (@ 3.3V) */
    do
    {
        DELAY(C_DELAY_3AXIS);  /*lint !e522 */
#if (_SUPPORT_TRIAXIS_MLX90363_XYZ == FALSE)
        if ( (u16TriaxisError = Triaxis_SendCmd(CMD_TRIAXIS_ALPHA)) == ERR_TRIAXIS_OK)
#else  /* (_SUPPORT_TRIAXIS_MLX90363_XYZ == FALSE) */
        if ( (u16TriaxisError = Triaxis_SendCmd(CMD_TRIAXIS_XYZ)) == ERR_TRIAXIS_OK)
#endif /* (_SUPPORT_TRIAXIS_MLX90363_XYZ == FALSE) */
        {
            break;
        }
        u16RetryCount--;
    } while (u16RetryCount != 0U);
    if (u16RetryCount == 0U)
    {
        g_u16ActualPosition = C_INV_POS;                                        /* Defective position-sensor */
        g_e8ErrorElectric = (uint8_t)C_ERR_SENSOR;                              /* Permanent Electric Failure */
#if (_SUPPORT_LOG_ERRORS != FALSE)
        SetLastError(C_ERR_TRIAXIS_FAILS | (C_ERR_EXT | ((u16TriaxisError & 0x0FU) << 8)));
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
    }

    /* ************************************** */
    /* *** P. Initialise External Sensors *** */
    /* ************************************** */
#define C_TRIAXIS_HYST  (uint16_t)(((2U * 65536UL) + (C_SHAFT_STEPS_PER_ROTATION / 2U)) / C_SHAFT_STEPS_PER_ROTATION)
    /* Get Triaxis absolute position */
    u16RetryCount = 3U;
    do
    {
        /* MLX90363 Frame-rate: Maximum 430Hz (all modes) to 862Hz (Mode 1) */
        DELAY(C_DELAY_3AXIS);  /*lint !e522 */
        if ( (u16TriaxisError = Triaxis_GetAbsPos(FALSE)) == ERR_TRIAXIS_OK)
        {
            uint16_t u16TriaxisPositionAvg = g_u16TriaxisActualPos;

            DELAY(C_DELAY_3AXIS);  /*lint !e522 */
            if (Triaxis_GetAbsPos(FALSE) == ERR_TRIAXIS_OK)
            {
                /* Average over two measurements */
                u16TriaxisPositionAvg = g_u16TriaxisActualPos +
                                        ((int16_t)(u16TriaxisPositionAvg - g_u16TriaxisActualPos) / 2);  /* MMP170301-1 */
            }
            if (u16TriaxisPositionAvg > (C_TRIAXIS_APP_END + C_TRIAXIS_HYST) )
            {
                /* Move actuator CCW from g_u16TriaxisActualPos to C_TRIAXIS_APP_END */
#if (_SUPPORT_LOG_ERRORS != FALSE)
                SetLastError(C_ERR_TRIAXIS_DEADZONE_CCW);
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
                Adapt_TriaxisAngleOffset(C_TRIAXIS_APP_END - u16TriaxisPositionAvg);
                g_u16TriaxisActualPos = C_TRIAXIS_APP_END;
                ConvTriaxisPos2ShaftSteps();
                g_u16TargetPosition = g_u16ActualPosition -
                                      p_MulU16hi_U16byU16( (u16TriaxisPositionAvg - C_TRIAXIS_APP_END),
                                                           C_SHAFT_STEPS_PER_ROTATION);
                /* TODO[MMP]: g_e8MotorRequest = (uint8_t) C_MOTOR_REQUEST_POR_START; */
                g_u8PorMovement = TRUE;
            }
            else if (u16TriaxisPositionAvg < (C_TRIAXIS_APP_BGN - C_TRIAXIS_HYST) )   /* MMP170203-1/MMP170209-2 */
            {
                /* Move actuator CW from g_u16TriaxisActualPos to C_TRIAXIS_APP_BGN */
#if (_SUPPORT_LOG_ERRORS != FALSE)
                SetLastError(C_ERR_TRIAXIS_DEADZONE_CW);
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
                Adapt_TriaxisAngleOffset(C_TRIAXIS_APP_BGN - u16TriaxisPositionAvg);
                g_u16TriaxisActualPos = C_TRIAXIS_APP_BGN;
                ConvTriaxisPos2ShaftSteps();
                g_u16TargetPosition = g_u16ActualPosition +
                                      p_MulU16hi_U16byU16( (C_TRIAXIS_APP_BGN - u16TriaxisPositionAvg),
                                                           C_SHAFT_STEPS_PER_ROTATION);
                /* TODO[MMP]: g_e8MotorRequest = (uint8_t) C_MOTOR_REQUEST_POR_START; */
                g_u8PorMovement = TRUE;
            }
            else
            {
                if (u16TriaxisPositionAvg > C_TRIAXIS_APP_END)
                {
                    g_u16TriaxisActualPos = C_TRIAXIS_APP_END;
                }
                else if (u16TriaxisPositionAvg < C_TRIAXIS_APP_BGN)
                {
                    g_u16TriaxisActualPos = C_TRIAXIS_APP_BGN;
                }
                else
                {
                    g_u16TriaxisActualPos = u16TriaxisPositionAvg;
                }
                ConvTriaxisPos2ShaftSteps();
                g_u8PorMovement = FALSE;
            }
            break;
        }
        u16RetryCount--;
    } while (u16RetryCount != 0U);
    if (u16RetryCount == 0U)
    {
        g_u16ActualPosition = C_INV_POS;                                        /* Defective position-sensor */
        g_e8ErrorElectric = (uint8_t)C_ERR_SENSOR;                              /* Permanent Electric Failure */
#if (_SUPPORT_LOG_ERRORS != FALSE)
        SetLastError(C_ERR_TRIAXIS_FAILS | (C_ERR_EXT | ((u16TriaxisError & 0x0FU) << 8)));
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
    }
#if (_SUPPORT_TRIAXIS_STANDBY != FALSE)
    if (g_u8PorMovement == FALSE)
    {
        (void)Triaxis_SendCmd(CMD_TRIAXIS_STANDBY);
    }
#endif /* (_SUPPORT_TRIAXIS_STANDBY != FALSE) */
} /* End of TriaxisPostInit() */
#endif /* (_SUPPORT_TRIAXIS_MLX90363 != FALSE) || (_SUPPORT_TRIAXIS_MLX90427 != FALSE) */

/*!*************************************************************************** *
 * main_Init()
 * \brief   Application code initialisation
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details The initialisation of the application is split in the following:
 * + Application Initialisation
 * *************************************************************************** *
 * - Call Hierarchy: main()
 * - Cyclomatic Complexity: 4+1
 * - Nesting: 2
 * - Function calling: 1 (AppInit())
 * *************************************************************************** */
static void main_Init(void)
{
    AppInit();

#if (_SUPPORT_TRIAXIS_MLX90363 != FALSE) || (_SUPPORT_TRIAXIS_MLX90427 != FALSE)
    TriaxisPostInit();
#endif /* (_SUPPORT_TRIAXIS_MLX90363 != FALSE) || (_SUPPORT_TRIAXIS_MLX90427 != FALSE) */

#if (_SUPPORT_HUMIDITY_HDC302x != FALSE)
    Humidity_Init();                                                            /* This Humidity Sensor makes use of I2C */
#endif /* (_SUPPORT_HUMIDITY_HDC302x != FALSE) */
} /* End of main_Init() */

#if (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT)
#if (LIN_COMM != FALSE) && (_SUPPORT_EMERGENCY_RUN != FALSE)
/*!*************************************************************************** *
 * HandleEmergencyRunMotorRequest()
 * \brief   Handle Emergency/Safety run actuator (e.g. LIN Bus-timeout)
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details
 * *************************************************************************** *
 * - Call Hierarchy: HandleMotorRequest()
 * - Cyclomatic Complexity: 5+1
 * - Nesting: 3
 * - Function calling: 0
 * *************************************************************************** */
static void HandleEmergencyRunMotorRequest(void)
{
#if (_SUPPORT_BUSTIMEOUT != FALSE)
    if (g_e8MotorRequest == (uint8_t)C_MOTOR_REQUEST_EMRUN)
    {
        /* Move actuator towards Safety/Emergency-Run position */
        g_e8MotorRequest = (uint8_t)C_MOTOR_REQUEST_NONE;

#if (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE)
        if (NV_SAFETY_POSITION == 0U)
        {
            g_u16TargetPosition = C_MIN_POS;
        }
        else
        {
            g_u16TargetPosition = C_MAX_POS;
        }

        if (g_u16ActualPosition != g_u16TargetPosition)
        {
            /* Only move actuator when not already at position */
            g_e8EmergencyRunOcc = (uint8_t)C_SAFETY_RUN_ACTIVE;
            g_e8StallDetectorEna = StallDetectorEna();                          /* Enable Stall Detector */
            g_u8StallOcc = FALSE;
            if (g_e8MotorStatus != C_MOTOR_STATUS_RUNNING)
            {
                /* If motor not running, set speed; Otherwise leave speed as is */
#if (_SUPPORT_ACT_SPEED_BY_LIN != FALSE)
                g_u16TargetMotorSpeedRPM = g_u16LowSpeedRPM;                    /* Speed #1 */
#else  /* (_SUPPORT_ACT_SPEED_BY_LIN != FALSE) */
                g_u8MotorCtrlSpeed = (uint8_t)C_MOTOR_SPEED_1;                  /* Speed #1 */
#endif /* (_SUPPORT_ACT_SPEED_BY_LIN != FALSE) */
            }
#if (_SUPPORT_DEGRADED_MODE != FALSE)
            if (g_e8DegradeStatus != FALSE)
            {
                g_e8DegradedMotorRequest = (uint8_t)C_MOTOR_REQUEST_START;
            }
#endif /* (_SUPPORT_DEGRADED_MODE != FALSE) */
            else
            {
                g_e8MotorRequest = (uint8_t)C_MOTOR_REQUEST_START;
            }
        }
#else  /* (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) */
        if (g_e8MotorStatus != C_MOTOR_STATUS_STOP)
        {
            g_e8MotorRequest = (uint8_t)C_MOTOR_REQUEST_STOP;
        }
#endif /* (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) */
#if (_SUPPORT_BUSTIMEOUT_SLEEP != FALSE) && (_SUPPORT_LIN_SLEEP != FALSE)       /* MMP180117-1: Add SLEEP support if already at Safety-position */
        else if (NV_BUSTIMEOUT_SLEEP != 0U)
        {
            g_e8MotorRequest = (uint8_t)C_MOTOR_REQUEST_SLEEP;
        }
#endif /* (_SUPPORT_BUSTIMEOUT_SLEEP != FALSE) && (_SUPPORT_LIN_SLEEP != FALSE) */
        else
        {
            /* Nothing */
        }
    }
#endif /* (_SUPPORT_BUSTIMEOUT != FALSE) */
} /* End of HandleEmergencyRunMotorRequest() */
#endif /* (LIN_COMM != FALSE) && (_SUPPORT_EMERGENCY_RUN != FALSE) */
#endif /* (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT) */

#if (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) && (_SUPPORT_SPEED_AUTO != FALSE)
/*!*************************************************************************** *
 * HandleAutoSpeedMode()
 * \brief   Handle Auto Speed-mode based on Supply and Temperature
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  uint8_t: Speed-mode 1...4
 * *************************************************************************** *
 * \details |
 *          |  1  :   2   :   4   :   2
 *  VOLT_2 -+.....:.......:.......:.....
 *          |     :       :       :
 *          |  1  :   2   :   2   :   2
 *  VOLT_1 -+.....:.......:.......:.....
 *          |  1  :   1       1   :   1
 *          +-----+-------+-------+-----
 *             TEMP_1  TEMP_2  TEMP_3
 * *************************************************************************** *
 * - Call Hierarchy: HandleStartMotorRequest()
 * - Cyclomatic Complexity: 4+1
 * - Nesting: 3
 * - Function calling: 0
 * *************************************************************************** */
uint8_t HandleAutoSpeedMode(void)
{
    /* Only allow initial speed "selection" */
    uint8_t u8MotorSpeedIdx;
    int16_t i16ChipTemperature = Get_ChipTemperature();
    uint16_t u16SupplyVoltage = Get_SupplyVoltage();
    int16_t i16TemperatureHyst = C_TEMPERATURE_HYS;
    uint16_t u16SupplyHyst = 25U;
    if ( (g_e8MotorStatus & C_MOTOR_STATUS_STOP_MASK) == C_MOTOR_STATUS_STOP)
    {
        i16TemperatureHyst = 0;
        u16SupplyHyst = 0U;
        g_u8MotorStatusSpeed = (uint8_t)C_MOTOR_SPEED_2;                /* Default: Speed-mode #2 */
    }
    u8MotorSpeedIdx = (uint8_t)g_u8MotorStatusSpeed;                    /* Last Speed-mode */
    if ( (i16ChipTemperature < (C_AUTOSPEED_TEMP_1 - i16TemperatureHyst)) ||
         (u16SupplyVoltage < (((C_AUTOSPEED_VOLT_1 * 25U) / 2U) - u16SupplyHyst)) )
    {
        u8MotorSpeedIdx = (uint8_t)C_MOTOR_SPEED_1;                     /* Speed-mode #1 */
    }
    else if ( ((u16SupplyVoltage >= (((C_AUTOSPEED_VOLT_1 * 25U) / 2U) + u16SupplyHyst)) &&
               (u16SupplyVoltage <= (((C_AUTOSPEED_VOLT_2 * 25U) / 2U) - u16SupplyHyst))) ||
              (((i16ChipTemperature >= (C_AUTOSPEED_TEMP_1 + i16TemperatureHyst)) &&
                (i16ChipTemperature <= (C_AUTOSPEED_TEMP_2 - i16TemperatureHyst))) ||
               (i16ChipTemperature > (C_AUTOSPEED_TEMP_3 + i16TemperatureHyst))) )
    {
        u8MotorSpeedIdx = (uint8_t)C_MOTOR_SPEED_2;                     /* Speed-mode #2 */
    }
    else if ( (i16ChipTemperature > (C_AUTOSPEED_TEMP_2 + i16TemperatureHyst)) &&
              (i16ChipTemperature < (C_AUTOSPEED_TEMP_3 - i16TemperatureHyst)) &&
         (u16SupplyVoltage > (((C_AUTOSPEED_VOLT_2 * 25U) / 2U) + u16SupplyHyst)) )
    {
#if defined (C_MOTOR_SPEED_4)
        u8MotorSpeedIdx = (uint8_t)C_MOTOR_SPEED_4;                     /* Speed-mode #4 */
#else  /* defined (C_MOTOR_SPEED_4) */
        u8MotorSpeedIdx = (uint8_t)C_MOTOR_SPEED_3;                     /* Speed-mode #3 */
#endif /* defined (C_MOTOR_SPEED_4) */
    }
    else
    {
        /* Nothing */
    }
    return (u8MotorSpeedIdx);
} /* End of HandleAutoSpeedMode() */
#endif /* (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) && (_SUPPORT_SPEED_AUTO != FALSE) */

/*!*************************************************************************** *
 * HandleStartMotorRequest()
 * \brief   Handle Start actuator
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  uint16_t: FALSE: Command not handled (continue)
 *                      TRUE : Command handled
 * *************************************************************************** *
 * \details In case motor is in stop-state, start-motor, otherwise change speed.
 * *************************************************************************** *
 * - Call Hierarchy: HandleMotorRequest()
 * - Cyclomatic Complexity: 8+1
 * - Nesting: 4
 * - Function calling: 3 (MotorDriverSpeed(), MotorDriverStart(), MotorDriverStop())
 * *************************************************************************** */
static uint16_t HandleStartMotorRequest(void)
{
    /* Start Actuator */
    uint16_t u16RequestHandled = TRUE;
#if (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT)
    uint16_t u16DeltaPosition;
    uint8_t u8NewMotorDirectionCCW;

#if (_SUPPORT_MOTOR_POSITION == C_MOTOR_POS_ROTOR)
    /* Based on Rotor (Full or Micro) steps */
    if (g_u16ActualPosition > g_u16TargetPosition)
    {
        u16DeltaPosition = g_u16ActualPosition - g_u16TargetPosition;
        u8NewMotorDirectionCCW = TRUE;
    }
    else
    {
        u16DeltaPosition = g_u16TargetPosition - g_u16ActualPosition;
        u8NewMotorDirectionCCW = FALSE;
    }
    if (u16DeltaPosition != 0U)
#elif (_SUPPORT_MOTOR_POSITION == C_MOTOR_POS_SHAFT)
    /* Based on (outer) shaft angle, using position sensor */
    if (g_u16ActualShaftAngle > g_u16TargetShaftAngle)
    {
        u16DeltaPosition = g_u16ActualShaftAngle - g_u16TargetShaftAngle;
        if (u16DeltaPosition > 0x7FFFU)
        {
            u8NewMotorDirectionCCW = FALSE;
        }
        else
        {
            u8NewMotorDirectionCCW = TRUE;
        }
    }
    else
    {
        u16DeltaPosition = g_u16TargetShaftAngle - g_u16ActualShaftAngle;
        if (u16DeltaPosition > 0x7FFFU)
        {
            u8NewMotorDirectionCCW = TRUE;
        }
        else
        {
            u8NewMotorDirectionCCW = FALSE;
        }
    }
    if (u16DeltaPosition > C_SHAFT_TOLERANCE_ANGLE)
#endif
    {
#if (_SUPPORT_ACT_SPEED_BY_LIN == FALSE)
#if (LINPROT == LIN22_SIMPLE_PCT)
        uint8_t u8MotorSpeedIdx = (uint8_t)(g_u8MotorCtrlSpeed & 0x03U);        /*Up to 4 speed-modes */
#elif (LINPROT == LIN2X_HVAC49) || (LINPROT == LIN2X_HVAC52) || (LINPROT == LIN2X_AIRVENT12) || (LINPROT == LIN13_HVACTB)
        uint8_t u8MotorSpeedIdx = (uint8_t)(g_u8MotorCtrlSpeed & 0x07U);        /*Up to 8 speed-modes */
#else /* (LINPROT) */
        uint8_t u8MotorSpeedIdx = C_DEFAULT_MOTOR_SPEED;
#endif /* (LINPROT) */
#if (_SUPPORT_SPEED_AUTO != FALSE)
#if (_SUPPORT_FOC_MODE == FOC_MODE_NONE)
        if (g_u8MotorCtrlSpeed == (uint8_t)C_MOTOR_SPEED_AUTO)
        {
            u8MotorSpeedIdx = HandleAutoSpeedMode();
        }
        g_u8MotorStatusSpeed = u8MotorSpeedIdx;
#endif /* (_SUPPORT_FOC_MODE == FOC_MODE_NONE) */
#endif /* (_SUPPORT_SPEED_AUTO != FALSE) */
#endif /* (_SUPPORT_ACT_SPEED_BY_LIN == FALSE) */

        if ( (g_e8MotorStatus & C_MOTOR_STATUS_STOP_MASK) == C_MOTOR_STATUS_STOP)
        {
            /* Motor is stopped; Start motor */
            g_e8MotorDirectionCCW = u8NewMotorDirectionCCW;
#if (_SUPPORT_ACT_SPEED_BY_LIN == FALSE)
            MotorDriverStart(u8MotorSpeedIdx);
#else  /* (_SUPPORT_ACT_SPEED_BY_LIN == FALSE) */
            MotorDriverStart(g_u16TargetMotorSpeedRPM);
#endif /* (_SUPPORT_ACT_SPEED_BY_LIN == FALSE) */
#if (_SUPPORT_TNCTOC != FALSE)
            g_e8TNCTOC |= C_TNCTOC_MOTOR;                                       /* Motor operation */
#endif /* (_SUPPORT_TNCTOC != FALSE) */
            g_e8MotorRequest = (uint8_t)C_MOTOR_REQUEST_NONE;
        }
        else if (u8NewMotorDirectionCCW != g_e8MotorDirectionCCW)
        {
            /* Changing direction; Stop motor first before starting in opposite direction */
            MotorDriverStop( (uint16_t)C_STOP_RAMPDOWN);                        /* Change of direction */
            u16RequestHandled = FALSE;
        }
        else
        {
            /* Motor is running; Change target-position */
            g_u32TargetPosition = ConvShaftSteps2MicroSteps(g_u16TargetPosition);
#if (_SUPPORT_ACT_SPEED_BY_LIN == FALSE)
            MotorDriverSpeed(u8MotorSpeedIdx);
#else  /* (_SUPPORT_ACT_SPEED_BY_LIN == FALSE) */
            MotorDriverSpeed(g_u16TargetMotorSpeedRPM);
#endif /* (_SUPPORT_ACT_SPEED_BY_LIN == FALSE) */
            g_e8MotorRequest = (uint8_t)C_MOTOR_REQUEST_NONE;
        }
    }
    else if (g_u8MotorHoldingCurrEna != Get_MotorHoldingCurrState() )
    {
        MotorDriverStop( (uint16_t)C_STOP_IMMEDIATE);
    }
    else
    {
        g_e8MotorRequest = (uint8_t)C_MOTOR_REQUEST_NONE;
    }
#elif (_SUPPORT_APP_TYPE == C_APP_SOLENOID)
    SolenoidDriverActivate();
    g_e8MotorRequest = (uint8_t)C_MOTOR_REQUEST_NONE;
#elif (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT)
    if (g_e8MotorStatus == C_MOTOR_STATUS_STOP)
    {
        g_u8StallOcc = FALSE;
        g_u8ChipResetOcc = FALSE;
        g_e8MotorDirectionCCW = g_u8NewMotorDirectionCCW;
        MotorDriverStart(g_u16TargetMotorSpeedRPM);
#if (_SUPPORT_TNCTOC != FALSE)
        g_e8TNCTOC |= C_TNCTOC_MOTOR;                                           /* Motor operation */
#endif /* (_SUPPORT_TNCTOC != FALSE) */
        g_e8MotorRequest = (uint8_t)C_MOTOR_REQUEST_NONE;
    }
    else if (g_u8NewMotorDirectionCCW != g_e8MotorDirectionCCW)
    {
        MotorDriverStop( (uint16_t)C_STOP_IMMEDIATE);
        u16RequestHandled = FALSE;
    }
    else
    {
        MotorDriverSpeed(g_u16TargetMotorSpeedRPM);
        g_e8MotorRequest = (uint8_t)C_MOTOR_REQUEST_NONE;
    }
#endif /* (_SUPPORT_APP_TYPE) */

    return (u16RequestHandled);
} /* End of HandleStartMotorRequest() */

#if (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT)
#if (_SUPPORT_CALIBRATION != FALSE)
/*!*************************************************************************** *
 * HandleCalibrationMotorRequest()
 * \brief   Handle Calibration Sequence
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details
 * *************************************************************************** *
 * - Call Hierarchy: HandleMotorRequest
 * - Cyclomatic Complexity: 12+1
 * - Nesting: 7
 * - Function calling: 4 (MotorDriverStop(), MotorDriverPosInit(),
 *                        AppResetFlags(), MotorDriverStart()
 * *************************************************************************** */
static void HandleCalibrationMotorRequest(void)
{
    /* Calibrate Levelling Actuator */
    /* The actuator calibration is performed in the following steps:
     * 1. Set current position as "MIN" (0)
     * 2. Set target position as "MAX" (LIN 1.3: 0x7FFE or LIN 2.x: 0xFFFE)
     * 3. Enable stall-detector and start actuator (C_CALIB_SETUP_HI_ENDPOS)
     * 4. Wait for a stall detected (C_CALIB_CHECK_HI_ENDPOS); If not detected (no end-stop): C_CALIB_FAILED_NO_ENDSTOP
     * 5. (Optional) Wait end-stop time (C_CALIB_PAUSE_HI_ENDSTOP)
     * 6. Set current position as "MAX" (LIN 1.3: 0x7FFE or LIN 2.x: 0xFFFE), set target position as "MIN" (0x0000),
     *    Clear stall-detection flag and start actuator (C_CALIB_SETUP_LO_ENDPOS)
     * 7. Wait for a stall detected (C_CALIB_CHECK_LO_ENDPOS); If not detected (no end-stop): C_CALIB_FAILED_NO_ENDSTOP
     * 8. If stall detected, check travel-range with NVRAM stored range. If within range +/- tolerance: C_CALIB_DONE
     *    If travel-range < NVRAM stored range - tolerance: C_CALIB_FAILED_TOO_SHORT
     *    If travel-range > NVRAM stored range + tolerance: C_CALIB_FAILED_TOO_LONG
     * 9. Set current position as "MIN" (0)
     */
    if (g_e8CalibrationStep == (uint8_t)C_CALIB_START)
    {
        /* Setup calibration (Low-position to High-position) */
        if (NV_ROTATION_DIRECTION != 0U)
        {
            g_u16ActualPosition = (l_u16CalibTravel + (2U * C_TRAVEL_OFFSET));
            g_u16TargetPosition = 0U;
        }
        else
        {
            g_u16ActualPosition = 0U;
            g_u16TargetPosition = (l_u16CalibTravel + (2U * C_TRAVEL_OFFSET));
        }
        g_u16RealTravel = g_u16ActualPosition;                                  /* g_u16RealTravel = Start-position */
        g_e8CalibrationStep = (uint8_t)C_CALIB_SETUP_HI_ENDPOS;

        if ( (g_e8MotorStatus & C_MOTOR_STATUS_STOP_MASK) != C_MOTOR_STATUS_STOP)
        {
            MotorDriverStop( (uint16_t)C_STOP_IMMEDIATE);
            l_u16CalibPauseCounter = NV_CALIB_ENDSTOP_TIME * C_PI_TICKS_10MS;   /* 10ms units */
        }
    }

    if ( (g_e8CalibrationStep == (uint8_t)C_CALIB_SETUP_HI_ENDPOS) && (l_u16CalibPauseCounter == 0U) )
    {
        g_e8MotorDirectionCCW =
            (g_u16TargetPosition < g_u16ActualPosition) ? (uint8_t)C_MOTOR_DIR_CLOSING : (uint8_t)C_MOTOR_DIR_OPENING;
        MotorDriverPosInit(g_u16ActualPosition);
        g_u8MotorCtrlSpeed = (uint8_t)C_DEFAULT_MOTOR_SPEED;
        g_e8StallDetectorEna |= (uint8_t)C_STALLDET_CALIB;
        AppResetFlags();
        MotorDriverStart(C_CALIB_MOTOR_SPEED);
        g_e8CalibrationStep = (uint8_t)C_CALIB_CHECK_HI_ENDPOS;                 /* Check for FIRST End-stop */
    }
    else if (g_e8CalibrationStep == (uint8_t)C_CALIB_CHECK_HI_ENDPOS)
    {
        if ( (g_e8MotorStatus & C_MOTOR_STATUS_STOP_MASK) == C_MOTOR_STATUS_STOP)
        {
            if (g_u8StallOcc != FALSE)
            {
                g_e8CalibrationStep = (uint8_t)C_CALIB_SETUP_LO_ENDPOS;
                l_u16CalibPauseCounter = (NV_CALIB_ENDSTOP_TIME * C_PI_TICKS_10MS);  /* 10ms units */
            }
            else /* if ( g_u16TargetPosition == g_u16ActualPosition ) */
            {
                /* Actuator has stopped reaching target position without stall detected, or
                 * actuator has stopped without reaching target position,
                 * nor without stall detected (possible error case) */
                g_e8CalibrationStep = (uint8_t)C_CALIB_FAILED_NO_ENDSTOP;
                g_e8MotorRequest = g_e8CalibPostMotorRequest;
                g_e8CalibPostMotorRequest = (uint8_t)C_MOTOR_REQUEST_NONE;
                g_e8StallDetectorEna &= (uint8_t) ~C_STALLDET_CALIB;
                g_u16RealTravel = l_u16CalibTravel;                             /* Restore travel range to last known (stored) travel-range */
            }
        }
    }
    else if ( (g_e8CalibrationStep == (uint8_t)C_CALIB_SETUP_LO_ENDPOS) && (l_u16CalibPauseCounter == 0) )
    {
        /* Setup calibration (High-position to Low-position) */
        g_u16ActualPosition = g_u16TargetPosition;                              /* g_u16ActualPosition = End-position */
        g_u16TargetPosition = g_u16RealTravel;                                  /* g_u16TargetPosition = Start-position */
        g_e8MotorDirectionCCW = (g_u16TargetPosition < g_u16ActualPosition) ? C_MOTOR_DIR_CLOSING : C_MOTOR_DIR_OPENING;
        g_u16RealTravel = g_u16ActualPosition;                                  /* g_u16RealTravel = End-position */
        MotorDriverPosInit(g_u16ActualPosition);
        g_u8MotorCtrlSpeed = (uint8_t)C_DEFAULT_MOTOR_SPEED;
        g_e8StallDetectorEna |= (uint8_t)C_STALLDET_CALIB;
        AppResetFlags();
        MotorDriverStart(C_CALIB_MOTOR_SPEED);
        g_e8CalibrationStep = (uint8_t)C_CALIB_CHECK_LO_ENDPOS;                 /* Check for SECOND End-stop */
    }
    else if (g_e8CalibrationStep == (uint8_t)C_CALIB_CHECK_LO_ENDPOS)
    {
        if ( (g_e8MotorStatus & C_MOTOR_STATUS_STOP_MASK) == C_MOTOR_STATUS_STOP)
        {
            if (g_u8StallOcc != FALSE)
            {
                if (g_u16RealTravel < g_u16ActualPosition)
                {
                    g_u16RealTravel = g_u16ActualPosition;
                }
                else
                {
                    g_u16RealTravel = g_u16RealTravel - g_u16ActualPosition;
                }
                g_e8CalibrationStep = C_CALIB_DONE;
                g_u16ActualPosition = ((NV_ROTATION_DIRECTION != FALSE) ? g_u16RealTravel : 0U);
                g_u16ActualPosition += C_TRAVEL_OFFSET;
                MotorDriverPosInit(g_u16ActualPosition);
                g_u8StallOcc = FALSE;
                g_u8StallTypeComm &= (uint8_t) ~M_STALL_MODE;
                g_e8StallDetectorEna &= (uint8_t) ~C_STALLDET_CALIB;
                g_u16MotorStartDelay = C_PI_TICKS_250MS;
                g_e8MotorRequest = g_e8CalibPostMotorRequest;
#if (_SUPPORT_DEGRADED_MODE != FALSE)
                g_e8DegradedMotorRequest = (uint8_t)C_MOTOR_REQUEST_NONE;
#endif /* (_SUPPORT_DEGRADED_MODE != FALSE) */
                g_e8CalibPostMotorRequest = (uint8_t)C_MOTOR_REQUEST_NONE;
            }
            else /* if ( g_u16TargetPosition == g_u16ActualPosition ) */
            {
                /* Actuator has stopped reaching target position without stall detected, or
                 * actuator has stopped without reaching target position,
                 * nor without stall detected (possible error case) */
                g_e8CalibrationStep = (uint8_t)C_CALIB_FAILED_NO_ENDSTOP;
                g_e8MotorRequest = g_e8CalibPostMotorRequest;
                g_e8CalibPostMotorRequest = (uint8_t)C_MOTOR_REQUEST_NONE;
                g_e8StallDetectorEna &= (uint8_t) ~C_STALLDET_CALIB;
                g_u16RealTravel = l_u16CalibTravel;                             /* Restore travel range to last known (stored) travel-range */
            }
        }
    }
    else
    {
        /* Nothing */
    }
} /* End of HandleCalibrationMotorRequest() */
#endif /* (_SUPPORT_CALIBRATION != FALSE) */

#if (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
/*!*************************************************************************** *
 * HandleCalibrationFactoryRequest()
 * \brief   Handle Calibration Sequence
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details
 * *************************************************************************** *
 * - Call Hierarchy: HandleMotorRequest
 * - Cyclomatic Complexity: 12+1
 * - Nesting: 7
 * - Function calling: 5 (MotorDriverStop(), MotorDriverPosInit(),
 *                        ResetFlags(), MotorDriverStart(), Triaxis_Calibrate()
 * *************************************************************************** */
static void HandleCalibrationFactoryRequest(void)
{
    /* Triaxis Calibration */
    /* The triaxis calibration is performed in the following steps:
     * 1. Determine the Triaxis X & Y minimum, maximum, sum and sum-count, over a number of rotations.
     * 2. If minimum != ADC_MIN and maximum != ADC_MAX calculate the X & Y offset (and amplitude)
     * 3. Determine the micro-step index 0 Triaxis angle (number of pole-pairs) in Clock Wise rotational direction and Counter Clock Wise direction
     */
    if (g_e8CalibrationStep == (uint8_t)C_CALIB_START)
    {
        if ( (g_e8MotorStatus & C_MOTOR_STATUS_STOP_MASK) != C_MOTOR_STATUS_STOP)
        {
            MotorDriverStop( (uint16_t)C_STOP_IMMEDIATE);
        }
        g_u8ChipResetOcc = FALSE;
        g_u8StallOcc = FALSE;
        g_e8StallDetectorEna = 0; /*TODO[MMP]:?? */

        /* CW X/Y Offset calibration */
        Triaxis_Calibrate(C_TRIAXIS_CALIB_RESET);
        g_e8CalibrationStep = (uint8_t)C_CALIB_SETUP_HI_ENDPOS;
#if (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE)
        g_u16ActualPosition = 0U;
        MotorDriverPosInit(g_u16ActualPosition);
        g_u16TargetPosition = (24U * Get_MotorMicroStepsPerMechRotation()) / C_MICROSTEP_PER_FULLSTEP;  /* 8 rotations */
        g_e8MotorDirectionCCW = FALSE;
        MotorDriverStart(0U);
#else  /* (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) */
        l_u16CalibPauseCounter = C_CALIB_PERIOD_LONG;
        g_e8MotorDirectionCCW = FALSE;
        MotorDriverStart(p_MulU16hi_U16byU16(g_u16LowSpeedRPM, C_CALIB_SPEED));
#endif /* (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) */
    }
    else if (g_e8CalibrationStep == (uint8_t)C_CALIB_SETUP_HI_ENDPOS)
    {
        /* Triaxis X/Y-Offset calibration (CW) */
        if ( (g_e8MotorStatus & C_MOTOR_STATUS_STOP_MASK) == C_MOTOR_STATUS_STOP)
        {
            /* Motor stopped */
#if (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE)
            if ( (g_u16TargetPosition == g_u16ActualPosition) && (g_u8StallOcc == FALSE) )
#else  /* (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) */
            if (g_u8StallOcc == FALSE)
#endif /* (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) */
            {
                /* Target-position without stall (CW); X/Y Offset calibration */
                if (Triaxis_Calibrate(C_TRIAXIS_CALIB_RESULT) != C_ERR_NONE)
                {
                    g_e8ErrorElectric = (uint8_t)C_ERR_SENSOR;
                    g_e8CalibrationStep = (uint8_t)C_CALIB_FAILED;
                    g_e8MotorRequest = (uint8_t)C_MOTOR_REQUEST_NONE;
                }
                else
                {
                    l_u16CalibPauseCounter = NV_CALIB_ENDSTOP_TIME * C_PI_TICKS_10MS;
                    g_e8CalibPostMotorRequest = (uint8_t)C_CALIB_CHECK_HI_ENDPOS;
                    g_e8CalibrationStep = (uint8_t)C_CALIB_PAUSE_HI_ENDSTOP;
                }
            }
            else
            {
                /* Stall or other reason of motor-stop */
#if (_SUPPORT_LOG_ERRORS != FALSE)
                SetLastError(C_ERR_TRIAXIS_CALIB | C_ERR_EXT | 0x100U);
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
                g_e8ErrorElectric = (uint8_t)C_ERR_SENSOR;
                g_e8CalibrationStep = (uint8_t)C_CALIB_FAILED;
                g_e8MotorRequest = (uint8_t)C_MOTOR_REQUEST_NONE;
            }
        }
    }
    else if ( (g_e8CalibrationStep == (uint8_t)C_CALIB_PAUSE_HI_ENDSTOP) && (l_u16CalibPauseCounter == 0U) )
    {
        if (g_e8CalibPostMotorRequest == (uint8_t)C_CALIB_CHECK_HI_ENDPOS)
        {
            /* CW Angle Offset calibration */
            g_e8CalibrationStep = (uint8_t)C_CALIB_CHECK_HI_ENDPOS;
#if (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE)
            g_u16ActualPosition = 0U;
            MotorDriverPosInit(g_u16ActualPosition);
            g_u16TargetPosition = (9U * Get_MotorMicroStepsPerMechRotation()) / C_MICROSTEP_PER_FULLSTEP;  /* 3 rotation */
            g_e8MotorDirectionCCW = FALSE;
            MotorDriverStart(0U);                                               /* Speed-mode: 0 */
#else  /* (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) */
            l_u16CalibPauseCounter = C_CALIB_PERIOD_SHORT;
            g_e8MotorDirectionCCW = FALSE;
            MotorDriverStart(p_MulU16hi_U16byU16(g_u16LowSpeedRPM, C_CALIB_SPEED));
#endif /* (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) */
        }
        else if (g_e8CalibPostMotorRequest == (uint8_t)C_CALIB_SETUP_LO_ENDPOS)
        {
            /* CCW X/Y Offset calibration */
            Triaxis_Calibrate(C_TRIAXIS_CALIB_INIT);
            g_e8CalibrationStep = (uint8_t)C_CALIB_SETUP_LO_ENDPOS;
#if (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE)
            g_u16TargetPosition = 0U;
            g_u16ActualPosition = (24U * Get_MotorMicroStepsPerMechRotation()) / C_MICROSTEP_PER_FULLSTEP;  /* 8 rotations */
            MotorDriverPosInit(g_u16ActualPosition);
            g_e8MotorDirectionCCW = TRUE;
            MotorDriverStart(0U);                                               /* Speed-mode: 0 */
#else  /* (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) */
            l_u16CalibPauseCounter = C_CALIB_PERIOD_LONG;
            g_e8MotorDirectionCCW = TRUE;
            MotorDriverStart(p_MulU16hi_U16byU16(g_u16LowSpeedRPM, C_CALIB_SPEED));
#endif /* (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) */
        }
        else if (g_e8CalibPostMotorRequest == (uint8_t)C_CALIB_CHECK_LO_ENDPOS)
        {
            /* CCW Angle Offset calibration */
            g_e8CalibrationStep = (uint8_t)C_CALIB_CHECK_LO_ENDPOS;
#if (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE)
            g_u16TargetPosition = 0U;
            g_u16ActualPosition = (9U * Get_MotorMicroStepsPerMechRotation()) / C_MICROSTEP_PER_FULLSTEP;  /* 3 rotation */
            MotorDriverPosInit(g_u16ActualPosition);
            g_e8MotorDirectionCCW = TRUE;
            MotorDriverStart(0U);                                               /* Speed-mode: 0 */
#else  /* (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) */
            l_u16CalibPauseCounter = C_CALIB_PERIOD_SHORT;
            g_e8MotorDirectionCCW = TRUE;
            MotorDriverStart(p_MulU16hi_U16byU16(g_u16LowSpeedRPM, C_CALIB_SPEED));
#endif /* (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) */
        }
        else
        {
            /* Nothing */
        }
        g_e8CalibPostMotorRequest = (uint8_t)C_MOTOR_REQUEST_NONE;
    }
    else if (g_e8CalibrationStep == (uint8_t)C_CALIB_CHECK_HI_ENDPOS)
    {
        /* Triaxis Angle Offset Calibration (CW) */
        if ( (g_e8MotorStatus & C_MOTOR_STATUS_STOP_MASK) == C_MOTOR_STATUS_STOP)
        {
            /* Motor stopped */
#if (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE)
            if ( (g_u16TargetPosition == g_u16ActualPosition) && (g_u8StallOcc == FALSE) )
#else  /* (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) */
            if (g_u8StallOcc == FALSE)
#endif /* (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) */
            {
                /* Target-position without stall (CW); Angle Offset calibration */
                if (Triaxis_Calibrate(C_TRIAXIS_CALIB_RESULT) != C_ERR_NONE)
                {
                    g_e8ErrorElectric = (uint8_t)C_ERR_SENSOR;
                    g_e8CalibrationStep = (uint8_t)C_CALIB_FAILED;
                    g_e8MotorRequest = (uint8_t)C_MOTOR_REQUEST_NONE;
                }
                else
                {
                    l_u16CalibPauseCounter = NV_CALIB_ENDSTOP_TIME * C_PI_TICKS_10MS;
                    g_e8CalibPostMotorRequest = (uint8_t)C_CALIB_SETUP_LO_ENDPOS;
                    g_e8CalibrationStep = (uint8_t)C_CALIB_PAUSE_HI_ENDSTOP;
                }
            }
            else
            {
                /* Stall or motor stopped */
#if (_SUPPORT_LOG_ERRORS != FALSE)
                SetLastError(C_ERR_TRIAXIS_CALIB | C_ERR_EXT | 0x200U);
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
                g_e8ErrorElectric = (uint8_t)C_ERR_SENSOR;
                g_e8CalibrationStep = (uint8_t)C_CALIB_FAILED;
                g_e8MotorRequest = (uint8_t)C_MOTOR_REQUEST_NONE;
            }
        }
    }
    else if (g_e8CalibrationStep == (uint8_t)C_CALIB_SETUP_LO_ENDPOS)
    {
        /* Triaxis X/Y-Offset calibration (CCW) */
        if ( (g_e8MotorStatus & C_MOTOR_STATUS_STOP_MASK) == C_MOTOR_STATUS_STOP)
        {
            /* Motor stopped */
#if (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE)
            if ( (g_u16TargetPosition == g_u16ActualPosition) && (g_u8StallOcc == FALSE) )
#else  /* (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) */
            if (g_u8StallOcc == FALSE)
#endif /* (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) */
            {
                /* Target-position without stall (CCW) */
                if (Triaxis_Calibrate(C_TRIAXIS_CALIB_RESULT) != C_ERR_NONE)
                {
                    g_e8ErrorElectric = (uint8_t)C_ERR_SENSOR;
                    g_e8CalibrationStep = (uint8_t)C_CALIB_FAILED;
                    g_e8MotorRequest = (uint8_t)C_MOTOR_REQUEST_NONE;
                }
                else
                {
                    l_u16CalibPauseCounter = NV_CALIB_ENDSTOP_TIME * C_PI_TICKS_10MS;
                    g_e8CalibPostMotorRequest = (uint8_t)C_CALIB_CHECK_LO_ENDPOS;
                    g_e8CalibrationStep = (uint8_t)C_CALIB_PAUSE_HI_ENDSTOP;
                }
            }
            else
            {
                /* Stall or motor stopped */
#if (_SUPPORT_LOG_ERRORS != FALSE)
                SetLastError(C_ERR_TRIAXIS_CALIB | C_ERR_EXT | 0x300U);
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
                g_e8ErrorElectric = (uint8_t)C_ERR_SENSOR;
                g_e8CalibrationStep = (uint8_t)C_CALIB_FAILED;
                g_e8MotorRequest = (uint8_t)C_MOTOR_REQUEST_NONE;
            }
        }
    }
    else if (g_e8CalibrationStep == (uint8_t)C_CALIB_CHECK_LO_ENDPOS)
    {
        /* Triaxis Angle Offset calibration (CCW) */
        if ( (g_e8MotorStatus & C_MOTOR_STATUS_STOP_MASK) == C_MOTOR_STATUS_STOP)
        {
            /* Motor stopped */
#if (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE)
            if ( (g_u16TargetPosition == g_u16ActualPosition) && (g_u8StallOcc == FALSE) )
#else  /* (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) */
            if (g_u8StallOcc == FALSE)
#endif /* (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) */
            {
                /* Target-position without stall (CCW) */
                if (Triaxis_Calibrate(C_TRIAXIS_CALIB_RESULT) != C_ERR_NONE)
                {
                    g_e8ErrorElectric = (uint8_t)C_ERR_SENSOR;
                }

                /* CW Angle Offset calibration */
                g_e8MotorRequest = (uint8_t)C_MOTOR_REQUEST_NONE;
                g_e8CalibrationStep = (uint8_t)C_CALIB_NONE;

                (void)Triaxis_Calibrate(C_TRIAXIS_CALIB_SAVE);
            }
            else
            {
                /* Stall or motor stopped */
#if (_SUPPORT_LOG_ERRORS != FALSE)
                SetLastError(C_ERR_TRIAXIS_CALIB | C_ERR_EXT | 0x400U);
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
                g_e8ErrorElectric = (uint8_t)C_ERR_SENSOR;
                g_e8CalibrationStep = (uint8_t)C_CALIB_FAILED;
                g_e8MotorRequest = (uint8_t)C_MOTOR_REQUEST_NONE;
            }
        }
    }
} /* End of HandleCalibrationFactoryRequest() */
#endif /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */
#endif /* (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT) */

#if (LIN_COMM != FALSE) && (_SUPPORT_LIN_SLEEP != FALSE)
/*!*************************************************************************** *
 * HandleSleepMotorRequest()
 * \brief   Handle Sleep Request
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details
 * *************************************************************************** *
 * - Call Hierarchy: HandleMotorRequest()
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 1 (AppSleep())
 * *************************************************************************** */
#if (_SUPPORT_APP_SAVE != FALSE) && (_SUPPORT_NV_EMERGENCY_STORE != FALSE)
void HandleSleepMotorRequest( void) __attribute__((noreturn));
void HandleSleepMotorRequest(void)
#else  /* (_SUPPORT_APP_SAVE != FALSE) && (_SUPPORT_NV_EMERGENCY_STORE != FALSE) */
static void HandleSleepMotorRequest( void) __attribute__((noreturn));
static void HandleSleepMotorRequest(void)
#endif /* (_SUPPORT_APP_SAVE != FALSE) && (_SUPPORT_NV_EMERGENCY_STORE != FALSE) */
{
    AppSleep();
    __builtin_unreachable();
} /* End of HandleSleepMotorRequest() */
#endif /* (LIN_COMM != FALSE) && (_SUPPORT_LIN_SLEEP != FALSE) */

#if (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT)
/*!*************************************************************************** *
 * HandleMotorRequest()
 * \brief   Handle Motor Request
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  uint16_t: FALSE: Command not handled (continue)
 *                      TRUE : Command handled
 * *************************************************************************** *
 * \details Handle actuator requests.
 *          (Optional) Prior to any request, check for Emergency/Safety Run request.
 *          Following requests are supported:
 *          * C_MOTOR_REQUEST_STOP: Stop the actuator from movement.
 *          * C_MOTOR_REQUEST_RESTART: Stop and restart actuator (Test-case for windmilling).
 *          * C_MOTOR_REQUEST_START: Start the actuator (after checking the  start-delay).
 *          * C_MOTOR_REQUEST_SLEEP: Enter (deep)sleep mode.
 * *************************************************************************** *
 * - Call Hierarchy: main()
 * - Cyclomatic Complexity: 6+1
 * - Nesting: 4
 * - Function calling: 5 (HandleEmergencyRunMotorRequest(),
 *                        MotorDriverPosInit(), HandleStartMotorRequest(),
 *                        HandleSleepMotorRequest(), HandleAutoSpeedMode())
 * *************************************************************************** */
static uint16_t HandleMotorRequest(void)
{
    uint16_t u16RequestHandled = TRUE;

#if (LIN_COMM != FALSE) && (_SUPPORT_EMERGENCY_RUN != FALSE)
    /* ************************************************* */
    /* *** i. Handling Motor Request (Emergency Run) *** */
    /* ************************************************* */
    HandleEmergencyRunMotorRequest();
#endif /* (LIN_COMM != FALSE) && (_SUPPORT_EMERGENCY_RUN != FALSE) */

    /* ********************************************************************************************* */
    /* *** j. Handling Motor Request (resp. STOP, INIT, START, CALIBRATION, SLEEP, SPEED-CHANGE) *** */
    /* ********************************************************************************************* */
#if (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) && (_SUPPORT_POS_INIT != FALSE)
    if (g_e8MotorRequest == (uint8_t)C_MOTOR_REQUEST_START_wINIT)
    {
        MotorDriverPosInit(g_u16ActualPosition);
        g_e8MotorRequest = (uint8_t)C_MOTOR_REQUEST_START;
    }
#endif /* (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) && (_SUPPORT_POS_INIT != FALSE) */
    if (g_e8MotorRequest == (uint8_t)C_MOTOR_REQUEST_NONE)
    {
        /* Nothing; No need to check the others */
    }
    else if (g_e8MotorRequest == (uint8_t)C_MOTOR_REQUEST_STOP)
    {
        /* Stop Actuator */
#if (_SUPPORT_FAST_STOP == FALSE)
        MotorDriverStop( (uint16_t)C_STOP_RAMPDOWN);                            /* LIN-command request */
#else  /* (_SUPPORT_FAST_STOP == FALSE) */
        MotorDriverStop( (uint16_t)C_STOP_FAST_STOP);                           /* Fast Stop */
#endif /* (_SUPPORT_FAST_STOP == FALSE) */
        g_e8MotorRequest = (uint8_t)C_MOTOR_REQUEST_NONE;
#if (_SUPPORT_DEGRADED_MODE != FALSE)
        g_e8DegradedMotorRequest = (uint8_t)C_MOTOR_REQUEST_NONE;
#endif /* (_SUPPORT_DEGRADED_MODE != FALSE) */
#if (_SUPPORT_CALIBRATION != FALSE)
        g_e8CalibPostMotorRequest = (uint8_t)C_MOTOR_REQUEST_NONE;
        if (g_e8CalibrationStep < (uint8_t)C_CALIB_DONE)
        {
            g_e8CalibrationStep = (uint8_t)C_CALIB_NONE;
            g_e8StallDetectorEna &= (uint8_t) ~C_STALLDET_CALIB;
            g_u16RealTravel = l_u16CalibTravel;                                 /* Restore travel range to last known (stored) travel-range */
        }
#endif /* (_SUPPORT_CALIBRATION != FALSE) */
    }
#if (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) && (_SUPPORT_POS_INIT != FALSE)
    else if (g_e8MotorRequest == (uint8_t)C_MOTOR_REQUEST_INIT)
    {
        /* Actuator initialisation: Set new actual position */
#if TRUE
        MotorDriverInit(FALSE);
#else
        MotorDriverPosInit(g_u16ActualPosition);
#endif
        g_e8MotorRequest = (uint8_t)C_MOTOR_REQUEST_NONE;
#if (_SUPPORT_DEGRADED_MODE != FALSE)
        g_e8DegradedMotorRequest = (uint8_t)C_MOTOR_REQUEST_NONE;
#endif /* (_SUPPORT_DEGRADED_MODE != FALSE) */
    }
#endif /* (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) && (_SUPPORT_POS_INIT != FALSE) */
    else if (g_e8MotorRequest == (uint8_t)C_MOTOR_REQUEST_START)
    {
        /* Motor Start request */
        if (g_u16MotorStartDelay == 0U)                                         /* Split Start-delay from Start-request to avoid Motor-request clearance (MMP230721-1) */
        {
            u16RequestHandled = HandleStartMotorRequest();
        }
    }
#if (_SUPPORT_CALIBRATION != FALSE)
    else if (g_e8MotorRequest == (uint8_t)C_MOTOR_REQUEST_CALIBRATION)
    {
        /* Calibration of the actuator by moving between end-stops */
        HandleCalibrationMotorRequest();
    }
#endif /* (_SUPPORT_CALIBRATION != FALSE) */
#if (_SUPPORT_SPEED_CHANGE != FALSE)
#if FALSE
    else if ( (g_e8MotorRequest == (uint8_t)C_MOTOR_REQUEST_SPEED_CHANGE) &&
              ((g_e8MotorStatus & C_MOTOR_STATUS_STOP_MASK) != C_MOTOR_STATUS_STOP) )
    {
#if (_SUPPORT_SPEED_AUTO != FALSE)
#if (LINPROT == LIN22_SIMPLE_PCT)
        uint8_t u8MotorSpeedIdx = (uint8_t)(g_u8MotorCtrlSpeed & 0x03U);
#elif (LINPROT == LIN2X_AGS) || (LINPROT == LIN2X_HVAC49) || (LINPROT == LIN2X_HVAC52) || (LINPROT == LIN2X_AIRVENT12)
        uint8_t u8MotorSpeedIdx = (uint8_t)(g_u8MotorCtrlSpeed & 0x07U);
#else /* (LINPROT) */
#error "ERROR: Speed index not handled"
#endif /* (LINPROT) */
        if (g_u8MotorCtrlSpeed == (uint8_t)C_MOTOR_SPEED_AUTO)
        {
            u8MotorSpeedIdx = HandleAutoSpeedMode();
        }
        g_u8MotorStatusSpeed = u8MotorSpeedIdx;
#else  /* (_SUPPORT_SPEED_AUTO != FALSE) */
        uint8_t u8MotorSpeedIdx = C_DEFAULT_MOTOR_SPEED;
#endif /* (_SUPPORT_SPEED_AUTO != FALSE) */
        MotorDriverSpeed(u8MotorSpeedIdx);
    }
#else
    else if (g_e8MotorRequest == (uint8_t)C_MOTOR_REQUEST_SPEED_CHANGE)
    {
        MotorDriverSpeed(g_u16TargetMotorSpeedRPM);
    }
#endif
#endif /* (_SUPPORT_SPEED_CHANGE != FALSE) */
#if (LIN_COMM != FALSE) && (_SUPPORT_LIN_SLEEP != FALSE)
    else if (g_e8MotorRequest == (uint8_t)C_MOTOR_REQUEST_SLEEP)
    {
        /* Enter deep-sleep mode */
        HandleSleepMotorRequest();
    }
#endif /* (LIN_COMM != FALSE) && (_SUPPORT_LIN_SLEEP != FALSE) */
    else if (g_e8MotorRequest == (uint8_t)C_MOTOR_REQUEST_RESET)
    {
        MotorDriverStop( (uint16_t)C_STOP_IMMEDIATE);                           /* PWM-command/time-out request */
        g_e8MotorRequest = (uint8_t)C_MOTOR_REQUEST_NONE;
#if (LIN_COMM != FALSE)
        ml_ResetDrv();                                                          /* Reset the Mlx4   */
        g_u8LinInFrameBufState = (uint8_t)C_LIN_IN_FREE;
#endif /* (LIN_COMM != FALSE) */
        MLX16_RESET_SIGNED( (BistResetInfo_t)C_CHIP_STATE_CMD_RESET);
    }
    else
    {
        /* Nothing (Unsupported request) */
        g_e8MotorRequest = (uint8_t)C_MOTOR_REQUEST_NONE;                       /* Clear unsupported motor-request (MMP230323-1) */
    }

    return (u16RequestHandled);

} /* End of HandleMotorRequest() */
#elif (_SUPPORT_APP_TYPE == C_APP_SOLENOID)
/*!*************************************************************************** *
 * HandleSolenoidRequest()
 * \brief   Handle Solenoid Request
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details Handle Solenoid application requests
 *          Following requests are supported:
 *          * C_SOLENOID_REQUEST_DEACTIVATE: De-activate the solenoid.
 *          * C_SOLENOID_REQUEST_ACTIVATE: Activate the solenoid
 *          * C_MOTOR_REQUEST_SLEEP: Enter (deep)sleep mode.
 * *************************************************************************** *
 * - Call Hierarchy: main()
 * - Cyclomatic Complexity: 6+1
 * - Nesting: 4
 * - Function calling: ? (,
 *                        HandleSleepMotorRequest())
 * *************************************************************************** */
static void HandleSolenoidRequest(void)
{
    if (g_e8MotorRequest == (uint8_t)C_MOTOR_REQUEST_NONE)
    {
        /* Nothing; No need to check the others */
    }
    else if (g_e8MotorRequest == (uint8_t)C_SOLENOID_REQUEST_DEACTIVATE)
    {
        SolenoidDriverDeactivate();
        g_e8MotorRequest = (uint8_t)C_MOTOR_REQUEST_NONE;
#if (_SUPPORT_DEGRADED_MODE != FALSE)
        g_e8DegradedMotorRequest = (uint8_t)C_MOTOR_REQUEST_NONE;
#endif /* (_SUPPORT_DEGRADED_MODE != FALSE) */
    }
    else if ( (g_e8MotorRequest == (uint8_t)C_SOLENOID_REQUEST_ACTIVATE) && (g_u16MotorStartDelay == 0U) )
    {
        if (g_e8SolenoidStatus == (uint8_t)C_SOLENOID_STATUS_DEACTIVATED)
        {
            SolenoidDriverActivate();
        }
        g_e8MotorRequest = (uint8_t)C_MOTOR_REQUEST_NONE;
    }
#if (_SUPPORT_LIN_SLEEP != FALSE)
    else if (g_e8MotorRequest == (uint8_t)C_MOTOR_REQUEST_SLEEP)
    {
        HandleSleepMotorRequest();
    }
#endif /* (_SUPPORT_LIN_SLEEP != FALSE) */
    else
    {
        /* Nothing */
    }
} /* End of HandleSolenoidRequest() */
#endif /* (_SUPPORT_APP_TYPE) */

/*!*************************************************************************** *
 * AppBackgroundTaskHandler()
 * \brief   Handle Back-ground tasks
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details Following background tasks are performed:
 *          - PID Threshold compensation
 *          - PID Control
 *          - LIN Co-processor check
 *          - Application Memory check
 *          - Application Temperature profile check
 *          - Application Processor Power-save mode
 * *************************************************************************** *
 * - Call Hierarchy: main()
 * - Cyclomatic Complexity: 0|2+1
 * - Nesting: 0|2
 * - Function calling: 3 (ThresholdControl(), PID_Control(), AppBackgroundHandler())
 * *************************************************************************** */
static void AppBackgroundTaskHandler(void)
{
#if (_SUPPORT_CURRENT_TEMP_COMPENSATION != FALSE)
    /* ************************************************************************************************ */
    /* *** l. Threshold control (DC-Motor: Motor PWM DC; Stepper: Current-threshold; BEMF: Nothing) *** */
    /* ************************************************************************************************ */
    ThresholdControl();
#endif /* (_SUPPORT_CURRENT_TEMP_COMPENSATION != FALSE) */

    /* ************************************************************************************************ */
    /* *** m. PID control (DC: Motor PWM Duty Cycle; Stepper: current-control; BEMF: speed-control) *** */
    /* ************************************************************************************************ */
#if (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT)
#if (_SUPPORT_FOC_MODE != FOC_MODE_NONE) && (_SUPPORT_PWM_MODE != SINGLE_COIL_PWM_BIPOLAR)
#if (_SUPPORT_STALLDET_LA != FALSE)
    if ( (PID_Control() == 0U) &&
         ((g_e8MotorStatus & C_MOTOR_STATUS_STOP_MASK) != C_MOTOR_STATUS_STOP) )  /* PID-control (Current/Speed/...) */
    {
        g_u8StallTypeComm |= (uint8_t)C_STALL_FOUND_PID_LA;
        if ( (g_e8StallDetectorEna & ((uint8_t)C_STALLDET_LA | (uint8_t)C_STALLDET_CALIB)) != 0U)
        {
            g_u8StallOcc = TRUE;                                                /* Report stall and ...  */
            MotorDriverStop( (uint16_t)C_STOP_EMERGENCY);                       /* ... stop motor (Stall) */
        }
    }
#else  /* (_SUPPORT_STALLDET_LA != FALSE) */
    (void)PID_Control();
#endif /* (_SUPPORT_STALLDET_LA != FALSE) */
#else  /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) && (_SUPPORT_PWM_MODE != SINGLE_COIL_PWM_BIPOLAR) */
    PID_Control();                                                              /* PID-control (Current/Speed/...) */
#endif /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) && (_SUPPORT_PWM_MODE != SINGLE_COIL_PWM_BIPOLAR) */
#endif /* (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT) */

    /* ********************************************* *
     * n. MLX4 status                                *
     * o. Background System check                    *
     * p. Chip temperature stability (profile) check *
     * q. Application check (MMP190920-1)            *
     * r. Power-saving (non-running)                 *
     * ********************************************* */
    AppBackgroundHandler();
} /* End of AppBackgroundTaskHandler() */

/*!*************************************************************************** *
 * main()
 * \brief   Application code (main-loop)
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details Main application loop
 *          After the application initialisation (main_init()), the main remains
 *          in an endless loop. The Endless loop consists of the following items:
 *          * Watchdog acknowledgement
 *          * (Optional) Autonomous demonstration mode support
 *          * Incoming message handling (communication)
 *          * Application Degraded mode check (Under-, Over-voltage and Over-temperature check)
 *          * Handling of actuator request
 *          * Actuator Status Update (Position & Speed)
 *          * Background Tasks handler
 * *************************************************************************** *
 * - Call Hierarchy: fw_start(), fatal()
 * - Cyclomatic Complexity: 10+1
 * - Nesting: 4
 * - Function calling: 8 (main_Init(), p_AwdAck(),
 *                        HandleLinInMsg(), AppDegradedCheck(),
 *                        HandleMotorRequest(), AppBackgroundTaskHandler(),
 *                        MotorDriverStop())
 * *************************************************************************** */
int main(void)
{
#if (_SUPPORT_MOTOR_POSITION == FALSE) || (_SUPPORT_FOC_MODE != FOC_MODE_NONE)
    uint32_t u32ActualMotorSpeedLPFx64 = 0U;                                    /* LPF of Actual Motor-speed x 8 (max = 8191 RPM) */
#endif /* (_SUPPORT_MOTOR_POSITION == FALSE) || (_SUPPORT_FOC_MODE != FOC_MODE_NONE) */

#if (_SUPPORT_PWM_SYNC != FALSE)
#if (defined (__MLX81332B02__) || defined (__MLX81334__) || defined (__MLX81344B01__) || defined (__MLX81346B01__))
    IO_TRIM_VDD = IO_TRIM_VDD | ((1U << 2) << 6);                               /* Set TR_SUP[2] to '1' (Sync PWM modules) */
#elif defined (__MLX81339__) || defined (__MLX81350__)
    IO_TRIM_MISC |= B_TRIM_MISC_PWM_SYNC_MODE;
#endif
#endif /* (_SUPPORT_PWM_SYNC != FALSE) */

    main_Init();

#if (_DEBUG_NV_WRITE_BACKGROUND != FALSE)
    static uint8_t NV_Test[8];
    NV_Test[0] = (uint8_t)0x00U;
    NV_Test[1] = (uint8_t)0x11U;
    NV_Test[2] = (uint8_t)0x22U;
    NV_Test[3] = (uint8_t)0x33U;
    NV_Test[4] = (uint8_t)0x44U;
    NV_Test[5] = (uint8_t)0x55U;
    NV_Test[6] = (uint8_t)0x66U;
    NV_Test[7] = (uint8_t)0x77U;
#if (_DEBUG_COMMUT_ISR != FALSE)
    DEBUG_SET_IO_A();
#endif /* (_DEBUG_COMMUT_ISR != FALSE) */
    ENTER_SECTION(SYSTEM_MODE); /*lint !e534 */
    IO_MLX16_ITC_PEND2_S = B_MLX16_ITC_PEND2_EE_COMPLETE;
    EEPROM_WriteWord64_non_blocking( (const uint16_t)0x980U, (const uint16_t *)((void *)NV_Test), 07U);
    EXIT_SECTION(); /*lint !e438 */
    while ( (IO_MLX16_ITC_PEND2_S & B_MLX16_ITC_PEND2_EE_COMPLETE) == 0U)
    {
#if (_DEBUG_COMMUT_ISR != FALSE)
        DEBUG_TOG_IO_A();
#endif /* (_DEBUG_COMMUT_ISR != FALSE) */
    }
#if (_DEBUG_COMMUT_ISR != FALSE)
    DEBUG_CLR_IO_A();
#endif /* (_DEBUG_COMMUT_ISR != FALSE) */
    g_u16TestCnt = 1U;
#endif /* (_DEBUG_NV_WRITE_BACKGROUND != FALSE) */

#if (_SUPPORT_CHIP_TEST != FALSE) && (_APP_DMA_STRESS_TEST != FALSE)
    ChipTestDMA_Init();
#endif /* (_SUPPORT_CHIP_TEST != FALSE) && (_APP_DMA_STRESS_TEST != FALSE) */

    for(;;)
    {
        /* *********************************** */
        /* *** a. Watchdog acknowledgement *** */
        /* *********************************** */
#if (_DEBUG_WDACK != FALSE)
        DEBUG_SET_IO_A();
#endif /* (_DEBUG_WDACK != FALSE) */
#if FALSE
        /* Include Atomic-section around AWD-Acknowledge in case Watchdog-acknowledge is done at other code places (at higher CPU-priority) as well */
        ENTER_SECTION(ATOMIC_KEEP_MODE); /*lint !e534 */
        p_AwdAck(); /*lint !e522 */                                             /* Acknowledge Analogue Watchdog .. (MMP200625-2) */
#if (_SUPPORT_DWD != FALSE)
        WDG_conditionalIwdRefresh(C_IWD_DIV, C_IWD_TO);                         /* .. acknowledge the digital watchdog */
#endif /* (_SUPPORT_DWD != FALSE) */
        EXIT_SECTION(); /*lint !e438 */
#else
        p_AwdAck(); /*lint !e522 */                                             /* Acknowledge Analogue Watchdog .. (MMP200625-2) */
#if (_SUPPORT_DWD != FALSE)
        WDG_conditionalIwdRefresh(C_IWD_DIV, C_IWD_TO);                         /* .. acknowledge the digital watchdog */
#endif /* (_SUPPORT_DWD != FALSE) */
#endif
#if (_DEBUG_WDACK != FALSE)
        DEBUG_CLR_IO_A();
#endif /* (_DEBUG_WDACK != FALSE) */

#if (_SUPPORT_AUTONOMOUS_DEMO != FALSE)
#define C_DEMO_POS_1                        0U                                  /*!< Demo position #1 */
#define C_DEMO_POS_2                        12500U                              /*!< Demo position #2 */
#define C_DEMO_SPEEDMODE                    1U                                  /*!< Demo speed-mode: Slowest */

        if ( (l_u8AutonomousMode != FALSE) &&                                   /* Still in autonomous mode */
             (g_e8MotorRequest == (uint8_t)C_MOTOR_REQUEST_NONE) &&             /* No pending motor requests */
             (g_u16MotorStartDelay == 0U) &&                                    /* Allow GUI to show STALL */
             ((g_e8MotorStatus & (uint8_t)C_MOTOR_STATUS_STOP_MASK) == (uint8_t)C_MOTOR_STATUS_STOP) )  /* Motor has stopped */
        {
            if ( (g_u16ActualPosition == C_INI_POS) && (g_u16TargetPosition == C_INV_POS) )
            {
                /* Power-on status */
                g_u8ChipResetOcc = FALSE;                                       /* Clear Reset flag */
                g_u16ActualPosition = C_DEMO_POS_1;
                g_u16TargetPosition = C_DEMO_POS_2;                             /* Move CW */
            }
            else if (g_u16TargetPosition < (C_DEMO_POS_2/2U) )
            {
                g_u16ActualPosition = C_DEMO_POS_1;
                g_u16TargetPosition = C_DEMO_POS_2;                             /* Move CW */
            }
            else
            {
                g_u16ActualPosition = C_DEMO_POS_2;
                g_u16TargetPosition = C_DEMO_POS_1;                             /* Move CCW */
            }
            g_u8StallOcc = FALSE;                                               /* Clear Stall Flag */
            g_e8EmergencyRunOcc = (uint8_t)C_SAFETY_RUN_NO;                     /* Clear Emergency Run flag */
            g_e8MotorCtrlMode = (uint8_t)C_MOTOR_CTRL_NORMAL;                   /* Set Motor Control Mode to Normal */
            g_e8StallDetectorEna = C_STALLDET_ALL;                              /* Enable all stall-detectors */
            g_u8MotorCtrlSpeed = C_DEMO_SPEEDMODE;                              /* Set speed-mode */
            g_e8MotorRequest = (uint8_t)C_MOTOR_REQUEST_START_wINIT;            /* Set initial position (actual position) and start actuator to move to target position */
        }
#endif /* (_SUPPORT_AUTONOMOUS_DEMO != FALSE) */

#if (LIN_COMM != FALSE)
        /* ********************************* */
        /* *** c. LIN(-IN) communication *** */
        /* ********************************* */
        if (g_u8LinInFrameBufState != (uint8_t)C_LIN_IN_FREE)
        {
            /* LIN message buffer filled */
#if (_SUPPORT_AUTONOMOUS_DEMO != FALSE)
            l_u8AutonomousMode = FALSE;
#endif /* (_SUPPORT_AUTONOMOUS_DEMO != FALSE) */
            HandleLinInMsg();
        }
#endif /* (LIN_COMM != FALSE) */

#if (_SUPPORT_CHIP_TEST != FALSE) && (_APP_DMA_STRESS_TEST != FALSE)
        ChipTestDMA();
#endif /* (_SUPPORT_CHIP_TEST != FALSE) && (_APP_DMA_STRESS_TEST != FALSE) */

        /* ************************************************************** */
        /* *** d. Motor Driver current                                *** */
        /* *** e. Chip and Motor Driver voltage (degraded-mode check) *** */
        /* *** f. Chip and ambient temperature (degraded-mode check)  *** */
        /* *** g. Degraded-mode check                                 *** */
        /* ************************************************************** */
        AppDegradedCheck();

        /* ********************************************************** */
        /* *** i. Handling Motor Request (Emergency Run)          *** */
        /* *** j. Handling Motor Request (resp. STOP, START, ...) *** */
        /* ********************************************************** */
#if (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT)
        if (HandleMotorRequest() == FALSE)
        {
            /* Command not handled */
            continue;
        }

        /* ************************ */
        /* *** k. Status update *** */
        /* ************************ */
        /* Actual Position update */
#if (_SUPPORT_TRIAXIS_MLX90363 != FALSE) || (_SUPPORT_TRIAXIS_MLX90427 != FALSE)
        /* SPI Triaxis */
#if (_SUPPORT_TRIAXIS_STANDBY == FALSE)
        /* Actual Position based on Triaxis */
        if (l_u16TriaxisPollCounter >= C_TRIAXIS_POLL_RATE)
        {
            l_u16TriaxisPollCounter = 0U;
            if (g_u8LinTriaxisCmd != CMD_TRIAXIS_STANDBY)
            {
                if (Triaxis_GetAbsPos(TRUE) == ERR_TRIAXIS_OK)
                {
                    /* Convert Triaxis position to "outer-shaft"-position */
                    ConvTriaxisPos2ShaftSteps();
                }
            }
        }
        else
        {
            /* Nothing */
        }
#else  /* (_SUPPORT_TRIAXIS_STANDBY == FALSE) */
        if (g_u8PorMovement != FALSE)
        {
#define C_TRIAXIS_POR_POLL_COUNT    50U
            if (l_u16TriaxisPollCounter > C_TRIAXIS_POR_POLL_COUNT)
            {
                ATOMIC_CODE( (void)Triaxis_GetAbsPos(TRUE); );
                l_u16TriaxisPollCounter = 0U;
            }
        }
#endif /* (_SUPPORT_TRIAXIS_STANDBY == FALSE) */
#elif (_SUPPORT_TRIAXIS_MLX90367 != FALSE) || (_SUPPORT_TRIAXIS_MLX90372 != FALSE)
        if (Triaxis_Data() == ERR_TRIAXIS_OK)
        {
            /* Convert Triaxis position to "outer-shaft"-position */
            /* ConvTriaxisPos2ShaftSteps(); TODO[MMP]: Insert */
        }
#elif (_SUPPORT_TRIAXIS_MLX90421 != FALSE)
        /* TODO: For test-purpose */
        g_u16ActualPosition = g_u16TriaxisAbsPos;
#elif (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT)
        ConvMicroSteps2ShaftSteps();
#endif /* (_SUPPORT_APP_TYPE) */

#if (_SUPPORT_TRIAXIS_MLX90422 != FALSE) || (_SUPPORT_TRIAXIS_MLX90426 != FALSE)
        (void)Triaxis_Data();
#endif /* (_SUPPORT_TRIAXIS_MLX90422 != FALSE) || (_SUPPORT_TRIAXIS_MLX90426 != FALSE) */

        /* Actual Speed update */
#if (_SUPPORT_DUAL_HALL_LATCH_MLX92251 != FALSE) || (_SUPPORT_DUAL_HALL_LATCH_MLX92255 != FALSE)
        if (DualHallLatch_Direction() != C_DIR_UNKNOWN)
        {
            /* When direction is known, speed is also known */
            uint16_t u16Speed = DualHallLatch_Speed();
            if ( (u16Speed != 0x0000U) && (u16Speed != 0xFFFFU) )
            {
                /* Valid speed */
                Set_ActualMotorSpeedRPM(u16Speed);
            }
        }
#endif /* (_SUPPORT_DUAL_HALL_LATCH_MLX92251 != FALSE) || (_SUPPORT_DUAL_HALL_LATCH_MLX92255 != FALSE) */

#if (_SUPPORT_FOC_MODE != FOC_MODE_NONE)                                        /* FOC for Positioning Device */
        {
            uint16_t u16CopyActualCommutTimerPeriod = g_u16ActualCommutTimerPeriod; /* MMP170805-1: Fix for Commutation-ISR calling MotorDriverStop() */
            if ( ((g_e8MotorStatus & C_MOTOR_STATUS_STOP_MASK) != C_MOTOR_STATUS_STOP) &&
                 (u16CopyActualCommutTimerPeriod != 0U) )
            {
                /* Convert RPS-electric to RPM-mechanical */
                /* IIR-1 LPF: Y[t] = 15/16*Y[t-1] + 1/16*X[t] (up to 4095 RPM) */
                /* IIR-1 LPF: Y[t] = 7/8*Y[t-1] + 1/8*X[t] (up to 8191 RPM) */
                /* IIR-1 LPF: Y[t] = 3/4*Y[t-1] + 1/4*X[t] (up to 16383 RPM) */
                if ( (l_e8MotorStartupMode & E_MSM_MODE_MASK) == E_MSM_STEPPER)
                {
                    uint16_t u16ActualMotorSpeed = p_DivU16_U32byU16(Get_MicroStepPeriodOneRPM(),
                                                                     u16CopyActualCommutTimerPeriod);
                    g_u16ActualMotorSpeedRPM = u16ActualMotorSpeed;
                    u32ActualMotorSpeedLPFx64 = ((uint32_t)u16ActualMotorSpeed << 16U);
                }
                else
                {
                    uint16_t u16LPF_Coef;
                    uint16_t u16ActualMotorSpeed =
                        p_DivU16_U32byU16( (uint32_t)g_u16ActualMotorSpeedRPMe, g_u16MotorPolePairs);
                    if (g_u16ActualMotorSpeedRPM > g_u16MinSpeedRPM)
                    {
                        u16LPF_Coef = (g_u16ActualMotorSpeedRPM >> 1);
                    }
                    else
                    {
                        u16LPF_Coef = 1024U;
                    }
                    g_u16ActualMotorSpeedRPM =
                        p_LpfU16_I16byI16(&u32ActualMotorSpeedLPFx64, u16LPF_Coef,
                                          (int16_t)(u16ActualMotorSpeed - g_u16ActualMotorSpeedRPM));  /* MMP171026-1 */
                }
            }
            else
            {
#if (_SUPPORT_MOTION_DET == C_MOTION_DET_NONE)
                g_u16ActualMotorSpeedRPM = 0U;
#endif /* (_SUPPORT_MOTION_DET == C_MOTION_DET_NONE) */
                u32ActualMotorSpeedLPFx64 = 0U;
            }
        }
#elif (_SUPPORT_STALLDET_LA != FALSE)
        {
            extern uint16_t l_u16CommutTimerPeriod;
            uint16_t u16CopyActualCommutTimerPeriod = l_u16CommutTimerPeriod; /* MMP170805-1: Fix for Commutation-ISR calling MotorDriverStop() */
            if ( ((g_e8MotorStatus & (uint8_t)C_MOTOR_STATUS_STOP_MASK) != (uint8_t)C_MOTOR_STATUS_STOP) &&
                 (u16CopyActualCommutTimerPeriod != 0U) )
            {
                g_u16ActualMotorSpeedRPM = p_DivU16_U32byU16(Get_MicroStepPeriodOneRPM(), u16CopyActualCommutTimerPeriod);
            }
            else
            {
                g_u16ActualMotorSpeedRPM = 0U;
            }
        }
#else  /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) */
        if (l_u8MotorStartupMode != (uint8_t)MSM_STOP)
        {
            g_u16ActualMotorSpeedRPM = g_u16ForcedSpeedRPM;
        }
        else
        {
            g_u16ActualMotorSpeedRPM = 0U;
        }
#endif /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) */
#elif (_SUPPORT_APP_TYPE == C_APP_SOLENOID)
        HandleSolenoidRequest();
#endif /* (_SUPPORT_APP_TYPE) */

#if (_SUPPORT_INDUCTIVE_POS_SENSOR_MLX90513 != FALSE)
        InductivePosSensor_Data();
#endif /* (_SUPPORT_INDUCTIVE_POS_SENSOR_MLX90513 != FALSE) */

#if (_SUPPORT_PRESSURE_MLX90829 != FALSE)
        Pressure_Data();
#endif /* (_SUPPORT_PRESSURE_MLX90829 != FALSE) */

#if (_SUPPORT_TRIAXIS_MLX90377 != FALSE) && (_SUPPORT_SENSOR_SENT_IO != PIN_FUNC_IO_NONE)
        Triaxis_Trigger(u16TriaxisSPC_ID);
#if FALSE  /* FALSE: Single device; TRUE: Multiple device (SPC-Trigger scan) */
        u16TriaxisSPC_ID = ((u16TriaxisSPC_ID + 1U) & 0x03U);
#endif
        DELAY_US(C_TRIAXIS_SPC_FRAME_TIME);                                     /* Wait for SPC Sensor to answer */
        (void)Triaxis_Data();
#endif /* (_SUPPORT_TRIAXIS_MLX90377 != FALSE) && (_SUPPORT_SENSOR_SENT_IO != PIN_FUNC_IO_NONE) */

#if (_SUPPORT_NTC != FALSE)
        l_i16NtcTemperature = NTC_Temperature();
#endif /* (_SUPPORT_NTC != FALSE) */

        /* ***************************************************** */
        /* *** l. Threshold control (Current vs. Temperature)*** */
        /* *** m. PID control (current/speed control)        *** */
        /* *** n. MLX4 status                                *** */
        /* *** o. Background System check                    *** */
        /* *** p. Motor-phase shortage to ground check       *** */
        /* *** q. Chip temperature stability (profile) check *** */
        /* *** r. Power-saving (non-running)                 *** */
        /* ***************************************************** */
#if (_SUPPORT_HUMIDITY_HDC302x != FALSE)
        HumiditySensorHandler();
#endif /* (_SUPPORT_HUMIDITY_HDC302x != FALSE) */
        AppBackgroundTaskHandler();
    } /* main-loop */

    return 0; /*lint !e527 */
} /* End of main() */

/* EOF */
