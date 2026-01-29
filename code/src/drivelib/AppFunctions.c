/*!*************************************************************************** *
 * \file        AppFunctions.c
 * \brief       MLX813xx application Functions
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
 *           -# AppInit()
 *           -# AppResetFlags()
 *           -# SelfHeatCompensation()
 *           -# AppPeriodicTimerEvent()
 *           -# AppDegradedCheck()
 *           -# AppBackgroundHandler()
 *           -# AppStop()
 *           -# AppSleepWithWakeUpTimer()
 *           -# AppSleep()
 *           -# AppReset()
 *  - Internal Functions:
 *           -# AppCurrentCheck()
 *           -# AppInternalSupplyCheck()
 *           -# AppSupplyCheck()
 *           -# AppTemperatureCheck()
 *           -# AppCheckLinProc()
 *           -# FlashBist()
 *           -# ColinRomBist()
 *           -# SysRomBist()
 *           -# RamTransparentBist()
 *           -# AppMemoryCheck()
 *           -# AppTemperatureProfileCheck()
 *           -# AppProcPowerSave()
 *           -# PendIrqCheck()
 *           -# AppChipCheck()
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

/*!*************************************************************************** */
/*                           INCLUDES                                          */
/* *************************************************************************** */
#include "AppBuild.h"                                                           /* Application Build */

#include "main.h"                                                               /* main support */

#if (_SUPPORT_VDDA_VDDD_LPF != FALSE)
#include "drivelib/ADC.h"                                                       /* ADC support */
#endif /* (_SUPPORT_VDDA_VDDD_LPF != FALSE) */
#include "drivelib/AppFunctions.h"                                              /* Application Functions support */
#include "drivelib/Diagnostic.h"                                                /* Chip Protection & Diagnostics support */
#include "drivelib/ErrorCodes.h"                                                /* Error logging support */
#include "drivelib/GlobalVars.h"                                                /* Global Variables support */
#if (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT)
#include "drivelib/MotorDriver.h"                                               /* Motor Driver support */
#elif (_SUPPORT_APP_TYPE == C_APP_RELAY)
#include "drivelib/RelayDriver.h"                                               /* Relay Driver support */
#elif (_SUPPORT_APP_TYPE == C_APP_SOLENOID)
#include "drivelib/SolenoidDriver.h"                                            /* Solenoid Driver support */
#endif /* (_SUPPORT_APP_TYPE) */
#include "drivelib/MotorStall.h"                                                /* Motor Stall Detectors */
#include "drivelib/NV_Functions.h"                                              /* Non Volatile Memory Functions & Layout */
#include "drivelib/PID_Control.h"                                               /* PID support */
#include "camculib/private_mathlib.h"                                           /* Private Math Library support */
#include "drivelib/Timer.h"                                                     /* Simple Timer support */

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
#if (_SUPPORT_HALL_LATCH_MLX9227x != FALSE)
#include "senselib/HallLatch_MLX9227x.h"                                        /* Hall Latch MLX9227x support */
#endif /* (_SUPPORT_HALL_LATCH_MLX9227x != FALSE) */
#if (_SUPPORT_HALL_LATCH != FALSE)
#include "senselib/HallLatch.h"                                                 /* Hall-Latch support */
#endif /* (_SUPPORT_HALL_LATCH != FALSE) */
#if (_SUPPORT_NTC != FALSE)
#include "senselib/NTC.h"                                                       /* NTC Support */
#endif /* (_SUPPORT_NTC != FALSE) */

#if (CAN_COMM != FALSE)
#include "commlib/CAN_Communication.h"                                          /* CAN Communication support */
#endif /* (CAN_COMM != FALSE) */
#if (GPIO_COMM != FALSE)
#include "commlib/GPIO_OnOff.h"                                                 /* GPIO support */
#endif /* (GPIO_COMM != FALSE) */
#if (defined (__MLX81332_w90381__) || defined (__MLX81340_w90381__) || (I2C_COMM != FALSE)) && (_SUPPORT_I2C != FALSE) && ((I2C_MASTER_PROT != I2C_MASTER_PROT_NONE) || (I2C_SLAVE_PROT != I2C_SLAVE_PROT_NONE))
#include "commlib/I2C_Generic.h"                                                /* I2C Generic support */
#endif /* (I2C_COMM != FALSE) && (_SUPPORT_I2C != FALSE) && ((I2C_MASTER_PROT != I2C_MASTER_PROT_NONE) || (I2C_SLAVE_PROT != I2C_SLAVE_PROT_NONE)) */
#if (_SUPPORT_IO_DUT_SELECT_HVIO != FALSE)
#include "commlib/IO_Select.h"                                                  /* IO-Select support */
#endif /* (_SUPPORT_IO_DUT_SELECT_HVIO != FALSE) */
#if (LIN_COMM != FALSE)
#if (_SUPPORT_LIN_AA != FALSE)
#include "commlib/LIN_AutoAddressing.h"                                         /* LIN Auto-Addressing support */
#endif /* (_SUPPORT_LIN_AA != FALSE) */
#include "commlib/LIN_Communication.h"                                          /* LIN Communication support */
#include "commlib/LIN_Diagnostics.h"                                            /* LIN Diagnostics support */
#endif /* (LIN_COMM != FALSE) */
#if (PWM_COMM != FALSE)
#include "commlib/PWM_Communication.h"                                          /* PWM Communication support */
#endif /* (PWM_COMM != FALSE) */
#if (SPI_COMM != FALSE)
#include "commlib/SPI_Communication.h"                                          /* SPI Communication support */
#elif (_SUPPORT_SPI != FALSE)
#include "commlib/SPI.h"                                                        /* SPI Support */
#endif /* (SPI_COMM != FALSE) */
#if (_SUPPORT_UART != FALSE)
#include "commlib/UART.h"                                                       /* UART support */
#if (_SUPPORT_UART_IF_APP == C_UART_IF_QUAD_WHEEL_CAR)
#include "commlib/UART_QuadCar.h"                                               /* UART Quadruple-actuator car support */
#elif (_SUPPORT_UART_IF_APP == C_UART_IF_ACTUATOR)
#include "commlib/UART_Actuator.h"                                              /* UART Actuator support */
#elif (_SUPPORT_UART_IF_APP == C_UART_IF_BLUETOOTH)
#include "commlib/UART_BlueTooth.h"                                             /* UART Bluetooth support */
#endif /* (_SUPPORT_UART_QUAD_CAR != FALSE) */
#if (_SUPPORT_UART2_IF_APP == C_UART_IF_BLUETOOTH)                              /* (MMP230810-1) */
#include "commlib/UART_BlueTooth.h"                                             /* UART2 Bluetooth support */
#endif /* (_SUPPORT_UART2_IF_APP == C_UART_IF_BLUETOOTH) */
#endif /* (_SUPPORT_UART != FALSE) */

#include "../ActADC.h"                                                          /* Application ADC support */

#include <bist_inline_impl.h>
#include <bl_bist.h>                                                            /* bistHeader/bistResetInfo */
#if (LIN_COMM != FALSE) && (_SUPPORT_LIN_BUS_ACTIVITY_CHECK != FALSE)
#include <bl_tools.h>                                                           /* Used by MLX16_RESET_SIGNED */
#endif /* (LIN_COMM != FALSE) && (_SUPPORT_LIN_BUS_ACTIVITY_CHECK != FALSE) */
#if defined (__MLX81339__)
#include <flash_kf_defines.h>
#else  /* defined (__MLX81339__) */
#include <flash_defines.h>
#endif /* defined (__MLX81339__) */
#include <lib_clock.h>
#include <string.h>

#define _SUPPORT_IO_STYLE                   FALSE /*TRUE*/                      /*!< FALSE: ioports.h-style; TRUE: io.h-style */
#define _SUPPORT_BG_MEM_TEST_SEGMENTS       /*FALSE*/ TRUE                      /*!< FALSE: Memory check at once; TRUE: Memory check at smaller segments */

/*!*************************************************************************** */
/*                           DEFINITIONS                                       */
/* *************************************************************************** */
#define C_VS_LIN_MIN                        525U                                /*!< Minimum IC Vs voltage for correct LIN operation [10mV]: 5.25V */
#define C_VS_LIN_HYST                       25U                                 /*!< Hysteric IC Vs voltage [10mV]: 0.25V */

#ifndef C_MIN_POS
#define C_MIN_POS                           0x0000U                             /*!< Minimum position */
#endif
#ifndef C_MAX_POS
#define C_MAX_POS                           0xFFFEU                             /*!< Maximum position */
#endif
#ifndef C_INI_POS
#define C_INI_POS                           0x7FFFU                             /*!< Initial position */
#endif
#if !defined (C_INV_POS)
#define C_INV_POS                           0xFFFFU                             /*!< Invalid position */
#endif

#if (_SUPPORT_SSCM != FALSE)
/* The SSCM Modulation Frequency is determine by STEP and DUR as follows:
 * Fmod = CPU-Freq / (STEP * DUR)
 * For minimum acoustic noise impact, the Fmod should be equal to 2*PWM_FREQ.
 */
#define SSCM_DUR_CNT     4U                                                     /*!< M_PORT_STEP_CONF_STEP_DUR = 4 */
#define SSCM_STEP_CNT    (uint8_t)(PLL_FREQ / (2UL * PWM_FREQ * SSCM_DUR_CNT))  /*!< M_PORT_STEP_CONF_STEP_CNT */
#endif /* (_SUPPORT_SSCM != FALSE) */

/*!*************************************************************************** */
/*                           GLOBAL & LOCAL VARIABLES                          */
/* *************************************************************************** */
#pragma space nodp                                                              /* __NEAR_SECTION__ */
#if (_SUPPORT_AMBIENT_TEMP != FALSE)
int16_t l_i16AmbientTemperature = 25;                                           /*!< Ambient Temperature */
uint16_t l_u16SelfHeatingCounter = 0U;                                          /*!< Self-heating timer counter */
static uint32_t l_u32SelfHeatingIntegrator = 0U;                                /*!< Self-heating integrator storage */
#endif /* (_SUPPORT_AMBIENT_TEMP != FALSE) */
#if (_SUPPORT_DEGRADED_MODE != FALSE)
uint16_t l_u16ReversePolarityVdrop = 0U;                                        /*!< Reverse polarity diode voltage drop */
static uint16_t l_u16UnderVoltageFilterCount = 0U;                              /*!< Under-voltage period counter */
static uint16_t l_u16OverVoltageFilterCount = 0U;                               /*!< Over-voltage period counter */
#endif /* (_SUPPORT_DEGRADED_MODE != FALSE) */
#if (LIN_COMM != FALSE)
#if (_SUPPORT_LIN_BUS_ACTIVITY_CHECK != FALSE)
uint16_t l_u16Mlx4CheckPeriodCount = 0U;                                        /*!< MLX4 State check period counter */
static uint16_t l_u16MLX4_RAM_Static_CRC = 0x0000U;                             /*!< MLX4-RAM Static Tables CRC */
#endif /* (_SUPPORT_LIN_BUS_ACTIVITY_CHECK != FALSE) */
uint8_t l_u8Mlx4ErrorState = 0U;                                                /*!< Number of MLX4 Error states occurred */
#if ((_SUPPORT_LIN_UV != FALSE) || (_SUPPORT_CPU_HALT != FALSE))
static uint8_t l_u8Mlx4Connected = FALSE;                                       /*!< MLX4 Connect flag (FALSE: Disconnected; TRUE: Connected) */
#endif /* ((_SUPPORT_LIN_UV != FALSE) || (_SUPPORT_CPU_HALT != FALSE)) */
#if (_SUPPORT_LIN_UV != FALSE)
static uint16_t l_u16LinUVTimeCounter = 0U;                                     /*!< LIN UV Time-counter */
#endif /* (_SUPPORT_LIN_UV != FALSE) */
#if (LINPROT == LIN2X_HVAC52)
static uint16_t l_u16DegradeDelay = 0xFFFFU;                                    /*!< Degraded mode exit delay */
#endif /* (LINPROT == LIN2X_HVAC52) */

#if (_SUPPORT_MLX_CHIP_STATUS != FALSE)
uint32_t g_u32FlashBist = 0xFFFFFFFFUL;                                         /*!< Flash BIST result */
uint32_t g_u32NvmBist = 0xFFFFFFFFUL;                                           /*!< Non Volatile Memory BIST result */
#endif /* (_SUPPORT_MLX_CHIP_STATUS != FALSE) */
#endif /* (LIN_COMM != FALSE) */

#if (_SUPPORT_ADC_VDDA_VDDD != FALSE) && (_SUPPORT_VDDA_VDDD_LPF != FALSE) && (_SUPPORT_TRIAXIS_MLX9038x == FALSE)
static uint32_t l_u32LpfAdcVdda = (C_MIN_VDDA + C_MAX_VDDA) * 32768UL;          /*!< Filtered Vddda [ADC-LSB] */
static uint32_t l_u32LpfAdcVddd = (C_MIN_VDDD + C_MAX_VDDD) * 32768UL;          /*!< Filtered Vdddd [ADC-LSB] */
static uint16_t l_u16AdcVdda = (C_MIN_VDDA + C_MAX_VDDA) / 2U;                    /*!< Filtered VDDA [ADC-LSB] */
static uint16_t l_u16AdcVddd = (C_MIN_VDDD + C_MAX_VDDD) / 2U;                    /*!< Filtered VDDD [ADC-LSB] */
#endif /* (_SUPPORT_ADC_VDDA_VDDD != FALSE) && (_SUPPORT_VDDA_VDDD_LPF != FALSE) && (_SUPPORT_TRIAXIS_MLX9038x == FALSE) */
#pragma space none                                                              /* __NEAR_SECTION__ */

#if (LIN_COMM != FALSE) && (defined (__MLX81330A01__) || defined (__MLX81330B01__) || defined (__MLX81332A01__))
/* MMP190405-1: Increase LIN Slewrate setting by '1'-step. As LIN Slewrate is not nicely organised, a table is used.
 * LIN Slewrate[8]:     One-up:
 * 0: 100.0% (default)  3
 * 1:  87.5%            0
 * 2: 125.0%            2 (can't be faster)
 * 3: 112.5%            2
 * 4:  50.0%            7
 * 5:  37.5%            4
 * 6:  75.0%            1
 * 7:  62.5%            6
 * (JIRA: MLX81330-163) */
/*!< LIN Slewrate settings */
static const uint16_t au16LIN_SLWRT[8] = { (3U << 8), (0U << 8),
                                           (2U << 8), (2U << 8),
                                           (7U << 8), (4U << 8),
                                           (1U << 8), (6U << 8)};               /*!< LIN Slew-rate settings */
#endif /* (LIN_COMM != FALSE) && (defined (__MLX81330A01__) || defined (__MLX81330B01__) || defined (__MLX81332A01__)) */

/*!*************************************************************************** *
 * App_noinit_section_init
 * \brief   Initialise the 'no-init' section global variables.
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details Define the initial value for all 'no-init' section global and local
 *          variables.
 *          Note: Function can only be called when Non Volatile Memory Write is inactive!
 * *************************************************************************** *
 * - Call Hierarchy: AppInit()
 * - Cyclomatic Complexity: 0/1+1
 * - Nesting: 0/1
 * - Function calling: 0
 * *************************************************************************** */
static void App_noinit_section_init(void)
{
#if (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT)
    g_e8StallDetectorEna = StallDetectorEna();                                  /* Enable Stall Detector */
#if (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE)
#if (_SUPPORT_MLX_CHIP_STATUS == FALSE)
    /* During idle, ADC should be inactive to allow Flash/Non Volatile Memory BIST (DMA) */
    if (NV_HOLDING_CURR_LEVEL != 0U)
    {
        g_u8MotorHoldingCurrEna = TRUE;
    }
    else
#endif /* (_SUPPORT_MLX_CHIP_STATUS != FALSE) */
    {
        g_u8MotorHoldingCurrEna = FALSE;
    }
#if (_SUPPORT_MOTOR_POSITION == C_MOTOR_POS_ROTOR)
    g_u16ActualPosition = C_INI_POS;                                            /* Set initial position (POR) */
    g_u16TargetPosition = C_INV_POS;                                            /* Set invalid position */
#elif (_SUPPORT_MOTOR_POSITION == C_MOTOR_POS_SHAFT)
    g_u16ActualShaftAngle = C_INI_ANGLE;
    g_u16TargetShaftAngle = C_INV_ANGLE;
#endif /* (_SUPPORT_MOTOR_POSITION) */
#if (_SUPPORT_ACT_SPEED_BY_LIN == FALSE)
    g_u8MotorCtrlSpeed = (uint8_t)C_DEFAULT_MOTOR_SPEED;
#endif /* (_SUPPORT_ACT_SPEED_BY_LIN == FALSE) */
#endif /* (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) */
#if (_SUPPORT_MOTOR_POSITION == C_MOTOR_POS_NONE) || (_SUPPORT_FOC_MODE != FOC_MODE_NONE)
    g_u16ActualMotorSpeedRPM = 0U;
#endif /* (_SUPPORT_MOTOR_POSITION == C_MOTOR_POS_NONE) || (_SUPPORT_FOC_MODE != FOC_MODE_NONE) */
#if (_SUPPORT_MOTOR_POSITION == C_MOTOR_POS_NONE) || (_SUPPORT_FOC_MODE != FOC_MODE_NONE) || _SUPPORT_ACT_SPEED_BY_LIN
    g_u16TargetMotorSpeedRPM = 0U;
#endif /* (_SUPPORT_MOTOR_POSITION == C_MOTOR_POS_NONE) || (_SUPPORT_FOC_MODE != FOC_MODE_NONE) || _SUPPORT_ACT_SPEED_BY_LIN */
    g_e8MotorStatus = C_MOTOR_STATUS_STOP;
#if (_SUPPORT_DEGRADED_MODE != FALSE)
    g_e8DegradeStatus = FALSE;
#endif /* (_SUPPORT_DEGRADED_MODE != FALSE) */
#if (LINPROT == LIN2X_HVAC49) || (LINPROT == LIN2X_HVAC52) || (LINPROT == LIN2X_AIRVENT12) || (LINPROT == LIN2X_FAN01) || \
    (I2C_SLAVE_PROT == I2C_SLAVE_PROT_CONTINUOUS)
    g_e8MotorCtrlMode = (uint8_t)C_MOTOR_CTRL_STOP;
#endif /* (LINPROT == LIN2X_HVAC49) || (LINPROT == LIN2X_HVAC52) || (LINPROT == LIN2X_AIRVENT12) || (LINPROT == LIN2X_FAN01) || (I2C_SLAVE_PROT == I2C_SLAVE_PROT_CONTINUOUS) */
#elif (_SUPPORT_APP_TYPE == C_APP_RELAY)
#if (_SUPPORT_DUAL_RELAY == FALSE)
    g_e8RelayStatus = (uint8_t)C_RELAY_STATUS_OFF;
#else  /* (_SUPPORT_DUAL_RELAY == FALSE) */
    g_e8RelayStatusA = (uint8_t)C_RELAY_STATUS_OFF;
    g_e8RelayStatusB = (uint8_t)C_RELAY_STATUS_OFF;
#endif /* (_SUPPORT_DUAL_RELAY == FALSE) */
#if (_SUPPORT_DEGRADED_MODE != FALSE)
    g_e8DegradeStatus = FALSE;
#endif /* (_SUPPORT_DEGRADED_MODE != FALSE) */
#endif /* (_SUPPORT_APP_TYPE) */
} /* End of noinit_section_init() */

/*!*************************************************************************** *
 * AppInit
 * \brief   Initialise Application
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details The initialisation of the application is split in the following:
 * + A. LIN Slew-rate adjustment (MLX81330A, MLX81330B1, MLX81332A)
 * + B. Flash pre-diction setting at '2 (Performance improvement)
 * + C. (Optional) VDDA at 5V (_SUPPORT_VDDA_5V)
 * + D. (Optional) Enter user-mode (_SUPPORT_APP_USER_MODE)
 * + E. (Optional) Setup and enable CPU Clock Spread-spectrum (_SUPPORT_SSCM)
 * + F. (Optional) Initialise Error-logging management (_SUPPORT_LOG_ERRORS)
 * + G. (Optional) MLX4 ROM BIST (C_ROM_MLX4_BIST_CRC)
 * + H. Melexis Calibration & Trim Non Volatile Memory check
 * + x. (Optional) Switch to 32MHz (SPEED4), with reduced Flash Wait-states (FPLL == 32000)
 * + I. (Optional) Initialise IWD, Acknowledge AWD
 * + J. (Optional) Initialise I/O for debugging (_DEBUG_IO)
 * + K. Initialise the noinit RAM section
 * + L. Initialise ADC (VS is above 4V)
 * + M. Check User Non Volatile Memory Area
 * + x. Check Flash CRC24 against EE stored CRC24 (Count flash write cycles)
 * + N. Initialise chip (H/W) Diagnostic Protection
 * + O. Initial (Motor) Supply check & temperature
 * + x. (Optional) OSD
 * + P. Initialise Motor-driver
 * + Q. Switch to lowest priority
 * + R. (Optional) Motor Coil & Driver Self-Test
 * + S. Initialise PID-Control
 * * T. (Optional) Initialise Sensors
 * + U. Initialise Background Timer
 * + V. Initialise communication (I2C, LIN, PWM)
 * + W. Stop any motor activity immediately
 * *************************************************************************** *
 * - Call Hierarchy: main_Init()
 * - Cyclomatic Complexity: 3(4)+1
 * - Nesting: 1
 * - Function calling: 17 (builtin_mlx16_enter_user_mode(), p_AwdAck(),
 *                         ErrorLogInit(), NV_MlxCalib(), SetLastError(),
 *                         HAL_ADC_Conv_Init(), SetSystemSpeed(), WDG_activateIwd(),
 *                         NV_CheckCRC(), NV_WriteUserDefaults(), NV_WriteAPP(),
 *                         DiagnosticInit(), ADC_Init(), TimerInit(),
 *                         MotorDriverTest_OSD(), ADC_Conv_Vmotor(), ADC_Conv_TempJ())
 * *************************************************************************** */
void AppInit(void)
{
#if ((_SUPPORT_I2C_SLAVE != FALSE) && defined (PIN_FUNC_LIN) && (C_I2C_SLAVE_SCK_IO == PIN_FUNC_LIN))
    I2C_PreInit();
#endif /* ((_SUPPORT_I2C_SLAVE != FALSE) && defined (PIN_FUNC_LIN) && (C_I2C_SLAVE_SCK_IO == PIN_FUNC_LIN)) */

#if (_SUPPORT_TRIAXIS_MLX90363 != FALSE) || (_SUPPORT_TRIAXIS_MLX90367 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90372 != FALSE) || (_SUPPORT_TRIAXIS_MLX90377 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90395 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90421 != FALSE) || (_SUPPORT_TRIAXIS_MLX90422 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90425 != FALSE) || (_SUPPORT_TRIAXIS_MLX90426 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90427 != FALSE)
    uint16_t u16TriaxisError = 0U;                                              /* Last Triaxis error */
#elif (_SUPPORT_INDUCTIVE_POS_SENSOR_MLX90513 != FALSE)
    uint16_t u16IndunctivePosSensorError = 0U;                                  /* Last Inductive Position Sensor error */
#endif /* (_SUPPORT_TRIAXIS_MLX90363 != FALSE) || (_SUPPORT_TRIAXIS_MLX90367 != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90372 != FALSE) || (_SUPPORT_TRIAXIS_MLX90377 != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90395 != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90421 != FALSE) || (_SUPPORT_TRIAXIS_MLX90422 != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90425 != FALSE) || (_SUPPORT_TRIAXIS_MLX90426 != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90427 != FALSE) */

#if (LIN_COMM != FALSE) && (defined (__MLX81330A01__) || defined (__MLX81330B01__) || defined (__MLX81332A01__))

    /* ************************************* *
     * A. LIN Slew-rate adjustment
     * ************************************* */
    IO_TRIM_RCO1M = (IO_TRIM_RCO1M & ~M_TRIM_RCO1M_PRE_TR_LIN_SLEWRATE) |
                    au16LIN_SLWRT[((TrimParams.MS_Trim.u16MS_TRIM2_RCO1M_LIN & 0x0700U) >> 8)]; /* MMP190405-1 */
#endif /* (LIN_COMM != FALSE) && (defined (__MLX81330A01__) || defined (__MLX81330B01__) || defined (__MLX81332A01__)) */

#if defined (__MLX81339__)
#if (_SUPPORT_SIMULATION == FALSE)
    /* MLX81339 doesn't have pre-diction, only DED-retries.
     * Note: this part of initialisation is still in system-mode */
    {
#if (_SUPPORT_DED_RETRY != FALSE)
        /* With DED Retries (7) */
        IO_FLASH_KFHP13_FL_CTRL_S = (IO_FLASH_KFHP13_FL_CTRL_S & ~M_FLASH_KFHP13_FL_DED_RETRY) | (7U << 3);
#else  /* (_SUPPORT_DED_RETRY != FALSE) */
        IO_FLASH_KFHP13_FL_CTRL_S = (IO_FLASH_KFHP13_FL_CTRL_S & ~M_FLASH_KFHP13_FL_DED_RETRY) | (0U << 3);
#endif /* (_SUPPORT_DED_RETRY != FALSE) */
    }
#endif /* (_SUPPORT_SIMULATION == FALSE) */
#else  /* defined (__MLX81339__) */
    /* ************************************* *
     * B. Flash pre-diction setting at '2'
     *    Performance improvement
     *    (Optional: Change of DED retries)
     * ************************************* */
    {
#if (_SUPPORT_DED_RETRY != FALSE)
        /* With DED Retries (7) */
        uint16_t u16FlashPD =
            (IO_EEPROM_FLASH_FL_CTRL & ~(M_EEPROM_FLASH_FL_PREDICTION_BEHAVIOR | M_EEPROM_FLASH_FL_DED_RETRY)) |
            (2U << 6) |
            (7U << 8);
#else  /* (_SUPPORT_DED_RETRY != FALSE) */
        /* Without DED Retries */
        uint16_t u16FlashPD =
            (IO_EEPROM_FLASH_FL_CTRL & ~(M_EEPROM_FLASH_FL_PREDICTION_BEHAVIOR | M_EEPROM_FLASH_FL_DED_RETRY)) |
            (2U << 6);
#endif /* (_SUPPORT_DED_RETRY != FALSE) */
        IO_EEPROM_FLASH_FL_CTRL = u16FlashPD;
    }

#if (FPLL == 40000)
    /* ************************************* *
     * x. Switch to 40MHz (SPEED5)
     * ************************************* */
    {
#if (_SUPPORT_CPUSPEED5 != FALSE)
        RC_Settings_t CpuSpeed;

        CpuSpeed.u = CalibrationParams.u16APP_TRIM38_Speed5;
#else  /* (_SUPPORT_CPUSPEED5 != FALSE) */
        uint16_t u16CpuSpeed1 = (TrimParams.MS_Trim.u16MS_TRIM6_RCO32M_24M & M_TRIM_RCO32M_TR_RCO32M_IN);
        uint16_t u16CpuSpeed3 = (TrimParams.MS_Trim.u16MS_TRIM7_RCO32M_28M & M_TRIM_RCO32M_TR_RCO32M_IN);  /* ~40 MHz */
        /* uint16_t u16CpuSpeed3 = (TrimParams.MS_Trim.u16MS_TRIM8_RCO32M_32M & M_TRIM_RCO32M_TR_RCO32M_IN); */  /* ~50 MHz */
        RC_Settings_t CpuSpeed;

        CpuSpeed.u = TrimParams.MS_Trim.u16MS_TRIM8_RCO32M_32M + (u16CpuSpeed3 - u16CpuSpeed1);  /* 3 FWS */
        /* CpuSpeed.u = CalibrationParams.u16APP_TRIM39_Speed4 + (u16CpuSpeed3 - u16CpuSpeed1); */  /* 2 FWS */
#endif /* (_SUPPORT_CPUSPEED5 != FALSE) */
#if (_SUPPORT_APP_USER_MODE != FALSE)
        ENTER_SECTION(SYSTEM_MODE); /*lint !e534 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
        SetSystemSpeed(CpuSpeed, 0U);                                           /* Set RCO32 at 40MHz */
#if (_SUPPORT_APP_USER_MODE != FALSE)
        EXIT_SECTION(); /*lint !e438 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
    }
#endif /* (FPLL == 40000) */
#endif /* !defined (__MLX81339__) */

#if (_SUPPORT_VDDA_5V != FALSE) || (_SUPPORT_SENSOR_VDDA_5V != FALSE)
    /* ************************************* *
     * C. VDDA at 5V
     * ************************************* */
    IO_PORT_MISC_OUT |= B_PORT_MISC_OUT_SWITCH_VDDA_TO_5V;
#if defined (__MLX81330__)
    /* Re-trim VDDA at 5V (MMP230306-2) */
    {
        uint16_t u16VDDA = IO_TRIM_VDD & M_TRIM_VDD_PRE_TR_VDDA;
        u16VDDA += (2U << 0);                                                   /* Increase VDDA trim-value by 2 */
        if (u16VDDA > M_TRIM_VDD_PRE_TR_VDDA)
        {
            u16VDDA = M_TRIM_VDD_PRE_TR_VDDA;
        }
        IO_TRIM_VDD = (IO_TRIM_VDD & ~M_TRIM_VDD_PRE_TR_VDDA) | u16VDDA;
    }
#endif /* defined (__MLX81330__) */
#endif /* (_SUPPORT_VDDA_5V != FALSE) || (_SUPPORT_SENSOR_VDDA_5V != FALSE) */

#if (_SUPPORT_APP_USER_MODE != FALSE)
    /* ************************************* *
     * D. Enter user-mode
     * ************************************* */
    builtin_mlx16_enter_user_mode();
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */

#if (_SUPPORT_SSCM != FALSE)
    /* ************************************* *
     * E. Setup and enable CPU Clock Spread-spectrum
     * ************************************* */
    IO_PORT_STEP_CONF = (SSCM_STEP_CNT << 8) |                                  /* M_PORT_STEP_CONF_STEP_CNT */
                        (SSCM_DUR_CNT << 4) |                                   /* M_PORT_STEP_CONF_STEP_DUR */
                        (2U);                                                   /* M_PORT_STEP_CONF_STEP_INC = 2 */
#if defined (__MLX81330A01__) || defined (__MLX81332A01__)
    IO_PORT_SSCM_CONF |= B_PORT_SSCM_CONF_SSCM_EN;                              /* Enable the spread spectrum modulation */
#else
    IO_PORT_SSCM_CONF |= (B_PORT_SSCM_CONF_SSCM_CENTERED |                      /* Centre SSCM around middle */
                          B_PORT_SSCM_CONF_SSCM_EN);                            /* Enable the spread spectrum modulation */
#endif
#endif /* (_SUPPORT_SSCM != FALSE) */

#if (_SUPPORT_LOG_ERRORS != FALSE)
    /* ************************************* *
     * F. Initialise Error-logging management
     * ************************************* */
    ErrorLogInit();
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */

#if (LIN_COMM != FALSE)
#if defined (C_ROM_MLX4_BIST_CRC)
    /* ************************************* *
     * G. MLX4 ROM BIST
     * ************************************* */
    IO_COLIN_CFG &= ~B_COLIN_RUN;                                               /* Stop MLX4 */
    IO_ROM_BIST_CTRL = (IO_ROM_BIST_CTRL & ~(B_ROM_BIST_MASK_SIG_ERR |
                                             B_ROM_BIST_BIST |
                                             M_ROM_BIST_ECC_POSITION)) |
                       (B_ROM_BIST_SINGLE_RAMP | 2U);                           /* ECC position is 0b10 for 64bit FLASH */
    IO_ROM_BIST_ADD_START_L = (C_ROM_MLX4_START_ADDR & 0xFFFFU);
    IO_ROM_BIST_ADD_START_H = (C_ROM_MLX4_START_ADDR >> 16);
    IO_ROM_BIST_ADD_STOP_L = ((C_ROM_MLX4_END_ADDR - 4U) & 0xFFFFU);
    IO_ROM_BIST_ADD_STOP_H = ((C_ROM_MLX4_END_ADDR - 4U) >> 16);
    IO_ROM_BIST_SIG_EXPECTED_L = 0U;
    IO_ROM_BIST_SIG_EXPECTED_H = 0U;
    IO_ROM_BIST_SIG_RECEIVED_L = (uint16_t)(C_ROM_BIST_CFG_SEED & 0xFFFFUL);
    IO_ROM_BIST_SIG_RECEIVED_H = (uint16_t)(C_ROM_BIST_CFG_SEED >> 16);  /*lint !e572 */
    IO_ROM_BIST_START_BIST = C_ROM_BIST_KEY;
    /* CPU will stop here until the BIST's finishes his job. */
    NOP(); /* Inserted to let the HW BIST to not to skip the signature check in FX */
    NOP();
    if ( ((((uint32_t)IO_ROM_BIST_SIG_RECEIVED_H) << 16) | IO_ROM_BIST_SIG_RECEIVED_L) != C_ROM_MLX4_BIST_CRC)
    {
        /* Error log not initialised! */
        g_u8ErrorCommunication = TRUE;
#if (_SUPPORT_LOG_ERRORS != FALSE)
        SetLastError(C_ERR_INV_MLX4ROM_CRC);
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
    }
    p_AwdAck(); /*lint !e522 */                                                 /* Acknowledge Analogue Watchdog (MMP200625-2) */
    IO_COLIN_CFG |= B_COLIN_RUN;                                                /* Start MLX4 */
#endif /* define (C_ROM_MLX4_BIST_CRC) */
    l_u16MLX4_RAM_Static_CRC = p_CalcCRC_U16((const uint16_t *)C_RAM_MLX4_STATIC_BGN_ADDR,
                                             (C_RAM_MLX4_STATIC_END_ADDR - C_RAM_MLX4_STATIC_BGN_ADDR)/sizeof(uint16_t));
#endif /* (LIN_COMM != FALSE) */

    /* ******************************************************* *
     * H. Melexis Calibration & Trim Non Volatile Memory check
     * ******************************************************* */
    if (NV_MlxCalib() != FALSE)
    {
        g_e8ErrorElectric = (uint8_t)C_ERR_MEM_NVM_CALIB;                       /* No motor operation allowed */
#if (_SUPPORT_LOG_ERRORS != FALSE)
        SetLastError(C_ERR_INV_MLXPAGE_CRC1);
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
    }
    else
    {
        /* Calibration area is valid */
        HAL_ADC_Conv_Init();                                                    /* Setup ADC calibration data */
    }

    /* ************************************* *
     * I. Initialise IWD, Acknowledge AWD
     * ************************************* */
#if (_SUPPORT_DWD != FALSE)
    WDG_activateIwd(C_IWD_DIV, C_IWD_TO);
#endif /* (_SUPPORT_DWD != FALSE) */
    p_AwdAck(); /*lint !e522 */                                                 /* Acknowledge Analogue Watchdog .. (MMP200625-2) */

#if (_DEBUG_IO != FALSE)
    /* ************************************* *
     * J. Initialise I/O for debugging
     * ************************************* */
    DEBUG_IO_INIT();
#ifdef DEBUG_SET_IO_A
    DEBUG_SET_IO_A();
#endif /* DEBUG_SET_IO_A */
#ifdef DEBUG_SET_IO_B
    DEBUG_SET_IO_B();
#endif /* DEBUG_SET_IO_B */
#ifdef DEBUG_SET_IO_C
    DEBUG_SET_IO_C();
#endif /* DEBUG_SET_IO_C */
#ifdef DEBUG_SET_IO_D
    DEBUG_SET_IO_D();
#endif /* DEBUG_SET_IO_D */
#ifdef DEBUG_CLR_IO_A
    DEBUG_CLR_IO_A();
#endif /* DEBUG_CLR_IO_A */
#ifdef DEBUG_CLR_IO_B
    DEBUG_CLR_IO_B();
#endif /* DEBUG_CLR_IO_B */
#ifdef DEBUG_CLR_IO_C
    DEBUG_CLR_IO_C();
#endif /* DEBUG_CLR_IO_C */
#ifdef DEBUG_CLR_IO_D
    DEBUG_CLR_IO_D();
#endif /* DEBUG_CLR_IO_D */
#endif /* (_DEBUG_IO != FALSE) */

    /* ************************************* *
     * L. Initialise ADC; VS > 4V (MMP210212-2)
     * ************************************* */
    HAL_ADC_PreCheck();                                                         /* Check ADC Supplies (VDDA) */
    ADC_Init();                                                                 /* Initialise ADC */

    /* ************************************* *
     * M. Check User Non Volatile Memory Area
     * (MMP221130-1: Moved; Non Volatile Memory check before noinit section initialisation)
     * ************************************* */
#if (_SUPPORT_NV_TYPE == C_NV_FLASH)
    NV_Init();                                                                  /* Initialise the NV module */
#endif /* (_SUPPORT_NV_TYPE == C_NV_FLASH) */
    {
        uint16_t u16NV_Error = NV_CheckCRC();
        if (u16NV_Error != C_ERR_NONE)
        {
#if (_SUPPORT_LOG_ERRORS != FALSE)
            SetLastError(C_ERR_INV_USERPAGE_1 | C_ERR_EXTW);
            SetLastError(u16NV_Error);
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
            (void)NV_WriteUserDefaults(u16NV_Error);
        }
#if (_SUPPORT_NV_NON_BLOCK_WRITE != FALSE)
        GlobalInit();
#endif /* (_SUPPORT_NV_NON_BLOCK_WRITE != FALSE) */
#if (I2C_COMM != FALSE) && ((I2C_SLAVE_PROT == I2C_SLAVE_PROT_POSITIONING) || \
                            (I2C_SLAVE_PROT == I2C_SLAVE_PROT_CONTINUOUS) || \
                            (I2C_SLAVE_PROT == I2C_SLAVE_PROT_MLX_ACT) || \
                            (I2C_SLAVE_PROT == I2C_SLAVE_PROT_AS_LIN))
        {
            NV_USER_MAP_t *pNV_User = (NV_USER_MAP_t *)ADDR_NV_USER;
#if (I2C_SLAVE_PROT == I2C_SLAVE_PROT_POSITIONING) || (I2C_SLAVE_PROT == I2C_SLAVE_PROT_CONTINUOUS)
            memcpy( (void *)&I2C_Parameters, (const void *)&(pNV_User->i2c), sizeof(STD_I2C_PARAMS_t));
#elif (I2C_SLAVE_PROT == I2C_SLAVE_PROT_MLX_ACT)
            g_u8Address = pNV_User->i2c.u8Address;
#elif (I2C_SLAVE_PROT == I2C_SLAVE_PROT_AS_LIN)
            g_u8Address = pNV_User->i2c.u8Address;
#endif /* (I2C_SLAVE_PROT) */
        }
#endif /* (I2C_COMM != FALSE) && I2C_NONAUTOMOTIVE */
    }

    /* ************************************* *
     * K. Initialise the noinit RAM section
     * ************************************* */
    if ( ((IO_PORT_MISC_IN & B_PORT_MISC_IN_RSTAT) == 0U) ||
         (bistResetInfo != C_CHIP_STATE_CMD_RESET) )
    {
#if (_DEBUG_RSTAT != FALSE)
        DEBUG_SET_IO_A();
#endif /* (_DEBUG_RSTAT != FALSE) */
        App_noinit_section_init();                                              /* As this function used application Non Volatile Memory area data, it should be called after the Non Volatile Memory ECC/CRC check */
        main_noinit_section_init(); /*lint !e522 */                             /* Same as above */
#if (_DEBUG_RSTAT != FALSE)
        DEBUG_CLR_IO_A();
#endif /* (_DEBUG_RSTAT != FALSE) */
    }

#if (_SUPPORT_APP_SAVE != FALSE) && (_DEBUG_FLASH_WRITE_CYCLES != FALSE)
    /* ************************************* *
     * x. Check Flash CRC24 against previously (EE) stored CRC24.
     *    In case not the same, increase WrtCnt (Number of flash write-cycles)
     * ************************************* */
    {
        uint32_t u32NV_FlashCRC24 = (((uint32_t)NV_APP_DATA_MSW << 16) | NV_APP_DATA_LSW);
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81339__) || defined (__MLX81350__)
        uint32_t u32FlashCRC24 = *(uint32_t *)C_FLASH_CRC_ADDR;
#elif defined (__MLX81160__) || defined (__MLX81340__)
        uint32_t u32FlashCRC24 = *(uint32_t *)(C_FLASH_CRC_ADDR << 1);
#elif defined (__MLX81344__) || defined (__MLX81346__) || defined (__MLX81334__)
        uint32_t u32FlashCRC24 = p_GetExMem( (uint32_t)C_FLASH_CRC_ADDR << 1) +
                                 ((uint32_t)p_GetExMem( ((uint32_t)C_FLASH_CRC_ADDR << 1) + 2U) << 16);  /*lint !e648 */
#endif
        if (u32FlashCRC24 != u32NV_FlashCRC24)
        {
            APP_PARAMS_t AppParams;

            p_CopyU16( (sizeof(APP_PARAMS_t) / sizeof(uint16_t)),
                       (uint16_t *)&AppParams,
                       (const uint16_t *)ADDR_NV_APP_PARAMS);
            AppParams.u16ParamLSW = (uint16_t)(u32FlashCRC24 & 0xFFFFU);
            AppParams.u16ParamMSW = (uint16_t)(u32FlashCRC24 >> 16);
            AppParams.u16WrtCntL = AppParams.u16WrtCntL + 1U;
            if (AppParams.u16WrtCntL == 0U)
            {
                AppParams.u4WrtCntH++;
            }
            (void)NV_WriteAPP(&AppParams);
        }
    }
#endif /* (_SUPPORT_APP_SAVE != FALSE) && (_DEBUG_FLASH_WRITE_CYCLES != FALSE) */

    /* ************************************* *
     * N. Initialise chip (H/W) Diagnostic Protection
     * ************************************* */
    DiagnosticInit();                                                           /* Initialise Diagnostic */

#if (_SUPPORT_RESET_BY_IO != FALSE)
    /* ************************************* *
     * x. Support IC reset by IO
     * (Optional): Check IO5 state (by ADC) prior to enabling of IO5 as IC reset.
     * ************************************* */
    IO_PORT_IO_ENABLE &= ~B_PORT_IO_ENABLE_IO_ENABLE_5;
    while ( (IO_PORT_IO_IN & B_PORT_IO_IN_IO_IN_SYNC_5) == 0U);                 /* Wait for IO[5] to be high */
    IO_TRIM_MISC |= B_TRIM_MISC_ENA_IO5_RESETB;
#endif /* (_SUPPORT_RESET_BY_IO != FALSE) */

#if (FPLL == 29500)
    {
        RC_Settings_t CpuSpeed;
        uint16_t u16CpuSpeed_28M = TrimParams.MS_Trim.u16MS_TRIM7_RCO32M_28M;
        uint16_t u16CpuSpeed_32M = TrimParams.MS_Trim.u16MS_TRIM8_RCO32M_32M;
        uint16_t u16CpuSpeed_29M5 = (u16CpuSpeed_28M & 0x0FFFU) +
                                    (((u16CpuSpeed_32M & 0x0FFFU) - (u16CpuSpeed_28M & 0x0FFFU)) * 3U) / 8U;
        CpuSpeed.u = (u16CpuSpeed_28M & 0xF000U) | (u16CpuSpeed_29M5 & 0x0FFFU); /* Use WS-settings from 28MHz */
#if (_SUPPORT_APP_USER_MODE != FALSE)
        ENTER_SECTION(SYSTEM_MODE); /*lint !e534 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
        SetSystemSpeed(CpuSpeed, 0U);                                           /* Set RCO32 at 29.5MHz */
#if (_SUPPORT_APP_USER_MODE != FALSE)
        EXIT_SECTION(); /*lint !e438 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
    }
#elif (FPLL == 32000) && !defined (__MLX81339__) && !defined (__MLX81350__)     /* MMP200319-1 */
    /* ************************************* *
     * x. Switch to 32MHz (SPEED4), with reduced Flash Wait-states
     * ************************************* */
    {
        /* 1. VS is atleast above 4V (ADC_Init())
         * 2. Calib & Trim Non Volatile Memory is valid (NV_MlxCalib()) */
#if (_SUPPORT_CPUSPEED4 != FALSE)
        uint16_t u16CpuSpeed4 = CalibrationParams.u16APP_TRIM39_Speed4;
#else  /* (_SUPPORT_CPUSPEED4 != FALSE) */
        uint16_t u16CpuSpeed4 = (TrimParams.MS_Trim.u16MS_TRIM8_RCO32M_32M - 0x1000U);
#endif /* (_SUPPORT_CPUSPEED4 != FALSE) */
        if (u16CpuSpeed4 != 0U)
        {
            RC_Settings_t CpuSpeed;

            CpuSpeed.u = u16CpuSpeed4;
#if (_SUPPORT_APP_USER_MODE != FALSE)
            ENTER_SECTION(SYSTEM_MODE); /*lint !e534 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
            SetSystemSpeed(CpuSpeed, 0U);                                       /* Set RCO32 at 32MHz, with reduced flash wait-cycles */
#if (_SUPPORT_APP_USER_MODE != FALSE)
            EXIT_SECTION(); /*lint !e438 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
        }
    }
#endif /* (FPLL == 32000) */

    /* ************************************* *
     * O. Initial (Motor) Supply check & temperature
     * ************************************* */
    ADC_MeasureVsupplyAndTemperature();
#if (_SUPPORT_DEGRADED_MODE != FALSE)
    {
        /* Check Motor Supply in range before MotorDriverSelfTest */
        uint16_t u16MotorVoltage = ADC_Conv_Vmotor();
        if (u16MotorVoltage < ((NV_APPL_UVOLT - C_VOLTAGE_HYS) - l_u16ReversePolarityVdrop) )
        {
            g_e8ErrorVoltage = (uint8_t)C_ERR_VOLTAGE_UNDER;
#if (_SUPPORT_MOTOR_SELFTEST != FALSE)
            g_u8ForceMotorDriverSelfTest = TRUE;                                /* MMP180917-1 */
#endif /* (_SUPPORT_MOTOR_SELFTEST != FALSE) */
        }
        else if (u16MotorVoltage > ((NV_APPL_OVOLT + C_VOLTAGE_HYS) - l_u16ReversePolarityVdrop) )
        {
            g_e8ErrorVoltage = (uint8_t)C_ERR_VOLTAGE_OVER;
#if (_SUPPORT_MOTOR_SELFTEST != FALSE)
            g_u8ForceMotorDriverSelfTest = TRUE;                                /* MMP180917-1 */
#endif /* (_SUPPORT_MOTOR_SELFTEST != FALSE) */
        }
        else
        {
            /* Nothing */
        }
        g_e8ErrorVoltageComm = g_e8ErrorVoltage;
    }
#endif /* (_SUPPORT_DEGRADED_MODE != FALSE) */
    (void)ADC_Conv_TempJ(TRUE);
#if (_SUPPORT_TNCTOC != FALSE)
    g_e8TNCTOC |= C_TNCTOC_ADC_DONE;                                            /* Voltage & Temperature checked */
#endif /* (_SUPPORT_TNCTOC != FALSE) */

#if (_SUPPORT_OSD != FALSE) && (!defined (__MLX81160__) && !defined (__MLX81330__) && !defined (__MLX81339__) && !defined (__MLX81350__))
    /* ********************************** *
     * x. Off-State Diagnostics
     * ********************************** */
    MotorDriverTest_OSD();
#endif /* (_SUPPORT_OSD != FALSE) && (!defined (__MLX81160__) && !defined (__MLX81330__) && !defined (__MLX81339__) && !defined (__MLX81350__)) */

    /* ********************************** *
     * P. Initialise Motor-driver
     * ********************************** */
#if (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT)
    MotorDriverInit(TRUE);                                                      /* Initialise Motor-Driver */
#if (_SUPPORT_POR_MOTORDRIVER != FALSE)
    MotorDriverConfig(TRUE);
#endif /* (_SUPPORT_POR_MOTORDRIVER != FALSE) */
#elif (_SUPPORT_APP_TYPE == C_APP_RELAY)
    RelayDriverInit(TRUE);                                                      /* Initialise Relay-Driver */
#elif (_SUPPORT_APP_TYPE == C_APP_SOLENOID)
    SolenoidDriverInit();                                                       /* Initialise Solenoid-Driver */
#endif /* (_SUPPORT_APP_TYPE) */

    /* ********************************** *
     * Q. Switch to lowest priority
     * ********************************** */
#if (_SUPPORT_APP_USER_MODE != FALSE)
    builtin_mlx16_set_priority(15U);
#else  /* (_SUPPORT_APP_USER_MODE != FALSE) */
    builtin_mlx16_set_priority(7U);
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */

#if (_SUPPORT_MOTOR_SELFTEST != FALSE) && \
    ((_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT))
    /* ************************************* *
     * R. Motor Coil & Driver Self-Test
     * ************************************* */
    if (g_e8ErrorVoltage == (uint8_t)C_ERR_VOLTAGE_IN_RANGE)
    {
        MotorDriverSelfTest();
    }
#endif /* (_SUPPORT_MOTOR_SELFTEST != FALSE) && ((_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT)) */

#if (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT) || (_SUPPORT_APP_TYPE ==  C_APP_SOLENOID)
    /* ********************************* *
     * S. Initialise PID-Control
     * ********************************* */
    PID_Init();                                                                 /* PID Control initialisation */
#endif /* (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT) || (_SUPPORT_APP_TYPE ==  C_APP_SOLENOID) */

    /* ********************************* *
     * T. (Optional) Initialise Sensors
     * ********************************* */
#if (_SUPPORT_TRIAXIS_MLX90363 != FALSE) || (_SUPPORT_TRIAXIS_MLX90367 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90372 != FALSE) || (_SUPPORT_TRIAXIS_MLX90377 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90395 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90421 != FALSE) || (_SUPPORT_TRIAXIS_MLX90422 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90425 != FALSE) || (_SUPPORT_TRIAXIS_MLX90426 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90427 != FALSE)
#if (_SUPPORT_TRIAXIS_MLX90395 != FALSE)
    /* Power-on to first SCI message (Start-up time, max): 2.4ms (@ 3.3V) */
    if ( (u16TriaxisError = Triaxis_Init(0U)) != ERR_TRIAXIS_OK)                /* None-burst mode */
#else  /* (_SUPPORT_TRIAXIS_MLX90395 != FALSE) */
    /* Power-on to first SCI message (Start-up time): 23.2ms (@ 3.3V) */
    if ( (u16TriaxisError = Triaxis_Init()) != ERR_TRIAXIS_OK)
#endif /* (_SUPPORT_TRIAXIS_MLX90395 != FALSE) */
    {
#if (_SUPPORT_MOTOR_POSITION == C_MOTOR_POS_ROTOR)
        g_u16ActualPosition = C_INV_POS;                                        /* Defective position-sensor */
#elif (_SUPPORT_MOTOR_POSITION == C_MOTOR_POS_SHAFT)
        g_u16ActualShaftAngle = C_INV_ANGLE;                                    /* Defective position-sensor */
#endif /* (_SUPPORT_MOTOR_POSITION == C_MOTOR_POS_ROTOR) */
        g_e8ErrorElectric = (uint8_t)C_ERR_SENSOR;                              /* Permanent Electric Failure */
#if (_SUPPORT_LOG_ERRORS != FALSE)
        SetLastError(C_ERR_TRIAXIS_FAILS | (C_ERR_EXT | ((u16TriaxisError & 0x0FU) << 8)));
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
    }
#if (_SUPPORT_TRIAXIS_MLX90367 != FALSE) || (_SUPPORT_TRIAXIS_MLX90372 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90377 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90421 != FALSE) || (_SUPPORT_TRIAXIS_MLX90422 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90425 != FALSE) || (_SUPPORT_TRIAXIS_MLX90426 != FALSE)
    else
    {
        Triaxis_Start();                                                        /* Start SENT reception */
#if (_SUPPORT_TRIAXIS_MLX90377 != FALSE) && (_SUPPORT_SENSOR_SENT_IO != PIN_FUNC_IO_NONE)
        /* First two SPC Triggers are SPC_SCN_INIT trigger-pulses and reply a Null Frame; Third SPC Triggers for Valid Angle */
        uint16_t u16TriaxisError;
        uint16_t u16Retries = 3U;
        do
        {
            Triaxis_Trigger(C_TRIAXIS_SPC_ID);
            DELAY_US(C_TRIAXIS_SPC_FRAME_TIME);                                 /* Wait for SPC Sensor to answer (Null-frame) */
            u16TriaxisError = Triaxis_Data();
            u16Retries--;
        } while (u16Retries != 0U);
        if (u16TriaxisError == ERR_TRIAXIS_NODATA)
        {
            g_e8ErrorElectric = (uint8_t)C_ERR_SENSOR;                          /* Permanent Electric Failure */
#if (_SUPPORT_LOG_ERRORS != FALSE)
            SetLastError(C_ERR_TRIAXIS_FAILS | (C_ERR_EXT | ((u16TriaxisError & 0x0FU) << 8)));
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
        }
#endif /* (_SUPPORT_TRIAXIS_MLX90377 != FALSE) && (_SUPPORT_SENSOR_SENT_IO != PIN_FUNC_IO_NONE) */
    }
#endif /* (_SUPPORT_TRIAXIS_MLX90367 != FALSE) || (_SUPPORT_TRIAXIS_MLX90372 != FALSE) || (_SUPPORT_TRIAXIS_MLX90377 != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90421 != FALSE) || (_SUPPORT_TRIAXIS_MLX90422 != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90425 != FALSE) || (_SUPPORT_TRIAXIS_MLX90426 != FALSE) */
#elif (_SUPPORT_INDUCTIVE_POS_SENSOR_MLX90513 != FALSE)
    if ( (u16IndunctivePosSensorError = InductivePosSensor_Init()) != ERR_INDUCTIVE_POS_SENSOR_OK)
    {
#if (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE)
        g_u16ActualPosition = C_INV_POS;                                        /* Defective position-sensor */
#endif /* (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) */
        g_e8ErrorElectric = (uint8_t)C_ERR_SENSOR;                              /* Permanent Electric Failure */
#if (_SUPPORT_LOG_ERRORS != FALSE)
        SetLastError(C_ERR_TRIAXIS_FAILS | (C_ERR_EXT | ((u16IndunctivePosSensorError & 0x0FU) << 8)));
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
    }
    else
    {
        InductivePosSensor_Start();                                             /* Start SENT reception */
    }
#endif /* (_SUPPORT_TRIAXIS_MLX90363 != FALSE) || (_SUPPORT_TRIAXIS_MLX90367 != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90372 != FALSE) || (_SUPPORT_TRIAXIS_MLX90377 != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90395 != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90421 != FALSE) || (_SUPPORT_TRIAXIS_MLX90422 != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90425 != FALSE) || (_SUPPORT_TRIAXIS_MLX90426 != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90427 != FALSE) */

#if (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
#if (_SUPPORT_I2C != FALSE)
    Triaxis_Config(C_TRIAXIS_XY /*C_TRIAXIS_YZ*/);
    DELAY_US(2200U);                                                            /* (Diagnostic) Recovery time */
#endif /* (_SUPPORT_I2C != FALSE) */
    (void)Triaxis_Init();
#endif /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */

#if (_SUPPORT_DUAL_HALL_LATCH_MLX92251 != FALSE) || (_SUPPORT_DUAL_HALL_LATCH_MLX92255 != FALSE)
    if (DualHallLatch_Init() == ERR_DUAL_HALL_LATCH_OK)
    {
        DualHallLatch_Start();
    }
    else
    {
        g_e8ErrorElectric = (uint8_t)C_ERR_SENSOR;                              /* Permanent Electric Failure */
    }
#endif /* (_SUPPORT_DUAL_HALL_LATCH_MLX92251 != FALSE) || (_SUPPORT_DUAL_HALL_LATCH_MLX92255 != FALSE) */

#if (_SUPPORT_HALL_LATCH_MLX9227x != FALSE)
    if (HallLatch_Init() == ERR_HALL_LATCH_OK)
    {
        HallLatch_Start();
    }
    else
    {
        g_e8ErrorElectric = (uint8_t)C_ERR_SENSOR;                              /* Permanent Electric Failure */
    }
#endif /* (_SUPPORT_HALL_LATCH_MLX9227x != FALSE) */

#if (_SUPPORT_HALL_LATCH != FALSE)
    HallLatchInit();                                                            /* Hall Latch initialisation */
#if (_SUPPORT_HALL_LATCH_DIAG == FALSE) && (_SUPPORT_NR_OF_HL == 3)
    (void)HallLatchStart();
#endif /* (_SUPPORT_HALL_LATCH_DIAG == FALSE) && (_SUPPORT_NR_OF_HL == 3) */
#endif /* (_SUPPORT_HALL_LATCH != FALSE) */

#if (_SUPPORT_PRESSURE_MLX90829 != FALSE)
    (void)Pressure_Init();
    Pressure_Start();                                                           /* Start SENT reception */
#endif /* (_SUPPORT_PRESSURE_MLX90829 != FALSE) */

#if (_SUPPORT_NTC != FALSE)
    NTC_Init();
#endif /* (_SUPPORT_NTC != FALSE) */

#if (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT)
    g_e8MotorStatus = C_MOTOR_STATUS_STOP;
#elif (_SUPPORT_APP_TYPE == C_APP_RELAY)
#if (_SUPPORT_DUAL_RELAY == FALSE)
    g_e8RelayStatus = (uint8_t)C_RELAY_STATUS_OFF;
#else  /* (_SUPPORT_DUAL_RELAY == FALSE) */
    g_e8RelayStatusA = (uint8_t)C_RELAY_STATUS_OFF;
    g_e8RelayStatusB = (uint8_t)C_RELAY_STATUS_OFF;
#endif /* (_SUPPORT_DUAL_RELAY == FALSE) */
#elif (_SUPPORT_APP_TYPE == C_APP_SOLENOID)
    g_e8SolenoidStatus = (uint8_t)C_SOLENOID_STATUS_DEACTIVATED;
#endif /* (_SUPPORT_APP_TYPE) */

    /* ************************************* *
     * U. Initialise Background Timer
     * ************************************* */
    TimerInit();                                                                /* Initialise (Core) Timer */

#if (_SUPPORT_IO_DUT_SELECT_HVIO != FALSE)
    /* *************************************** *
     * ?. I/O Configuration Selection
     * *************************************** */
    {
        uint8_t u8IoSelection;

        IO_DUT_SelectInit();
        u8IoSelection = IO_DUT_GetSelection(C_PI_TICKS_20MS);
        if (u8IoSelection == C_IO_SELECT_LOW)
        {
            /* IO-Selection is LOW */
        }
        else if (u8IoSelection == C_IO_SELECT_HIGH)
        {
            /* IO-Selection is HIGH */
        }
        else
        {
            /* Error */
        }
    }
#endif /* (_SUPPORT_IO_DUT_SELECT_HVIO != FALSE) */

    /* *************************************** *
     * V. Initialise communication (I2C, LIN, PWM)
     *   1. Analogue (ANA_COMM)
     *   2. CAN (via SPI) (CAN_COMM)
     *   3. I2C (I2C_COMM)
     *   4. LIN (LIN_COMM)
     *   5. PWM (PWM_COMM)
     *   6. SPI (SPI_COMM)
     *   7. UART (UART_COMM), including Blue-tooth
     * *************************************** */
#if (ANA_COMM != FALSE)
    /* Analogue communication */
#if (C_ANA_IO == PIN_FUNC_IO_1)
    IO_PORT_IO_CFG1 = (IO_PORT_IO_CFG1 & ~M_PORT_IO_CFG1_IO1_OUT_SEL) | C_PORT_IO_CFG1_IO1_OUT_SEL_SOFT;
    IO_PORT_IO_OUT_SOFT = IO_PORT_IO_OUT_SOFT & ~C_PORT_IO_OUT_SOFT_IO1_OUT;
    IO_PORT_IO_ENABLE = (IO_PORT_IO_ENABLE & ~(B_PORT_IO_ENABLE_IO_ENABLE_1 | B_PORT_IO_ENABLE_IO_DISREC_1));
#if defined (__MLX81340B01__) || defined (__MLX81344B01__) || defined (__MLX81346B01__)
    IO_PORT_IO_OUT_EN = (IO_PORT_IO_OUT_EN & ~(B_PORT_IO_OUT_EN_IO_HS_ENABLE_1 | B_PORT_IO_OUT_EN_IO_OD_ENABLE_1)) | B_PORT_IO_OUT_EN_IO_LV_ENABLE_1;  /* LV-mode */
#endif
#elif (C_ANA_IO == PIN_FUNC_IO_2)
    IO_PORT_IO_CFG1 = (IO_PORT_IO_CFG1 & ~M_PORT_IO_CFG1_IO2_OUT_SEL) | C_PORT_IO_CFG1_IO2_OUT_SEL_SOFT;
    IO_PORT_IO_OUT_SOFT = IO_PORT_IO_OUT_SOFT & ~C_PORT_IO_OUT_SOFT_IO2_OUT;
    IO_PORT_IO_ENABLE = (IO_PORT_IO_ENABLE & ~(B_PORT_IO_ENABLE_IO_ENABLE_2 | B_PORT_IO_ENABLE_IO_DISREC_2));
#if defined (__MLX81340B01__) || defined (__MLX81344B01__) || defined (__MLX81346B01__)
    IO_PORT_IO_OUT_EN = (IO_PORT_IO_OUT_EN & ~(B_PORT_IO_OUT_EN_IO_HS_ENABLE_2 | B_PORT_IO_OUT_EN_IO_OD_ENABLE_2)) | B_PORT_IO_OUT_EN_IO_LV_ENABLE_2;  /* LV-mode */
#endif
#elif (C_ANA_IO == PIN_FUNC_IO_3)
    IO_PORT_IO_CFG1 = (IO_PORT_IO_CFG1 & ~M_PORT_IO_CFG1_IO3_OUT_SEL) | C_PORT_IO_CFG1_IO3_OUT_SEL_SOFT;
    IO_PORT_IO_OUT_SOFT = IO_PORT_IO_OUT_SOFT & ~C_PORT_IO_OUT_SOFT_IO3_OUT;
    IO_PORT_IO_ENABLE = (IO_PORT_IO_ENABLE & ~(B_PORT_IO_ENABLE_IO_ENABLE_3 | B_PORT_IO_ENABLE_IO_DISREC_3));
#if defined (__MLX81344B01__) || defined (__MLX81346B01__)
    IO_PORT_IO_OUT_EN = (IO_PORT_IO_OUT_EN & ~(B_PORT_IO_OUT_EN_IO_HS_ENABLE_3 | B_PORT_IO_OUT_EN_IO_OD_ENABLE_3)) | B_PORT_IO_OUT_EN_IO_LV_ENABLE_3;  /* LV-mode */
#endif
#elif (C_ANA_IO == PIN_FUNC_IO_4)
    IO_PORT_IO_CFG1 = (IO_PORT_IO_CFG1 & ~M_PORT_IO_CFG1_IO4_OUT_SEL) | C_PORT_IO_CFG1_IO4_OUT_SEL_SOFT;
    IO_PORT_IO_OUT_SOFT = IO_PORT_IO_OUT_SOFT & ~C_PORT_IO_OUT_SOFT_IO4_OUT;
    IO_PORT_IO_ENABLE = (IO_PORT_IO_ENABLE & ~(B_PORT_IO_ENABLE_IO_ENABLE_4 | B_PORT_IO_ENABLE_IO_DISREC_4));
#if defined (__MLX81344B01__) || defined (__MLX81346B01__)
    IO_PORT_IO_OUT_EN = (IO_PORT_IO_OUT_EN & ~(B_PORT_IO_OUT_EN_IO_HS_ENABLE_4 | B_PORT_IO_OUT_EN_IO_OD_ENABLE_4)) | B_PORT_IO_OUT_EN_IO_LV_ENABLE_4;  /* LV-mode */
#endif
#elif (C_ANA_IO == PIN_FUNC_IO_5)
    IO_PORT_IO_CFG1 = (IO_PORT_IO_CFG1 & ~M_PORT_IO_CFG1_IO5_OUT_SEL) | C_PORT_IO_CFG1_IO5_OUT_SEL_SOFT;
    IO_PORT_IO_OUT_SOFT = IO_PORT_IO_OUT_SOFT & ~C_PORT_IO_OUT_SOFT_IO5_OUT;
    IO_PORT_IO_ENABLE = (IO_PORT_IO_ENABLE & ~(B_PORT_IO_ENABLE_IO_ENABLE_5 | B_PORT_IO_ENABLE_IO_DISREC_5));
#elif (C_ANA_IO == PIN_FUNC_IO_6)
    IO_PORT_IO_CFG1 = (IO_PORT_IO_CFG1 & ~M_PORT_IO_CFG1_IO6_OUT_SEL) | C_PORT_IO_CFG1_IO6_OUT_SEL_SOFT;
    IO_PORT_IO_OUT_SOFT = IO_PORT_IO_OUT_SOFT & ~C_PORT_IO_OUT_SOFT_IO6_OUT;
    IO_PORT_IO_ENABLE = (IO_PORT_IO_ENABLE & ~(B_PORT_IO_ENABLE_IO_ENABLE_6 | B_PORT_IO_ENABLE_IO_DISREC_6));
#elif (C_ANA_IO == PIN_FUNC_IO_7)
    IO_PORT_IO_CFG1 = (IO_PORT_IO_CFG1 & ~M_PORT_IO_CFG1_IO7_OUT_SEL) | C_PORT_IO_CFG1_IO7_OUT_SEL_SOFT;
    IO_PORT_IO_OUT_SOFT = IO_PORT_IO_OUT_SOFT & ~C_PORT_IO_OUT_SOFT_IO7_OUT;
    IO_PORT_IO_ENABLE = (IO_PORT_IO_ENABLE & ~(B_PORT_IO_ENABLE_IO_ENABLE_7 | B_PORT_IO_ENABLE_IO_DISREC_7));
#endif /* (C_ANA_IO == PIN_FUNC_IO_4) */
#endif /* (ANA_COMM != FALSE) */

#if (CAN_COMM != FALSE)
    /* Initialise CAN interface */
    CAN_Init();                                                                 /* Initialise CAN communication interface */
#endif /* (CAN_COMM != FALSE) */

#if (_SUPPORT_I2C != FALSE)
    /* I2C communication */
#if (I2C_MASTER_PROT == I2C_MASTER_PROT_GENERIC) || (I2C_MASTER_PROT == I2C_MASTER_PROT_MEMORY)
    I2C_MasterInit();
#elif (I2C_SLAVE_PROT == I2C_SLAVE_PROT_GENERIC)
    I2C_SlaveInit(C_I2C_NV_ADDR);
    I2C_SlaveStart();
#elif (I2C_SLAVE_PROT == I2C_SLAVE_PROT_POSITIONING) || (I2C_SLAVE_PROT == I2C_SLAVE_PROT_CONTINUOUS)
    I2C_SlaveInit(I2C_Parameters.u8Address);
    I2C_ActuatorInit();
    I2C_SlaveStart();
#elif (I2C_SLAVE_PROT == I2C_SLAVE_PROT_MLX_ACT)
    I2C_SlaveInit(g_u8Address);
    I2C_MlxActuatorInit();
    I2C_MlxActStatus(TRUE);
    I2C_SlaveStart();
#elif (I2C_SLAVE_PROT == I2C_SLAVE_PROT_AS_LIN)
    I2C_SlaveInit(g_u8Address);
    I2C_AsLinInit();
    I2C_SlaveStart();
#endif
#endif /* (_SUPPORT_I2C != FALSE) */

#if (LIN_COMM != FALSE)
    /* LIN communication */
#if (_DEBUG_RSTAT != FALSE)
    DEBUG_SET_IO_A();
#endif /* (_DEBUG_RSTAT != FALSE) */
    LIN_Init();                                                                 /* Initialise LIN communication interface */
#if ((_SUPPORT_LIN_UV != FALSE) || (_SUPPORT_CPU_HALT != FALSE))
    l_u8Mlx4Connected = TRUE;
#endif /* ((_SUPPORT_LIN_UV != FALSE) || (_SUPPORT_CPU_HALT != FALSE)) */

#if (_SUPPORT_MLX_DEBUG_MODE != FALSE)
    /* Check chip-state for LIN-command RESET, to setup diagnostic-response */
    if (bistResetInfo == (uint16_t)C_CHIP_STATE_CMD_RESET)
    {
        RfrDiagReset();                                                         /* Prepare a diagnostics response reply */
        bistResetInfo = (BistResetInfo_t)C_CHIP_STATE_COLD_START;
    }
#endif /* (_SUPPORT_MLX_DEBUG_MODE != FALSE) */
#if (_DEBUG_RSTAT != FALSE)
    DEBUG_CLR_IO_A();
#endif /* (_DEBUG_RSTAT != FALSE) */
#endif /* (LIN_COMM != FALSE) */

#if (GPIO_COMM != FALSE)
    GPIO_OnOff_Init();
#endif /* (GPIO_COMM != FALSE) */

#if (PWM_COMM != FALSE)
    /* PWM communication */
    PWM_Init();                                                                 /* Initialise PWM communication interface */
#endif /* (PWM_COMM != FALSE) */

#if (SPI_COMM != FALSE)
    /* SPI communication */
    SPI_Init();
#elif (_SUPPORT_SPI != FALSE)
#if (_SUPPORT_SPI_MASTER != FALSE) && (_SUPPORT_CHIP_TEST == FALSE) && \
    (_SUPPORT_TRIAXIS_MLX90363 == FALSE) && (_SUPPORT_TRIAXIS_MLX90395 == FALSE) && (_SUPPORT_TRIAXIS_MLX90427 == FALSE)
    InitSpiModule(C_SPI_BUF_SZ);
    g_au8SpiTxBuf[0U] = (uint8_t)0xAAU;                                         /* Test */
    g_au8SpiTxBuf[1U] = (uint8_t)0x11U;
    g_au8SpiTxBuf[2U] = (uint8_t)0x22U;
    g_au8SpiTxBuf[3U] = (uint8_t)0x44U;
    g_u8SpiTxLen = 4U;
    g_u8SpiRxLen = 4U;
    SPI_Transfer();
#endif /* (_SUPPORT_SPI_MASTER != FALSE) && (_SUPPORT_CHIP_TEST == FALSE) */
#endif /* (SPI_COMM != FALSE) */

#if (UART_COMM != FALSE)
#if (_SUPPORT_UART_IF_APP == C_UART_IF_SCOPE)
    UART_ScopeInit();
#elif (_SUPPORT_UART_IF_APP == C_UART_IF_QUAD_WHEEL_CAR)
    UART_QuadCarInit();
#elif (_SUPPORT_UART_IF_APP == C_UART_IF_ACTUATOR)
    UART_ActuatorInit();
#elif (_SUPPORT_UART_IF_APP == C_UART_IF_BLUETOOTH) || (_SUPPORT_UART2_IF_APP == C_UART_IF_BLUETOOTH)  /* (MMP230810-1) */
    UART_BluetoothInit();
#elif (_SUPPORT_UART_IF_APP == C_UART_IF_TERMINAL) || (_SUPPORT_UART2_IF_APP == C_UART_IF_TERMINAL)  /* (MMP230810-1) */
    UART_TerminalInit();
#elif (_SUPPORT_UART_IF_APP == C_UART_IF_MLX_ACT) || (_SUPPORT_UART2_IF_APP == C_UART_IF_MLX_ACT)  /* (MMP240430-1) */
    UART_MlxActInit();
    UART_MlxActuatorInit();
    UART_MlxActStatus(TRUE);
#elif (_SUPPORT_UART_IF_APP == C_UART_IF_DEBUG)
    UART_Init();
    g_au8UartTxBuf[0] = 0x96U;
    g_au8UartTxBuf[1] = 0x11U;
    g_au8UartTxBuf[2] = 0x22U;
    g_au8UartTxBuf[3] = 0x44U;
    g_au8UartTxBuf[4] = 0x88U;
    g_au8UartTxBuf[5] = 0xAAU;
    g_au8UartTxBuf[6] = 0x55U;
    g_au8UartTxBuf[7] = 0x69U;
#if (_SUPPORT_UART_DMA == FALSE)
    g_u8UartTxBufIdx = 0U;
#endif /* (_SUPPORT_UART_DMA == FALSE) */
    UART_Start(C_UART_MODE_RX | C_UART_MODE_TX);                                /* Enable UART RX and TX */
#endif /* (_SUPPORT_UART_IF_APP) */
#endif /* (UART_COMM != FALSE) */

#if (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT)
    /* ********************************************** *
     * W. Stop any motor activity immediately
     * ********************************************** */
#if (_SUPPORT_FAST_STOP == FALSE)
    MotorDriverStop( (uint16_t)C_STOP_IMMEDIATE);                               /* Start-up: Energies coils if needed */
#else  /* (_SUPPORT_FAST_STOP == FALSE) */
    MotorDriverStop( (uint16_t)C_STOP_FAST_STOP);                               /* Start-up: Energies coils if needed */
#endif /* (_SUPPORT_FAST_STOP == FALSE) */
#if (_SUPPORT_REWIND != FALSE)
    if (NV_RESTALL_POR != 0U)
    {
        g_u8RewindFlags = (uint8_t)(C_REWIND_DIRECTION_ANY | C_REWIND_STALL_DETECT);
    }
#endif /* (_SUPPORT_REWIND != FALSE) */
#endif /* (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT) */

    bistHeader = (BistHeader_t)C_CHIP_HEADER;
    bistResetInfo = (BistResetInfo_t)C_CHIP_STATE_FATAL_RECOVER_ENA;            /* Don't clear RAM at reset */
#if (_SUPPORT_RSTAT != FALSE)
    {
        uint16_t u16MiscOut = IO_PORT_MISC_OUT & ~(B_PORT_MISC_OUT_CLEAR_RSTAT | B_PORT_MISC_OUT_SET_RSTAT);
        IO_PORT_MISC_OUT = u16MiscOut | B_PORT_MISC_OUT_SET_RSTAT;              /* RAM is valid */
        IO_PORT_MISC_OUT = u16MiscOut;
    }
#endif /* (_SUPPORT_RSTAT != FALSE) */

} /* End of AppInit() */

/*!*************************************************************************** *
 * AppResetFlags
 * \brief   Clear (reset) LIN status flags
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details This function reset flags before each actuator operation
 * *************************************************************************** *
 * - Call Hierarchy: HandleCalibrationMotorRequest()
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 0
 * *************************************************************************** */
void AppResetFlags(void)
{
    g_u8ChipResetOcc = FALSE;
    g_u8StallOcc = FALSE;
    g_e8EmergencyRunOcc = (uint8_t)C_SAFETY_RUN_NO;
#if (_SUPPORT_MECHANICAL_ERROR != FALSE)
    g_u8MechError = FALSE;
#endif /* (_SUPPORT_MECHANICAL_ERROR != FALSE) */
#if FALSE
    if ( (g_e8ErrorElectric != (uint8_t)C_ERR_NONE) &&
         (g_e8ErrorElectric & (uint8_t)C_ERR_PERMANENT) == (uint8_t)C_ERR_NONE)
    {
        /* Clear non-permanent error code */
        g_e8ErrorElectric = (uint8_t)C_ERR_NONE;
    }
#endif
} /* End of AppResetFlags() */

#if (_SUPPORT_AMBIENT_TEMP != FALSE)
/*!*************************************************************************** *
 * SelfHeatCompensation
 * \brief   Self heating compensation
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details The self-heating compensation is split into two parts:
 *  1. Chip self-heating based on supply-voltage level (3-6C).
 *  2. Chip self-heating based on motor-current through FET's (up to 10-15C).
 * *************************************************************************** *
 * - Call Hierarchy: AppTemperatureCheck()
 * - Cyclomatic Complexity: 2(4)+1
 * - Nesting: 2
 * - Function calling: 3 (Get_ChipTemperature(), Get_SupplyVoltage(),
 *                        Get_MotorCurrentMovAvgxN())
 *           Optional: 1 (Get_RawTemperature())
 * *************************************************************************** */
void SelfHeatCompensation(void)
{
    if (l_u16SelfHeatingCounter >= C_SELFHEAT_COMP_PERIOD)
    {
        uint16_t u16SelfHeatingDrv;
        uint16_t u16SelfHeatingIC;

        l_u16SelfHeatingCounter -= C_SELFHEAT_COMP_PERIOD;
#if ((_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT)) && \
    (defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__))
        /* Driver IC */
#if (C_MOVAVG_SSZ >= 1)
        l_u32SelfHeatingIntegrator = p_MulU32byU16(l_u32SelfHeatingIntegrator, C_SELFHEAT_INTEGRATOR) +
                                     ((Get_MotorCurrentMovAvgxN() + (1U << (C_MOVAVG_SSZ - 1U))) >> C_MOVAVG_SSZ);
#else  /* (C_MOVAVG_SSZ >= 1) */
        l_u32SelfHeatingIntegrator = p_MulU32byU16(l_u32SelfHeatingIntegrator, C_SELFHEAT_INTEGRATOR) +
                                     Get_MotorCurrentMovAvgxN();
#endif /* (C_MOVAVG_SSZ >= 1) */
#elif (_SUPPORT_APP_TYPE == C_APP_RELAY) || (defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__))
        /* Pre-driver IC (MMP220128-2) */
        l_u32SelfHeatingIntegrator = p_MulU32byU16(l_u32SelfHeatingIntegrator, C_SELFHEAT_INTEGRATOR);
#elif (_SUPPORT_APP_TYPE == C_APP_SOLENOID)
        /* Driver IC */
        l_u32SelfHeatingIntegrator = p_MulU32byU16(l_u32SelfHeatingIntegrator, C_SELFHEAT_INTEGRATOR) +
                                     ((Get_MotorCurrentMovAvgxN() + (1U << (C_MOVAVG_SSZ - 1U))) >> C_MOVAVG_SSZ);
#endif /* (_SUPPORT_APP_TYPE ) */
        u16SelfHeatingDrv = (uint16_t)p_MulU32byU16(l_u32SelfHeatingIntegrator, C_SELFHEAT_CONST);    /* (MMP200616-1) */

        /* Chip-selfheating based on supply-level/mode (MMP200616-1) */
#if (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT)
        if ( (g_e8MotorStatus & C_MOTOR_STATUS_STOP_MASK) == C_MOTOR_STATUS_STOP)
#elif (_SUPPORT_APP_TYPE == C_APP_RELAY)
#if (_SUPPORT_DUAL_RELAY == FALSE)
        if ( (g_e8RelayStatus & (uint8_t)C_RELAY_STATUS_MASK) == (uint8_t)C_RELAY_STATUS_OFF)
#else  /* (_SUPPORT_DUAL_RELAY == FALSE) */
        if ( ((g_e8RelayStatusA & (uint8_t)C_RELAY_STATUS_MASK) == (uint8_t)C_RELAY_STATUS_OFF) &&
             ((g_e8RelayStatusB & (uint8_t)C_RELAY_STATUS_MASK) == (uint8_t)C_RELAY_STATUS_OFF) )
#endif /* (_SUPPORT_DUAL_RELAY == FALSE) */
#elif (_SUPPORT_APP_TYPE == C_APP_SOLENOID)
        if ( (g_e8SolenoidStatus & C_SOLENOID_STATUS_MASK) == C_SOLENOID_STATUS_DEACTIVATED)
#endif /* (_SUPPORT_APP_TYPE) */
        {
            u16SelfHeatingIC = p_DivU16_U32byU16( (uint32_t)Get_SupplyVoltage(), C_SELFHEAT_IC_IDLE);
        }
        else
        {
            u16SelfHeatingIC = p_DivU16_U32byU16( (uint32_t)Get_SupplyVoltage(), C_SELFHEAT_IC_ACTIVE);
        }
        u16SelfHeatingIC += C_SELFHEAT_IC;

        /* Ambient temperature estimation */
        l_i16AmbientTemperature = (Get_ChipTemperature() - (int16_t)(u16SelfHeatingDrv + u16SelfHeatingIC));

#if (_SUPPORT_CPFREQ_TEMPCOMP != FALSE)
#if (defined (__MLX81330B02__) || defined (__MLX81332B02__) || defined (__MLX81334__))
        /* Charge-pump frequency compensation */
        {
            int16_t i16TempDiff = (Get_RawTemperature() - Get_TempMidADC());
            uint16_t u16TrimChargePumpClock = g_u16CP_FreqTrim_RT;
            int16_t i16CPCLK82_TempCoef = (int16_t)CalibrationParams.u8APP_TRIM29_CPCLK82_HighT;
#if !defined (__MLX81339__)
            int16_t i16CPCLK60_TempCoef = (int16_t)CalibrationParams.u8APP_TRIM29_CPCLK60_HighT;
            int16_t i16CPCLK_TempCoef;
            if (i16CPCLK60_TempCoef == 0)
            {
                i16CPCLK60_TempCoef = C_DEF_CPCLK60_TC;
            }
#endif /* !defined (__MLX81339__) */
            if (i16CPCLK82_TempCoef == 0)
            {
                i16CPCLK82_TempCoef = C_DEF_CPCLK82_TC;
            }
#if !defined (__MLX81339__)
            i16CPCLK_TempCoef = i16CPCLK60_TempCoef +
                                p_MulDivI16_I16byI16byI16( (i16CPCLK82_TempCoef -
                                                            i16CPCLK60_TempCoef),
                                                           (_SUPPORT_CPFREQ - 60), (82 - 60));
            u16TrimChargePumpClock =
                (u16TrimChargePumpClock + (int16_t)(p_MulI32_I16byI16(i16CPCLK_TempCoef, i16TempDiff) >> 7));
            u16TrimChargePumpClock = ((u16TrimChargePumpClock << 2) & M_TRIM1_DRV_PRE_TRIM_DRVMOD_CPCLK);
#else  /* !defined (__MLX81339__) */
            u16TrimChargePumpClock =
                (u16TrimChargePumpClock + (int16_t)(p_MulI32_I16byI16(i16CPCLK82_TempCoef, i16TempDiff) >> 7));
            u16TrimChargePumpClock = (u16TrimChargePumpClock & M_TRIM1_DRV_PRE_TRIM_DRVMOD_CPCLK);
#endif /* !defined (__MLX81339__) */
            IO_PORT_SSCM2_CONF &= ~B_PORT_SSCM2_CONF_SSCM2_EN;                  /* Disable the spread spectrum modulation */
            IO_TRIM1_DRV = (IO_TRIM1_DRV & ~M_TRIM1_DRV_PRE_TRIM_DRVMOD_CPCLK) | u16TrimChargePumpClock;
#if (_SUPPORT_CP_SSCM != FALSE)
            IO_PORT_SSCM2_CONF |= B_PORT_SSCM2_CONF_SSCM2_EN;                   /* Enable the spread spectrum modulation */
#endif /* (_SUPPORT_CP_SSCM != FALSE) */
        }
#elif defined (__MLX81339__) || defined (__MLX81350__)
        /* Charge-pump frequency compensation */
        {
            int16_t i16TempDiff = (Get_RawTemperature() - Get_TempMidADC());
            uint16_t u16TrimChargePumpClock = g_u16CP_FreqTrim_RT;
            int16_t i16CPCLK_TempCoef;
            if (i16TempDiff > 0)
            {
                /* Below Mid-Temperature (25/35C) */
                i16CPCLK_TempCoef = (int16_t)CalibrationParams.u8APP_TRIM29_CPCLK_LowT;
            }
            else
            {
                /* Above Mid-Temperature (25/35C) */
                i16CPCLK_TempCoef = (int16_t)CalibrationParams.u8APP_TRIM29_CPCLK_HighT;
            }
            u16TrimChargePumpClock =
                (u16TrimChargePumpClock + (int16_t)(p_MulI32_I16byI16(i16CPCLK_TempCoef, i16TempDiff) >> 7));
            u16TrimChargePumpClock = (u16TrimChargePumpClock & M_TRIM1_DRV_PRE_TRIM_DRVMOD_CPCLK);
            IO_PORT_SSCM2_CONF &= ~B_PORT_SSCM2_CONF_SSCM2_EN;                  /* Disable the spread spectrum modulation */
            IO_TRIM1_DRV = (IO_TRIM1_DRV & ~M_TRIM1_DRV_PRE_TRIM_DRVMOD_CPCLK) | u16TrimChargePumpClock;
#if (_SUPPORT_CP_SSCM != FALSE)
            IO_PORT_SSCM2_CONF |= B_PORT_SSCM2_CONF_SSCM2_EN;                   /* Enable the spread spectrum modulation */
#endif /* (_SUPPORT_CP_SSCM != FALSE) */
        }
#endif /* (defined (__MLX81330B02__) || defined (__MLX81332B02__) || defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__)) */
#endif /* (_SUPPORT_CPFREQ_TEMPCOMP != FALSE) */
    }
} /* End of SelfHeatCompensation() */
#endif /* (_SUPPORT_AMBIENT_TEMP != FALSE) */

/*!*************************************************************************** *
 * AppCurrentCheck()
 * \brief   Application Current check
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details Measure/check motor driver current
 * *************************************************************************** *
 * - Call Hierarchy: AppDegradedCheck()
 * - Cyclomatic Complexity: 2+1
 * - Nesting: 2
 * - Function calling: 3 (ADC_MeasureVsupplyAndTemperature(),
 *                        MotorDriverHoldCurrentMeasure(),
 *                        ADC_Conv_Cmotor())
 * *************************************************************************** */
static void AppCurrentCheck(void)
{
    /* ******************************* */
    /* *** d. Motor Driver current *** */
    /* ******************************* */
#if (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT)
    if ( (g_e8MotorStatus == C_MOTOR_STATUS_STOP)
#if (LIN_COMM != FALSE) && (_SUPPORT_LIN_AA != FALSE)
         && (l_u8AdcMode != (uint8_t)C_ADC_MODE_LINAA)
#endif /* (LIN_COMM != FALSE) && (_SUPPORT_LIN_AA != FALSE) */
#if (_SUPPORT_MOTION_DET != C_MOTION_DET_NONE)
         && (l_u8AdcMode != (uint8_t)C_ADC_MODE_MOV_DET)
#endif /* (_SUPPORT_MOTION_DET != C_MOTION_DET_NONE) */
         )
#elif (_SUPPORT_APP_TYPE == C_APP_RELAY) || (_SUPPORT_APP_TYPE == C_APP_SOLENOID)
    if (l_u8AdcMode != (uint8_t)C_ADC_MODE_RUN_HW)
#endif /* (_SUPPORT_APP_TYPE) */
    {
        /* Stop-mode without holding current */
        ADC_MeasureVsupplyAndTemperature();
    }
#if (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT)
    else if (g_e8MotorStatus == C_MOTOR_STATUS_HOLD)
    {
        /* In holding mode, ADC samples motor current continuously, as well as (Motor)supply and temperature. */
        MotorDriverHoldCurrentMeasure();                                        /* MMP200223-1 */
    }
#endif /* (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT) */
    else
    {
        /* In running mode, ADC samples motor current continuously, as well as (Motor)supply and temperature. */
    }
    (void)ADC_Conv_Cmotor();
} /* End of AppCurrentCheck() */

#if (_SUPPORT_ADC_VDDA_VDDD != FALSE) && (_SUPPORT_TRIAXIS_MLX9038x == FALSE)
/*!*************************************************************************** *
 * AppInternalSupplyCheck()
 * \brief   Application Internal Supply check
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details Check the internal supply system against MIN-MAX levels
 *          VDDA
 *          VDDD
 * *************************************************************************** *
 * - Call Hierarchy: AppSupplyCheck()
 * - Cyclomatic Complexity: 4+1
 * - Nesting: 2
 * - Function calling: 3 (Get_AdcVdda(), Get_AdcVddd(), SetLastError())
 * *************************************************************************** */
static void AppInternalSupplyCheck(void)
{
    /* VDDA check */
#if (_SUPPORT_VDDA_VDDD_LPF == FALSE)
    uint16_t u16AdcValue = Get_AdcVdda();
#else  /* (_SUPPORT_VDDA_VDDD_LPF == FALSE) */
    uint16_t u16AdcValue = l_u16AdcVdda;
#endif /* (_SUPPORT_VDDA_VDDD_LPF == FALSE) */
#if (_SUPPORT_VDDA_5V == FALSE) && (_SUPPORT_SENSOR_VDDA_5V == FALSE)
    if ( (u16AdcValue < C_MIN_VDDA) || (u16AdcValue > C_MAX_VDDA) )
    {
        g_e8ErrorElectric = (uint8_t)(C_ERR_SUP_VDDA & ~C_ERR_PERMANENT);
#if (_SUPPORT_LOG_ERRORS != FALSE)
        if (u16AdcValue < C_MIN_VDDA)
        {
            SetLastError(C_ERR_VDDA | C_ERR_EXTW | 0x0C00U);
        }
        else
        {
            SetLastError(C_ERR_VDDA | C_ERR_EXTW | 0x0D00U);
        }
        SetLastError(u16AdcValue);
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
    }
#endif /* (_SUPPORT_VDDA_5V == FALSE) && (_SUPPORT_SENSOR_VDDA_5V == FALSE) */

    /* VDDD check */
#if (_SUPPORT_VDDA_VDDD_LPF == FALSE)
    u16AdcValue = Get_AdcVddd();
#else  /* (_SUPPORT_VDDA_VDDD_LPF == FALSE) */
    u16AdcValue = l_u16AdcVddd;
#endif /* (_SUPPORT_VDDA_VDDD_LPF == FALSE) */
    if ( (u16AdcValue < C_MIN_VDDD) || (u16AdcValue > C_MAX_VDDD) )
    {
        g_e8ErrorElectric = (uint8_t)(C_ERR_SUP_VDDD & ~C_ERR_PERMANENT);
#if (_SUPPORT_LOG_ERRORS != FALSE)
        if (u16AdcValue < C_MIN_VDDD)
        {
            SetLastError(C_ERR_VDDD | C_ERR_EXTW | 0x0C00U);
        }
        else
        {
            SetLastError(C_ERR_VDDD | C_ERR_EXTW | 0x0D00U);
        }
        SetLastError(u16AdcValue);
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
    }

#if (_SUPPORT_ADC_BGD != FALSE)
    /* VBGD check (MMP220307-1) */
    u16AdcValue = Get_AdcVbgd();
    if ( (u16AdcValue < C_MIN_VBGD) || (u16AdcValue > C_MAX_VBGD) )
    {
        g_e8ErrorElectric = (uint8_t)(C_ERR_SUP_VBGD & ~C_ERR_PERMANENT);
#if (_SUPPORT_LOG_ERRORS != FALSE)
        if (u16AdcValue < C_MIN_VBGD)
        {
            SetLastError(C_ERR_VBGD | C_ERR_EXTW | 0x0C00U);
        }
        else
        {
            SetLastError(C_ERR_VBGD | C_ERR_EXTW | 0x0D00U);
        }
        SetLastError(u16AdcValue);
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
    }
#endif /* (_SUPPORT_ADC_BGD != FALSE) */
} /* End of AppInternalSupplyCheck() */
#endif /* (_SUPPORT_ADC_VDDA_VDDD != FALSE) && (_SUPPORT_TRIAXIS_MLX9038x == FALSE) */

/*!*************************************************************************** *
 * AppSupplyCheck()
 * \brief   Application Supply check
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details Measure/check motor driver current
 * *************************************************************************** *
 * - Call Hierarchy: AppDegradedCheck()
 * - Cyclomatic Complexity: 11+1
 * - Nesting: 5
 * - Function calling: 3 (ADC_Conv_Vmotor(), ADC_Conv_Vsupply(),
 *                        SetLastError())
 *           Optional: 1 (AppInternalSupplyCheck())
 * *************************************************************************** */
static void AppSupplyCheck(void)
{
    /* Calculate Voltage (100LSB/V) [10mV] */
#if (_SUPPORT_DEGRADED_MODE != FALSE)
    static uint8_t e8ErrorDebounceFilter = 0U;
    uint16_t u16MotorVoltage = ADC_Conv_Vmotor();
#else  /* (_SUPPORT_DEGRADED_MODE != FALSE) */
    (void)ADC_Conv_Vmotor();
#endif /* (_SUPPORT_DEGRADED_MODE != FALSE) */
    (void)ADC_Conv_Vsupply();
#if (_SUPPORT_DEGRADED_MODE != FALSE)
    if ( (u16MotorVoltage < ((NV_APPL_UVOLT - C_VOLTAGE_HYS) - l_u16ReversePolarityVdrop)) &&
         (g_e8ErrorVoltage != (uint8_t)C_ERR_VOLTAGE_UNDER) )
    {
        /* First time application under-voltage error */
        if ( (e8ErrorDebounceFilter & (uint8_t)C_DEBFLT_ERR_UV) == (uint8_t)0x00U)
        {
            /* Need twice a under-voltage detection, to avoid ESD-pulses disturbance will cause degraded mode entering */
            e8ErrorDebounceFilter |= (uint8_t)C_DEBFLT_ERR_UV;
            l_u16UnderVoltageFilterCount = C_UV_FILTER_COUNT;
        }
        else if (l_u16UnderVoltageFilterCount == 0U)
        {
            g_e8ErrorVoltage = (uint8_t)C_ERR_VOLTAGE_UNDER;
            g_e8ErrorVoltageComm = g_e8ErrorVoltage;
#if (_SUPPORT_LOG_ERRORS != FALSE)
            SetLastError(C_ERR_APPL_UNDER_VOLT);
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
        }
        else
        {
            /* Nothing */
        }
#if (LINPROT == LIN2X_HVAC52)
        l_u16DegradeDelay = 0xFFFFU;                                            /* Disable degrade delay timer */
#endif /* (LINPROT == LIN2X_HVAC52) */
    }
    else if ( (u16MotorVoltage > ((NV_APPL_OVOLT + C_VOLTAGE_HYS) - l_u16ReversePolarityVdrop)) &&
              (g_e8ErrorVoltage != (uint8_t)C_ERR_VOLTAGE_OVER) )
    {
        /* First time application over-voltage error */
        if ( (e8ErrorDebounceFilter & C_DEBFLT_ERR_OV) == (uint8_t)0x00U)
        {
            /* Need twice a over-voltage detection, to avoid ESD-pulses disturbance will cause degraded mode entering */
            e8ErrorDebounceFilter |= (uint8_t)C_DEBFLT_ERR_OV;
            l_u16OverVoltageFilterCount = C_OV_FILTER_COUNT;
        }
        else if (l_u16OverVoltageFilterCount == 0U)
        {
            g_e8ErrorVoltage = (uint8_t)C_ERR_VOLTAGE_OVER;
            g_e8ErrorVoltageComm = g_e8ErrorVoltage;
#if (_SUPPORT_LOG_ERRORS != FALSE)
            SetLastError(C_ERR_APPL_OVER_VOLT);
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
        }
        else
        {
            /* Nothing */
        }
#if (LINPROT == LIN2X_HVAC52)
        l_u16DegradeDelay = 0xFFFFU;                                            /* Disable degrade delay timer */
#endif /* (LINPROT == LIN2X_HVAC52) */
    }
    else if ( (u16MotorVoltage >= ((NV_APPL_UVOLT + C_VOLTAGE_HYS) - l_u16ReversePolarityVdrop)) &&
              (u16MotorVoltage <= ((NV_APPL_OVOLT - C_VOLTAGE_HYS) - l_u16ReversePolarityVdrop)) )
    {
#if (LINPROT == LIN2X_HVAC52)                                                   /* MMP200729-1: Add Exit Degraded-mode delay (LH V5.2 - 9.6.3.4) */
        if (g_e8ErrorVoltage != (uint8_t)C_ERR_VOLTAGE_IN_RANGE)
        {
            if (l_u16DegradeDelay == 0xFFFFU)
            {
                /* Degrade delay timer is disabled */
                if (g_e8DegradedMotorRequest != (uint8_t)C_MOTOR_REQUEST_NONE)
                {
                    /* Degrade Request is pending; Set Degrade delay time. See 9.6.3.4 of LH V5.2 */
                    l_u16DegradeDelay = (100U + ((g_u8NAD & 0x1FU) * 10U)) * PI_TICKS_PER_MILLISECOND;
                }
                else
                {
                    g_e8ErrorVoltage = (uint8_t)C_ERR_VOLTAGE_IN_RANGE;
                }
            }
            else if (l_u16DegradeDelay == 0x0000U)
            {
                /* Degrade delay time-out */
                l_u16DegradeDelay = 0xFFFFU;
                g_e8ErrorVoltage = (uint8_t)C_ERR_VOLTAGE_IN_RANGE;
            }
            else
            {
                /* Nothing */
            }
        }
#else  /* (LINPROT == LIN2X_HVAC52) */
        /* Remove under/over voltage errors */
        g_e8ErrorVoltage = (uint8_t)C_ERR_VOLTAGE_IN_RANGE;
#endif /* (LINPROT == LIN2X_HVAC52) */
        /* Reset filter counters and reset start of counting */
        e8ErrorDebounceFilter &= (uint8_t) ~(C_DEBFLT_ERR_UV | C_DEBFLT_ERR_OV);
        l_u16UnderVoltageFilterCount = 0U;
        l_u16OverVoltageFilterCount = 0U;
    }
    else
    {
        /* Reset filter counters and error flags (MMP170406-6) */
        e8ErrorDebounceFilter &= (uint8_t) ~(C_DEBFLT_ERR_UV | C_DEBFLT_ERR_OV);
        l_u16UnderVoltageFilterCount = 0U;
        l_u16OverVoltageFilterCount = 0U;
    }

#if FALSE && (LIN_COMM != FALSE) && (_SUPPORT_LIN_UV != FALSE)
    /* LIN Under-voltage with +/- 0.5V hysteric (RQ003_03c.1/2) */
    if (NV_LIN_UV != 0U)
    {
        /* Check above/equal (6.0V + n * 0.5V + 0.5V) to stop LIN UV check */
        if (u16MotorVoltage >= (650U + (NV_LIN_UV * 50U)) )
        {
            l_u16LinUVTimeCounter = 0U;                                         /* Stop LIN UV time-counter */
        }
        else if (l_u16LinUVTimeCounter > PI_TICKS_PER_SECOND)
        {
            /* Restart MLX4 Bus-timeout, every second during LIN UV */
            (void)ml_Disconnect();
            (void)ml_Connect();
            l_u16LinUVTimeCounter = 1U;                                         /* Re-start LIN UV time-counter */
        }
        /* Check above/equal (6.0V + n * 0.5V - 0.5V) to start LIN UV check */
        else if ( (l_u16LinUVTimeCounter == 0U) && (u16MotorVoltage <= (550U + (NV_LIN_UV * 50U))) )
        {
            l_u16LinUVTimeCounter = 1U;                                         /* Start LIN UV time-counter */
        }
        else
        {
            /* Nothing */
        }
    }
#endif /* (LIN_COMM != FALSE) && (_SUPPORT_LIN_UV != FALSE) */
#endif /* (_SUPPORT_DEGRADED_MODE != FALSE) */

#if (_SUPPORT_ADC_VDDA_VDDD != FALSE) && (_SUPPORT_TRIAXIS_MLX9038x == FALSE)
    AppInternalSupplyCheck();
#endif /* (_SUPPORT_ADC_VDDA_VDDD != FALSE) && (_SUPPORT_TRIAXIS_MLX9038x == FALSE) */
} /* End of AppSupplyCheck() */

/*!*************************************************************************** *
 * AppTemperatureCheck()
 * \brief   Application Temperature check
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details Measure/check ambient/junction temperature
 * *************************************************************************** *
 * - Call Hierarchy: AppDegradedCheck()
 * - Cyclomatic Complexity: 6+1
 * - Nesting: 5
 * - Function calling: 2 (ADC_Conv_TempJ(), SetLastError())
 * *************************************************************************** */
static void AppTemperatureCheck(void)
{
    /* Calculate Chip internal temperature (1LSB/C) [C] */
#if (_SUPPORT_DEGRADED_MODE != FALSE)
    static uint8_t e8ErrorDebounceFilter = 0U;
    int16_t i16ChipTemperature = ADC_Conv_TempJ(FALSE);
#else  /* (_SUPPORT_DEGRADED_MODE != FALSE) */
    (void)ADC_Conv_TempJ(FALSE);
#endif /* (_SUPPORT_DEGRADED_MODE != FALSE) */

    if ( (g_e8ErrorOverTemperature & C_ERR_OTEMP_ERROR) == 0U)
    {
#if (_SUPPORT_AMBIENT_TEMP != FALSE)
        SelfHeatCompensation();
#endif /* (_SUPPORT_AMBIENT_TEMP != FALSE) */

#if (_SUPPORT_DEGRADED_MODE != FALSE)
        if ( (((l_i16AmbientTemperature > (int16_t)(NV_APPL_OTEMP + C_TEMPERATURE_HYS)) &&  /* MMP200617-1 */
#if (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT)
               ((g_e8MotorStatus & C_MOTOR_STATUS_STOP_MASK) == C_MOTOR_STATUS_STOP)) ||
#elif (_SUPPORT_APP_TYPE == C_APP_RELAY)
#if (_SUPPORT_DUAL_RELAY == FALSE)
               ((g_e8RelayStatus & (uint8_t)C_RELAY_STATUS_MASK) == (uint8_t)C_RELAY_STATUS_OFF)) ||
#else  /* (_SUPPORT_DUAL_RELAY == FALSE) */
               (((g_e8RelayStatusA & (uint8_t)C_RELAY_STATUS_MASK) == (uint8_t)C_RELAY_STATUS_OFF) &&
                ((g_e8RelayStatusB & (uint8_t)C_RELAY_STATUS_MASK) == (uint8_t)C_RELAY_STATUS_OFF))) ||
#endif /* (_SUPPORT_DUAL_RELAY == FALSE) */
#elif (_SUPPORT_APP_TYPE == C_APP_SOLENOID)
               ((g_e8SolenoidStatus & C_SOLENOID_STATUS_MASK) == C_SOLENOID_STATUS_DEACTIVATED)) ||
#endif /* (_SUPPORT_APP_TYPE) */
#if (_SUPPORT_DIAG_OT != FALSE)
              (i16ChipTemperature > (int16_t)(C_CHIP_OVERTEMP_LEVEL + C_TEMPERATURE_HYS))) &&
#else  /* (_SUPPORT_DIAG_OT != FALSE) */
              (i16ChipTemperature > (int16_t)(NV_APPL_OTEMP + C_TEMPERATURE_HYS))) &&
#endif /* (_SUPPORT_DIAG_OT != FALSE) */
             (g_e8ErrorOverTemperature != (uint8_t)C_ERR_OTEMP_SHUTDOWN) )
        {
            if ( (e8ErrorDebounceFilter & (uint8_t)C_DEBFLT_ERR_OVT) == (uint8_t)0x00U)
            {
                /* Need twice a over-temperature detection, to avoid ESD-pulses disturbance will cause degraded mode entering */
                e8ErrorDebounceFilter |= (uint8_t)C_DEBFLT_ERR_OVT;
            }
            else
            {
                g_e8ErrorOverTemperature = (uint8_t)C_ERR_OTEMP_SHUTDOWN;
#if (_SUPPORT_MOTOR_POSITION == C_MOTOR_POS_ROTOR)
                g_u16TargetPosition = g_u16ActualPosition;                      /* If over-temperature, then FPOS = CPOS (9.5.3.2) */
#endif /* (_SUPPORT_MOTOR_POSITION == C_MOTOR_POS_ROTOR) */
#if (_SUPPORT_OT_PERMANENT_ERROR != FALSE)
                if ( (g_e8ErrorElectric & (uint8_t)(C_ERR_PERMANENT | C_ERR_RPT_OVER_TEMPERATURE) == 0U)
                {
                    g_u8OverTemperatureCount++;
                    if (g_u8OverTemperatureCount >= (uint8)C_OVERTEMP_TO_PERMDEFECT_THRSHLD)
                    {
                        /* Turn off motor-driver; Permanent Electric Defect - Repeated Over Temperature */
                        MotorDriverPermanentError( (uint8_t)(C_ERR_PERMANENT | C_ERR_RPT_OVER_TEMPERATURE));
                    }
                }
#endif /* _SUPPORT_OT_PERMANENT_ERROR != FALSE) */
#if (_SUPPORT_LOG_ERRORS != FALSE)
                SetLastError(C_ERR_APPL_OVER_TEMP);
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
            }
        }
        else if (l_i16AmbientTemperature < (NV_APPL_OTEMP - C_TEMPERATURE_HYS) )
        {
            g_e8ErrorOverTemperature = (uint8_t)C_ERR_OTEMP_NO;
            e8ErrorDebounceFilter &= (uint8_t) ~C_DEBFLT_ERR_OVT;
        }
        else
        {
            /* Nothing */
        }
#endif /* (_SUPPORT_DEGRADED_MODE != FALSE) */
    }
} /* End of AppTemperatureCheck() */

/*!*************************************************************************** *
 * AppDegradedCheck()
 * \brief   Application degraded check
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details Based on supply-level and temperature, application mode may changes
 *          to Degraded and Normal mode
 * *************************************************************************** *
 * - Call Hierarchy: main_Init()
 * - Cyclomatic Complexity: 6+1
 * - Nesting: 4
 * - Function calling: 4 (AppCurrentCheck(), AppSupplyCheck(), AppTemperatureCheck(),
 *                        MotorDriverStop())
 * *************************************************************************** */
void AppDegradedCheck(void)
{
    /* ******************************* */
    /* *** d. Motor Driver current *** */
    /* ******************************* */
    AppCurrentCheck();

    /* ************************************************************** */
    /* *** e. Chip and Motor Driver voltage (degraded-mode check) *** */
    /* ************************************************************** */
    AppSupplyCheck();

    /* ************************************************************* */
    /* *** f. Chip and ambient temperature (degraded-mode check) *** */
    /* ************************************************************* */
    AppTemperatureCheck();

#if (_SUPPORT_DEGRADED_MODE != FALSE)
    /* ****************************** */
    /* *** g. Degraded-mode check *** */
    /* ****************************** */
    if ( ((g_e8ErrorVoltage != (uint8_t)C_ERR_VOLTAGE_IN_RANGE) ||
          (g_e8ErrorOverTemperature != (uint8_t)C_ERR_OTEMP_NO)) &&
         (g_e8DegradeStatus == FALSE) )
    {
        /* Not in degradation state; Stop motor, remember last "request" and enter degradation state */
        if (g_e8MotorRequest != C_MOTOR_REQUEST_NONE)
        {
            g_e8DegradedMotorRequest = g_e8MotorRequest;
#if (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT)
            MotorDriverStop( (uint16_t)C_STOP_RAMPDOWN);                        /* Degraded-mode (Running) */
#elif (_SUPPORT_APP_TYPE == C_APP_RELAY)
            RelayDriverOff();                                                   /* Degraded-mode (Running) */
#elif (_SUPPORT_APP_TYPE == C_APP_SOLENOID)
            SolenoidDriverDeactivate();
#endif /* (_SUPPORT_APP_TYPE) */
        }
#if (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT)
        else if ( (g_e8MotorStatus & C_MOTOR_STATUS_STOP_MASK) != C_MOTOR_STATUS_STOP)
#elif (_SUPPORT_APP_TYPE == C_APP_RELAY)
#if (_SUPPORT_DUAL_RELAY == FALSE)
        else if ( (g_e8RelayStatus & (uint8_t)C_RELAY_STATUS_MASK) == (uint8_t)C_RELAY_STATUS_OFF)
#else  /* (_SUPPORT_DUAL_RELAY == FALSE) */
        else if ( ((g_e8RelayStatusA & (uint8_t)C_RELAY_STATUS_MASK) == (uint8_t)C_RELAY_STATUS_OFF) &&
                  ((g_e8RelayStatusB & (uint8_t)C_RELAY_STATUS_MASK) == (uint8_t)C_RELAY_STATUS_OFF) )
#endif /* (_SUPPORT_DUAL_RELAY == FALSE) */
#elif (_SUPPORT_APP_TYPE == C_APP_SOLENOID)
        else if ( (g_e8SolenoidStatus & C_SOLENOID_STATUS_MASK) != C_SOLENOID_STATUS_DEACTIVATED)
#endif /* (_SUPPORT_APP_TYPE) */
        {
#if (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT)
            g_e8DegradedMotorRequest = (uint8_t)C_MOTOR_REQUEST_START;
            MotorDriverStop( (uint16_t)C_STOP_RAMPDOWN);                        /* Degraded-mode (Running) */
#elif (_SUPPORT_APP_TYPE == C_APP_RELAY)
            g_e8DegradedMotorRequest = (uint8_t)C_RELAYS_REQUEST_OFF;
            RelayDriverOff();                                                   /* Degraded-mode (Running) */
#elif (_SUPPORT_APP_TYPE == C_APP_SOLENOID)
            g_e8DegradedMotorRequest = (uint8_t)C_SOLENOID_REQUEST_DEACTIVATE;
            SolenoidDriverDeactivate();
#endif /* (_SUPPORT_APP_TYPE) */
        }
        else if (g_e8DegradedMotorRequest == (uint8_t)C_MOTOR_REQUEST_NONE)
        {
#if (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT)
            g_e8DegradedMotorRequest = (uint8_t)C_MOTOR_REQUEST_STOP;
            MotorDriverStop( (uint16_t)C_STOP_EMERGENCY);                       /* Degraded-mode */
#elif (_SUPPORT_APP_TYPE == C_APP_RELAY)
            g_e8DegradedMotorRequest = (uint8_t)C_RELAYS_REQUEST_OFF;
            RelayDriverOff();                                                   /* Degraded-mode (Running) */
#elif (_SUPPORT_APP_TYPE == C_APP_SOLENOID)
            g_e8DegradedMotorRequest = (uint8_t)C_SOLENOID_REQUEST_DEACTIVATE;
            SolenoidDriverDeactivate();
#endif /* (_SUPPORT_APP_TYPE) */
        }
        else
        {
            /* Nothing */
        }
        g_e8DegradeStatus = TRUE;
    }
    else if ( (g_e8DegradeStatus != FALSE) &&
              (g_e8ErrorVoltage == (uint8_t)C_ERR_VOLTAGE_IN_RANGE) &&
              (g_e8ErrorOverTemperature == (uint8_t)C_ERR_OTEMP_NO) )
    {
        /* No longer degraded mode */
        if (g_e8DegradedMotorRequest != (uint8_t)C_MOTOR_REQUEST_NONE)
        {
            g_e8MotorRequest = (uint8_t)g_e8DegradedMotorRequest;
            g_e8DegradedMotorRequest = (uint8_t)C_MOTOR_REQUEST_NONE;
        }
        g_e8DegradeStatus = FALSE;
    }
    else
    {
        /* Nothing */
    }
#endif /* (_SUPPORT_DEGRADED_MODE != FALSE) */

#if (LIN_COMM != FALSE) && (_SUPPORT_LIN_UV != FALSE)
    if (ADC_Conv_Vsupply() > (C_VS_LIN_MIN + C_VS_LIN_HYST) )
    {
        if (l_u8Mlx4Connected == FALSE)
        {
            /* Re-connect LIN above C_VS_LIN_MIN as LIN operate within of LIN Specification */
            ENTER_SECTION(ATOMIC_SYSTEM_MODE);  /*lint !e534 */
            if (ml_Connect() == ML_SUCCESS)
            {
                l_u8Mlx4Connected = TRUE;
            }
            EXIT_SECTION(); /*lint !e438 */
        }
    }
    else if (ADC_Conv_Vsupply() < (C_VS_LIN_MIN - C_VS_LIN_HYST) )
    {
        if (l_u8Mlx4Connected != FALSE)
        {
            /* Disconnect LIN below C_VS_LIN_MIN as LIN operate out of LIN Specification */
            ENTER_SECTION(ATOMIC_SYSTEM_MODE);  /*lint !e534 */
            if (ml_Disconnect() == ML_SUCCESS)
            {
                l_u8Mlx4Connected = FALSE;
            }
            EXIT_SECTION(); /*lint !e438 */
        }
    }
    else
    {
        /* Nothing */
    }
#endif /* (LIN_COMM != FALSE) && (_SUPPORT_LIN_UV != FALSE) */
} /* End of AppDegradedCheck() */

#if (LIN_COMM != FALSE) && (_SUPPORT_LIN_BUS_ACTIVITY_CHECK != FALSE)
/*!*************************************************************************** *
 * AppCheckLinProc()
 * \brief   LIN Communication CPU check
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details Check LIN co-processor (MLX4) state. In case of time-out (ml_stINVALID),
 *          the LIN co-processor is either busy or hanging; In case this occurs
 *          often (more than C_MLX4_STATE_ERROR_THRSHLD), restart MLX4 and
 *          re-initialise the LIN.
 *          In case of LIN BUFFER not free (not empty), wait...
 *          In case of LIN Command overflow, MLX4 and MLX16FX lost synchronisation.
 *          Release the MLX4
 *          In case of DISCONNECTION, re-connect in case LIN communication was used.
 *          Check LIN Baudrate to be in expected range of supported LIN Baudrates.
 *          This is: 9600, 10417 and 19200 all with +/- 10% tolerance
 * *************************************************************************** *
 * - Call Hierarchy: AppBackgroundTaskHandler()
 * - Cyclomatic Complexity: 3+1
 * - Nesting: 2
 * - Function calling: 5 (ml_GetState(), ml_ResetDrv(), ml_StartDrv(),
 *                        SetLastError(), LIN_Init())
 * *************************************************************************** */
static void AppCheckLinProc(void)
{
    static uint8_t l_u8LinBufferFull = 0U;

#if (_SUPPORT_LIN_AA != FALSE)
    if ( (l_u16Mlx4CheckPeriodCount > C_MLX4_STATE_TIMEOUT) &&
         (g_u8LinAAMode == (uint8_t)C_SNPD_SUBFUNC_INACTIVE) )
#else  /* (_SUPPORT_LIN_AA != FALSE) */
    if (l_u16Mlx4CheckPeriodCount > C_MLX4_STATE_TIMEOUT)
#endif /* (_SUPPORT_LIN_AA != FALSE) */
    {
        ml_LinState_t eLinState;
        ENTER_SECTION(ATOMIC_KEEP_MODE);  /*lint !e534 */
        eLinState = ml_GetState(ML_CLR_LIN_BUS_ACTIVITY);
        EXIT_SECTION(); /*lint !e438 */
        /* Only check MLX4 response */
        if (eLinState != ml_stINVALID)
        {
            /* MLX4 has detected a SYNC field */
            l_u8Mlx4ErrorState &= (uint8_t) ~C_MLX4_STATE_IMMEDIATE_RST;        /* Reset/clear the MLX Error state counter */
            if ( (ML_DATA_LIN_STATUS & ML_LIN_BUFFER_NOT_FREE) != 0U)           /* MMP210411-1: Check MLX4 buffer not-free */
            {
                /* LIN buffer not free */
                l_u8LinBufferFull++;
                if (l_u8LinBufferFull >= 2U)
                {
/*                    l_u8Mlx4ErrorState = C_MLX4_STATE_IMMEDIATE_RST; */
#if (_SUPPORT_LOG_ERRORS != FALSE)
                    SetLastError(C_ERR_LIN_BUF_NOT_FREE);
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
                    l_u8LinBufferFull = 0U;
                }
            }
            else if ( (ML_DATA_LIN_STATUS & ML_LIN_CMD_OVERFLOW) != 0U)         /* MMP210411-3: Check MLX4 Command Overflow */
            {
                /* LIN Command Overflow */
                ENTER_SECTION(ATOMIC_KEEP_MODE);  /*lint !e534 */
                (void)ml_GetState(ML_CLR_LIN_CMD_OVERFLOW);
                EXIT_SECTION(); /*lint !e438 */
                ml_SetSLVCMD(0x42U);                                            /* Do the handshake and let the LIN Module go */
/*                l_u8Mlx4ErrorState = C_MLX4_STATE_IMMEDIATE_RST; */
#if (_SUPPORT_LOG_ERRORS != FALSE)
                SetLastError(C_ERR_LIN_CMD_OVF);
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
                l_u8LinBufferFull = 0U;
            }
#if ((_SUPPORT_LIN_UV != FALSE) || (_SUPPORT_CPU_HALT != FALSE))
            else if ( (l_u8Mlx4Connected != FALSE) && (eLinState == ml_stDISCONNECTED) )
            {
                (void)ml_Connect();
            }
#endif /* ((_SUPPORT_LIN_UV != FALSE) || (_SUPPORT_CPU_HALT != FALSE)) */
            else
            {
                /* uint16_t u16LinBaudRate = ml_GetBaudRate(MLX4_FPLL); */      /* MMP210411-2: Add Baudrate check */
                uint16_t u16LinBaudRate = p_ml_GetBaudRate(MLX4_FPLL);          /* MMP220913-1: Fixed function */
                if ( (u16LinBaudRate != 0U) &&                                  /* No error */
                     ((u16LinBaudRate > C_LIN_19200_MAX) ||
                      ((u16LinBaudRate < C_LIN_19200_MIN) && (u16LinBaudRate > C_LIN_10417_MAX)) ||
                      (u16LinBaudRate < C_LIN_9600_MIN)) )
                {
                    (void)ml_SetAutoBaudRateMode(ML_ABR_ON_FIRST_FRAME);
                    /* l_u8Mlx4ErrorState = (uint8_t)C_MLX4_STATE_IMMEDIATE_RST; */
#if (_SUPPORT_LOG_ERRORS != FALSE)
                    SetLastError(C_ERR_LIN_BAUDRATE | C_ERR_EXTW);
                    SetLastError(u16LinBaudRate);
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
                }
                l_u8LinBufferFull = 0U;
            }
        }
        else
        {
            /* MLX4 response Time-out */
            l_u8Mlx4ErrorState++;                                               /* MLX4 Error state increased */
#if (_SUPPORT_LOG_ERRORS != FALSE)
            SetLastError(C_WRN_LIN_TIMEOUT | C_ERR_EXT | ((l_u8Mlx4ErrorState & 0x0FU) << 8));
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
        }

        if (l_u16MLX4_RAM_Static_CRC != p_CalcCRC_U16((const uint16_t *)C_RAM_MLX4_STATIC_BGN_ADDR,
                                                      (C_RAM_MLX4_STATIC_END_ADDR - C_RAM_MLX4_STATIC_BGN_ADDR)/sizeof(uint16_t)) )
        {
            /* MLX4 Static RAM area corrupted! Restart IC (initialised by: fw_ram_section_init() */
#if (_SUPPORT_LOG_ERRORS != FALSE)
            SetLastError(C_ERR_LIN_RAM_STATIC);
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
            (void)mlu_ApplicationStop();
            MLX16_RESET_COLD();
        }
        if (g_u16MLX4_RAM_Dynamic_CRC1 != p_CalcCRC_U16((const uint16_t *)C_RAM_MLX4_DYNAMIC_BGN_ADR1,
                                                        (C_RAM_MLX4_DYNAMIC_END_ADR1 - C_RAM_MLX4_DYNAMIC_BGN_ADR1)/sizeof(uint16_t)) )
        {
            /* MLX4 Dynamic RAM area #1 corrupted! Restart MLX4 and LIN_Init() */
#if (_SUPPORT_LOG_ERRORS != FALSE)
            SetLastError(C_ERR_LIN_RAM_DYNAMIC | C_ERR_EXT | 0x0100U);
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
            l_u8Mlx4ErrorState = (uint8_t)C_MLX4_STATE_IMMEDIATE_RST;
        }
        if (g_u16MLX4_RAM_Dynamic_CRC2 != p_CalcCRC_U16((const uint16_t *)C_RAM_MLX4_DYNAMIC_BGN_ADR2,
                                                        (C_RAM_MLX4_DYNAMIC_END_ADR2 - C_RAM_MLX4_DYNAMIC_BGN_ADR2)/sizeof(uint16_t)) )
        {
            /* MLX4 Dynamic RAM area #2 corrupted! Restart MLX4 and LIN_Init() */
#if (_SUPPORT_LOG_ERRORS != FALSE)
            SetLastError(C_ERR_LIN_RAM_DYNAMIC | C_ERR_EXT | 0x0200U);
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
            l_u8Mlx4ErrorState = (uint8_t)C_MLX4_STATE_IMMEDIATE_RST;
        }
        {
            /* MMP221207-2: Check MLX4 Private Auto Baudrate */
            uint16_t u16LinBaudRate = p_ml_GetLastBaudRate(MLX4_FPLL);
            uint16_t u16LinAutoBaudRate = p_ml_GetAutoBaudRate(MLX4_FPLL);
            if ( (u16LinAutoBaudRate != 0U) &&                                  /* No error */
                 ((u16LinAutoBaudRate > C_LIN_19200_MAX) ||
                  ((u16LinAutoBaudRate < C_LIN_19200_MIN) && (u16LinAutoBaudRate > C_LIN_10417_MAX)) ||
                  (u16LinAutoBaudRate < C_LIN_9600_MIN)) )
            {
                /* Auto Baudrate not as expected; Reset IC */
                (void)mlu_ApplicationStop();
#if (_SUPPORT_LOG_ERRORS != FALSE)
                SetLastError(C_ERR_LIN_RAM_DYNAMIC | C_ERR_EXT | 0x0300U);
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
                l_u8Mlx4ErrorState = (uint8_t)C_MLX4_STATE_IMMEDIATE_RST;
            }

            /* Check MLX4 Private Last Baudrate */
            if ( (u16LinBaudRate != 0U) &&                                      /* No error */
                 ((u16LinBaudRate > (u16LinAutoBaudRate + (u16LinAutoBaudRate >> 3))) ||  /* Above Auto-baudrate + 12.5% */
                  (u16LinBaudRate < (u16LinAutoBaudRate - (u16LinAutoBaudRate >> 3)))) )  /* Below Auto-baudrate - 12.5% */
            {
                /* Last Baudrate not as expected; Reset IC */
                (void)mlu_ApplicationStop();
#if (_SUPPORT_LOG_ERRORS != FALSE)
                SetLastError(C_ERR_LIN_RAM_DYNAMIC | C_ERR_EXT | 0x0400U);
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
                l_u8Mlx4ErrorState = (uint8_t)C_MLX4_STATE_IMMEDIATE_RST;
            }
        }
        l_u16Mlx4CheckPeriodCount = 0U;
    }

    if ( (l_u8Mlx4ErrorState >= (uint8_t)C_MLX4_STATE_ERROR_THRSHLD)
#if (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT)
         && ((g_e8MotorStatus & (uint8_t)C_MOTOR_STATUS_APPL_STOP) == 0x00U)
#elif (_SUPPORT_APP_TYPE == C_APP_RELAY)
#if (_SUPPORT_DUAL_RELAY == FALSE)
         && ((g_e8RelayStatus & (uint8_t)C_MOTOR_STATUS_APPL_STOP) == 0x00U)
#else  /* (_SUPPORT_DUAL_RELAY == FALSE) */
         && ((g_e8RelayStatusA & (uint8_t)C_MOTOR_STATUS_APPL_STOP) == 0x00U)
         && ((g_e8RelayStatusB & (uint8_t)C_MOTOR_STATUS_APPL_STOP) == 0x00U)
#endif /* (_SUPPORT_DUAL_RELAY == FALSE) */
#elif (_SUPPORT_APP_TYPE == C_APP_SOLENOID)
         && ((g_e8SolenoidStatus & (uint8_t)C_MOTOR_STATUS_APPL_STOP) == 0x00U)
#endif /* (_SUPPORT_APP_TYPE) */
#if (_SUPPORT_LIN_SLEEP != FALSE)
         && (g_e8MotorRequest != (uint8_t)C_MOTOR_REQUEST_SLEEP)
#endif /* (_SUPPORT_LIN_SLEEP != FALSE) */
         )
    {
        /* Didn't receive MLX4 LIN command and/or data-request in the last period, or need immediate reset */
        /* Signal Error; Reset MLX4 */
#if (_SUPPORT_APP_USER_MODE != FALSE)
        ENTER_SECTION(SYSTEM_MODE); /*lint !e534 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
        ml_ResetDrv();
#if (_SUPPORT_APP_USER_MODE != FALSE)
        EXIT_SECTION(); /*lint !e438 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
        NOP();
        NOP();
        NOP();
        ml_StartDrv();
#if (_SUPPORT_LOG_ERRORS != FALSE)
        SetLastError(C_ERR_MLX4_RESTART);
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
        LIN_Init();                                                             /* Re-initialise LIN interface */
        l_u8Mlx4ErrorState = 0U;
    }
} /* End of AppCheckLinProc() */
#endif /* (LIN_COMM != FALSE) && (_SUPPORT_LIN_BUS_ACTIVITY_CHECK != FALSE) */

/*!*************************************************************************** *
 * AppPeriodicTimerEvent
 * \brief   Perform Application support Functions Periodic Timer Event updates.
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16Period: Time-period in core-timer units [500us]
 * \return  -
 * *************************************************************************** *
 * \details This function reset flags before each actuator operation
 * *************************************************************************** *
 * - Call Hierarchy: main_PeriodicTimerEvent()
 * - Cyclomatic Complexity: 2+1
 * - Nesting: 1
 * - Function calling: -
 * *************************************************************************** */
void AppPeriodicTimerEvent(uint16_t u16Period)
{
#if (_SUPPORT_DEGRADED_MODE != FALSE)
    /* Supply Under- & Over-voltage filter period counters (MMP170221-1) */
    if (l_u16UnderVoltageFilterCount > u16Period)
    {
        l_u16UnderVoltageFilterCount -= u16Period;
    }
    else
    {
        l_u16UnderVoltageFilterCount = 0U;
    }
    if (l_u16OverVoltageFilterCount > u16Period)
    {
        l_u16OverVoltageFilterCount -= u16Period;
    }
    else
    {
        l_u16OverVoltageFilterCount = 0U;
    }
#endif /* (_SUPPORT_DEGRADED_MODE != FALSE) */

#if (LIN_COMM != FALSE) && (_SUPPORT_LIN_UV != FALSE)
    if (l_u16LinUVTimeCounter != 0U)
    {
        l_u16LinUVTimeCounter += u16Period;
    }
#endif /* (LIN_COMM != FALSE) && (_SUPPORT_LIN_UV != FALSE) */

#if (LIN_COMM != FALSE) && (_SUPPORT_LIN_BUS_ACTIVITY_CHECK != FALSE)
    l_u16Mlx4CheckPeriodCount += u16Period;
#endif /* (LIN_COMM != FALSE) && (_SUPPORT_LIN_BUS_ACTIVITY_CHECK != FALSE) */

#if (LINPROT == LIN2X_HVAC52)
    if ( (l_u16DegradeDelay != 0U) && (l_u16DegradeDelay != 0xFFFFU) )
    {
        if (l_u16DegradeDelay > u16Period)
        {
            l_u16DegradeDelay -= u16Period;
        }
        else
        {
            l_u16DegradeDelay = 0U;
        }
    }
#endif /* (LINPROT == LIN2X_HVAC52) */

#if (_SUPPORT_ADC_VDDA_VDDD != FALSE) && (_SUPPORT_VDDA_VDDD_LPF != FALSE) && (_SUPPORT_TRIAXIS_MLX9038x == FALSE)
    l_u16AdcVdda = p_LpfU16_I16byI16(&l_u32LpfAdcVdda, C_VDDA_LPF_COEF, (int16_t)(Get_AdcVdda() - l_u16AdcVdda));
    l_u16AdcVddd = p_LpfU16_I16byI16(&l_u32LpfAdcVddd, C_VDDD_LPF_COEF, (int16_t)(Get_AdcVddd() - l_u16AdcVddd));
#endif /* (_SUPPORT_ADC_VDDA_VDDD != FALSE) && (_SUPPORT_VDDA_VDDD_LPF != FALSE) && (_SUPPORT_TRIAXIS_MLX9038x == FALSE) */

#if (_SUPPORT_STALL_AUTO_CLEAR != FALSE)
    if (g_u16StallPeriod != 0U)
    {
        if (g_u16StallPeriod > u16Period)
        {
            g_u16StallPeriod -= u16Period;
        }
        else
        {
            g_u16StallPeriod = 0U;
            g_u8StallOcc = FALSE;                                               /* Stall is cleared after delay-time; New command is accepted */
#if (PWM_COMM != FALSE)
            PwmInMsgClr();
#endif /* (PWM_COMM != FALSE) */
        }
    }
    else
    {
        /* Nothing */
    }
#endif /* (_SUPPORT_STALL_AUTO_CLEAR != FALSE) */

    (void) u16Period;
} /* End of AppPeriodicTimerEvent() */

#if (_SUPPORT_BG_MEM_TEST != FALSE) && (_SUPPORT_BG_FLASH_TEST != FALSE)
#if (_SUPPORT_IO_STYLE == FALSE)
/*!*************************************************************************** *
 * FlashBist()
 * \brief   Calculate the Flash BIST in blocks of 256 Bytes
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  (uint16_t) u16ValidUpdate: FALSE: Not completed
 *                                     TRUE: Completely checked.
 * *************************************************************************** *
 * \details Flash BIST CRC24 calculation, using chip hardware BIST block
 * Notice:  During the BIST calculation the flash/CPU execution stops!
 *          Don't execute the Flash BIST during high-speed motor-operation
 *          or other time-critical operations (e.g. LIN Auto-Addressing)!
 * Performance: <20us (per 256 Bytes)
 *             (Complete Flash) 2.35 ms
 * *************************************************************************** *
 * - Call Hierarchy: AppMemoryCheck()
 * - Cyclomatic Complexity: 7+1
 * - Nesting: 2
 * - Function calling: 0
 * *************************************************************************** */
static uint16_t FlashBist(void)
{
    static uint16_t u16SegmentAddr = C_FLASH_START_ADDR;
    static uint32_t u32FlashBist = C_ROM_BIST_CFG_SEED;
    uint16_t u16ValidUpdate = FALSE;

    if ( (u16SegmentAddr == C_FLASH_START_ADDR)
#if (_SUPPORT_MLX_CHIP_STATUS != FALSE)
         || (u16SegmentAddr == C_NVM_START_ADDR)
#endif /* (_SUPPORT_MLX_CHIP_STATUS != FALSE) */
         )
    {
        u32FlashBist = C_ROM_BIST_CFG_SEED;                                     /* Initialise the CRC preset with SEED */
    }

    IO_ROM_BIST_CTRL = (IO_ROM_BIST_CTRL & ~(B_ROM_BIST_MASK_SIG_ERR |
                                             B_ROM_BIST_BIST |
                                             M_ROM_BIST_ECC_POSITION)) |
                       (B_ROM_BIST_SINGLE_RAMP | 2U);                           /* ECC position is 0b10 for 64bit FLASH */
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81339__) || defined (__MLX81350__)
    IO_ROM_BIST_ADD_START_L = u16SegmentAddr;
    IO_ROM_BIST_ADD_START_H = 0U;
    u16SegmentAddr += 0x0100U;
#elif defined (__MLX81160__) || defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__) || \
    defined (__MLX81334__)
    IO_ROM_BIST_ADD_START_L = (u16SegmentAddr << 1);
    IO_ROM_BIST_ADD_START_H = (u16SegmentAddr >> 15);
    u16SegmentAddr += (0x0100U >> 1);
#endif
#if (_SUPPORT_MLX_CHIP_STATUS != FALSE)
    if (u16SegmentAddr > C_FLASH_START_ADDR)
#endif /* (_SUPPORT_MLX_CHIP_STATUS != FALSE) */
    {
        if (u16SegmentAddr >= C_FLASH_STOP_ADDR)
        {
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81339__) || defined (__MLX81350__)
            IO_ROM_BIST_ADD_STOP_L = C_FLASH_STOP_ADDR - 6U;                    /* Skip the last 32-bits */
            IO_ROM_BIST_ADD_STOP_H = 0U;                                        /* MMP220123-2 */
#elif defined (__MLX81160__) || defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__) || \
            defined (__MLX81334__)
            IO_ROM_BIST_ADD_STOP_L = ((uint16_t)((C_FLASH_STOP_ADDR - 6U) << 1) & 0xFFFEU);   /* Skip the last 32-bits */
            IO_ROM_BIST_ADD_STOP_H = (uint16_t)((C_FLASH_STOP_ADDR - 6U) >> 15);
#endif
        }
        else
        {
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81339__) || defined (__MLX81350__)
            IO_ROM_BIST_ADD_STOP_L = (u16SegmentAddr - 1U);
            IO_ROM_BIST_ADD_STOP_H = 0U;                                        /* MMP220123-2 */
#elif defined (__MLX81160__) || defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__) || \
            defined (__MLX81334__)
            uint16_t u16EndSegmentAddr = u16SegmentAddr - 1U;
            IO_ROM_BIST_ADD_STOP_L = ((u16EndSegmentAddr << 1) + 1U);
            IO_ROM_BIST_ADD_STOP_H = (u16EndSegmentAddr >> 15);
#endif
        }
    }
#if (_SUPPORT_MLX_CHIP_STATUS != FALSE)
    else
    {
        if (u16SegmentAddr >= C_NVM_STOP_ADDR)
        {
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81339__) || defined (__MLX81350__)
            IO_ROM_BIST_ADD_STOP_L = C_NVM_STOP_ADDR - 6U;                      /* Skip the last 32-bits */
#elif defined (__MLX81160__) || defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__) || \
            defined (__MLX81334__)
            IO_ROM_BIST_ADD_STOP_L = (C_NVM_STOP_ADDR << 1) - 6U;               /* Skip the last 32-bits */
#endif
        }
        else
        {
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81339__) || defined (__MLX81350__)
            IO_ROM_BIST_ADD_STOP_L = (u16SegmentAddr - 1U);
#elif defined (__MLX81160__) || defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__) || \
            defined (__MLX81334__)
            IO_ROM_BIST_ADD_STOP_L = ((u16SegmentAddr << 1) - 1U);
#endif
        }
        IO_ROM_BIST_ADD_STOP_H = 0U;                                            /* MMP220123-2 */
    }
#endif /* (_SUPPORT_MLX_CHIP_STATUS != FALSE) */
    IO_ROM_BIST_SIG_EXPECTED_L = 0U;
    IO_ROM_BIST_SIG_EXPECTED_H = 0U;
    IO_ROM_BIST_SIG_RECEIVED_L = (uint16_t)(u32FlashBist & 0xFFFFUL);
    IO_ROM_BIST_SIG_RECEIVED_H = (uint16_t)(u32FlashBist >> 16);
#if (_DEBUG_FLASH_BIST != FALSE)
    DEBUG_SET_IO_A();
#endif /* (_DEBUG_FLASH_BIST != FALSE) */
    IO_ROM_BIST_START_BIST = C_ROM_BIST_KEY;
    /* CPU will stop here until the BIST's finishes his job. */
#if (_DEBUG_FLASH_BIST != FALSE)
    DEBUG_CLR_IO_A();
#endif /* (_DEBUG_FLASH_BIST != FALSE) */
    NOP(); /* Inserted to let the HW BIST to not to skip the signature check in FX */
    NOP();
    u32FlashBist = (((uint32_t)IO_ROM_BIST_SIG_RECEIVED_H) << 16) | IO_ROM_BIST_SIG_RECEIVED_L;

#if (_SUPPORT_MLX_CHIP_STATUS != FALSE)
    if (u16SegmentAddr > C_FLASH_START_ADDR)
#endif /* (_SUPPORT_MLX_CHIP_STATUS != FALSE) */
    {
        /* Flash BIST */
        if (u16SegmentAddr >= C_FLASH_STOP_ADDR)
        {
#if (_DEBUG_FLASH_BIST != FALSE)
            DEBUG_SET_IO_A();
#endif /* (_DEBUG_FLASH_BIST != FALSE) */
#if (_SUPPORT_MLX_CHIP_STATUS != FALSE)
            g_u32FlashBist = u32FlashBist;
            u16SegmentAddr = C_NVM_START_ADDR;
#else  /* (_SUPPORT_MLX_CHIP_STATUS != FALSE) */
            u16SegmentAddr = C_FLASH_START_ADDR;
#endif /* (_SUPPORT_MLX_CHIP_STATUS != FALSE) */
            u16ValidUpdate = TRUE;
#if (_DEBUG_FLASH_BIST != FALSE)
            DEBUG_CLR_IO_A();
#endif /* (_DEBUG_FLASH_BIST != FALSE) */
        }
    }
#if (_SUPPORT_MLX_CHIP_STATUS != FALSE)
    else
    {
        /* Non Volatile Memory BIST */
        if (u16SegmentAddr >= C_NVM_STOP_ADDR)
        {
#if (_DEBUG_FLASH_BIST != FALSE)
            DEBUG_SET_IO_A();
#endif /* (_DEBUG_FLASH_BIST != FALSE) */
#if (_SUPPORT_MLX_CHIP_STATUS != FALSE)
            g_u32NvmBist = u32FlashBist;
#endif /* (_SUPPORT_MLX_CHIP_STATUS != FALSE) */
            u16SegmentAddr = C_FLASH_START_ADDR;
#if (_DEBUG_FLASH_BIST != FALSE)
            DEBUG_CLR_IO_A();
#endif /* (_DEBUG_FLASH_BIST != FALSE) */
        }
    }
#endif /* (_SUPPORT_MLX_CHIP_STATUS != FALSE) */
    return (u16ValidUpdate);
} /* End of FlashBist() */
#else  /* (_SUPPORT_IO_STYLE == FALSE) */
/*!*************************************************************************** *
 * FlashBist()
 * \brief   Calculate the Flash BIST in blocks of 256 Bytes
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  (uint16_t) u16ValidUpdate: FALSE: Not completed
 *                                     TRUE: Completely checked.
 * *************************************************************************** *
 * \details Flash BIST CRC24 calculation, using chip hardware BIST block
 * Notice:  During the BIST calculation the flash/CPU execution stops!
 *          Don't execute the Flash BIST during high-speed motor-operation
 *          or other time-critical operations (e.g. LIN Auto-Addressing)!
 * Performance: <20us (per 256 Bytes)
 *             (Complete Flash) 2.35 ms
 * *************************************************************************** *
 * - Call Hierarchy: AppMemoryCheck()
 * - Cyclomatic Complexity: 7+1
 * - Nesting: 2
 * - Function calling: 0
 * *************************************************************************** */
static uint16_t FlashBist(void)
{
    static uint16_t u16SegmentAddr = C_FLASH_START_ADDR;
    static uint32_t u32FlashBist = C_ROM_BIST_CFG_SEED;
    uint16_t u16ValidUpdate = FALSE;

    if (u16SegmentAddr == C_FLASH_START_ADDR)
    {
        u32FlashBist = C_ROM_BIST_CFG_SEED;               /* Initialise the CRC preset with SEED */
    }

    IO_SET(ROM_BIST, MASK_SIG_ERR, 0U,
           SINGLE_RAMP, 1U,
           BIST, 0U,
           ECC_POSITION, 2U);                 /* ECC position is 0b10 for 64bit FLASH */
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81339__) || defined (__MLX81350__)
    IO_SET(ROM_BIST, ADD_START_L, u16SegmentAddr);
    IO_SET(ROM_BIST, ADD_START_H, 0U);
    u16SegmentAddr += 0x0100U;
#elif defined (__MLX81160__) || defined (__MLX81340__) || defined (__MLX81344__)||defined (__MLX81346__)
    IO_SET(ROM_BIST, ADD_START_L, (u16SegmentAddr << 1));
    IO_SET(ROM_BIST, ADD_START_H, (u16SegmentAddr >> 15));
    u16SegmentAddr += (0x0100U >> 1);
#endif
    if (u16SegmentAddr >= C_FLASH_STOP_ADDR)
    {
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81339__) || defined (__MLX81350__)
        IO_SET(ROM_BIST, ADD_STOP_L, (C_FLASH_STOP_ADDR - 6U));     /* Skip the last 32-bits */
#elif defined (__MLX81160__)||defined (__MLX81340__)||defined (__MLX81344__)||defined (__MLX81346__)
        /* Skip the last 32-bits */
        IO_SET(ROM_BIST, ADD_STOP_L, (((uint16_t)(C_FLASH_STOP_ADDR << 1) & 0xFFFEU) - 6U));
        IO_SET(ROM_BIST, ADD_STOP_H, (uint16_t)(C_FLASH_STOP_ADDR >> 15));
#endif
    }
    else
    {
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81339__) || defined (__MLX81350__)
        IO_SET(ROM_BIST, ADD_STOP_L, (u16SegmentAddr - 1U));
#elif defined (__MLX81160__)||defined (__MLX81340__)||defined (__MLX81344__)||defined (__MLX81346__)
        uint16_t u16EndSegmentAddr = u16SegmentAddr - 1U;
        IO_SET(ROM_BIST, ADD_STOP_L, ((u16EndSegmentAddr << 1) + 1U));
        IO_SET(ROM_BIST, ADD_STOP_H, (u16EndSegmentAddr >> 15));
#endif
    }
    IO_SET(ROM_BIST, ADD_STOP_H, 0U);
    IO_SET(ROM_BIST, SIG_EXPECTED_L, 0U);
    IO_SET(ROM_BIST, SIG_EXPECTED_H, 0U);
    IO_SET(ROM_BIST, SIG_RECEIVED_L, (uint16_t)(u32FlashBist & 0xFFFFUL));
    IO_SET(ROM_BIST, SIG_RECEIVED_H, (uint16_t)(u32FlashBist >> 16));
    IO_SET(ROM_BIST, START_BIST, C_ROM_BIST_KEY);
    /* CPU will stop here until the BIST's finishes his job. */
    NOP(); /* Inserted to let the HW BIST to not to skip the signature check in FX */
    NOP();
    u32FlashBist = (((uint32_t)IO_GET(ROM_BIST, SIG_RECEIVED_H)) << 16) |
                   IO_GET(ROM_BIST, SIG_RECEIVED_L);

    /* Flash BIST */
    if (u16SegmentAddr >= C_FLASH_STOP_ADDR)
    {
#if (_SUPPORT_MLX_CHIP_STATUS != FALSE)
        g_u32FlashBist = u32FlashBist;
#endif /* (_SUPPORT_MLX_CHIP_STATUS != FALSE) */
        u16ValidUpdate = TRUE;
        u16SegmentAddr = C_FLASH_START_ADDR;
    }
    return (u16ValidUpdate);
} /* End of FlashBist() */
#endif /* (_SUPPORT_IO_STYLE == FALSE) */
#endif /* (_SUPPORT_BG_MEM_TEST != FALSE) && (_SUPPORT_BG_FLASH_TEST != FALSE) */

#if (_SUPPORT_BG_MEM_TEST != FALSE) && (_SUPPORT_BG_ROM_TEST != FALSE)
#if (_SUPPORT_IO_STYLE == FALSE)
/*!*************************************************************************** *
 * ColinRomBist()
 * \brief   Calculate the COLIN ROM BIST
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  (uint16_t) FALSE: Incorrect CRC24
 *                      TRUE: Correct CRC24
 * *************************************************************************** *
 * \details COLIN ROM BIST CRC24 calculation, using chip hardware BIST block
 * Notice:  During the BIST calculation the flash/CPU execution stops!
 *          Don't execute the Flash BIST during high-speed motor-operation
 *          or other time-critical operations (e.g. LIN Auto-Addressing)!
 * Performance: 110us (6kB) (@ 28MHz)
 *               96us (6kB) (@ 32MHz)
 * *************************************************************************** *
 * - Call Hierarchy: AppMemoryCheck()
 * - Cyclomatic Complexity: 2+1
 * - Nesting: 2
 * - Function calling: 0
 * *************************************************************************** */
static uint16_t ColinRomBist(void)
{
#if defined (C_ROM_MLX4_BIST_CRC)
#else  /* defined (C_ROM_MLX4_BIST_CRC) */
    static uint32_t u32ColinRomBist = 0U;
#endif /* defined (C_ROM_MLX4_BIST_CRC) */
    uint16_t u16Valid = FALSE;

    IO_ROM_BIST_CTRL = (IO_ROM_BIST_CTRL & ~(B_ROM_BIST_MASK_SIG_ERR |
                                             B_ROM_BIST_BIST |
                                             M_ROM_BIST_ECC_POSITION)) |
                       (B_ROM_BIST_SINGLE_RAMP | 2U);                           /* ECC position is 0b10 for 64bit FLASH */
    IO_ROM_BIST_ADD_START_L = (C_ROM_MLX4_START_ADDR & 0xFFFFU);
    IO_ROM_BIST_ADD_START_H = (C_ROM_MLX4_START_ADDR >> 16);
    IO_ROM_BIST_ADD_STOP_L = ((C_ROM_MLX4_END_ADDR - 4U) & 0xFFFFU);
    IO_ROM_BIST_ADD_STOP_H = ((C_ROM_MLX4_END_ADDR - 4U) >> 16);
    IO_ROM_BIST_SIG_EXPECTED_L = 0U;
    IO_ROM_BIST_SIG_EXPECTED_H = 0U;
    IO_ROM_BIST_SIG_RECEIVED_L = (C_ROM_BIST_CFG_SEED & 0xFFFFU);
    IO_ROM_BIST_SIG_RECEIVED_H = (C_ROM_BIST_CFG_SEED >> 16); /*lint !e572 */
#if (_DEBUG_FLASH_BIST != FALSE)
    DEBUG_SET_IO_A();
#endif /* (_DEBUG_FLASH_BIST != FALSE) */
    IO_ROM_BIST_START_BIST = C_ROM_BIST_KEY;
    /* CPU will stop here until the BIST's finishes his job. */
#if (_DEBUG_FLASH_BIST != FALSE)
    DEBUG_CLR_IO_A();
#endif /* (_DEBUG_FLASH_BIST != FALSE) */
    NOP(); /* Inserted to let the HW BIST to not to skip the signature check in FX */
    NOP();
#if defined (C_ROM_MLX4_BIST_CRC)
    if (C_ROM_MLX4_BIST_CRC == ((((uint32_t)IO_ROM_BIST_SIG_RECEIVED_H) << 16) | IO_ROM_BIST_SIG_RECEIVED_L) )
    {
        u16Valid = TRUE;
    }
#else  /* defined (C_ROM_MLX4_BIST_CRC) */
    if (u32ColinRomBist == 0U)
    {
        u32ColinRomBist = (((uint32_t)IO_ROM_BIST_SIG_RECEIVED_H) << 16) | IO_ROM_BIST_SIG_RECEIVED_L;
        u16Valid = TRUE;
    }
    else if (u32ColinRomBist == ((((uint32_t)IO_ROM_BIST_SIG_RECEIVED_H) << 16) | IO_ROM_BIST_SIG_RECEIVED_L) )
    {
        u16Valid = TRUE;
    }
    else
    {
        u16Valid = FALSE;
    }
#endif /* defined (C_ROM_MLX4_BIST_CRC) */
    return (u16Valid);
} /* End of ColinRomBist() */
#else  /* (_SUPPORT_IO_STYLE == FALSE) */
/*!*************************************************************************** *
 * ColinRomBist()
 * \brief   Calculate the COLIN ROM BIST
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  (uint16_t) FALSE: Incorrect CRC24
 *                      TRUE: Correct CRC24
 * *************************************************************************** *
 * \details COLIN ROM BIST CRC24 calculation, using chip hardware BIST block
 * Notice:  During the BIST calculation the flash/CPU execution stops!
 *          Don't execute the Flash BIST during high-speed motor-operation
 *          or other time-critical operations (e.g. LIN Auto-Addressing)!
 * Performance: 110us @ 28MHz (6kB)
 * *************************************************************************** *
 * - Call Hierarchy: AppMemoryCheck()
 * - Cyclomatic Complexity: 2+1
 * - Nesting: 2
 * - Function calling: 0
 * *************************************************************************** */
static uint16_t ColinRomBist(void)
{
#if !defined (C_ROM_MLX4_BIST_CRC)
    static uint32_t u32ColinRomBist = 0U;
#endif /* defined (C_ROM_MLX4_BIST_CRC) */
    uint16_t u16Valid = FALSE;

    IO_SET(ROM_BIST, MASK_SIG_ERR, 0U,
           SINGLE_RAMP, 1U,
           BIST, 0U,
           ECC_POSITION, 2U);                 /* ECC position is 0b10 for 64bit FLASH */
    IO_SET(ROM_BIST, ADD_START_L, (C_ROM_MLX4_START_ADDR & 0xFFFFU));
    IO_SET(ROM_BIST, ADD_START_H, (C_ROM_MLX4_START_ADDR >> 16));
    IO_SET(ROM_BIST, ADD_STOP_L, ((C_ROM_MLX4_END_ADDR - 4U) & 0xFFFFU));
    IO_SET(ROM_BIST, ADD_STOP_H, ((C_ROM_MLX4_END_ADDR - 4U) >> 16));
    IO_SET(ROM_BIST, SIG_EXPECTED_L, 0U);
    IO_SET(ROM_BIST, SIG_EXPECTED_H, 0U);
    IO_SET(ROM_BIST, SIG_RECEIVED_L, (C_ROM_BIST_CFG_SEED & 0xFFFFU));
    IO_SET(ROM_BIST, SIG_RECEIVED_H, (C_ROM_BIST_CFG_SEED >> 16));  /*lint !e572 */
    IO_SET(ROM_BIST, START_BIST, C_ROM_BIST_KEY);
    /* CPU will stop here until the BIST's finishes his job. */
    NOP(); /* Inserted to let the HW BIST to not to skip the signature check in FX */
    NOP();
#if defined (C_ROM_MLX4_BIST_CRC)
    if (C_ROM_MLX4_BIST_CRC == ((((uint32_t)IO_GET(ROM_BIST, SIG_RECEIVED_H)) << 16) |
                                IO_GET(ROM_BIST, SIG_RECEIVED_L)))
    {
        u16Valid = TRUE;
    }
#else  /* defined (C_ROM_MLX4_BIST_CRC) */
    if (u32ColinRomBist == 0U)
    {
        u32ColinRomBist = (((uint32_t)IO_GET(ROM_BIST, SIG_RECEIVED_H)) << 16) |
                          IO_GET(ROM_BIST, SIG_RECEIVED_L);
        u16Valid = TRUE;
    }
    else if (u32ColinRomBist == ((((uint32_t)IO_GET(ROM_BIST, SIG_RECEIVED_H)) << 16) |
                                 IO_GET(ROM_BIST, SIG_RECEIVED_L)) )
    {
        u16Valid = TRUE;
    }
    else
    {
        u16Valid = FALSE;
    }
#endif /* defined (C_ROM_MLX4_BIST_CRC) */
    return (u16Valid);
} /* End of ColinRomBist() */
#endif /* (_SUPPORT_IO_STYLE == FALSE) */

#if (_SUPPORT_IO_STYLE == FALSE)
/*!*************************************************************************** *
 * SysRomBist()
 * \brief   Calculate the System ROM BIST
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  (uint16_t) FALSE: Incorrect CRC24
 *                      TRUE: Correct CRC24
 *                   UNKNOWN: Busy
 * *************************************************************************** *
 * \details System ROM BIST CRC24 calculation, using chip hardware BIST block
 * Notice:  During the BIST calculation the flash/CPU execution stops!
 *          Don't execute the Flash BIST during high-speed motor-operation
 *          or other time-critical operations (e.g. LIN Auto-Addressing)!
 * Performance: 370 us @ 28MHz (10kB)
 * *************************************************************************** *
 * - Call Hierarchy: AppMemoryCheck()
 * - Cyclomatic Complexity: 1+1
 * - Nesting: 1
 * - Function calling: 0
 * *************************************************************************** */
static uint16_t SysRomBist(void)
{
#if (_SUPPORT_BG_MEM_TEST_SEGMENTS != FALSE)
    static uint16_t u16SegmentAddr = C_ROM_SYS_START_ADDR;
    static uint32_t u32RomBist = C_ROM_BIST_CFG_SEED;
#endif /* (_SUPPORT_BG_MEM_TEST_SEGMENTS != FALSE) */
#if defined (__MLX81330B02__) || defined (__MLX81332B02__) || defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__) || \
    defined (__MLX81340B01__) || defined (__MLX81344B01__) || defined (__MLX81346B01__)
#else  /* defined (__MLX81330B02__) || defined (__MLX81332B02__) || defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__) ||\
        * defined (__MLX81340B01__) || defined (__MLX81344B01__) || defined (__MLX81346B01__) */
    static uint32_t u32SysRomBist = 0U;
#endif /* defined (__MLX81330B02__) || defined (__MLX81332B02__) || defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__) ||\
        * defined (__MLX81340B01__) || defined (__MLX81344B01__) || defined (__MLX81346B01__) */
    uint16_t u16Valid = FALSE;

    IO_ROM_BIST_CTRL = (IO_ROM_BIST_CTRL & ~(B_ROM_BIST_MASK_SIG_ERR |
                                             B_ROM_BIST_BIST |
                                             M_ROM_BIST_ECC_POSITION)) |
                       (B_ROM_BIST_SINGLE_RAMP | 2U);                           /* ECC position is 0b10 for 64bit FLASH */
#if (_SUPPORT_BG_MEM_TEST_SEGMENTS == FALSE)
    IO_ROM_BIST_ADD_START_L = (C_ROM_SYS_START_ADDR & 0xFFFFU);
    IO_ROM_BIST_ADD_START_H = (C_ROM_SYS_START_ADDR >> 16); /*lint !e572 */
    IO_ROM_BIST_ADD_STOP_L = ((C_ROM_SYS_END_ADDR - 4U) & 0xFFFFU);
    IO_ROM_BIST_ADD_STOP_H = ((C_ROM_SYS_END_ADDR - 4U) >> 16); /*lint !e572 */
    IO_ROM_BIST_SIG_EXPECTED_L = 0U;
    IO_ROM_BIST_SIG_EXPECTED_H = 0U;
    IO_ROM_BIST_SIG_RECEIVED_L = (C_ROM_BIST_CFG_SEED & 0xFFFFU);
    IO_ROM_BIST_SIG_RECEIVED_H = (C_ROM_BIST_CFG_SEED >> 16); /*lint !e572 */
#else  /* (_SUPPORT_BG_MEM_TEST_SEGMENTS == FALSE) */
    /* Small RAM blocks of 32 Bytes per time */
    IO_ROM_BIST_ADD_START_L = u16SegmentAddr;
    IO_ROM_BIST_ADD_START_H = 0x0U; /*lint !e572 */
    u16SegmentAddr += 1024U;
    if (u16SegmentAddr >= C_ROM_SYS_END_ADDR)
    {
        IO_ROM_BIST_ADD_STOP_L = (u16SegmentAddr - 5U);
    }
    else
    {
        IO_ROM_BIST_ADD_STOP_L = (u16SegmentAddr - 1U);
    }
    IO_ROM_BIST_ADD_STOP_H = 0x0U; /*lint !e572 */
    IO_ROM_BIST_SIG_EXPECTED_L = 0U;
    IO_ROM_BIST_SIG_EXPECTED_H = 0U;
    IO_ROM_BIST_SIG_RECEIVED_L = (uint16_t)(u32RomBist & 0xFFFFUL);
    IO_ROM_BIST_SIG_RECEIVED_H = (uint16_t)(u32RomBist >> 16);
#endif /* (_SUPPORT_BG_MEM_TEST_SEGMENTS == FALSE) */
#if (_DEBUG_FLASH_BIST != FALSE)
    DEBUG_SET_IO_A();
#endif /* (_DEBUG_FLASH_BIST != FALSE) */
    IO_ROM_BIST_START_BIST = C_ROM_BIST_KEY;
    /* CPU will stop here until the BIST's finishes his job. */
#if (_DEBUG_FLASH_BIST != FALSE)
    DEBUG_CLR_IO_A();
#endif /* (_DEBUG_FLASH_BIST != FALSE) */
    NOP(); /* Inserted to let the HW BIST to not to skip the signature check in FX */
    NOP();
#if (_SUPPORT_BG_MEM_TEST_SEGMENTS != FALSE)
    u32RomBist = (((uint32_t)IO_ROM_BIST_SIG_RECEIVED_H) << 16) | IO_ROM_BIST_SIG_RECEIVED_L;
    if (u16SegmentAddr >= C_ROM_SYS_END_ADDR)
    {
#if defined (__MLX81330B02__) || defined (__MLX81332B02__) || defined (__MLX81334__) || defined (__MLX81339__) || \
        defined (__MLX81340B01__) || defined (__MLX81344B01__) || defined (__MLX81346B01__) || defined (__MLX81350__)
        if (*((uint32_t *)C_ROM_SYS_CRC_ADDR) == u32RomBist)
        {
            u16Valid = TRUE;
        }
#else  /* defined (__MLX81330B02__) || defined (__MLX81332B02__) || defined (__MLX81334__) || defined (__MLX81339__) || \
        * defined (__MLX81340B01__) || defined (__MLX81344B01__) || defined (__MLX81346B01__) || defined (__MLX81350__) */
        if (u32SysRomBist == 0U)
        {
            u32SysRomBist = u32RomBist;
            u16Valid = TRUE;
        }
        else if (u32SysRomBist == u32RomBist)
        {
            u16Valid = TRUE;
        }
        else
        {
            u16Valid = FALSE;
        }
#endif /* defined (__MLX81330B02__) || defined (__MLX81332B02__) || defined (__MLX81334__) || defined (__MLX81339__) || \
        * defined (__MLX81340B01__) || defined (__MLX81344B01__) || defined (__MLX81346B01__) || defined (__MLX81350__) */
        u16SegmentAddr = C_ROM_SYS_START_ADDR;
        u32RomBist = C_ROM_BIST_CFG_SEED;
    }
    else
    {
        u16Valid = UNKNOWN;
    }
#else  /* (_SUPPORT_BG_MEM_TEST_SEGMENTS != FALSE) */
#if defined (__MLX81330B02__) || defined (__MLX81332B02__) || defined (__MLX81334__) || defined (__MLX81339__) || \
    defined (__MLX81340B01__) || defined (__MLX81344B01__) || defined (__MLX81346B01__) || defined (__MLX81350__)
    if (*((uint32_t *)C_ROM_SYS_CRC_ADDR) ==
        ((((uint32_t)IO_ROM_BIST_SIG_RECEIVED_H) << 16) | IO_ROM_BIST_SIG_RECEIVED_L) )
    {
        u16Valid = TRUE;
    }
#else  /* defined (__MLX81330B02__) || defined (__MLX81332B02__) || defined (__MLX81334__) || defined (__MLX81339__) ||\
        * defined (__MLX81340B01__) || defined (__MLX81344B01__) || defined (__MLX81346B01__) || defined (__MLX81350__) */
    if (u32SysRomBist == 0U)
    {
        u32SysRomBist = (((uint32_t)IO_ROM_BIST_SIG_RECEIVED_H) << 16) | IO_ROM_BIST_SIG_RECEIVED_L;
        u16Valid = TRUE;
    }
    else if (u32SysRomBist == ((((uint32_t)IO_ROM_BIST_SIG_RECEIVED_H) << 16) | IO_ROM_BIST_SIG_RECEIVED_L) )
    {
        u16Valid = TRUE;
    }
    else
    {
        u16Valid = FALSE;
    }
#endif /* defined (__MLX81330B02__) || defined (__MLX81332B02__) || defined (__MLX81334__) || defined (__MLX81339__) ||\
        * defined (__MLX81340B01__) || defined (__MLX81344B01__) || defined (__MLX81346B01__) || defined (__MLX81350__) */
#endif /* (_SUPPORT_BG_MEM_TEST_SEGMENTS != FALSE) */
    return (u16Valid);
} /* End of SysRomBist() */
#else  /* (_SUPPORT_IO_STYLE == FALSE) */
/*!*************************************************************************** *
 * SysRomBist()
 * \brief   Calculate the System ROM BIST
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  (uint16_t) FALSE: Incorrect CRC24
 *                      TRUE: Correct CRC24
 * *************************************************************************** *
 * \details System ROM BIST CRC24 calculation, using chip hardware BIST block
 * Notice:  During the BIST calculation the flash/CPU execution stops!
 *          Don't execute the Flash BIST during high-speed motor-operation
 *          or other time-critical operations (e.g. LIN Auto-Addressing)!
 * Performance: 370 us @ 28MHz (10kB)
 * *************************************************************************** *
 * - Call Hierarchy: AppMemoryCheck()
 * - Cyclomatic Complexity: 2+1
 * - Nesting: 2
 * - Function calling: 0
 * *************************************************************************** */
static uint16_t SysRomBist(void)
{
#if defined (__MLX81330B02__) || defined (__MLX81332B02__) || defined (__MLX81334__) || defined (__MLX81339__) || \
    defined (__MLX81340B01__) || defined (__MLX81344B01__) || defined (__MLX81346B01__) || defined (__MLX81350__)
#else  /* defined (__MLX81330B02__) || defined (__MLX81332B02__) || defined (__MLX81334__) || defined (__MLX81339__) ||\
        * defined (__MLX81340B01__) || defined (__MLX81344B01__) || defined (__MLX81346B01__) || defined (__MLX81350__) */
    static uint32_t u32SysRomBist = 0U;
#endif /* defined (__MLX81330B02__) || defined (__MLX81332B02__) || defined (__MLX81334__) || defined (__MLX81339__) ||\
        * defined (__MLX81340B01__) || defined (__MLX81344B01__) || defined (__MLX81346B01__) || defined (__MLX81350__) */
    uint16_t u16Valid = FALSE;

    IO_SET(ROM_BIST, MASK_SIG_ERR, 0U,
           SINGLE_RAMP, 1U,
           BIST, 0U,
           ECC_POSITION, 2U);                 /* ECC position is 0b10 for 64bit FLASH */
    IO_SET(ROM_BIST, ADD_START_L, (C_ROM_SYS_START_ADDR & 0xFFFFU));
    IO_SET(ROM_BIST, ADD_START_H, (C_ROM_SYS_START_ADDR >> 16));  /*lint !e572 */
    IO_SET(ROM_BIST, ADD_STOP_L, ((C_ROM_SYS_END_ADDR - 4U) & 0xFFFFU));
    IO_SET(ROM_BIST, ADD_STOP_H, ((C_ROM_SYS_END_ADDR - 4U) >> 16));  /*lint !e572 */
    IO_SET(ROM_BIST, SIG_EXPECTED_L, 0U);
    IO_SET(ROM_BIST, SIG_EXPECTED_H, 0U);
    IO_SET(ROM_BIST, SIG_RECEIVED_L, (C_ROM_BIST_CFG_SEED & 0xFFFFU));
    IO_SET(ROM_BIST, SIG_RECEIVED_H, (C_ROM_BIST_CFG_SEED >> 16));  /*lint !e572 */
    IO_SET(ROM_BIST, START_BIST, C_ROM_BIST_KEY);
    /* CPU will stop here until the BIST's finishes his job. */
    NOP(); /* Inserted to let the HW BIST to not to skip the signature check in FX */
    NOP();
#if defined (__MLX81330B02__) || defined (__MLX81332B02__) || defined (__MLX81334__) || defined (__MLX81339__) || \
    defined (__MLX81340B01__) || defined (__MLX81344B01__) || defined (__MLX81346B01__) || defined (__MLX81350__)
    if (*((uint32_t *)C_ROM_SYS_CRC_ADDR) == ((((uint32_t)IO_GET(ROM_BIST, SIG_RECEIVED_H)) << 16) |
                                              IO_GET(ROM_BIST, SIG_RECEIVED_L)) )
    {
        u16Valid = TRUE;
    }
#else  /* defined (__MLX81330B02__) || defined (__MLX81332B02__) || defined (__MLX81334__) || defined (__MLX81339__) ||\
        * defined (__MLX81340B01__) || defined (__MLX81344B01__) || defined (__MLX81346B01__) || defined (__MLX81350__) */
    if (u32SysRomBist == 0U)
    {
        u32SysRomBist = (((uint32_t)IO_GET(ROM_BIST, SIG_RECEIVED_H)) << 16) | IO_GET(ROM_BIST, SIG_RECEIVED_L);
        u16Valid = TRUE;
    }
    else if (u32SysRomBist == ((((uint32_t)IO_GET(ROM_BIST, SIG_RECEIVED_H)) << 16) |
                               IO_GET(ROM_BIST, SIG_RECEIVED_L)) )
    {
        u16Valid = TRUE;
    }
    else
    {
        u16Valid = FALSE;
    }
#endif /* defined (__MLX81330B02__) || defined (__MLX81332B02__) || defined (__MLX81334__) || defined (__MLX81339__) ||\
        * defined (__MLX81340B01__) || defined (__MLX81344B01__) || defined (__MLX81346B01__) || defined (__MLX81350__) */
    return (u16Valid);
} /* End of SysRomBist() */
#endif /* (_SUPPORT_IO_STYLE == FALSE) */
#endif /* (_SUPPORT_BG_MEM_TEST != FALSE) && (_SUPPORT_BG_ROM_TEST != FALSE) */

#if (_SUPPORT_BG_MEM_TEST != FALSE) && (_SUPPORT_BG_RAM_TEST != FALSE)
#if (_SUPPORT_IO_STYLE == FALSE)
/*!*************************************************************************** *
 * RamTransparentBist()
 * \brief   Check System RAM by Transparent BIST
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  (uint16_t) FALSE: RAM Failure
 *                      TRUE: RAM Okay
 * *************************************************************************** *
 * \details RAM BIST performs a transparent RAM check.
 * Notice:  During the RAM BIST check, no RAM can be used. Therefore all
 *          DMA-peripherals should be disabled, as well as the IRQ's.
 * Performance: 1.33ms @ 28MHz (2kB)
 * *************************************************************************** *
 * - Call Hierarchy: AppMemoryCheck()
 * - Cyclomatic Complexity: 1+1
 * - Nesting: 1
 * - Function calling: 0
 * *************************************************************************** */
static uint16_t RamTransparentBist(void)
{
#if (_SUPPORT_BG_MEM_TEST_SEGMENTS != FALSE)
    static uint16_t u16SegmentAddr = C_RAM_MLX16_START_ADDR;
#endif /* (_SUPPORT_BG_MEM_TEST_SEGMENTS != FALSE) */
    uint16_t u16Valid = FALSE;

    IO_RAM_BIST_CTRL = /*B_RAM_BIST_SIGNATURE_INIT |*/                          /* Signature initialised with 0xFFFF */
                       (2U << 8) |                                              /* Number of bits used for address scrambling */
                       /*B_RAM_BIST_FUNCTIONAL_BIST |*/                         /* Perform a non-functional BIST (?) */
                       /*B_RAM_BIST_WORD_BIST |*/                               /* BIST in Word */
                       (0U << 0);                                               /* Number of ECC-bits */
#if (_SUPPORT_BG_MEM_TEST_SEGMENTS == FALSE)
    /* Complete RAM at once */
    IO_RAM_BIST_ADD_START_L = (C_RAM_MLX16_START_ADDR & 0xFFFFU);
    IO_RAM_BIST_ADD_START_H = (C_RAM_MLX16_START_ADDR >> 16); /*lint !e572 */
    IO_RAM_BIST_ADD_STOP_L = (C_RAM_MLX16_END_ADDR & 0xFFFFU);
    IO_RAM_BIST_ADD_STOP_H = (C_RAM_MLX16_END_ADDR >> 16); /*lint !e572 */
#else  /* (_SUPPORT_BG_MEM_TEST_SEGMENTS == FALSE) */
    /* Small RAM blocks of 32 Bytes per time */
    IO_RAM_BIST_ADD_START_L = u16SegmentAddr;
    IO_RAM_BIST_ADD_START_H = 0x0U; /*lint !e572 */
    u16SegmentAddr += 32U;
    IO_RAM_BIST_ADD_STOP_L = (u16SegmentAddr - 1U);
    IO_RAM_BIST_ADD_STOP_H = 0x0U; /*lint !e572 */
#endif /* (_SUPPORT_BG_MEM_TEST_SEGMENTS == FALSE) */
    ENTER_SECTION(ATOMIC_SYSTEM_MODE); /*lint !e534 */
#if (_DEBUG_FLASH_BIST != FALSE)
    DEBUG_SET_IO_A();
#endif /* (_DEBUG_FLASH_BIST != FALSE) */
    IO_RAM_BIST = C_RAM_TRANSPARENT_BIST_KEY;
    NOP(); /* Inserted to let the HW BIST to not to skip the signature check in FX */
    /* CPU will stop here until the BIST's finishes his job. */
    while ( (IO_RAM_BIST & B_RAM_BIST_COMPLETED) == 0)
    {}
#if (_DEBUG_FLASH_BIST != FALSE)
    DEBUG_CLR_IO_A();
#endif /* (_DEBUG_FLASH_BIST != FALSE) */
    EXIT_SECTION(); /*lint !e438 */
    if ( (IO_RAM_BIST & B_RAM_BIST_REGULAR_BIST_ERROR) == 0U)
    {
        u16Valid = TRUE;
    }
    else
    {
        u16Valid = FALSE;
        /* It is not possible to diagnose which (address, data) is faulty */
    }
    (void)IO_RAM_BIST_SIGN_DATA;
#if (_SUPPORT_BG_MEM_TEST_SEGMENTS != FALSE)
    if (u16SegmentAddr >= C_RAM_MLX16_END_ADDR)
    {
        u16SegmentAddr = C_RAM_MLX16_START_ADDR;
    }
#endif /* (_SUPPORT_BG_MEM_TEST_SEGMENTS != FALSE) */
    return (u16Valid);
} /* End of RamTransparentBist() */
#else  /* (_SUPPORT_IO_STYLE == FALSE) */
/*!*************************************************************************** *
 * RamTransparentBist()
 * \brief   Check System RAM by Transparent BIST
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  (uint16_t) FALSE: RAM error
 *                      TRUE: RAM okay
 * *************************************************************************** *
 * \details RAM BIST performs a transparent RAM check.
 * Notice:  During the RAM BIST check, no RAM can be used. Therefore all
 *          DMA-peripherals should be disabled, as well as the IRQ's.
 * Performance: 1.33ms @ 28MHz (2kB)
 * *************************************************************************** *
 * - Call Hierarchy: AppMemoryCheck()
 * - Cyclomatic Complexity: 1+1
 * - Nesting: 1
 * - Function calling: 0
 * *************************************************************************** */
static uint16_t RamTransparentBist(void)
{
    uint16_t u16Valid = FALSE;

    IO_SET(RAM_BIST, SIGNATURE_INIT, 0U,
           ADD_SCRAMBLE, 2U,
           FUNCTIONAL_BIST, 0U,
           WORD_BIST, 0U,
           NB_ECC_BITS, 0U);
    IO_SET(RAM_BIST, ADD_START_L, (C_RAM_MLX16_START_ADDR & 0xFFFFU));
    IO_SET(RAM_BIST, ADD_START_H, (C_RAM_MLX16_START_ADDR >> 16));
    IO_SET(RAM_BIST, ADD_STOP_L, (C_RAM_MLX16_END_ADDR & 0xFFFFU));
    IO_SET(RAM_BIST, ADD_STOP_H, (C_RAM_MLX16_END_ADDR >> 16));
    ENTER_SECTION(ATOMIC_SYSTEM_MODE); /*lint !e534 */
    IO_SET(RAM_BIST, KEY, C_RAM_TRANSPARENT_BIST_KEY);
    NOP(); /* Inserted to let the HW BIST to not to skip the signature check in FX */
    /* CPU will stop here until the BIST's finishes his job. */
    while ( (IO_GET(RAM_BIST, COMPLETED)) == 0) {}
    EXIT_SECTION(); /*lint !e438 */
    if ( (IO_GET(RAM_BIST, REGULAR_BIST_ERROR)) == 0U)
    {
        u16Valid = TRUE;
    }
    else
    {
        u16Valid = FALSE;
        /* It is not possible to diagnose which (address, data) is faulty */
    }
    (void)IO_GET(RAM_BIST, LFSR);
    return (u16Valid);
} /* End of RamTransparentBist() */

#endif /* (_SUPPORT_IO_STYLE == FALSE) */
#endif /* (_SUPPORT_BG_MEM_TEST != FALSE) && (_SUPPORT_BG_RAM_TEST != FALSE) */

#if (_SUPPORT_BG_MEM_TEST != FALSE) && ((_SUPPORT_BG_FLASH_TEST != FALSE) || \
    (_SUPPORT_BG_RAM_TEST != FALSE) || (_SUPPORT_BG_ROM_TEST != FALSE))
/*!*************************************************************************** *
 * AppMemoryCheck()
 * \brief   Application Memory check
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details Memory Check
 *          Mode 0: Flash Check, based of Flash CRC24
 *          Mode 1: CoLIN (MLX4) ROM BIST Check
 *          Mode 2: System (MLX16) ROM BIST Check
 *          Mode 3: System (MLX16) RAM Transparent BIST Check (IRQ's will be blocked)
 * Note: Prior to the memory check, DMA must be stopped (ADC, I2C, UART, SPI, PPM).
 * *************************************************************************** *
 * - Call Hierarchy: AppBackgroundTaskHandler()
 * - Cyclomatic Complexity: 2+1
 * - Nesting: 2
 * - Function calling: 0
 * *************************************************************************** */
static void AppMemoryCheck(void)
{
    /* Make sure no DMA access is active */
    if ( ((IO_ADC_CTRL & B_ADC_STOP) != 0U)                                     /* ADC is inactive */
#if (_SUPPORT_I2C != FALSE) && (_SUPPORT_I2C_SLAVE != FALSE)
         && ((IO_PORT_I2C_CONF & B_PORT_I2C_CONF_I2C_ADDR_VALID) == 0U)         /* No I2C active (Always use DMA) */
#endif /* (_SUPPORT_I2C != FALSE) && (_SUPPORT_I2C_SLAVE != FALSE) */
#if (defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__)) && _SUPPORT_UART
         && ((IO_PORT_UDMA_CTRL & (B_PORT_UDMA_CTRL_UDMA_TXSTART | B_PORT_UDMA_CTRL_UDMA_EN)) == 0U)  /* No UART DMA active */
#elif (defined (__MLX81160__) || defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)) && \
         _SUPPORT_UART
         && ((IO_PORT_UDMA0_CTRL & (B_PORT_UDMA0_CTRL_UDMA0_TXSTART | B_PORT_UDMA0_CTRL_UDMA0_EN)) == 0U)  /* No UART DMA active */
         && ((IO_PORT_UDMA1_CTRL & (B_PORT_UDMA1_CTRL_UDMA1_TXSTART | B_PORT_UDMA1_CTRL_UDMA1_EN)) == 0U)  /* No UART DMA active */
#endif /* (defined (__MLX81160__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)) && _SUPPORT_UART */
#if (_SUPPORT_SPI != FALSE) && (_SUPPORT_SPI_DMA != FALSE)
         && ((IO_SPI_CTRL & (B_SPI_DMA | B_SPI_STOP)) != B_SPI_DMA)             /* No SPI DMA active */
#endif /* (_SUPPORT_SPI != FALSE) && (_SUPPORT_SPI_DMA != FALSE) */
#if FALSE
         && ((IO_PORT_PPM_CTRL & B_PORT_PPM_CTRL_PPM_EN) == 0U)                 /* No PPM active (Always use DMA) */
#endif
#if (LIN_COMM != FALSE) && (_SUPPORT_LIN_AA != FALSE)
         && (g_u8LinAAMode == (uint8_t)C_SNPD_SUBFUNC_INACTIVE)                 /* and: Not in LIN-AA mode (MMP181109-1) */
#endif /* (LIN_COMM != FALSE) && (_SUPPORT_LIN_AA != FALSE) */
#if (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT)
         && ((g_e8MotorStatus & C_MOTOR_STATUS_STOP_MASK) == C_MOTOR_STATUS_STOP) /* and: Motor not running (MMP181109-1) */
#endif /* (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT) */
         )
    {
        static uint16_t u16MemChkMode = 0U;

#if (_SUPPORT_BG_FLASH_TEST != FALSE)
        if (u16MemChkMode == 0U)
        {
            /* Flash BIST test will block CPU! */
            if (FlashBist() != FALSE)
            {
                /* Flash BIST completed */
#if (_SUPPORT_MLX_CHIP_STATUS != FALSE)
                if (*((uint32_t *)C_FLASH_CRC_ADDR) != g_u32FlashBist)
                {
                    g_e8ErrorElectric = C_ERR_MEM_FLASH;
                    MotorDriverStop(C_STOP_IMMEDIATE);
#if (_SUPPORT_LOG_ERRORS != FALSE)
                    SetLastError(C_ERR_FLASH_BG);
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
                }
#endif /* (_SUPPORT_MLX_CHIP_STATUS != FALSE) */
                u16MemChkMode++;
            }
            else
            {
                /* Flash BIST not finished */
            }
        }
        else
#endif /* (_SUPPORT_BG_FLASH_TEST != FALSE) */
#if (_SUPPORT_BG_ROM_TEST != FALSE)
        if (u16MemChkMode == 1U)
        {
            if (ColinRomBist() == FALSE)
            {
                g_e8ErrorElectric = C_ERR_MEM_COLIN_ROM;
#if (_SUPPORT_LOG_ERRORS != FALSE)
                SetLastError(C_ERR_ROM_BG | C_ERR_EXT | 0x0100U);
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
            }
            u16MemChkMode++;
        }
        else if (u16MemChkMode == 2U)
        {
            uint16_t u16Result = SysRomBist();
            if (u16Result == FALSE)
            {
                g_e8ErrorElectric = C_ERR_MEM_SYS_ROM;
#if (_SUPPORT_LOG_ERRORS != FALSE)
                SetLastError(C_ERR_ROM_BG | C_ERR_EXT | 0x0200U);
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
            }
#if (_SUPPORT_BG_MEM_TEST_SEGMENTS != FALSE)
            if (u16Result != UNKNOWN)
            {
                u16MemChkMode++;
            }
#endif /* (_SUPPORT_BG_MEM_TEST_SEGMENTS != FALSE) */
        }
        else
#endif /* (_SUPPORT_BG_ROM_TEST != FALSE) */
#if (_SUPPORT_BG_RAM_TEST != FALSE)
        if (u16MemChkMode == 3U)
        {
            if (RamTransparentBist() == FALSE)
            {
                g_e8ErrorElectric = C_ERR_MEM_RAM;
#if (_SUPPORT_LOG_ERRORS != FALSE)
                SetLastError(C_ERR_RAM_BG);
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
            }
            u16MemChkMode++;
        }
        else
#endif /* (_SUPPORT_BG_RAM_TEST != FALSE) */
        if (u16MemChkMode >= 4U)
        {
            u16MemChkMode = 0U;
        }
        else
        {
            u16MemChkMode++;
        }
    }
} /* End of AppMemoryCheck() */
#endif /* (_SUPPORT_BG_MEM_TEST != FALSE) && ((_SUPPORT_BG_FLASH_TEST != FALSE) || (_SUPPORT_BG_RAM_TEST != FALSE) || (_SUPPORT_BG_ROM_TEST != FALSE)) */

/*!*************************************************************************** *
 *   AppTemperatureProfileCheck()
 * \brief   Application Temperature profile check
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details
 * *************************************************************************** *
 * - Call Hierarchy: AppBackgroundTaskHandler()
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 0
 * *************************************************************************** */
static void AppTemperatureProfileCheck(void)
{
    /* TODO[MMP]: Nothing for now */
} /* End of AppTemperatureProfileCheck() */

#if (_SUPPORT_CPU_HALT != FALSE) && (I2C_COMM == FALSE) && (SPI_COMM == FALSE) && (UART_COMM == FALSE)
/*!*************************************************************************** *
 * AppProcPowerSave()
 * \brief     Application Low-power mode
 * \author    mmp
 * *************************************************************************** *
 * \param     -
 * \return    (uint16_t) Number of [PLL/CTimerDiv] ticks
 * *************************************************************************** *
 * \details   Switch main-CPU to halt
 * *************************************************************************** *
 * - Call Hierarchy: AppBackgroundTaskHandler()
 * - Cyclomatic Complexity: 3+1
 * - Nesting: 2
 * - Function calling: 3 (Get_MotorStopDelay(), ADC_PowerOff(),
 *                        TimerSleepCompensation())
 * *************************************************************************** */
static void AppProcPowerSave(void)
{
    uint16_t u16TimerCnt = 0U;

#if (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT)
    if ( ((g_e8MotorStatus & C_MOTOR_STATUS_STOP_MASK) == C_MOTOR_STATUS_STOP)  /* Actuator is not running */
#elif (_SUPPORT_APP_TYPE == C_APP_RELAY)
#if (_SUPPORT_DUAL_RELAY == FALSE)
    if ( ((g_e8RelayStatus & (uint8_t)C_RELAY_STATUS_MASK) == (uint8_t)C_RELAY_STATUS_OFF)
#else  /* (_SUPPORT_DUAL_RELAY == FALSE) */
    if ( (((g_e8RelayStatusA & (uint8_t)C_RELAY_STATUS_MASK) == (uint8_t)C_RELAY_STATUS_OFF) &&
          ((g_e8RelayStatusB & (uint8_t)C_RELAY_STATUS_MASK) == (uint8_t)C_RELAY_STATUS_OFF))
#endif /* (_SUPPORT_DUAL_RELAY == FALSE) */
#elif (_SUPPORT_APP_TYPE == C_APP_SOLENOID)
    if ( ((g_e8SolenoidStatus & C_SOLENOID_STATUS_MASK) == C_SOLENOID_STATUS_DEACTIVATED)  /* Solenoid is not de-activated */
#endif /* (_SUPPORT_APP_TYPE) */
         && (Get_MotorDriverDisconDelay() == 0U)                                /* Motor Driver is disconnected, or holding-mode */
#if (LIN_COMM != FALSE)
#if ((CAN_COMM != FALSE) || (PWM_COMM != FALSE) || (SPI_COMM != FALSE) || \
    ((_SUPPORT_UART != FALSE) && (_SUPPORT_UART_IF_APP == C_UART_IF_QUAD_WHEEL_CAR)))
         && (g_u8LinConnected != C_COMM_DISCONNECTED)
#endif /* ((CAN_COMM != FALSE) || (PWM_COMM != FALSE) || (SPI_COMM != FALSE) || ((_SUPPORT_UART != FALSE) && (_SUPPORT_UART_IF_APP == C_UART_IF_QUAD_WHEEL_CAR))) */
       /*&& (l_u8Mlx4Connected != FALSE) */
#if (_SUPPORT_LIN_AA != FALSE)
         && (g_u8LinAAMode == (uint8_t)C_SNPD_SUBFUNC_INACTIVE)                 /* LIN-AA is not active */
#endif /* (_SUPPORT_LIN_AA != FALSE) */
#endif /* (LIN_COMM != FALSE) */
         )
    {
        /* Save */
        uint16_t u16CopyMask0;
        uint16_t u16CopyMask1;
        uint16_t u16CopyMask2;
        uint16_t u16CopyMask3;
#if defined (__MLX81160__) || defined (__MLX81332B01__) || defined (__MLX81332B02__) || defined (__MLX81334__) || defined (__MLX81339__) || \
        defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
        uint16_t u16CopyMask4;
#endif /* defined (__MLX81332B01__) || defined (__MLX81332B02__) || defined (__MLX81334__) || defined (__MLX81339__) ||\
        * defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__) */
        uint16_t u16Timer0Ctrl;

#if (_SUPPORT_APP_USER_MODE != FALSE)
        ENTER_SECTION(SYSTEM_MODE); /*lint !e534 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
        u16CopyMask0 = IO_MLX16_ITC_MASK0_S;
        u16CopyMask1 = IO_MLX16_ITC_MASK1_S;
        u16CopyMask2 = IO_MLX16_ITC_MASK2_S;
        u16CopyMask3 = IO_MLX16_ITC_MASK3_S;
#if defined (__MLX81160__) || defined (__MLX81332B01__) || defined (__MLX81332B02__) || defined (__MLX81334__) || defined (__MLX81339__) || \
        defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
        u16CopyMask4 = IO_MLX16_ITC_MASK4_S;
#endif /* defined (__MLX81332B01__) || defined (__MLX81332B02__) || defined (__MLX81334__) || defined (__MLX81339__) ||\
        * defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__) */
        u16Timer0Ctrl = IO_CTIMER0_CTRL;
#if (_SUPPORT_CPU_HALT_ADC_OFF != FALSE)
#if (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT)
        if ( (g_e8MotorStatus & C_MOTOR_STATUS_MASK) == C_MOTOR_STATUS_STOP)
#elif (_SUPPORT_APP_TYPE == C_APP_RELAY)
#if (_SUPPORT_DUAL_RELAY == FALSE)
        if ( (g_e8RelayStatus & (uint8_t)C_RELAY_STATUS_MASK) == (uint8_t)C_RELAY_STATUS_OFF)
#else  /* (_SUPPORT_DUAL_RELAY == FALSE) */
        if ( ((g_e8RelayStatusA & (uint8_t)C_RELAY_STATUS_MASK) == (uint8_t)C_RELAY_STATUS_OFF) &&
             ((g_e8RelayStatusB & (uint8_t)C_RELAY_STATUS_MASK) == (uint8_t)C_RELAY_STATUS_OFF) )
#endif /* (_SUPPORT_DUAL_RELAY == FALSE) */
#elif (_SUPPORT_APP_TYPE == C_APP_SOLENOID)
        if ( (g_e8SolenoidStatus & C_SOLENOID_STATUS_MASK) == C_SOLENOID_STATUS_DEACTIVATED)
#endif /* (_SUPPORT_APP_TYPE) */
        {
            HAL_ADC_PowerOff();
        }
        /* Setup */
#endif /* (_SUPPORT_CPU_HALT_ADC_OFF != FALSE) */
        IO_CTIMER0_CTRL = B_CTIMER0_STOP;
        IO_CTIMER0_CTRL = C_SLEEP_TIMER_CONFIG;                                 /* Timer mode */
        IO_CTIMER0_TREGB = C_SLEEP_TIMER_PERIOD;
        /* Keep Diagnostics IT enabled (MMP220315-1) */
        IO_MLX16_ITC_MASK0_S = (B_MLX16_ITC_MASK0_AWD_ATT                       /* Resume on Absolute Watchdog Attention */
                                | B_MLX16_ITC_MASK0_UV_VDDA                     /* Resume on VDDA UV */
#if defined (__MLX81160__) || defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
                                /* | B_MLX16_ITC_MASK0_UV_VSM */                /* Resume on VSM UV */
#else
                                | B_MLX16_ITC_MASK0_UV_VS                       /* Resume on VS UV */
#endif /* !defined (__MLX81160__) */
#if (_SUPPORT_DIAG_OT != FALSE)
                                | B_MLX16_ITC_MASK0_OVT                         /* Resume on Over-temperature */
#endif /* (_SUPPORT_DIAG_OT != FALSE) */
#if (_SUPPORT_DIAG_OC != FALSE)
#if defined (__MLX81160__)
#if (_SUPPORT_ADC_CUR_SENSE_CH == ADC_CUR_SENSE_CH1) || (_SUPPORT_ADC_CUR_SENSE_CH == ADC_CUR_SENSE_CHX)
                                | B_MLX16_ITC_MASK0_OVC0                        /* Enable Over-current */
#endif /* (_SUPPORT_ADC_CUR_SENSE_CH == ADC_CUR_SENSE_CH1) || (_SUPPORT_ADC_CUR_SENSE_CH == ADC_CUR_SENSE_CHX) */
#if (_SUPPORT_ADC_CUR_SENSE_CH == ADC_CUR_SENSE_CH2) || (_SUPPORT_ADC_CUR_SENSE_CH == ADC_CUR_SENSE_CHX)
                                | B_MLX16_ITC_MASK0_OVC1                        /* Enable Over-current */
#endif /* (_SUPPORT_ADC_CUR_SENSE_CH == ADC_CUR_SENSE_CH2) || (_SUPPORT_ADC_CUR_SENSE_CH == ADC_CUR_SENSE_CHX) */
#else  /* defined (__MLX81160__) */
                                | B_MLX16_ITC_MASK0_OVC                         /* Resume on Over-current */
#endif /* defined (__MLX81160__) */
#endif /* (_SUPPORT_DIAG_OC != FALSE) */
#if (_SUPPORT_DIAG_VDS != FALSE)
                                | B_MLX16_ITC_MASK0_OV_HS_VDS0                  /* Resume on VDS0 HS */
                                | B_MLX16_ITC_MASK0_OV_HS_VDS1                  /* Resume on VDS1 HS */
                                | B_MLX16_ITC_MASK0_OV_HS_VDS2                  /* Resume on VDS2 HS */
                                | B_MLX16_ITC_MASK0_OV_HS_VDS3                  /* Resume on VDS3 HS */
#endif /* (_SUPPORT_DIAG_VDS != FALSE) */
                                );
        IO_MLX16_ITC_MASK1_S = (0
#if (_SUPPORT_DIAG_VDS != FALSE)
                                | B_MLX16_ITC_MASK1_OV_LS_VDS0                  /* Resume on VDS0 LS */
                                | B_MLX16_ITC_MASK1_OV_LS_VDS1                  /* Resume on VDS1 LS */
                                | B_MLX16_ITC_MASK1_OV_LS_VDS2                  /* Resume on VDS2 LS */
                                | B_MLX16_ITC_MASK1_OV_LS_VDS3                  /* Resume on VDS3 LS */
#endif /* (_SUPPORT_DIAG_VDS != FALSE) */
                                | B_MLX16_ITC_MASK1_CTIMER0_3                   /* Resume on CTimer0 (HALT-Resume timer) */
#if (_SUPPORT_HALL_LATCH != FALSE) && (_SUPPORT_NR_OF_HL == 3U) && (_SUPPORT_MICRO_STEP_COMMUTATION != FALSE)
                                | HL_TIMER_MASK_CONF                            /* Resume on HL event (MMP230707-2) */
#endif /* (_SUPPORT_HALL_LATCH != FALSE) && (_SUPPORT_NR_OF_HL == 3U) && (_SUPPORT_MICRO_STEP_COMMUTATION != FALSE) */
                                );
        IO_MLX16_ITC_MASK2_S = (B_MLX16_ITC_MASK2_COLIN_LIN                     /* Resume on LIN */
#if defined (__MLX81160__)
                                | B_MLX16_ITC_MASK2_OV_VSM                      /* Resume on Over-voltage */
#elif defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__)
                                | B_MLX16_ITC_MASK2_OV_VS                       /* Resume on Over-voltage */
#endif /* defined (__MLX81160__) || defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__) */
#if (_SUPPORT_DIAG_DRV_PROT != FALSE)
                                | B_MLX16_ITC_MASK2_DIAG                        /* Resume on Diagnostic */
#endif /* (_SUPPORT_DIAG_DRV_PROT != FALSE) */
#if (_SUPPORT_TRIAXIS_MLX90422 != FALSE) || (_SUPPORT_TRIAXIS_MLX90426 != FALSE)
                                | B_MLX16_ITC_MASK2_PPM_ERR                     /* Resume on PPM-ERR (Sent) (MMP240607-1) */
#endif /* (_SUPPORT_TRIAXIS_MLX90422 != FALSE) || (_SUPPORT_TRIAXIS_MLX90426 != FALSE) */
                                );
#if defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
        IO_MLX16_ITC_MASK3_S = (B_MLX16_ITC_MASK3_OV_VSM                        /* Clear Over-Voltage VS */
                                | B_MLX16_ITC_MASK3_OV_VDDA                     /* Clear Over-Voltage VDDA */
                                | B_MLX16_ITC_MASK3_OV_BOOST                    /* Clear Over-Voltage VBOOST */
#if (_SUPPORT_HALL_LATCH != FALSE) && (_SUPPORT_NR_OF_HL == 3U) && (_SUPPORT_MICRO_STEP_COMMUTATION == FALSE)
                                | B_HL_ITC_MASK_CONF                            /* Resume on Hall-Latch event */
#endif /* (_SUPPORT_HALL_LATCH != FALSE) && (_SUPPORT_NR_OF_HL == 3U) && (_SUPPORT_MICRO_STEP_COMMUTATION == FALSE) */
                                );
#else  /* defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__) */
        IO_MLX16_ITC_MASK3_S = 0U;
#endif /* defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__) */
#if defined (__MLX81160__) || defined (__MLX81332B01__) || defined (__MLX81332B02__) || defined (__MLX81334__) || defined (__MLX81339__) || \
    defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX813446__)
        IO_MLX16_ITC_MASK4_S = 0U;
#endif /* defined (__MLX81332B01__) || defined (__MLX81332B02__) || defined (__MLX81334__) || defined (__MLX81339__) ||\
        * defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__) */

        IO_CTIMER0_CTRL = B_CTIMER0_START;
        /* Stop */
#if (_DEBUG_CPU_HALT != FALSE)
        DEBUG_SET_IO_A();
#endif /* (_DEBUG_CPU_HALT != FALSE) */
        IO_PORT_STOPMD_CTRL_S = B_PORT_STOPMD_CTRL_SEL_STOP_MODE;
        /* __asm__ ("HALT\n\t" :::); */
        /* The above code fails as the LIN message is received just after
         * the LinInFrameBufState check and before the actual entering of
         * the HALT-state. The LIN message is not lost, but delayed by the
         * HALT time-out period. Below code blocks the IRQ's (postpone)
         * until the MLX16 enters HALT-state. (MMP180420-1)
         */
        __asm__ __volatile__ (
#if (LIN_COMM != FALSE)
            "mov Y, M \n\t"
            "clrb MH.2 \n\t"     /* clear Pr part only staring from msbit */
            "clrb MH.1 \n\t"
            "clrb MH.0 \n\t"
            "lod AL, _g_u8LinInFrameBufState \n\t"                              /* Check for LIN message received */
            "jne _HALT_10 \n\t"                                                 /* Skip HALT in case LIN message received */
            "mov M, Y \n\t"
#endif /* (LIN_COMM != FALSE) */
            "HALT \n\t"                                                         /* Enter HALT-state */
#if (LIN_COMM != FALSE)
            "jmp _HALT_20 \n\t"                                                 /* Leave HALT-state */
            "_HALT_10: \n\t"
            "mov M, Y \n\t"
            "_HALT_20:"
#endif /* (LIN_COMM != FALSE) */
            :
            :
            : "A","Y"                                                           /* Add used registers for the C-compiler (MMP181008-1) */
            );
        /* Restore */
#if (_DEBUG_CPU_HALT != FALSE)
        DEBUG_CLR_IO_A();
#endif /* (_DEBUG_CPU_HALT != FALSE) */
        u16TimerCnt = IO_CTIMER0_TCNT;                                          /* Take a copy of the Timer-count value */
        {
            uint16_t u16MiscOut = IO_PORT_MISC_OUT & ~B_PORT_MISC_OUT_CLEAR_STOP;  /* Make sure the CLEAR_STOP bit is cleared (MMP210729-1) */
            IO_PORT_MISC_OUT = u16MiscOut | B_PORT_MISC_OUT_CLEAR_STOP;
            IO_PORT_MISC_OUT = u16MiscOut;
        }
#if (_SUPPORT_CPU_HALT_ADC_OFF != FALSE)
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__)
        IO_PORT_ADC_CTRL_S = B_PORT_ADC_CTRL_ADC_EN;
#endif /* defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__) */
        DELAY_US(10);                                                           /* MMP190311-1: Add delay after re-enabling ADC */
#endif /* (_SUPPORT_CPU_HALT_ADC_OFF != FALSE) */
        if (u16TimerCnt <= 1U)                                                  /* MMP180924-1 */
        {
            /* Assume Timer resume */
            u16TimerCnt += IO_CTIMER0_TREGB;
        }
        IO_CTIMER0_CTRL = B_CTIMER0_STOP;
        IO_CTIMER0_CTRL = (u16Timer0Ctrl & ~(B_CTIMER0_STOP | B_CTIMER0_START));
        /*IO_CTIMER0_CTRL = C_TMRx_CTRL_MODE0; */
        IO_MLX16_ITC_MASK0_S |= u16CopyMask0;                                   /* MMP201203-1: Fix IRQ mask restore due to Diagnostics */
        IO_MLX16_ITC_MASK1_S |= u16CopyMask1;
        IO_MLX16_ITC_MASK2_S |= u16CopyMask2;
        IO_MLX16_ITC_MASK3_S |= u16CopyMask3;
#if defined (__MLX81160__) || defined (__MLX81332B01__) || defined (__MLX81332B02__) || defined (__MLX81334__) || defined (__MLX81339__) || \
    defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
        IO_MLX16_ITC_MASK4_S |= u16CopyMask4;
#endif /* defined (__MLX81332B01__) || defined (__MLX81332B02__) || defined (__MLX81334__) || defined (__MLX81339__) ||\
        * defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__) */
#if (_SUPPORT_APP_USER_MODE != FALSE)
        EXIT_SECTION(); /*lint !e438 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
        TimerSleepCompensation(u16TimerCnt);
    }
} /* End of AppProcPowerSave() */
#endif /* (_SUPPORT_CPU_HALT != FALSE) && (I2C_COMM == FALSE) && (SPI_COMM == FALSE) && (UART_COMM == FALSE) */

#if (_SUPPORT_APP_TYPE == C_APP_RELAY)
/*!*************************************************************************** *
 * ISR_CTIMER0_3
 * \brief   Halt-Result
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details Resume from CPU HALT
 *          IRQ-Priority: 4
 * *************************************************************************** *
 * - Call Hierarchy: IRQ
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 0
 * ************************************************************************** */
__attribute__((interrupt)) void ISR_CTIMER0_3(void)
{
#if (_DEBUG_CPU_LOAD != FALSE)
    DEBUG_SET_IO_B();                                                           /* IRQ-Priority: 4 */
#endif /* (_DEBUG_CPU_LOAD != FALSE) */
#if (_DEBUG_CPU_LOAD != FALSE)
    DEBUG_CLR_IO_B();                                                           /* IRQ-Priority: 4 */
#endif /* (_DEBUG_CPU_LOAD != FALSE) */
} /* End of ISR_CTIMER0_3() */
#endif /* (_SUPPORT_APP_TYPE == C_APP_RELAY) */

#if (_SUPPORT_CRITICAL_PERIPHERAL_CHECK != FALSE)
/*!*************************************************************************** *
 * PendIrqCheck()
 * \brief     Check pending IRQ's
 * \author    mmp
 * *************************************************************************** *
 * \param     [in] pu16Pend: Address of PEND-port
 * \param     [in] pu16Mask: Address of MASK-port
 * \return    (uint16_t) Result of pending IRQ
 * *************************************************************************** *
 * \details   Check if MASK-enabled pending IRQ are present
 * Note: As the IRQ may occur just before the actual read-out of the PEND-register,
 * the interrupt bit may be present; Therefore a second check of the same PEND-bit
 * required and should not have any IRQ present.
 * *************************************************************************** *
 * - Call Hierarchy: AppChipCheck()
 * - Cyclomatic Complexity: 1+1
 * - Nesting: 1
 * - Function calling: 0
 * *************************************************************************** */
static inline uint16_t PendIrqCheck(volatile uint16_t *pu16Pend, volatile uint16_t *pu16Mask)
{
    uint16_t u16PendingIRQ;

    u16PendingIRQ = (*pu16Pend & *pu16Mask);
    if (u16PendingIRQ != 0U)
    {
        u16PendingIRQ = u16PendingIRQ & *pu16Pend;
    }
    return (u16PendingIRQ);
} /* End of PendIrqCheck() */

/*!*************************************************************************** *
 * AppChipCheck()
 * \brief     Application/Chip Check
 * \author    mmp
 * *************************************************************************** *
 * \param     -
 * \return    -
 * *************************************************************************** *
 * \details   Check Chip CPU and IRQ priorities and application critical IO-Ports
 * - Check IRQ Priorities (MASK & PEND)
 * - Check CPU Priority
 * - Check Critical IO-ports
 * TODO: Reset chip (application) in case of IRQ missed
 * *************************************************************************** *
 * - Call Hierarchy: AppBackgroundTaskHandler()
 * - Cyclomatic Complexity: 7+1
 * - Nesting: 2
 * - Function calling: 3 (PendIrqCheck(), SetLastError(), TimerCheck())
 * *************************************************************************** */
static void AppChipCheck(void)
{
    uint16_t u16PendingIRQ;

    /* Check Interrupt priorities */
    u16PendingIRQ = PendIrqCheck(&IO_MLX16_ITC_PEND0_S, &IO_MLX16_ITC_MASK0_S);
    if (u16PendingIRQ != 0U)
    {
        /* Some IRQ's are not handled */
#if (_SUPPORT_LOG_ERRORS != FALSE)
        SetLastError(C_ERR_PRIO | (C_ERR_EXT | 0x0100U));
        SetLastError(u16PendingIRQ);
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
    }
    u16PendingIRQ = PendIrqCheck(&IO_MLX16_ITC_PEND1_S, &IO_MLX16_ITC_MASK1_S);
#if (_SUPPORT_PWM_SYNC != FALSE) || (defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__))
    if (u16PendingIRQ != 0U)
#else  /* (_SUPPORT_PWM_SYNC != FALSE) */
    if ( (u16PendingIRQ & ~B_MLX16_ITC_PEND1_PWM_MASTER1_END) != 0U)            /* PWM_MASTER1_END Pend is accepted */
#endif /* (_SUPPORT_PWM_SYNC != FALSE) */
    {
        /* Some IRQ's are not handled */
#if (_SUPPORT_LOG_ERRORS != FALSE)
        SetLastError(C_ERR_PRIO | (C_ERR_EXT | 0x0200U));
        SetLastError(u16PendingIRQ);
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
    }
    u16PendingIRQ = PendIrqCheck(&IO_MLX16_ITC_PEND2_S, &IO_MLX16_ITC_MASK2_S);
#if (_SUPPORT_PWM_SYNC != FALSE) || (defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__))
    if (u16PendingIRQ != 0U)
#else  /* (_SUPPORT_PWM_SYNC != FALSE) */
    if ( (u16PendingIRQ & ~B_MLX16_ITC_PEND2_PWM_MASTER1_END) != 0U)            /* PWM_MASTER1_END Pend is accepted */
#endif /* (_SUPPORT_PWM_SYNC != FALSE) */
    {
        /* Some IRQ's are not handled */
#if (_SUPPORT_LOG_ERRORS != FALSE)
        SetLastError(C_ERR_PRIO | (C_ERR_EXT | 0x0300U));
        SetLastError(u16PendingIRQ);
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
    }
    u16PendingIRQ = PendIrqCheck(&IO_MLX16_ITC_PEND3_S, &IO_MLX16_ITC_MASK3_S);
    if (u16PendingIRQ != 0U)
    {
        /* Some IRQ's are not handled */
#if (_SUPPORT_LOG_ERRORS != FALSE)
        SetLastError(C_ERR_PRIO | (C_ERR_EXT | 0x0400U));
        SetLastError(u16PendingIRQ);
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
    }
#if defined (__MLX81160__) || defined (__MLX81332B01__) || defined (__MLX81332B02__) || defined (__MLX81334__) || \
    defined (__MLX81339__) || defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
    u16PendingIRQ = PendIrqCheck(&IO_MLX16_ITC_PEND4_S, &IO_MLX16_ITC_MASK4_S);
    if (u16PendingIRQ != 0U)
    {
        /* Some IRQ's are not handled */
#if (_SUPPORT_LOG_ERRORS != FALSE)
        SetLastError(C_ERR_PRIO | (C_ERR_EXT | 0x0500U));
        SetLastError(u16PendingIRQ);
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
       /* TODO: Reset chip (application) */
    }
#endif /* defined (__MLX81160__) || defined (__MLX81332B01__) || defined (__MLX81332B02__) || defined (__MLX81334__) || defined (__MLX81339__) ||\
        * defined (__MLX81339__) || defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__) */
#if defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
    u16PendingIRQ = PendIrqCheck(&IO_MLX16_ITC_PEND5_S, &IO_MLX16_ITC_MASK5_S);
    if (u16PendingIRQ != 0U)
    {
        /* Some IRQ's are not handled */
#if (_SUPPORT_LOG_ERRORS != FALSE)
        SetLastError(C_ERR_PRIO | (C_ERR_EXT | 0x0600U));
        SetLastError(u16PendingIRQ);
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
       /* TODO: Reset chip (application) */
    }
#endif /* defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__) */

#if (LIN_COMM != FALSE)
#if ((CAN_COMM != FALSE) || (PWM_COMM != FALSE) || (SPI_COMM != FALSE) || \
    ((_SUPPORT_UART != FALSE) && (_SUPPORT_UART_IF_APP == C_UART_IF_QUAD_WHEEL_CAR)))
    if (g_u8LinConnected != C_COMM_DISCONNECTED)
#endif /* ((CAN_COMM != FALSE) || (PWM_COMM != FALSE) || (SPI_COMM != FALSE) || ((_SUPPORT_UART != FALSE) && (_SUPPORT_UART_IF_APP == C_UART_IF_QUAD_WHEEL_CAR))) */
    {
        /* MLX4 Second-level IRQ controller (MMP191030-3) */
        if ( (ml_GetSLVIT() & (uint8_t)0x01U) == (uint8_t)0x0U)
        {
            /* MLX4 Slave IT are disabled */
            ml_SetSLVIT(0xABU);                                                 /* Re-enabled IT */
#if (_SUPPORT_LOG_ERRORS != FALSE)
            SetLastError(C_ERR_PRIO | (C_ERR_EXT | 0x0E00U));
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
        }
    }
#endif /* (LIN_COMM != FALSE) */

    /* CPU priority check (MMP190920-1) */
    {
        uint16_t u16CpuStatus = builtin_mlx16_get_status();
        if ( (u16CpuStatus & 0x0700U) != 0x0700U)
        {
            /* Main-loop priority is incorrect; This may cause that other priority tasks (IRQ's) are blocked */
#if (_SUPPORT_APP_USER_MODE != FALSE)
            builtin_mlx16_set_priority(15U);
#else  /* (_SUPPORT_APP_USER_MODE != FALSE) */
            builtin_mlx16_set_priority(7U);
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
#if (_SUPPORT_LOG_ERRORS != FALSE)
            SetLastError(C_ERR_PRIO | (C_ERR_EXT | 0x0F00U));
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
        }
    }

    /* Check application critical IO-ports */
    /* Check background timer */
    (void)TimerCheck();
} /* End of AppChipCheck() */
#endif /* (_SUPPORT_CRITICAL_PERIPHERAL_CHECK != FALSE) */

/*!*************************************************************************** *
 * AppBackgroundHandler()
 * \brief   Handle Generic Back-ground tasks
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details Following background tasks are performed:
 *          - LIN Co-processor check
 *          - Application Memory check
 *          - Application Temperature profile check
 *          - Application Processor Power-save mode
 * *************************************************************************** *
 * - Call Hierarchy: main()
 * - Cyclomatic Complexity: 1+1
 * - Nesting: 1
 * - Function calling: 5 ( AppCheckLinProc(), AppMemoryCheck(),
 *                         AppTemperatureProfileCheck(), AppProcPowerSave(), AppCheck())
 * *************************************************************************** */
void AppBackgroundHandler(void)
{
#if (LIN_COMM != FALSE) && (_SUPPORT_LIN_BUS_ACTIVITY_CHECK != FALSE)
    /* ********************** */
    /* *** n. MLX4 status *** */
    /* ********************** */
#if (LIN_COMM != FALSE) && ((CAN_COMM != FALSE) || (PWM_COMM != FALSE) || (SPI_COMM != FALSE) || \
    ((_SUPPORT_UART != FALSE) && (_SUPPORT_UART_IF_APP == C_UART_IF_QUAD_WHEEL_CAR)))
    if (g_u8LinConnected != C_COMM_DISCONNECTED)
#endif /* (LIN_COMM != FALSE) && ((CAN_COMM != FALSE) || (PWM_COMM != FALSE) || (SPI_COMM != FALSE) || ((_SUPPORT_UART != FALSE) && ((_SUPPORT_UART_IF_APP == C_UART_IF_QUAD_WHEEL_CAR))) */
    {
        AppCheckLinProc();
    }
#endif /* (LIN_COMM != FALSE) && (_SUPPORT_LIN_BUS_ACTIVITY_CHECK != FALSE) */

#if (_SUPPORT_BG_MEM_TEST != FALSE) && ((_SUPPORT_BG_FLASH_TEST != FALSE) || (_SUPPORT_BG_RAM_TEST != FALSE) || (_SUPPORT_BG_ROM_TEST != FALSE))
    /* ********************************** */
    /* *** o. Background System check *** */
    /* ********************************** */
    AppMemoryCheck();
#endif /* (_SUPPORT_BG_MEM_TEST != FALSE) && ((_SUPPORT_BG_FLASH_TEST != FALSE) || (_SUPPORT_BG_RAM_TEST != FALSE) || (_SUPPORT_BG_ROM_TEST != FALSE)) */

#if (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
    /* ****************************** */
    /* *** (Optional) SPI support *** */
    /* ****************************** */
    if ( ((g_e8MotorStatus & C_MOTOR_STATUS_RUNNING) == 0U) &&                  /* Motor stopped */
         (l_u8AdcMode == (uint8_t)C_ADC_MODE_IDLE))                             /* ADC not in use */
    {
        MeasureResolverPos();
    }
    GetResolverPosition();
#endif /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */

    /* ***************************************************** */
    /* *** q. Chip temperature stability (profile) check *** */
    /* ***************************************************** */
    AppTemperatureProfileCheck(); /*lint !e522 */

#if (_SUPPORT_CRITICAL_PERIPHERAL_CHECK != FALSE)
    /* Application check (MMP190920-1) */
    AppChipCheck();
#endif /* (_SUPPORT_CRITICAL_PERIPHERAL_CHECK != FALSE) */

#if (_SUPPORT_CPU_HALT != FALSE) && (I2C_COMM == FALSE) && (SPI_COMM == FALSE) && (UART_COMM == FALSE)
    /* ************************************* */
    /* *** r. Power-saving (non-running) *** */
    /* ************************************* */
    AppProcPowerSave();
#endif /* (_SUPPORT_CPU_HALT != FALSE) && (I2C_COMM == FALSE) && (SPI_COMM == FALSE) && (UART_COMM == FALSE) */

} /* End of AppBackgroundHandler() */

/*!*************************************************************************** *
 * AppStop()
 * \brief   Stop the application
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details Stop the application, by:
 *          - Stopping the actuator (without holding current)
 *          - Stop all DMA's
 *            - ADC
 *            - SPI
 *            - UART
 *            - I2C
 *          - Block all IRQ's, except LIN
 * *************************************************************************** *
 * - Call Hierarchy: AppSleep(), I2C_HandleChipReset()
 * - Cyclomatic Complexity: 2+1
 * - Nesting: 1
 * - Function calling: 4-7 (MotorDriverStop(), SetLastError(),
 *                          HAL_ADC_StopSafe(), SPI_Disconnect(), UART_Stop(), UART2_Stop(),
 *                          I2C_MasterExit(), I2C_SlaveStop())
 * *************************************************************************** */
void AppStop(void)
{
    /* Stop motor (e.g. disconnect drivers) */
#if (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT)
    MotorDriverStop( (uint16_t)C_STOP_WO_HOLDING);                              /* Application stop */
#elif (_SUPPORT_APP_TYPE == C_APP_RELAY)
    RelayDriverOff();                                                           /* Application stop */
#endif /* (_SUPPORT_APP_TYPE) */
#if (_SUPPORT_LOG_ERRORS != FALSE)
    SetLastError(C_ERR_APPL_STOP);
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */

    /* Stop DMA's */
    HAL_ADC_StopSafe();
#if ((_SUPPORT_SPI != FALSE) || (_SUPPORT_3WIRE_SPI != FALSE)) && (_SUPPORT_SPI_DMA != FALSE)
    SPI_Disconnect();
#endif /* ((_SUPPORT_SPI != FALSE) || (_SUPPORT_3WIRE_SPI != FALSE)) && (_SUPPORT_SPI_DMA != FALSE) */
#if (_SUPPORT_UART != FALSE)
#if (_SUPPORT_UART_IF_APP != C_UART_IF_NONE)                                    /* (MMP230810-1) */
    UART_Stop();
#endif /* (_SUPPORT_UART_IF_APP != C_UART_IF_NONE)  */
#if (_SUPPORT_UART2_IF_APP != C_UART_IF_NONE)                                   /* (MMP230810-1) */
    UART2_Stop();
#endif /* (_SUPPORT_UART2_IF_APP != C_UART_IF_NONE)  */
#endif /* (_SUPPORT_UART != FALSE) */
#if (_SUPPORT_I2C != FALSE)
#if (_SUPPORT_I2C_MASTER != FALSE)
    I2C_MasterExit();
#endif /* (_SUPPORT_I2C_MASTER != FALSE) */
#if (_SUPPORT_I2C_SLAVE != FALSE)
    I2C_SlaveStop();
#endif /* (_SUPPORT_I2C_SLAVE != FALSE) */
#endif /* (_SUPPORT_I2C != FALSE) */
    IO_PORT_PPM_CTRL &= ~B_PORT_PPM_CTRL_PPM_EN;                                /* Stop PPM module */

    /* Disable all IRQ's, except LIN */
#if (_SUPPORT_APP_USER_MODE != FALSE)
    ENTER_SECTION(SYSTEM_MODE); /*lint !e534 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
    IO_MLX16_ITC_MASK0_S = 0U;
    IO_MLX16_ITC_MASK1_S = 0U;
#if (LIN_COMM != FALSE)
    IO_MLX16_ITC_MASK2_S = B_MLX16_ITC_MASK2_COLIN_LIN;                         /* Disable all interrupts, except COLIN_LIN */
#else  /* (LIN_COMM != FALSE) */
    IO_MLX16_ITC_MASK2_S = 0U;                                                  /* Disable all interrupts */
#endif /* (LIN_COMM != FALSE) */
    IO_MLX16_ITC_MASK3_S = 0U;
#if defined (__MLX81160__) || defined (__MLX81332B01__) || defined (__MLX81332B02__) || defined (__MLX81334__) || defined (__MLX81339__)
    IO_MLX16_ITC_MASK4_S = 0U;
#elif defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
    IO_MLX16_ITC_MASK4_S = 0U;
    IO_MLX16_ITC_MASK5_S = 0U;
#endif
#if (_SUPPORT_APP_USER_MODE != FALSE)
    EXIT_SECTION(); /*lint !e438 !e529 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
#if (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT)
    g_e8MotorStatus = C_MOTOR_STATUS_APPL_STOP;                                 /* Don't perform periodic MLX4 Status checks */
#elif (_SUPPORT_APP_TYPE == C_APP_RELAY)
#if (_SUPPORT_DUAL_RELAY == FALSE)
    g_e8RelayStatus = C_MOTOR_STATUS_APPL_STOP;                                 /* Don't perform periodic MLX4 Status checks */
#else  /* (_SUPPORT_DUAL_RELAY == FALSE) */
    g_e8RelayStatusA = C_MOTOR_STATUS_APPL_STOP;                                /* Don't perform periodic MLX4 Status checks */
    g_e8RelayStatusB = C_MOTOR_STATUS_APPL_STOP;                                /* Don't perform periodic MLX4 Status checks */
#endif /* (_SUPPORT_DUAL_RELAY == FALSE) */
#endif /* (_SUPPORT_APP_TYPE) */
} /* End of AppStop */

/*!*************************************************************************** *
 * AppSleepWithWakeUpTimer()
 * \brief   Application into Sleep-mode with activated Wake-up Timer
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details
 * *************************************************************************** *
 * - Call Hierarchy: HandleDiagnosticsVDDA()
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 1 (AppSleep())
 * *************************************************************************** */
void AppSleepWithWakeUpTimer(void)
{
    IO_PORT_MISC_OUT = (IO_PORT_MISC_OUT & ~M_PORT_MISC_OUT_WUI) | C_PORT_MISC_OUT_WUI_400ms;
    AppSleep();
} /* End of AppSleepWithWakeUpTimer() */

#if (LIN_COMM != FALSE) && (_SUPPORT_LIN_SLEEP != FALSE)
void AppSleep( void) __attribute__((noreturn));
/*!*************************************************************************** *
 * AppSleep()
 * \brief   Application into Sleep-mode
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details Enter the IC (deep) SLEEP mode (with or without RAM-retention)
 *          Before entering SLEEP mode, stop the application, stop DMA access,
 *          disable all IRQ's, stop COLIN (MLX4), and finish any pending Non Volatile Memory
 *          update.
 *          LIN resume (wake-up) is always enabled; I/O and/or Wake-up timer
 *          resume is by default disabled and has to be configured accordingly.
 * *************************************************************************** *
 * - Call Hierarchy: HandleSleepMotorRequest()
 * - Cyclomatic Complexity: 2+1
 * - Nesting: 1
 * - Function calling: 3 (mlu_ApplicationStop(), builtin_mlx16_disable_interrupts(),
 *                        ml_ResetDrv())
 * *************************************************************************** */
void AppSleep(void)
{
#if (_SUPPORT_WHILE_RETRY_LIMITED != FALSE)
    uint16_t u16Retries;
#endif /* (_SUPPORT_WHILE_RETRY_LIMITED != FALSE) */

    /* Stop the application, including DMA accesses and interrupts (MMP230531-1) */
    AppStop();

    builtin_mlx16_disable_interrupts();                                         /* Disable the interrupts */

#if (_SUPPORT_APP_SAVE != FALSE) && (_DEBUG_FLASH_WRITE_CYCLES == FALSE)
#if (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE)
    NV_AppStore(g_u16ActualPosition, g_e8ErrorElectric);
#else
    NV_AppStore(g_u16ActualMotorSpeedRPM, g_e8ErrorElectric);
#endif
#endif /* (_SUPPORT_APP_SAVE != FALSE) && (_DEBUG_FLASH_WRITE_CYCLES == FALSE) && (_SUPPORT_NV_EMERGENCY_STORE == FALSE) */

#if (LIN_COMM != FALSE)
#if (_SUPPORT_APP_USER_MODE != FALSE)
    ENTER_SECTION(SYSTEM_MODE); /*lint !e534 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
    ml_ResetDrv();
#if (_SUPPORT_APP_USER_MODE != FALSE)
    EXIT_SECTION(); /*lint !e438 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
#endif /* (LIN_COMM != FALSE) */

#if (_SUPPORT_WAKEUP_BY_IO != FALSE)                                            /* MMP211105-2 */
    IO_PORT_MISC2_OUT |= B_PORT_MISC2_OUT_WU_IO_EN;                             /* Enable Wake-up by IO */
#endif /* (_SUPPORT_WAKEUP_BY_IO != FALSE) */

#if (_SUPPORT_NV_TYPE == C_NV_EEPROM)
    /* Check EEPROM update Busy */
#if (_SUPPORT_WHILE_RETRY_LIMITED != FALSE)
    u16Retries = 100U;
#endif /* (_SUPPORT_WHILE_RETRY_LIMITED != FALSE) */
    while (IO_GET(EEPROM_FLASH, EE_BUSY) != 0U)
    {/* Wait till the Non Volatile Memory will be started TODO: What if not? */
#if (_SUPPORT_WHILE_RETRY_LIMITED != FALSE)
        if (--u16Retries == 0U)
        {
#if (_SUPPORT_LOG_ERRORS != FALSE)
            SetLastError(C_ERR_TO_NV_SLEEP);
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
            break;
        }
        DELAY_US(150U);
#endif /* (_SUPPORT_WHILE_RETRY_LIMITED != FALSE) */
    };
#endif /* (_SUPPORT_NV_TYPE == C_NV_EEPROM) */

    /* Check Flash update Busy */
#if (_SUPPORT_WHILE_RETRY_LIMITED != FALSE)
    u16Retries = 100U;
#endif /* (_SUPPORT_WHILE_RETRY_LIMITED != FALSE) */
#if defined (__MLX81339__)
#ifdef HAS_FLASH_KF
    while ( ((IO_FLASH_KFHP13_CMD_STS_S & M_FLASH_KFHP13_STATUS) == FLASH_STATUS_PAGE_ERASE) ||
            ((IO_FLASH_KFHP13_CMD_STS_S & M_FLASH_KFHP13_STATUS) == FLASH_STATUS_PAGE_PROGRAM) ) {}
#else  /* HAS_FLASH_KF */
    while ( ((IO_FLASH_KFHP13_CMD_STS_S & M_FLASH_KFHP13_STATUS) == FLASH_STATUS_SECTOR_ERASE) ||
            ((IO_FLASH_KFHP13_CMD_STS_S & M_FLASH_KFHP13_STATUS) == FLASH_STATUS_PAGE_PROGRAM) ) {}
#endif /* HAS_FLASH_KF */
#else  /* defined (__MLX81339__) */
    while ((IO_GET(EEPROM_FLASH, FL_STATUS) == C_EEPROM_FLASH_FL_STATUS_PAGE_PROGRAM) ||   /* Wait till the EEPROM will be started TODO: What if not? */
           (IO_GET(EEPROM_FLASH, FL_STATUS) == C_EEPROM_FLASH_FL_STATUS_SECTOR_ERASE) )
#endif /* defined (__MLX81339__) */
    {
#if (_SUPPORT_WHILE_RETRY_LIMITED != FALSE)
        if (--u16Retries == 0U)
        {
#if (_SUPPORT_LOG_ERRORS != FALSE)
            SetLastError(C_ERR_TO_FL_SLEEP);
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
            break;
        }
        DELAY_US(150U);
#endif /* (_SUPPORT_WHILE_RETRY_LIMITED != FALSE) */
    };

    ENTER_SECTION(ATOMIC_SYSTEM_MODE); /*lint !e534 */
#if (_SUPPORT_SLEEP_RAM_RETENTION != FALSE)
    allowWarmReboot();                                                          /* Fast reset without (Flash, ROM and) RAM check */
    {
        uint16_t u16MiscOut = IO_PORT_MISC_OUT & ~B_PORT_MISC_OUT_CLEAR_STOP;   /* Make sure the CLEAR_STOP bit is cleared (MMP210729-1) */
        IO_PORT_MISC_OUT = u16MiscOut | B_PORT_MISC_OUT_CLEAR_STOP;
        IO_PORT_MISC_OUT = u16MiscOut;
    }
    IO_PORT_STOPMD_CTRL_S = B_PORT_STOPMD_CTRL_SEL_STOP_MODE;                   /* Sleep with RAM preserved (~250uA) */
#else  /* (_SUPPORT_SLEEP_RAM_RETENTION != FALSE) */
    IO_PORT_STOPMD_CTRL_S &= ~B_PORT_STOPMD_CTRL_SEL_STOP_MODE;                 /* Deep-sleep (~10uA) */
#endif /* (_SUPPORT_SLEEP_RAM_RETENTION != FALSE) */

    __asm__ ("HALT\n\t" :::);
    EXIT_SECTION(); /*lint !e438 */
    __builtin_unreachable();  /*lint !e526 */
} /* End of AppSleep() */
#endif /* (LIN_COMM != FALSE) && (_SUPPORT_LIN_SLEEP != FALSE) */

void AppReset( void) __attribute__((noreturn));
/*!*************************************************************************** *
 * AppReset()
 * \brief   Reset Application (and IC)
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details
 * *************************************************************************** *
 * - Call Hierarchy:
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 2 (ml_ResetDrv(), allowWarmReboot())
 * *************************************************************************** */
void AppReset(void)
{
#if (LIN_COMM != FALSE)
    (void)mlu_ApplicationStop();
#endif /* (LIN_COMM != FALSE) */

#if (_SUPPORT_APP_USER_MODE != FALSE)
    ENTER_SECTION(SYSTEM_MODE); /*lint !e534 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
#if (LIN_COMM != FALSE)
    ml_ResetDrv();                                                              /* Reset the Mlx4   */
    g_u8LinInFrameBufState = (uint8_t)C_LIN_IN_FREE;
#endif /* (LIN_COMM != FALSE) */
    allowWarmReboot();                                                          /* Fast reset without Flash, ROM and RAM check */
    IO_RST_CTRL_S = B_RST_CTRL_SOFT_RESET;
    /* MLX16_RESET_WARM(); */
#if (_SUPPORT_APP_USER_MODE != FALSE)
    EXIT_SECTION(); /*lint !e438 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
    __builtin_unreachable();  /*lint !e526 */
} /* End of AppReset() */

/* EOF */
