/*!************************************************************************** *
 * \file        Timer.c
 * \brief       MLX813xx Core Timer handling
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
 *           -# TimerInit()
 *           -# TimerStop()
 *           -# TimerCheck()
 *           -# TimerSleepCompensation()
 *           -# TIMER_IT()
 *
 *
 * \copyright   MELEXIS Microelectronic Integrated Systems
 *
 * Copyright (C) 2017-2023 Melexis N.V.
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

#include "main.h"                                                               /* main support (PWM_FREQ) */

/* Driver */
#include "drivelib/Diagnostic.h"                                                /* Chip Protection & Diagnostics support */
#include "drivelib/ErrorCodes.h"                                                /* Support error logging */
#include "drivelib/GlobalVars.h"                                                /* Global Variables support */
#if (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT)
#include "drivelib/MotorDriver.h"                                               /* Motor-driver support */
#if (_SUPPORT_WINDMILL != FALSE)
#include "drivelib/MotorWindmill.h"                                             /* Motor wind-mill support */
#endif /* (_SUPPORT_WINDMILL != FALSE) */
#include "drivelib/PID_Control.h"                                               /* PID support */
#elif (_SUPPORT_APP_TYPE == C_APP_SOLENOID)
#include "drivelib/PID_Control.h"                                               /* PID support */
#include "drivelib/SolenoidDriver.h"                                            /* Solenoid Driver support */
#endif /* (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT) */
#include "drivelib/Timer.h"                                                     /* Simple Timer support */
#include "camculib/private_mathlib.h"                                           /* Private Math Library support */

#include "hal_lib/hal_STimer.h"                                                 /* STimer HAL support */

/* Communication */
#if (CAN_COMM != FALSE)
#include "commlib/CAN_Communication.h"                                          /* CAN support */
#endif /* (CAN_COMM != FALSE) */
#if (I2C_COMM != FALSE) && (_SUPPORT_I2C_RW_ISR == FALSE)
#include "commlib/I2C_Generic.h"                                                /* I2C Generic support */
#endif /* (I2C_COMM != FALSE) && (_SUPPORT_I2C_RW_ISR == FALSE) */
#if (_SUPPORT_IO_DUT_SELECT_HVIO != FALSE) || (_SUPPORT_IO_DUT_SELECT_PWR != FALSE) || (_SUPPORT_IO_CMD_SELECT_LVIO != FALSE)
#include "commlib/IO_Select.h"                                                  /* IO-Select support */
#endif /* (_SUPPORT_IO_DUT_SELECT_HVIO != FALSE) || (_SUPPORT_IO_DUT_SELECT_PWR != FALSE) || (_SUPPORT_IO_CMD_SELECT_LVIO != FALSE) */
#if (LIN_COMM != FALSE)
#include "commlib/LIN_Diagnostics.h"                                            /* LIN Diagnostics support */
#if (_SUPPORT_LIN_AA != FALSE)
#include "commlib/LIN_AutoAddressing.h"                                         /* LIN Auto-Addressing support */
#endif /* (_SUPPORT_LIN_AA != FALSE) */
#endif /* (LIN_COMM != FALSE) */
#if (_SUPPORT_UART != FALSE) && ((_SUPPORT_UART_IF_APP == C_UART_IF_SCOPE) || \
    (_SUPPORT_UART_IF_APP == C_UART_IF_QUAD_WHEEL_CAR) || (_SUPPORT_UART_IF_APP == C_UART_IF_ACTUATOR) || \
    (_SUPPORT_UART_IF_APP == C_UART_IF_BLUETOOTH) || (_SUPPORT_UART2_IF_APP == C_UART_IF_BLUETOOTH) || \
    (_SUPPORT_UART_IF_APP == C_UART_IF_TERMINAL) || (_SUPPORT_UART2_IF_APP == C_UART_IF_TERMINAL) || /* (MMP230810-1) */ \
    (_SUPPORT_UART_IF_APP == C_UART_IF_MLX_ACT))
#include "commlib/UART.h"                                                       /* UART support */
#endif /* (_SUPPORT_UART != FALSE) && ((_SUPPORT_UART_IF_APP == C_UART_IF_SCOPE) || (_SUPPORT_UART_IF_APP == C_UART_IF_QUAD_WHEEL_CAR) || (_SUPPORT_UART_IF_APP == C_UART_IF_BLUETOOTH) || (_SUPPORT_UART2_IF_APP == C_UART_IF_BLUETOOTH) || (_SUPPORT_UART_IF_APP == C_UART_IF_ACTUATOR)) */

/* Sensor(s) */
#if (_SUPPORT_HUMIDITY_HDC302x != FALSE)
#include "senselib/Humidity_TI_HDC302x.h"                                       /* (I2C) Humidity TI HDC302x Sensor support */
#endif /* (_SUPPORT_HUMIDITY_HDC302x != FALSE) */

/* Platform */
#include <atomic.h>

/*!*************************************************************************** */
/*                           GLOBAL & LOCAL VARIABLES                          */
/* *************************************************************************** */
#pragma space dp                                                                /* __TINY_SECTION__ */
#pragma space none                                                              /* __TINY_SECTION__ */

#pragma space nodp                                                              /* __NEAR_SECTION__ */
#pragma space none                                                              /* __NEAR_SECTION__ */

/*!*************************************************************************** *
 * TimerInit
 * \brief   Initialise the simple timer, at a periodic rate of 500us
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details configure Simple timer as background timer at 500us intervals and use
 *          priority to '6' (lowest) and enable IRQ
 * *************************************************************************** *
 * - Call Hierarchy: main_Init()
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 0
 * *************************************************************************** */
void TimerInit(void)
{
    HAL_STimer_Init();
} /* End of Timer_Init() */

/*!*************************************************************************** *
 * TimerStop
 * \brief   Stop the Simple timer, and disable the IRQ
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details Stop background timer by stopping the timer itself and disabling the
 *          interrupt
 * *************************************************************************** *
 * - Call Hierarchy: -
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 0
 * *************************************************************************** */
void TimerStop(void)
{
    HAL_STimer_Stop();
} /* End of TimerStop() */

/*!*************************************************************************** *
 * TimerCheck
 * \brief   Check Timer operation
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  (uint16_t) Status: C_ERR_APP_OK: Okay
 *                             C_ERR_APP_RST: Not okay (reset required)
 * *************************************************************************** *
 * \details Check if background timer still enabled and has the correct priority.
 *          If not, re-initialise the background timer
 * *************************************************************************** *
 * - Call Hierarchy: AppChipCheck()
 * - Cyclomatic Complexity: 1+1
 * - Nesting: 1
 * - Function calling: 2 ( SetLastError(), TimerInit())
 * *************************************************************************** */
uint16_t TimerCheck(void)
{
    if ( HAL_STimer_Check() != FALSE )
    {
#if (_SUPPORT_LOG_ERRORS != FALSE)
        SetLastError(C_ERR_PRIO | (C_ERR_EXT | 0x0A00U));
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
        TimerInit();
    }
    return (C_ERR_APP_OK);
} /* End of TimerCheck() */

#if (_SUPPORT_CPU_HALT != FALSE) && (I2C_COMM == FALSE) && (SPI_COMM == FALSE) && (UART_COMM == FALSE)
/*!*************************************************************************** *
 * TimerSleepCompensation
 * \brief   Compensate the various timer-counters for the sleep-period
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16SleepCompensationPeriod [PLL/CTimerDiv]
 * \return  -
 * *************************************************************************** *
 * \details Update background timer after CPU HALT period (sleep-compensation).
 * *************************************************************************** *
 * - Call Hierarchy: AppProcPowerSave()
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 4 (MotorDriverPeriodicTimer(), PID_CtrlCounter(),
 *                        LinDiagResponseTimeoutCount(), main_PeriodicTimerEvent())
 * *************************************************************************** */
void TimerSleepCompensation(uint16_t u16SleepCompensationPeriod)
{
    /* 256 = CTimer0 divider */
    uint16_t u16SleepPeriod =
        p_MulDivU16_U16byU16byU16(u16SleepCompensationPeriod, 256U,
                                  (uint16_t)(CT_PERIODIC_RATE * (PLL_FREQ / 1000000U)));
    ENTER_SECTION(ATOMIC_KEEP_MODE); /*lint !e534 */
    {
/*lint !e436 */
#if (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT)
        MotorDriverPeriodicTimer(u16SleepPeriod);

        PID_CtrlCounter(u16SleepPeriod);                                        /* PID Current/Speed control */
#elif (_SUPPORT_APP_TYPE == C_APP_SOLENOID)
        SolenoidDriverPeriodicTimer();
        PID_CtrlCounter(u16SleepPeriod);                                        /* PID Current/Speed control */
/*lint !e436 */
#endif /* (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT) */

/*lint !e436 */
#if (LIN_COMM != FALSE)
        LinDiagResponseTimeoutCount(u16SleepPeriod);
/*lint !e436 */
#endif /* (LIN_COMM != FALSE) */

/*lint !e436 */
#if (CAN_COMM != FALSE)
        CanPeriodicTimerEvent(u16SleepPeriod);
/*lint !e436 */
#endif /* (CAN_COMM != FALSE) */

        main_PeriodicTimerEvent(u16SleepPeriod);

/*lint !e436 */
#if (_SUPPORT_HUMIDITY_HDC302x != FALSE)
        HumidityPeriodicTimerevent(u16SleepPeriod);
/*lint !e436 */
#endif /* (_SUPPORT_HUMIDITY_HDC302x != FALSE) */
    }
    EXIT_SECTION(); /*lint !e438 */
} /* End of TimerSleepCompensation() */
#endif /* (_SUPPORT_CPU_HALT != FALSE) && (I2C_COMM == FALSE) && (SPI_COMM == FALSE) && (UART_COMM == FALSE) */

/*!*************************************************************************** *
 * ISR_STIMER
 * \brief   Periodic (Simple) Timer ISR
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details IRQ-Priority: 6
 * *************************************************************************** *
 * - Call Hierarchy: IRQ
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 6..11 (MotorDriverPeriodicTimer(), PID_CtrlCounter(),
 *                            LinDiagResponseTimeoutCount(), LinAutoAddressingTimer(),
     *                        main_PeriodicTimerEvent(), DiagnosticPeriodicTimerEvent(),
 *   (optional): MotorWindMillPeriodicTimer(), I2C_Tick(), CanPeriodicTimerEvent(),
 *                            IO_PeriodicTimer(), UART_PeriodicTimer())
 * *************************************************************************** */
__attribute__((interrupt)) void ISR_STIMER(void)
{
#if (_DEBUG_CPU_LOAD != FALSE)
    DEBUG_SET_IO_D();                                                           /* IRQ-Priority: 6 */
#endif /* (_DEBUG_CPU_LOAD != FALSE) */
#if (_DEBUG_STIMER_ISR != FALSE)
    DEBUG_SET_IO_A();
#endif /* (_DEBUG_STIMER_ISR != FALSE) */

#if (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT)
    MotorDriverPeriodicTimer(1U);

#if (_SUPPORT_WINDMILL != FALSE)
    MotorWindMillPeriodicTimer();
#endif /* (_SUPPORT_WINDMILL != FALSE) */

    PID_CtrlCounter(1U);                                                        /* PID Current/Speed control */
#elif (_SUPPORT_APP_TYPE == C_APP_SOLENOID)
    SolenoidDriverPeriodicTimer();
    PID_CtrlCounter(1U);                                                        /* PID Current/Speed control */
#endif /* (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT) */

#if (I2C_COMM != FALSE) && (_SUPPORT_I2C_SLAVE != FALSE) && (_SUPPORT_I2C_RW_ISR == FALSE)
    I2C_PeriodicTimer();
#endif /* (I2C_COMM != FALSE) && (_SUPPORT_I2C_SLAVE != FALSE) && (_SUPPORT_I2C_RW_ISR == FALSE) */

#if (LIN_COMM != FALSE)
    LinDiagResponseTimeoutCount(1U);

#if (_SUPPORT_LIN_AA != FALSE)
    LinAutoAddressingTimer();
#endif /* (_SUPPORT_LIN_AA != FALSE) */
#endif /* (LIN_COMM != FALSE) */

/*lint !e436 */
#if (CAN_COMM != FALSE)
    CanPeriodicTimerEvent(1U);
/*lint !e436 */
#endif /* (CAN_COMM != FALSE) */

    main_PeriodicTimerEvent(1U);

    DiagnosticPeriodicTimerEvent();

#if (_SUPPORT_IO_DUT_SELECT_HVIO != FALSE) || (_SUPPORT_IO_DUT_SELECT_PWR != FALSE) || (_SUPPORT_IO_CMD_SELECT_LVIO != FALSE)
    IO_PeriodicTimer();
#endif /* (_SUPPORT_IO_DUT_SELECT_HVIO != FALSE) || (_SUPPORT_IO_DUT_SELECT_PWR != FALSE) || (_SUPPORT_IO_CMD_SELECT_LVIO != FALSE) */

#if (_SUPPORT_UART != FALSE) && ((_SUPPORT_UART_IF_APP == C_UART_IF_SCOPE) || \
    (_SUPPORT_UART_IF_APP == C_UART_IF_QUAD_WHEEL_CAR) || (_SUPPORT_UART_IF_APP == C_UART_IF_ACTUATOR) || \
    (_SUPPORT_UART_IF_APP == C_UART_IF_BLUETOOTH) || (_SUPPORT_UART2_IF_APP == C_UART_IF_BLUETOOTH) || \
    (_SUPPORT_UART_IF_APP == C_UART_IF_TERMINAL) || (_SUPPORT_UART2_IF_APP == C_UART_IF_TERMINAL) || \
    (_SUPPORT_UART_IF_APP == C_UART_IF_MLX_ACT))
    UART_PeriodicTimer();
#endif /* (_SUPPORT_UART != FALSE) && ((_SUPPORT_UART_IF_APP == C_UART_IF_SCOPE) || (_SUPPORT_UART_IF_APP == C_UART_IF_QUAD_WHEEL_CAR) || (_SUPPORT_UART_IF_APP == C_UART_IF_BLUETOOTH) || (_SUPPORT_UART2_IF_APP == C_UART_IF_BLUETOOTH) || (_SUPPORT_UART_IF_APP == C_UART_IF_ACTUATOR)) */

#if (_SUPPORT_HUMIDITY_HDC302x != FALSE)
    HumidityPeriodicTimerevent(1U);
#endif /* (_SUPPORT_HUMIDITY_HDC302x != FALSE) */

#if (_DEBUG_STIMER_ISR != FALSE)
    DEBUG_CLR_IO_A();
#endif /* (_DEBUG_STIMER_ISR != FALSE) */
#if (_DEBUG_CPU_LOAD != FALSE)
    DEBUG_CLR_IO_D();                                                           /* IRQ-Priority: 6 */
#endif /* (_DEBUG_CPU_LOAD != FALSE) */
} /* End of ISR_STIMER() */

/* EOF */
