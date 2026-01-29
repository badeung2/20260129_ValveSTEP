/*!*************************************************************************** *
 * \file        Timer.h
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

#ifndef DRIVE_LIB_TIMER_H
#define DRIVE_LIB_TIMER_H

/*!*************************************************************************** */
/*                           INCLUDES                                          */
/* *************************************************************************** */
#include "AppBuild.h"                                                           /* Application Build */

#include "hal_lib/hal_STimer.h"                                                 /* STimer HAL support */

/*!*************************************************************************** */
/*                           DEFINITIONS                                       */
/* *************************************************************************** */
#define CT_PERIODIC_RATE            500U                                        /*!< Periodic interrupt at 500us rate */
#if (_SUPPORT_STIMER_MODE == C_STIMER_CLKSRC_CPU)
#define CT_CPU_PERIODIC_RATE        ((FPLL / 1000U) * CT_PERIODIC_RATE)         /*!< Periodic interrupt at 500us rate, using the CPU-Clock Oscillator */
#elif (_SUPPORT_STIMER_MODE == C_STIMER_CLKSRC_1MHz)
#define CT_1MHz_PERIODIC_RATE       CT_PERIODIC_RATE                            /*!< Periodic interrupt at 500us rate, using the 1MHz Oscillator */
#elif (_SUPPORT_STIMER_MODE == C_STIMER_CLKSRC_10kHz)
#define CT_10kHz_PERIODIC_RATE      (CT_PERIODIC_RATE / 100U)                   /*!< Periodic interrupt at 500us rate, using the 10kHz Oscillator */
#else
#define CT_1MHz_PERIODIC_RATE       CT_PERIODIC_RATE                            /*!< Periodic interrupt at 500us rate */
#endif
#define PI_TICKS_PER_MINUTE         (60000000UL / CT_PERIODIC_RATE)             /*!< Core-Timer ticks per minute */
#define PI_TICKS_PER_SECOND         (1000000UL / CT_PERIODIC_RATE)              /*!< Core-Timer ticks per second */
#define PI_TICKS_PER_HALF_SECOND    (500000UL / CT_PERIODIC_RATE)               /*!< Core-Timer ticks per half-a-second */
#define PI_TICKS_PER_MILLISECOND    (1000U / CT_PERIODIC_RATE)                  /*!< Core-Timer ticks per milli-second */

#define C_PI_TICKS_10MS             (10U * PI_TICKS_PER_MILLISECOND)            /*!< Core-Timer ticks per 10ms */
#define C_PI_TICKS_20MS             (20U * PI_TICKS_PER_MILLISECOND)            /*!< Core-Timer ticks per 20ms */
#define C_PI_TICKS_25MS             (25U * PI_TICKS_PER_MILLISECOND)            /*!< Core-Timer ticks per 25ms */
#define C_PI_TICKS_50MS             (50U * PI_TICKS_PER_MILLISECOND)            /*!< Core-Timer ticks per 50ms */
#define C_PI_TICKS_100MS            (100U * PI_TICKS_PER_MILLISECOND)           /*!< Core-Timer ticks per 100ms */
#define C_PI_TICKS_125MS            (125U * PI_TICKS_PER_MILLISECOND)           /*!< Core-Timer ticks per 125ms */
#define C_PI_TICKS_250MS            (250U * PI_TICKS_PER_MILLISECOND)           /*!< Core-Timer ticks per 250ms */
#define C_PI_TICKS_500MS            (500U * PI_TICKS_PER_MILLISECOND)           /*!< Core-Timer ticks per 500ms */
#define C_PI_TICKS_PWM_INACTIVE     (4U * PI_TICKS_PER_SECOND)                  /*!< Core-Timer ticks per Communication PWM-INACTIVE */
#define C_PI_TICKS_MOTOR_RESTART    (250U * PI_TICKS_PER_MILLISECOND)           /*!< Core-Timer ticks per MOTOR_RESTART */
#define C_PI_TICKS_MOTOR_RAMPDOWN   (1U * PI_TICKS_PER_SECOND)                  /*!< Core-Timer ticks per MOTOR-RAMPDOWN */

#define C_MOTOR_START_DELAY_50MSEC  (50U * PI_TICKS_PER_MILLISECOND)            /*!< Core-Timer ticks per 50ms (Motor-start delay) */
#define C_MOTOR_START_DELAY_100MSEC (100U * PI_TICKS_PER_MILLISECOND)           /*!< Core-Timer ticks per 100ms (Motor-start delay) */
#define C_MOTOR_START_DELAY_250MSEC (250U * PI_TICKS_PER_MILLISECOND)           /*!< Core-Timer ticks per 250ms (Motor-start delay) */
#define C_MOTOR_START_DELAY_500MSEC (500U * PI_TICKS_PER_MILLISECOND)           /*!< Core-Timer ticks per 500ms (Motor-start delay) */
#define C_MOTOR_START_DELAY_1SEC    (1U * PI_TICKS_PER_SECOND)                  /*!< Core-Timer ticks per 1s (Motor-start delay) */
#define C_MOTOR_START_DELAY_2SEC    (2U * PI_TICKS_PER_SECOND)                  /*!< Core-Timer ticks per 2s (Motor-start delay) */
#define C_MOTOR_START_DELAY_3SEC    (3U * PI_TICKS_PER_SECOND)                  /*!< Core-Timer ticks per 3s (Motor-start delay) */
#define C_MOTOR_START_DELAY_4SEC    (4U * PI_TICKS_PER_SECOND)                  /*!< Core-Timer ticks per 4s (Motor-start delay) */
#define C_MOTOR_START_DELAY_5SEC    (5U * PI_TICKS_PER_SECOND)                  /*!< Core-Timer ticks per 5s (Motor-start delay) */
#define C_MOTOR_START_DELAY_7SEC    (7U * PI_TICKS_PER_SECOND)                  /*!< Core-Timer ticks per 7s (Motor-start delay) */
#define C_MOTOR_START_DELAY_10SEC   (10U * PI_TICKS_PER_SECOND)                 /*!< Core-Timer ticks per 10s (Motor-start delay) */

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
extern void TimerInit(void);                                                    /*!< Initialise the core timer (Mulan2-timer), at a periodic rate of 500us */
extern void TimerStop(void);                                                    /*!< Stop timer */
extern uint16_t TimerCheck(void);                                               /*!< Check timer functionality */
#if (_SUPPORT_CPU_HALT != FALSE) && (I2C_COMM == FALSE) && (SPI_COMM == FALSE) && (UART_COMM == FALSE)
extern void TimerSleepCompensation(uint16_t u16SleepCompensationPeriod);        /*!< Compensate the various timer-counters for the sleep-period */
#endif /* (_SUPPORT_CPU_HALT != FALSE) && (I2C_COMM == FALSE) && (SPI_COMM == FALSE) && (UART_COMM == FALSE) */

#endif /* DRIVE_LIB_TIMER_H */

/* EOF */

