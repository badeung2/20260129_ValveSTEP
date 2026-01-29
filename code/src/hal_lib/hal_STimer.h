/*!************************************************************************** *
 * \file        hal_STimer.h
 * \brief       Hardware Abstraction Layer for STimer handling
 *
 * \note        project MLX81160/33x/34x/35x
 *
 * \author      Marcel Braat
 *
 * \date        2024-03-22
 *
 * \version     2.0
 *
 *
 * MELEXIS Microelectronic Integrated Systems
 *
 * Copyright (C) 2024-2024 Melexis N.V.
 * The Software is being delivered 'AS IS' and Melexis, whether explicitly or
 * implicitly, makes no warranty as to its Use or performance.
 * The user accepts the Melexis Firmware License Agreement.
 *
 * Melexis confidential & proprietary
 *
 * ************************************************************************** */

#ifndef HAL_LIB_STIMER_H
#define HAL_LIB_STIMER_H

/*!************************************************************************** */
/*                          INCLUDES                                          */
/* ************************************************************************** */
#include "../AppBuild.h"                                                        /* Application Build */

/*!************************************************************************** */
/*                          DEFINITIONS                                       */
/* ************************************************************************** */
#define CT_PERIODIC_RATE            500U                                        /*!< Periodic interrupt at 500us rate */
#if (_SUPPORT_STIMER_MODE == C_STIMER_CLKSRC_CPU)
#define CT_CPU_PERIODIC_RATE        ((FPLL / 1000U) * CT_PERIODIC_RATE)         /*!< Periodic interrupt at 500us rate */
#elif (_SUPPORT_STIMER_MODE == C_STIMER_CLKSRC_1MHz)
#define CT_1MHz_PERIODIC_RATE       CT_PERIODIC_RATE                            /*!< Periodic interrupt at 500us rate */
#elif (_SUPPORT_STIMER_MODE == C_STIMER_CLKSRC_10kHz)
#define CT_10kHz_PERIODIC_RATE      (CT_PERIODIC_RATE / 100U)                   /*!< Periodic interrupt at 500us rate */
#else
#error "ERROR: Unsupported STIMER Clock Source."
#endif

/*!*************************************************************************** */
/*                           GLOBAL VARIABLES                                  */
/* *************************************************************************** */
#pragma space dp
#pragma space none

#pragma space nodp                                                              /* __NEAR_SECTION__ */
#pragma space none                                                              /* __NEAR_SECTION__ */

/*!*************************************************************************** */
/*                           GLOBAL FUNCTIONS                                  */
/* *************************************************************************** */
#include "hal_STimer_inline.h"                                                  /* STimer inline function support */

#endif /* HAL_LIB_STIMER_H */

/* EOF */
