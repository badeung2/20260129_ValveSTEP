/*!*************************************************************************** *
 * \file        main.h
 * \brief       MLX8133x Main
 *
 * \note        project MLX8133x
 *
 * \author      Marcel Braat
 *
 * \date        2017-08-15
 *
 * \version     2.0
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
 * *************************************************************************** */

#ifndef MAIN_H
#define MAIN_H

/*!*************************************************************************** */
/*                           INCLUDES                                          */
/* *************************************************************************** */
#include "AppBuild.h"                                                           /* Application Build */

/* *************************************************************************** */
/*                           DEFINITIONS                                       */
/* *************************************************************************** */
/* Maximum: 12 ms (@32MHz, 1WS) */
#define C_FET_SETTING           (uint16_t)((10UL * FPLL) / C_DELAY_CONST)       /*!<  10us delay */
#define C_DELAY_3AXIS           (uint16_t)((1250UL * FPLL) / C_DELAY_CONST)     /*!<   1.25ms delay */

/*!*************************************************************************** */
/*                           GLOBAL VARIABLES                                  */
/* *************************************************************************** */

/*!*************************************************************************** */
/*                           GLOBAL FUNCTIONS                                  */
/* *************************************************************************** */
extern void main_PeriodicTimerEvent(uint16_t u16Period);
extern void main_noinit_section_init(void);
extern int main(void);
#if (_SUPPORT_NV_EMERGENCY_STORE != FALSE)
extern void HandleSleepMotorRequest(void);
#endif /* (_SUPPORT_NV_EMERGENCY_STORE != FALSE) */

#endif /* MAIN_H */

/* EOF */
