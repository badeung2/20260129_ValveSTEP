/*!*************************************************************************** *
 * \file        Diagnostic.h
 * \brief       MLX8133x/4x Diagnostic handling
 *
 * \note        project MLX8133x/4x
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
 * Copyright (C) 2017-2018 Melexis N.V.
 * The Software is being delivered 'AS IS' and Melexis, whether explicitly or
 * implicitly, makes no warranty as to its Use or performance.
 * The user accepts the Melexis Firmware License Agreement.
 *
 * Melexis confidential & proprietary
 *
 * *************************************************************************** */

#ifndef DRIVE_LIB_DIAGNOSTIC_H
#define DRIVE_LIB_DIAGNOSTIC_H

/*!*************************************************************************** */
/*                           INCLUDES                                          */
/* *************************************************************************** */
#include "AppBuild.h"                                                           /* Application Build */

/*!*************************************************************************** */
/*                           DEFINITIONS                                       */
/* *************************************************************************** */

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
extern void DiagnosticInit(void);
extern void DiagnosticReset(void);
extern void DiagnosticPeriodicTimerEvent(void);

#endif /* DRIVE_LIB_DIAGNOSTIC_H */

/* EOF */
