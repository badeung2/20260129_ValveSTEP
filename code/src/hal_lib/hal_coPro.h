/*!************************************************************************** *
 * \file        HAL_coPro.h
 * \brief       Hardware Abstraction Layer for co-PRO handling
 *
 * \note        project MLX81339/350
 *
 * \author      Marcel Braat
 *
 * \date        2024-03-21
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

#ifndef HAL_LIB_coPRO_H
#define HAL_LIB_coPRO_H

/*!************************************************************************** */
/*                          INCLUDES                                          */
/* ************************************************************************** */
#include "../AppBuild.h"                                                        /* Application Build */

#if (_SUPPORT_COPRO != FALSE) && (defined (__MLX81339__) || defined (__MLX81350__))

#include "coPRO_FOC_api.h"                                                      /* co-PRO FOC support */
#include "hal_coPro_inline.h"                                                   /* co-PRO inline function support */

/*!************************************************************************** */
/*                          DEFINITIONS                                       */
/* ************************************************************************** */


/*!*************************************************************************** */
/*                           GLOBAL VARIABLES                                  */
/* *************************************************************************** */
#pragma space dp
#pragma space none

#pragma space nodp                                                              /* __NEAR_SECTION__ */
extern volatile uint16_t g_u16coPRO_Status;                                     /*!< coPRO (last) status */
#pragma space none                                                              /* __NEAR_SECTION__ */

/*!*************************************************************************** */
/*                           GLOBAL FUNCTIONS                                  */
/* *************************************************************************** */

/*! Inline coPro functions */
static __inline__ void HAL_coPro_Init(void);                                    /*!< Initialise coPRO interface */
static __inline__ void HAL_coPro_Start(uint16_t u16Request);                    /*!< Start coPRO */
static __inline__ void HAL_coPro_Stop(void);                                    /*!< Stop coPRO */
static __inline__ void HAL_coPro_Exit(void);                                    /*!< Exit coPRO interface */
static __inline__ void HAL_coPro_WaitForReady(void);                            /*!< Wait for coPRO to be ready (not busy) */

#endif /* (_SUPPORT_COPRO != FALSE) && (defined (__MLX81339__) || defined (__MLX81350__)) */

#endif /* HAL_LIB_coPRO_H */

/* EOF */
