/*!*************************************************************************** *
 * \file        HAL_coPro.c
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
 * A list of functions:
 *  - Global Functions:
 *           -# ISR_COPRO()
 *  - Internal Functions:
 *
 * \copyright   MELEXIS Microelectronic Integrated Systems
 *
 * Copyright (C) 2024-2024 Melexis N.V.
 * The Software is being delivered 'AS IS' and Melexis, whether explicitly or
 * implicitly, makes no warranty as to its Use or performance.
 * The user accepts the Melexis Firmware License Agreement.
 *
 * Melexis confidential & proprietary
 * *************************************************************************** *
 * Resources:
 *
 * *************************************************************************** */

/*!*************************************************************************** */
/*                           INCLUDES                                          */
/* *************************************************************************** */
#include "AppBuild.h"                                                           /* Application Build */

#if (_SUPPORT_COPRO != FALSE) && (defined (__MLX81339__) || defined (__MLX81350__))
#include "hal_coPro.h"                                                          /* co-PRO HAL support */

#include <atomic.h>
#include <sys_tools.h>

/*!************************************************************************** */
/*                          GLOBAL & LOCAL VARIABLES                          */
/* ************************************************************************** */
#pragma space dp
#pragma space none

#pragma space nodp                                                              /* __NEAR_SECTION__ */
volatile uint16_t g_u16coPRO_Status;                                            /*!< coPRO (last) status */
#pragma space none                                                              /* __NEAR_SECTION__ */

#if (_SUPPORT_COPRO_ISR != FALSE)
/*!*************************************************************************** *
 * ISR_COPRO
 * \brief   coPRO interrupt handler
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details
 *           IRQ Priority: 4
 * *************************************************************************** *
 * - Call Hierarchy: IRQ()
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 0
 * *************************************************************************** */
#if (_SUPPORT_RAM_FUNC != FALSE)
__attribute__((interrupt)) void ISR_COPRO(void) __attribute__ ((section(".ramfunc"))) __attribute__ ((aligned(8)));
#elif (_SUPPORT_OPTIMIZE_FOR_SPEED != FALSE)
__attribute__((interrupt)) void ISR_COPRO(void) __attribute__((aligned(8)));
#endif /* (_SUPPORT_OPTIMIZE_FOR_SPEED != FALSE) */
__attribute__((interrupt)) void ISR_COPRO(void)
{
    HAL_coPro_Stop();                                                           /* Stop coPro to get access to coPro RAM */
#if (_SUPPORT_COPRO_INTERFACE == C_COPRO_IF_MAILBOX)
    g_u16coPRO_Status = au16CoProDataBuff[C_COPRO_IDX_STS];
#elif (_SUPPORT_COPRO_INTERFACE == C_COPRO_IF_API)
    g_u16coPRO_Status = coProAPI.u16CoProStatus;
#endif
} /* End of ISR_COPRO() */
#endif /* (_SUPPORT_COPRO_ISR != FALSE) */

#endif /* (_SUPPORT_COPRO != FALSE) && (defined (__MLX81339__) || defined (__MLX81350__)) */

/* EOF */
