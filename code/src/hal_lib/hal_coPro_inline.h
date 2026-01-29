/*!************************************************************************** *
 * \file        HAL_coPro_inline.h
 * \brief       Hardware Abstraction Layer for co-PRO handling (inline)
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
 *  - Inline Functions:
 *           -# HAL_coPro_Init()
 *           -# HAL_coPro_Start()
 *           -# HAL_coPro_Stop()
 *           -# HAL_coPro_WaitForReady()
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

#ifndef HAL_LIB_coPRO_INLINE_H
#define HAL_LIB_coPRO_INLINE_H

/*!************************************************************************** */
/*                          INCLUDES                                          */
/* ************************************************************************** */
#include "../AppBuild.h"                                                        /* Application Build */

#if (_SUPPORT_COPRO != FALSE) && (defined (__MLX81339__) || defined (__MLX81350__))

#include "coPRO_FOC_api.h"                                                      /* co-PRO FOC support */

#include <atomic.h>
#include <sys_tools.h>

/*!************************************************************************** */
/*                          DEFINITIONS                                       */
/* ************************************************************************** */
#define C_CORPO_PRIO                        C_MLX16_ITC_PRIO5_COPRO_PRIO4       /*!< coPRO IRQ Priority: 4 */

/*!*************************************************************************** */
/*                           GLOBAL VARIABLES                                  */
/* *************************************************************************** */
#pragma space dp
#pragma space none

#pragma space nodp                                                              /* __NEAR_SECTION__ */
extern volatile uint16_t g_u16coPRO_Status;                                     /*!< coPRO (last) status */
#pragma space none                                                              /* __NEAR_SECTION__ */

/*!*************************************************************************** *
 * HAL_coPro_Init
 * \brief   Initialise coPRO interface
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details Set up coPRO IRQ Priority (if enabled: _SUPPORT_COPRO_ISR), clear any
 *          pending IRQ and enable the IRQ-mask.
 *          The coPRO IRQ priority is 4.
 *          The function is defined inline as it is normally only "called" once.
 * *************************************************************************** *
 * - Call Hierarchy:
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 0
 * *************************************************************************** */
static __inline__ void HAL_coPro_Init(void)
{
#if (_SUPPORT_COPRO_ISR != FALSE)
#if (_SUPPORT_APP_USER_MODE != FALSE)
    ENTER_SECTION(SYSTEM_MODE); /*lint !e534 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
    IO_MLX16_ITC_PRIO5_S = (IO_MLX16_ITC_PRIO5_S & ~M_MLX16_ITC_PRIO5_COPRO) | C_CORPO_PRIO;  /* Set coPRO IT priority to '4' */
    IO_MLX16_ITC_PEND3_S = B_MLX16_ITC_PEND3_COPRO;                             /* Clear any pending coPRO IT */
    IO_MLX16_ITC_MASK3_S |= B_MLX16_ITC_MASK3_COPRO;                            /* Enable coPRO_IT mask */
#if (_SUPPORT_APP_USER_MODE != FALSE)
    EXIT_SECTION(); /*lint !e438 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
#endif /* (_SUPPORT_COPRO_ISR != FALSE) */
} /* End of HAL_coPro_Init() */

/*!*************************************************************************** *
 * HAL_coPro_Start
 * \brief   Start coPRO
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16Request: Type of coPro request
 *                           C_COPRO_CMD_FOC_INIT: FOC initialisation
 *                           C_COPRO_CMD_FOC_STEP: FOC calculation
 *                           C_COPRO_CMD_FOC_OPEN2CLOSE: FOC Open-to-Close
 * \return  -
 * *************************************************************************** *
 * \details Set the coPro request, set coPro Status to Busy and activate the coPro.
 *          For performance reasons this function is defined to be inline.
 * *************************************************************************** *
 * - Call Hierarchy:
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 0
 * *************************************************************************** */
static __inline__ void HAL_coPro_Start(uint16_t u16Request)
{
#if (_SUPPORT_COPRO_INTERFACE == C_COPRO_IF_MAILBOX)
    au16CoProDataBuff[C_COPRO_IDX_CMD] = u16Request;
#elif (_SUPPORT_COPRO_INTERFACE == C_COPRO_IF_API)
    coProAPI.u16CoProRequest = u16Request;
#endif /* _SUPPORT_COPRO_INTERFACE */

#if (_SUPPORT_COPRO_ISR != FALSE)
    g_u16coPRO_Status = C_COPRO_STS_BUSY;                                       /* Set coPro status to 'Busy' */
#endif /* (_SUPPORT_COPRO_ISR != FALSE) */

    IO_PORT_COPRO_CTRL = IO_PORT_COPRO_CTRL | B_PORT_COPRO_CTRL_COPRO_RUN;      /* Start the coPRO */
} /* End of HAL_coPro_Start() */

/*!*************************************************************************** *
 * HAL_coPro_Stop
 * \brief   Stop coPRO
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details Stop coPro by clearing Run-bit; Allow access to (coPRO) RAM.
 *          For performance reasons this function is defined to be inline.
 * *************************************************************************** *
 * - Call Hierarchy:
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 0
 * *************************************************************************** */
static __inline__ void HAL_coPro_Stop(void)
{
    IO_PORT_COPRO_CTRL = IO_PORT_COPRO_CTRL & ~B_PORT_COPRO_CTRL_COPRO_RUN;     /* Stop coPro by clearing Run-bit; Allow access to (coPRO) RAM */
} /* End of HAL_coPro_Stop() */

/*!*************************************************************************** *
 * HAL_coPro_Exit
 * \brief   Exit coPRO interface
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details Exit the coPro interface by disabling the ISR
 *          the IRQ-mask.
 *          The function is defined inline as it is normally only "called" once.
 * *************************************************************************** *
 * - Call Hierarchy:
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 0
 * *************************************************************************** */
static __inline__ void HAL_coPro_Exit(void)
{
    HAL_coPro_Stop();
#if (_SUPPORT_COPRO_ISR != FALSE)
#if (_SUPPORT_APP_USER_MODE != FALSE)
    ENTER_SECTION(SYSTEM_MODE); /*lint !e534 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
    IO_MLX16_ITC_MASK3_S &= ~B_MLX16_ITC_MASK3_COPRO;                           /* Disable coPRO_IT mask */
#if (_SUPPORT_APP_USER_MODE != FALSE)
    EXIT_SECTION(); /*lint !e438 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
#endif /* (_SUPPORT_COPRO_ISR != FALSE) */
} /* End of HAL_coPro_Exit() */

/*!*************************************************************************** *
 * HAL_coPro_WaitForReady
 * \brief   Wait for Ready coPRO
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details The function waits until the coPro status is no longer BUSY (as this
 *          has been set by the HAL_coPro_Start).
 *          For performance reasons this function is defined to be inline.
 * *************************************************************************** *
 * - Call Hierarchy:
 * - Cyclomatic Complexity: 1+1
 * - Nesting: 1
 * - Function calling: 0
 * *************************************************************************** */
static __inline__ void HAL_coPro_WaitForReady(void)
{
#if (_SUPPORT_COPRO_ISR == FALSE)
    while ((IO_PORT_COPRO_FLAGS & B_PORT_COPRO_FLAGS_MLX16_CO_HALTED) == 0U) {}
    IO_PORT_COPRO_CTRL = IO_PORT_COPRO_CTRL & ~B_PORT_COPRO_CTRL_COPRO_RUN;     /* Clear Run-bit, to allow access to (coPRO) RAM */
#if (_SUPPORT_COPRO_INTERFACE == C_COPRO_IF_MAILBOX)
    g_u16coPRO_Status = au16CoProDataBuff[C_COPRO_IDX_STS];
#elif (_SUPPORT_COPRO_INTERFACE == C_COPRO_IF_SPI)
    g_u16coPRO_Status = coProAPI.u16CoProStatus;
#endif
#else  /* (_SUPPORT_COPRO_ISR == FALSE) */
    while (g_u16coPRO_Status == C_COPRO_STS_BUSY) {}
#endif /* (_SUPPORT_COPRO_ISR == FALSE) */
} /* End of HAL_coPro_WaitForReady */

#endif /* (_SUPPORT_COPRO != FALSE) && (defined (__MLX81339__) || defined (__MLX81350__)) */

#endif /* HAL_LIB_coPRO_INLINE_H */

/* EOF */
