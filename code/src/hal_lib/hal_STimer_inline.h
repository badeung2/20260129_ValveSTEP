/*!************************************************************************** *
 * \file        hal_STimer_inline.h
 * \brief       Hardware Abstraction Layer for STimer handling (inline)
 *
 * \note        project MLX81160/33x/34x/35x
 *
 * \author      Marcel Braat
 *
 * \date        2024-03-22
 *
 * \version     2.0
 *
 * A list of functions:
 *  - Inline Functions:
 *           -# HAL_STimer_Init()
 *           -# HAL_STimer_Stop()
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

#ifndef HAL_LIB_STIMER_INLINE_H
#define HAL_LIB_STIMER_INLINE_H

/*!************************************************************************** */
/*                          INCLUDES                                          */
/* ************************************************************************** */
#include "../AppBuild.h"                                                        /* Application Build */

#include <atomic.h>

/*!************************************************************************** */
/*                          DEFINITIONS                                       */
/* ************************************************************************** */
#define C_STIMER_PRIO                       C_MLX16_ITC_PRIO0_STIMER_PRIO6      /*!< coPRO IRQ Priority: 6 */

/*!*************************************************************************** *
 * HAL_STimer_Init
 * \brief   Initialise STimer interface
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details Set up STimer period and interrupt.
 *          The STimer IRQ priority is 6.
 *          The function is defined inline as it is normally only "called" once.
 * *************************************************************************** *
 * - Call Hierarchy:
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 0
 * *************************************************************************** */
static __inline__ void HAL_STimer_Init(void)
{
    /* Setup back-ground timer to 500us period intervals */
#if (_SUPPORT_STIMER_MODE == C_STIMER_CLKSRC_CPU)
    /* Use CPU Clock as reference */
    IO_STIMER_CTRL = C_STIMER_MODE_CPU | CT_CPU_PERIODIC_RATE;
#elif (_SUPPORT_STIMER_MODE == C_STIMER_CLKSRC_1MHz)
    /* Use 1MHz Clock as reference */
    IO_STIMER_CTRL = C_STIMER_MODE_1MHz | CT_1MHz_PERIODIC_RATE;
#elif (_SUPPORT_STIMER_MODE == C_STIMER_CLKSRC_10kHz)
    /* Use 10kHz Clock as reference */
    IO_STIMER_CTRL = C_STIMER_MODE_10kHz | CT_10kHz_PERIODIC_RATE;
#else
    /* Default: Use 1MHz Clock as reference */
    IO_STIMER_CTRL = C_STIMER_MODE_1MHz | CT_1MHz_PERIODIC_RATE;
#endif
    /* Configure background timer interrupt, including priority */
#if (_SUPPORT_APP_USER_MODE != FALSE)
    ENTER_SECTION(SYSTEM_MODE); /*lint !e534 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
    IO_MLX16_ITC_PRIO0_S = (IO_MLX16_ITC_PRIO0_S & ~M_MLX16_ITC_PRIO0_STIMER) | C_STIMER_PRIO;  /* Set CoreTimer priority to 6 (3..6); Lowest */
#if defined (__MLX81160__)
    IO_MLX16_ITC_PEND0_S = B_MLX16_ITC_PEND0_STIMER;                            /* Clear possible pending interrupt, before enabling the interrupt */
    IO_MLX16_ITC_MASK0_S |= B_MLX16_ITC_MASK0_STIMER;                           /* Enable Timer interrupt */
#else  /* defined (__MLX81160__) */
    IO_MLX16_ITC_PEND1_S = B_MLX16_ITC_PEND1_STIMER;                            /* Clear possible pending interrupt, before enabling the interrupt */
    IO_MLX16_ITC_MASK1_S |= B_MLX16_ITC_MASK1_STIMER;                           /* Enable Timer interrupt */
#endif /* defined (__MLX81160__) */
#if (_SUPPORT_APP_USER_MODE != FALSE)
    EXIT_SECTION(); /*lint !e438 !e529 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
} /* End of HAL_STimer_Init() */

/*!*************************************************************************** *
 * HAL_STimer_Stop
 * \brief   Stop the Simple timer, and disable the IRQ
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details Stop background timer by stopping the timer itself and disabling the
 *          interrupt
 *          The function is defined inline as it is normally only "called" once.
 * *************************************************************************** *
 * - Call Hierarchy: -
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 0
 * *************************************************************************** */
static __inline__ void HAL_STimer_Stop(void)
{
    IO_STIMER_CTRL = C_STIMER_MODE_OFF;
#if (_SUPPORT_APP_USER_MODE != FALSE)
    ENTER_SECTION(SYSTEM_MODE); /*lint !e534 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
#if defined (__MLX81160__)
    IO_MLX16_ITC_MASK0_S &= ~B_MLX16_ITC_MASK0_STIMER;                          /* Disable Timer interrupt */
    IO_MLX16_ITC_PEND0_S = B_MLX16_ITC_PEND0_STIMER;                            /* Clear possible pending interrupt */
#else  /* defined (__MLX81160__) */
    IO_MLX16_ITC_MASK1_S &= ~B_MLX16_ITC_MASK1_STIMER;                          /* Disable Timer interrupt */
    IO_MLX16_ITC_PEND1_S = B_MLX16_ITC_PEND1_STIMER;                            /* Clear possible pending interrupt */
#endif /* defined (__MLX81160__) */
#if (_SUPPORT_APP_USER_MODE != FALSE)
    EXIT_SECTION(); /*lint !e438 !e529 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
} /* End of HAL_STimer_Stop() */

/*!*************************************************************************** *
 * HAL_STimer_Check
 * \brief   Check Simple timer is still active
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  (uint16_t) u16Result: FALSE: No problem found.
 *                                TRUE: Problem found; Suggest to initialise again.
 * *************************************************************************** *
 * \details
 *          The function is defined inline as it is normally only "called" once.
 * *************************************************************************** *
 * - Call Hierarchy: -
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 0
 * *************************************************************************** */
static __inline__ uint16_t HAL_STimer_Check(void)
{
    uint16_t u16Result = FALSE;

    if ( ((IO_STIMER_CTRL & M_STIMER_MODE) == C_STIMER_MODE_OFF) ||             /* Check STimer is still enabled */
         ((IO_MLX16_ITC_PRIO0_S & M_MLX16_ITC_PRIO0_STIMER) != C_STIMER_PRIO) ||  /* Check STimer Priority */
#if defined (__MLX81160__)
         ((IO_MLX16_ITC_MASK0_S & B_MLX16_ITC_MASK0_STIMER) == 0U) )
#else  /* defined (__MLX81160__) */
         ((IO_MLX16_ITC_MASK1_S & B_MLX16_ITC_MASK1_STIMER) == 0U) )
#endif /* defined (__MLX81160__) */
    {
        u16Result = TRUE;
    }
    return ( u16Result );
} /* End of HAL_STimer_Check() */

#endif /* HAL_LIB_STIMER_INLINE_H */

/* EOF */
