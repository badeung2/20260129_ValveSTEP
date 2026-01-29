/*!************************************************************************** *
 * \file        hal_PWM_inline.h
 * \brief       Hardware Abstraction Layer for PWM handling (inline)
 *
 * \note        project MLX81160/33x/34x/35x
 *
 * \author      Marcel Braat
 *
 * \date        2024-03-25
 *
 * \version     2.0
 *
 * A list of functions:
 *  - Inline Functions:
 *           -# HAL_PWM_MasterIrqDisable()
 *           -# HAL_PWM_MasterIrqEnable()
 *           -# HAL_PWM_MasterPendClear()
 *           -# HAL_PWM_MasterPendClear()
 *           -# HAL_PWM_MasterPendWait()
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

#ifndef HAL_LIB_PWM_INLINE_H
#define HAL_LIB_PWM_INLINE_H

/*!************************************************************************** */
/*                          INCLUDES                                          */
/* ************************************************************************** */
#include "../AppBuild.h"                                                        /* Application Build */

#include <atomic.h>

/*!************************************************************************** */
/*                          DEFINITIONS                                       */
/* ************************************************************************** */

/*!*************************************************************************** *
 * HAL_PWM_MasterIrqDisable
 * \brief   Disable PWM Master IRQ
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details Disable PWM Master1 IRQ
 * *************************************************************************** *
 * - Call Hierarchy:
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 0
 * *************************************************************************** */
static __inline__ void HAL_PWM_MasterIrqDisable(void)
{
#if (_SUPPORT_APP_USER_MODE != FALSE)
    ENTER_SECTION(SYSTEM_MODE); /*lint !e534 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
#if defined (__MLX81160__) || defined (__MLX81330__) || defined (__MLX81332__) || \
    defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__)
    IO_MLX16_ITC_MASK1_S &= ~B_MLX16_ITC_MASK1_PWM_MASTER1_END;
#elif defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
    IO_MLX16_ITC_MASK2_S &= ~B_MLX16_ITC_MASK2_PWM_MASTER1_END;
#endif
#if (_SUPPORT_APP_USER_MODE != FALSE)
    EXIT_SECTION(); /*lint !e438 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
} /* End of HAL_PWM_MasterIrqDisable() */

/*!*************************************************************************** *
 * HAL_PWM_MasterIrqEnable
 * \brief   Enable PWM Master IRQ
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details Enable PWM Master1 IRQ
 * *************************************************************************** *
 * - Call Hierarchy:
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 0
 * *************************************************************************** */
static __inline__ void HAL_PWM_MasterIrqEnable(void)
{
#if (_SUPPORT_APP_USER_MODE != FALSE)
    ENTER_SECTION(SYSTEM_MODE); /*lint !e534 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
#if defined (__MLX81160__) || defined (__MLX81330__) || defined (__MLX81332__) || \
    defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__)
    IO_MLX16_ITC_PEND1_S = B_MLX16_ITC_PEND1_PWM_MASTER1_END;
    IO_MLX16_ITC_MASK1_S |= B_MLX16_ITC_MASK1_PWM_MASTER1_END;
#elif defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
    IO_MLX16_ITC_PEND2_S = B_MLX16_ITC_PEND2_PWM_MASTER1_END;
    IO_MLX16_ITC_MASK2_S |= B_MLX16_ITC_MASK2_PWM_MASTER1_END;
#endif
#if (_SUPPORT_APP_USER_MODE != FALSE)
    EXIT_SECTION(); /*lint !e438 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
} /* End of HAL_PWM_MasterIrqEnable() */

/*!*************************************************************************** *
 * HAL_PWM_MasterPendClear
 * \brief   Clear PWM Master PEND-flag
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details Clear PWM Master1 PEND-flag (polling)
 * *************************************************************************** *
 * - Call Hierarchy:
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 0
 * *************************************************************************** */
static __inline__ void HAL_PWM_MasterPendClear(void)
{
#if (_SUPPORT_APP_USER_MODE != FALSE)
    ENTER_SECTION(SYSTEM_MODE); /*lint !e534 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
#if defined (__MLX81160__) || defined (__MLX81330__) || defined (__MLX81332__) || \
    defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__)
    IO_MLX16_ITC_PEND1_S = B_MLX16_ITC_PEND1_PWM_MASTER1_END;
#elif defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
    IO_MLX16_ITC_PEND2_S = B_MLX16_ITC_PEND2_PWM_MASTER1_END;
#endif
#if (_SUPPORT_APP_USER_MODE != FALSE)
    EXIT_SECTION(); /*lint !e438 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
} /* End of HAL_PWM_MasterPendClear() */

/*!*************************************************************************** *
 * HAL_PWM_MasterPendWait
 * \brief   Wait for PWM Master PEND-flag
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details Wait PWM Master1 PEND-flag (polling)
 * *************************************************************************** *
 * - Call Hierarchy:
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 0
 * *************************************************************************** */
static __inline__ void HAL_PWM_MasterPendWait(void)
{
#if defined (__MLX81160__) || defined (__MLX81330__) || defined (__MLX81332__) || \
    defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__)
    while ( (IO_MLX16_ITC_PEND1_S & B_MLX16_ITC_PEND1_PWM_MASTER1_END) == 0U) {}  /* Wait for PWM1 CNTI, to switch PWM synchronised */
#elif defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
    while ( (IO_MLX16_ITC_PEND2_S & B_MLX16_ITC_PEND2_PWM_MASTER1_END) == 0U) {}  /* Wait for PWM1 CNTI, to switch PWM synchronised */
#endif
} /* End of HAL_PWM_MasterPendWait() */

#endif /* HAL_LIB_PWM_INLINE_H */

/* EOF */
