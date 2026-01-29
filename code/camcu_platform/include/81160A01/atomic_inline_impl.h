/**
 * @file
 * @brief Atomic section utilities
 * @internal
 *
 * @copyright (C) 2015-2016 Melexis N.V.
 * git flash d0014c23
 *
 * Melexis N.V. is supplying this code for use with Melexis N.V. processor based microcontrollers only.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY,
 * INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.  MELEXIS N.V. SHALL NOT IN ANY CIRCUMSTANCES,
 * BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 * @endinternal
 *
 * @addtogroup chip support package
 *
 * @details
 */

#ifndef ATOMIC_INLINE_IMPL_H_
#define ATOMIC_INLINE_IMPL_H_

#include "atomic.h"

#if !defined(__DOXYGEN__)

STATIC INLINE void mlx16_restore_status(const uint16_t* pm)
{
    uint16_t status = *pm;      /* get saved M value */
    builtin_mlx16_set_status(status);
}

STATIC INLINE uint16_t mlx16_di_keep_mode(void)
{
    builtin_mlx16_disable_interrupts();
    return 1;
}

STATIC INLINE uint16_t mlx16_di_enter_system_mode(void)
{
    __asm__ __volatile__ (".word OPCODE_CALL_FP_ENTER_SYSTEM_MODE_PRIO_0");
    return 1;
}

STATIC INLINE uint16_t mlx16_enter_system_mode_keep_prio(void)
{
    __asm__ __volatile__ (".word OPCODE_CALL_FP_ENTER_SYSTEM_MODE_KEEP_PRIO");
    return 1;
}

#endif  /* !__DOXYGEN__ */

#endif  /* ATOMIC_INLINE_IMPL_H_ */

