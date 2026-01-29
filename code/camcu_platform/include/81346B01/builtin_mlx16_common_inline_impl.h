/**
 * @file
 * @brief Header file for common functions specifically optimized for the MLX16 processor
 * @internal
 *
 * @copyright (C) Melexis N.V.
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

#ifndef BUILTIN_MLX16_COMMON_INLINE_IMPL_H_
#define BUILTIN_MLX16_COMMON_INLINE_IMPL_H_

#include "builtin_mlx16_common.h"

STATIC INLINE void builtin_mlx16_disable_interrupts(void)
{
    __asm__ __volatile__ (
        "clrb MH.2\n\t"     /* clear Pr part only staring from msbit */
        "clrb MH.1\n\t"
        "clrb MH.0"
        ::: "M", "memory"
        );
}

STATIC INLINE void builtin_mlx16_set_priority(uint8_t level)
{
    __asm__ __volatile__ (
        "mov UPR, %[upr]"
        :: [upr] "ri" (level)
        : "M", "memory");
}

STATIC INLINE uint16_t builtin_mlx16_get_status(void)
{
    uint16_t status;

    __asm__ __volatile__ (
        "mov %[dest], M" "\n\t"
        : [dest] "=r" (status)
        :: "memory"
        );

    return status;
}

STATIC INLINE void builtin_mlx16_set_status(uint16_t status)
{
    __asm__ __volatile__ (
        "mov M, %[src]"
        :: [src] "r" (status)
        : "M", "memory"
        );
}

STATIC INLINE void builtin_mlx16_enter_user_mode(void)
{
    __asm__ __volatile__ ("setb MH.3" ::: "M", "memory");
}

STATIC INLINE void builtin_mlx16_movb(uint8_t volatile* addr, uint8_t mask, uint8_t value)
{
    uint16_t mask_and_value = ((uint16_t)mask << 8u) | value;

    __asm__ __volatile__ (
        "movb [%[addr]], %[mask_and_value]\n\t"
        :: [addr] "c" (addr), [mask_and_value] "r" (mask_and_value)
        : "memory"
        );
}

STATIC INLINE void builtin_mlx16_nop(void)
{
    __asm__ __volatile__ ("nop");
}

STATIC INLINE void builtin_mlx16_halt(void)
{
    __asm__ __volatile__ ("HALT");
}

#endif /* BUILTIN_MLX16_COMMON_INLINE_IMPL_H_ */

