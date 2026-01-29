/**
 * @file
 * @brief Most common and basic routines and helpers
 * @internal
 *
 * @copyright (C) 2019 Melexis N.V.
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
 * @ingroup sys_tools
 *
 * @brief Most common and basic routines and helpers
 * @details
 * @{
 */

#ifndef SYS_TOOLS_INLINE_IMPL_H
#define SYS_TOOLS_INLINE_IMPL_H

#include "io.h"
#include <stdint.h>
#include <stdbool.h>
#include "compiler_abstraction.h"

#ifdef UNITTEST
#include "sys_tools.h"
#endif

STATIC INLINE void Sys_SetStack(const uint16_t* const stackptr)
{
    __asm__ __volatile__ (
        "mov S, %[oper]"
        :
        : [oper] "r" (stackptr)
        : "S"
        );
}

STATIC INLINE uint16_t Sys_GetStack(void)
{
    uint16_t stack_pointer;
    __asm__ __volatile__ (
        "mov %[pointer], S"
        : [pointer] "=r" (stack_pointer)
        :
        );
    return (stack_pointer);
}

STATIC INLINE void SYS_clearCpuStatus (void)
{
    __asm__ __volatile__ (
        "mov A, #0" "\n\t"
        "mov M, A"  "\n\t"
        :
        :
        : "A"
        );
}

STATIC INLINE void jmp2Address(uint16_t label)
{
    __asm__ __volatile__ (
        "jmp %[address]\n\t"
        :
        : [address] "d" (label)
        :
        );
}

STATIC INLINE NO_RETURN void Sys_ResetCpu (void)
{
    (void) mlx16_enter_system_mode_keep_prio();
    IO_SET(RST_CTRL, SOFT_RESET, 1u);    /* Reset happens immediately after writing SOFT_RESET bit */
#ifndef UNITTEST
    __builtin_unreachable();
#endif
}

STATIC INLINE NO_RETURN void LOOP_FOREVER(void)
{
    while (true) {

    }
}

STATIC INLINE void DELAY(const uint16_t loops)
{
    delay_cycles(loops);
}

STATIC INLINE void DELAY_US(uint16_t us)
{
    DELAY((uint16_t)((((uint32_t)FPLL * (uint32_t)(us)) + (((uint32_t)DELAY_NR_CYCLES * 1000u) / 2u)) /
                     ((uint32_t)DELAY_NR_CYCLES * 1000u)));
}

STATIC INLINE void MSEC_DELAY(int16_t msec)
{
    int16_t i;
    for(i = 0; i < msec; i++) {
        delay_cycles(((uint16_t)FPLL / (uint16_t)DELAY_NR_CYCLES));
    }
}

#endif /* SYS_TOOLS_INLINE_IMPL_H */
/// @}
