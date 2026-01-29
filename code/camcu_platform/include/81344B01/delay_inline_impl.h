/**
 * @file
 * @brief Delay functions
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

#ifndef DELAY_INLINE_IMPL_H_
#define DELAY_INLINE_IMPL_H_

#include "delay.h"

STATIC INLINE void delay_nops(uint16_t nops)
{
    uint16_t dummy;     /* used to inform that input operand is clobbered  */
    __asm__ __volatile__ (
        "add %[cnt], #-2 \n\t"  /* '-2' to compensate time for X loading */
        "djnz %[cnt], ."
        : "=x" (dummy)
        : [cnt] "x" (nops)
        );
}

#endif /* DELAY_INLINE_IMPL_H_ */

