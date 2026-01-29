/**
 * @file
 * @brief System library functionality
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

#ifndef SYSLIB_H
#define SYSLIB_H

#include <stdint.h>
#include <compiler_abstraction.h>
#include <builtin_mlx16_common.h>
#include <static_assert.h>

#define NOP() builtin_mlx16_nop()              /* no operation */
#define HALT() builtin_mlx16_halt()             /* halt the CPU */

/** Macro to use atomic setb instruction for io or dp bits
 *
 * @note Only 32 first bytes of IO page could have bit access (instruction set
 *       limitation) and this bit access should be supported by hardware for
 *       these IO ports. Check io portman of the project to find out which
 *       IO ports have bit access.
 * @note Only 16 first bytes of RAM direct page (dp) have bit access
 */
#define setbit(addr, bitno) setbit_expanded(addr, bitno)

#define setbit_expanded(addr, bitno) \
    __asm__ ("setb\t%0." #bitno :: "m" (addr) )

/** Macro to use atomic clrb instruction for io or dp bits
 *
 * @note See notes in setbit
 */
#define clrbit(addr, bitno) clrbit_expanded(addr, bitno)

#define clrbit_expanded(addr, bitno) \
    __asm__ ("clrb\t%0." #bitno :: "m" (addr) )

/** Ceil 'number' to a multiple of 'unit'.
 *
 * This macro finds the smallest multiple of 'unit' that is bigger than 'number'.
 *
 * Examples:
 *
 * - When 'number' = 6 and 'unit' = 8, than CEIL(6, 8) = 8
 * - When 'number' = 9 and 'unit' = 8, than CEIL(9, 8) = 16
 */
#define CEIL(number, unit) ((((number) + (unit) - 1u) / (unit)) * (unit))

#endif /* SYSLIB_H */

