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

#ifndef BUILTIN_MLX16_COMMON_H_
#define BUILTIN_MLX16_COMMON_H_

#include <stdint.h>
#include "compiler_abstraction.h"

/** Disable all interrupts
 *
 * Disables all interrupts by setting CPU priority to 0 (the highest).
 * The CPU priority is located in M register -- MH[2:0].
 *
 * This function actually compiles into three lines of assembly, so there is no function
 * call overhead.  However, the macro also implies a <i>memory barrier</i>
 * which can cause additional loss of optimization.
 *
 * In order to implement atomic access to multi-byte objects,
 * consider using the ENTER_SECTION(ATOMIC_KEEP_MODE) macro from <atomic.h>, rather than
 * implementing them manually.
 */
STATIC INLINE void builtin_mlx16_disable_interrupts(void);

/** Set CPU priority (UPr register)
 *
 * Sets CPU priority to the level specified by the <b>level</b> parameter.
 * All interrupts with level greater than specified CPU priority will be effectively
 * disabled. Please note that the lower level corresponds to the higher priority
 * with level 0 being the highest priority.
 *
 * This function implies a <i>memory barrier</i> which can cause additional loss of optimization.
 *
 * @param level [0-7]      CPU priority level
 */
STATIC INLINE void builtin_mlx16_set_priority(uint8_t level);

/** Get CPU status (M register) value
 *
 * @return M register value
 */
STATIC INLINE uint16_t builtin_mlx16_get_status(void);

/** Set CPU status (M register)
 *
 * @param status   Value to be written into M register
 */
STATIC INLINE void builtin_mlx16_set_status(uint16_t status);

/** Switch CPU into User Mode
 */
STATIC INLINE void builtin_mlx16_enter_user_mode(void);

/** Atomically change (set/clear) one or several bits in a byte.
 *
 * @param addr  Pointer to the byte.
 * @param mask  Bit mask to indicate the target bits.
 * @param value Bit pattern to be written to target bits.
 *
 * The function is the atomic equivalent of the following expression:
 *
 * \code{.c}
 *
 *    *addr = (*addr & ~mask) | (value & mask)
 *
 * \endcode
 *
 * Example usage:
 *
 * \code{.c}
 *
 *    uint8_t v = 0xA5;
 *
 *    builtin_mlx16_movb(&v, 0x3C, 0x18);
 *    assert(v == 0x99);
 *
 * \endcode
 *
 * @internal The function makes use of the MLX16 `MOVB` instruction.
 *
 * Instruction details:
 *
 * \code{.asm}
 *
 *     MOVB <dst>,   <src>
 *          [X | Y]  A | X | Y
 *
 *     mask  = high_byte(src)
 *     value = low_byte(src)
 *     dst = (dst & ~mask) | (value & mask)
 *
 * \endcode
 */
STATIC INLINE void builtin_mlx16_movb(uint8_t volatile* addr, uint8_t mask, uint8_t value);

/** NOP: No operation
 *
 * Compiles into CPUs `nop` instruction
 */
STATIC INLINE void builtin_mlx16_nop(void);

/** HALT: halt the CPU
 *
 * Compiles into CPUs `HALT` instruction
 */
STATIC INLINE void builtin_mlx16_halt(void);

#ifndef UNITTEST
    #include "builtin_mlx16_common_inline_impl.h"
#endif /* !UNITTEST */

#endif /* BUILTIN_MLX16_COMMON_H_ */

