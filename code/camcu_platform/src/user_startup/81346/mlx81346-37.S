/**
 * @file
 * @brief The system services routines.
 * @internal
 *
 * @copyright (C) 2020 Melexis N.V.
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
 * @ingroup fw_user_startup
 *
 * @details
 * This file contains the implementations of a MLX81346-37 workaround.
 */

#ifdef HAS_MLX81346_37_FIX

    ;-------------------------------------------------------------
    ; System services
    .section .mlx81346_37_fix,"xa",@progbits
    .global __mlx81346_37_fix

__mlx81346_37_fix:
    .word   0x76DA          ; JMPF but used as word to evade relaxation
    .word   0x9000/2

    end

#endif /* HAS_MLX81346_37_FIX */

