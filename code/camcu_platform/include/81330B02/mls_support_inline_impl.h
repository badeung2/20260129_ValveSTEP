/**
 * @file
 * @brief MLX LIN API platform specifics
 * @internal
 *
 * @copyright (C) 2017-2020 Melexis N.V.
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
 * @ingroup mls_support
 *
 * @details MLX LIN API platform specific support functions.
 */

#ifndef MLS_SUPPORT_INLINE_IMPL_H_
#define MLS_SUPPORT_INLINE_IMPL_H_

#include <stdint.h>

#include "compiler_abstraction.h"
#include "io.h"

/** Set the MLX4 clock divider on top of MLX16 clock */
STATIC INLINE void mls_SetMlx4ClockDivider(void)
{
    IO_SET(COLIN, SPEED, (uint16_t)LIN_CLK_DIV);
}

#endif /* MLS_SUPPORT_INLINE_IMPL_H_ */
