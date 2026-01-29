/**
 * @file
 * @brief Simple Timer hardware support library
 * @internal
 *
 * @copyright (C) 2017 Melexis N.V.
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
 * @ingroup simple_timer
 *
 * @brief Simple Timer hardware support library
 * @details
 * @{
 */

#ifndef STIMERLIB_INLINE_IMPL_H
#define STIMERLIB_INLINE_IMPL_H

#include <stdbool.h>
#include "io.h"

/** STIMER_SET_MODE sets the simple timer's mode
 * @param[in] mode new Simple Timer's mode
 */
STATIC INLINE void STIMER_SET_MODE( STimerClk_t mode )
{
    IO_SET(STIMER, MODE, (uint16_t)mode);
}

/** STIMER_SET_VALUE sets the simple timer's mode
 * @param[in] value is new 14-bit timer value
 */
STATIC INLINE void STIMER_SET_VALUE( uint16_t value )
{
    IO_SET(STIMER, VALUE, value);
}

/** STIMER_INIT sets the simple timer's mode and value
 * @param[in] mode new Simple Timer's mode
 * @param[in] value represents the period value
 */
STATIC INLINE void STIMER_INIT( STimerClk_t mode, uint16_t value )
{
    STIMER_SET_MODE(mode);
    STIMER_SET_VALUE(value);
}

/** STIMER_GET_CURRENT Returns the current timer's value
 * @return 14bit timer's counter value
 */
STATIC INLINE uint16_t STIMER_GET_CURRENT(void)
{
    return IO_GET(STIMER, CURRENT);
}

/** STIMER_INT_DISABLE Disables the simple timer's interrupt */
STATIC INLINE void STIMER_INT_DISABLE(void)
{
    IO_SET(MLX16, STIMER_ITC, 0u);
}

/** STIMER_INT_ENABLE Enables the simple timer's interrupt */
STATIC INLINE void STIMER_INT_ENABLE(void)
{
    IO_SET(MLX16, STIMER_PEND, 1u);
    IO_SET(MLX16, STIMER_ITC, 1u);
}

#endif /* STIMERLIB_INLINE_IMPL_H */
/// @}
