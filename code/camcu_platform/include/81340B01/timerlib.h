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
 * @addtogroup simple_timer Simple Timer
 *
 * @brief Simple Timer hardware support library
 * @details
 * @{
 */

#ifndef STIMERLIB_H
#define STIMERLIB_H

#include <stdint.h>
#include "compiler_abstraction.h"
#include "io.h"

/** Simple timer clock's mode definitions */
typedef enum {
    STIMER_DISABLE_CLOCK = 0u,      /**< Simple timer is turned off */
    STIMER_CPU_CLOCK = 1u,          /**< Simple timer counts on the CPU clocks frequency */
    STIMER_1US_CLOCK = 2u,          /**< Simple timer runs on 1us period */
    STIMER_100US_CLOCK = 3u,        /**< Simple timer runs on 100us period */
} STimerClk_t;


/** STIMER_SET_MODE sets the simple timer's mode
 * @param[in] mode new Simple Timer's mode
 */
STATIC INLINE void STIMER_SET_MODE( STimerClk_t mode );

/** STIMER_SET_VALUE sets the simple timer's mode
 * @param[in] value is new 14-bit timer value
 */
STATIC INLINE void STIMER_SET_VALUE( uint16_t value );

/** STIMER_INIT sets the simple timer's mode and value
 * @param[in] mode new Simple Timer's mode
 * @param[in] value represents the period value
 */
STATIC INLINE void STIMER_INIT( STimerClk_t mode, uint16_t value );

/** STIMER_GET_CURRENT Returns the current timer's value
 * @return 14bit timer's counter value
 */
STATIC INLINE uint16_t STIMER_GET_CURRENT(void);

/** STIMER_INT_DISABLE Disables the simple timer's interrupt */
STATIC INLINE void STIMER_INT_DISABLE(void);

/** STIMER_INT_ENABLE Enables the simple timer's interrupt */
STATIC INLINE void STIMER_INT_ENABLE(void);

#if !defined(UNITTEST)
#include "timerlib_inline_impl.h"
#endif /* UNITTEST */

#endif /* STIMERLIB_H */
/// @}
