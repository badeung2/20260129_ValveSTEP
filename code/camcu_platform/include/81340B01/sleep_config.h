/**
 * @file
 * @brief Provides the sleep/stop-mode support function
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
 * @ingroup sleep
 *
 * @brief Provides the sleep/stop-mode support function
 * @details
 * SLEEP_MODE common settings and definitions. Used to manage the common SLEEP_MODE structure for the chips.
 * @{
 */

#ifndef SLEEP_CONFIG_H
#define SLEEP_CONFIG_H

#include "compiler_abstraction.h"

/** Determines the wake-up timer interval */
typedef enum {
    WUI_NOWAKEUP = 0u, /**< No wake-up by internal timer */
    WUI_410MS = 1u,   /**< Wakeup after 4096 of 10khz clock cycles. 409,6ms */
    WUI_819MS = 2u,   /**< Wakeup after 8192 of 10khz clock cycles. 819,2ms */
    WUI_1638MS = 3u,  /**< Wakeup after 16384 of 10khz clock cycles. 1638,4ms */
} WakeUpTimerInterval_t;

#endif /* SLEEP_CONFIG_H */
/// @}
