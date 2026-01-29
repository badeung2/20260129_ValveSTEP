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

#ifndef DELAY_H_
#define DELAY_H_

#include <stdint.h>
#include "compiler_abstraction.h"

/** Delay for specified number of 'nop' instructions
 *
 * @param nops [3..65535]  A number of 'nop' instructions for delay.
 *
 * @note
 * The minimum value for the @p nops parameter is 3!
 *
 * @warning
 * Be careful when using this function to measure/delay for an absolute time.
 * The number of nop instructions could be translated to a theoretical
 * number of clock cycles, but the actual time from start to finish taken by the function
 * can be impacted by a number of factors, such as:
 *
 * - Type of the memory (ROM, Flash, RAM) and, hence, possible wait-states
 * - Background DMA activity
 * - Other interrupts interrupting this function
 * - ...
 *
 * Therefore, if a delay of an absolute time with decent precision is needed, the use
 * of a dedicated HW timer is advised. If not available, attempt to measure the actual
 * duration and limit factors delaying the execution.
 */
STATIC INLINE void delay_nops(uint16_t nops);

#ifndef UNITTEST
    #include "delay_inline_impl.h"
#endif /* !UNITTEST */

#endif /* DELAY_H_ */

