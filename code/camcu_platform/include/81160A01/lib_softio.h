/**
 * @file
 * @brief header file of a library to drive the IO pins from software
 *
 * @internal
 *
 * @copyright (C) 2015-2018 Melexis N.V.
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
 * @ingroup softio
 * @addtogroup softio Software IO
 *
 * @brief library to drive the IO pins from software
 * @details
 * This library allows to access the IO pins from the firmware. It is always possible to read
 * IO pins as input, therefore there is no special configuration necessary. To drive an IO from
 * firmware, it has to be configured as output.
 * Interrupts on the different pins can be enabled. The ISR for each IO has to be implemented by the
 * application itself.
 *
 * @{
 */

#ifndef LIB_SOFTIO_H
#define LIB_SOFTIO_H

#include "io.h"
#include "lib_gpio.h"
#include <stdbool.h>
#include <syslib.h>
#include "sys_tools.h"
#include "plib.h"

/** Configure the IO as an output
 *
 * Configure the pin as an output under software control
 * @param[in] IO IO pin number from 0 to 3
 * @return success or not
 * @retval 1 in case the pin is correctly configured
 * @retval 0 in case the pin cannot be configured
 *
 * Usage example:
 *
 *   - configure IO 3 as output under software control
 * @code {.c}
 *     softioConfigureOutput(3);
 * @endcode

 */
bool softio_configureOutput(GpioIo_t IO);

/** Function to enable the interrupt of an IO port
 *
 * This function enables the interrupt on the rising edge of an IO port.
 * The interrupt service routine is not implemented and has to be
 * written by the application.
 * @param[in] IO  pin number from 0 to 3
 * retval 1 interrupt on IO is correctly enabled
 * retval 0 interrupt is not correctly enabled
 */
bool softio_enableInt(GpioIo_t IO);

/** Function to get the actual value of the IO pin
 *
 * This function returns the actual state on the IO pin. This function can
 * always be called, also when a pin has not explicitly been configured.
 * @param[in] IO IO pin number from 0 to 3
 * @retval 1 in case the pin is high
 * @retval 0 in case the pin is low
 * @retval 0xFF in all other cases (invalid pin number...)
 */
STATIC INLINE uint8_t softio_get(GpioIo_t IO);

/** Function to set the IO to a logic high value
 *
 * This function sets the selected pin to a logic high value. The effect
 * can only be seen when the IO has been configured as output.
 * @param[in] IO IO pin number from 0 to 3
 * @retval true in case the pin is correctly set
 * @retval false in all other cases
 */
STATIC INLINE bool softio_set(GpioIo_t IO);

/** Function to reset the IO to a logic low value
 *
 * This function resets the selected pin to a logic low value. The effect
 * can only be seen when the IO has been configured as output.
 * @param[in] IO pin number from 0 to 3
 * @retval true in case the pin is correctly cleared
 * @retval false in all other cases
 */
STATIC INLINE bool softio_clr(GpioIo_t IO);

/** Function to toggle the logical level of the IO
 *
 * This function changes the output level of the IO. A logic low value
 * becomes high and a logic high value becomes low.
 * @param[in] IO pin number from 0 to 3
 * @retval true in case the pin is correctly toggled
 * @retval false in all other cases
 */
STATIC INLINE bool softio_toggle(GpioIo_t IO);


#if !defined(UNITTEST)
#include "lib_softio_inline_impl.h"
#endif

#endif
/// @}
