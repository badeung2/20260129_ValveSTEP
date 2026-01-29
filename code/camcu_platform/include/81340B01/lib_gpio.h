/**
 * @file
 * @brief headerfile for general purpose IO library
 * @internal
 *
 * @copyright (C) 2018 Melexis N.V.
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
 * @addtogroup lib_gpio
 *
 * @details
 * This library allows to configure the IO pins to connect them to the different signals.
 */

#ifndef LIB_GPIO_H
#define LIB_GPIO_H

#include "compiler_abstraction.h"
#include <stdbool.h>
#include <lib_miscio.h>

#define gpio_outputSelections_t GpioOutputSelections_t __attribute__ ((deprecated("Renamed to GpioOutputSelections_t")))

/** Configure the IO pins of the device
 *
 * Select desired source from #GpioOutputSelections_t and configure selected IO accordingly
 *
 * @param[in] IO IO pin number
 * @param[in] selection signal source to connect to the IO pin
 * @return success or not
 * @retval 1 in case the pin is correctly configured
 * @retval 0 in case the pin cannot be configured
 */
bool gpio_configureOutput(GpioIo_t IO, GpioOutputSelections_t selection);

/** Enable the interrupt of an IO
 *
 * @param[in] IO IO pin number
 * @return success or not
 * @retval 1 in case the pin is correctly configured
 * @retval 0 in case the pin cannot be configured
 */
bool gpio_enableInterrupt(GpioIo_t IO);

/** Enable the IO0 open drain output.
 */
STATIC INLINE void gpio_enableOpenDrain0(void);

/** Enable the push pull output of IO 0
 */
STATIC INLINE void gpio_enablePushPull0(void);

/** Enable the open drain output of IO 1
 */
STATIC INLINE void gpio_enableOpenDrain1(void);

/** Enable the push pull output of IO 1
 */
STATIC INLINE void gpio_enablePushPull1(void);

/** Enable the open drain output of IO 2
 */
STATIC INLINE void gpio_enableOpenDrain2(void);

/** Enable the push pull output of IO 2
 */
STATIC INLINE void gpio_enablePushPull2(void);

/** Enable the open drain output of IO 3
 */
STATIC INLINE void gpio_enableOpenDrain3(void);

/** Enable the push pull output of IO 3
 */
STATIC INLINE void gpio_enablePushPull3(void);

/** Enable the open drain output of IO 4
 */
STATIC INLINE void gpio_enableOpenDrain4(void);

/** Enable the push pull output of IO 4
 */
STATIC INLINE void gpio_enablePushPull4(void);

/** Enable high voltage cell of IO 0
 */
STATIC INLINE void gpio_io0HvEnable(void);

/** Disable the high voltage cell of IO 0
 */
STATIC INLINE void gpio_io0HvDisable(void);

/** Enable high voltage cell of IO 1
 */
STATIC INLINE void gpio_io1HvEnable(void);

/** Disable the high voltage cell of IO 1
 */
STATIC INLINE void gpio_io1HvDisable(void);

/** Enable high voltage cell of IO 2
 */
STATIC INLINE void gpio_io2HvEnable(void);

/** Disable the high voltage cell of IO 2
 */
STATIC INLINE void gpio_io2HvDisable(void);

/** Enable the HS transistor of IO 0
 */
STATIC INLINE void gpio_io0HsEnable(void);

/** Disable the HS transistor of IO 0
 */
STATIC INLINE void gpio_io0HsDisable(void);

/** Enable the HS transistor of IO 1
 */
STATIC INLINE void gpio_io1HsEnable(void);

/** Disable the HS transistor of IO 1
 */
STATIC INLINE void gpio_io1HsDisable(void);

/** Enable the HS transistor of IO 2
 */
STATIC INLINE void gpio_io2HsEnable(void);

/** Disable the HS transistor of IO 2
 */
STATIC INLINE void gpio_io2HsDisable(void);

#if !defined(UNITTEST) /* for unit test, mocks will be generated */
#include "lib_gpio_inline_impl.h"
#endif

#endif

