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
 * This function enables the interrupt on the rising edge of an IO port.
 * The interrupt service routine is not implemented and has to be
 * written by the application.
 * @param[in] IO IO pin number
 * @return success or not
 * @retval 1 in case the pin is correctly configured
 * @retval 0 in case the pin cannot be configured
 */
bool gpio_enableInterrupt(GpioIo_t IO);

/** Enable the open drain output of IO 0
 */
STATIC INLINE void gpio_enableOpenDrain0(void);

/** Disable the open drain output of IO 0
 */
STATIC INLINE void gpio_disableOpenDrain0(void);
#define gpio_enablePushPull0 gpio_disableOpenDrain0 __attribute__ ((deprecated("Renamed to gpio_disableOpenDrain0")))

/** Enable the open drain output of IO 1
 */
STATIC INLINE void gpio_enableOpenDrain1(void);

/** Disable the open drain output of IO 1
 */
STATIC INLINE void gpio_disableOpenDrain1(void);
#define gpio_enablePushPull1 gpio_disableOpenDrain1 __attribute__ ((deprecated("Renamed to gpio_disableOpenDrain1")))

/** Enable the open drain output of IO 2
 */
STATIC INLINE void gpio_enableOpenDrain2(void);

/** Disable the open drain output of IO 2
 */
STATIC INLINE void gpio_disableOpenDrain2(void);
#define gpio_enablePushPull2 gpio_disableOpenDrain2 __attribute__ ((deprecated("Renamed to gpio_disableOpenDrain2")))

/** Enable the open drain output of IO 3
 */
STATIC INLINE void gpio_enableOpenDrain3(void);

/** Disable the open drain output of IO 3
 */
STATIC INLINE void gpio_disableOpenDrain3(void);
#define gpio_enablePushPull3 gpio_disableOpenDrain3 __attribute__ ((deprecated("Renamed to gpio_disableOpenDrain3")))

/** Enable the open drain output of IO 4
 */
STATIC INLINE void gpio_enableOpenDrain4(void);

/** Disable the open drain output of IO 4
 */
STATIC INLINE void gpio_disableOpenDrain4(void);
#define gpio_enablePushPull4 gpio_disableOpenDrain4 __attribute__ ((deprecated("Renamed to gpio_disableOpenDrain4")))

/** Enable low voltage mode of IO 0
 */
STATIC INLINE void gpio_io0LvEnable(void);

/** Disable the low voltage mode of IO 0
 */
STATIC INLINE void gpio_io0LvDisable(void);

/** Enable low voltage mode of IO 1
 */
STATIC INLINE void gpio_io1LvEnable(void);

/** Disable the low voltage mode of IO 1
 */
STATIC INLINE void gpio_io1LvDisable(void);

/** Enable low voltage mode of IO 2
 */
STATIC INLINE void gpio_io2LvEnable(void);

/** Disable the low voltage mode of IO 2
 */
STATIC INLINE void gpio_io2LvDisable(void);

/** Enable low voltage mode of IO 3
 */
STATIC INLINE void gpio_io3LvEnable(void);

/** Disable the low voltage mode of IO 3
 */
STATIC INLINE void gpio_io3LvDisable(void);

/** Enable low voltage mode of IO 4
 */
STATIC INLINE void gpio_io4LvEnable(void);

/** Disable the low voltage mode of IO 4
 */
STATIC INLINE void gpio_io4LvDisable(void);

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

/** Enable the HS transistor of IO 3
 */
STATIC INLINE void gpio_io3HsEnable(void);

/** Disable the HS transistor of IO 3
 */
STATIC INLINE void gpio_io3HsDisable(void);

/** Enable the HS transistor of IO 4
 */
STATIC INLINE void gpio_io4HsEnable(void);

/** Disable the HS transistor of IO 4
 */
STATIC INLINE void gpio_io4HsDisable(void);

#if !defined(UNITTEST) /* for unit test, mocks will be generated */
#include "lib_gpio_inline_impl.h"
#endif

#endif
