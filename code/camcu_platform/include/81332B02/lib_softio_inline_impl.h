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

#ifndef LIB_SOFTIO_INLINE_IMPL_H
#define LIB_SOFTIO_INLINE_IMPL_H

#include "io.h"
#include "lib_gpio.h"
#include <stdbool.h>
#include <syslib.h>
#include "sys_tools.h"
#include "plib.h"

#if defined(HAS_HW_IO_12_CONFIGURABLE_GPIO)
/** Max number of IO pins in the chip */
#define MAXIO_INDEX GPIO_IO11
#elif defined(HAS_HW_IO_8_CONFIGURABLE_GPIO)
/** Max number of IO pins in the chip */
#define MAXIO_INDEX GPIO_IO7
#else
/** Max number of IO pins in the chip */
#define MAXIO_INDEX GPIO_IO3
#endif

STATIC INLINE uint8_t softio_get(GpioIo_t IO)
{
    uint8_t pinlevel = 0xFFu;
    if(IO <= MAXIO_INDEX) {
        pinlevel = (uint8_t)(IO_GET(PORT_IO_IN, IO_IN_SYNC) >> (uint16_t)IO) & 0x01u;
    }
    return pinlevel;
}

STATIC INLINE bool softio_set(GpioIo_t IO)
{
    bool success = false;
    if(IO <= MAXIO_INDEX) {
        IO_HOST(PORT_IO_OUT_SOFT, IO_OUT_SOFT) |= ((uint16_t)0x01u << (uint16_t)IO);
        success = true;
    }
    return success;
}

STATIC INLINE bool softio_clr(GpioIo_t IO)
{
    bool success = false;
    if(IO <= MAXIO_INDEX) {
        IO_HOST(PORT_IO_OUT_SOFT, IO_OUT_SOFT) &= ~((uint16_t)0x01u << (uint16_t)IO);
        success = true;
    }
    return success;
}

STATIC INLINE bool softio_toggle(GpioIo_t IO)
{
    bool success = false;
    if(IO <= MAXIO_INDEX) {
        IO_HOST(PORT_IO_OUT_SOFT, IO_OUT_SOFT) ^= ((uint16_t)0x01u << (uint16_t)IO);
        success = true;
    }
    return success;
}


#endif /* LIB_SOFTIO_INLINE_IMPL_H */
/// @}
