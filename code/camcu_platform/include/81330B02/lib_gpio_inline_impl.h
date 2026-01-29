/**
 * @file
 * @brief headerfile for the library to connect the IO pins to the different signals
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
 * @ingroup lib_gpio
 *
 * @brief general purpose IO library
 * @details
 * @{
 */

#ifndef LIB_GPIO_INLINE_IMPL_H
#define LIB_GPIO_INLINE_IMPL_H

#include "compiler_abstraction.h"
#include "io.h"

#if defined(HAS_HW_IO0_OD)
/** Enable the open drain output of IO
 */
STATIC INLINE void gpio_enableOpenDrain0(void)
{
    IO_SET(PORT_IO_OUT_EN, IO0_OD_ENABLE, 1u);
}

/** Enable the push pull output of IO 0
 */
STATIC INLINE void gpio_enablePushPull0(void)
{
    IO_SET(PORT_IO_OUT_EN, IO0_OD_ENABLE, 0u);
}
#elif defined(HAS_HW_IO_12_CONFIGURABLE_GPIO)
/** Enable the open drain output of IO 0
 */
STATIC INLINE void gpio_enableOpenDrain0(void)
{
    IO_HOST(PORT_IO_OUT_EN, IO_OD_ENABLE) |= (1u << 0);
}

/** Enable the push pull output of IO 0
 */
STATIC INLINE void gpio_enablePushPull0(void)
{
    IO_HOST(PORT_IO_OUT_EN, IO_OD_ENABLE) &= ~(1u << 0);
}

/** Enable the open drain output of IO 1
 */
STATIC INLINE void gpio_enableOpenDrain1(void)
{
    IO_HOST(PORT_IO_OUT_EN, IO_OD_ENABLE) |= (1u << 1);
}

/** Enable the push pull output of IO 1
 */
STATIC INLINE void gpio_enablePushPull1(void)
{
    IO_HOST(PORT_IO_OUT_EN, IO_OD_ENABLE) &= ~(1u << 1);
}

/** Enable the open drain output of IO 2
 */
STATIC INLINE void gpio_enableOpenDrain2(void)
{
    IO_HOST(PORT_IO_OUT_EN, IO_OD_ENABLE) |= (1u << 2);
}

/** Enable the push pull output of IO 2
 */
STATIC INLINE void gpio_enablePushPull2(void)
{
    IO_HOST(PORT_IO_OUT_EN, IO_OD_ENABLE) &= ~(1u << 2);
}

/** Enable the open drain output of IO 3
 */
STATIC INLINE void gpio_enableOpenDrain3(void)
{
    IO_HOST(PORT_IO_OUT_EN, IO_OD_ENABLE) |= (1u << 3);
}

/** Enable the push pull output of IO 3
 */
STATIC INLINE void gpio_enablePushPull3(void)
{
    IO_HOST(PORT_IO_OUT_EN, IO_OD_ENABLE) &= ~(1u << 3);
}

/** Enable the open drain output of IO 4
 */
STATIC INLINE void gpio_enableOpenDrain4(void)
{
    IO_HOST(PORT_IO_OUT_EN, IO_OD_ENABLE) |= (1u << 4);
}

/** Enable the push pull output of IO 4
 */
STATIC INLINE void gpio_enablePushPull4(void)
{
    IO_HOST(PORT_IO_OUT_EN, IO_OD_ENABLE) &= ~(1u << 4);
}
#endif

#if defined(HAS_HW_IO0_HV)
/** Enable high voltage cell of IO 0
 */
STATIC INLINE void gpio_io0HvEnable(void)
{
    IO_SET(PORT_IO_OUT_EN, IO0_LV_ENABLE, 0u);
}

/** Disable the high voltage cell of IO 0
 */
STATIC INLINE void gpio_io0HvDisable(void)
{
    IO_SET(PORT_IO_OUT_EN, IO0_LV_ENABLE, 1u);
}

/** Enable low voltage cell of IO 0
 */
STATIC INLINE void gpio_io0LvEnable(void)
{
    IO_SET(PORT_IO_OUT_EN, IO0_LV_ENABLE, 1u);
}

/** Disable the low voltage cell of IO 0
 */
STATIC INLINE void gpio_io0LvDisable(void)
{
    IO_SET(PORT_IO_OUT_EN, IO0_LV_ENABLE, 0u);
}

#elif defined(HAS_HW_IO_12_CONFIGURABLE_GPIO)
/** Enable high voltage cell of IO 0
 */
STATIC INLINE void gpio_io0HvEnable(void)
{
    IO_HOST(PORT_IO_OUT_EN, IO_HV_ENABLE) |= ((uint16_t)1 << (0u + IO_OFFSET(PORT_IO_OUT_EN, IO_HV_ENABLE)));
}

/** Disable the high voltage cell of IO 0
 */
STATIC INLINE void gpio_io0HvDisable(void)
{
    IO_HOST(PORT_IO_OUT_EN, IO_HV_ENABLE) &= ~((uint16_t)1 << (0u + IO_OFFSET(PORT_IO_OUT_EN, IO_HV_ENABLE)));
}

/** Enable high voltage cell of IO 1
 */
STATIC INLINE void gpio_io1HvEnable(void)
{
    IO_HOST(PORT_IO_OUT_EN, IO_HV_ENABLE) |= ((uint16_t)1 << (1u + IO_OFFSET(PORT_IO_OUT_EN, IO_HV_ENABLE)));
}

/** Disable the high voltage cell of IO 1
 */
STATIC INLINE void gpio_io1HvDisable(void)
{
    IO_HOST(PORT_IO_OUT_EN, IO_HV_ENABLE) &= ~((uint16_t)1 << (1u + IO_OFFSET(PORT_IO_OUT_EN, IO_HV_ENABLE)));
}

/** Enable high voltage cell of IO 2
 */
STATIC INLINE void gpio_io2HvEnable(void)
{
    IO_HOST(PORT_IO_OUT_EN, IO_HV_ENABLE) |= ((uint16_t)1 << (2u + IO_OFFSET(PORT_IO_OUT_EN, IO_HV_ENABLE)));
}

/** Disable the high voltage cell of IO 2
 */
STATIC INLINE void gpio_io2HvDisable(void)
{
    IO_HOST(PORT_IO_OUT_EN, IO_HV_ENABLE) &= ~((uint16_t)1 << (2u + IO_OFFSET(PORT_IO_OUT_EN, IO_HV_ENABLE)));
}
#ifndef IO_PORT_IO_OUT_EN__UNUSED
/** Enable high voltage cell of IO 3
 */
STATIC INLINE void gpio_io3HvEnable(void)
{
    IO_HOST(PORT_IO_OUT_EN, IO_HV_ENABLE) |= ((uint16_t)1 << (3u + IO_OFFSET(PORT_IO_OUT_EN, IO_HV_ENABLE)));
}

/** Disable the high voltage cell of IO 3
 */
STATIC INLINE void gpio_io3HvDisable(void)
{
    IO_HOST(PORT_IO_OUT_EN, IO_HV_ENABLE) &= ~((uint16_t)1 << (3u + IO_OFFSET(PORT_IO_OUT_EN, IO_HV_ENABLE)));
}

/** Enable high voltage cell of IO 4
 */
STATIC INLINE void gpio_io4HvEnable(void)
{
    IO_HOST(PORT_IO_OUT_EN, IO_HV_ENABLE) |= ((uint16_t)1 << (4u + IO_OFFSET(PORT_IO_OUT_EN, IO_HV_ENABLE)));
}

/** Disable the high voltage cell of IO 4
 */
STATIC INLINE void gpio_io4HvDisable(void)
{
    IO_HOST(PORT_IO_OUT_EN, IO_HV_ENABLE) &= ~((uint16_t)1 << (4u + IO_OFFSET(PORT_IO_OUT_EN, IO_HV_ENABLE)));
}

/** Enable the HS transistor of IO 3
 */
STATIC INLINE void gpio_io3HsEnable(void)
{
    IO_HOST(PORT_IO_OUT_EN, IO_HS_ENABLE) |= ((uint16_t)1 << (3u + IO_OFFSET(PORT_IO_OUT_EN, IO_HS_ENABLE)));
}

/** Disable the HS transistor of IO 3
 */
STATIC INLINE void gpio_io3HsDisable(void)
{
    IO_HOST(PORT_IO_OUT_EN, IO_HS_ENABLE) &= ~((uint16_t)1 << (3u + IO_OFFSET(PORT_IO_OUT_EN, IO_HS_ENABLE)));
}

/** Enable the HS transistor of IO 4
 */
STATIC INLINE void gpio_io4HsEnable(void)
{
    IO_HOST(PORT_IO_OUT_EN, IO_HS_ENABLE) |= ((uint16_t)1 << (4u + IO_OFFSET(PORT_IO_OUT_EN, IO_HS_ENABLE)));
}

/** Disable the HS transistor of IO 4
 */
STATIC INLINE void gpio_io4HsDisable(void)
{
    IO_HOST(PORT_IO_OUT_EN, IO_HS_ENABLE) &= ~((uint16_t)1 << (4u + IO_OFFSET(PORT_IO_OUT_EN, IO_HS_ENABLE)));
}
#endif /* IO_PORT_IO_OUT_EN__UNUSED */
#endif

#ifdef HAS_HW_IOX_OD
/** Enable the IO 7 open drain output.
 */
STATIC INLINE void gpio_enableOpenDrain7(void)
{
    IO_SET(PORT_IO_OUT_EN, IOX_OD_ENABLE, 1u);
}

/** Enable the push pull output of IO 7
 */
STATIC INLINE void gpio_enablePushPull7(void)
{
    IO_SET(PORT_IO_OUT_EN, IOX_OD_ENABLE, 0u);
}
#endif /* HAS_HW_IOX_OD */

#endif
/// @}
