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
 * @addtogroup lib_gpio
 *
 * @details
 */

#ifndef LIB_GPIO_INLINE_IMPL_H
#define LIB_GPIO_INLINE_IMPL_H

#include "compiler_abstraction.h"
#include "io.h"

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

/** Enable high voltage cell of IO 0
 */
STATIC INLINE void gpio_io0HvEnable(void)
{
#ifdef IO_PORT_IO_OUT_EN__IO_LV_ENABLE
    IO_HOST(PORT_IO_OUT_EN, IO_LV_ENABLE) &= ~((uint16_t)1 << (0u + IO_OFFSET(PORT_IO_OUT_EN, IO_LV_ENABLE)));
#else
    IO_HOST(PORT_IO_OUT_EN, IO_HV_ENABLE) |= ((uint16_t)1 << (0u + IO_OFFSET(PORT_IO_OUT_EN, IO_HV_ENABLE)));
#endif
}

/** Disable the high voltage cell of IO 0
 */
STATIC INLINE void gpio_io0HvDisable(void)
{
#ifdef IO_PORT_IO_OUT_EN__IO_LV_ENABLE
    IO_HOST(PORT_IO_OUT_EN, IO_LV_ENABLE) |= ((uint16_t)1 << (0u + IO_OFFSET(PORT_IO_OUT_EN, IO_LV_ENABLE)));
#else
    IO_HOST(PORT_IO_OUT_EN, IO_HV_ENABLE) &= ~((uint16_t)1 << (0u + IO_OFFSET(PORT_IO_OUT_EN, IO_HV_ENABLE)));
#endif
}

/** Enable high voltage cell of IO 1
 */
STATIC INLINE void gpio_io1HvEnable(void)
{
#ifdef IO_PORT_IO_OUT_EN__IO_LV_ENABLE
    IO_HOST(PORT_IO_OUT_EN, IO_LV_ENABLE) &= ~((uint16_t)1 << (1u + IO_OFFSET(PORT_IO_OUT_EN, IO_LV_ENABLE)));
#else
    IO_HOST(PORT_IO_OUT_EN, IO_HV_ENABLE) |= ((uint16_t)1 << (1u + IO_OFFSET(PORT_IO_OUT_EN, IO_HV_ENABLE)));
#endif
}

/** Disable the high voltage cell of IO 1
 */
STATIC INLINE void gpio_io1HvDisable(void)
{
#ifdef IO_PORT_IO_OUT_EN__IO_LV_ENABLE
    IO_HOST(PORT_IO_OUT_EN, IO_LV_ENABLE) |= ((uint16_t)1 << (1u + IO_OFFSET(PORT_IO_OUT_EN, IO_LV_ENABLE)));
#else
    IO_HOST(PORT_IO_OUT_EN, IO_HV_ENABLE) &= ~((uint16_t)1 << (1u + IO_OFFSET(PORT_IO_OUT_EN, IO_HV_ENABLE)));
#endif
}

/** Enable high voltage cell of IO 2
 */
STATIC INLINE void gpio_io2HvEnable(void)
{
#ifdef IO_PORT_IO_OUT_EN__IO_LV_ENABLE
    IO_HOST(PORT_IO_OUT_EN, IO_LV_ENABLE) &= ~((uint16_t)1 << (2u + IO_OFFSET(PORT_IO_OUT_EN, IO_LV_ENABLE)));
#else
    IO_HOST(PORT_IO_OUT_EN, IO_HV_ENABLE) |= ((uint16_t)1 << (2u + IO_OFFSET(PORT_IO_OUT_EN, IO_HV_ENABLE)));
#endif
}

/** Disable the high voltage cell of IO 2
 */
STATIC INLINE void gpio_io2HvDisable(void)
{
#ifdef IO_PORT_IO_OUT_EN__IO_LV_ENABLE
    IO_HOST(PORT_IO_OUT_EN, IO_LV_ENABLE) |= ((uint16_t)1 << (2u + IO_OFFSET(PORT_IO_OUT_EN, IO_LV_ENABLE)));
#else
    IO_HOST(PORT_IO_OUT_EN, IO_HV_ENABLE) &= ~((uint16_t)1 << (2u + IO_OFFSET(PORT_IO_OUT_EN, IO_HV_ENABLE)));
#endif
}

/** Enable high voltage cell of IO 3
 */
STATIC INLINE void gpio_io3HvEnable(void)
{
#ifdef IO_PORT_IO_OUT_EN__IO_LV_ENABLE
    IO_HOST(PORT_IO_OUT_EN, IO_LV_ENABLE) &= ~((uint16_t)1 << (3u + IO_OFFSET(PORT_IO_OUT_EN, IO_LV_ENABLE)));
#else
    IO_HOST(PORT_IO_OUT_EN, IO_HV_ENABLE) |= ((uint16_t)1 << (3u + IO_OFFSET(PORT_IO_OUT_EN, IO_HV_ENABLE)));
#endif
}

/** Disable the high voltage cell of IO 3
 */
STATIC INLINE void gpio_io3HvDisable(void)
{
#ifdef IO_PORT_IO_OUT_EN__IO_LV_ENABLE
    IO_HOST(PORT_IO_OUT_EN, IO_LV_ENABLE) |= ((uint16_t)1 << (3u + IO_OFFSET(PORT_IO_OUT_EN, IO_LV_ENABLE)));
#else
    IO_HOST(PORT_IO_OUT_EN, IO_HV_ENABLE) &= ~((uint16_t)1 << (3u + IO_OFFSET(PORT_IO_OUT_EN, IO_HV_ENABLE)));
#endif
}

/** Enable high voltage cell of IO 4
 */
STATIC INLINE void gpio_io4HvEnable(void)
{
#ifdef IO_PORT_IO_OUT_EN__IO_LV_ENABLE
    IO_HOST(PORT_IO_OUT_EN, IO_LV_ENABLE) &= ~((uint16_t)1 << (4u + IO_OFFSET(PORT_IO_OUT_EN, IO_LV_ENABLE)));
#else
    IO_HOST(PORT_IO_OUT_EN, IO_HV_ENABLE) |= ((uint16_t)1 << (4u + IO_OFFSET(PORT_IO_OUT_EN, IO_HV_ENABLE)));
#endif
}

/** Disable the high voltage cell of IO 4
 */
STATIC INLINE void gpio_io4HvDisable(void)
{
#ifdef IO_PORT_IO_OUT_EN__IO_LV_ENABLE
    IO_HOST(PORT_IO_OUT_EN, IO_LV_ENABLE) |= ((uint16_t)1 << (4u + IO_OFFSET(PORT_IO_OUT_EN, IO_LV_ENABLE)));
#else
    IO_HOST(PORT_IO_OUT_EN, IO_HV_ENABLE) &= ~((uint16_t)1 << (4u + IO_OFFSET(PORT_IO_OUT_EN, IO_HV_ENABLE)));
#endif
}
/** Enable the HS transistor of IO 0
 */
STATIC INLINE void gpio_io0HsEnable(void)
{
    IO_HOST(PORT_IO_OUT_EN, IO_HS_ENABLE) |= ((uint16_t)1 << (0u + IO_OFFSET(PORT_IO_OUT_EN, IO_HS_ENABLE)));
}

/** Disable the HS transistor of IO 0
 */
STATIC INLINE void gpio_io0HsDisable(void)
{
    IO_HOST(PORT_IO_OUT_EN, IO_HS_ENABLE) &= ~((uint16_t)1 << (0u + IO_OFFSET(PORT_IO_OUT_EN, IO_HS_ENABLE)));
}

/** Enable the HS transistor of IO 1
 */
STATIC INLINE void gpio_io1HsEnable(void)
{
    IO_HOST(PORT_IO_OUT_EN, IO_HS_ENABLE) |= ((uint16_t)1 << (1u + IO_OFFSET(PORT_IO_OUT_EN, IO_HS_ENABLE)));
}

/** Disable the HS transistor of IO 1
 */
STATIC INLINE void gpio_io1HsDisable(void)
{
    IO_HOST(PORT_IO_OUT_EN, IO_HS_ENABLE) &= ~((uint16_t)1 << (1u + IO_OFFSET(PORT_IO_OUT_EN, IO_HS_ENABLE)));
}

/** Enable the HS transistor of IO 2
 */
STATIC INLINE void gpio_io2HsEnable(void)
{
    IO_HOST(PORT_IO_OUT_EN, IO_HS_ENABLE) |= ((uint16_t)1 << (2u + IO_OFFSET(PORT_IO_OUT_EN, IO_HS_ENABLE)));
}

/** Disable the HS transistor of IO 2
 */
STATIC INLINE void gpio_io2HsDisable(void)
{
    IO_HOST(PORT_IO_OUT_EN, IO_HS_ENABLE) &= ~((uint16_t)1 << (2u + IO_OFFSET(PORT_IO_OUT_EN, IO_HS_ENABLE)));
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

#endif

