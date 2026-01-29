/**
 * @file
 * @brief Motor driver support library
 * @internal
 *
 * @copyright (C) 2020 Melexis N.V.
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
 * @ingroup lib_motor_driver
 *
 * @brief Motor driver support library
 * @details
 * @{
 */

#ifndef LIB_MOTOR_DRIVER_INLINE_IMPL_H
#define LIB_MOTOR_DRIVER_INLINE_IMPL_H

#include "io.h"

#if !defined(HAS_HW_DRIVER_6_PHASES)
STATIC INLINE void MotorDriverUSelectSource(DrvCtrlSelect_t drvctrl)
{
    IO_SET(PORT_DRV_CTRL, DRV0_CTRL, (uint16_t)drvctrl);
}

STATIC INLINE void MotorDriverVSelectSource(DrvCtrlSelect_t drvctrl)
{
    IO_SET(PORT_DRV_CTRL, DRV1_CTRL, (uint16_t)drvctrl);
}

STATIC INLINE void MotorDriverWSelectSource(DrvCtrlSelect_t drvctrl)
{
    IO_SET(PORT_DRV_CTRL, DRV2_CTRL, (uint16_t)drvctrl);
}

STATIC INLINE void MotorDriverUVWSelectSource(DrvCtrlSelect_t drvctrl_u,
                                              DrvCtrlSelect_t drvctrl_v,
                                              DrvCtrlSelect_t drvctrl_w)
{
    IO_SET(PORT_DRV_CTRL,
           DRV0_CTRL, (uint16_t)drvctrl_u,
           DRV1_CTRL, (uint16_t)drvctrl_v,
           DRV2_CTRL, (uint16_t)drvctrl_w);
}

#ifdef HAS_HW_DRIVER_4_PHASES
STATIC INLINE void MotorDriverTSelectSource(DrvCtrlSelect_t drvctrl)
{
    IO_SET(PORT_DRV_CTRL, DRV3_CTRL, (uint16_t)drvctrl);
}

STATIC INLINE void MotorDriverUVWTSelectSource(DrvCtrlSelect_t drvctrl_u,
                                               DrvCtrlSelect_t drvctrl_v,
                                               DrvCtrlSelect_t drvctrl_w,
                                               DrvCtrlSelect_t drvctrl_t)
{
    IO_SET(PORT_DRV_CTRL,
           DRV0_CTRL, (uint16_t)drvctrl_u,
           DRV1_CTRL, (uint16_t)drvctrl_v,
           DRV2_CTRL, (uint16_t)drvctrl_w,
           DRV3_CTRL, (uint16_t)drvctrl_t);
}
#endif /* HAS_HW_DRIVER_4_PHASES */

#else /* HAS_HW_DRIVER_6_PHASES */

STATIC INLINE void MotorDriverUSelectSource(DrvCtrlSelect_t drvctrl)
{
    IO_SET(PORT_DRV1_CTRL, DRV3_CTRL, (uint16_t)drvctrl);
}

STATIC INLINE void MotorDriverVSelectSource(DrvCtrlSelect_t drvctrl)
{
    IO_SET(PORT_DRV2_CTRL, DRV4_CTRL, (uint16_t)drvctrl);
}

STATIC INLINE void MotorDriverWSelectSource(DrvCtrlSelect_t drvctrl)
{
    IO_SET(PORT_DRV2_CTRL, DRV5_CTRL, (uint16_t)drvctrl);
}

STATIC INLINE void MotorDriverRSelectSource(DrvCtrlSelect_t drvctrl)
{
    IO_SET(PORT_DRV1_CTRL, DRV0_CTRL, (uint16_t)drvctrl);
}

STATIC INLINE void MotorDriverSSelectSource(DrvCtrlSelect_t drvctrl)
{
    IO_SET(PORT_DRV1_CTRL, DRV1_CTRL, (uint16_t)drvctrl);
}

STATIC INLINE void MotorDriverTSelectSource(DrvCtrlSelect_t drvctrl)
{
    IO_SET(PORT_DRV1_CTRL, DRV2_CTRL, (uint16_t)drvctrl);
}

STATIC INLINE void MotorDriverUVWSelectSource(DrvCtrlSelect_t drvctrl_u,
                                              DrvCtrlSelect_t drvctrl_v,
                                              DrvCtrlSelect_t drvctrl_w)
{
    IO_SET(PORT_DRV1_CTRL,
           DRV3_CTRL, (uint16_t)drvctrl_u);
    IO_SET(PORT_DRV2_CTRL,
           DRV4_CTRL, (uint16_t)drvctrl_v,
           DRV5_CTRL, (uint16_t)drvctrl_w);
}

STATIC INLINE void MotorDriverRSTSelectSource(DrvCtrlSelect_t drvctrl_r,
                                              DrvCtrlSelect_t drvctrl_s,
                                              DrvCtrlSelect_t drvctrl_t)
{
    IO_SET(PORT_DRV1_CTRL,
           DRV0_CTRL, (uint16_t)drvctrl_r,
           DRV1_CTRL, (uint16_t)drvctrl_s,
           DRV2_CTRL, (uint16_t)drvctrl_t);
}

STATIC INLINE void MotorDriverUVWTSelectSource(DrvCtrlSelect_t drvctrl_u,
                                               DrvCtrlSelect_t drvctrl_v,
                                               DrvCtrlSelect_t drvctrl_w,
                                               DrvCtrlSelect_t drvctrl_t)
{
    IO_SET(PORT_DRV1_CTRL,
           DRV2_CTRL, (uint16_t)drvctrl_t,
           DRV3_CTRL, (uint16_t)drvctrl_u);
    IO_SET(PORT_DRV2_CTRL,
           DRV4_CTRL, (uint16_t)drvctrl_v,
           DRV5_CTRL, (uint16_t)drvctrl_w);
}

STATIC INLINE void MotorDriverUVWRSTSelectSource(DrvCtrlSelect_t drvctrl_u,
                                                 DrvCtrlSelect_t drvctrl_v,
                                                 DrvCtrlSelect_t drvctrl_w,
                                                 DrvCtrlSelect_t drvctrl_r,
                                                 DrvCtrlSelect_t drvctrl_s,
                                                 DrvCtrlSelect_t drvctrl_t)
{
    IO_SET(PORT_DRV1_CTRL,
           DRV0_CTRL, (uint16_t)drvctrl_r,
           DRV1_CTRL, (uint16_t)drvctrl_s,
           DRV2_CTRL, (uint16_t)drvctrl_t,
           DRV3_CTRL, (uint16_t)drvctrl_u);
    IO_SET(PORT_DRV2_CTRL,
           DRV4_CTRL, (uint16_t)drvctrl_v,
           DRV5_CTRL, (uint16_t)drvctrl_w);
}

#endif /* HAS_HW_DRIVER_6_PHASES */

#endif /* LIB_MOTOR_DRIVER_INLINE_IMPL_H */
/// @}
