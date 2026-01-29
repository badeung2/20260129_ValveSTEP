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
 * @addtogroup lib_motor_driver Motor Driver
 *
 * @brief Motor driver support library
 * @details
 * @{
 */

#ifndef LIB_MOTOR_DRIVER_H
#define LIB_MOTOR_DRIVER_H

#include <syslib.h>
#include "lib_miscio.h"
#include "sys_tools.h"

/** Select the source for the output for driver U
 * @param[in] drvctrl specifies which input source to be used by the driver to output.
 */
STATIC INLINE void MotorDriverUSelectSource(DrvCtrlSelect_t drvctrl);
#define MotorDriver0SelectSource MotorDriverUSelectSource

/** Select the source for the output for driver V
 * @param[in] drvctrl specifies which input source to be used by the driver to output.
 */
STATIC INLINE void MotorDriverVSelectSource(DrvCtrlSelect_t drvctrl);
#define MotorDriver1SelectSource MotorDriverVSelectSource

/** Select the source for the output for driver W
 * @param[in] drvctrl specifies which input source to be used by the driver to output.
 */
STATIC INLINE void MotorDriverWSelectSource(DrvCtrlSelect_t drvctrl);
#define MotorDriver2SelectSource MotorDriverWSelectSource

/** Select the source for the outputs of drivers U, V and W at once
 * @param[in] drvctrl_u specifies which input source to be used by the U driver to output.
 * @param[in] drvctrl_v specifies which input source to be used by the V driver to output.
 * @param[in] drvctrl_w specifies which input source to be used by the W driver to output.
 */
STATIC INLINE void MotorDriverUVWSelectSource(DrvCtrlSelect_t drvctrl_u,
                                              DrvCtrlSelect_t drvctrl_v,
                                              DrvCtrlSelect_t drvctrl_w);

#if defined(HAS_HW_DRIVER_6_PHASES)
/** Select the source for the output for driver R
 * @param[in] drvctrl specifies which input source to be used by the driver to output.
 */
STATIC INLINE void MotorDriverRSelectSource(DrvCtrlSelect_t drvctrl);

/** Select the source for the output for driver S
 * @param[in] drvctrl specifies which input source to be used by the driver to output.
 */
STATIC INLINE void MotorDriverSSelectSource(DrvCtrlSelect_t drvctrl);
#endif /* HAS_HW_DRIVER_6_PHASES */

#if defined(HAS_HW_DRIVER_4_PHASES) || defined(HAS_HW_DRIVER_6_PHASES)
/** Select the source for the output for driver T
 * @param[in] drvctrl specifies which input source to be used by the driver to output.
 */
STATIC INLINE void MotorDriverTSelectSource(DrvCtrlSelect_t drvctrl);
#define MotorDriver3SelectSource MotorDriverTSelectSource

/** Select the source for the output of drivers U, V, W and T at once
 * @param[in] drvctrl_u specifies which input source to be used by the U driver to output.
 * @param[in] drvctrl_v specifies which input source to be used by the V driver to output.
 * @param[in] drvctrl_w specifies which input source to be used by the W driver to output.
 * @param[in] drvctrl_t specifies which input source to be used by the T driver to output.
 */
STATIC INLINE void MotorDriverUVWTSelectSource(DrvCtrlSelect_t drvctrl_u,
                                               DrvCtrlSelect_t drvctrl_v,
                                               DrvCtrlSelect_t drvctrl_w,
                                               DrvCtrlSelect_t drvctrl_t);
#endif /* HAS_HW_DRIVER_4_PHASES || HAS_HW_DRIVER_6_PHASES */

#if defined(HAS_HW_DRIVER_6_PHASES)
/** Select the source for the output of drivers R, S and T at once
 * @param[in] drvctrl_r specifies which input source to be used by the R driver to output.
 * @param[in] drvctrl_s specifies which input source to be used by the S driver to output.
 * @param[in] drvctrl_t specifies which input source to be used by the T driver to output.
 */
STATIC INLINE void MotorDriverRSTSelectSource(DrvCtrlSelect_t drvctrl_r,
                                              DrvCtrlSelect_t drvctrl_s,
                                              DrvCtrlSelect_t drvctrl_t);

/** Select the source for the output of drivers U, V, W, R, S and T at once
 * @param[in] drvctrl_u specifies which input source to be used by the U driver to output.
 * @param[in] drvctrl_v specifies which input source to be used by the V driver to output.
 * @param[in] drvctrl_w specifies which input source to be used by the W driver to output.
 * @param[in] drvctrl_r specifies which input source to be used by the R driver to output.
 * @param[in] drvctrl_s specifies which input source to be used by the S driver to output.
 * @param[in] drvctrl_t specifies which input source to be used by the T driver to output.
 */
STATIC INLINE void MotorDriverUVWRSTSelectSource(DrvCtrlSelect_t drvctrl_u,
                                                 DrvCtrlSelect_t drvctrl_v,
                                                 DrvCtrlSelect_t drvctrl_w,
                                                 DrvCtrlSelect_t drvctrl_r,
                                                 DrvCtrlSelect_t drvctrl_s,
                                                 DrvCtrlSelect_t drvctrl_t);
#endif /* HAS_HW_DRIVER_6_PHASES */

#if !defined(UNITTEST)
#include "lib_motor_driver_inline_impl.h"
#endif /* UNITTEST */

#endif /* LIB_MOTOR_DRIVER_H */
/// @}
