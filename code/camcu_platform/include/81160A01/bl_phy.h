/**
 * @file
 * @brief The common bootloader's interface physical layer.
 * @internal
 *
 * @copyright (C) 2015-2017 Melexis N.V.
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
 * @ingroup bootloaders
 *
 * @details
 * The PHY-layer wrapper used to manage the Melibus, fast-protocol LIN and the SPI physical interfaces.
 * All functions from the interfaces should be provided externally in case if they will be used.
 * @{
 */

#ifndef BL_PHY_H
#define BL_PHY_H

#include <stdint.h>

/* The outside info should can have some set from the following defines:
   BL_PHY_USE_MELIBUS
   BL_PHY_USE_PPM
   BL_PHY_USE_SPI
 */

/** PHY layer state type */
typedef enum {
    phyDisabled = 0u, /**< PHY is disabled */
    phyEnabled,     /**< PHY is enabled */
} phyState_t;

/// @}
#endif
