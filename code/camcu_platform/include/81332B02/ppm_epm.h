/**
 * @file ppm_epm.h
 * @brief The Enter-to PPM-mode of communication support
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
 * @ingroup ppm_bootloader
 * @addtogroup ppm_bootloader PPM Bootloader
 *
 * @brief The Enter-to PPM-mode of communication support
 * @details
 * @{
 */

#ifndef PPM_EPM_H
#define PPM_EPM_H

#include <syslib.h>

/** Specifies in 100us values the period to wait till the end of Enter PPM-MODE KEY */
#define PPM_EPM_TIMER_PERIOD  25u

/** PPM EPM timeout in clock cycles
 *
 * Timeout is 200 us
 *
 * @note This is only valid for the inital ROM phase since the initial MLX16 frequency is 24MHz.
 */
#define PPM_EPM_TIMEOUT  (24u * 200u)


/** PPM flash BIST result */
typedef enum {
    PPM_BIST_FAILED,    /**< Occurs when the BIST has failed and the EPM sequence is not on the PPM pin */
    PPM_BIST_OK,        /**< The BIST is OK and the EPM KEY sequence is not on the PPM pin */
    PPM_BIST_EPM,       /**< The EPM KEY sequence is detected on the PPM pin */
} ppm_FlashBistRes_t;

/** Configures and runs the HW BIST check for application FLASH memory and checks whether a PPM EPM is received.
 *
 * @retval  PPM_BIST_OK  No BIST errors where detected.
 * @retval  PPM_BIST_FAILED  A BIST signature or flash double-bit error was detected.
 * @retval  PPM_BIST_EPM  A PPM EPM signal was detected.
 */
ppm_FlashBistRes_t PPM_FLASH_BistTest(void);

#endif
/// @}
