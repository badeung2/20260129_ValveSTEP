/**
 * @file
 * @brief Common library for the hw_diag module Complex timer
 * @internal
 *
 * @copyright (C) 2019 Melexis N.V.
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
 * @ingroup ctimer
 *
 * @brief Common library for the hw_diag module Complex timer
 * @details
 *
 * @{
 */

#ifndef CTIMER_LIB_DIAG_H
#define CTIMER_LIB_DIAG_H

#include <stdint.h>
#include "io.h"
#include "compiler_abstraction.h"

#if defined(HAS_HW_DIAG_CTIMER0_ENABLE)

#define HW_DIAG_CTIMER0_DATA_SIZE   (3u)

/** Fills-in an array of an CTIMER0 HW registers data with the current state
 *
 * all CTIMER0 HW configuaration is saved
 *
 * @param[in] buf   specifies an array to write into
 */
STATIC INLINE void GetCtimer0DataHwDiagFootprint(uint16_t buf[]);

/** Returns the checksum for the trimming values' set
 * @return checksum
 */
STATIC INLINE uint16_t GetCtimer0DataHwDiagFootprintCs(void);

#endif /* HAS_HW_DIAG_CTIMER0_ENABLE */

#if defined(IO_CTIMER1__START_GET) && defined(HAS_HW_DIAG_CTIMER1_ENABLE)

#define HW_DIAG_CTIMER1_DATA_SIZE   (3u)

/** Fills-in an array of an CTIMER1 HW registers data with the current state
 *
 * all CTIMER1 HW configuaration is saved
 *
 * @param[in] buf   specifies an array to write into
 */
STATIC INLINE void GetCtimer1DataHwDiagFootprint(uint16_t buf[]);

/** Returns the checksum for the trimming values' set
 * @return checksum
 */
STATIC INLINE uint16_t GetCtimer1DataHwDiagFootprintCs(void);

#endif /* CTIMER1 */

#ifndef UNITTEST
#include "ctimerlib_diag_inline_impl.h"
#endif /* UNITTEST */

/// @}
#endif /* CTIMER_LIB_DIAG_H */

