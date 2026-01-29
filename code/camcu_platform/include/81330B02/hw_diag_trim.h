
/**
 * @file
 * @brief Common library for the hw_diag module
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
 * @ingroup hw_diag
 * @ingroup startup
 *
 * @brief Common library for the hw_diag module
 * @details
 * @{
 */

#ifndef HW_DIAG_TRIM_H
#define HW_DIAG_TRIM_H

#include <stdint.h>
#include "compiler_abstraction.h"


#define HW_DIAG_TRIM_DATA_SIZE 6

STATIC INLINE void GetTrimDataHwDiagFootprint(uint16_t buf[]);
STATIC INLINE uint16_t GetTrimDataHwDiagFootprintCs(void);

#ifndef UNITTEST
#include "hw_diag_trim_inline_impl.h"
#endif /* UNITTEST */

#endif /* HW_DIAG_TRIM_H */
/// @}
