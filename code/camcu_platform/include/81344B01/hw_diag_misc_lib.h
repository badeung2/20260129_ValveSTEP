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
 * @ingroup library
 *
 * @brief Common library for the hw_diag module
 * @details
 * @{
 */

#ifndef HW_DIAG_MISC_LIB_H
#define HW_DIAG_MISC_LIB_H

#include <stdint.h>
#include "compiler_abstraction.h"

/** Specifies the amount of 16bit words to store the basic data content being checked */
#define HW_DIAG_BASIC_DATA_SIZE        (3u)
STATIC INLINE void GetBasicDataDiagFootprint(uint16_t buf[]);
STATIC INLINE uint16_t GetBasicDataDiagFootprintCs(void);

#ifndef UNITTEST
#include "hw_diag_misc_lib_inline_impl.h"
#endif /* UNITTEST */

#endif /* HW_DIAG_MISC_LIB_H */
/// @}

