/**
 * @file
 * @brief Common library for the hw_diag module ITC HW
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
 * @brief Common library for the hw_diag module ITC HW
 * @details
 * @{
 */

#ifndef HW_DIAG_ITC_H
#define HW_DIAG_ITC_H

#include <stdint.h>
#include "io.h"
#include "compiler_abstraction.h"

#define HW_DIAG_ITC_REC_SIZE  2u

/** Specifies the amount of 16bit words to store the basic data content being checked */
#if defined(IO_MLX16__ITC_MASK8_GET)
#define HW_DIAG_ITC_DATA_SIZE        (9u * HW_DIAG_ITC_REC_SIZE)
#elif defined(IO_MLX16__ITC_MASK7_GET)
#define HW_DIAG_ITC_DATA_SIZE        (8u * HW_DIAG_ITC_REC_SIZE)
#elif defined(IO_MLX16__ITC_MASK6_GET)
#define HW_DIAG_ITC_DATA_SIZE        (7u * HW_DIAG_ITC_REC_SIZE)
#elif defined(IO_MLX16__ITC_MASK5_GET)
#define HW_DIAG_ITC_DATA_SIZE        (6u * HW_DIAG_ITC_REC_SIZE)
#elif defined(IO_MLX16__ITC_MASK4_GET)
#define HW_DIAG_ITC_DATA_SIZE        (5u * HW_DIAG_ITC_REC_SIZE)
#elif defined(IO_MLX16__ITC_MASK3_GET)
#define HW_DIAG_ITC_DATA_SIZE        (4u * HW_DIAG_ITC_REC_SIZE)
#elif defined(IO_MLX16__ITC_MASK2_GET)
#define HW_DIAG_ITC_DATA_SIZE        (3u * HW_DIAG_ITC_REC_SIZE)
#elif defined(IO_MLX16__ITC_MASK1_GET)
#define HW_DIAG_ITC_DATA_SIZE        (2u * HW_DIAG_ITC_REC_SIZE)
#else
#define HW_DIAG_ITC_DATA_SIZE        (HW_DIAG_ITC_REC_SIZE)
#endif /* ITC_MASKn */

STATIC INLINE void GetItcDataHwDiagFootprint(uint16_t buf[]);
STATIC INLINE uint16_t GetItcDataHwDiagFootprintCs(void);

#ifndef UNITTEST
#include "hw_diag_itc_inline_impl.h"
#endif /* UNITTEST */

#endif /* HW_DIAG_ITC_H */
/// @}

