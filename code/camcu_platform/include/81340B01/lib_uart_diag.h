/**
 * @file
 * @brief Common library for the hw_diag module UART
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
 * @ingroup lib_uart
 *
 * @brief Common library for the hw_diag module UART
 * @details
 * @{
 */

#ifndef LIB_UART_DIAG_H
#define LIB_UART_DIAG_H

#include <stdint.h>
#include "io.h"
#include "compiler_abstraction.h"

#define HW_DIAG_UART_DATA_SIZE  3u

#ifndef HAS_TWO_UARTS
STATIC INLINE void GetUartDataHwDiagFootprint(uint16_t buf[]);
STATIC INLINE uint16_t GetUartDataHwDiagFootprintCs(void);
#else /* HAS_TWO_UARTS */
STATIC INLINE void GetUart0DataHwDiagFootprint(uint16_t buf[]);
STATIC INLINE uint16_t GetUart0DataHwDiagFootprintCs(void);

STATIC INLINE void GetUart1DataHwDiagFootprint(uint16_t buf[]);
STATIC INLINE uint16_t GetUart1DataHwDiagFootprintCs(void);
#endif /* HAS_TWO_UARTS */

#ifndef UNITTEST
#include "lib_uart_diag_inline_impl.h"
#endif /* UNITTEST */

#endif /* LIB_UART_DIAG_H */
/// @}

