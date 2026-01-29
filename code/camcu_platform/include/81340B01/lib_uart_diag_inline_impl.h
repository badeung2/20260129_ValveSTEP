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

#ifndef LIB_UART_DIAG_INLINE_IMPL_H
#define LIB_UART_DIAG_INLINE_IMPL_H

#include <stdint.h>
#include "io.h"
#include "static_assert.h"
#include "compiler_abstraction.h"
#include "sys_tools.h"
#include "hw_diag.h"


#ifndef HAS_TWO_UARTS
/** Fills-in an array of an UART HW registers data with the current state
 *
 * all UART HW configuaration is saved
 *
 * @param[in] buf   specifies an array to write into
 */
STATIC INLINE void GetUartDataHwDiagFootprint(uint16_t buf[])
{
    uint16_t buf_offset = 0u;
    buf[buf_offset] = IO_GET(UART, BRRD);
    ++buf_offset;
    buf[buf_offset] = IO_HOST(UART, LFCD);
    ++buf_offset;
    buf[buf_offset] = IO_HOST(UART, TRE);
}

/* Additional assertions assuring the fields are properly grouped. If ok, IO_MASK() will not fail */
ASSERT( IO_MASK(UART, LFCD, LFDD) != 0u );
ASSERT( IO_MASK(UART, TRE, REE, ISB, LBE, LSE, LTE, MLS, BSC, LBC, LSC, LDC) != 0u );


/** Returns the checksum for the trimming values' set
 * @return checksum
 */
STATIC INLINE uint16_t GetUartDataHwDiagFootprintCs(void)
{
    uint16_t buf[HW_DIAG_UART_DATA_SIZE];
    GetUartDataHwDiagFootprint(buf);
    return HW_DIAG_GetDataCs(buf, HW_DIAG_UART_DATA_SIZE);
}
#else /* HAS_TWO_UARTS */
/** Fills-in an array of an UART HW registers data with the current state
 *
 * all UART HW configuaration is saved
 *
 * @param[in] buf   specifies an array to write into
 */
STATIC INLINE void GetUart0DataHwDiagFootprint(uint16_t buf[])
{
    uint16_t buf_offset = 0u;
    buf[buf_offset] = IO_GET(UART0, BRRD);
    ++buf_offset;
    buf[buf_offset] = IO_HOST(UART0, LFCD);
    ++buf_offset;
    buf[buf_offset] = IO_HOST(UART0, TRE);
}

/* Additional assertions assuring the fields are properly grouped. If ok, IO_MASK() will not fail */
ASSERT( IO_MASK(UART0, LFCD, LFDD) != 0u );
ASSERT( IO_MASK(UART0, TRE, REE, ISB, LBE, LSE, LTE, MLS, BSC, LBC, LSC, LDC) != 0u );


/** Returns the checksum for the trimming values' set
 * @return checksum
 */
STATIC INLINE uint16_t GetUart0DataHwDiagFootprintCs(void)
{
    uint16_t buf[HW_DIAG_UART_DATA_SIZE];
    GetUart0DataHwDiagFootprint(buf);
    return HW_DIAG_GetDataCs(buf, HW_DIAG_UART_DATA_SIZE);
}



/** Fills-in an array of an UART HW registers data with the current state
 *
 * all UART HW configuaration is saved
 *
 * @param[in] buf   specifies an array to write into
 */
STATIC INLINE void GetUart1DataHwDiagFootprint(uint16_t buf[])
{
    uint16_t buf_offset = 0u;
    buf[buf_offset] = IO_GET(UART1, BRRD);
    ++buf_offset;
    buf[buf_offset] = IO_HOST(UART1, LFCD);
    ++buf_offset;
    buf[buf_offset] = IO_HOST(UART1, TRE);
}

/* Additional assertions assuring the fields are properly grouped. If ok, IO_MASK() will not fail */
ASSERT( IO_MASK(UART1, LFCD, LFDD) != 0u );
ASSERT( IO_MASK(UART1, TRE, REE, ISB, LBE, LSE, LTE, MLS, BSC, LBC, LSC, LDC) != 0u );


/** Returns the checksum for the trimming values' set
 * @return checksum
 */
STATIC INLINE uint16_t GetUart0DataHwDiagFootprintCs(void)
{
    uint16_t buf[HW_DIAG_UART_DATA_SIZE];
    GetUart1DataHwDiagFootprint(buf);
    return HW_DIAG_GetDataCs(buf, HW_DIAG_UART_DATA_SIZE);
}

#endif  /* HAS_TWO_UARTS */

#endif /* LIB_UART_DIAG_INLINE_IMPL_H */
/// @}

