/** @file
 *  @brief Header file for various CRC calculation functions
 *  @internal
 *
 *  @copyright (C) Melexis N.V.
 * git flash d0014c23
 *
 *  Melexis N.V. is supplying this code for use with Melexis N.V. processor based microcontrollers only.
 *
 *  THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY,
 *  INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.  MELEXIS N.V. SHALL NOT IN ANY CIRCUMSTANCES,
 *  BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 *  @endinternal
 *
 *  @ingroup mathlib
 *
 *  @details
 *
 *  This header file is the starting point for all the CRC-related functionality.
 *
 *  The complete set of headers containing CRC calculation functions is:
 *
 *  - crc.h: contains declarations for non-generated CRC implementation, written by hand for optimization or increased clarity
 *  - crc_gen.h: contains declarations for the generated CRC implementations, see below.
 *
 *  The second file is included by this file. In case you need a function declared in crc_gen.h,
 *  we still recommend including crc.h in order to be more robust in case headers get reorganized
 *  in the future.
 */


/** \defgroup crc CRC calculation
 *  \ingroup mathlib
 *
 *  There are 3 categories of CRC functions available:
 *
 *  - Purely algorithmic implementations: crc_algorithms
 *  - Table-based implementations: crc_table
 *  - Table-based implementation on a full dataset: crc_table_dataset
 */


/** \defgroup crc_algorithmic Purely algorithmic implementations
 *  \ingroup crc
 *
 *  Purely algorithmic implementations:
 *
 *  \b Function \b prototype: \a crc{crc_size}(r)_{polynomial}
 *
 *  \b Example: uint16_t crc16_1021 (uint8_t c, uint16_t seed)
 *
 *  These functions calculate the crc for a single byte using a purely algorithmic approach.\n
 *  Both normal (crc{crc_size} prefix) and reflected (crc{crc_size}r prefix) are supported
 *
 *  With:
 *
 *  - Supported crc sizes: 8, 16, 32
 *  - Supported polynomials: any polynomial valid for that CRC type is supported
 *
 *  In case the {polynomial} part is omitted, the following polynomials are taken: 0xD5 (CRC8), 0x1021 (CRC16) or 0x04C11DB7 (CRC32)
 *
 *  In case the specific combination of parameters for your application is not available by default
 *  you can request a custom mathlib release to be made.
 */

/** \defgroup crc_table Table-based implementations
 *  \ingroup crc
 *
 *  Table-based implementations
 *
 *  \b Function \b prototype: \a crc{crc_size}r_{polynomial}_table{table_size}
 *
 *  \b Example: uint16_t crc16r_8408_table256 (uint8_t c, uint16_t seed)
 *
 *  These functions calculate the crc for a single byte using a lookup table approach.\n
 *  3 table sizes are supported: 16 elements, 256 elements and a dual 16-element table. The choice is a tradeof between size and speed.
 *  Both normal (crc{crc_size} prefix) and reflected (crc{crc_size}r prefix) are supported
 *
 *  With:
 *
 *  - Supported crc sizes: 8, 16, 32
 *  - Supported polynomials: any polynomial valid for that CRC type is supported
 *
 *  In case the {polynomial} part is omitted, the following polynomials are taken: 0xD5 (CRC8), 0x1021 (CRC16) or 0x04C11DB7 (CRC32)
 *
 *  - Supported table sizes: 16, 256 or dual 16
 *
 *  In case the specific combination of parameters for your application is not available by default
 *  you can request a custom mathlib release to be made.
 */

/** \defgroup crc_table_dataset Table-based implementation on a full dataset
 *  \ingroup crc
 *
 *  Table-based implementation on a full dataset:
 *
 *  \b Function \b prototype: \a crc{crc_size}r_{polynomial}_table{table_size}_data{data_width}
 *
 *  \b Example: uint16_t crc16r_8408_table256_data16 (const uint16_t *data, uint16_t length, uint16_t init)
 *
 *  Same options as for the table-based implementations described above are available. The calculation is also
 *  performed similarly, just on a larger dataset rather than a single byte \n
 *  Both normal (crc{crc_size} prefix) and reflected (crc{crc_size}r prefix) are supported
 *
 *  With:
 *
 *  - Supported crc sizes: 8, 16, 32
 *  - Supported polynomials: any polynomial valid for that CRC type is supported
 *
 *  In case the {polynomial} part is omitted, the following polynomials are taken: 0xD5 (CRC8), 0x1021 (CRC16) or 0x04C11DB7 (CRC32)
 *
 *  - Supported table sizes: 16, 256 or dual 16
 *  - Supported data widths for full dataset calculations: 8, 16, 32.
 *
 *  In case the specific combination of parameters for your application is not available by default
 *  you can request a custom mathlib release to be made.
 */

 /** \addtogroup crc
 *  @{
 */

#ifndef MATHLIB_CRC_H_
#define MATHLIB_CRC_H_

#include <stdint.h>
#include "crc_gen.h"

/** CRC-16 CCITT (polynome 0x1021, non-reflected) implementation on a single byte using a purely algorithmic approach
 *
 *  @param[in]  c       Value to calculate seed for
 *  @param[in]  seed    Seed value for CRC calculation
 *
 *  @return     Calculated CRC value
 */
extern uint16_t crc_ccitt(uint8_t c, uint16_t seed);

#endif /* MATHLIB_CRC_H_ */

/** @}*/
