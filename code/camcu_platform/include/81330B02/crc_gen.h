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
 */

/** \addtogroup crc
 *  @{
 */

#ifndef GEN_CRC_H_
#define GEN_CRC_H_

#include <stdint.h>


/** CRC-8 implementation on a single byte with polynome 0xD5u using a purely algorithmic approach
 *
 *  @param[in]  c       Value to calculate seed for
 *  @param[in]  seed    Seed value for CRC calculation
 *
 *  @return     Calculated CRC value
 *  @ingroup    crc_algorithmic
 */
uint8_t crc8 (uint8_t c, uint8_t seed);


/** CRC-8 implementation on a single byte with polynome 0xDu using a purely algorithmic approach
 *
 *  @param[in]  c       Value to calculate seed for
 *  @param[in]  seed    Seed value for CRC calculation
 *
 *  @return     Calculated CRC value
 *  @ingroup    crc_algorithmic
 */
uint8_t crc8_D (uint8_t c, uint8_t seed);


/** CRC-8 implementation on a single byte with polynome 0xD1u using a purely algorithmic approach
 *
 *  @param[in]  c       Value to calculate seed for
 *  @param[in]  seed    Seed value for CRC calculation
 *
 *  @return     Calculated CRC value
 *  @ingroup    crc_algorithmic
 */
uint8_t crc8_D1 (uint8_t c, uint8_t seed);



/** CRC-8 implementation on a single byte with polynome 0xD5u using a 16 element table lookup approach.
 *
 *  www.ece.cmu.edu/~koopman/roses/dsn04/koopman04_crc_poly_embedded.pdf
 *
 *  @param[in]  c     8-bit value to calculate seed for
 *  @param[in]  seed  8 - bit Seed value for CRC calculation
 *
 *  @return     Calculated CRC value
 *  @ingroup    crc_table
 */
uint8_t crc8_table16 (uint8_t c, uint8_t seed);



/** CRC-8 implementation on a single byte with polynome 0xD5u using a dual 16 element table lookup approach.
 *
 *  www.ece.cmu.edu/~koopman/roses/dsn04/koopman04_crc_poly_embedded.pdf
 *
 *  @param[in]  c     8-bit value to calculate seed for
 *  @param[in]  seed  8 - bit Seed value for CRC calculation
 *
 *  @return     Calculated CRC value
 *  @ingroup    crc_table
 */
uint8_t crc8_tabletwo16 (uint8_t c, uint8_t seed);



/** CRC-8 implementation on a single byte with polynome 0xD5u using a 256 element table lookup approach.
 *
 *  www.ece.cmu.edu/~koopman/roses/dsn04/koopman04_crc_poly_embedded.pdf
 *
 *  @param[in]  c     8-bit value to calculate seed for
 *  @param[in]  seed  8 - bit Seed value for CRC calculation
 *
 *  @return     Calculated CRC value
 *  @ingroup    crc_table
 */
uint8_t crc8_table256 (uint8_t c, uint8_t seed);


/** Reflected CRC-8 implementation on a single byte with polynome 0xD5u using a purely algorithmic approach
 *
 *  @param[in]  c       Value to calculate seed for
 *  @param[in]  seed    Seed value for CRC calculation
 *
 *  @return     Calculated CRC value
 *  @ingroup    crc_algorithmic
 */
uint8_t crc8r (uint8_t c, uint8_t seed);



/** CRC-8 implementation on a single byte with polynome 0xD5u using a 16 element table lookup approach.
 *
 *  www.ece.cmu.edu/~koopman/roses/dsn04/koopman04_crc_poly_embedded.pdf
 *
 *  @param[in]  c     8-bit value to calculate seed for
 *  @param[in]  seed  8 - bit Seed value for CRC calculation
 *
 *  @return     Calculated CRC value
 *  @ingroup    crc_table
 */
uint8_t crc8r_table16 (uint8_t c, uint8_t seed);



/** CRC-8 implementation on a single byte with polynome 0xD5u using a dual 16 element table lookup approach.
 *
 *  www.ece.cmu.edu/~koopman/roses/dsn04/koopman04_crc_poly_embedded.pdf
 *
 *  @param[in]  c     8-bit value to calculate seed for
 *  @param[in]  seed  8 - bit Seed value for CRC calculation
 *
 *  @return     Calculated CRC value
 *  @ingroup    crc_table
 */
uint8_t crc8r_tabletwo16 (uint8_t c, uint8_t seed);



/** CRC-8 implementation on a single byte with polynome 0xD5u using a 256 element table lookup approach.
 *
 *  www.ece.cmu.edu/~koopman/roses/dsn04/koopman04_crc_poly_embedded.pdf
 *
 *  @param[in]  c     8-bit value to calculate seed for
 *  @param[in]  seed  8 - bit Seed value for CRC calculation
 *
 *  @return     Calculated CRC value
 *  @ingroup    crc_table
 */
uint8_t crc8r_table256 (uint8_t c, uint8_t seed);


/** Reflected CRC-8 implementation on a single byte with polynome 0xABu using a purely algorithmic approach
 *
 *  @param[in]  c       Value to calculate seed for
 *  @param[in]  seed    Seed value for CRC calculation
 *
 *  @return     Calculated CRC value
 *  @ingroup    crc_algorithmic
 */
uint8_t crc8r_AB (uint8_t c, uint8_t seed);



/** CRC-8 implementation on a single byte with polynome 0xABu using a 16 element table lookup approach.
 *
 *  www.ece.cmu.edu/~koopman/roses/dsn04/koopman04_crc_poly_embedded.pdf
 *
 *  @param[in]  c     8-bit value to calculate seed for
 *  @param[in]  seed  8 - bit Seed value for CRC calculation
 *
 *  @return     Calculated CRC value
 *  @ingroup    crc_table
 */
uint8_t crc8r_AB_table16 (uint8_t c, uint8_t seed);



/** CRC-8 implementation on a single byte with polynome 0xABu using a dual 16 element table lookup approach.
 *
 *  www.ece.cmu.edu/~koopman/roses/dsn04/koopman04_crc_poly_embedded.pdf
 *
 *  @param[in]  c     8-bit value to calculate seed for
 *  @param[in]  seed  8 - bit Seed value for CRC calculation
 *
 *  @return     Calculated CRC value
 *  @ingroup    crc_table
 */
uint8_t crc8r_AB_tabletwo16 (uint8_t c, uint8_t seed);



/** CRC-8 implementation on a single byte with polynome 0xABu using a 256 element table lookup approach.
 *
 *  www.ece.cmu.edu/~koopman/roses/dsn04/koopman04_crc_poly_embedded.pdf
 *
 *  @param[in]  c     8-bit value to calculate seed for
 *  @param[in]  seed  8 - bit Seed value for CRC calculation
 *
 *  @return     Calculated CRC value
 *  @ingroup    crc_table
 */
uint8_t crc8r_AB_table256 (uint8_t c, uint8_t seed);


/** CRC-16 implementation on a single byte with polynome 0x1021u using a purely algorithmic approach
 *
 *  @param[in]  c       Value to calculate seed for
 *  @param[in]  seed    Seed value for CRC calculation
 *
 *  @return     Calculated CRC value
 *  @ingroup    crc_algorithmic
 */
uint16_t crc16 (uint8_t c, uint16_t seed);


/** CRC-16 implementation on a single byte with polynome 0x1021u using a purely algorithmic approach
 *
 *  @param[in]  c       Value to calculate seed for
 *  @param[in]  seed    Seed value for CRC calculation
 *
 *  @return     Calculated CRC value
 *  @ingroup    crc_algorithmic
 */
uint16_t crc16_1021 (uint8_t c, uint16_t seed);


/** CRC-16 implementation on a single byte with polynome 0x8005u using a purely algorithmic approach
 *
 *  @param[in]  c       Value to calculate seed for
 *  @param[in]  seed    Seed value for CRC calculation
 *
 *  @return     Calculated CRC value
 *  @ingroup    crc_algorithmic
 */
uint16_t crc16_8005 (uint8_t c, uint16_t seed);



/** CRC-16 implementation on a single byte with polynome 0x1021u using a 16 element table lookup approach.
 *
 *  www.ece.cmu.edu/~koopman/roses/dsn04/koopman04_crc_poly_embedded.pdf
 *
 *  @param[in]  c     8-bit value to calculate seed for
 *  @param[in]  seed  16 - bit Seed value for CRC calculation
 *
 *  @return     Calculated CRC value
 *  @ingroup    crc_table
 */
uint16_t crc16_table16 (uint8_t c, uint16_t seed);



/** CRC-16 implementation on a single byte with polynome 0x1021u using a dual 16 element table lookup approach.
 *
 *  www.ece.cmu.edu/~koopman/roses/dsn04/koopman04_crc_poly_embedded.pdf
 *
 *  @param[in]  c     8-bit value to calculate seed for
 *  @param[in]  seed  16 - bit Seed value for CRC calculation
 *
 *  @return     Calculated CRC value
 *  @ingroup    crc_table
 */
uint16_t crc16_tabletwo16 (uint8_t c, uint16_t seed);



/** CRC-16 implementation on a single byte with polynome 0x1021u using a 256 element table lookup approach.
 *
 *  www.ece.cmu.edu/~koopman/roses/dsn04/koopman04_crc_poly_embedded.pdf
 *
 *  @param[in]  c     8-bit value to calculate seed for
 *  @param[in]  seed  16 - bit Seed value for CRC calculation
 *
 *  @return     Calculated CRC value
 *  @ingroup    crc_table
 */
uint16_t crc16_table256 (uint8_t c, uint16_t seed);


/** Reflected CRC-16 implementation on a single byte with polynome 0x1021u using a purely algorithmic approach
 *
 *  @param[in]  c       Value to calculate seed for
 *  @param[in]  seed    Seed value for CRC calculation
 *
 *  @return     Calculated CRC value
 *  @ingroup    crc_algorithmic
 */
uint16_t crc16r (uint8_t c, uint16_t seed);



/** CRC-16 implementation on a single byte with polynome 0x1021u using a 16 element table lookup approach.
 *
 *  www.ece.cmu.edu/~koopman/roses/dsn04/koopman04_crc_poly_embedded.pdf
 *
 *  @param[in]  c     8-bit value to calculate seed for
 *  @param[in]  seed  16 - bit Seed value for CRC calculation
 *
 *  @return     Calculated CRC value
 *  @ingroup    crc_table
 */
uint16_t crc16r_table16 (uint8_t c, uint16_t seed);



/** CRC-16 implementation on a single byte with polynome 0x1021u using a dual 16 element table lookup approach.
 *
 *  www.ece.cmu.edu/~koopman/roses/dsn04/koopman04_crc_poly_embedded.pdf
 *
 *  @param[in]  c     8-bit value to calculate seed for
 *  @param[in]  seed  16 - bit Seed value for CRC calculation
 *
 *  @return     Calculated CRC value
 *  @ingroup    crc_table
 */
uint16_t crc16r_tabletwo16 (uint8_t c, uint16_t seed);



/** CRC-16 implementation on a single byte with polynome 0x1021u using a 256 element table lookup approach.
 *
 *  www.ece.cmu.edu/~koopman/roses/dsn04/koopman04_crc_poly_embedded.pdf
 *
 *  @param[in]  c     8-bit value to calculate seed for
 *  @param[in]  seed  16 - bit Seed value for CRC calculation
 *
 *  @return     Calculated CRC value
 *  @ingroup    crc_table
 */
uint16_t crc16r_table256 (uint8_t c, uint16_t seed);


/** Reflected CRC-16 implementation on a single byte with polynome 0x8408u using a purely algorithmic approach
 *
 *  @param[in]  c       Value to calculate seed for
 *  @param[in]  seed    Seed value for CRC calculation
 *
 *  @return     Calculated CRC value
 *  @ingroup    crc_algorithmic
 */
uint16_t crc16r_8408 (uint8_t c, uint16_t seed);



/** CRC-16 implementation on a single byte with polynome 0x8408u using a 256 element table lookup approach.
 *
 *  www.ece.cmu.edu/~koopman/roses/dsn04/koopman04_crc_poly_embedded.pdf
 *
 *  @param[in]  c     8-bit value to calculate seed for
 *  @param[in]  seed  16 - bit Seed value for CRC calculation
 *
 *  @return     Calculated CRC value
 *  @ingroup    crc_table
 */
uint16_t crc16r_8408_table256 (uint8_t c, uint16_t seed);


/** Reflected CRC-16 implementation on a single byte with polynome 0xA001u using a purely algorithmic approach
 *
 *  @param[in]  c       Value to calculate seed for
 *  @param[in]  seed    Seed value for CRC calculation
 *
 *  @return     Calculated CRC value
 *  @ingroup    crc_algorithmic
 */
uint16_t crc16r_A001 (uint8_t c, uint16_t seed);



/** CRC-16 implementation on a single byte with polynome 0xA001u using a 16 element table lookup approach.
 *
 *  www.ece.cmu.edu/~koopman/roses/dsn04/koopman04_crc_poly_embedded.pdf
 *
 *  @param[in]  c     8-bit value to calculate seed for
 *  @param[in]  seed  16 - bit Seed value for CRC calculation
 *
 *  @return     Calculated CRC value
 *  @ingroup    crc_table
 */
uint16_t crc16r_A001_table16 (uint8_t c, uint16_t seed);



/** CRC-16 implementation on a single byte with polynome 0xA001u using a dual 16 element table lookup approach.
 *
 *  www.ece.cmu.edu/~koopman/roses/dsn04/koopman04_crc_poly_embedded.pdf
 *
 *  @param[in]  c     8-bit value to calculate seed for
 *  @param[in]  seed  16 - bit Seed value for CRC calculation
 *
 *  @return     Calculated CRC value
 *  @ingroup    crc_table
 */
uint16_t crc16r_A001_tabletwo16 (uint8_t c, uint16_t seed);



/** CRC-16 implementation on a single byte with polynome 0xA001u using a 256 element table lookup approach.
 *
 *  www.ece.cmu.edu/~koopman/roses/dsn04/koopman04_crc_poly_embedded.pdf
 *
 *  @param[in]  c     8-bit value to calculate seed for
 *  @param[in]  seed  16 - bit Seed value for CRC calculation
 *
 *  @return     Calculated CRC value
 *  @ingroup    crc_table
 */
uint16_t crc16r_A001_table256 (uint8_t c, uint16_t seed);


/** CRC-32 implementation on a single byte with polynome 0x04C11DB7u using a purely algorithmic approach
 *
 *  @param[in]  c       Value to calculate seed for
 *  @param[in]  seed    Seed value for CRC calculation
 *
 *  @return     Calculated CRC value
 *  @ingroup    crc_algorithmic
 */
uint32_t crc32 (uint8_t c, uint32_t seed);


/** Reflected CRC-32 implementation on a single byte with polynome 0x04C11DB7u using a purely algorithmic approach
 *
 *  @param[in]  c       Value to calculate seed for
 *  @param[in]  seed    Seed value for CRC calculation
 *
 *  @return     Calculated CRC value
 *  @ingroup    crc_algorithmic
 */
uint32_t crc32r (uint8_t c, uint32_t seed);


/** Reflected CRC-32 implementation on a single byte with polynome 0xEDB88320u using a purely algorithmic approach
 *
 *  @param[in]  c       Value to calculate seed for
 *  @param[in]  seed    Seed value for CRC calculation
 *
 *  @return     Calculated CRC value
 *  @ingroup    crc_algorithmic
 */
uint32_t crc32r_EDB88320 (uint8_t c, uint32_t seed);



/** CRC-32 implementation on a single byte with polynome 0xEDB88320u using a 256 element table lookup approach.
 *
 *  www.ece.cmu.edu/~koopman/roses/dsn04/koopman04_crc_poly_embedded.pdf
 *
 *  @param[in]  c     8-bit value to calculate seed for
 *  @param[in]  seed  32 - bit Seed value for CRC calculation
 *
 *  @return     Calculated CRC value
 *  @ingroup    crc_table
 */
uint32_t crc32r_EDB88320_table256 (uint8_t c, uint32_t seed);



/** CRC-32 implementation on a single byte with polynome 0x04C11DB7u using a 16 element table lookup approach.
 *
 *  www.ece.cmu.edu/~koopman/roses/dsn04/koopman04_crc_poly_embedded.pdf
 *
 *  @param[in]  c     8-bit value to calculate seed for
 *  @param[in]  seed  32 - bit Seed value for CRC calculation
 *
 *  @return     Calculated CRC value
 *  @ingroup    crc_table
 */
uint32_t crc32_table16 (uint8_t c, uint32_t seed);



/** CRC-32 implementation on a single byte with polynome 0x04C11DB7u using a 16 element table lookup approach.
 *
 *  www.ece.cmu.edu/~koopman/roses/dsn04/koopman04_crc_poly_embedded.pdf
 *
 *  @param[in]  c     8-bit value to calculate seed for
 *  @param[in]  seed  32 - bit Seed value for CRC calculation
 *
 *  @return     Calculated CRC value
 *  @ingroup    crc_table
 */
uint32_t crc32r_table16 (uint8_t c, uint32_t seed);



/** CRC-32 implementation on a single byte with polynome 0x04C11DB7u using a dual 16 element table lookup approach.
 *
 *  www.ece.cmu.edu/~koopman/roses/dsn04/koopman04_crc_poly_embedded.pdf
 *
 *  @param[in]  c     8-bit value to calculate seed for
 *  @param[in]  seed  32 - bit Seed value for CRC calculation
 *
 *  @return     Calculated CRC value
 *  @ingroup    crc_table
 */
uint32_t crc32_tabletwo16 (uint8_t c, uint32_t seed);



/** CRC-32 implementation on a single byte with polynome 0x04C11DB7u using a dual 16 element table lookup approach.
 *
 *  www.ece.cmu.edu/~koopman/roses/dsn04/koopman04_crc_poly_embedded.pdf
 *
 *  @param[in]  c     8-bit value to calculate seed for
 *  @param[in]  seed  32 - bit Seed value for CRC calculation
 *
 *  @return     Calculated CRC value
 *  @ingroup    crc_table
 */
uint32_t crc32r_tabletwo16 (uint8_t c, uint32_t seed);



/** CRC-32 implementation on a single byte with polynome 0x04C11DB7u using a 256 element table lookup approach.
 *
 *  www.ece.cmu.edu/~koopman/roses/dsn04/koopman04_crc_poly_embedded.pdf
 *
 *  @param[in]  c     8-bit value to calculate seed for
 *  @param[in]  seed  32 - bit Seed value for CRC calculation
 *
 *  @return     Calculated CRC value
 *  @ingroup    crc_table
 */
uint32_t crc32_table256 (uint8_t c, uint32_t seed);



/** CRC-32 implementation on a single byte with polynome 0x04C11DB7u using a 256 element table lookup approach.
 *
 *  www.ece.cmu.edu/~koopman/roses/dsn04/koopman04_crc_poly_embedded.pdf
 *
 *  @param[in]  c     8-bit value to calculate seed for
 *  @param[in]  seed  32 - bit Seed value for CRC calculation
 *
 *  @return     Calculated CRC value
 *  @ingroup    crc_table
 */
uint32_t crc32r_table256 (uint8_t c, uint32_t seed);

/** CRC-8 implementation over a dataset with polynome 0xD5u using a 16 element table lookup approach.
 *
 *  This implementation allows for calculating the CRC over a set of data words (laid out continuously in memory)
 *
 *  @param[in]  data    The address to the first element in the uint8_t array to calculate the CRC for
 *  @param[in]  length  The number of words that the CRC should be calculated for
 *  @param[in]  init    The initial seed used for the CRC calculation
 *
 *  @return     The calculated CRC over the full data set provided
 *  @ingroup    crc_table_dataset
 */
uint8_t crc8_table16_data8 (const uint8_t *data, uint16_t length, uint8_t init);

/** CRC-8 implementation over a dataset with polynome 0xD5u using a 16 element table lookup approach.
 *
 *  This implementation allows for calculating the CRC over a set of data words (laid out continuously in memory)
 *
 *  @param[in]  data    The address to the first element in the uint16_t array to calculate the CRC for
 *  @param[in]  length  The number of words that the CRC should be calculated for
 *  @param[in]  init    The initial seed used for the CRC calculation
 *
 *  @return     The calculated CRC over the full data set provided
 *  @ingroup    crc_table_dataset
 */
uint8_t crc8_table16_data16 (const uint16_t *data, uint16_t length, uint8_t init);

/** CRC-16 implementation over a dataset with polynome 0x1021u using a 16 element table lookup approach.
 *
 *  This implementation allows for calculating the CRC over a set of data words (laid out continuously in memory)
 *
 *  @param[in]  data    The address to the first element in the uint8_t array to calculate the CRC for
 *  @param[in]  length  The number of words that the CRC should be calculated for
 *  @param[in]  init    The initial seed used for the CRC calculation
 *
 *  @return     The calculated CRC over the full data set provided
 *  @ingroup    crc_table_dataset
 */
uint16_t crc16_table16_data8 (const uint8_t *data, uint16_t length, uint16_t init);

/** CRC-16 implementation over a dataset with polynome 0x1021u using a 16 element table lookup approach.
 *
 *  This implementation allows for calculating the CRC over a set of data words (laid out continuously in memory)
 *
 *  @param[in]  data    The address to the first element in the uint16_t array to calculate the CRC for
 *  @param[in]  length  The number of words that the CRC should be calculated for
 *  @param[in]  init    The initial seed used for the CRC calculation
 *
 *  @return     The calculated CRC over the full data set provided
 *  @ingroup    crc_table_dataset
 */
uint16_t crc16_table16_data16 (const uint16_t *data, uint16_t length, uint16_t init);

/** CRC-32 implementation over a dataset with polynome 0x04C11DB7u using a 16 element table lookup approach.
 *
 *  This implementation allows for calculating the CRC over a set of data words (laid out continuously in memory)
 *
 *  @param[in]  data    The address to the first element in the uint8_t array to calculate the CRC for
 *  @param[in]  length  The number of words that the CRC should be calculated for
 *  @param[in]  init    The initial seed used for the CRC calculation
 *
 *  @return     The calculated CRC over the full data set provided
 *  @ingroup    crc_table_dataset
 */
uint32_t crc32_table16_data8 (const uint8_t *data, uint16_t length, uint32_t init);

/** CRC-32 implementation over a dataset with polynome 0x04C11DB7u using a 16 element table lookup approach.
 *
 *  This implementation allows for calculating the CRC over a set of data words (laid out continuously in memory)
 *
 *  @param[in]  data    The address to the first element in the uint16_t array to calculate the CRC for
 *  @param[in]  length  The number of words that the CRC should be calculated for
 *  @param[in]  init    The initial seed used for the CRC calculation
 *
 *  @return     The calculated CRC over the full data set provided
 *  @ingroup    crc_table_dataset
 */
uint32_t crc32_table16_data16 (const uint16_t *data, uint16_t length, uint32_t init);

/** CRC-8 implementation over a dataset with polynome 0xD5u using a dual 16 element table lookup approach.
 *
 *  This implementation allows for calculating the CRC over a set of data words (laid out continuously in memory)
 *
 *  @param[in]  data    The address to the first element in the uint8_t array to calculate the CRC for
 *  @param[in]  length  The number of words that the CRC should be calculated for
 *  @param[in]  init    The initial seed used for the CRC calculation
 *
 *  @return     The calculated CRC over the full data set provided
 *  @ingroup    crc_table_dataset
 */
uint8_t crc8_tabletwo16_data8 (const uint8_t *data, uint16_t length, uint8_t init);

/** CRC-8 implementation over a dataset with polynome 0xD5u using a dual 16 element table lookup approach.
 *
 *  This implementation allows for calculating the CRC over a set of data words (laid out continuously in memory)
 *
 *  @param[in]  data    The address to the first element in the uint16_t array to calculate the CRC for
 *  @param[in]  length  The number of words that the CRC should be calculated for
 *  @param[in]  init    The initial seed used for the CRC calculation
 *
 *  @return     The calculated CRC over the full data set provided
 *  @ingroup    crc_table_dataset
 */
uint8_t crc8_tabletwo16_data16 (const uint16_t *data, uint16_t length, uint8_t init);

/** CRC-16 implementation over a dataset with polynome 0x1021u using a dual 16 element table lookup approach.
 *
 *  This implementation allows for calculating the CRC over a set of data words (laid out continuously in memory)
 *
 *  @param[in]  data    The address to the first element in the uint8_t array to calculate the CRC for
 *  @param[in]  length  The number of words that the CRC should be calculated for
 *  @param[in]  init    The initial seed used for the CRC calculation
 *
 *  @return     The calculated CRC over the full data set provided
 *  @ingroup    crc_table_dataset
 */
uint16_t crc16_tabletwo16_data8 (const uint8_t *data, uint16_t length, uint16_t init);

/** CRC-16 implementation over a dataset with polynome 0x1021u using a dual 16 element table lookup approach.
 *
 *  This implementation allows for calculating the CRC over a set of data words (laid out continuously in memory)
 *
 *  @param[in]  data    The address to the first element in the uint16_t array to calculate the CRC for
 *  @param[in]  length  The number of words that the CRC should be calculated for
 *  @param[in]  init    The initial seed used for the CRC calculation
 *
 *  @return     The calculated CRC over the full data set provided
 *  @ingroup    crc_table_dataset
 */
uint16_t crc16_tabletwo16_data16 (const uint16_t *data, uint16_t length, uint16_t init);

/** CRC-32 implementation over a dataset with polynome 0x04C11DB7u using a dual 16 element table lookup approach.
 *
 *  This implementation allows for calculating the CRC over a set of data words (laid out continuously in memory)
 *
 *  @param[in]  data    The address to the first element in the uint8_t array to calculate the CRC for
 *  @param[in]  length  The number of words that the CRC should be calculated for
 *  @param[in]  init    The initial seed used for the CRC calculation
 *
 *  @return     The calculated CRC over the full data set provided
 *  @ingroup    crc_table_dataset
 */
uint32_t crc32_tabletwo16_data8 (const uint8_t *data, uint16_t length, uint32_t init);

/** CRC-32 implementation over a dataset with polynome 0x04C11DB7u using a dual 16 element table lookup approach.
 *
 *  This implementation allows for calculating the CRC over a set of data words (laid out continuously in memory)
 *
 *  @param[in]  data    The address to the first element in the uint16_t array to calculate the CRC for
 *  @param[in]  length  The number of words that the CRC should be calculated for
 *  @param[in]  init    The initial seed used for the CRC calculation
 *
 *  @return     The calculated CRC over the full data set provided
 *  @ingroup    crc_table_dataset
 */
uint32_t crc32_tabletwo16_data16 (const uint16_t *data, uint16_t length, uint32_t init);

/** CRC-8 implementation over a dataset with polynome 0xD5u using a 256 element table lookup approach.
 *
 *  This implementation allows for calculating the CRC over a set of data words (laid out continuously in memory)
 *
 *  @param[in]  data    The address to the first element in the uint8_t array to calculate the CRC for
 *  @param[in]  length  The number of words that the CRC should be calculated for
 *  @param[in]  init    The initial seed used for the CRC calculation
 *
 *  @return     The calculated CRC over the full data set provided
 *  @ingroup    crc_table_dataset
 */
uint8_t crc8_table256_data8 (const uint8_t *data, uint16_t length, uint8_t init);

/** CRC-8 implementation over a dataset with polynome 0xD5u using a 256 element table lookup approach.
 *
 *  This implementation allows for calculating the CRC over a set of data words (laid out continuously in memory)
 *
 *  @param[in]  data    The address to the first element in the uint16_t array to calculate the CRC for
 *  @param[in]  length  The number of words that the CRC should be calculated for
 *  @param[in]  init    The initial seed used for the CRC calculation
 *
 *  @return     The calculated CRC over the full data set provided
 *  @ingroup    crc_table_dataset
 */
uint8_t crc8_table256_data16 (const uint16_t *data, uint16_t length, uint8_t init);

/** CRC-16 implementation over a dataset with polynome 0x1021u using a 256 element table lookup approach.
 *
 *  This implementation allows for calculating the CRC over a set of data words (laid out continuously in memory)
 *
 *  @param[in]  data    The address to the first element in the uint8_t array to calculate the CRC for
 *  @param[in]  length  The number of words that the CRC should be calculated for
 *  @param[in]  init    The initial seed used for the CRC calculation
 *
 *  @return     The calculated CRC over the full data set provided
 *  @ingroup    crc_table_dataset
 */
uint16_t crc16_table256_data8 (const uint8_t *data, uint16_t length, uint16_t init);

/** CRC-16 implementation over a dataset with polynome 0x1021u using a 256 element table lookup approach.
 *
 *  This implementation allows for calculating the CRC over a set of data words (laid out continuously in memory)
 *
 *  @param[in]  data    The address to the first element in the uint16_t array to calculate the CRC for
 *  @param[in]  length  The number of words that the CRC should be calculated for
 *  @param[in]  init    The initial seed used for the CRC calculation
 *
 *  @return     The calculated CRC over the full data set provided
 *  @ingroup    crc_table_dataset
 */
uint16_t crc16_table256_data16 (const uint16_t *data, uint16_t length, uint16_t init);

/** CRC-32 implementation over a dataset with polynome 0x04C11DB7u using a 256 element table lookup approach.
 *
 *  This implementation allows for calculating the CRC over a set of data words (laid out continuously in memory)
 *
 *  @param[in]  data    The address to the first element in the uint8_t array to calculate the CRC for
 *  @param[in]  length  The number of words that the CRC should be calculated for
 *  @param[in]  init    The initial seed used for the CRC calculation
 *
 *  @return     The calculated CRC over the full data set provided
 *  @ingroup    crc_table_dataset
 */
uint32_t crc32_table256_data8 (const uint8_t *data, uint16_t length, uint32_t init);

/** CRC-32 implementation over a dataset with polynome 0x04C11DB7u using a 256 element table lookup approach.
 *
 *  This implementation allows for calculating the CRC over a set of data words (laid out continuously in memory)
 *
 *  @param[in]  data    The address to the first element in the uint16_t array to calculate the CRC for
 *  @param[in]  length  The number of words that the CRC should be calculated for
 *  @param[in]  init    The initial seed used for the CRC calculation
 *
 *  @return     The calculated CRC over the full data set provided
 *  @ingroup    crc_table_dataset
 */
uint32_t crc32_table256_data16 (const uint16_t *data, uint16_t length, uint32_t init);

/** Reflected CRC-8 implementation over a dataset with polynome 0xD5u using a 16 element table lookup approach.
 *
 *  This implementation allows for calculating the CRC over a set of data words (laid out continuously in memory)
 *
 *  @param[in]  data    The address to the first element in the uint8_t array to calculate the CRC for
 *  @param[in]  length  The number of words that the CRC should be calculated for
 *  @param[in]  init    The initial seed used for the CRC calculation
 *
 *  @return     The calculated CRC over the full data set provided
 *  @ingroup    crc_table_dataset
 */
uint8_t crc8r_table16_data8 (const uint8_t *data, uint16_t length, uint8_t init);

/** Reflected CRC-8 implementation over a dataset with polynome 0xABu using a 16 element table lookup approach.
 *
 *  This implementation allows for calculating the CRC over a set of data words (laid out continuously in memory)
 *
 *  @param[in]  data    The address to the first element in the uint8_t array to calculate the CRC for
 *  @param[in]  length  The number of words that the CRC should be calculated for
 *  @param[in]  init    The initial seed used for the CRC calculation
 *
 *  @return     The calculated CRC over the full data set provided
 *  @ingroup    crc_table_dataset
 */
uint8_t crc8r_AB_table16_data8 (const uint8_t *data, uint16_t length, uint8_t init);

/** Reflected CRC-8 implementation over a dataset with polynome 0xD5u using a 16 element table lookup approach.
 *
 *  This implementation allows for calculating the CRC over a set of data words (laid out continuously in memory)
 *
 *  @param[in]  data    The address to the first element in the uint16_t array to calculate the CRC for
 *  @param[in]  length  The number of words that the CRC should be calculated for
 *  @param[in]  init    The initial seed used for the CRC calculation
 *
 *  @return     The calculated CRC over the full data set provided
 *  @ingroup    crc_table_dataset
 */
uint8_t crc8r_table16_data16 (const uint16_t *data, uint16_t length, uint8_t init);

/** Reflected CRC-8 implementation over a dataset with polynome 0xABu using a 16 element table lookup approach.
 *
 *  This implementation allows for calculating the CRC over a set of data words (laid out continuously in memory)
 *
 *  @param[in]  data    The address to the first element in the uint16_t array to calculate the CRC for
 *  @param[in]  length  The number of words that the CRC should be calculated for
 *  @param[in]  init    The initial seed used for the CRC calculation
 *
 *  @return     The calculated CRC over the full data set provided
 *  @ingroup    crc_table_dataset
 */
uint8_t crc8r_AB_table16_data16 (const uint16_t *data, uint16_t length, uint8_t init);

/** Reflected CRC-16 implementation over a dataset with polynome 0x1021u using a 16 element table lookup approach.
 *
 *  This implementation allows for calculating the CRC over a set of data words (laid out continuously in memory)
 *
 *  @param[in]  data    The address to the first element in the uint8_t array to calculate the CRC for
 *  @param[in]  length  The number of words that the CRC should be calculated for
 *  @param[in]  init    The initial seed used for the CRC calculation
 *
 *  @return     The calculated CRC over the full data set provided
 *  @ingroup    crc_table_dataset
 */
uint16_t crc16r_table16_data8 (const uint8_t *data, uint16_t length, uint16_t init);

/** Reflected CRC-16 implementation over a dataset with polynome 0xA001u using a 16 element table lookup approach.
 *
 *  This implementation allows for calculating the CRC over a set of data words (laid out continuously in memory)
 *
 *  @param[in]  data    The address to the first element in the uint8_t array to calculate the CRC for
 *  @param[in]  length  The number of words that the CRC should be calculated for
 *  @param[in]  init    The initial seed used for the CRC calculation
 *
 *  @return     The calculated CRC over the full data set provided
 *  @ingroup    crc_table_dataset
 */
uint16_t crc16r_A001_table16_data8 (const uint8_t *data, uint16_t length, uint16_t init);

/** Reflected CRC-16 implementation over a dataset with polynome 0x1021u using a 16 element table lookup approach.
 *
 *  This implementation allows for calculating the CRC over a set of data words (laid out continuously in memory)
 *
 *  @param[in]  data    The address to the first element in the uint16_t array to calculate the CRC for
 *  @param[in]  length  The number of words that the CRC should be calculated for
 *  @param[in]  init    The initial seed used for the CRC calculation
 *
 *  @return     The calculated CRC over the full data set provided
 *  @ingroup    crc_table_dataset
 */
uint16_t crc16r_table16_data16 (const uint16_t *data, uint16_t length, uint16_t init);

/** Reflected CRC-16 implementation over a dataset with polynome 0xA001u using a 16 element table lookup approach.
 *
 *  This implementation allows for calculating the CRC over a set of data words (laid out continuously in memory)
 *
 *  @param[in]  data    The address to the first element in the uint16_t array to calculate the CRC for
 *  @param[in]  length  The number of words that the CRC should be calculated for
 *  @param[in]  init    The initial seed used for the CRC calculation
 *
 *  @return     The calculated CRC over the full data set provided
 *  @ingroup    crc_table_dataset
 */
uint16_t crc16r_A001_table16_data16 (const uint16_t *data, uint16_t length, uint16_t init);

/** Reflected CRC-32 implementation over a dataset with polynome 0x04C11DB7u using a 16 element table lookup approach.
 *
 *  This implementation allows for calculating the CRC over a set of data words (laid out continuously in memory)
 *
 *  @param[in]  data    The address to the first element in the uint8_t array to calculate the CRC for
 *  @param[in]  length  The number of words that the CRC should be calculated for
 *  @param[in]  init    The initial seed used for the CRC calculation
 *
 *  @return     The calculated CRC over the full data set provided
 *  @ingroup    crc_table_dataset
 */
uint32_t crc32r_table16_data8 (const uint8_t *data, uint16_t length, uint32_t init);

/** Reflected CRC-32 implementation over a dataset with polynome 0x04C11DB7u using a 16 element table lookup approach.
 *
 *  This implementation allows for calculating the CRC over a set of data words (laid out continuously in memory)
 *
 *  @param[in]  data    The address to the first element in the uint16_t array to calculate the CRC for
 *  @param[in]  length  The number of words that the CRC should be calculated for
 *  @param[in]  init    The initial seed used for the CRC calculation
 *
 *  @return     The calculated CRC over the full data set provided
 *  @ingroup    crc_table_dataset
 */
uint32_t crc32r_table16_data16 (const uint16_t *data, uint16_t length, uint32_t init);

/** Reflected CRC-8 implementation over a dataset with polynome 0xD5u using a dual 16 element table lookup approach.
 *
 *  This implementation allows for calculating the CRC over a set of data words (laid out continuously in memory)
 *
 *  @param[in]  data    The address to the first element in the uint8_t array to calculate the CRC for
 *  @param[in]  length  The number of words that the CRC should be calculated for
 *  @param[in]  init    The initial seed used for the CRC calculation
 *
 *  @return     The calculated CRC over the full data set provided
 *  @ingroup    crc_table_dataset
 */
uint8_t crc8r_tabletwo16_data8 (const uint8_t *data, uint16_t length, uint8_t init);

/** Reflected CRC-8 implementation over a dataset with polynome 0xABu using a dual 16 element table lookup approach.
 *
 *  This implementation allows for calculating the CRC over a set of data words (laid out continuously in memory)
 *
 *  @param[in]  data    The address to the first element in the uint8_t array to calculate the CRC for
 *  @param[in]  length  The number of words that the CRC should be calculated for
 *  @param[in]  init    The initial seed used for the CRC calculation
 *
 *  @return     The calculated CRC over the full data set provided
 *  @ingroup    crc_table_dataset
 */
uint8_t crc8r_AB_tabletwo16_data8 (const uint8_t *data, uint16_t length, uint8_t init);

/** Reflected CRC-8 implementation over a dataset with polynome 0xD5u using a dual 16 element table lookup approach.
 *
 *  This implementation allows for calculating the CRC over a set of data words (laid out continuously in memory)
 *
 *  @param[in]  data    The address to the first element in the uint16_t array to calculate the CRC for
 *  @param[in]  length  The number of words that the CRC should be calculated for
 *  @param[in]  init    The initial seed used for the CRC calculation
 *
 *  @return     The calculated CRC over the full data set provided
 *  @ingroup    crc_table_dataset
 */
uint8_t crc8r_tabletwo16_data16 (const uint16_t *data, uint16_t length, uint8_t init);

/** Reflected CRC-8 implementation over a dataset with polynome 0xABu using a dual 16 element table lookup approach.
 *
 *  This implementation allows for calculating the CRC over a set of data words (laid out continuously in memory)
 *
 *  @param[in]  data    The address to the first element in the uint16_t array to calculate the CRC for
 *  @param[in]  length  The number of words that the CRC should be calculated for
 *  @param[in]  init    The initial seed used for the CRC calculation
 *
 *  @return     The calculated CRC over the full data set provided
 *  @ingroup    crc_table_dataset
 */
uint8_t crc8r_AB_tabletwo16_data16 (const uint16_t *data, uint16_t length, uint8_t init);

/** Reflected CRC-16 implementation over a dataset with polynome 0x1021u using a dual 16 element table lookup approach.
 *
 *  This implementation allows for calculating the CRC over a set of data words (laid out continuously in memory)
 *
 *  @param[in]  data    The address to the first element in the uint8_t array to calculate the CRC for
 *  @param[in]  length  The number of words that the CRC should be calculated for
 *  @param[in]  init    The initial seed used for the CRC calculation
 *
 *  @return     The calculated CRC over the full data set provided
 *  @ingroup    crc_table_dataset
 */
uint16_t crc16r_tabletwo16_data8 (const uint8_t *data, uint16_t length, uint16_t init);

/** Reflected CRC-16 implementation over a dataset with polynome 0xA001u using a dual 16 element table lookup approach.
 *
 *  This implementation allows for calculating the CRC over a set of data words (laid out continuously in memory)
 *
 *  @param[in]  data    The address to the first element in the uint8_t array to calculate the CRC for
 *  @param[in]  length  The number of words that the CRC should be calculated for
 *  @param[in]  init    The initial seed used for the CRC calculation
 *
 *  @return     The calculated CRC over the full data set provided
 *  @ingroup    crc_table_dataset
 */
uint16_t crc16r_A001_tabletwo16_data8 (const uint8_t *data, uint16_t length, uint16_t init);

/** Reflected CRC-16 implementation over a dataset with polynome 0x1021u using a dual 16 element table lookup approach.
 *
 *  This implementation allows for calculating the CRC over a set of data words (laid out continuously in memory)
 *
 *  @param[in]  data    The address to the first element in the uint16_t array to calculate the CRC for
 *  @param[in]  length  The number of words that the CRC should be calculated for
 *  @param[in]  init    The initial seed used for the CRC calculation
 *
 *  @return     The calculated CRC over the full data set provided
 *  @ingroup    crc_table_dataset
 */
uint16_t crc16r_tabletwo16_data16 (const uint16_t *data, uint16_t length, uint16_t init);

/** Reflected CRC-16 implementation over a dataset with polynome 0xA001u using a dual 16 element table lookup approach.
 *
 *  This implementation allows for calculating the CRC over a set of data words (laid out continuously in memory)
 *
 *  @param[in]  data    The address to the first element in the uint16_t array to calculate the CRC for
 *  @param[in]  length  The number of words that the CRC should be calculated for
 *  @param[in]  init    The initial seed used for the CRC calculation
 *
 *  @return     The calculated CRC over the full data set provided
 *  @ingroup    crc_table_dataset
 */
uint16_t crc16r_A001_tabletwo16_data16 (const uint16_t *data, uint16_t length, uint16_t init);

/** Reflected CRC-32 implementation over a dataset with polynome 0x04C11DB7u using a dual 16 element table lookup approach.
 *
 *  This implementation allows for calculating the CRC over a set of data words (laid out continuously in memory)
 *
 *  @param[in]  data    The address to the first element in the uint8_t array to calculate the CRC for
 *  @param[in]  length  The number of words that the CRC should be calculated for
 *  @param[in]  init    The initial seed used for the CRC calculation
 *
 *  @return     The calculated CRC over the full data set provided
 *  @ingroup    crc_table_dataset
 */
uint32_t crc32r_tabletwo16_data8 (const uint8_t *data, uint16_t length, uint32_t init);

/** Reflected CRC-32 implementation over a dataset with polynome 0x04C11DB7u using a dual 16 element table lookup approach.
 *
 *  This implementation allows for calculating the CRC over a set of data words (laid out continuously in memory)
 *
 *  @param[in]  data    The address to the first element in the uint16_t array to calculate the CRC for
 *  @param[in]  length  The number of words that the CRC should be calculated for
 *  @param[in]  init    The initial seed used for the CRC calculation
 *
 *  @return     The calculated CRC over the full data set provided
 *  @ingroup    crc_table_dataset
 */
uint32_t crc32r_tabletwo16_data16 (const uint16_t *data, uint16_t length, uint32_t init);

/** Reflected CRC-8 implementation over a dataset with polynome 0xD5u using a 256 element table lookup approach.
 *
 *  This implementation allows for calculating the CRC over a set of data words (laid out continuously in memory)
 *
 *  @param[in]  data    The address to the first element in the uint8_t array to calculate the CRC for
 *  @param[in]  length  The number of words that the CRC should be calculated for
 *  @param[in]  init    The initial seed used for the CRC calculation
 *
 *  @return     The calculated CRC over the full data set provided
 *  @ingroup    crc_table_dataset
 */
uint8_t crc8r_table256_data8 (const uint8_t *data, uint16_t length, uint8_t init);

/** Reflected CRC-8 implementation over a dataset with polynome 0xABu using a 256 element table lookup approach.
 *
 *  This implementation allows for calculating the CRC over a set of data words (laid out continuously in memory)
 *
 *  @param[in]  data    The address to the first element in the uint8_t array to calculate the CRC for
 *  @param[in]  length  The number of words that the CRC should be calculated for
 *  @param[in]  init    The initial seed used for the CRC calculation
 *
 *  @return     The calculated CRC over the full data set provided
 *  @ingroup    crc_table_dataset
 */
uint8_t crc8r_AB_table256_data8 (const uint8_t *data, uint16_t length, uint8_t init);

/** Reflected CRC-8 implementation over a dataset with polynome 0xD5u using a 256 element table lookup approach.
 *
 *  This implementation allows for calculating the CRC over a set of data words (laid out continuously in memory)
 *
 *  @param[in]  data    The address to the first element in the uint16_t array to calculate the CRC for
 *  @param[in]  length  The number of words that the CRC should be calculated for
 *  @param[in]  init    The initial seed used for the CRC calculation
 *
 *  @return     The calculated CRC over the full data set provided
 *  @ingroup    crc_table_dataset
 */
uint8_t crc8r_table256_data16 (const uint16_t *data, uint16_t length, uint8_t init);

/** Reflected CRC-8 implementation over a dataset with polynome 0xABu using a 256 element table lookup approach.
 *
 *  This implementation allows for calculating the CRC over a set of data words (laid out continuously in memory)
 *
 *  @param[in]  data    The address to the first element in the uint16_t array to calculate the CRC for
 *  @param[in]  length  The number of words that the CRC should be calculated for
 *  @param[in]  init    The initial seed used for the CRC calculation
 *
 *  @return     The calculated CRC over the full data set provided
 *  @ingroup    crc_table_dataset
 */
uint8_t crc8r_AB_table256_data16 (const uint16_t *data, uint16_t length, uint8_t init);

/** Reflected CRC-16 implementation over a dataset with polynome 0x1021u using a 256 element table lookup approach.
 *
 *  This implementation allows for calculating the CRC over a set of data words (laid out continuously in memory)
 *
 *  @param[in]  data    The address to the first element in the uint8_t array to calculate the CRC for
 *  @param[in]  length  The number of words that the CRC should be calculated for
 *  @param[in]  init    The initial seed used for the CRC calculation
 *
 *  @return     The calculated CRC over the full data set provided
 *  @ingroup    crc_table_dataset
 */
uint16_t crc16r_table256_data8 (const uint8_t *data, uint16_t length, uint16_t init);

/** Reflected CRC-16 implementation over a dataset with polynome 0x8408u using a 256 element table lookup approach.
 *
 *  This implementation allows for calculating the CRC over a set of data words (laid out continuously in memory)
 *
 *  @param[in]  data    The address to the first element in the uint8_t array to calculate the CRC for
 *  @param[in]  length  The number of words that the CRC should be calculated for
 *  @param[in]  init    The initial seed used for the CRC calculation
 *
 *  @return     The calculated CRC over the full data set provided
 *  @ingroup    crc_table_dataset
 */
uint16_t crc16r_8408_table256_data8 (const uint8_t *data, uint16_t length, uint16_t init);

/** Reflected CRC-16 implementation over a dataset with polynome 0xA001u using a 256 element table lookup approach.
 *
 *  This implementation allows for calculating the CRC over a set of data words (laid out continuously in memory)
 *
 *  @param[in]  data    The address to the first element in the uint8_t array to calculate the CRC for
 *  @param[in]  length  The number of words that the CRC should be calculated for
 *  @param[in]  init    The initial seed used for the CRC calculation
 *
 *  @return     The calculated CRC over the full data set provided
 *  @ingroup    crc_table_dataset
 */
uint16_t crc16r_A001_table256_data8 (const uint8_t *data, uint16_t length, uint16_t init);

/** Reflected CRC-16 implementation over a dataset with polynome 0x1021u using a 256 element table lookup approach.
 *
 *  This implementation allows for calculating the CRC over a set of data words (laid out continuously in memory)
 *
 *  @param[in]  data    The address to the first element in the uint16_t array to calculate the CRC for
 *  @param[in]  length  The number of words that the CRC should be calculated for
 *  @param[in]  init    The initial seed used for the CRC calculation
 *
 *  @return     The calculated CRC over the full data set provided
 *  @ingroup    crc_table_dataset
 */
uint16_t crc16r_table256_data16 (const uint16_t *data, uint16_t length, uint16_t init);

/** Reflected CRC-16 implementation over a dataset with polynome 0x8408u using a 256 element table lookup approach.
 *
 *  This implementation allows for calculating the CRC over a set of data words (laid out continuously in memory)
 *
 *  @param[in]  data    The address to the first element in the uint16_t array to calculate the CRC for
 *  @param[in]  length  The number of words that the CRC should be calculated for
 *  @param[in]  init    The initial seed used for the CRC calculation
 *
 *  @return     The calculated CRC over the full data set provided
 *  @ingroup    crc_table_dataset
 */
uint16_t crc16r_8408_table256_data16 (const uint16_t *data, uint16_t length, uint16_t init);

/** Reflected CRC-16 implementation over a dataset with polynome 0xA001u using a 256 element table lookup approach.
 *
 *  This implementation allows for calculating the CRC over a set of data words (laid out continuously in memory)
 *
 *  @param[in]  data    The address to the first element in the uint16_t array to calculate the CRC for
 *  @param[in]  length  The number of words that the CRC should be calculated for
 *  @param[in]  init    The initial seed used for the CRC calculation
 *
 *  @return     The calculated CRC over the full data set provided
 *  @ingroup    crc_table_dataset
 */
uint16_t crc16r_A001_table256_data16 (const uint16_t *data, uint16_t length, uint16_t init);

/** Reflected CRC-32 implementation over a dataset with polynome 0xEDB88320u using a 256 element table lookup approach.
 *
 *  This implementation allows for calculating the CRC over a set of data words (laid out continuously in memory)
 *
 *  @param[in]  data    The address to the first element in the uint8_t array to calculate the CRC for
 *  @param[in]  length  The number of words that the CRC should be calculated for
 *  @param[in]  init    The initial seed used for the CRC calculation
 *
 *  @return     The calculated CRC over the full data set provided
 *  @ingroup    crc_table_dataset
 */
uint32_t crc32r_EDB88320_table256_data8 (const uint8_t *data, uint16_t length, uint32_t init);

/** Reflected CRC-32 implementation over a dataset with polynome 0x04C11DB7u using a 256 element table lookup approach.
 *
 *  This implementation allows for calculating the CRC over a set of data words (laid out continuously in memory)
 *
 *  @param[in]  data    The address to the first element in the uint8_t array to calculate the CRC for
 *  @param[in]  length  The number of words that the CRC should be calculated for
 *  @param[in]  init    The initial seed used for the CRC calculation
 *
 *  @return     The calculated CRC over the full data set provided
 *  @ingroup    crc_table_dataset
 */
uint32_t crc32r_table256_data8 (const uint8_t *data, uint16_t length, uint32_t init);

/** Reflected CRC-32 implementation over a dataset with polynome 0xEDB88320u using a 256 element table lookup approach.
 *
 *  This implementation allows for calculating the CRC over a set of data words (laid out continuously in memory)
 *
 *  @param[in]  data    The address to the first element in the uint16_t array to calculate the CRC for
 *  @param[in]  length  The number of words that the CRC should be calculated for
 *  @param[in]  init    The initial seed used for the CRC calculation
 *
 *  @return     The calculated CRC over the full data set provided
 *  @ingroup    crc_table_dataset
 */
uint32_t crc32r_EDB88320_table256_data16 (const uint16_t *data, uint16_t length, uint32_t init);

/** Reflected CRC-32 implementation over a dataset with polynome 0x04C11DB7u using a 256 element table lookup approach.
 *
 *  This implementation allows for calculating the CRC over a set of data words (laid out continuously in memory)
 *
 *  @param[in]  data    The address to the first element in the uint16_t array to calculate the CRC for
 *  @param[in]  length  The number of words that the CRC should be calculated for
 *  @param[in]  init    The initial seed used for the CRC calculation
 *
 *  @return     The calculated CRC over the full data set provided
 *  @ingroup    crc_table_dataset
 */
uint32_t crc32r_table256_data16 (const uint16_t *data, uint16_t length, uint32_t init);
#endif /* GEN_CRC_H_ */
/** @}*/
