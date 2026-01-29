/** @file
 *  @brief Header file for functions related to finding set bits in a word/byte
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

#ifndef FIND_BITS_H
#define FIND_BITS_H

#include <stdint.h>

/** \addtogroup math_bit_operation_group
 *  @{
 */

/* Each return the position, zero-indexed from the right, or -1 for not found. */

/** Find the first 1-bit from left in a 16-bit value
 *
 *  @param[in]   search_word     16-bit value to inspect
 *  @retval      [0:15]          Index of the first 1-bit from left in given search_word
 *  @retval      -1              Return value in case no bit is set
 */
int8_t find_first_one_left  (uint16_t search_word);

/** Find the first 1-bit from right in a 16-bit value
 *
 *  @param[in]   search_word_in  16-bit value to inspect
 *  @retval      [0:15]          Index of the first 1-bit from right in given search_word_in
 *  @retval      -1              Return value in case no bit is set
 */
int8_t find_first_one_right (uint16_t search_word_in);

/** Find the first 1-bit from right in a byte
 *
 *  This algorithm uses DeBruijn sequences in order to get the first 1-bit:
 *  supertech.csail.mit.edu/papers/debruijn.pdf
 *
 *  @param[in]   x               Byte to inspect
 *  @retval      [0:7]           Index of the first 1-bit from right in given x
 *  @retval      -1              Return value in case no bit is set
 */
int8_t find_first_one_right_U8_fast(uint8_t x);

/** Find the first 0-bit from left in a 16-bit value
 *
 *  @param[in]   search_word     16-bit value to inspect
 *  @retval      [0:15]          Index of the first 0-bit from left in given search_word
 *  @retval      -1              Return value in case all bits are set
 */
int8_t find_first_zero_left (uint16_t search_word);

/** Find the first 0-bit from right in a 16-bit value
 *
 *  @param[in]   search_word     16-bit value to inspect
 *  @retval      [0:15]          Index of the first 0-bit from right in given search_word_in
 *  @retval      -1              Return value in case all bits are set
 */
int8_t find_first_zero_right    (uint16_t search_word);

#endif
/** @}*/
