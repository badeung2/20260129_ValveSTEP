/** @file
 *  @brief Header file for various filter functions
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

/** \addtogroup filter
 *  @{
 */

#ifndef GEN_FILTER_H_
#define GEN_FILTER_H_

#include <stdint.h>



/** @brief Calculates (first order) unsigned \b half-range low pass filter with coefficient \f$ 2^{1} \f$
 *
 *  Calculates result of a first order low pass filter on input *in* and filter
 *  delay line values *p_out*, and update the filter delay line *p_out*,
 *  according to the formula:
 *
 *  \f[
 *      out[k] = \frac{(2^{1} - 1) * out[k-1] + in}{2^{1}}
 *  \f]
 *
 *  This is a special version of a first order IIR filter with coefficient
 *  \f$ a = -2^{-n} \f$ with n = 1
 *
 *  @param[in]      in      Input value for filter
 *  @param[in, out] p_out   Filter delay line
 *
 *  @return         The result of the filter calculation
 *
 *  @note       Before usage, the filter delay line *p_out* should be initialized
 *              with the correct lpf_init function.
 *  @warning    The output is calculated exactly, without risk of overflow,
 *              only on half of the input range, i.e. (0 <= in < 32768).
 *              The implementation is generally faster and smaller than its full-range counterpart.
 *
 *  @internal
 *      For the implementation, the formula stated above is simplified to
 *      out[k] = out[k-1] - (out[k-1] - in)/2^1
 *  @endinternal
 *  @ingroup    filter_lpf_group
 */
uint16_t lpf1 (uint16_t in, uint16_t * const p_out);



/** @brief Calculates (first order) unsigned \b half-range low pass filter with coefficient \f$ 2^{2} \f$
 *
 *  Calculates result of a first order low pass filter on input *in* and filter
 *  delay line values *p_out*, and update the filter delay line *p_out*,
 *  according to the formula:
 *
 *  \f[
 *      out[k] = \frac{(2^{2} - 1) * out[k-1] + in}{2^{2}}
 *  \f]
 *
 *  This is a special version of a first order IIR filter with coefficient
 *  \f$ a = -2^{-n} \f$ with n = 2
 *
 *  @param[in]      in      Input value for filter
 *  @param[in, out] p_out   Filter delay line
 *
 *  @return         The result of the filter calculation
 *
 *  @note       Before usage, the filter delay line *p_out* should be initialized
 *              with the correct lpf_init function.
 *  @warning    The output is calculated exactly, without risk of overflow,
 *              only on half of the input range, i.e. (0 <= in < 32768).
 *              The implementation is generally faster and smaller than its full-range counterpart.
 *
 *  @internal
 *      For the implementation, the formula stated above is simplified to
 *      out[k] = out[k-1] - (out[k-1] - in)/2^2
 *  @endinternal
 *  @ingroup    filter_lpf_group
 */
uint16_t lpf2 (uint16_t in, uint16_t * const p_out);



/** @brief Calculates (first order) unsigned \b half-range low pass filter with coefficient \f$ 2^{3} \f$
 *
 *  Calculates result of a first order low pass filter on input *in* and filter
 *  delay line values *p_out*, and update the filter delay line *p_out*,
 *  according to the formula:
 *
 *  \f[
 *      out[k] = \frac{(2^{3} - 1) * out[k-1] + in}{2^{3}}
 *  \f]
 *
 *  This is a special version of a first order IIR filter with coefficient
 *  \f$ a = -2^{-n} \f$ with n = 3
 *
 *  @param[in]      in      Input value for filter
 *  @param[in, out] p_out   Filter delay line
 *
 *  @return         The result of the filter calculation
 *
 *  @note       Before usage, the filter delay line *p_out* should be initialized
 *              with the correct lpf_init function.
 *  @warning    The output is calculated exactly, without risk of overflow,
 *              only on half of the input range, i.e. (0 <= in < 32768).
 *              The implementation is generally faster and smaller than its full-range counterpart.
 *
 *  @internal
 *      For the implementation, the formula stated above is simplified to
 *      out[k] = out[k-1] - (out[k-1] - in)/2^3
 *  @endinternal
 *  @ingroup    filter_lpf_group
 */
uint16_t lpf3 (uint16_t in, uint16_t * const p_out);



/** @brief Calculates (first order) unsigned \b half-range low pass filter with coefficient \f$ 2^{4} \f$
 *
 *  Calculates result of a first order low pass filter on input *in* and filter
 *  delay line values *p_out*, and update the filter delay line *p_out*,
 *  according to the formula:
 *
 *  \f[
 *      out[k] = \frac{(2^{4} - 1) * out[k-1] + in}{2^{4}}
 *  \f]
 *
 *  This is a special version of a first order IIR filter with coefficient
 *  \f$ a = -2^{-n} \f$ with n = 4
 *
 *  @param[in]      in      Input value for filter
 *  @param[in, out] p_out   Filter delay line
 *
 *  @return         The result of the filter calculation
 *
 *  @note       Before usage, the filter delay line *p_out* should be initialized
 *              with the correct lpf_init function.
 *  @warning    The output is calculated exactly, without risk of overflow,
 *              only on half of the input range, i.e. (0 <= in < 32768).
 *              The implementation is generally faster and smaller than its full-range counterpart.
 *
 *  @internal
 *      For the implementation, the formula stated above is simplified to
 *      out[k] = out[k-1] - (out[k-1] - in)/2^4
 *  @endinternal
 *  @ingroup    filter_lpf_group
 */
uint16_t lpf4 (uint16_t in, uint16_t * const p_out);



/** @brief Calculates (first order) unsigned \b half-range low pass filter with coefficient \f$ 2^{5} \f$
 *
 *  Calculates result of a first order low pass filter on input *in* and filter
 *  delay line values *p_out*, and update the filter delay line *p_out*,
 *  according to the formula:
 *
 *  \f[
 *      out[k] = \frac{(2^{5} - 1) * out[k-1] + in}{2^{5}}
 *  \f]
 *
 *  This is a special version of a first order IIR filter with coefficient
 *  \f$ a = -2^{-n} \f$ with n = 5
 *
 *  @param[in]      in      Input value for filter
 *  @param[in, out] p_out   Filter delay line
 *
 *  @return         The result of the filter calculation
 *
 *  @note       Before usage, the filter delay line *p_out* should be initialized
 *              with the correct lpf_init function.
 *  @warning    The output is calculated exactly, without risk of overflow,
 *              only on half of the input range, i.e. (0 <= in < 32768).
 *              The implementation is generally faster and smaller than its full-range counterpart.
 *
 *  @internal
 *      For the implementation, the formula stated above is simplified to
 *      out[k] = out[k-1] - (out[k-1] - in)/2^5
 *  @endinternal
 *  @ingroup    filter_lpf_group
 */
uint16_t lpf5 (uint16_t in, uint16_t * const p_out);



/** @brief Calculates (first order) unsigned \b half-range low pass filter with coefficient \f$ 2^{6} \f$
 *
 *  Calculates result of a first order low pass filter on input *in* and filter
 *  delay line values *p_out*, and update the filter delay line *p_out*,
 *  according to the formula:
 *
 *  \f[
 *      out[k] = \frac{(2^{6} - 1) * out[k-1] + in}{2^{6}}
 *  \f]
 *
 *  This is a special version of a first order IIR filter with coefficient
 *  \f$ a = -2^{-n} \f$ with n = 6
 *
 *  @param[in]      in      Input value for filter
 *  @param[in, out] p_out   Filter delay line
 *
 *  @return         The result of the filter calculation
 *
 *  @note       Before usage, the filter delay line *p_out* should be initialized
 *              with the correct lpf_init function.
 *  @warning    The output is calculated exactly, without risk of overflow,
 *              only on half of the input range, i.e. (0 <= in < 32768).
 *              The implementation is generally faster and smaller than its full-range counterpart.
 *
 *  @internal
 *      For the implementation, the formula stated above is simplified to
 *      out[k] = out[k-1] - (out[k-1] - in)/2^6
 *  @endinternal
 *  @ingroup    filter_lpf_group
 */
uint16_t lpf6 (uint16_t in, uint16_t * const p_out);



/** @brief Calculates (first order) unsigned \b half-range low pass filter with coefficient \f$ 2^{7} \f$
 *
 *  Calculates result of a first order low pass filter on input *in* and filter
 *  delay line values *p_out*, and update the filter delay line *p_out*,
 *  according to the formula:
 *
 *  \f[
 *      out[k] = \frac{(2^{7} - 1) * out[k-1] + in}{2^{7}}
 *  \f]
 *
 *  This is a special version of a first order IIR filter with coefficient
 *  \f$ a = -2^{-n} \f$ with n = 7
 *
 *  @param[in]      in      Input value for filter
 *  @param[in, out] p_out   Filter delay line
 *
 *  @return         The result of the filter calculation
 *
 *  @note       Before usage, the filter delay line *p_out* should be initialized
 *              with the correct lpf_init function.
 *  @warning    The output is calculated exactly, without risk of overflow,
 *              only on half of the input range, i.e. (0 <= in < 32768).
 *              The implementation is generally faster and smaller than its full-range counterpart.
 *
 *  @internal
 *      For the implementation, the formula stated above is simplified to
 *      out[k] = out[k-1] - (out[k-1] - in)/2^7
 *  @endinternal
 *  @ingroup    filter_lpf_group
 */
uint16_t lpf7 (uint16_t in, uint16_t * const p_out);



/** @brief Calculates (first order) unsigned \b half-range low pass filter with coefficient \f$ 2^{8} \f$
 *
 *  Calculates result of a first order low pass filter on input *in* and filter
 *  delay line values *p_out*, and update the filter delay line *p_out*,
 *  according to the formula:
 *
 *  \f[
 *      out[k] = \frac{(2^{8} - 1) * out[k-1] + in}{2^{8}}
 *  \f]
 *
 *  This is a special version of a first order IIR filter with coefficient
 *  \f$ a = -2^{-n} \f$ with n = 8
 *
 *  @param[in]      in      Input value for filter
 *  @param[in, out] p_out   Filter delay line
 *
 *  @return         The result of the filter calculation
 *
 *  @note       Before usage, the filter delay line *p_out* should be initialized
 *              with the correct lpf_init function.
 *  @warning    The output is calculated exactly, without risk of overflow,
 *              only on half of the input range, i.e. (0 <= in < 32768).
 *              The implementation is generally faster and smaller than its full-range counterpart.
 *
 *  @internal
 *      For the implementation, the formula stated above is simplified to
 *      out[k] = out[k-1] - (out[k-1] - in)/2^8
 *  @endinternal
 *  @ingroup    filter_lpf_group
 */
uint16_t lpf8 (uint16_t in, uint16_t * const p_out);

/** @brief Calculates (first order) unsigned low pass filter with 16-bit filter delay line with coefficient \f$ 2^{1} \f$
 *
 *  Calculates result of a first order low pass filter on input *in* and filter
 *  delay line values *p_out*, and update the filter delay line *p_out*,
 *  according to the formula:
 *
 *  \f[
 *		out[k] = \frac{(2^{1} - 1) * out[k-1] + in}{2^{1}}
 *  \f]
 *
 *  This is a special version of a first order IIR filter with coefficient
 *  \f$ a = -2^{-n} \f$ with n = 1
 *
 *  The output is calculated exactly, without loss of precision, and the entire
 *  input range is supported
 *
 *  @param[in]		in      Input value for filter
 *  @param[in, out]	p_out   Filter delay line
 *
 *  @return         The result of the filter calculation
 *
 *  @note 	Before usage, the filter delay line *p_out* should be initialized
 *			with the correct lpf_init function.
 *
 *  @internal
 *		For the implementation, the formula stated above is simplified to
 *		out[k] = out[k-1] - (out[k-1] - in)/2^1
 *  @endinternal
 *  @ingroup    filter_lpf_group
 */
 uint16_t lpf1_U16 (uint16_t in, uint16_t *p_out);

/** @brief Calculates (first order) unsigned low pass filter with 16-bit filter delay line with coefficient \f$ 2^{2} \f$
 *
 *  Calculates result of a first order low pass filter on input *in* and filter
 *  delay line values *p_out*, and update the filter delay line *p_out*,
 *  according to the formula:
 *
 *  \f[
 *		out[k] = \frac{(2^{2} - 1) * out[k-1] + in}{2^{2}}
 *  \f]
 *
 *  This is a special version of a first order IIR filter with coefficient
 *  \f$ a = -2^{-n} \f$ with n = 2
 *
 *  The output is calculated exactly, without loss of precision, and the entire
 *  input range is supported
 *
 *  @param[in]		in      Input value for filter
 *  @param[in, out]	p_out   Filter delay line
 *
 *  @return         The result of the filter calculation
 *
 *  @note 	Before usage, the filter delay line *p_out* should be initialized
 *			with the correct lpf_init function.
 *
 *  @internal
 *		For the implementation, the formula stated above is simplified to
 *		out[k] = out[k-1] - (out[k-1] - in)/2^2
 *  @endinternal
 *  @ingroup    filter_lpf_group
 */
 uint16_t lpf2_U16 (uint16_t in, uint16_t *p_out);

/** @brief Calculates (first order) unsigned low pass filter with 16-bit filter delay line with coefficient \f$ 2^{3} \f$
 *
 *  Calculates result of a first order low pass filter on input *in* and filter
 *  delay line values *p_out*, and update the filter delay line *p_out*,
 *  according to the formula:
 *
 *  \f[
 *		out[k] = \frac{(2^{3} - 1) * out[k-1] + in}{2^{3}}
 *  \f]
 *
 *  This is a special version of a first order IIR filter with coefficient
 *  \f$ a = -2^{-n} \f$ with n = 3
 *
 *  The output is calculated exactly, without loss of precision, and the entire
 *  input range is supported
 *
 *  @param[in]		in      Input value for filter
 *  @param[in, out]	p_out   Filter delay line
 *
 *  @return         The result of the filter calculation
 *
 *  @note 	Before usage, the filter delay line *p_out* should be initialized
 *			with the correct lpf_init function.
 *
 *  @internal
 *		For the implementation, the formula stated above is simplified to
 *		out[k] = out[k-1] - (out[k-1] - in)/2^3
 *  @endinternal
 *  @ingroup    filter_lpf_group
 */
 uint16_t lpf3_U16 (uint16_t in, uint16_t *p_out);

/** @brief Calculates (first order) unsigned low pass filter with 16-bit filter delay line with coefficient \f$ 2^{4} \f$
 *
 *  Calculates result of a first order low pass filter on input *in* and filter
 *  delay line values *p_out*, and update the filter delay line *p_out*,
 *  according to the formula:
 *
 *  \f[
 *		out[k] = \frac{(2^{4} - 1) * out[k-1] + in}{2^{4}}
 *  \f]
 *
 *  This is a special version of a first order IIR filter with coefficient
 *  \f$ a = -2^{-n} \f$ with n = 4
 *
 *  The output is calculated exactly, without loss of precision, and the entire
 *  input range is supported
 *
 *  @param[in]		in      Input value for filter
 *  @param[in, out]	p_out   Filter delay line
 *
 *  @return         The result of the filter calculation
 *
 *  @note 	Before usage, the filter delay line *p_out* should be initialized
 *			with the correct lpf_init function.
 *
 *  @internal
 *		For the implementation, the formula stated above is simplified to
 *		out[k] = out[k-1] - (out[k-1] - in)/2^4
 *  @endinternal
 *  @ingroup    filter_lpf_group
 */
 uint16_t lpf4_U16 (uint16_t in, uint16_t *p_out);

/** @brief Calculates (first order) unsigned low pass filter with 16-bit filter delay line with coefficient \f$ 2^{5} \f$
 *
 *  Calculates result of a first order low pass filter on input *in* and filter
 *  delay line values *p_out*, and update the filter delay line *p_out*,
 *  according to the formula:
 *
 *  \f[
 *		out[k] = \frac{(2^{5} - 1) * out[k-1] + in}{2^{5}}
 *  \f]
 *
 *  This is a special version of a first order IIR filter with coefficient
 *  \f$ a = -2^{-n} \f$ with n = 5
 *
 *  The output is calculated exactly, without loss of precision, and the entire
 *  input range is supported
 *
 *  @param[in]		in      Input value for filter
 *  @param[in, out]	p_out   Filter delay line
 *
 *  @return         The result of the filter calculation
 *
 *  @note 	Before usage, the filter delay line *p_out* should be initialized
 *			with the correct lpf_init function.
 *
 *  @internal
 *		For the implementation, the formula stated above is simplified to
 *		out[k] = out[k-1] - (out[k-1] - in)/2^5
 *  @endinternal
 *  @ingroup    filter_lpf_group
 */
 uint16_t lpf5_U16 (uint16_t in, uint16_t *p_out);

/** @brief Calculates (first order) unsigned low pass filter with 16-bit filter delay line with coefficient \f$ 2^{6} \f$
 *
 *  Calculates result of a first order low pass filter on input *in* and filter
 *  delay line values *p_out*, and update the filter delay line *p_out*,
 *  according to the formula:
 *
 *  \f[
 *		out[k] = \frac{(2^{6} - 1) * out[k-1] + in}{2^{6}}
 *  \f]
 *
 *  This is a special version of a first order IIR filter with coefficient
 *  \f$ a = -2^{-n} \f$ with n = 6
 *
 *  The output is calculated exactly, without loss of precision, and the entire
 *  input range is supported
 *
 *  @param[in]		in      Input value for filter
 *  @param[in, out]	p_out   Filter delay line
 *
 *  @return         The result of the filter calculation
 *
 *  @note 	Before usage, the filter delay line *p_out* should be initialized
 *			with the correct lpf_init function.
 *
 *  @internal
 *		For the implementation, the formula stated above is simplified to
 *		out[k] = out[k-1] - (out[k-1] - in)/2^6
 *  @endinternal
 *  @ingroup    filter_lpf_group
 */
 uint16_t lpf6_U16 (uint16_t in, uint16_t *p_out);

/** @brief Calculates (first order) unsigned low pass filter with 16-bit filter delay line with coefficient \f$ 2^{7} \f$
 *
 *  Calculates result of a first order low pass filter on input *in* and filter
 *  delay line values *p_out*, and update the filter delay line *p_out*,
 *  according to the formula:
 *
 *  \f[
 *		out[k] = \frac{(2^{7} - 1) * out[k-1] + in}{2^{7}}
 *  \f]
 *
 *  This is a special version of a first order IIR filter with coefficient
 *  \f$ a = -2^{-n} \f$ with n = 7
 *
 *  The output is calculated exactly, without loss of precision, and the entire
 *  input range is supported
 *
 *  @param[in]		in      Input value for filter
 *  @param[in, out]	p_out   Filter delay line
 *
 *  @return         The result of the filter calculation
 *
 *  @note 	Before usage, the filter delay line *p_out* should be initialized
 *			with the correct lpf_init function.
 *
 *  @internal
 *		For the implementation, the formula stated above is simplified to
 *		out[k] = out[k-1] - (out[k-1] - in)/2^7
 *  @endinternal
 *  @ingroup    filter_lpf_group
 */
 uint16_t lpf7_U16 (uint16_t in, uint16_t *p_out);

/** @brief Calculates (first order) unsigned low pass filter with 16-bit filter delay line with coefficient \f$ 2^{8} \f$
 *
 *  Calculates result of a first order low pass filter on input *in* and filter
 *  delay line values *p_out*, and update the filter delay line *p_out*,
 *  according to the formula:
 *
 *  \f[
 *		out[k] = \frac{(2^{8} - 1) * out[k-1] + in}{2^{8}}
 *  \f]
 *
 *  This is a special version of a first order IIR filter with coefficient
 *  \f$ a = -2^{-n} \f$ with n = 8
 *
 *  The output is calculated exactly, without loss of precision, and the entire
 *  input range is supported
 *
 *  @param[in]		in      Input value for filter
 *  @param[in, out]	p_out   Filter delay line
 *
 *  @return         The result of the filter calculation
 *
 *  @note 	Before usage, the filter delay line *p_out* should be initialized
 *			with the correct lpf_init function.
 *
 *  @internal
 *		For the implementation, the formula stated above is simplified to
 *		out[k] = out[k-1] - (out[k-1] - in)/2^8
 *  @endinternal
 *  @ingroup    filter_lpf_group
 */
 uint16_t lpf8_U16 (uint16_t in, uint16_t *p_out);

/** @brief Calculates (first order) signed low pass filter with 16-bit filter delay line with coefficient \f$ 2^{1} \f$
 *
 *  Calculates result of a first order low pass filter on input *in* and filter
 *  delay line values *p_out*, and update the filter delay line *p_out*,
 *  according to the formula:
 *
 *  \f[
 *		out[k] = \frac{(2^{1} - 1) * out[k-1] + in}{2^{1}}
 *  \f]
 *
 *  This is a special version of a first order IIR filter with coefficient
 *  \f$ a = -2^{-n} \f$ with n = 1
 *
 *  The output is calculated exactly, without loss of precision, and the entire
 *  input range is supported
 *
 *  @param[in]		in      Input value for filter
 *  @param[in, out]	p_out   Filter delay line
 *
 *  @return         The result of the filter calculation
 *
 *  @note 	Before usage, the filter delay line *p_out* should be initialized
 *			with the correct lpf_init function.
 *
 *  @internal
 *		For the implementation, the formula stated above is simplified to
 *		out[k] = out[k-1] - (out[k-1] - in)/2^1
 *  @endinternal
 *  @ingroup    filter_lpf_group
 */
 int16_t lpf1_I16 (int16_t in, int16_t *p_out);

/** @brief Calculates (first order) signed low pass filter with 16-bit filter delay line with coefficient \f$ 2^{2} \f$
 *
 *  Calculates result of a first order low pass filter on input *in* and filter
 *  delay line values *p_out*, and update the filter delay line *p_out*,
 *  according to the formula:
 *
 *  \f[
 *		out[k] = \frac{(2^{2} - 1) * out[k-1] + in}{2^{2}}
 *  \f]
 *
 *  This is a special version of a first order IIR filter with coefficient
 *  \f$ a = -2^{-n} \f$ with n = 2
 *
 *  The output is calculated exactly, without loss of precision, and the entire
 *  input range is supported
 *
 *  @param[in]		in      Input value for filter
 *  @param[in, out]	p_out   Filter delay line
 *
 *  @return         The result of the filter calculation
 *
 *  @note 	Before usage, the filter delay line *p_out* should be initialized
 *			with the correct lpf_init function.
 *
 *  @internal
 *		For the implementation, the formula stated above is simplified to
 *		out[k] = out[k-1] - (out[k-1] - in)/2^2
 *  @endinternal
 *  @ingroup    filter_lpf_group
 */
 int16_t lpf2_I16 (int16_t in, int16_t *p_out);

/** @brief Calculates (first order) signed low pass filter with 16-bit filter delay line with coefficient \f$ 2^{3} \f$
 *
 *  Calculates result of a first order low pass filter on input *in* and filter
 *  delay line values *p_out*, and update the filter delay line *p_out*,
 *  according to the formula:
 *
 *  \f[
 *		out[k] = \frac{(2^{3} - 1) * out[k-1] + in}{2^{3}}
 *  \f]
 *
 *  This is a special version of a first order IIR filter with coefficient
 *  \f$ a = -2^{-n} \f$ with n = 3
 *
 *  The output is calculated exactly, without loss of precision, and the entire
 *  input range is supported
 *
 *  @param[in]		in      Input value for filter
 *  @param[in, out]	p_out   Filter delay line
 *
 *  @return         The result of the filter calculation
 *
 *  @note 	Before usage, the filter delay line *p_out* should be initialized
 *			with the correct lpf_init function.
 *
 *  @internal
 *		For the implementation, the formula stated above is simplified to
 *		out[k] = out[k-1] - (out[k-1] - in)/2^3
 *  @endinternal
 *  @ingroup    filter_lpf_group
 */
 int16_t lpf3_I16 (int16_t in, int16_t *p_out);

/** @brief Calculates (first order) signed low pass filter with 16-bit filter delay line with coefficient \f$ 2^{4} \f$
 *
 *  Calculates result of a first order low pass filter on input *in* and filter
 *  delay line values *p_out*, and update the filter delay line *p_out*,
 *  according to the formula:
 *
 *  \f[
 *		out[k] = \frac{(2^{4} - 1) * out[k-1] + in}{2^{4}}
 *  \f]
 *
 *  This is a special version of a first order IIR filter with coefficient
 *  \f$ a = -2^{-n} \f$ with n = 4
 *
 *  The output is calculated exactly, without loss of precision, and the entire
 *  input range is supported
 *
 *  @param[in]		in      Input value for filter
 *  @param[in, out]	p_out   Filter delay line
 *
 *  @return         The result of the filter calculation
 *
 *  @note 	Before usage, the filter delay line *p_out* should be initialized
 *			with the correct lpf_init function.
 *
 *  @internal
 *		For the implementation, the formula stated above is simplified to
 *		out[k] = out[k-1] - (out[k-1] - in)/2^4
 *  @endinternal
 *  @ingroup    filter_lpf_group
 */
 int16_t lpf4_I16 (int16_t in, int16_t *p_out);

/** @brief Calculates (first order) signed low pass filter with 16-bit filter delay line with coefficient \f$ 2^{5} \f$
 *
 *  Calculates result of a first order low pass filter on input *in* and filter
 *  delay line values *p_out*, and update the filter delay line *p_out*,
 *  according to the formula:
 *
 *  \f[
 *		out[k] = \frac{(2^{5} - 1) * out[k-1] + in}{2^{5}}
 *  \f]
 *
 *  This is a special version of a first order IIR filter with coefficient
 *  \f$ a = -2^{-n} \f$ with n = 5
 *
 *  The output is calculated exactly, without loss of precision, and the entire
 *  input range is supported
 *
 *  @param[in]		in      Input value for filter
 *  @param[in, out]	p_out   Filter delay line
 *
 *  @return         The result of the filter calculation
 *
 *  @note 	Before usage, the filter delay line *p_out* should be initialized
 *			with the correct lpf_init function.
 *
 *  @internal
 *		For the implementation, the formula stated above is simplified to
 *		out[k] = out[k-1] - (out[k-1] - in)/2^5
 *  @endinternal
 *  @ingroup    filter_lpf_group
 */
 int16_t lpf5_I16 (int16_t in, int16_t *p_out);

/** @brief Calculates (first order) signed low pass filter with 16-bit filter delay line with coefficient \f$ 2^{6} \f$
 *
 *  Calculates result of a first order low pass filter on input *in* and filter
 *  delay line values *p_out*, and update the filter delay line *p_out*,
 *  according to the formula:
 *
 *  \f[
 *		out[k] = \frac{(2^{6} - 1) * out[k-1] + in}{2^{6}}
 *  \f]
 *
 *  This is a special version of a first order IIR filter with coefficient
 *  \f$ a = -2^{-n} \f$ with n = 6
 *
 *  The output is calculated exactly, without loss of precision, and the entire
 *  input range is supported
 *
 *  @param[in]		in      Input value for filter
 *  @param[in, out]	p_out   Filter delay line
 *
 *  @return         The result of the filter calculation
 *
 *  @note 	Before usage, the filter delay line *p_out* should be initialized
 *			with the correct lpf_init function.
 *
 *  @internal
 *		For the implementation, the formula stated above is simplified to
 *		out[k] = out[k-1] - (out[k-1] - in)/2^6
 *  @endinternal
 *  @ingroup    filter_lpf_group
 */
 int16_t lpf6_I16 (int16_t in, int16_t *p_out);

/** @brief Calculates (first order) signed low pass filter with 16-bit filter delay line with coefficient \f$ 2^{7} \f$
 *
 *  Calculates result of a first order low pass filter on input *in* and filter
 *  delay line values *p_out*, and update the filter delay line *p_out*,
 *  according to the formula:
 *
 *  \f[
 *		out[k] = \frac{(2^{7} - 1) * out[k-1] + in}{2^{7}}
 *  \f]
 *
 *  This is a special version of a first order IIR filter with coefficient
 *  \f$ a = -2^{-n} \f$ with n = 7
 *
 *  The output is calculated exactly, without loss of precision, and the entire
 *  input range is supported
 *
 *  @param[in]		in      Input value for filter
 *  @param[in, out]	p_out   Filter delay line
 *
 *  @return         The result of the filter calculation
 *
 *  @note 	Before usage, the filter delay line *p_out* should be initialized
 *			with the correct lpf_init function.
 *
 *  @internal
 *		For the implementation, the formula stated above is simplified to
 *		out[k] = out[k-1] - (out[k-1] - in)/2^7
 *  @endinternal
 *  @ingroup    filter_lpf_group
 */
 int16_t lpf7_I16 (int16_t in, int16_t *p_out);

/** @brief Calculates (first order) signed low pass filter with 16-bit filter delay line with coefficient \f$ 2^{8} \f$
 *
 *  Calculates result of a first order low pass filter on input *in* and filter
 *  delay line values *p_out*, and update the filter delay line *p_out*,
 *  according to the formula:
 *
 *  \f[
 *		out[k] = \frac{(2^{8} - 1) * out[k-1] + in}{2^{8}}
 *  \f]
 *
 *  This is a special version of a first order IIR filter with coefficient
 *  \f$ a = -2^{-n} \f$ with n = 8
 *
 *  The output is calculated exactly, without loss of precision, and the entire
 *  input range is supported
 *
 *  @param[in]		in      Input value for filter
 *  @param[in, out]	p_out   Filter delay line
 *
 *  @return         The result of the filter calculation
 *
 *  @note 	Before usage, the filter delay line *p_out* should be initialized
 *			with the correct lpf_init function.
 *
 *  @internal
 *		For the implementation, the formula stated above is simplified to
 *		out[k] = out[k-1] - (out[k-1] - in)/2^8
 *  @endinternal
 *  @ingroup    filter_lpf_group
 */
 int16_t lpf8_I16 (int16_t in, int16_t *p_out);

/** @brief Calculates (first order) unsigned low pass filter with 32-bit filter delay line with coefficient \f$ 2^{8} \f$
 *
 *  Calculates result of a first order low pass filter on input *in* and filter
 *  delay line values *p_out*, and update the filter delay line *p_out*,
 *  according to the formula:
 *
 *  \f[
 *		out[k] = \frac{(2^{8} - 1) * out[k-1] + in}{2^{8}}
 *  \f]
 *
 *  This is a special version of a first order IIR filter with coefficient
 *  \f$ a = -2^{-n} \f$ with n = 8
 *
 *  The output is calculated exactly, without loss of precision, and the entire
 *  input range is supported
 *
 *  @param[in]		in      Input value for filter
 *  @param[in, out]	p_out   Filter delay line
 *
 *  @return         The result of the filter calculation
 *
 *  @note 	Before usage, the filter delay line *p_out* should be initialized
 *			with the correct lpf_init function.
 *
 *  @internal
 *		For the implementation, the formula stated above is simplified to
 *		out[k] = out[k-1] - (out[k-1] - in)/2^8
 *  @endinternal
 *  @ingroup    filter_lpf_group
 */
 uint16_t lpf8_U32 (uint16_t in, uint32_t *p_out);

/** @brief Calculates (first order) unsigned low pass filter with 32-bit filter delay line with coefficient \f$ 2^{9} \f$
 *
 *  Calculates result of a first order low pass filter on input *in* and filter
 *  delay line values *p_out*, and update the filter delay line *p_out*,
 *  according to the formula:
 *
 *  \f[
 *		out[k] = \frac{(2^{9} - 1) * out[k-1] + in}{2^{9}}
 *  \f]
 *
 *  This is a special version of a first order IIR filter with coefficient
 *  \f$ a = -2^{-n} \f$ with n = 9
 *
 *  The output is calculated exactly, without loss of precision, and the entire
 *  input range is supported
 *
 *  @param[in]		in      Input value for filter
 *  @param[in, out]	p_out   Filter delay line
 *
 *  @return         The result of the filter calculation
 *
 *  @note 	Before usage, the filter delay line *p_out* should be initialized
 *			with the correct lpf_init function.
 *
 *  @internal
 *		For the implementation, the formula stated above is simplified to
 *		out[k] = out[k-1] - (out[k-1] - in)/2^9
 *  @endinternal
 *  @ingroup    filter_lpf_group
 */
 uint16_t lpf9_U32 (uint16_t in, uint32_t *p_out);

/** @brief Calculates (first order) unsigned low pass filter with 32-bit filter delay line with coefficient \f$ 2^{10} \f$
 *
 *  Calculates result of a first order low pass filter on input *in* and filter
 *  delay line values *p_out*, and update the filter delay line *p_out*,
 *  according to the formula:
 *
 *  \f[
 *		out[k] = \frac{(2^{10} - 1) * out[k-1] + in}{2^{10}}
 *  \f]
 *
 *  This is a special version of a first order IIR filter with coefficient
 *  \f$ a = -2^{-n} \f$ with n = 10
 *
 *  The output is calculated exactly, without loss of precision, and the entire
 *  input range is supported
 *
 *  @param[in]		in      Input value for filter
 *  @param[in, out]	p_out   Filter delay line
 *
 *  @return         The result of the filter calculation
 *
 *  @note 	Before usage, the filter delay line *p_out* should be initialized
 *			with the correct lpf_init function.
 *
 *  @internal
 *		For the implementation, the formula stated above is simplified to
 *		out[k] = out[k-1] - (out[k-1] - in)/2^10
 *  @endinternal
 *  @ingroup    filter_lpf_group
 */
 uint16_t lpf10_U32 (uint16_t in, uint32_t *p_out);

/** @brief Calculates (first order) unsigned low pass filter with 32-bit filter delay line with coefficient \f$ 2^{11} \f$
 *
 *  Calculates result of a first order low pass filter on input *in* and filter
 *  delay line values *p_out*, and update the filter delay line *p_out*,
 *  according to the formula:
 *
 *  \f[
 *		out[k] = \frac{(2^{11} - 1) * out[k-1] + in}{2^{11}}
 *  \f]
 *
 *  This is a special version of a first order IIR filter with coefficient
 *  \f$ a = -2^{-n} \f$ with n = 11
 *
 *  The output is calculated exactly, without loss of precision, and the entire
 *  input range is supported
 *
 *  @param[in]		in      Input value for filter
 *  @param[in, out]	p_out   Filter delay line
 *
 *  @return         The result of the filter calculation
 *
 *  @note 	Before usage, the filter delay line *p_out* should be initialized
 *			with the correct lpf_init function.
 *
 *  @internal
 *		For the implementation, the formula stated above is simplified to
 *		out[k] = out[k-1] - (out[k-1] - in)/2^11
 *  @endinternal
 *  @ingroup    filter_lpf_group
 */
 uint16_t lpf11_U32 (uint16_t in, uint32_t *p_out);

/** @brief Calculates (first order) unsigned low pass filter with 32-bit filter delay line with coefficient \f$ 2^{12} \f$
 *
 *  Calculates result of a first order low pass filter on input *in* and filter
 *  delay line values *p_out*, and update the filter delay line *p_out*,
 *  according to the formula:
 *
 *  \f[
 *		out[k] = \frac{(2^{12} - 1) * out[k-1] + in}{2^{12}}
 *  \f]
 *
 *  This is a special version of a first order IIR filter with coefficient
 *  \f$ a = -2^{-n} \f$ with n = 12
 *
 *  The output is calculated exactly, without loss of precision, and the entire
 *  input range is supported
 *
 *  @param[in]		in      Input value for filter
 *  @param[in, out]	p_out   Filter delay line
 *
 *  @return         The result of the filter calculation
 *
 *  @note 	Before usage, the filter delay line *p_out* should be initialized
 *			with the correct lpf_init function.
 *
 *  @internal
 *		For the implementation, the formula stated above is simplified to
 *		out[k] = out[k-1] - (out[k-1] - in)/2^12
 *  @endinternal
 *  @ingroup    filter_lpf_group
 */
 uint16_t lpf12_U32 (uint16_t in, uint32_t *p_out);

/** @brief Calculates (first order) unsigned low pass filter with 32-bit filter delay line with coefficient \f$ 2^{13} \f$
 *
 *  Calculates result of a first order low pass filter on input *in* and filter
 *  delay line values *p_out*, and update the filter delay line *p_out*,
 *  according to the formula:
 *
 *  \f[
 *		out[k] = \frac{(2^{13} - 1) * out[k-1] + in}{2^{13}}
 *  \f]
 *
 *  This is a special version of a first order IIR filter with coefficient
 *  \f$ a = -2^{-n} \f$ with n = 13
 *
 *  The output is calculated exactly, without loss of precision, and the entire
 *  input range is supported
 *
 *  @param[in]		in      Input value for filter
 *  @param[in, out]	p_out   Filter delay line
 *
 *  @return         The result of the filter calculation
 *
 *  @note 	Before usage, the filter delay line *p_out* should be initialized
 *			with the correct lpf_init function.
 *
 *  @internal
 *		For the implementation, the formula stated above is simplified to
 *		out[k] = out[k-1] - (out[k-1] - in)/2^13
 *  @endinternal
 *  @ingroup    filter_lpf_group
 */
 uint16_t lpf13_U32 (uint16_t in, uint32_t *p_out);

/** @brief Calculates (first order) unsigned low pass filter with 32-bit filter delay line with coefficient \f$ 2^{14} \f$
 *
 *  Calculates result of a first order low pass filter on input *in* and filter
 *  delay line values *p_out*, and update the filter delay line *p_out*,
 *  according to the formula:
 *
 *  \f[
 *		out[k] = \frac{(2^{14} - 1) * out[k-1] + in}{2^{14}}
 *  \f]
 *
 *  This is a special version of a first order IIR filter with coefficient
 *  \f$ a = -2^{-n} \f$ with n = 14
 *
 *  The output is calculated exactly, without loss of precision, and the entire
 *  input range is supported
 *
 *  @param[in]		in      Input value for filter
 *  @param[in, out]	p_out   Filter delay line
 *
 *  @return         The result of the filter calculation
 *
 *  @note 	Before usage, the filter delay line *p_out* should be initialized
 *			with the correct lpf_init function.
 *
 *  @internal
 *		For the implementation, the formula stated above is simplified to
 *		out[k] = out[k-1] - (out[k-1] - in)/2^14
 *  @endinternal
 *  @ingroup    filter_lpf_group
 */
 uint16_t lpf14_U32 (uint16_t in, uint32_t *p_out);

/** @brief Calculates (first order) unsigned low pass filter with 32-bit filter delay line with coefficient \f$ 2^{15} \f$
 *
 *  Calculates result of a first order low pass filter on input *in* and filter
 *  delay line values *p_out*, and update the filter delay line *p_out*,
 *  according to the formula:
 *
 *  \f[
 *		out[k] = \frac{(2^{15} - 1) * out[k-1] + in}{2^{15}}
 *  \f]
 *
 *  This is a special version of a first order IIR filter with coefficient
 *  \f$ a = -2^{-n} \f$ with n = 15
 *
 *  The output is calculated exactly, without loss of precision, and the entire
 *  input range is supported
 *
 *  @param[in]		in      Input value for filter
 *  @param[in, out]	p_out   Filter delay line
 *
 *  @return         The result of the filter calculation
 *
 *  @note 	Before usage, the filter delay line *p_out* should be initialized
 *			with the correct lpf_init function.
 *
 *  @internal
 *		For the implementation, the formula stated above is simplified to
 *		out[k] = out[k-1] - (out[k-1] - in)/2^15
 *  @endinternal
 *  @ingroup    filter_lpf_group
 */
 uint16_t lpf15_U32 (uint16_t in, uint32_t *p_out);

/** @brief Calculates (first order) signed low pass filter with 32-bit filter delay line with coefficient \f$ 2^{8} \f$
 *
 *  Calculates result of a first order low pass filter on input *in* and filter
 *  delay line values *p_out*, and update the filter delay line *p_out*,
 *  according to the formula:
 *
 *  \f[
 *		out[k] = \frac{(2^{8} - 1) * out[k-1] + in}{2^{8}}
 *  \f]
 *
 *  This is a special version of a first order IIR filter with coefficient
 *  \f$ a = -2^{-n} \f$ with n = 8
 *
 *  The output is calculated exactly, without loss of precision, and the entire
 *  input range is supported
 *
 *  @param[in]		in      Input value for filter
 *  @param[in, out]	p_out   Filter delay line
 *
 *  @return         The result of the filter calculation
 *
 *  @note 	Before usage, the filter delay line *p_out* should be initialized
 *			with the correct lpf_init function.
 *
 *  @internal
 *		For the implementation, the formula stated above is simplified to
 *		out[k] = out[k-1] - (out[k-1] - in)/2^8
 *  @endinternal
 *  @ingroup    filter_lpf_group
 */
 int16_t lpf8_I32 (int16_t in, int32_t *p_out);

/** @brief Calculates (first order) signed low pass filter with 32-bit filter delay line with coefficient \f$ 2^{9} \f$
 *
 *  Calculates result of a first order low pass filter on input *in* and filter
 *  delay line values *p_out*, and update the filter delay line *p_out*,
 *  according to the formula:
 *
 *  \f[
 *		out[k] = \frac{(2^{9} - 1) * out[k-1] + in}{2^{9}}
 *  \f]
 *
 *  This is a special version of a first order IIR filter with coefficient
 *  \f$ a = -2^{-n} \f$ with n = 9
 *
 *  The output is calculated exactly, without loss of precision, and the entire
 *  input range is supported
 *
 *  @param[in]		in      Input value for filter
 *  @param[in, out]	p_out   Filter delay line
 *
 *  @return         The result of the filter calculation
 *
 *  @note 	Before usage, the filter delay line *p_out* should be initialized
 *			with the correct lpf_init function.
 *
 *  @internal
 *		For the implementation, the formula stated above is simplified to
 *		out[k] = out[k-1] - (out[k-1] - in)/2^9
 *  @endinternal
 *  @ingroup    filter_lpf_group
 */
 int16_t lpf9_I32 (int16_t in, int32_t *p_out);

/** @brief Calculates (first order) signed low pass filter with 32-bit filter delay line with coefficient \f$ 2^{10} \f$
 *
 *  Calculates result of a first order low pass filter on input *in* and filter
 *  delay line values *p_out*, and update the filter delay line *p_out*,
 *  according to the formula:
 *
 *  \f[
 *		out[k] = \frac{(2^{10} - 1) * out[k-1] + in}{2^{10}}
 *  \f]
 *
 *  This is a special version of a first order IIR filter with coefficient
 *  \f$ a = -2^{-n} \f$ with n = 10
 *
 *  The output is calculated exactly, without loss of precision, and the entire
 *  input range is supported
 *
 *  @param[in]		in      Input value for filter
 *  @param[in, out]	p_out   Filter delay line
 *
 *  @return         The result of the filter calculation
 *
 *  @note 	Before usage, the filter delay line *p_out* should be initialized
 *			with the correct lpf_init function.
 *
 *  @internal
 *		For the implementation, the formula stated above is simplified to
 *		out[k] = out[k-1] - (out[k-1] - in)/2^10
 *  @endinternal
 *  @ingroup    filter_lpf_group
 */
 int16_t lpf10_I32 (int16_t in, int32_t *p_out);

/** @brief Calculates (first order) signed low pass filter with 32-bit filter delay line with coefficient \f$ 2^{11} \f$
 *
 *  Calculates result of a first order low pass filter on input *in* and filter
 *  delay line values *p_out*, and update the filter delay line *p_out*,
 *  according to the formula:
 *
 *  \f[
 *		out[k] = \frac{(2^{11} - 1) * out[k-1] + in}{2^{11}}
 *  \f]
 *
 *  This is a special version of a first order IIR filter with coefficient
 *  \f$ a = -2^{-n} \f$ with n = 11
 *
 *  The output is calculated exactly, without loss of precision, and the entire
 *  input range is supported
 *
 *  @param[in]		in      Input value for filter
 *  @param[in, out]	p_out   Filter delay line
 *
 *  @return         The result of the filter calculation
 *
 *  @note 	Before usage, the filter delay line *p_out* should be initialized
 *			with the correct lpf_init function.
 *
 *  @internal
 *		For the implementation, the formula stated above is simplified to
 *		out[k] = out[k-1] - (out[k-1] - in)/2^11
 *  @endinternal
 *  @ingroup    filter_lpf_group
 */
 int16_t lpf11_I32 (int16_t in, int32_t *p_out);

/** @brief Calculates (first order) signed low pass filter with 32-bit filter delay line with coefficient \f$ 2^{12} \f$
 *
 *  Calculates result of a first order low pass filter on input *in* and filter
 *  delay line values *p_out*, and update the filter delay line *p_out*,
 *  according to the formula:
 *
 *  \f[
 *		out[k] = \frac{(2^{12} - 1) * out[k-1] + in}{2^{12}}
 *  \f]
 *
 *  This is a special version of a first order IIR filter with coefficient
 *  \f$ a = -2^{-n} \f$ with n = 12
 *
 *  The output is calculated exactly, without loss of precision, and the entire
 *  input range is supported
 *
 *  @param[in]		in      Input value for filter
 *  @param[in, out]	p_out   Filter delay line
 *
 *  @return         The result of the filter calculation
 *
 *  @note 	Before usage, the filter delay line *p_out* should be initialized
 *			with the correct lpf_init function.
 *
 *  @internal
 *		For the implementation, the formula stated above is simplified to
 *		out[k] = out[k-1] - (out[k-1] - in)/2^12
 *  @endinternal
 *  @ingroup    filter_lpf_group
 */
 int16_t lpf12_I32 (int16_t in, int32_t *p_out);

/** @brief Calculates (first order) signed low pass filter with 32-bit filter delay line with coefficient \f$ 2^{13} \f$
 *
 *  Calculates result of a first order low pass filter on input *in* and filter
 *  delay line values *p_out*, and update the filter delay line *p_out*,
 *  according to the formula:
 *
 *  \f[
 *		out[k] = \frac{(2^{13} - 1) * out[k-1] + in}{2^{13}}
 *  \f]
 *
 *  This is a special version of a first order IIR filter with coefficient
 *  \f$ a = -2^{-n} \f$ with n = 13
 *
 *  The output is calculated exactly, without loss of precision, and the entire
 *  input range is supported
 *
 *  @param[in]		in      Input value for filter
 *  @param[in, out]	p_out   Filter delay line
 *
 *  @return         The result of the filter calculation
 *
 *  @note 	Before usage, the filter delay line *p_out* should be initialized
 *			with the correct lpf_init function.
 *
 *  @internal
 *		For the implementation, the formula stated above is simplified to
 *		out[k] = out[k-1] - (out[k-1] - in)/2^13
 *  @endinternal
 *  @ingroup    filter_lpf_group
 */
 int16_t lpf13_I32 (int16_t in, int32_t *p_out);

/** @brief Calculates (first order) signed low pass filter with 32-bit filter delay line with coefficient \f$ 2^{14} \f$
 *
 *  Calculates result of a first order low pass filter on input *in* and filter
 *  delay line values *p_out*, and update the filter delay line *p_out*,
 *  according to the formula:
 *
 *  \f[
 *		out[k] = \frac{(2^{14} - 1) * out[k-1] + in}{2^{14}}
 *  \f]
 *
 *  This is a special version of a first order IIR filter with coefficient
 *  \f$ a = -2^{-n} \f$ with n = 14
 *
 *  The output is calculated exactly, without loss of precision, and the entire
 *  input range is supported
 *
 *  @param[in]		in      Input value for filter
 *  @param[in, out]	p_out   Filter delay line
 *
 *  @return         The result of the filter calculation
 *
 *  @note 	Before usage, the filter delay line *p_out* should be initialized
 *			with the correct lpf_init function.
 *
 *  @internal
 *		For the implementation, the formula stated above is simplified to
 *		out[k] = out[k-1] - (out[k-1] - in)/2^14
 *  @endinternal
 *  @ingroup    filter_lpf_group
 */
 int16_t lpf14_I32 (int16_t in, int32_t *p_out);

/** @brief Calculates (first order) signed low pass filter with 32-bit filter delay line with coefficient \f$ 2^{15} \f$
 *
 *  Calculates result of a first order low pass filter on input *in* and filter
 *  delay line values *p_out*, and update the filter delay line *p_out*,
 *  according to the formula:
 *
 *  \f[
 *		out[k] = \frac{(2^{15} - 1) * out[k-1] + in}{2^{15}}
 *  \f]
 *
 *  This is a special version of a first order IIR filter with coefficient
 *  \f$ a = -2^{-n} \f$ with n = 15
 *
 *  The output is calculated exactly, without loss of precision, and the entire
 *  input range is supported
 *
 *  @param[in]		in      Input value for filter
 *  @param[in, out]	p_out   Filter delay line
 *
 *  @return         The result of the filter calculation
 *
 *  @note 	Before usage, the filter delay line *p_out* should be initialized
 *			with the correct lpf_init function.
 *
 *  @internal
 *		For the implementation, the formula stated above is simplified to
 *		out[k] = out[k-1] - (out[k-1] - in)/2^15
 *  @endinternal
 *  @ingroup    filter_lpf_group
 */
 int16_t lpf15_I32 (int16_t in, int32_t *p_out);

/** Calculates unsigned hysteresis filter with fixed hysteresis value 16
 *
 *  The fhystn functions return the output of a hysteresis filter with
 *  hysteresis value \b 16 on input \b in and filter delay line values \b p_out,
 *  and update the filter delay line \b p_out.
 *
 *  The behavior of hysteresis filter is that the output is not updated when
 *  the input step is smaller than the hysteresis.
 *  The output is modified when the input step is larger than the hysteresis.
 *
 *  \f{equation}{
 *      \begin{cases}
 *          in + 16  & \text{if(out[k-1] -in) $>$ 16}\\
 *          in - 16  & \text{if(in-out[k-1]) $>$ 16}\\
 *          in             & \text{otherwise}
 *      \end{cases}
 *  \f}
 *
 *  @param[in]      in      Input value
 *  @param[in, out] *p_out  Filter delay line
 *
 *  @return         The result of the filter calculation
 *  @ingroup        filter_hysteresis_group
 */
uint16_t fhyst16_U16c (uint16_t in, uint16_t * const p_out);


/** @brief 	Calculates unsigned hysteresis filter with fixed hysteresis value  1 
 *
 *  The fhystn functions return the output of a hysteresis filter  with
 *  configurable hysteresis value 1 on input \b in and filter delay line values \b p_out,
 *  and update the filter delay line \b p_out.
 *
 *  The behavior of hysteresis filter is that the output is not updated when
 *  the input step is smaller than the hysteresis.
 *  The output is modified when the input step is larger than the hysteresis.
 *
 *  \f{equation}{
 *      \begin{cases}
 *          in + 1  & \text{if(out[k-1] -in) $>$ 1}\\
 *          in - 1  & \text{if(in-out[k-1]) $>$ 1}\\
 *          in               & \text{otherwise}
 *      \end{cases}
 *  \f}
 *
 *  @param[in]		in      Input value
 *  @param[in, out]	*p_out  Filter delay line
 * 
 *
 *  @return	     	The result of the filter calculation
 *  @ingroup        filter_hysteresis_group
 */
 uint16_t fhyst1_U16 (uint16_t in, uint16_t *p_out);

/** @brief 	Calculates unsigned hysteresis filter with fixed hysteresis value  16 
 *
 *  The fhystn functions return the output of a hysteresis filter  with
 *  configurable hysteresis value 16 on input \b in and filter delay line values \b p_out,
 *  and update the filter delay line \b p_out.
 *
 *  The behavior of hysteresis filter is that the output is not updated when
 *  the input step is smaller than the hysteresis.
 *  The output is modified when the input step is larger than the hysteresis.
 *
 *  \f{equation}{
 *      \begin{cases}
 *          in + 16  & \text{if(out[k-1] -in) $>$ 16}\\
 *          in - 16  & \text{if(in-out[k-1]) $>$ 16}\\
 *          in               & \text{otherwise}
 *      \end{cases}
 *  \f}
 *
 *  @param[in]		in      Input value
 *  @param[in, out]	*p_out  Filter delay line
 * 
 *
 *  @return	     	The result of the filter calculation
 *  @ingroup        filter_hysteresis_group
 */
 uint16_t fhyst16_U16 (uint16_t in, uint16_t *p_out);

/** @brief 	Calculates unsigned hysteresis filter with fixed hysteresis value  64 
 *
 *  The fhystn functions return the output of a hysteresis filter  with
 *  configurable hysteresis value 64 on input \b in and filter delay line values \b p_out,
 *  and update the filter delay line \b p_out.
 *
 *  The behavior of hysteresis filter is that the output is not updated when
 *  the input step is smaller than the hysteresis.
 *  The output is modified when the input step is larger than the hysteresis.
 *
 *  \f{equation}{
 *      \begin{cases}
 *          in + 64  & \text{if(out[k-1] -in) $>$ 64}\\
 *          in - 64  & \text{if(in-out[k-1]) $>$ 64}\\
 *          in               & \text{otherwise}
 *      \end{cases}
 *  \f}
 *
 *  @param[in]		in      Input value
 *  @param[in, out]	*p_out  Filter delay line
 * 
 *
 *  @return	     	The result of the filter calculation
 *  @ingroup        filter_hysteresis_group
 */
 uint16_t fhyst64_U16 (uint16_t in, uint16_t *p_out);

/** @brief 	Calculates unsigned hysteresis filter with fixed hysteresis value  256 
 *
 *  The fhystn functions return the output of a hysteresis filter  with
 *  configurable hysteresis value 256 on input \b in and filter delay line values \b p_out,
 *  and update the filter delay line \b p_out.
 *
 *  The behavior of hysteresis filter is that the output is not updated when
 *  the input step is smaller than the hysteresis.
 *  The output is modified when the input step is larger than the hysteresis.
 *
 *  \f{equation}{
 *      \begin{cases}
 *          in + 256  & \text{if(out[k-1] -in) $>$ 256}\\
 *          in - 256  & \text{if(in-out[k-1]) $>$ 256}\\
 *          in               & \text{otherwise}
 *      \end{cases}
 *  \f}
 *
 *  @param[in]		in      Input value
 *  @param[in, out]	*p_out  Filter delay line
 * 
 *
 *  @return	     	The result of the filter calculation
 *  @ingroup        filter_hysteresis_group
 */
 uint16_t fhyst256_U16 (uint16_t in, uint16_t *p_out);

/** @brief 	Calculates unsigned hysteresis filter with configurable hysteresis value
 *
 *  The fhystn functions return the output of a hysteresis filter  with
 *  configurable hysteresis value hyst on input \b in and filter delay line values \b p_out,
 *  and update the filter delay line \b p_out.
 *
 *  The behavior of hysteresis filter is that the output is not updated when
 *  the input step is smaller than the hysteresis.
 *  The output is modified when the input step is larger than the hysteresis.
 *
 *  \f{equation}{
 *      \begin{cases}
 *          in + hyst  & \text{if(out[k-1] -in) $>$ hyst}\\
 *          in - hyst  & \text{if(in-out[k-1]) $>$ hyst}\\
 *          in               & \text{otherwise}
 *      \end{cases}
 *  \f}
 *
 *  @param[in]		in      Input value
 *  @param[in, out]	*p_out  Filter delay line
 *	@param[in]		hyst    Hysteresis value (0 < hyst <= \f$ 2^{15} \f$ )
 *
 *  @return	     	The result of the filter calculation
 *  @ingroup        filter_hysteresis_group
 */
 uint16_t fhystn_U16 (uint16_t in, uint16_t *p_out, const uint16_t hyst);

/** Calculates signed hysteresis filter with fixed hysteresis value 16
 *
 *  The fhystn functions return the output of a hysteresis filter with
 *  hysteresis value \b 16 on input \b in and filter delay line values \b p_out,
 *  and update the filter delay line \b p_out.
 *
 *  The behavior of hysteresis filter is that the output is not updated when
 *  the input step is smaller than the hysteresis.
 *  The output is modified when the input step is larger than the hysteresis.
 *
 *  \f{equation}{
 *      \begin{cases}
 *          in + 16  & \text{if(out[k-1] -in) $>$ 16}\\
 *          in - 16  & \text{if(in-out[k-1]) $>$ 16}\\
 *          in             & \text{otherwise}
 *      \end{cases}
 *  \f}
 *
 *  @param[in]      in      Input value
 *  @param[in, out] *p_out  Filter delay line
 *
 *  @return         The result of the filter calculation
 *  @ingroup        filter_hysteresis_group
 */
int16_t fhyst16_I16c (int16_t in, int16_t * const p_out);


/** @brief 	Calculates signed hysteresis filter with fixed hysteresis value  1 
 *
 *  The fhystn functions return the output of a hysteresis filter  with
 *  configurable hysteresis value 1 on input \b in and filter delay line values \b p_out,
 *  and update the filter delay line \b p_out.
 *
 *  The behavior of hysteresis filter is that the output is not updated when
 *  the input step is smaller than the hysteresis.
 *  The output is modified when the input step is larger than the hysteresis.
 *
 *  \f{equation}{
 *      \begin{cases}
 *          in + 1  & \text{if(out[k-1] -in) $>$ 1}\\
 *          in - 1  & \text{if(in-out[k-1]) $>$ 1}\\
 *          in               & \text{otherwise}
 *      \end{cases}
 *  \f}
 *
 *  @param[in]		in      Input value
 *  @param[in, out]	*p_out  Filter delay line
 * 
 *
 *  @return	     	The result of the filter calculation
 *  @ingroup        filter_hysteresis_group
 */
 int16_t fhyst1_I16 (int16_t in, int16_t *p_out);

/** @brief 	Calculates signed hysteresis filter with fixed hysteresis value  16 
 *
 *  The fhystn functions return the output of a hysteresis filter  with
 *  configurable hysteresis value 16 on input \b in and filter delay line values \b p_out,
 *  and update the filter delay line \b p_out.
 *
 *  The behavior of hysteresis filter is that the output is not updated when
 *  the input step is smaller than the hysteresis.
 *  The output is modified when the input step is larger than the hysteresis.
 *
 *  \f{equation}{
 *      \begin{cases}
 *          in + 16  & \text{if(out[k-1] -in) $>$ 16}\\
 *          in - 16  & \text{if(in-out[k-1]) $>$ 16}\\
 *          in               & \text{otherwise}
 *      \end{cases}
 *  \f}
 *
 *  @param[in]		in      Input value
 *  @param[in, out]	*p_out  Filter delay line
 * 
 *
 *  @return	     	The result of the filter calculation
 *  @ingroup        filter_hysteresis_group
 */
 int16_t fhyst16_I16 (int16_t in, int16_t *p_out);

/** @brief 	Calculates signed hysteresis filter with fixed hysteresis value  64 
 *
 *  The fhystn functions return the output of a hysteresis filter  with
 *  configurable hysteresis value 64 on input \b in and filter delay line values \b p_out,
 *  and update the filter delay line \b p_out.
 *
 *  The behavior of hysteresis filter is that the output is not updated when
 *  the input step is smaller than the hysteresis.
 *  The output is modified when the input step is larger than the hysteresis.
 *
 *  \f{equation}{
 *      \begin{cases}
 *          in + 64  & \text{if(out[k-1] -in) $>$ 64}\\
 *          in - 64  & \text{if(in-out[k-1]) $>$ 64}\\
 *          in               & \text{otherwise}
 *      \end{cases}
 *  \f}
 *
 *  @param[in]		in      Input value
 *  @param[in, out]	*p_out  Filter delay line
 * 
 *
 *  @return	     	The result of the filter calculation
 *  @ingroup        filter_hysteresis_group
 */
 int16_t fhyst64_I16 (int16_t in, int16_t *p_out);

/** @brief 	Calculates signed hysteresis filter with fixed hysteresis value  256 
 *
 *  The fhystn functions return the output of a hysteresis filter  with
 *  configurable hysteresis value 256 on input \b in and filter delay line values \b p_out,
 *  and update the filter delay line \b p_out.
 *
 *  The behavior of hysteresis filter is that the output is not updated when
 *  the input step is smaller than the hysteresis.
 *  The output is modified when the input step is larger than the hysteresis.
 *
 *  \f{equation}{
 *      \begin{cases}
 *          in + 256  & \text{if(out[k-1] -in) $>$ 256}\\
 *          in - 256  & \text{if(in-out[k-1]) $>$ 256}\\
 *          in               & \text{otherwise}
 *      \end{cases}
 *  \f}
 *
 *  @param[in]		in      Input value
 *  @param[in, out]	*p_out  Filter delay line
 * 
 *
 *  @return	     	The result of the filter calculation
 *  @ingroup        filter_hysteresis_group
 */
 int16_t fhyst256_I16 (int16_t in, int16_t *p_out);

/** @brief 	Calculates signed hysteresis filter with configurable hysteresis value
 *
 *  The fhystn functions return the output of a hysteresis filter  with
 *  configurable hysteresis value hyst on input \b in and filter delay line values \b p_out,
 *  and update the filter delay line \b p_out.
 *
 *  The behavior of hysteresis filter is that the output is not updated when
 *  the input step is smaller than the hysteresis.
 *  The output is modified when the input step is larger than the hysteresis.
 *
 *  \f{equation}{
 *      \begin{cases}
 *          in + hyst  & \text{if(out[k-1] -in) $>$ hyst}\\
 *          in - hyst  & \text{if(in-out[k-1]) $>$ hyst}\\
 *          in               & \text{otherwise}
 *      \end{cases}
 *  \f}
 *
 *  @param[in]		in      Input value
 *  @param[in, out]	*p_out  Filter delay line
 *	@param[in]		hyst    Hysteresis value (0 < hyst <= \f$ 2^{15} \f$ )
 *
 *  @return	     	The result of the filter calculation
 *  @ingroup        filter_hysteresis_group
 */
 int16_t fhystn_I16 (int16_t in, int16_t *p_out, const uint16_t hyst);

/** Calculates signed angular hysteresis filter with fixed hysteresis value 16
 *
 *  The fhystn functions return the output of a hysteresis filter with
 *  hysteresis value \b 16 on input \b in and filter delay line values \b p_out,
 *  and update the filter delay line \b p_out.
 *
 *  The behavior of hysteresis filter is that the output is not updated when
 *  the input step is smaller than the hysteresis.
 *  The output is modified when the input step is larger than the hysteresis.
 *
 *  \f{equation}{
 *      \begin{cases}
 *          in + 16  & \text{if(out[k-1] -in) $>$ 16}\\
 *          in - 16  & \text{if(in-out[k-1]) $>$ 16}\\
 *          in             & \text{otherwise}
 *      \end{cases}
 *  \f}
 *
 *  @param[in]      in      Input value
 *  @param[in, out] *p_out  Filter delay line
 *
 *  @return         The result of the filter calculation
 *  @ingroup        filter_hysteresis_group
 */
int16_t fhyst16_A16c (int16_t in, int16_t * const p_out);


/** @brief 	Calculates signed angular hysteresis filter with fixed hysteresis value  1 
 *
 *  The fhystn functions return the output of a hysteresis filter (angle modulo 2^n version) with
 *  configurable hysteresis value 1 on input \b in and filter delay line values \b p_out,
 *  and update the filter delay line \b p_out.
 *
 *  The behavior of hysteresis filter is that the output is not updated when
 *  the input step is smaller than the hysteresis.
 *  The output is modified when the input step is larger than the hysteresis.
 *
 *  \f{equation}{
 *      \begin{cases}
 *          in + 1  & \text{if(out[k-1] -in) $>$ 1}\\
 *          in - 1  & \text{if(in-out[k-1]) $>$ 1}\\
 *          in               & \text{otherwise}
 *      \end{cases}
 *  \f}
 *
 *  @param[in]		in      Input value
 *  @param[in, out]	*p_out  Filter delay line
 * 
 *
 *  @return	     	The result of the filter calculation
 *  @ingroup        filter_hysteresis_group
 */
 int16_t fhysta1_I16 (int16_t in, int16_t *p_out);

/** @brief 	Calculates signed angular hysteresis filter with fixed hysteresis value  16 
 *
 *  The fhystn functions return the output of a hysteresis filter (angle modulo 2^n version) with
 *  configurable hysteresis value 16 on input \b in and filter delay line values \b p_out,
 *  and update the filter delay line \b p_out.
 *
 *  The behavior of hysteresis filter is that the output is not updated when
 *  the input step is smaller than the hysteresis.
 *  The output is modified when the input step is larger than the hysteresis.
 *
 *  \f{equation}{
 *      \begin{cases}
 *          in + 16  & \text{if(out[k-1] -in) $>$ 16}\\
 *          in - 16  & \text{if(in-out[k-1]) $>$ 16}\\
 *          in               & \text{otherwise}
 *      \end{cases}
 *  \f}
 *
 *  @param[in]		in      Input value
 *  @param[in, out]	*p_out  Filter delay line
 * 
 *
 *  @return	     	The result of the filter calculation
 *  @ingroup        filter_hysteresis_group
 */
 int16_t fhysta16_I16 (int16_t in, int16_t *p_out);

/** @brief 	Calculates signed angular hysteresis filter with fixed hysteresis value  64 
 *
 *  The fhystn functions return the output of a hysteresis filter (angle modulo 2^n version) with
 *  configurable hysteresis value 64 on input \b in and filter delay line values \b p_out,
 *  and update the filter delay line \b p_out.
 *
 *  The behavior of hysteresis filter is that the output is not updated when
 *  the input step is smaller than the hysteresis.
 *  The output is modified when the input step is larger than the hysteresis.
 *
 *  \f{equation}{
 *      \begin{cases}
 *          in + 64  & \text{if(out[k-1] -in) $>$ 64}\\
 *          in - 64  & \text{if(in-out[k-1]) $>$ 64}\\
 *          in               & \text{otherwise}
 *      \end{cases}
 *  \f}
 *
 *  @param[in]		in      Input value
 *  @param[in, out]	*p_out  Filter delay line
 * 
 *
 *  @return	     	The result of the filter calculation
 *  @ingroup        filter_hysteresis_group
 */
 int16_t fhysta64_I16 (int16_t in, int16_t *p_out);

/** @brief 	Calculates signed angular hysteresis filter with fixed hysteresis value  256 
 *
 *  The fhystn functions return the output of a hysteresis filter (angle modulo 2^n version) with
 *  configurable hysteresis value 256 on input \b in and filter delay line values \b p_out,
 *  and update the filter delay line \b p_out.
 *
 *  The behavior of hysteresis filter is that the output is not updated when
 *  the input step is smaller than the hysteresis.
 *  The output is modified when the input step is larger than the hysteresis.
 *
 *  \f{equation}{
 *      \begin{cases}
 *          in + 256  & \text{if(out[k-1] -in) $>$ 256}\\
 *          in - 256  & \text{if(in-out[k-1]) $>$ 256}\\
 *          in               & \text{otherwise}
 *      \end{cases}
 *  \f}
 *
 *  @param[in]		in      Input value
 *  @param[in, out]	*p_out  Filter delay line
 * 
 *
 *  @return	     	The result of the filter calculation
 *  @ingroup        filter_hysteresis_group
 */
 int16_t fhysta256_I16 (int16_t in, int16_t *p_out);

/** @brief 	Calculates signed angular hysteresis filter with configurable hysteresis value
 *
 *  The fhystn functions return the output of a hysteresis filter (angle modulo 2^n version) with
 *  configurable hysteresis value hyst on input \b in and filter delay line values \b p_out,
 *  and update the filter delay line \b p_out.
 *
 *  The behavior of hysteresis filter is that the output is not updated when
 *  the input step is smaller than the hysteresis.
 *  The output is modified when the input step is larger than the hysteresis.
 *
 *  \f{equation}{
 *      \begin{cases}
 *          in + hyst  & \text{if(out[k-1] -in) $>$ hyst}\\
 *          in - hyst  & \text{if(in-out[k-1]) $>$ hyst}\\
 *          in               & \text{otherwise}
 *      \end{cases}
 *  \f}
 *
 *  @param[in]		in      Input value
 *  @param[in, out]	*p_out  Filter delay line
 *	@param[in]		hyst    Hysteresis value (0 < hyst <= \f$ 2^{15} \f$ )
 *
 *  @return	     	The result of the filter calculation
 *  @ingroup        filter_hysteresis_group
 */
 int16_t fhystan_I16 (int16_t in, int16_t *p_out, const uint16_t hyst);

#endif /* GEN_FILTER_H_ */

/** @}*/

