/**
 * @file
 * @brief ADC sample to shunt current conversion library
 * @internal
 *
 * @copyright (C) 2019-2020 Melexis N.V.
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
 * @ingroup calculate_shunt_current
 * @addtogroup calculate_shunt_current Calculate Shunt Current
 *
 * @brief ADC sample to shunt current conversion library
 * @details
 * Definitions of the generic conversion function for the adc module, this will convert adc
 * measurements into SI units.
 * @{
 */

#ifndef LIB_CALCULATE_SHUNT_CURRENT_H_
#define LIB_CALCULATE_SHUNT_CURRENT_H_

#include <stdint.h>

/** Convert adc bits to shunt current
 *
 * @param[in]  adcValue  Raw adc measurement.
 * @return  the current in mA.
 */
int16_t CalculateShuntCurrent(uint16_t adcValue);

/// @}
#endif /* LIB_CALCULATE_SHUNT_CURRENT_H_ */
