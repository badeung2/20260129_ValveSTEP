/**
 * @file
 * @brief ADC sample to motor supply voltage conversion library
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
 * @ingroup calculate_motor_supply_voltage
 * @addtogroup calculate_motor_supply_voltage Calculate Voltage
 *
 * @brief ADC sample to motor supply voltage conversion library
 * @details
 * Definitions of the generic conversion function for the adc module, this will convert adc
 * measurements into SI units.
 * @{
 */

#ifndef LIB_CALCULATE_VOLTAGE_H_
#define LIB_CALCULATE_VOLTAGE_H_

#include <stdint.h>

/** Convert adc bits to high voltage (Vs, Vsm, Vph_x, IOx_HV)
 *
 * @param[in]  adcValue  Raw adc measurement.
 * @return  the high voltage in 10 mV.
 */
int16_t CalculateHighVoltage(uint16_t adcValue);

/** Convert adc bits to low voltage (IOx_LV)
 *
 * @param[in]  adcValue  Raw adc measurement.
 * @return  the supply voltage in 1 mV.
 */
int16_t CalculateLowVoltage(uint16_t adcValue);

/** Convert adc bits to motor supply voltage filtered
 *
 * @param[in]  adcValue  Raw adc measurement.
 * @return  the supply voltage in 10 mV.
 */
int16_t CalculateMotorSupplyVoltageFilt(uint16_t adcValue);

/// @}
#endif /* LIB_CALCULATE_VOLTAGE_H_ */
