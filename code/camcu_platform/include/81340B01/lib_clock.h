/**
 * @file
 * @brief Definitions of the clock library
 * @internal
 *
 * @copyright (C) 2020 Melexis N.V.
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
 * @ingroup clock
 * @addtogroup clock Clock
 *
 * @brief Definitions of the clock library
 * @details
 * @{
 */

#ifndef LIB_CLOCK_H_
#define LIB_CLOCK_H_

#include <stdint.h>
#include "compiler_abstraction.h"

/** EEPROM-based configuration description type */
typedef union RC_Settings_t {
    struct __attribute__((packed)) {
        uint16_t RCO_OSC : 10;              /**< 10 bits of data represent the RCO_OSC trimming value for certain frequency */
        uint16_t EE_WaitStatesP : 2;        /**< The Base EEPROM WaitStates value for this frequency */
        uint16_t FL_WaitStatesP : 2;        /**< The Base FLASH WaitStates value for this frequency */
        uint16_t FL_AcSelWaitOption : 1;    /**< Defines whether the FLASH Waitstates will be decreased by 1 in case of AcSel=1 option */
        uint16_t EE_AcSelWaitOption : 1;    /**< Defines whether the EEPROM Waitstates will be decreased by 1 in case of AcSel=1 option */
    };
    uint16_t u;
} RC_Settings_t;

/** Sets the RCO and FLASH/EEPROM wait-states configuration
 *
 * Usage example:
 * @code
 * RC_Settings_t tmp;
 * tmp.u = EE_MS_TRIM6_VALUE;
 * SetSystemSpeed(tmp, 0u);
 * @endcode
 *
 * @param[in] trim_cfg The RCO trimming and FLASH+EEPROM wait-states combined in one 16bit value.
 * @param[in] AcSel The divider for the MCU clock by 2. Set AcSel=1 to divide the MCU clock frequency.
 */
void SetSystemSpeed(const RC_Settings_t trim_cfg, const uint8_t AcSel);

/** SetRcoAndAcSel sets the RCO32M trimming and AcSel bit in one scope
 * @param[in] rco takes the value for the RCO trimming. 10bits value is used.
 * @param[in] AcSel defined whether the divider by 2 will be used for the MCU clock. 1- the divider is used.
 */

STATIC INLINE void SetRcoAndAcSel(uint16_t rco, uint8_t AcSel);

#if !defined (HAS_NO_SYSTEM_CLOCK_ERROR_CALIBRATION)
/** System clock error for speed1 at given temperature
 *
 * @param[in]  raw_temperature  chip temperature raw adc measurement.
 * @return  clock frequency difference vs. nominal, scaled by 1/2^11.
 */
int16_t clock_CalculateClockErrorForSpeed1(uint16_t raw_temperature);

/** System clock error for speed2 at given temperature
 *
 * @param[in]  raw_temperature  chip temperature raw adc measurement.
 * @return  clock frequency difference vs. nominal, scaled by 1/2^11.
 */
int16_t clock_CalculateClockErrorForSpeed2(uint16_t raw_temperature);

/** System clock error for speed3 at given temperature
 *
 * @param[in]  raw_temperature  chip temperature raw adc measurement.
 * @return  clock frequency difference vs. nominal, scaled by 1/2^11.
 */
int16_t clock_CalculateClockErrorForSpeed3(uint16_t raw_temperature);

/** Correct a period with a given system clock error
 *
 * @param[in]  period  period to correct.
 * @param[in]  clock_error  clock error as reported by clock_CalculateClockErrorForSpeedX().
 * @return  corrected period.
 */
STATIC INLINE uint16_t clock_CorrectPeriodWithClockError(uint16_t period, int16_t clock_error);
#endif /* HAS_NO_SYSTEM_CLOCK_ERROR_CALIBRATION */

#if !defined (UNITTEST)
#include "lib_clock_inline_impl.h"
#endif /* UNITTEST */
/// @}
#endif /* LIB_CLOCK_H_ */
