/**
 * @file
 * @brief Provides the sleep/stop-mode support function
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
 * @ingroup sleep
 * @addtogroup sleep Low-power Mode
 *
 * @brief Provides the sleep/stop-mode support function
 * @details
 * SLEEP_MODE common settings and definitions. Used to manage the common SLEEP_MODE structure for the chips.
 * @{
 */

#ifndef SLEEP_MODE_H
#define SLEEP_MODE_H

/** Finishes all the processes in IC and goes to sleep
 *
 * The function disables the interrupts, stops ADC, SPI, acknowledges the AWD if needed, disables the COLIN (if present), waits for FLASH and EEPROM and goes to sleep
 */
void enter_SleepMode(void);

/** Finishes all the processes in IC and goes to stop-mode
 *
 * The function disables the interrupts, stops ADC, SPI, acknowledges the AWD if needed, disables the COLIN (if present), waits for FLASH and EEPROM and goes to stop mode
 */
void enter_StopMode(void);

/** Finishes all the processes in IC and goes to stby-mode
 *
 * The function disables the interrupts, stops ADC, SPI, acknowledges the AWD if needed,
 * disables the COLIN (if present), waits for FLASH and EEPROM and goes to standby mode
 */
void enter_StandbyMode(void);


#endif

/// @}
