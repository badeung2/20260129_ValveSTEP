/**
 * @file
 * @brief ROM startup routines for the trimming
 * @internal
 *
 * @copyright (C) 2015-2017 Melexis N.V.
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
 * @ingroup startup
 *
 * @brief ROM startup routines for the trimming
 * @details Set of functions to set-up the trimming and IC's wait-states
 * @{
 */

#ifndef TRIMMING_H
#define TRIMMING_H

/** Compile-time Wait-states and R1,R2,R2 EE and FL settings to be loaded before everything else */
#define ROM_R1_EE      (0x5366u)
#ifdef HAS_HW_FLASH_15 /* see issue #112 on Gitlab. Bootcode rev.20 describes the new update for FLASH 1.4 and 1.5 */
#define ROM_R2_EE      (0x4087u)
#else
#define ROM_R2_EE      (0x4000u)
#endif
#define ROM_WS_EE      (0x0Fu)
#define ROM_WS_FL      (0x07u)

/** TrimAnalog Sets the analog control IO-ports to make the chip voltages, clocks, etc. stable
 *
 * The function uploads the analog trimming registers to their saved configuration from the EEPROM.
 *
 * @note function uses the RAM buffer to read the data from EEPROM and copy them into IO-ports afterwards.
 * This is done to avoid EEPROM-read disturbances during the trimming.
 */
void TrimAnalog(void);

/** Verifies the analog control IO-ports to make the chip voltages, clocks, etc. stable
 *
 * The function confirms that uploaded the analog trimming registers match saved configuration from the EEPROM.
 *
 * @note Function in error cases ends in Error_Handler and might not return.
 */
void TrimAnalogVerify(void);
#ifdef HAS_SUPPLY_SYSTEM_TRIM
void TrimSupplySystem(void);
#endif
void SetRomMemoryFlags(void);

#endif /* TRIMMING_H */
/// @}

