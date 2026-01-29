/**
 * @file
 * @brief FLASH write support and initialization functions library
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
 * @ingroup flash
 * @addtogroup flash Flash Library
 *
 * @brief FLASH write support and initialization functions library
 * @details
 * FLASH driver. Used to manage the dedicated FLASH type.
 *
 * @{
 */

#ifndef FLASH_DRV_H
#define FLASH_DRV_H

#include "flash_defines.h"
#include "memory.h"

void FLASH_TurnOn(void); /* Enables the FLASH shell and waits a state */
void FLASH_Init(void);   /* Initiates the FLASH with timings  */
void FLASH_Tick(void);

#if defined(HAS_FL_EE_WRITE_FUNC) || (FL_EE_HAS_WRITE_FUNC != 0)
FLASH_Result_t FLASH_Write( void );
void FLASH_EraseFlash(void);
void FLASH_EraseFlash_CS(void);
#endif /* HAS_FL_EE_WRITE_FUNC */

void WaitForEEandFL(const volatile FLASH_WriteStruct_t* fl_state);

#define FLASH_highMemRead(address)    memoryReadWord(address) __attribute__ ((deprecated("Renamed to memoryReadWord")))
extern void FLASH_SetR3(uint16_t NewValue);

#ifdef FLASH_USE_INTERRUPTS
extern void FLASH_eventHandler(void);
#endif

/// @}
#endif

