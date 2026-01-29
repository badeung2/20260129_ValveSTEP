/**
 * @file
 * @brief EEPROM support library functions
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
 * @ingroup eeprom
 * @addtogroup eeprom EEPROM
 *
 * @brief EEPROM support library functions
 * @details
 * EEPROM driver. Used to manage the dedicated EEPROM type.
 *
 * @{
 */

#ifndef EEPROM_DRV_H
#define EEPROM_DRV_H

#include <stddef.h>
#include <stdint.h>

#include "compiler_abstraction.h"
#include "sys_tools.h"
#include "eeprom_map.h"
#include "flash_cfg.h"
#include "io.h"

#ifdef DBG_USE_CHAR_OUTPUT
#include "dbg_tools.h"
#endif

#ifdef EEP_USE_DIRECT_SETTINGS

#ifndef EE_EEP_WR_TIMING_VALUE
/* Use default value to initialize the EEPROM. This definition should exist outside the library */
#warning "EE_EEP_WR_TIMING_VALUE is not set. Using default value"
#define EE_EEP_WR_TIMING_VALUE                          0x3B1Du
#endif

#ifndef EE_EEP_ER_TIMING_VALUE
/* Use default value to initialize the EEPROM. This definition should exist outside the library */
#warning "EE_EEP_ER_TIMING_VALUE is not set. Using default value"
#define EE_EEP_ER_TIMING_VALUE                          0x3B1Du
#endif

#ifndef EE_EEP_PROG_CYCLE_VALUE
/* Use default value to initialize the EEPROM. This definition should exist outside the library */
#warning "EE_EEP_PROG_CYCLE_VALUE is not set. Using default value"
#define EE_EEP_PROG_CYCLE_VALUE                            0x000Fu
#endif

#ifndef EE_EEP_READ_TIMING_VALUE
/* Use default value to initialize the EEPROM. This definition should exist outside the library */
#warning "EE_EEP_READ_TIMING_VALUE is not set. Using default value"
#define EE_EEP_READ_TIMING_VALUE                            0x000Fu
#endif


#ifndef EE_EEP_WAIT_STATES_VALUE
/* Use default value to initialize the EEPROM. This definition should exist outside the library */
#warning "EE_EEP_WAIT_STATES_VALUE is not set. Using default value"
#define EE_EEP_WAIT_STATES_VALUE                          0x0F00u
#endif


#else /* EEP_USE_DIRECT_SETTINGS */
/* EEPROM Timing parameters, in microseconds: ( 100 us step for write-time, 10us step for erase-time )*/
#ifndef EEP_WRITE_TIME
#warning "EEP_WRITE_TIME is not set. Using default value"
#define EEP_WRITE_TIME                                  6000u
#endif

#ifndef EEP_PREWRITE_TIME
#warning "EEP_PREWRITE_TIME is not set. Using default value"
#define EEP_PREWRITE_TIME                                  300u
#endif

#ifndef EEP_ERASE_TIME
#warning "EEP_ERASE_TIME is not set. Using default value"
#define EEP_ERASE_TIME                                  300u
#endif

#ifndef EEP_PROGRAM_CYCLES /* EEP_PROGRAM_CYCLES - cycles quantity for write, -1 (value 0..31)*/
#warning "EEP_PROGRAM_CYCLES is not set. Using default value"
#define EEP_PROGRAM_CYCLES                              16u
#endif

#ifndef EEP_WAIT_STATES /* EEP_WAIT_STATES - EEPROM Wait-states to read, -1 (value 0..15)*/
#warning "EEP_WAIT_STATES is not set. Using default value"
#define EEP_WAIT_STATES                                 15u
#endif

/* Driver settings constraints' checks */
#if (EEP_WRITE_TIME < 100) || (EEP_WRITE_TIME > 6300)
#error EEPROM_WRITE_TIME cannot be less than 100uS and more then 6300uS!!!
#endif

#if (EEP_PREWRITE_TIME < 100) || (EEP_PREWRITE_TIME > 1270)
#error EEP_PREWRITE_TIME cannot be less than 10uS and more then 1270uS!!!
#endif

#if (EEP_ERASE_TIME < 100) || (EEP_ERASE_TIME > 1270)
#error EEPROM_ERASE_TIME cannot be less than 10uS and more then 1270uS!!!
#endif

#if (EEP_PROGRAM_CYCLES > 31)
#error EEP_PROGRAM_CYCLES cannot be more than 31!!!
#endif

#if (EEP_WAIT_STATES > 15)
#error EEP_WAIT_STATES cannot be more than 15!!!
#endif


/* Internal declarations */
#define EEPROM_PREWRITE_TIME_INT    ( (uint16_t)( EEP_PREWRITE_TIME / 10u - 1 ) )
#define EEPROM_WRITE_TIME_INT       ( (uint16_t)( EEP_WRITE_TIME / 100u - 1 ) )
#define EEPROM_ERASE_TIME_INT       ( (uint16_t)( EEP_ERASE_TIME / 10u - 1 ) )

#endif /* EEP_USE_DIRECT_SETTINGS */

/* EEPROM NVM Mode Register (R1_EE) configuration for EEPROM main or configuration array selection */
#define EE_R1_SEL_MASK      0x0F00u
#define EE_R1_SEL_MA_CFG    0x0100u
#define EE_R1_SEL_CS_CFG    0x0200u

/* EEPROM buffer size, in words, should be 16bit size!!!          */
/* Note: EEPROM_DATA_SIZE should be 1, 2, 4, 8 ... word's size!!! */
#define EEPROM_DATA_SIZE ((uint16_t)4u)
#define EEPROM_DATA_SIZE_MSK (~((EEPROM_DATA_SIZE << 1) - 1u))

/** Number of bytes in a single page of EEPROM */
#define EEPROM_BYTE_SIZE (EEPROM_DATA_SIZE * sizeof(uint16_t))

#define EEPROM_WE_KEY_VALUE 0x07u



/* functions prefix which manages the library's functions memory mapping */
#ifndef EEPROM_PREFIX
#define EEPROM_PREFIX
#endif

extern volatile uint16_t EEPROM_CFG_REGS[3];

/** EEPROM-write blocking write process result */
typedef enum EEPROM_WriteResult_e {
    eepWrOk = 0,        /**< Write is OK */
    eepWrOutOfRange,    /**< Write failed because the page/s to write is/are out of range */
} EEPROM_WriteResult_t;

/** EEPROM-write process current state */
typedef enum {
    eepIdle = 0,          /**< Write is not executed */
    eepBusy = 0xFF,       /**< Write is in active phase */
} EEPROM_Result_t;

/** EEPROM_Write_Mode_t defines the mode for the EEPROM write operation */
typedef enum EEPROM_Write_Mode_e {
    eepFullWrite = 0x00u, /**< performs the complete XFAB flow (pre-write, erase, write steps) */
    eepWriteOnly = 0x01u, /**< performs only write */
    eepPreWrErase = 0x02u,/**< performs pre-write and erase */
    eepEraseOnly = 0x03u, /**< performs only erase */
    eepPreWrite = 0x04u   /**< performs only pre-write */
} EEPROM_Write_Mode_t;

/** The structure for the write EEPROM functions state-machine */
typedef struct {
    uint16_t address;                   /**< The EEPROM's address */
    uint16_t* data64bit;                /**< The data to write */
    uint16_t write_access_key;          /**< Write KEY, should be 0x07 before the write */
    EEPROM_Result_t write_state;        /**< The current write-operation state */
} EEPROM_WriteStruct_t;

#define EEPROM_CRC_Check(offs, size)                nvram_CalcCRC(offs, size)

void EEPROM_TurnOn(void);
void EEPROM_Init(void);
void EEPROM_Configure(void);

/** EEPROM_WriteWord64_non_blocking writes 64 bits (one EEPROM page) into the EEPROM from buffer.
 *
 * Function assumes 64bit address alignment for the EEPROM page address
 *
 * @param[in] address EEPROM address to write to. Should be 64bit aligned
 * @param[in] data64bit pointer to 64bit buffer of data to write.
 * @param[in] write_acces_key KEY to enable the data writing (0x07 should be used)
 *
 * @note: Function assumes that system's in System mode and cannot be interrupted.
 * @note: Function assumes no EEPROM actions is ongoing (nonblocking)
 */
void EEPROM_WriteWord64_non_blocking(const uint16_t address, const uint16_t* data64bit, uint16_t const write_acces_key);

/** EEPROM_WriteWord64_blocking writes 64 bits (one EEPROM page) into the EEPROM from buffer.
 *
 * Function assumes 64bit address alignment for the EEPROM page address
 *
 * @param[in] address EEPROM address to write to. Should be 64bit aligned
 * @param[in] data64bit pointer to 64bit buffer of data to write.
 * @param[in] write_acces_key KEY to enable the data writing (0x07 should be used)
 *
 * @note: Function assumes that system's in System mode and cannot be interrupted.
 * @note: Function blocks the application code to prepare-erase-write operation.
 */
void EEPROM_WriteWord64_blocking(const uint16_t address, uint16_t* data64bit, uint16_t const write_acces_key);
EEPROM_Result_t EEPROM_WriteWord64_nonblocking(EEPROM_WriteStruct_t* writeData );

/* "Common" (universal) implementation takes too much size. Should be used from the FW library.
 * BL needs to use the 64bit alignment.
 */
void EEPROM_WriteData(uint16_t address, const uint16_t* data, uint8_t size16, uint16_t write_access_key);

/** EEPROM_WritePatchArea allows to write the data into EEPROM Patch area address with a size that fits in it
 *
 * The function performs the read-modify-write sequence for the head and tail of the data incoming to allow using the write operations for the EEPROM
 * in any combination of sizes and offsets (16-bit)
 *
 * @param[in] address Patch area EEPROM address to write to. Should be 16bit aligned
 * @param[in] data pointer data to write.
 * @param[in] size16 size of the data to write, in 16bit words
 * @param[in] write_access_key KEY to enable the data writing (0x07 should be used)
 *
 * @note: Function assumes that system's in System mode and cannot be interrupted.
 * @note: Function blocks the application code to prepare-erase-write operation.
 * @warning: The data is not checked after write. Please use EEPROM_isCorrupted
 * in conjunction to see if there was a data write error.
 */
EEPROM_PREFIX EEPROM_WriteResult_t EEPROM_WritePatchArea( uint16_t address,
                                                          const uint16_t* data,
                                                          uint8_t size16,
                                                          uint16_t write_access_key );

/** EEPROM_WriteUserArea allows to write the data into EEPROM User area address with a size that fits in it
 *
 * The function performs the read-modify-write sequence for the head and tail of the data incoming to allow using the write operations for the EEPROM
 * in any combination of sizes and offsets (16-bit)
 *
 * @param[in] address Patch area EEPROM address to write to. Should be 16bit aligned
 * @param[in] data pointer data to write.
 * @param[in] size16 size of the data to write, in 16bit words
 * @param[in] write_access_key KEY to enable the data writing (0x07 should be used)
 *
 * @note: Function assumes that system's in System mode and cannot be interrupted.
 * @note: Function blocks the application code to prepare-erase-write operation.
 * @warning: The data is not checked after write. Please use EEPROM_isCorrupted
 * in conjunction to see if there was a data write error.
 */
EEPROM_PREFIX EEPROM_WriteResult_t EEPROM_WriteUserArea( uint16_t address,
                                                         const uint16_t* data,
                                                         uint8_t size16,
                                                         uint16_t write_access_key );

/** EEPROM_WriteBytes allows to write data into the EEPROM
 *
 * @param[in]  address  Start address of the to be written EEPROM area.
 * @param[in]  data  Pointer to the data to be written.
 * @param[in]  size  Length of the data to be written.
 * @param[in]  write_access_key  KEY to enable the data writing (0x07 should be used)
 */
void EEPROM_WriteBytes(uint16_t address, const uint8_t* data, size_t size, uint16_t write_access_key);

/** EEPROM_isCorrupted Checks the corruption of the eeprom block (64bits)
 *
 * @param[in] address of the eeprom register to check. The full eeprom block (64bits) will be check for corruption
 *
 * @retval true the eeprom block is corrupted
 * @retval false the eeprom block is ok
 */
EEPROM_PREFIX bool EEPROM_isCorrupted(uint16_t* address);

/** Erases EE_SBE (Single bit error) and EE_CORRUPTED flags
 *
 *  Function's used as a common pattern before running any of EEPROM tests
 */
STATIC INLINE void EEPROM_ClearErrorFlags(void);

/** Gets error EE_CORRUPTED flag
 *
 *  @retval true EEPROM block (64b) is corrupted
 *  @retval false EEPROM block (64b) is OK
 */
STATIC INLINE bool EEPROM_GetErrorFlags(void);

/** Copies the timings for the EEPROM for write/erase/prog_cycle operations from the specified EEPROM settings
 *
 * Function is called when the EEPROM was inited and configured. The configured state protects some settings for the EEPROM.
 * Leaves the EEPROM wait_states
 */
STATIC INLINE void EEPROM_SetTimings(void);

/** Copies the EE_R1, EE_R2 registers for the EEPROM from the specified EEPROM settings */
STATIC INLINE void EEPROM_SetRxRegisters(void);

/** Changes the EEPROM's mode to the configured state
 *
 * The function is called when the EEPROM was inited and configured. The configured state protects some settings for the EEPROM
 */
STATIC INLINE void EEPROM_SetConfigured(void);

/** getter for EEPROM status "BUSY"
 *
 * @retval false EEPROM is idle
 * @retval true  EEPROM is busy
 */
STATIC INLINE bool EEPROM_getEEBUSY(void);

/** Checks if the given address and the size after that address are all included in the EEPROM User area
 *
 * @retval true  Given address and size are in the EEPROM User area
 * @retval false Given address and size are NOT in the EEPROM User area
 */
STATIC INLINE bool EEPROM_isUserArea(uint16_t address, uint8_t size16);

/** Checks if the given address and the size after that address are all included in the EEPROM Patch area
 *
 * @retval true  Given address and size are in the EEPROM Patch area
 * @retval false Given address and size are NOT in the EEPROM Patch area
 */
STATIC INLINE bool EEPROM_isPatchArea(uint16_t address, uint8_t size16);

#ifndef UNITTEST
#include "eeprom_drv_inline_impl.h"
#endif /* UNITTEST */

/// @}
#endif /* EEPROM_DRV_H */
