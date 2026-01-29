/**
 * @file
 * @brief Provides the link between the FLASH and EEPROM settings and ports to control.
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
 * @ingroup library
 *
 * @brief Provides the link between the FLASH and EEPROM settings and ports to control.
 * @details
 * @{
 */

#ifndef FLASH_CFG_H
#define FLASH_CFG_H

#include <stdint.h>

/* ---- Flash constants ------------------------------------------ */
#define ML_FLASH_BUFFER_SIZE_IN_WORDS   64          /** FLASH buffer size */

/*
 * Validate ML_FLASH_BUFFER_SIZE_IN_WORDS
 */
#if (ML_FLASH_BUFFER_SIZE_IN_WORDS < 4)
#error \
    "ML_FLASH_BUFFER_SIZE_IN_WORDS size must be at least 4 bytes (normally it's defined by HW and equal to 128 bytes)"
#endif

#if (ML_FLASH_BUFFER_SIZE_IN_WORDS & (ML_FLASH_BUFFER_SIZE_IN_WORDS - 1))
#error "ML_FLASH_BUFFER_SIZE_IN_WORDS size must be a power of 2 (normally it's defined by HW and equal to 128 bytes)"
#endif

/** Margin mode register (R2) bits structure */
typedef union {
    struct {
        uint16_t SW_VMG  : 5;       /**< VMG voltage Flash and EEPROM */
        uint16_t SW_IREF : 6;       /**< set IREF for margin mode */
        uint16_t SW_IREF_DOUBLE : 1;/**< IREF doubled mode */
        uint16_t T_MEAS  : 4;       /**< Measurement mode */
    } bits;
    uint16_t ctrl;
} R2_t;


/** FLASH control and configuration registers description */
typedef struct {
    uint16_t R0_FL_addr;            /**< FL_R0 register */
    uint16_t R1_FL_nv_mode;         /**< FL_R1 register */
    uint16_t R2_FL_margin_mode;     /**< FL_R2 register */
    uint16_t R3_FL_read_mode;       /**< FL_R3 register */
} FLASH_regs_t;

/** FLASH status registers description */
typedef struct {
    uint16_t S0_FL;                 /**< FL_S0 register */
    uint16_t S1_FL;                 /**< FL_S1 register */
    uint16_t S2_FL;                 /**< FL_S2 register */
    uint16_t S3_FL;                 /**< FL_S3 register */
} FLASH_status_regs_t;

/** FLASH module (not shell!!!) latches and registers map */
typedef struct {
    uint16_t latches[0x40];         /**< 128 bytes - one-page latches, BLLA */
    uint16_t latch_refs[0x04];      /**< 8 bytes of latch references, BLLR */
    FLASH_regs_t regs;              /**< Rx_FL registers */
    FLASH_status_regs_t statuses;   /**< R/O status registers */
    uint16_t Config_Sector[0x80];   /**< FLASH CS (configuration sector) */
} FLASH_control_t;

/* Project's default settings used to initialize the FLASH and EEPROM timings */

#define EE_FL_WAIT_STATES_VALUE                0x0007u          /** FLASH WAIT_STATES value used as a 'worst-case' */
#define EE_FL_10NS_50NS_VALUE                  EE_TIM_PORT1     /** Mapping the value used for the FLASH nS timings configuration */
#define EE_FL_1US_VALUE                        EE_TIM_PORT2     /** Mapping the value used for the FLASH uS timings configuration */
#define EE_FL_ER_WR_TIMING_VALUE               EE_FL_ER_WR_FL   /** FLASH Write/Erase timings mapping */


/* NOTE: FFlash erase time MUST BE in range  19...21 ms !!!
 * Flash program time MUST BE in range  4,75 .. 5,25 ms
 * Flash write and erase time is checked in the digital simulator
 */
#define EE_EEP_WR_TIMING_VALUE                 EE_FL_WR_EE      /** EEPROM Write timings mapping */
#define EE_EEP_ER_TIMING_VALUE                 EE_FL_ERA_EE     /** EEPROM Erase timings mapping */
#define EE_EEP_PROG_CYCLE_VALUE                EE_FL_PROG_EE    /** EEPROM Programming timings mapping */
#define EE_EEP_READ_TIMING_VALUE               EE_FL_TIM_EE     /** EEPROM Read timings mapping */
#define EE_EEP_WAIT_STATES_VALUE               0x0F00u          /** EEPROM WAIT_STATES value used as a 'worst-case' */


#endif
/// @}

