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
 *
 * @brief FLASH write support and initialization functions library
 * @details
 * FLASH driver. Used to manage the dedicated FLASH type.
 *
 * @{
 */

#ifndef FLASH_DEFINES_H
#define FLASH_DEFINES_H

#include "compiler_abstraction.h"
#include "io.h"
#include "eeprom_map.h"
#include "flash_cfg.h"
#include "sys_tools.h"

/**
 * Debug output for the Flash.
 * Use general project settings.
 */
#define FL_DEBUG_OUTPUT(info)   DEBUG_OUTPUT(info)


#ifdef FL_USE_DIRECT_SETTINGS

#ifndef EE_FL_ER_WR_TIMING_VALUE
/* Use default value to initialize the FLASH. This definition should exist outside the library */
#warning "EE_FL_ER_WR_TIMING_VALUE is not set. Using default value"
#define EE_FL_ER_WR_TIMING_VALUE                          0x63B1u
#endif

#ifndef EE_FL_10NS_50NS_VALUE
/* Use default value to initialize the FLASH. This definition should exist outside the library */
#warning "EE_FL_10NS_50NS_VALUE is not set. Using default value"
#define EE_FL_10NS_50NS_VALUE                            0x0442u
#endif

#ifndef EE_FL_WAIT_STATES_VALUE
/* Use default value to initialize the FLASH. This definition should exist outside the library */
#warning "EE_FL_WAIT_STATES_VALUE is not set. Using default value"
#define EE_FL_WAIT_STATES_VALUE                          0x0007u
#endif



#else /* FL_USE_DIRECT_SETTINGS */

/* FLASH Timing parameters, in microseconds: ( 100 us step for write-time, 10us step for erase-time )*/
#ifndef FL_WRITE_TIME
#warning "FL_WRITE_TIME is not set. Using default value"
#define FL_WRITE_TIME                                  6000u
#endif

#ifndef FL_ERASE_TIME
#warning "FL_ERASE_TIME is not set. Using default value"
#define FL_ERASE_TIME                                  20000u
#endif

#ifndef FL_WAIT_STATES /* FL_WAIT_STATES - FLASH Wait-states to read, -1 (value 0..7)*/
#warning "FL_WAIT_STATES is not set. Using default value"
#define FL_WAIT_STATES                                 (7u)
#endif

#ifndef FL_DED_RETRIES /* FL_DED_RETRIES - FLASH Double-error detected retries, -1 (value 0..7)*/
#warning "FL_DED_RETRIES is not set. Using default value"
#define FL_DED_RETRIES                                 (7u)
#endif

/*
 *  FLASH_TIME_50US is value to fullfill the 50uS time in 10*uS. Default is 2.
 */
#ifndef FLASH_TIME_50US
#warning "FLASH_TIME_50US is not set. Using default value"
#define FLASH_TIME_50US                                (2u)
#endif

/*
 *  FLASH_TIME_50NS is value to fullfill the 50nS time in nS*MCU_CLK. Default is 4
 */
#ifndef FLASH_TIME_50NS
#warning "FLASH_TIME_50NS is not set. Using default value"
#define FLASH_TIME_50NS                                ((uint16_t)4u)
#endif

/*
 *  FLASH_TIME_10NS is value to fullfill the 20nS time in nS*MCU_CLK. Default is 1
 */
#ifndef FLASH_TIME_10NS
#warning "FLASH_TIME_10NS is not set. Using default value"
#define FLASH_TIME_10NS                                ((uint16_t)1u)
#endif

#ifndef FLASH_HALT_BEHAVIOR
#warning "FLASH_HALT_BEHAVIOR is not set. Using default value"
#define FLASH_HALT_BEHAVIOR                             (0u) /* 0 - Stay active when application is halted. 1 - Go in STBY mode */
#endif

#ifndef FLASH_BYPASS_QUEUE
#warning "FLASH_BYPASS_QUEUE is not set. Using default value"
#define FLASH_BYPASS_QUEUE                              (0u) /* 0 - Queue is used */
#endif

#ifndef FLASH_DATA_CORRUPTED
#warning "FLASH_DATA_CORRUPTED is not set. Using default value"
#define FLASH_DATA_CORRUPTED                            ((uint16_t)1u)   /* 2 errors in data or 1 error in address detected. Writing '1' clears the flag. */
#endif

#ifndef FLASH_SBE
#warning "FLASH_SBE is not set. Using default value"
#define FLASH_SBE                                      ((uint16_t)1u)   /* Single bit Error Flag. Writing '1' clears the flag. */
#endif

#ifndef FLASH_DED_RETRY
#warning "FLASH_DED_RETRY is not set. Using default value"
#define FLASH_DED_RETRY                         7            /* [6:8] Retries number when Double Error's detected. Value [0..7] */
#endif

#ifndef FLASH_MFA_WAIT_STATES
#warning "FLASH_MFA_WAIT_STATES is not set. Using default value"
#define FLASH_MFA_WAIT_STATES                   7            /* [3:5] Wait states number for MFA (main FLASH array). [0..7] */
#endif

#ifndef FLASH_PL_WAIT_STATES
#warning "FLASH_PL_WAIT_STATES is not set. Using default value"
#define FLASH_PL_WAIT_STATES                    7            /* [0:2] Wait states number for PL (page latches). [0..7] */
#endif



/* Driver settings constraints' checks */
#if (FL_WRITE_TIME < 100) || (FL_WRITE_TIME > 12700)
#error FL_WRITE_TIME cannot be less than 100uS and more then 12700uS!!!
#endif

#if (FL_ERASE_TIME < 100) || (FL_ERASE_TIME > 25500)
#error FL_ERASE_TIME cannot be less than 10uS and more then 25500uS!!!
#endif

#if (FL_DED_RETRIES > 7)
#error FL_DED_RETRIES cannot be more than 7!!!
#endif

#if (FL_WAIT_STATES > 7)
#error FL_WAIT_STATES cannot be more than 7!!!
#endif


/* Internal declarations */
#define FLASH_WRITE_TIME_INT  ( (uint16_t)( FL_WRITE_TIME / 100u - 1 ) )
#define FLASH_ERASE_TIME_INT  ( (uint16_t)( FL_ERASE_TIME / 100u - 1 ) )


#endif /* FL_USE_DIRECT_SETTINGS */

/* FLASH data cell definitions */
#define FLASH_32K_SIZE_IN_BYTES  (0x8000u)
#define FLASH_64K_SIZE_IN_BYTES  (0x10000UL)
/* FLASH buffer size, in words, should be 16bit size!!!          */
/* Note: FLASH_DATA_SIZE should be 1, 2, 4, 8 ... word's size!!! */
#define FLASH_DATA_SIZE ((uint16_t)4u)
#define FLASH_DATA_SIZE_MSK (~((FLASH_DATA_SIZE << 1) - 1))

/** Represents the FLASH's page count */
#define FL_PAGE_COUNT       ((uint16_t)(MEM_FLASH_SIZE / MEM_FLASH_PAGE_SIZE))
/** Represent the page count for a 32KB section of flash */
#define FL_PAGE_COUNT_32K   (FLASH_32K_SIZE_IN_BYTES / (uint16_t)MEM_FLASH_PAGE_SIZE)
/** Represents the FLASH CS's page count */
#define FL_CS_PAGE_NUMBER   (FL_PAGE_COUNT + 0u)


/** FLASH Control commands */
/** Request the FLASH to switch in normal, active state */
#define FLASH_CMD_NORMAL                                                 0x0000u

/** Request the FLASH to switch in the STDBY mode */
#define FLASH_CMD_STDBY                                                  0x0001u

/** Request the FLASH to Power ON */
#define FLASH_CMD_PWR_ON                                                 0x0006u

/** Request the FLASH to switch in the configurable mode */
#define FLASH_CMD_CONFIGURATE                                            0x0007u


/** Predefined KEY for FLASH erase operations */
#define FLASH_CMD_ERASE_KEY                                              0xA5E3u

/** Predefined KEY for FLASH write operations */
#define FLASH_CMD_WRITE_KEY                                              0xA5E4u

/** Predefined KEY for unlocking the write and erase operations */
#define FLASH_CMD_UNLOCK_KEY                                             0x5648u


/** FLASH Control command statuses (LSB nibble) */
/** FLASH is in NORMAL mode */
#define FLASH_STATUS_NORMAL                                              0x0u

/** FLASH is in STDBY mode */
#define FLASH_STATUS_STDBY                                               0x1u

/** FLASH is in DEEP POWER DOWN mode */
#define FLASH_STATUS_DEEP_PWD_DWN                                        0x2u

/** FLASH is in SECTOR_ERASE mode */
#define FLASH_STATUS_SECTOR_ERASE                                        0x3u

/** FLASH is in PAGE_PROGRAM mode */
#define FLASH_STATUS_PAGE_PROGRAM                                        0x4u

/** FLASH is in CONFIGURABLE mode */
#define FLASH_STATUS_CONFIGURABLE                                        0x7u

/** FLASH is in POWER_ON mode */
#define FLASH_STATUS_POWER_ON                                            0x6u

/** FLASH is in POWER_OFF mode */
#define FLASH_STATUS_POWER_OFF                                           0xFu

/** LSB nibble mask to get the status */
#define FLASH_STATUS_MASK                                                0x000Fu


#ifdef HAS_HW_FLASH_VCTAT_CTRL
#if defined(HAS_MEM_GT_64KB) && defined(HAS_HW_FLASH_21)
/** The mask defines the VCTAT control bits placement in the EE configuration for the trimmings.
 * It uses the not-used filed in 1_US trimming IO-port (IO_EEPROM_FLASH__T_1US [4:0]) */
#define EE_FL_1US_VALUE_VCTAT_MASK                                       (0x0F00U)
/** This mask defines the same bits placement inside the FL_R3 register */
#define FLASH_VCTAT_CTRL_MASK                                            (0xF000U)
#else
/** The mask defines the VCTAT control bits placement in the EE configuration for the trimmings.
 * It uses the not-used filed in 1_US trimming IO-port (IO_EEPROM_FLASH__T_1US [4:0]) */
#define EE_FL_1US_VALUE_VCTAT_MASK                                       (0x0E00U)
/** This mask defines the same bits placement inside the FL_R3 register */
#define FLASH_VCTAT_CTRL_MASK                                            (0xE000U)
#endif

/** This value specifies how to get the data from EE and set it to certain position in 16bit field for IO-PORT */
#define FLASH_VCTAT_CTRL                                                 ((EE_FL_1US_VALUE & \
                                                                           EE_FL_1US_VALUE_VCTAT_MASK) << 4)
#endif /* HAS_HW_FLASH_VCTAT_CTRL */

#ifndef FLASH_PREFIX
/** functions prefix which manages the library's functions memory mapping */
#define FLASH_PREFIX
#endif

/** The dummy pointer which is used to run the FLASH state-machine in write-mode within the erase sequence */
#define FLASH_NON_NULL_DUMMY        ((uint16_t*)1)

/**< ERA_MODE_BIT specifies the erase mode bits field in the FLASH_EEPROM configuration register */
#define ERA_MODE_BIT        8
/**< ERA_MODE_MASK is calculated mask for setting/masking the erase-mode */
#define ERA_MODE_MASK       ((uint16_t)0x03 << ERA_MODE_BIT)

/** FLASH write operation type */
typedef enum {
    FL_WNV_PAE = 0,              /**< Write the page */
    FL_WNV_SEC,                /**< Write the sector */
    FL_WNV_FL,                 /**< Write flash MA or CS */
    FL_WNV_FULL,               /**< Write flash MA and CS */
    FL_WNV_EVEN_PAE_FL,        /**< Write even page MA or CS */
    FL_WNV_ODD_PAE_FL,         /**< Write odd page MA or CS */
    FL_WNV_EVEN_PAE_FULL,      /**< Write even page MA and CS */
    FL_WNV_ODD_PAE_FULL,       /**< Write odd page MA and CS */
} fl_wnv_mode_t;

/**< WNV_MODE_BIT specifies the write mode bits field in the FLASH_EEPROM configuration register */
#define WNV_MODE_BIT        12
/**< WNV_MODE_MASK is calculated mask for setting/masking the write-mode */
#define WNV_MODE_MASK       ((uint16_t)0x07 << WNV_MODE_BIT)

/** the current state of the write-process type */
typedef enum {
    flIdle = 0,                 /**< Inactive state */
    flActive,                   /**< Write state (in write operation, or the latest phase for erase operation */

#ifdef HAS_ROM_SPLIT_FLASH_PREWRITE
    flPreWrite1,                /**< Prewrite phase 1 for the erase operation */
    flPreWrite2,                /**< Prewrite phase 2 for the erase operation */
#else
    flPreWrite,                 /**< Prewrite for the erase operation */
#endif
    flFinished = 0xFD,          /**< used to indicate the settings for the write-operation is finished */
    flError = 0xFE,             /**< used to indicate the settings for the write-operation are incorrect */
    flBusy = 0xFF,              /**< Reserved for the common 'busy' state */
}
FLASH_Result_t;

#ifdef HAS_ROM_SPLIT_FLASH_PREWRITE
#define flPreWrite flPreWrite1
#endif

/** Specifies the FLASH write-keys placement structure and the flash erase phase-2 reference pattern */
typedef struct {
    uint16_t Unlock_key;     /**< Unlock_key, Needs to be 0x5648 to unlock the FLASH write-access */
    uint16_t Write_key;      /**< Write_key, Needs to be 0xA5E4 to trigger the FLASH WRITE operation */
    uint16_t Erase_key;      /**< Erase_key, Needs to be 0xA5E3 to trigger the FLASH ERASE operation */
    uint16_t EepromWriteKey; /**< EEPROM WRITE_KEY (0x0007) is stored here, indirectly accessed by pointer aritmatic (Melibu BL) */
    uint16_t bllrPattern;    /**< Flash erase phase-2 reference pattern Either 0xAAAA or 0xAA4A or ... */
} fl_ctrl_keys_t;

/** Specifies the type for pointer to the keys location */
typedef fl_ctrl_keys_t* fl_ctrl_func_p;

/** Defines the data area for the non-blocking FLASH access. An additional func-address for FLASH CONTROL register needed.
 *
 * Used by the FLASH_write function to write or erase the FLASH content.
 */
typedef struct {
    fl_ctrl_func_p func;            /**< the pointer to the FLASH keys' area. */
    uint16_t page_number;           /**< Specifies the page_number or sector to write/erase. If page_number==PAGES_COUNT - the FLASH CS page is used. */
    uint16_t* data;                 /**< The pointer of the buffer to write. If data is NULL - Erase operation will be chosen */
    FLASH_Result_t write_state;     /**< The current write state */
    fl_wnv_mode_t write_mode;       /**< The mode selection for the operation */
#if defined(HAS_HW_FLASH_15) || defined(HAS_HW_FLASH_21)
    uint16_t iteration;            /**< Specifies the number of iterations of certain mode */
#endif /* HAS_HW_FLASH_15 || HAS_HW_FLASH_21 */
} FLASH_WriteStruct_t;

/** Declaration to be satisfied in the project which provides an access to the FLASH control registers, latches and reference registers */
extern volatile FLASH_control_t FLASH_Control;
/** Declaration to be satisfied in the project which provides the flash_state engine for the write operation support */
extern volatile FLASH_WriteStruct_t* flash_state;

/** FLASH_getStatus returns the status of the FLASH HW */
STATIC INLINE uint16_t FLASH_getStatus(void);
/** FLASH_WAIT_STATUS waits for the FLASH's status specified
 * @param[in] status the status to expect
 * @note the function performs the infinite loop. Therefore AWD acknowledge should be handled accordingly
 */
STATIC INLINE void FLASH_WAIT_STATUS(uint16_t status);
/** FLASH_WAIT_NOT_STATUS waits for the FLASH's status will be changed to any than specified
 * @param[in] status the status to expect it changed
 * @note the function performs the infinite loop. Therefore AWD acknowledge should be handled accordingly
 */
STATIC INLINE void FLASH_WAIT_NOT_STATUS(uint16_t status);
STATIC INLINE void FLASH_SetTimings(void);
/** FLASH_ClearDEDflag clears the DATA_CORRUPTED flag */
STATIC INLINE void FLASH_ClearDEDflag(void);
/** FLASH_ClearSECflag clears the FLASH single bit error flag */
STATIC INLINE void FLASH_ClearSECflag(void);

#ifndef UNITTEST
#include "flash_defines_inline_impl.h"
#endif /* UNITTEST */

/// @}
#endif /* FLASH_DEFINES_H */

