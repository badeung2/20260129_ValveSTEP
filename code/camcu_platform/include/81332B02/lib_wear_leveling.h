/**
 * @file
 * @brief Wear-Leveling library
 * @internal
 *
 * @copyright (C) 2015-2019 Melexis N.V.
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
 * @ingroup lib_wear_leveling
 * @addtogroup lib_wear_leveling Wear Leveling
 *
 * @brief Wear-Leveling library
 * @details
 * Wear-Leveling support library
 * @{
 */

#ifndef LIB_WEAR_LEVELING_H
#define LIB_WEAR_LEVELING_H

#include <errno.h>
#include <stdint.h>
#include "eeprom_drv.h"

/** Wear leveling write process result */
typedef enum WL_EE_WriteResult_e {
    WL_EE_WRITE_OK = 0,                 /**< Write was successful */
    WL_EE_WRITE_BLOCK_ID_OUT_OF_RANGE,  /**< Write failed because the block id is too high */
    WL_EE_WRITE_HIGH_TEMPERATURE_ERROR, /**< Write failed because the temperature is too high */
    WL_EE_WRITE_DATA_NULL = -EINVAL     /**< Write failed because the data pointer is NULL */
} WL_EE_WriteResult_t;

/** Wear leveling read process result */
typedef enum WL_EE_ReadResult_e {
    WL_EE_READ_OK = 0,                /**< Read was successful */
    WL_EE_READ_EEPROM_ERROR,          /**< Read failed because an EEPROM error occurred */
    WL_EE_READ_BLOCK_ID_OUT_OF_RANGE, /**< Read failed because the block id is too high */
    WL_EE_READ_DATA_NULL = -EINVAL    /**< Read failed because the data pointer is NULL */
} WL_EE_ReadResult_t;

/** Wear leveling config process result */
typedef enum WL_EE_ConfigResult_e {
    WL_EE_CONFIG_OK = 0,                        /**< Configuring was successful */
    WL_EE_CONFIG_EEPROM_ERROR,                  /**< Configuring failed because an EEPROM error occurred */
    WL_EE_CONFIG_NUMBER_OF_BLOCKS_OUT_OF_RANGE, /**< Configuring failed because the number of blocks requested is too high */
    WL_EE_CONFIG_CYCLE_MULTIPLIER_OUT_OF_RANGE, /**< Configuring failed because the cycle multiplier is out of range */
    WL_EE_CONFIG_NUMBER_OF_PAGES_OUT_OF_RANGE,  /**< Configuring failed because the cycle multiplier or accel_factor is too high */
    WL_EE_CONFIG_START_ADDRESS_NULL = -EINVAL,  /**< Configuring failed because the start_address pointer is NULL */
    WL_EE_CONFIG_GET_TEMP_NULL                  /**< Configuring failed because the get_temp function pointer is NULL */
} WL_EE_ConfigResult_t;

/** WL_EE_WearFunction_t stores wear function variables
 * \f[ wear = \frac{offset_{lowest\,upper\,temperature}}{temperature*gain+offset} \f]
 */
typedef struct WL_EE_WearFunction_s {
    int16_t upper_temp; /**< The upper limit temperature for the function */
    int16_t gain;       /**< The gain of the function */
    int32_t offset;     /**< The offset of the function */
} WL_EE_WearFunction_t;

#define WL_EE_NUMBER_OF_WEAR_FUNCTIONS  (3u)
#define WL_EE_COUNTER_SIZE   20u
#define WL_EE_LAST_PAGE_SIZE 4u
#define WL_EE_BLOCK_ID_SIZE  4u
#define WL_EE_PAGE_ID_SIZE   4u

/** WL_EE_Page_t stores all the wear level variables */
typedef struct __attribute__ ((packed)) WL_EE_page_s {
    uint32_t payload;
    uint32_t counter: WL_EE_COUNTER_SIZE;
    uint8_t last_written_page: WL_EE_LAST_PAGE_SIZE;
    uint8_t block_id: WL_EE_BLOCK_ID_SIZE;
    uint8_t page_id: WL_EE_PAGE_ID_SIZE;
} __attribute__ ((aligned (8))) WL_EE_Page_t;

/** WL_EE_Config_t stores all the wear level variables */
typedef struct WL_EE_Config_s {
    uint8_t nb_of_blocks;        /**< Number of 32-bit wear-leveled blocks accessible (R/W) */
    uint8_t nb_of_pages;         /**< Number of pages inside each block */
    WL_EE_Page_t* start_address; /**< Start of the physical EEPROM address where the wear leveled data will be allocated */
    int16_t (* get_temp)(void);  /**< Function poiter to a function that returns the EEPROM die temperature in degrees celsius */
} WL_EE_Config_t;

/**
 * WL_EE_ConfigWearFunctions configures the wear functions in EEPROM
 *
 * @param[in] start_address Start of the physical EEPROM address where the wear leveled data will be allocated.
 *     This address must be 64 bit aligned inside the EEPROM.
 * @param[in] wear_functions This is an array of 3 wear functions as defined in ::WL_EE_WearFunction_s
 * @return Each return value is described in ::EEPROM_WriteResult_e
 *
 * @note: This functions only needs to be called once on the chip to set the EEPROM to the correct values
 */
EEPROM_WriteResult_t WL_EE_ConfigWearFunctions(WL_EE_Page_t* start_address, const WL_EE_WearFunction_t* wear_functions);

/**
 * WL_EE_Config configures all wear leveling blocks and checks for any possible errors.
 *
 * @param[in] nb_of_blocks Number of 32-bit wear-leveled blocks accessible (R/W).
 *     It must be less  or equal than WL_EE_MAX_NUMBER_OF_BLOCKS.
 * @param[in] accel_factor Acceleration factor given by the mission profile
 * @param[in] cycles_multiplier Expected number of write cycles over lifetime multiplicator
 * @param[in] start_address Start of the physical EEPROM address where the wear leveled data will be allocated.
 *     This address must be 64 bit aligned inside the EEPROM.
 * @param[in] get_temp Function poiter to a function that returns the EEPROM die temperature in degrees celsius
 * @return Each return value is described in ::WL_EE_ConfigResult_e
 */
WL_EE_ConfigResult_t WL_EE_Config(uint8_t nb_of_blocks,
                                  uint8_t accel_factor,
                                  uint8_t cycles_multiplier,
                                  WL_EE_Page_t* start_address,
                                  int16_t (* get_temp)(void));

/**
 * WL_EE_Write writes the 32bit data into the wear leveling block.
 *
 * @param[in] block_id bock id that will be written
 * @param[in] data pointer to the 32bit buffer of data that will be written to the correct page
 * @return Each return value is described in ::WL_EE_WriteResult_e
 */
WL_EE_WriteResult_t WL_EE_Write(uint8_t block_id, const uint32_t* data);

/**
 * WL_EE_Read reads the 32bit data from the wear leveling block.
 *
 * @param[in] block_id bock id that will be read
 * @param[out] data pointer to the 32bit buffer of data that will be written with the value of the wear leveling block
 * @return Each return value is described in ::WL_EE_ReadResult_e
 */
WL_EE_ReadResult_t WL_EE_Read(uint8_t block_id, uint32_t* data);

/**
 * WL_EE_GetBlockCycles retrieves the written cycles of a block
 *
 * @param[in] block_id bock id where the counters will be read
 * @return uint32_t The number of written cycles in block, when the block_id is larger then the number of blocks
 *     configured it will return 0
 */
uint32_t WL_EE_GetBlockCycles(uint8_t block_id);

#endif /* LIB_WEAR_LEVELING_H */
/// @}
