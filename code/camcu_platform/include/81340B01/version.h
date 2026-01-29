/**
 * @file
 * @brief Version information for each project
 * @internal
 *
 * @copyright (C) 2017 Melexis N.V.
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
 * @ingroup version
 * @addtogroup version Version
 *
 * @brief version info
 * @details
 * @{
 */

#ifndef VERSION_H_
#define VERSION_H_

#ifndef __ASSEMBLER__
#include <stdint.h>
#include "eeprom_map.h"

/** Special unique pattern to define the product's version and HW revision.*/
#define PRODUCT_VERSION_16   EE_CHIP_ID_3
/** 32bit value uses only 16 LSB for the PRODUCT VERSION */
#define PRODUCT_VERSION_32   ((uint32_t)PRODUCT_VERSION_16)

/** Flash FW PATTERM used to code the expected FLASH FW KEY */
#define FLASH_FW_PATTERN     ((uint16_t)0xA55Au)
/** Flash UDS PATTERN used to code the expected FLASH UDS KEY */
#define FLASH_UDS_PATTERN    ((uint16_t)0x6336u)

#else /* __ASSEMBLER__ */

#endif /* __ASSEMBLER__ */

/** Version data placement structure */
struct VERSION_Info_s {
    uint32_t gitHash;           /**< holds the Git hash used by the released SWTO */
    uint8_t compileConfig;      /**< Configuration holds the project-specific data */
    uint8_t reserved[3u];       /**< Reserved */
};

extern volatile const struct VERSION_Info_s VERSION_info;

#endif /* VERSION_H_ */
/// @}

