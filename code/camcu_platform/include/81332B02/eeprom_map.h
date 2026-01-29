/**
 * @file
 * @brief CAMCU EEPROM map description concerning the startup code and the common parts of applications.
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
 * @brief CAMCU EEPROM map description concerning the startup code and the common parts of applications.
 * @details
 * @{
 */

#ifndef EEPROM_MAP_H
#define EEPROM_MAP_H

#include <stdint.h>
#include "memory_map.h"
#include <syslib.h>
#include "lib_miscio.h"
#include "eeprom_parameters.h"

/* PATCHes area definition */
/** The PATCH area start offset */
#define EE_PATCH_AREA_START            (MEM_EEPROM_START + 0x00u) /* Cell 0 in EEPROM map excel sheet */
/** The PATCH version offset */
#define EE_PATCH_VERSION               (*(uint16_t*)(MEM_EEPROM_START + 0x02u)) /* Cell 1 in EEPROM map excel sheet, patch area size in words */
/** The PATCH area size offset */
#define EE_PATCH_AREA_SIZE             ((uint8_t)((*(uint16_t*)(MEM_EEPROM_START + 0x00u)) >> 8))  /* Cell 2 in EEPROM map excel sheet. EEPROM access is 16bit, the addr 0x01 is used */
/** The PATCH area start data to copy offset */
#define EE_PATCH_DATA_TO_COPY          (MEM_EEPROM_START + 0x04u) /* Cell 3 in EEPROM map excel sheet */
/** The PATCH area end offset */
#define EE_PATCH_AREA_END              (MEM_EEPROM_START + 0x40u) /* Cell 32 in EEPROM map excel sheet */

/** Sets the patch data structure in EEPROM */
typedef struct {
    uint16_t address;   /**< patch address has [15:0] address format (addr[19:16]=0). The least bit is used to enable/disable the patch. */
    uint16_t data;      /**< patch instruction */
} PatchContent_t;

/** PATCHes area maximum size in words */
#define EE_PATCH_MAX_AREA_SIZE  0x20u
/** PATCHes quantity */
#ifndef HAS_PATCHES_AND_APPTRIM_IN_FLASH_CS
#define PATCH_COUNT     3u
#endif /* HAS_PATCHES_AND_APPTRIM_IN_FLASH_CS */

/* Adding this since it was changed in 48b338f718c72053a505ab033f1f3e6ce36452c3 */
#ifndef MEM_EEPROM_CS_START
#define MEM_EEPROM_CS_START MEM_EEPROM_START
#endif

#ifndef MEM_EEPROM_APP_TRIM_START
#define MEM_EEPROM_APP_TRIM_START (MEM_EEPROM_START + 0x01B0u)
#endif

#ifdef HAS_EE_WR_AREA_CTRL
/* The data of EEPROM map area is taken from the HW ports, which should be adjusted before use.
 * Adjustment is currently done by function SetEepromRegions() */
#define EE_USER_AREA_START      (MEM_EEPROM_START + \
                                 (IO_GET(EEPROM_FLASH, EE_PHY_ADD_AREA_1_START) * MEM_EEPROM_PAGE_SIZE))
#define EE_USER_AREA_END        MEM_EEPROM_APP_TRIM_START
#define EE_APP_TRIM_AREA_START  (MEM_EEPROM_START + \
                                 (IO_GET(EEPROM_FLASH, EE_PHY_ADD_AREA_2_START) * MEM_EEPROM_PAGE_SIZE))
#define EE_APP_TRIM_AREA_END    EE_TRIM_AREA_START
#else
/* USER area definition */
#define EE_USER_AREA_START      (EE_PATCH_AREA_END)
#define EE_USER_AREA_END        (EE_PATCH_AREA_START + MEM_EEPROM_WR_SIZE) /* MEM_EEPROM_WR_SIZE comprises USER and PATCH areas*/

#define EE_APP_TRIM_AREA_START         (MEM_EEPROM_APP_TRIM_START + 0x000U)
#define EE_APP_TRIM_AREA_END           (MEM_EEPROM_APP_TRIM_START + 0x050U)
#endif /* HAS_EE_WR_AREA_CTRL */

/* TRIMMING VALUES AREA DEFINITION */
#define EE_TRIM_AREA_START             (MEM_EEPROM_CS_START + 0x0000U)
#define EE_TRIM_AREA_SIZE              (0x38u)                                              /* From camcu EEPROM map file 0x236 - 0x200 + 0x2 (counting in bytes) */
#define EE_APP_TRIM_AREA_SIZE          (EE_APP_TRIM_AREA_END - EE_APP_TRIM_AREA_START)      /* -- Calculated size -- */

/* FLASH and EEPROM shell definitions in EEPROM area */

/* Enables the EEPROM-mapped EEPROM memory shell settings */
#define EEP_USE_DIRECT_SETTINGS

/** Holds the project and HW-specific settings being used in the ROM or FLASH */
#define EE_ROM_CONFIG                  (*(RomConfiguration_t*)(MEM_EEPROM_CS_START + 0x001u))

/** For platform (bandgap, bias) */
#define EE_MS_TRIM0_VALUE              (*(uint16_t*)(MEM_EEPROM_CS_START + 0x002u))

/** For platform (vddd, vdda) */
#define EE_MS_TRIM1_VALUE              (*(uint16_t*)(MEM_EEPROM_CS_START + 0x004u))

/** For platform (1MHz RCO (150d), LIN) */
#define EE_MS_TRIM2_VALUE              (*(uint16_t*)(MEM_EEPROM_CS_START + 0x006u))

/** For platform (ADC refs 1+2) */
#define EE_MS_TRIM3_VALUE              (*(uint16_t*)(MEM_EEPROM_CS_START + 0x008u))

/** For platform (ADC ref 3) */
#define EE_MS_TRIM4_VALUE              (*(uint16_t*)(MEM_EEPROM_CS_START + 0x00Au))

/** For platform (second regulator vddio) */
#define EE_MS_TRIM5_VALUE              (*(uint16_t*)(MEM_EEPROM_CS_START + 0x00Cu))

/** For platform (speed1 for fast RCO + EE/Fl wait states - 24/12 MHz (0d530=0x212) */
#define EE_MS_TRIM6_VALUE              (*(uint16_t*)(MEM_EEPROM_CS_START + 0x00Eu))

/** For platform (speed2 for fast RCO + EE/Fl wait states - 28/14 MHz (0d634=0x27A)) */
#define EE_MS_TRIM7_VALUE              (*(uint16_t*)(MEM_EEPROM_CS_START + 0x010u))

/** For platform (speed3 for fast RCO + EE/Fl wait states - 32/16 MHz (0d761=0x2F9)) */
#define EE_MS_TRIM8_VALUE              (*(uint16_t*)(MEM_EEPROM_CS_START + 0x012u))


#ifdef EEP_USE_DIRECT_SETTINGS

/** R2_EE (default: 0x4000) */
#define EE_FL_R2_EE                    (*(volatile uint16_t*)(MEM_EEPROM_CS_START + 0x014u))

/** R1_EE (default: 0x5366) */
#define EE_FL_R1_EE                    (*(volatile uint16_t*)(MEM_EEPROM_CS_START + 0x016u))

#endif /* EEP_USE_DIRECT_SETTINGS */

/* Enables the EEPROM-mapped FLASH memory shell settings */
#define FL_USE_DIRECT_SETTINGS

#ifdef FL_USE_DIRECT_SETTINGS

/** R3_FL (default: 0x6744) */
#define EE_FL_R3_FL                    (*(uint16_t*)(MEM_EEPROM_CS_START + 0x018u))

/** R2_FL (default: 0x0010) */
#define EE_FL_R2_FL                    (*(uint16_t*)(MEM_EEPROM_CS_START + 0x01Au))

/** R1_FL (default: 0x0188) */
#define EE_FL_R1_FL                    (*(uint16_t*)(MEM_EEPROM_CS_START + 0x01Cu))


/** EEPROM read timing port */
#define EE_FL_TIM_EE                   (*(uint16_t*)(MEM_EEPROM_CS_START + 0x01Eu))

/** EEPROM program cycle port */
#define EE_FL_PROG_EE                  (*(uint16_t*)(MEM_EEPROM_CS_START + 0x020u))

/** EEPROM erase timing */
#define EE_FL_ERA_EE                   (*(uint16_t*)(MEM_EEPROM_CS_START + 0x022u))

/** EEPROM write timing */
#define EE_FL_WR_EE                    (*(uint16_t*)(MEM_EEPROM_CS_START + 0x024u))


/** NV_TRIM9.  Flash read configuration word */
#define EE_FL_CFG_FL                   (*(uint16_t*)(MEM_EEPROM_CS_START + 0x026u))

/** NV_TRIM10. Flash erase/write timing */
#define EE_FL_ER_WR_FL                 (*(uint16_t*)(MEM_EEPROM_CS_START + 0x028u))

/** NV_TRIM11. Timing port 2 */
#define EE_TIM_PORT2                   (*(uint16_t*)(MEM_EEPROM_CS_START + 0x02Au))

/** NV_TRIM12. Timing port 1 */
#define EE_TIM_PORT1                   (*(uint16_t*)(MEM_EEPROM_CS_START + 0x02Cu))

#endif /* FL_USE_DIRECT_SETTINGS */

/* Chip revision definition */
/** IC ID0 */
#define EE_CHIP_ID_0                   (*(uint16_t*)(MEM_EEPROM_CS_START + 0x030u))

/** IC ID1 */
#define EE_CHIP_ID_1                   (*(uint16_t*)(MEM_EEPROM_CS_START + 0x032u))

/** IC ID2 */
#define EE_CHIP_ID_2                   (*(uint16_t*)(MEM_EEPROM_CS_START + 0x034u))

/** IC ID3 used as a PRODUCT ID cell */
#define EE_CHIP_ID_3                   (*(uint16_t*)(MEM_EEPROM_CS_START + 0x036u))

#endif /* EEPROM_MAP_H */
/// @}
