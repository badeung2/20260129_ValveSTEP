/**
 * @file
 * @brief Error hanlders definition file. Common configuration
 * @internal
 *
 * @copyright (C) 2021 Melexis N.V.
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
 * @ingroup error_handler
 * @ingroup hw_diag
 *
 * @brief Error hanlders definition file. Common configuration
 * @details
 * Automatic generation by render_xlsx2mako.py 1.1
 * CVSURL:  cvs.sensors.elex.be
 * CVSROOT: :pserver:user@cvs.sensors.elex.be:/var/cvsdesign
 * CVSPATH: 91230/product/design_input/Software/91230_SW_DiagIds.xlsx
 * CVS Tag: 1.29
 * Template version 1.0.0
 *
 * @{
 */

#ifndef ERROR_HANDLER_IDS_H
#define ERROR_HANDLER_IDS_H

/** An unique Error ID for defining the reason of failure and call the corresponding reaction to handle the failure */
/** Coprocessor check failure */
#define FAIL_CPU_COPROCESSOR            0x0010

/** EEPROM Region 0 [patches] BIST check failure */
#define FAIL_EEPROM_REG0_BIST           0x0020

/** EEPROM Region 1 [trimming] BIST check failure */
#define FAIL_EEPROM_REG1_BIST           0x0021

/** EEPROM Region 2 [mlx calib trimming] BIST check failure */
#define FAIL_EEPROM_REG2_BIST           0x0022

/** EEPROM Region 3 [user calib trimming] BIST check failure, size is read wrong or region is corrupted */
#define FAIL_EEPROM_REG3_BIST           0x0023

/** FLASH Region 0 BIST check failure */
#define FAIL_FLASH_REG0_BIST            0x0030

/** FLASH Region 1 BIST check failure */
#define FAIL_FLASH_REG1_BIST            0x0031

/** FLASH Region 2 BIST check failure */
#define FAIL_FLASH_REG2_BIST            0x0032

/** FLASH Region 3 BIST check failure */
#define FAIL_FLASH_REG3_BIST            0x0033

/** ROM Region 0 BIST check failure */
#define FAIL_ROM_REG0_BIST              0x0038

/** ROM Region 1 BIST check failure */
#define FAIL_ROM_REG1_BIST              0x0039

/** ROM Region 2 BIST check failure */
#define FAIL_ROM_REG2_BIST              0x003A

/** ROM Region 3 BIST check failure */
#define FAIL_ROM_REG3_BIST              0x003B

/** RAM Region 0 BIST check failure */
#define FAIL_RAM_REG0_BIST              0x0040

/** RAM Region 1 BIST check failure */
#define FAIL_RAM_REG1_BIST              0x0041

/** RAM Region 2 BIST check failure */
#define FAIL_RAM_REG2_BIST              0x0042

/** RAM Region 3 BIST check failure */
#define FAIL_RAM_REG3_BIST              0x0043

/** Rom info : Initialize RAM content */
#define ROM_INFO_RAM_INIT               0x0050

/** Rom mode : Maximum reset counter reached */
#define ROM_MODE_RESET_COUNTER          0x0060

/** Rom mode : Flash has incorrect project id, run bootloader mode */
#define ROM_MODE_PROJECT_ID             0x0061

/** Rom mode : PPM_FLASH_BistTest failed */
#define ROM_MODE_FLASH_BIST_FAILED      0x0062

/** Rom mode : (Uds) app consistency error */
#define ROM_MODE_FLASH_APP_CONSISTENCY  0x0063

/** Rom mode : Run ROM mode flag on */
#define ROM_MODE_FLAG_RUN_RAM           0x0064

/** ROM low trimming error */
#define FAIL_ROM_LOW_TRIMMING           0x0070

/** ROM high trimming error */
#define FAIL_ROM_HIGH_TRIMMING          0x0071

/** ROM ibat trimming error */
#define FAIL_ROM_IBAT_TRIMMING          0x0072

/** COLIN ROM Checksum check failure */
#define FAIL_COLINROM_CHECKSUM          0x0080

/** Stack Error Interrupt Event */
#define FAIL_MLX16_STACKERR             0xFF80

/** Protection Error Interrupt Event */
#define FAIL_MLX16_PROTERR              0xFF81

/** Memory Error Interrupt Event */
#define FAIL_MLX16_MEMERR               0xFF82

/** Operation Error Interrupt Event */
#define FAIL_MLX16_OPERR                0xFF83

/** DMA Error Interrupt Event */
#define FAIL_MLX16_DMAERR               0xFF84

/** FLASH ECC Error Interrupt Event */
#define FAIL_FL_ECC                     0xFF85

/** EEPROM ECC Error Interrupt Event */
#define FAIL_EE_ECC                     0xFF86

/** RAM_SHELL_MEM_ERR RAM Interrupt Event */
#define FAIL_RAM_SHELL_MEM_ERR          0xFF87

/** Unhandled Interrupt */
#define FAIL_UNHANDLED_ISR              0xFF90

/** Unknown failure */
#define FAIL_UNKNOWN_REASON             0xFFFF


#endif /* ERROR_HANDLER_IDS_H */

/// @}
/* EOF */
