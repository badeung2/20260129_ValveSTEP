/*!*************************************************************************** *
 * \file        mem_map_MLX81160.h
 * \brief       MLX81160 Memory-map
 *
 * \note        project MLX81160
 *
 * \author      Marcel Braat
 *
 * \date        2022-12-02
 *
 * \version     1.0 - preliminary
 *
 *
 * \copyright   MELEXIS Microelectronic Integrated Systems
 *
 * Copyright (C) 2022-2022 Melexis N.V.
 * The Software is being delivered 'AS IS' and Melexis, whether explicitly or
 * implicitly, makes no warranty as to its Use or performance.
 * The user accepts the Melexis Firmware License Agreement.
 *
 * Melexis confidential & proprietary
 * *************************************************************************** */

#ifndef MEM_MAP_MLX81160_H
#define MEM_MAP_MLX81160_H

/*!*************************************************************************** *
 *                              I N C L U D E S                                *
 * *************************************************************************** */
#include <syslib.h>

#if defined (__MLX81160__)

/*!*************************************************************************** *
 *                            DEFINES                                          *
 * *************************************************************************** */
/* MLX16 Memory addresses (RAM, ROM and FLASH) */
#define C_RAM_MLX16_START_ADDR      0x00C00UL                                   /*!< MLX16 RAM Start-address */
#define C_RAM_MLX16_END_ADDR        0x013FFUL                                   /*!< MLX16 RAM End-address */
#define C_ROM_SYS_START_ADDR        0x02C00UL                                   /*!< System ROM Start-address */
#define C_ROM_SYS_CRC_ADDR          0x053FCUL                                   /*!< System ROM CRC address */
#define C_ROM_SYS_END_ADDR          0x053FFUL                                   /*!< System ROM End-address */
#define C_FLASH_START_ADDR          (uint16_t)(0x8000UL >> 1)                   /*!< Flash start address */
#define C_FLASH_CRC_ADDR            (uint16_t)(0xFFFCUL >> 1)                   /*!< Flash CRC address */
#define C_FLASH_STOP_ADDR           (uint16_t)(0x10000UL >> 1)                  /*!< Flash end address */

#define C_RAM_TRANSPARENT_BIST_KEY  0x0A6EU                                     /*!< The HW BIST KEY used to run the Transparent RAM BIST */
#define C_RAM_REGULAR_BIST_KEY      0x0F59U                                     /*!< The HW BIST KEY used to run the Regular RAM BIST */
#define C_RAM_MLX16_BIST_CFG_SEED   0x000001UL                                  /*!< An initial SEED for the RAM BIST checks */

#define C_ROM_BIST_CFG_SEED         0x000001UL                                  /*!< An initial SEED for the FLASH and ROM BIST checks */
#define C_ROM_BIST_KEY              0x11EBU                                     /*!< The HW BIST KEY used to run the BIST */

/* MLX81160 */
#define C_ADDR_IO_BGN               0x0000U                                     /*!< Begin-address I/O */
#define C_ADDR_IO_END               0x02ACU                                     /*!< End-address I/O (included) */
#define C_ADDR_NV_BGN               0x0400U                                     /*!< Begin-address Non Volatile Memory */
#define C_ADDR_NV_END               0x0636U                                     /*!< End-address Non Volatile Memory (included) */
#define C_ADDR_RAM_BGN              0x0A00U                                     /*!< Begin-address RAM */
#define C_ADDR_RAM_END              0x13FEU                                     /*!< End-address RAM (included) */
#define C_ADDR_ROM_BGN              0x2C00U                                     /*!< Begin-address ROM */
#define C_ADDR_ROM_END              0x53FEU                                     /*!< End-address ROM (included) */
#define C_ADDR_FLASH_BGN            0x8000U                                     /*!< Begin-address Flash */
#define C_ADDR_FLASH_END            0xFFFEU                                     /*!< End-address Flash (included) */
#define C_ADDR_APP_STRING           0xFFE4U                                     /*!< Application string offset */
#define C_ADDR_FLASH_CRC            0xFFFCU                                     /*!< Flash CRC offset */
#define C_ADDR_MLX4_FW_VERSION      0x28018UL                                   /*!< MLX4/COLIN Firmware version offset */
#define C_ADDR_MLX4_LOADER_VERSION  0x2801AUL                                   /*!< MLX4/COLIN Loader version offset */

/* NVM Memory addresses */
#define C_NVM_START_ADDR            (uint16_t)(0x0400U >> 1)                    /*!< Non Volatile Memory Start address */
#define C_NVM_STOP_ADDR             (uint16_t)(0x0638U >> 1)                    /*!< Non Volatile Memory End address */

/* MLX4 Memory addresses (RAM & ROM) */
#define C_RAM_MLX4_START_ADDR       0x0A00U                                     /*!< MLX4 RAM Start-address */
#define C_RAM_MLX4_END_ADDR         0x0BFFU                                     /*!< MLX4 RAM End-address */
#define C_RAM_MLX4_DYNAMIC_BGN_ADR1 0x0A08U                                     /*!< MLX4 Dynamic (Frame-ID) Begin-address #1 */
#define C_RAM_MLX4_DYNAMIC_END_ADR1 0x0A0EU                                     /*!< MLX4 Dynamic (Frame-ID) End-address #1 */
#define C_RAM_MLX4_DYNAMIC_BGN_ADR2 0x0A10U                                     /*!< MLX4 Dynamic (Frame-ID) Begin-address #2  */
#define C_RAM_MLX4_DYNAMIC_END_ADR2 0x0A16U                                     /*!< MLX4 Dynamic (Frame-ID) End-address #2 */
#define C_RAM_MLX4_STATIC_BGN_ADDR  0x0A50U                                     /*!< MLX4 Static (Tables) Begin-address */
#define C_RAM_MLX4_STATIC_END_ADDR  0x0C00U                                     /*!< MLX4 Static (Tables) End-address */
#define C_ROM_MLX4_START_ADDR       0x28000UL                                   /*!< MLX4 ROM Start-address */
#define C_ROM_MLX4_END_ADDR         0x297FFUL                                   /*!< MLX4 ROM End-address */
#if defined (__MLX81160A01__)
#define C_ROM_MLX4_BIST_CRC         0x00244A51UL                                /*!< MLX4 ROM BIST Checksum */
#endif

/* NVM map */
#define ADDR_NV_START               0x0400U                                     /*!< Non Volatile Memory Begin address */
#define ADDR_NV_USER                0x0440U                                     /*!< Non Volatile Memory User Area address */
#define ADDR_NV_MLX_CALIB           0x05B0U                                     /*!< Begin address Melexis Calibration Area */
#define ADDR_NV_MLX_TRIM            0x0600U                                     /*!< Begin address Melexis Trim Area */
#define ADDR_NV_CHIPID              0x0630U                                     /*!< Begin address Melexis IC ID field (8-bytes) */

/* MELEXIS AREA:
 *  1) APP_CHIP_ID:       1 block  of 64-bits ( 4x 16-bits words)
 *  2) FL_EE_TRIM:    3.5/4 blocks of 64-bits (14/16x 16-bits words)
 *  3) MLX_PLTF:          2 blocks of 64-bits ( 8x 16-bits words)
 *  4) PATCH_HDR:       2.5 blocks of 64-bits (10x 16-bits words)
 *  5) PATCH_CODE:     1-14 blocks of 64-bits (14x 16-bits words)
 * TOTAL:
 */
/*! 1) APP_CHIP_ID: 1 block  of 64-bits (4x 16-bits words) */
typedef struct
{
    uint16_t u16AppChipID_1;                                                    /**< 0x00: Application Chip ID (part 1) */
    uint16_t u16AppChipID_2;                                                    /**< 0x02: Application Chip ID (part 2) */
    uint16_t u16AppChipID_3;                                                    /**< 0x04: Application Chip ID (part 3) */
    uint16_t u16AppChipID_4;                                                    /**< 0x06: Application Chip ID (part 4) */
} APP_CHIP_ID_t;

/*! 2) FL_EE_TRIM: 3.5/4 blocks of 64-bits (14/16x 16-bits words) */
typedef struct
{
    uint16_t u16NVTRIM_00;                                                      /**< 0x00: R2_EE */
    uint16_t u16NVTRIM_01;                                                      /**< 0x02: R1_EE */
    uint16_t u16NVTRIM_02;                                                      /**< 0x04: R3_FL */
    uint16_t u16NVTRIM_03;                                                      /**< 0x06: R2_FL */
    uint16_t u16NVTRIM_04;                                                      /**< 0x08: R1_FL */
    uint16_t u16NVTRIM_05;                                                      /**< 0x0A: Non Volatile Memory read timing port */
    uint16_t u16NVTRIM_06;                                                      /**< 0x0C: Non Volatile Memory program cycle port */
    uint16_t u16NVTRIM_07;                                                      /**< 0x0E: Non Volatile Memory erase timing */
    uint16_t u16NVTRIM_08;                                                      /**< 0x10: Non Volatile Memory write timing */
    uint16_t u16NVTRIM_09;                                                      /**< 0x12: flash read configuration word */
    uint16_t u16NVTRIM_10;                                                      /**< 0x14: flash erase/write timing */
    uint16_t u16NVTRIM_11;                                                      /**< 0x16: Timing Port 2 */
    uint16_t u16NVTRIM_12;                                                      /**< 0x18: Timing Port 1 */
    uint16_t u16NVTRIM_13;                                                      /**< 0x1A: NV Memory Open */
} NV_TRIM_t;

/*! 3) MLX_PLTF: 2 blocks of 64-bits ( 8x 16-bits words) */
typedef struct
{
    uint16_t u16MS_TRIM0_BG_BIAS;                                               /**< 0x00: Trim BG BIAS */
    uint16_t u16MS_TRIM1_VDD;                                                   /**< 0x02: Trim VDD */
    uint16_t u16MS_TRIM2_RCO1M_LIN;                                             /**< 0x04: Trim RCO1M LIN */
    uint16_t u16MS_TRIM3_ADCREF1n2;                                             /**< 0x06: ADC SAR ADCREF1 & ADCREF2*/
    uint16_t u16MS_TRIM4_ADCREF3;                                               /**< 0x08: ADC SAR ADCREF3 */
    uint16_t u16MS_TRIM5_VDD_EXT;                                               /**< 0x0A: TRIM VDD EXT */
    uint16_t u16MS_TRIM6_RCO32M_24M;                                            /**< 0x0C: Trim RCO32M @ 24MHz */
    uint16_t u16MS_TRIM7_RCO32M_28M;                                            /**< 0x0E: Trim RCO32M @ 28MHz */
    uint16_t u16MS_TRIM8_RCO32M_32M;                                            /**< 0x10: Trim RCO32M @ 32MHz */
} MS_TRIM_t;

/*! 4) PATCH_HDR: 3 blocks of 64-bits (12x 16-bits words) */
typedef struct
{
    uint16_t u8CRC                      : 8;                                    /**< 0x00: Patch header & code CRC */
    uint16_t u8Length                   : 8;                                    /**< 0x01: Patch code size */
    uint16_t u16PatchProjID;                                                    /**< 0x02: Patch Project ID */
    uint16_t u16PatchAddress0;                                                  /**< 0x04: Patch 0 Address[15:1]; Bit 0 = activation-bit */
    uint16_t u16PatchInstruction0;                                              /**< 0x06: Patch 0 Instruction */
    uint16_t u16PatchAddress1;                                                  /**< 0x08: Patch 1 Address[15:1]; Bit 0 = activation-bit */
    uint16_t u16PatchInstruction1;                                              /**< 0x0A: Patch 1 Instruction */
    uint16_t u16PatchAddress2;                                                  /**< 0x0C: Patch 2 Address[15:1]; Bit 0 = activation-bit */
    uint16_t u16PatchInstruction2;                                              /**< 0x0E: Patch 2 Instruction */
    uint16_t u16Code[24];                                                       /**< 0x10..0x3F: Patch code */
} PATCH_HDR_t;

#define C_NV_MLX_VER_1                      0x01U                               /*!< Below Non Volatile Memory Layout version */
#define C_NV_MLX_VER_4                      0x04U                               /*!< Non Volatile Memory Layout with Offset VSMF2 support (CSA/OCD on) */
/*! APP_TRIM: Melexis Application Trim area (10 blocks), based on Calibration doc Rev 1.1, 26-FEB-2020 */
typedef struct
{
    uint16_t u8APP_TRIM00_CalibCRC          : 8;                                /**< 0x00: Calibration area CRC */
    uint16_t u8APP_TRIM00_TempMid           : 8;                                /**< 0x01: Mid-Temperature (e.g. 35) */
    uint16_t u12App_TRIM01_OffVSMF          : 12;                               /**< 0x02: Offset VSMF/26 at RT with CSA/OCD Off */
    uint16_t u4App_TRIM01_OffVSMF2_LSB      : 4;                                /**< 0x02: Offset VSMF/26 at RT with CSA/OCD On */
    uint16_t u16APP_TRIM02_OTempCal;                                            /**< 0x04: Mid-temperature ADC value */
    uint16_t u16APP_TRIM03_TempMid          : 8;                                /**< 0x06: Mid-Temperature (e.g. 35) */
    uint16_t u8APP_TRIM03_CalibVersion      : 8;                                /**< 0x07: Calibration data version */
    uint16_t u8APP_TRIM04_GainTempLow       : 8;                                /**< 0x08: Gain Temperature "Low"  (Below TempMid) */
    uint16_t u8APP_TRIM04_GainTempHigh      : 8;                                /**< 0x09: Gain Temperature "High" (Above TempMid) */
    uint16_t u8APP_TRIM05_ADC_ORef          : 8;                                /**< 0x0A: ADC reference offset */
    uint16_t u8APP_TRIM05_LINAA_TRIM        : 8;                                /**< 0x0B: LIN-AA Trim[4:0] */
    uint16_t u8APP_TRIM06_ADC_GainLow       : 8;                                /**< 0x0C: ADC Gain Low */
    uint16_t u8APP_TRIM06_ADC_GainHigh      : 8;                                /**< 0x0D: ADC Gain High */
    uint16_t u8APP_TRIM07_GainClock24LowT   : 8;                                /**< 0x0E: RCO32 @ 24MHz Gain "Low" */
    uint16_t u8APP_TRIM07_GainClock24HighT  : 8;                                /**< 0x0F: RCO32 @ 24MHz Gain "High" */
    int16_t i8APP_TRIM08_OffClock24         : 8;                                /**< 0x10: RCO32 @ 24MHz Offset */
    uint16_t u8APP_TRIM08_GainClock28LowT   : 8;                                /**< 0x11: RCO32 @ 28MHz Gain "Low" */
    uint16_t u8APP_TRIM09_GainClock28HighT  : 8;                                /**< 0x12: RCO32 @ 28MHz Gain "High" */
    int16_t i8APP_TRIM09_OffClock28         : 8;                                /**< 0x13: RCO32 @ 28MHz Offset */
    uint16_t u8APP_TRIM10_GainClock32LowT   : 8;                                /**< 0x14: RCO32 @ 32MHz Gain "Low" */
    uint16_t u8APP_TRIM10_GainClock32HighT  : 8;                                /**< 0x15: RCO32 @ 32MHz Gain "High" */
    int16_t i8APP_TRIM11_OffClock32         : 8;                                /**< 0x16: RCO32 @ 32MHz Offset */
    uint16_t u8APP_TRIM11_Off10kHz          : 8;                                /**< 0x17: 10kHz Offset*/
    uint16_t u16APP_TRIM12_Reserved;                                            /**< 0x18: Reserved */
    uint16_t u16APP_TRIM13_Reserved;                                            /**< 0x1A: Reserved */
    uint16_t u16APP_TRIM14_Reserved;                                            /**< 0x1C: Reserved */
    uint16_t u16APP_TRIM15_Reserved;                                            /**< 0x1E: Reserved */
    uint16_t u16APP_TRIM16_Reserved;                                            /**< 0x20: Reserved */
    uint8_t u8APP_TRIM17_GainDiffMCur1      : 8;                                /**< 0x22: Gain difference between 10x and 20x gain settings for CSA #1 */
    uint8_t u8APP_TRIM17_GainDiffMCur2      : 8;                                /**< 0x23: Gain difference between 10x and 20x gain settings for CSA #2 */
    /* Base on Calibration & Trim Document: Revision 0.7, 07-10-2020 */
    int8_t i8App_TRIM18_GainLo_VSMF         : 8;                                /**< 0x24: Gain-change of VSMF/26 input over low temperature [8bit signed] */
    int8_t i8App_TRIM18_GainHi_VSMF         : 8;                                /**< 0x25: Gain-change of VSMF/26 input over high temperature [8bit signed] */
    uint16_t u12App_TRIM19_GainVSMF         : 12;                               /**< 0x26: Gain of VSMF/26 at mid temp [12bit unsigned] */
    uint16_t u4App_TRIM19_Unused            : 4;                                /**< 0x26: Unused */
    uint16_t u16App_TRIM20_LowTempADC;                                          /**< 0x28: Raw ADC measurement of the temperature sensor at Low Test Temperature  [12 bit unsigned] */
    uint16_t u16App_TRIM21_HighTempADC;                                         /**< 0x2A: Raw ADC measurement of the temperature sensor at High Test Temperature  [12 bit unsigned] */
    int8_t i8App_TRIM22_GainLo_CURR2        : 8;                                /**< 0x2C: Gain-change of current sensor #2 input over low temperature range [8 bit signed] */
    int8_t i8App_TRIM22_GainHi_CURR2        : 8;                                /**< 0x2D: Gain-change of current sensor #2 input over high temperature range [8 bit signed] */
    uint16_t u12App_TRIM23_Gain_CURR2       : 12;                               /**< 0x2E: Gain of current sensor #2 input *10 at mid temperature [12 bit unsigned] */
    uint16_t u4App_TRIM23_O_CURR2_LSB       : 4;                                /**< 0x2E: Offset of current sensor #2 input *10 at mid temperature [12 bit unsigned] (LSB) */
    uint16_t u8App_TRIM24_O_CURR2_MSB       : 8;                                /**< 0x30: Offset of current sensor #2 input *10 at mid temperature [12 bit unsigned] (MSB) */
    int8_t i8App_TRIM24_GainLo_CURR1        : 8;                                /**< 0x31: Gain-change of current sensor #1 input over low temperature range [8 bit signed] */
    int8_t i8App_TRIM25_GainHi_CURR1        : 8;                                /**< 0x32: Gain-change of current sensor #1 input over high temperature range [8 bit signed] */
    uint16_t u8App_TRIM25_Gain_CURR1_LSB    : 8;                                /**< 0x33: Gain of current sensor #1 input *10 at mid temperature [12 bit unsigned] (LSB) */
    uint16_t u4App_TRIM26_Gain_CURR1_MSB    : 4;                                /**< 0x34: Gain of current sensor #1 input *10 at mid temperature [12 bit unsigned] (MSB) */
    uint16_t u12App_TRIM26_O_CURR1          : 12;                               /**< 0x34: Offset of current sensor #1 input *10 at mid temperature [12 bit unsigned] */
    int8_t i8App_TRIM27_GainLo_HVI          : 8;                                /**< 0x36: Gain-change of High-voltage input over low temperature range [8 bit signed] */
    int8_t i8App_TRIM27_GainHi_HVI          : 8;                                /**< 0x37: Gain-change of High-voltage input over high temperature range [8 bit signed] */
    uint16_t u12App_TRIM28_Gain_HVI         : 12;                               /**< 0x38: Gain of High-voltage input HVI/26 at mid temperature [12 bit unsigned] */
    uint16_t u4App_TRIM28_O_HVI_LSB         : 4;                                /**< 0x38: Offset of High-voltage input HVI/26 at mid temperature [12 bit unsigned] (LSB) */
    uint16_t u8App_TRIM29_O_HVI_MSB         : 8;                                /**< 0x3A: Offset of High-voltage input HVI/26 at mid temperature [12 bit unsigned] (MSB) */
    int8_t i8App_TRIM29_GainLo_LVI          : 8;                                /**< 0x3B: Gain-change of Low-voltage input over low temperature range [8 bit unsigned] */
    int8_t i8App_TRIM30_GainHi_LVI          : 8;                                /**< 0x3C: Gain-change of Low-voltage input over high temperature range [8 bit unsigned] */
    uint16_t u8App_TRIM30_Gain_LVI_LSB      : 8;                                /**< 0x3D: Gain of Low-voltage input LVI/2.5 at mid temperature [12 bit unsigned] (LSB) */
    uint16_t u4App_TRIM31_Gain_LVI_MSB      : 4;                                /**< 0x3E: Gain of Low-voltage input LVI/2.5 at mid temperature [12 bit unsigned] (MSB) */
    uint16_t u12App_TRIM31_O_LVI            : 12;                               /**< 0x3E: Offset of Low-voltage input LVI/2.5 at mid temperature [12 bit unsigned] */
    uint16_t u6App_TRIM32_OTD               : 6;                                /**< 0x40: Trim setting over-temperature detection */
    uint16_t u10App_TRIM32_CPCLK            : 10;                               /**< 0x40: Trim setting charge pump clock (60MHz) */
    uint8_t u8APP_TRIM33_GOCS2              : 8;                                /**< 0x42: Gain of the over-current DAC */
    uint8_t u8APP_TRIM33_OOCS2              : 8;                                /**< 0x43: Offset of the over-current DAC */
    uint8_t u8APP_TRIM34_GOCS1              : 8;                                /**< 0x44: Gain of the over-current DAC */
    uint8_t u8APP_TRIM34_OOCS1              : 8;                                /**< 0x45: Offset of the over-current DAC */
    uint16_t u16APP_TRIM35_Empty;                                               /**< 0x46: Empty */
    uint16_t u16APP_TRIM36_Speed7;                                              /**< 0x48: RCO32 Speed #7 */
    uint16_t u16APP_TRIM37_Speed6;                                              /**< 0x4A: RCO32 Speed #6 */
    uint16_t u16APP_TRIM38_Speed5;                                              /**< 0x4C: RCO32 Speed #5 */
    uint16_t u16APP_TRIM39_Speed4;                                              /**< 0x4E: RCO32 Speed #4 */
} MLX_CALIB_t;
extern volatile MLX_CALIB_t CalibrationParams __attribute__((nodp, addr(ADDR_NV_MLX_CALIB)));  /*!< Calibration parameters Non Volatile Memory location */

/*! MLX_TRIM: Melexis factory Trim area (17 blocks) */
typedef struct
{
    uint16_t u16CRC;                                                            /**< 256: Checksum */
    MS_TRIM_t MS_Trim;                                                          /**< 257-265: Platform */
    NV_TRIM_t NV_Trim;                                                          /**< 266-279: Flash and Non Volatile Memory trim */
    APP_CHIP_ID_t AppChipID;                                                    /**< 280-283: "Application" Chip-ID */
} NV_MLX_MAP_t;
extern volatile NV_MLX_MAP_t TrimParams __attribute__((nodp, addr(ADDR_NV_MLX_TRIM)));  /*!< Trim parameters Non Volatile Memory location */

#define C_OADC                      2048U                                       /*!< ADC Offset is 50% of range */
#define C_GADC_HV                   962U                                        /*!< 1.48V * 26 * 100 * C_VOLTGAIN_DIV / (4095/2) [10mV] */
#define C_GADC_VSMF                 962U                                        /*!< 1.48V * 26 * 100 * C_VOLTGAIN_DIV / (4095/2) (filtered) [10mV] */
#define C_GADC_LV                   925U                                        /*!< 1.48V * 2.5 * 1000 * C_VOLTGAIN_DIV / (4095/2) [10mV] */
#define C_OADC_HV                   C_OADC                                      /*!< HV Offset */
#define C_OADC_LV                   C_OADC                                      /*!< LV Offset */
#define C_OADC_VSMF                 C_OADC                                      /*!< VSMF Offset */
#define C_GADC_MCUR                 1852U                                       /*!< Motor Current Gain (1.41mA/LSB * C_GMCURR_DIV) */
#define C_OADC_MCUR                 C_OADC                                      /*!< Motor Current Offset */
#define C_ADC_TEMP_LOW              0x0CB3U                                     /*!< Default-value for Low-temperature ADC-readout */
#define C_ADC_TEMP_MID              0x0C1FU                                     /*!< Default-value for Mid-temperature ADC-readout */
#define C_ADC_TEMP_HIGH             0x0B18U                                     /*!< Default-value for High-temperature ADC-readout */
#define C_GADC_TEMP                 128                                         /*!< Temperature gain factor */
#define C_GADC_VDDA                 (uint16_t)(1.48 * 3U * 100U)                /*!< Gain VDDA / 2048 */
#define C_GADC_VDDD                 (uint16_t)(1.48 * 2U * 100U)                /*!< Gain VDDD / 2048 */
#define C_GADC_VAUX                 (uint16_t)(1.48 * 4U * 100U)                /*!< Gain VAUX / 2048 (MMP220307-1) */
#define C_GADC_VBGD                 (uint16_t)(1.48 * 1U * 100U)                /*!< Gain VBGD / 2048 (MPP220307-1) */
#define C_MIDTEMP                   35                                          /*!< Mid-temperature [C] */

#define C_DEF_CPCLK60_TC            103                                         /*!< Default Temperature coefficient for CP-Clock at 60MHz */
#define C_DEF_CPCLK82_TC            79                                          /*!< Default Temperature coefficient for CP-Clock at 82MHz */

#endif /* defined (__MLX81160__) */

#endif /* MEM_MAP_MLX81160_H */

/* EOF */
