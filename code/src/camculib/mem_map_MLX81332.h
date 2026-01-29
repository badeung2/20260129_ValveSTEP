/*!*************************************************************************** *
 * \file        mem_map_MLX81332.h
 * \brief       MLX81332 Memory-map
 *
 * \note        project MLX81332
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

#ifndef MEM_MAP_MLX81332_H
#define MEM_MAP_MLX81332_H

/*!*************************************************************************** *
 *                              I N C L U D E S                                *
 * *************************************************************************** */
#include <syslib.h>

#if defined (__MLX81332__)

/*!*************************************************************************** *
 *                            DEFINES                                          *
 * *************************************************************************** */
/* MLX16 Memory addresses (RAM, ROM and FLASH) */
#define C_RAM_MLX16_START_ADDR      0x01000UL                                   /*!< MLX16 RAM Start-address */
#define C_RAM_MLX16_END_ADDR        0x017FFUL                                   /*!< MLX16 RAM End-address */
#define C_ROM_SYS_START_ADDR        0x02000UL                                   /*!< System ROM Start-address */
#define C_ROM_SYS_CRC_ADDR          0x047FCUL                                   /*!< System ROM CRC address */
#define C_ROM_SYS_END_ADDR          0x047FFUL                                   /*!< System ROM End-address */
#define C_FLASH_START_ADDR          FLASH_BIST_START                            /*!< Flash start address */
#define C_FLASH_CRC_ADDR            0xD7FCU                                     /*!< Flash CRC address */
#define C_FLASH_STOP_ADDR           0xD800U                                     /*!< Flash end address */

#define C_RAM_TRANSPARENT_BIST_KEY  0x0A6EU                                     /*!< The HW BIST KEY used to run the Transparent RAM BIST */
#define C_RAM_REGULAR_BIST_KEY      0x0F59U                                     /*!< The HW BIST KEY used to run the Regular RAM BIST */
#define C_RAM_MLX16_BIST_CFG_SEED   0x000001UL                                  /*!< An initial SEED for the RAM BIST checks */

#define C_ROM_BIST_CFG_SEED         0x000001UL                                  /*!< An initial SEED for the FLASH and ROM BIST checks */
#define C_ROM_BIST_KEY              0x11EBU                                     /*!< The HW BIST KEY used to run the BIST */

/* MLX81332 */
#define C_ADDR_IO_BGN               0x0000U                                     /*!< Begin-address I/O */
#if defined (__MLX81332A01__)
#define C_ADDR_IO_END               0x0240U                                     /*!< End-address I/O (included) */
#else
#define C_ADDR_IO_END               0x025CU                                     /*!< End-address I/O (included) */
#endif
#define C_ADDR_NV_BGN               0x0800U                                     /*!< Begin-address Non Volatile Memory */
#define C_ADDR_NV_END               0x0A36U                                     /*!< End-address Non Volatile Memory (included) */
#define C_ADDR_RAM_BGN              0x0E00U                                     /*!< Begin-address RAM */
#define C_ADDR_RAM_END              0x17FEU                                     /*!< End-address RAM (included) */
#define C_ADDR_ROM_BGN              0x2000U                                     /*!< Begin-address ROM */
#define C_ADDR_ROM_END              0x47FEU                                     /*!< End-address ROM (included) */
#define C_ADDR_FLASH_BGN            0x5800U                                     /*!< Begin-address Flash */
#define C_ADDR_FLASH_END            0xD7FEU                                     /*!< End-address Flash (included) */
#define C_ADDR_APP_STRING           0xD7E4U                                     /*!< Application string offset */
#define C_ADDR_FLASH_CRC            0xD7FCU                                     /*!< Flash CRC offset */
#define C_ADDR_MLX4_FW_VERSION      0x15818UL                                   /*!< MLX4/COLIN Firmware version offset */
#define C_ADDR_MLX4_LOADER_VERSION  0x1581AUL                                   /*!< MLX4/COLIN Loader version offset */

/* Non Volatile Memory Memory addresses */
#define C_NVM_START_ADDR            0x0800U                                     /*!< Non Volatile Memory Start address */
#define C_NVM_STOP_ADDR             0x0A38U                                     /*!< Non Volatile Memory End address */

/* MLX4 Memory addresses (RAM & ROM) */
#define C_RAM_MLX4_START_ADDR       0x0E00U                                     /*!< MLX4 RAM Start-address */
#define C_RAM_MLX4_END_ADDR         0x0FFFU                                     /*!< MLX4 RAM End-address */
#define C_RAM_MLX4_DYNAMIC_BGN_ADR1 0x0E08U                                     /*!< MLX4 Dynamic (Frame-ID) Begin-address #1 */
#define C_RAM_MLX4_DYNAMIC_END_ADR1 0x0E0EU                                     /*!< MLX4 Dynamic (Frame-ID) End-address #1 */
#define C_RAM_MLX4_DYNAMIC_BGN_ADR2 0x0E10U                                     /*!< MLX4 Dynamic (Frame-ID) Begin-address #2  */
#define C_RAM_MLX4_DYNAMIC_END_ADR2 0x0E16U                                     /*!< MLX4 Dynamic (Frame-ID) End-address #2 */
#define C_RAM_MLX4_STATIC_BGN_ADDR  0x0E50U                                     /*!< MLX4 Static (Tables) Begin-address */
#define C_RAM_MLX4_STATIC_END_ADDR  0x1000U                                     /*!< MLX4 Static (Tables) End-address */
#define C_ROM_MLX4_START_ADDR       0x15800UL                                   /*!< MLX4 ROM Start-address */
#define C_ROM_MLX4_END_ADDR         0x16FFFUL                                   /*!< MLX4 ROM End-address */
#if defined (__MLX81330B02__) || defined (__MLX81332B02__)
#define C_ROM_MLX4_BIST_CRC         0x00244A51UL                                /*!< MLX4 ROM BIST Checksum (V4.1.0.3) */
#endif

/* Non Volatile Memory map */
#define ADDR_NV_START               0x0800U                                     /*!< Non Volatile Memory Begin address */
#define ADDR_NV_USER                0x0840U                                     /*!< Non Volatile Memory User Area address */
#define ADDR_NV_MLX_CALIB           0x09B0U                                     /*!< Begin address Melexis Calibration Area */
#define ADDR_NV_MLX_TRIM            0x0A00U                                     /*!< Begin address Melexis Trim Area */
#define ADDR_NV_CHIPID              0x0A30U                                     /*!< Begin address Melexis IC ID field (8-bytes) */

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

#define C_NV_MLX_VER_3                      0x03U                               /*!< Below Non Volatile Memory Layout version */
/*! APP_TRIM: Melexis Application Trim area (10 blocks), based on Calibration doc Rev 1.1, 26-FEB-2020 */
typedef struct
{
    uint16_t u8APP_TRIM00_CalibCRC          : 8;                                /**< 0x00: Calibration area CRC */
    uint16_t u8APP_TRIM00_TempMid           : 8;                                /**< 0x01: Mid-Temperature (e.g. 35) */
    uint16_t u16APP_TRIM01_Free;                                                /**< 0x02: Reserved for Low-temperature ADC value */
    uint16_t u16APP_TRIM02_OTempCal;                                            /**< 0x04: Mid-temperature ADC value */
    uint16_t u16APP_TRIM03_TempMid          : 8;                                /**< 0x06: Mid-Temperature (e.g. 35) */
    uint16_t u8APP_TRIM03_CalibVersion      : 8;                                /**< 0x07: Calibration data version */
    uint16_t u8APP_TRIM04_GainTempLow       : 8;                                /**< 0x08: Gain Temperature "Low"  (Below TempMid) */
    uint16_t u8APP_TRIM04_GainTempHigh      : 8;                                /**< 0x09: Gain Temperature "High" (Above TempMid) */
    uint16_t u8APP_TRIM05_ADC_ORef          : 8;                                /**< 0x0A: ADC reference offset */
#if defined (__MLX81332A01__)
    uint16_t u8APP_TRIM05_Reserved          : 8;                                /**< 0x0B: Reserved */
#else
    uint16_t u8APP_TRIM05_LINAA_TRIM        : 8;                                /**< 0x0B: LIN-AA Trim[4:0] */
#endif
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
    uint16_t u16APP_TRIM12_LINAA_ISHUNT;                                        /**< 0x18: LIN-AA Offset Differential Mode */
    uint16_t u16APP_TRIM13_Free;                                                /**< 0x1A: Reserved for LIN-AA Offset Common-Mode */
    int16_t i16APP_TRIM14_AASDMCM;                                              /**< 0x1C: LIN-AA Gain Differential Mode from Common Mode */
    uint16_t u16APP_TRIM15_SAADM;                                               /**< 0x1E: LIN-AA Gain Differential Mode */
    uint16_t u8APP_TRIM16_IAA_Trim045mA     : 8;                                /**< 0x20: LIN-AA Current Source Trim for 0.45mA */
    uint16_t u8APP_TRIM16_IAA_Trim205mA     : 8;                                /**< 0x21: LIN-AA Current Source Trim for 2.05mA */
    uint16_t u8APP_TRIM17_IAA_Trim240mA     : 8;                                /**< 0x22: LIN-AA Current Source Trim for 2.40mA */
#if defined (__MLX81332A01__)
    uint16_t u8APP_TRIM17_SC_GAIN_TRIM      : 8;                                /**< 0x23: LIN-AA SC Gain[3:0] & Trim[3:0] */
#else  /* defined (__MLX81332A01__) */
    uint16_t u8APP_TRIM17_SC_GAIN           : 8;                                /**< 0x23: LIN-AA SC Gain[3:0] */
#endif /* defined (__MLX81332A01__) */
    uint16_t u8APP_TRIM18_GainVSMF          : 8;                                /**< 0x24: Gain VSMF (MMP210312-1) */
    int16_t i8APP_TRIM18_OffHV              : 8;                                /**< 0x25: Offset HV */
    uint16_t u8APP_TRIM19_GainLV            : 8;                                /**< 0x26: Gain LV */
    int16_t i8APP_TRIM19_OffLV              : 8;                                /**< 0x27: Offset LV */
    uint16_t u8APP_TRIM20_GainHV            : 8;                                /**< 0x28: Gain HV (MMP210312-1) */
    int16_t i8APP_TRIM20_OffVSMF            : 8;                                /**< 0x29: Offset VSM Filtered */
    int16_t i8APP_TRIM21_GainLV_LowT        : 8;                                /**< 0x2A: Gain LV Low-Temperature */
    int16_t i8APP_TRIM21_GainLV_HighT       : 8;                                /**< 0x2B: Gain LV High-Temperature */
    uint16_t u8APP_TRIM22_GainMCur          : 8;                                /**< 0x2C: Gain Motor Current */
    int16_t i8APP_TRIM22_OffMCur            : 8;                                /**< 0x2D: Offset Motor Current */
    uint16_t u16APP_TRIM23_Reserved;                                            /**< 0x2E: Reserved */
    int16_t i8APP_TRIM24_GainMCur_LowT      : 8;                                /**< 0x30: Gain Motor Current Low-Temperature */
    int16_t i8APP_TRIM24_GainMCur_HighT     : 8;                                /**< 0x31: Gain Motor Current High-Temperature */
    int16_t i8APP_TRIM25_GainHV_LowT        : 8;                                /**< 0x32: Gain HV Low-Temperature */
    int16_t i8APP_TRIM25_GainHV_HighT       : 8;                                /**< 0x33: Gain HV High-Temperature */
    uint16_t u16APP_TRIM26_Reserved;                                            /**< 0x34: Reserved */
    uint16_t u16APP_TRIM27_LowTempADC;                                          /**< 0x36: Low-temperature ADC value */
    uint16_t u16APP_TRIM28_HighTempADC;                                         /**< 0x38: High-temperature ADC value */
    uint16_t u8APP_TRIM29_CPCLK60_HighT     : 8;                                /**< 0x3A: Charge-pump Clock (60 MHz) High-Temperature coefficient */
    uint16_t u8APP_TRIM29_CPCLK82_HighT     : 8;                                /**< 0x3B: Charge-pump Clock (82 MHz) High-Temperature coefficient */
    uint16_t u16APP_TRIM30_TRIM1_DRV;                                           /**< 0x3C: TRIM1_DRV (CP=82MHz) */
    uint16_t u16APP_TRIM31_TRIM2_DRV;                                           /**< 0x3E: TRIM2_DRV */
    uint16_t u8APP_TRIM32_TRIM3_DRV_HIGH    : 8;                                /**< 0x40: TRIM3_DRV - High Current level (dual phase parallel) (MMP190312-1) */
    uint16_t u8APP_TRIM32_TRIM3_DRV_LOW     : 8;                                /**< 0x41: TRIM3_DRV - Low Current level (single phase) (MMP190312-1) */
    uint16_t u16APP_TRIM33_TRIM_MISC;                                           /**< 0x42: TRIM_MISC */
#if defined (__MLX81330B02__) || defined (__MLX81332B02__) || defined (__MLX81334__)
    uint16_t u16APP_TRIM34_TRIM1_DRV_60M;                                       /**< 0x44: TRIM1_DRV (CP=60MHz) */
#else
    uint16_t u16APP_TRIM34_Empty;                                               /**< 0x44: Empty */
#endif
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

#define C_OADC                      0U                                          /*!< ADC Offset is 0 */
#define C_GADC_HV                   164U                                        /*!< 2.5V * 21 * 100 * C_VOLTGAIN_DIV / 1023 */
#define C_GADC_VSMF                 164U                                        /*!< 2.5V * 21 * 100 * C_VOLTGAIN_DIV / 1023 (filtered) */
#define C_OADC_HV                   C_OADC                                      /*!< HV Offset */
#define C_OADC_LV                   C_OADC                                      /*!< LV Offset */
#define C_OADC_VSMF                 C_OADC                                      /*!< VSMF Offset */
#define C_GADC_LV                   211U                                        /*!< 2.5V * 1.35 * 2000 * C_VOLTGAIN_DIV / 1023 */
#define C_GADC_MCUR                 180U                                        /*!< Motor Current Gain (1.41mA/LSB * C_GMCURR_DIV) */
#define C_OADC_MCUR                 485U                                        /*!< Motor Current Offset: (1.185V/2.5V)*1024 */
#define C_ADC_TEMP_LOW              715U                                        /*!< Default-value for Low-temperature ADC-readout */
#define C_ADC_TEMP_MID              623U                                        /*!< Default-value for Mid-temperature ADC-readout */
#define C_ADC_TEMP_HIGH             477U                                        /*!< Default-value for High-temperature ADC-readout */
#define C_GADC_TEMP                 128                                         /*!< Temperature gain factor */
#define C_GADC_VDDA                 (uint16_t)(2.5 * 2U * 100U)                 /*!< Gain VDDA / 1024 */
#define C_GADC_VDDD                 (uint16_t)(2.5 * 1U * 100U)                 /*!< Gain VDDD / 1024 */
#define C_OADC_VDDD                 12U                                         /*!< VDDD Offset: 30mV; (30mV/2.5V) * 1024 */
#define C_GADC_VAUX                 (uint16_t)(2.5 * 4U * 100U)                 /*!< Gain VAUX / 1024 (MMP220307-1) */
#define C_GADC_VBGD                 (uint16_t)(2.5 * 1U * 100U)                 /*!< Gain VBGD / 1024 (MMP220307-1) */
#define C_MIDTEMP                   35                                          /*!< Mid-temperature [C] */

#define C_DEF_CPCLK60_TC            103                                         /*!< Default Temperature coefficient for CP-Clock at 60MHz */
#define C_DEF_CPCLK82_TC            79                                          /*!< Default Temperature coefficient for CP-Clock at 82MHz */

#endif /* defined (__MLX81332__) */

#endif /* MEM_MAP_MLX81332_H */

/* EOF */
