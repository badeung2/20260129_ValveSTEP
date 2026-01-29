/*!*************************************************************************** *
 * \file        ErrorCodes.h
 * \brief       MLX813xx ErrorCodes handling
 *
 * \note        project MLX813xx
 *
 * \author      Marcel Braat
 *
 * \date        2017-08-15
 *
 * \version     2.0
 *
 *
 * \copyright   MELEXIS Microelectronic Integrated Systems
 *
 * Copyright (C) 2017-2022 Melexis N.V.
 * The Software is being delivered 'AS IS' and Melexis, whether explicitly or
 * implicitly, makes no warranty as to its Use or performance.
 * The user accepts the Melexis Firmware License Agreement.
 *
 * Melexis confidential & proprietary
 *
 * *************************************************************************** */

#ifndef DRIVE_LIB_ERRORCODES_H
#define DRIVE_LIB_ERRORCODES_H

/*!*************************************************************************** */
/*                           INCLUDES                                          */
/* *************************************************************************** */
#include "AppBuild.h"                                                           /* Application Build */

/*!*************************************************************************** */
/*                           DEFINITIONS                                       */
/* *************************************************************************** */
#define C_ERR_NONE                  0x00U                                       /*!< No error */
/* Unsupported MLX16 IRQ's (0x00..0x3F) */
#define C_MLX16_IRQ_01              0x01U                                       /*!< MLX16_STACKERR Stack error */
#define C_MLX16_IRQ_02              0x02U                                       /*!< MLX16_PROTERR Protection error */
#define C_MLX16_IRQ_03              0x03U                                       /*!< MLX16_MEMERR Memory error */
#define C_MLX16_IRQ_04              0x04U                                       /*!< MLX16_OPERR Operation error */
#define C_MLX16_IRQ_05              0x05U                                       /*!< Unused */
#define C_MLX16_IRQ_06              0x06U                                       /*!< MLX16_EXCHG Exchange (PTC and Debugger_fatal) */
#define C_MLX16_IRQ_07              0x07U                                       /*!< MLX16_DMAERR DMA error */
#define C_MLX16_IRQ_08              0x08U                                       /*!< AWD_ATT Absolute watchdog attention */
#define C_MLX16_IRQ_09              0x09U                                       /*!< IWD_ATT Intelligent watchdog attention */
#define C_MLX16_IRQ_0A              0x0AU                                       /*!< EE_SH_ECC */
#define C_MLX16_IRQ_0B              0x0BU                                       /*!< FL_SH_ECC */
#define C_MLX16_IRQ_0C              0x0CU                                       /*!< UV_VDDA */
#define C_MLX16_IRQ_0D              0x0DU                                       /*!< UV_VS */
#define C_MLX16_IRQ_0E              0x0EU                                       /*!< UV_VDDAF */
#define C_MLX16_IRQ_0F              0x0FU                                       /*!< ANA_PLL_ERR */
#define C_MLX16_IRQ_10              0x10U                                       /*!< OVT */
#define C_MLX16_IRQ_11              0x11U                                       /*!< OVC */
#define C_MLX16_IRQ_12              0x12U                                       /*!< OV_HS_VDS0 */
#define C_MLX16_IRQ_13              0x13U                                       /*!< OV_HS_VDS1 */
#define C_MLX16_IRQ_14              0x14U                                       /*!< OV_HS_VDS2 */
#define C_MLX16_IRQ_15              0x15U                                       /*!< OV_HS_VDS3 */
#define C_MLX16_IRQ_16              0x16U                                       /*!< OV_LS_VDS0 */
#define C_MLX16_IRQ_17              0x17U                                       /*!< OV_LS_VDS1 */
#define C_MLX16_IRQ_18              0x18U                                       /*!< OV_LS_VDS2 */
#define C_MLX16_IRQ_19              0x19U                                       /*!< OV_LS_VDS3 */
#define C_MLX16_IRQ_1A              0x1AU                                       /*!< STIMER Simple Timer interrupt */
#define C_MLX16_IRQ_1B              0x1BU                                       /*!< CTIMER0_1 Complex Timer interrupt 1 */
#define C_MLX16_IRQ_1C              0x1CU                                       /*!< CTIMER0_2 Complex Timer interrupt 2 */
#define C_MLX16_IRQ_1D              0x1DU                                       /*!< CTIMER0_3 Complex Timer interrupt 3 */
#define C_MLX16_IRQ_1E              0x1EU                                       /*!< CTIMER1_1 Complex Timer interrupt 1 */
#define C_MLX16_IRQ_1F              0x1FU                                       /*!< CTIMER1_2 Complex Timer interrupt 2 */
#define C_MLX16_IRQ_20              0x20U                                       /*!< CTIMER1_3 Complex Timer interrupt 3 */
#define C_MLX16_IRQ_21              0x21U                                       /*!< SPI_TE SPI transmit register is empty */
#define C_MLX16_IRQ_22              0x22U                                       /*!< SPI_RF SPI receive register full */
#define C_MLX16_IRQ_23              0x23U                                       /*!< SPI_ER SPI transmission error */
#define C_MLX16_IRQ_24              0x24U                                       /*!< PWM_MASTER1_CMP Custom interrupt during the PWM period */
#define C_MLX16_IRQ_25              0x25U                                       /*!< PWM_MASTER1_END Interrupt at the end of the PWM period */
#define C_MLX16_IRQ_26              0x26U                                       /*!< PWM_SLAVE1_CMP Custom interrupt during the PWM period */
#define C_MLX16_IRQ_27              0x27U                                       /*!< PWM_SLAVE2_CMP Custom interrupt during the PWM period */
#define C_MLX16_IRQ_28              0x28U                                       /*!< PWM_SLAVE3_CMP Custom interrupt during the PWM period */
#define C_MLX16_IRQ_29              0x29U                                       /*!< PWM_MASTER2_CMP Custom interrupt during the PWM period */
#define C_MLX16_IRQ_2A              0x2AU                                       /*!< PWM_MASTER2_END Interrupt at the end of the PWM period */
#define C_MLX16_IRQ_2B              0x2BU                                       /*!< ADC_SAR ADC interrupt */
#define C_MLX16_IRQ_2C              0x2CU                                       /*!< EE_COMPLETE */
#define C_MLX16_IRQ_2D              0x2DU                                       /*!< FL_COMPLETE */
#define C_MLX16_IRQ_2E              0x2EU                                       /*!< COLIN_OWNMTX mutex interrupt */
#define C_MLX16_IRQ_2F              0x2FU                                       /*!< COLIN_LIN LIN interrupt */
#define C_MLX16_IRQ_30              0x30U                                       /*!< OV_VS */
#define C_MLX16_IRQ_31              0x31U                                       /*!< DIAG */
#define C_MLX16_IRQ_32              0x32U                                       /*!< IO_IN0 */
#define C_MLX16_IRQ_33              0x33U                                       /*!< IO_IN1 */
#define C_MLX16_IRQ_34              0x34U                                       /*!< IO_IN2 */
#define C_MLX16_IRQ_35              0x35U                                       /*!< IO_IN3 */
#define C_MLX16_IRQ_36              0x36U                                       /*!< I2C_GLOBAL_RESET */
#define C_MLX16_IRQ_37              0x37U                                       /*!< PPM_RX */
#define C_MLX16_IRQ_38              0x38U                                       /*!< PPM_TX */
#define C_MLX16_IRQ_39              0x39U                                       /*!< PPM_ERR */
#define C_MLX16_IRQ_3A              0x3AU                                       /*!< MLX16_SOFT Software Interrupt request */

/* UDS (0x20..0x3F) */
#define C_ERR_LIN2X_SESSION_CTRL    0x20U                                       /*!< LIN UDS Session Control error */
#define C_ERR_LIN2X_ECU_RESET       0x21U                                       /*!< LIN UDS ECU Reset error */
#define C_ERR_LIN2X_22              0x22U                                       /*!< LIN UDS Read by Identifier error */
#define C_ERR_LIN2X_2E              0x2EU                                       /*!< LIN UDS Write by Identifier error */
#define C_ERR_LIN2X_3E              0x3EU                                       /*!< LIN UDS Tester error */
/* Warnings (0x40..0x5F) */
#define C_WRN_AWD_ATT               0x40U                                       /*!< Watchdog pre-alert (attention) warning */
#define C_WRN_APPL_OVER_TEMP        0x41U                                       /*!< Application Over-temperature warning */
#define C_WRN_DIAG_VDS              0x42U                                       /*!< VDS Warning */
#define C_WRN_DIAG_OVER_CURRENT     0x43U                                       /*!< Over Current  Warning */
#define C_WRN_VREF                  0x44U                                       /*!< VDDAF Warning */
#define C_WRN_VBOOST                0x44U                                       /*!< VBOOST Warning */
#define C_WRN_VS_UV                 0x45U                                       /*!< VS UV Warning */
#define C_WRN_VDDA_UV               0x46U                                       /*!< VDDA UV Warning */
#define C_WRN_LIN_TIMEOUT           0x47U                                       /*!< LIN Time-out warning ml_GetState() */
#define C_WRN_ACT_RESTART           0x4FU                                       /*!< Actuator Restart */
/* Info */
#define C_INF_SRP                   0x50U                                       /*!< SRP Info */
#define C_INF_WINDMILL              0x51U                                       /*!< WindMill Info */
#define C_INF_WU                    0x52U                                       /*!< Wake-up reason */
/* Additional info (0x60..0x6F) */
#define C_ERR_EXT                   0x6000U                                     /*!< Extended Error's: 0x60-0x6F */
#define C_ERR_EXTW                  0x7000U                                     /*!< Extended Word Error's: 0x70 0xNNNN */
/* LIN Communication errors (0x80..0x9F) */
#define C_ERR_LIN_COMM              0x80U                                       /*!< 0x81: Short-Done
                                                                                 * 0x82: Crash/LIN Module reset
                                                                                 * 0x83: ID Parity error
                                                                                 * 0x84: Checksum error
                                                                                 * 0x85: Transmission (S2M) bit error
                                                                                 * 0x86: Data Framing
                                                                                 * 0x87: ID Framing (Stop-bit)
                                                                                 * 0x88: Synch field
                                                                                 * 0x89: Buffer locked (LIN message OVL)
                                                                                 * 0x8A: Short
                                                                                 * 0x8B: Time-out response
                                                                                 * 0x8C: Break detected
                                                                                 * 0x8F: Wake-up initialisation
                                                                                 * 0x90: Error during STOP bit transmission
                                                                                 */
#define C_ERR_LIN_RAM_DYNAMIC       0x97U                                       /*!< LIN Private RAM Dynamic error */
#define C_ERR_LIN_RAM_STATIC        0x98U                                       /*!< LIN Private RAM Static error */
#define C_ERR_LIN_BUF_NOT_FREE      0x99U                                       /*!< LIN Buffer not free */
#define C_ERR_LIN_CMD_OVF           0x9AU                                       /*!< LIN Command Overflow/Buffer not free */
#define C_ERR_LIN_BAUDRATE          0x9BU                                       /*!< LIN Baudrate error */
#define C_ERR_CAN_COMM              0x9CU                                       /*!< CAN I/F Error */
#define C_ERR_LIN_API               0x9EU                                       /*!< LIN API error */
#define C_ERR_MLX4_RESTART          0x9FU                                       /*!< MLX4 have been restarted */
/* Application errors (0xA0..0xAF) */
#define C_ERR_APPL_UNDER_TEMP       0xA0U                                       /*!< Application: Under Temperature */
#define C_ERR_APPL_OVER_TEMP        0xA1U                                       /*!< Application: Over Temperature */
#define C_ERR_APPL_UNDER_VOLT       0xA2U                                       /*!< Application: Under Voltage */
#define C_ERR_APPL_OVER_VOLT        0xA3U                                       /*!< Application: Over Voltage */
#define C_ERR_APPL_SPI_INIT         0xA4U                                       /*!< Soft SPI Initialisation failed */
#define C_ERR_APPL_SPI_NOT_INIT     0xA5U                                       /*!< Soft SPI not initialised */
#define C_ERR_APPL_SPI_READ         0xA6U                                       /*!< Soft SPI Read failure */
#define C_ERR_APPL_SPI_WRITE        0xA7U                                       /*!< Soft SPI Write failure */
#define C_ERR_APPL_STOP             0xA8U                                       /*!< Application stop */
/* #define C_ERR_APPL_                 0A9U */
#define C_ERR_TRIAXIS_X_CLIP        0xAAU                                       /*!< Triaxis calibration error: X-clipping */
#define C_ERR_TRIAXIS_Y_CLIP        0xABU                                       /*!< Triaxis calibration error: Y-clipping */
#define C_ERR_TRIAXIS_CALIB         0xACU                                       /*!< Triaxis calibration error: Rotation */
#define C_ERR_TRIAXIS_INTERFACE     0xADU                                       /*!< Triaxis interface error */
#define C_ERR_TRIAXIS_DEADZONE_CW   0xAEU                                       /*!< Triaxis Dead-zone; Move CW */
#define C_ERR_TRIAXIS_DEADZONE_CCW  0xAFU                                       /*!< Triaxis Dead-zone; Move CCW */
/* LIN frame errors (0xB0..0xBF) */
#define C_ERR_LIN2X_B0              0xB0U                                       /*!< LIN 2.x NAD Change error */
#define C_ERR_LIN2X_B1              0xB1U                                       /*!< LIN 2.x Assign MessageID to FrameID failed/unsupported */
#define C_ERR_LIN2X_B2              0xB2U                                       /*!< LIN 2.x Read by Identifier (unsupported ID) */
#define C_ERR_LIN2X_B3              0xB3U                                       /*!< n.a. */
#define C_ERR_LIN2X_B4              0xB4U                                       /*!< LIN 2.x Assign Variant-ID, HW-Ref and SW-Ref */
#define C_ERR_LIN2X_B5              0xB5U                                       /*!< LIN 2.x Auto Addressing failure */
#define C_ERR_LIN2X_B6              0xB6U                                       /*!< LIN 2.x Save Configuration failure */
#define C_ERR_LIN2X_B7              0xB7U                                       /*!< LIN 2.x Assign Frame-ID range failure */
#define C_ERR_LIN2X_LINAA           0xB8U                                       /*!< LIN 2.x Non LIN-AA Diagnostic command during LIN-AA mode */
#define C_ERR_LIN2X_WRITE           0xB9U                                       /*!< LIN 2.x Write not allowed (not stop-mode) */
#define C_ERR_LIN_INV_PARAM         0xBAU                                       /*!< LIN Invalid Parameter */
/* #define C_ERR_LIN_                  0xBBU */
#define C_ERR_LIN_INV_CMD           0xBCU                                       /*!< LIN Invalid Command Request */
#define C_ERR_LIN2X_CB              0xBDU                                       /*!< LIN 2.x Write by Identifier (unsupported ID, or incorrect) */
/* #define C_ERR_LIN_                  0xBEU */
#define C_ERR_LIN_BUS_TIMEOUT       0xBFU                                       /*!< LIN Bus-timeout */
/* User NVRAM Page errors (0xC0..0xCF) */
#define C_ERR_INV_USERPAGE_1        0xC0U                                       /*!< Invalid NVRAM User Page #1 (use User Page #2 or #3) */
#define C_ERR_INV_USERPAGE_2        0xC1U                                       /*!< Invalid NVRAM User Page #2 (use User Page #1 or #3) */
#define C_ERR_INV_USERPAGE_BOTH     0xC2U                                       /*!< Invalid NVRAM User Page #1 & #2 (using defaults) */
#define C_ERR_INV_NAD               0xC3U                                       /*!< Invalid Node Address */
#define C_ERR_INV_USERPAGE_3        0xC4U                                       /*!< Invalid NVRAM user Page #3 (use User Page #1 or defaults) */
/* #define C_ERR_INV_                  0xC5U */
#define C_ERR_INV_MLX4ROM_CRC       0xC6U                                       /*!< Invalid MLX4 ROM */
#define C_ERR_INV_MLX16ROM_CRC      0xC7U                                       /*!< Invalid MLX16 ROM */
/* MLX NVRAM Page errors */
#define C_ERR_INV_MLXPAGE_CRC1      0xC8U                                       /*!< Invalid MLX Calibration Area */
/* #define C_ERR_INV_                  0xC9U */
/* #define C_ERR_INV_                  0xCAU */
/* #define C_ERR_INV_                  0xCBU */
/* #define C_ERR_INV_                  0xCCU */
/* #define C_ERR_INV_                  0xCDU */
#define C_ERR_NVWRITE               0xCEU                                       /*!< Non Volatile Memory Write failure */
/* #define C_ERR_INV_                  0xCFU */
/* Chip Diagnostics errors (0xD0..0xDF) */
#define C_ERR_DIAG_OVER_CURRENT     0xD0U                                       /*!< Diagnostic: Over Current */
#define C_ERR_DIAG_OVER_TEMP        0xD1U                                       /*!< Diagnostic: Over Temperature */
#define C_ERR_DIAG_UNDER_VOLT       0xD2U                                       /*!< Diagnostic: Under Voltage */
#define C_ERR_DIAG_OVER_VOLT        0xD3U                                       /*!< Diagnostic: Over Voltage */
#define C_ERR_DIAG_VDS              0xD4U                                       /*!< Diagnostic: Vds */
#define C_ERR_VREF                  0xD5U                                       /*!< ADC VREF check (MLX81235) */
#define C_ERR_VDDAF                 0xD5U                                       /*!< ADC VDDAF check (MLX8133x) */
#define C_ERR_VBOOST                0xD5U                                       /*!< ADC VBOOST check (MLX8134x) */
#define C_ERR_VBGD                  0xD5U                                       /*!< ADC VBGD check (MLX8133x) (MMP220307-1) */
#define C_ERR_VAUX                  0xD5U                                       /*!< ADC VAUX check (MLX8133x) (Not used) */
#define C_ERR_VDDA                  0xD6U                                       /*!< ADC VDDA check */
#define C_ERR_VDDD                  0xD7U                                       /*!< ADC VDDD check */
#define C_ERR_VIO_0                 0xD8U                                       /*!< ADC VIO[0] check */
#define C_ERR_VIO_1                 0xD9U                                       /*!< ADC VIO[1] check */
#define C_ERR_VIO_2                 0xDAU                                       /*!< ADC VIO[2] check */
#define C_ERR_VIO_3                 0xDBU                                       /*!< ADC VIO[3] check */
#define C_ERR_VIO_4                 0xDCU                                       /*!< ADC VIO[4] check */
#define C_ERR_VIO_5                 0xDDU                                       /*!< ADC VIO[5] check */
#define C_ERR_VIO_6                 0xDEU                                       /*!< ADC VIO[6] check */
#define C_ERR_VIO_7                 0xDFU                                       /*!< ADC VIO[7] check */
/* Electric Errors (0xE0..0xEF) */
#define C_ERR_UART0                 0xE0U                                       /*!< UART0 error */
#define C_ERR_UART1                 0xE1U                                       /*!< UART1 error */
#define C_ERR_HL_CW                 0xE2U                                       /*!< Hall Latch CW sequence error */
#define C_ERR_HL_CCW                0xE3U                                       /*!< Hall Latch CCW sequence error */
#define C_ERR_GEARBOX               0xE4U                                       /*!< Gear-box error (outer-shaft) */
#define C_ERR_TRIAXIS_FAILS         0xE5U                                       /*!< Triaxis Error */
#define C_ERR_COIL_ZERO_CURRENT     0xE6U                                       /*!< Actuator coil zero current (open) */
#define C_ERR_COIL_OVER_CURRENT     0xE7U                                       /*!< Actuator coil over current (short) */
#define C_ERR_CHIP_TEMP_PROFILE     0xE8U                                       /*!< Chip temperature profile error */
#define C_ERR_PHASE_SHORT_GND       0xE9U                                       /*!< Motor driver phase to GND short */
#define C_ERR_SELFTEST_A            0xEAU                                       /*!< Self-Test: FET Shortage (Over Current) */
#define C_ERR_SELFTEST_B            0xEBU                                       /*!< Self-Test: Short with other Phase (Over Current) */
#define C_ERR_SELFTEST_C            0xECU                                       /*!< Self-Test: Open Phase */
#define C_ERR_SELFTEST_D            0xEDU                                       /*!< Self-Test: Low-ohmic Phase U */
#define C_ERR_SELFTEST_E            0xEEU                                       /*!< Self-Test: Low-ohmic Phase V */
#define C_ERR_SELFTEST_F            0xEFU                                       /*!< Self-Test: Low-ohmic Phase W */
/* Fatal errors (0xF0..0xFE) */
#define C_ERR_FATAL                 0xF0U                                       /*!< Fatal-Error */
#define C_ERR_WD_RST                0xF1U                                       /*!< (Digital) Watchdog-reset */
#define C_ERR_AWD_RST               0xF2U                                       /*!< Analogue Watchdog-reset */
#define C_ERR_WD_AWD_RST            0xF3U                                       /*!< Analogue & Digital Watchdog-reset */
#define C_ERR_IO_RST                0xF4U                                       /*!< I/O-reset */
#define C_ERR_ROM_BG                0xF6U                                       /*!< ROM Background test error */
#define C_ERR_RAM                   0xF7U                                       /*!< (POR) RAM Complete-test error */
#define C_ERR_RAM_BG                0xF8U                                       /*!< RAM Background test error */
#define C_ERR_FLASH_BG              0xF9U                                       /*!< Flash Background test error */
#define C_ERR_ADC                   0xFAU                                       /*!< ADC error */
/* #define C_ERR_                      0xFBU */
#define C_ERR_IOREG                 0xFCU                                       /*!< I/O-registers error */
#define C_ERR_PRIO                  0xFDU                                       /*!< CPU & IRQ Priority error */
#define C_ERR_FATAL_EMRUN           0xFEU                                       /*!< Fatal Emergency-run */

#if (_SUPPORT_LOG_ERRORS != FALSE)

/*!*************************************************************************** */
/*                           GLOBAL VARIABLES                                  */
/* *************************************************************************** */
#pragma space nodp                                                              /* __NEAR_SECTION__ */
#pragma space none                                                              /* __NEAR_SECTION__ */

/*!*************************************************************************** */
/*                           GLOBAL FUNCTIONS                                  */
/* *************************************************************************** */
extern void ErrorLogInit(void);
extern void SetLastError(uint16_t u16ErrorCode);
extern uint16_t GetFirstError(void);
extern uint16_t PeakFirstError(void);
#if (_SUPPORT_NV_LOG_ERROR != FALSE)
extern uint8_t GetNV_LogError(uint8_t u8Idx);
extern void ClearNV_LogError(void);
#endif /* (_SUPPORT_NV_LOG_ERROR != FALSE) */

#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */

#endif /* DRIVE_LIB_ERRORCODES_H */

/* EOF */
