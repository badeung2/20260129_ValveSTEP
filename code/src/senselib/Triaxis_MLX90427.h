/*!************************************************************************** *
 * \file        Triaxis_MLX90427.h
 * \brief       MLX813xx (SPI) Triaxis MLX90427 handling
 *
 * \note        project MLX813xx
 *
 * \author      Marcel Braat
 *
 * \date        2024-06-18
 *
 * \version     2.0
 *
 * \copyright   MELEXIS Microelectronic Integrated Systems
 *
 * Copyright (C) 2024-2024 Melexis N.V.
 * The Software is being delivered 'AS IS' and Melexis, whether explicitly or
 * implicitly, makes no warranty as to its Use or performance.
 * The user accepts the Melexis Firmware License Agreement.
 *
 * Melexis confidential & proprietary
 *
 * *************************************************************************** */

#ifndef SENSE_LIB_TRIAXIS_MLX90427_H
#define SENSE_LIB_TRIAXIS_MLX90427_H

/*!*************************************************************************** */
/*                           INCLUDES                                          */
/* *************************************************************************** */
#include "AppBuild.h"                                                           /* Application Build */

#if (_SUPPORT_TRIAXIS_MLX90427 != FALSE)

/*!*************************************************************************** */
/*                           DEFINITIONS                                       */
/* *************************************************************************** */
/* Triaxis Commands */
#define CMD_TRIAXIS_MASK                0x07U                                   /*!< LIN Commands Mask */
#define CMD_TRIAXIS_NOP                 0x00U                                   /*!< Triaxis command: NOP */
#define CMD_TRIAXIS_ALPHA               0x01U                                   /*!< Triaxis command: ANGLE (1D) */
#define CMD_TRIAXIS_ALPHA_BETA          0x02U                                   /*!< Triaxis command: ALPHA & BETA (2D) */
#define CMD_TRIAXIS_XYZ                 0x03U                                   /*!< Triaxis command: XYZ (3D, only in Joy-stick configuration) */
#define CMD_TRIAXIS_STANDBY             0x04U                                   /*!< Triaxis command: STANDBY */
#define CMD_TRIAXIS_RESET               0x05U                                   /*!< Triaxis command: RESET */
#define CMD_TRIAXIS_MEM_RD              0x07U                                   /*!< Triaxis command: MEMORY READ */
/* #define CMD_TRIAXIS_MEM_WR           0x08U */                                /*!< Doesn't work (LIN-command mask = 3-bits) */
#define CMD_TRIAXIS_GET                 0x12U                                   /*!< Triaxis command: GET */

/* Triaxis Errors */
#define ERR_TRIAXIS_OK                  0x00U                                   /*!< Triaxis Error: OK */
#define ERR_TRIAXIS_CRC                 0x01U                                   /*!< Triaxis Error: CRC */
#define ERR_TRIAXIS_NODATA              0x02U                                   /*!< Triaxis Error: No Data */
#define ERR_TRIAXIS_INVCMD              0x03U                                   /*!< Triaxis Error: Invalid Command */
#define ERR_TRIAXIS_INVLEN              0x04U                                   /*!< Triaxis Error: Invalid Length */
#define ERR_TRIAXIS_UNKNOWN             0x05U                                   /*!< Triaxis Error: Unknown */
#define ERR_TRIAXIS_INVREPLY            0x06U                                   /*!< Triaxis Error: Invalid Reply */
#define ERR_TRIAXIS_MALFUNCTION         0x07U                                   /*!< Triaxis Error: Mall Function */
#define ERR_TRIAXIS_DIAGNOSTIC_INFO     0x08U                                   /*!< Triaxis Error: Diagnostics Info */
#define ERR_TRIAXIS_EE_WRITE            0x09U                                   /*!< Triaxis Error: EEPROM Write */
#define ERR_TRIAXIS_INVDATA             0x0AU                                   /*!< Triaxis Error: Invalid Data */

/* MLX90427 MOSI Commands */
#define CMD_MLX90427_GET                0x07U                                   /*!< MLX90427 Command: GET */
#define CMD_MLX90427_GET_NEXT           0x0BU                                   /*!< MLX90427 Command: GET_NEXT */
#define CMD_MLX90427_SET                0x0DU                                   /*!< MLX90427 Command: SET */
#define CMD_MLX90427_NOP                0x13U                                   /*!< MLX90427 Command: NOP */
#define CMD_MLX90427_RST                0x15U                                   /*!< MLX90427 Command: RST */
#define CMD_MLX90427_STBY               0x16U                                   /*!< MLX90427 Command: STBY */
#define CMD_MLX90427_TRG_NORMAL         0x19U                                   /*!< MLX90427 Command: TRG_NORMAL */
#define CMD_MLX90427_TRG_SYNC           0x1AU                                   /*!< MLX90427 Command: TRG_SYNC */
#define CMD_MLX90427_PROTECTED_MODE     0x23U                                   /*!< MLX90427 Command: PROTECTED_MODE */
#define CMD_MLX90427_EXIT               0x25U                                   /*!< MLX90427 Command: EXIT */
#define CMD_MLX90427_NVM_RECALL         0x26U                                   /*!< MLX90427 Command: NVM_RECALL */
#define CMD_MLX90427_NVM_STORE          0x29U                                   /*!< MLX90427 Command: NVM_STORE */
#define CMD_MLX90427_READ               0x2AU                                   /*!< MLX90427 Command: READ */
#define CMD_MLX90427_READ_NEXT          0x2CU                                   /*!< MLX90427 Command: READ_NEXT */
#define CMD_MLX90427_WRITE              0x31U                                   /*!< MLX90427 Command: WRITE */
#define CMD_MLX90427_WRITE_NEXT         0x32U                                   /*!< MLX90427 Command: WRITE_NEXT */
#define CMD_MLX90427_RESET_PARTIAL      0x34U                                   /*!< MLX90427 Command: RESET_PARTIAL */

#define C_MLX90427_RST_KEY              0x1F4CU                                 /*!< MLX90427 Reset command 'key' */
#define C_MLX90427_STBY_KEY             0x6B8CU                                 /*!< MLX90427 Standby command 'key' */
#define C_MLX90427_PROTECTED_MODE_KEY1  0xB255U                                 /*!< MLX90427 Protected Mode 'key', upper 16-bits of 48-bits */
#define C_MLX90427_PROTECTED_MODE_KEY2  0xA2D3U                                 /*!< MLX90427 Protected Mode 'key', middle 16-bits of 48-bits */
#define C_MLX90427_PROTECTED_MODE_KEY3  0x8C5EU                                 /*!< MLX90427 Protected Mode 'key', lower 16-bits of 48-bits */
#define C_MLX90427_RST_PARTIAL_KEY      0x6CF0U                                 /*!< MLX90427 Partial Reset 'key' */

/*! GET mode (Show below is the default (SEL=0) Secondary Data) */
#define C_MLX90427_GET_MODE_LEGACY              0x1U                            /*!< Primary Data: ANGLE[13:0], Secondary Data: 0x0000U */
#define C_MLX90427_GET_MODE_STRAY_FIELD_IMMUNE  0x2U                            /*!< Primary Data: ANGLE_DBZ[13:0], Secondary Data: 0x0000U */
#define C_MLX90427_GET_MODE_DUAL_ROTATY         0x3U                            /*!< Primary Data: ANGLE[13:0], Secondary Data: ANGLE_DBZ[13:0] */
#define C_MLX90427_GET_MODE_DIAG_ROTATY         0x4U                            /*!< Primary Data: ANGLE[13:0], Secondary Data: ANGLE_HALF[13:0] */
#define C_MLX90427_GET_MODE_2D_LEGACY           0x5U                            /*!< Primary Data: FIELD_B0[13:0], Secondary Data: FIELD_B1[13:0] */
#define C_MLX90427_GET_MODE_2D_DBZ              0x6U                            /*!< Primary Data: FIELD_B0_DBZ[13:0], Secondary Data: FIELD_B1_DBZ[13:0] */
#define C_MLX90427_GET_MODE_FULL_DIAG_SEQ       0x7U                            /*!< Primary Data: N/A, Secondary Data: N/A */
#define C_MLX90427_GET_MODE_JOYSTICK            0x9U                            /*!< Primary Data: ALPHA[13:0], Secondary Data: PUSH_BUTTON[1:0] | BETA[13:0] */
#define C_MLX90427_GET_MODE_3D                  0xEU                            /*!< Primary Data: N/A, Secondary Data: N/A */

/*! GET SEL-field */
#define C_MLX90427_GET_SEL_DEFAULT              0x0U                            /*!< Secondary Data: Default (See above) */
#define C_MLX90427_GET_SEL_FIELD_B0             0x1U                            /*!< Secondary Data: legacy FIELD_B0[13:0] */
#define C_MLX90427_GET_SEL_FIELD_B1             0x2U                            /*!< Secondary Data: legacy FIELD_B1[13:0] */
#define C_MLX90427_GET_SEL_FIELD_B0_DBZ         0x3U                            /*!< Secondary Data: FIELD_B0_DBZ[13:0] */
#define C_MLX90427_GET_SEL_FIELD_B1_DBZ         0x4U                            /*!< Secondary Data: FIELD_B1_DBZ[13:0] */
#define C_MLX90427_GET_SEL_FIELD_MAG_LEG        0x6U                            /*!< Secondary Data: legacy FIELD_MAG_LEG[15:0] */
#define C_MLX90427_GET_SEL_FIELD_MAG_DBZ        0x7U                            /*!< Secondary Data: FIELD_MAG_DBZ[15:0] */
#define C_MLX90427_GET_SEL_GAIN                 0x8U                            /*!< Secondary Data: Virtual gain (AGC) GAIN_DBZ[7:0] | GAIN[7:0] */
#define C_MLX90427_GET_SEL_ROC_LEG              0x9U                            /*!< Secondary Data: Rough offset correction, legacy ROC_SIN[7:0] | ROC_COS[7:0] */
#define C_MLX90427_GET_SEL_ROC_DBZ              0xAU                            /*!< Secondary Data: Rough offset correction, dBz ROC_SIN_DBZ[7:0] | ROC_COS_DBZ[7:0] */

/*! Type-field */
#define C_ML90427_TYPE_RESULT_STATUS    0x0U                                    /*!< MLX90427 Type: RESULT_STATUS */
#define C_ML90427_TYPE_RESULT_ACK       0x1U                                    /*!< MLX90427 Type: RESULT_ACK */
#define C_ML90427_TYPE_ERROR            0x8U                                    /*!< MLX90427 Type: ERROR */

/*! State-field */
#define C_MLX90427_STATE_STARTUP        0x0U                                    /*!< MLX90427 State: Start-up */
#define C_MLX90427_STATE_SAFE_STARTUP   0x1U                                    /*!< MLX90427 State: Safe startup */
#define C_MLX90427_STATE_NORMAL         0x4U                                    /*!< MLX90427 State: Normal */
#define C_MLX90427_STATE_PROTECTED_MODE 0x5U                                    /*!< MLX90427 State: Protected Mode */

/*! Error-codes */
#define C_MLX90427_ERR_FRAME            0xCCU                                   /*!< MLX90427 Error-code ERR_FRAME: low level error (CS rising edge received in middle of a byte, RX/TX buffer overflow) */
#define C_MLX90427_ERR_CRC              0x69U                                   /*!< MLX90427 Error-code ERR_CRC: a CRC error on the MOSI message is detected. */
#define C_MLX90427_ERR_RDY              0x33U                                   /*!< MLX90427 Error-code ERR_RDY: system is not yet ready to accept and process any command (during startup or safe startup phases). */
#define C_MLX90427_ERR_ONGOING          0x5AU                                   /*!< MLX90427 Error-code ERR_ONGOING: previous command is still ongoing (includes opcode) and any other command is being refused for now. Currently received command is dropped and result of ongoing command is delayed to next transaction. */
#define C_MLX90427_ERR_OPC              0x3CU                                   /*!< MLX90427 Error-code ERR_OPC: an invalid opcode is received. */
#define C_MLX90427_ERR_STATE            0x55U                                   /*!< MLX90427 Error-code ERR_STATE: a command was received that is not accepted in the current state. */
#define C_MLX90427_ERR_KEY              0x96U                                   /*!< MLX90427 Error-code ERR_KEY: an invalid key is received. */
#define C_MLX90427_ERR_ACCESS           0x66U                                   /*!< MLX90427 Error-code ERR_ACCESS: the current access level is not accepting the command. */
#define C_MLX90427_ERR_ADDRESS          0x99U                                   /*!< MLX90427 Error-code ERR_ADDRESS: an invalid or byte-aligned address is provided to read from or write to. */
#define C_MLX90427_ERR_ARGS             0xA5U                                   /*!< MLX90427 Error-code ERR_ARGS: invalid arguments are provided. */
#define C_MLX90427_ERR_TIME             0xAAU                                   /*!< MLX90427 Error-code ERR_TIME: a timeout is detected, on for example the measurements. */
#define C_MLX90427_ERR_DIAGS            0x0FU                                   /*!< MLX90427 Error-code ERR_DIAGS: a diagnostic error is detected, with DIAGS_STATE returned in bytes 7 to 4. */
#define C_MLX90427_ERR_STORE            0xC3U                                   /*!< MLX90427 Error-code ERR_STORE: the store operation is faulty (not performed). This can be due to under-voltage detected, wrong CRC or already locked NVRAM. */

/*! The DIAGS_STATE bytes are encoded following below: */
#define B_MLX90427_DIAGS_STATE_ADC_ERR              (1UL << 0)                  /*!< MLX90427 ADC error detected */
#define B_MLX90427_DIAGS_STATE_SYS_ADC_TIME ADC     (1UL << 1)                  /*!< MLX90427 Sequence timing (TRG-to-EOS) fail detected. */
#define B_MLX90427_DIAGS_STATE_SYS_APS_TIME APS     (1UL << 2)                  /*!< MLX90427 Sequence timing (TRG-to-End of DSP) fail detected. */
#define B_MLX90427_DIAGS_STATE_DSP_OVF_APS DSP      (1UL << 3)                  /*!< MLX90427 Overflow in APS (prevention & detection). */
#define B_MLX90427_DIAGS_STATE_OV_VDD_5V            (1UL << 4)                  /*!< MLX90427 VDD over-voltage detected. */
#define B_MLX90427_DIAGS_STATE_UV_VDD_5V            (1UL << 5)                  /*!< MLX90427 VDD under-voltage detected. */
#define B_MLX90427_DIAGS_STATE_OV_VDDA              (1UL << 6)                  /*!< MLX90427 VDDA over-voltage detected. */
#define B_MLX90427_DIAGS_STATE_UV_VDDA              (1UL << 7)                  /*!< MLX90427 VDDA under-voltage detected. */
#define B_MLX90427_DIAGS_STATE_OV_VDDD              (1UL << 8)                  /*!< MLX90427 VDDD over-voltage detected. */
#define B_MLX90427_DIAGS_STATE_AFE_HP_DIAG          (1UL << 9)                  /*!< MLX90427 Diagnostic rotary magnetic mode comparison check failed. */
#define B_MLX90427_DIAGS_STATE_AFE_HP_DUAL          (1UL << 10)                 /*!< MLX90427 Dual mode rotary magnetic mode comparison check failed. */
#define B_MLX90427_DIAGS_STATE_AFE_AROC             (1UL << 11)                 /*!< MLX90427 Automatic rough offset correction (AROC) consistency failed. */
#define B_MLX90427_DIAGS_STATE_AFE_GAIN             (1UL << 12)                 /*!< MLX90427 Automatic gain control (AGC) consistency failed. */
#define B_MLX90427_DIAGS_STATE_AFE_FIELD_MAG_HIGH   (1UL << 13)                 /*!< MLX90427 Field magnitude too high. */
#define B_MLX90427_DIAGS_STATE_AFE_FIELD_MAG_LOW    (1UL << 14)                 /*!< MLX90427 Field magnitude too low. */
#define B_MLX90427_DIAGS_STATE_DSP_OVF_BTF          (1UL << 15)                 /*!< MLX90427 DSP overflow in background tasks (prevention & detection). */
#define B_MLX90427_DIAGS_STATE_HIGH_TEMP            (1UL << 16)                 /*!< MLX90427 Over-temperature detected. */
#define B_MLX90427_DIAGS_STATE_LOW_TEMP             (1UL << 17)                 /*!< MLX90427 Under-temperature detected. */
#define B_MLX90427_DIAGS_STATE_ADC_REF              (1UL << 18)                 /*!< MLX90427 ADC self-test (reference test points) failed. */
#define B_MLX90427_DIAGS_STATE_AFE_TEMP             (1UL << 19)                 /*!< MLX90427 Temperature sensors comparison failed. */
#define B_MLX90427_DIAGS_STATE_AFE_TESTBRIDGE       (1UL << 20)                 /*!< MLX90427 AFE test bridge diagnostic (reference test points) failed. */
#define B_MLX90427_DIAGS_STATE_SYS_CTM_LEGACY       (1UL << 22)                 /*!< MLX90427 Warning, cycle time monitoring for AGC and AROC calculation for legacy acquisition failed. */
#define B_MLX90427_DIAGS_STATE_SYS_CTM_DCZ          (1UL << 23)                 /*!< MLX90427 Warning, cycle time monitoring for AGC and AROC calculation for dBz acquisition failed. */
#define B_MLX90427_DIAGS_STATE_SYS_CTM_TEMP         (1UL << 24)                 /*!< MLX90427 Warning, cycle time monitoring for temperature compensation calculation failed. */
#define B_MLX90427_DIAGS_STATE_SYS_DCT              (1UL << 25)                 /*!< MLX90427 Warning, diagnostic cycle time monitoring failed. */

/* GET-command */
#define C_MLX90427_GET_SEL_CHIP_ID          0x01U                               /*!< MLX90427 GET_SEL: Chip ID (No GET_NEXT needed) */
#define C_MLX90427_GET_SEL_HW_VERSION       0x02U                               /*!< MLX90427 GET_SEL: HW version (No GET_NEXT needed) */
#define C_MLX90427_GET_SEL_RESET_SRC        0x03U                               /*!< MLX90427 GET_SEL: Warm reset source (Fatal) (No GET_NEXT needed) */
#define C_MLX90427_GET_SEL_NVM_CRC_CALC     0x04U                               /*!< MLX90427 GET_SEL: NVM CRC calculation (No GET_NEXT needed) */
#define C_MLX90427_GET_SEL_NVM_CRC_STORED   0x05U                               /*!< MLX90427 GET_SEL: NVM CRC stored (No GET_NEXT needed) */
#define C_MLX90427_GET_SEL_SW_VERSION       0x06U                               /*!< MLX90427 GET_SEL: SW version (2x GET_NEXT needed) */
#define C_MLX90427_GET_SEL_ADDER_2D         0x08U                               /*!< MLX90427 GET_SEL: Adder 2D (No GET_NEXT needed) */
#define C_MLX90427_GET_SEL_ADDER_3D         0x09U                               /*!< MLX90427 GET_SEL: Adder 3D (No GET_NEXT needed) */
#define C_MLX90427_GET_SEL_ADDER_4D         0x0AU                               /*!< MLX90427 GET_SEL: Adder 4D (1x GET_NEXT needed) */
#define C_MLX90427_GET_SEL_RAW_2D           0x10U                               /*!< MLX90427 GET_SEL: Raw 2D (3x GET_NEXT needed) */
#define C_MLX90427_GET_SEL_RAW_3D           0x11U                               /*!< MLX90427 GET_SEL: Raw 3D (4x GET_NEXT needed) */
#define C_MLX90427_GET_SEL_RAW_4D           0x12U                               /*!< MLX90427 GET_SEL: Raw 4D (6x GET_NEXT needed) */
#define C_MLX90427_GET_SEL_RAW_TEMP         0x13U                               /*!< MLX90427 GET_SEL: Raw Temperature (No GET_NEXT needed) */
#define C_MLX90427_GET_SEL_RAW_FDS          0x14U                               /*!< MLX90427 GET_SEL: Raw FDS (Full Diagnostic Sequence) (7x GET_NEXT needed) */
#define C_MLX90427_GET_SEL_NV_DSP           0x15U                               /*!< MLX90427 GET_SEL: NV_DSP (5x GET_NEXT needed) */

#define C_MLX90427_VALUE_MASK           0x3FFFU                                 /*!< Value MASK */

#define C_DELAY_1MS             (uint16_t)((1000UL * FPLL) / C_DELAY_CONST)     /*!< 1ms delay */
#define DELAY_tStartUp                  5U                                      /*!< 5ms (minimum) delay (@ 3.3V); From cold power-on to system in application mode */

#define ERR_MLX90427_OK                 0x00U                                   /*!< MLX90427 Low-level SPI Transfer status: OK */
#define ERR_MLX90427_CRC                0x01U                                   /*!< MLX90427 Low-level SPI Transfer status: CRC-error */

#define C_TRIAXIS_POLL_RATE             4U                                      /*!< 4 x 500us = 2ms (Must be longer than 1.1ms) */
/* #define C_TRIAXIS_POLL_RATE          20U */                                  /*!< 20 x 500us = 10ms (Must be longer than 1.1ms) */

/* Triaxis BGN to END = 320 degrees */
#define C_TRIAXIS_0DEG      (uint16_t)(((0UL * 65536U) + 180U) / 360U)          /*!< 0-degrees */
#define C_TRIAXIS_APP_BGN   (uint16_t)(((20UL * 65536U) + 180U) / 360U)         /*!< Begin application at 20 degrees */
#define C_TRIAXIS_APP_END   (uint16_t)(((340UL * 65536U) + 180U) / 360U)        /*!< End application at 340 degrees */
#define C_TRIAXIS_APP_RNG              (C_TRIAXIS_APP_END - C_TRIAXIS_APP_BGN)  /*!< Application range */

/*!*************************************************************************** */
/*                           GLOBAL VARIABLES                                  */
/* *************************************************************************** */
#pragma space dp                                                                /* __TINY_SECTION__ */
#pragma space none                                                              /* __TINY_SECTION__ */

extern uint16_t g_u16TriaxisActualPos __attribute__ ((section(".dp.noinit")));  /*!< Triaxis Actual Position */
extern uint16_t g_u16TriaxisTargetPos __attribute__ ((section(".dp.noinit")));  /*!< Triaxis Target Position */

#pragma space nodp                                                              /* __NEAR_SECTION__ */
extern uint16_t g_u16ShaftAngle;                                                /*!< Shaft Angle; Triaxis angle corrected with Sense-magnet offset */
extern uint8_t g_u8TriaxisError;                                                /*!< Triaxis Error */

#if (_SUPPORT_TRIAXIS_STANDBY == FALSE)
extern uint8_t g_u8LinTriaxisCmd;                                               /*!< SPI-command Triaxis Command field (3-bits) */
extern uint16_t g_u16TriaxisPollCounter;                                        /*!< Triaxis Poll Period Counter */
#endif /* (_SUPPORT_TRIAXIS_STANDBY == FALSE) */
extern uint16_t g_u16LinTriaxisInfo;                                            /*!< SPI-command Triaxis Info field (13-bits) */
#pragma space none                                                              /* __NEAR_SECTION__ */

/*!*************************************************************************** */
/*                           GLOBAL FUNCTIONS                                  */
/* *************************************************************************** */
extern uint16_t Triaxis_Init(void);                                             /*!< Initialise SPI interface and Triaxis MLX90427 */
extern uint16_t Triaxis_SendCmd(uint8_t u8TriaxisApiCmd);                       /*!< Send command to Triaxis */
extern uint16_t Triaxis_GetAbsPos(uint16_t u16Trancate);                        /*!< Get Absolute position */
extern void Adapt_TriaxisAngleOffset(uint16_t u16Offset);                       /*!< Adapt the Triaxis Angle Offset */
#if (LIN_COMM != FALSE)
extern void HandleTriaxisStatus(void);                                          /*!< Reply LIN response */
#endif /* (LIN_COMM != FALSE) */

#endif /* (_SUPPORT_TRIAXIS_MLX90427 != FALSE) */

#endif /* SENSE_LIB_TRIAXIS_MLX90427_H */

/* EOF */

