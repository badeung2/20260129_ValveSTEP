/*!*************************************************************************** *
 * \file        LIN_Diagnostics.h
 * \brief       MLX813xx LIN Diagnostics communication handling
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

#ifndef LIN_DIAGNOSTICS_H
#define LIN_DIAGNOSTICS_H

/*!*************************************************************************** */
/*                           INCLUDES                                          */
/* *************************************************************************** */
#include "AppBuild.h"                                                           /* Application Build */

#if (LIN_COMM != FALSE)

#include "drivelib/NV_UserPage.h"                                               /* Non Volatile Memory User Application Page Layout */

/*!*************************************************************************** */
/*                           DEFINITIONS                                       */
/* *************************************************************************** */
#if (_SUPPORT_LIN_AA != FALSE) && (LIN_AA_TEST_DUT != FALSE)
#define C_TEST_DUT_NAD                      0x01U                               /*!< Test DUT NAD (0x01 or 0x02) for LIN-AA 3-device test */
#endif /* (_SUPPORT_LIN_AA != FALSE) && (LIN_AA_TEST_DUT != FALSE) */
#define C_BROADCAST_NAD                     0x7FU                               /*!< Broadcast NAD */
#define C_INVALID_NAD                       0xFFU                               /*!< Invalid NAD (MMP190801-1) */
/* Note: NAD 0x5A-0x5F are special devices */
#define C_LINAA_MODULE_NAD                  0x5AU                               /*!< LIN-AA Test-module */
#define C_DEFAULT_SNIFFER_NAD               0x5FU                               /*!< LIN Sniffer */

/* Supplier ID */
#define C_WILDCARD_SUPPLIER_ID              0x7FFFU                             /*!< Wild-card LIN Supplier ID */

/* Function ID */
#define C_WILDCARD_FUNCTION_ID              0xFFFFU                             /*!< Wild-card LIN Function ID */

#define mlxDFR_DIAG                         0x10U                               /*!< Demand for diagnostic */
#define mlxRFR_DIAG                         0x11U                               /*!< Response diagnostic */


/*
 *      +-----+-----+-----+----+----+----+----+----+
 *  SF  | NAD | PCI | SID | D1 | D2 | D3 | D4 | D5 |
 *      +-----+-----+-----+----+----+----+----+----+
 *           /       \
 *           7654 3210
 *  SF-Type: 0000 Length
 */
#define M_PCI_TYPE                              0xF0U                           /*!< PCI Mask */
#define C_PCI_SF_TYPE                           0x00U                           /*!< PCI Type: Single Frame (SF) */
#define C_PCI_FF_TYPE                           0x10U                           /*!< PCI Type: First Frame (FF) */
#define C_PCI_CF_TYPE                           0x20U                           /*!< PCI Type: Continuous Frame (CF) */
#define M_PCI_SF_LEN                            0x0FU                           /*!< LEN Mask */
#define M_PCI_FF_LEN256                         0x0FU                           /*!< FF-LEN Mask */
#define M_PCI_CF_FRAMECOUNTER                   0x0FU                           /*!< CF Frame-counter mask */
/* UDS Service */
#define C_SID_DIAG_SESSION_CTRL                 0x10U                           /*!< (Mandatory-DK2F) Diagnostics Session Control */
#define C_PCI_SID_DIAG_SESSION_CTRL             0x0210U                         /*!< PCI-SID of Diagnostics Session Control */
#define C_SESSION_DEFAULT_DIAGNOSTIC                0x01U                       /*!< (Mandatory) Default session */
#define C_SESSION_PROGRAMMING                       0x02U                       /*!< (Conditional: Flash Programming) Programming session */
#define C_SESSION_EXTENDED_DIAGNOSTIC               0x03U                       /*!< (Mandatory) Extended diagnostic session */
#define C_SESSION_EOL                               0x40U                       /*!< (Mandatory: VW) EOL session */
#define C_SESSION_DEVELOPMENT                       0x4FU                       /*!< (-) Development session */
#define C_SID_ECU_RESET                         0x11U                           /*!< (Mandatory-DK2F) ECU Reset */
#define C_PCI_SID_ECU_RESET                     0x0211U                         /*!< PCI-SID of Reset */
#define C_ECU_RESET_HARD                            0x01U                       /*!< (Conditional: Flash-programming) */
#define C_ECU_RESET_OFF_ON                          0x02U                       /*!< (Mandatory) */
#define C_ECU_RESET_SOFT                            0x03U                       /*!< (-) */
#define C_ECU_RESET_RAPID_POWER_SHUTDOWN            0x04U                       /*!< EnableRapidPowerShutDownSequence */
#define C_SID_CLR_DIAG_INFO                     0x14U                           /*!< (Mandatory) Clear Diagnostic Information */
#define C_PCI_SID_CLR_DIAG_INFO                 0x0314U                         /*!< PCI-SID of Clear Diagnostic Information */
#define C_SID_READ_DTC_INFO                     0x19U                           /*!< (Mandatory) Read DTC Information */
#define C_PCI_SID_READ_DTC_INFO                 0x0319U                         /*!< PCI-SID of Read DTC Information */
#define C_SID_READ_DATA_BY_IDENTIFY             0x22U                           /*!< (Mandatory-DK2F) Reading of the data associated to a DataIdentifier */
#define C_SID_READ_MEMORY_BY_ADDRESS            0x23U                           /*!< Read Memory by address */
#define C_SID_SECURITY_ACCESS                   0x27U                           /*!< (Mandatory-DK2F) Security Access */
#define C_SECURITY_ACCESS_TYPE_3                    0x03U                       /*!< Security access Type 3 */
#define C_SECURITY_ACCESS_TYPE_4                    0x04U                       /*!< Security access Type 4 */
#define C_SID_COMMUNICATION_CONTROL             0x28U                           /*!< (Mandatory-DK2F) Communication Control */
#define C_PCI_SID_COMMUNICATION_CONTROL         0x0428U                         /*!< PCI-SID of Communication Control */
#define C_COMM_CONTROL_TYPE_0                       0x00U                       /*!< Control Type 0 */
#define C_COMM_CONTROL_TYPE_1                       0x01U                       /*!< Control Type 1 */
#define C_COMM_CONTROL_TYPE_4                       0x04U                       /*!< Control Type 4 */
#define C_COMM_CONTROL_TYPE_5                       0x05U                       /*!< Control Type 5 */
#define C_SID_WRITE_DATA_BY_IDENTIFY            0x2EU                           /*!< (Mandatory-DK2F) Writing of the data associated to a DataIdentifier */
#define C_SID_ROUTINE_CONTROL                   0x31U                           /*!< (Mandatory-DK2F) Routine control */
#define C_ROUTINE_CONTROL_TYPE_1                    0x01U                       /*!< Routine Control Type 1 */
#define C_ROUTINE_CONTROL_TYPE_2                    0x02U                       /*!< Routine Control Type 2 */
#define C_ROUTINE_CONTROL_TYPE_3                    0x03U                       /*!< Routine Control Type 3 */
#define C_SID_REQUEST_DOWNLOAD                  0x34U                           /*!< (Mandatory-DK2F) Request Download */
#define C_SID_REQUEST_UPLOAD                    0x35U                           /*!< (Mandatory-DK2F) Request Upload */
#define C_SID_TRANSFER_DATA                     0x36U                           /*!< (Mandatory-DK2F) Transfer Data */
#define C_SID_REQUEST_TRANSFER_EXIT             0x37U                           /*!< (Mandatory-DK2F) Request Transfer Exit */
#define C_SID_WRITE_MEMORY_BY_ADDRESS           0x3DU                           /*!< Write Memory by Address */
#define C_SID_TESTER_PRESENT                    0x3EU                           /*!< (Mandatory-DK2F) Tester Present */
#define C_TESTER_PRESENT_TYPE_0                     0x00U                       /*!< Tester Type 0 present */
#define C_SID_CONTROL_DTC_SETTING               0x85U                           /*!< (Mandatory-DK2F) Control DTC Setting */
#define C_PCI_SID_CONTROL_DTC_SETTING           0x0485U                         /*!< PCI-SID of Control DTC Setting */
/* Service Identifier (SID) for node configuration (0xB0-0xB7) */
#define C_PCI_REASSIGN_NAD                      0x06U                           /*!< LEN of Reassign NAD */
#define C_SID_REASSIGN_NAD                      0xB0U                           /*!< (Optional) Reassign NAD */
#define C_PCI_SID_REASSIGN_NAD                  0x06B0U                         /*!< (Optional) Reassign NAD */
#define C_PCI_ASSIGN_FRAME_ID                   0x06U                           /*!< LEN of Assign Message-ID to Frame-ID */
#define C_SID_ASSIGN_FRAME_ID                   0xB1U                           /*!< (Obsolete) Assign Message-ID to Frame-ID */
#define C_PCI_SID_ASSIGN_FRAME_ID               0x06B1U                         /*!< (Obsolete) Assign Message-ID to Frame-ID */
#define C_PCI_READ_BY_ID                        0x06U                           /*!< LEN of Read by ID */
#define C_SID_READ_BY_ID                        0xB2U                           /*!< (Mandatory) Read by identifier */
#define C_PCI_SID_READ_BY_ID                    0x06B2U                         /*!< (Mandatory) Read by identifier */
#define C_LIN_PROD_ID                               0x00U                       /*!< (00, Mandatory) LIN Product Identification */
#define C_SERIAL_NR                                 0x01U                       /*!< (01, Optional) Serial number */
#if (_SUPPORT_ISO17987 != FALSE)
#define C_NEG_RESPONSE_ID                           0x02U                       /*!< Negative Response ID */
#define C_NCF_LDF_VERSION_ID                        0x03U                       /*!< NCF LDF Version ID */
#define C_NCF_LDF_VERSION                           (0x01U | (0x01U << 8))      /*!< Resp. Major (0x01) & Minor (0x01) version */
#define C_NCF_LDF_SUBVER_SOURCE                     (0xE1U | (0x02U << 8))      /*!< Resp. Sub version (0xE1) & Source (0x02) */
#endif /* (_SUPPORT_ISO17987 != FALSE) */
#define C_MSG_ID_1                                  0x10U                       /*!< (10, Optional) Message ID #1 (MMP160909-1) */
#define C_MSG_ID_2                                  0x11U                       /*!< (11, Optional) Message ID #2 */
#define C_MSG_ID_3                                  0x12U                       /*!< (12, Optional) Message ID #3 */
#define C_MSG_ID_4                                  0x13U                       /*!< (13, Optional) Message ID #4 */
#define C_MSG_ID_5                                  0x14U                       /*!< (14, Optional) Message ID #5 */
#define C_MSG_ID_6                                  0x15U                       /*!< (15, Optional) Message ID #6 */
#define C_MSG_ID_7                                  0x16U                       /*!< (16, Optional) Message ID #7 */
#define C_MSG_ID_8                                  0x17U                       /*!< (17, Optional) Message ID #8 */
#define C_MSG_ID_9                                  0x18U                       /*!< (18, Optional) Message ID #9 */
#define C_MSG_ID_10                                 0x19U                       /*!< (19, Optional) Message ID #10 */
#define C_MSG_ID_11                                 0x1AU                       /*!< (1A, Optional) Message ID #11 */
#define C_MSG_ID_12                                 0x1BU                       /*!< (1B, Optional) Message ID #12 */
#define C_MSG_ID_13                                 0x1CU                       /*!< (1C, Optional) Message ID #13 */
#define C_MSG_ID_14                                 0x1DU                       /*!< (1D, Optional) Message ID #14 */
#define C_MSG_ID_15                                 0x1EU                       /*!< (1E, Optional) Message ID #15 */
#define C_MSG_ID_16                                 0x1FU                       /*!< (1F, Optional) Message ID #16 (MMP160909-1) */
#if (LINPROT == LIN22_SIMPLE_PCT)
#define C_USER_CALIB_STATE                          0x21U                       /*!< (33, User defined) Calibration State */
#define C_USER_SPECIAL_POSITIONS                    0x22U                       /*!< (34, User defined) Special Positions */
#define C_USER_ACTUATOR_TYPE                        0x23U                       /*!< (35, User defined) Actuator Type */
#elif (LINPROT == LIN2X_HVAC49) || (LINPROT == LIN2X_HVAC52) || (LINPROT == LIN2X_AIRVENT12) || (LINPROT == LIN2X_AGS)
#define C_HVAC4x_VERIFY_NAD                         0x21U                       /*!< (33, user defined) Verify NAD */
#define C_HVAC4x_SW_HW_REF                          0x2AU                       /*!< (42, user defined) S/W & H/W Reference */
#endif /* (LINPROT == LIN2X_HVAC49) || (LINPROT == LIN2X_HVAC52) || (LINPROT == LIN2X_AIRVENT12) */
#define C_LIN_CUST_ID                               0x3DU                       /*!< (61, User defined) Customer Identification */
#define C_PROD_DATE                                 0x3EU                       /*!< (62, User defined) production Date */
#define C_PCI_CC_NAD                            0x06U                           /*!< LEN of Conditional change NAD */
#define C_SID_CC_NAD                            0xB3U                           /*!< (Optional) Conditional change NAD */
#define C_PCI_SID_CC_NAD                        0x06B3U                         /*!< (Optional) Conditional change NAD */
#define C_PCI_DATA_DUMP                         0x06U                           /*!< LEN of Data dump */
#define C_SID_DATA_DUMP                         0xB4U                           /*!< (Optional) Data dump */
#define C_PCI_SID_DATA_DUMP                     0x06B4U                         /*!< (Optional) Data dump */
#define C_PCI_STOP_ACTUATOR                     0x06U                           /*!< LEN of Stop (all) actuators */
#define C_SID_STOP_ACTUATOR                     0xB5U                           /*!< Stop (all) actuators */
#define C_PCI_SID_STOP_ACTUATOR                 0x06B5U                         /*!< Stop (all) actuators */
#define C_PCI_ASSIGN_NAD                        0x06U                           /*!< LEN of Assign NAD (Bus Shunt Method) */
#define C_SID_ASSIGN_NAD                        0xB5U                           /*!< Assign NAD (Bus Shunt Method) */
#define C_PCI_SID_ASSIGN_NAD                    0x06B5U                         /*!< Assign NAD (Bus Shunt Method) */
#if (_SUPPORT_ISO17987)
#define C_SID_ASSIGN_NAD_ISO                    0xB8U                           /*!< Assign NAD (Bus Shunt Method) (ISO17987) */
#define C_PCI_SID_ASSIGN_NAD_ISO                0x06B8U                         /*!< Assign NAD (Bus Shunt Method) (ISO17987) */
#endif /* (_SUPPORT_ISO17987) */
/* #define C_SNPD_METHOD_BSM                        0x02U */                    /*!< SNPD Method: Bus Shunt Method */
#define C_SNPD_METHOD_BSM2                          0xF1U                       /*!< HVAC 4.9/5.2 SNPD Method: Bus Shunt Method 2 */
#define C_SNPD_SUBFUNC_INACTIVE                     0x00U                       /*!< SNPD inactive */
#define C_SNPD_SUBFUNC_START                        0x01U                       /*!< Start of SNPD */
#define C_SNPD_SUBFUNC_ADDR                         0x02U                       /*!< Address last not-addressed NAD */
#define C_SNPD_SUBFUNC_STORE                        0x03U                       /*!< Store new NAD */
#define C_SNPD_SUBFUNC_FINISH                       0x04U                       /*!< Finish SNPD */
/* #define M_SNPD_SUBFUNC                           0x07U */                    /*!< Sub-function mask */
#define C_PCI_TARGETED_RESET                    0x01U                           /*!< LEN of Targeted Reset */
#define C_SID_TARGETED_RESET                    0xB5U                           /*!< Targeted Reset */
#define C_PCI_SID_TARGETED_RESET                0x01B5U                         /*!< Targeted Reset */
#define C_PCI_SAVE_CONFIG                       0x01U                           /*!< LEN of Save configuration */
#define C_SID_SAVE_CONFIG                       0xB6U                           /*!< (Optional) Save configuration */
#define C_PCI_SID_SAVE_CONFIG                   0x01B6U                         /*!< PCI-SID of  Save configuration */
#define C_PCI_ASSIGN_FRAME_ID_RNG               0x06U                           /*!< LEN of Assign frame ID range */
#define C_SID_ASSIGN_FRAME_ID_RNG               0xB7U                           /*!< (Mandatory) Assign frame ID range */
#define C_PCI_SID_ASSIGN_FRAME_ID_RNG           0x06B7U                         /*!< (Mandatory) Assign frame ID range */

#define C_SID_WRITE_BY_ID                       0xCBU                           /*!< (Optional) Write-by-ID */
#define C_PCI_WRITE_BY_ID                           0x06U                       /*!< LEN of Write-by-ID */
#define C_PCI_SID_WRITE_BY_ID                   0x06CBU                         /*!< (Optional) Write-by-ID */
#define C_RPCI_WRITE_BY_ID                      0x0BU                           /*!< Return PCI Write-by-ID */
#define C_RSID_WRITE_BY_ID                          0x06U                       /*!< Return LEN Write-by-ID */

/* *** MELEXIS *** */
#define C_SID_MLX_DEBUG                         0xDBU                           /*!< Debug Support */
#define C_DBG_SUBFUNC_SUPPORT                       0x00U                       /*!< Support 0xA0-0xFE */
#define C_DBG_SUBFUNC_SUPPORT_0                     0xFFF1U                     /*!< F, E, D, C, B, A, 9, 8, 7, 6, 5, 4, 0 */
#define C_DBG_SUBFUNC_SUPPORT_1                     0xFFFFU                     /*!< F, E, D, C, B, A, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0 */
#define C_DBG_SUBFUNC_SUPPORT_2                     0xFFFFU                     /*!< F, E, D, C, B, A, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0 */
#define C_DBG_SUBFUNC_SUPPORT_3                     0xFFFFU                     /*!< F, E, D, C, B, A, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0 */
#define C_DBG_SUBFUNC_SUPPORT_4                     0xFFFFU                     /*!< F, E, D, C, B, A, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0 */
#define C_DBG_SUBFUNC_SUPPORT_5                     0x2FFFU                     /*!< D, B, A, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0 */
#define C_DBG_SUBFUNC_SUPPORT_A                     0x7CFEU                     /*!< E, D, C, B, A, 7, 6, 5, 4, 3, 2, 1 */
#define C_DBG_SUBFUNC_SUPPORT_B                     0x007FU                     /*!< 6, 5, 4, 3, 2, 1, 0 */
#define C_DBG_SUBFUNC_SUPPORT_C                     0xFFC7U                     /*!< F, E, D, C, B, A, 9, 8, 7, 6, 2, 1, 0 */
#if (_DEBUG_MOTOR_CURRENT_FLT != FALSE)
#define C_DBG_SUBFUNC_SUPPORT_D                     0xD100U                     /*!< F, E, C, 8 */
#else  /* (_DEBUG_MOTOR_CURRENT_FLT != FALSE) */
#define C_DBG_SUBFUNC_SUPPORT_D                     0xC100U                     /*!< F, E, 8 */
#endif /* (_DEBUG_MOTOR_CURRENT_FLT != FALSE) */
#define C_DBG_SUBFUNC_SUPPORT_E                     0x3000U                     /*!< D, C */
#define C_DBG_SUBFUNC_SUPPORT_F                     0x7D00U                     /*!< E, D, C, B, A, 8 */
#define C_DBG_SUBFUNC_NV_WR_IDX0                    0x04U                       /*!< EE Write Index '0' */
#define C_DBG_SUBFUNC_NV_WR_IDX_MAX                 0x5BU                       /*!< Maximum EE Write Index */
#define C_DBG_SUBFUNC_STALLDET                      0x5DU                       /*!< Stall Detector */
#if (_SUPPORT_LIN_AA != FALSE)
#define C_DBG_SUBFUNC_LINAA_1                       0xA1U                       /*!< LIN-AA BSM Ishunt #1,2 & 3 and flags */
#define C_DBG_SUBFUNC_LINAA_2                       0xA2U                       /*!< LIN-AA BSM Common-mode & Differential-mode levels #1 */
#define C_DBG_SUBFUNC_LINAA_3                       0xA3U                       /*!< LIN-AA BSM Common-mode & Differential-mode levels #2 */
#define C_DBG_SUBFUNC_LINAA_4                       0xA4U                       /*!< LIN-AA BSM Common-mode & Differential-mode levels #3 */
#endif /* (_SUPPORT_LIN_AA != FALSE) */
#define C_DBG_SUBFUNC_APPLSTATE                     0xA5U                       /*!< Application Status */
#define C_DBG_SUBFUNC_LIN_BAUDRATE                  0xA6U                       /*!< LIN Slave baudrate */
#define C_DBG_SUBFUNC_RESTART_AUTO_BAUDRATE         0xA7U                       /*!< Restart Auto baud-rate detection */
#if (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
#define C_DBG_SUBFUNC_RESOLVER                      0xAAU                       /*!< Triaxis/Resolver MLX90380 Analogue data */
#define CMD_MASK_RESOLVER                               0x07U                   /*!< Bit 2:0 */
#define CMD_RESOLVER_GET_XY                             0x00U                   /*!< Get X/Y data */
#define CMD_RESOLVER_CALIB                              0x01U                   /*!< Calibrate resolver */
#define CMD_RESOLVER_CALIB_RESULT                       0x02U                   /*!< Calibration Result (Resolver) */
#define CMD_RESOLVER_CALIB_RESULT_ACT                   0x03U                   /*!< Calibration Result (Resolver vs. Actuator) */
#endif /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */
#define C_DBG_SUBFUNC_HALL_LATCH                    0xABU                       /*!< Hall-Latch low and high period */
#define C_DBG_SUBFUNC_SPEED_INFO                    0xACU                       /*!< Speed info */
#define C_DBG_SUBFUNC_SPEED                         0xADU                       /*!< Target vs. Actual speed [RPM] */
#define C_DBG_SUBFUNC_AMBJENV                       0xAEU                       /*!< Ambient Environment */
#define C_DBG_SUNFUNC_PWM_IN                        0xAFU                       /*!< PWM-In period & duty-cycle */
#define C_DBG_SUBFUNC_FOC_IV                        0xB0U                       /*!< FOC: I/V-angle */
#define C_DBG_SUBFUNC_SET_MPWM_DC                   0xB0U                       /*!< Set Motor PWM Duty Cycle */
#define C_DBG_SUBFUNC_SET_DRV_SR                    0xB1U                       /*!< Set Motor Driver Slew-rate */
#define C_DBG_SUBFUNC_SET_CPU_FREQ                  0xB2U                       /*!< Set CPU Frequency */
#define C_DBG_SUBFUNC_SET_CPU_SSCM                  0xB3U                       /*!< Set CPU Clock Spread-Spectrum */
#define C_DBG_SUBFUNC_SET_LIN_SR                    0xB4U                       /*!< Set LIN Slew-rate */
#define C_DBG_SUBFUNC_SET_DRV_SSCM                  0xB5U                       /*!< Set Driver Charge-pump Spread-Spectrum */
#define C_DBG_SUBFUNC_SET_LIN_TERM                  0xB6U                       /*!< Set LIN Termination */
#define C_DBG_SUBFUNC_MLX16_CLK                     0xC0U                       /*!< MLX16 Clock */
#define C_DBG_SUBFUNC_CHIPID                        0xC1U                       /*!< Chip ID */
#define C_DBG_SUBFUNC_HWSWID                        0xC2U                       /*!< HW/SW ID */
#if (_SUPPORT_MLX_DEBUG_OPTIONS != FALSE)
#define C_DBG_SUBFUNC_DEBUG_OPTIONS                 0xC5U                       /*!< Get _DEBUG options */
#endif /* (_SUPPORT_MLX_DEBUG_OPTIONS != FALSE) */
#define C_DBG_SUBFUNC_SUPPORT_OPTIONS               0xC6U                       /*!< Get _SUPPORT options */
#define C_DBG_SUBFUNC_MLX4_VERSION                  0xC7U                       /*!< MLX4 F/W & Loader */
#define C_DBG_SUBFUNC_PLTF_VERSION                  0xC8U                       /*!< Get Platform version */
#define C_DBG_SUBFUNC_APP_VERSION                   0xC9U                       /*!< Get application version */
#define C_DBG_SUBFUNC_MLXPAGE                       0xCAU                       /*!< Melexis Calibration page */
#define C_DBG_SUBFUNC_MLXPID                        0xCBU                       /*!< Get PID info */
#define C_DBG_SUBFUNC_NV_ERRORCODES                 0xCCU                       /*!< EE error-codes */
#define C_DBG_SUBFUNC_CLR_NV_ERRORCODES             0xCDU                       /*!< Clear EE error-codes */
#define C_DBG_SUBFUNC_CHIPENV                       0xCEU                       /*!< Chip Environment */
#define C_DBG_SUBFUNC_FUNC                          0xCFU                       /*!< Chip (Reset) Function */
#define C_DBG_DBGFUNC_RESET     ((('R' - '@') << 10) | (('S' - '@') << 5) | ('T' - '@'))  /*!< Module Reset */
#define C_DBG_DBGFUNC_ENTER_EPM ((('E' - '@') << 10) | (('P' - '@') << 5) | ('M' - '@'))  /*!< Enter EPM mode */
#define C_DBG_DBGFUNC_LOCK_PPM  ((('L' - '@') << 10) | (('P' - '@') << 5) | ('B' - '@'))  /*!< Lock PPM Bootloader (ROM-patch) */
#define C_DBG_DBGFUNC_UNLOCK_PPM ((('U' - '@') << 10) | (('P' - '@') << 5) | ('B' - '@')) /*!< Unlock PPM Bootloader (ROM-patch) */
#define C_DBG_DBGFUNC_FAST_WU   ((('F' - '@') << 10) | (('W' - '@') << 5) | ('U' - '@'))  /*!< Fast Wake-up (ROM-patch) */
#define C_DBG_SUBFUNC_SET_ANAOUTA                   0xD0U                       /*!< Set ANA_OUTA */
#define C_DBG_SUBFUNC_SET_ANAOUTB                   0xD1U                       /*!< Set ANA_OUTB */
#define C_DBG_SUBFUNC_SET_ANAOUTC                   0xD2U                       /*!< Set ANA_OUTC */
#define C_DBG_SUBFUNC_SET_ANAOUTD                   0xD3U                       /*!< Set ANA_OUTD */
#define C_DBG_SUBFUNC_SET_ANAOUTE                   0xD4U                       /*!< Set ANA_OUTE */
#define C_DBG_SUBFUNC_SET_ANAOUTF                   0xD5U                       /*!< Set ANA_OUTF */
#define C_DBG_SUBFUNC_SET_ANAOUTG                   0xD6U                       /*!< Set ANA_OUTG */
#define C_DBG_SUBFUNC_SET_ANAOUTH                   0xD7U                       /*!< Set ANA_OUTH */
#define C_DBG_SUBFUNC_SET_MICROSTEP                 0xD8U                       /*!< Set Micro-step index */
#if (_SUPPORT_ADC_BGD != FALSE)
#define C_DBG_SUBFUNC_VAUX_VBGD                     0xD9U                       /*!< Get VAUX and VBGD (MMP220307-1) */
#endif /* (_SUPPORT_ADC_BGD != FALSE) */
#if (_SUPPORT_ADC_VDDA_VDDD != FALSE)
#define C_DBG_SUBFUNC_VDDA_VDDD                     0xDAU                       /*!< Get VDDA and VDDD */
#endif /* (_SUPPORT_ADC_VDDA_VDDD != FALSE) */
#if (_SUPPORT_ADC_VBOOST != FALSE)
#define C_DBG_SUBFUNC_VBOOST                        0xDBU                       /*!< Get VBOOST */
#elif (_SUPPORT_IO_DUT_SELECT_HVIO != FALSE) && (HVIO_DUT_SELECT == PIN_FUNC_IO_0)
#define C_DBG_SUBFUNC_VIO0HV                        0xDBU                       /*!< Get VIO0HV */
#endif /* _SUPPORT_ADC_VBOOST */
#if (_DEBUG_MOTOR_CURRENT_FLT != FALSE)
#define C_DBG_MOTOR_CURR_RAW                        0xDCU                       /*!< Get debug-bugger contents "Motor Current" */
#endif /* (_DEBUG_MOTOR_CURRENT_FLT != FALSE) */
#if (_SUPPORT_HALL_LATCH_DIAG != FALSE)
#define C_DBG_MOTOR_MICRO_STEP                      0xDCU                       /*!< Get Hall-Latch state per micro-step */
#endif /* (_SUPPORT_HALL_LATCH_DIAG != FALSE) */
#define C_DBG_RD_TMTR                               0xDEU                       /*!< Read TM_TR register */
#define C_DBG_WR_TMTR                               0xDFU                       /*!< Write TM_TR register */
#if (_SUPPORT_FLASH_PRODUCTION_DATA != FALSE)
#define C_DBG_SUBFUNC_ERASE_PRODUCTION_DATA         0xEAU                       /*!< Erase Production Data block(s) */
#define C_DBG_SUBFUNC_WRITE_PRODUCTION_DATA_PAGE    0xEBU                       /*!< Write Production Data Page */
#endif /* (_SUPPORT_FLASH_PRODUCTION_DATA != FALSE) */
#define C_DBG_SUBFUNC_ERRORCODES                    0xECU                       /*!< (New) replace C_SID_MLX_ERROR_CODES */
#define C_DBG_SUBFUNC_NV_READ                       0xEDU                       /*!< Non Volatile Memory Read */
#if (_SUPPORT_START_TO_RUN != FALSE)
#define C_DBG_SUBFUNC_START2RUN                     0xEFU                       /*!< Start-to-Run functions */
#define C_S2R_SUBFUNC_INIT                              0x01U                   /*!< Start-to-Run Initialisation */
#define C_S2R_SUBFUNC_PRE_CHECK                         0x02U                   /*!< Start-to-Run Pre-check */
#define C_S2R_SUBFUNC_MOTOR_CONFIG                      0x03U                   /*!< Start-to-Run Motor coil configuration */
#define C_S2R_SUBFUNC_COIL_PARAMS                       0x04U                   /*!< Start-to-Run Motor coil Parameters (R/L) */
#define C_S2R_SUBFUNC_MOTOR_CONST                       0x05U                   /*!< Start-to-Run Motor Constant (BEMF) */
#if FALSE
#define C_S2R_SUBFUNC_SET_MOTOR_R                       0x06U                   /*!< Start-to-Run Set application motor coil resistance (Base:5, Exp:3) * 10mR (80R, max) */
#define C_S2R_SUBFUNC_SET_MOTOR_L                       0x07U                   /*!< Start-to-Run Set application motor coil inductance (Base:5, Exp:3) * 5uH (40mH, max) */
#endif /* FALSE */
#define C_S2R_SUBFUNC_STATUS                            0x0DU                   /*!< Start-to-Run Status */
#define C_S2R_SUBFUNC_ABORT                             0x0EU                   /*!< Start-to-Run Abort */
#define C_S2R_SUBFUNC_EXIT                              0x0FU                   /*!< Start-to-Run Exit */
#endif /* (_SUPPORT_START_TO_RUN != FALSE) */
#define C_DBG_SUBFUNC_FILLNVRAM                     0xF8U                       /*!< Fill NVRAM */
#define C_DBG_SUBFUNC_SET_IO_VALUE                  0xFAU                       /*!< Write I/O-value (to RAM) (MMP170715-1) */
#define C_DBG_SUBFUNC_SET_IO_REG                    0xFBU                       /*!< Write I/O-register (from RAM) (MMP170715-1) */
#define C_DBG_SUBFUNC_CLR_FATAL_ERRORCODES          0xFCU                       /*!< Clear Fatal-error logging */
#define C_DBG_SUBFUNC_GET_IO_REG                    0xFDU                       /*!< Read I/O-register */
#define C_DBG_SUBFUNC_FATAL_ERRORCODES              0xFEU                       /*!< Fatal-error logging */
/*
 *      +-----+-----+------+----+----+----+----+----+
 *  SF  | NAD | PCI | RSID | D1 | D2 | D3 | D4 | D5 |
 *      +-----+-----+------+----+----+----+----+----+
 *           /       \
 *           7654 3210
 *  SF-Type: 0000 Length
 *
 *      +-----+-----+------+----+----+----+----+----+
 * NOK  | NAD | 0x03| 0x7F | SID|0x12|0xFF|0xFF|0xFF|
 *      +-----+-----+------+----+----+----+----+----+
 */
#define C_RSID_OK                               0x40U                           /*!< Positive feedback: SID | 0x40 */
#define C_RPCI_NOK                              0x03U                           /*!< Negative feedback length */
#define C_RSID_NOK                              0x7FU                           /*!< Negative feedback RSID */
/* Negative Response Codes */
#define C_ERRCODE_POSITIVE_RESPONSE                 0x00U                       /*!< Error-code: Positive Response */
#define C_ERRCODE_GENERAL_REJECT                    0x10U                       /*!< Error-code: General reject */
#define C_ERRCODE_SERVNOSUP                         0x11U                       /*!< Error-code: Service not supported */
#define C_ERRCODE_SFUNC_NOSUP                       0x12U                       /*!< Error-code: Sub-function not supported */
#define C_ERRCODE_INV_MSG_INV_SZ                    0x13U                       /*!< Error-code: Incorrect message length or invalid format */
#define C_ERRCODE_RESPONSE_TOO_LONG                 0x14U                       /*!< Error-code: Response too long */
#define C_ERRCODE_BUSY_REP_REQ                      0x21U                       /*!< Error-code: Busy repeat request */
#define C_ERRCODE_COND_SEQ                          0x22U                       /*!< Error-code: Conditions Not Correct */
#define C_ERRCODE_REQ_SEQ                           0x24U                       /*!< Error-code: Request Sequence error */
#define C_ERRCODE_NO_RESP_FROM_SUBNET               0x25U                       /*!< Error-code: No response from sub-net component */
#define C_ERRCODE_FAIL_PREV_EXE_REQ                 0x26U                       /*!< Error-code: Failure prevents execution of requested action */
#define C_ERRCODE_REQ_OUT_OF_RANGE                  0x31U                       /*!< Error-code: Request out of range */
#define C_ERRCODE_SEC_ACC_DENIED                    0x33U                       /*!< Error-code: Security access denied */
#define C_ERRCODE_INV_KEY                           0x35U                       /*!< Error-code: Invalid Key */
#define C_ERRCODE_EXEED_NR_ATTEMPTS                 0x36U                       /*!< Error-code: Exceed number of attempts (to get security access) */
#define C_ERRCODE_REQ_TIME_DLY_NOT_EXP              0x37U                       /*!< Error-code: Required time delay not expired */
#define C_ERRCODE_UP_DOWNLOAD_NOT_ACC               0x70U                       /*!< Error-code: Upload/Download not accepted */
#define C_ERRCODE_TRANS_DATA_SUSP                   0x71U                       /*!< Error-code: Transfer data suspended */
#define C_ERRCODE_GEN_PROG_FAIL                     0x72U                       /*!< Error-code: General Programming Failure */
#define C_ERRCODE_WRONG_BLOCK_SEQ_CNT               0x73U                       /*!< Error-code: Wrong block sequence counter */
#define C_ERRCODE_PENDING                           0x78U                       /*!< Error-code: Request Correctly Received / Response Pending */
#define C_ERRCODE_SUBFUNC_NOT_SUPP_IN_ACT_SESSION   0x7EU                       /*!< Error-code: Sub-function not supported in active session */
#define C_ERRCODE_SERV_NOT_SUPP_IN_ACT_SESSION      0x7FU                       /*!< Error-code: Service not supported in active session */
/* UDS Service */
#define C_RPCI_DIAG_SESSION_CTRL                0x06U                           /*!< Return PCI LEN Diagnose Session*/
#define C_RSID_DIAG_SESSION_CTRL                (C_SID_DIAG_SESSION_CTRL | C_RSID_OK)  /*!< Return  SID for Diagnose Session*/
#define C_P2_CAN_SERVER_MAX                     200U                            /*!< P2-CAN Server Max */
#define C_P2_CAN_SERVER_MAX_EX                  700U                            /*!< P2-CAN Server Max Ex */
#define C_RPCI_ECU_RESET                        0x02U                           /*!< Return PCI LEN Reset */
#define C_RPCI_ECU_RESET_04                     0x03U                           /*!< Return PCI LEN Reset */
#define C_RSID_ECU_RESET                        (C_SID_ECU_RESET | C_RSID_OK)   /*!< Return-SID Reset */
#define C_TIME_TO_RAPID_POWER_DOWN              100U                            /*!< Time to Rapid power-down */
#define C_RSID_READ_DATA_BY_IDENTIFY            (C_SID_READ_DATA_BY_IDENTIFY | C_RSID_OK)  /*!< Return-SID Read Data by ID */
#define C_RPCI_WRITE_DATA_BY_IDENTIFY           0x03U                           /*!< Return PCI LEN Write data by Identifier (UDS) */
#define C_RSID_WRITE_DATA_BY_IDENTIFY           (C_SID_WRITE_DATA_BY_IDENTIFY | C_RSID_OK)  /*!< Return-SID Write Data by ID */
#define C_RPCI_TESTER_PRESENT                   0x02U                           /*!< Return PCI LEN Tester present */
#define C_RSID_TESTER_PRESENT                   (C_SID_TESTER_PRESENT | C_RSID_OK) /*!< Return-SID Tester Present */
/* LIN 2.x Service */
#define C_RPCI_REASSIGN_NAD                     0x01U                           /*!< Return LEN Reassign NAD */
#define C_RSID_REASSIGN_NAD                     (C_SID_REASSIGN_NAD | C_RSID_OK)  /*!< Return (positive) SID Reassign NAD */
#define C_RPCI_READ_BY_ID_00                    0x06U                           /*!< Response-PCI: LIN Product Identification */
#define C_RPCI_READ_BY_ID_01                    0x05U                           /*!< Response-PCI: Serial number */
#if (_SUPPORT_ISO17987)
#define C_RPCI_READ_BY_ID_03                    0x06U                           /*!< Response-PCI: NCD/LDF Version info */
#endif /* (_SUPPORT_ISO17987) */
#define C_RPCI_READ_BY_ID_1X                    0x04U                           /*!< Response-PCI: (Optional) Message ID's (MMP160909-1) */
#define C_RPCI_READ_BY_ID_20                    0x04U                           /*!< Response-PCI: NAD-information */
#if (LINPROT == LIN22_SIMPLE_PCT)
#define C_RPCI_READ_BY_ID_21                    0x04U                           /*!< Response-PCI: Emergency Run position */
#define C_RPCI_READ_BY_ID_22                    0x03U                           /*!< Response-PCI: Get LIN Frame PID's */
#elif (LINPROT == LIN2X_HVAC49) || (LINPROT == LIN2X_HVAC52) || (LINPROT == LIN2X_AIRVENT12) || (LINPROT == LIN2X_AGS)
#define C_RPCI_READ_BY_ID_21                    0x04U                           /*!< Response-PCI: Verify NAD */
#define C_RPCI_READ_BY_ID_2A                    0x03U                           /*!< Response-PCI: SW and HW reference */
#endif /* (LINPROT == LIN2X_HVAC49) || (LINPROT == LIN2X_HVAC52) || (LINPROT == LIN2X_AIRVENT12) */
#define C_RSID_READ_BY_ID                       (C_SID_READ_BY_ID | C_RSID_OK)  /*!< Return SID (positive) Ready by ID */
#define C_RPCI_ASSIGN_FRAME_ID                  0x01U                           /*!< Return LEN Assign Message-ID to Frame-ID */
#define C_RSID_ASSIGN_FRAME_ID                  (C_SID_ASSIGN_FRAME_ID | C_RSID_OK)  /*!< Return SID (positive) Assign Message-ID to Frame-ID */
#define C_RPCI_CC_NAD                           0x01U                           /*!< Return LEN Conditional Change NAD */
#define C_RSID_CC_NAD                           (C_SID_CC_NAD | C_RSID_OK)      /*!< Return SID (positive) Conditional Change NAD */
#define C_RPCI_DATA_DUMP                        0x06U                           /*!< Return LEN Data Dump */
#define C_RPCI_ASSIGN_VARIANT_ID                0x01U                           /*!< Return LEN Assign Variant ID */
#define C_RSID_DATA_DUMP                        (C_SID_DATA_DUMP | C_RSID_OK)   /*!< Return SID (positive) Data Dump */
#define C_RPCI_ASSIGN_NAD                       0x01U                           /*!< Return LEN Assign NAD */
#define C_RSID_ASSIGN_NAD                       (C_SID_ASSIGN_NAD | C_RSID_OK)  /*!< Return SID (positive) Assign NAD */
#define C_RPCI_STOP_ACTUATOR                    0x01U                           /*!< Return LEN Stop Actuators */
#define C_RSID_STOP_ACTUATOR                    (C_SID_STOP_ACTUATOR | C_RSID_OK)  /*!< Return SID (positive) Stop Actuators */
#define C_RPCI_SAVE_CONFIG                      0x01U                           /*!< Return LEN Save Configuration */
#define C_RSID_SAVE_CONFIG                      (C_SID_SAVE_CONFIG | C_RSID_OK)  /*!< Return SID (positive) save Configuration */
#define C_RPCI_ASSIGN_FRAME_ID_RNG              0x01U                           /*!< Return LEN Assign Frame ID range */
#define C_RSID_ASSIGN_FRAME_ID_RNG              (C_SID_ASSIGN_FRAME_ID_RNG | C_RSID_OK)  /*!< Return SID (positive) Assign frame ID range */
#define C_DIAG_RES                              0xFFU                           /*!< Reserved fields feedback */
#define C_RSID_MLX_DEBUG                        ((uint8_t)(C_SID_MLX_DEBUG + C_RSID_OK))   /*!< Return SID (Positive) Melexis Diagnostics */

/* UDS */
/* VW80125_Ident_el_Fahrzeugsys_V4.0_2010_04.pdf */
#if (_SUPPORT_UDS_DK >= _SUPPORT_UDS_DK1)
#define C_VW_FAZIT_STRING                   0xF17CU                             /*!< VWSlaveFazitString */
#define C_VW_SPAREPART_NUMBER               0xF187U                             /*!< VWSlaveSparePartNumber */
#define C_VW_SOFTWARE_VERSION               0xF189U                             /*!< VWSoftwareVersion */
#define C_VW_ECU_SERIAL_NUMBER              0xF18CU                             /*!< VWEcuSerialNumber */
#define C_VW_ECU_HW_NUMBER                  0xF191U                             /*!< VWSlaveSerialNumber */
#define C_VW_SYSTEM_NAME                    0xF197U                             /*!< VWSlaveHardwareVersionNumber */
#define C_VW_ECU_HW_VERSION                 0xF1A3U                             /*!< VWEcuHardwareVersion */
#if (_SUPPORT_UDS_DK >= _SUPPORT_UDS_DK4)
#if (_SUPPORT_UDS_READ_BY_ID_SUBSYS != FALSE)
#define C_ID_SUB_SYSTEM_NR                  0x0608U                             /*!< IdentifiedSubSystemNumbers 0x0608 (NEW) */
#define C_SUBSYS_MASK                       0x01FFU
#define C_SLAVE_ID_MASK                     0xFE00U
#define C_SLAVE_SPAREPART_NR                0x6200U                             /*!< VWSlaveSparePartNumber 0x6200 - 0x63FF */
#define C_SLAVE_APPL_SW_VERSION_NR          0x6400U                             /*!< VWSlaveApplicationSoftwareVersionNumber 0x6400 - 0x65FF */
#define C_SLAVE_HW_NR                       0x6600U                             /*!< VWSlaveHardwareNumber 0x6600 - 0x67FF */
#define C_SLAVE_HW_VERSION_NR               0x6800U                             /*!< VWSlaveHardwareVersionNumber 0x6800 - 0x69FF */
#define C_SLAVE_SERIAL_NR                   0x6A00U                             /*!< VWSlaveSerialNumber 0x6A00 - 0x6BFF */
#define C_SLAVE_SYSTEM_NAME                 0x6C00U                             /*!< VWSlaveSystemName 0x6C00 - 0x6DFF */
#define C_SLAVE_FAZIT_STRING                0x6E00U                             /*!< VWSlaveFazitString 0x6E00 - 0x6FFF */
#else /* Old SLAVE SYSTEM Read-Data-by-ID */
#define C_ID_SLAVE_SYSTEM                   0x0606U                             /*!< IdentifiedSlaveSystems 0x0606 (OLD) */
#define C_SLAVE_SPAREPART_NR                0x0640U                             /*!< VWSlaveSparePartNumber 0x0640 - 0x066F */
#define C_SLAVE_SPAREPART_NR_E              0x066FU                             /*!< VWSlaveSparePartNumber 0x0640 - 0x066F */
#define C_SLAVE_APPL_SW_VERSION_NR          0x0670U                             /*!< VWSlaveApplicationSoftwareVersionNumber 0x0670 - 0x069F */
#define C_SLAVE_APPL_SW_VERSION_NR_E        0x069FU                             /*!< VWSlaveApplicationSoftwareVersionNumber 0x0670 - 0x069F */
#define C_SLAVE_HW_NR                       0x06A0U                             /*!< VWSlaveHardwareNumber 0x06A0 - 0x06CF */
#define C_SLAVE_HW_NR_E                     0x06CFU                             /*!< VWSlaveHardwareNumber 0x06A0 - 0x06CF */
#define C_SLAVE_HW_VERSION_NR               0x06D0U                             /*!< VWSlaveHardwareVersionNumber 0x06D0 - 0x06FF */
#define C_SLAVE_HW_VERSION_NR_E             0x06FFU                             /*!< VWSlaveHardwareVersionNumber 0x06D0 - 0x06FF */
#define C_SLAVE_SERIAL_NR                   0x0700U                             /*!< VWSlaveSerialNumber 0x0700 - 0x072F */
#define C_SLAVE_SERIAL_NR_E                 0x072FU                             /*!< VWSlaveSerialNumber 0x0700 - 0x072F */
#define C_SLAVE_SYSTEM_NAME                 0x0730U                             /*!< VWSlaveSystemName 0x0730 - 0x075F */
#define C_SLAVE_SYSTEM_NAME_E               0x075FU                             /*!< VWSlaveSystemName 0x0730 - 0x075F */
#define C_SLAVE_FAZIT_STRING                0x07A0U                             /*!< VWSlaveFazitString 0x07A0 - 0x07CF */
#define C_SLAVE_FAZIT_STRING_E              0x07CFU                             /*!< VWSlaveFazitString 0x07A0 - 0x07CF */
#endif /* (_SUPPORT_UDS_READ_BY_ID_SUBSYS != FALSE) */
#endif /* (_SUPPORT_UDS_DK >= _SUPPORT_UDS_DK4) */
#endif /* (_SUPPORT_UDS_DK >= _SUPPORT_UDS_DK1) */

#define QR_RFR_DIAG                         7U                                  /*!< Single-frame RFR Diagnostics response */
#define QR_RFR_DIAG_MF                      8U                                  /*!< Multi-frame RFR Diagnostics response */
#define QR_INVALID                          0xFFU                               /*!< Diagnostics response invalid */

/*! LIN Diagnostics Demand Frame layout */
typedef struct _DFR_DIAG                                                        /*!< Description of DFR_DIAGNOSTIC LIN-Frame */
{
    uint8_t byNAD;                                                              /**< NAD field */
    uint8_t byPCI;                                                              /**< PCI field */
    union
    {
        struct
        {
            uint8_t bySID;                                                      /**< SID field */
            uint8_t byD1;                                                       /**< Data_1 field */
            uint8_t byD2;                                                       /**< Data_2 field */
            uint8_t byD3;                                                       /**< Data_2 field */
            uint8_t byD4;                                                       /**< Data_3 field */
            uint8_t byD5;                                                       /**< Data_5 field */
        } SF;                                                                   /**< Single Frame structure */
        struct
        {
            uint8_t byLEN;                                                      /**< LENgth field */
            uint8_t bySID;                                                      /**< SID field */
            uint8_t byD1;                                                       /**< Data_1 field */
            uint8_t byD2;                                                       /**< Data_2 field */
            uint8_t byD3;                                                       /**< Data_3 field */
            uint8_t byD4;                                                       /**< Data_4 field */
        } FF;                                                                   /**< First Frame structure */
        struct
        {
            uint8_t byD1;                                                       /**< Data_1 field */
            uint8_t byD2;                                                       /**< Data_2 field */
            uint8_t byD3;                                                       /**< Data_3 field */
            uint8_t byD4;                                                       /**< Data_4 field */
            uint8_t byD5;                                                       /**< Data_5 field */
            uint8_t byD6;                                                       /**< Data_6 field */
        } CF;                                                                   /**< Continuous Frame structure */
    } u;                                                                        /**< LIN Frame union */
} __attribute__((packed)) DFR_DIAG;

/*! LIN Diagnostics Response Frame layout */
typedef struct _RFR_DIAG                                                        /*!< Description of RDR_DIAGNOSTIC LIN-Frame */
{
    uint8_t byNAD;                                                              /**< NAD field */
    uint8_t byPCI;                                                              /**< PCI field */
    union
    {
        struct
        {
            uint8_t byRSID;                                                     /**< Return-SID field */
            uint8_t byD1;                                                       /**< Data_1 field */
            uint8_t byD2;                                                       /**< Data_2 field */
            uint8_t byD3;                                                       /**< Data_3 field */
            uint8_t byD4;                                                       /**< Data_4 field */
            uint8_t byD5;                                                       /**< Data_5 field */
        } SF;                                                                   /**< Single Frame structure */
        struct
        {
            uint8_t byLEN;                                                      /**< LENgth field */
            uint8_t byRSID;                                                      /**< Return-SID field */
            uint8_t byD1;                                                       /**< Data_1 field */
            uint8_t byD2;                                                       /**< Data_2 field */
            uint8_t byD3;                                                       /**< Data_3 field */
            uint8_t byD4;                                                       /**< Data_4 field */
        } FF;                                                                   /**< First Frame structure */
        struct
        {
            uint8_t byD1;                                                       /**< Data_1 field */
            uint8_t byD2;                                                       /**< Data_2 field */
            uint8_t byD3;                                                       /**< Data_3 field */
            uint8_t byD4;                                                       /**< Data_4 field */
            uint8_t byD5;                                                       /**< Data_5 field */
            uint8_t byD6;                                                       /**< Data_6 field */
        } CF;                                                                   /**< Continuous Frame structure */
    } u;                                                                        /**< LIN Frame union */
} __attribute__((packed)) RFR_DIAG;

#if (_SUPPORT_MLX_CHIP_STATUS != FALSE)
/*! Description of DFR_CHIP_CONTROL */
typedef struct _DFR_CHIP_CONTROL
{
    uint16_t u8PWM_DC_LSB        : 8;                                           /**< PWM DC LSB */
    uint16_t u8PWM_DC_MSB        : 8;                                           /**< PWM DC MSB */
    uint16_t u1SSCM              : 1;                                           /**< CPU Spread Spectrum On/Off */
#if FALSE
    uint16_t u3LIN_SR            : 3;                                           /**< LIN SLew-rate setting */
    uint16_t u1SSCM2             : 1;                                           /**< Charge Pump Motor Driver Spread Spectrum On/Off */
    uint16_t u3Reserved2         : 3;                                           /**< Reserved #2 */
    uint16_t u8Reserved3         : 8;                                           /**< Reserved #3 */
    uint16_t u8Reserved4         : 8;                                           /**< Reserved #4 */
#else
    uint16_t u3Reserved2a        : 3;                                           /**< Reserved #2 */
    uint16_t u1SSCM2             : 1;                                           /**< Charge Pump Motor Driver Spread Spectrum On/Off */
    uint16_t u3Reserved2b        : 3;                                           /**< Reserved #2 */
    uint16_t u8SSCM_CNT          : 8;                                           /**< SSCM Step CNT */
    uint16_t u1LIN_SR_Source     : 1;                                           /**< LIN Slew-rate Source; 0: Trim value; 1: bit 6:4 of Byte 4 */
    uint16_t u3Reserved4a        : 3;                                           /**< Reserved #4 */
    uint16_t u3LIN_SR            : 3;                                           /**< LIN SLew-rate setting */
    uint16_t u1Reserved4b        : 1;                                           /**< Reserved #4 */
#endif
    uint16_t u8Reserved5         : 8;                                           /**< Reserved #5 */
    uint16_t u8Reserved6         : 8;                                           /**< Reserved #6 */
    uint16_t u8Reserved7         : 8;                                           /**< Reserved #7 */
} DFR_CHIP_CONTROL;

/*! Description of _RFR_CHIP_STATUS */
typedef struct _RFR_CHIP_STATUS                                                 /* Description of _RFR_CHIP_STATUS */
{
    uint16_t u8NAD               : 8;                                           /**< NAD */
    uint16_t u8M2S_Counter       : 8;                                           /**< Master-to-Slave counter */
    uint16_t u8IO_Status         : 8;                                           /**< I/O Status */
    uint16_t u8FlashCRC_LSB      : 8;                                           /**< Flash CRC (LSB) */
    uint16_t u8FlashCRC_MSB      : 8;                                           /**< Flash CRC (MSB) */
    uint16_t u8NvmCRC_LSB        : 8;                                           /**< Non Volatile Memory CRC (LSB) */
    uint16_t u8NvmCRC_MSB        : 8;                                           /**< Non Volatile Memory CRC (MSB) */
    uint16_t u1LinResponseError  : 1;                                           /**< LIN Communication error */
    uint16_t u1TempSensorError   : 1;                                           /**< Temperature Sensor Error */
    uint16_t u2Reserved          : 2;                                           /**< Reserved */
    uint16_t u4WakeupSource      : 4;                                           /**< Wake-up source */
} RFR_CHIP_STATUS;
#endif /* (_SUPPORT_MLX_CHIP_STATUS != FALSE) */


#define C_LOADER_CMD_NONE           0                                           /*!< No Loader Command */
#define C_LOADER_CMD_RESET          1                                           /*!< Loader Reset command (C_CHIP_STATE_CMD_RESET) */
#define C_LOADER_CMD_EPM            2                                           /*!< Loader EPM Command (C_CHIP_STATE_CMD_EPM) */

/*!*************************************************************************** */
/*                           GLOBAL VARIABLES                                  */
/* *************************************************************************** */
#pragma space dp                                                                /* __TINY_SECTION__ */
#pragma space none                                                              /* __TINY_SECTION__ */

#pragma space nodp                                                              /* __NEAR_SECTION__ */
#if (_SUPPORT_BOOTLOADER_PPM != FALSE)
extern uint8_t g_e8LoaderReset;
#endif /* (_SUPPORT_BOOTLOADER_PPM != FALSE) */
#if (_SUPPORT_UDS != FALSE)
extern UDS_LIN_PARAMS_t RAM_UDS;
#endif /* (_SUPPORT_UDS != FALSE) */
#if (_SUPPORT_MLX_CHIP_STATUS != FALSE)
extern uint8_t g_u8MlxChipStatusM2SCounter;                                     /* Melexis Chip Status: M2S Counter */
#endif /* (_SUPPORT_MLX_CHIP_STATUS != FALSE) */
#pragma space none                                                              /* __NEAR_SECTION__ */

/*!*************************************************************************** */
/*                           GLOBAL FUNCTIONS                                  */
/* *************************************************************************** */
extern void SetupDiagResponse(uint8_t u8NAD, uint8_t u8SID, uint8_t u8ResponseCode);
extern void HandleDfrDiag(void);                                                /* Diagnostics */
#if (_SUPPORT_UDS != FALSE)
extern void HandleRfrDiagMF(void);                                              /* Handle Multi-frame RFR_Diagnostics */
#endif /* (_SUPPORT_UDS != FALSE) */
#if ((LINPROT & LINX) == LIN2) || (LINPROT == LIN13_HVACTB)
extern void LinDiagResponseTimeoutCount(uint16_t u16Period);
#endif /* ((LINPROT & LINX) == LIN2) || (LINPROT == LIN13_HVACTB) */
#if (_SUPPORT_MLX_DEBUG_MODE != FALSE)
extern void RfrDiagReset(void);
#endif /* (_SUPPORT_MLX_DEBUG_MODE != FALSE) */
#if (_SUPPORT_MLX_CHIP_STATUS != FALSE)
extern void HandleMlxChipStatus(void);
extern void HandleMlxChipControl(void);
#endif /* (_SUPPORT_MLX_CHIP_STATUS != FALSE) */

/*!*************************************************************************** *
 * StoreD1to2()
 * \brief   Store 16-bit data in Diagnostic Response Frame D1/D2
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16A: 16-bit data
 * \return  -
 * *************************************************************************** *
 * \details Helper-function store 16-bit value in Diagnostic Response Frame
 *          at D1 & D2. Use g_DiagResponse-buffer
 * *************************************************************************** *
 * - Call Hierarchy: DfrDiagMlxDebug(), DfrDiagReadByIdentifier()
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 0
 * *************************************************************************** */
static inline void StoreD1to2(uint16_t u16A)
{
    extern RFR_DIAG g_DiagResponse;                                             /*!< LIN Diagnostics Response Buffer */
    extern uint8_t g_u8BufferOutID;                                             /*!< LIN output buffer is invalid */

    g_DiagResponse.u.SF.byD1 = (uint8_t)(u16A & 0xFFU);
    g_DiagResponse.u.SF.byD2 = (uint8_t)(u16A >> 8);
    g_u8BufferOutID = (uint8_t)QR_RFR_DIAG;                                     /* LIN Output buffer is valid (RFR_DIAG) */
    return;
} /* End of StoreD1to2() */

/*!*************************************************************************** *
 * StoreD1to4()
 * \brief   Store two 16-bit data in Diagnostic Response Frame D1..D4
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16A: 16-bit data (to be stored in D1/D2)
 * \param   [in] u16B: 16-bit data (to be stored in D3/D4)
 * \return  -
 * *************************************************************************** *
 * \details Helper-function store two 16-bit values in Diagnostic Response
 *          Frame at D1 & D2 and D3 & D4. Use g_DiagResponse-buffer
 * *************************************************************************** *
 * - Call Hierarchy: DfrDiagMlxDebug(), DfrDiagReadByIdentifier(), RfrDiagReset()
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 0
 * *************************************************************************** */
static inline void StoreD1to4(uint16_t u16A, uint16_t u16B)
{
    extern RFR_DIAG g_DiagResponse;                                             /*!< LIN Diagnostics Response Buffer */
    extern uint8_t g_u8BufferOutID;                                             /*!< LIN output buffer is invalid */

    g_DiagResponse.u.SF.byD1 = (uint8_t)(u16A & 0xFFU);
    g_DiagResponse.u.SF.byD2 = (uint8_t)(u16A >> 8);
    g_DiagResponse.u.SF.byD3 = (uint8_t)(u16B & 0xFFU);
    g_DiagResponse.u.SF.byD4 = (uint8_t)(u16B >> 8);
    g_u8BufferOutID = (uint8_t)QR_RFR_DIAG;                                     /* LIN Output buffer is valid (RFR_DIAG) */
    return;
} /* End of StoreD1to4() */

#if (_SUPPORT_MLX_DEBUG_MODE != FALSE)
/*!*************************************************************************** *
 * StoreD2to5
 * \brief   Store two 16-bit data in Diagnostic Response Frame D2..D5
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16A: 16-bit data (to be stored in D2/D3)
 * \param   [in] u16B: 16-bit data (to be stored in D4/D5)
 * \return  -
 * *************************************************************************** *
 * \details Helper-function store two 16-bit values in Diagnostic Response
 *          Frame at D2 & D3 and D4 & D5. Use g_DiagResponse-buffer
 *          Note: g_DiagResponse is (16-bits) word-aligned.
 * *************************************************************************** *
 * - Call Hierarchy: DfrDiagMlxDebug()
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 0
 * *************************************************************************** */
static inline void StoreD2to5(uint16_t u16A, uint16_t u16B)
{
    extern RFR_DIAG g_DiagResponse;                                             /*!< LIN Diagnostics Response Buffer */
    extern uint8_t g_u8BufferOutID;                                             /*!< LIN output buffer is invalid */

    uint16_t *p = (uint16_t *)((void *)&g_DiagResponse.u.SF.byD2);
    *p++ = u16A;
    *p = u16B; /*lint !e415 */
    g_u8BufferOutID = (uint8_t)QR_RFR_DIAG;                                     /* LIN Output buffer is valid (RFR_DIAG) */
    return;
} /* End of StoreD2to5() */
#endif /* (_SUPPORT_MLX_DEBUG_MODE != FALSE) */

#endif /* (LIN_COMM != FALSE) */

#endif /* LIN_DIAGNOSTICS_H */

/* EOF */
