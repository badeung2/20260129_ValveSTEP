/*!**************************************************************************** *
 * \file        LIN_2x_HVAC.h
 * \brief       MLX813xx LIN HVAC-Flap communication handling
 *
 * \note        project MLX813xx
 *
 * \author      Marcel Braat
 *
 * \date        2012-02-11
 *
 * \version     2.0
 *
 *
 * MELEXIS Microelectronic Integrated Systems
 *
 * Copyright (C) 2012-2022 Melexis N.V.
 * The Software is being delivered 'AS IS' and Melexis, whether explicitly or
 * implicitly, makes no warranty as to its Use or performance.
 * The user accepts the Melexis Firmware License Agreement.
 *
 * Melexis confidential & proprietary
 *
 * **************************************************************************** */

#ifndef LIN_2X_HVAC_H
#define LIN_2X_HVAC_H

/*!************************************************************************** */
/*                          INCLUDES                                          */
/* ************************************************************************** */
#include "AppBuild.h"                                                           /* Application Build */

#if (LIN_COMM != FALSE) && ((LINPROT == LIN2X_HVAC49) || (LINPROT == LIN2X_HVAC52))

#include <mls_api.h>                                                            /* Melexis LIN module (MMP180430-1) */

/*!*************************************************************************** */
/*                           DEFINITIONS                                       */
/* *************************************************************************** */
#define C_DEFAULT_NAD                       0x7FU                               /*!< Default (not-programmed) NAD */
#define C_BROADCAST_NAD                     0x7FU                               /*!< Broadcast NAD */
/* Note: NAD 0x5A-0x5F are special devices */
#define C_LINAA_MODULE_NAD                  0x5AU                               /*!< LIN-AA Test-module */
#define C_DEFAULT_SNIFFER_NAD               0x5FU                               /*!< LIN Sniffer */
#if (_SUPPORT_HVAC_GROUP_ADDRESS != FALSE)                                      /* MMP150125-1 - Begin */
#define C_INVALD_GAD                        0xFF                                /*!< Default/not-programmed Group-address */
#endif /* (_SUPPORT_HVAC_GROUP_ADDRESS != FALSE) */                             /* MMP150125-1 - End */

/* Supplier ID */
#define C_WILDCARD_SUPPLIER_ID              0x7FFFU                             /*!< Wild-card supplier ID */
#define C_MLX_SUPPLIER_ID                   ((('M' - '@') << 10) | (('L' - '@') << 5) | ('X' - '@'))  /*!< MeLeXis (test) supplier ID */
#define C_SUPPLIER_ID                       C_MLX_SUPPLIER_ID                   /*!< Supplier ID */

/* Function ID */
#define C_WILDCARD_FUNCTION_ID              0xFFFFU                             /*!< Wild card Function ID */
#if (_SUPPORT_HVAC_GROUP_ADDRESS != FALSE)
#define C_FUNCTION_ID                       (0x8000U | (('A' - '@') << 10) | (('C' - '@') << 5) | ('T' - '@'))    /*!< Function-ID for a Group-Actuator */
#else  /* (_SUPPORT_HVAC_GROUP_ADDRESS != FALSE) */
#define C_FUNCTION_ID                       ((('A' - '@') << 10) | (('C' - '@') << 5) | ('T' - '@'))  /*!< Function-ID for an Actuator */
#endif /* (_SUPPORT_HVAC_GROUP_ADDRESS != FALSE) */
#define C_VARIANT_ID                        0x01U                               /*!< Variant ID */

/* Variant ID */
#define C_SW_REF                            0x10U                               /*!< UniROM SW revision 1.0 */
#define C_HW_REF                            0x10U                               /*!< HW Revision 1.0 */
#define C_PROJECT_ID                        0x71U                               /*!< Project #1 */

#define MSG_CONTROL                         0x0001U                             /*!< Actuator Control Message-ID */
#define MSG_STATUS                          0x0002U                             /*!< Actuator Status Message-ID */
#if (_SUPPORT_HVAC_GROUP_ADDRESS != FALSE)
#define MSG_GROUP_CONTROL                   0x0003U                             /*!< Actuator Group Control Message-ID */
#endif /* (_SUPPORT_HVAC_GROUP_ADDRESS != FALSE) */
#if (_SUPPORT_TRIAXIS_MLX90363 != FALSE) || (_SUPPORT_TRIAXIS_MLX90367 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90372 != FALSE) || (_SUPPORT_TRIAXIS_MLX90377 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX9038x != FALSE) || (_SUPPORT_TRIAXIS_MLX90395 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90421 != FALSE) || (_SUPPORT_TRIAXIS_MLX90422 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90425 != FALSE) || (_SUPPORT_TRIAXIS_MLX90426 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90427 != FALSE) || \
    (_SUPPORT_HUMIDITY_HDC302x != FALSE) || \
    (_SUPPORT_INDUCTIVE_POS_SENSOR_MLX90513 != FALSE) || (_SUPPORT_PRESSURE_MLX90829 != FALSE)
#if (LINPROT == LIN2X_HVAC52)
#define MSG_SENSOR                          0x0004U                             /*!< Sensor Status Message-ID */
#else  /* (LINPROT == LIN2X_HVAC52) */
#define MSG_SENSOR                          0x0003U                             /*!< Sensor Status Message-ID */
#endif /* (LINPROT == LIN2X_HVAC52) */
#endif /* (_SUPPORT_TRIAXIS_MLX90363 != FALSE) || (_SUPPORT_TRIAXIS_MLX90367 != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90372 != FALSE) || (_SUPPORT_TRIAXIS_MLX90377 != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX9038x != FALSE) || (_SUPPORT_TRIAXIS_MLX90395 != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90421 != FALSE) || (_SUPPORT_TRIAXIS_MLX90422 != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90425 != FALSE) || (_SUPPORT_TRIAXIS_MLX90426 != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90427 != FALSE) || \
        * (_SUPPORT_HUMIDITY_HDC302x != FALSE) || \
        * (_SUPPORT_INDUCTIVE_POS_SENSOR_MLX90513 != FALSE) || (_SUPPORT_PRESSURE_MLX90829 != FALSE) */
#if (_SUPPORT_MLX_CHIP_STATUS != FALSE)
#define MSG_MLX_CHIP_STATUS                 0x000CU                             /*!< Melexis Chip Status Frame */
#define MSG_MLX_CHIP_CONTROL                0x000DU                             /*!< Melexis Chip Control Frame */
#endif /* (_SUPPORT_MLX_CHIP_STATUS != FALSE) */

#define C_MIN_POS                           0x0000U                             /*!< Minimum position */
#define C_MAX_POS                           0xFFFEU                             /*!< Maximum position */
#define C_INI_POS                           0x7FFFU                             /*!< Initial position */
#define C_INV_POS                           0xFFFFU                             /*!< Invalid position */

#define C_LINAA_TIMEOUT                     40U                                 /*!< LIN-AutoAddressing time-out of 40 sec */

#define mlxCONTROL                          0xC1U                               /*!< Actuator Control Frame-ID */
#define mlxSTATUS                           0x42U                               /*!< Actuator Status Frame-ID */
#if (_SUPPORT_HVAC_GROUP_ADDRESS != FALSE)
#define mlxGROUP_CONTROL                    0x03U                               /*!< Actuator Group-Control Frame-ID */
#endif /* (_SUPPORT_HVAC_GROUP_ADDRESS != FALSE) */
#if (_SUPPORT_MLX_CHIP_STATUS != FALSE)
#if (_SUPPORT_2ND_MLX_CHIP_STATUS == FALSE)
/* Node "C" */
#define mlxCHIP_CONTROL                     0x3AU                               /*!< Melexis Chip Control Frame-ID */
#define mlxCHIP_STATUS                      0x3BU                               /*!< Melexis Chip Status Frame-ID */
#else  /* (_SUPPORT_2ND_MLX_CHIP_STATUS == FALSE) */
/* Node "D" */
#define mlxCHIP_CONTROL                     0x38U                               /*!< Melexis Chip Control Frame-ID */
#define mlxCHIP_STATUS                      0x39U                               /*!< Melexis Chip Status Frame-ID */
#endif /* (_SUPPORT_2ND_MLX_CHIP_STATUS == FALSE) */
#endif /* (_SUPPORT_MLX_CHIP_STATUS != FALSE) */

#define QR_INVALID                          0xFFU                               /*!< Invalid QR */

/*!< Motor position type */
#define C_POSTYPE_NONE          0U                                              /*!< Motor position Type: Unknown */
#define C_POSTYPE_INIT          1U                                              /*!< Motor position Type: Initial */
#define C_POSTYPE_TARGET        2U                                              /*!< Motor position Type: Target */

/*! Description of HVAC_CONTROL LIN-Frame */
typedef struct _HVAC_CTRL
{
    uint16_t u8NAD                : 8;                                          /**< Byte 1: NAD field */
    uint16_t u2Program            : 2;                                          /**< Byte 2: Program field */
#define C_CTRL_PROGRAM_DIS          0U                                          /*!< Store in NVRAM: Disabled */
#define C_CTRL_PROGRAM_ENA          1U                                          /*!< Store in NVRAM: Enabled */
#define C_CTRL_PROGRAM_RES          2U                                          /*!< Store in NVRAM: Reserved */
#define C_CTRL_PROGRAM_INV          3U                                          /*!< Store in NVRAM: Invalid */
    uint16_t u2StallDetector      : 2;                                          /**< Stall detector field */
#define C_CTRL_STALLDET_DIS         0U                                          /*!< Stall-detector: Disabled */
#define C_CTRL_STALLDET_ENA         1U                                          /*!< Stall-detector: Enabled */
#define C_CTRL_STALLDET_RES         2U                                          /*!< Stall-detector: Reserved */
#define C_CTRL_STALLDET_INV         3U                                          /*!< Stall-detector: Invalid */
    uint16_t u4ClearEventFlags    : 4;                                          /**< Clear Event Flags field */
#define C_CTRL_CLREVENT_NONE        0x0U                                        /*!< Clear-Event: None */
#define C_CTRL_CLREVENT_RESET       0x8U                                        /*!< Clear-Event: Reset */
#define C_CTRL_CLREVENT_STALL       0x4U                                        /*!< Clear-Event: Stall */
#define C_CTRL_CLREVENT_EMRUN       0x2U                                        /*!< Clear-Event: Emergency-run */
#define C_CTRL_CLREVENT_RES         0x1U                                        /*!< Clear-Event: Reserved */
#define C_CTRL_CLREVENT_INV         0xFU                                        /*!< Clear-Event: Invalid */
    uint16_t u2HoldingCurrent     : 2;                                          /**< Byte 3: Holding Current field */
#define C_CTRL_MHOLDCUR_DIS         0U                                          /*!< Motor holding-current: Disabled */
#define C_CTRL_MHOLDCUR_ENA         1U                                          /*!< Motor holding-current: Enabled */
#define C_CTRL_MHOLDCUR_RES         2U                                          /*!< Motor holding-current: Reserved */
#define C_CTRL_MHOLDCUR_INV         3U                                          /*!< Motor holding-current: Invalid */
    uint16_t u2PositionType       : 2;                                          /**< Position Type field */
#define C_CTRL_POSITION_TARGET      0U                                          /*!< Position: Target */
#define C_CTRL_POSITION_INITIAL     1U                                          /*!< Position: Initial */
#define C_CTRL_POSITION_NONE        2U                                          /*!< Position: None */
#define C_CTRL_POSITION_INV         3U                                          /*!< Position: Invalid */
    uint16_t u4Speed              : 4;                                          /**< Speed selection field */
#define C_CTRL_SPEED_RES            0U                                          /*!< Motor speed: Reserved */
#define C_CTRL_SPEED_1              1U                                          /*!< Motor speed: 1 (2.25 RPM) */
#define C_CTRL_SPEED_2              2U                                          /*!< Motor speed: 2 (2.25 RPM .. 3.00 RPM) */
#define C_CTRL_SPEED_3              3U                                          /*!< Motor speed: 3 (3.00 RPM .. 4.00 RPM) */
#define C_CTRL_SPEED_4              4U                                          /*!< Motor speed: 4 (> 4.00 RPM) */
#define C_CTRL_SPEED_AUTO           5U                                          /*!< Motor speed: Automatically */
/* #define C_CTRL_SPEED_RES         6..14 */                                    /*!< Motor speed: Reserved */
#define C_CTRL_SPEED_INV            15U                                         /*!< Motor speed: Invalid */
    uint16_t u8TargetPositionLSB  : 8;                                          /**< Byte 4: Target-position LSB field */
    uint16_t u8TargetPositionMSB  : 8;                                          /**< Byte 5: Target-position MSB field  */
    uint16_t u8StartPositionLSB   : 8;                                          /**< Byte 6: Start/Initial-position LSB field */
    uint16_t u8StartPositionMSB   : 8;                                          /**< Byte 7: Start/Initial-position MSB field */
    uint16_t u2EmergencyRun       : 2;                                          /**< Byte 8: Emergency Run/Safety position field */
#define C_CTRL_EMRUN_DIS            0U                                          /*!< Emergency-run: Disabled */
#define C_CTRL_EMRUN_ENA            1U                                          /*!< Emergency-run: Enabled */
#define C_CTRL_EMRUN_RES            2U                                          /*!< Emergency-run: Reserved */
#define C_CTRL_EMRUN_INV            3U                                          /*!< Emergency-run: Invalid */
    uint16_t u2EmergencyEndStop   : 2;                                          /**< Emergency End-stop selection field */
#define C_CTRL_ENRUN_ENDSTOP_LO     0U                                          /*!< Emergency-run End-stop: Low */
#define C_CTRL_ENRUN_ENDSTOP_HI     1U                                          /*!< Emergency-run End-stop: High */
#define C_CTRL_ENRUN_ENDSTOP_RES    2U                                          /*!< Emergency-run End-stop: Reserved */
#define C_CTRL_ENRUN_ENDSTOP_INV    3U                                          /*!< Emergency-run End-stop: Invalid */
    uint16_t u2RotationDirection  : 2;                                          /**< Rotational Direction field */
#define C_CTRL_DIR_CW               0U                                          /*!< Rotational-direction: ClockWise */
#define C_CTRL_DIR_CCW              1U                                          /*!< Rotational-direction: CounterClockWise */
#define C_CTRL_DIR_RES              2U                                          /*!< Rotational-direction: Reserved */
#define C_CTRL_DIR_INV              3U                                          /*!< Rotational-direction: Invalid */
    uint16_t u2StopMode           : 2;                                          /**< Stop Mode field */
#define C_CTRL_STOPMODE_NORMAL      0U                                          /*!< Stop-mode: Normal */
#define C_CTRL_STOPMODE_STOP        1U                                          /*!< Stop-mode: Stop */
#if (LINPROT == LIN2X_HVAC52)                                                   /* (MMP201217-2) */
#define C_CTRL_STOPMODE_NORMAL_GAD  2U                                          /*!< Stop-mode: Normal, reply GAD in status */
#else  /* (LINPROT == LIN2X_HVAC52) */
#define C_CTRL_STOPMODE_RES         2U                                          /*!< Stop-mode: Reserved */
#endif /* (LINPROT == LIN2X_HVAC52) */
#define C_CTRL_STOPMODE_INV         3U                                          /*!< Stop-mode: Invalid */
} HVAC_CTRL;

/*! Description of HVAC_STATUS LIN-Frame */
typedef struct _HVAC_STATUS
{
    uint16_t u1ResponseError      : 1;                                          /**< Byte 1: (LIN) Response Error field */
#define C_STATUS_ERROR_NONE         0U                                          /*!< Error: None */
#define C_STATUS_ERROR              1U                                          /*!< Error: Available */
    uint16_t u1Reserved1          : 1;                                          /**< Reserved field */
    uint16_t u2OverTemperature    : 2;                                          /**< Over-temperature field */
#define C_STATUS_OTEMP_NO           0U                                          /*!< Over-temperature: No */
#define C_STATUS_OTEMP_YES          1U                                          /*!< Over-temperature: Yes */
#define C_STATUS_OTEMP_RES          2U                                          /*!< Over-temperature: Reserved */
#define C_STATUS_OTEMP_INV          3U                                          /*!< Over-temperature: Invalid */
    uint16_t u2ElectricDefect     : 2;                                          /**< Electric Defect field */
#define C_STATUS_ELECDEFECT_NO      0U                                          /*!< Electric-defect: No */
#define C_STATUS_ELECDEFECT_YES     1U                                          /*!< Electric-defect: Yes */
#define C_STATUS_ELECDEFECT_PERM    2U                                          /*!< Electric-defect: Yes, permanent */
#define C_STATUS_ELECDEFECT_INV     3U                                          /*!< Electric-defect: Invalid */
    uint16_t u2VoltageError       : 2;                                          /**< Voltage Range error field */
#define C_STATUS_VOLTAGE_OK         0U                                          /*!< Voltage-range: Ok */
#define C_STATUS_VOLTAGE_UNDER      1U                                          /*!< Voltage-range: Under-voltage */
#define C_STATUS_VOLTAGE_OVER       2U                                          /*!< Voltage-range: Over-voltage */
#define C_STATUS_VOLTAGE_INV        3U                                          /*!< Voltage-range: Invalid */
    uint16_t u2EmergencyOccurred  : 2;                                          /**< Byte 2: Emergency Run Occurred field */
#define C_STATUS_EMRUNOCC_NO        0U                                          /*!< Emergency-run occurred: No */
#define C_STATUS_EMRUNOCC_YES       1U                                          /*!< Emergency-run occurred: Yes */
#define C_STATUS_EMRUNOCC_RES       2U                                          /*!< Emergency-run occurred: Reserved */
#define C_STATUS_EMRUNOCC_INV       3U                                          /*!< Emergency-run occurred: Invalid */
    uint16_t u2StallDetector      : 2;                                          /**< Stall Detector field */
#define C_STATUS_STALLDET_DIS       0U                                          /*!< Stall Detector: Disabled */
#define C_STATUS_STALLDET_ENA       1U                                          /*!< Stall Detector: Enabled */
#define C_STATUS_STALLDET_RES       2U                                          /*!< Stall Detector: Reserved */
#define C_STATUS_STALLDET_INV       3U                                          /*!< Stall Detector: Invalid */
    uint16_t u2StallOccurred      : 2;                                          /**< Stall Occurred field */
#define C_STATUS_STALLOCC_NO        0U                                          /*!< Stall occurred: No */
#define C_STATUS_STALLOCC_YES       1U                                          /*!< Stall occurred: Yes */
#define C_STATUS_STALLOCC_RES       2U                                          /*!< Stall occurred: Reserved */
#define C_STATUS_STALLOCC_INV       3U                                          /*!< Stall occurred: Invalid */
    uint16_t u2Reset              : 2;                                          /**< IC/Module reset field */
#define C_STATUS_RESETOCC_NO        0U                                          /*!< Reset occurred: No */
#define C_STATUS_RESETOCC_YES       1U                                          /*!< Reset occurred: Yes */
#define C_STATUS_RESETOCC_RES       2U                                          /*!< Reset occurred: Reserved */
#define C_STATUS_RESETOCC_INV       3U                                          /*!< Reset occurred: Invalid */
    uint16_t u2HoldingCurrent     : 2;                                          /**< Byte 3: Holding Current Mode field */
#define C_STATUS_MHOLDCUR_DIS       0U                                          /*!< Motor holding-current: No */
#define C_STATUS_MHOLDCUR_ENA       1U                                          /*!< Motor holding-current: Yes */
#define C_STATUS_MHOLDCUR_RES       2U                                          /*!< Motor holding-current: Reserved */
#define C_STATUS_MHOLDCUR_INV       3U                                          /*!< Motor holding-current: Invalid */
    uint16_t u2PositionTypeStatus : 2;                                          /**< Position Type Status field */
#define C_STATUS_POSITION_ACTUAL    0U                                          /*!< Valid position: Actual */
#define C_STATUS_POSITION_INIT      1U                                          /*!< Valid position: Initial */
#define C_STATUS_POSITION_NONE      2U                                          /*!< Valid position: None */
#define C_STATUS_POSITION_INV       3U                                          /*!< Valid position: Invalid */
    uint16_t u4SpeedStatus        : 4;                                          /**< Actual Speed status field */
#define C_STATUS_SPEED_STOP         0U                                          /*!< Actual speed: Stop */
#define C_STATUS_SPEED_1            1U                                          /*!< Actual speed: 1 (2.25 RPM) */
#define C_STATUS_SPEED_2            2U                                          /*!< Actual speed: 2 (2.25 RPM .. 3.00 RPM) */
#define C_STATUS_SPEED_3            3U                                          /*!< Actual speed: 3 (3.00 RPM .. 4.00 RPM) */
#define C_STATUS_SPEED_4            4U                                          /*!< Actual speed: 4 (> 4.00 RPM) */
#define C_STATUS_SPEED_AUTO         5U                                          /*!< Actual speed: Auto */
#define C_STATUS_SPEED_INV          15U                                         /*!< Actual speed: Invalid */
    uint16_t u8ActualPositionLSB  : 8;                                          /**< Byte 4: Actual Position LSB field */
    uint16_t u8ActualPositionMSB  : 8;                                          /**< Byte 5: Actual Position MSB field */
    uint16_t u2ActualRotationalDir : 2;                                         /**< Byte 6: Actual Motor Rotational Direction field */
#define C_STATUS_ACT_DIR_CLOSING    0U                                          /*!< Actual/last direction is: Closing */
#define C_STATUS_ACT_DIR_OPENING    1U                                          /*!< Actual/last direction is: Opening */
#define C_STATUS_ACT_DIR_UNKNOWN    2U                                          /*!< Actual/last direction is: Unknown */
#define C_STATUS_ACT_DIR_INV        3U                                          /*!< Actual/last direction is: Invalid */
    uint16_t u2SelfHoldingTorque  : 2;                                          /**< Self-holding torque field */
#define C_STATUS_HOLDING_TORQUE_DIS 0U                                          /*!< Self Holding Torque: Disabled */
#define C_STATUS_HOLDING_TORQUE_ENA 1U                                          /*!< Self Holding Torque: Enabled */
#define C_STATUS_HOLDING_TORQUE_RES 2U                                          /*!< Self Holding Torque: Reserved */
#define C_STATUS_HOLDING_TORQUE_INV 3U                                          /*!< Self Holding Torque: Invalid */
    uint16_t u2SpecialFunctionActive : 2;                                       /**< Special Function Active field */
#define C_STATUS_SFUNC_ACTIVE_NO    0U                                          /*!< Special Function Active: No */
#define C_STATUS_SFUNC1_ACTIVE_YES  1U                                          /*!< Special Function #1 Active (Rewind): Yes */
#define C_STATUS_SFUNC_ACTIVE_RES   2U                                          /*!< Special Function Active: Reserved */
#define C_STATUS_SFUNC_ACTIVE_INV   3U                                          /*!< Special Function Active: Invalid */
    uint16_t u2Reserved           : 2;                                          /**< Reserved field */
    uint16_t u8NAD                : 8;                                          /**< Byte 7: NAD field */
    uint16_t u2EmergencyRun       : 2;                                          /**< Byte 8: Emergency Run field */
#define C_STATUS_EMRUN_DIS          0U                                          /*!< Emergency-run: Disabled */
#define C_STATUS_EMRUN_ENA          1U                                          /*!< Emergency-run: Enabled */
#define C_STATUS_EMRUN_RES          2U                                          /*!< Emergency-run: Reserved */
#define C_STATUS_EMRUN_INV          3U                                          /*!< Emergency-run: Invalid */
    uint16_t u2EmergencyRunEndStop : 2;                                         /**< Emergency Run End-stop field */
#define C_STATUS_EMRUN_ENDPOS_LO    0U                                          /*!< Emergency-run End-position: Low */
#define C_STATUS_EMRUN_ENDPOS_HI    1U                                          /*!< Emergency-run End-position: High */
#define C_STATUS_EMRUN_ENDPOS_RES   2U                                          /*!< Emergency-run End-position: Reserved */
#define C_STATUS_EMRUN_ENDPOS_INV   3U                                          /*!< Emergency-run End-position: Invalid */
    uint16_t u2RotationDirection  : 2;                                          /**< Rotational Direction field */
#define C_STATUS_DIRECTION_CW       0U                                          /*!< Rotational direction: ClockWise */
#define C_STATUS_DIRECTION_CCW      1U                                          /*!< Rotational direction: Counter-ClockWise */
#define C_STATUS_DIRECTION_RES      2U                                          /*!< Rotational direction: Reserved */
#define C_STATUS_DIRECTION_INV      3U                                          /*!< Rotational direction: Invalid */
    uint16_t u2StopMode           : 2;                                          /**< Stop-Mode field */
#define C_STATUS_STOPMODE_NORMAL    0U                                          /*!< Stop-mode: Normal */
#define C_STATUS_STOPMODE_STOP      1U                                          /*!< Stop-mode: Stop */
#define C_STATUS_STOPMODE_RES       2U                                          /*!< Stop-mode: Reserved */
#define C_STATUS_STOPMODE_DEGRADED  2U                                          /*!< Stop-mode: Degraded */
#define C_STATUS_STOPMODE_INV       3U                                          /*!< Stop-mode: Invalid */
} HVAC_STATUS;

/*!*************************************************************************** */
/*                           GLOBAL VARIABLES                                  */
/* *************************************************************************** */
#pragma space dp                                                                /* __TINY_SECTION__ */
extern uint8_t g_u8NAD;
extern uint8_t g_u8CtrlPID;
extern uint8_t g_u8StsPID;
#pragma space none                                                              /* __TINY_SECTION__ */

#pragma space nodp                                                              /* __NEAR_SECTION__ */
#pragma space none                                                              /* __NEAR_SECTION__ */

/*!*************************************************************************** */
/*                           GLOBAL FUNCTIONS                                  */
/* *************************************************************************** */
extern void LIN_2x_Init(void);                                                  /* LIN 2.x initialisation */
#if (LINPROT == LIN2X_HVAC52) && (_SUPPORT_HVAC_GROUP_ADDRESS != FALSE)
extern void HandleActCtrl(uint16_t u16Group);                                   /* HVAC Actuator Control (with Group-support) */
#else  /* (LINPROT == LIN2X_HVAC52) && (_SUPPORT_HVAC_GROUP_ADDRESS != FALSE) */
extern void HandleActCtrl(void);                                                /* HVAC Actuator Control */
#endif /* (LINPROT == LIN2X_HVAC52) || (_SUPPORT_HVAC_GROUP_ADDRESS != FALSE) */
extern void HandleActStatus(void);                                              /* HVAC Actuator Status */
extern void HandleBusTimeout(void);                                             /* Bus time-out */
extern void HandleDataTransmitted(ml_MessageID_t Index);
extern void HandleLinError(ml_LinError_t Error);                                /* LIN2x Error handling */

#endif /* (LIN_COMM != FALSE) && ((LINPROT == LIN2X_HVAC49) || (LINPROT == LIN2X_HVAC52)) */

#endif /* LIN_2X_HVAC_H */

/* EOF */
