/*!*************************************************************************** *
 * \file        NV_UserPage.h
 * \brief       MLX8133x/4x NVRAM User Page handling
 *
 * \note        project MLX8133x/4x
 *
 * \author      Marcel Braat
 *
 * \date        2017-08-23
 *
 * \version     2.0
 *
 *
 * \copyright   MELEXIS Microelectronic Integrated Systems
 *
 * Copyright (C) 2017-2024 Melexis N.V.
 * The Software is being delivered 'AS IS' and Melexis, whether explicitly or
 * implicitly, makes no warranty as to its Use or performance.
 * The user accepts the Melexis Firmware License Agreement.
 *
 * Melexis confidential & proprietary
 *
 * Non Volatile Memory Map:
 *
 * WordIdx  ByteIdx Size    Description
 *                   32     PATCH_HDR (Keep together with PATCH_CODE as one block (HEX))
 *   000    0x0000          Patch header & code CRC (LSB) and patch code size (MSB)
 *   001    0x0002          Patch ID and version field
 *   002    0x0004          Patch 0 Data/Instruction
 *   003    0x0006          Patch 1 Data/Instruction
 *   004    0x0008          Patch 2 Data/Instruction
 *   005    0x000A          Patch 3 Data/Instruction
 *   006    0x000C          Patch 0 Address[15:0]
 *   007    0x000E          Patch 0 Address[19:16] & Control
 *   008    0x0010          Patch 1 Address[15:0]
 *   009    0x0012          Patch 1 Address[19:16] & Control
 *   010    0x0014          Patch 2 Address[15:0]
 *   011    0x0016          Patch 2 Address[19:16] & Control
 *   012    0x0018          Patch 3 Address[15:0]
 *   013    0x001A          Patch 3 Address[19:16] & Control
 *   014    0x001C  ( 4)    Reserved
 *                  0-32   PATCH_CODE (Must start at EP/FP7)
 *   016    0x0020          Patch code
 *    :        :            Patch code
 * MLX81339 only:
 * | 063    0x007F          Patch code
 * | 064    0x0080    8     HEADER_t                (ADDR_NV_HDR)
 * | 068    0x0088  2x8     STD_LIN_PARAMS_t[2]     (ADDR_NV_STD_LIN_1, ADDR_NV_STD_LIN_2)
 * | 076    0x0098  2x8     ENH_LIN_PARAMS_t[2]     (ADDR_NV_ENH_LIN_1, ADDR_NV_ENH_LIN_2)
 * | 084    0x00A8   64     UDS_LIN_PARAMS_t        (ADDR_NV_UDS)
 * | 116    0x00E8    8     APP_EOL_t               (ADDR_NV_EOL)
 * | 124    0x00F0  2x8     APP_STORE_t
 * | 132    0x0100   40     ACT_PARAMS_t            (ADDR_NV_ACT_PARAMS)
 * | 152    0x0128    8     SENSOR_t                (ADDR_NV_SENSOR_PARAMS)
 * | 156    0x0130    8     ACT_STALL_t             (ADDR_NV_ACT_STALL)
 * | 160    0x0138   16     APP_LOG_t               (ADDR_NV_ERRORLOG)
 * | 168    0x0148    8     I2C_PARAMS_t            (ADDR_NV_I2C_PARAMS)
 * | 172    0x0150   40     Free
 * |
 * | 192    0x0180  128     Wear-leveling page #1
 * | 256    0x0200  128     Wear-leveling page #1
 * | 320    0x0280  128     Wear-leveling page #1
 * |
 * | 384    0x0300  128     CALIBRATION
 * | 424    0x0350   48     TRIM
 * | 448    0x0380    6     APP_CHIP_ID
 * | 451    0x0386    2     PROJECT_ID
 * | 452    0x0388  112     Reserved
 * | 508    0x03F8    8     MLX_CHIP_ID
 *
 * Others:
 * | 031    0x003F          Patch code
 * | 032    0x0040    8     HEADER_t                (ADDR_NV_HDR)
 * | 036    0x0048  2x8     STD_LIN_PARAMS_t[2]     (ADDR_NV_STD_LIN_1, ADDR_NV_STD_LIN_2)
 * | 044    0x0058  2x8     ENH_LIN_PARAMS_t[2]     (ADDR_NV_ENH_LIN_1, ADDR_NV_ENH_LIN_2)
 * | 052    0x0068   64     UDS_LIN_PARAMS_t        (ADDR_NV_UDS)
 * | 084    0x00A8    8     APP_EOL_t               (ADDR_NV_EOL)
 * | 088    0x00B0  2x8     APP_STORE_t
 * | 096    0x00C0   32     ACT_PARAMS_t            (ADDR_NV_ACT_PARAMS)
 * | 112    0x00E0    8     SENSOR_t                (ADDR_NV_SENSOR_PARAMS)
 * | 116    0x00E8    8     ACT_STALL_t             (ADDR_NV_ACT_STALL)
 * | 120    0x00F0   16     APP_LOG_t               (ADDR_NV_ERRORLOG)
 * | 128    0x0100    8     I2C_PARAMS_t            (ADDR_NV_I2C_PARAMS)
 * | 132    0x0108  168     Free
 * |
 * | 224    0x01B0   80     CALIBRATION
 * | 256    0x0200   48     TRIM
 * | 280    0x0230    8     APP_CHIP_ID
 * | 283    0x0236          PROJECT_ID
 * | 284    0x0238    8     MLX_CHIP_ID
 *
 * *************************************************************************** */

#ifndef DRIVE_LIB_NV_USER_PAGE_H
#define DRIVE_LIB_NV_USER_PAGE_H

/*!*************************************************************************** */
/*                           INCLUDES                                          */
/* *************************************************************************** */
#include "AppBuild.h"                                                           /* Application Build */

/*!*************************************************************************** */
/*                           DEFINITIONS                                       */
/* *************************************************************************** */
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__)
#define C_NV_USER_REV        0x01U                                              /*!< NV User Header version ID */
#elif defined (__MLX81339__) || defined (__MLX81350__)
#define C_NV_USER_REV        0x03U                                              /*!< NV User Header version ID */
#elif defined (__MLX81160__) || defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
#define C_NV_USER_REV        0x02U                                              /*!< Non Volatile Memory User Layout Version ID (16-bit speeds) */
/* #define C_NV_USER_REV     0x03U */                                           /*!< Non Volatile Memory User Layout Version ID (9-bit speeds-base + 3-bits exponent) */
#if (_SUPPORT_RSHUNT < 3) && (C_NV_CURR_DIV > 4)                                /* Up to 64A */
#define C_MULTIPLIER_OFF     5U                                                 /*!< Motor current 32x, 64x, 128x or 256x (up to 65A) */
#elif (_SUPPORT_RSHUNT < 6) && (C_NV_CURR_DIV > 3)                              /* Up to 32A */
#define C_MULTIPLIER_OFF     4U                                                 /*!< Motor current 16x, 32x, 64x or 128x (up to 32A) */
#elif (_SUPPORT_RSHUNT < 12) && (C_NV_CURR_DIV > 2)                             /* Up to 16A */
#define C_MULTIPLIER_OFF     3U                                                 /*!< Motor current 8x, 16x, 32x or 64x (up to 16A) */
#elif (_SUPPORT_RSHUNT < 25) && (C_NV_CURR_DIV > 1)                             /* Up to 8A */
#define C_MULTIPLIER_OFF     2U                                                 /*!< Motor current 4x, 8x, 16x or 32x (up to 8A) */
#elif (_SUPPORT_RSHUNT < 50) && (C_NV_CURR_DIV > 0)                             /* Up to 4A */
#define C_MULTIPLIER_OFF     1U                                                 /*!< Motor current 2x, 4x, 8x or 16x (up to 4A) */
#else  /* _SUPPORT_RSHUNT */
#define C_MULTIPLIER_OFF     0U                                                 /*!< Motor current 1x, 2x, 4x or 8x (up to 2A) */
#endif /* _SUPPORT_RSHUNT */
#endif

/* USER AREA:
 *  1) HEADER:            1 block  of 64-bits ( 4x 16-bits words) -    0 bits unused ( 0.0%)
 * 2a) STD_LIN_PARAMS:    2 blocks of 64-bits ( 8x 16-bits words) -  2x4 bits unused ( 6.25%) (MLX81339: Not used)
 * 2b) ENH_LIN_PARAMS:    2 blocks of 64-bits ( 8x 16-bits words) - 2x16 bits unused (25.0%) (MLX81339: Not used)
 * 2c) UDS_LIN_PARAMS:    8 blocks of 64-bits (32x 16-bits words) -   48 bits unused ( 9.375%) (MLX81339: Not used)
 *  3) APP_EOL:           1 blocks of 64-bits ( 4x 16-bits words) -    4 bits unused ( 6.25%)
 *  4) APP_PARAMS:        1 block  of 64-bits ( 4x 16-bits words) -    0 bits unused ( 0.0%)
 *  5) ACT_PARAMS:        4 blocks of 64-bits (16x 16-bits words) -    0 bits unused ( 0.0%) (Others)
 *  5) ACT_PARAMS:        5 blocks of 64-bits (20x 16-bits words) -   19 bits unused ( 0.0%) (MLX81339 only
 *  6) SENSOR_PARAMS:     1 block  of 64-bits ( 4x 16-bits words) -    4 bits unused ( 6.25%)
 *  7) ACT_STALL:         1 block  of 64-bits ( 4x 16-bits words) -    2 bits unused ( 3.125%)
 *  8) APP_LOG:           2 blocks of 64-bits ( 8x 16-bits words) -    0 bits unused ( 0.0%)
 *  9) STD_I2C_PARAMS:    1 block  of 64-bits ( 4x 16-bits words) -   48 bits unused (75.0%)
 * 10) STD_CAN_PARAMS:    2 blocks of 64-bits ( 4x 16-bits words) -   32 bits unused (25.0%)
 * 11) EACT_PARAMS:       2 blocks of 64-bits ( 4x 16-bits words) -   51 bits unused (39.8%)
 * TOTAL:             21-22 blocks of 64-bits (92x 16-bits words) -
 *
 * MLX81339:             10x8 bytes = 80 Bytes (1 Block of 128By)
 */
/*! 1) HEADER: 1 block  of 64-bits (4x 16-bits words) */
typedef struct
{
    uint16_t u8CRC8                   : 8;                                      /**< 0x00: CRC-8 */
    uint16_t u8Revision               : 8;                                      /**< 0x01: Non Volatile Memory Structure revision */
    uint16_t u16ConfigurationID;                                                /**< 0x02: Configuration ID */
    uint16_t u12StructIDs             : 14;                                     /**< 0x04: Structure ID's */
#define C_HEADER_PARAMS                 (1U << 0)                               /*!< Bit 0: Header Parameters ID */
#if (LIN_COMM != FALSE)
#define C_STD_LIN_PARAMS                (1U << 1)                               /*!< Bit 1: Standard LIN  Parameters ID */
#if (LINPROT == LIN2X_HVAC52)
#define C_ENH_LIN_PARAMS                (1U << 2)                               /*!< Bit 2: Enhanced LIN Parameters ID */
#else  /* (LINPROT == LIN2X_HVAC52) */
#define C_ENH_LIN_PARAMS                (0U << 2)                               /*!< Bit 2: Enhanced LIN Parameters ID */
#endif /* (LINPROT == LIN2X_HVAC52) */
#if (_SUPPORT_UDS != FALSE)
#define C_UDS_LIN_PARAMS                (1U << 3)                               /*!< Bit 3: UDS LIN Parameters ID */
#else  /* (_SUPPORT_UDS != FALSE) */
#define C_UDS_LIN_PARAMS                (0U << 3)                               /*!< Bit 3: UDS LIN Parameters ID */
#endif /* (_SUPPORT_UDS != FALSE) */
#else  /* (LIN_COMM != FALSE) */
#define C_STD_LIN_PARAMS                (0U << 1)                               /*!< Bit 1: Standard LIN  Parameters ID */
#define C_ENH_LIN_PARAMS                (0U << 2)                               /*!< Bit 2: Enhanced LIN Parameters ID */
#define C_UDS_LIN_PARAMS                (0U << 3)                               /*!< Bit 3: UDS LIN Parameters ID */
#endif /* (LIN_COMM != FALSE) */
#define C_APP_EOL                       (1U << 4)                               /*!< Bit 4: End-of-Line ID */
#if (_SUPPORT_APP_SAVE != FALSE)
#define C_APP_STORE                     (1U << 5)                               /*!< Bit 5: Application Storage ID */
#else  /* (_SUPPORT_APP_SAVE != FALSE) */
#define C_APP_STORE                     (0U << 5)                               /*!< Bit 5: Application Storage ID */
#endif /* (_SUPPORT_APP_SAVE != FALSE) */
#define C_ACT_PARAMS                    (1U << 6)                               /*!< Bit 6: Actuator Parameters ID */
#if (_SUPPORT_TRIAXIS_MLX90363 != FALSE) || (_SUPPORT_TRIAXIS_MLX9038x != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90367 != FALSE) || (_SUPPORT_TRIAXIS_MLX90372 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90377 != FALSE) || (_SUPPORT_TRIAXIS_MLX90395 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90421 != FALSE) || (_SUPPORT_TRIAXIS_MLX90422 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90425 != FALSE) || (_SUPPORT_TRIAXIS_MLX90426 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90427 != FALSE) || \
    (_SUPPORT_INDUCTIVE_POS_SENSOR_MLX90513 != FALSE) || \
    (_SUPPORT_DUAL_HALL_LATCH_MLX92251 != FALSE) || (_SUPPORT_DUAL_HALL_LATCH_MLX92255 != FALSE) || \
    (_SUPPORT_FOC_MODE != FOC_MODE_NONE)
#define C_SENSOR_PARAMS                 (1U << 7)                               /*!< Bit 7: Sensor Parameters ID */
#else  /* (_SUPPORT_TRIAXIS_MLX90363 != FALSE) || (_SUPPORT_TRIAXIS_MLX9038x != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90367 != FALSE) || (_SUPPORT_TRIAXIS_MLX90372 != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90377 != FALSE) || (_SUPPORT_TRIAXIS_MLX90395 != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90421 != FALSE) || (_SUPPORT_TRIAXIS_MLX90422 != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90425 != FALSE) || (_SUPPORT_TRIAXIS_MLX90426 != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90427 != FALSE) || \
        * (_SUPPORT_INDUCTIVE_POS_SENSOR_MLX90513 != FALSE) || \
        * (_SUPPORT_DUAL_HALL_LATCH_MLX92251 != FALSE) || (_SUPPORT_DUAL_HALL_LATCH_MLX92255 != FALSE) || \
        * (_SUPPORT_FOC_MODE != FOC_MODE_NONE) */
#define C_SENSOR_PARAMS                 (0U << 7)                               /*!< Bit 7: Sensor Parameters ID */
#endif /* (_SUPPORT_TRIAXIS_MLX90363 != FALSE) || (_SUPPORT_TRIAXIS_MLX9038x != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90367 != FALSE) || (_SUPPORT_TRIAXIS_MLX90372 != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90377 != FALSE) || (_SUPPORT_TRIAXIS_MLX90395 != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90421 != FALSE) || (_SUPPORT_TRIAXIS_MLX90422 != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90425 != FALSE) || (_SUPPORT_TRIAXIS_MLX90426 != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90427 != FALSE) || \
        * (_SUPPORT_INDUCTIVE_POS_SENSOR_MLX90513 != FALSE) || \
        * (_SUPPORT_DUAL_HALL_LATCH_MLX92251 != FALSE) || (_SUPPORT_DUAL_HALL_LATCH_MLX92255 != FALSE) || \
        * (_SUPPORT_FOC_MODE != FOC_MODE_NONE) */
#define C_ACT_STALL                     (1U << 8)                               /*!< Bit 8: Actuator Stall Parameters ID */
#if (_SUPPORT_NV_LOG_ERROR != FALSE)
#define C_APP_ERROR_LOG                 (1U << 9)                               /*!< Bit 9: Application Error Log ID */
#else  /* (_SUPPORT_NV_LOG_ERROR != FALSE) */
#define C_APP_ERROR_LOG                 (0U << 9)                               /*!< Bit 9: Application Error Log ID */
#endif /* (_SUPPORT_NV_LOG_ERROR != FALSE) */
#if (I2C_COMM != FALSE)
#define C_I2C_PARAMS                    (1U << 10)                              /*!< Bit 10: I2C parameters bit-ID */
#else  /* (I2C_COMM != FALSE) */
#define C_I2C_PARAMS                    (0U << 10)                              /*!< Bit 10: I2C parameters bit-ID */
#endif /* (I2C_COMM != FALSE) */
#if (CAN_COMM != FALSE)
#define C_CAN_PARAMS                    (1U << 11)                              /*!< Bit 11: CAN parameters bit-ID */
#else  /* (CAN_COMM != FALSE) */
#define C_CAN_PARAMS                    (0U << 11)                              /*!< Bit 11: CAN parameters bit-ID */
#endif /* (CAN_COMM != FALSE) */
#define C_RES1_PARAMS                   (0U << 12)                              /*!< Bit 12: Reserved 1 parameters bit-ID */
#define C_RES2_PARAMS                   (0U << 13)                              /*!< Bit 13: Reserved 2 parameters bit-ID */
    uint16_t u2WriteCycleCountMSW     : 2;                                      /**< 0x05: Write Cycle Count MSW */
    uint16_t u16WriteCycleCountLSW;                                             /**< 0x06: Write Cycle Count LSW */
} HEADER_t;

#if (LIN_COMM != FALSE)
/*! 2a) STD_LIN_PARAMS_t: 1 block of 64-bits of Standard LIN parameters (in-car programming) */
typedef struct
{
    uint16_t u8CRC8                   : 8;                                      /**< 0x00: CRC8 */
    uint16_t u3LinUV                  : 3;                                      /**< 0x01.[2:0]: LIN UV threshold */
    uint16_t u1BusTimeOutSleep        : 1;                                      /**< 0x01.[3]: Bus-TimeOut Sleep */
    uint16_t u4Reserved               : 4;                                      /**< 0x01.[7:4]: Reserved */
    uint16_t u8NAD                    : 8;                                      /**< 0x02: Node Address (W:B5) */
    uint16_t u8ControlFrameID         : 8;                                      /**< 0x03: Control-message Frame-ID (R:B2-10/W:B7) */
    uint16_t u8StatusFrameID          : 8;                                      /**< 0x04: Status-message Frame-ID (R:B2-11/W:B7) */
    uint16_t u8Variant                : 8;                                      /**< 0x05: Product Reference (R:B2-00/W:B4) */
    uint16_t u8HardwareID             : 8;                                      /**< 0x06: Hardware ID (R:B2-xx/W:B4) */
#if (LINPROT != LIN2X_AIRVENT12)
    uint16_t u8SoftwareID             : 8;                                      /**< 0x07: Software ID (R:B2-xx/W:B4) */
#else  /* (LINPROT != LIN2X_AIRVENT12) */
    uint16_t u8Status2FrameID         : 8;                                      /**< 0x07: Status2-message Frame-ID (R:B2-12/W:B7) */
#endif /* (LINPROT != LIN2X_AIRVENT12) */
} STD_LIN_PARAMS_t;

#if (LINPROT == LIN2X_HVAC52)
/*! 2b) ENH_LIN_PARAMS_t: 1 block of 64-bits of Enhanced LIN parameters (in-car programming) */
typedef struct
{
    uint16_t u8CRC8                   : 8;                                      /**< 0x00: CRC8 */
    uint16_t u8LINAA_EShunt           : 8;                                      /**< 0x01: LIN-AA External shunt */
    uint16_t u16FunctionID;                                                     /**< 0x02: Function ID */
    uint16_t u8GAD                    : 8;                                      /**< 0x04: Group Address */
    uint16_t u8GroupControlFrameID    : 8;                                      /**< 0x05: Group Control-message Frame-ID */
    uint16_t u16Reserved;                                                       /**< 0x06: Reserved */
} ENH_LIN_PARAMS_t;
#endif /* (LINPROT != LIN2X_HVAC52) */

#if (_SUPPORT_UDS != FALSE)
/*! UDS String Pack structure; Note: This UDS String implementation does not support ASCII 0x60..0x7F, which includes: a-z. */
typedef struct _PCKCHR
{
    uint8_t u6Char1 : 6;                                                        /**< ASCII 0x20..0x5F: ' ' .. '_' --> 0x00..0x3F */
    uint8_t u6Char2 : 6;                                                        /**< ASCII 0x20..0x5F: ' ' .. '_' --> 0x00..0x3F */
    uint8_t u6Char3 : 6;                                                        /**< ASCII 0x20..0x5F: ' ' .. '_' --> 0x00..0x3F */
    uint8_t u6Char4 : 6;                                                        /**< ASCII 0x20..0x5F: ' ' .. '_' --> 0x00..0x3F */
} __attribute__((packed)) PCKCHR_t;

/*! 2c) UDS_LIN_PARAMS_t: 8 blocks of 64-bits (32x 16-bits words) (Example) */
typedef struct
{
    uint16_t u8CRC8                   : 8;                                      /**< 0x00: CRC8 */
    uint16_t u8Reserved               : 8;                                      /**< 0x01: Reserved */
#if (_SUPPORT_UDS_DK >= _SUPPORT_UDS_DK4)
    uint16_t u16UDS_SubSystemID;                                                /**< 0x02: UDS Sub-System ID */
#else /* (_SUPPORT_UDS_DK >= _SUPPORT_UDS_DK4) */
    uint16_t UDS_Reserved;                                                      /**< 0x02: Reserved for UDS Sub-System ID */
#endif /* (_SUPPORT_UDS_DK >= _SUPPORT_UDS_DK4) */
    uint16_t UDS_FazitProductionDate;                                           /**< 0x04: Production Date Tier 1: [15:9] Year (00-99), [8:5] Month (1-12), [4:0] Day (1-31) (0x6E00-0x6FFF) */
    uint16_t UDS_FazitEOLNumber;                                                /**< 0x06: EOL Number Fazit (4 digits-BCD) (0x6E00-0x6FFF) */
    uint16_t UDS_FazitManufacturerNumber;                                       /**< 0x08: Manufacturer Number Fazit (4 digits-BCD) (0x6E00-0x6FFF) */
    PCKCHR_t UDS_SystemName[3];                                                 /**< 0x0A-0x12: UDS System Name [11:0] (0x6C00-0x6DFF) */
    uint8_t UDS_SystemName_12;                                                  /**< 0x13: UDS System Name [12] (0x6C00-0x6DFF) */
    PCKCHR_t UDS_SerialNumber[5];                                               /**< 0x14-0x22: UDS Serial-number[20] (0x6A00-0x6BFF) */
    uint8_t UDS_HwVersion_LSB;                                                  /**< 0x23: UDS Hardware version (LSB) (0x6800-0x69FF) */
    uint8_t UDS_HwVersion_MSB;                                                  /**< 0x24: UDS Hardware version (MSB) (0x6800-0x69FF) */
    uint8_t UDS_HwVersion_XSB;                                                  /**< 0x25: UDS Hardware version (XSB) (0x6800-0x69FF) */
    PCKCHR_t UDS_HardwareNumber[3];                                             /**< 0x26-0x2E: UDS Hardware number[11] (0x6600-0x67FF) */
    PCKCHR_t UDS_SwVersion[1];                                                  /**< 0x2F-0x31: UDS Software Version[4] (0x6400-0x65FF) */
    PCKCHR_t UDS_SparePartNumber[3];                                            /**< 0x32-0x3A: UDS Spare part number[11] (0x6200-0x63FF) */
    uint8_t UDS_ReservedB;                                                      /**< 0x3B: Reserved */
    uint16_t u16Reserved[2];                                                    /**< 0x3C-0x3F: Reserved */
} UDS_LIN_PARAMS_t;
#endif /* (_SUPPORT_UDS != FALSE) */
#endif /* (LIN_COMM != FALSE) */

/*! 3) APP_EOL: 1 block of 64-bits (12x 16-bits words) */
typedef struct
{
    uint16_t u8CRC8                   : 8;                                      /**< 0x00: CRC8 */
    uint16_t u7EmergencyRunPos        : 7;                                      /**< 0x01.[6:0]: Emergency/Safety position (0-100%) */
    uint16_t u1EmergencyRunPosEna     : 1;                                      /**< 0x01.[7]: Emergency/Safety position Disable/Enable */
    uint16_t u1MotorDirectionCCW      : 1;                                      /**< 0x02.[0]: Motor rotational direction: 0=CW, 1=CCW */
#define C_MOTOR_ROTATION_CW         0U                                          /*!< Motor rotational Direction: Clock Wise */
#define C_MOTOR_ROTATION_CCW        1U                                          /*!< Motor rotational Direction: Counter Clock Wise */
    uint16_t u1StallDetectorEna       : 1;                                      /**< 0x02.[1]: Stall-detector: 0=Disabled, 1=Enabled */
#define C_STALLDET_DIS              0U                                          /*!< Stall-Detector: Disabled */
#define C_STALLDET_ENA              1U                                          /*!< Stall-Detector: Enabled */
    uint16_t u1RealTravelSaved        : 1;                                      /**< 0x02.[2]: Travel Saved */
    uint16_t u1PorCalibration         : 1;                                      /**< 0x02.[3]: Power-on calibration */
    uint16_t u4Reserved_2             : 4;                                      /**< 0x02.[7:4]: Reserved */
#if FALSE
    uint16_t u2DefaultSpeed           : 2;                                      /**< 0x02.[5:4]: Motor default Speed mode */
    #define C_DEFSPEEDMODE_AUTO         0U
    #define C_DEFSPEEDMODE_1            1U
    #define C_DEFSPEEDMODE_2            2U
    #define C_DEFSPEEDMODE_3            3U
    uint16_t u2DefaultTorque          : 2;                                      /**< 0x02.[7:6]: Motor default Torque level */
    #define C_DEFTORQUELEVEL_NORMAL     0U
    #define C_DEFTORQUELEVEL_LOW        1U
    #define C_DEFTORQUELEVEL_HIGH       2U
    #define C_DEFTORQUELEVEL_MAX        3U
#endif
    uint16_t u8EndStopTime            : 8;                                      /**< 0x03: End-stop pause time */
    uint16_t u16RealTravel;                                                     /**< 0x04: Real Travel */
    uint16_t u8TravelToleranceLo      : 8;                                      /**< 0x06: Default Travel Tolerance (Lower) */
    uint16_t u8TravelToleranceUp      : 8;                                      /**< 0x07: Default Travel Tolerance (Upper) */
} APP_EOL_t;

#if (_SUPPORT_APP_SAVE != FALSE)
/*! 4) APP_PARAMS: 1 blocks of 64-bits ( 4x 16-bits words) */
typedef struct
{
    uint16_t u8CRC8                    : 8;                                     /**< PARAM structure CRC8 */
    uint16_t u4PageID                  : 4;                                     /**< PARAM structure Page ID (XOR) */
    uint16_t u4WrtCntH                 : 4;                                     /**< PARAM structure write cycle counter (MSnibble) */
    uint16_t u16WrtCntL;                                                        /**< PARAM structure write cycle counter (LSW) */
    uint16_t u16ParamLSW;                                                       /**< PARAM structure Data (LSW) */
    uint16_t u16ParamMSW;                                                       /**< PARAM structure Data (MSW) */
} APP_PARAMS_t;
#endif /* (_SUPPORT_APP_SAVE != FALSE) */

/*! 5) ACT_PARAMS: 4 blocks of 64-bits (16x 16-bits words)
 *     ACT_PARAMS: 5 blocks of 64-bits (20x 16-bits words) (MLX81339 only) */
typedef struct
{
    uint16_t u8CRC8                   : 8;                                      /**< 0x00: CRC8 */
    uint16_t u8VsupRef                : 8;                                      /**< 0x01: Vsupply reference [1/8V] */
    uint16_t u12GearBoxRatio          : 12;                                     /**< 0x02 [11: 0]: Gear-box ratio, e.g. 600:1 */
    uint16_t u4PolePairs              : 4;                                      /**< 0x02.[15:12]: Number of pole-pairs + 1 (1-16) */
    uint16_t u8MotorConstant          : 8;                                      /**< 0x04: Motor Constant [mV/RPS] or [10mV/RPS] (MMP240726-2) */
    uint16_t u8MotorCoilRtot          : 8;                                      /**< 0x05: Motor coil resistance (total) */
    uint16_t u13MinSpeed              : 13;                                     /**< 0x06.[12: 0]: Minimum speed (0 ... 8191) */
    uint16_t u3MicroSteps             : 3;                                      /**< 0x06.[15:13]: Number of micro-steps: 2^n (or 1 << n); 0 = Full-steps */
    uint16_t u16Speed_1;                                                        /**< 0x08: Speed_1 */
    uint16_t u16Speed_2;                                                        /**< 0x0A: Speed_2 */
    uint16_t u16Speed_3;                                                        /**< 0x0C: Speed_3 */
#if (LINPROT != LIN2X_AIRVENT12)
    uint16_t u16Speed_4;                                                        /**< 0x0E: Speed_4 */
#else  /* (LINPROT != LIN2X_AIRVENT12) */
    uint16_t u16TorqueSpeed;                                                    /**< 0x0E: Torque-mode Speed */
#endif /* (LINPROT != LIN2X_AIRVENT12) */
    uint16_t u16AccelerationConst;                                              /**< 0x10: Acceleration-constant */
    uint16_t u3AccelerationSteps      : 3;                                      /**< 0x12.[2:0]: Acceleration-(u)Steps */
    uint16_t u3DecelerationSteps      : 3;                                      /**< 0x12.[5:3]: Deceleration-(u)Steps */
    uint16_t u2MotorCurrentMultiplier : 2;                                      /**< 0x12.[7:6]: Motor current multiplier: 1, 2, 4 or 8 */
    uint16_t u8HoldingTorqueCurrent   : 8;                                      /**< 0x13: Holding Torque current threshold */
    uint16_t u8RunningTorqueCurrent   : 8;                                      /**< 0x14: Running Torque current threshold */
    uint16_t u4TorqueBoost1           : 4;                                      /**< 0x15.[3:0]: TorqueBoost1 (5%) -25%..50% */
    uint16_t u4TorqueBoost2           : 4;                                      /**< 0x15.[7:4]: TorqueBoost2 (5%)   0%..75% */
    uint16_t u8PidCoefP               : 8;                                      /**< 0x16: Speed PID-Coefficient P */
    uint16_t u8PidCoefI               : 8;                                      /**< 0x17: Speed PID-Coefficient I */
    uint16_t u8PidCoefD               : 8;                                      /**< 0x18: Speed PID-Coefficient D */
    uint16_t u8PidStartupOrLowerHoldingLimit : 8;                               /**< 0x19: Startup-current limit/PID Lower-limit Holding (output) */
    uint16_t u8PidLowerLimit          : 8;                                      /**< 0x1A: PID Lower-limit (output) */
    uint16_t u8PidUpperLimit          : 8;                                      /**< 0x1B: PID Upper-limit (output) */
    uint16_t u7PidCtrlPeriod          : 7;                                      /**< 0x1C.[6:0]: PID running control-period */
    uint16_t u1PidPeriodTimeOrSpeed   : 1;                                      /**< 0x1C.[7]: PID Period is speed or Time based */
    uint16_t u8AppOT                  : 8;                                      /**< 0x1D: Application Over-Temperature [C] */
    uint16_t u8AppUV                  : 8;                                      /**< 0x1E: Application Under-voltage level [1/8V] */
    uint16_t u8AppOV                  : 8;                                      /**< 0x1F: Application Over-voltage level [1/8V] */
#if defined (__MLX81339__)
    uint16_t u16Inductance;                                                     /**< 0x20: Coil Inductance [uH] */
    uint16_t u8RampUpSpeedLimit       : 8;                                      /**< 0x22: Ramp-up Speed Limit */
    uint16_t u8RampDownSpeedLimit     : 8;                                      /**< 0x23: Ramp-down Speed Limit */
    uint16_t u3ChipUV                 : 3;                                      /**< 0x24: Chip Under Voltage */
    uint16_t u2ChipOV                 : 2;                                      /**< 0x24: Chip Over Voltage */
    uint16_t u3Reserved               : 3;                                      /**< 0x24: Reserved */
    uint16_t u8AlignmentSpeed         : 8;                                      /**< 0x25: Alignment Speed */
    uint16_t u16Reserved;                                                       /**< 0x26: Reserved */
#endif /* defined (__MLX81339__) */
} ACT_PARAMS_t;

#if (_SUPPORT_TRIAXIS_MLX90363 != FALSE) || (_SUPPORT_TRIAXIS_MLX9038x != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90367 != FALSE) || (_SUPPORT_TRIAXIS_MLX90372 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90377 != FALSE) || (_SUPPORT_TRIAXIS_MLX90395 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90421 != FALSE) || (_SUPPORT_TRIAXIS_MLX90422 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90425 != FALSE) || (_SUPPORT_TRIAXIS_MLX90426 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90427 != FALSE) || \
    (_SUPPORT_INDUCTIVE_POS_SENSOR_MLX90513 != FALSE) || \
    (_SUPPORT_DUAL_HALL_LATCH_MLX92251 != FALSE) || (_SUPPORT_DUAL_HALL_LATCH_MLX92255 != FALSE) || \
    (_SUPPORT_FOC_MODE != FOC_MODE_NONE)
/*! 6) SENSOR: 1 block  of 64-bits (4x 16-bits words) */
typedef struct
{
    uint16_t u8CRC8                   : 8;                                      /**< 0x00: CRC8 */
    uint16_t u4SensorType             : 4;                                      /**< 0x01[3:0]: Sensor-type */
#define C_SENSOR_MLX90363               1U                                      /*!< Triaxis with SPI; 4-wire interface; 3.3V */
#define C_SENSOR_MLX90380               2U                                      /*!< Resolver; 2-wire-analogue; 3.3V */
#define C_SENSOR_MLX90381               2U                                      /*!< Resolver; 2-wire-analogue; 3.3V */
#define C_SENSOR_MLX92251               3U                                      /*!< Dual Hall-Latch; 2-wire interface; 3.3V */
#define C_SENSOR_MLX92255               4U                                      /*!< Dual Hall-Latch; 2-wire interface; 3.3V */
#define C_SENSOR_MLX90367               5U                                      /*!< Triaxis with SENT; 1-wire interface; 5.0V */
#define C_SENSOR_MLX90372               5U                                      /*!< Triaxis with SENT; 1-wire interface; 5.0V */
#define C_SENSOR_MLX90421               6U                                      /*!< Triaxis with PWM; 1-wire interface; 5.0V */
#define C_SENSOR_MLX90377               7U                                      /*!< Triaxis with SENT/SPC; 1-wire interface; 5.0V */
#define C_SENSOR_MLX90513               8U                                      /*!< Inductive Position Sensor with SENT/SPC/PWM; 1-wire interface; 5.0V */
#define C_SENSOR_MLX90395               9U                                      /*!< Triaxis with SPI; 4-wire interface; 3.3V */
#define C_SENSOR_MLX90422               10U                                     /*!< Triaxis with SENT; 1-wire interface; 5.0V */
#define C_SENSOR_MLX90425               11U                                     /*!< Triaxis with PWM; 1-wire interface; 5.0V */
#define C_SENSOR_MLX90426               12U                                     /*!< Triaxis with SENT; 1-wire interface; 5.0V */
#define C_SENSOR_MLX90427               13U                                     /*!< Triaxis with SPI; 4-wire interface; 3.3V */
#define C_SENSOR_FOC                    15U                                     /*!< FOC */
    uint16_t u3Reserved               : 3;                                      /**< 0x01[6:4]: Reserved */
    uint16_t u1CDI                    : 1;                                      /**< 0x01[7]: CDI On/Off */
#if (_SUPPORT_TRIAXIS_MLX90363 != FALSE) || (_SUPPORT_TRIAXIS_MLX9038x != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90367 != FALSE) || (_SUPPORT_TRIAXIS_MLX90372 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90377 != FALSE) || (_SUPPORT_TRIAXIS_MLX90395 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90421 != FALSE) || (_SUPPORT_TRIAXIS_MLX90422 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90425 != FALSE) || (_SUPPORT_TRIAXIS_MLX90426 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90427 != FALSE) || \
    (_SUPPORT_DUAL_HALL_LATCH_MLX92251 != FALSE) || (_SUPPORT_DUAL_HALL_LATCH_MLX92255 != FALSE)
    uint16_t u8TriaxisOffsetX         : 8;                                      /**< 0x02: Triaxis Offset X */
    uint16_t u8TriaxisOffsetY         : 8;                                      /**< 0x03: Triaxis Offset Y */
    uint16_t u4TriaxisMagnetPolePairs : 4;                                      /**< 0x04.[3:0]: Triaxis Magnet Pole Pairs */
    uint16_t u11TriaxisAmplitudeCorrection : 11;                                /**< 0x04.[14:4]: Triaxis X/Y-amplitude correction */
    uint16_t u1TriaxisDirection       : 1;                                      /**< 0x04.[15]: Triaxis Direction (0 = same as Act; 1 = Reverse) */
    uint16_t u16TriaxisAngleOffset;                                             /**< 0x06: Triaxis Angle Offset */
#elif (_SUPPORT_INDUCTIVE_POS_SENSOR_MLX90513 != FALSE)
    uint16_t u8ReservedX              : 8;                                      /**< 0x02: Reserved X */
    uint16_t u8ReservedY              : 8;                                      /**< 0x03: Reserved Y */
    uint16_t u4Reserved               : 4;                                      /**< 0x04.[3:0]: Reserved */
    uint16_t u11Reserved              : 11;                                     /**< 0x04.[14:4]: Reserved */
    uint16_t u1TriaxisDirection       : 1;                                      /**< 0x04.[15]: Triaxis Direction (0 = same as Act; 1 = Reverse) */
    uint16_t u16TriaxisAngleOffset;                                             /**< 0x06: Triaxis Angle Offset */
#elif (_SUPPORT_FOC_MODE != FOC_MODE_NONE)
    uint16_t u8PidCoefP_LA            : 8;                                      /**< 0x02: PID Coefficient P for Load-angle */
    uint16_t u8PidCoefI_LA            : 8;                                      /**< 0x03: PID Coefficient I for Load-angle */
    uint16_t u8PidCoefD_LA            : 8;                                      /**< 0x04: PID Coefficient D for Load-angle */
    uint16_t u8TargetLA               : 8;                                      /**< 0x05: Target Load-Angle */
    uint16_t u4StallLA_Threshold      : 4;                                      /**< 0x06: Stall-detector "LA" angle-threshold (5 degrees) (MMP220720-1) */
    uint16_t u4StallLA_Width          : 4;                                      /**< 0x06: Stall-detector "LA" width (MMP220720-1) */
    uint16_t u3OpenToCloseAmplitude   : 3;                                      /**< 0x07[2:0]: Open-to-Close amplitude factor */
    uint16_t u5StallSpeed_Threshold   : 5;                                      /**< 0x07[7:3]: Stall-detector "Speed" threshold */
#endif
#if FALSE /* Enhanced */
    uint16_t u8ReservedA              : 8;                                      /**< 0x08: Reserved A */
    uint16_t u8ReservedB              : 8;                                      /**< 0x09: Reserved A */
    uint16_t u8PidCoefP_IPK_IQ        : 8;                                      /**< 0x0A: PID Coefficient P for Load-angle */
    uint16_t u8PidCoefI_IPK_IQ        : 8;                                      /**< 0x0B: PID Coefficient I for Load-angle */
    uint16_t u8PidCoefD_IPK_IQ        : 8;                                      /**< 0x0C: PID Coefficient D for Load-angle */
    uint16_t u8PidCoefP_ID            : 8;                                      /**< 0x0D: PID Coefficient P for Load-angle */
    uint16_t u8PidCoefI_ID            : 8;                                      /**< 0x0E: PID Coefficient I for Load-angle */
    uint16_t u8PidCoefD_ID            : 8;                                      /**< 0x0F: PID Coefficient D for Load-angle */
#endif
} SENSOR_t;
#endif /* (_SUPPORT_TRIAXIS_MLX90363 != FALSE) || (_SUPPORT_TRIAXIS_MLX9038x != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90367 != FALSE) || (_SUPPORT_TRIAXIS_MLX90372 != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90377 != FALSE) || (_SUPPORT_TRIAXIS_MLX90395 != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90421 != FALSE) || (_SUPPORT_TRIAXIS_MLX90422 != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90425 != FALSE) || (_SUPPORT_TRIAXIS_MLX90426 != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90427 != FALSE) || \
        * (_SUPPORT_INDUCTIVE_POS_SENSOR_MLX90513 != FALSE) || \
        * (_SUPPORT_DUAL_HALL_LATCH_MLX92251 != FALSE) || (_SUPPORT_DUAL_HALL_LATCH_MLX92255 != FALSE) || \
        * (_SUPPORT_FOC_MODE != FOC_MODE_NONE) */

/*! 7) ACT_STALL: 1 block  of 64-bits (4x 16-bits words) (MMP190412-1) */
typedef struct
{
    uint16_t u8CRC8                   : 8;                                      /**< 0x00: CRC8 */
    uint16_t u7StallA_Threshold       : 7;                                      /**< 0x01.[6:0]: Stall-detector "A" Threshold (Amplitude) */
    uint16_t u1StallA_Ena             : 1;                                      /**< 0x01.[7]: Stall-detector "A" Enable */
    uint16_t u4StallA_Width           : 4;                                      /**< 0x02.[3:0]: Stall-detector "A" Width */
    uint16_t u4StallO_Width           : 4;                                      /**< 0x02.[3:0]: Stall-detector "O" Width */
    uint16_t u7StallO_Threshold       : 7;                                      /**< 0x03.[6:0]: Stall-detector "O" Threshold (Oscillation) */
    uint16_t u1StallO_Ena             : 1;                                      /**< 0x03.[7]: Stall-detector "O" Enable */
    uint16_t u7StallS_Threshold       : 7;                                      /**< 0x04.[6:0]: Stall-detector "S" Threshold (Speed) */
    uint16_t u1StallS_Ena             : 1;                                      /**< 0x04.[7]: Stall-detector "S" Enable */
    uint16_t u4StallS_Width           : 4;                                      /**< 0x05.[3:0]: Stall-detector "S" Width */
    uint16_t u1RestallPor             : 1;                                      /**< 0x05.[4]: Re-stall after POR disable/enable */
    uint16_t u1StallSpeedDepended     : 1;                                      /**< 0x05.[5]: Stall speed depended */
    uint16_t u2Reserved               : 2;                                      /**< 0x05.[7:6]: Reserved */
    uint16_t u8RewindSteps            : 8;                                      /**< 0x06: Rewind full steps */
    uint16_t u8StallDetectorDelay     : 8;                                      /**< 0x07: Stall detector delay */
#if FALSE /* Enhanced */
    uint16_t u7StallP_Threshold       : 7;                                      /**< 0x08.[6:0]: Stall-detector "P" Threshold (Position) */
    uint16_t u1StallP_Ena             : 1;                                      /**< 0x08.[7]: Stall-detector "P" Enable */
    uint16_t u4StallP_Width           : 4;                                      /**< 0x09.[3:0]: Stall-detector "P" Width */
    uint16_t u4Reserved               : 4;                                      /**< 0x09.[7:4]: Reserved */
    uint16_t u16ReservedA;                                                      /**< 0x0A: Reserved A */
    uint16_t u16ReservedB;                                                      /**< 0x0C: Reserved B */
    uint16_t u16ReservedC;                                                      /**< 0x0E: Reserved C */
#endif
} ACT_STALL_t;
#define C_STALL_THRESHOLD_SDIV        7U                                        /*!< Stall-threshold shift-divider */
#define C_STALL_THRESHOLD_DIV         (1 << C_STALL_THRESHOLD_SDIV)             /*!< Stall-threshold divider */

#if (_SUPPORT_NV_LOG_ERROR != FALSE)
/*! 8) APP_LOG: 2 block  of 64-bits (8x 16-bits words) */
typedef struct
{
    uint8_t u8LogIdx;                                                           /**< Application Log write-index */
    uint8_t au8LogData[15];                                                     /**< Application Log array */
} APP_LOG_t;
#endif /* (_SUPPORT_NV_LOG_ERROR != FALSE) */

#if (I2C_COMM != FALSE)
/*! 9) STD_I2C_PARAMS_t: 1 block of 64-bits of I2C parameters */
typedef struct
{
    uint16_t u8CRC8                   : 8;                                      /*!< 0x00: CRC8 */
    uint16_t u8Address                : 8;                                      /*!< 0x01: Slave Address */
    uint16_t u16ReservedA;                                                      /*!< 0x02: Reserved A */
    uint16_t u16ReservedB;                                                      /*!< 0x04: Reserved B */
    uint16_t u16ReservedC;                                                      /*!< 0x06: Reserved C */
} STD_I2C_PARAMS_t;
#endif /* (I2C_COMM != FALSE) */

#if (CAN_COMM != FALSE)
/*! 9) STD_CAN_PARAMS_t: 2 block of 64-bits of CAN parameters */
typedef struct
{
    uint16_t u8CRC8                   : 8;                                      /**< 0x00: CRC8 */
    uint16_t u8Baudrate               : 8;                                      /**< 0x01: CAN Baudrate */
    uint16_t u8StatusPeriod           : 8;                                      /**< 0x02: CAN Status Period */
    uint16_t u8FDBaudrate             : 8;                                      /**< 0x03: CAN FD Baudrate */
    uint16_t u16IdInLSW;                                                        /**< 0x04: CAN-in ID (LSW) */
    uint16_t u16IdInMSW;                                                        /**< 0x06: CAN-in ID (MSW) */
    uint16_t u16IdOutLSW;                                                       /**< 0x08: CAN-out ID (LSW) */
    uint16_t u16IdOutMSW;                                                       /**< 0x0A: CAN-out ID (MSW) */
    uint16_t u16ReservedB;                                                      /**< 0x0C: Reserved B */
    uint16_t u16ReservedC;                                                      /**< 0x0E: Reserved C */
} STD_CAN_PARAMS_t;
#endif /* (CAN_COMM != FALSE) */

/*! x) BOOTLOADER PARAMS: 1 block of 64-bits (4x 16-bits words) */
typedef struct
{
    uint16_t u16ReservedA;                                                      /**< 0x00: Reserved A */
    uint16_t u16ReservedB;                                                      /**< 0x02: Reserved B */
    uint16_t u16ReservedC;                                                      /**< 0x04: Reserved C */
    uint16_t u8Address                : 8;                                      /**< 0x06: Slave Address */
    uint16_t u8CRC                    : 8;                                      /**< 0x07: Reserved for CRC (not used) */
} BOOTLOADER_PARAMS_t;

/*! user-application structure Non Volatile Memory */
typedef struct
{
    HEADER_t hdr;                                                               /**< 1) header */
#if (LIN_COMM != FALSE)
    STD_LIN_PARAMS_t stdlin[2];                                                 /**< 2a) Standard LIN parameters */
#if (LINPROT == LIN2X_HVAC52)
    ENH_LIN_PARAMS_t enhlin[2];                                                 /**< 2b) Enhanced LIN parameters */
#endif /* (LINPROT == LIN2X_HVAC52) */
#if (_SUPPORT_UDS != FALSE)
    UDS_LIN_PARAMS_t uds;                                                       /**< 2c) UDS Data */
#endif /* (_SUPPORT_UDS != FALSE) */
#endif /* (LIN_COMM != FALSE) */
    APP_EOL_t eol;                                                              /**< 3) Application EOL parameters */
#if (_SUPPORT_APP_SAVE != FALSE)
    APP_PARAMS_t app;                                                           /**< 4) Application parameters */
#endif /* (_SUPPORT_APP_SAVE != FALSE) */
    ACT_PARAMS_t act;                                                           /**< 5) Actuator Parameters */
#if (_SUPPORT_TRIAXIS_MLX90363 != FALSE) || (_SUPPORT_TRIAXIS_MLX9038x != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90367 != FALSE) || (_SUPPORT_TRIAXIS_MLX90372 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90377 != FALSE) || (_SUPPORT_TRIAXIS_MLX90395 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90421 != FALSE) || (_SUPPORT_TRIAXIS_MLX90422 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90425 != FALSE) || (_SUPPORT_TRIAXIS_MLX90426 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90427 != FALSE) || \
    (_SUPPORT_INDUCTIVE_POS_SENSOR_MLX90513 != FALSE) || \
    (_SUPPORT_DUAL_HALL_LATCH_MLX92251 != FALSE) || (_SUPPORT_DUAL_HALL_LATCH_MLX92255 != FALSE) || \
    (_SUPPORT_FOC_MODE != FOC_MODE_NONE)
    SENSOR_t sensor;                                                            /**< 6) Sensor parameters */
#endif /* (_SUPPORT_TRIAXIS_MLX90363 != FALSE) || (_SUPPORT_TRIAXIS_MLX9038x != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90367 != FALSE) || (_SUPPORT_TRIAXIS_MLX90372 != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90377 != FALSE) || (_SUPPORT_TRIAXIS_MLX90395 != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90421 != FALSE) || (_SUPPORT_TRIAXIS_MLX90422 != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90425 != FALSE) || (_SUPPORT_TRIAXIS_MLX90426 != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90427 != FALSE) || \
        * (_SUPPORT_INDUCTIVE_POS_SENSOR_MLX90513 != FALSE) || \
        * (_SUPPORT_DUAL_HALL_LATCH_MLX92251 != FALSE) || (_SUPPORT_DUAL_HALL_LATCH_MLX92255 != FALSE) || \
        * (_SUPPORT_FOC_MODE != FOC_MODE_NONE) */
    ACT_STALL_t stall;                                                          /**< 7) Stall parameters */
#if (_SUPPORT_NV_LOG_ERROR != FALSE)
    APP_LOG_t errlog;                                                           /**< 8) Error Log */
#endif /* (_SUPPORT_NV_LOG_ERROR != FALSE) */
#if (I2C_COMM != FALSE)
    STD_I2C_PARAMS_t i2c;                                                       /**< 9) LIN Parameters */
#endif /* (I2C_COMM != FALSE) */
#if (CAN_COMM != FALSE)
    STD_CAN_PARAMS_t can;                                                       /**< 10) CAN Parameters */
#endif /* (CAN_COMM != FALSE) */
} NV_USER_MAP_t;
extern volatile NV_USER_MAP_t UserParams __attribute__((nodp, addr(ADDR_NV_USER)));  /*lint !e526 */  /*!< UserParams mapped to Non Volatile Memory space */

#define SZ_NV_HDR           sizeof(HEADER_t)                                    /*!< 1 : Size-of-USER_1 */
#if (LIN_COMM != FALSE)
#define SZ_NV_STD_LIN       sizeof(STD_LIN_PARAMS_t)                            /*!< 2a: Size-of-USER_2 */
#if (LINPROT == LIN2X_HVAC52)
#define SZ_NV_ENH_LIN       sizeof(ENH_LIN_PARAMS_t)                            /*!< 2b: Size-of-USER_3 */
#endif /* (LINPROT == LIN2X_HVAC52) */
#if (_SUPPORT_UDS != FALSE)
#define SZ_NV_UDS           sizeof(UDS_LIN_PARAMS_t)                            /*!< 2c: Size-of-USER_4 */
#endif /* (_SUPPORT_UDS != FALSE) */
#endif /* (LIN_COMM != FALSE) */
#define SZ_NV_EOL           sizeof(APP_EOL_t)                                   /*!< 3 : Size-of-USER_5 */
#if (_SUPPORT_APP_SAVE != FALSE)
#define SZ_NV_APP_PARAMS    sizeof(APP_PARAMS_t)                                /*!< 4 : Size-of-USER_6 */
#endif /* (_SUPPORT_APP_SAVE != FALSE) */
#define SZ_NV_ACT_PARAMS    sizeof(ACT_PARAMS_t)                                /*!< 5 : Size-of-USER_7 */
#if (_SUPPORT_TRIAXIS_MLX90363 != FALSE) || (_SUPPORT_TRIAXIS_MLX9038x != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90367 != FALSE) || (_SUPPORT_TRIAXIS_MLX90372 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90377 != FALSE) || (_SUPPORT_TRIAXIS_MLX90395 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90421 != FALSE) || (_SUPPORT_TRIAXIS_MLX90422 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90425 != FALSE) || (_SUPPORT_TRIAXIS_MLX90426 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90427 != FALSE) || \
    (_SUPPORT_INDUCTIVE_POS_SENSOR_MLX90513 != FALSE) || \
    (_SUPPORT_DUAL_HALL_LATCH_MLX92251 != FALSE) || (_SUPPORT_DUAL_HALL_LATCH_MLX92255 != FALSE) || \
    (_SUPPORT_FOC_MODE != FOC_MODE_NONE)
#define SZ_NV_SENSOR        sizeof(SENSOR_t)                                    /*!< 6 : Size-of-USER_8 */
#endif /* (_SUPPORT_TRIAXIS_MLX90363 != FALSE) || (_SUPPORT_TRIAXIS_MLX9038x != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90367 != FALSE) || (_SUPPORT_TRIAXIS_MLX90372 != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90377 != FALSE) || (_SUPPORT_TRIAXIS_MLX90395 != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90421 != FALSE) || (_SUPPORT_TRIAXIS_MLX90422 != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90425 != FALSE) || (_SUPPORT_TRIAXIS_MLX90426 != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90427 != FALSE) || \
        * (_SUPPORT_INDUCTIVE_POS_SENSOR_MLX90513 != FALSE) || \
        * (_SUPPORT_DUAL_HALL_LATCH_MLX92251 != FALSE) || (_SUPPORT_DUAL_HALL_LATCH_MLX92255 != FALSE) || \
        * (_SUPPORT_FOC_MODE != FOC_MODE_NONE) */
#define SZ_NV_ACT_STALL     sizeof(ACT_STALL_t)                                 /*!< 7 : Size-of-USER_9 */
#if (_SUPPORT_NV_LOG_ERROR != FALSE)
#define SZ_NV_ERRORLOG      sizeof(APP_LOG_t)                                   /*!< 8 : Size-of-USER_10 */
#endif /* (_SUPPORT_NV_LOG_ERROR != FALSE) */
#if (I2C_COMM != FALSE)
#define SZ_NV_I2C_PARAMS    sizeof(STD_I2C_PARAMS_t)                            /*!< 9 : Size-of-USER_11 */
#endif /* (I2C_COMM != FALSE) */
#if (CAN_COMM != FALSE)
#define SZ_NV_CAN_PARAMS    sizeof(STD_CAN_PARAMS_t)                            /*!< 10: Size-of-USER_12 */
#endif /* (CAN_COMM != FALSE) */

#define ADDR_NV_USER_1      ((uint16_t)ADDR_NV_USER)                            /*!< Address for user-area #01 (Begin of Non Volatile Memory User Area) */
/* Non Volatile Memory Block address #1: Header */
#define ADDR_NV_HDR         ((uint16_t)ADDR_NV_USER_1)                          /*!< Address for user-area #01: Header */
#define ADDR_NV_USER_2      ((uint16_t)(ADDR_NV_HDR + SZ_NV_HDR))               /*!< Address for user-area #02 */
/* Non Volatile Memory Block address #2: Standard LIN */
#if (LIN_COMM != FALSE)
#define ADDR_NV_STD_LIN_1   ((uint16_t)ADDR_NV_USER_2)                          /*!< Address for user-area #02: Standard LIN #1 */
#define ADDR_NV_STD_LIN_2   ((uint16_t)(ADDR_NV_STD_LIN_1 + SZ_NV_STD_LIN))     /*!< Address for user-area #02: Standard LIN #2 */
#define ADDR_NV_USER_3      ((uint16_t)(ADDR_NV_STD_LIN_2 + SZ_NV_STD_LIN))     /*!< Address for user-area #03 */
#if (LINPROT == LIN2X_HVAC52)
/* Non Volatile Memory Block address #3: Enhanced LIN */
#define ADDR_NV_ENH_LIN_1   ((uint16_t)ADDR_NV_USER_3)                          /*!< Address for user-area #03: Enhanced LIN #1 */
#define ADDR_NV_ENH_LIN_2   ((uint16_t)(ADDR_NV_ENH_LIN_1 + SZ_NV_ENH_LIN))     /*!< Address for user-area #03: Enhanced LIN #2 */
#define ADDR_NV_USER_4      ((uint16_t)(ADDR_NV_ENH_LIN_2 + SZ_NV_ENH_LIN))     /*!< Address for user-area #04 */
#else
#define ADDR_NV_USER_4      ((uint16_t)ADDR_NV_USER_3)                          /*!< Address for user-area #04 */
#endif
#if (_SUPPORT_UDS != FALSE)
/* Non Volatile Memory Block address #4: UDS LIN */
#define ADDR_NV_UDS         ((uint16_t)ADDR_NV_USER_4)                          /*!< Address for user-area #04: LIN UDS */
#define ADDR_NV_USER_5      ((uint16_t)(ADDR_NV_UDS + SZ_NV_UDS))               /*!< Address for user-area #05 */
#else  /* (_SUPPORT_UDS != FALSE) */
#define ADDR_NV_USER_5      ((uint16_t)ADDR_NV_USER_4)                          /*!< Address for user-area #05 */
#endif /* (_SUPPORT_UDS != FALSE) */
#else  /* (LIN_COMM != FALSE) */
#define ADDR_NV_USER_5      ((uint16_t)ADDR_NV_USER_2)                          /*!< Address for user-area #05 */
#endif /* (LIN_COMM != FALSE) */
/* Non Volatile Memory Block address #5: End-of-Line */
#define ADDR_NV_EOL         ((uint16_t)ADDR_NV_USER_5)                          /*!< Address for user-area #05: End-of-Line Parameters */
#define ADDR_NV_USER_6      ((uint16_t)(ADDR_NV_EOL + SZ_NV_EOL))               /*!< Address for user-area #06 */
#if (_SUPPORT_APP_SAVE != FALSE)
/* Non Volatile Memory Block address #6: Application Parameters */
#define ADDR_NV_APP_PARAMS  ((uint16_t)ADDR_NV_USER_6)                          /*!< Address for user-area #06: Application Parameters (#1) */
#define ADDR_NV_USER_7      ((uint16_t)(ADDR_NV_APP_PARAMS + SZ_NV_APP_PARAMS))  /*!< Address for user-area #07 */
#else  /* (_SUPPORT_APP_SAVE != FALSE) */
#define ADDR_NV_USER_7      ((uint16_t)ADDR_NV_USER_6)                          /*!< Address for user-area #07 */
#endif /* (_SUPPORT_APP_SAVE != FALSE) */
/* Non Volatile Memory Block address #7: Actuator Parameters */
#define ADDR_NV_ACT_PARAMS  ((uint16_t)ADDR_NV_USER_7)                          /*!< Address for user-area #07: Actuator Parameters */
#define ADDR_NV_USER_8      ((uint16_t)(ADDR_NV_ACT_PARAMS + SZ_NV_ACT_PARAMS))  /*!< Address for user-area #08*/
/* Non Volatile Memory Block address #8: Sensor/FOC */
#if (_SUPPORT_TRIAXIS_MLX90363 != FALSE) || (_SUPPORT_TRIAXIS_MLX9038x != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90367 != FALSE) || (_SUPPORT_TRIAXIS_MLX90372 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90377 != FALSE) || (_SUPPORT_TRIAXIS_MLX90395 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90421 != FALSE) || (_SUPPORT_TRIAXIS_MLX90422 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90425 != FALSE) || (_SUPPORT_TRIAXIS_MLX90426 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90427 != FALSE) || \
    (_SUPPORT_INDUCTIVE_POS_SENSOR_MLX90513 != FALSE) || \
    (_SUPPORT_DUAL_HALL_LATCH_MLX92251 != FALSE) || (_SUPPORT_DUAL_HALL_LATCH_MLX92255 != FALSE) || \
    (_SUPPORT_FOC_MODE != FOC_MODE_NONE)
#define ADDR_NV_SENSOR      ((uint16_t)ADDR_NV_USER_8)                          /*!< Address for user-area #08: Sensor or FOC parameters */
#define ADDR_NV_USER_9      ((uint16_t)(ADDR_NV_SENSOR + SZ_NV_SENSOR))         /*!< Address for user-area #09 */
#else  /* (_SUPPORT_TRIAXIS_MLX90363 != FALSE) || (_SUPPORT_TRIAXIS_MLX9038x != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90367 != FALSE) || (_SUPPORT_TRIAXIS_MLX90372 != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90377 != FALSE) || (_SUPPORT_TRIAXIS_MLX90395 != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90421 != FALSE) || (_SUPPORT_TRIAXIS_MLX90422 != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90425 != FALSE) || (_SUPPORT_TRIAXIS_MLX90426 != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90427 != FALSE) || \
        * (_SUPPORT_INDUCTIVE_POS_SENSOR_MLX90513 != FALSE) || \
        * (_SUPPORT_DUAL_HALL_LATCH_MLX92251 != FALSE) || (_SUPPORT_DUAL_HALL_LATCH_MLX92255 != FALSE) || \
        * (_SUPPORT_FOC_MODE != FOC_MODE_NONE) */
#define ADDR_NV_USER_9      ((uint16_t)ADDR_NV_USER_8)                          /*!< Address for user-area #09 */
#endif /* (_SUPPORT_TRIAXIS_MLX90363 != FALSE) || (_SUPPORT_TRIAXIS_MLX9038x != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90367 != FALSE) || (_SUPPORT_TRIAXIS_MLX90372 != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90377 != FALSE) || (_SUPPORT_TRIAXIS_MLX90395 != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90421 != FALSE) || (_SUPPORT_TRIAXIS_MLX90422 != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90425 != FALSE) || (_SUPPORT_TRIAXIS_MLX90426 != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90427 != FALSE) || \
        * (_SUPPORT_INDUCTIVE_POS_SENSOR_MLX90513 != FALSE) || \
        * (_SUPPORT_DUAL_HALL_LATCH_MLX92251 != FALSE) || (_SUPPORT_DUAL_HALL_LATCH_MLX92255 != FALSE) || \
        * (_SUPPORT_FOC_MODE != FOC_MODE_NONE) */
/* Non Volatile Memory Block address #9: Actuator Stall detectors */
#define ADDR_NV_ACT_STALL   ((uint16_t)ADDR_NV_USER_9)                          /*!< Address for user-area #09: Actuator Stall Parameters */
#define ADDR_NV_USER_10     ((uint16_t)(ADDR_NV_ACT_STALL + SZ_NV_ACT_STALL))   /*!< Address for user-area #10 */
#if (_SUPPORT_NV_LOG_ERROR != FALSE)
/* Non Volatile Memory Block address #10: Error-log */
#define ADDR_NV_ERRORLOG    ((uint16_t)ADDR_NV_USER_10)                         /*!< Address for user-area #10: Error-Log */
#define ADDR_NV_USER_11     ((uint16_t)(ADDR_NV_ERRORLOG + SZ_NV_ERRORLOG))     /*!< Address for user-area #11 */
#else  /* (_SUPPORT_NV_LOG_ERROR != FALSE) */
#define ADDR_NV_USER_11     ADDR_NV_USER_10                                     /*!< Address for user-area #11 */
#endif /* (_SUPPORT_NV_LOG_ERROR != FALSE) */
/* Non Volatile Memory Block address #11: I2C Parameters */
#if (I2C_COMM != FALSE)
#define ADDR_NV_I2C_PARAMS  ((uint16_t)ADDR_NV_USER_11)                         /*!< Address for user-area #11: I2C Parameters */
#define ADDR_NV_USER_12     ((uint16_t)(ADDR_NV_I2C_PARAMS + SZ_NV_I2C_PARAMS))  /*!< Address for user-area #12 */
#else  /* (I2C_COMM != FALSE) */
#define ADDR_NV_USER_12     ((uint16_t)ADDR_NV_USER_11)                         /*!< Address for user-area #12 */
#endif /* (I2C_COMM != FALSE) */
/* Non Volatile Memory Block address #12: CAN Parameters */
#if (CAN_COMM != FALSE)
#define ADDR_NV_CAN_PARAMS  ((uint16_t)ADDR_NV_USER_12)                         /*!< Address for user-area #12: CAN Parameters */
#define ADDR_NV_USER_13     ((uint16_t)(ADDR_NV_CAN_PARAMS + SZ_NV_CAN_PARAMS))  /*!< Address for user-area #13 */
#else  /* (CAN_COMM != FALSE) */
#define ADDR_NV_USER_13     ((uint16_t)ADDR_NV_USER_12)                         /*!< Address for user-area #13 */
#endif /* (CAN_COMM != FALSE) */
#define ADDR_NV_END         ((uint16_t)ADDR_NV_USER_13)                         /*!< End of application Non Volatile Memory area */
#if (_SUPPORT_DUAL_APP_BLOCK != FALSE)
#define ADDR_NV_APP_PARAMS_2 ((uint16_t)ADDR_NV_END)                            /*!< Application Parameters #2 */
#endif /* (_SUPPORT_DUAL_APP_BLOCK != FALSE) */

#if (_SUPPORT_NV_TYPE == C_NV_EEPROM)
#define ADDR_NV_BL          ((uint16_t)0x09A8U)                                 /*!< Boot-Loader Parameters */

#define C_MAX_NV_PROGRAM_COUNT          65000U                                  /*!< Maximum 65000 Write-cycles */
#elif (_SUPPORT_NV_TYPE == C_NV_FLASH)
#define ADDR_NV_BL          ((uint16_t)0x0878U)                                 /*!< Boot-Loader Parameters */

#define C_MAX_NV_PROGRAM_COUNT          10000U                                  /*!< Maximum 1000 Write-cycles */
#endif

#define C_NV_STORE_OKAY                 0x00U                                   /*!< Non Volatile Memory Write successful */
#define C_NV_STORE_MAX_WRITE_CYCLE      0x01U                                   /*!< Non Volatile Memory Write Maximum Write-cycles */
#define C_NV_STORE_INVALID_COUNTER      0x02U                                   /*!< Non Volatile Memory Write Invalid counter */
#define C_NV_STORE_WRITE_FAILED         0x03U                                   /*!< Non Volatile Memory Write Write failure */

#if (_SUPPORT_NV_MOTOR_PARAMS != FALSE)
/* **************************************************************************** *
 * Non Volatile Memory (UniROM) parameters (Code size increase by approximate 512B)
 * **************************************************************************** */
#if (_SUPPORT_BEMF_LEADANGLE_COMPENSATE != FALSE)
#define NV_LEADANGLE_COMPENSATION           C_LEADANGLE_COMPENSATION                                 /*!< Lead-angle compensation (n * 1.875 degrees, with n=0..15) */
#endif /* (_SUPPORT_BEMF_LEADANGLE_COMPENSATE != FALSE) */
#define NV_IC_UV_LEVEL                      ((uint16_t)C_IC_UV_LEVEL)                                /*!< Non Volatile Memory IC UV level */
#define NV_IC_OV_LEVEL                      ((uint16_t)C_IC_OV_LEVEL)                                /*!< Non Volatile Memory IC OV level */
#define NV_PWM_COMM_IN_ACTIVE_LEVEL         0U                                                       /*!< Active period: 0 = Low, 1 = High */
#define NV_PWM_COMM_IN_FILTER_TYPE          1U                                                       /*!< 0: Low-Pass Filter, 1: Stability filter */
#define NV_PWM_COMM_IN_STABLE_WIDTH         5U                                                       /*!< Number of PWM communication periods with same duty-cycle */
#define NV_PWM_COMM_IN_DC_STABLE            ((uint16_t)(0.005 * 65536U))                             /*!< PWM duty cycle stable within 0.5% tolerance */
#define NV_PWM_COMM_IN_DC_MIN               ((uint16_t)(0.100 * 65536U))                             /*!< PWM Communication minimum Duty cycle: 10% */
#define NV_PWM_COMM_IN_DC_MAX               ((uint16_t)(0.900 * 65536U))                             /*!< PWM Communication maximum Duty cycle: 90% */
#define NV_PWM_COMM_IN_DC_NV_WRT            ((uint16_t)(0.050 * 65536U))                             /*!< PWM Communication Duty Cycle for EE-Write: 5% */
#define NV_PWM_COMM_IN_TIMEOUT              (((10UL * C_PWM_COMM_IN_SAMPLE_FREQ) + 32768) / 65536U)    /*!< PWM Communication Time-out: 10s */
#define NV_PWM_COMM_IN_TO_ACTION            1U                                                       /*!< PWM Communication Time-out Action: 0 = Nothing, 1 = Stop motor & Reset */
#define NV_PID_HOLDINGCTRL_PER              ((uint16_t)(C_PID_HOLDINGCTRL_PERIOD << 1))              /*!< Non Volatile Memory PID Holding Control period (MMP240709-2) */
#define NV_PID_THRSHLDCTRL_PER              ((uint16_t)(C_PID_THRSHLDCTRL_PERIOD << 6))              /*!< Non Volatile Memory PID Current threshold compensation period (MMP240709-2) */
#define NV_MOTOR_COIL_LTOT                  ((uint16_t)C_TOT_COILS_L)                                /*!< Non Volatile Memory Actuator Coil Inductance */
#if (_SUPPORT_MOTOR_POSITION == C_MOTOR_POS_NONE)
#define NV_ALIGNMENT_SPEED                  ((uint16_t)C_SPEED_ALIGNMENT)                            /*!< Non Volatile Memory Application: Alignment Speed */
#endif /* (_SUPPORT_MOTOR_POSITION == C_MOTOR_POS_NONE) */
#if (_SUPPORT_TACHO_OUT != FALSE)
#define NV_TACHO_MODE                       ((uint16_t)C_TACHO_MODE)                                 /*!< Non Volatile Memory Application: Tacho-mode */
#endif /* (_SUPPORT_TACHO_OUT != FALSE) */
#define NV_PID_RAMP_UP                      ((uint16_t)C_PID_RAMP_UP)                                /*!< Non Volatile Memory Ramp-up speed limit */
#define NV_PID_RAMP_DOWN                    ((uint16_t)C_PID_RAMP_DOWN)                              /*!< Non Volatile Memory Ramp-down speed limit */
#if defined (__MLX81160__) || defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
#define NV_CURR_SENS_OC_DAC                 ((uint16_t)C_CURR_SENS_OC_DAC)                           /*!< Non Volatile Memory OC level (DAC) */
#endif /* defined (__MLX81160__) || defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__) */

#if ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_ID_IQ)
#define NV_PID_ID_COEF_P                    ((uint16_t)C_PID_ID_COEF_P)                              /*!< Non Volatile Memory PID Coefficient P for Id-path */
#define NV_PID_ID_COEF_I                    ((uint16_t)C_PID_ID_COEF_I)                              /*!< Non Volatile Memory PID Coefficient I for Id-path */
#define NV_PID_ID_COEF_D                    ((uint16_t)C_PID_ID_COEF_D)                              /*!< Non Volatile Memory PID Coefficient D for Id-path */
#define NV_PID_IQ_COEF_P                    ((uint16_t)C_PID_IQ_COEF_P)                              /*!< Non Volatile Memory PID Coefficient P for Iq-path */
#define NV_PID_IQ_COEF_I                    ((uint16_t)C_PID_IQ_COEF_I)                              /*!< Non Volatile Memory PID Coefficient I for Iq-path */
#define NV_PID_IQ_COEF_D                    ((uint16_t)C_PID_IQ_COEF_D)                              /*!< Non Volatile Memory PID Coefficient D for Iq-path */
#endif /* ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_ID_IQ) */

#define NV_APPL_OTEMP                       ( (int16_t)UserParams.act.u8AppOT - 60)                  /*!< Non Volatile Memory Application Over-temperature [C] */
#if (_APP_SUPPLY_RANGE != C_APP_SUPPLY_RANGE_48V)
#define NV_VSUP_REF                         ((uint16_t)(UserParams.act.u8VsupRef * 25U) >> 1)        /*!< Non Volatile Memory Reference supply [1/8V] to [10mV] */
#define NV_APPL_UVOLT                       ((uint16_t)(UserParams.act.u8AppUV * 25U) >> 1)          /*!< Non Volatile Memory Application Under-voltage [1/8V] to [10mV] */
#define NV_APPL_OVOLT                       ((uint16_t)((UserParams.act.u8AppOV * 25U) >> 1) + C_APP_OV_OFF)  /*!< Non Volatile Memory Application Over-voltage [1/8V] to [10mV] */
#else  /* (_APP_SUPPLY_RANGE != C_APP_SUPPLY_RANGE_48V) */
#define NV_VSUP_REF                         ((uint16_t)UserParams.act.u8VsupRef * 25U)               /*!< Non Volatile Memory Reference supply [1/4V] to [10mV] */
#define NV_APPL_UVOLT                       ((uint16_t)(UserParams.act.u8AppUV * 25U))               /*!< Non Volatile Memory Application Under-voltage [1/4V] to [10mV] */
#define NV_APPL_OVOLT                       ((uint16_t)(UserParams.act.u8AppOV * 25U) + C_APP_OV_OFF)  /*!< Non Volatile Memory Application Over-voltage [1/4V] to [10mV] */
#endif /* (_APP_SUPPLY_RANGE != C_APP_SUPPLY_RANGE_48V) */
#define NV_GEARBOX_RATIO                    ((uint16_t)UserParams.act.u12GearBoxRatio)               /*!< Non Volatile Memory Gear-box ratio */
#define NV_POLE_PAIRS                       ((uint16_t)UserParams.act.u4PolePairs + 1U)              /*!< Non Volatile Memory Actuator Pole-Pairs */
#define NV_MICRO_STEPS                      ((uint16_t)(1U << UserParams.act.u3MicroSteps))          /*!< Non Volatile Memory Micro-steps per full-step */
#define NV_ACT_SPEED1                       ((uint16_t)UserParams.act.u16Speed_1)                    /*!< Non Volatile Memory Speed-mode #1 speed [RPM] */
#define NV_ACT_SPEED2                       ((uint16_t)UserParams.act.u16Speed_2)                    /*!< Non Volatile Memory Speed-mode #2 speed [RPM] */
#define NV_ACT_SPEED3                       ((uint16_t)UserParams.act.u16Speed_3)                    /*!< Non Volatile Memory Speed-mode #3 speed [RPM] */
#if (LINPROT != LIN2X_AIRVENT12)
#define NV_ACT_SPEED4                       ((uint16_t)UserParams.act.u16Speed_4)                    /*!< Non Volatile Memory Speed-mode #4 speed [RPM] */
#else  /* (LINPROT != LIN2X_AIRVENT12) */
#define NV_ACT_TORQUE_SPEED                 ((uint16_t)UserParams.act.u16TorqueSpeed)                /*!< Non Volatile Memory Torque-mode speed [RPM] */
#endif /* (LINPROT != LIN2X_AIRVENT12) */
#define NV_MIN_SPEED                        ((uint16_t)UserParams.act.u13MinSpeed)                   /*!< Non Volatile Memory Minimum startup speed [RPM] */
#define NV_ACCELERATION_CONST               ((uint16_t)UserParams.act.u16AccelerationConst)          /*!< Non Volatile Memory Acceleration constant */
#define NV_ACCELERATION_PWR                 ((uint16_t)UserParams.act.u3AccelerationSteps)           /*!< Non Volatile Memory Acceleration 2^power */
#define NV_ACCELERATION_STEPS               ((uint16_t)(1U << UserParams.act.u3AccelerationSteps))   /*!< Non Volatile Memory Acceleration steps */
#define NV_DECELERATION_PWR                 ((uint16_t)UserParams.act.u3DecelerationSteps)           /*!< Non Volatile Memory Deceleration 2^power */
#define NV_DECELERATION_STEPS               ((uint16_t)(1U << UserParams.act.u3DecelerationSteps))   /*!< Non Volatile Memory Deceleration steps */
#define NV_MOTOR_CONSTANT                   ((uint16_t)UserParams.act.u8MotorConstant)               /*!< Non Volatile Memory Actuator BEMF constant */
#define NV_MOTOR_COIL_RTOT                  ((uint16_t)UserParams.act.u8MotorCoilRtot)               /*!< Non Volatile Memory Actuator coil resistance between phase-pins */
#if (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) || (_SUPPORT_APP_TYPE == C_APP_SOLENOID) || (_SUPPORT_APP_TYPE == C_APP_RELAY)
#if defined (__MLX81160__) || defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__)
#define NV_HOLDING_CURR_LEVEL               ((uint16_t)UserParams.act.u8HoldingTorqueCurrent)        /*!< Non Volatile Memory Actuator holding current [mA] */
#elif defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
#define NV_HOLDING_CURR_LEVEL               ((uint16_t)UserParams.act.u8HoldingTorqueCurrent << \
                                             UserParams.act.u2MotorCurrentMultiplier)                /*!< Non Volatile Memory Application Holding Current */
#endif
#define NV_MIN_HOLDCORR_RATIO               ((uint16_t)(p_MulU32_U16byU16(UserParams.act.u8PidStartupOrLowerHoldingLimit, \
                                                                          PWM_REG_PERIOD) >> (8U - C_PID_FACTOR)))  /*!< PID Minimum ratio (Holding mode) */
#define NV_STARTUP_CORR_RATIO               ((uint16_t)(p_MulU32_U16byU16(UserParams.act.u8PidStartupOrLowerHoldingLimit, \
                                                                          PWM_REG_PERIOD) >> (8U - C_PID_FACTOR)))  /*!< Non Volatile Memory Start-up Running PWM Duty-Cycle */
#if (_SUPPORT_FOC_MODE != FOC_MODE_NONE)
/* TODO[MMP]: This conflicts with holding current ! */
#define NV_RUNNING_CURR_MIN                 ((uint16_t)C_PID_RUNNING_CURR_MIN)                       /*!< Non Volatile Memory Application Minimum Running Current */
#if defined (__MLX81160__) || defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__)
#define NV_STARTUP_CURR_MAX                 ((uint16_t)(UserParams.act.u8PidStartupOrLowerHoldingLimit << \
                                                        UserParams.act.u2MotorCurrentMultiplier))    /*!< Non Volatile Memory Application Startup-Current (MMP190809-1) */
#elif defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
#define NV_STARTUP_CURR_MAX                 ((uint16_t)(UserParams.act.u8PidStartupOrLowerHoldingLimit << \
                                                        (UserParams.act.u2MotorCurrentMultiplier + C_MULTIPLIER_OFF)))  /*!< Non Volatile Memory Application Startup-Current (MMP190809-1) */
#endif
#endif /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) */
#else  /* (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) */
#if defined (__MLX81160__) || defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__)
#define NV_RUNNING_CURR_MIN                 ((uint16_t)UserParams.act.u8HoldingTorqueCurrent)        /*!< Non Volatile Memory Application Minimum Running Current */
#define NV_STARTUP_CURR_MAX                 ((uint16_t)(UserParams.act.u8PidStartupOrLowerHoldingLimit << \
                                                        UserParams.act.u2MotorCurrentMultiplier))    /*!< Non Volatile Memory Application Startup-Current (MMP190809-1) */
#elif defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
#define NV_RUNNING_CURR_MIN                 ((uint16_t)(UserParams.act.u8HoldingTorqueCurrent << \
                                                        (UserParams.act.u2MotorCurrentMultiplier + C_MULTIPLIER_OFF)))  /*!< Non Volatile Memory Application Minimum Running Current */
#define NV_STARTUP_CURR_MAX                 ((uint16_t)(UserParams.act.u8PidStartupOrLowerHoldingLimit << \
                                                        (UserParams.act.u2MotorCurrentMultiplier + C_MULTIPLIER_OFF)))  /*!< Non Volatile Memory Application Startup-Current (MMP190809-1) */
#endif
#endif /* (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) */
#if defined (__MLX81160__) || defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__)
#define NV_RUNNING_CURR_LEVEL               ((uint16_t)(UserParams.act.u8RunningTorqueCurrent << \
                                                        UserParams.act.u2MotorCurrentMultiplier))    /*!< Non Volatile Memory Actuator running current [mA] */
#define NV_RUNNING_CURR_MAX                 ((uint16_t)(UserParams.act.u8RunningTorqueCurrent << \
                                                        UserParams.act.u2MotorCurrentMultiplier))    /*!< Non Volatile Memory Application Running Current */
#elif defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
#define NV_RUNNING_CURR_LEVEL               ((uint16_t)(UserParams.act.u8RunningTorqueCurrent << \
                                                        (UserParams.act.u2MotorCurrentMultiplier + C_MULTIPLIER_OFF)))  /*!< Non Volatile Memory Actuator running current [mA] */
#define NV_RUNNING_CURR_MAX                 ((uint16_t)(UserParams.act.u8RunningTorqueCurrent << \
                                                        (UserParams.act.u2MotorCurrentMultiplier + C_MULTIPLIER_OFF)))  /*!< Non Volatile Memory Application Running Current */
#endif
#define NV_PID_RUNNINGCTRL_PER              ((uint16_t)UserParams.act.u7PidCtrlPeriod)               /*!< Non Volatile Memory PID Running Current/Speed Control period */
#define NV_PID_RUNNINGCTRL_PER_UNIT         ((uint16_t)UserParams.act.u1PidPeriodTimeOrSpeed)        /*!< Non Volatile Memory PID Running Current/Speed Control period unit */
#define NV_PID_COEF_P                       ((uint16_t)UserParams.act.u8PidCoefP)                    /*!< Non Volatile Memory PID current Control P-coefficient */
#define NV_PID_COEF_I                       ((uint16_t)UserParams.act.u8PidCoefI)                    /*!< Non Volatile Memory PID current Control I-coefficient */
#define NV_PID_COEF_D                       ((uint16_t)UserParams.act.u8PidCoefD)                    /*!< Non Volatile Memory PID current Control D-coefficient */
#define NV_MIN_CORR_RATIO                   ((uint16_t)(p_MulU32_U16byU16(UserParams.act.u8PidLowerLimit, \
                                                                          PWM_REG_PERIOD) >> (8U - C_PID_FACTOR)))  /*!< PID Minimum ratio (Running mode) */
//#if (PWM_LIMIT == PWM_LIMIT_BY_PID)
#define NV_MAX_CORR_RATIO_PID               ((uint16_t)(p_MulU32_U16byU16( (UserParams.act.u8PidUpperLimit + 1), \
                                                                           PWM_REG_PERIOD) >> (8U - C_PID_FACTOR)))  /*!< Non Volatile Memory Maximum Running PWM Duty-Cycle */
//#else  /* (PWM_LIMIT == PWM_LIMIT_BY_PID) */
#ifndef _SUPPORT_BOOST_MODE
#define NV_MAX_CORR_RATIO_PWM               ((uint16_t)(p_MulU32_U16byU16( (UserParams.act.u8PidUpperLimit + 1), \
                                                                           (uint16_t)(PWM_REG_PERIOD * 1.291)) >> \
                                                        ((8U - C_PID_FACTOR) - 1)))                  /*!< Non Volatile Memory Maximum Running PWM Duty-Cycle */
#elif (_SUPPORT_BOOST_MODE == BOOST_MODE_CLIPPING)
/* Factor 1.291: for stretch */
#define NV_MAX_CORR_RATIO_PWM               ((uint16_t)(p_MulU32_U16byU16( (UserParams.act.u8PidUpperLimit + 1), \
                                                                           (uint16_t)(PWM_REG_PERIOD * 1.291)) >> \
                                                        (8U - C_PID_FACTOR)))                        /*!< Non Volatile Memory Maximum Running PWM Duty-Cycle */
#elif (_SUPPORT_BOOST_MODE == BOOST_MODE_WAVEFORM)
/* Block waveform: factor: 25% */
#define NV_MAX_CORR_RATIO_PWM               ((uint16_t)(p_MulU32_U16byU16( (UserParams.act.u8PidUpperLimit + 1), \
                                                                           (uint16_t)(PWM_REG_PERIOD * 1.25)) >> \
                                                        (8U - C_PID_FACTOR)))                        /*!< Non Volatile Memory Maximum Running PWM Duty-Cycle */
#endif
//#endif /* (PWM_LIMIT == PWM_LIMIT_BY_PID) */
#if (LIN_COMM != FALSE)
#define NV_BUSTIMEOUT_SLEEP                 ((uint16_t)UserParams.stdlin[0].u1BusTimeOutSleep)       /*!< Non Volatile Memory Standard LIN: Bus-time to sleep support */
#define NV_LIN_UV                           ((uint16_t)UserParams.stdlin[0].u3LinUV)                 /*!< Non Volatile Memory Standard LIN: LIN UV level */
#endif /* (LIN_COMM != FALSE) */
#define NV_AUTO_RECALIBRATE                 ((uint16_t)UserParams.eol.u1PorCalibration)              /*!< Non Volatile Memory End-of-Line: Power-on auto-calibration */
#define NV_STALL_DETECTOR_DELAY             (((uint16_t)UserParams.stall.u8StallDetectorDelay) * \
                                             C_MICROSTEP_PER_FULLSTEP)                               /*!< Non Volatile Memory Application Stall detection delay */
#define NV_STALL_A                          ((uint16_t)UserParams.stall.u1StallA_Ena)                /*!< Non Volatile Memory Stall Detector: Stall "A" Enable */
#define NV_STALL_A_WIDTH                    ((uint16_t)UserParams.stall.u4StallA_Width)              /*!< Non Volatile Memory Stall Detector: Stall "A" Width */
#define NV_STALL_A_THRSHLD                  ((uint16_t)UserParams.stall.u7StallA_Threshold)          /*!< Non Volatile Memory Stall Detector: Stall "A" Threshold (MMP190412-1) */
#if (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
#define NV_STALL_P                          ((uint16_t)UserParams.stall.u1StallO_Ena)                /*!< Non Volatile Memory Stall Detector: Stall "P" Enable */
#define NV_STALL_P_POS_REVERSE              ((uint16_t)UserParams.stall.u4StallO_Width)              /*!< Non Volatile Memory Stall Detector: Stall "P" Threshold */
#define NV_STALL_P_THRSHLD                  ((uint16_t)(UserParams.stall.u7StallO_Threshold << 1))   /*!< Non Volatile Memory Stall Detector: Stall "P" Width */
#else  /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */
#define NV_STALL_O                          ((uint16_t)UserParams.stall.u1StallO_Ena)                /*!< Non Volatile Memory Stall Detector: Stall "O" Enable */
#define NV_STALL_O_THRSHLD                  ((uint16_t)UserParams.stall.u7StallO_Threshold)          /*!< Non Volatile Memory Stall Detector: Stall "O" Threshold */
#define NV_STALL_O_WIDTH                    ((uint16_t)UserParams.stall.u4StallO_Width)              /*!< Non Volatile Memory Stall Detector: Stall "O" Width */
#define NV_STALL_P                          ((uint16_t)UserParams.stall.u1StallO_Ena)                /*!< Non Volatile Memory Stall Detector: Stall "P" Enable */
#define NV_STALL_P_POS_REVERSE              ((uint16_t)UserParams.stall.u4StallO_Width)              /*!< Non Volatile Memory Stall Detector: Stall "P" Threshold */
#define NV_STALL_P_THRSHLD                  ((uint16_t)(UserParams.stall.u7StallO_Threshold << 3))   /*!< Non Volatile Memory Stall Detector: Stall "P" Width */
#endif /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */
#define NV_STALL_S                          ((uint16_t)UserParams.stall.u1StallS_Ena)                /*!< Non Volatile Memory Stall Detector: Stall "S" Enable */
#define NV_STALL_S_WIDTH                    ((uint16_t)UserParams.stall.u4StallS_Width)              /*!< Non Volatile Memory Stall Detector: Stall "S" Width */
#define NV_STALL_S_THRSHLD                  ((uint16_t)UserParams.stall.u7StallS_Threshold)          /*!< Non Volatile Memory Stall Detector: Stall "S" Threshold (MMP190412-1) */
#define NV_STALL_SPEED_DEPENDED             ((uint16_t)UserParams.stall.u1StallSpeedDepended)        /*!< Non Volatile Memory Stall Detector: Stall speed dependency */
#if (_SUPPORT_REWIND != FALSE)
#define NV_RESTALL_POR                      ((uint16_t)UserParams.stall.u1RestallPor)                /*!< POR re-stall setting */
#endif /* (_SUPPORT_REWIND != FALSE) */
#if (_SUPPORT_REWIND != FALSE) || (_SUPPORT_STALL_REVERSE != FALSE)
#define NV_REWIND_STEPS                     ((uint16_t)UserParams.stall.u8RewindSteps)               /*!< Re-wind (full) steps */
#endif /* (_SUPPORT_REWIND != FALSE) || (_SUPPORT_STALL_REVERSE != FALSE) */
#define NV_ROTATION_DIRECTION               ((uint16_t)UserParams.eol.u1MotorDirectionCCW)           /*!< Non Volatile Memory End-of-Line: Motor directional rotation: CW (0) or CCW (1) */
#define NV_REAL_TRAVEL                      ((uint16_t)UserParams.eol.u16RealTravel)                 /*!< Non Volatile Memory End-of-Line: Real Travel range */
#define NV_CALIB_TRAVEL                     ((uint16_t)UserParams.eol.u16RealTravel)                 /*!< Non Volatile Memory End-of-Line: Calibration Travel range */
#define NV_CALIB_ENDSTOP_TIME               ((uint16_t)UserParams.eol.u8EndStopTime)                 /*!< Non Volatile Memory End-of-Line: Calibration end-stop pause-period */
#define NV_CALIB_TOLERANCE_LOW              ((uint16_t)UserParams.eol.u8TravelToleranceLo)           /*!< Non Volatile Memory End-of-Line: Calibration Lower Limit tolerance */
#define NV_CALIB_TOLERANCE_HIGH             ((uint16_t)UserParams.eol.u8TravelToleranceUp)           /*!< Non Volatile Memory End-of-Line: Calibration Upper Limit tolerance */
#define NV_SAFETY_POSITION                  ((uint16_t)UserParams.eol.u7EmergencyRunPos)             /*!< Non Volatile Memory End-of-Line: Emergency/Safety position */
#if (_SUPPORT_TRIAXIS_MLX90363 != FALSE) || (_SUPPORT_TRIAXIS_MLX9038x != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90367 != FALSE) || (_SUPPORT_TRIAXIS_MLX90372 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90377 != FALSE) || (_SUPPORT_TRIAXIS_MLX90395 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90421 != FALSE) || (_SUPPORT_TRIAXIS_MLX90422 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90425 != FALSE) || (_SUPPORT_TRIAXIS_MLX90426 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90427 != FALSE) || \
    (_SUPPORT_DUAL_HALL_LATCH_MLX92251 != FALSE) || (_SUPPORT_DUAL_HALL_LATCH_MLX92255 != FALSE) || \
    (_SUPPORT_FOC_MODE != FOC_MODE_NONE)
#define NV_CDI_ENA                          ((uint16_t)UserParams.sensor.u1CDI)                      /*!< CDI On/Off */
#endif
#if (_SUPPORT_TRIAXIS_MLX90363 != FALSE) || (_SUPPORT_TRIAXIS_MLX9038x != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90367 != FALSE) || (_SUPPORT_TRIAXIS_MLX90372 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90377 != FALSE) || (_SUPPORT_TRIAXIS_MLX90395 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90421 != FALSE) || (_SUPPORT_TRIAXIS_MLX90422 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90425 != FALSE) || (_SUPPORT_TRIAXIS_MLX90426 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90427 != FALSE) || \
    (_SUPPORT_DUAL_HALL_LATCH_MLX92251 != FALSE) || (_SUPPORT_DUAL_HALL_LATCH_MLX92255 != FALSE)
#define NV_TRIAXIS_X_OFF                    ((uint16_t)UserParams.sensor.u8TriaxisOffsetX)           /*!< Non Volatile Memory Sensor: Triaxis Offset X */
#define NV_TRIAXIS_Y_OFF                    ((uint16_t)UserParams.sensor.u8TriaxisOffsetY)           /*!< Non Volatile Memory Sensor: Triaxis Offset Y */
#define NV_SENSE_POLE_PAIRS                 ((uint16_t)UserParams.sensor.u4TriaxisMagnetPolePairs)   /*!< Non Volatile Memory Sensor: Sense Magnet Pole-pairs */
#define NV_TRIAXIS_AMPL_CORR                ((uint16_t)UserParams.sensor.u11TriaxisAmplitudeCorrection)   /*!< Non Volatile Memory Sensor: Triaxis X/Y-amplitude correction */
#define NV_TRIAXIS_DIRECTION                ((uint16_t)UserParams.sensor.u1TriaxisDirection)         /*!< Non Volatile Memory Sensor: Triaxis Direction (0 = same as Act; 1 = Reverse) */
#define NV_TRIAXIS_ANGLE_OFF                ((uint16_t)UserParams.sensor.u16TriaxisAngleOffset)      /*!< Non Volatile Memory Sensor: Triaxis Angle Offset */
#elif (_SUPPORT_INDUCTIVE_POS_SENSOR_MLX90513 != FALSE)
#define NV_INDUCTIVE_POS_SENSOR_DIRECTION   ((uint16_t)UserParams.sensor.u1TriaxisDirection)         /*!< Non Volatile Memory Sensor: Inductive Position Sensor Direction (0 = same as Act; 1 = Reverse) */
#define NV_INDUCTIVE_POS_SENSOR_ANGLE_OFF   ((uint16_t)UserParams.sensor.u16TriaxisAngleOffset)      /*!< Non Volatile Memory Sensor: Inductive Position Sensor Angle Offset */
#endif
#if (_SUPPORT_FOC_MODE != FOC_MODE_NONE)
#if (_SUPPORT_TRIAXIS_MLX90363 != FALSE) || (_SUPPORT_TRIAXIS_MLX9038x != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90367 != FALSE) || (_SUPPORT_TRIAXIS_MLX90372 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90377 != FALSE) || (_SUPPORT_TRIAXIS_MLX90395 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90421 != FALSE) || (_SUPPORT_TRIAXIS_MLX90422 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90425 != FALSE) || (_SUPPORT_TRIAXIS_MLX90426 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90427 != FALSE) || \
    (_SUPPORT_DUAL_HALL_LATCH_MLX92251 != FALSE) || (_SUPPORT_DUAL_HALL_LATCH_MLX92255 != FALSE) || \
    (_SUPPORT_INDUCTIVE_POS_SENSOR_MLX90513 != FALSE)
#define NV_TARGET_LA                        ((uint16_t)(32U * C_TARGET_LA))                          /*!< Non Volatile Memory FOC: target Load-Angle (at max speed4) */
#define NV_PID_LA_COEF_P                    ((uint16_t)C_PID_LA_COEF_P)                              /*!< Non Volatile Memory FOC: PID P-Coefficient for Load Angle */
#define NV_PID_LA_COEF_I                    ((uint16_t)C_PID_LA_COEF_I)                              /*!< Non Volatile Memory FOC: PID I-Coefficient for Load Angle */
#define NV_PID_LA_COEF_D                    ((uint16_t)C_PID_LA_COEF_D)                              /*!< Non Volatile Memory FOC: PID D-Coefficient for Load Angle */
#define NV_STALL_LA_THRSHLD                 ((uint16_t)C_STALL_LA_THRESHOLD_FOC)                     /*!< Non Volatile Memory FOC: Load angle Stall detection threshold */
#define NV_STALL_LA_WIDTH                   ((uint16_t)C_STALL_LA_WIDTH_FOC)                         /*!< Non Volatile Memory Stall Detector: Stall "LA" Width (MMP220720-1) */
#define NV_STALL_LA_THRSHLD_OL              ((uint16_t)C_STALL_LA_THRESHOLD_OL)                      /*!< Non Volatile Memory FOC: Load angle Stall detection threshold (MMP230824-1) */
#define NV_STALL_LA_WIDTH_OL                ((uint16_t)C_STALL_LA_WIDTH_OL)                          /*!< Non Volatile Memory Stall Detector: Stall "LA" Width (MMP230824-1) */
#define NV_LA_OPEN_TO_CLOSE_AMPL            ((uint16_t)C_LA_OPEN_TO_CLOSE_AMPL)                      /*!< Non Volatile Memory FOC: Open-to-Close amplitude factor */
#else
#define NV_TARGET_LA                        ((uint16_t)(32U * UserParams.sensor.u8TargetLA))         /*!< Non Volatile Memory FOC: target Load-Angle (at max speed4) */
#define NV_PID_LA_COEF_P                    ((uint16_t)UserParams.sensor.u8PidCoefP_LA)              /*!< Non Volatile Memory FOC: PID P-Coefficient for Load Angle */
#define NV_PID_LA_COEF_I                    ((uint16_t)UserParams.sensor.u8PidCoefI_LA)              /*!< Non Volatile Memory FOC: PID I-Coefficient for Load Angle */
#define NV_PID_LA_COEF_D                    ((uint16_t)UserParams.sensor.u8PidCoefD_LA)              /*!< Non Volatile Memory FOC: PID D-Coefficient for Load Angle */
#define NV_STALL_LA_THRSHLD                 ((uint16_t)UserParams.sensor.u4StallLA_Threshold)        /*!< Non Volatile Memory Stall Detector: Stall "LA" angle-threshold (MMP220720-1) */
#define NV_STALL_LA_WIDTH                   ((uint16_t)UserParams.sensor.u4StallLA_Width)            /*!< Non Volatile Memory Stall Detector: Stall "LA" Width (MMP220720-1) */
#define NV_STALL_FLUX_THRSHLD               ((uint16_t)UserParams.sensor.u4StallLA_Threshold)        /*!< Non Volatile Memory Stall Detector: Stall "Flux" angle-threshold (MMP220725-1) */
#define NV_STALL_FLUX_WIDTH                 ((uint16_t)UserParams.sensor.u4StallLA_Width)            /*!< Non Volatile Memory Stall Detector: Stall "Flux" Width (MMP220725-1) */
#define NV_LA_OPEN_TO_CLOSE_AMPL            ((uint16_t)(UserParams.sensor.u3OpenToCloseAmplitude + 1U))   /*!< Non Volatile Memory FOC: Open-to-Close amplitude factor */
#define NV_STALL_LA_THRSHLD_OL              ((uint16_t)C_STALL_LA_THRESHOLD_OL)                      /*!< Non Volatile Memory FOC: Load angle Stall detection threshold (MMP230824-1) */
#define NV_STALL_LA_WIDTH_OL                ((uint16_t)C_STALL_LA_WIDTH_OL)                          /*!< Non Volatile Memory Stall Detector: Stall "LA" Width (MMP230824-1) */
#endif
#elif (_SUPPORT_STALLDET_LA != FALSE)
/*#define NV_STALL_LA_THRSHLD_SPEED1        ((uint16_t)C_STALL_LA_THRESHOLD_SPEED_0)*/               /*!< Non Volatile Memory FOC: Load angle Stall detection threshold (MMP230824-1) */
/*#define NV_STALL_LA_THRSHLD_SPEED2        ((uint16_t)C_STALL_LA_THRESHOLD_SPEED_1)*/               /*!< Non Volatile Memory FOC: Load angle Stall detection threshold (MMP230824-1) */
/*#define NV_STALL_LA_THRSHLD_SPEED3        ((uint16_t)C_STALL_LA_THRESHOLD_SPEED_2)*/               /*!< Non Volatile Memory FOC: Load angle Stall detection threshold (MMP230824-1) */
/*#define NV_STALL_LA_THRSHLD_SPEED4        ((uint16_t)C_STALL_LA_THRESHOLD_SPEED_3)*/               /*!< Non Volatile Memory FOC: Load angle Stall detection threshold (MMP230824-1) */
#define NV_STALL_LA_THRSHLD                 ((uint16_t)C_STALL_LA_THRESHOLD_FOC)                     /*!< Non Volatile Memory FOC: Load angle Stall detection threshold (MMP230824-1) */
#define NV_STALL_LA_WIDTH                   ((uint16_t)C_STALL_LA_WIDTH_FOC)                         /*!< Non Volatile Memory Stall Detector: Stall "LA" Width (MMP230824-1) */
#define NV_STALL_LA_THRSHLD_OL              ((uint16_t)C_STALL_LA_THRESHOLD_OL)                      /*!< Non Volatile Memory FOC: Load angle Stall detection threshold (MMP230824-1) */
#define NV_STALL_LA_WIDTH_OL                ((uint16_t)C_STALL_LA_WIDTH_OL)                          /*!< Non Volatile Memory Stall Detector: Stall "LA" Width (MMP230824-1) */
#endif /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) */

#if (_SUPPORT_APP_SAVE != FALSE)
#define NV_APP_DATA_MSW                     ((uint16_t)UserParams.app.u16ParamMSW)                   /*!< Non Volatile Memory Application LSW parameter */
#define NV_APP_DATA_LSW                     ((uint16_t)UserParams.app.u16ParamLSW)                   /*!< Non Volatile Memory Application MSW parameter */
#endif /* (_SUPPORT_APP_SAVE != FALSE) */
#else  /* (_SUPPORT_NV_MOTOR_PARAMS != FALSE) */

/* **************************************************************************** *
 * Fixed constants (iso NVRAM)
 * **************************************************************************** */
#if (_SUPPORT_BEMF_LEADANGLE_COMPENSATE != FALSE)
#define NV_LEADANGLE_COMPENSATION           C_LEADANGLE_COMPENSATION                                 /*!< Lead-angle compensation (n * 1.875 degrees, with n=0..15) */
#endif /* (_SUPPORT_BEMF_LEADANGLE_COMPENSATE != FALSE) */
#define NV_VSUP_REF                         ((uint16_t)(C_VSUP_REF * 25U) >> 1)                      /*!< CONSTANT Reference supply [1/8V] to [10mV] */
#define NV_IC_UV_LEVEL                      ((uint16_t)C_IC_UV_LEVEL)                                /*!< CONSTANT IC UV level */
#define NV_IC_OV_LEVEL                      ((uint16_t)C_IC_OV_LEVEL)                                /*!< CONSTANT IC OV level */
#define NV_GEARBOX_RATIO                    ((uint16_t)C_MOTOR_GEAR_BOX_RATIO)                       /*!< CONSTANT Gear-box ratio */
#define NV_POLE_PAIRS                       ((uint16_t)C_MOTOR_POLE_PAIRS)                           /*!< CONSTANT Actuator Pole-Pairs */
#define NV_MICRO_STEPS                      ((uint16_t)(1U << C_MOTOR_MICROSTEPS))                   /*!< CONSTANT Micro-steps per full-step */
#define NV_STALL_SPEED_DEPENDED             ((uint16_t)C_STALL_SPEED_DEPENDED)                       /*!< CONSTANT Stall Detector: Stall speed dependency */
#define NV_ACT_SPEED1                       ((uint16_t)C_SPEED_0)                                    /*!< CONSTANT Speed-mode #1 speed [RPM] */
#define NV_ACT_SPEED2                       ((uint16_t)C_SPEED_1)                                    /*!< CONSTANT Speed-mode #2 speed [RPM] */
#if (LINPROT != LIN2X_AIRVENT12)
#define NV_ACT_SPEED3                       ((uint16_t)C_SPEED_2)                                    /*!< CONSTANT Speed-mode #3 speed [RPM] */
#define NV_ACT_SPEED4                       ((uint16_t)C_SPEED_3)                                    /*!< CONSTANT Speed-mode #4 speed [RPM] */
#elif (LINPROT == LIN22_SIMPLE_PCT) || (LINPROT == NOLIN)
#define NV_ACT_SPEED3                       ((uint16_t)C_SPEED_3)                                    /*!< CONSTANT Speed-mode #3 speed [RPM] */
#endif
#define NV_MIN_SPEED                        ((uint16_t)C_SPEED_MIN)                                  /*!< CONSTANT Minimum startup speed [RPM] */
#define NV_ACCELERATION_CONST               ((uint16_t)C_ACCELERATION_CONST)                         /*!< CONSTANT Acceleration constant */
#define NV_ACCELERATION_STEPS               ((uint16_t)(C_ACCELERATION_STEPS - 1U))                  /*!< CONSTANT Acceleration steps */
#define NV_DECELERATION_STEPS               ((uint16_t)(C_DECELERATION_STEPS - 1U))                  /*!< CONSTANT Deceleration steps */
#define NV_MOTOR_CONSTANT                   ((uint16_t)C_MOTOR_CONST_10MV_PER_RPS)                   /*!< CONSTANT Actuator BEMF constant */
#define NV_MOTOR_COIL_RTOT                  ((uint16_t)C_TOT_COILS_R)                                /*!< CONSTANT Actuator coil resistance between phase-pins */
#define NV_MOTOR_COIL_LTOT                  ((uint16_t)C_TOT_COILS_L)                                /*!< CONSTANT Application Coil Inductance */
#define NV_RUNNING_CURR_MIN                 ((uint16_t)C_PID_RUNNING_CURR_MIN)                       /*!< CONSTANT Application Minimum Running Current */
#define NV_RUNNING_CURR_MAX                 ((uint16_t)C_PID_RUNNING_CURR_MAX)                       /*!< CONSTANT Application Running Current */
#define NV_HOLDING_CURR_LEVEL               ((uint16_t)C_PID_HOLDING_CURR_LEVEL)                     /*!< CONSTANT Actuator holding current [mA] */
#define NV_RUNNING_CURR_LEVEL               ((uint16_t)C_PID_RUNNING_CURR_LEVEL)                     /*!< CONSTANT Actuator running current [mA] */
#define NV_STARTUP_CURR_MAX                 ((uint16_t)C_PID_STARTING_CURR_MAX)                      /*!< CONSTANT Application Startup-Current (MMP190809-1) */
#define NV_APPL_OTEMP                       ( (int16_t)C_APP_OT_LEVEL)                               /*!< CONSTANT Application Over-temperature [C] */
#define NV_APPL_UVOLT                       ((uint16_t)(C_APP_UV_LEVEL * 25U) >> 1)                  /*!< CONSTANT Application Under-voltage [1/8V] to [10mV] */
#define NV_APPL_OVOLT                       ((uint16_t)(C_APP_OV_LEVEL * 25U) >> 1)                  /*!< CONSTANT Application Over-voltage [1/8V] to [10mV] */
#define NV_PID_RUNNINGCTRL_PER_UNIT         ((uint16_t)C_PID_RUNNINGCTRL_PERIOD_UNIT)                /*!< CONSTANT PID Running Current/Speed Control period unit */
#define NV_PID_RUNNINGCTRL_PER              ((uint16_t)C_PID_RUNNINGCTRL_PERIOD)                     /*!< CONSTANT PID Running Current/Speed Control period */
#define NV_PID_HOLDINGCTRL_PER              ((uint16_t)(C_PID_HOLDINGCTRL_PERIOD << 2))              /*!< CONSTANT PID Holding Control period */
#define NV_PID_COEF_P                       ((uint16_t)C_PID_COEF_P)                                 /*!< CONSTANT PID current Control P-coefficient */
#define NV_PID_COEF_I                       ((uint16_t)C_PID_COEF_I)                                 /*!< CONSTANT PID current Control I-coefficient */
#define NV_PID_COEF_D                       ((uint16_t)C_PID_COEF_D)                                 /*!< CONSTANT PID current Control D-coefficient */
#define NV_PID_THRSHLDCTRL_PER              ((uint16_t)(C_PID_THRSHLDCTRL_PERIOD << 7))              /*!< CONSTANT PID Current threshold compensation period */
#define NV_MIN_HOLDCORR_RATIO               ((uint16_t)((((uint32_t)PWM_REG_PERIOD) * C_MIN_HOLDCORR_RATIO) >> \
                                                        (8U - C_PID_FACTOR)))                        /*!< CONSTANT PID Minimum ratio (Holding mode) */
#define NV_MIN_CORR_RATIO                   ((uint16_t)((((uint32_t)PWM_REG_PERIOD) * C_MIN_CORR_RATIO) >> \
                                                        (8U - C_PID_FACTOR)))                        /*!< CONSTANT PID Minimum ratio (Running mode) */
#define NV_MAX_CORR_RATIO                   ((uint16_t)((((uint32_t)PWM_REG_PERIOD) * (C_MAX_CORR_RATIO + 1)) >> \
                                                        (8U - C_PID_FACTOR)))                        /*!< CONSTANT Maximum Running PWM Duty-Cycle */
#define NV_STARTUP_CORR_RATIO               ((uint16_t)((((uint32_t)PWM_REG_PERIOD) * C_STARTUP_CORR_RATIO) >> \
                                                        (8U - C_PID_FACTOR)))                        /*!< CONSTANT Start-up Running PWM Duty-Cycle */
#define NV_STALL_DETECTOR_DELAY             ((uint16_t)C_DETECTOR_DELAY)                             /*!< CONSTANT Application Stall detection delay */
#define NV_LIN_UV                           ((uint16_t)C_LIN_UV)                                     /*!< CONSTANT Standard LIN: LIN UV level */
#define NV_AUTO_RECALIBRATE                 ((uint16_t)C_AUTO_RECALIBRATE)                           /*!< CONSTANT End-of-Line: Power-on auto-calibration */
#define NV_BUSTIMEOUT_SLEEP                 ((uint16_t)C_BUSTIMEOUT_SLEEP)                           /*!< CONSTANT Standard LIN: Bus-time to sleep support */
#define NV_STALL_A                          ((uint16_t)C_STALL_A_DET)                                /*!< CONSTANT Stall Detector: Stall "A" Enable */
#define NV_STALL_A_WIDTH                    ((uint16_t)C_STALL_A_WIDTH)                              /*!< CONSTANT Stall Detector: Stall "A" Width */
#define NV_STALL_A_THRSHLD                  ((uint16_t)C_STALL_A_THRESHOLD)                          /*!< CONSTANT Stall Detector: Stall "A" Threshold (MMP190412-1) */
#define NV_STALL_O                          ((uint16_t)C_STALL_O_DET)                                /*!< CONSTANT Stall Detector: Stall "O" Enable */
#define NV_STALL_O_THRSHLD                  ((uint16_t)C_STALL_O_THRESHOLD)                          /*!< CONSTANT Stall Detector: Stall "O" Threshold */
#define NV_STALL_O_WIDTH                    ((uint16_t)C_STALL_O_WIDTH)                              /*!< CONSTANT Stall Detector: Stall "O" Width */
#define NV_STALL_P                          ((uint16_t)C_STALL_O_DET)                                /*!< CONSTANT Stall Detector: Stall "P" Enable */
#define NV_STALL_P_POS_REVERSE              ((uint16_t)C_STALL_O_WIDTH)                              /*!< CONSTANT Stall Detector: Stall "P" Threshold */
#define NV_STALL_P_THRSHLD                  ((uint16_t)(C_STALL_O_THRESHOLD << 1))                   /*!< CONSTANT Stall Detector: Stall "P" Width */
#define NV_STALL_S                          ((uint16_t)C_STALL_S_DET)                                /*!< CONSTANT Stall Detector: Stall "S" Enable */
#define NV_STALL_S_WIDTH                    ((uint16_t)C_STALL_S_WIDTH)                              /*!< CONSTANT Stall Detector: Stall "S" Width */
#define NV_STALL_S_THRSHLD                  ((uint16_t)C_STALL_S_THRESHOLD)                          /*!< CONSTANT Stall Detector: Stall "S" Threshold (MMP190412-1) */
#define NV_STALL_FLUX_THRSHLD               ((uint16_t)C_STALL_LA_THRESHOLD)                         /*!< CONSTANT Stall Detector: Stall "Flux" angle-threshold (MMP220725-1) */
#define NV_STALL_FLUX_WIDTH                 ((uint16_t)C_STALL_LA_WIDTH)                             /*!< CONSTANT Stall Detector: Stall "Flux" Width (MMP220725-1) */
#define NV_STALL_LA_THRSHLD                 ((uint16_t)C_STALL_LA_THRESHOLD)                         /*!< CONSTANT Stall Detector: Stall "LA" angle-threshold (MMP220720-1) */
#define NV_STALL_LA_WIDTH                   ((uint16_t)C_STALL_LA_WIDTH)                             /*!< CONSTANT Stall Detector: Stall "LA" Width (MMP220720-1) */
#define NV_ROTATION_DIRECTION               FALSE                                                    /*!< CONSTANT End-of-Line: Motor directional rotation: CW (0) or CCW (1) */
#define NV_REAL_TRAVEL                      0U                                                       /*!< CONSTANT End-of-Line: Real Travel range */
#define NV_CALIB_TRAVEL                     C_DEFAULT_TRAVEL                                         /*!< CONSTANT End-of-Line: Calibration Travel range */
#define NV_CALIB_ENDSTOP_TIME               C_ENDSTOP_TIME                                           /*!< CONSTANT End-of-Line: Calibration end-stop pause-period */
#define NV_CALIB_TOLERANCE_LOW              C_TRAVEL_TOLERANCE_LO                                    /*!< CONSTANT End-of-Line: Calibration Lower Limit tolerance */
#define NV_CALIB_TOLERANCE_HIGH             C_TRAVEL_TOLERANCE_UP                                    /*!< CONSTANT End-of-Line: Calibration Upper Limit tolerance */
#define NV_SAFETY_POSITION                  127U                                                     /*!< CONSTANT End-of-Line: Emergency/Safety position */
#define NV_SENSE_POLE_PAIRS                 ((uint16_t)C_SENSE_MAGNET_POLE_PAIRS)                    /*!< CONSTANT Sensor: Sense Magnet Pole-pairs */
#if (_SUPPORT_FOC_MODE != FOC_MODE_NONE)
#define NV_TARGET_LA                        ((uint16_t)C_TARGET_LA)
#define NV_PID_LA_COEF_P                    ((uint16_t)C_PID_LA_COEF_P)
#define NV_PID_LA_COEF_I                    ((uint16_t)C_PID_LA_COEF_I)
#define NV_PID_LA_COEF_D                    ((uint16_t)C_PID_LA_COEF_D)
#define NV_STALL_LA_THRSHLD                 ((uint16_t)C_STALL_LA_THRESHOLD)
#define NV_LA_OPEN_TO_CLOSE_AMPL            ((uint16_t)C_LA_OPEN_TO_CLOSE_AMPL)
#endif /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) */
#define NV_ALIGNMENT_SPEED                  ((uint16_t)C_SPEED_ALIGNMENT)                            /*!< CONSTANT Application: Alignment Speed */
#define NV_TACHO_MODE                       ((uint16_t)C_TACHO_MODE)                                 /*!< CONSTANT Application: Tacho-mode */
#define NV_PID_RAMP_UP                      ((uint16_t)C_PID_RAMP_UP)                                /*!< CONSTANT Ramp-up speed limit */
#define NV_PID_RAMP_DOWN                    ((uint16_t)C_PID_RAMP_DOWN)                              /*!< CONSTANT Ramp-down speed limit */
#define NV_STARTUP_CURR_MAX                 ((uint16_t)C_PID_STARTING_CURR_MAX)                      /*!< CONSTANT Application Startup-Current (MMP190809-1) */
#if ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_ID_IQ)
#define NV_PID_ID_COEF_P                    ((uint16_t)C_PID_ID_COEF_P)                              /*!< CONSTANT PID Coefficient P for Id-path */
#define NV_PID_ID_COEF_I                    ((uint16_t)C_PID_ID_COEF_I)                              /*!< CONSTANT PID Coefficient I for Id-path */
#define NV_PID_ID_COEF_D                    ((uint16_t)C_PID_ID_COEF_D)                              /*!< CONSTANT PID Coefficient D for Id-path */
#define NV_PID_IQ_COEF_P                    ((uint16_t)C_PID_IQ_COEF_P)                              /*!< CONSTANT PID Coefficient P for Iq-path */
#define NV_PID_IQ_COEF_I                    ((uint16_t)C_PID_IQ_COEF_I)                              /*!< CONSTANT PID Coefficient I for Iq-path */
#define NV_PID_IQ_COEF_D                    ((uint16_t)C_PID_IQ_COEF_D)                              /*!< CONSTANT PID Coefficient D for Iq-path */
#endif /* ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_ID_IQ) */
#endif /* (_SUPPORT_NV_MOTOR_PARAMS != FALSE) */

/*!*************************************************************************** */
/*                           GLOBAL VARIABLES                                  */
/* *************************************************************************** */
#pragma space nodp
#pragma space none

/*!*************************************************************************** */
/*                           GLOBAL FUNCTIONS                                  */
/* *************************************************************************** */

#endif /* DRIVE_LIB_NV_USER_PAGE_H */

/* EOF */
