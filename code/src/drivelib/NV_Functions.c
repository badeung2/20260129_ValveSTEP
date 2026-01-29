/*!*************************************************************************** *
 * \file        NV_Functions.c
 * \brief       MLX8133x/4x Non Volatile Memory handling
 *
 * \note        project MLX8133x/4x
 *
 * \author      Marcel Braat
 *
 * \date        2017-11-29
 *
 * \version     2.0
 *
 * A list of functions:
 *  - Global Functions:
 *           -# NV_CalcCRC()
 *           -# NV_iCalcCRC()
 *           -# p_CopyU8()
 *           -# p_CopyU16()
 *           -# p_NV_WriteWord64_blocking()
 *           -# NV_WriteBlock()
 *           -# NV_CheckCRC()
 *           -# NV_WriteLIN_STD()
 *           -# NV_WriteLIN_ENH()
 *           -# NV_WriteUDS()
 *           -# NV_WriteEOL()
 *           -# NV_WriteAPP()
 *           -# NV_WriteActParams()
 *           -# NV_WriteSensor()
 *           -# NV_WriteActStall()
 *           -# NV_WritePatch()
 *           -# NV_WriteErrorLog()
 *           -# NV_WriteI2cParams()
 *           -# NV_WriteCanParams()
 *           -# NV_WriteUserDefaults()
 *           -# NV_MlxCalib()
 *           -# NV_AppStore()
 *  - Internal Functions:
 *           -# p_CopyU64()
 *           -# p_CompareU64()
 *           -# p_NV_BusyChecks();
 *           -# NV_BlockCheckCRC()
 *           -# NV_CheckStdLin()
 *           -# NV_CheckEnhLin
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

/*!*************************************************************************** *
 *                              I N C L U D E S                                *
 * *************************************************************************** */
#include "AppBuild.h"                                                           /* Application Build */

#include "../AppVersion.h"                                                      /* Application Version support */
#include "../ActADC.h"

/* Driver */
#include "drivelib/ADC.h"                                                       /* ADC support */
#include "drivelib/ErrorCodes.h"                                                /* Error logging support */
#include "drivelib/NV_Functions.h"                                              /* Non Volatile Memory Functions & Layout */
#include "drivelib/NV_UserPage.h"                                               /* Non Volatile Memory User Application Page Layout */
#if (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT)
#include "drivelib/MotorDriverTables.h"                                         /* Wave-form vector tables */
#endif /* (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT) */
#include "camculib/private_mathlib.h"                                           /* Private Math Library support */

/* Communication */
#if (CAN_COMM != FALSE)
#include "commlib/CAN_Communication.h"                                          /* CAN Communication support */
#endif /* (CAN_COMM != FALSE) */
#if (I2C_COMM != FALSE) && (_SUPPORT_I2C != FALSE) && ((I2C_MASTER_PROT != I2C_MASTER_PROT_NONE) || (I2C_SLAVE_PROT != I2C_SLAVE_PROT_NONE))
#include "commlib/I2C_Generic.h"                                                /* I2C Generic support */
#endif /* (I2C_COMM != FALSE) && (_SUPPORT_I2C != FALSE) && ((I2C_MASTER_PROT != I2C_MASTER_PROT_NONE) || (I2C_SLAVE_PROT != I2C_SLAVE_PROT_NONE)) */
#if (LIN_COMM != FALSE)
#include "commlib/LIN_Communication.h"                                          /* LIN Communication support */
#include "commlib/LIN_Diagnostics.h"                                            /* LIN Diagnostics support */
#endif /* (LIN_COMM != FALSE) */

/* Platform */
#include <bl_bist.h>
#include <builtin_mlx16.h>
#if (_SUPPORT_NV_TYPE == C_NV_EEPROM)
#include <eeprom_drv.h>                                                         /* Use Melexis EEPROM library functions */
#elif (_SUPPORT_NV_TYPE == C_NV_FLASH)
#include <flash_kf_drv.h>                                                       /* Use Melexis FLASH driver library functions */
#endif /* (_SUPPORT_NV_TYPE == C_NV_EEPROM) */
#include <mem_checks.h>                                                         /* Use Melexis Memory check functions */
#include <plib.h>                                                               /* Product libraries */
#include <atomic.h>

/*!*************************************************************************** */
/*                           DEFINITIONS                                       */
/* *************************************************************************** */
#define C_MAX_TEMP_WRITE_CNT_INCR_1     60                                      /*!< Maximum Junction-temperature for write-count increase by 1 */
#define C_MAX_TEMP_WRITE_CNT_INCR_2     92                                      /*!< Maximum Junction-temperature for write-count increase by 2 */
#define C_MAX_TEMP_WRITE_CNT_INCR_3     111                                     /*!< Maximum Junction-temperature for write-count increase by 3 */
#define C_MAX_TEMP_WRITE_CNT_INCR_4     123                                     /*!< Maximum Junction-temperature for write-count increase by 4 */
#define C_MAX_TEMP_WRITE_CNT_INCR_5     131                                     /*!< Maximum Junction-temperature for write-count increase by 5 */
#define C_MAX_TEMP_WRITE_CNT_INCR_6     138                                     /*!< Maximum Junction-temperature for write-count increase by 6 */
#define C_MAX_TEMP_WRITE_CNT_INCR_7     142                                     /*!< Maximum Junction-temperature for write-count increase by 7 */
#define C_MAX_TEMP_WRITE_CNT_INCR_8     146                                     /*!< Maximum Junction-temperature for write-count increase by 8 */
#define C_MAX_TEMP_WRITE_CNT_INCR_9     149                                     /*!< Maximum Junction-temperature for write-count increase by 9 */
/*! Temperature Scale for Non Volatile Memory Write count increase */
const int16_t ai16TemperatureScale[] =
{
    C_MAX_TEMP_WRITE_CNT_INCR_1,
    C_MAX_TEMP_WRITE_CNT_INCR_2,
    C_MAX_TEMP_WRITE_CNT_INCR_3,
    C_MAX_TEMP_WRITE_CNT_INCR_4,
    C_MAX_TEMP_WRITE_CNT_INCR_5,
    C_MAX_TEMP_WRITE_CNT_INCR_6,
    C_MAX_TEMP_WRITE_CNT_INCR_7,
    C_MAX_TEMP_WRITE_CNT_INCR_8,
    C_MAX_TEMP_WRITE_CNT_INCR_9
};

#define C_NV_REV            0x02U                                               /*!< Non Volatile Memory revision */

/*! Header structure ID */
 #define C_HDR_STRUCT_ID     (C_HEADER_PARAMS \
                              | C_STD_LIN_PARAMS \
                              | C_ENH_LIN_PARAMS \
                              | C_UDS_LIN_PARAMS \
                              | C_APP_EOL \
                              | C_APP_STORE \
                              | C_ACT_PARAMS \
                              | C_SENSOR_PARAMS \
                              | C_ACT_STALL \
                              | C_I2C_PARAMS \
                              | C_CAN_PARAMS \
                              )                                                 /*!< Header Structure ID */

/*! Non Volatile Memory Structure blocks address, size and error bit-mask */
NV_CRC eeCrc[] =
{
#if (LIN_COMM != FALSE)
    { ADDR_NV_STD_LIN_1, (sizeof(STD_LIN_PARAMS_t) / sizeof(uint16_t)), C_ERR_NV_LIN_STD_1},
    { ADDR_NV_STD_LIN_2, (sizeof(STD_LIN_PARAMS_t) / sizeof(uint16_t)), C_ERR_NV_LIN_STD_2},
#if (LINPROT == LIN2X_HVAC52)
    { ADDR_NV_ENH_LIN_1, (sizeof(ENH_LIN_PARAMS_t) / sizeof(uint16_t)), C_ERR_NV_LIN_ENH_1},
    { ADDR_NV_ENH_LIN_2, (sizeof(ENH_LIN_PARAMS_t) / sizeof(uint16_t)), C_ERR_NV_LIN_ENH_2},
#endif /* (LINPROT == LIN2X_HVAC52) */
#if ((_SUPPORT_UDS != FALSE) != FALSE)
    { ADDR_NV_UDS, (sizeof(UDS_LIN_PARAMS_t) / sizeof(uint16_t)), C_ERR_NV_LIN_UDS},
#endif /* ((_SUPPORT_UDS != FALSE) != FALSE) */
#endif /* (LIN_COMM != FALSE) */
    { ADDR_NV_EOL, (sizeof(APP_EOL_t) / sizeof(uint16_t)), C_ERR_NV_EOL},
#if (_SUPPORT_APP_SAVE != FALSE)
    { ADDR_NV_APP_PARAMS, (sizeof(APP_PARAMS_t) / sizeof(uint16_t)), C_ERR_NV_APP_PARAMS},
#endif /* (_SUPPORT_APP_SAVE != FALSE) */
    { ADDR_NV_ACT_PARAMS, (sizeof(ACT_PARAMS_t) / sizeof(uint16_t)),C_ERR_NV_ACT_PARAMS},
#if (_SUPPORT_TRIAXIS_MLX90363 != FALSE) || (_SUPPORT_TRIAXIS_MLX9038x != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90367 != FALSE) || (_SUPPORT_TRIAXIS_MLX90372 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90377 != FALSE) || (_SUPPORT_TRIAXIS_MLX90395 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90421 != FALSE) || (_SUPPORT_TRIAXIS_MLX90422 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90425 != FALSE) || (_SUPPORT_TRIAXIS_MLX90426 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90427 != FALSE) || \
    (_SUPPORT_INDUCTIVE_POS_SENSOR_MLX90513 != FALSE) || \
    (_SUPPORT_DUAL_HALL_LATCH_MLX92251 != FALSE) || (_SUPPORT_DUAL_HALL_LATCH_MLX92255 != FALSE) || \
    (_SUPPORT_FOC_MODE != FOC_MODE_NONE)
    { ADDR_NV_SENSOR, (sizeof(SENSOR_t) / sizeof(uint16_t)), C_ERR_NV_SENSOR},
#endif /* (_SUPPORT_TRIAXIS_MLX90363 != FALSE) || (_SUPPORT_TRIAXIS_MLX9038x != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90367 != FALSE) || (_SUPPORT_TRIAXIS_MLX90372 != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90377 != FALSE) || (_SUPPORT_TRIAXIS_MLX90395 != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90421 != FALSE) || (_SUPPORT_TRIAXIS_MLX90422 != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90425 != FALSE) || (_SUPPORT_TRIAXIS_MLX90426 != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90427 != FALSE) || \
        * (_SUPPORT_INDUCTIVE_POS_SENSOR_MLX90513 != FALSE) || \
        * (_SUPPORT_DUAL_HALL_LATCH_MLX92251 != FALSE) || (_SUPPORT_DUAL_HALL_LATCH_MLX92255 != FALSE) ||\
        * (_SUPPORT_FOC_MODE != FOC_MODE_NONE) */
    { ADDR_NV_ACT_STALL, (sizeof(ACT_STALL_t) / sizeof(uint16_t)), C_ERR_NV_ACT_STALL},
#if (I2C_COMM != FALSE)
    { ADDR_NV_I2C_PARAMS, (sizeof(STD_I2C_PARAMS_t) / sizeof(uint16_t)), C_ERR_NV_I2C_PARAMS},
#endif /* (I2C_COMM != FALSE) */
#if (CAN_COMM != FALSE)
    { ADDR_NV_CAN_PARAMS, (sizeof(STD_CAN_PARAMS_t) / sizeof(uint16_t)), C_ERR_NV_CAN_PARAMS},
#endif /* (CAN_COMM != FALSE) */
};


/*! Default Non Volatile Memory Header structure values */
#if (_APP_ZWICKAU != FALSE)
#define C_NV_DEF_HEADER \
    { \
        .u8CRC8 = 0x00U,                                                        /* 0x00: CRC-8 */ \
        .u8Revision = C_NV_REV,                                                 /* 0x01: NVRAM Structure revision */ \
        .u16ConfigurationID = ((('M' - '@') << 10) | (('T' - '@') << 5) | ('Z' - '@')), /* 0x02: Configuration ID */ \
        .u12StructIDs = C_HDR_STRUCT_ID,                                        /* 0x04.[11:0]: Structure-ID's */ \
        .u4WriteCycleCountMSW = 0x0U,                                           /* 0x04.[15:12]: Write Cycle Count (MSW) */ \
        .u16WriteCycleCountLSW = 0x0001U                                        /* 0x06: Write Cycle Count (LSW) */ \
    }
#else  /* (_APP_ZWICKAU != FALSE) */
#define C_NV_DEF_HEADER \
    { \
        .u8CRC8 = 0x00U,                                                        /* 0x00: CRC-8 */ \
        .u8Revision = C_NV_REV,                                                 /* 0x01: NVRAM Structure revision */ \
        .u16ConfigurationID = CONFIGURATION_ID,                                 /* 0x02: Configuration ID */ \
        .u12StructIDs = C_HDR_STRUCT_ID,                                        /* 0x04.[11:0]: Structure-ID's */ \
        .u2WriteCycleCountMSW = 0x0U,                                           /* 0x04.[15:12]: Write Cycle Count (MSW) */ \
        .u16WriteCycleCountLSW = 0x0001U                                        /* 0x06: Write Cycle Count (LSW) */ \
    }
#endif /* (_APP_ZWICKAU != FALSE) */

#if (LIN_COMM != FALSE)
#if (LINPROT != LIN2X_AIRVENT12)
/*! Default Standard LIN Parameters */
#define C_NV_DEF_STD_LIN_PARAMS \
    { \
        .u8CRC8 = 0x00U,                                                        /* 0x00: CRC8 */ \
        .u3LinUV = 0U,                                                          /* 0x01.[2:0]: LIN UV threshold */ \
        .u1BusTimeOutSleep = 1U,                                                /* 0x01.[3]: Bus-TimeOut Sleep */ \
        .u4Reserved = 0U,                                                       /* 0x01.[7:4]: Reserved */ \
        .u8NAD = 0x7FU,                                                         /* 0x02: Node Address */ \
        .u8ControlFrameID = mlxCONTROL,                                         /* 0x03: Control-message Frame-ID */ \
        .u8StatusFrameID = mlxSTATUS,                                           /* 0x04: Status-message Frame-ID */ \
        .u8Variant = C_VARIANT_ID,                                              /* 0x05: Product Reference (R:B2-00/W:B4) */ \
        .u8HardwareID = 0xFFU,                                                  /* 0x06: Product Reference (R:B2-00/W:B4) */ \
        .u8SoftwareID = 0xFFU                                                   /* 0x07: Product Reference (R:B2-00/W:B4) */ \
    }
#else  /* (LINPROT != LIN2X_AIRVENT12) */
/*! Default Standard LIN Parameters (AirVent) */
#define C_NV_DEF_STD_LIN_PARAMS \
    { \
        .u8CRC8 = 0x00U,                                                        /* 0x00: CRC8 */ \
        .u3LinUV = 1U,                                                          /* 0x01.[2:0]: LIN UV threshold (7.0V) - MGU_STELLMOTOR_LUFTDUESE_V12: 4.6.2.1 */ \
        .u1BusTimeOutSleep = 1U,                                                /* 0x01.[3]: Bus-TimeOut Sleep */ \
        .u4Reserved = 0U,                                                       /* 0x01.[7:4]: Reserved */ \
        .u8NAD = 0x7FU,                                                         /* 0x02: Node Address */ \
        .u8ControlFrameID = mlxCONTROL,                                         /* 0x03: Control-message Frame-ID */ \
        .u8StatusFrameID = mlxSTATUS,                                           /* 0x04: Status-message Frame-ID */ \
        .u8Variant = C_VARIANT_ID,                                              /* 0x05: Product Reference (R:B2-00/W:B4) */ \
        .u8HardwareID = 0xFFU,                                                  /* 0x06: Product Reference (R:B2-00/W:B4) */ \
        .u8Status2FrameID = mlxSTATUS2                                          /* 0x07: Status-message Frame-ID */ \
    }
#endif /* (LINPROT != LIN2X_AIRVENT12) */

#if (LINPROT == LIN2X_HVAC52)
#if (_SUPPORT_LINAA_ESHUNT != FALSE)
/*! Default data for User Non Volatile Memory ENH_LIN_PARAMS */
#define C_NV_DEF_ENH_LIN_PARAMS \
    { \
        .u8CRC8 = 0x00U,                                                        /* 0x00: CRC8 */ \
        .u8LINAA_EShunt = 200U,                                                 /* 0x01: External LIN shunt */ \
        .u16FunctionID = C_FUNCTION_ID,                                         /* 0x02: Function ID */ \
        .u8GAD = 0xFFU,                                                         /* 0x04: Group Address */ \
        .u8GroupControlFrameID = 0xFFU,                                         /* 0x05: Group Control-message Frame-ID */ \
        .u16Reserved = 0xFFFFU                                                  /* 0x06: Reserved */ \
    }
#else  /* (_SUPPORT_LINAA_ESHUNT != FALSE) */
/*! Default data for User Non Volatile Memory ENH_LIN_PARAMS, without external LIN-shunt */
#define C_NV_DEF_ENH_LIN_PARAMS \
    { \
        .u8CRC8 = 0x00U,                                                        /* 0x00: CRC8 */ \
        .u8LINAA_EShunt = 0x00U,                                                /* 0x01: External LIN shunt */ \
        .u16FunctionID = C_FUNCTION_ID,                                         /* 0x02: Function ID */ \
        .u8GAD = 0xFFU,                                                         /* 0x04: Group Address */ \
        .u8GroupControlFrameID = 0xFFU,                                         /* 0x05: Group Control-message Frame-ID */ \
        .u16Reserved = 0xFFFFU                                                  /* 0x06: Reserved */ \
    }
#endif /* (_SUPPORT_LINAA_ESHUNT != FALSE) */
#endif /* (LINPROT == LIN2X_HVAC52) */

#if (_SUPPORT_UDS != FALSE)
#if ((__APP_VERSION_REVISION__ & 0xF000) != 0xE000)
/*! Default data for UDS */
#define C_NV_DEF_UDS_LIN_PARAMS \
    { \
        .u8CRC8 = 0x00U,                                                        /*!< 0x00: CRC8 */ \
        .u8Reserved = 0x00U,                                                    /*!< 0x01: Reserved */ \
        .UDS_Reserved = 0x0123U,                                                /*!< 0x02: Reserved for UDS Sub-System ID */ \
        .UDS_FazitProductionDate = 0xFFFFU,                                     /*!< 0x04: Production Date Tier 1: [15:9] Year (00-99), [8:5] Month (1-12), [4:0] Day (1-31) (0x6E00-0x6FFF) */ \
        .UDS_FazitEOLNumber = 0xFFFFU,                                          /*!< 0x06: EOL Number Fazit (4 digits-BCD) (0x6E00-0x6FFF) */ \
        .UDS_FazitManufacturerNumber = 0xFFFFU,                                 /*!< 0x08: Manufacturer Number Fazit (4 digits-BCD) (0x6E00-0x6FFF) */ \
        .UDS_SystemName = {{('-' - ' '), ('-' - ' '), ('-' - ' '), ('-' - ' ')},  /*!< 0x0A-0x12: UDS System Name [11:0] (0x6C00-0x6DFF) */ \
                           {('-' - ' '), ('-' - ' '), ('-' - ' '), ('-' - ' ')},  /* 0x0D-0x0F: UDS System Name [7:4] */ \
                           {('-' - ' '), ('-' - ' '), ('-' - ' '), ('-' - ' ')}}, /* 0x10-0x12: UDS System Name [11:8] */ \
        .UDS_SystemName_12 = ('-' - ' '),                                       /*!< 0x13: UDS System Name [12] (0x6C00-0x6DFF) */ \
        .UDS_SerialNumber = {{('-' - ' '), ('-' - ' '), ('-' - ' '), ('-' - ' ')},  /*!< 0x14-0x22: UDS Serial-number[20] (0x6A00-0x6BFF) */ \
                             {('-' - ' '), ('-' - ' '), ('-' - ' '), ('-' - ' ')},  /* 0x17-0x19 */ \
                             {('-' - ' '), ('-' - ' '), ('-' - ' '), ('-' - ' ')},  /* 0x1A-0x1C */ \
                             {('-' - ' '), ('-' - ' '), ('-' - ' '), ('-' - ' ')},  /* 0x1D-0x1F */ \
                             {('-' - ' '), ('-' - ' '), ('-' - ' '), ('-' - ' ')}}, /* 0x20-0x22 */ \
        .UDS_HwVersion_LSB = '-',                                               /*!< 0x23: UDS Hardware version (LSB) (0x6800-0x69FF) */ \
        .UDS_HwVersion_MSB = '-',                                               /*!< 0x24: UDS Hardware version (MSB) (0x6800-0x69FF) */ \
        .UDS_HwVersion_XSB = '-',                                               /*!< 0x25: UDS Hardware version (XSB) (0x6800-0x69FF) */ \
        .UDS_HardwareNumber = {{('-' - ' '), ('-' - ' '), ('-' - ' '), ('-' - ' ')},  /*!< 0x26-0x2E: UDS Hardware number[11] (0x6600-0x67FF) */ \
                               {('-' - ' '), ('-' - ' '), ('-' - ' '), ('-' - ' ')},  /* 0x29-0x2B */ \
                               {('-' - ' '), ('-' - ' '), ('-' - ' '), ('-' - ' ')}}, /* 0x2C-0x2E */ \
        .UDS_SwVersion = {{('-' - ' '), ('-' - ' '), ('-' - ' '), ('-' - ' ')}}, /*!< 0x2F-0x31: UDS Software Version[4] (0x6400-0x65FF) */ \
        .UDS_SparePartNumber = {{('-' - ' '), ('-' - ' '), ('-' - ' '), ('-' - ' ')},  /*!< 0x32-0x3A: UDS Spare part number[11] (0x6200-0x63FF) */ \
                                {('-' - ' '), ('-' - ' '), ('-' - ' '), ('-' - ' ')},  /* 0x35-0x37 */ \
                                {('-' - ' '), ('-' - ' '), ('-' - ' '), ('-' - ' ')}}, /* 0x38-0x3A */ \
        .UDS_ReservedB = 0x00U,                                                 /*!< 0x3B: Reserved */ \
        .u16Reserved = {0x0000U, 0x0000U}                                       /*!< 0x3C-0x3F: Reserved */ \
    }
#else  /* ((__APP_VERSION_REVISION__ & 0xF000) != 0xE000) */
/*! Default data for UDS (Engineering version) */
#define C_NV_DEF_UDS_LIN_PARAMS \
    { \
        .u8CRC8 = 0x00U,                                                        /*!< 0x00: CRC8 */ \
        .u8Reserved = 0x00U,                                                    /*!< 0x01: Reserved */ \
        .UDS_Reserved = 0x0123U,                                                /*!< 0x02: Reserved for UDS Sub-System ID */ \
        .UDS_FazitProductionDate = ((17U << 9) | (12 << 5) | (5U)),             /*!< 0x04: Production Date Tier 1: [15:9] Year (00-99), [8:5] Month (1-12), [4:0] Day (1-31) (0x6E00-0x6FFF) */ \
        .UDS_FazitEOLNumber = 0x0001U,                                          /*!< 0x06: EOL Number Fazit (4 digits-BCD) (0x6E00-0x6FFF) */ \
        .UDS_FazitManufacturerNumber = 0x0001U,                                 /*!< 0x08: Manufacturer Number Fazit (4 digits-BCD) (0x6E00-0x6FFF) */ \
        .UDS_SystemName = {{('E' - ' '), ('M' - ' '), ('A' - ' '), ('N' - ' ')},  /*!< 0x0A-0x12: UDS System Name [11:0] (0x6C00-0x6DFF) */ \
                           {('M' - ' '), ('E' - ' '), ('T' - ' '), ('S' - ' ')},  /* 0x0D-0x0F: UDS System Name [7:4] */ \
                           {('Y' - ' '), ('S' - ' '), ('S' - ' '), ('D' - ' ')}}, /* 0x10-0x12: UDS System Name [11:8] */ \
        .UDS_SystemName_12 = ('U' - ' '),                                       /*!< 0x13: UDS System Name [12] (0x6C00-0x6DFF) */ \
        .UDS_SerialNumber = {{(((__APP_VERSION_REVISION__ & 0xF) + '0') - ' '),  /*!< 0x14-0x22: UDS Serial-number[20] (0x6A00-0x6BFF) */ \
                              ((((__APP_VERSION_REVISION__ >> 4) & 0xF) + '0') - ' '), \
                              ((((__APP_VERSION_REVISION__ >> 8) & 0xF) + '0') - ' '), \
                              (((__APP_VERSION_REVISION__ >> 12) + 'A' - 10) - ' ')}, /* 0x14-0x16 */ \
                             {((__APP_VERSION_MINOR__ + '0') - ' '), ('.' - ' '), \
                              ((__APP_VERSION_MAJOR__ + '0') - ' '), ('_' - ' ')}, /* 0x17-0x19 */ \
                             {('R' - ' '), ('E' - ' '), ('B' - ' '), ('M' - ' ')}, /* 0x1A-0x1C */ \
                             {('U' - ' '), ('N' - ' '), ('L' - ' '), ('A' - ' ')}, /* 0x1D-0x1F */ \
                             {('I' - ' '), ('R' - ' '), ('E' - ' '), ('S' - ' ')}}, /* 0x20-0x22 */ \
        .UDS_HwVersion_LSB = '1',                                               /*!< 0x23: UDS Hardware version (LSB) (0x6800-0x69FF) */ \
        .UDS_HwVersion_MSB = '0',                                               /*!< 0x24: UDS Hardware version (MSB) (0x6800-0x69FF) */ \
        .UDS_HwVersion_XSB = 'H',                                               /*!< 0x25: UDS Hardware version (XSB) (0x6800-0x69FF) */ \
        .UDS_HardwareNumber = {{('1' - ' '), ('_' - ' '), ('R' - ' '), ('N' - ' ')},  /*!< 0x26-0x2E: UDS Hardware number[11] (0x6600-0x67FF) */ \
                               {('_' - ' '), ('W' - ' '), ('H' - ' '), ('_' - ' ')},  /* 0x29-0x2B */ \
                               {('S' - ' '), ('D' - ' '), ('U' - ' '), ('-' - ' ')}}, /* 0x2C-0x2E */ \
        .UDS_SwVersion = {{('1' - ' '), ('0' - ' '), ('0' - ' '), ('E' - ' ')}},  /*!< 0x2F-0x31: UDS Software Version[4] (0x6400-0x65FF) */ \
        .UDS_SparePartNumber = {{('R' - ' '), ('N' - ' '), ('T' - ' '), ('R' - ' ')},  /*!< 0x32-0x3A: UDS Spare part number[11] (0x6200-0x63FF) */ \
                                {('A' - ' '), ('P' - ' '), ('E' - ' '), ('R' - ' ')},  /* 0x35-0x37 */ \
                                {('A' - ' '), ('P' - ' '), ('S' - ' '), ('-' - ' ')}}, /* 0x38-0x3A */ \
        .UDS_ReservedB = 0x00U,                                                 /*!< 0x3B: Reserved */ \
        .u16Reserved = {0x0000U, 0x0000U}                                       /*!< 0x3C-0x3F: Reserved */ \
    }
#endif /* ((__APP_VERSION_REVISION__ & 0xF000) != 0xE000) */
#endif /* (_SUPPORT_UDS != FALSE) */
#endif /* (LIN_COMM != FALSE) */

/*! Default End-of-Line Parameters */
#define C_NV_DEF_EOL_PARAMS \
    { \
        .u8CRC8 = 0x00U,                                                        /* 0x00: CRC8 */ \
        .u7EmergencyRunPos = 100U,                                              /* 0x01.[6:0]: Emergency/Safety position (0-100%) */ \
        .u1EmergencyRunPosEna = FALSE,                                          /* 0x01.[7]: Emergency/Safety position Disable/Enable */ \
        .u1MotorDirectionCCW = C_MOTOR_ROTATION_CW,                             /* 0x02.[0]: Motor rotational direction: 0=CW, 1=CCW */ \
        .u1StallDetectorEna = C_STALLDET_ENA,                                   /* 0x02.[1]: Stall-detector: 0=Disabled, 1=Enabled */ \
        .u1RealTravelSaved = FALSE,                                             /* 0x02.[2]: Travel Saved */ \
        .u1PorCalibration = FALSE,                                              /* 0x02.[3]: Power-on calibration */ \
        .u4Reserved_2 = 0x0U,                                                   /* 0x02.[7:4]: Reserved */ \
        .u8EndStopTime = C_ENDSTOP_TIME,                                        /* 0x03: End-stop pause time */ \
        .u16RealTravel = C_DEFAULT_TRAVEL,                                      /* 0x04: Real Travel */ \
        .u8TravelToleranceLo = C_TRAVEL_TOLERANCE_LO,                           /* 0x06: Default Travel Tolerance (Lower) */ \
        .u8TravelToleranceUp = C_TRAVEL_TOLERANCE_UP                            /* 0x07: Default Travel Tolerance (Upper) */ \
    }

#if (_SUPPORT_APP_SAVE != FALSE)
/*! Default data for APP_PARAMS_t */
#define C_NV_DEF_APP_PARAMS \
    { \
        .u8CRC8 = 0x00U,                                                        /* 0x00: CRC8 */ \
        .u4PageID = 0x0U,                                                       /* 0x01.[3:0]: u4PageID */ \
        .u4WrtCntH = 0x0U,                                                      /* 0x01.[7:4]: u4WrtCntH */ \
        .u16WrtCntL = 0x0000U,                                                  /* 0x02: u16WrtCntL */ \
        .u16ParamLSW = 0x0000U,                                                 /* 0x04: u16ParamLSW */ \
        .u16ParamMSW = 0x0000U                                                  /* 0x06: u16ParamMSW */ \
    }
#endif /* (_SUPPORT_APP_SAVE != FALSE) */

#if (_SUPPORT_TRIAXIS_MLX90363 != FALSE)
/*! Default data for SENSOR_t (SPI Triaxis) */
#define C_NV_DEF_SENSOR_PARAMS \
    { \
        .u8CRC8 = 0x00U,                                                        /* 0x00: CRC8 */ \
        .u4SensorType = C_SENSOR_MLX90363,                                      /* 0x01[3:0]: Sensor-type */ \
        .u3Reserved = 0x0U,                                                     /* 0x01[6:4]: Reserved */ \
        .u1CDI = FALSE,                                                         /* 0x01[7]: CDI On/Off */ \
        .u8TriaxisOffsetX = 0x00U,                                              /* 0x02: Triaxis Offset X */ \
        .u8TriaxisOffsetY = 0x00U,                                              /* 0x03: Triaxis Offset Y */ \
        .u4TriaxisMagnetPolePairs = C_SENSE_MAGNET_POLE_PAIRS,                  /* 0x04.[3:0]: Triaxis Magnet Pole Pairs */ \
        .u11TriaxisAmplitudeCorrection = 0x000U,                                /* 0x04.[14:4]: Triaxis X/Y-amplitude correction */ \
        .u1TriaxisDirection = FALSE,                                            /* 0x04.[15]: Triaxis Direction (0 = same as Act; 1 = Reverse) */ \
        .u16TriaxisAngleOffset = 0x0000U                                        /* 0x06: Triaxis Angle Offset */ \
    }
#elif (_SUPPORT_TRIAXIS_MLX90367 != FALSE) || (_SUPPORT_TRIAXIS_MLX90372 != FALSE)
/*! Default data for SENSOR_t (SENT Triaxis) */
#define C_NV_DEF_SENSOR_PARAMS \
    { \
        .u8CRC8 = 0x00U,                                                        /* 0x00: CRC8 */ \
        .u4SensorType = C_SENSOR_MLX90372,                                      /* 0x01[3:0]: Sensor-type */ \
        .u3Reserved = 0x0U,                                                     /* 0x01[6:4]: Reserved */ \
        .u1CDI = FALSE,                                                         /* 0x01[7]: CDI On/Off */ \
        .u8TriaxisOffsetX = 0x00U,                                              /* 0x02: Triaxis Offset X */ \
        .u8TriaxisOffsetY = 0x00U,                                              /* 0x03: Triaxis Offset Y */ \
        .u4TriaxisMagnetPolePairs = C_SENSE_MAGNET_POLE_PAIRS,                  /* 0x04.[3:0]: Triaxis Magnet Pole Pairs */ \
        .u11TriaxisAmplitudeCorrection = 0x000U,                                /* 0x04.[14:4]: Triaxis X/Y-amplitude correction */ \
        .u1TriaxisDirection = FALSE,                                            /* 0x04.[15]: Triaxis Direction (0 = same as Act; 1 = Reverse) */ \
        .u16TriaxisAngleOffset = 0x0000U                                        /* 0x06: Triaxis Angle Offset */ \
    }
#elif (_SUPPORT_TRIAXIS_MLX90377 != FALSE)
/*! Default data for SENSOR_t (SENT/SPC Triaxis) */
#define C_NV_DEF_SENSOR_PARAMS \
    { \
        .u8CRC8 = 0x00U,                                                        /* 0x00: CRC8 */ \
        .u4SensorType = C_SENSOR_MLX90377,                                      /* 0x01[3:0]: Sensor-type */ \
        .u3Reserved = 0x0U,                                                     /* 0x01[6:4]: Reserved */ \
        .u1CDI = FALSE,                                                         /* 0x01[7]: CDI On/Off */ \
        .u8TriaxisOffsetX = 0x00U,                                              /* 0x02: Triaxis Offset X */ \
        .u8TriaxisOffsetY = 0x00U,                                              /* 0x03: Triaxis Offset Y */ \
        .u4TriaxisMagnetPolePairs = C_SENSE_MAGNET_POLE_PAIRS,                  /* 0x04.[3:0]: Triaxis Magnet Pole Pairs */ \
        .u11TriaxisAmplitudeCorrection = 0x000U,                                /* 0x04.[14:4]: Triaxis X/Y-amplitude correction */ \
        .u1TriaxisDirection = FALSE,                                            /* 0x04.[15]: Triaxis Direction (0 = same as Act; 1 = Reverse) */ \
        .u16TriaxisAngleOffset = 0x0000U                                        /* 0x06: Triaxis Angle Offset */ \
    }
#elif (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
/*! Default data for SENSOR_t (Analogue Resolver) */
#define C_NV_DEF_SENSOR_PARAMS \
    { \
        .u8CRC8 = 0x00U,                                                        /* 0x00: CRC8 */ \
        .u4SensorType = C_SENSOR_MLX90380,                                      /* 0x01[3:0]: Sensor-type */ \
        .u3Reserved = 0x0U,                                                     /* 0x01[6:4]: Reserved */ \
        .u1CDI = FALSE,                                                         /* 0x01[7]: CDI On/Off */ \
        .u8TriaxisOffsetX = 0x00U,                                              /* 0x02: Triaxis Offset X */ \
        .u8TriaxisOffsetY = 0x00U,                                              /* 0x03: Triaxis Offset Y */ \
        .u4TriaxisMagnetPolePairs = C_SENSE_MAGNET_POLE_PAIRS,                  /* 0x04.[3:0]: Triaxis Magnet Pole Pairs */ \
        .u11TriaxisAmplitudeCorrection = 0x000U,                                /* 0x04.[14:4]: Triaxis X/Y-amplitude correction */ \
        .u1TriaxisDirection = FALSE,                                            /* 0x04.[15]: Triaxis Direction (0 = same as Act; 1 = Reverse) */ \
        .u16TriaxisAngleOffset = 0x0000U                                        /* 0x06: Triaxis Angle Offset */ \
    }
#elif (_SUPPORT_TRIAXIS_MLX90395 != FALSE)
/*! Default data for SENSOR_t (SPI Triaxis) */
#define C_NV_DEF_SENSOR_PARAMS \
    { \
        .u8CRC8 = 0x00U,                                                        /* 0x00: CRC8 */ \
        .u4SensorType = C_SENSOR_MLX90395,                                      /* 0x01[3:0]: Sensor-type */ \
        .u3Reserved = 0x0U,                                                     /* 0x01[6:4]: Reserved */ \
        .u1CDI = FALSE,                                                         /* 0x01[7]: CDI On/Off */ \
        .u8TriaxisOffsetX = 0x00U,                                              /* 0x02: Triaxis Offset X */ \
        .u8TriaxisOffsetY = 0x00U,                                              /* 0x03: Triaxis Offset Y */ \
        .u4TriaxisMagnetPolePairs = C_SENSE_MAGNET_POLE_PAIRS,                  /* 0x04.[3:0]: Triaxis Magnet Pole Pairs */ \
        .u11TriaxisAmplitudeCorrection = 0x000U,                                /* 0x04.[14:4]: Triaxis X/Y-amplitude correction */ \
        .u1TriaxisDirection = FALSE,                                            /* 0x04.[15]: Triaxis Direction (0 = same as Act; 1 = Reverse) */ \
        .u16TriaxisAngleOffset = 0x0000U                                        /* 0x06: Triaxis Angle Offset */ \
    }
#elif (_SUPPORT_TRIAXIS_MLX90421 != FALSE)
/*! Default data for SENSOR_t (PWM Triaxis) */
#define C_NV_DEF_SENSOR_PARAMS \
    { \
        .u8CRC8 = 0x00U,                                                        /* 0x00: CRC8 */ \
        .u4SensorType = C_SENSOR_MLX90421,                                      /* 0x01[3:0]: Sensor-type */ \
        .u3Reserved = 0x0U,                                                     /* 0x01[6:4]: Reserved */ \
        .u1CDI = FALSE,                                                         /* 0x01[7]: CDI On/Off */ \
        .u8TriaxisOffsetX = 0x00U,                                              /* 0x02: Triaxis Offset X */ \
        .u8TriaxisOffsetY = 0x00U,                                              /* 0x03: Triaxis Offset Y */ \
        .u4TriaxisMagnetPolePairs = C_SENSE_MAGNET_POLE_PAIRS,                  /* 0x04.[3:0]: Triaxis Magnet Pole Pairs */ \
        .u11TriaxisAmplitudeCorrection = 0x000U,                                /* 0x04.[14:4]: Triaxis X/Y-amplitude correction */ \
        .u1TriaxisDirection = FALSE,                                            /* 0x04.[15]: Triaxis Direction (0 = same as Act; 1 = Reverse) */ \
        .u16TriaxisAngleOffset = 0x0000U                                        /* 0x06: Triaxis Angle Offset */ \
    }
#elif (_SUPPORT_TRIAXIS_MLX90422 != FALSE)
/*! Default data for SENSOR_t (SENT Triaxis) */
#define C_NV_DEF_SENSOR_PARAMS \
    { \
        .u8CRC8 = 0x00U,                                                        /* 0x00: CRC8 */ \
        .u4SensorType = C_SENSOR_MLX90422,                                      /* 0x01[3:0]: Sensor-type */ \
        .u3Reserved = 0x0U,                                                     /* 0x01[6:4]: Reserved */ \
        .u1CDI = FALSE,                                                         /* 0x01[7]: CDI On/Off */ \
        .u8TriaxisOffsetX = 0x00U,                                              /* 0x02: Triaxis Offset X */ \
        .u8TriaxisOffsetY = 0x00U,                                              /* 0x03: Triaxis Offset Y */ \
        .u4TriaxisMagnetPolePairs = C_SENSE_MAGNET_POLE_PAIRS,                  /* 0x04.[3:0]: Triaxis Magnet Pole Pairs */ \
        .u11TriaxisAmplitudeCorrection = 0x000U,                                /* 0x04.[14:4]: Triaxis X/Y-amplitude correction */ \
        .u1TriaxisDirection = FALSE,                                            /* 0x04.[15]: Triaxis Direction (0 = same as Act; 1 = Reverse) */ \
        .u16TriaxisAngleOffset = 0x0000U                                        /* 0x06: Triaxis Angle Offset */ \
    }
#elif (_SUPPORT_TRIAXIS_MLX90425 != FALSE)
/*! Default data for SENSOR_t (PWM Triaxis) */
#define C_NV_DEF_SENSOR_PARAMS \
    { \
        .u8CRC8 = 0x00U,                                                        /* 0x00: CRC8 */ \
        .u4SensorType = C_SENSOR_MLX90425,                                      /* 0x01[3:0]: Sensor-type */ \
        .u3Reserved = 0x0U,                                                     /* 0x01[6:4]: Reserved */ \
        .u1CDI = FALSE,                                                         /* 0x01[7]: CDI On/Off */ \
        .u8TriaxisOffsetX = 0x00U,                                              /* 0x02: Triaxis Offset X */ \
        .u8TriaxisOffsetY = 0x00U,                                              /* 0x03: Triaxis Offset Y */ \
        .u4TriaxisMagnetPolePairs = C_SENSE_MAGNET_POLE_PAIRS,                  /* 0x04.[3:0]: Triaxis Magnet Pole Pairs */ \
        .u11TriaxisAmplitudeCorrection = 0x000U,                                /* 0x04.[14:4]: Triaxis X/Y-amplitude correction */ \
        .u1TriaxisDirection = FALSE,                                            /* 0x04.[15]: Triaxis Direction (0 = same as Act; 1 = Reverse) */ \
        .u16TriaxisAngleOffset = 0x0000U                                        /* 0x06: Triaxis Angle Offset */ \
    }
#elif (_SUPPORT_TRIAXIS_MLX90426 != FALSE)
/*! Default data for SENSOR_t (SENT Triaxis) */
#define C_NV_DEF_SENSOR_PARAMS \
    { \
        .u8CRC8 = 0x00U,                                                        /* 0x00: CRC8 */ \
        .u4SensorType = C_SENSOR_MLX90426,                                      /* 0x01[3:0]: Sensor-type */ \
        .u3Reserved = 0x0U,                                                     /* 0x01[6:4]: Reserved */ \
        .u1CDI = FALSE,                                                         /* 0x01[7]: CDI On/Off */ \
        .u8TriaxisOffsetX = 0x00U,                                              /* 0x02: Triaxis Offset X */ \
        .u8TriaxisOffsetY = 0x00U,                                              /* 0x03: Triaxis Offset Y */ \
        .u4TriaxisMagnetPolePairs = C_SENSE_MAGNET_POLE_PAIRS,                  /* 0x04.[3:0]: Triaxis Magnet Pole Pairs */ \
        .u11TriaxisAmplitudeCorrection = 0x000U,                                /* 0x04.[14:4]: Triaxis X/Y-amplitude correction */ \
        .u1TriaxisDirection = FALSE,                                            /* 0x04.[15]: Triaxis Direction (0 = same as Act; 1 = Reverse) */ \
        .u16TriaxisAngleOffset = 0x0000U                                        /* 0x06: Triaxis Angle Offset */ \
    }
#elif (_SUPPORT_TRIAXIS_MLX90427 != FALSE)
    /*! Default data for SENSOR_t (SPI Triaxis) */
    #define C_NV_DEF_SENSOR_PARAMS \
    { \
        .u8CRC8 = 0x00U,                                                        /* 0x00: CRC8 */ \
        .u4SensorType = C_SENSOR_MLX90427,                                      /* 0x01[3:0]: Sensor-type */ \
        .u3Reserved = 0x0U,                                                     /* 0x01[6:4]: Reserved */ \
        .u1CDI = FALSE,                                                         /* 0x01[7]: CDI On/Off */ \
        .u8TriaxisOffsetX = 0x00U,                                              /* 0x02: Triaxis Offset X */ \
        .u8TriaxisOffsetY = 0x00U,                                              /* 0x03: Triaxis Offset Y */ \
        .u4TriaxisMagnetPolePairs = C_SENSE_MAGNET_POLE_PAIRS,                  /* 0x04.[3:0]: Triaxis Magnet Pole Pairs */ \
        .u11TriaxisAmplitudeCorrection = 0x000U,                                /* 0x04.[14:4]: Triaxis X/Y-amplitude correction */ \
        .u1TriaxisDirection = FALSE,                                            /* 0x04.[15]: Triaxis Direction (0 = same as Act; 1 = Reverse) */ \
        .u16TriaxisAngleOffset = 0x0000U                                        /* 0x06: Triaxis Angle Offset */ \
    }
#elif (_SUPPORT_INDUCTIVE_POS_SENSOR_MLX90513 != FALSE)
/*! Default data for SENSOR_t (SENT/SPC Inductive Position Sensor) */
#define C_NV_DEF_SENSOR_PARAMS \
    { \
        .u8CRC8 = 0x00U,                                                        /* 0x00: CRC8 */ \
        .u4SensorType = C_SENSOR_MLX90513,                                      /* 0x01[3:0]: Sensor-type */ \
        .u3Reserved = 0x0U,                                                     /* 0x01[6:4]: Reserved */ \
        .u1CDI = FALSE,                                                         /* 0x01[7]: CDI On/Off */ \
        .u8ReservedX = 0x00U,                                                   /* 0x02: Reserved X */ \
        .u8ReservedY = 0x00U,                                                   /* 0x03: Reserved Y */ \
        .u4Reserved = 0U,                                                       /* 0x04.[3:0]: Reserved */ \
        .u11Reserved = 0x000U,                                                  /* 0x04.[14:4]: Reserved */ \
        .u1TriaxisDirection = FALSE,                                            /* 0x04.[15]: Triaxis Direction (0 = same as Act; 1 = Reverse) */ \
        .u16TriaxisAngleOffset = 0x0000U                                        /* 0x06: Triaxis Angle Offset */ \
    }
#elif (_SUPPORT_DUAL_HALL_LATCH_MLX92251 != FALSE)
/*! Default data for SENSOR_t (Dual Hall Latch) */
#define C_NV_DEF_SENSOR_PARAMS \
    { \
        .u8CRC8 = 0x00U,                                                        /* 0x00: CRC8 */ \
        .u4SensorType = C_SENSOR_MLX92251,                                      /* 0x01[3:0]: Sensor-type */ \
        .u3Reserved = 0x0U,                                                     /* 0x01[6:4]: Reserved */ \
        .u1CDI = FALSE,                                                         /* 0x01[7]: CDI On/Off */ \
        .u8TriaxisOffsetX = 0x00U,                                              /* 0x02: Triaxis Offset X */ \
        .u8TriaxisOffsetY = 0x00U,                                              /* 0x03: Triaxis Offset Y */ \
        .u4TriaxisMagnetPolePairs = C_SENSE_MAGNET_POLE_PAIRS,                  /* 0x04.[3:0]: Triaxis Magnet Pole Pairs */ \
        .u11TriaxisAmplitudeCorrection = 0x000U,                                /* 0x04.[14:4]: Triaxis X/Y-amplitude correction */ \
        .u1TriaxisDirection = FALSE,                                            /* 0x04.[15]: Triaxis Direction (0 = same as Act; 1 = Reverse) */ \
        .u16TriaxisAngleOffset = 0x0000U                                        /* 0x06: Triaxis Angle Offset */ \
    }
#elif (_SUPPORT_DUAL_HALL_LATCH_MLX92255 != FALSE)
/*! Default data for SENSOR_t (Dual Hall Latch) */
#define C_NV_DEF_SENSOR_PARAMS \
    { \
        .u8CRC8 = 0x00U,                                                        /* 0x00: CRC8 */ \
        .u4SensorType = C_SENSOR_MLX92255,                                      /* 0x01[3:0]: Sensor-type */ \
        .u3Reserved = 0x0U,                                                     /* 0x01[6:4]: Reserved */ \
        .u1CDI = FALSE,                                                         /* 0x01[7]: CDI On/Off */ \
        .u8TriaxisOffsetX = 0x00U,                                              /* 0x02: Triaxis Offset X */ \
        .u8TriaxisOffsetY = 0x00U,                                              /* 0x03: Triaxis Offset Y */ \
        .u4TriaxisMagnetPolePairs = C_SENSE_MAGNET_POLE_PAIRS,                  /* 0x04.[3:0]: Triaxis Magnet Pole Pairs */ \
        .u11TriaxisAmplitudeCorrection = 0x000U,                                /* 0x04.[14:4]: Triaxis X/Y-amplitude correction */ \
        .u1TriaxisDirection = FALSE,                                            /* 0x04.[15]: Triaxis Direction (0 = same as Act; 1 = Reverse) */ \
        .u16TriaxisAngleOffset = 0x0000U                                        /* 0x06: Triaxis Angle Offset */ \
    }
#elif (_SUPPORT_FOC_MODE != FOC_MODE_NONE)
/*! Default data for SENSOR_t (FOC Sensorless) */
#define C_NV_DEF_SENSOR_PARAMS \
    { \
        .u8CRC8 = 0x00U,                                                        /* 0x00: CRC8 */ \
        .u4SensorType = C_SENSOR_FOC,                                           /* 0x01[3:0]: Sensor-type */ \
        .u3Reserved = 0x0U,                                                     /* 0x01[6:4]: Reserved */ \
        .u1CDI = FALSE,                                                         /* 0x01[7]: CDI On/Off */ \
        .u8PidCoefP_LA = C_PID_LA_COEF_P,                                       /* 0x02: PID P-Coefficient LA-Target */ \
        .u8PidCoefI_LA = C_PID_LA_COEF_I,                                       /* 0x03: PID I-Coefficient LA-Target */ \
        .u8PidCoefD_LA = C_PID_LA_COEF_D,                                       /* 0x04: PID D-Coefficient LA-Target */ \
        .u8TargetLA = C_TARGET_LA,                                              /* 0x05: LA-Target */ \
        .u4StallLA_Threshold = C_STALL_LA_THRESHOLD_FOC,                        /* 0x06.[3:0]: Stall detector "LA" Angle-Threshold (MMP220720-1) */ \
        .u4StallLA_Width = C_STALL_LA_WIDTH_FOC,                                /* 0x06.[7:4]: Stall detector "LA" Width (MMP220720-1) */ \
        .u3OpenToCloseAmplitude = (C_LA_OPEN_TO_CLOSE_AMPL - 1U),               /* 0x07.[2:0]: Open-to-Close amplitude */ \
        .u5StallSpeed_Threshold = C_STALL_SPEED_THRESHOLD                       /* 0x07.[7:3] Speed Threshold (stall) */ \
    }
#endif /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) */

/*! Default data for User Non Volatile Memory ACT_STALL_t */
#define C_NV_DEF_ACT_STALL \
    { \
        .u8CRC8 = 0x00U,                                                        /* 0x00: CRC8 */ \
        .u7StallA_Threshold = C_STALL_A_THRESHOLD,                              /* 0x01.[6:0]: Stall-detector "A" threshold (Amplitude) */ \
        .u1StallA_Ena = C_STALL_A_DET,                                          /* 0x01.[7]: Stall-detector "A" Enable */ \
        .u4StallA_Width = C_STALL_A_WIDTH,                                      /* 0x02.[3:0]: Stall-detector "A" Width */ \
        .u4StallO_Width = C_STALL_O_WIDTH,                                      /* 0x02.[7:4]: Stall-detector "O" Width */ \
        .u7StallO_Threshold = C_STALL_O_THRESHOLD,                              /* 0x03.[6:0]: Stall-detector "A" threshold (Amplitude) */ \
        .u1StallO_Ena = C_STALL_O_DET,                                          /* 0x03.[7]: Stall-detector "A" Enable */ \
        .u7StallS_Threshold = C_STALL_S_THRESHOLD,                              /* 0x04.[6:0]: Stall-detector "S" threshold (Amplitude) */ \
        .u1StallS_Ena = C_STALL_S_DET,                                          /* 0x04.[7]: Stall-detector "S" Enable */ \
        .u4StallS_Width = C_STALL_S_WIDTH,                                      /* 0x05.[3:0]: Stall-detector "S" Width */ \
        .u1RestallPor = C_RESTALL_POR,                                          /* 0x05.[4]: Reserved */ \
        .u1StallSpeedDepended = C_STALL_SPEED_DEPENDED,                         /* 0x05.[5]: Stall speed depended */ \
        .u2Reserved = 0U,                                                       /* 0x05.[7:6]: Reserved */ \
        .u8RewindSteps = C_REWIND_STEPS,                                        /* 0x06: Reserved */ \
        .u8StallDetectorDelay = (C_DETECTOR_DELAY / C_MICROSTEP_PER_FULLSTEP)   /* 0x07: Stall detector delay */ \
    }

#if (I2C_COMM != FALSE)
/*! Default data for User Non Volatile Memory STD_I2C_PARAMS */
#define C_NV_DEF_I2C_PARAMS \
    { \
        .u8CRC8 = 0x00U,                                                        /* 0x00: CRC8 */ \
        .u8Address = C_I2C_ADDRESS,                                             /* 0x01: I2C Slave Address */ \
        .u16ReservedA = 0x0000U,                                                /* 0x02: Reserved A */ \
        .u16ReservedB = 0x0000U,                                                /* 0x04: Reserved B */ \
        .u16ReservedC = 0x0000U                                                 /* 0x06: Reserved C */ \
    }
#endif /* (I2C_COMM != FALSE) */

#if (CAN_COMM != FALSE)
#if (_SUPPORT_CAN_MCP2515 != FALSE)
/*! Default data for User Non Volatile Memory STD_CAN_PARAMS */
#define C_NV_DEF_CAN_PARAMS \
    { \
        .u8CRC8 = 0x00U,                                                        /* 0x00: CRC8 */ \
        .u8Baudrate = C_CAN_BAUDRATE_500K,                                      /* 0x01: CAN Baudrate */ \
        .u8StatusPeriod = 10U,                                                  /* 0x02: CAN Status Period */ \
        .u8FDBaudrate = C_CAN_BAUDRATE_NONE,                                    /* 0x01: CAN FD Baudrate (Not supported) */ \
        .u16IdInLSW = (DEMO_ID_IN & 0xFFFFU),                                   /* 0x04: CAN-in ID (LSW) */ \
        .u16IdInMSW = (DEMO_ID_IN >> 16), /*lint !e572 */                       /* 0x06: CAN-in ID (MSW) */ \
        .u16IdOutLSW = (ECU_1_ID_OUT & 0xFFFFU),                                /* 0x08: CAN-out ID (LSW) */ \
        .u16IdOutMSW = (ECU_1_ID_OUT >> 16), /*lint !e572 */                    /* 0x0A: CAN-out ID (MSW) */ \
        .u16ReservedB = 0x0000U,                                                /* 0x0C: Reserved B */ \
        .u16ReservedC = 0x0000U                                                 /* 0x0E: Reserved C */ \
    }
#elif (_SUPPORT_CAN_TCAN4550 != FALSE)
/*! Default data for User Non Volatile Memory STD_CAN_PARAMS */
#define C_NV_DEF_CAN_PARAMS \
    { \
        .u8CRC8 = 0x00U,                                                        /* 0x00: CRC8 */ \
        .u8Baudrate = C_CAN_BAUDRATE_500K,                                      /* 0x01: CAN Baudrate */ \
        .u8StatusPeriod = 10U,                                                  /* 0x02: CAN Status Period */ \
        .u8FDBaudrate = C_CAN_BAUDRATE_2000K,                                   /* 0x01: CAN FD Baudrate */ \
        .u16IdInLSW = (DEMO_ID_IN & 0xFFFFU),                                   /* 0x04: CAN-in ID (LSW) */ \
        .u16IdInMSW = (DEMO_ID_IN >> 16), /*lint !e572 */                       /* 0x06: CAN-in ID (MSW) */ \
        .u16IdOutLSW = (ECU_1_ID_OUT & 0xFFFFU),                                /* 0x08: CAN-out ID (LSW) */ \
        .u16IdOutMSW = (ECU_1_ID_OUT >> 16), /*lint !e572 */                    /* 0x0A: CAN-out ID (MSW) */ \
        .u16ReservedB = 0x0000U,                                                /* 0x0C: Reserved B */ \
        .u16ReservedC = 0x0000U                                                 /* 0x0E: Reserved C */ \
    }
#endif
#endif /* (CAN_COMM != FALSE) */

#if (_SUPPORT_NV_HEX != FALSE)
/** Non Volatile Memory values */
extern const NV_USER_MAP_t nv_defaults __attribute__((ep, addr(0x0000)));
/** Non Volatile Memory default values */
const NV_USER_MAP_t nv_defaults = {
    .hdr = C_NV_DEF_HEADER,                                                     /**< 1) header */
#if (LIN_COMM != FALSE)
    .stdlin[0] = C_NV_DEF_STD_LIN_PARAMS,                                       /**< 2a) Standard LIN parameters */
    .stdlin[1] = C_NV_DEF_STD_LIN_PARAMS,                                       /**< 2a) Standard LIN parameters */
#if (LINPROT == LIN2X_HVAC52)
    .enhlin[0] = C_NV_DEF_ENH_LIN_PARAMS,                                       /**< 2b) Enhanced LIN parameters */
    .enhlin[1] = C_NV_DEF_ENH_LIN_PARAMS,                                       /**< 2b) Enhanced LIN parameters */
#endif /* (LINPROT == LIN2X_HVAC52) */
#if (_SUPPORT_UDS != FALSE)
    .uds = C_NV_DEF_UDS_LIN_PARAMS,                                             /**< 2c) UDS Data */
#endif /* (_SUPPORT_UDS != FALSE) */
#endif /* (LIN_COMM != FALSE) */
    .eol = C_NV_DEF_EOL_PARAMS,                                                 /**< 3) Application EOL parameters */
#if (_SUPPORT_APP_SAVE != FALSE)
    .app = C_NV_DEF_APP_PARAMS,                                                 /**< 4) Application parameters */
#endif /* (_SUPPORT_APP_SAVE != FALSE) */
    .act =                                                                      /**< 5) Actuator Parameters */
    {
        .u8CRC8 = 0x00U,                                                        /* 0x00: CRC8 */
        .u8VsupRef = C_VSUP_REF,                                                /* 0x01: Vsupply reference [1/8V] */
        .u12GearBoxRatio = C_MOTOR_GEAR_BOX_RATIO,                              /* 0x02 [11: 0]: Gear-box ratio, e.g. 600:1 */
#if (MOTOR_TYPE == MT_NEMA_17HS19)
        .u4PolePairs = ((C_MOTOR_POLE_PAIRS / 5U) - 1U),                        /* 0x02.[15:12]: (Number of pole-pairs + 1) * 5 (5-80) */
#else  /* (MOTOR_TYPE == MT_NEMA_17HS19) */
        .u4PolePairs = (C_MOTOR_POLE_PAIRS - 1U),                               /* 0x02.[15:12]: Number of pole-pairs + 1 (1-16) */
#endif /* (MOTOR_TYPE == MT_NEMA_17HS19) */
#if defined (C_MOTOR_CONST_MV_PER_RPS)                                          /* MMP240726-2 */
        .u8MotorConstant = C_MOTOR_CONST_MV_PER_RPS,                            /* 0x04: Motor Constant [mV/RPS] */
#else  /* defined (C_MOTOR_CONST_MV_PER_RPS) */
        .u8MotorConstant = C_MOTOR_CONST_10MV_PER_RPS,                          /* 0x04: Motor Constant [10mV/RPS] */
#endif /* defined (C_MOTOR_CONST_MV_PER_RPS) */
#if (_SUPPORT_COIL_UNIT_10mR != FALSE)
        .u8MotorCoilRtot = C_TOT_COILS_R,                                       /* 0x05: Motor coil resistance (total, between phases) */
#elif (_SUPPORT_COIL_UNIT_100mR != FALSE)
        .u8MotorCoilRtot = ((C_TOT_COILS_R + 5U) / 10U),                        /* 0x05: Motor coil resistance (total, between phases) */
#else
        .u8MotorCoilRtot = ((C_TOT_COILS_R + 50U) / 100U),                      /* 0x05: Motor coil resistance (total, between phases) */
#endif
        .u13MinSpeed = C_SPEED_MIN,                                             /* 0x06.[12: 0]: Minimum speed */
        .u3MicroSteps = C_MOTOR_MICROSTEPS,                                     /* 0x06.[15:13]: Number of micro-steps: 2^n (or 1 << n); 0 = Full-steps */
        .u16Speed_1 = C_SPEED_0,                                                /* 0x08: Speed_1 */
        .u16Speed_2 = C_SPEED_1,                                                /* 0x0A: Speed_2 */
        .u16Speed_3 = C_SPEED_2,                                                /* 0x0C: Speed_3 */
        .u16Speed_4 = C_SPEED_3,                                                /* 0x0E: Speed_4 */
        .u16AccelerationConst = C_ACCELERATION_CONST,                           /* 0x10: Acceleration-constant */
        .u3AccelerationSteps = C_ACCELERATION_STEPS,                            /* 0x12.[2:0]: Acceleration-(u)Steps */
        .u3DecelerationSteps = C_DECELERATION_STEPS,                            /* 0x12.[5:3]: Deceleration-(u)Steps */
#if defined (__MLX81160__) || defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__)
        .u2MotorCurrentMultiplier = C_NV_CURR_DIV,                              /* 0x12.[7:6]: Motor current multiplier: 1, 2, 4 or 8 */
#elif defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
        .u2MotorCurrentMultiplier = (C_NV_CURR_DIV - C_MULTIPLIER_OFF),         /* 0x12.[7:6]: Motor current multiplier: 8, 16, 32 or 64 */
#endif
#if (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE)
#if (_APP_ZWICKAU != FALSE)
        .u8HoldingTorqueCurrent = 0U,                                           /* 0x13: Holding Torque current threshold */
#else  /* (_APP_ZWICKAU != FALSE) */
#if defined (__MLX81160__) || defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__)
        .u8HoldingTorqueCurrent = C_PID_HOLDING_CURR_LEVEL,                     /* 0x13: Minimum Running Torque current threshold (MMP240730-1) */
#elif defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
        .u8HoldingTorqueCurrent = (C_PID_HOLDING_CURR_LEVEL >> C_NV_CURR_DIV),  /* 0x13: Holding Torque current threshold (MMP240730-1) */
#endif
#endif /* (_APP_ZWICKAU != FALSE) */
        .u8RunningTorqueCurrent = (C_PID_RUNNING_CURR_LEVEL >> C_NV_CURR_DIV),  /* 0x14: Running Torque current threshold */
#else  /* (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) */
#if defined (__MLX81160__) || defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__)
        .u8HoldingTorqueCurrent = C_PID_RUNNING_CURR_MIN,                       /* 0x13: Minimum Running Torque current threshold */
#elif defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
        .u8HoldingTorqueCurrent = (C_PID_RUNNING_CURR_MIN >> C_NV_CURR_DIV),    /* 0x13: Holding Torque current threshold */
#endif
        .u8RunningTorqueCurrent = (C_PID_RUNNING_CURR_MAX >> C_NV_CURR_DIV),    /* 0x14: maximum Running Torque current threshold */
#endif /* (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) */
        .u4TorqueBoost1 = 0U,                                                   /* 0x15.[3:0]: TorqueBoost1 (5%) -25%..50% */
        .u4TorqueBoost2 = 5U,                                                   /* 0x15.[7:4]: TorqueBoost2 (5%)   0%..75% */
        .u8PidCoefP = C_PID_COEF_P,                                             /* 0x16: PID-Coefficient P */
        .u8PidCoefI = C_PID_COEF_I,                                             /* 0x17: PID-Coefficient I */
        .u8PidCoefD = C_PID_COEF_D,                                             /* 0x18: PID-Coefficient D */
#if (_SUPPORT_MOTOR_POSITION == C_MOTOR_POS_NONE)
        .u8PidStartupOrLowerHoldingLimit = (C_PID_STARTING_CURR_MAX >> C_NV_CURR_DIV),  /* 0x19: Startup current limit (MMP190809-1) */
#else  /* (_SUPPORT_MOTOR_POSITION == C_MOTOR_POS_NONE) */
        .u8PidStartupOrLowerHoldingLimit = C_MIN_HOLDCORR_RATIO,                /* 0x19: PID Lower-limit Holding (output) */
#endif /* (_SUPPORT_MOTOR_POSITION == C_MOTOR_POS_NONE) */
        .u8PidLowerLimit = C_MIN_CORR_RATIO,                                    /* 0x1A: PID Lower-limit Running (output) */
        .u8PidUpperLimit = C_MAX_CORR_RATIO,                                    /* 0x1B: PID Upper-limit (output) */
        .u7PidCtrlPeriod = C_PID_RUNNINGCTRL_PERIOD,                            /* 0x1C.[6:0]: PID running control-period */
        .u1PidPeriodTimeOrSpeed = C_PID_RUNNINGCTRL_PERIOD_UNIT,                /* 0x1C.[7]: PID Period is Time(0) or Speed(1) based */
        .u8AppOT = C_APP_OT_LEVEL + 60U,                                        /* 0x1D: Application Over-Temperature [C] */
        .u8AppUV = C_APP_UV_LEVEL,                                              /* 0x1E: Application Under-voltage level [1/8V] */
#if (_APP_SUPPLY_RANGE != C_APP_SUPPLY_RANGE_48V)
        .u8AppOV = (C_APP_OV_LEVEL - (uint8_t)(C_APP_OV_OFF / 12.5))            /* 0x1F: Application Over-voltage level [1/8V] */
#else  /* (_SUPPORT_APP_48V_ADC == FALSE) */
        .u8AppOV = (C_APP_OV_LEVEL - (uint8_t)(C_APP_OV_OFF / 25))              /* 0x1F: Application Over-voltage level [1/4V] */
#endif /* (_SUPPORT_APP_48V_ADC == FALSE) */
    },
#if (_SUPPORT_TRIAXIS_MLX90363 != FALSE) || (_SUPPORT_TRIAXIS_MLX9038x != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90367 != FALSE) || (_SUPPORT_TRIAXIS_MLX90372 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90377 != FALSE) || (_SUPPORT_TRIAXIS_MLX90395 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90421 != FALSE) || (_SUPPORT_TRIAXIS_MLX90422 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90425 != FALSE) || (_SUPPORT_TRIAXIS_MLX90426 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90427 != FALSE) || \
    (_SUPPORT_INDUCTIVE_POS_SENSOR_MLX90513 != FALSE) || \
    (_SUPPORT_DUAL_HALL_LATCH_MLX92251 != FALSE) || (_SUPPORT_DUAL_HALL_LATCH_MLX92255 != FALSE) || \
    (_SUPPORT_FOC_MODE != FOC_MODE_NONE)
    .sensor = C_NV_DEF_SENSOR_PARAMS,                                           /**< 6) Sensor parameters */
#endif /* (_SUPPORT_TRIAXIS_MLX90363 != FALSE) || (_SUPPORT_TRIAXIS_MLX9038x != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90367 != FALSE) || (_SUPPORT_TRIAXIS_MLX90372 != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90377 != FALSE) || (_SUPPORT_TRIAXIS_MLX90395 != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90421 != FALSE) || (_SUPPORT_TRIAXIS_MLX90422 != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90425 != FALSE) || (_SUPPORT_TRIAXIS_MLX90426 != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90427 != FALSE) || \
        * (_SUPPORT_INDUCTIVE_POS_SENSOR_MLX90513 != FALSE) || \
        * (_SUPPORT_DUAL_HALL_LATCH_MLX92251 != FALSE) || (_SUPPORT_DUAL_HALL_LATCH_MLX92255 != FALSE) || \
        * (_SUPPORT_FOC_MODE != FOC_MODE_NONE) */
    .stall = C_NV_DEF_ACT_STALL                                                 /**< 7) Stall parameters */
#if (_SUPPORT_NV_LOG_ERROR != FALSE)
             .errlog = 0,                                                       /**< 8) Error Log */
#endif /* (_SUPPORT_NV_LOG_ERROR != FALSE) */
#if (I2C_COMM != FALSE)
             .i2c = C_NV_DEF_I2C_PARAMS,                                        /**< 9) LIN Parameters */
#endif /* (I2C_COMM != FALSE) */
#if (CAN_COMM != FALSE)
             .can = C_NV_DEF_CAN_PARAMS                                         /**< 10) CAN Parameters */
#endif /* (CAN_COMM != FALSE) */
};
#endif /* (_SUPPORT_NV_HEX != FALSE) */

/*! Default data for User NVM HEADER */
static const HEADER_t DefHeader = C_NV_DEF_HEADER;

#if (LIN_COMM != FALSE)
/*! Default data for User NVM STD_LIN_PARAMS */
static const STD_LIN_PARAMS_t DefStdLinParams = C_NV_DEF_STD_LIN_PARAMS;

#if (LINPROT == LIN2X_HVAC52)
/*! Default data for User NVM ENH_LIN_PARAMS */
const ENH_LIN_PARAMS_t DefEnhLinParams = C_NV_DEF_ENH_LIN_PARAMS;
#endif /* (LINPROT == LIN2X_HVAC52) */

#if (_SUPPORT_UDS != FALSE)
/*! Default data for User NVM UDS_LIN_PARAMS */
const UDS_LIN_PARAMS_t DefUds = C_NV_DEF_UDS_LIN_PARAMS; /*lint !e651 */
#endif /* (_SUPPORT_UDS != FALSE) */
#endif /* (LIN_COMM != FALSE) */

/*! Default data for User NVM APP_EOL */
const APP_EOL_t DefEolParams = C_NV_DEF_EOL_PARAMS;

/*! Default data for User NVM ACT_PARAMS */
const ACT_PARAMS_t DefActParams =
{
    .u8CRC8 = 0x00U,                                                            /* 0x00: CRC8 */
#if (_APP_SUPPLY_RANGE != C_APP_SUPPLY_RANGE_48V)
    .u8VsupRef = C_VSUP_REF,                                                    /* 0x01: Vsupply reference [1/8V] */
#else  /* (_APP_SUPPLY_RANGE != C_APP_SUPPLY_RANGE_48V) */
    .u8VsupRef = C_VSUP_REF,                                                    /* 0x01: Vsupply reference [1/4V] */
#endif /* (_APP_SUPPLY_RANGE != C_APP_SUPPLY_RANGE_48V) */
    .u12GearBoxRatio = C_MOTOR_GEAR_BOX_RATIO,                                  /* 0x02 [11: 0]: Gear-box ratio, e.g. 600:1 */
    .u4PolePairs = (C_MOTOR_POLE_PAIRS - 1U),                                   /* 0x02.[15:12]: Number of pole-pairs + 1 (1-16) */
#if defined (C_MOTOR_CONST_MV_PER_RPS)                                          /* MMP240726-2 */
    .u8MotorConstant = C_MOTOR_CONST_MV_PER_RPS,                                /* 0x04: Motor Constant [mV/RPS] */
#else  /* defined (C_MOTOR_CONST_MV_PER_RPS) */
    .u8MotorConstant = C_MOTOR_CONST_10MV_PER_RPS,                              /* 0x04: Motor Constant [10mV/RPS] */
#endif /* defined (C_MOTOR_CONST_MV_PER_RPS) */
#if (_SUPPORT_COIL_UNIT_10mR != FALSE)
    .u8MotorCoilRtot = C_TOT_COILS_R,                                           /* 0x05: Motor coil resistance (total, between phases) */
#elif (_SUPPORT_COIL_UNIT_100mR != FALSE)
    .u8MotorCoilRtot = ((C_TOT_COILS_R + 5U) / 10U),                            /* 0x05: Motor coil resistance (total, between phases) */
#else
    .u8MotorCoilRtot = ((C_TOT_COILS_R + 50U) / 100U),                          /* 0x05: Motor coil resistance (total, between phases) */
#endif
    .u13MinSpeed = C_SPEED_MIN,                                                 /* 0x06.[12: 0]: Minimum speed */
    .u3MicroSteps = C_MOTOR_MICROSTEPS,                                         /* 0x06.[15:13]: Number of micro-steps: 2^n (or 1 << n); 0 = Full-steps */
    .u16Speed_1 = C_SPEED_0,                                                    /* 0x08: Speed_1 */
    .u16Speed_2 = C_SPEED_1,                                                    /* 0x0A: Speed_2 */
    .u16Speed_3 = C_SPEED_2,                                                    /* 0x0C: Speed_3 */
#if (LINPROT != LIN2X_AIRVENT12)
    .u16Speed_4 = C_SPEED_3,                                                    /* 0x0E: Speed_4 */
#else  /* (LINPROT != LIN2X_AIRVENT12) */
    .u16TorqueSpeed = C_TORQUE_SPEED,                                           /* 0x0E: Torque-mode Speed */
#endif /* (LINPROT != LIN2X_AIRVENT12) */
    .u16AccelerationConst = C_ACCELERATION_CONST,                               /* 0x10: Acceleration-constant */
    .u3AccelerationSteps = C_ACCELERATION_STEPS,                                /* 0x12.[2:0]: Acceleration-(u)Steps */
    .u3DecelerationSteps = C_DECELERATION_STEPS,                                /* 0x12.[5:3]: Deceleration-(u)Steps */
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__)
    .u2MotorCurrentMultiplier = C_NV_CURR_DIV,                                  /* 0x12.[7:6]: Motor current multiplier: 1, 2, 4 or 8 */
#elif defined (__MLX81160__) || defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
    .u2MotorCurrentMultiplier = (C_NV_CURR_DIV - C_MULTIPLIER_OFF),             /* 0x12.[7:6]: Motor current multiplier: 8, 16, 32 or 64 */
#endif
#if (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) || (_SUPPORT_APP_TYPE == C_APP_RELAY) || (_SUPPORT_APP_TYPE == C_APP_SOLENOID)
#if (_APP_ZWICKAU != FALSE)
    .u8HoldingTorqueCurrent = 0U,                                               /* 0x13: Holding Torque current threshold */
#else  /* (_APP_ZWICKAU != FALSE) */
#if defined (__MLX81160__) || defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__)
        .u8HoldingTorqueCurrent = C_PID_HOLDING_CURR_LEVEL,                     /* 0x13: Minimum Running Torque current threshold (MMP240730-1) */
#elif defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
        .u8HoldingTorqueCurrent = (C_PID_HOLDING_CURR_LEVEL >> C_NV_CURR_DIV),  /* 0x13: Holding Torque current threshold (MMP240730-1) */
#endif
#endif /* (_APP_ZWICKAU != FALSE) */
    .u8RunningTorqueCurrent = (C_PID_RUNNING_CURR_LEVEL >> C_NV_CURR_DIV),      /* 0x14: Running Torque current threshold */
#else  /* (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) */
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__)
    .u8HoldingTorqueCurrent = C_PID_RUNNING_CURR_MIN,                           /* 0x13: Minimum Running Torque current threshold */
#elif defined (__MLX81160__) || defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
    .u8HoldingTorqueCurrent = (C_PID_RUNNING_CURR_MIN >> C_NV_CURR_DIV),        /* 0x13: Holding Torque current threshold */
#endif
    .u8RunningTorqueCurrent = (C_PID_RUNNING_CURR_MAX >> C_NV_CURR_DIV),        /* 0x14: maximum Running Torque current threshold */
#endif /* (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) */
    .u4TorqueBoost1 = 0U,                                                       /* 0x15.[3:0]: TorqueBoost1 (5%) -25%..50% */
    .u4TorqueBoost2 = 5U,                                                       /* 0x15.[7:4]: TorqueBoost2 (5%)   0%..75% */
    .u8PidCoefP = C_PID_COEF_P,                                                 /* 0x16: PID-Coefficient P */
    .u8PidCoefI = C_PID_COEF_I,                                                 /* 0x17: PID-Coefficient I */
    .u8PidCoefD = C_PID_COEF_D,                                                 /* 0x18: PID-Coefficient D */
#if (_SUPPORT_MOTOR_POSITION == C_MOTOR_POS_NONE) && (_SUPPORT_APP_TYPE != C_APP_RELAY) && (_SUPPORT_APP_TYPE != C_APP_SOLENOID)
    .u8PidStartupOrLowerHoldingLimit = (C_PID_STARTING_CURR_MAX >> C_NV_CURR_DIV),  /* 0x19: Startup current limit (MMP190809-1) */
#else  /* (_SUPPORT_MOTOR_POSITION == C_MOTOR_POS_NONE) */
    .u8PidStartupOrLowerHoldingLimit = C_MIN_HOLDCORR_RATIO,                    /* 0x19: PID Lower-limit Holding (output) */
#endif /* (_SUPPORT_MOTOR_POSITION == C_MOTOR_POS_NONE) */
    .u8PidLowerLimit = C_MIN_CORR_RATIO,                                        /* 0x1A: PID Lower-limit Running (output) */
    .u8PidUpperLimit = C_MAX_CORR_RATIO,                                        /* 0x1B: PID Upper-limit (output) */
    .u7PidCtrlPeriod = C_PID_RUNNINGCTRL_PERIOD,                                /* 0x1C.[6:0]: PID running control-period */
    .u1PidPeriodTimeOrSpeed = C_PID_RUNNINGCTRL_PERIOD_UNIT,                    /* 0x1C.[7]: PID Period is Time(0) or Speed(1) based */
    .u8AppOT = C_APP_OT_LEVEL + 60U,                                            /* 0x1D: Application Over-Temperature [C] */
    .u8AppUV = C_APP_UV_LEVEL,                                                  /* 0x1E: Application Under-voltage level [1/8V] */
#if (_APP_SUPPLY_RANGE != C_APP_SUPPLY_RANGE_48V)
    .u8AppOV = (C_APP_OV_LEVEL - (uint8_t)(C_APP_OV_OFF / 12.5)),               /* 0x1F: Application Over-voltage level [1/8V] */
#else  /* (_SUPPORT_APP_48V_ADC == FALSE) */
    .u8AppOV = (C_APP_OV_LEVEL - (uint8_t)(C_APP_OV_OFF / 25)),                 /* 0x1F: Application Over-voltage level [1/4V] */
#endif /* (_SUPPORT_APP_48V_ADC == FALSE) */
#if defined (__MLX81339__)
    .u16Inductance = C_TOT_COILS_L,                                             /* 0x20: Coil Inductance [uH] */
    .u8RampUpSpeedLimit = C_PID_RAMP_UP,                                        /* 0x22: Ramp-up Speed Limit */
    .u8RampDownSpeedLimit = C_PID_RAMP_DOWN,                                    /* 0x23: Ramp-down Speed Limit */
    .u3ChipUV = C_IC_UV_LEVEL,                                                  /* 0x24.[2:0]: Chip Under Voltage */
    .u2ChipOV = C_IC_OV_LEVEL,                                                  /* 0x24.[4:3]: Chip Over Voltage */
    .u3Reserved = 0U,                                                           /* 0x24.[7:5]: Reserved */
    .u8AlignmentSpeed = C_SPEED_ALIGNMENT,                                      /* 0x25: Alignment Speed [RPM] */
    .u16Reserved = 0U                                                           /* 0x26: Reserved */
#endif /* defined (__MLX81339__) */
};

#if (_SUPPORT_TRIAXIS_MLX90363 != FALSE) || (_SUPPORT_TRIAXIS_MLX9038x != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90367 != FALSE) || (_SUPPORT_TRIAXIS_MLX90372 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90377 != FALSE) || (_SUPPORT_TRIAXIS_MLX90395 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90421 != FALSE) || (_SUPPORT_TRIAXIS_MLX90422 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90425 != FALSE) || (_SUPPORT_TRIAXIS_MLX90426 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90427 != FALSE) || \
    (_SUPPORT_INDUCTIVE_POS_SENSOR_MLX90513 != FALSE) || \
    (_SUPPORT_DUAL_HALL_LATCH_MLX92251 != FALSE) || (_SUPPORT_DUAL_HALL_LATCH_MLX92255 != FALSE) || \
    (_SUPPORT_FOC_MODE != FOC_MODE_NONE)
/*! Default for sensor/FOC */
CONST SENSOR_t DefSensor = C_NV_DEF_SENSOR_PARAMS;
#endif /* (_SUPPORT_TRIAXIS_MLX90363 != FALSE) || (_SUPPORT_TRIAXIS_MLX9038x != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90367 != FALSE) || (_SUPPORT_TRIAXIS_MLX90372 != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90377 != FALSE) || (_SUPPORT_TRIAXIS_MLX90395 != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90421 != FALSE) || (_SUPPORT_TRIAXIS_MLX90422 != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90425 != FALSE) || (_SUPPORT_TRIAXIS_MLX90426 != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90427 != FALSE) || \
        * (_SUPPORT_INDUCTIVE_POS_SENSOR_MLX90513 != FALSE) || \
        * (_SUPPORT_DUAL_HALL_LATCH_MLX92251 != FALSE) || (_SUPPORT_DUAL_HALL_LATCH_MLX92255 != FALSE) || \
        * (_SUPPORT_FOC_MODE != FOC_MODE_NONE) */

/*! Default Stall-detector data; Updated: MMP190412-1 */
CONST ACT_STALL_t DefActStall = C_NV_DEF_ACT_STALL;

#if (_SUPPORT_APP_SAVE != FALSE)
/*! Default data for APP_PARAMS_t */
const APP_PARAMS_t DefAppParams = C_NV_DEF_APP_PARAMS;
#endif /* (_SUPPORT_APP_SAVE != FALSE) */

#if (I2C_COMM != FALSE)
/*! Default data for User NVM STD_I2C_PARAMS */
const STD_I2C_PARAMS_t DefI2cParams = C_NV_DEF_I2C_PARAMS;
#endif /* (I2C_COMM != FALSE) */

#if (CAN_COMM != FALSE)
/*! Default data for User NVM STD_CAN_PARAMS */
const STD_CAN_PARAMS_t DefCanParams = C_NV_DEF_CAN_PARAMS;
#endif /* (CAN_COMM != FALSE) */

/*!*************************************************************************** */
/*                           GLOBAL & LOCAL VARIABLES                          */
/* *************************************************************************** */
#pragma space dp                                                                /* __TINY_SECTION__ */
#pragma space none                                                              /* __TINY_SECTION__ */

#pragma space nodp                                                              /* __NEAR_SECTION__ */
#if (_SUPPORT_CPFREQ_TEMPCOMP != FALSE)
uint16_t g_u16CP_FreqTrim_RT;                                                   /*!< Charge-pump clock frequency trim value at RT (Only: MLX8133xBB) */
#endif /* (_SUPPORT_CPFREQ_TEMPCOMP != FALSE) */
#if (_SUPPORT_NV_TYPE == C_NV_FLASH)
FLASH_KF_WriteStateMachine_t l_Flash_KF_StateMachine;                           /*!< Flash Write-state machine structure */
#endif /* (_SUPPORT_NV_TYPE == C_NV_FLASH) */
#pragma space none                                                              /* __NEAR_SECTION__ */

#if (_SUPPORT_NV_TYPE == C_NV_FLASH)
/*!*************************************************************************** *
 * NV_Init()
 * \brief   Initialise the NV module
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details
 * *************************************************************************** *
 * - Call Hierarchy:
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 0
 * *************************************************************************** */
void NV_Init(void)
{
    FLASH_KF_Init(&l_Flash_KF_StateMachine);
} /* End of NV_Init() */
#endif /* (_SUPPORT_NV_TYPE == C_NV_FLASH) */

/*!*************************************************************************** *
 * NV_CalcCRC()
 * \brief   Calculate CRC
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] pu16BeginAddress: Begin-address of area
 * \param   [in] u16Length: LengthW of area (in 16-bits words)
 * \return  (uint16_t) CRC8 result
 * *************************************************************************** *
 * \details Use own CRC routine instead of ROM nvram_CalcCRC, as linker
 *          fails to link correctly
 * *************************************************************************** *
 * - Call Hierarchy: NV_CheckCRC(), NV_MlxCalib(), NV_WriteActParams(), NV_WriteActStall(),
 *                   NV_WriteAPP(), NV_WriteEOL(), NV_WriteLIN_STD(), NV_WriteSensor(),
 *                   NV_WriteUserDefaults(), NV_AppStore()
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 0
 * *************************************************************************** */
static uint16_t NV_CalcCRC(const uint16_t *pu16BeginAddress, const uint16_t u16LengthW)
{
    uint16_t u16Result = p_CalcCRC_U8(pu16BeginAddress, u16LengthW);
    return (u16Result);
} /* End of NV_CalcCRC() */

/*!*************************************************************************** *
 * p_CopyU16
 * \brief   Move 16-bit data area
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16SizeW: size of area in 16-bit
 * \param   [in] *pu16Dest: Address of destination area
 * \param   [in] *pu16Src: Address of source/origin area
 * \return  -
 * *************************************************************************** *
 * \details Destination and Source buffer must be 16-bit aligned
 * *************************************************************************** *
 * - Call Hierarchy: DfrDiagAssignNAD(), DfrDiagReassignNAD(), DfrDiagSaveConfig(),
 *                   NV_WriteActParams(), NV_WriteActStall(), NV_WriteAPP(),
 *                   NV_WriteEOL(), NV_WriteLIN_STD(), NV_WriteSensor(),
 *                   NV_WriteUserDefaults(), NV_AppStore(), ErrorLogInit(),
 *                   HandleFanInit(), main_Init()
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 0
 * *************************************************************************** */
void p_CopyU16(const uint16_t u16SizeW, uint16_t *pu16Dest, const uint16_t *pu16Src)
{
    uint16_t *pu16Dest_Waste;
    uint16_t *pu16Src_Waste;
    uint16_t u16Size_Waste;

#if defined (__MLX81339__)
    __asm__ /*__volatile__*/
#else  /* defined (__MLX81339__) */
    __asm__ __volatile__
#endif /* defined (__MLX81339__) */
    (
#if (_SUPPORT_BUGFIX_DCC_1293 != FALSE)
        "mov c, ml.7\n\t"                                                       /*  (MMP190110-1) Bugfix: JIRA DCC-1293 */
#endif /* (_SUPPORT_BUGFIX_DCC_1293 != FALSE) */
        "movsw [X++], [Y++]\n\t"
        "add A, #0xFFFF\n\t"
        "jnz .-4"
        : "=x" (pu16Dest_Waste), "=y" (pu16Src_Waste), "=a" (u16Size_Waste)
        : "x" (pu16Dest), "y" (pu16Src), "a" (u16SizeW)
        :
    );
} /* End of p_CopyU16() */

#if (_SUPPORT_NV_TYPE == C_NV_EEPROM)
/*!*************************************************************************** *
 * p_CopyU64
 * \brief   Move 64-bit data area
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] *pu16Dest: Address of destination area
 * \param   [in] *pu16Src: Address of source/origin area
 * \return  -
 * *************************************************************************** *
 * \details Destination and Source buffer must be 16-bit aligned
 * *************************************************************************** *
 * - Call Hierarchy: p_NV_WriteWord64()
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 0
 * *************************************************************************** */
static void p_CopyU64(uint16_t *pu16Dest, const uint16_t *pu16Src)
{
    uint16_t *pu16Dest_Waste;
    uint16_t *pu16Src_Waste;
    __asm__ __volatile__
    (
#if (_SUPPORT_BUGFIX_DCC_1293 != FALSE)
        "mov c, ml.7\n\t"                                                       /*  (MMP190110-1) Bugfix: JIRA DCC-1293 */
#endif /* (_SUPPORT_BUGFIX_DCC_1293 != FALSE) */
        "movsw [X++], [Y++]\n\t"
        "movsw [X++], [Y++]\n\t"
        "movsw [X++], [Y++]\n\t"
        "movsw [X++], [Y++]"
        : "=x" (pu16Dest_Waste), "=y" (pu16Src_Waste)
        : "x" (pu16Dest), "y" (pu16Src)
        :
    );
} /* End of p_CopyU64() */

/*!*************************************************************************** *
 * p_CompareU64
 * \brief   Verify (binary compare) Non Volatile Memory block's with RAM block's
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] *pu16Dest: Address of destination area
 * \param   [in] *pu16Src: Address of source/origin area
 * \return  uint16_t u16Result: FALSE = Not equal
 *                              TRUE  = Equal
 * *************************************************************************** *
 * \details Binary compare between Non Volatile Memory and RAM buffer
 * *************************************************************************** *
 * - Call Hierarchy: NV_WriteBlock()
 * - Cyclomatic Complexity: 2+1
 * - Nesting: 2
 * - Function calling: 0
 * *************************************************************************** */
static uint16_t p_CompareU64(const uint16_t *pu16Dest, const uint16_t *pu16Src)
{
    uint16_t u16Result = FALSE;
    if ( (*pu16Dest++ == *pu16Src++) && (*pu16Dest++ == *pu16Src++) &&
         (*pu16Dest++ == *pu16Src++) && (*pu16Dest++ == *pu16Src++) )
    {
        /* Equal */
        u16Result = TRUE;
    }
    return (u16Result);
} /* End of p_CompareU64() */

/*!*************************************************************************** *
 * p_NV_BusyChecks
 * \brief   Check Non Volatile Memory Busy
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details
 * *************************************************************************** *
 * - Call Hierarchy: p_NV_WriteWord64()
 * - Cyclomatic Complexity: 2+1
 * - Nesting: 2
 * - Function calling: 0
 * *************************************************************************** */
static void p_NV_BusyChecks(void)
{
    if ( (IO_EEPROM_FLASH_EE_CTRL_S & B_EEPROM_FLASH_EE_BUSY_STBY) != 0u)
    {
        IO_EEPROM_FLASH_EE_CTRL_S |= B_EEPROM_FLASH_EE_ACTIVE;                    /* IO_SET(EEPROM_FLASH, NV_ACTIVE, 1) */
    }
    else /* TODO: check if we need to wait here until the Non Volatile Memory will be woken up */
    if ( (IO_EEPROM_FLASH_EE_CTRL_S & B_EEPROM_FLASH_EE_BUSY_BUF_NOT_EMPTY) != 0u) /* Erase the buffer if it's not empty */
    {
        IO_EEPROM_FLASH_EE_CTRL_S = (IO_EEPROM_FLASH_EE_CTRL_S & ~M_EEPROM_FLASH_EE_WE_KEY) | 0U; /* IO_SET(EEPROM_FLASH, NV_WE_KEY, 0u) */
    }
    else
    {
        /* Then this is Write-operation. We need just to wait until it ends. */
    }
} /* End of p_NV_BusyChecks() */

#if (_DEBUG_NV_WRITE_BACKGROUND == FALSE)
/*!*************************************************************************** *
 * p_NV_WriteWord64_blocking
 * \brief   Check Non Volatile Memory Busy
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16Address: Begin-address
 * \param   [in] *pu64Data: Pointer to 64-bit data-buffer
 * \param   [in] u16WriteEnaKey
 * \return  -
 * *************************************************************************** *
 * \details Writes 64bits of data into the Non Volatile Memory
 * NOTE: This function needed to be called in SYSTEM-mode
 * *************************************************************************** *
 * - Call Hierarchy: p_NV_WriteBlock()
 * - Cyclomatic Complexity: 2+1
 * - Nesting: 1
 * - Function calling: 2 (p_NV_BusyChecks(), p_CopyU64())
 * *************************************************************************** */
static void p_NV_WriteWord64_blocking(const uint16_t u16Address, uint16_t *pu64Data, const uint16_t u16WriteEnaKey)
{
    /* TODO: Need to mask the interrupt. */
#if (_SUPPORT_WHILE_RETRY_LIMITED != FALSE)
    uint16_t u16Retries = 100U;
    while (IO_GET(EEPROM_FLASH, EE_BUSY)!= 0u)
    {
        p_EEPROM_BusyChecks();
        if (--u16Retries == 0U)
        {
#if (_SUPPORT_LOG_ERRORS != FALSE)
            SetLastError(C_ERR_TO_NV_BUSY);
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
            break;
        }
        DELAY_US(150U);
    }
#else  /* (_SUPPORT_WHILE_RETRY_LIMITED != FALSE) */
    while (IO_GET(EEPROM_FLASH, EE_BUSY)!= 0u)
    {
        p_NV_BusyChecks();
    }
#endif /* (_SUPPORT_WHILE_RETRY_LIMITED != FALSE) */

    IO_EEPROM_FLASH_EE_DATA_ERROR = (IO_EEPROM_FLASH_EE_DATA_ERROR |
                                     (B_EEPROM_FLASH_EE_DATA_CORRUPTED_2 |
                                      B_EEPROM_FLASH_EE_SBE_2 |
                                      B_EEPROM_FLASH_EE_DATA_CORRUPTED_1 |
                                      B_EEPROM_FLASH_EE_SBE_1));
    IO_EEPROM_FLASH_EE_CTRL_S = (IO_EEPROM_FLASH_EE_CTRL_S & ~(M_EEPROM_FLASH_EE_W_MODE | M_EEPROM_FLASH_EE_WE_KEY)) |
                                (u16WriteEnaKey << 4);
    p_CopyU64( (uint16_t *)u16Address, (const uint16_t *)pu64Data);
#if (_SUPPORT_WHILE_RETRY_LIMITED != FALSE)
    u16Retries = 100U;
    while (IO_GET(EEPROM_FLASH, EE_BUSY_WR) != 0u)
    {
        /* Wait till the write operation will be finished. */
        if (--u16Retries == 0U)
        {
#if (_SUPPORT_LOG_ERRORS != FALSE)
            SetLastError(C_ERR_TO_NV_WR_BUSY);
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
            break;
        }
        DELAY_US(150U);
    }
#else  /* (_SUPPORT_WHILE_RETRY_LIMITED != FALSE) */
    while (IO_GET(EEPROM_FLASH, EE_BUSY_WR) != 0u)
    {
        /* Wait till the write operation will be finished. */
    }
#endif /* (_SUPPORT_WHILE_RETRY_LIMITED != FALSE) */
} /* End of p_NV_WriteWord64_blocking() */
#endif /* (_DEBUG_NV_WRITE_BACKGROUND == FALSE) */

/*!*************************************************************************** *
 * NV_WriteBlock
 * \brief   Write to user area Non Volatile Memory block(64-bits)
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16Address: Non Volatile Memory Address (64-bit block aligned)
 * \param   [in] *pu64Data: Pointer to RAM buffer of 64-bit data
 * \param   [in] u16SizeW: Size of buffer in 16-bit Words
 * \param   [in] u16Force: Force Non Volatile Memory (don't check)
 * \return  uint16_t: Non Volatile Memory update result
 *                    C_ERR_NONE: No errors
 *                    C_ERR_NVWRITE: Non Volatile Memory Write failure
 * *************************************************************************** *
 * \details Write one NV_Block of 64-bits
 * *************************************************************************** *
 * - Call Hierarchy: DfrDiagMlxDebug(), NV_WriteActParams(), NV_WriteActStall(),
 *                   NV_WriteAPP(), NV_WriteEOL(), NV_WriteLIN_STD(),
 *                   NV_WriteSensor(), NV_WriteUserDefaults(), NV_AppStore()
 *
 * - Cyclomatic Complexity: 1+1
 * - Nesting: 1
 * - Function calling: 3 (p_CompareU64(), p_NV_WriteWord64_blocking(),
 *                        p_AwdAck())
 * *************************************************************************** */
uint16_t NV_WriteBlock(const uint16_t u16Address64, uint16_t *pu16Data, uint16_t u16SizeW, uint16_t u16Force)
{
    uint16_t u16Result = C_ERR_NONE;
    uint16_t *pBlock = (uint16_t *)pu16Data;
    uint16_t u16Address = u16Address64;
#if (_DEBUG_NV_WRITE != FALSE)
    DEBUG_SET_IO_A();
#endif /* (_DEBUG_NV_WRITE != FALSE) */

    do
    {
        /* Prior to the block write, first check if data is same; Skip physical block write */
        if ( (u16Force != FALSE) ||
             (p_CompareU64( (uint16_t *)u16Address, (uint16_t *)pBlock) == FALSE) )  /* Verify failed */
        {
#if (_DEBUG_NV_WRITE_BACKGROUND != FALSE)
            ENTER_SECTION(SYSTEM_MODE); /*lint !e534 */
            IO_MLX16_ITC_PEND2_S = B_MLX16_ITC_PEND2_NV_COMPLETE;
            EEPROM_WriteWord64_non_blocking( (const uint16_t)0x980U, (const uint16_t *)((void *)NV_Test), C_NV_WRT_KEY);
            EXIT_SECTION(); /*lint !e438 */
            while ( (IO_MLX16_ITC_PEND2_S & B_MLX16_ITC_PEND2_NV_COMPLETE) == 0U) {}
#else  /* (_DEBUG_NV_WRITE_BACKGROUND != FALSE) */
#if (_SUPPORT_APP_USER_MODE != FALSE)
            ENTER_SECTION(ATOMIC_SYSTEM_MODE); /*lint !e534 */  /* MMP200624-1 */
#else  /* (_SUPPORT_APP_USER_MODE != FALSE) */
            ENTER_SECTION(ATOMIC_KEEP_MODE); /*lint !e534 */  /* MMP200624-1 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
            if (ADC_GetNewSampleVsupply() > C_MIN_VS_EEWRT)                         /* Minimum Non Volatile Memory Write voltage */
            {
                p_NV_WriteWord64_blocking(u16Address, pBlock, C_NV_WRT_KEY);
            }

            p_AwdAck(); /*lint !e522 */                                             /* Acknowledge Analogue Watchdog .. (MMP200625-2) */
#if (_SUPPORT_DWD != FALSE)
            WDG_conditionalIwdRefresh(C_IWD_DIV, C_IWD_TO);                         /* .. acknowledge the digital watchdog */
#endif /* (_SUPPORT_DWD != FALSE) */
            EXIT_SECTION(); /*lint !e438 */
#endif /* (_DEBUG_NV_WRITE_BACKGROUND != FALSE) */
            if (p_CompareU64( (uint16_t *)u16Address, (uint16_t *)pBlock) == FALSE)
            {
                u16Result = C_ERR_NVWRITE;
            }
        }
        u16Address += SZ_NV_BLOCK;
        pBlock += (SZ_NV_BLOCK / sizeof(uint16_t));
    } while ( (pBlock < (uint16_t *)pu16Data + u16SizeW) && (u16Result == C_ERR_NONE) );
#if (_DEBUG_NV_WRITE != FALSE)
    DEBUG_CLR_IO_A();
#endif /* (_DEBUG_NV_WRITE != FALSE) */
    return (u16Result);
} /* End of NV_WriteBlock() */

#elif (_SUPPORT_NV_TYPE == C_NV_FLASH)

/*!*************************************************************************** *
 * p_CompareSz
 * \brief   Verify (binary compare) USER-PAGE block's with RAM block's
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] *pu16Dest: Address of destination area
 * \param   [in] *pu16Src: Address of source/origin area
 * \param   [in] u16SizeW: Size of area in 16-bit Words
 * \return  uint16_t u16Result: FALSE = Not equal
 *                              TRUE  = Equal
 * *************************************************************************** *
 * \details Binary compare between EEPROM and RAM buffer
 * *************************************************************************** *
 * - Call Hierarchy: NV_WriteBlock()
 * - Cyclomatic Complexity: 2+1
 * - Nesting: 2
 * - Function calling: 0
 * *************************************************************************** */
static uint16_t p_CompareSz(const uint16_t *pu16Dest, const uint16_t *pu16Src, uint16_t u16SizeW)
{
    uint16_t u16Result = TRUE;
    while (u16SizeW-- != 0U)
    {
        /* Not-Equal */
        if (*pu16Dest++ != *pu16Src++)
        {
            u16Result = FALSE;
            break;
        }
    }
    return (u16Result);
} /* End of p_CompareSz() */

/*!*************************************************************************** *
 * p_NV_UpdatePage_blocking
 * \brief   Update Non Volatile Memory Block
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16Address: Begin-address
 * \param   [in] *puData: Pointer to data-buffer
 * \param   [in] u16SizeW: Size of area in 16-bit Words; Maximum length 64 Words.
 * \param   [in] u16WriteEnaKey
 * \return  -
 * *************************************************************************** *
 * \details Update page into NV-Memory.
 *          In case data is aligned with a NV-block, the complete block will be
 *          re-written;
 *          In case the data is shorter or overlapping with two NV-blocks, a
 *          read-modify-write will be performed.
 * NOTE: This function needed to be called in SYSTEM-mode
 * *************************************************************************** *
 * - Call Hierarchy: NV_WriteBlock()
 * - Cyclomatic Complexity: 2+1
 * - Nesting: 2
 * - Function calling: 3 (p_CopyU16(), FLASH_KF_SetKey(), FLASH_KF_WriteIUM_Page_blocking())
 * *************************************************************************** */
static void p_NV_UpdatePage_blocking(const uint16_t u16Address, uint16_t *pu16Data, uint16_t u16SizeW, const uint16_t u16WriteEnaKey)
{
    uint16_t au16NVBlock[64U];
    uint16_t u16AddressNV;
    uint16_t u16AddressBlock;
    fl_ctrl_keys_t keys;

    if ( (u16SizeW == SZ_NV_BLOCK) && ((u16Address & 0x007F) == 0x0000) )
    {
        /* Write a complete NV-Block */
        keys.IUM_Unlock_key = u16WriteEnaKey;                                   /* This is only key required for data-flash (FLASH_CMD_EEPROM_UNLOCK_KEY) */
        keys.Erase_key = 0x0000U;                                               /* Not required for data-flash (FLASH_CMD_ERASE_KEY) */
        keys.Unlock_key = 0x0000U;                                              /* Not required for data-flash (FLASH_CMD_UNLOCK_KEY) */
        keys.Write_key = 0x0000U;                                               /* Not required for data-flash (FLASH_CMD_WRITE_KEY) */
        FLASH_KF_SetKey(&l_Flash_KF_StateMachine, (const fl_ctrl_keys_t*)&keys);
        FLASH_KF_WriteIUM_Page_blocking(&l_Flash_KF_StateMachine, (const void*)pu16Data, u16Address, FALSE);
    }
    else
    {
        /* Partial update of NV-Block */
        /* Read NV-Block */
        u16AddressNV = (u16Address & 0xFF80U);
        p_CopyU16((const uint16_t)(SZ_NV_BLOCK/2U), (uint16_t *)&au16NVBlock[0], (const uint16_t *)u16AddressNV);

        /* Update NV-Block */
        u16AddressBlock = (u16Address & 0x007EU);
        if ( (u16AddressBlock + (u16SizeW << 1)) <= SZ_NV_BLOCK)
        {
            /* Data block fits in one NV-Block; Partial overwrite */
            p_CopyU16(u16SizeW, &au16NVBlock[u16AddressBlock], pu16Data);

            /* Write NV-Block */
            keys.IUM_Unlock_key = u16WriteEnaKey;                               /* This is only key required for data-flash (FLASH_CMD_EEPROM_UNLOCK_KEY) */
            keys.Erase_key = 0x0000U;                                           /* Not required for data-flash (FLASH_CMD_ERASE_KEY) */
            keys.Unlock_key = 0x0000U;                                          /* Not required for data-flash (FLASH_CMD_UNLOCK_KEY) */
            keys.Write_key = 0x0000U;                                           /* Not required for data-flash (FLASH_CMD_WRITE_KEY) */
            FLASH_KF_SetKey(&l_Flash_KF_StateMachine, (const fl_ctrl_keys_t*)&keys);
            FLASH_KF_WriteIUM_Page_blocking(&l_Flash_KF_StateMachine, (const void*)&au16NVBlock[0], u16AddressNV, FALSE);
        }
        else
        {
            /* Split data block over multiple NV-Blocks */
            uint16_t u16SizeW_Partial = SZ_NV_BLOCK - u16AddressBlock;
            p_CopyU16(u16SizeW_Partial, &au16NVBlock[u16AddressBlock], pu16Data);

            /* Write 1st NV-Block */
            keys.IUM_Unlock_key = u16WriteEnaKey;                               /* This is only key required for data-flash (FLASH_CMD_EEPROM_UNLOCK_KEY) */
            keys.Erase_key = 0x0000U;                                           /* Not required for data-flash (FLASH_CMD_ERASE_KEY) */
            keys.Unlock_key = 0x0000U;                                          /* Not required for data-flash (FLASH_CMD_UNLOCK_KEY) */
            keys.Write_key = 0x0000U;                                           /* Not required for data-flash (FLASH_CMD_WRITE_KEY) */
            FLASH_KF_SetKey(&l_Flash_KF_StateMachine, (const fl_ctrl_keys_t*)&keys);
            FLASH_KF_WriteIUM_Page_blocking(&l_Flash_KF_StateMachine, (const void*)&au16NVBlock[0], u16AddressNV, FALSE);

            /* Read 2nd NV-Block */
            u16AddressNV += SZ_NV_BLOCK;
            p_CopyU16((const uint16_t)(SZ_NV_BLOCK/2U), (uint16_t *)&au16NVBlock[0], (const uint16_t *)u16AddressNV);

            p_CopyU16((u16SizeW - u16SizeW_Partial), &au16NVBlock[0U], &pu16Data[u16SizeW_Partial]);

            /* Write 2nd NV-Block */
            keys.IUM_Unlock_key = u16WriteEnaKey;                               /* This is only key required for data-flash (FLASH_CMD_EEPROM_UNLOCK_KEY) */
            keys.Erase_key = 0x0000U;                                           /* Not required for data-flash (FLASH_CMD_ERASE_KEY) */
            keys.Unlock_key = 0x0000U;                                          /* Not required for data-flash (FLASH_CMD_UNLOCK_KEY) */
            keys.Write_key = 0x0000U;                                           /* Not required for data-flash (FLASH_CMD_WRITE_KEY) */
            FLASH_KF_SetKey(&l_Flash_KF_StateMachine, (const fl_ctrl_keys_t*)&keys);
            FLASH_KF_WriteIUM_Page_blocking(&l_Flash_KF_StateMachine, (const void*)&au16NVBlock[0], u16AddressNV, FALSE);
        }
    }
} /* End of p_NV_UpdatePage_blocking() */

/*!*************************************************************************** *
 * NV_WriteBlock
 * \brief   Write to non-volatile area
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16Address: Non-Volatile Memory Address
 * \param   [in] *pu16Data: Pointer to RAM buffer of (multiple of) 8-byte data
 * \param   [in] u16SizeW: Size of area in 16-bit Words
 * \param   [in] u16Force: Force Non-Volatile Memory update (don't check)
 * \return  uint16_t: Non-Volatile Memory update result
 *                    C_ERR_NONE: No errors
 *                    C_ERR_NVM_WRITE: Non-Volatile Memory Write failure
 * *************************************************************************** *
 * \details Write one NV_Block of 64-bits
 * *************************************************************************** *
 * - Call Hierarchy: DfrDiagMlxDebug(), NV_WriteActParams(), NV_WriteActStall(),
 *                   NV_WriteAPP(), NV_WriteEOL(), NV_WriteLIN_STD(),
 *                   NV_WriteSensor(), NV_WriteUserDefaults(), NV_AppStore()
 *
 * - Cyclomatic Complexity: 1+1
 * - Nesting: 1
 * - Function calling: 3 (p_CompareSz(), p_NV_UpdatePage_blocking(),
 *                        p_AwdAck())
 * *************************************************************************** */
uint16_t NV_WriteBlock(const uint16_t u16Address, uint16_t *pu16Data, uint16_t u16SizeW, uint16_t u16Force)
{
    uint16_t u16Result = C_ERR_NONE;
#if (_DEBUG_NV_WRITE != FALSE)
    DEBUG_SET_IO_A();
#endif /* (_DEBUG_NV_WRITE != FALSE) */

    /* Prior to the block write, first check if data is same; Skip physical block write */
    if ( (u16Force != FALSE) ||
         (p_CompareSz( (const uint16_t *)u16Address, (const uint16_t *)pu16Data, u16SizeW) == FALSE) )    /* Verify failed */
    {
#if (_SUPPORT_APP_USER_MODE != FALSE)
        ENTER_SECTION(ATOMIC_SYSTEM_MODE); /*lint !e534 */  /* MMP200624-1 */
#else  /* (_SUPPORT_APP_USER_MODE != FALSE) */
        ENTER_SECTION(ATOMIC_KEEP_MODE); /*lint !e534 */  /* MMP200624-1 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
        if (ADC_GetNewSampleVsupply() > C_MIN_VS_EEWRT)                         /* Minimum EEPROM Write voltage */
        {
            p_NV_UpdatePage_blocking(u16Address, pu16Data, u16SizeW, FLASH_CMD_EEPROM_UNLOCK_KEY);
        }

#if FALSE
        p_AwdAck(); /*lint !e522 */                                             /* Acknowledge Analogue Watchdog .. (MMP200625-2) */
#if (_SUPPORT_DWD != FALSE)
        WDG_conditionalIwdRefresh(C_IWD_DIV, C_IWD_TO);                         /* .. acknowledge the digital watchdog */
#endif /* (_SUPPORT_DWD != FALSE) */
#endif
        EXIT_SECTION(); /*lint !e438 */
        if (p_CompareSz( (const uint16_t *)u16Address, (const uint16_t *)pu16Data, u16SizeW) == FALSE)
        {
            u16Result = C_ERR_NVWRITE;
        }
    }
#if (_DEBUG_NV_WRITE != FALSE)
    DEBUG_CLR_IO_A();
#endif /* (_DEBUG_NV_WRITE != FALSE) */
    return (u16Result);
} /* End of NV_WriteBlock() */
#endif /* (_SUPPORT_NV_TYPE == C_NV_EEPROM) */

/*!*************************************************************************** *
 * NV_BlockCheckCRC
 * \brief   Check user area Non Volatile Memory CRC's for Non Volatile Memory Structure
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  uint16_t: Result
 *                      C_ERR_NONE : No error
 *                      Others: Bit-map of which structure fails
 * *************************************************************************** *
 * \details Check of each User Non Volatile Memory structure the checksum
 * *************************************************************************** *
 * - Call Hierarchy: NV_CheckCRC()
 * - Cyclomatic Complexity: 2+1
 * - Nesting: 1
 * - Function calling: 1 (NV_CalcCRC())
 * *************************************************************************** */
static uint16_t NV_BlockCheckCRC(void)
{
    uint16_t idx;
    uint16_t u16Result = 0U;

    for (idx = 0U; idx < (sizeof(eeCrc) / sizeof(eeCrc[0])); idx++)
    {
        IO_MLX16_ITC_PEND0_S = B_MLX16_ITC_PEND0_EE_SH_ECC;
        if ( (NV_CalcCRC( (const uint16_t *)eeCrc[idx].u16Address, eeCrc[idx].u16Size) != 0xFFU) ||
             ((IO_MLX16_ITC_PEND0_S & B_MLX16_ITC_PEND0_EE_SH_ECC) != 0U) )
        {
            u16Result |= eeCrc[idx].u16ErrorCode;
        }
    }

    return (u16Result);
} /* End of NV_BlockCheckCRC() */

#if (LIN_COMM != FALSE)
/*!*************************************************************************** *
 * NV_CheckStdLin
 * \brief   Check Standard LIN Non Volatile Memory structures
 * \author  mmp
 * *************************************************************************** *
 * \param   u16Result: LIN Block CRC error bit-mask.
 * \return  uint16_t: u16Result error bit-mask
 * *************************************************************************** *
 * \details Check use of backup possible
 * *************************************************************************** *
 * - Call Hierarchy: NV_CheckCRC()
 * - Cyclomatic Complexity: 2+1
 * - Nesting: 1
 * - Function calling: 4 (p_CompareU64(), p_CopyU16(), NV_WriteLIN_STD(), SetLastError())
 * *************************************************************************** */
static uint16_t NV_CheckStdLin(uint16_t u16Result)
{
    if ( (u16Result & (C_ERR_NV_LIN_STD_1 | C_ERR_NV_LIN_STD_2)) == 0U)
    {
        /* Both Standard LIN Blocks are correct; Check binary if same */
        if (p_CompareU64( (const uint16_t *)ADDR_NV_STD_LIN_1,
                          (const uint16_t *)ADDR_NV_STD_LIN_2) == FALSE)
        {
            /* Verification failed. Assumed Block #1 is correct; Copy Block #1 to Block #2 */
            STD_LIN_PARAMS_t StdLin;

#if (_SUPPORT_LOG_ERRORS != FALSE)
            SetLastError(C_ERR_INV_USERPAGE_2 | C_ERR_EXT | 0x0300U);
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
            p_CopyU16( (sizeof(STD_LIN_PARAMS_t) / sizeof(uint16_t)),
                       (uint16_t *)&StdLin,
                       (const uint16_t *)ADDR_NV_STD_LIN_1);
            if (NV_WriteLIN_STD(&StdLin, (C_NV_WRT_FORCE | C_NV_WRT_LIN_ID_2)) != C_ERR_NONE)
            {
                /* Write failure when copying Block 1 to Block 2 */
                u16Result |= C_ERR_NV_LIN_STD_2;
            }
        }
    }
    else if ( (u16Result & (C_ERR_NV_LIN_STD_1 | C_ERR_NV_LIN_STD_2)) == C_ERR_NV_LIN_STD_1)
    {
        /* Standard LIN Block 1 is corrupted and Block 2 not; Copy Block 2 to 1 */
        STD_LIN_PARAMS_t StdLin;

#if (_SUPPORT_LOG_ERRORS != FALSE)
        SetLastError(C_ERR_INV_USERPAGE_2 | C_ERR_EXT | 0x0100U);
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
        p_CopyU16( (sizeof(STD_LIN_PARAMS_t) / sizeof(uint16_t)),
                   (uint16_t *)&StdLin,
                   (const uint16_t *)ADDR_NV_STD_LIN_2);
        if (NV_WriteLIN_STD(&StdLin, (C_NV_WRT_FORCE | C_NV_WRT_LIN_ID_1)) == C_ERR_NONE)
        {
            /* Successfully copied Block 2 to Block 1 */
            u16Result &= ~C_ERR_NV_LIN_STD_1;
        }
    }
    else if ( (u16Result & (C_ERR_NV_LIN_STD_1 | C_ERR_NV_LIN_STD_2)) == C_ERR_NV_LIN_STD_2)
    {
        /* Standard LIN Block 2 is corrupted and Block 1 not; Copy Standard LIN Block 1 to 2 */
        STD_LIN_PARAMS_t StdLin;

#if (_SUPPORT_LOG_ERRORS != FALSE)
        SetLastError(C_ERR_INV_USERPAGE_2 | C_ERR_EXT | 0x0200U);
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
        p_CopyU16( (sizeof(STD_LIN_PARAMS_t) / sizeof(uint16_t)),
                   (uint16_t *)&StdLin,
                   (const uint16_t *)ADDR_NV_STD_LIN_1);
        if (NV_WriteLIN_STD(&StdLin, (C_NV_WRT_FORCE | C_NV_WRT_LIN_ID_2)) == C_ERR_NONE)
        {
            /* Successfully copied Block 1 to Block 2 */
            u16Result &= ~C_ERR_NV_LIN_STD_2;
        }
    }
    return (u16Result);
} /* End of NV_CheckStdLin() */

#if (LINPROT == LIN2X_HVAC52)
/*!*************************************************************************** *
 * NV_CheckEnhLin
 * \brief   Check Enhanced LIN Non Volatile Memory structures
 * \author  mmp
 * *************************************************************************** *
 * \param   u16Result: LIN Block CRC error bit-mask.
 * \return  uint16_t: u16Result error bit-mask
 * *************************************************************************** *
 * \details Check use of backup possible
 * *************************************************************************** *
 * - Call Hierarchy: NV_CheckCRC()
 * - Cyclomatic Complexity: 2+1
 * - Nesting: 1
 * - Function calling: 4 (p_CompareU64(), p_CopyU16(), NV_WriteLIN_ENH(), SetLastError())
 * *************************************************************************** */
static uint16_t NV_CheckEnhLin(uint16_t u16Result)
{
    if ( (u16Result & (C_ERR_NV_LIN_ENH_1 | C_ERR_NV_LIN_ENH_2)) == 0U)
    {
        /* Both Standard LIN Blocks are correct; Check binary if same */
        if (p_CompareU64( (const uint16_t *)ADDR_NV_ENH_LIN_1,
                          (const uint16_t *)ADDR_NV_ENH_LIN_2) == FALSE)
        {
            /* Verification failed. Assumed Block #1 is correct; Copy Block #1 to Block #2 */
            ENH_LIN_PARAMS_t EnhLin;

#if (_SUPPORT_LOG_ERRORS != FALSE)
            SetLastError(C_ERR_INV_USERPAGE_2 | C_ERR_EXT | 0x0700U);
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
            p_CopyU16( (sizeof(ENH_LIN_PARAMS_t) / sizeof(uint16_t)),
                       (uint16_t *)&EnhLin,
                       (const uint16_t *)ADDR_NV_ENH_LIN_1);
            if (NV_WriteLIN_ENH(&EnhLin, (C_NV_WRT_FORCE | C_NV_WRT_LIN_ID_2)) != C_ERR_NONE)
            {
                /* Write failure when copying Block 1 to Block 2 */
                u16Result |= C_ERR_NV_LIN_ENH_2;
            }
        }
    }
    else if ( (u16Result & (C_ERR_NV_LIN_ENH_1 | C_ERR_NV_LIN_ENH_2)) == C_ERR_NV_LIN_ENH_1)
    {
        /* Standard LIN Block 1 is corrupted and Block 2 not; Copy Block 2 to 1 */
        ENH_LIN_PARAMS_t EnhLin;

#if (_SUPPORT_LOG_ERRORS != FALSE)
        SetLastError(C_ERR_INV_USERPAGE_2 | C_ERR_EXT | 0x0500U);
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
        p_CopyU16( (sizeof(ENH_LIN_PARAMS_t) / sizeof(uint16_t)),
                   (uint16_t *)&EnhLin,
                   (const uint16_t *)ADDR_NV_ENH_LIN_2);
        if (NV_WriteLIN_ENH(&EnhLin, (C_NV_WRT_FORCE | C_NV_WRT_LIN_ID_1)) == C_ERR_NONE)
        {
            /* Successfully copied Block 2 to Block 1 */
            u16Result &= ~C_ERR_NV_LIN_ENH_1;
        }
    }
    else if ( (u16Result & (C_ERR_NV_LIN_ENH_1 | C_ERR_NV_LIN_ENH_2)) == C_ERR_NV_LIN_ENH_2)
    {
        /* Standard LIN Block 2 is corrupted and Block 1 not; Copy Standard LIN Block 1 to 2 */
        ENH_LIN_PARAMS_t EnhLin;

#if (_SUPPORT_LOG_ERRORS != FALSE)
        SetLastError(C_ERR_INV_USERPAGE_2 | C_ERR_EXT | 0x0600U);
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
        p_CopyU16( (sizeof(ENH_LIN_PARAMS_t) / sizeof(uint16_t)),
                   (uint16_t *)&EnhLin,
                   (const uint16_t *)ADDR_NV_ENH_LIN_1);
        if (NV_WriteLIN_ENH(&EnhLin, (C_NV_WRT_FORCE | C_NV_WRT_LIN_ID_2)) == C_ERR_NONE)
        {
            /* Successfully copied Block 1 to Block 2 */
            u16Result &= ~C_ERR_NV_LIN_ENH_2;
        }
    }
    return (u16Result);
} /* End of NV_CheckEnhLin() */
#endif /* (LINPROT == LIN2X_HVAC52) */
#endif /* (LIN_COMM != FALSE) */

/*!*************************************************************************** *
 * NV_CheckCRC
 * \brief   Check user area Non Volatile Memory CRC's
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  uint16_t: Result
 *                      C_ERR_NONE : No error
 *                (LSB) C_ERR_NV_PG11: User Non Volatile Memory structure error
 *                (MSB) Bit-map of which structure fails
 * *************************************************************************** *
 * \details Check of each User Non Volatile Memory structure the checksum
 *          MMP220203-1: Reduce code size by ~390 bytes.
 * *************************************************************************** *
 * - Call Hierarchy: main_Init()
 * - Cyclomatic Complexity: 10+1
 * - Nesting: 5
 * - Function calling: 4 (NV_CalcCRC(), NV_BlockCheckCRC(),
 *                        NV_CheckStdLin(), NV_CheckEnhLin())
 * *************************************************************************** */
uint16_t NV_CheckCRC(void)
{
    uint16_t u16Result = C_ERR_NONE;
    NV_USER_MAP_t *pNV_User = (NV_USER_MAP_t *)ADDR_NV_USER;

#if (_SUPPORT_APP_USER_MODE != FALSE)
    ENTER_SECTION(SYSTEM_MODE); /*lint !e534 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
    /* First check Non Volatile Memory Header structure */
    IO_MLX16_ITC_PEND0_S = B_MLX16_ITC_PEND0_EE_SH_ECC;
    if ( (NV_CalcCRC( (const uint16_t *)ADDR_NV_HDR, (sizeof(HEADER_t) / sizeof(uint16_t))) != 0xFFU) || /* Non Volatile Memory CRC-Error */
         ((IO_MLX16_ITC_PEND0_S & B_MLX16_ITC_PEND0_EE_SH_ECC) != 0U) ||        /* Non Volatile Memory Double bit-error */
         (pNV_User->hdr.u8Revision != C_NV_REV) )                               /* Non Volatile Memory Structure Revision error */
    {
        /* Non Volatile Memory Header structure is corrupted; Assume other Non Volatile Memory structure invalid too. */
        u16Result = (C_ERR_NV_HDR_RST_COUNT                                     /* Header block is corrupt */
#if (LIN_COMM != FALSE)
                     | (C_ERR_NV_LIN_STD_1 | C_ERR_NV_LIN_STD_2)                /* Standard LIN Block Error */
#if (LINPROT == LIN2X_HVAC52)
                     | (C_ERR_NV_LIN_ENH_1 | C_ERR_NV_LIN_ENH_2)                /* Enhanced LIN Block Error */
#endif /* (LINPROT == LIN2X_HVAC52) */
#if (_SUPPORT_UDS != FALSE)
                     | C_ERR_NV_LIN_UDS                                         /* UDS Block Error */
#endif /* (_SUPPORT_UDS != FALSE) */
#endif /* (LIN_COMM != FALSE) */
                     | C_ERR_NV_EOL                                             /* EOL Block Error */
#if (_SUPPORT_APP_SAVE != FALSE) && (_DEBUG_FLASH_WRITE_CYCLES == FALSE)
                     | C_ERR_NV_APP_PARAMS                                      /* Application Storage */
#endif /* (_SUPPORT_APP_SAVE != FALSE) && (_DEBUG_FLASH_WRITE_CYCLES == FALSE) */
                     | C_ERR_NV_ACT_PARAMS
#if (_SUPPORT_TRIAXIS_MLX90363 != FALSE) || (_SUPPORT_TRIAXIS_MLX9038x != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90367 != FALSE) || (_SUPPORT_TRIAXIS_MLX90372 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90377 != FALSE) || (_SUPPORT_TRIAXIS_MLX90395 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90421 != FALSE) || (_SUPPORT_TRIAXIS_MLX90422 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90425 != FALSE) || (_SUPPORT_TRIAXIS_MLX90426 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90427 != FALSE) || \
    (_SUPPORT_INDUCTIVE_POS_SENSOR_MLX90513 != FALSE) || \
    (_SUPPORT_DUAL_HALL_LATCH_MLX92251 != FALSE) || (_SUPPORT_DUAL_HALL_LATCH_MLX92255 != FALSE) || \
    (_SUPPORT_FOC_MODE != FOC_MODE_NONE)
                     | C_ERR_NV_SENSOR
#endif /* (_SUPPORT_TRIAXIS_MLX90363 != FALSE) || (_SUPPORT_TRIAXIS_MLX9038x != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90367 != FALSE) || (_SUPPORT_TRIAXIS_MLX90372 != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90377 != FALSE) || (_SUPPORT_TRIAXIS_MLX90395 != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90421 != FALSE) || (_SUPPORT_TRIAXIS_MLX90422 != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90425 != FALSE) || (_SUPPORT_TRIAXIS_MLX90426 != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90427 != FALSE) || \
        * (_SUPPORT_INDUCTIVE_POS_SENSOR_MLX90513 != FALSE) || \
        * (_SUPPORT_DUAL_HALL_LATCH_MLX92251 != FALSE) || (_SUPPORT_DUAL_HALL_LATCH_MLX92255 != FALSE) || \
        * (_SUPPORT_FOC_MODE != FOC_MODE_NONE) */
                     | C_ERR_NV_ACT_STALL
#if (I2C_COMM != FALSE)
                     | C_ERR_NV_I2C_PARAMS
#endif /* (I2C_COMM != FALSE) */
#if (CAN_COMM != FALSE)
                     | C_ERR_NV_CAN_PARAMS
#endif /* (CAN_COMM != FALSE) */
                     );
    }
#if (_APP_ZWICKAU != FALSE)
    else if ( (pNV_User->hdr.u16ConfigurationID != ((('M' - '@') << 10) | (('T' - '@') << 5) | ('Z' - '@'))) ||  /* Non Volatile Memory Configuration ID error */
              (pNV_User->hdr.u12StructIDs != C_HDR_STRUCT_ID) )                 /* Change in Non Volatile Memory layout (MMP190904-1) */
#else  /* (_APP_ZWICKAU != FALSE) */
    else if ( (pNV_User->hdr.u16ConfigurationID != CONFIGURATION_ID) ||         /* Non Volatile Memory Configuration ID error */
              (pNV_User->hdr.u12StructIDs != C_HDR_STRUCT_ID) )                 /* Change in Non Volatile Memory layout (MMP190904-1) */
#endif /* (_APP_ZWICKAU != FALSE) */
    {
        /* Non Volatile Memory Header structure is valid, however the configuration ID doesn't
         * match or the following expected Non Volatile Memory structures are not as expected;
         * Assume other Non Volatile Memory structure invalid too. */
        u16Result = (C_ERR_NV_HDR_KEEP_COUNT                                    /* Header block is corrupt */
#if (LIN_COMM != FALSE)
                     | (C_ERR_NV_LIN_STD_1 | C_ERR_NV_LIN_STD_2)                /* Standard LIN Block Error */
#if (LINPROT == LIN2X_HVAC52)
                     | (C_ERR_NV_LIN_ENH_1 | C_ERR_NV_LIN_ENH_2)                /* Enhanced LIN Block Error */
#endif /* (LINPROT == LIN2X_HVAC52) */
#if (_SUPPORT_UDS != FALSE)
                     | C_ERR_NV_LIN_UDS                                         /* UDS Block Error */
#endif /* (_SUPPORT_UDS != FALSE) */
#endif /* (LIN_COMM != FALSE) */
                     | C_ERR_NV_EOL                                             /* EOL Block Error */
#if (_SUPPORT_APP_SAVE != FALSE) && (_DEBUG_FLASH_WRITE_CYCLES == FALSE)
                     | C_ERR_NV_APP_PARAMS                                      /* Application Storage */
#endif /* (_SUPPORT_APP_SAVE != FALSE) && (_DEBUG_FLASH_WRITE_CYCLES == FALSE) */
                     | C_ERR_NV_ACT_PARAMS
#if (_SUPPORT_TRIAXIS_MLX90363 != FALSE) || (_SUPPORT_TRIAXIS_MLX9038x != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90367 != FALSE) || (_SUPPORT_TRIAXIS_MLX90372 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90377 != FALSE) || (_SUPPORT_TRIAXIS_MLX90395 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90421 != FALSE) || (_SUPPORT_TRIAXIS_MLX90422 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90425 != FALSE) || (_SUPPORT_TRIAXIS_MLX90426 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90427 != FALSE) || \
    (_SUPPORT_INDUCTIVE_POS_SENSOR_MLX90513 != FALSE) || \
    (_SUPPORT_DUAL_HALL_LATCH_MLX92251 != FALSE) || (_SUPPORT_DUAL_HALL_LATCH_MLX92255 != FALSE) || \
    (_SUPPORT_FOC_MODE != FOC_MODE_NONE)
                     | C_ERR_NV_SENSOR
#endif /* (_SUPPORT_TRIAXIS_MLX90363 != FALSE) || (_SUPPORT_TRIAXIS_MLX9038x != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90367 != FALSE) || (_SUPPORT_TRIAXIS_MLX90372 != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90377 != FALSE) || (_SUPPORT_TRIAXIS_MLX90395 != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90421 != FALSE) || (_SUPPORT_TRIAXIS_MLX90422 != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90425 != FALSE) || (_SUPPORT_TRIAXIS_MLX90426 != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90427 != FALSE) || \
        * (_SUPPORT_INDUCTIVE_POS_SENSOR_MLX90513 != FALSE) || \
        * (_SUPPORT_DUAL_HALL_LATCH_MLX92251 != FALSE) || (_SUPPORT_DUAL_HALL_LATCH_MLX92255 != FALSE) || \
        * (_SUPPORT_FOC_MODE != FOC_MODE_NONE) */
                     | C_ERR_NV_ACT_STALL
#if (I2C_COMM != FALSE)
                     | C_ERR_NV_I2C_PARAMS
#endif /* (I2C_COMM != FALSE) */
#if (CAN_COMM != FALSE)
                     | C_ERR_NV_CAN_PARAMS
#endif /* (CAN_COMM != FALSE) */
                     );
    }
    else
    {
        /* Non Volatile Memory Header structure is valid and configuration ID matches.
         * Check the other Non Volatile Memory structures now. */
        u16Result = NV_BlockCheckCRC();

#if (LIN_COMM != FALSE)
        /* Check Standard LIN Backup */
        u16Result = NV_CheckStdLin(u16Result);
#if (LINPROT == LIN2X_HVAC52)
        /* Check Enhanced LIN Backup */
        u16Result = NV_CheckEnhLin(u16Result);
#endif /* (LINPROT == LIN2X_HVAC52) */
#endif /* (LIN_COMM != FALSE) */
    }
#if (_SUPPORT_APP_USER_MODE != FALSE)
    EXIT_SECTION(); /*lint !e438 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
    return (u16Result);
} /* End of NV_CheckCRC() */

#if (LIN_COMM != FALSE)
/*!*************************************************************************** *
 * NV_WriteLIN_STD
 * \brief   Write Standard LIN data structure to Non Volatile Memory
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] *pLinData: Pointer to LIN Data structure
 * \param   [in] u16BlockID: Block ID
 * \return  uint16_t: Non Volatile Memory update result
 *                    C_ERR_NONE: No errors
 *                    C_ERR_EEWRITE: Non Volatile Memory Write failure
 * *************************************************************************** *
 * \details Check of each User Non Volatile Memory structure the checksum
 * *************************************************************************** *
 * - Call Hierarchy: DfrDiagAssignNAD(), DfrDiagReassignNAD(),
 *                   DfrDiagSaveConfig(), NV_WriteUserDefaults()
 * - Cyclomatic Complexity: 4+1
 * - Nesting: 1
 * - Function calling: 3 (p_CopyU16(), NV_CalcCRC(), NV_WriteBlock())
 * *************************************************************************** */
uint16_t NV_WriteLIN_STD(volatile STD_LIN_PARAMS_t *pLinData, uint16_t u16BlockID)
{
    STD_LIN_PARAMS_t StdLinDef;
    uint16_t u16Force = FALSE;
    uint16_t u16Result = C_ERR_NONE;

    if ( (u16BlockID & C_NV_WRT_FORCE) != 0x0000U)
    {
        u16Force = TRUE;
    }
    if (pLinData == NULL)
    {
        p_CopyU16( (sizeof(STD_LIN_PARAMS_t) / sizeof(uint16_t)),
                   (uint16_t *)&StdLinDef,
                   (const uint16_t *)&DefStdLinParams);
        pLinData = &StdLinDef;
    }
    pLinData->u8CRC8 = 0x00U;
    pLinData->u8CRC8 = 0xFFU -
                       (uint8_t)NV_CalcCRC((const uint16_t *)pLinData,
                                           (sizeof(STD_LIN_PARAMS_t) / sizeof(uint16_t)));
    if ( (u16BlockID & C_NV_WRT_LIN_ID_1) != 0x0000U)                           /* Update LIN block ID #1 */
    {
        /* Standard LIN Parameters is one block */
        u16Result = NV_WriteBlock(ADDR_NV_STD_LIN_1, (uint16_t *)pLinData,
                                 (sizeof(STD_LIN_PARAMS_t) / sizeof(uint16_t)),
                                 u16Force);
    }
    if ( ((u16BlockID & C_NV_WRT_LIN_ID_2) != 0x0000U) &&                       /* Update LIN block ID #2 */
         (u16Result == C_ERR_NONE) ) /*lint !e845 */
    {
        /* Standard LIN Parameters is one block */
        u16Result = NV_WriteBlock(ADDR_NV_STD_LIN_2, (uint16_t *)pLinData,
                                  (sizeof(STD_LIN_PARAMS_t) / sizeof(uint16_t)),
                                  u16Force);
    }

    return (u16Result);
} /* End of NV_WriteLIN_STD() */

#if (LINPROT == LIN2X_HVAC52)
/*!*************************************************************************** *
 * NV_WriteLIN_ENH
 * \brief   Write Enhanced LIN data structure to Non Volatile Memory
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] *pLinData: Pointer to LIN Data structure
 * \param   [in] u16BlockID: Block ID
 * \return  uint16_t: Non Volatile Memory update result
 *                    C_ERR_NONE: No errors
 *                    C_ERR_EEWRITE: Non Volatile Memory Write failure
 * *************************************************************************** *
 * \details Write Enhanced LIN data to Non Volatile Memory, in multiple 64-bit Non Volatile Memory
 *          blocks. Prior to write, calculate the LIN structure CRC.
 * *************************************************************************** *
 * - Call Hierarchy: -
 * - Cyclomatic Complexity: 4+1
 * - Nesting: 1
 * - Function calling: 3 (p_CopyU16(), NV_CalcCRC(), NV_WriteBlock())
 * *************************************************************************** */
uint16_t NV_WriteLIN_ENH(volatile ENH_LIN_PARAMS_t *pLinData, uint16_t u16BlockID)
{
    ENH_LIN_PARAMS_t EnhLinDef;
    uint16_t u16Force = FALSE;
    uint16_t u16Result = C_ERR_NONE;

    if ( (u16BlockID & C_NV_WRT_FORCE) != 0x0000U)
    {
        u16Force = TRUE;
    }
    if (pLinData == NULL)
    {
        p_CopyU16( (sizeof(ENH_LIN_PARAMS_t) / sizeof(uint16_t)),
                   (uint16_t *)&EnhLinDef,
                   (const uint16_t *)&DefEnhLinParams);
        pLinData = &EnhLinDef;
    }
    pLinData->u8CRC8 = 0x00U;
    pLinData->u8CRC8 = 0xFFU -
                       (uint8_t)NV_CalcCRC((const uint16_t *)pLinData,
                                           (sizeof(ENH_LIN_PARAMS_t) / sizeof(uint16_t)));
    if ( (u16BlockID & C_NV_WRT_LIN_ID_1) != 0x0000U)                           /* Update LIN block ID #1 */
    {
        /* Enhanced LIN Parameters is one block */
        u16Result = NV_WriteBlock(ADDR_NV_ENH_LIN_1, (uint16_t *)pLinData,
                                  (sizeof(ENH_LIN_PARAMS_t) / sizeof(uint16_t)),
                                  u16Force);
    }
    if ( ((u16BlockID & C_NV_WRT_LIN_ID_2) != 0x0000U) &&                       /* Update LIN block ID #2 */
         (u16Result == C_ERR_NONE) ) /*lint !e845 */
    {
        /* Enhanced LIN Parameters is one block */
        u16Result = NV_WriteBlock(ADDR_NV_ENH_LIN_2, (uint16_t *)pLinData,
                                  (sizeof(ENH_LIN_PARAMS_t) / sizeof(uint16_t)),
                                  u16Force);
    }

    return (u16Result);
} /* End of NV_WriteLIN_ENH() */
#endif /* (LINPROT == LIN2X_HVAC52) */

#if (_SUPPORT_UDS != FALSE)
/*!*************************************************************************** *
 * NV_WriteUDS
 * \brief   Write UDS data structure to Non Volatile Memory
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] *pUdsData: Pointer to UDS Data structure
 * \return  uint16_t: Non Volatile Memory update result
 *                    C_ERR_NONE: No errors
 *                    C_ERR_EEWRITE: Non Volatile Memory Write failure
 * *************************************************************************** *
 * \details Write UDS data to Non Volatile Memory, in multiple 64-bit Non Volatile Memory blocks.
 *          Prior to write, calculate the UDS structure CRC.
 * *************************************************************************** *
 * - Call Hierarchy: -
 * - Cyclomatic Complexity: 1+1
 * - Nesting: 1
 * - Function calling: 3 (p_CopyU16(), NV_CalcCRC(), NV_WriteBlock())
 * *************************************************************************** */
uint16_t NV_WriteUDS(volatile UDS_LIN_PARAMS_t* pUdsData)
{
    UDS_LIN_PARAMS_t UdsDef;
    uint16_t u16Result = C_ERR_NONE;

    if (pUdsData == NULL)
    {
        p_CopyU16( (sizeof(UDS_LIN_PARAMS_t) / sizeof(uint16_t)),
                   (uint16_t *)&UdsDef,
                   (const uint16_t *)&DefUds);
        pUdsData = &UdsDef;
    }
    pUdsData->u8CRC8 = 0x00U;
    pUdsData->u8CRC8 = 0xFFU -
                       (uint8_t)NV_CalcCRC((const uint16_t *)pUdsData,
                                           (sizeof(UDS_LIN_PARAMS_t) / sizeof(uint16_t)));
    u16Result = NV_WriteBlock(ADDR_NV_UDS, (uint16_t *)pUdsData,
                              (sizeof(UDS_LIN_PARAMS_t) / sizeof(uint16_t)),
                              FALSE);

    return (u16Result);
} /* End of NV_WriteUDS() */
#endif /* (_SUPPORT_UDS != FALSE) */
#endif /* (LIN_COMM != FALSE) */

/*!*************************************************************************** *
 * NV_WriteEOL
 * \brief   Write EOL data structure to Non Volatile Memory
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] *pEolData: Pointer to EOL Data structure
 * \return  uint16_t: Non Volatile Memory update result
 *                    C_ERR_NONE: No errors
 *                    C_ERR_EEWRITE: Non Volatile Memory Write failure
 * *************************************************************************** *
 * \details Write EOL data to Non Volatile Memory, in multiple 64-bit Non Volatile Memory blocks.
 *          Prior to write, calculate the EOL structure CRC.
 * *************************************************************************** *
 * - Call Hierarchy: NV_WriteUserDefaults(), HandleFanCtrl()
 * - Cyclomatic Complexity: 2+1
 * - Nesting: 1
 * - Function calling: 3 (p_CopyU16(), NV_CalcCRC(), NV_WriteBlock())
 * *************************************************************************** */
uint16_t NV_WriteEOL(volatile APP_EOL_t* pEolData)
{
    APP_EOL_t EolDef;
    uint16_t u16Result = C_ERR_NONE;

    if (pEolData == NULL)
    {
        p_CopyU16( (sizeof(APP_EOL_t) / sizeof(uint16_t)),
                   (uint16_t *)&EolDef,
                   (const uint16_t *)&DefEolParams);
        pEolData = &EolDef;
    }
    pEolData->u8CRC8 = 0x00U;
    pEolData->u8CRC8 = 0xFFU - (uint8_t)NV_CalcCRC((const uint16_t *)pEolData,
                                                   (sizeof(APP_EOL_t) / sizeof(uint16_t)));
    u16Result = NV_WriteBlock(ADDR_NV_EOL, (uint16_t *)pEolData,
                              (sizeof(APP_EOL_t) / sizeof(uint16_t)),
                              FALSE);

    return (u16Result);
} /* End of NV_WriteEOL() */

#if (_SUPPORT_APP_SAVE != FALSE)
/*!*************************************************************************** *
 * NV_WriteAPP
 * \brief   Write APP data structure to Non Volatile Memory
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] *pAppData: Pointer to APP Data structure
 * \return  uint16_t: Non Volatile Memory update result
 *                    C_ERR_NONE: No errors
 *                    C_ERR_EEWRITE: Non Volatile Memory Write failure
 * *************************************************************************** *
 * \details Write APP data to Non Volatile Memory, in multiple 64-bit Non Volatile Memory blocks.
 *          Prior to write, calculate the EOL structure CRC.
 * *************************************************************************** *
 * - Call Hierarchy: NV_WriteUserDefaults(), main_init()
 * - Cyclomatic Complexity: 2+1
 * - Nesting: 1
 * - Function calling: 3 (p_CopyU16(), NV_CalcCRC(), NV_WriteBlock())
 * *************************************************************************** */
uint16_t NV_WriteAPP(volatile APP_PARAMS_t* pAppData)
{
    APP_PARAMS_t AppDef;
    uint16_t u16Result = C_ERR_NONE;

    if (pAppData == NULL)
    {
        p_CopyU16( (sizeof(APP_PARAMS_t) / sizeof(uint16_t)),
                   (uint16_t *)&AppDef,
                   (const uint16_t *)&DefAppParams);
        pAppData = &AppDef;
    }
    pAppData->u8CRC8 = 0x00U;
    pAppData->u8CRC8 = 0xFFU - (uint8_t)NV_CalcCRC( (const uint16_t *)pAppData,
                                                    (sizeof(APP_PARAMS_t) / sizeof(uint16_t)));
    u16Result = NV_WriteBlock(ADDR_NV_APP_PARAMS, (uint16_t *)pAppData,
                              (sizeof(APP_PARAMS_t) / sizeof(uint16_t)), FALSE);

    return (u16Result);
} /* End of NV_WriteAPP() */
#endif /* (_SUPPORT_APP_SAVE != FALSE) */

/*!*************************************************************************** *
 * NV_WriteActParams
 * \brief   Write Actuator Parameters data structure to Non Volatile Memory
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] *pActData: Pointer to ACT Data structure
 * \return  uint16_t: Non Volatile Memory update result
 *                    C_ERR_NONE: No errors
 *                    C_ERR_EEWRITE: Non Volatile Memory Write failure
 * *************************************************************************** *
 * \details Write ACT data to Non Volatile Memory, in multiple 64-bit Non Volatile Memory blocks.
 *          Prior to write, calculate the EOL structure CRC.
 * *************************************************************************** *
 * - Call Hierarchy: NV_WriteUserDefaults()
 * - Cyclomatic Complexity: 2+1
 * - Nesting: 1
 * - Function calling: 3 (p_CopyU16(), NV_CalcCRC(), NV_WriteBlock())
 * *************************************************************************** */
uint16_t NV_WriteActParams(volatile ACT_PARAMS_t* pActData)
{
    ACT_PARAMS_t ActDef;
    uint16_t u16Result = C_ERR_NONE;

    if (pActData == NULL)
    {
        p_CopyU16( (sizeof(ACT_PARAMS_t) / sizeof(uint16_t)),
                   (uint16_t *)&ActDef,
                   (const uint16_t *)&DefActParams);
        pActData = &ActDef;
    }
    pActData->u8CRC8 = 0x00U;
    pActData->u8CRC8 = 0xFFU - (uint8_t)NV_CalcCRC( (const uint16_t *)pActData,
                                                    (sizeof(ACT_PARAMS_t) / sizeof(uint16_t)));
    u16Result = NV_WriteBlock(ADDR_NV_ACT_PARAMS, (uint16_t *)pActData,
                              (sizeof(ACT_PARAMS_t) / sizeof(uint16_t)), FALSE);

    return (u16Result);
} /* End of NV_WriteActParams() */

#if (_SUPPORT_TRIAXIS_MLX90363 != FALSE) || (_SUPPORT_TRIAXIS_MLX9038x != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90367 != FALSE) || (_SUPPORT_TRIAXIS_MLX90372 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90377 != FALSE) || (_SUPPORT_TRIAXIS_MLX90395 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90421 != FALSE) || (_SUPPORT_TRIAXIS_MLX90422 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90425 != FALSE) || (_SUPPORT_TRIAXIS_MLX90426 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90427 != FALSE) || \
    (_SUPPORT_INDUCTIVE_POS_SENSOR_MLX90513 != FALSE) || \
    (_SUPPORT_DUAL_HALL_LATCH_MLX92251 != FALSE) || (_SUPPORT_DUAL_HALL_LATCH_MLX92255 != FALSE) || \
    (_SUPPORT_FOC_MODE != FOC_MODE_NONE)
/*!*************************************************************************** *
 * NV_WriteSensor
 * \brief   Write Sensor data structure to Non Volatile Memory
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] *pSensor: Pointer to Sensor Data structure
 * \return  uint16_t: Non Volatile Memory update result
 *                    C_ERR_NONE: No errors
 *                    C_ERR_EEWRITE: Non Volatile Memory Write failure
 * *************************************************************************** *
 * \details Write Sensor data to Non Volatile Memory, in multiple 64-bit Non Volatile Memory blocks.
 *          Prior to write, calculate the EOL structure CRC.
 * *************************************************************************** *
 * - Call Hierarchy: NV_WriteUserDefaults()
 * - Cyclomatic Complexity: 2+1
 * - Nesting: 1
 * - Function calling: 3 (p_CopyU16(), NV_CalcCRC(), NV_WriteBlock())
 * *************************************************************************** */
uint16_t NV_WriteSensor(volatile SENSOR_t* pSensor)
{
    SENSOR_t SensorDef;
    uint16_t u16Result = C_ERR_NONE;

    if (pSensor == NULL)
    {
        p_CopyU16( (sizeof(SENSOR_t) / sizeof(uint16_t)),
                   (uint16_t *)&SensorDef,
                   (const uint16_t *)&DefSensor);
        pSensor = &SensorDef;
    }
    pSensor->u8CRC8 = 0x00U;
    pSensor->u8CRC8 = 0xFFU - (uint8_t)NV_CalcCRC( (const uint16_t *)pSensor,
                                                   (sizeof(ACT_STALL_t) / sizeof(uint16_t)));
    u16Result = NV_WriteBlock(ADDR_NV_SENSOR, (uint16_t *)pSensor,
                              (sizeof(ACT_STALL_t) / sizeof(uint16_t)), FALSE);

    return (u16Result);
} /* End of NV_WriteSensor() */
#endif /* (_SUPPORT_TRIAXIS_MLX90363 != FALSE) || (_SUPPORT_TRIAXIS_MLX9038x != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90367 != FALSE) || (_SUPPORT_TRIAXIS_MLX90372 != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90377 != FALSE) || (_SUPPORT_TRIAXIS_MLX90395 != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90421 != FALSE) || (_SUPPORT_TRIAXIS_MLX90422 != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90425 != FALSE) || (_SUPPORT_TRIAXIS_MLX90426 != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90427 != FALSE) || \
        * (_SUPPORT_INDUCTIVE_POS_SENSOR_MLX90513 != FALSE) || \
        * (_SUPPORT_DUAL_HALL_LATCH_MLX92251 != FALSE) || (_SUPPORT_DUAL_HALL_LATCH_MLX92255 != FALSE) || \
        * (_SUPPORT_FOC_MODE != FOC_MODE_NONE) */

/*!*************************************************************************** *
 * NV_WriteActStall
 * \brief   Write Stall data structure to Non Volatile Memory
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] *pActStall: Pointer to Stall Data structure
 * \return  uint16_t: Non Volatile Memory update result
 *                    C_ERR_NONE: No errors
 *                    C_ERR_EEWRITE: Non Volatile Memory Write failure
 * *************************************************************************** *
 * \details Write Stall data to Non Volatile Memory, in multiple 64-bit Non Volatile Memory blocks.
 *          Prior to write, calculate the EOL structure CRC.
 * *************************************************************************** *
 * - Call Hierarchy: NV_WriteUserDefaults()
 * - Cyclomatic Complexity: 2+1
 * - Nesting: 1
 * - Function calling: 3 (p_CopyU16(), NV_CalcCRC(), NV_WriteBlock())
 * *************************************************************************** */
uint16_t NV_WriteActStall(volatile ACT_STALL_t* pActStall)
{
    ACT_STALL_t ActStallDef;
    uint16_t u16Result = C_ERR_NONE;

    if (pActStall == NULL)
    {
        p_CopyU16( (sizeof(ACT_STALL_t) / sizeof(uint16_t)),
                   (uint16_t *)&ActStallDef,
                   (const uint16_t *)&DefActStall);
        pActStall = &ActStallDef;
    }
    pActStall->u8CRC8 = 0x00U;
    pActStall->u8CRC8 = 0xFFU -
                        (uint8_t)NV_CalcCRC( (const uint16_t *)pActStall,
                                             (sizeof(ACT_STALL_t) / sizeof(uint16_t)));
    u16Result = NV_WriteBlock(ADDR_NV_ACT_STALL, (uint16_t *)pActStall,
                              (sizeof(ACT_STALL_t) / sizeof(uint16_t)), FALSE);

    return (u16Result);
} /* End of NV_WriteActStall() */

/*!*************************************************************************** *
 * NV_WritePatch
 * \brief   Write Patch data structure to Non Volatile Memory
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] *pPatch: Pointer to Patch Data structure
 * \return  uint16_t: Non Volatile Memory update result
 *                    C_ERR_NONE: No errors
 *                    C_ERR_EEWRITE: Non Volatile Memory Write failure
 * *************************************************************************** *
 * \details Write Patch data to Non Volatile Memory, in multiple 64-bit Non Volatile Memory blocks.
 *          Prior to write, calculate the EOL structure CRC.
 * *************************************************************************** *
 * - Call Hierarchy: NV_WriteUserDefaults()
 * - Cyclomatic Complexity: 2+1
 * - Nesting: 1
 * - Function calling: 3 (p_CopyU16(), NV_CalcCRC(), NV_WriteBlock())
 * *************************************************************************** */
uint16_t NV_WritePatch(volatile PATCH_HDR_t* pPatch)
{
    uint16_t u16Result = C_ERR_NONE;
    uint16_t u16LengthW = ((pPatch->u8Length + 3U) & 0x1CU);                    /* Length in 16-bit Words */

    if (pPatch->u8Length != 0x00U)
    {
        pPatch->u8CRC = 0x00U;
        pPatch->u8CRC = 0xFFU - (uint8_t)NV_CalcCRC( (const uint16_t *)pPatch,
                                                     (uint16_t)pPatch->u8Length);
    }
    else
    {
        u16LengthW = 0x0004U;                                                   /* At last one Non Volatile Memory block */
    }
    u16Result = NV_WriteBlock(ADDR_NV_START, (uint16_t *)pPatch,
                              u16LengthW, FALSE);

    return (u16Result);
} /* End of NV_WritePatch() */

#if (_SUPPORT_NV_LOG_ERROR != FALSE)
/*!*************************************************************************** *
 * NV_WriteErrorLog
 * \brief   Write Error Log data structure to Non Volatile Memory
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] *pLogData: Pointer to Error-log Data structure
 * \return  uint16_t: Non Volatile Memory update result
 *                    C_ERR_NONE: No errors
 *                    C_ERR_EEWRITE: Non Volatile Memory Write failure
 * *************************************************************************** *
 * \details Write LOG data to Non Volatile Memory, in multiple 64-bit Non Volatile Memory blocks.
 *          This routine is only for test-purpose!! Creating external
 *          failures frequently may cause damage on these Non Volatile Memory cells!
 * *************************************************************************** *
 * - Call Hierarchy: NV_WriteUserDefaults()
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 3 (p_CopyU16(), NV_CalcCRC(), NV_WriteBlock())
 * *************************************************************************** */
uint16_t NV_WriteErrorLog(APP_LOG_t *pLogData)
{
    uint16_t NV_BLOCK_ADDR = (uint16_t)ADDR_NV_ERRORLOG;
    uint16_t *pBlock = (uint16_t *)((void *)pLogData);
    uint16_t u16Result = C_ERR_NONE;

    u16Result = NV_WriteBlock(NV_BLOCK_ADDR, pBlock,
                              (sizeof(APP_LOG_t) / sizeof(uint16_t)), FALSE);
    return (u16Result);
}
#endif /* (_SUPPORT_NV_LOG_ERROR != FALSE) */

#if (I2C_COMM != FALSE)
/*!*************************************************************************** *
 * NV_WriteI2cParams
 * \brief   Write I2C Parameters data structure to Non Volatile Memory
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] *pI2cData: Pointer to I2C Data structure
 * \return  uint16_t: Non Volatile Memory update result
 *                    C_ERR_NONE: No errors
 *                    C_ERR_EEWRITE: Non Volatile Memory Write failure
 * *************************************************************************** *
 * \details Write I2C data to Non Volatile Memory, in multiple 64-bit Non Volatile Memory blocks.
 *          Prior to write, calculate the EOL structure CRC.
 * *************************************************************************** *
 * - Call Hierarchy: NV_WriteUserDefaults()
 * - Cyclomatic Complexity: 2+1
 * - Nesting: 1
 * - Function calling: 3 (p_CopyU16(), NV_CalcCRC(), NV_WriteBlock())
 * *************************************************************************** */
uint16_t NV_WriteI2cParams(volatile STD_I2C_PARAMS_t* pI2cData)
{
    STD_I2C_PARAMS_t I2cDef;
    uint16_t u16Result = C_ERR_NONE;

    if (pI2cData == NULL)
    {
        p_CopyU16( (sizeof(STD_I2C_PARAMS_t) / sizeof(uint16_t)),
                   (uint16_t *)&I2cDef,
                   (const uint16_t *)&DefI2cParams);
        pI2cData = &I2cDef;
    }
    pI2cData->u8CRC8 = 0x00U;
    pI2cData->u8CRC8 = 0xFFU -
                       (uint8_t)NV_CalcCRC( (const uint16_t *)pI2cData,
                                            (sizeof(STD_I2C_PARAMS_t) / sizeof(uint16_t)));
    u16Result = NV_WriteBlock(ADDR_NV_I2C_PARAMS, (uint16_t *)pI2cData,
                              (sizeof(STD_I2C_PARAMS_t) / sizeof(uint16_t)), FALSE);

    return (u16Result);
} /* End of NV_WriteI2cParams() */
#endif /* (I2C_COMM != FALSE) */

#if (CAN_COMM != FALSE)
/*!*************************************************************************** *
 * NV_WriteCanParams
 * \brief   Write CAN Parameters data structure to Non Volatile Memory
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] *pCanData: Pointer to I2C Data structure
 * \return  uint16_t: Non Volatile Memory update result
 *                    C_ERR_NONE: No errors
 *                    C_ERR_EEWRITE: Non Volatile Memory Write failure
 * *************************************************************************** *
 * \details Write CAN data to Non Volatile Memory, in multiple 64-bit Non Volatile Memory blocks.
 *          Prior to write, calculate the EOL structure CRC.
 * *************************************************************************** *
 * - Call Hierarchy: NV_WriteUserDefaults()
 * - Cyclomatic Complexity: 2+1
 * - Nesting: 1
 * - Function calling: 3 (p_CopyU16(), NV_CalcCRC(), NV_WriteBlock())
 * *************************************************************************** */
uint16_t NV_WriteCanParams(volatile STD_CAN_PARAMS_t* pCanData)
{
    STD_CAN_PARAMS_t CanDef;
    uint16_t u16Result = C_ERR_NONE;

    if (pCanData == NULL)
    {
        p_CopyU16( (sizeof(STD_CAN_PARAMS_t) / sizeof(uint16_t)),
                   (uint16_t *)&CanDef,
                   (const uint16_t *)&DefCanParams);
        pCanData = &CanDef;
    }
    pCanData->u8CRC8 = 0x00U;
    pCanData->u8CRC8 = 0xFFU -
                       (uint8_t)NV_CalcCRC( (const uint16_t *)pCanData,
                                            (sizeof(STD_CAN_PARAMS_t) / sizeof(uint16_t)));
    u16Result = NV_WriteBlock(ADDR_NV_CAN_PARAMS, (uint16_t *)pCanData,
                              (sizeof(STD_CAN_PARAMS_t) / sizeof(uint16_t)), FALSE);

    return (u16Result);
} /* End of NV_WriteCanParams() */
#endif /* (CAN_COMM != FALSE) */

/*!*************************************************************************** *
 * NV_WriteUserDefaults
 * \brief   Write to user area Non Volatile Memory block(64-bits) defaults
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16ErrorIdMask: bit-mask of user Non Volatile Memory structures to
 *               be written with default data
 * \return  uint16_t: Non Volatile Memory update result
 *                    C_ERR_NONE: No errors
 *                    C_ERR_EEWRITE: Non Volatile Memory Write failure
 * *************************************************************************** *
 * \details Write User defaults in Non Volatile Memory
 * *************************************************************************** *
 * - Call Hierarchy: DfrDiagMlxDebug(), main_Init()
 * - Cyclomatic Complexity: 11+1
 * - Nesting: 3
 * - Function calling: 10 (p_CopyU16(), NV_CalcCRC(), NV_WriteBlock(),
 *                         NV_WriteLIN_STD(), NV_WriteUDS(), NV_WriteEOL(),
 *                         NV_WriteAPP(), NV_WriteActParams(), NV_WriteSensor(),
 *                         NV_WriteActStall())
 * *************************************************************************** */
uint16_t NV_WriteUserDefaults(uint16_t u16ErrorIdMask)
{
    uint16_t u16Result = C_ERR_NONE;

#if (LIN_COMM != FALSE)
    if ( (u16ErrorIdMask & (C_ERR_NV_LIN_STD_1 | C_ERR_NV_LIN_STD_2)) != 0x0000U)
    {
        /* At least one Standard LIN Block is invalid; Check for valid Back-up */
        if ( (u16ErrorIdMask & (C_ERR_NV_LIN_STD_1 | C_ERR_NV_LIN_STD_2)) == (C_ERR_NV_LIN_STD_1 | C_ERR_NV_LIN_STD_2) )
        {
            /* Both Standard LIN blocks are invalid: Write default LIN block */
            u16Result |= NV_WriteLIN_STD(NULL, C_NV_WRT_LIN_ID_ALL);
        }
        else
        {
            (void)NV_CheckStdLin(u16ErrorIdMask);
        }
    }
#if (LINPROT == LIN2X_HVAC52)
    if ( (u16ErrorIdMask & (C_ERR_NV_LIN_ENH_1 | C_ERR_NV_LIN_ENH_2)) != 0x0000U)
    {
        /* At least one Enhanced LIN Block is invalid; Check for valid Back-up */
        if ( (u16ErrorIdMask & (C_ERR_NV_LIN_ENH_1 | C_ERR_NV_LIN_ENH_2)) == (C_ERR_NV_LIN_ENH_1 | C_ERR_NV_LIN_ENH_2) )
        {
            /* Both Enhanced LIN blocks are invalid: Write default LIN block */
            u16Result |= NV_WriteLIN_ENH(NULL, C_NV_WRT_LIN_ID_ALL);
        }
        else
        {
            (void)NV_CheckEnhLin(u16ErrorIdMask);
        }
    }
#endif /* (LINPROT == LIN2X_HVAC52) */
#if (_SUPPORT_UDS != FALSE)
    if ( (u16ErrorIdMask & C_ERR_NV_LIN_UDS) != 0x0000U)
    {
        /* Write default UDS block */
        u16Result |= NV_WriteUDS(NULL);
    }
#endif /* (_SUPPORT_UDS != FALSE) */
#endif /* (LIN_COMM != FALSE) */
    if ( (u16ErrorIdMask & C_ERR_NV_EOL) != 0x0000U)
    {
        /* Write default EOL block */
        u16Result |= NV_WriteEOL(NULL);
    }
#if (_SUPPORT_APP_SAVE != FALSE)
    if ( (u16ErrorIdMask & C_ERR_NV_APP_PARAMS) != 0x0000U)
    {
        /* Write default Application block */
        u16Result |= NV_WriteAPP(NULL);
    }
#endif /* (_SUPPORT_APP_SAVE != FALSE) */
    if ( (u16ErrorIdMask & C_ERR_NV_ACT_PARAMS) != 0x0000U)
    {
        /* Write default Actuator Parameters block */
        u16Result |= NV_WriteActParams(NULL);
    }
#if (_SUPPORT_TRIAXIS_MLX90363 != FALSE) || (_SUPPORT_TRIAXIS_MLX9038x != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90367 != FALSE) || (_SUPPORT_TRIAXIS_MLX90372 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90377 != FALSE) || (_SUPPORT_TRIAXIS_MLX90395 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90421 != FALSE) || (_SUPPORT_TRIAXIS_MLX90422 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90425 != FALSE) || (_SUPPORT_TRIAXIS_MLX90426 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90427 != FALSE) || \
    (_SUPPORT_INDUCTIVE_POS_SENSOR_MLX90513 != FALSE) || \
    (_SUPPORT_DUAL_HALL_LATCH_MLX92251 != FALSE) || (_SUPPORT_DUAL_HALL_LATCH_MLX92255 != FALSE) || \
    (_SUPPORT_FOC_MODE != FOC_MODE_NONE)
    if ( (u16ErrorIdMask & C_ERR_NV_SENSOR) != 0x0000U)
    {
        /* Write default Sensor Parameters block */
        u16Result |= NV_WriteSensor(NULL);
    }
#endif /* (_SUPPORT_TRIAXIS_MLX90363 != FALSE) || (_SUPPORT_TRIAXIS_MLX9038x != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90367 != FALSE) || (_SUPPORT_TRIAXIS_MLX90372 != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90377 != FALSE) || (_SUPPORT_TRIAXIS_MLX90395 != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90421 != FALSE) || (_SUPPORT_TRIAXIS_MLX90422 != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90425 != FALSE) || (_SUPPORT_TRIAXIS_MLX90426 != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90427 != FALSE) || \
        * (_SUPPORT_INDUCTIVE_POS_SENSOR_MLX90513 != FALSE) || \
        * (_SUPPORT_DUAL_HALL_LATCH_MLX92251 != FALSE) || (_SUPPORT_DUAL_HALL_LATCH_MLX92255 != FALSE) || \
        * (_SUPPORT_FOC_MODE != FOC_MODE_NONE) */
    if ( (u16ErrorIdMask & C_ERR_NV_ACT_STALL) != 0x0000U)
    {
        /* Write default Stall Parameters block */
        u16Result |= NV_WriteActStall(NULL);
    }
#if (I2C_COMM != FALSE)
    if ( (u16ErrorIdMask & C_ERR_NV_I2C_PARAMS) != 0x0000U)
    {
        /* Write default I2C Parameters block */
        u16Result |= NV_WriteI2cParams(NULL);
    }
#endif /* (I2C_COMM != FALSE) */
#if (CAN_COMM != FALSE)
    if ( (u16ErrorIdMask & C_ERR_NV_CAN_PARAMS) != 0x0000U)
    {
        /* Write default CAN Parameters block */
        u16Result |= NV_WriteCanParams(NULL);
    }
#endif /* (CAN_COMM != FALSE) */

    /* Check/update header at last */
    if ( (u16ErrorIdMask & (C_ERR_NV_HDR_KEEP_COUNT | C_ERR_NV_HDR_RST_COUNT)) != 0x0000U)
    {
        /* Write default header block */
        HEADER_t hdr;
        p_CopyU16( (sizeof(HEADER_t) / sizeof(uint16_t)),
                   (uint16_t *)&hdr,
                   (const uint16_t *)&DefHeader);
        hdr.u12StructIDs = C_HDR_STRUCT_ID;
        if ( (u16ErrorIdMask & C_ERR_NV_HDR_KEEP_COUNT) != 0x0000U)
        {
            hdr.u16WriteCycleCountLSW = UserParams.hdr.u16WriteCycleCountLSW + 1U;
            if (hdr.u16WriteCycleCountLSW != 0U)
            {
                hdr.u2WriteCycleCountMSW = UserParams.hdr.u2WriteCycleCountMSW;
            }
            else
            {
                hdr.u2WriteCycleCountMSW = UserParams.hdr.u2WriteCycleCountMSW + 1U;
            }
        }
        hdr.u8CRC8 = 0xFFU - (uint8_t)NV_CalcCRC( (const uint16_t *)&hdr,
                                                  (sizeof(HEADER_t) / sizeof(uint16_t)));
        u16Result |= NV_WriteBlock(ADDR_NV_HDR, (uint16_t *)&hdr,
                                   (sizeof(HEADER_t) / sizeof(uint16_t)), TRUE);
    }

    return (u16Result);
} /* End of NV_WriteUserDefaults() */

/*!*************************************************************************** *
 * NV_MlxCalib
 * \brief   Check Melexis Calibration area and trim peripherals
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  uint16_t: Error-result
 *                    FALSE: No error (chip successfully trimmed)
 *                    TRUE: Chip peripherals not trimmed (LIN communication
 *                          is still fully-working)
 * *************************************************************************** *
 * \details Check the Melexis Calibration CRC
 * *************************************************************************** *
 * - Call Hierarchy: DfrDiagMlxDebug(), main_Init()
 * - Cyclomatic Complexity: 3+1
 * - Nesting: 3
 * - Function calling: 1 (NV_CalcCRC())
 * *************************************************************************** */
uint16_t NV_MlxCalib(void)
{
    uint16_t u16ErrorResult;
#if (_SUPPORT_APP_USER_MODE != FALSE)
    ENTER_SECTION(SYSTEM_MODE); /*lint !e534 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
    IO_MLX16_ITC_PEND0_S = B_MLX16_ITC_PEND0_EE_SH_ECC;
#if (_SUPPORT_APP_USER_MODE != FALSE)
    EXIT_SECTION(); /*lint !e438 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
    if ( (NV_CalcCRC( (const uint16_t *)&CalibrationParams, (sizeof(MLX_CALIB_t) / sizeof(uint16_t))) != 0xFFU) ||
         ((IO_MLX16_ITC_PEND0_S & B_MLX16_ITC_PEND0_EE_SH_ECC) != 0U) )
    {
        /* Return error-condition; Motor operation is not allowed; LIN communication is okay */
#if defined (__MLX81160__)
#if (_SUPPORT_APP_USER_MODE != FALSE)
        ENTER_SECTION(SYSTEM_MODE); /*lint !e534 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
        IO_TRIM_ADC_S = C_DEF_TRIM_ADC;
#if (_SUPPORT_APP_USER_MODE != FALSE)
        EXIT_SECTION(); /*lint !e438 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
#elif defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
        IO_TRIM_IILD = C_DEF_TRIM_IILD;
#if (_SUPPORT_APP_USER_MODE != FALSE)
        ENTER_SECTION(SYSTEM_MODE); /*lint !e534 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
        IO_TRIM_ADC_S = C_DEF_TRIM_ADC;
#if (_SUPPORT_APP_USER_MODE != FALSE)
        EXIT_SECTION(); /*lint !e438 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
#endif /* defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__) */
        u16ErrorResult = TRUE;
    }
#if defined (__MLX81330B02__) || defined (__MLX81332B02__) || defined (__MLX81334__)
    else if (CalibrationParams.u8APP_TRIM03_CalibVersion < C_NV_MLX_VER_3)      /* MMP200626-2 */
    {
        /* Incorrect Non Volatile Memory layout Version */
        u16ErrorResult = TRUE;
    }
#elif defined (__MLX81339__) || defined (__MLX81350__)
    else if (CalibrationParams.u8APP_TRIM03_CalibVersion < C_NV_MLX_VER_1)
    {
        /* Incorrect Non Volatile Memory layout Version */
        u16ErrorResult = TRUE;
    }
#endif
    else
    {
#if defined (__MLX81160__)
        /* TODO[MMP] IO_TRIM1_DRV = CalibrationParams.u16APP_TRIM30_TRIM1_DRV;
         *           IO_TRIM2_DRV = CalibrationParams.u16APP_TRIM31_TRIM2_DRV; */
#if (_SUPPORT_APP_USER_MODE != FALSE)
        ENTER_SECTION(SYSTEM_MODE); /*lint !e534 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
        IO_TRIM_ADC_S = C_DEF_TRIM_ADC;
#if (_SUPPORT_APP_USER_MODE != FALSE)
        EXIT_SECTION(); /*lint !e438 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
        IO_TRIM1_DRV = ((CalibrationParams.u10App_TRIM32_CPCLK) << 2);
        IO_TRIM_MISC = CalibrationParams.u6App_TRIM32_OTD;
        IO_PORT_SSCM2_CONF = B_PORT_SSCM2_CONF_SSCM2_CENTERED;                  /* Centre SSCM around middle (MMP210527-2) */
        IO_PORT_STEP2_CONF = (138U << 8) |                                      /* M_PORT_STEP_CONF_STEP_CNT = 138 */
                             (5U << 4) |                                        /* M_PORT_STEP_CONF_STEP_DUR = 5 */
                             (1U);                                              /* M_PORT_STEP_CONF_STEP_INC = 1 */
#elif defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__)
        /* Load CALIB data to driver registers */
        uint16_t u16Trim;
#if defined (__MLX81330B02__) || defined (__MLX81332B02__) || defined (__MLX81334__)
#if (_SUPPORT_CPFREQ == 82U)
        IO_TRIM1_DRV = CalibrationParams.u16APP_TRIM30_TRIM1_DRV;
#elif (_SUPPORT_CPFREQ == 60U)
        IO_TRIM1_DRV = CalibrationParams.u16APP_TRIM34_TRIM1_DRV_60M;
#elif ((_SUPPORT_CPFREQ > 50U) && (_SUPPORT_CPFREQ < 95U))
        uint16_t u16CP_Freq = CalibrationParams.u16APP_TRIM34_TRIM1_DRV_60M +
                              (p_MulDivI16_I16byI16byI16( (CalibrationParams.u16APP_TRIM30_TRIM1_DRV -
                                                           CalibrationParams.u16APP_TRIM34_TRIM1_DRV_60M),
                                                          (_SUPPORT_CPFREQ - 60), (82 - 60)) &
                               M_TRIM1_DRV_PRE_TRIM_DRVMOD_CPCLK);
        IO_TRIM1_DRV = u16CP_Freq;
#else  /* _SUPPORT_CPFREQ */
#error "ERROR: Invalid CP-Frequency defined"
#endif /* _SUPPORT_CPFREQ */
#if (_SUPPORT_CPFREQ_TEMPCOMP != FALSE)
        g_u16CP_FreqTrim_RT = (IO_TRIM1_DRV & M_TRIM1_DRV_PRE_TRIM_DRVMOD_CPCLK) >> 2;
#endif /* (_SUPPORT_CPFREQ_TEMPCOMP != FALSE) */
#elif defined (__MLX81339__) || defined (__MLX81350__)
#if (_SUPPORT_CPFREQ == 83U)
        IO_TRIM1_DRV = CalibrationParams.u16APP_TRIM30_TRIM1_DRVMOD_CPCLK_HIGH;
#elif (_SUPPORT_CPFREQ == 79U)
        IO_TRIM1_DRV = CalibrationParams.u16APP_TRIM31_TRIM1_DRVMOD_CPCLK_LOW;
#elif ((_SUPPORT_CPFREQ > 50U) && (_SUPPORT_CPFREQ < 95U))
        uint16_t u16CP_Freq = CalibrationParams.u16APP_TRIM31_TRIM1_DRVMOD_CPCLK_LOW +
                              (p_MulDivI16_I16byI16byI16( (CalibrationParams.u16APP_TRIM30_TRIM1_DRVMOD_CPCLK_HIGH -
                                                           CalibrationParams.u16APP_TRIM31_TRIM1_DRVMOD_CPCLK_LOW),
                                                          (_SUPPORT_CPFREQ - 79), (83 - 79)) &
                               M_TRIM1_DRV_PRE_TRIM_DRVMOD_CPCLK);
        IO_TRIM1_DRV = u16CP_Freq;
#else  /* _SUPPORT_CPFREQ */
#error "ERROR: Invalid CP-Frequency defined"
#endif /* _SUPPORT_CPFREQ */
#if (_SUPPORT_CPFREQ_TEMPCOMP != FALSE)
        g_u16CP_FreqTrim_RT = (IO_TRIM1_DRV & M_TRIM1_DRV_PRE_TRIM_DRVMOD_CPCLK);
#endif /* (_SUPPORT_CPFREQ_TEMPCOMP != FALSE) */
#else  /* defined (__MLX81330B02__) || defined (__MLX81332B02__) || defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__) */
        IO_TRIM1_DRV = CalibrationParams.u16APP_TRIM30_TRIM1_DRV;
#endif /* defined (__MLX81330B02__) || defined (__MLX81332B02__) || defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__) */
#if defined (__MLX81339__) || defined (__MLX81350__)
        IO_TRIM2_DRV = CalibrationParams.u16APP_TRIM32_TRIM2_DRVSUP_CSAGAIN_SLWRT;
#if ((C_MOTOR_FAMILY & MF_TYPE) == MF_DC)                                       /* MMP190312-1 */
        u16Trim = CalibrationParams.u16APP_TRIM33_TRIM_CSA_HIGH;
#else  /* ((C_MOTOR_FAMILY & MF_TYPE) == MF_DC) */
        u16Trim = CalibrationParams.u16APP_TRIM34_TRIM_CSA_LOW;
#endif /* ((C_MOTOR_FAMILY & MF_TYPE) == MF_DC) */
        IO_TRIM3_DRV = u16Trim;
        IO_TRIM_MISC = CalibrationParams.u16APP_TRIM35_TRIM_MISC;
#else  /* defined (__MLX81350__) */
        IO_TRIM2_DRV = CalibrationParams.u16APP_TRIM31_TRIM2_DRV;
#if ((C_MOTOR_FAMILY & MF_TYPE) == MF_DC)                                       /* MMP190312-1 */
        u16Trim = CalibrationParams.u8APP_TRIM32_TRIM3_DRV_HIGH;
#else  /* ((C_MOTOR_FAMILY & MF_TYPE) == MF_DC) */
        u16Trim = CalibrationParams.u8APP_TRIM32_TRIM3_DRV_LOW;
#endif /* ((C_MOTOR_FAMILY & MF_TYPE) == MF_DC) */
        if ( (u16Trim & M_TRIM3_DRV_TRIM_CSA_CL) == 0x00U)
        {
            u16Trim = 0x002EU;
        }
#if (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM_BIPOLAR)
        else if (CalibrationParams.u8APP_TRIM32_TRIM3_DRV_LOW != 0U)
        {
            /* MLX81330: LOW = 0.75A, HIGH = 1.0A; Set Trim at 1.25A
             * MLX81332/34: LOW = 1.5A, HIGH = 2.0A; Set Trim at 2.5A */
            u16Trim += (u16Trim - CalibrationParams.u8APP_TRIM32_TRIM3_DRV_LOW);
            if (u16Trim > M_TRIM3_DRV_TRIM_CSA_CL)
            {
                /* Maximise to CSA range */
                u16Trim = M_TRIM3_DRV_TRIM_CSA_CL;
            }
        }
#endif /* (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM_BIPOLAR) */
        IO_TRIM3_DRV = u16Trim;
        IO_TRIM_MISC = CalibrationParams.u16APP_TRIM33_TRIM_MISC;
#endif /* defined (__MLX81350__) */
#if !defined (__MLX81330A01__) && !defined (__MLX81332A01__)
        IO_PORT_SSCM2_CONF = B_PORT_SSCM2_CONF_SSCM2_CENTERED;                  /* Centre SSCM around middle (MMP190211-2) */
#endif /* !defined (__MLX81330A01__) && !defined (__MLX81332A01__) */
        IO_PORT_STEP2_CONF = (138U << 8) |                                      /* M_PORT_STEP_CONF_STEP_CNT = 138 */
                             (5U << 4) |                                        /* M_PORT_STEP_CONF_STEP_DUR = 5 */
                             (1U);                                              /* M_PORT_STEP_CONF_STEP_INC = 1 */
#elif defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
        /* TODO[MMP]: Set OC-Level */
        /* IO_PORT_CURR_SENS = (IO_PORT_CURR_SENS & ~M_PORT_CURR_SENS_SET_CS) | CalibrationParams.u16APP_TRIM32_TRIM3_DRV_LOW; */

        IO_TRIM_IILD = ((CalibrationParams.u16APP_TRIM33_TRIM1 >> 8) & M_TRIM_IILD_CNT_IILD);
        IO_TRIM_CP = (CalibrationParams.u16APP_TRIM33_TRIM1 & (M_TRIM_CP_TR_CP | M_TRIM_CP_TR_RCO50K));
        if ( (CalibrationParams.u16APP_TRIM33_TRIM1 & 0x8000U) != 0U)
        {
            IO_TRIM_IILD = (IO_TRIM_IILD | B_TRIM_IILD_LOCK);
            IO_TRIM_CP = (IO_TRIM_CP | B_TRIM_CP_LOCK);
        }
#if (_SUPPORT_APP_USER_MODE != FALSE)
        ENTER_SECTION(SYSTEM_MODE); /*lint !e534 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
#if defined (__MLX81340A01__) || defined (__MLX81344A01__) || defined (__MLX81346A01__)
        IO_TRIM_ADC_S =
            ((CalibrationParams.u16APP_TRIM32_TRIM0 >> 8) &
             (B_TRIM_ADC_ADC_BLOCK_BYPASSBUF | B_TRIM_ADC_ADC_BLOCK_INTERCHOP | B_TRIM_ADC_ADC_BLOCK_LATCHCTRL |
              M_TRIM_ADC_ADC_INTREF_TRM));
#else  /* defined (__MLX81340A01__) || defined (__MLX81344A01__) || defined (__MLX81346A01__) */
        /* Fixed value for TRIM_ADC:
         * B_TRIM_ADC_ADC_BLOCK_BYPASSBUF = 0
         * B_TRIM_ADC_ADC_BLOCK_INTERCHOP = 0
         * B_TRIM_ADC_ADC_BLOCK_LATCHCTRL = 0
         * M_TRIM_ADC_ADC_INTREF_TRM = 3
         */
        IO_TRIM_ADC_S = 0x03U;
#endif /* defined (__MLX81340A01__) || defined (__MLX81344A01__) || defined (__MLX81346A01__) */
        IO_TRIM_MISC = (CalibrationParams.u16APP_TRIM32_TRIM0 & (M_TRIM_MISC_TRIM_SDAFILT_IO | M_TRIM_MISC_TRIM_OTD));
        if ( (CalibrationParams.u16APP_TRIM32_TRIM0 & 0x8000U) != 0U)
        {
            IO_TRIM_ADC_S = (IO_TRIM_ADC_S | B_TRIM_ADC_LOCK);
            IO_TRIM_MISC = (IO_TRIM_MISC | B_TRIM_MISC_LOCK);
        }
#if (_SUPPORT_APP_USER_MODE != FALSE)
        EXIT_SECTION(); /*lint !e438 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
#endif
        u16ErrorResult = FALSE;
    }
    return (u16ErrorResult);
} /* End of NV_MlxCalib() */

#if (_SUPPORT_APP_SAVE != FALSE)
/*!*************************************************************************** *
 * NV_TemperatureBasedWriteCountIncrease
 * \brief   Write counter increase junction temperature dependent
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  (uint16_t) Write count increase value (temperature dependent)
 * *************************************************************************** *
 * \details Write application dependent data to Non Volatile Memory
 * *************************************************************************** *
 * - Call Hierarchy: Non Volatile Memory_AppStore()
 * - Cyclomatic Complexity: 9+1
 * - Nesting: 5
 * - Function calling: 1 (ADC_Conv_TempJ())
 * *************************************************************************** */
uint16_t NV_TemperatureBasedWriteCountIncrease(void)
{
    uint16_t u16Result;
    int16_t i16ChipTemperature = ADC_Conv_TempJ(FALSE);

    for (u16Result = 0; u16Result < (sizeof(ai16TemperatureScale) / sizeof(ai16TemperatureScale[0])); u16Result++)
    {
        if (i16ChipTemperature < ai16TemperatureScale[u16Result])
        {
            break;
        }
    }
    return (u16Result + 1U);
} /* End of NV_TemperatureBasedWriteCountIncrease() */

/*!*************************************************************************** *
 * NV_AppStore
 * \brief   Write application relevant data to Non Volatile Memory (during UV (Emergency))
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16ParamLSW: 16-bit application-parameters "LSW"
 * \param   [in] u16ParamMSW: 16-bit application-parameters "MSW"
 * \return  -
 * *************************************************************************** *
 * \details Write application dependent data to Non Volatile Memory
 * *************************************************************************** *
 * - Call Hierarchy: HandleSleepMotorRequest()
 * - Cyclomatic Complexity: 1+1
 * - Nesting: 1
 * - Function calling: 1 (NV_CalcCRC())
 * *************************************************************************** */
void NV_AppStore(uint16_t u16ParamLSW, uint16_t u16ParamMSW)
{
    uint16_t *pNV_AppData_Rd = (uint16_t *)ADDR_NV_APP_PARAMS;
    uint16_t *pNV_AppData_Wrt = (uint16_t *)ADDR_NV_APP_PARAMS;
#if (_SUPPORT_DUAL_APP_BLOCK != FALSE)
    uint16_t *pNV_AppData_1 = (uint16_t *)ADDR_NV_APP_PARAMS;
    uint16_t *pNV_AppData_2 = (uint16_t *)ADDR_NV_APP_PARAMS_2;
#endif /* (_SUPPORT_DUAL_APP_BLOCK != FALSE) */
    APP_PARAMS_t AppData;

#if (_SUPPORT_NV_EMERGENCY_STORE != FALSE)
    /* Don't get interrupted any more */
#if (_SUPPORT_APP_USER_MODE != FALSE)
    ENTER_SECTION(SYSTEM_MODE); /*lint !e534 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
    IO_MLX16_ITC_MASK0_S = 0x0000U;
    IO_MLX16_ITC_MASK1_S = 0x0000U;
    IO_MLX16_ITC_MASK2_S = 0x0000U;
    IO_MLX16_ITC_MASK3_S = 0x0000U;
#if defined (__MLX81160__) || defined (__MLX81332B01__) || defined (__MLX81332B02__) || defined (__MLX81334__) || defined (__MLX81339__)
    IO_MLX16_ITC_MASK4_S = 0x0000U;
#elif defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
    IO_MLX16_ITC_MASK4_S = 0x0000U;
    IO_MLX16_ITC_MASK5_S = 0x0000U;
#endif
    IO_PORT_DRV_OUT &= ~M_PORT_DRV_OUT_ENABLE_DRV;                              /* Disable Motor driver */
    IO_PWM_MASTER1_CTRL = B_PWM_MASTER1_STOP;                                   /* Stop PWM Units */
    HAL_ADC_Stop();                                                             /* Stop ADC */
    IO_SPI_CTRL = B_SPI_STOP;                                                   /* Stop the SPI despite it's run or not */
    IO_CTIMER0_CTRL = B_CTIMER0_STOP;                                           /* Stop CTimer #0 */
    IO_CTIMER1_CTRL = B_CTIMER1_STOP;                                           /* Stop CTimer #1 */
    IO_STIMER_CTRL = 0x0000U;                                                   /* Stop STimer */
    ml_ResetDrv();                                                              /* Stop MLX4 */
    IO_PORT_DRV_OUT = 0x0000U;                                                  /* Disable current sense amplifier, driver clock and driver supply */
    IO_PORT_ADC_CTRL_S = 0x0000U;                                               /* Power off ADC */
#if (_SUPPORT_APP_USER_MODE != FALSE)
    EXIT_SECTION(); /*lint !e438 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
#endif /* (_SUPPORT_NV_EMERGENCY_STORE != FALSE) */

    /* Copy Non Volatile Memory Application to System RAM */
#if (_SUPPORT_DUAL_APP_BLOCK != FALSE)
#if (_SUPPORT_APP_USER_MODE != FALSE)
    ENTER_SECTION(SYSTEM_MODE); /*lint !e534 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
    IO_MLX16_ITC_PEND0_S = B_MLX16_ITC_PEND0_EE_SH_ECC;
    if ( (NV_CalcCRC( (const uint16_t *)pNV_AppData_1, (sizeof(APP_PARAMS_t) / sizeof(uint16_t))) == 0xFFU) &&
         ((IO_MLX16_ITC_PEND0_S & B_MLX16_ITC_PEND0_EE_SH_ECC) == 0U) )
    {
        /* 1st Block is valid; Check 2nd Block too */
        if ( (NV_CalcCRC( (const uint16_t *)pNV_AppData_2, (sizeof(APP_PARAMS_t) / sizeof(uint16_t))) == 0xFFU) &&
             ((IO_MLX16_ITC_PEND0_S & B_MLX16_ITC_PEND0_EE_SH_ECC) == 0U) )
        {
            /* 2nd Block is valid too; Check which block has most recent data */
            if ( ( ((APP_PARAMS_t *)pNV_AppData_1)->u4PageID ^ ((APP_PARAMS_t *)pNV_AppData_2)->u4PageID) == 0U)
            {
                /* 1st Block is most recent data */
                pNV_AppData_Rd = pNV_AppData_1;
            }
            else
            {
                /* 2nd Block is most recent data */
                pNV_AppData_Rd = pNV_AppData_2;
            }
            /* Copy Non Volatile Memory block to System RAM */
            p_CopyU16( (sizeof(APP_PARAMS_t) / sizeof(uint16_t)),
                       (uint16_t *)&AppData,
                       (const uint16_t *)pNV_AppData_Rd);
            /* Determine which Non Volatile Memory block will be written */
            if ( (((APP_PARAMS_t *)pNV_AppData_1)->u4WrtCntH < ((APP_PARAMS_t *)pNV_AppData_2)->u4WrtCntH) ||
                 ((((APP_PARAMS_t *)pNV_AppData_1)->u4WrtCntH == ((APP_PARAMS_t *)pNV_AppData_2)->u4WrtCntH) &&
                  (((APP_PARAMS_t *)pNV_AppData_1)->u16WrtCntL <= ((APP_PARAMS_t *)pNV_AppData_2)->u16WrtCntL)) )
            {
                /* 1st Block has the lowest Write Cycle counter */
                pNV_AppData_Wrt = pNV_AppData_1;
                AppData.u4PageID = ((APP_PARAMS_t *)pNV_AppData_2)->u4PageID;   /* 1st Block becomes newest */
            }
            else
            {
                /* 2nd Block has the lowest Write Cycle counter */
                pNV_AppData_Wrt = pNV_AppData_2;
                AppData.u4PageID = ((APP_PARAMS_t *)pNV_AppData_1)->u4PageID ^ 1U;   /* 2nd Block becomes newest */
            }
            /* Copy the write cycle counter from the lowest write cycle counter page */
            AppData.u16WrtCntL = ((APP_PARAMS_t *)pNV_AppData_Wrt)->u16WrtCntL;
            AppData.u4WrtCntH = ((APP_PARAMS_t *)pNV_AppData_Wrt)->u4WrtCntH;
        }
        else
        {
            /* 1st Block is valid and 2nd is not; Use it! */
            pNV_AppData_Rd = pNV_AppData_1;                                     /* Set Read to Block #1 */
            pNV_AppData_Wrt = pNV_AppData_2;                                    /* Set Write to Block #2 */
            p_CopyU16( (sizeof(APP_PARAMS_t) / sizeof(uint16_t)),
                       (uint16_t *)&AppData,
                       (const uint16_t *)pNV_AppData_Rd);
            AppData.u4PageID = ((APP_PARAMS_t *)pNV_AppData_Rd)->u4PageID ^ 1U;   /* 2nd Block becomes newest */
        }
    }
    else
    {
        /* 1st Block is invalid; Check 2nd Block */
        IO_MLX16_ITC_PEND0_S = B_MLX16_ITC_PEND0_EE_SH_ECC;
        if ( (NV_CalcCRC( (const uint16_t *)pNV_AppData_2, (sizeof(APP_PARAMS_t) / sizeof(uint16_t))) == 0xFFU) &&
             ((IO_MLX16_ITC_PEND0_S & B_MLX16_ITC_PEND0_EE_SH_ECC) == 0U) )
        {
            /* 2nd Block is valid; Use it! */
            pNV_AppData_Rd = pNV_AppData_2;                                     /* Set Read to Block #2 */
            pNV_AppData_Wrt = pNV_AppData_1;                                    /* Set Write to Block #1 */
            p_CopyU16( (sizeof(APP_PARAMS_t) / sizeof(uint16_t)),
                       (uint16_t *)&AppData,
                       (const uint16_t *)pNV_AppData_Rd);
            AppData.u4PageID = ((APP_PARAMS_t *)pNV_AppData_Rd)->u4PageID;      /* 1st Block becomes newest */
        }
        else
        {
            /* Both Blocks are invalid */
            pNV_AppData_Rd = (uint16_t *)&DefAppParams;                         /* Read default parameters */
            pNV_AppData_Wrt = pNV_AppData_1;                                    /* Set Write to Block #1 */
            p_CopyU16( (sizeof(APP_PARAMS_t) / sizeof(uint16_t)),
                       (uint16_t *)&AppData,
                       (const uint16_t *)pNV_AppData_Rd);
        }
    }
#if (_SUPPORT_APP_USER_MODE != FALSE)
    EXIT_SECTION(); /*lint !e438 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
#else  /* (_SUPPORT_DUAL_APP_BLOCK != FALSE) */
    p_CopyU16( (sizeof(APP_PARAMS_t) / sizeof(uint16_t)),
               (uint16_t *)&AppData,
               (const uint16_t *)pNV_AppData_Rd);
    AppData.u4PageID = 0U;
#endif /* (_SUPPORT_DUAL_APP_BLOCK != FALSE) */
    {
        uint16_t u16WrtCntL = AppData.u16WrtCntL;
        AppData.u16WrtCntL = AppData.u16WrtCntL + NV_TemperatureBasedWriteCountIncrease();
        if (AppData.u16WrtCntL < u16WrtCntL)
        {
            AppData.u4WrtCntH = AppData.u4WrtCntH + 1U;
        }
    }
    /* Modify Application data */
    AppData.u16ParamLSW = u16ParamLSW;
    AppData.u16ParamMSW = u16ParamMSW;

    /* Write System RAM Application Data to Non Volatile Memory */
    AppData.u8CRC8 = 0x00U;
    AppData.u8CRC8 = 0xFFU -
                     (uint8_t)NV_CalcCRC( (const uint16_t *)&AppData,
                                          (sizeof(APP_PARAMS_t) / sizeof(uint16_t)));

#if (_SUPPORT_NV_EMERGENCY_STORE != FALSE)
    /* Prior to the block write, first check if data is same; Skip physical block write */
    if (p_CompareU64( (uint16_t *)pNV_AppData_Wrt, (uint16_t *)&AppData) == FALSE)  /* Verify failed */
    {
#if (_SUPPORT_APP_USER_MODE != FALSE)
        ENTER_SECTION(SYSTEM_MODE); /*lint !e534 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
        while (IO_GET(EEPROM_FLASH, EE_BUSY)!= 0u)
        {
            p_NV_BusyChecks();
        }

        IO_MLX16_ITC_MASK2_S = B_MLX16_ITC_MASK2_EE_COMPLETE;                   /* Note: All MASK's are disabled, except EE_COMPLETE */
        IO_MLX16_ITC_PRIO2_S = (IO_MLX16_ITC_PRIO2_S & ~M_MLX16_ITC_PRIO2_EE_COMPLETE) | (3U - 3U);  /* Priority #3 */
        __asm__ ("mov UPR, #0x3" :::);
        IO_EEPROM_FLASH_EE_DATA_ERROR = (IO_EEPROM_FLASH_EE_DATA_ERROR |
                                         (B_EEPROM_FLASH_EE_DATA_CORRUPTED_2 |
                                          B_EEPROM_FLASH_EE_SBE_2 |
                                          B_EEPROM_FLASH_EE_DATA_CORRUPTED_1 |
                                          B_EEPROM_FLASH_EE_SBE_1));
        IO_EEPROM_FLASH_EE_CTRL_S =
            (IO_EEPROM_FLASH_EE_CTRL_S & ~(M_EEPROM_FLASH_EE_W_MODE | M_EEPROM_FLASH_EE_WE_KEY)) | (C_EE_WRT_KEY << 4);
        p_CopyU64( (uint16_t *)pNV_AppData_Wrt, (const uint16_t *)&AppData);
        IO_PORT_CLOCK_CTRL_S = B_PORT_CLOCK_CTRL_AC_SEL;
        if ( (IO_TRIM_RCO32M & B_TRIM_RCO32M_LOCK) == 0U)
        {
            /* Minimum RCO32 Clock */
            IO_TRIM_RCO32M = 0U;
        }
#if (_SUPPORT_APP_USER_MODE != FALSE)
        EXIT_SECTION(); /*lint !e438 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
        __asm__ ("HALT\n\t" :::);
    }

#else  /* (_SUPPORT_NV_EMERGENCY_STORE != FALSE) */
    (void)NV_WriteBlock( (const uint16_t)pNV_AppData_Wrt, (uint16_t *)&AppData,
                         (sizeof(APP_PARAMS_t) / sizeof(uint16_t)), FALSE);
#endif /* (_SUPPORT_NV_EMERGENCY_STORE != FALSE) */

} /* End of NV_AppStore() */

#if (_SUPPORT_BOOTLOADER != BOOTLOADER_NONE)
/*!*************************************************************************** *
 * NV_BootLoader
 * \brief   Write boot-loader relevant data (address)
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u8Address: 8-bit DUT address
 * \return  -
 * *************************************************************************** *
 * \details Write boot-loader dependent data to Non Volatile Memory
 * *************************************************************************** *
 * - Call Hierarchy: DfrDiagReadByIdentifier()
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 1 (NV_CalcCRC())
 * *************************************************************************** */
void NV_BootLoader(uint8_t u8Address)
{
    /* Melexis LIN Boot-loader */
    BOOTLOADER_PARAMS_t BootLoader;
    BootLoader.u16ReservedA = 0x0000U;
    BootLoader.u16ReservedB = 0x0000U;
    BootLoader.u16ReservedC = 0x0000U;
    BootLoader.u8Address = u8Address;
    BootLoader.u8CRC = 0x00U;
    BootLoader.u8CRC = 0xFFU -
                       (uint8_t)NV_CalcCRC( (const uint16_t *)&BootLoader,
                                            (sizeof(BOOTLOADER_PARAMS_t) / sizeof(uint16_t)));
    NV_WriteBlock(ADDR_NV_BL, (uint16_t *)&BootLoader, (sizeof(BOOTLOADER_PARAMS_t) / sizeof(uint16_t)), FALSE);
} /* End of NV_BootLoader() */
#endif /* (_SUPPORT_BOOTLOADER != BOOTLOADER_NONE) */

/*!*************************************************************************** *
 * NV_COMPLETE_ISR
 * \brief   Non Volatile Memory Write completion.
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details
 * *************************************************************************** *
 * - Call Hierarchy: IRQ
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 0
 * *************************************************************************** */
__attribute__((interrupt)) void NV_COMPLETE_ISR(void)
{
    NOP();
}
#endif /* (_SUPPORT_APP_SAVE != FALSE) */

/* EOF */
