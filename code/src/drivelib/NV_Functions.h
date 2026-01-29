/*!*************************************************************************** *
 * \file        NV_Functions.h
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
 * ************************************************************************** */

#ifndef DRIVE_LIB_NV_FUNCTIONS_H_
#define DRIVE_LIB_NV_FUNCTIONS_H_

/*!*************************************************************************** *
 *                           INCLUDES                                          *
 * *************************************************************************** */
#include "AppBuild.h"                                                           /* Application Build */

#include "drivelib/NV_UserPage.h"                                               /* Non Volatile Memory User Application Page Layout */

/*!*************************************************************************** */
/*                           DEFINITIONS                                       */
/* *************************************************************************** */
#if !defined (__MLX81339__)
#define C_NV_WRT_KEY            0x07U                                           /*!< Non Volatile Memory Write Key */
#define SZ_NV_BLOCK             0x0008U                                         /*!< Non Volatile Memory Block size */
#else  /* !defined (__MLX81339__) */
#define SZ_NV_BLOCK             0x0080U                                         /*!< Non Volatile Memory Block size */
#endif /* !defined (__MLX81339__) */

/* Write NV ID's */
#define ID_NV_HDR               0x0001U                                         /*!< Bit 0: HEADER */
#if (LIN_COMM != FALSE)
#define ID_NV_STD_LIN           0x0002U                                         /*!< Bit 1: Standard LIN */
#if (LINPROT == LIN2X_HVAC52)
#define ID_NV_ENH_LIN           0x0004U                                         /*!< Bit 2: Enhanced LIN */
#endif /* (LINPROT == LIN2X_HVAC52) */
#if (_SUPPORT_UDS != FALSE)
#define ID_NV_UDS_LIN           0x0008U                                         /*!< Bit 3: UDS LIN */
#endif /* (_SUPPORT_UDS != FALSE) */
#endif /* (LIN_COMM != FALSE) */
#define ID_NV_EOL               0x0010U                                         /*!< Bit 4: End-of-Line */
#define ID_NV_ACT_PARAMS        0x0020U                                         /*!< Bit 5: Actuator parameters */
#define ID_NV_SENSOR_PARAMS     0x0040U                                         /*!< Bit 6: Sensor parameters */
#define ID_NV_ACT_STALL         0x0080U                                         /*!< Bit 7: Stall Detector parameters */

#define ID_NV_PERIODIC          0x0020U                                         /*!< Bit 3 */

/* Error NV blocks */
#if (LIN_COMM != FALSE)
#define C_ERR_NV_LIN_STD_1      0x0001U                                         /*!< Bit 0: Error LIN Standard #1 */
#define C_ERR_NV_LIN_STD_2      0x0002U                                         /*!< Bit 1: Error LIN Standard #2 */
#if (LINPROT == LIN2X_HVAC52)
#define C_ERR_NV_LIN_ENH_1      0x0004U                                         /*!< Bit 2: Error LIN Enhanced #1 */
#define C_ERR_NV_LIN_ENH_2      0x0008U                                         /*!< Bit 3: Error LIN Enhanced #2 */
#endif /* (LINPROT == LIN2X_HVAC52) */
#define C_ERR_NV_HDR_RST_COUNT  0x0010U                                         /*!< Bit 4: Error Header; Reset Count */
#if (_SUPPORT_UDS != FALSE)
#define C_ERR_NV_LIN_UDS        0x0020U                                         /*!< Bit 5: Error LIN UDS */
#endif /* (_SUPPORT_UDS != FALSE) */
#else  /* (LIN_COMM != FALSE) */
#define C_ERR_NV_HDR_RST_COUNT  0x0010U                                         /*!< Bit 4: Error Header; Reset Count */
#endif /* (LIN_COMM != FALSE) */
#define C_ERR_NV_EOL            0x0040U                                         /*!< Bit 6: Error EOL */
#if (_SUPPORT_APP_SAVE != FALSE)
#define C_ERR_NV_APP_PARAMS     0x0080U                                         /*!< Bit 7: Error APP parameters */
#endif /* (_SUPPORT_APP_SAVE != FALSE) */
#define C_ERR_NV_ACT_PARAMS     0x0100U                                         /*!< Bit 8: Error Actuator parameters */
#define C_ERR_NV_SENSOR         0x0200U                                         /*!< Bit 9: Error Sensor parameters */
#define C_ERR_NV_ACT_STALL      0x0400U                                         /*!< Bit 10: Error Stall detector parameters */
#define C_ERR_NV_I2C_PARAMS     0x0800U                                         /*!< Bit 11: I2C Parameters */
#define C_ERR_NV_CAN_PARAMS     0x1000U                                         /*!< Bit 12: CAN Parameters */
#define C_ERR_NV_HDR_KEEP_COUNT 0x8000U                                         /*!< Bit 15: Error Header; Keep Count */


#define C_NV_WRT_LIN_ID_1       0x0001U                                         /*!< Bit 0: Sub-ID for Block #1 */
#define C_NV_WRT_LIN_ID_2       0x0002U                                         /*!< Bit 1: Sub-ID for Block #2 */
#define C_NV_WRT_LIN_ID_ALL     (C_NV_WRT_LIN_ID_1 | C_NV_WRT_LIN_ID_2)         /*!< Bit0+1: Sub-ID for Block #1 & #2 */
#define C_NV_WRT_FORCE          0x0004U                                         /*!< Bit 2: Force write */

#if defined (__MLX81160__)
#define C_DEF_TRIM_ADC          0x0003U                                         /*!< ADC Reference selection */
#elif defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
/* Default TRIM values, in case Non Volatile Memory is corrupted */
#define C_DEF_TRIM_IILD         0x0005U                                         /*!< Internal Inter-Lock Delay: 150ns delay, based on RCO32 */
#define C_DEF_TRIM_ADC          0x0003U                                         /*!< ADC Reference selection */
#endif

/*! Structure definition for Non Volatile Memory Structure blocks to store address, size and error bit-mask */
typedef struct
{
    uint16_t u16Address;                                                        /**< Non Volatile Memory Address */
    uint16_t u16Size;                                                           /**< Non Volatile Memory Structure size */
    uint16_t u16ErrorCode;                                                      /**< Error bit-mask */
} NV_CRC;

/*!*************************************************************************** *
 *                           GLOBAL VARIABLES                                  *
 * *************************************************************************** */
#pragma space dp                                                                /* __TINY_SECTION__ */
#pragma space none                                                              /* __TINY_SECTION__ */

#pragma space nodp                                                              /* __NEAR_SECTION__ */
#if (_SUPPORT_CPFREQ_TEMPCOMP != FALSE)
extern uint16_t g_u16CP_FreqTrim_RT;                                            /*!< Charge-pump clock frequency trim value at RT (Only: MLX8133xBB) */
#endif /* (_SUPPORT_CPFREQ_TEMPCOMP != FALSE) */
#pragma space none                                                              /* __NEAR_SECTION__ */

/*!*************************************************************************** */
/*                           GLOBAL FUNCTIONS                                  */
/* *************************************************************************** */
#if (_SUPPORT_NV_TYPE == C_NV_FLASH)
extern void NV_Init(void);                                                      /*!< Initialise the NV module */
#endif /* (_SUPPORT_NV_TYPE == C_NV_FLASH) */
extern uint16_t NV_CheckCRC(void);
extern uint16_t NV_WriteBlock(const uint16_t u16Address, uint16_t *pu16Data, uint16_t u16SizeW, uint16_t u16Force);
#if (LIN_COMM != FALSE)
extern uint16_t NV_WriteLIN_STD(volatile STD_LIN_PARAMS_t *pLinData, uint16_t u16BlockID);
#if (LINPROT == LIN2X_HVAC52)
extern uint16_t NV_WriteLIN_ENH(volatile ENH_LIN_PARAMS_t *pLinData, uint16_t u16BlockID);
#endif /* (LINPROT == LIN2X_HVAC52) */
#if (_SUPPORT_UDS != FALSE)
extern uint16_t NV_WriteUDS(volatile UDS_LIN_PARAMS_t* pUdsData);
#endif /* (_SUPPORT_UDS != FALSE) */
#endif /* (LIN_COMM != FALSE) */
#if (_SUPPORT_APP_SAVE != FALSE)
extern uint16_t NV_WriteAPP(volatile APP_PARAMS_t* pAppData);
#endif /* (_SUPPORT_APP_SAVE != FALSE) */
extern uint16_t NV_WriteEOL(volatile APP_EOL_t* pEolData);
extern uint16_t NV_WriteActParams(volatile ACT_PARAMS_t* pActData);
#if (_SUPPORT_TRIAXIS_MLX90363 != FALSE) || (_SUPPORT_TRIAXIS_MLX9038x != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90367 != FALSE) || (_SUPPORT_TRIAXIS_MLX90372 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90377 != FALSE) || (_SUPPORT_TRIAXIS_MLX90395 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90421 != FALSE) || (_SUPPORT_TRIAXIS_MLX90422 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90425 != FALSE) || (_SUPPORT_TRIAXIS_MLX90426 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90427 != FALSE) || \
    (_SUPPORT_INDUCTIVE_POS_SENSOR_MLX90513 != FALSE) || \
    (_SUPPORT_DUAL_HALL_LATCH_MLX92251 != FALSE) || (_SUPPORT_DUAL_HALL_LATCH_MLX92255 != FALSE) || \
    (_SUPPORT_FOC_MODE != FOC_MODE_NONE)
extern uint16_t NV_WriteSensor(volatile SENSOR_t* pSensor);
#endif /* (_SUPPORT_TRIAXIS_MLX90363 != FALSE) || (_SUPPORT_TRIAXIS_MLX9038x != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90367 != FALSE) || (_SUPPORT_TRIAXIS_MLX90372 != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90377 != FALSE) || (_SUPPORT_TRIAXIS_MLX90395 != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90421 != FALSE) || (_SUPPORT_TRIAXIS_MLX90422 != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90425 != FALSE) || (_SUPPORT_TRIAXIS_MLX90426 != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90427 != FALSE) || \
        * (_SUPPORT_INDUCTIVE_POS_SENSOR_MLX90513 != FALSE) || \
        * (_SUPPORT_DUAL_HALL_LATCH_MLX92251 != FALSE) || (_SUPPORT_DUAL_HALL_LATCH_MLX92255 != FALSE) || \
        * (_SUPPORT_FOC_MODE != FOC_MODE_NONE) */
extern uint16_t NV_WriteActStall(volatile ACT_STALL_t* pActStall);
extern uint16_t NV_WritePatch(volatile PATCH_HDR_t* pPatch);
#if (_SUPPORT_NV_LOG_ERROR != FALSE)
extern uint16_t NV_WriteErrorLog(volatile APP_LOG_t *pLogData);
#endif /* (_SUPPORT_NV_LOG_ERROR != FALSE) */
#if (I2C_COMM != FALSE)
extern uint16_t NV_WriteI2cParams(volatile STD_I2C_PARAMS_t* pI2cData);
#endif /* (I2C_COMM != FALSE) */
#if (CAN_COMM != FALSE)
uint16_t NV_WriteCanParams(volatile STD_CAN_PARAMS_t* pCanData);
#endif /* (CAN_COMM != FALSE) */
extern uint16_t NV_WriteUserDefaults(uint16_t u16BlockIdMask);
extern void p_CopyU16(const uint16_t u16Size, uint16_t *pu16Dest, const uint16_t *pu16Src);
extern uint16_t NV_MlxCalib(void);
#if (_SUPPORT_APP_SAVE != FALSE)
extern void NV_AppStore(uint16_t u16ParamLSW, uint16_t u16ParamMSW);
#endif /* (_SUPPORT_APP_SAVE != FALSE) */
#if (_SUPPORT_BOOTLOADER != BOOTLOADER_NONE)
extern void NV_BootLoader(uint8_t u8Address);
#endif /* (_SUPPORT_BOOTLOADER != BOOTLOADER_NONE) */

#endif /* DRIVE_LIB_NV_FUNCTIONS_H_ */
