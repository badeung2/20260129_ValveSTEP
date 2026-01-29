/*!************************************************************************** *
 * \file        LIN_MlxDiagnostics.c
 * \brief       MLX813xx LIN Diagnostics communication handling
 *
 * \note        Project MLX813xx
 *
 * \author      Marcel Braat
 *
 * \date        2017-08-15
 *
 * \version     2.0
 *
 * A list of functions:
 *  - Global Functions:
 *           -# DfrDiagMlxDebug                 (0xDB)
 *  - Internal Functions:
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
 *                          RAM     Flash   Non Volatile Memory
 * _SUPPORT_MLX_DEBUG_MODE:  14 B    916 B
 * *************************************************************************** */

/*!*************************************************************************** */
/*                           INCLUDES                                          */
/* *************************************************************************** */
#include "AppBuild.h"                                                           /* Application Build */

#if (LIN_COMM != FALSE) && ((_SUPPORT_MLX_DEBUG_MODE != FALSE) || (_SUPPORT_MLX_CHIP_STATUS != FALSE))

/* Application includes */
#include "../AppVersion.h"                                                      /* Application Version support */
#include "../ActADC.h"                                                          /* Application ADC support */
#if (_SUPPORT_START_TO_RUN != FALSE)
#include "../StartToRun.h"                                                      /* Start-to-Run support */
#endif /* (_SUPPORT_START_TO_RUN != FALSE) */

#if (_SUPPORT_FLASH_PRODUCTION_DATA != FALSE)
#include "camculib/FL_ProductionData.h"                                         /* Flash Production Data support */
#endif /* (_SUPPORT_FLASH_PRODUCTION_DATA != FALSE) */

/* Actuator Platform includes */
#include "drivelib/AppFunctions.h"                                              /* Application Functions support */
#include "drivelib/NV_Functions.h"                                              /* Non Volatile Memory Functions & Layout */
#include "drivelib/ErrorCodes.h"                                                /* Error-logging support */
#include "drivelib/GlobalVars.h"                                                /* Global Variables support */
#if (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT)
#include "drivelib/MotorDriver.h"                                               /* Motor driver support */
#elif (_SUPPORT_APP_TYPE == C_APP_RELAY)
#include "drivelib/RelayDriver.h"                                               /* Relay driver support */
#elif (_SUPPORT_APP_TYPE == C_APP_SOLENOID)
#include "drivelib/SolenoidDriver.h"                                            /* Solenoid Driver support */
#endif /* (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT) */
#if (_SUPPORT_MLX_DEBUG_MODE != FALSE) && ((_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT))
#include "drivelib/MotorStall.h"                                                /* Motor Stall Detectors */
#endif /* (_SUPPORT_MLX_DEBUG_MODE != FALSE) && ((_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT)) */
#if (_SUPPORT_MLX_DEBUG_MODE != FALSE) && ((_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT) || (_SUPPORT_APP_TYPE == C_APP_RELAY) || (_SUPPORT_APP_TYPE == C_APP_SOLENOID))
#include "drivelib/PID_Control.h"                                               /* PID support */
#endif /* (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT) || (_SUPPORT_APP_TYPE == C_APP_SOLENOID) */
#include "camculib/private_mathlib.h"                                           /* Private Math Library support */

#if (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
#include "senselib/Triaxis_MLX9038x.h"                                          /* Triaxis MLX9038x support */
#elif (_SUPPORT_TRIAXIS_MLX90377 != FALSE)
#include "senselib/Triaxis_MLX90377.h"                                          /* (PWM/SENT/SPC) Triaxis MLX90377 support */
#endif /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */

/* Communication Platform includes */
#include "commlib/LIN_Communication.h"                                          /* LIN Communication support */
#if (_SUPPORT_LIN_AA != FALSE)
#include "commlib/LIN_AutoAddressing.h"                                         /* LIN Auto-Addressing support */
#endif /* (_SUPPORT_LIN_AA != FALSE) */

/* CAMCU Platform includes */
#include <bist_inline_impl.h>
#include <bl_bist.h>
#include <bl_tools.h>                                                           /* Used by MLX16_RESET_SIGNED */
#include <fwversion.h>
#include <string.h>                                                             /* memmove */

/*!*************************************************************************** */
/*                           DEFINITIONS                                       */
/* *************************************************************************** */

/*!*************************************************************************** */
/*                           GLOBAL & LOCAL VARIABLES                          */
/* *************************************************************************** */
#pragma space dp                                                                /* __TINY_SECTION__ */
#pragma space none                                                              /* __TINY_SECTION__ */

#pragma space nodp                                                              /* __NEAR_SECTION__ */
#if (_SUPPORT_MLX_DEBUG_MODE != FALSE)
static uint16_t l_u16IoValue = 0x0000U;                                         /*!< Set I/O value (MMP190715-1) */
#endif /* (_SUPPORT_MLX_DEBUG_MODE != FALSE) */

#if (_SUPPORT_MLX_CHIP_STATUS != FALSE)
uint8_t g_u8MlxChipStatusM2SCounter = 0U;                                       /*!< Melexis Chip Status: M2S Counter */
#endif /* (_SUPPORT_MLX_CHIP_STATUS != FALSE) */

#if (_SUPPORT_MLX_CHIP_STATUS != FALSE) && (_SUPPORT_APP_TYPE == C_APP_RELAY)
extern uint16_t l_u16RelayOnTime;
extern uint16_t l_u16RelayOffTime;
extern uint16_t l_u16RelayOnCount;
extern uint16_t l_u16RelayOffCount;
#endif /* (_SUPPORT_MLX_CHIP_STATUS != FALSE) && (_SUPPORT_APP_TYPE == C_APP_RELAY) */
#pragma space none                                                              /* __NEAR_SECTION__ */

#if (_SUPPORT_FLASH_PRODUCTION_DATA != FALSE)
extern uint16_t _fw_prod_data_start1;                                           /**< linker symbol with start of the production data memory (Area #1) */
extern uint16_t _fw_prod_data_start2;                                           /**< linker symbol with start of the production data memory (Area #2) */
#endif /* (_SUPPORT_FLASH_PRODUCTION_DATA != FALSE) */

#if (_SUPPORT_MLX_DEBUG_MODE != FALSE)
/*                               ANA_OUTA,ANA_OUTB,ANA_OUTC,ANA_OUTD,ANA_OUTE,ANA_OUTF,ANA_OUTG,ANA_OUTH */
const uint16_t au16AnaOutRegs[8] = { 0x201CU, 0x201EU, 0x2020U, 0x204AU, 0x204CU, 0x204EU, 0x28CCU, 0x28CEU};  /*!< Melexis Diagnostics registers */

/*! Melexis Debug support sub-function mask */
const uint16_t tMlxDbgSupport[16] = {                                           /*!< Melexis Debug support sub-function mask */
    C_DBG_SUBFUNC_SUPPORT_0, C_DBG_SUBFUNC_SUPPORT_1, C_DBG_SUBFUNC_SUPPORT_2, C_DBG_SUBFUNC_SUPPORT_3,  /*!< Melexis Debug support sub-function 0x, 1x, 2x, 3x */
    C_DBG_SUBFUNC_SUPPORT_4, C_DBG_SUBFUNC_SUPPORT_5, 0x0000U, 0x0000U,         /*!< Melexis Debug support sub-function 4x, 5x, 6x, 7x */
    0x0000U, 0x0000U, C_DBG_SUBFUNC_SUPPORT_A, C_DBG_SUBFUNC_SUPPORT_B,         /*!< Melexis Debug support sub-function 8x, 9x, Ax, Bx */
    C_DBG_SUBFUNC_SUPPORT_C, C_DBG_SUBFUNC_SUPPORT_D, C_DBG_SUBFUNC_SUPPORT_E, C_DBG_SUBFUNC_SUPPORT_F  /*!< Melexis Debug support sub-function Cx, Dx, Ex, Fx */
};
#endif /* (_SUPPORT_MLX_DEBUG_MODE != FALSE) */

#if (_SUPPORT_MLX_DEBUG_MODE != FALSE)
/*!*************************************************************************** *
 * DfrDiagMlxDebug
 * \brief   Melexis LIN Diagnostics frames
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] *pDiag: Pointer to LIN DFR message
 * \param   [in] u16PCI_SID: PCI and SID
 * \return  -
 * *************************************************************************** *
 * \details
 * Function-ID and Description
 * 0x00: Supported Function-ID's
 * 0x5D: Stall detector
 * 0xA1: LIN-AA BSM Ishunt #1,2 & 3 and flags (_SUPPORT_LIN_AA)
 * 0xA2: LIN-AA BSM Common-mode & Differential-mode levels 1 (_SUPPORT_LIN_AA)
 * 0xA3: LIN-AA BSM Common-mode & Differential-mode levels 2 (_SUPPORT_LIN_AA)
 * 0xA4: LIN-AA BSM Common-mode & Differential-mode levels 3 (_SUPPORT_LIN_AA)
 * 0xA5: Application Status
 * 0xA6: LIN Slave Baudrate
 * 0xA7: Re-start Auto Baud-rate
 * 0xAA: Resolver (9038x)
 * 0xAB: Hall latch: low & high period
 * 0xAD: Speed info
 * 0xAE: Ambient-environment: Temperature
 * 0xB0: Set Set Motor PWM Duty Cycle
 * 0xB1: Set Motor Driver Slew-rate
 * 0xB2: Set CPU Frequency
 * 0xB3: Set CPU Spread Spectrum
 * 0xB4: Set LIN Slew-rate
 * 0xB5: Set Driver Spread Spectrum
 * 0xB6: Set LIN Termination
 * 0xC0: Get CPU-clock
 * 0xC1: Get Chip-ID
 * 0xC2: Get HW/SW-ID of chip
 * 0xC4: Get Performance counter
 * 0xC5: Get _DEBUG options
 * 0xC6: Get _SUPPORT options
 * 0xC7: MLX4 F/W & Loader
 * 0xC8: Get Platform version)
 * 0xC9: Get application version
 * 0xCA: Get Melexis NVRAM page info
 * 0xCB: Get PID l_u16PidCtrlRatio & l_u16PID_I
 * 0xCC: Get NVRAM stored errorcode's
 * 0xCD: Clear NVRAM error-logging
 * 0xCE: Chip-environment: Temperature, Motor driver current, Supply-voltage
 * 0xCF: Chip functions
 * 0xD0-0xD7: Set ANA_OUT[A:H]
 * 0xD8: Set Micro-step
 * 0xEA: Erase Production Data
 * 0xEB: Write Production Data Page
 * 0xEC: Read Error Codes
 * 0xED: Read Non Volatile Memory
 * 0xEE: Write Non Volatile Memory
 * 0xEF: Start-to-Run functions
 * 0xFC: Clear Fatal-handler error logging
 * 0xFD: Get I/O-register value (16-bits)
 * 0xFE: Get Fatal-error: error-code, info and address
 * *************************************************************************** *
 * - Call Hierarchy: HandleDfrDiag()
 * - Cyclomatic Complexity: ??
 * - Nesting: ?
 * - Function calling: ?
 * *************************************************************************** */
void DfrDiagMlxDebug(DFR_DIAG *pDiag, uint16_t u16PCI_SID)
{
#if FALSE
    uint8_t u8PCI = (uint8_t)pDiag->byPCI;                                      /* TODO[MMP]: To avoid compiler bug (A-1/3) */
    if (CollectUDSData(pDiag, u16PCI_SID) != FALSE)
#else
    (void)u16PCI_SID;
#endif
    {
        uint8_t u8FunctionID;
        uint16_t u16SupplierID;
        /*  Single Frame
         *  +-----+-----+------+----------+----------+----------+----------+----------+
         *  | NAD | PCI |  SID |    D1    |    D2    |    D3    |    D4    |    D5    |
         *  +-----+-----+------+----------+----------+----------+----------+----------+
         *  | NAD | 0x06| Debug| Supplier | Supplier | Param #1 | Param #2 | Function |
         *  |     |     | 0xDB | ID (LSB) | ID (MSB) |          |          |    ID    |
         *  +-----+-----+------+----------+----------+----------+----------+----------+
         *
         *  Multi Frame (Not supported)
         *  +-----+-----+------+------+----------+----------+----------+----------+ +-----+-----+----------+----------+----------+----------+----------+----------+
         *  | NAD | PCI |  LEN |  SID |    D1    |    D2    |    D3    |    D4    | | NAD | PCI |    D1    |    D2    |    D3    |    D4    |    D5    |    D6    |
         *  +-----+-----+------+------+----------+----------+----------+----------+ +-----+-----+----------+----------+----------+----------+----------+----------+
         *  | NAD | 0x10|Length| Debug| Supplier | Supplier | Function | Param #1 | | NAD | 0x21| Param #2 | Param #3 | Param #4 | Param #5 | Param #6 | Param #7 |
         *  |     |     |      | 0xDB | ID (LSB) | ID (MSB) |    ID    |          | |     |     |          |          |          |          |          |          |
         *  +-----+-----+------+------+----------+----------+----------+----------+ +-----+-----+----------+----------+----------+----------+----------+----------+
         */
#if FALSE
        /* if ( (pDiag->byPCI & (uint8_t) 0xF0U) == (uint8_t) 0x00U ) */        /* TODO[MMP]: To avoid compiler bug (A-2/3) */
        if ( (u8PCI & (uint8_t)0xF0U) == (uint8_t)0x00U)                        /* TODO[MMP]: To avoid compiler bug (A-3/3) */
        {
            u16SupplierID = (((uint16_t)pDiag->u.SF.byD2) << 8) | ((uint16_t)pDiag->u.SF.byD1);
            u8FunctionID = (uint8_t)pDiag->u.SF.byD5;
        }
        else
        {
            u16SupplierID = (((uint16_t)au8UDS_DataBuffer[1]) << 8) | ((uint16_t)au8UDS_DataBuffer[0]);
            u8FunctionID = (uint8_t)au8UDS_DataBuffer[2];
        }
#else
        u16SupplierID = (((uint16_t)pDiag->u.SF.byD2) << 8) | ((uint16_t)pDiag->u.SF.byD1);
        u8FunctionID = (uint8_t)pDiag->u.SF.byD5;
#endif
        if ( (u8FunctionID >= C_DBG_SUBFUNC_NV_WR_IDX0) && (u8FunctionID <= C_DBG_SUBFUNC_NV_WR_IDX_MAX) )
        {
            /* Write Non Volatile Memory (max. 176 Bytes; 0x0840...0x8EF)
             *  +-----+-----+------+----------+----------+----------+----------+----------+
             *  | NAD | PCI |  SID |    D1    |    D2    |    D3    |    D4    |    D5    |
             *  +-----+-----+------+----------+----------+----------+----------+----------+
             *  | NAD | 0x06| Debug| W[index] | W[index] |W[index+1]|W[index+1]|   FUNC   |
             *  |     |     | 0xDB |  (LSB)   |  (MSB)   |   (LSB)  |   (MSB)  |0x04..0x5B|
             *  +-----+-----+------+----------+----------+----------+----------+----------+
             *  (No Response)
             *  Address = ADDR_NV_USER + (FUNC - 0x04) * 2 (Bit 0 = 0)
             */
            static uint16_t au16NvBlock[4];
            uint16_t u16Index = (pDiag->u.SF.byD5 - C_DBG_SUBFUNC_NV_WR_IDX0);  /* Index in Words */
            if ( (u16Index & 0x0002U) == 0x0000U)
            {
                /* First 4-bytes of Non Volatile Memory-block */
                au16NvBlock[0] = ((uint16_t)pDiag->u.SF.byD1) | (((uint16_t)pDiag->u.SF.byD2) << 8);
                au16NvBlock[1] = ((uint16_t)pDiag->u.SF.byD3) | (((uint16_t)pDiag->u.SF.byD4) << 8);
            }
            else
            {
                /* Second 4-bytes of Non Volatile Memory-block */
                const uint16_t u16Address = (uint16_t)(ADDR_NV_USER + ((u16Index & 0xFFFCU) << 1));   /* Non Volatile Memory 16-bit pointer */
                au16NvBlock[2] = ((uint16_t)pDiag->u.SF.byD1) | (((uint16_t)pDiag->u.SF.byD2) << 8);
                au16NvBlock[3] = ((uint16_t)pDiag->u.SF.byD3) | (((uint16_t)pDiag->u.SF.byD4) << 8);
                (void)NV_WriteBlock(u16Address, (uint16_t *)&au16NvBlock[0], 4U, FALSE);
            }
        }
        else if (u16SupplierID == C_SUPPLIER_ID)
        {
            g_DiagResponse.byNAD = g_u8NAD;
            g_DiagResponse.byPCI = 0x06U;
            g_DiagResponse.u.SF.byRSID = (uint8_t)C_RSID_MLX_DEBUG;
            if (u8FunctionID == (uint8_t)C_DBG_SUBFUNC_SUPPORT)
            {
                /* Get MLX Debug Support
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | PCI |  SID |    D1    |    D2    |    D3    |    D4    |    D5    |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | 0x06| Debug| Supplier | Supplier |   index  | Reserved |   FUNC   |
                 *  |     |     | 0xDB | ID (LSB) | ID (MSB) |          |   0xFF   |   0x00   |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 * Response
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | PCI | RSID |    D1    |    D2    |    D3    |    D4    |    D5    |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | 0x06| 0x1B |   index  |MLX DBG[i]|MLX DBG[i]| Reserved | Reserved |
                 *  |     |     |      |          |   (LSB)  |   (MSB)  |          |          |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 */
                uint16_t u16Index = (uint16_t)(pDiag->u.SF.byD3 & 0x0FU);
                StoreD1to2(tMlxDbgSupport[u16Index]);
            }
#if (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT) || (_SUPPORT_APP_TYPE == C_APP_SOLENOID)
            else if (u8FunctionID == (uint8_t)C_DBG_SUBFUNC_STALLDET)
            {
                /* Stall detector
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | PCI |  SID |    D1    |    D2    |    D3    |    D4    |    D5    |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | 0x06| Debug| Supplier | Supplier | Reserved | Reserved |   FUNC   |
                 *  |     |     | 0xDB | ID (LSB) | ID (MSB) |   0xFF   |   0xFF   |   0x5D   |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 * Response
                 *  +-----+-----+------+------------+------------+------------+------------+----------+
                 *  | NAD | PCI | RSID |     D1     |     D2     |     D3     |     D4     |    D5    |
                 *  +-----+-----+------+------------+------------+------------+------------+----------+
                 *  | NAD | 0x06| 0x1B |Stallcurrent|Stallcurrent|Motorcurrent|Motorcurrent|StallFlags|
                 *  |     |     |      |Thrshld(LSB)|Thrshld(LSB)|MovAvg (LSB)|MovAvg (MSB)|          |
                 *  +-----+-----+------+------------+------------+------------+------------+----------+
                 */
                LinDiag_MotorStall();
                g_u8BufferOutID = (uint8_t)QR_RFR_DIAG;                           /* LIN Output buffer is valid (RFR_DIAG) */
            }
#endif /* (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT) || (_SUPPORT_APP_TYPE == C_APP_SOLENOID) */
#if ((_SUPPORT_LIN_AA != FALSE) && (LIN_AA_INFO != FALSE))
            else if (u8FunctionID == (uint8_t)C_DBG_SUBFUNC_LINAA_1)
            {
                /* LIN-AA BSM Ishunt #1,2 & 3 and flags
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | PCI |  SID |    D1    |    D2    |    D3    |    D4    |    D5    |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | 0x06| Debug| Supplier | Supplier | Reserved | Reserved |   FUNC   |
                 *  |     |     | 0xDB | ID (LSB) | ID (MSB) |   0xFF   |   0xFF   |   0xA1   |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 * Response
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | PCI | RSID |    D1    |    D2    |    D3    |    D4    |    D5    |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | 0x06| 0x1B |   Step   |  Ishunt1 |  Ishunt2 |  Ishunt3 | AA-Flags |
                 *  |     |     |      |CycleCount|          |          |          |          |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 */
                PSNPD_DATA pSNPD_Data = &l_aSNPD_Data[g_u8SNPD_CycleCountComm];
                g_DiagResponse.u.SF.byD1 = (uint8_t)((pSNPD_Data->byStepAndFlags << 1) & 0xF0U) |
                                           (g_u8SNPD_CycleCountComm & 0x0FU);
                g_DiagResponse.u.SF.byD2 = pSNPD_Data->byIshunt1;
                g_DiagResponse.u.SF.byD3 = pSNPD_Data->byIshunt2;
                g_DiagResponse.u.SF.byD4 = pSNPD_Data->byIshunt3;
                g_DiagResponse.u.SF.byD5 = (pSNPD_Data->byStepAndFlags & 0x87U);
                g_u8SNPD_CycleCountComm++;
                if (g_u8SNPD_CycleCountComm >= LIN_AA_INFO_SZ)                  /* Don't increase index in case last AA-structure index */
                {
                    g_u8SNPD_CycleCountComm = 0U;
                }
                g_u8BufferOutID = (uint8_t)QR_RFR_DIAG;                           /* LIN Output buffer is valid (RFR_DIAG) */
            }
#if ((LIN_AA_INFO != FALSE) && (LIN_AA_SCREENTEST != FALSE))
            else if ( (u8FunctionID >= (uint8_t)C_DBG_SUBFUNC_LINAA_2) &&
                      (u8FunctionID <= (uint8_t)C_DBG_SUBFUNC_LINAA_4) )
            {
                /* LIN-AA BSM Common-mode & Differential-mode levels #1, #2 or #3
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | PCI |  SID |    D1    |    D2    |    D3    |    D4    |    D5    |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | 0x06| Debug| Supplier | Supplier | Reserved | Reserved |   FUNC   |
                 *  |     |     | 0xDB | ID (LSB) | ID (MSB) |   0xFF   |   0xFF   |0xA2/A3/A4|
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 * Response
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | PCI | RSID |    D1    |    D2    |    D3    |    D4    |    D5    |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | 0x06| 0x1B |CycleCount|CommonMode|CommonMode|DifferMode|DifferMode|
                 *  |     |     |      |          |   (LSB)  |   (MSB)  |   (LSB)  |   (MSB)  |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 */
                PSNPD_DATA pSNPD_Data = &l_aSNPD_Data[g_u8SNPD_CycleCountComm];
                uint16_t *pu16CMDM;
                g_DiagResponse.u.SF.byD1 = g_u8SNPD_CycleCountComm;
                if ( (u8FunctionID == (uint8_t)C_DBG_SUBFUNC_LINAA_2))
                {
                    pu16CMDM = (uint16_t *)&(pSNPD_Data->u16CM_1);
                }
                else if (u8FunctionID == (uint8_t)C_DBG_SUBFUNC_LINAA_3)
                {
                    pu16CMDM = (uint16_t *)&(pSNPD_Data->u16CM_2);
                }
                else /* if (u8FunctionID == (uint8_t) C_DBG_SUBFUNC_LINAA_4) */
                {
                    pu16CMDM = (uint16_t *)&(pSNPD_Data->u16CM_3);
                }
                StoreD2to5(pu16CMDM[0], pu16CMDM[1]);  /*lint !e415 */
            }
#endif /* ((LIN_AA_INFO != FALSE) && (LIN_AA_SCREENTEST != FALSE)) */
#endif /* (_SUPPORT_LIN_AA != FALSE) && (LIN_AA_INFO != FALSE) */
            else if (u8FunctionID == (uint8_t)C_DBG_SUBFUNC_APPLSTATE)
            {
                /* LIN-AA BSM Common-mode & Differential-mode levels #3
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | PCI |  SID |    D1    |    D2    |    D3    |    D4    |    D5    |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | 0x06| Debug| Supplier | Supplier | Reserved | Reserved |   FUNC   |
                 *  |     |     | 0xDB | ID (LSB) | ID (MSB) |   0xFF   |   0xFF   |   0xA5   |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 * Response
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | PCI | RSID |    D1    |    D2    |    D3    |    D4    |    D5    |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | 0x06| 0x1B |   Motor  |  Actual  |  Actual  |   Motor  |ErrorFlags|
                 *  |     |     |      |StatusMode| Pos (MSB)| Pos (LSB)|  Request |          |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 * ErrorFlags:
                 *  bit 7: Chip Reset occurred
                 *  bit 6: Stall occurred
                 *  bit 5: Emergency Run occurred
                 *  bit 4: Over-temperature
                 *  bit 3:2: Voltage (In-range, UV and OV)
                 *  bit 1:0: Electric Error (Ok, Error, Permanent)
                 */
#if (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT)
                g_DiagResponse.u.SF.byD1 = g_e8MotorStatus;
#elif (_SUPPORT_APP_TYPE == C_APP_RELAY)
#if (_SUPPORT_DUAL_RELAY == FALSE)
                g_DiagResponse.u.SF.byD1 = g_e8RelayStatus;
#else  /* (_SUPPORT_DUAL_RELAY == FALSE) */
                g_DiagResponse.u.SF.byD1 = (g_e8RelayStatusA & 0x0FU) | (g_e8RelayStatusB << 4);
#endif /* (_SUPPORT_DUAL_RELAY == FALSE) */
#endif /* (_SUPPORT_APP_TYPE) */
                {
#if (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT)
#if (_SUPPORT_MOTOR_POSITION == C_MOTOR_POS_ROTOR)
                    uint16_t u16CopyValue = g_u16ActualPosition;
#elif (_SUPPORT_MOTOR_POSITION == C_MOTOR_POS_SHAFT)
                    uint16_t u16CopyValue = g_u16ActualShaftAngle;
#else
                    uint16_t u16CopyValue = g_u16ActualMotorSpeedRPM;
#endif
#elif (_SUPPORT_APP_TYPE == C_APP_RELAY)
                    uint16_t u16CopyValue = 0U;
#elif (_SUPPORT_APP_TYPE == C_APP_SOLENOID)
                    uint16_t u16CopyValue = 0U;
#endif /* (_SUPPORT_APP_TYPE) */
                    g_DiagResponse.u.SF.byD2 = (uint8_t)(u16CopyValue & 0xFFU);
                    g_DiagResponse.u.SF.byD3 = (uint8_t)(u16CopyValue >> 8);
                }
                g_DiagResponse.u.SF.byD4 = (g_e8MotorRequest & 0x0FU);
                {
                    uint8_t u8D5 = ((g_e8ErrorVoltage & 0x03U) << 2);
                    if ( (g_e8ErrorElectric & (uint8_t)C_ERR_PERMANENT) != 0U)
                    {
                        u8D5 |= 0x02U;                                          /* Permanent error */
                    }
                    else if ( (g_e8ErrorElectric & (uint8_t) ~C_ERR_PERMANENT) != 0U)
                    {
                        u8D5 |= 0x01U;                                          /* Electric error */
                    }
                    if (g_e8ErrorOverTemperature != (uint8_t)C_ERR_OTEMP_NO)
                    {
                        u8D5 |= 0x10U;                                          /* Over-temperature error */
                    }
                    if (g_e8EmergencyRunOcc != (uint8_t)C_SAFETY_RUN_NO)
                    {
                        u8D5 |= 0x20U;                                          /* Emergency/Safety run */
                    }
                    if (g_u8StallOcc != FALSE)
                    {
                        u8D5 |= 0x40U;                                          /* (Unexpected) blockage/stall */
                    }
                    if (g_u8ChipResetOcc != FALSE)
                    {
                        u8D5 |= 0x80U;                                          /* Chip reset */
                    }
                    g_DiagResponse.u.SF.byD5 = u8D5;
                }
                g_u8BufferOutID = (uint8_t)QR_RFR_DIAG;                         /* LIN Output buffer is valid (RFR_DIAG) */
            }
            else if (u8FunctionID == (uint8_t)C_DBG_SUBFUNC_LIN_BAUDRATE)
            {
                /* LIN Slave baudrate
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | PCI |  SID |    D1    |    D2    |    D3    |    D4    |    D5    |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | 0x06| Debug| Supplier | Supplier | Reserved | Reserved |   FUNC   |
                 *  |     |     | 0xDB | ID (LSB) | ID (MSB) |   0xFF   |   0xFF   |   0xA6   |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 * Response
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | PCI | RSID |    D1    |    D2    |    D3    |    D4    |    D5    |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | 0x06| 0x1B |  MCU_PLL |NomLINBaud|NomLINBaud|ActLINBaud|ActLINBaud|
                 *  |     |     |      |   _MULT  |rate (LSB)|rate (MSB)|rate (LSB)|rate (MSB)|
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 */
                g_DiagResponse.u.SF.byD1 = (uint8_t)(PLL_FREQ / 250000UL);
                StoreD2to5(0xFFFFU, p_ml_GetBaudRate(MLX4_FPLL));
            }
            else if (u8FunctionID == (uint8_t)C_DBG_SUBFUNC_RESTART_AUTO_BAUDRATE)
            {
                /* Restart auto baudrate detection
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | PCI |  SID |    D1    |    D2    |    D3    |    D4    |    D5    |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | 0x06| Debug| Supplier | Supplier | Reserved | Reserved |   FUNC   |
                 *  |     |     | 0x1B | ID (LSB) | ID (MSB) |   0xFF   |   0xFF   |   0xA7   |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 * Response (none)
                 */
                (void)ml_SetAutoBaudRateMode(ML_ABR_ON_FIRST_FRAME);
                Set_Mlx4ErrorState(C_MLX4_STATE_IMMEDIATE_RST);                 /* Reset MLX4 too */
            }
#if (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
            else if (u8FunctionID == (uint8_t)C_DBG_SUBFUNC_RESOLVER)
            {
                if ( (pDiag->u.SF.byD3 & (uint8_t)CMD_MASK_RESOLVER) == (uint8_t)CMD_RESOLVER_GET_XY)
                {
                    /* Triaxis MLX9038x Get Resolver Data
                     *  +-----+-----+------+----------+----------+----------+----------+----------+
                     *  | NAD | PCI |  SID |    D1    |    D2    |    D3    |    D4    |    D5    |
                     *  +-----+-----+------+----------+----------+----------+----------+----------+
                     *  | NAD | 0x06| Debug| Supplier | Supplier | Reserved | Reserved |   FUNC   |
                     *  |     |     | 0xDB | ID (LSB) | ID (MSB) |   0xF8   |   0xFF   |   0xAA   |
                     *  +-----+-----+------+----------+----------+----------+----------+----------+
                     * Response
                     *  +-----+-----+------+----------+----------+----------+----------+----------+
                     *  | NAD | PCI | RSID |    D1    |    D2    |    D3    |    D4    |    D5    |
                     *  +-----+-----+------+----------+----------+----------+----------+----------+
                     *  | NAD | 0x06| 0x1B | Resolver | Resolver | Resolver | Resolver |  Command |
                     *  |     |     |      |  X (LSB) |  X (MSB) |  Y (LSB) |  Y (MSB) |   0x00   |
                     *  +-----+-----+------+----------+----------+----------+----------+----------+
                     */
                    Triaxis_MlxDiagnostics(CMD_RESOLVER_GET_XY);
                }
#if (_SUPPORT_CALIBRATION != FALSE)
                else if ( (pDiag->u.SF.byD3 & (uint8_t)CMD_MASK_RESOLVER) == (uint8_t)CMD_RESOLVER_CALIB)
                {
                    /* Triaxis MLX9038x calibration
                     *  +-----+-----+------+----------+----------+----------+----------+----------+
                     *  | NAD | PCI |  SID |    D1    |    D2    |    D3    |    D4    |    D5    |
                     *  +-----+-----+------+----------+----------+----------+----------+----------+
                     *  | NAD | 0x06| Debug| Supplier | Supplier | Reserved | Reserved |   FUNC   |
                     *  |     |     | 0x1B | ID (LSB) | ID (MSB) |   0xF9   |   0xFF   |   0xAA   |
                     *  +-----+-----+------+----------+----------+----------+----------+----------+
                     * Response: None
                     */
                    if ( (g_e8MotorRequest == (uint8_t)C_MOTOR_REQUEST_NONE) &&
                         ((g_e8CalibrationStep == (uint8_t)C_CALIB_NONE) ||
                          (g_e8CalibrationStep >= (uint8_t)C_CALIB_FAILED)) )
                    {
                        g_e8CalibrationStep = (uint8_t)C_CALIB_START;
                        g_e8MotorRequest = (uint8_t)C_MOTOR_REQUEST_CALIB_FACTORY;
                    }
                }
#endif /* (_SUPPORT_CALIBRATION != FALSE) */
                else if ( (pDiag->u.SF.byD3 & (uint8_t)CMD_MASK_RESOLVER) == (uint8_t)CMD_RESOLVER_CALIB_RESULT)
                {
                    /* Triaxis MLX9038x Calibration Result
                     *  +-----+-----+------+----------+----------+----------+----------+----------+
                     *  | NAD | PCI |  SID |    D1    |    D2    |    D3    |    D4    |    D5    |
                     *  +-----+-----+------+----------+----------+----------+----------+----------+
                     *  | NAD | 0x06| Debug| Supplier | Supplier | Reserved | Reserved |   FUNC   |
                     *  |     |     | 0xDB | ID (LSB) | ID (MSB) |   0xFA   |   0xFF   |   0xAA   |
                     *  +-----+-----+------+----------+----------+----------+----------+----------+
                     * Response
                     *  +-----+-----+------+----------+----------+----------+----------+----------+
                     *  | NAD | PCI | RSID |    D1    |    D2    |    D3    |    D4    |    D5    |
                     *  +-----+-----+------+----------+----------+----------+----------+----------+
                     *  | NAD | 0x06| 0x1B | Resolver | Resolver | AmplCorr | AmplCorr |  Command |
                     *  |     |     |      | X Offset | Y Offset |   (LSB)  |   (MSB)  |   0x02   |
                     *  +-----+-----+------+----------+----------+----------+----------+----------+
                     */
                    Triaxis_MlxDiagnostics(CMD_RESOLVER_CALIB_RESULT);
                }
                else if ( (pDiag->u.SF.byD3 & (uint8_t)CMD_MASK_RESOLVER) == (uint8_t)CMD_RESOLVER_CALIB_RESULT_ACT)
                {
                    /* Triaxis MLX9038x Calibration Result (Actuator)
                     *  +-----+-----+------+----------+----------+----------+----------+----------+
                     *  | NAD | PCI |  SID |    D1    |    D2    |    D3    |    D4    |    D5    |
                     *  +-----+-----+------+----------+----------+----------+----------+----------+
                     *  | NAD | 0x06| Debug| Supplier | Supplier | Reserved | Reserved |   FUNC   |
                     *  |     |     | 0xDB | ID (LSB) | ID (MSB) |   0xFB   |   0xFF   |   0xAA   |
                     *  +-----+-----+------+----------+----------+----------+----------+----------+
                     * Response
                     *  +-----+-----+------+----------+----------+----------+----------+----------+
                     *  | NAD | PCI | RSID |    D1    |    D2    |    D3    |    D4    |    D5    |
                     *  +-----+-----+------+----------+----------+----------+----------+----------+
                     *  | NAD | 0x06| 0x1B | Angle Off| Angle Off| Resolver |  Sense   | Reserved |
                     *  |     |     |      |   (LSB)  |   (MSB)  | Direction|Pole-Pairs|          |
                     *  +-----+-----+------+----------+----------+----------+----------+----------+
                     */
                    Triaxis_MlxDiagnostics(CMD_RESOLVER_CALIB_RESULT_ACT);
                }
            }
#endif /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */
#if ((_SUPPORT_HALL_LATCH != FALSE) || (_SUPPORT_DUAL_HALL_LATCH_MLX92251 != FALSE) || \
     (_SUPPORT_DUAL_HALL_LATCH_MLX92255 != FALSE) || (_SUPPORT_HALL_LATCH_MLX9227x != FALSE))
            else if (u8FunctionID == (uint8_t)C_DBG_SUBFUNC_HALL_LATCH)
            {
                /* Hall Latch low and high period
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | PCI |  SID |    D1    |    D2    |    D3    |    D4    |    D5    |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | 0x06| Debug| Supplier | Supplier | Reserved | Reserved |   FUNC   |
                 *  |     |     | 0xDB | ID (LSB) | ID (MSB) |   0xFF   |   0xFF   |   0xAB   |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 * Response
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | PCI | RSID |    D1    |    D2    |    D3    |    D4    |    D5    |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | 0x06| 0xDB | Low Time | Low Time | High Time| High Time| Reserved |
                 *  |     |     |      |   (LSB)  |   (MSB)  |   (LSB)  |   (MSB)  |  (0xFF)  |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 */
                StoreD1to4(0xFFFFU, 0xFFFFU);
            }
#elif (_SUPPORT_TRIAXIS_MLX90377 != FALSE)
            else if (u8FunctionID == (uint8_t)C_DBG_SUBFUNC_HALL_LATCH)
            {
                /* Hall Latch low and high period
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | PCI |  SID |    D1    |    D2    |    D3    |    D4    |    D5    |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | 0x06| Debug| Supplier | Supplier | Reserved | Reserved |   FUNC   |
                 *  |     |     | 0xDB | ID (LSB) | ID (MSB) |   0xFF   |   0xFF   |   0xAB   |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 * Response
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | PCI | RSID |    D1    |    D2    |    D3    |    D4    |    D5    |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | 0x06| 0xDB |   Angle  |   Angle  | Reserved | Reserved | Reserved |
                 *  |     |     |      |   (LSB)  |   (MSB)  |  (0xFF)  |  (0xFF)  |  (0xFF)  |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 */
                extern int16_t g_i16DeltaShaftAngle;                            /* TODO[MMP]: For debug purpose */

                StoreD1to4(GetShaftAngle(), g_i16DeltaShaftAngle);
            }
#endif /* ((_SUPPORT_HALL_LATCH != FALSE) || (_SUPPORT_DUAL_HALL_LATCH_MLX92251 != FALSE) || \
        *  (_SUPPORT_DUAL_HALL_LATCH_MLX92255 != FALSE) || (_SUPPORT_HALL_LATCH_MLX9227x != FALSE)) */
#if (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT)
            else if (u8FunctionID == (uint8_t)C_DBG_SUBFUNC_SPEED)
            {
                /* Target vs. Actual Speed
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | PCI |  SID |    D1    |    D2    |    D3    |    D4    |    D5    |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | 0x06| Debug| Supplier | Supplier | Reserved | Reserved |   FUNC   |
                 *  |     |     | 0xDB | ID (LSB) | ID (MSB) |   0xFF   |   0xFF   |   0xAD   |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 * Response
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | PCI | RSID |    D1    |    D2    |    D3    |    D4    |    D5    |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | 0x06| 0xDB | Reserved | TargetSpd| TargetSpd| ActualSpd| ActualSpd|
                 *  |     |     |      |  (0xFF)  |  (LSB)   |   (MSB)  |   (LSB)  |   (MSB)  |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 */
#if (_SUPPORT_SPEED_CTRL != FALSE)
#if (_SUPPORT_VOLTAGE_CTRL != FALSE)
                if (g_e8ControlType == C_VOLTAGE_CTRL)
                {
                    StoreD2to5(g_u16TargetMotorVoltage, g_u16ActualMotorVoltage);
                }
                else
#endif /* (_SUPPORT_VOLTAGE_CTRL != FALSE) */
                {
                    StoreD2to5(g_u16TargetMotorSpeedRPM, g_u16ActualMotorSpeedRPM);
                }
#elif (_SUPPORT_VOLTAGE_CTRL != FALSE)
                StoreD2to5(g_u16TargetMotorVoltage, g_u16ActualMotorVoltage);
#else
                StoreD2to5(0xFFFFU, 0xFFFFU);
#endif /* (_SUPPORT_SPEED_CTRL != FALSE) */
            }
#endif /* (_SUPPORT_APP_TYPE) */
            else if (u8FunctionID == (uint8_t)C_DBG_SUBFUNC_AMBJENV)
            {
                /* Motor-voltage
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | PCI |  SID |    D1    |    D2    |    D3    |    D4    |    D5    |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | 0x06| Debug| Supplier | Supplier | Reserved | Reserved |   FUNC   |
                 *  |     |     | 0xDB | ID (LSB) | ID (MSB) |   0xFF   |   0xFF   |   0xAE   |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 * Response
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | PCI | RSID |    D1    |    D2    |    D3    |    D4    |    D5    |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | 0x06| 0x1B |  Ambient |Motor-volt|Motor-volt|Phase-volt|Phase-volt|
                 *  |     |     |      |Temperatur|  (LSB)   |   (MSB)  |   (LSB)  |   (MSB)  |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 */
#if (_SUPPORT_AMBIENT_TEMP != FALSE)
                uint16_t u16Value = (uint16_t)(Get_AmbientTemperature() + C_TEMPOFF);  /* Ambient Junction temperature + offset (C_TEMPOFF); Range: -C_TEMPOFF .. +(255-C_TEMPOFF) */
                g_DiagResponse.u.SF.byD1 = (uint8_t)(u16Value & 0xFFU);
#endif /* (_SUPPORT_AMBIENT_TEMP != FALSE) */
                StoreD2to5(Get_MotorVoltage(), 0xFFFFU);
            }
#if (_SUPPORT_FOC_MODE != FOC_MODE_NONE)
            else if (u8FunctionID == (uint8_t)C_DBG_SUBFUNC_FOC_IV)
            {
                /* FOC: I/V-angle
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | PCI |  SID |    D1    |    D2    |    D3    |    D4    |    D5    |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | 0x06| Debug| Supplier | Supplier | Reserved | Reserved |   FUNC   |
                 *  |     |     | 0xDB | ID (LSB) | ID (MSB) |   0xFF   |   0xFF   |   0xB0   |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 * Response (Lead-Angle)
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | PCI | RSID |    D1    |    D2    |    D3    |    D4    |    D5    |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | 0x06| 0xDB | Reserved | Target LA| Target LA| Actual LA| Actual LA|
                 *  |     |     |      |   0xFF   |   (LSB)  |   (MSB)  |   (LSB)  |   (MSB)  |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 * Response (Flux)
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | PCI | RSID |    D1    |    D2    |    D3    |    D4    |    D5    |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | 0x06| 0xDB | Reserved |ThrshldFlx|ThrshldFlx|ActualFlux|ActualFlux|
                 *  |     |     |      |   0xFF   |   (LSB)  |   (MSB)  |   (LSB)  |   (MSB)  |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 */
#if (_SUPPORT_STALLDET_LA != FALSE)
                StoreD2to5( (uint16_t)Get_TgtLoadAngle(), (uint16_t)Get_ActLoadAngleLPF());  /* MMP220720-1 */
#elif (_SUPPORT_STALLDET_FLUX != FALSE)
                extern uint16_t l_u16Flux;
                extern uint16_t l_u16StallThresholdFlux;
                StoreD2to5(l_u16StallThresholdFlux, l_u16Flux);                 /* MMP220725-1 */
#else
                StoreD2to5(0xFFFFU, 0xFFFFU);
#endif /* _SUPPORT_STALLDET_LA */
            }
#elif (_SUPPORT_STALLDET_LA != FALSE)
            else if (u8FunctionID == (uint8_t)C_DBG_SUBFUNC_FOC_IV)
            {
                /* Stall Detector LA Threshold-Angle and Actual-Angle
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | PCI |  SID |    D1    |    D2    |    D3    |    D4    |    D5    |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | 0x06| Debug| Supplier | Supplier | Reserved | Reserved |   FUNC   |
                 *  |     |     | 0xDB | ID (LSB) | ID (MSB) |   0xFF   |   0xFF   |   0xB0   |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 * Response
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | PCI | RSID |    D1    |    D2    |    D3    |    D4    |    D5    |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | 0x06| 0xDB | Reserved |ThrhdAngle|ThrhdAngle| ActAngle | ActAngle |
                 *  |     |     |      |  (0xFF)  |  (LSB)   |   (MSB)  |   (LSB)  |   (MSB)  |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 */
                 StoreD2to5(l_i16StallThresholdLA, l_i16ActLoadAngleLPF);
            }
#else  /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) */
            else if (u8FunctionID == (uint8_t)C_DBG_SUBFUNC_SET_MPWM_DC)
            {
                /* Set Motor PWM Duty Cycle
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | PCI |  SID |    D1    |    D2    |    D3    |    D4    |    D5    |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | 0x06| Debug| Supplier | Supplier |  PWM1-DC |  PWM2-DC |   FUNC   |
                 *  |     |     | 0xDB | ID (LSB) | ID (MSB) |  0-100%  |  0-100%  |   0xB0   |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 * (No response)
                 */
                uint16_t u16PwmU = 0U;
                uint16_t u16PwmV = 0U;
                if ( (pDiag->u.SF.byD3 & 0x7FU) <= 100U)
                {
                    u16PwmU = p_MulDivU16_U16byU16byU16(pDiag->u.SF.byD3, PWM_REG_PERIOD, 200U);
                }
                if ( (pDiag->u.SF.byD3 & 0x7FU) <= 100U)
                {
                    u16PwmV = p_MulDivU16_U16byU16byU16(pDiag->u.SF.byD4, PWM_REG_PERIOD, 200U);
                }
                IO_PWM_SLAVE1_LT = (uint16_t)u16PwmV;
                IO_PWM_SLAVE2_LT = 0U;
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81350__)
                IO_PWM_SLAVE3_LT = 0U;
#endif /* defined (__MLX81330__) || (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81350__) */
                IO_PWM_MASTER1_LT = (uint16_t)u16PwmU;                          /* Master must be modified at last (value is not important) */
#if (C_MOTOR_PHASES == 3)
#if defined (__MLX81160__)
#if (_SUPPORT_EVB == MLX81160EVB_3P_3N)
                IO_PWM_SLAVE3_LT = (uint16_t)u16PwmV;
                IO_PWM_SLAVE4_LT = 0U;
                IO_PWM_MASTER2_LT = (uint16_t)u16PwmU;                          /* Master must be modified at last (value is not important) */
                DRVCFG_PWM_3P_3N();
                DRVCFG_ENA_3P_3N();                                             /* Enable the driver and the PWM phase R, S and T and U, V and W */
#elif (_SUPPORT_DUAL_BLDC != FALSE)
                DRVCFG_PWM_RST_UVW();
                DRVCFG_ENA_RST_UVW();                                           /* Enable the driver and the PWM phase R, S and T and U, V and W */
#elif (_SUPPORT_1ST_BLDC != FALSE)
                DRVCFG_PWM_RST();
                DRVCFG_ENA_RST();                                               /* Enable the driver and the PWM phase R, S and T */
#else  /* (_SUPPORT_1ST_BLDC != FALSE) */
                DRVCFG_PWM_UVW();
                DRVCFG_ENA_UVW();                                               /* Enable the driver and the PWM phase U, V and W */
#endif
#elif defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81350__)
                DRVCFG_PWM_UVW();
                DRVCFG_ENA_UVW();                                               /* Enable the driver and the PWM phase W, V and U */
#elif defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
                DRVCFG_PWM_UVW();
                DRVCFG_ENA();                                                   /* Enable the driver and the PWM phase W, V and U */
#endif
#elif defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81350__)
                DRVCFG_PWM_TUVW();
                DRVCFG_ENA_TUVW();                                              /* Enable the driver and the PWM phase W, V and U */
#endif /* (C_MOTOR_PHASES == 3) */
            }
#endif /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) */
#if defined (__MLX81160__) || defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81350__)
            else if (u8FunctionID == (uint8_t)C_DBG_SUBFUNC_SET_DRV_SR)
            {
                /* Set Motor Driver Slew-rate
                 *  +-----+-----+------+----------+----------+-----------+----------+----------+
                 *  | NAD | PCI |  SID |    D1    |    D2    |     D3    |    D4    |    D5    |
                 *  +-----+-----+------+----------+----------+-----------+----------+----------+
                 *  | NAD | 0x06| Debug| Supplier | Supplier |OPT|SUP|S/R|  DRV-CLK |   FUNC   |
                 *  |     |     | 0xDB | ID (LSB) | ID (MSB) |0-3|0-3|0-F| 4x(0-255)|   0xB1   |
                 *  +-----+-----+------+----------+----------+-----------+----------+----------+
                 *  D3[7:6] = IO_PORT_DRV_OUT.M_PORT_DRV_OUT_DRVMOD_OPTION
                 *  D3[5:4] = IO_TRIM1_DRV.M_TRIM1_DRV_TRIM_DRVSUP
                 *  D3[3:0] = IO_TRIM2_DRV.M_TRIM2_DRV_TRIM_SLWRT
                 *  D4[7:0] * 4 = IO_TRIM1_DRV.M_TRIM1_DRV_PRE_TRIM_DRVMOD_CPCLK
                 * Response
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | PCI | RSID |    D1    |    D2    |    D3    |    D4    |    D5    |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | 0x06| 0x1B |  DRV_OUT | TRIM1_DRV| TRIM1_DRV| TRIM2_DRV| TRIM2_DRV|
                 *  |     |     |      |   (MSB)  |   (LSB)  |   (MSB)  |   (LSB)  |   (MSB)  |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 */
                if ( (pDiag->u.SF.byD3 != 0xFFU) || (pDiag->u.SF.byD4 != 0xFFU) )
                {
                    /* Bit 3:0 of D3 is DRV Slew-rate */
                    IO_TRIM2_DRV = (IO_TRIM2_DRV & ~M_TRIM2_DRV_TRIM_SLWRT) | (pDiag->u.SF.byD3 & 0x0FU);
                    /* Bit 5:4 of D3 is DRV Supply level; D4 * 4 is DRV-Clock */
#if defined (__MLX81350__)
                    IO_TRIM1_DRV = (IO_TRIM1_DRV & ~M_TRIM1_DRV_PRE_TRIM_DRVMOD_CPCLK) | (((uint16_t)pDiag->u.SF.byD4) << 2);
                    IO_TRIM2_DRV = (IO_TRIM2_DRV & ~M_TRIM2_DRV_TRIM_DRVSUP) | ((uint16_t)(pDiag->u.SF.byD3 & 0x30U) << 6);
#else  /* defined (__MLX81350__) */
                    IO_TRIM1_DRV = (IO_TRIM1_DRV & ~(M_TRIM1_DRV_PRE_TRIM_DRVMOD_CPCLK | M_TRIM1_DRV_TRIM_DRVSUP)) |
                                   (((uint16_t)pDiag->u.SF.byD4) << 4) | ((pDiag->u.SF.byD3 & 0x30U) >> 4);
                    IO_PORT_DRV_OUT = (IO_PORT_DRV_OUT & ~M_PORT_DRV_OUT_DRVMOD_OPTION) |
                                      (((uint16_t)(pDiag->u.SF.byD3 & 0xC0U)) << (9 - 6));
#endif /* defined (__MLX81350__) */
                }
                g_DiagResponse.u.SF.byD1 = (uint8_t)(IO_PORT_DRV_OUT >> 8);
                StoreD2to5(IO_TRIM1_DRV, IO_TRIM2_DRV);
            }
#endif /* defined (__MLX81160__) || defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81350__) */
            else if (u8FunctionID == (uint8_t)C_DBG_SUBFUNC_SET_CPU_FREQ)
            {
                /* Set CPU Frequency
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | PCI |  SID |    D1    |    D2    |    D3    |    D4    |    D5    |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | 0x06| Debug| Supplier | Supplier | CPU-FREQ | CPU-FEAT |   FUNC   |
                 *  |     |     | 0xDB | ID (LSB) | ID (MSB) |  250kHz  |     ?    |   0xB2   |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 * Response
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | PCI | RSID |    D1    |    D2    |    D3    |    D4    |    D5    |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | 0x06| 0x1B | Reserved |  RCO32M  |  RCO32M  | Reserved | Reserved |
                 *  |     |     |      |   0xFF   |   (LSB)  |   (MSB)  |   (LSB)  |   (MSB)  |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 */
                IO_TRIM_RCO32M = (IO_TRIM_RCO32M & ~M_TRIM_RCO32M_TR_RCO32M_IN) | (((uint16_t)pDiag->u.SF.byD3) << 2);
                StoreD2to5(IO_TRIM_RCO32M, 0xFFFFU);
            }
            else if (u8FunctionID == (uint8_t)C_DBG_SUBFUNC_SET_CPU_SSCM)
            {
                /* Set CPU Spread Spectrum
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | PCI |  SID |    D1    |    D2    |    D3    |    D4    |    D5    |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | 0x06| Debug| Supplier | Supplier | CPU-SSCM | CPU-SSCM |   FUNC   |
                 *  |     |     | 0xDB | ID (LSB) | ID (MSB) |   (LSB)  |   (MSB)  |   0xB3   |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 * Response
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | PCI | RSID |    D1    |    D2    |    D3    |    D4    |    D5    |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | 0x06| 0x1B | Reserved | SSCM_CONF| SSCM_CONF| STEP_CONF| STEP_CONF|
                 *  |     |     |      |   0xFF   |   (LSB)  |   (MSB)  |   (LSB)  |   (MSB)  |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 * If (SSCM_CONF[MSB] == 0), then SSCM_CONF[LSB] sets IO_PORT_SSCM_CONF
                 * otherwise: IO_PORT_STEP_CONF = SSCM_CONF[MSB:LSB]
                 */
                if (pDiag->u.SF.byD4 == 0x00U)
                {
                    /* Disable the spread spectrum modulation (and centred) */
                    IO_PORT_SSCM_CONF = pDiag->u.SF.byD3;
                }
                else
                {
                    /* Enable the spread spectrum modulation */
                    IO_PORT_STEP_CONF = (((uint16_t)pDiag->u.SF.byD4) << 8) | pDiag->u.SF.byD3;
                }
                StoreD2to5(IO_PORT_SSCM_CONF, IO_PORT_STEP_CONF);
            }
            else if (u8FunctionID == (uint8_t)C_DBG_SUBFUNC_SET_LIN_SR)
            {
                /* Set LIN Slew-rate
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | PCI |  SID |    D1    |    D2    |    D3    |    D4    |    D5    |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | 0x06| Debug| Supplier | Supplier |  LIN-SR  | LIN-FEAT |   FUNC   |
                 *  |     |     | 0xDB | ID (LSB) | ID (MSB) |    0-7   |     ?    |   0xB4   |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 * Response
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | PCI | RSID |    D1    |    D2    |    D3    |    D4    |    D5    |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | 0x06| 0x1B | Reserved |   RCO1M  |   RCO1M  | Reserved | Reserved |
                 *  |     |     |      |   0xFF   |   (LSB)  |   (MSB)  |   (LSB)  |   (MSB)  |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 */
                uint8_t u8LinSR = pDiag->u.SF.byD3 & 0x07U;
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81350__)
                IO_TRIM_RCO1M = (IO_TRIM_RCO1M & ~M_TRIM_RCO1M_PRE_TR_LIN_SLEWRATE) | (((uint16_t)u8LinSR) << 8);
#elif defined (__MLX81160__) || defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
                IO_TRIM_RCO1M = (IO_TRIM_RCO1M & ~M_TRIM_RCO1M_TR_LIN_SLEWRATE) | (((uint16_t)u8LinSR) << 8);
#endif
                StoreD2to5(IO_TRIM_RCO1M, 0xFFFFU);
            }
#if defined (__MLX81160__) || defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81350__)
            else if (u8FunctionID == (uint8_t)C_DBG_SUBFUNC_SET_DRV_SSCM)
            {
                /* Set Driver Spread Spectrum
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | PCI |  SID |    D1    |    D2    |    D3    |    D4    |    D5    |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | 0x06| Debug| Supplier | Supplier | DRV-SSCM | DRV-SSCM |   FUNC   |
                 *  |     |     | 0xDB | ID (LSB) | ID (MSB) |   (LSB)  |   (MSB)  |   0xB5   |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 * Response
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | PCI | RSID |    D1    |    D2    |    D3    |    D4    |    D5    |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | 0x06| 0x1B | Reserved |SSCM2_CONF|SSCM2_CONF|STEP2_CONF|STEP2_CONF|
                 *  |     |     |      |   0xFF   |   (LSB)  |   (MSB)  |   (LSB)  |   (MSB)  |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 */
                if (pDiag->u.SF.byD4 == 0x00U)
                {
                    /* Disable the spread spectrum modulation (and centred) */
                    IO_PORT_SSCM2_CONF = pDiag->u.SF.byD3;
                }
                else
                {
                    /* Enable the spread spectrum modulation */
                    IO_PORT_STEP2_CONF = (((uint16_t)pDiag->u.SF.byD4) << 8) | pDiag->u.SF.byD3;
                }
                StoreD2to5(IO_PORT_SSCM2_CONF, IO_PORT_STEP2_CONF);
            }
#endif /* defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81350__) */
            else if (u8FunctionID == (uint8_t)C_DBG_SUBFUNC_SET_LIN_TERM)
            {
                /* Set LIN Termination
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | PCI |  SID |    D1    |    D2    |    D3    |    D4    |    D5    |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | 0x06| Debug| Supplier | Supplier | LIN-TERM | LIN-FEAT |   FUNC   |
                 *  |     |     | 0xDB | ID (LSB) | ID (MSB) |    0-7   |     ?    |   0xB6   |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 * Response
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | PCI | RSID |    D1    |    D2    |    D3    |    D4    |    D5    |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | 0x06| 0xDB | Reserved |   RCO1M  |   RCO1M  | Reserved | Reserved |
                 *  |     |     |      |   0xFF   |   (LSB)  |   (MSB)  |   (LSB)  |   (MSB)  |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 */
                uint8_t u8LinTERM = pDiag->u.SF.byD3 & 0x07U;
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81350__)
                IO_TRIM_RCO1M = (IO_TRIM_RCO1M & ~M_TRIM_RCO1M_PRE_TR_LIN_SLVTERM) | (((uint16_t)u8LinTERM) << 11);
#elif defined (__MLX81160__) || defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
                IO_TRIM_RCO1M = (IO_TRIM_RCO1M & ~M_TRIM_RCO1M_TR_LIN_SLVTERM) | (((uint16_t)u8LinTERM) << 11);
#endif
                StoreD2to5(IO_TRIM_RCO1M, 0xFFFFU);
            }
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81350__) || \
            (PLL_FREQ == 32000000UL) && (defined (__MLX81340__) || defined (__MLX81344__))
            else if (u8FunctionID == (uint8_t)C_DBG_SUBFUNC_MLX16_CLK)
            {
                /* Get MLX16 Clock
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | PCI |  SID |    D1    |    D2    |    D3    |    D4    |    D5    |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | 0x06| Debug| Supplier | Supplier | Reserved | Reserved |   FUNC   |
                 *  |     |     | 0xDB | ID (LSB) | ID (MSB) |   0xFF   |   0xFF   |   0xC0   |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 * Response
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | PCI | RSID |    D1    |    D2    |    D3    |    D4    |    D5    |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | 0x06| 0x1B |MLX16Clock|MLX16Clock| Reserved | Reserved | Reserved |
                 *  |     |     |      |[kHz](LSB)|[kHz](MSB)|   0xFF   |   0xFF   |   0xFF   |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 */
#if (PLL_FREQ == 24000000UL)
                int8_t i8OffsetClock = CalibrationParams.i8APP_TRIM08_OffClock24;
                uint8_t u8GainClockHigh = CalibrationParams.u8APP_TRIM07_GainClock24HighT;
                uint8_t u8GainClockLow = CalibrationParams.u8APP_TRIM07_GainClock24LowT;
                uint16_t u16RC_Clock = 24000U + (int16_t)(p_MulI32_I16byI16(i8OffsetClock, 24000) >> 11);
#elif (PLL_FREQ == 28000000UL)
                int8_t i8OffsetClock = CalibrationParams.i8APP_TRIM09_OffClock28;
                uint8_t u8GainClockHigh = CalibrationParams.u8APP_TRIM09_GainClock28HighT;
                uint8_t u8GainClockLow = CalibrationParams.u8APP_TRIM08_GainClock28LowT;
                uint16_t u16RC_Clock = 28000U + (int16_t)(p_MulI32_I16byI16(i8OffsetClock, 28000) >> 11);
#elif (PLL_FREQ == 32000000UL)
                int8_t i8OffsetClock = (int8_t)CalibrationParams.i8APP_TRIM11_OffClock32;
                uint8_t u8GainClockHigh = CalibrationParams.u8APP_TRIM10_GainClock32HighT;
                uint8_t u8GainClockLow = CalibrationParams.u8APP_TRIM10_GainClock32LowT;
                uint16_t u16RC_Clock = 32000U + (int16_t)(p_MulI32_I16byI16(i8OffsetClock, 32000) >> 11);
#endif
#if (PLL_FREQ == 24000000UL) || (PLL_FREQ == 28000000UL) || (PLL_FREQ == 32000000UL)
                int16_t i16ADC_Temp = (int16_t)(Get_RawTemperature() - Get_TempMidADC());
                int16_t i16Coef;
                if (i16ADC_Temp <= 0)
                {
                    /* ((dTemp * G_HT) * 1000)/2^17 --> ((dTemp * Gp) * 125)/16384 */
                    i16Coef = u8GainClockHigh;
                }
                else
                {
                    /* ((dTemp * G_LT) * 1000)/2^17 --> ((dTemp * Gain) * 125)/16384 */
                    i16Coef = u8GainClockLow;
                }
                i16Coef = (125 * i16Coef);
                u16RC_Clock += (int16_t)(p_MulI32_I16byI16(i16ADC_Temp, i16Coef) >> 14);
#elif (FPLL == 29500)
                uint16_t u16RC_Clock = 29500U;
#elif (FPLL == 40000)
                uint16_t u16RC_Clock = 40000U;
#else
                uint16_t u16RC_Clock = 0U;
#endif /* (PLL_FREQ == 24000000UL) || (PLL_FREQ == 28000000UL) || (PLL_FREQ == 32000000UL) */
                StoreD1to2(u16RC_Clock);
            }
#endif /* defined (__MLX81344__) && (PLL_FREQ == 32000000UL) */
            else if (u8FunctionID == (uint8_t)C_DBG_SUBFUNC_CHIPID)
            {
                /* Get Chip-ID
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | PCI |  SID |    D1    |    D2    |    D3    |    D4    |    D5    |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | 0x06| Debug| Supplier | Supplier |   index  | Reserved |   FUNC   |
                 *  |     |     | 0xDB | ID (LSB) | ID (MSB) |          |   0xFF   |   0xC1   |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 * Response
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | PCI | RSID |    D1    |    D2    |    D3    |    D4    |    D5    |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | 0x06| 0x1B |   index  | NVRAM[i] | NVRAM[i] |NVRAM[i+1]|NVRAM[i+1]|
                 *  |     |     |      |          |   (LSB)  |   (MSB)  |   (LSB)  |   (MSB)  |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 */
                uint16_t u16Index = (uint16_t)(pDiag->u.SF.byD3 & 0x02U);
                g_DiagResponse.u.SF.byD1 = (uint8_t)u16Index;
                {
                    const uint16_t *pu16NvramData = ((uint16_t *)ADDR_NV_CHIPID) + u16Index;  /* NVRAM 16-bit pointer */
                    StoreD2to5(pu16NvramData[0], pu16NvramData[1]);
                }
            }
#if (_SUPPORT_MLX_DEBUG_OPTIONS != FALSE)
            else if (u8FunctionID == (uint8_t)C_DBG_SUBFUNC_DEBUG_OPTIONS)
            {
                /* Get _DEBUG options
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | PCI |  SID |    D1    |    D2    |    D3    |    D4    |    D5    |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | 0x06| Debug| Supplier | Supplier | Reserved | Reserved |   FUNC   |
                 *  |     |     | 0xDB | ID (LSB) | ID (MSB) |   0xFF   |   0xFF   |   0xC5   |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 * Response
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | PCI | RSID |    D1    |    D2    |    D3    |    D4    |    D5    |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | 0x06| 0x1B |   DEBUG  |   DEBUG  |   DEBUG  |   DEBUG  |   DEBUG  |
                 *  |     |     |      |   (LSB)  |          |          |          |   (MSB)  |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 */
                g_DiagResponse.u.SF.byD1 = (uint8_t)(C_DIAG_RES
/*                                                    & ~(1U << 0) */           /* bit 0: IO[B:A]: See also _DEBUG_STARTUP in syslib.h */
/*                                                    & ~(1U << 1) */           /* bit 1: IO[B:A]: Various LIN-AA stages */
/*                                                    & ~(1U << 2) */           /* bit 2: IO[A]: PID Control period */
/*                                                    & ~(1U << 3) */           /* bit 3: TC1:   MLX16-HALT */
/*                                                    & ~(1U << 4) */           /* bit 4: IO[A]: Toggle when MLX4 resets */
/*                                                    & ~(1U << 5) */           /* bit 5: IO[B]: Toggle to show MLX16 performance */
/*                                                    & ~(1U << 6) */           /* bit 6: IO[A]: Toggle when status frame-ID is not correct 03 - 02 - 01 */
/*                                                    & ~(1U << 7) */           /* bit 7: IO[B:A]: Watchdog events */
                                                     );
                g_DiagResponse.u.SF.byD2 = (uint8_t)(C_DIAG_RES
/*                                                    & ~(1U << 0) */           /* bit 0: IO[A]: Stall A to B and visa versa */
/*                                                    & ~(1U << 1) */           /* bit 1: IO[B]: FALSE: Normal operation; TRUE: Test stall detector "B" only */
/*                                                    & ~(1U << 2) */           /* bit 2: IO[A]: Zero-crossing */
/*                                                    & ~(1U << 3) */           /* bit 3: IO[A]: Zero-Crossing Blank Period (Fly-back Pulse) */
/*                                                    & ~(1U << 4) */           /* bit 4: IO[A]: Test-mode */
/*                                                    & ~(1U << 5) */           /* bit 5: IO[B]: Toggle when Diagnostics; Toggle @ UV */
                                                     );
                g_DiagResponse.u.SF.byD3 = (uint8_t)(C_DIAG_RES
/*                                                    & ~(1U << 0) */           /* bit 0: IO[A]: ADC_ISR period */
/*                                                    & ~(1U << 1) */           /* bit 1: IO[A]: Core-Timer ISR period */
#if (_DEBUG_COMMUT_ISR != FALSE)
                                                     & ~(1U << 2)               /* bit 2: IO[B]: Commutation-ISR period */
#endif
/*                                                    & ~(1U << 3) */           /* bit 3: IO[A]: Toggle during Hall-latch Timer2 capture */
/*                                                    & ~(1U << 4) */           /* bit 4: IO[A]: Toggle at entry and exit of DIAG_ISR */
/*                                                    & ~(1U << 5) */           /* bit 5: IO[C:A]: PWM-IN */
/*                                                    & ~(1U << 6) */           /* bit 6: IO[B:A]: NVRAM Update & Write */
/*                                                    & ~(1U << 7) */           /* bit 7: IO[B:A]: Flash CRC16 period */
                                                     );
                g_DiagResponse.u.SF.byD4 = (uint8_t)(C_DIAG_RES
/*                                                    & ~(1U << 0) */           /* bit 0: Save MLX-state in case of Fatal-error occurs in RAM */
#if (_DEBUG_MOTOR_CURRENT_FLT != FALSE)
                                                     & ~(1U << 1)               /* bit 1: Motor current filter debug-buffers */
#endif /* _DEBUG_MOTOR_CURRENT_FLT */
/*                                                    & ~(1U << 3) */           /* bit 3: Debugging HALL-Sensor(s) */
                                                     );
                g_DiagResponse.u.SF.byD5 = (uint8_t)C_DIAG_RES;

                g_u8BufferOutID = (uint8_t)QR_RFR_DIAG;                         /* LIN Output buffer is valid (RFR_DIAG) */
            }
#endif /* _SUPPORT_MLX_DEBUG_OPTIONS */
            else if (u8FunctionID == (uint8_t)C_DBG_SUBFUNC_SUPPORT_OPTIONS)
            {
                /* Get _SUPPORT options
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | PCI |  SID |    D1    |    D2    |    D3    |    D4    |    D5    |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | 0x06| Debug| Supplier | Supplier | Reserved | Reserved |   FUNC   |
                 *  |     |     | 0xDB | ID (LSB) | ID (MSB) |   0xFF   |   0xFF   |   0xC6   |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 * Response
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | PCI | RSID |    D1    |    D2    |    D3    |    D4    |    D5    |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | 0x06| 0x1B | SUPPORT  | SUPPORT  |  SUPPORT |  SUPPORT |  SUPPORT |
                 *  |     |     |      |   (LSB)  |          |          |          |   (MSB)  |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 */
                g_DiagResponse.u.SF.byD1 = (uint8_t)(C_DIAG_RES
#if (_SUPPORT_NV_BACKUP != FALSE)
                                                     & ~(1U << 0)               /* bit 0: NVRAM Backup support */
#endif /* (_SUPPORT_NV_BACKUP != FALSE) */
#if (C_NV_CURR_DIV == 1)
                                                     & ~(1U << 1)               /* bit 1: Double motor-current support */
#endif /* (C_NV_CURR_DIV == 1) */
/*                                                    & ~(1U << 2) */           /* bit 2: MLX Flash TMTR support */
/*                                                    & ~(1U << 3) */           /* bit 3: Watchdog reset fast recovery support */
#if (_SUPPORT_CRASH_RECOVERY != FALSE)
                                                     & ~(1U << 4)               /* bit 4: Crash recovery support */
#endif /* (_SUPPORT_CRASH_RECOVERY != FALSE) */
#if (_SUPPORT_CPU_HALT != FALSE)
                                                     & ~(1U << 6)               /* bit 6: MLX16 enters HALT during holding mode (power-safe) support */
#endif /* (_SUPPORT_CPU_HALT != FALSE) */
/*                                                    & ~(1U << 7) */           /* bit 7: Chip temperature profile check (dT/dt) support */
                                                     );
                g_DiagResponse.u.SF.byD2 = (uint8_t)(C_DIAG_RES
#if (_SUPPORT_AUTO_BAUDRATE != FALSE)
                                                     & ~(1U << 0)               /* bit 0: Auto-detection of baudrate support */
#endif /* (_SUPPORT_AUTO_BAUDRATE != FALSE) */
#if (_SUPPORT_LIN_UV != FALSE)
                                                     & ~(1U << 1)               /* bit 1: LIN UV check (reset Bus-time-out) support */
#endif /* (_SUPPORT_LIN_UV != FALSE) */
#if (_SUPPORT_LINNETWORK_LOADER != FALSE)
                                                     & ~(1U << 2)               /* bit 2: Network Flash-loading (NAD) support */
#endif /* (_SUPPORT_LINNETWORK_LOADER != FALSE) */
#if (_SUPPORT_COIL_UNIT_100mR != FALSE)
                                                     & ~(1U << 5)               /* bit 5: 100mR units for coil resistance */
#endif /* (_SUPPORT_COIL_UNIT_100mR != FALSE) */
#if (_SUPPORT_COIL_UNIT_10mR != FALSE)
                                                     & ~(1U << 6)               /* bit 6: 10mR units for coil resistance */
#endif /* (_SUPPORT_COIL_UNIT_10mR != FALSE) */
                                                     );
                g_DiagResponse.u.SF.byD3 = (uint8_t)(C_DIAG_RES
#if (_SUPPORT_PWM_MODE == TRIPLEPHASE_TWOPWM_INDEPENDENT_GND) || \
                                                     (_SUPPORT_PWM_MODE == BIPOLAR_PWM_SINGLE_INDEPENDENT_GND)
                                                     & ~(1U << 1)               /* bit 1: Two phase PWM support */
#endif /* (_SUPPORT_PWM_MODE == TRIPLEPHASE_TWOPWM_INDEPENDENT_GND) || (_SUPPORT_PWM_MODE == BIPOLAR_PWM_SINGLE_INDEPENDENT_GND) */
/*                                                    & ~(1U << 2) */           /* bit 2: Hardware control Motor-driver state support */
                                                     & ~(1U << 3)               /* bit 3: Increasing mPWM-DC at ramp-up support */
                                                     & ~(1U << 4)               /* bit 4: Decrease mPWM-DC at ramp-down support */
#if (_SUPPORT_MOTOR_SELFTEST != FALSE)
                                                     & ~(1U << 5)               /* bit 5: Motor driver check at POR support */
#endif /* (_SUPPORT_MOTOR_SELFTEST != FALSE) */
/*                                                    & ~(1U << 6) */           /* bit 6: Phase-short-to-GND detection support */
/*                                                    & ~(1U << 7) */           /* bit 7: Current-oscillation stall-detection support */
                                                     );
                g_DiagResponse.u.SF.byD4 = (uint8_t)(C_DIAG_RES
#if (_SUPPORT_DIAG_OC != FALSE)
                                                     & ~(1U << 0)               /* bit 0: Diagnostic OC support */
#endif /* (_SUPPORT_DIAG_OC != FALSE) */
#if (C_NV_CURR_DIV == 2)
                                                     & ~(1U << 1)               /* bit 1: Quadruple motor-current support */
#endif /* (C_NV_CURR_DIV == 2) */
/*                                                    & ~(1U << 2) */           /* bit 2: VDS Threshold */
/*                                                    & ~(1U << 3) */           /* bit 3: Double micro-steps */
/*                                                    & ~(1U << 4) */           /* bit 4: Hex-times motor-current support */
#if (C_NV_CURR_DIV == 3)
                                                     & ~(1U << 5)               /* bit 5: Octal-times motor-current support */
#endif /* (C_NV_CURR_DIV == 3) */
                                                     );
                g_DiagResponse.u.SF.byD5 = (uint8_t)(C_DIAG_RES
#if (_SUPPORT_TRIAXIS_MLX90363 != FALSE) || (_SUPPORT_TRIAXIS_MLX90377 != FALSE) || (_SUPPORT_TRIAXIS_MLX90395 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90421 != FALSE) || (_SUPPORT_TRIAXIS_MLX90422 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90425 != FALSE) || (_SUPPORT_TRIAXIS_MLX90426 != FALSE) || (_SUPPORT_TRIAXIS_MLX90427 != FALSE)
                                                     & ~(1U << 0)               /* bit 0: SPI Triaxis support (MLX90363) */
#endif /* (_SUPPORT_TRIAXIS_MLX90363 != FALSE) || (_SUPPORT_TRIAXIS_MLX90377 != FALSE) || (_SUPPORT_TRIAXIS_MLX90395 != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90421 != FALSE) || (_SUPPORT_TRIAXIS_MLX90422 != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90425 != FALSE) || (_SUPPORT_TRIAXIS_MLX90426 != FALSE) || (_SUPPORT_TRIAXIS_MLX90427 != FALSE) */
/*                                                    & ~(1U << 1) */           /* bit 1: Hall-Latch support */
#if (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
                                                     & ~(1U << 3)               /* bit 3: Analogue Triaxis Resolver support (MLX90380) */
#endif /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */
#if defined(__MLX81330__) || defined(__MLX81332__) || defined (__MLX81334__) || defined (__MLX81350__)   /* MMP200525-1 */
                                                     & ~(1U << 4)               /* bit 5:4: 0b11: Mulan2/3, 0b10: CAMCU 8133x, 0b01: CAMCU 8134x, 0b00: Reserved */
#elif defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
                                                     & ~(1U << 5)               /* bit 5:4: 0b11: Mulan2/3, 0b10: CAMCU 8133x, 0b01: CAMCU 8134x, 0b00: Reserved */
#elif defined (__MLX81160__)
                                                     & ~(3U << 4)               /* bit 5:4: 0b11: Mulan2/3, 0b10: CAMCU 8133x, 0b01: CAMCU 8134x, 0b00: CAMCU 81160 */
#endif
                                                     & ~(1U << 6)               /* bit 6: Non Volatile Memory */
                                                     );

                g_u8BufferOutID = (uint8_t)QR_RFR_DIAG;                         /* LIN Output buffer is valid (RFR_DIAG) */
            }
            else if (u8FunctionID == (uint8_t)C_DBG_SUBFUNC_MLX4_VERSION)
            {
                /* MLX4 version
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | PCI |  SID |    D1    |    D2    |    D3    |    D4    |    D5    |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | 0x06| Debug| Supplier | Supplier | Reserved | Reserved |   FUNC   |
                 *  |     |     | 0xDB | ID (LSB) | ID (MSB) |   0xFF   |   0xFF   |   0xC7   |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 * Response
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | PCI | RSID |    D1    |    D2    |    D3    |    D4    |    D5    |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | 0x06| 0x1B | MLX4 F/W | MLX4 F/W |  Loader  |  Loader  | Reserved |
                 *  |     |     |      |   (LSB)  |   (MSB)  |   (LSB)  |   (MSB)  |   0xFF   |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 */
                /* MMP190716-1 */
                uint16_t u16ValueR = p_GetExMem(C_ADDR_MLX4_FW_VERSION);
                uint16_t u16ValueF = p_GetExMem(C_ADDR_MLX4_LOADER_VERSION);
                StoreD1to4(u16ValueR, u16ValueF);
            }
            else if (u8FunctionID == (uint8_t)C_DBG_SUBFUNC_PLTF_VERSION)
            {
                /* Get Platform version
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | PCI |  SID |    D1    |    D2    |    D3    |    D4    |    D5    |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | 0x06| Debug| Supplier | Supplier | Reserved | Reserved |   FUNC   |
                 *  |     |     | 0xDB | ID (LSB) | ID (MSB) |   0xFF   |   0xFF   |   0xC8   |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 * Response
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | PCI | RSID |    D1    |    D2    |    D3    |    D4    |    D5    |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | 0x06| 0x1B | PLTF Ver | PLTF ver | PLTF ver | PLTF ver | Reserved |
                 *  |     |     |      |  (Major) |  (Minor) |   (Rev)  |  (Build) |   0xFF   |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 */
                uint32_t version = VERSION_getFwPltfVersion();
                StoreD1to4( (((version >> 24) & 0xFF) | (((version >> 16) & 0xFF) << 8)),
                            (((version >> 8) & 0xFF) | (((version >> 0) & 0xFF) << 8)));
            }
            else if (u8FunctionID == (uint8_t)C_DBG_SUBFUNC_APP_VERSION)
            {
                /* Get Application version
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | PCI |  SID |    D1    |    D2    |    D3    |    D4    |    D5    |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | 0x06| Debug| Supplier | Supplier |   index  | Reserved |   FUNC   |
                 *  |     |     | 0xDB | ID (LSB) | ID (MSB) |          |   0xFF   |   0xC9   |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 * Response
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | PCI | RSID |    D1    |    D2    |    D3    |    D4    |    D5    |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | 0x06| 0x1B | Appl ver | Appl ver | Appl ver | Appl ver | Appl ver |
                 *  |     |     |      |  (Major) |  (Minor) | (Rev LSB)| (Rev MSB)|   0xFF   |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 */
                if (pDiag->u.SF.byD3 == 0U)
                {
                    StoreD1to4( (__APP_VERSION_MAJOR__ | (__APP_VERSION_MINOR__ << 8)), __APP_VERSION_REVISION__);
                }
                else if (pDiag->u.SF.byD3 == 1U)
                {
                    StoreD1to4(p_GetExMem(C_ADDR_APP_STRING), p_GetExMem(C_ADDR_APP_STRING + 2U));
                }
                else if (pDiag->u.SF.byD3 == 2U)
                {
                    StoreD1to4(p_GetExMem(C_ADDR_APP_STRING + 4U), p_GetExMem(C_ADDR_APP_STRING + 6U));
                }
            }
            else if (u8FunctionID == (uint8_t)C_DBG_SUBFUNC_MLXPAGE)
            {
                /* Get Melexis NVRAM page info
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | PCI |  SID |    D1    |    D2    |    D3    |    D4    |    D5    |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | 0x06| Debug| Supplier | Supplier |   index  | Reserved |   FUNC   |
                 *  |     |     | 0xDB | ID (LSB) | ID (MSB) |          |   0xFF   |   0xCA   |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 * Response
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | PCI | RSID |    D1    |    D2    |    D3    |    D4    |    D5    |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | 0x06| 0x1B |   index  | NVRAM[i] | NVRAM[i] |NVRAM[i+1]|NVRAM[i+1]|
                 *  |     |     |      |          |   (LSB)  |   (MSB)  |   (LSB)  |   (MSB)  |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 */
                uint16_t u16Index = (uint16_t)(pDiag->u.SF.byD3 & 0x3EU);
                g_DiagResponse.u.SF.byD1 = (uint8_t)u16Index;
                {
                    const uint16_t *pu16NvramData = ((uint16_t *)ADDR_NV_MLX_CALIB) + u16Index;  /* NVRAM 16-bit pointer */
                    StoreD2to5(pu16NvramData[0], pu16NvramData[1]);
                }
            }
#if (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT) || \
    (_SUPPORT_APP_TYPE == C_APP_RELAY) || (_SUPPORT_APP_TYPE == C_APP_SOLENOID)
            else if (u8FunctionID == (uint8_t)C_DBG_SUBFUNC_MLXPID)
            {
                /* Get PID info
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | PCI |  SID |    D1    |    D2    |    D3    |    D4    |    D5    |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | 0x06| Debug| Supplier | Supplier | Reserved | Reserved |   FUNC   |
                 *  |     |     | 0xDB | ID (LSB) | ID (MSB) |   0xFF   |   0xFF   |   0xCB   |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 * Response
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | PCI | RSID |    D1    |    D2    |    D3    |    D4    |    D5    |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | 0x06| 0x1B | Reserved | CtrlRatio| CtrlRatio|   PID_I  |   PID_I  |
                 *  |     |     |      |          |   (LSB)  |   (MSB)  |   (LSB)  |   (MSB)  |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 */
#if (_SUPPORT_SPEED_CTRL != FALSE) && (_SUPPORT_APP_TYPE != C_APP_RELAY)
                StorePID_Info();
#endif /* (_SUPPORT_SPEED_CTRL != FALSE) && (_SUPPORT_APP_TYPE != C_APP_RELAY) */
                g_u8BufferOutID = (uint8_t)QR_RFR_DIAG;                         /* LIN Output buffer is valid (RFR_DIAG) */
            }
#endif /* (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT) || (_SUPPORT_APP_TYPE == C_APP_RELAY) || (_SUPPORT_APP_TYPE == C_APP_SOLENOID) */
#if (_SUPPORT_NV_LOG_ERROR != FALSE)
            else if (u8FunctionID == (uint8_t)C_DBG_SUBFUNC_NV_ERRORCODES)
            {
                /* Get Non Volatile Memory log
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | PCI |  SID |    D1    |    D2    |    D3    |    D4    |    D5    |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | 0x06| Debug| Supplier | Supplier |   index  | Reserved |   FUNC   |
                 *  |     |     | 0xDB | ID (LSB) | ID (MSB) |          |   0xFF   |   0xCC   |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 * Response
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | PCI | RSID |    D1    |    D2    |    D3    |    D4    |    D5    |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | 0x06| 0x1B |   Index  | Error[0] | Error[1] | Error[2] | Error[3] |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 */
                uint8_t u8Index = pDiag->u.SF.byD3;
                g_DiagResponse.u.SF.byD1 = u8Index;
                g_DiagResponse.u.SF.byD2 = GetNV_LogError(u8Index++);           /* Oldest Error-code */
                g_DiagResponse.u.SF.byD3 = GetNV_LogError(u8Index++);
                g_DiagResponse.u.SF.byD4 = GetNV_LogError(u8Index++);
                g_DiagResponse.u.SF.byD5 = GetNV_LogError(u8Index++);
                g_u8BufferOutID = (uint8_t)QR_RFR_DIAG;                         /* LIN Output buffer is valid (RFR_DIAG) */
            }
            else if (u8FunctionID == (uint8_t)C_DBG_SUBFUNC_CLR_NV_ERRORCODES)
            {
                /* Get Non Volatile Memory log
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | PCI |  SID |    D1    |    D2    |    D3    |    D4    |    D5    |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | 0x06| Debug| Supplier | Supplier | Reserved | Reserved |   FUNC   |
                 *  |     |     | 0x1B | ID (LSB) | ID (MSB) |   0xFF   |   0xFF   |   0xCD   |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 * No Response
                 */
                ClearNV_LogError();
            }
#endif /* (_SUPPORT_NV_LOG_ERROR != FALSE) */
            else if (u8FunctionID == (uint8_t)C_DBG_SUBFUNC_CHIPENV)
            {
                /* Chip-environment: Temperature, Motor driver current, Supply-voltage
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | PCI |  SID |    D1    |    D2    |    D3    |    D4    |    D5    |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | 0x06| Debug| Supplier | Supplier | Reserved | Reserved |   FUNC   |
                 *  |     |     | 0xDB | ID (LSB) | ID (MSB) |   0xFF   |   0xFF   |   0xCE   |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 * Response
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | PCI | RSID |    D1    |    D2    |    D3    |    D4    |    D5    |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | 0x06| 0x1B | Chip-Temp|  Current |  Current |  Voltage |  Voltage |
                 *  |     |     |      |          |   (LSB)  |   (MSB)  |   (LSB)  |   (MSB)  |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 */
                uint16_t u16Value = (uint16_t)(Get_ChipTemperature() + C_TEMPOFF);  /* Chip Junction temperature + offset (C_TEMPOFF); Range: -C_TEMPOFF .. +(255-C_TEMPOFF) */
                g_DiagResponse.u.SF.byD1 = (uint8_t)(u16Value & 0xFFU);
                if (pDiag->u.SF.byD3 == 1U)
                {
#if (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT)
                    StoreD2to5(Get_MotorCurrentMovAvgxN_mA(), Get_MotorVoltage());  /* Motor driver current [mA] & Motor voltage [10mV] */
#elif (_SUPPORT_APP_TYPE == C_APP_RELAY)
                    StoreD2to5(0U, Get_MotorVoltage());                         /* Motor driver current [mA] & Motor voltage [10mV] */
#endif /* (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT) */
                }
                else
                {
                    StoreD2to5(Get_MotorCurrent_mA(), Get_SupplyVoltage());     /* Motor driver current [mA] & Supply voltage [10mV] */
                }
            }
            else if (u8FunctionID == (uint8_t)C_DBG_SUBFUNC_FUNC)
            {
                /* Chip functions
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | PCI |  SID |    D1    |    D2    |    D3    |    D4    |    D5    |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | 0x06| Debug| Supplier | Supplier | Function | Function |   FUNC   |
                 *  |     |     | 0xDB | ID (LSB) | ID (MSB) | ID (LSB) | ID (MSB) |   0xCF   |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 * (No response)
                 */
                uint16_t u16FunctionID = (((uint16_t)pDiag->u.SF.byD4) << 8) | ((uint16_t)pDiag->u.SF.byD3);

                if (u16FunctionID == C_DBG_DBGFUNC_RESET)
                {
                    /* Function ID = Chip reset */
                    AppReset();
                }
                else if (u16FunctionID == C_DBG_DBGFUNC_ENTER_EPM)
                {
                    g_u8LinInFrameBufState = (uint8_t)C_LIN_IN_FREE;            /* MMP190116-2 */
                    MLX16_RESET_SIGNED(C_CHIP_STATE_PPM_CMD_EPM);
                }
#if defined (__MLX81160A01__) || defined (__MLX81330B02__) || defined (__MLX81332B02__) || defined (__MLX81334A01__) || \
                defined (__MLX81340B01__) || defined (__MLX81344B01__) || defined (__MLX81346B01__)
                else if (u16FunctionID == C_DBG_DBGFUNC_LOCK_PPM)               /* MMP210701-1 */
                {
                    /* Place Patch in Non Volatile Memory
                     * [CRC8] [LEN] PROJECT_ID[LSB] PROJECT_ID[MSB] [ADDR_LSB] [ADDR_MSB] [INSTR_LSB] [INSTR_MSB] */
                    PATCH_HDR_t Patch;

                    SetupDiagResponse(g_u8NAD, pDiag->u.SF.bySID,
                                      (uint8_t)C_ERRCODE_PENDING);              /* Status = Pending */
#if defined (__MLX81160A01__)
                    Patch.u8CRC = 0xAEU;                                        /* CRC */
                    Patch.u8Length = 0x04U;                                     /* Patch Length */
                    Patch.u16PatchProjID = PROJECT_ID;                          /* Project ID */
                    Patch.u16PatchAddress0 = 0x39F7U;                           /* Patch address */
                    Patch.u16PatchInstruction0 = 0x0005U;                       /* Patch instruction: "Jump $+5" */
#elif defined (__MLX81330B02__)
                    Patch.u8CRC = 0x8DU;                                        /* CRC */
                    Patch.u8Length = 0x04U;                                     /* Patch Length */
                    Patch.u16PatchProjID = PROJECT_ID;                          /* Project ID */
                    Patch.u16PatchAddress0 = 0x2B25U;                           /* Patch address */
                    Patch.u16PatchInstruction0 = 0x0005U;                       /* Patch instruction: "Jump $+5" */
#elif defined (__MLX81332B02__)
                    Patch.u8CRC = 0x89U;                                        /* CRC */
                    Patch.u8Length = 0x04U;                                     /* Patch Length */
                    Patch.u16PatchProjID = PROJECT_ID;                          /* Project ID */
                    Patch.u16PatchAddress0 = 0x2C2BU;                           /* Patch address */
                    Patch.u16PatchInstruction0 = 0x0005U;                       /* Patch instruction: "Jump $+5" */
#elif defined (__MLX81334A01__)
                    Patch.u8CRC = 0x3EU;                                        /* CRC */
                    Patch.u8Length = 0x04U;                                     /* Patch Length */
                    Patch.u16PatchProjID = PROJECT_ID;                          /* Project ID */
                    Patch.u16PatchAddress0 = 0x2985U;                           /* Patch address */
                    Patch.u16PatchInstruction0 = 0x0005U;                       /* Patch instruction: "Jump $+5" */
#elif defined (__MLX81340B01__)
                    Patch.u8CRC = 0xA2U;                                        /* CRC */
                    Patch.u8Length = 0x04U;                                     /* Patch Length */
                    Patch.u16PatchProjID = PROJECT_ID;                          /* Project ID */
                    Patch.u16PatchAddress0 = 0x55EFU;                           /* Patch address */
                    Patch.u16PatchInstruction0 = 0x0005U;                       /* Patch instruction: "Jump $+5" */
#elif defined (__MLX81344B01__)
                    Patch.u8CRC = 0x41U;                                        /* CRC */
                    Patch.u8Length = 0x04U;                                     /* Patch Length */
                    Patch.u16PatchProjID = PROJECT_ID;                          /* Project ID */
                    Patch.u16PatchAddress0 = 0x574FU;                           /* Patch address */
                    Patch.u16PatchInstruction0 = 0x0005U;                       /* Patch instruction: "Jump $+5" */
#elif defined (__MLX81346B01__)
                    Patch.u8CRC = 0x0BU;                                        /* CRC */
                    Patch.u8Length = 0x04U;                                     /* Patch Length */
                    Patch.u16PatchProjID = PROJECT_ID;                          /* Project ID */
                    Patch.u16PatchAddress0 = 0x5781U;                           /* Patch address */
                    Patch.u16PatchInstruction0 = 0x0005U;                       /* Patch instruction: "Jump $+5" */
#else
#error "ERROR: Lock PPM not supported."
#endif
                    if (NV_WritePatch(&Patch) == C_ERR_NONE)
                    {
                        SetupDiagResponse(g_u8NAD, pDiag->u.SF.bySID,
                                          (uint8_t)C_ERRCODE_POSITIVE_RESPONSE);    /* Status = Positive feedback */
                    }
                    else
                    {
                        SetupDiagResponse(g_u8NAD, pDiag->u.SF.bySID,
                                          (uint8_t)C_ERRCODE_COND_SEQ);         /* Status = Negative feedback */
                    }
                }
                else if (u16FunctionID == C_DBG_DBGFUNC_UNLOCK_PPM)
                {
                    /* Remove Patch from Non Volatile Memory
                     * 0xFF - 0x00 - 0x00 - 0x00 - 0x00 - 0x00 - 0x00 - 0x00 */
                    PATCH_HDR_t Patch;

                    SetupDiagResponse(g_u8NAD, pDiag->u.SF.bySID,
                                      (uint8_t)C_ERRCODE_PENDING);              /* Status = Pending */
                    Patch.u8CRC = 0xFFU;                                        /* CRC */
                    Patch.u8Length = 0x00U;                                     /* Patch Length */
                    Patch.u16PatchProjID = 0x0000U;                             /* Project ID (Invalid) */
                    Patch.u16PatchAddress0 = 0x0000U;                           /* Patch address */
                    Patch.u16PatchInstruction0 = 0x0000U;                       /* Patch instruction */
                    if (NV_WritePatch(&Patch) == C_ERR_NONE)
                    {
                        SetupDiagResponse(g_u8NAD, pDiag->u.SF.bySID,
                                          (uint8_t)C_ERRCODE_POSITIVE_RESPONSE);    /* Status = Positive feedback */
                    }
                    else
                    {
                        SetupDiagResponse(g_u8NAD, pDiag->u.SF.bySID,
                                          (uint8_t)C_ERRCODE_COND_SEQ);         /* Status = Negative feedback */
                    }
                }
#if (_DEBUG_SYSERR != FALSE)
#define C_DBG_DBGFUNC_WRLOCKIO  ((('W' - '@') << 10) | (('L' - '@') << 5) | ('I' - '@'))  /*!< Write Locked I/O */
#define C_DBG_DBGFUNC_WRSYSIO   ((('W' - '@') << 10) | (('S' - '@') << 5) | ('I' - '@'))  /*!< Write System I/O (in user-mode) */
                else if (u16FunctionID == C_DBG_DBGFUNC_WRLOCKIO)
                {
                    IO_COLIN_RAM_PROT |= B_COLIN_LOCK;                          /* Set I/O-register LOCK bit */
                    DEBUG_SET_IO_A();
                    IO_COLIN_RAM_PROT = 0U;                                     /* Write a Locked I/O-register */
                }
                else if (u16FunctionID == C_DBG_DBGFUNC_WRSYSIO)
                {
#if (_SUPPORT_APP_USER_MODE == FALSE)
                    builtin_mlx16_enter_user_mode();                            /* Switch to user-mode if APP runs in system mode */
#endif /* (_SUPPORT_APP_USER_MODE == FALSE) */
                    DEBUG_SET_IO_A();
                    IO_PORT_LIN_XKEY_S = 0U;                                    /* Write a System-port */
                }
#endif /* (_DEBUG_SYSERR != FALSE) */
#endif /* defined (__MLX81160__) || defined (__MLX81330B02__) || defined (__MLX81332B02__) || defined (__MLX81334A01__) || defined (__MLX81340B01__) || defined (__MLX81344B01__) || defined (__MLX81346B01__) */
#if defined (__MLX81160__)
                else if (u16FunctionID == C_DBG_DBGFUNC_FAST_WU)                /* MMP211105-1 */
                {
                    PATCH_HDR_t Patch;

                    SetupDiagResponse(g_u8NAD, pDiag->u.SF.bySID,
                                      (uint8_t)C_ERRCODE_PENDING);              /* Status = Pending */

                    Patch.u8CRC = 0x62U;                                        /* CRC */
                    Patch.u8Length = 0x10U;                                     /* Patch Length */
                    Patch.u16PatchProjID = PROJECT_ID;                          /* Project ID */
                    Patch.u16PatchAddress0 = 0x470BU;                           /* Patch address */
                    Patch.u16PatchInstruction0 = 0x40E2U;                       /* Patch instruction: "Jump $+5" */
                    Patch.u16Code[0] = 0x72D8U;                                 /* lod A, ... */
                    Patch.u16Code[1] = 0x020AU;                                 /* ... IO_PORT_MISC_IN */
                    Patch.u16Code[2] = 0xD448U;                                 /* and AH, #(B_PORT_MISC_IN_LOCAL_WU|B_PORT_MISC_IN_LIN_WU) */
                    Patch.u16Code[3] = 0x1902U;                                 /* je .+2 */
                    Patch.u16Code[4] = 0x76DAU;                                 /* jmpf ... */
                    Patch.u16Code[5] = 0x2269U;                                 /* ... 0x44D2/2 */
                    Patch.u16Code[6] = 0x76DAU;                                 /* jmpf ... */
                    Patch.u16Code[7] = 0x2221U;                                 /* ... 0x4442/2*/

                    if (NV_WritePatch(&Patch) == C_ERR_NONE)
                    {
                        SetupDiagResponse(g_u8NAD, pDiag->u.SF.bySID,
                                          (uint8_t)C_ERRCODE_POSITIVE_RESPONSE);    /* Status = Positive feedback */
                    }
                    else
                    {
                        SetupDiagResponse(g_u8NAD, pDiag->u.SF.bySID,
                                          (uint8_t)C_ERRCODE_COND_SEQ);         /* Status = Negative feedback */
                    }
                }
#endif /* defined (__MLX81160__) */
                else
                {
                    /* Nothing */
                }
            }
#if (_SUPPORT_ADC_BGD != FALSE)
            else if (u8FunctionID == (uint8_t)C_DBG_SUBFUNC_VAUX_VBGD)
            {
                /* Get VAUX and VBGD data (MMP220307-1)
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | PCI |  SID |    D1    |    D2    |    D3    |    D4    |    D5    |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | 0x06| Debug| Supplier | Supplier | Reserved | Reserved |   FUNC   |
                 *  |     |     | 0xDB | ID (LSB) | ID (MSB) |   0xFF   |   0xFF   |   0xD9   |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 * Response
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | PCI | RSID |    D1    |    D2    |    D3    |    D4    |    D5    |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | 0x06| 0x1B |   VAUX   |   VAUX   |   VBGD   |   VBGD   | Reserved |
                 *  |     |     |      |   [LSB]  |   [MSB]  |   [LSB]  |   [MSB]  |          |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 */
                LinDiag_VauxVbgd();
            }
#endif /* (_SUPPORT_ADC_BGD != FALSE) */
#if (_SUPPORT_ADC_VDDA_VDDD != FALSE)
            else if (u8FunctionID == (uint8_t)C_DBG_SUBFUNC_VDDA_VDDD)
            {
                /* Get VDDA and VDDD data
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | PCI |  SID |    D1    |    D2    |    D3    |    D4    |    D5    |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | 0x06| Debug| Supplier | Supplier | Reserved | Reserved |   FUNC   |
                 *  |     |     | 0xDB | ID (LSB) | ID (MSB) |   0xFF   |   0xFF   |   0xDA   |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 * Response
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | PCI | RSID |    D1    |    D2    |    D3    |    D4    |    D5    |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | 0x06| 0x1B |   VDDA   |   VDDA   |   VDDD   |   VDDD   | Reserved |
                 *  |     |     |      |   [LSB]  |   [MSB]  |   [LSB]  |   [MSB]  |          |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 */
                LinDiag_VddaVddd();
            }
#endif /* (_SUPPORT_ADC_VDDA_VDDD != FALSE) */
#if (_SUPPORT_ADC_VBOOST != FALSE)
            else if (u8FunctionID == (uint8_t)C_DBG_SUBFUNC_VBOOST)
            {
                /* Get VBoost Voltage
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | PCI |  SID |    D1    |    D2    |    D3    |    D4    |    D5    |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | 0x06| Debug| Supplier | Supplier | Reserved | Reserved |   FUNC   |
                 *  |     |     | 0xDB | ID (LSB) | ID (MSB) |   0xFF   |   0xFF   |   0xDB   |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 * Response
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | PCI | RSID |    D1    |    D2    |    D3    |    D4    |    D5    |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | 0x06| 0x1B |  VBOOST  |  VBOOST  | Reserved | Reserved | Reserved |
                 *  |     |     |      |   [LSB]  |   [MSB]  |          |          |          |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 */
                LinDiag_Vboost();
            }
#elif (_SUPPORT_IO_DUT_SELECT_HVIO != FALSE) && (HVIO_DUT_SELECT == PIN_FUNC_IO_0)
            else if (u8FunctionID == (uint8_t)C_DBG_SUBFUNC_VIO0HV)
            {
                /* Get IO0-HV Voltage
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | PCI |  SID |    D1    |    D2    |    D3    |    D4    |    D5    |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | 0x06| Debug| Supplier | Supplier | Reserved | Reserved |   FUNC   |
                 *  |     |     | 0xDB | ID (LSB) | ID (MSB) |   0xFF   |   0xFF   |   0xDB   |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 * Response
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | PCI | RSID |    D1    |    D2    |    D3    |    D4    |    D5    |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | 0x06| 0x1B |  VIO0HV  |  VIO0HV  | Reserved | Reserved | Reserved |
                 *  |     |     |      |   [LSB]  |   [MSB]  |          |          |          |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 */
                uint16_t u16Value = Get_IO0HV();
                u16Value =
                    (uint16_t)((p_MulU32_U16byU16(u16Value,
                                                  Get_HighVoltGain()) + (C_VOLTGAIN_DIV / 2)) / C_VOLTGAIN_DIV);                   /* Including rounding */
                StoreD1to2(u16Value);
            }

#endif /* (_SUPPORT_ADC_VBOOST != FALSE) */
#if (_DEBUG_MOTOR_CURRENT_FLT != FALSE)
            else if (u8FunctionID == (uint8_t)C_DBG_MOTOR_CURR_RAW)
            {
                /* Get Motor current (raw) data
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | PCI |  SID |    D1    |    D2    |    D3    |    D4    |    D5    |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | 0x06| Debug| Supplier | Supplier |   Index  |   Index  |   FUNC   |
                 *  |     |     | 0xDB | ID (LSB) | ID (MSB) |   (LSB)  |   (MSB)  |   0xDC   |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 * Response
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | PCI | RSID |    D1    |    D2    |    D3    |    D4    |    D5    |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | 0x06| 0x1B | Curr[i]  | Curr[i+1]| Curr[i+2]| Curr[i+3]| Reserved |
                 *  |     |     |      |          |          |          |          |          |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 */
                uint16_t u16Index = (((uint16_t)pDiag->u.SF.byD4) << 8) | ((uint16_t)pDiag->u.SF.byD3);
                if (u16Index == 0xFFFFU)
                {
                    StoreD1to4(C_DEBUG_BUF_SZ, g_u16DebugBufWrIdx);
                }
                else if (u16Index <= (C_DEBUG_BUF_SZ - 4U) )
                {
                    const uint16_t *pu16MotorCurrRaw = (uint16_t *)((void *)&g_au8DebugBuf[u16Index]);
                    StoreD1to4(pu16MotorCurrRaw[0], pu16MotorCurrRaw[1]);
                }
                else
                {
                    /* Nothing */
                }
            }
#endif /* (_DEBUG_MOTOR_CURRENT_FLT != FALSE) */
#if (_SUPPORT_HALL_LATCH_DIAG != FALSE)
            else if (u8FunctionID == (uint8_t)C_DBG_MOTOR_MICRO_STEP)
            {
                /* Get Motor current (raw) data
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | PCI |  SID |    D1    |    D2    |    D3    |    D4    |    D5    |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | 0x06| Debug| Supplier | Supplier |   Index  |   Index  |   FUNC   |
                 *  |     |     | 0xDB | ID (LSB) | ID (MSB) |   (LSB)  |   (MSB)  |   0xDC   |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 * Response
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | PCI | RSID |    D1    |    D2    |    D3    |    D4    |    D5    |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | 0x06| 0xDB | uStep[i] |uStep[i+1]|uStep[i+2]|uStep[i+3]| Reserved |
                 *  |     |     |      |          |          |          |          |          |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 */
                uint16_t u16Index = (((uint16_t)pDiag->u.SF.byD4) << 8) | ((uint16_t)pDiag->u.SF.byD3);
                if (u16Index == 0xFFFFU)
                {
                    StoreD1to4( (C_MOTOR_PHASES * C_MOTOR_STEP_PER_PHASE * C_MICROSTEP_PER_FULLSTEP), 0U);
                }
                else if (u16Index <= ((C_MOTOR_PHASES * C_MOTOR_STEP_PER_PHASE * C_MICROSTEP_PER_FULLSTEP) - 4U) )
                {
                    const uint16_t *pu16MotorMicroStep = (uint16_t *)((void *)&l_au8MotorMicroStep[u16Index]);
                    StoreD1to4(pu16MotorMicroStep[0], pu16MotorMicroStep[1]);
                }
                else
                {
                    /* Nothing */
                }
            }
#endif /* (_SUPPORT_HALL_LATCH_DIAG != FALSE) */
#if (_SUPPORT_FLASH_PRODUCTION_DATA != FALSE)
            else if (u8FunctionID == (uint8_t)C_DBG_SUBFUNC_ERASE_PRODUCTION_DATA)
            {
                /* Erase Production Data
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | PCI |  SID |    D1    |    D2    |    D3    |    D4    |    D5    |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | 0x06| Debug| Supplier | Supplier |    Key   |    Key   |   FUNC   |
                 *  |     |     | 0xDB | ID (LSB) | ID (MSB) |   (LSB)  |   (MSB)  |   0xEA   |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  Key should be 0xA5E3
                 * Response (positive)
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | PCI | RSID |    D1    |    D2    |    D3    |    D4    |    D5    |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | 0x01| 0x1B | Reserved | Reserved | Reserved | Reserved | Reserved |
                 *  |     |     |      |          |          |          |          |          |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 */
                uint16_t au16FlashKeys[3];
                uint16_t u16Key = (((uint16_t)pDiag->u.SF.byD4) << 8) | ((uint16_t)pDiag->u.SF.byD3);

                SetupDiagResponse(g_u8NAD, pDiag->u.SF.bySID, (uint8_t)C_ERRCODE_PENDING);
                au16FlashKeys[0] = 0x5648U;                                     /* Unlock_key, Needs to be 0x5648 to unlock the FLASH write-access */
                au16FlashKeys[1] = (u16Key + 1U);                               /* Write_key, Needs to be 0xA5E4 to trigger the FLASH WRITE operation */
                au16FlashKeys[2] = u16Key;                                      /* Erase_key, Needs to be 0xA5E3 to trigger the FLASH ERASE operation */
                ENTER_SECTION(ATOMIC_SYSTEM_MODE); /*lint !e534 */
                FL_SetupProductionData(&au16FlashKeys[0]);
#if (_DEBUG_FLPD_ERASE != FALSE)
                DEBUG_SET_IO_A();
#endif /* (_DEBUG_FLPD_ERASE != FALSE) */
                FL_EraseProductionDataSector(0);
                FL_EraseProductionDataSector(1);
#if (_DEBUG_FLPD_ERASE != FALSE)
                DEBUG_CLR_IO_A();
#endif /* (_DEBUG_FLPD_ERASE != FALSE) */
                EXIT_SECTION(); /*lint !e438 */
                SetupDiagResponse(g_u8NAD, pDiag->u.SF.bySID,
                                  (uint8_t)C_ERRCODE_POSITIVE_RESPONSE);        /* Status = Positive feedback */
                au16FlashKeys[0] = 0x0000U;                                     /* Unlock_key, Needs to be 0x5648 to unlock the FLASH write-access */
                au16FlashKeys[1] = 0x0000U;                                     /* Write_key, Needs to be 0xA5E4 to trigger the FLASH WRITE operation */
                au16FlashKeys[2] = 0x0000U;                                     /* Erase_key, Needs to be 0xA5E3 to trigger the FLASH ERASE operation */
                FL_SetupProductionData(&au16FlashKeys[0]);
            }
            else if (u8FunctionID == (uint8_t)C_DBG_SUBFUNC_WRITE_PRODUCTION_DATA_PAGE)
            {
                /* Erase Production Data
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | PCI |  SID |    D1    |    D2    |    D3    |    D4    |    D5    |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | 0x06| Debug| Supplier | Supplier |    Key   |    Key   |   FUNC   |
                 *  |     |     | 0xDB | ID (LSB) | ID (MSB) |   (LSB)  |   (MSB)  |   0xEA   |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  Key should be 0xA5E4
                 * Response (positive)
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | PCI | RSID |    D1    |    D2    |    D3    |    D4    |    D5    |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | 0x01| 0x1B | RecordIdx| RecordIdx| Reserved | Reserved | Reserved |
                 *  |     |     |      |   (LSB)  |   (MSB)  |          |          |          |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 */
#define C_FLASH_PAGE_SIZE      (MEM_FLASH_PAGE_SIZE / sizeof(uint16_t))           /* Flash Page in 16-bit words */
#define C_RECORD_SIZE          (3U * C_FLASH_PAGE_SIZE)                         /* Record length in Flash-pages */
#define C_FLASH_SECTOR_SIZE    (16U * C_FLASH_PAGE_SIZE)                        /* Flash Sector in Flash-pages */
                /* Test:
                 * Record = 3 Flash-pages;
                 * Record must fit on a Flash-sector (2kB)
                 * Last 16-bit word of record is CRC_U16; 16-bit prior to CRC_U16 is Record Index; Record with highest index is newest
                 * Record copy based on Flash-pages (limit RAM buffer)
                 * Production Data is 2 Flash-sectors (to allow flash-page copy via limited RAM buffer)
                 */
                uint16_t au16FlashKeys[3];
                uint16_t u16Key = (((uint16_t)pDiag->u.SF.byD4) << 8) | ((uint16_t)pDiag->u.SF.byD3);
                uint16_t *pu16RecordPage, *pu16NewestRecord = NULL;  /* pu16NewestRecord points to newest record */
                uint16_t u16NewestRecordIdx = 0U;
                uint16_t u16CRC_Record, u16CRC_Page;
                uint8_t uFlashPageIdx = 0U;

                /* Check first Production Data Area */
                /* To check for valid and invalid (erased) flash pages, the Flash DED should be turned OFF (set to '0').
                 * In case DED is non-zero, accessing erased flash lead to a system error MEMERR_IT (which is non-mask-able and non-return-able)
                 * With DED set to '0', the DATA_CORRUPTED or FL_ECC event-flags are set to indicate for double bit errors.
                 */
                SetupDiagResponse(g_u8NAD, pDiag->u.SF.bySID, (uint8_t)C_ERRCODE_PENDING);
                pu16RecordPage = (uint16_t *)&_fw_prod_data_start1;
#if (_SUPPORT_APP_USER_MODE != FALSE)
                ENTER_SECTION(SYSTEM_MODE); /*lint !e534 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
                do
                {
                    IO_EEPROM_FLASH_FL = (B_EEPROM_FLASH_FL_DATA_CORRUPTED | B_EEPROM_FLASH_FL_SBE);
                    IO_MLX16_ITC_PEND0_S = B_MLX16_ITC_PEND0_FL_ECC;
                    if ( (p_CalcCRC_U16(pu16RecordPage, C_RECORD_SIZE) == 0xFFFFU) &&
                         ((IO_MLX16_ITC_PEND0_S & B_MLX16_ITC_PEND0_FL_ECC) == 0U) )
                    {
                        /* Record is valid */
                        uint16_t u16RecordIdx = pu16RecordPage[C_RECORD_SIZE - 2U];
                        if (u16RecordIdx > u16NewestRecordIdx)
                        {
                            u16NewestRecordIdx = u16RecordIdx;
                            pu16NewestRecord = pu16RecordPage;
                        }
                    }
                    pu16RecordPage += C_RECORD_SIZE;
                } while ( (pu16RecordPage + C_RECORD_SIZE) <
                          (uint16_t *)(&_fw_prod_data_start1 + C_FLASH_SECTOR_SIZE) );
                /* Check second Production Data Area */
                pu16RecordPage = (uint16_t *)&_fw_prod_data_start2;
                do
                {
                    IO_EEPROM_FLASH_FL = (B_EEPROM_FLASH_FL_DATA_CORRUPTED | B_EEPROM_FLASH_FL_SBE);
                    IO_MLX16_ITC_PEND0_S = B_MLX16_ITC_PEND0_FL_ECC;
                    if ( (p_CalcCRC_U16(pu16RecordPage, C_RECORD_SIZE) == 0xFFFFU) &&
                         ((IO_MLX16_ITC_PEND0_S & B_MLX16_ITC_PEND0_FL_ECC) == 0U) )
                    {
                        /* Record is valid */
                        uint16_t u16RecordIdx = pu16RecordPage[C_RECORD_SIZE - 2U];
                        if (u16RecordIdx > u16NewestRecordIdx)
                        {
                            u16NewestRecordIdx = u16RecordIdx;
                            pu16NewestRecord = pu16RecordPage;
                        }
                    }
                    pu16RecordPage += C_RECORD_SIZE;
                } while ( (pu16RecordPage + C_RECORD_SIZE) <
                          (uint16_t *)(&_fw_prod_data_start2 + C_FLASH_SECTOR_SIZE) );
#if (_SUPPORT_APP_USER_MODE != FALSE)
                EXIT_SECTION(); /*lint !e438 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */

                au16FlashKeys[0] = 0x5648U;                                     /* Unlock_key, to unlock the FLASH write-access */
                au16FlashKeys[1] = u16Key;                                      /* Write_key, to trigger the FLASH WRITE operation */
                au16FlashKeys[2] = (u16Key - 1U);                               /* Erase_key, Needs to be 0xA5E3 to trigger the FLASH ERASE operation */
                FL_SetupProductionData(&au16FlashKeys[0]);

                ENTER_SECTION(ATOMIC_SYSTEM_MODE); /*lint !e534 */              /* Don't allow interrupt and make sure to be in SYSTEM mode */

                pu16RecordPage = (uint16_t *)((void *)&au8UDS_DataBuffer[0]);
                if (pu16NewestRecord == NULL)
                {
                    /* No valid record found; Write defaults in Flash Page 1, 2 and 3 (test-pattern) */
#if (_DEBUG_FLPD_ERASE != FALSE)
                    DEBUG_SET_IO_A();
#endif /* (_DEBUG_FLPD_ERASE != FALSE) */
                    FL_EraseProductionDataSector(0U);
#if (_DEBUG_FLPD_ERASE != FALSE)
                    DEBUG_CLR_IO_A();
#endif /* (_DEBUG_FLPD_ERASE != FALSE) */
                    uFlashPageIdx = 0U;

                    for (uint8_t u8Idx = 0U; u8Idx < sizeof(au8UDS_DataBuffer); u8Idx++)
                    {
                        au8UDS_DataBuffer[u8Idx] = (0xAAU ^ u8Idx);
                    }
#if (_DEBUG_FLPD_WRITE != FALSE)
                    DEBUG_SET_IO_B();
#endif /* (_DEBUG_FLPD_WRITE != FALSE) */
                    FL_WriteProductionDataPage(&au8UDS_DataBuffer[0], uFlashPageIdx++);
                    FL_WriteProductionDataPage(&au8UDS_DataBuffer[0], uFlashPageIdx++);
#if (_DEBUG_FLPD_WRITE != FALSE)
                    DEBUG_CLR_IO_B();
#endif /* (_DEBUG_FLPD_WRITE != FALSE) */
                    u16CRC_Page = p_CalcCRC_U16(pu16RecordPage, C_FLASH_PAGE_SIZE);   /* CRC of single page */
                    u16CRC_Record = u16CRC_Page + u16CRC_Page;                    /* CRC of second page is same as first page */
                    if (u16CRC_Record < u16CRC_Page)
                    {
                        u16CRC_Record += 1U;                                      /* Overflow */
                    }
                }
                else
                {
                    if ( ((pu16NewestRecord + (2U * C_RECORD_SIZE)) <
                          (((uint16_t *)&_fw_prod_data_start1) + C_FLASH_SECTOR_SIZE)) ||                                              /* Does block fit in 1st Production Data Area */
                         ((pu16NewestRecord >= (uint16_t *)&_fw_prod_data_start2) &&
                          ((pu16NewestRecord + (2U * C_RECORD_SIZE)) <
                           (((uint16_t *)&_fw_prod_data_start2) + C_FLASH_SECTOR_SIZE))) )                                               /* Does block fit in 2nd Production Data Area */
                    {
                        /* Write Production Data Area #1 or #2 (Append) */
                        uFlashPageIdx =
                            (uint8_t)(((pu16NewestRecord - &_fw_prod_data_start1) / C_FLASH_PAGE_SIZE) + 3U);
                    }
                    else if (pu16NewestRecord < (uint16_t *)(&_fw_prod_data_start1 + C_FLASH_SECTOR_SIZE) )
                    {
                        /* Write Production Data Area #2 (Begin, with Erase) */
#if (_DEBUG_FLPD_ERASE != FALSE)
                        DEBUG_SET_IO_A();
#endif /* (_DEBUG_FLPD_ERASE != FALSE) */
                        FL_EraseProductionDataSector(1U);
#if (_DEBUG_FLPD_WRITE != FALSE)
                        DEBUG_CLR_IO_A();
#endif /* (_DEBUG_FLPD_WRITE != FALSE) */
                        uFlashPageIdx = ((&_fw_prod_data_start2 - &_fw_prod_data_start1) / C_FLASH_PAGE_SIZE);
                    }
                    else
                    {
                        /* Write Production Data Area #1 (Begin, with Erase) */
#if (_DEBUG_FLPD_ERASE != FALSE)
                        DEBUG_SET_IO_A();
#endif /* (_DEBUG_FLPD_ERASE != FALSE) */
                        FL_EraseProductionDataSector(0U);
#if (_DEBUG_FLPD_ERASE != FALSE)
                        DEBUG_CLR_IO_A();
#endif /* (_DEBUG_FLPD_ERASE != FALSE) */
                        uFlashPageIdx = 0U;
                    }

                    /* Copy first Flash page of the Record */
                    for (uint8_t u8Idx = 0U; u8Idx < sizeof(au8UDS_DataBuffer); u8Idx += 2U)
                    {
                        *((uint16_t *)((void *)&au8UDS_DataBuffer[u8Idx])) = *pu16NewestRecord++;
                    }
                    u16CRC_Page = p_CalcCRC_U16(pu16RecordPage, C_FLASH_PAGE_SIZE);   /* CRC of first page */
                    u16CRC_Record = u16CRC_Page;
#if (_DEBUG_FLPD_WRITE != FALSE)
                    DEBUG_SET_IO_B();
#endif /* (_DEBUG_FLPD_WRITE != FALSE) */
                    FL_WriteProductionDataPage(&au8UDS_DataBuffer[0], uFlashPageIdx++);
#if (_DEBUG_FLPD_WRITE != FALSE)
                    DEBUG_CLR_IO_B();
#endif /* (_DEBUG_FLPD_WRITE != FALSE) */
                    /* Copy second Flash page of the Record */
                    for (uint8_t u8Idx = 0U; u8Idx < sizeof(au8UDS_DataBuffer); u8Idx += 2U)
                    {
                        *((uint16_t *)((void *)&au8UDS_DataBuffer[u8Idx])) = *pu16NewestRecord++;
                    }
                    u16CRC_Page = p_CalcCRC_U16(pu16RecordPage, C_FLASH_PAGE_SIZE);   /* CRC of second page */
                    u16CRC_Record += u16CRC_Page;
                    if (u16CRC_Record < u16CRC_Page)
                    {
                        u16CRC_Record += 1U;                                    /* Overflow */
                    }
#if (_DEBUG_FLPD_WRITE != FALSE)
                    DEBUG_SET_IO_B();
#endif /* (_DEBUG_FLPD_WRITE != FALSE) */
                    FL_WriteProductionDataPage(&au8UDS_DataBuffer[0], uFlashPageIdx++);
#if (_DEBUG_FLPD_WRITE != FALSE)
                    DEBUG_CLR_IO_B();
#endif /* (_DEBUG_FLPD_WRITE != FALSE) */
                    /* Copy third Flash page of the Record */
                    for (uint8_t u8Idx = 0U; u8Idx < sizeof(au8UDS_DataBuffer); u8Idx += 2U)
                    {
                        *((uint16_t *)((void *)&au8UDS_DataBuffer[u8Idx])) = *pu16NewestRecord++;
                    }
                }
                *((uint16_t *)((void *)&au8UDS_DataBuffer[124])) = (u16NewestRecordIdx + 1U);
                u16CRC_Page = p_CalcCRC_U16(pu16RecordPage, (C_FLASH_PAGE_SIZE - 1U));   /* CRC of third page */
                u16CRC_Record += u16CRC_Page;
                if (u16CRC_Record < u16CRC_Page)
                {
                    u16CRC_Record += 1U;                                        /* Overflow */
                }
                *((uint16_t *)((void *)&au8UDS_DataBuffer[126])) = (0xFFFFU - u16CRC_Record);
#if (_DEBUG_FLPD_WRITE != FALSE)
                DEBUG_SET_IO_B();
#endif /* (_DEBUG_FLPD_WRITE != FALSE) */
                FL_WriteProductionDataPage(&au8UDS_DataBuffer[0], uFlashPageIdx);
#if (_DEBUG_FLPD_WRITE != FALSE)
                DEBUG_CLR_IO_B();
#endif /* (_DEBUG_FLPD_WRITE != FALSE) */
                EXIT_SECTION(); /*lint !e438 */
                au16FlashKeys[0] = 0x0000U;                                     /* Unlock_key, to unlock the FLASH write-access */
                au16FlashKeys[1] = 0x0000U;                                     /* Write_key, to trigger the FLASH WRITE operation */
                au16FlashKeys[2] = 0x0000U;                                     /* Erase_key, to trigger the FLASH ERASE operation */
                FL_SetupProductionData(&au16FlashKeys[0]);
                g_DiagResponse.byNAD = g_u8NAD;
                g_DiagResponse.byPCI = 0x06U;
                g_DiagResponse.u.SF.byRSID = (uint8_t)C_RSID_MLX_DEBUG;
                StoreD1to2(u16NewestRecordIdx + 1U);
            }
#endif /* (_SUPPORT_FLASH_PRODUCTION_DATA != FALSE) */
#if ((_SUPPORT_LOG_ERRORS != FALSE) || (_SUPPORT_LOG_LIN_ERRORS != FALSE))
            else if (u8FunctionID == (uint8_t)C_DBG_SUBFUNC_ERRORCODES)
            {
                /* Read Error Codes
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | PCI |  SID |    D1    |    D2    |    D3    |    D4    |    D5    |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | 0x06| Debug| Supplier | Supplier | Reserved | Reserved |   FUNC   |
                 *  |     |     | 0xDB | ID (LSB) | ID (MSB) |  (0xFF)  |  (0xFF)  |   0xEC   |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 * Response
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | PCI | RSID |    D1    |    D2    |    D3    |    D4    |    D5    |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | 0x06| 0x1B |Error1_LSB|Error1_MSB|Error2_LSB|Error2_MSB| Not used |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 */
                uint16_t u16Error1 = GetFirstError();
                uint16_t u16Error2 = 0U;
                if ( ((u16Error1 & C_ERR_EXTW) == C_ERR_EXTW) ||
                     ((PeakFirstError() & C_ERR_EXTW) != C_ERR_EXTW) )
                {
                    u16Error2 = GetFirstError();
                }
                g_DiagResponse.u.SF.byD5 = 0x00U;
                StoreD1to4(u16Error1, u16Error2);
            }
#endif /* ((_SUPPORT_LOG_ERRORS != FALSE) || (_SUPPORT_LOG_LIN_ERRORS != FALSE)) */
            else if (u8FunctionID == (uint8_t)C_DBG_SUBFUNC_NV_READ)
            {
                /* Read Non Volatile Memory in blocks of 8 bytes.
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | PCI |  SID |    D1    |    D2    |    D3    |    D4    |    D5    |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | 0x06| Debug| Supplier | Supplier | Reserved |  Offset  |   FUNC   |
                 *  |     |     | 0xDB | ID (LSB) | ID (MSB) |  (0xFF)  |  (Words) |   0xED   |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 * Response (0xFF)
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | PCI | RSID |    D1    |    D2    |    D3    |    D4    |    D5    |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | 0x06| Debug|  Length  |          |          |          |          |
                 *  |     |     | 0x1B |  (Words) |          |          |          |          |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 * Others
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | PCI | RSID |    D1    |    D2    |    D3    |    D4    |    D5    |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | 0x06| 0x1B |Read Index| R[index] | R[index] |R[index+1]|R[index+1]|
                 *  |     |     |      |          |  (LSB)   |  (MSB)   |   (LSB)  |   (MSB)  |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 */
                if (pDiag->u.SF.byD3 == 0xFFU)
                {
                    g_DiagResponse.u.SF.byD1 = (ADDR_NV_END - ADDR_NV_USER) >> 1;
                    g_u8BufferOutID = (uint8_t)QR_RFR_DIAG;                     /* LIN Output buffer is valid (RFR_DIAG) */
                }
                else
                {
                    uint16_t u16Index = pDiag->u.SF.byD3;                       /* Index in Words */
                    g_DiagResponse.u.SF.byD1 = (uint8_t)u16Index;
                    {
                        const uint16_t *pu16NvmData = ((uint16_t *)ADDR_NV_USER) + u16Index;  /* NVRAM 16-bit pointer */
                        StoreD2to5(pu16NvmData[0], pu16NvmData[1]);
                    }
                }
            }
#if (_SUPPORT_START_TO_RUN != FALSE)
            else if (u8FunctionID == (uint8_t)C_DBG_SUBFUNC_START2RUN)
            {
                /* Start-to-Run
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | PCI |  SID |    D1    |    D2    |    D3    |    D4    |    D5    |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | 0x06| Debug| Supplier | Supplier |  Various | Sub-Func |   FUNC   |
                 *  |     |     | 0xDB | ID (LSB) | ID (MSB) |          |          |   0xEF   |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 * Response (0xFF)
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | PCI | RSID |    D1    |    D2    |    D3    |    D4    |    D5    |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | 0x06| 0x1B | Data #3  | Data #1  | Data #1  | Data #2  | Data #2  |
                 *  |     |     |      |          |  (LSB)   |  (MSB)   |   (LSB)  |   (MSB)  |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 */
                uint8_t u8S2R_SubFuncID = pDiag->u.SF.byD4;
                if (u8S2R_SubFuncID == (uint8_t)C_S2R_SUBFUNC_INIT)
                {
                    uint16_t u16Result = StartToRunInit();
                    g_DiagResponse.u.SF.byD1 = (uint8_t)u16Result;
                    StoreD2to5(0x0000U, 0x0000U);
                }
                else if (u8S2R_SubFuncID == (uint8_t)C_S2R_SUBFUNC_PRE_CHECK)
                {
                    uint16_t u16Result = StartToRunPreCheck();
                    g_DiagResponse.u.SF.byD1 = (uint8_t)u16Result;
                    StoreD2to5(0x0000U, 0x0000U);
                }
                else if (u8S2R_SubFuncID == (uint8_t)C_S2R_SUBFUNC_MOTOR_CONFIG)
                {
                    uint16_t u16Result = StartToRunMotorConfig();
                    g_DiagResponse.u.SF.byD1 = (uint8_t)u16Result;
                    StoreD2to5(g_u16CoilConfig, 0x0000U);
                }
                else if (u8S2R_SubFuncID == (uint8_t)C_S2R_SUBFUNC_COIL_PARAMS)
                {
                    uint16_t u16ConfigIdx = pDiag->u.SF.byD3;
                    uint16_t u16Result = StartToRunCoilResistance(u16ConfigIdx);
                    if ( (uint8_t)u16Result == (uint8_t)C_ERR_NONE)
                    {
                        u16Result = StartToRunCoilInductance(u16ConfigIdx);
                    }
                    g_DiagResponse.u.SF.byD1 = (uint8_t)u16Result;
                    StoreD2to5(g_u16CoilR, g_u16CoilL);
                }
                else if (u8S2R_SubFuncID == (uint8_t)C_S2R_SUBFUNC_MOTOR_CONST)
                {
                    uint16_t u16ConfigIdx = pDiag->u.SF.byD3;
                    uint16_t u16Result = StartToRunMotorConst(u16ConfigIdx);
                    g_DiagResponse.u.SF.byD1 = (uint8_t)u16Result;
                    StoreD2to5(0, 0);
                }
                else if (u8S2R_SubFuncID == (uint8_t)C_S2R_SUBFUNC_STATUS)
                {
                    g_DiagResponse.u.SF.byD1 = g_e8MotorStatus;
                    StoreD2to5(g_u16MotorConst, 0x0000U);
                }
                else if (u8S2R_SubFuncID == (uint8_t)C_S2R_SUBFUNC_ABORT)
                {
                    (void)StartToRunAbort();
                    g_DiagResponse.u.SF.byD1 = C_ERR_NONE;
                    StoreD2to5(0x0000U, 0x0000U);
                }
                else if (u8S2R_SubFuncID == (uint8_t)C_S2R_SUBFUNC_EXIT)
                {
                    MotorDriverInit(TRUE);
                    g_DiagResponse.u.SF.byD1 = C_ERR_NONE;
                    StoreD2to5(0x0000U, 0x0000U);
                }
#if FALSE
                else if (u8S2R_SubFuncID == (uint8_t)C_S2R_SUBFUNC_SET_MOTOR_R)
                {
                    uint8_t u8Base = (pDiag->u.SF.byD3 & (uint8_t)0x1FU);
                    uint8_t u8Exp = (pDiag->u.SF.byD3 >> 5);
                    uint16_t u16Value = (((u8Base + 32U) << u8Exp) - 32U);      /* Resistance in [10mR]; Phase-to-Phase */
                    /* Single (star) Resistance in [mR] */
                    u16Value = p_MulDivU16_U16byU16byU16(u16Value, Get_MCurrGain(), Get_MotorVoltGainF());   /* mR * mA/ADC_LSB_A (10 = 10mV/mA) */
                    l_u16mR_AT =
                        p_MulDivU16_U16byU16byU16(u16Value, (128U * C_VOLTGAIN_DIV), (125U * (C_GMCURR_DIV * 2U)));             /* ADC_LSB_V/10mV (MMP210129-1) */
                }
                else if (u8S2R_SubFuncID == (uint8_t)C_S2R_SUBFUNC_SET_MOTOR_L)
                {
                    uint8_t u8Base = (pDiag->u.SF.byD3 & (uint8_t)0x1FU);
                    uint8_t u8Exp = (pDiag->u.SF.byD3 >> 5);
                    uint16_t u16Value = (((u8Base + 32U) << u8Exp) - 32U);      /* Inductance in [5uH] */
                    uint16_t u16Divisor;
                    uint16_t u16MotorVoltGain = Get_MotorVoltGainF();
                    u16Value *= 5U;                                             /* Inductance in [uH] */
                    uint16_t u16Z_CONST = C_Z_CONST_DIV16;
                    if (u16Value > 32000U)
                    {
                        u16Divisor = (1U << 8);
                        u16Z_CONST >>= 4U;
                    }
                    else if (u16Value > 16000U)
                    {
                        u16Divisor = (1U << 7);
                        u16Z_CONST >>= 3U;
                    }
                    else if (u16Value > 8000U)
                    {
                        u16Divisor = (1U << 6);
                        u16Z_CONST >>= 2U;
                    }
                    else if (u16Value > 4000U)
                    {
                        u16Divisor = (1U << 5);
                        u16Z_CONST >>= 1U;
                    }
                    else if (u16Value > 2000U)
                    {
                        u16Divisor = (1U << 4);
                    }
                    else if (u16Value > 1000U)
                    {
                        u16Divisor = (1U << 3);
                        u16MotorVoltGain <<= 1;
                    }
                    else
                    {
                        u16Divisor = (1U << 2);
                        u16MotorVoltGain <<= 2;
                    }
                    l_u16mZ = (uint16_t)p_MulDivU16_U16byU16byU16(Get_MCurrGain(), u16Value, u16Divisor);
                    l_u16mZ = p_MulDivU16_U16byU16byU16(u16Z_CONST, u16MotorVoltGain, l_u16mZ);
                }
#endif /* FALSE */
            }
#endif /* (_SUPPORT_START_TO_RUN != FALSE) */
            else if (u8FunctionID == (uint8_t)C_DBG_SUBFUNC_FILLNVRAM)
            {
                /* Fill NVRAM
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | PCI |  SID |    D1    |    D2    |    D3    |    D4    |    D5    |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | 0x06| Debug| Supplier | Supplier | NVRAM ID |  Pattern |   FUNC   |
                 *  |     |     | 0xDB | ID (LSB) | ID (MSB) |          |          |   0xF8   |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 * (No response)
                 */
                uint8_t u8NvramID = pDiag->u.SF.byD3;
                uint16_t u16Pattern = (((uint16_t)pDiag->u.SF.byD4) << 8) | ((uint16_t)pDiag->u.SF.byD4);
#define C_BUF_SZ 4
                uint16_t u16Buffer[C_BUF_SZ];
                u16Buffer[0] = u16Pattern;
                u16Buffer[1] = u16Pattern;
                u16Buffer[2] = u16Pattern;
                u16Buffer[3] = u16Pattern;
                if ( (u8NvramID & 0x80U) != 0x00)
                {
                    (void)NV_WriteUserDefaults( (C_ERR_NV_LIN_STD_1 | C_ERR_NV_LIN_STD_2)
#if (LINPROT == LIN2X_HVAC52)
                                                | (C_ERR_NV_LIN_ENH_1 | C_ERR_NV_LIN_ENH_2)
#endif /* (LINPROT == LIN2X_HVAC52) */
#if (_SUPPORT_UDS != FALSE)
                                                | C_ERR_NV_LIN_UDS
#endif /* (_SUPPORT_UDS != FALSE) */
                                                | C_ERR_NV_EOL
#if (_SUPPORT_APP_SAVE != FALSE) && (_DEBUG_FLASH_WRITE_CYCLES == FALSE)
                                                | C_ERR_NV_APP_PARAMS
#endif /* (_SUPPORT_APP_SAVE != FALSE) && (_DEBUG_FLASH_WRITE_CYCLES == FALSE) */
                                                | C_ERR_NV_ACT_PARAMS
#if (_SUPPORT_TRIAXIS_MLX90363 != FALSE) || (_SUPPORT_TRIAXIS_MLX9038x != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90367 != FALSE) || (_SUPPORT_TRIAXIS_MLX90372 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90377 != FALSE) || (_SUPPORT_TRIAXIS_MLX90395 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90421 != FALSE) || (_SUPPORT_TRIAXIS_MLX90422 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90425 != FALSE) || (_SUPPORT_TRIAXIS_MLX90426 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90427 != FALSE) || \
    (_SUPPORT_INDUCTIVE_POS_SENSOR_MLX90513 != FALSE) || \
    (_SUPPORT_DUAL_HALL_LATCH_MLX92251 != FALSE) ||  (_SUPPORT_DUAL_HALL_LATCH_MLX92255 != FALSE) || \
    (_SUPPORT_FOC_MODE != FOC_MODE_NONE)
                                                | C_ERR_NV_SENSOR
#endif /* (_SUPPORT_TRIAXIS_MLX90363 != FALSE) || (_SUPPORT_TRIAXIS_MLX9038x != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90367 != FALSE) || (_SUPPORT_TRIAXIS_MLX90372 != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90377 != FALSE) || (_SUPPORT_TRIAXIS_MLX90395 != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90421 != FALSE) || (_SUPPORT_TRIAXIS_MLX90422 != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90425 != FALSE) || (_SUPPORT_TRIAXIS_MLX90426 != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90427 != FALSE) || \
        * (_SUPPORT_INDUCTIVE_POS_SENSOR_MLX90513 != FALSE) || \
        * (_SUPPORT_DUAL_HALL_LATCH_MLX92251 != FALSE) || (_SUPPORT_DUAL_HALL_LATCH_MLX92255 != FALSE) ||\
        * (_SUPPORT_FOC_MODE != FOC_MODE_NONE) */
                                                | C_ERR_NV_ACT_STALL
                                                | C_ERR_NV_HDR_KEEP_COUNT);
                }
                else
                {
                    if ( (u8NvramID & ID_NV_HDR) != (uint8_t)0x00U)
                    {
                        /* Bit 0: Non Volatile Memory Header */
                        (void)NV_WriteBlock(ADDR_NV_HDR, (uint16_t *)u16Buffer, C_BUF_SZ, FALSE);
                    }
                    if ( (u8NvramID & ID_NV_STD_LIN) != (uint8_t)0x00U)
                    {
                        /* Bit 1: Fill Standard LIN block */
                        (void)NV_WriteBlock(ADDR_NV_STD_LIN_1, (uint16_t *)u16Buffer, C_BUF_SZ, FALSE);
                        (void)NV_WriteBlock(ADDR_NV_STD_LIN_2, (uint16_t *)u16Buffer, C_BUF_SZ, FALSE);
                    }
#if (LINPROT == LIN2X_HVAC52)
                    if ( (u8NvramID & ID_NV_ENH_LIN) != (uint8_t)0x00U)
                    {
                        /* Bit 2: Fill Enhanced LIN block */
                        (void)NV_WriteBlock(ADDR_NV_ENH_LIN_1, (uint16_t *)u16Buffer, C_BUF_SZ, FALSE);
                        (void)NV_WriteBlock(ADDR_NV_ENH_LIN_2, (uint16_t *)u16Buffer, C_BUF_SZ, FALSE);
                    }
#endif /* (LINPROT == LIN2X_HVAC52) */
#if (_SUPPORT_UDS != FALSE)
                    if ( (u8NvramID & ID_NV_UDS_LIN) != (uint8_t)0x00U)
                    {
                        /* Bit 3: Fill UDS LIN block */
                        uint16_t u16Address = ADDR_NV_UDS;
                        do
                        {
                            (void)NV_WriteBlock(u16Address, (uint16_t *)u16Buffer, C_BUF_SZ, FALSE);
                            u16Address += SZ_NV_BLOCK;
                        } while (u16Address < (ADDR_NV_UDS + SZ_NV_UDS) );
                    }
#endif /* (_SUPPORT_UDS != FALSE) */
                    if ( (u8NvramID & ID_NV_EOL) != (uint8_t)0x00U)
                    {
                        /* Bit 4: Fill End-of-Line block */
                        uint16_t u16Address = ADDR_NV_EOL;
                        do
                        {
                            (void)NV_WriteBlock(u16Address, (uint16_t *)u16Buffer, C_BUF_SZ, FALSE);
                            u16Address += SZ_NV_BLOCK;
                        } while (u16Address < (ADDR_NV_EOL + SZ_NV_EOL) );
                    }
                    if ( (u8NvramID & ID_NV_ACT_PARAMS) != (uint8_t)0x00U)
                    {
                        /* Bit 5: Fill Actuator block */
                        uint16_t u16Address = ADDR_NV_ACT_PARAMS;
                        do
                        {
                            (void)NV_WriteBlock(u16Address, (uint16_t *)u16Buffer, C_BUF_SZ, FALSE);
                            u16Address += SZ_NV_BLOCK;
                        } while (u16Address < (ADDR_NV_ACT_PARAMS + SZ_NV_ACT_PARAMS) );
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
                    if ( (u8NvramID & ID_NV_SENSOR_PARAMS) != (uint8_t)0x00U)
                    {
                        uint16_t u16Address = ADDR_NV_SENSOR;
                        do
                        {
                            (void)NV_WriteBlock(u16Address, (uint16_t *)u16Buffer, C_BUF_SZ, FALSE);
                            u16Address += SZ_NV_BLOCK;
                        } while (u16Address < (ADDR_NV_SENSOR + SZ_NV_SENSOR) );
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
                    if ( (u8NvramID & ID_NV_ACT_STALL) != (uint8_t)0x00U)
                    {
                        /* Bit 6: Fill Stall block */
                        uint16_t u16Address = ADDR_NV_ACT_STALL;
                        do
                        {
                            (void)NV_WriteBlock(u16Address, (uint16_t *)u16Buffer, C_BUF_SZ, FALSE);
                            u16Address += SZ_NV_BLOCK;
                        } while (u16Address < (ADDR_NV_ACT_STALL + SZ_NV_ACT_STALL) );
                    }
                }
            }
            else if (u8FunctionID == (uint8_t)C_DBG_SUBFUNC_SET_IO_VALUE)       /* MMP190715-1 */
            {
                /* Set I/O-register value (16-bits)
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | PCI |  SID |    D1    |    D2    |    D3    |    D4    |    D5    |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | 0x06| Debug| Supplier | Supplier | I/O-value| I/O-Value|   FUNC   |
                 *  |     |     | 0xDB | ID (LSB) | ID (MSB) |   (LSB)  |   (MSB)  |   0xFA   |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *
                 * None
                 */
                l_u16IoValue = (((uint16_t)pDiag->u.SF.byD4) << 8) | ((uint16_t)pDiag->u.SF.byD3);
            }
            else if (u8FunctionID == (uint8_t)C_DBG_SUBFUNC_SET_IO_REG)       /* MMP190715-1 */
            {
                /* Set I/O-register value (16-bits)
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | PCI |  SID |    D1    |    D2    |    D3    |    D4    |    D5    |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | 0x06| Debug| Supplier | Supplier |  I/O-reg |  I/O-reg |   FUNC   |
                 *  |     |     | 0xDB | ID (LSB) | ID (MSB) |   (LSB)  |   (MSB)  |   0xFB   |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *
                 * None
                 */
                uint16_t u16IoAddress = (((uint16_t)pDiag->u.SF.byD4) << 8) | ((uint16_t)(pDiag->u.SF.byD3 & 0xFEU));
                if (u16IoAddress <= C_ADDR_IO_END)
                {
                    *((uint16_t *)u16IoAddress) = l_u16IoValue;
                }
            }
            else if (u8FunctionID == (uint8_t)C_DBG_SUBFUNC_GET_IO_REG)
            {
                /* Get I/O-register value (16-bits)
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | PCI |  SID |    D1    |    D2    |    D3    |    D4    |    D5    |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | 0x06| Debug| Supplier | Supplier |  I/O-reg |  I/O-reg |   FUNC   |
                 *  |     |     | 0xDB | ID (LSB) | ID (MSB) |   (LSB)  |   (MSB)  |   0xFD   |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *
                 * Response
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | PCI | RSID |    D1    |    D2    |    D3    |    D4    |    D5    |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | 0x06| 0x1B |  I/O-reg |  I/O-reg | I/O-value| I/O-value| Reserved |
                 *  |     |     |      |   (LSB)  |   (MSB)  |   (LSB)  |   (MSB)  |  (0xFF)  |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 */
                uint16_t u16IoAddress = (((uint16_t)pDiag->u.SF.byD4) << 8) | ((uint16_t)pDiag->u.SF.byD3);
                if (  (u16IoAddress <= C_ADDR_IO_END) ||                                          /* System I/O */
                      ((u16IoAddress >= C_ADDR_NV_BGN) && (u16IoAddress <= C_ADDR_NV_END)) ||     /* Non Volatile Memory */
                      ((u16IoAddress >= C_ADDR_RAM_BGN) && (u16IoAddress <= C_ADDR_RAM_END)) ||   /* CoLIN RAM + System RAM */
                      ((u16IoAddress >= C_ADDR_ROM_BGN) && (u16IoAddress <= C_ADDR_ROM_END)) ||   /* System ROM */
#if defined (__MLX81344__) || defined (__MLX81346__)
                      (u16IoAddress >= C_ADDR_FLASH_BGN) )                                        /* System FLASH (32k) */
#else  /* defined (__MLX81344__) || defined (__MLX81346__) */
                      ((u16IoAddress >= C_ADDR_FLASH_BGN) && (u16IoAddress <= C_ADDR_FLASH_END)) ) /* System FLASH (32k) */
#endif /* defined (__MLX81344__) || defined (__MLX81346__) */
                {
                    StoreD1to4(u16IoAddress, *((uint16_t *)u16IoAddress));
                }
            }
            else if (u8FunctionID == (uint8_t)C_DBG_SUBFUNC_FATAL_ERRORCODES)
            {
                /* Get (last) Fatal-error: error-code, info and address
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | PCI |  SID |    D1    |    D2    |    D3    |    D4    |    D5    |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | 0x06| Debug| Supplier | Supplier |   Index  | Reserved |   FUNC   |
                 *  |     |     | 0xDB | ID (LSB) | ID (MSB) |          |          |   0xFE   |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *
                 * Response
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | PCI | RSID |    D1    |    D2    |    D3    |    D4    |    D5    |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | 0x06| 0x1B |   0xFF   | ErrorCode|   Info   |AddressLSB|AddressMSB|
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 */
                StoreD2to5(bistError, bistErrorInfo);
            }
            else
            {
                /* Nothing */
            }
        }
        else
        {
            SetupDiagResponse(g_u8NAD, pDiag->u.SF.bySID,
                              (uint8_t)C_ERRCODE_SFUNC_NOSUP);                  /* Status = Negative feedback */
        }
    }
} /* End of DfrDiagMlxDebug() */
#endif /* (_SUPPORT_MLX_DEBUG_MODE != FALSE) */

#if (_SUPPORT_MLX_CHIP_STATUS != FALSE)
extern void MotorDriverConfig(uint16_t u16State);
/*!*************************************************************************** *
 * HandleMlxChipControl
 * \brief   Handle Melexis Chip Control frame (DPI)
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details Special Chip Demand Frame for IC test (e.g. DPI)
 *  Message ID: 0x000C
 *  Message size: 8-bytes
 *  Repetitive time: min. 10ms (at 19200 Baud)
 *
 *  RELAY
 *          | Bit 7 | Bit 6 | Bit 5 | Bit 4 | Bit 3 | Bit 2 | Bit 1 | Bit 0 |
 *          +---------------------------------------------------------------+
 *  Byte 8  |                           Reserved                            |
 *          +---------------------------------------------------------------+
 *  Byte 7  |                           Reserved                            |
 *          +---------------------------------------------------------------+
 *  Byte 6  |                           Reserved                            |
 *          +-------+-----------------------+-----------------------+-------+
 *  Byte 5  |Reserve|         LIN_SR        |         Reserved      |LIN_SRM|
 *          +-------+-----------------------+-----------------------+-------+
 *  Byte 4  |                           SSCM_CNT                            |
 *          +-----------------------+-------+-----------------------+-------+
 *  Byte 3  |        Reserved       | SSCM2 |         Reserved      |  SSCM |
 *          +-----------------------+-------+-----------------------+-------+
 *  Byte 2  |                   RELAY Off time [x 100ms]                    |
 *          +---------------------------------------------------------------+
 *  Byte 1  |                    RELAY On time [x 100ms]                    |
 *          +---------------------------------------------------------------+
 * *************************************************************************** *
 * - Call Hierarchy: HandleLinInMsg()
 * - Cyclomatic Complexity: 4+1
 * - Nesting: 2
 * - Function calling: 1 (MotorDriverConfig())
 * *************************************************************************** */
void HandleMlxChipControl(void)
{
#if (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT)
    static uint16_t u16Slave1_LT = 0U; /* Last known Slave 1 LT period */
    static uint16_t u16Master_LT = 0U; /* Last known Master 1 LT period */
    uint16_t u16NewSlave1_LT;
    uint16_t u16NewMaster1_LT;
#endif /* (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT) */
    DFR_CHIP_CONTROL *pChipControl = (DFR_CHIP_CONTROL *)&g_LinCmdFrameBuffer.ChipCtrl;

    /* PWM Stuff */
#if (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT)
    u16NewSlave1_LT = p_MulDivU16_U16byU16byU16(pChipControl->u8PWM_DC_MSB, PWM_REG_PERIOD, 510U);
    u16NewMaster1_LT = p_MulDivU16_U16byU16byU16(pChipControl->u8PWM_DC_LSB, PWM_REG_PERIOD, 510U);
    if ( (pChipControl->u8PWM_DC_MSB == 0xFFU) && (pChipControl->u8PWM_DC_LSB == 0xFFU) )  /* MMP201012-1 */
    {
        g_u16ActualPosition = 0xFFFEU;
        extern uint16_t l_u16ShaftRatiox512;
        l_u16ShaftRatiox512 = 0xFFFFU;
        MotorDriverPosInit(g_u16ActualPosition);
        g_u16TargetPosition = 0x0U;
        g_u8MotorCtrlSpeed = (uint8_t)C_DEFAULT_MOTOR_SPEED;
        g_e8StallDetectorEna = (uint8_t)C_STALLDET_NONE;
        AppResetFlags();
        MotorDriverStart(C_DEFAULT_MOTOR_SPEED);
    }
    else if ( (u16NewSlave1_LT != u16Slave1_LT) || (u16NewMaster1_LT != u16Master_LT) )
    {
        if ( (u16NewSlave1_LT == 0U) && (u16NewMaster1_LT == 0U) )
        {
            MotorDriverConfig(FALSE);
        }
        else
        {
            MotorDriverConfig(TRUE);
        }
#if defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
        u16NewSlave1_LT = (PWM_SCALE_OFFSET - u16NewSlave1_LT);
        u16NewMaster1_LT = (PWM_SCALE_OFFSET - u16NewMaster1_LT);
#endif /* defined (__MLX81340__) || (__MLX81344__) */
        IO_PWM_MASTER1_CTRL = B_PWM_MASTER1_STOP;
#if defined (__MLX81160__) && (_SUPPORT_EVB == MLX81160EVB_3P_3N)
        IO_PWM_SLAVE4_LT = PWM_SCALE_OFFSET;
        IO_PWM_SLAVE2_LT = PWM_SCALE_OFFSET;
        IO_PWM_SLAVE3_LT = u16NewSlave1_LT + C_EXT_FET_DEADTIME;
        IO_PWM_SLAVE1_LT = u16NewSlave1_LT - C_EXT_FET_DEADTIME;
        IO_PWM_MASTER2_LT = u16NewMaster1_LT + C_EXT_FET_DEADTIME;
        IO_PWM_MASTER1_LT = u16NewMaster1_LT - C_EXT_FET_DEADTIME;
#else  /* defined (__MLX81160__) && (_SUPPORT_EVB == MLX81160EVB_3P_3N) */
#if (C_MOTOR_PHASES == 3)
        IO_PWM_SLAVE2_LT = 0U;
        IO_PWM_SLAVE1_LT = u16NewSlave1_LT;
        IO_PWM_MASTER1_LT = u16NewMaster1_LT;
#else  /* (C_MOTOR_PHASES == 3) */
#if (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UV_WT)
        IO_PWM_SLAVE3_LT = 0U;
        IO_PWM_SLAVE2_LT = u16NewSlave1_LT;
        IO_PWM_SLAVE1_LT = 0U;
        IO_PWM_MASTER1_LT = u16NewMaster1_LT;
#else  /* (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UV_WT) */
#error "ERROR: Only support coils between U-V and W-T"
#endif /* (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UV_WT) */
#endif /* (C_MOTOR_PHASES == 3) */
#endif /* defined (__MLX81160__) && (_SUPPORT_EVB == MLX81160EVB_3P_3N) */
        IO_PWM_MASTER1_CTRL = B_PWM_MASTER1_START;
#if defined (__MLX81160__) && (_SUPPORT_EVB == MLX81160EVB_3P_3N)
        DRVCFG_PWM_3P_3N();
        DRVCFG_ENA_3P_3N();                                                     /* Enable the driver and the PWM phase W, V and U */
#else  /* defined (__MLX81160__) && (_SUPPORT_EVB == MLX81160EVB_3P_3N) */
#if (C_MOTOR_PHASES == 3)
        DRVCFG_PWM_UVW();
#if defined (__MLX81160__) || defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81350__)
        DRVCFG_ENA_UVW();                                                       /* Enable the driver and the PWM phase W, V and U */
#elif defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
        DRVCFG_ENA();                                                           /* Enable the driver and the PWM phase W, V and U */
#endif
#else
        DRVCFG_PWM_TUVW();
#if defined (__MLX81160__) || defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81350__)
        DRVCFG_ENA_TUVW();                                                      /* Enable the driver and the PWM phase W, V, U and T */
#elif defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
        DRVCFG_ENA();                                                           /* Enable the driver and the PWM phase W, V and U */
#endif
#endif
#endif /* defined (__MLX81160__) && (_SUPPORT_EVB == MLX81160EVB_3P_3N) */
        u16Slave1_LT = u16NewSlave1_LT;
        u16Master_LT = u16NewMaster1_LT;
    }
#elif (_SUPPORT_APP_TYPE == C_APP_RELAY)
    uint16_t u16RelayOnTime = (uint16_t)p_MulU32_U16byU16(pChipControl->u8PWM_DC_MSB, C_PI_TICKS_100MS);
    uint16_t u16RelayOffTime = (uint16_t)p_MulU32_U16byU16(pChipControl->u8PWM_DC_LSB, C_PI_TICKS_100MS);
    if ( (l_u16RelayOnTime != u16RelayOnTime) || (l_u16RelayOffTime != u16RelayOffTime) )
    {
        l_u16RelayOnTime = u16RelayOnTime;
        l_u16RelayOffTime = u16RelayOffTime;
        l_u16RelayOnCount = l_u16RelayOnTime;
        l_u16RelayOffCount = l_u16RelayOffTime;
    }
    else
    {
        /* Ignore; Same ON and OFF times */
    }
#endif /* (_SUPPORT_APP_TYPE) */

    /* SSCM */
    if (pChipControl->u1SSCM == 0U)
    {
        /* Disable the spread spectrum modulation */
        IO_PORT_SSCM_CONF &= ~(B_PORT_SSCM_CONF_SSCM_SINGLEBIT | B_PORT_SSCM_CONF_SSCM_EN);
    }
    else
    {
        /* Enable the spread spectrum modulation */
        IO_PORT_STEP_CONF = ((uint16_t)(pChipControl->u8SSCM_CNT << 8)) | (0x08U << 4) | 0x1U;
        IO_PORT_SSCM_CONF |= B_PORT_SSCM_CONF_SSCM_EN;
    }

#if defined (__MLX81160__) || defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81350__)
    /* SSCM2 */
    if (pChipControl->u1SSCM2 == 0U)
    {
        /* Disable the spread spectrum modulation */
        IO_PORT_SSCM2_CONF &= ~(B_PORT_SSCM2_CONF_SSCM2_SINGLEBIT | B_PORT_SSCM2_CONF_SSCM2_EN);
    }
    else
    {
        /* Enable the spread spectrum modulation */
        IO_PORT_STEP2_CONF = ((uint16_t)(195U << 8)) | (0x08U << 4) | 0x1U;
        IO_PORT_SSCM2_CONF |= B_PORT_SSCM2_CONF_SSCM2_EN;
    }

#if FALSE
    /* TEST */
    IO_TRIM2_DRV = (IO_TRIM2_DRV & ~(15 << 9)) | ((pChipControl->u8Reserved5 & 0x0F) << 9);
    if ((pChipControl->u8Reserved5 & 0xF0) != 0)
    {
        uint8_t u8Option = (pChipControl->u8Reserved5 >> 4) & 0x03;
        IO_PORT_DRV_OUT = (IO_PORT_DRV_OUT & ~M_PORT_DRV_OUT_DRVMOD_OPTION) || ((uint16_t)u8Option << 9);
    }
    else
    {
        IO_PORT_DRV_OUT = (IO_PORT_DRV_OUT & ~M_PORT_DRV_OUT_DRVMOD_OPTION);
    }
    if (pChipControl->u8Reserved6 != 0)
    {
        IO_TRIM1_DRV = (IO_TRIM1_DRV & ~M_TRIM1_DRV_PRE_TRIM_DRVMOD_CPCLK) | (((uint16_t)pChipControl->u8Reserved6 << 2) << 2);
    }
    else
    {
        IO_TRIM1_DRV = CalibrationParams.u16APP_TRIM30_TRIM1_DRV;
    }
#endif
#endif /* defined (__MLX81160__) || defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81350__) */

#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81350__)
    /* LIN Slew-rate */
    IO_TRIM_RCO1M = (IO_TRIM_RCO1M & ~M_TRIM_RCO1M_PRE_TR_LIN_SLEWRATE) | (((uint16_t)pChipControl->u3LIN_SR) << 8);
#elif defined (__MLX81160__) || defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
    /* LIN Slew-rate */
    if (pChipControl->u1LIN_SR_Source != 0U)
    {
        IO_TRIM_RCO1M = (IO_TRIM_RCO1M & ~M_TRIM_RCO1M_TR_LIN_SLEWRATE) | (((uint16_t)pChipControl->u3LIN_SR) << 8);
    }
    else
    {
        IO_TRIM_RCO1M = (IO_TRIM_RCO1M & ~M_TRIM_RCO1M_TR_LIN_SLEWRATE) |
                        (TrimParams.MS_Trim.u16MS_TRIM2_RCO1M_LIN & M_TRIM_RCO1M_TR_LIN_SLEWRATE);
    }
#endif

} /* End of HandleMlxChipControl() */

/*!*************************************************************************** *
 * HandleMlxChipStatus
 * \brief   Melexis Chip Status
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details Special Chip Status Frame for IC test (e.g. DPI)
 *  Message ID: 0x000D
 *  Message size: 8-bytes
 *  Repetitive time: min. 10ms (at 19200 Baud)
 *
 *  3PN/6PN-FET/RELAY
 *          | Bit 7 | Bit 6 | Bit 5 | Bit 4 | Bit 3 | Bit 2 | Bit 1 | Bit 0 |
 *          +-------------------------------+---------------+-------+-------+
 *  Byte 8  |         Wake-up source        |    Reserved   |TmpSnsE|LIN-Err|
 *          +-------------------------------+---------------+-------+-------+
 *  Byte 7  |                 Non Volatile Memory-CRC [MSB]                 |
 *          +---------------------------------------------------------------+
 *  Byte 6  |                 Non Volatile Memory-CRC [LSB]                 |
 *          +---------------------------------------------------------------+
 *  Byte 5  |                         Flash-CRC [MSB]                       |
 *          +---------------------------------------------------------------+
 *  Byte 4  |                         Flash-CRC [LSB]                       |
 *          +---------------------------------------------------------------+
 *  Byte 3  |                           I/O-State                           |
 *          +---------------------------------------------------------------+
 *  Byte 2  |                          M2S Counter                          |
 *          +---------------------------------------------------------------+
 *  Byte 1  |                              NAD                              |
 *          +---------------------------------------------------------------+
 * *************************************************************************** *
 * - Call Hierarchy: mlu_DataRequest()
 * - Cyclomatic Complexity: 10+1
 * - Nesting: 3
 * - Function calling: 3 (ADC_Conv_TempJ(), ml_DataReady(), SetLastError())
 * *************************************************************************** */
void HandleMlxChipStatus(void)
{
    static int16_t i16ChipJunctionTemperature = -273;

#if defined (__MLX81330A01__) || defined (__MLX81332A01__)
    volatile RFR_CHIP_STATUS *pChipStatus = (RFR_CHIP_STATUS *)((void *)&LinFrameDataBuffer[0]);
#else
    volatile RFR_CHIP_STATUS *pChipStatus = (RFR_CHIP_STATUS *)((void *)ML_DATA_LIN_FRAME_DATA_BUFFER);
#endif
    uint16_t i;

    for (i = 0U; i < sizeof(RFR_CHIP_STATUS) / 2U; i++)
    {
        ((uint16_t *)pChipStatus)[i] = 0xFFFFU;                                 /* Fields set to 0xFFFF or 0b11 are invalid */
    }
    pChipStatus->u8NAD = g_u8NAD;
    pChipStatus->u8M2S_Counter = g_u8MlxChipStatusM2SCounter;
    pChipStatus->u8IO_Status = (IO_PORT_IO_IN & 0xFFU);
    pChipStatus->u8FlashCRC_LSB = (uint8_t)(g_u32FlashBist & 0xFFU);
    pChipStatus->u8FlashCRC_MSB = (uint8_t)((g_u32FlashBist >> 8) & 0xFFU);
    pChipStatus->u8NvmCRC_LSB = (uint8_t)(g_u32NvmBist & 0xFFU);
    pChipStatus->u8NvmCRC_MSB = (uint8_t)((g_u32NvmBist >> 8) & 0xFFU);
    if (g_u8ErrorCommunication != FALSE)
    {
        pChipStatus->u1LinResponseError = TRUE;
    }
    else
    {
        pChipStatus->u1LinResponseError = FALSE;
    }
    {
        int16_t i16NewChipTemperature = ADC_Conv_TempJ(FALSE);
        if (i16ChipJunctionTemperature != -273)
        {
            if (i16NewChipTemperature > i16ChipJunctionTemperature)
            {
                if ( (i16NewChipTemperature - i16ChipJunctionTemperature) > 20)
                {
                    /* Temperature delta more than 20C (unexpected) */
                    pChipStatus->u1TempSensorError = TRUE;
                }
                else
                {
                    pChipStatus->u1TempSensorError = FALSE;
                }
            }
            else
            {
                if ( (i16ChipJunctionTemperature - i16NewChipTemperature) > 20)
                {
                    /* Temperature delta more than 20C (unexpected) */
                    pChipStatus->u1TempSensorError = TRUE;
                }
                else
                {
                    pChipStatus->u1TempSensorError = FALSE;
                }
            }
        }
        i16ChipJunctionTemperature = i16NewChipTemperature;
        pChipStatus->u1TempSensorError = FALSE;
    }
    pChipStatus->u4WakeupSource = 0U;
    if ( (IO_PORT_MISC_IN & B_PORT_MISC_IN_INTERNAL_WU) != 0U)
    {
        pChipStatus->u4WakeupSource = 1U;                                       /* Internal WU */
    }
    if ( (IO_PORT_MISC_IN & B_PORT_MISC_IN_LIN_WU) != 0U)
    {
        pChipStatus->u4WakeupSource = 2U;                                       /* LIN WU */
    }
    if (COLIN_LINstatus.buffer_used == 0U)                                      /* MMP240408-1 */
    {
        if (ml_DataReady(ML_END_OF_TX_DISABLED) != ML_SUCCESS)
        {
            g_u8ErrorCommunication = TRUE;
#if (_SUPPORT_LOG_ERRORS != FALSE)
            SetLastError(C_ERR_LIN_API);
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
        }
    }
#if (_SUPPORT_LOG_ERRORS != FALSE)
    else
    {
        g_u8ErrorCommunication = TRUE;
        SetLastError(C_ERR_LIN_BUF_NOT_FREE);
    }
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */

} /* End of HandleMlxChipStatus() */
#endif /* (_SUPPORT_MLX_CHIP_STATUS != FALSE) */

#endif /* (LIN_COMM != FALSE) && ((_SUPPORT_MLX_DEBUG_MODE != FALSE) || (_SUPPORT_MLX_CHIP_STATUS != FALSE)) */

/* EOF */
