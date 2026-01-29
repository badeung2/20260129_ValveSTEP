/*!************************************************************************** *
 * \file        LIN_Diagnostics.c
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
 *           -# SetupDiagResponse()
 *           -# HandleDfrDiag()
 *           -# LinDiagResponseTimeoutCount()
 *           -# RfrDiagReset()
 *  - Internal Functions:
 *           -# CalcProtectionBits()
 *           -# CheckSupplier()
 *           -# ValidSupplierFunctionID()
 *           -# DfrDiagReassignNAD()            (0xB0) (Generic, LH4.9 + LH5.2)
 *           -# DfrDiagAssignMessageToFrameID() (0xB1) (Generic, LH4.9 + LH5.2)
 *           -# DfrDiagReadByIdentifier         (0xB2) (Generic)
 *           -# GetdataByte()
 *           -# DfrDiagConditionalChangeNAD()   (0xB3) (Generic)
 *           -# DfrDiagDumpData()               (0xB4) (Generic)
 *           -# DfrDiagAssignVariantID          (0xB4) (LH4.9 + LH5.2)
 *           -# DfrDiagAssignNAD()              (0xB5/0xB8) (LH4.9 + LH5.2)
 *           -# DfrDiagTargetedReset()          (0xB5) (ISO17987/LIN SAE J2602)
 *           -# DfrDiagSaveConfig()             (0xB6) (Generic)
 *           -# DfrDiagAssignGroupAddress()     (0xB6) (LH5.2)
 *           -# DfrDiagAssignFrameIdRange()     (0xB7) (Generic, LH4.9 + LH5.2)
 *           -# HandleStandardLIN               (0xB0-0xB8)
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

/*!*************************************************************************** */
/*                           INCLUDES                                          */
/* *************************************************************************** */
#include "AppBuild.h"                                                           /* Application Build */

#if (LIN_COMM != FALSE)

/* Actuator Platform includes */
#include "drivelib/AppFunctions.h"                                              /* Application Functions support */
#include "drivelib/NV_Functions.h"                                              /* Non Volatile Memory Functions & Layout */
#include "drivelib/ErrorCodes.h"                                                /* Error-logging support */
#include "drivelib/GlobalVars.h"                                                /* Global Variables support */
#if (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT)
#if (_SUPPORT_LIN_AA != FALSE)
#include "drivelib/MotorDriver.h"                                               /* Motor Driver support */
#endif /* (_SUPPORT_LIN_AA != FALSE) */
#elif (_SUPPORT_APP_TYPE == C_APP_SOLENOID)
#include "drivelib/SolenoidDriver.h"                                            /* Solenoid Driver support */
#endif
#include "drivelib/Timer.h"                                                     /* Simple Timer support */
#include "camculib/private_mathlib.h"                                           /* Private Math Library support */

/* Communication Platform includes */
#include "commlib/LIN_Communication.h"                                          /* LIN Communication support */

/* CAMCU Platform includes */
#if (_SUPPORT_ISO17987) || ((LINPROT & LINXX) == LIN2J)
#include <bist_inline_impl.h>
#endif /* (_SUPPORT_ISO17987) || ((LINPROT & LINXX) == LIN2J) */
#if (_SUPPORT_BOOTLOADER_PPM != FALSE)
#include <version.h>
#endif /* (_SUPPORT_BOOTLOADER_PPM != FALSE) */

#if (_SUPPORT_MLX_DEBUG_MODE != FALSE)
extern void DfrDiagMlxDebug(DFR_DIAG *pDiag, uint16_t u16PCI_SID);
#endif /* (_SUPPORT_MLX_DEBUG_MODE != FALSE) */

/*!*************************************************************************** */
/*                           DEFINITIONS                                       */
/* *************************************************************************** */

/*!*************************************************************************** */
/*                           GLOBAL & LOCAL VARIABLES                          */
/* *************************************************************************** */
#pragma space dp                                                                /* __TINY_SECTION__ */
#pragma space none                                                              /* __TINY_SECTION__ */

#pragma space nodp                                                              /* __NEAR_SECTION__ */
static uint16_t l_u16DiagResponseTimeoutCount = 0U;                             /*!< LIN Diagnostics response timeout-counter */
#if (_SUPPORT_BOOTLOADER_PPM != FALSE)
uint8_t g_e8LoaderReset = (uint8_t)C_LOADER_CMD_NONE;                           /*!< (PPM) Loader Reset flag */
#endif /* (_SUPPORT_BOOTLOADER_PPM != FALSE) */

#if (_SUPPORT_UDS != FALSE) || (_SUPPORT_FLASH_PRODUCTION_DATA != FALSE)
uint8_t au8UDS_DataBuffer[128] __attribute__((aligned(2)));                     /*!< This data-buffer should be replaced by flash-loader "page_buffer" */
#endif /* (_SUPPORT_UDS != FALSE) || (_SUPPORT_FLASH_PRODUCTION_DATA != FALSE) */

#if (_SUPPORT_MLX_CHIP_STATUS != FALSE) && (_SUPPORT_APP_TYPE == C_APP_RELAY)
uint16_t l_u16RelayOnTime = 0U;
uint16_t l_u16RelayOffTime = 0U;
uint16_t l_u16RelayOnCount = 0U;
uint16_t l_u16RelayOffCount = 0U;
#endif /* (_SUPPORT_MLX_CHIP_STATUS != FALSE) && (_SUPPORT_APP_TYPE == C_APP_RELAY) */
#pragma space none                                                              /* __NEAR_SECTION__ */

#if (LINPROT == LIN2X_HVAC52) && (_SUPPORT_DIAG_B6 != FALSE)
extern uint8_t l_u8GAD;                                                         /*!< Group-address field */
#endif /* (LINPROT == LIN2X_HVAC52) && (_SUPPORT_DIAG_B6 != FALSE) */

#if (_SUPPORT_UDS != FALSE)
extern uint8_t l_u8FrameCount;                                                  /*!< Multi-frame Frame-count */
extern uint16_t l_u16FF_LEN_SID;                                                /*!< Multi-frame data-length */

extern void HandleUDS(DFR_DIAG *pDiag, uint16_t u16PCI_SID);
#endif /* (_SUPPORT_UDS != FALSE) */

/*!*************************************************************************** *
 * CalcProtectionBits
 * \brief
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] byFrameID: LIN Frame ID (without protection bits)
 * \return  (uint8_t) byFrameID: LIN Frame ID (with protection bits)
 * *************************************************************************** *
 * \details
 * *************************************************************************** *
 * - Call Hierarchy: DfrDiagReadByIdentifier()
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 0
 * *************************************************************************** */
static uint8_t CalcProtectionBits(uint8_t byFrameID)
{
    uint8_t u8FrameID = byFrameID;
    u8FrameID |=
        (((u8FrameID & 0x01U) ^ ((u8FrameID & 0x02U) >> 1) ^ ((u8FrameID & 0x04U) >> 2) ^ ((u8FrameID & 0x10U) >> 4)) !=
         0U) ? 0x40U : 0x00U;
    u8FrameID |=
        ((((u8FrameID & 0x02U) >>
           1) ^ ((u8FrameID & 0x08U) >> 3) ^ ((u8FrameID & 0x10U) >> 4) ^ ((u8FrameID & 0x20U) >> 5)) !=
         0U) ? 0x00U : 0x80U;
    return (u8FrameID);
} /* End of CalcProtectionBits() */

/*!*************************************************************************** *
 * SetupDiagResponse
 * \brief   Setup Diagnostics Response
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u8NAD: Node Address
 * \param   [in] u8SID: SID
 * \param   [in] u8ResponseCode: Response code (error)
 * \return  -
 * *************************************************************************** *
 * \details
 * *************************************************************************** *
 * - Call Hierarchy: DfrDiagAssignFrameIdRange(), DfrDiagMlxDebug(),
 *                   DfrDiagReadByIdentifier(), DfrDiagReassignNAD(),
 *                   DfrDiagSaveConfig()
 * - Cyclomatic Complexity: 1+1
 * - Nesting: 1
 * - Function calling: 0
 * *************************************************************************** */
void SetupDiagResponse(uint8_t u8NAD, uint8_t u8SID, uint8_t u8ResponseCode)
{
    g_DiagResponse.byNAD = u8NAD;
    if (u8ResponseCode == (uint8_t)C_ERRCODE_POSITIVE_RESPONSE)
    {
        /* Positive feedback
         *  +-------+-----+------+----------+----------+----------+----------+----------+
         *  |  NAD  | PCI | RSID |    D1    |    D2    |    D3    |    D4    |    D5    |
         *  +-------+-----+------+----------+----------+----------+----------+----------+
         *  |Initial| 0x01|  SID | Reserved | Reserved | Reserved | Reserved | Reserved |
         *  |  NAD  |     | +0x40|   0xFF   |   0xFF   |   0xFF   |   0xFF   |   0xFF   |
         *  +-------+-----+------+----------+----------+----------+----------+----------+
         */
        g_DiagResponse.byPCI = (uint8_t)C_RPCI_REASSIGN_NAD;
        g_DiagResponse.u.SF.byRSID = (uint8_t)(u8SID + (uint8_t)C_RSID_OK);
        g_DiagResponse.u.SF.byD1 = (uint8_t)C_DIAG_RES;                         /* Clear Pending feedback */
        g_DiagResponse.u.SF.byD2 = (uint8_t)C_DIAG_RES;                         /* Clear Pending feedback */
    }
    else
    {
        /* Pending or Negative feedback
         *  +-------+-----+------+----------+----------+----------+----------+----------+
         *  |  NAD  | PCI | RSID |    D1    |    D2    |    D3    |    D4    |    D5    |
         *  +-------+-----+------+----------+----------+----------+----------+----------+
         *  |Initial| 0x03| 0x7F | Requested| Response | Reserved | Reserved | Reserved |
         *  |  NAD  |     |      |    SID   |   Code   |   0xFF   |   0xFF   |   0xFF   |
         *  +-------+-----+------+----------+----------+----------+----------+----------+
         */
        g_DiagResponse.byPCI = (uint8_t)C_RPCI_NOK;
        g_DiagResponse.u.SF.byRSID = (uint8_t)C_RSID_NOK;
        g_DiagResponse.u.SF.byD1 = u8SID;
        g_DiagResponse.u.SF.byD2 = u8ResponseCode;
    }
    g_u8BufferOutID = (uint8_t)QR_RFR_DIAG;                                     /* LIN Output buffer is valid (RFR_DIAG) */
} /* End of SetupDiagResponse() */

#if (_SUPPORT_DIAG_B1 != FALSE) || (_SUPPORT_LIN_AA != FALSE)
/*!*************************************************************************** *
 * CheckSupplier
 * \brief   Check Diagnostics Demand frame Supplier ID
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16SupplierID: Supplier ID
 * \return  uint16_t
 *             FALSE: Incorrect/not expected Supplier ID
 *              TRUE: Correct/expected Supplier ID
 * *************************************************************************** *
 * \details
 * *************************************************************************** *
 * - Call Hierarchy: DfrDiagAssignNAD()
 * - Cyclomatic Complexity: 1+1
 * - Nesting: 1
 * - Function calling: 0
 * *************************************************************************** */
static uint16_t CheckSupplier(uint16_t const u16SupplierID)
{
    uint16_t u16Result = FALSE;
    if ( (u16SupplierID == (uint16_t)C_WILDCARD_SUPPLIER_ID) ||
         (u16SupplierID == (uint16_t)C_SUPPLIER_ID) )
    {
        u16Result = TRUE;
    }
    return (u16Result);
} /* End of CheckSupplier() */
#endif /* (_SUPPORT_DIAG_B1 != FALSE) || (_SUPPORT_LIN_AA != FALSE) */

/*!*************************************************************************** *
 * ValidSupplierFunctionID
 * \brief   Check Diagnostics Demand frame Supplier and Function ID
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16SupplierID: Supplier ID
 * \param   [in] u16FunctionID: Function ID
 * \return  uint16_t
 *             FALSE: Incorrect/not expected Supplier ID and/or Function ID
 *              TRUE: Correct/expected Supplier ID and Function ID
 * *************************************************************************** *
 * \details
 * NOTE: As this function is called by main loop, it is assumed that no Non Volatile Memory
 *       Write is busy.
 * *************************************************************************** *
 * - Call Hierarchy: DfrDiagReadByIdentifier(), DfrDiagReassignNAD()
 * - Cyclomatic Complexity: 1+1
 * - Nesting: 1
 * - Function calling: 0
 * *************************************************************************** */
static uint16_t ValidSupplierFunctionID(uint16_t const u16SupplierID, uint16_t const u16FunctionID)
{
    uint16_t u16Result = FALSE;
#if (LINPROT == LIN2X_HVAC52)
    ENH_LIN_PARAMS_t *pEnhLin = (ENH_LIN_PARAMS_t *)ADDR_NV_ENH_LIN_1;
#endif /* (LINPROT == LIN2X_HVAC52) */
    if ( ((u16SupplierID == C_SUPPLIER_ID) ||
          (u16SupplierID == C_WILDCARD_SUPPLIER_ID)) &&
         ((u16FunctionID == C_FUNCTION_ID) ||
#if (LINPROT == LIN2X_HVAC52)
          (u16FunctionID == pEnhLin->u16FunctionID) ||
#endif /* (LINPROT == LIN2X_HVAC52) */
          (u16FunctionID == C_WILDCARD_FUNCTION_ID)) )
    {
        u16Result = TRUE;
    }
    return (u16Result);
} /* End of ValidSupplierFunctionID() */

#if (_SUPPORT_DIAG_B0 != FALSE)
/*!*************************************************************************** *
 * DfrDiagReassignNAD()
 * \brief   Assign NAD
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] *pDiag: Pointer to LIN DFR message
 * \return  -
 * *************************************************************************** *
 * \details Assign NAD is used to resolve conflicting NADs in LIN clusters
 *          built using off-the-shelves slave nodes or reused slave nodes.
 *          This request uses the initial NAD (or the NAD wild-card); this
 *          is to avoid the risk of losing the address of a slave node.
 *          The NAD used for the response shall be the same as in the
 *          request, i.e. the initial NAD.
 *  +-----+-----+------+----------+----------+----------+----------+----------+
 *  | NAD | PCI |  SID |    D1    |    D2    |    D3    |    D4    |    D5    |
 *  +-----+-----+------+----------+----------+----------+----------+----------+
 *  | Init| 0x06| 0xB0 | Supplier | Supplier | Function | Function |  New NAD |
 *  | NAD |     |      | ID (LSB) | ID (MSB) | ID (LSB) | ID (MSB) |          |
 *  +-----+-----+------+----------+----------+----------+----------+----------+
 *
 * NOTE: As this function is called by main loop, it is assumed that no Non Volatile Memory
 *       Write is busy.
 * *************************************************************************** *
 * - Call Hierarchy: HandleDfrDiag()
 * - Cyclomatic Complexity: 2+1
 * - Nesting: 2
 * - Function calling: 3(4) (ValidSupplierFunctionID(), SetupDiagResponse(),
 *                           NV_Store(), SetLastError())
 * *************************************************************************** */
static void DfrDiagReassignNAD(DFR_DIAG *pDiag)
{
    if (ValidSupplierFunctionID( (pDiag->u.SF.byD1) | ((uint16_t)(pDiag->u.SF.byD2) << 8),
                                 (pDiag->u.SF.byD3) | ((uint16_t)(pDiag->u.SF.byD4) << 8)) != FALSE)
    {
        STD_LIN_PARAMS_t *pStdLin = (STD_LIN_PARAMS_t *)ADDR_NV_STD_LIN_1;
        uint8_t byInitialNAD = pStdLin->u8NAD;
#if (_SUPPORT_NAD_RANGE_CHECK != FALSE)
#if ((LINPROT & LINXX) == LIN2J)
        /* SAE J2602: NAD range: 0x60-0x6F */
        if ((pDiag->u.SF.byD5 >= 0x60U) && (pDiag->u.SF.byD5 <= 0x6FU))
#elif (((LINPROT & LINXX) >= LIN20) && ((LINPROT & LINXX) <= LIN22)) || ((LINPROT & LINXX) == LIN2X)
        /* ISO 17987: NAD range: 0x01-0x7F */
        if ((pDiag->u.SF.byD5 >= 0x01U) && (pDiag->u.SF.byD5 <= 0x7FU))
#endif
#endif /* (_SUPPORT_NAD_RANGE_CHECK != FALSE) */
        {
            STD_LIN_PARAMS_t StdLinRam;
#if (_SUPPORT_DIAG_B0_NV_STORE != FALSE)
            SetupDiagResponse(byInitialNAD, pDiag->u.SF.bySID,
                              (uint8_t)C_ERRCODE_PENDING);                      /* Status = Pending */
            p_CopyU16( (sizeof(STD_LIN_PARAMS_t) / sizeof(uint16_t)),
                       (uint16_t *)&StdLinRam,
                       (const uint16_t *)ADDR_NV_STD_LIN_1);
            g_u8NAD = pDiag->u.SF.byD5;
            StdLinRam.u8NAD = g_u8NAD;
            /* Store NVRAM */
            if ( (NV_WriteLIN_STD(&StdLinRam, C_NV_WRT_LIN_ID_ALL) == C_ERR_NONE) &&
                 (pStdLin->u8NAD == pDiag->u.SF.byD5) )
            {
                /* NAD changed.
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | PCI | RSID |    D1    |    D2    |    D3    |    D4    |    D5    |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | Init| 0x01| 0xF0 | Reserved | Reserved | Reserved | Reserved | Reserved |
                 *  | NAD |     |      |  (0xFF)  |  (0xFF)  |  (0xFF)  |  (0xFF)  |  (0xFF)  |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 */
                SetupDiagResponse(byInitialNAD, pDiag->u.SF.bySID,
                                  (uint8_t)C_ERRCODE_POSITIVE_RESPONSE);        /* Status = Positive feedback */
            }
            else
            {
                /* NAD couldn't be changed.
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | PCI | RSID |    D1    |    D2    |    D3    |    D4    |    D5    |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | Init| 0x03| 0x7F |   SID    | ErrorCode| Reserved | Reserved | Reserved |
                 *  | NAD |     |      |  (0xB0)  |  (0x12)  |  (0xFF)  |  (0xFF)  |  (0xFF)  |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 */
                SetupDiagResponse(byInitialNAD, pDiag->u.SF.bySID,
                                  (uint8_t)C_ERRCODE_SFUNC_NOSUP);              /* Status = Negative feedback */
#if (_SUPPORT_LOG_LIN_ERRORS != FALSE)
                SetLastError(C_ERR_LIN2X_B0);
#endif /* (_SUPPORT_LOG_LIN_ERRORS != FALSE) */
            }
#else  /* (_SUPPORT_DIAG_B0_NV_STORE != FALSE) */
            g_u8NAD = pDiag->u.SF.byD5;
            /* (RAM) NAD changed.
             *  +-----+-----+------+----------+----------+----------+----------+----------+
             *  | NAD | PCI | RSID |    D1    |    D2    |    D3    |    D4    |    D5    |
             *  +-----+-----+------+----------+----------+----------+----------+----------+
             *  | Init| 0x01| 0xF0 | Reserved | Reserved | Reserved | Reserved | Reserved |
             *  | NAD |     |      |  (0xFF)  |  (0xFF)  |  (0xFF)  |  (0xFF)  |  (0xFF)  |
             *  +-----+-----+------+----------+----------+----------+----------+----------+
             */
            SetupDiagResponse(byInitialNAD, pDiag->u.SF.bySID,
                              (uint8_t)C_ERRCODE_POSITIVE_RESPONSE);            /* Status = Positive feedback */
#endif /* (_SUPPORT_DIAG_B0_NV_STORE != FALSE) */
        }
#if (_SUPPORT_NAD_RANGE_CHECK != FALSE)
        else
        {
            /* NAD couldn't be changed.
             *  +-----+-----+------+----------+----------+----------+----------+----------+
             *  | NAD | PCI | RSID |    D1    |    D2    |    D3    |    D4    |    D5    |
             *  +-----+-----+------+----------+----------+----------+----------+----------+
             *  | Init| 0x03| 0x7F |   SID    | ErrorCode| Reserved | Reserved | Reserved |
             *  | NAD |     |      |  (0xB0)  |  (0x12)  |  (0xFF)  |  (0xFF)  |  (0xFF)  |
             *  +-----+-----+------+----------+----------+----------+----------+----------+
             */
            SetupDiagResponse(byInitialNAD, pDiag->u.SF.bySID,
                              (uint8_t)C_ERRCODE_SFUNC_NOSUP);                  /* Status = Negative feedback */
        }
#endif /* (_SUPPORT_NAD_RANGE_CHECK != FALSE) */
    }
} /* End of DfrDiagReassignNAD() */
#endif /* (_SUPPORT_DIAG_B0 != FALSE) */

#if (_SUPPORT_DIAG_B1 != FALSE)
/*!*************************************************************************** *
 * DfrDiagAssignMessageToFrameID()
 * \brief   Assign LIN Frame ID to LIN Message ID
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] *pDiag: Pointer to LIN DFR message
 * \return  -
 * *************************************************************************** *
 * \details Only LIN 2.0 (Obsolete in LIN 2.1)
 *  +-----+-----+-----+----------+----------+----------+----------+----------+
 *  | NAD | PCI | SID |    D1    |    D2    |    D3    |    D4    |    D5    |
 *  +-----+-----+-----+----------+----------+----------+----------+----------+
 *  | NAD | 0x06| 0xB1| Supplier | Supplier |  Message |  Message |   Frame  |
 *  |     |     |     | ID (LSB) | ID (MSB) | ID (LSB) | ID (MSB) |    ID    |
 *  +-----+-----+-----+----------+----------+----------+----------+----------+
 *
 * NOTE: As this function is called by main loop, it is assumed that no Non Volatile Memory
 *       Write is busy.
 * *************************************************************************** *
 * - Call Hierarchy: HandleDfrDiag()
 * - Cyclomatic Complexity: 3(5)+1
 * - Nesting: 2
 * - Function calling: 5(7) (CheckSupplier(), SetupDiagResponse(),
 *                           ml_Disconnect(), ml_AssignFrameToMessageID(),
 *                           ml_Connect(), NV_Store(), SetLastError())
 * *************************************************************************** */
static void DfrDiagAssignMessageToFrameID(DFR_DIAG *pDiag)
{
    if (CheckSupplier( (pDiag->u.SF.byD1) | ((uint16_t)(pDiag->u.SF.byD2) << 8)) )
    {
        uint16_t wMessageID = (((uint16_t)pDiag->u.SF.byD4) << 8) | ((uint16_t)pDiag->u.SF.byD3);
        STD_LIN_PARAMS_t *pStdLin = (STD_LIN_PARAMS_t *)ADDR_NV_STD_LIN_1;
        STD_LIN_PARAMS_t StdLinRam;

        if (wMessageID == MSG_CONTROL)
        {
            g_u8CtrlPID = pDiag->u.SF.byD5;
#if (_SUPPORT_DIAG_B1_NV_STORE != FALSE)
            SetupDiagResponse(g_u8NAD, pDiag->u.SF.bySID,
                              (uint8_t)C_ERRCODE_PENDING);                      /* Status = Pending */
            p_CopyU16( (sizeof(STD_LIN_PARAMS_t) / sizeof(uint16_t)),
                       (uint16_t *)&StdLinRam,
                       (const uint16_t *)ADDR_NV_STD_LIN_1);
            StdLinRam.u8ControlFrameID = g_u8CtrlPID;
#endif /* (_SUPPORT_DIAG_B1_NV_STORE != FALSE) */
            (void)ml_Disconnect();
            (void)ml_AssignFrameToMessageID(MSG_CONTROL, g_u8CtrlPID);
#if (_SUPPORT_LIN_BUS_ACTIVITY_CHECK != FALSE)
#if (MSG_CONTROL <= 7)
            /* Dynamic Message ID's: 0...7 */
            g_u16MLX4_RAM_Dynamic_CRC1 = p_CalcCRC_U16((const uint16_t *)C_RAM_MLX4_DYNAMIC_BGN_ADR1,
                                                       (C_RAM_MLX4_DYNAMIC_END_ADR1 - C_RAM_MLX4_DYNAMIC_BGN_ADR1)/sizeof(uint16_t));
#else  /* (MSG_CONTROL <= 7) */
            /* Dynamic Message ID's: 8...15 */
            g_u16MLX4_RAM_Dynamic_CRC2 = p_CalcCRC_U16((const uint16_t *)C_RAM_MLX4_DYNAMIC_BGN_ADR2,
                                                       (C_RAM_MLX4_DYNAMIC_END_ADR2 - C_RAM_MLX4_DYNAMIC_BGN_ADR2)/sizeof(uint16_t));
#endif /* (MSG_CONTROL <= 7) */
#endif /* (_SUPPORT_LIN_BUS_ACTIVITY_CHECK != FALSE) */
            (void)ml_Connect();
#if (_SUPPORT_DIAG_B1_NV_STORE != FALSE)
            /* Store NVRAM */
            if ( (NV_WriteLIN_STD(&StdLinRam, C_NV_WRT_LIN_ID_ALL) == C_ERR_NONE) &&
                 (pStdLin->u8ControlFrameID == pDiag->u.SF.byD5) )
            {
                /* Control Frame-ID changed.
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | PCI | RSID |    D1    |    D2    |    D3    |    D4    |    D5    |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | 0x01| 0xF1 | Reserved | Reserved | Reserved | Reserved | Reserved |
                 *  |     |     |      |  (0xFF)  |  (0xFF)  |  (0xFF)  |  (0xFF)  |  (0xFF)  |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 */
                SetupDiagResponse(g_u8NAD, pDiag->u.SF.bySID,
                                  (uint8_t)C_ERRCODE_POSITIVE_RESPONSE);        /* Status = Positive feedback */
            }
            else
            {
                /* Control Frame-ID couldn't be changed.
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | PCI | RSID |    D1    |    D2    |    D3    |    D4    |    D5    |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | 0x03| 0x7F |   SID    | ErrorCode| Reserved | Reserved | Reserved |
                 *  |     |     |      |  (0xB1)  |  (0x12)  |  (0xFF)  |  (0xFF)  |  (0xFF)  |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 */
                SetupDiagResponse(g_u8NAD, pDiag->u.SF.bySID,
                                  (uint8_t)C_ERRCODE_SFUNC_NOSUP);              /* Status = Negative feedback */
#if (_SUPPORT_LOG_LIN_ERRORS != FALSE)
                SetLastError(C_ERR_LIN2X_B1);
#endif /* (_SUPPORT_LOG_LIN_ERRORS != FALSE) */
            }
#else  /* (_SUPPORT_DIAG_B1_NV_STORE != FALSE) */
            /* Control Frame-ID changed.
             *  +-----+-----+------+----------+----------+----------+----------+----------+
             *  | NAD | PCI | RSID |    D1    |    D2    |    D3    |    D4    |    D5    |
             *  +-----+-----+------+----------+----------+----------+----------+----------+
             *  | NAD | 0x01| 0xF1 | Reserved | Reserved | Reserved | Reserved | Reserved |
             *  |     |     |      |  (0xFF)  |  (0xFF)  |  (0xFF)  |  (0xFF)  |  (0xFF)  |
             *  +-----+-----+------+----------+----------+----------+----------+----------+
             */
            SetupDiagResponse(g_u8NAD, pDiag->u.SF.bySID,
                              (uint8_t)C_ERRCODE_POSITIVE_RESPONSE);            /* Status = Positive feedback */
#endif /* (_SUPPORT_DIAG_B1_NV_STORE != FALSE) */
        }
        else if (wMessageID == MSG_STATUS)
        {
            g_u8StsPID = pDiag->u.SF.byD5;
#if (_SUPPORT_DIAG_B1_NV_STORE != FALSE)
            SetupDiagResponse(g_u8NAD, pDiag->u.SF.bySID,
                              (uint8_t)C_ERRCODE_PENDING);                      /* Status = Pending */
            p_CopyU16( (sizeof(STD_LIN_PARAMS_t) / sizeof(uint16_t)),
                       (uint16_t *)&StdLinRam,
                       (const uint16_t *)ADDR_NV_STD_LIN_1);
            StdLinRam.u8StatusFrameID = g_u8StsPID;
#endif /* (_SUPPORT_DIAG_B1_NV_STORE != FALSE) */
            (void)ml_Disconnect();
            (void)ml_AssignFrameToMessageID(MSG_STATUS, g_u8StsPID);
#if (_SUPPORT_LIN_BUS_ACTIVITY_CHECK != FALSE)
#if (MSG_STATUS <= 7)
            /* Dynamic Message ID's: 0...7 */
            g_u16MLX4_RAM_Dynamic_CRC1 = p_CalcCRC_U16((const uint16_t *)C_RAM_MLX4_DYNAMIC_BGN_ADR1,
                                                       (C_RAM_MLX4_DYNAMIC_END_ADR1 - C_RAM_MLX4_DYNAMIC_BGN_ADR1)/sizeof(uint16_t));
#else  /* (MSG_STATUS <= 7) */
            /* Dynamic Message ID's: 8...15 */
            g_u16MLX4_RAM_Dynamic_CRC2 = p_CalcCRC_U16((const uint16_t *)C_RAM_MLX4_DYNAMIC_BGN_ADR2,
                                                       (C_RAM_MLX4_DYNAMIC_END_ADR2 - C_RAM_MLX4_DYNAMIC_BGN_ADR2)/sizeof(uint16_t));
#endif /* (MSG_STATUS <= 7) */
#endif /* (_SUPPORT_LIN_BUS_ACTIVITY_CHECK != FALSE) */
            (void)ml_Connect();
#if (_SUPPORT_DIAG_B1_NV_STORE != FALSE)
            /* Store NVRAM */
            if ( (NV_WriteLIN_STD(&StdLinRam, C_NV_WRT_LIN_ID_ALL) == C_ERR_NONE) &&
                 (pStdLin->u8StatusFrameID == pDiag->u.SF.byD5) )
            {
                /* Status Frame-ID changed.
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | PCI | RSID |    D1    |    D2    |    D3    |    D4    |    D5    |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | 0x01| 0xF1 | Reserved | Reserved | Reserved | Reserved | Reserved |
                 *  |     |     |      |  (0xFF)  |  (0xFF)  |  (0xFF)  |  (0xFF)  |  (0xFF)  |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 */
                SetupDiagResponse(g_u8NAD, pDiag->u.SF.bySID,
                                  (uint8_t)C_ERRCODE_POSITIVE_RESPONSE);        /* Status = Positive feedback */
            }
            else
            {
                /* Status Frame-ID couldn't be changed.
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | PCI | RSID |    D1    |    D2    |    D3    |    D4    |    D5    |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | 0x03| 0x7F |   SID    | ErrorCode| Reserved | Reserved | Reserved |
                 *  |     |     |      |  (0xB1)  |  (0x12)  |  (0xFF)  |  (0xFF)  |  (0xFF)  |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 */
                SetupDiagResponse(g_u8NAD, pDiag->u.SF.bySID,
                                  (uint8_t)C_ERRCODE_SFUNC_NOSUP);              /* Status = Negative feedback */
#if (_SUPPORT_LOG_LIN_ERRORS != FALSE)
                SetLastError(C_ERR_LIN2X_B1);
#endif /* (_SUPPORT_LOG_LIN_ERRORS != FALSE) */
            }
#else  /* (_SUPPORT_DIAG_B1_NV_STORE != FALSE) */
            /* Status Frame-ID changed.
             *  +-----+-----+------+----------+----------+----------+----------+----------+
             *  | NAD | PCI | RSID |    D1    |    D2    |    D3    |    D4    |    D5    |
             *  +-----+-----+------+----------+----------+----------+----------+----------+
             *  | NAD | 0x01| 0xF1 | Reserved | Reserved | Reserved | Reserved | Reserved |
             *  |     |     |      |  (0xFF)  |  (0xFF)  |  (0xFF)  |  (0xFF)  |  (0xFF)  |
             *  +-----+-----+------+----------+----------+----------+----------+----------+
             */
            SetupDiagResponse(g_u8NAD, pDiag->u.SF.bySID,
                              (uint8_t)C_ERRCODE_POSITIVE_RESPONSE);            /* Status = Positive feedback */
#endif /* (_SUPPORT_DIAG_B1_NV_STORE != FALSE) */
        }
#if (LINPROT == LIN2X_AIRVENT12)
        else if (wMessageID == MSG_STATUS2)
        {
            g_u8Sts2PID = pDiag->u.SF.byD5;
#if (_SUPPORT_DIAG_B1_NV_STORE != FALSE)
            SetupDiagResponse(g_u8NAD, pDiag->u.SF.bySID,
                              (uint8_t)C_ERRCODE_PENDING);                      /* Status = Pending */
            p_CopyU16( (sizeof(STD_LIN_PARAMS_t) / sizeof(uint16_t)),
                       (uint16_t *)&StdLinRam,
                       (const uint16_t *)ADDR_NV_STD_LIN_1);
            StdLinRam.u8Status2FrameID = g_u8Sts2PID;
#endif /* (_SUPPORT_DIAG_B1_NV_STORE != FALSE) */
            (void)ml_Disconnect();
            (void)ml_AssignFrameToMessageID(MSG_STATUS2, g_u8Sts2PID);
#if (_SUPPORT_LIN_BUS_ACTIVITY_CHECK != FALSE)
#if (MSG_STATUS <= 7)
            /* Dynamic Message ID's: 0...7 */
            g_u16MLX4_RAM_Dynamic_CRC1 = p_CalcCRC_U16((const uint16_t *)C_RAM_MLX4_DYNAMIC_BGN_ADR1,
                                                       (C_RAM_MLX4_DYNAMIC_END_ADR1 - C_RAM_MLX4_DYNAMIC_BGN_ADR1)/sizeof(uint16_t));
#else  /* (MSG_STATUS <= 7) */
            /* Dynamic Message ID's: 8...15 */
            g_u16MLX4_RAM_Dynamic_CRC2 = p_CalcCRC_U16((const uint16_t *)C_RAM_MLX4_DYNAMIC_BGN_ADR2,
                                                       (C_RAM_MLX4_DYNAMIC_END_ADR2 - C_RAM_MLX4_DYNAMIC_BGN_ADR2)/sizeof(uint16_t));
#endif /* (MSG_STATUS <= 7) */
#endif /* (_SUPPORT_LIN_BUS_ACTIVITY_CHECK != FALSE) */
            (void)ml_Connect();
#if (_SUPPORT_DIAG_B1_NV_STORE != FALSE)
            /* Store NVRAM */
            if ( (NV_WriteLIN_STD(&StdLinRam, C_NV_WRT_LIN_ID_ALL) == C_ERR_NONE) &&
                 (pStdLin->u8Status2FrameID == pDiag->u.SF.byD5) )
            {
                /* Status Frame-ID changed.
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | PCI | RSID |    D1    |    D2    |    D3    |    D4    |    D5    |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | 0x01| 0xF1 | Reserved | Reserved | Reserved | Reserved | Reserved |
                 *  |     |     |      |  (0xFF)  |  (0xFF)  |  (0xFF)  |  (0xFF)  |  (0xFF)  |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 */
                SetupDiagResponse(g_u8NAD, pDiag->u.SF.bySID,
                                  (uint8_t)C_ERRCODE_POSITIVE_RESPONSE);        /* Status = Positive feedback */
            }
            else
            {
                /* Status Frame-ID couldn't be changed.
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | PCI | RSID |    D1    |    D2    |    D3    |    D4    |    D5    |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | 0x03| 0x7F |   SID    | ErrorCode| Reserved | Reserved | Reserved |
                 *  |     |     |      |  (0xB1)  |  (0x12)  |  (0xFF)  |  (0xFF)  |  (0xFF)  |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 */
                SetupDiagResponse(g_u8NAD, pDiag->u.SF.bySID,
                                  (uint8_t)C_ERRCODE_SFUNC_NOSUP);              /* Status = Negative feedback */
#if (_SUPPORT_LOG_LIN_ERRORS != FALSE)
                SetLastError(C_ERR_LIN2X_B1);
#endif /* (_SUPPORT_LOG_LIN_ERRORS != FALSE) */
            }
#else  /* (_SUPPORT_DIAG_B1_NV_STORE != FALSE) */
            /* Status Frame-ID changed.
             *  +-----+-----+------+----------+----------+----------+----------+----------+
             *  | NAD | PCI | RSID |    D1    |    D2    |    D3    |    D4    |    D5    |
             *  +-----+-----+------+----------+----------+----------+----------+----------+
             *  | NAD | 0x01| 0xF1 | Reserved | Reserved | Reserved | Reserved | Reserved |
             *  |     |     |      |  (0xFF)  |  (0xFF)  |  (0xFF)  |  (0xFF)  |  (0xFF)  |
             *  +-----+-----+------+----------+----------+----------+----------+----------+
             */
            SetupDiagResponse(g_u8NAD, pDiag->u.SF.bySID,
                              (uint8_t)C_ERRCODE_POSITIVE_RESPONSE);            /* Status = Positive feedback */
#endif /* (_SUPPORT_DIAG_B1_NV_STORE != FALSE) */
        }
#endif /* (LINPROT == LIN2X_AIRVENT12) */
        else
        {
            /* Wrong Message-ID.
             *  +-----+-----+------+----------+----------+----------+----------+----------+
             *  | NAD | PCI | RSID |    D1    |    D2    |    D3    |    D4    |    D5    |
             *  +-----+-----+------+----------+----------+----------+----------+----------+
             *  | NAD | 0x03| 0x7F |   SID    | ErrorCode| Reserved | Reserved | Reserved |
             *  |     |     |      |  (0xB1)  |  (0x12)  |  (0xFF)  |  (0xFF)  |  (0xFF)  |
             *  +-----+-----+------+----------+----------+----------+----------+----------+
             */
            SetupDiagResponse(g_u8NAD, pDiag->u.SF.bySID,
                              (uint8_t)C_ERRCODE_SFUNC_NOSUP);                  /* Status = Negative feedback */
#if (_SUPPORT_LOG_LIN_ERRORS != FALSE)
            SetLastError(C_ERR_LIN2X_B1);
#endif /* (_SUPPORT_LOG_LIN_ERRORS != FALSE) */
        }
    }
    else
    {
        /* Wrong Supplier-ID.
         *  +-----+-----+------+----------+----------+----------+----------+----------+
         *  | NAD | PCI | RSID |    D1    |    D2    |    D3    |    D4    |    D5    |
         *  +-----+-----+------+----------+----------+----------+----------+----------+
         *  | NAD | 0x03| 0x7F |   SID    | ErrorCode| Reserved | Reserved | Reserved |
         *  |     |     |      |  (0xB1)  |  (0x12)  |  (0xFF)  |  (0xFF)  |  (0xFF)  |
         *  +-----+-----+------+----------+----------+----------+----------+----------+
         */
        SetupDiagResponse(g_u8NAD, pDiag->u.SF.bySID,
                          (uint8_t)C_ERRCODE_SFUNC_NOSUP);                      /* Status = Negative feedback */
#if (_SUPPORT_LOG_LIN_ERRORS != FALSE)
        SetLastError(C_ERR_LIN2X_B1);
#endif /* (_SUPPORT_LOG_LIN_ERRORS != FALSE) */
    }
} /* End of DfrDiagAssignMessageToFrameID() */
#endif /* (_SUPPORT_DIAG_B1 != FALSE) */

/*!*************************************************************************** *
 * DfrDiagReadByIdentifier()
 * \brief   Read information by ID
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] *pDiag: Pointer to LIN DFR message
 * \return  -
 * *************************************************************************** *
 * \details
 *  +-----+-----+-----+----------+----------+----------+----------+----------+
 *  | NAD | PCI | SID |    D1    |    D2    |    D3    |    D4    |    D5    |
 *  +-----+-----+-----+----------+----------+----------+----------+----------+
 *  | NAD | 0x06| 0xB2|  ReadID  | Supplier | Supplier | Function | Function |
 *  |     |     |     |          | ID (LSB) | ID (MSB) | ID (LSB) | ID (MSB) |
 *  +-----+-----+-----+----------+----------+----------+----------+----------+
 * It is possible to read the supplier identity and other properties from a slave node using this request
 * ReadID   Description
 * 0x00 (M) LIN Product identification
 * 0x01 (O) Serial number (32-bits)
 * 0x02 (M) Test purpose (ISO17987)
 * 0x03 (M) NCF/LDF version (ISO17987)
 * 0x10 (O) Frame-ID for first LIN Message (Control)
 * 0x11 (O) Frame-ID for second LIN Message (Status)
 * 0x20 (U) Get HW- and Non Volatile Memory NAD (3.11) ???
 * 0x21 (U) Get EmergencyRun Position (3.7)
 * 0x22 (U) Get Frame-ID's
 * (M) = Mandatory
 * (O) = Optional
 * (U) = User defined
 *
 * NOTE: As this function is called by main loop, it is assumed that no Non Volatile Memory
 *       Write is busy.
 * *************************************************************************** *
 * - Call Hierarchy: HandleDfrDiag()
 * - Cyclomatic Complexity: 5+1
 * - Nesting: 3
 * - Function calling: 3 (ValidSupplierFunctionID(), SetupDiagResponse(),
 *                      SetLastError())
 * *************************************************************************** */
static void DfrDiagReadByIdentifier(DFR_DIAG *pDiag)
{
#if (_SUPPORT_BOOTLOADER_PPM != FALSE)
    if ( (pDiag->u.SF.byD1 == 0x33U) &&
         (pDiag->u.SF.byD2 == 0x13U) && (pDiag->u.SF.byD3 == 0x00U) &&
         (pDiag->u.SF.byD5 == 0xCAU) )
    {
        /* Melexis (PPM) Boot-loader */
        if ( (pDiag->u.SF.byD4 == 0xBCU) || (pDiag->u.SF.byD4 == 0xFCU) )
        {
            /* Respond with Project ID */
            g_DiagResponse.byNAD = g_u8NAD;
            g_DiagResponse.byPCI = C_RPCI_READ_BY_ID_01;
            g_DiagResponse.u.SF.byRSID = C_RSID_READ_BY_ID;
            StoreD1to4( (uint16_t)PRODUCT_VERSION_32, (uint16_t)(PRODUCT_VERSION_32 >> 16));   /*lint !e572 */
        }
        else if ( (pDiag->u.SF.byD4 == 0xBDU) || (pDiag->u.SF.byD4 == 0xFDU) )
        {
            /* Reset */
            g_DiagResponse.byNAD = g_u8NAD;
            g_DiagResponse.byPCI = C_RPCI_READ_BY_ID_01;
            g_DiagResponse.u.SF.byRSID = C_RSID_READ_BY_ID;
            StoreD1to4( (uint16_t)PRODUCT_VERSION_32, (uint16_t)(PRODUCT_VERSION_32 >> 16));   /*lint !e572 */
            g_e8LoaderReset = (uint8_t)C_LOADER_CMD_EPM;
        }
#if (_SUPPORT_BOOTLOADER_LIN != FALSE)
        else if (pDiag->u.SF.byD4 == 0xFEU)
        {
            /* Melexis LIN Boot-loader */
            NV_BootLoader(g_u8NAD);

            g_DiagResponse.byNAD = g_u8NAD;
            g_DiagResponse.byPCI = C_RPCI_READ_BY_ID_01;
            g_DiagResponse.u.SF.byRSID = C_RSID_READ_BY_ID;
            StoreD1to4( (uint16_t)PRODUCT_VERSION_32, (uint16_t)(PRODUCT_VERSION_32 >> 16));   /*lint !e572 */
            g_e8LoaderReset = (uint8_t)C_LOADER_CMD_RESET;
        }
#endif /* (_SUPPORT_BOOTLOADER_LIN != FALSE) */
        else
        {
            /* Nothing */
        }
    }
    else
#elif (_SUPPORT_BOOTLOADER_LIN != FALSE)
    if ( (pDiag->u.SF.byD1 == 0x33U) &&
         (pDiag->u.SF.byD2 == 0x13U) && (pDiag->u.SF.byD3 == 0x00U) &&
         (pDiag->u.SF.byD4 == 0xFEU) && (pDiag->u.SF.byD5 == 0xCAU) )
    {
        /* Melexis LIN Boot-loader */
        NV_BootLoader(g_u8NAD);

        g_DiagResponse.byNAD = g_u8NAD;
        g_DiagResponse.byPCI = C_RPCI_READ_BY_ID_01;
        g_DiagResponse.u.SF.byRSID = C_RSID_READ_BY_ID;
        StoreD1to4( (uint16_t)PRODUCT_VERSION_32, (uint16_t)(PRODUCT_VERSION_32 >> 16));   /*lint !e572 */
        g_e8LoaderReset = (uint8_t)C_LOADER_CMD_RESET;
    }
    else
#endif /* (_SUPPORT_BOOTLOADER_LIN != FALSE) */
    if (ValidSupplierFunctionID( (pDiag->u.SF.byD2) | ((uint16_t)(pDiag->u.SF.byD3) << 8),
                                 (pDiag->u.SF.byD4) | ((uint16_t)(pDiag->u.SF.byD5) << 8)) != FALSE)
    {
        if (pDiag->u.SF.byD1 == C_LIN_PROD_ID)
        {
            /* LIN Product identification
             *  +-----+-----+------+----------+----------+----------+----------+----------+
             *  | NAD | PCI | RSID |    D1    |    D2    |    D3    |    D4    |    D5    |
             *  +-----+-----+------+----------+----------+----------+----------+----------+
             *  | NAD | 0x06| 0xF2 | Supplier | Supplier | Function | Function |  Variant |
             *  |     |     |      | ID (LSB) | ID (MSB) | ID (LSB) | ID (MSB) |    ID    |
             *  +-----+-----+------+----------+----------+----------+----------+----------+
             */
            g_DiagResponse.byNAD = g_u8NAD;
            g_DiagResponse.byPCI = C_RPCI_READ_BY_ID_00;
            g_DiagResponse.u.SF.byRSID = C_RSID_READ_BY_ID;
            g_DiagResponse.u.SF.byD5 = C_VARIANT_ID;
#if (LINPROT == LIN2X_HVAC52)
            {
                ENH_LIN_PARAMS_t *pEnhLin = (ENH_LIN_PARAMS_t *)ADDR_NV_ENH_LIN_1;
                StoreD1to4(C_SUPPLIER_ID, pEnhLin->u16FunctionID);
            }
#else  /* (LINPROT == LIN2X_HVAC52) */
            StoreD1to4(C_SUPPLIER_ID, C_FUNCTION_ID);
#endif /* (LINPROT == LIN2X_HVAC52) */
        }
#if (_SUPPORT_DIAG_B2_SERIALNR != FALSE)
        else if (pDiag->u.SF.byD1 == C_SERIAL_NR)
        {
            /* (Optional) Serial number
             *  +-----+-----+------+----------+----------+----------+----------+----------+
             *  | NAD | PCI | RSID |    D1    |    D2    |    D3    |    D4    |    D5    |
             *  +-----+-----+------+----------+----------+----------+----------+----------+
             *  | NAD | 0x05| 0xF2 | SerialNr | SerialNr | SerialNr | SerialNr | Reserved |
             *  |     |     |      |   (LSB)  |          |          |   (MSB)  |          |
             *  +-----+-----+------+----------+----------+----------+----------+----------+
             */
            OPT_LIN_PARAMS_t *pOptLin = (OPT_LIN_PARAMS_t *)ADDR_NV_OPT_LIN;
            g_DiagResponse.byNAD = g_u8NAD;
            g_DiagResponse.byPCI = C_RPCI_READ_BY_ID_01;
            g_DiagResponse.u.SF.byRSID = C_RSID_READ_BY_ID;
            StoreD1to4(pOptLin->u16SerialNumberLSW, pOptLin->u16SerialNumberMSW);   /* Serial-number */
        }
#endif /* (_SUPPORT_DIAG_B2_SERIALNR != FALSE) */
#if (_SUPPORT_ISO17987)
        else if (pDiag->u.SF.byD1 == C_NEG_RESPONSE_ID)
        {
            /* Triggers a negative response, used for bit timing test.
             * Service is defined in the ISO 17987-6:2016, 8.15
             *  +-----+-----+------+----------+----------+----------+----------+----------+
             *  | NAD | PCI | RSID |    D1    |    D2    |    D3    |    D4    |    D5    |
             *  +-----+-----+------+----------+----------+----------+----------+----------+
             *  | NAD | 0x03| 0x7F | Req. SID |    NRC   |  Unused  |  Unused  |  Unused  |
             *  |     |     |      |   0xB2   |   0x12   |   0xFF   |   0xFF   |   0xFF   |
             *  +-----+-----+------+----------+----------+----------+----------+----------+
             */
            SetupDiagResponse(g_u8NAD, pDiag->u.SF.bySID,
                              (uint8_t)C_ERRCODE_SFUNC_NOSUP);                  /* Status = Negative feedback */
        }
        else if (pDiag->u.SF.byD1 == C_NCF_LDF_VERSION_ID)
        {
            /* NCF/LDF version.
             * If no NCF/LDF definition is available for the slave node, a negative response is provided.
             *  +-----+-----+------+----------+----------+----------+----------+----------+
             *  | NAD | PCI | RSID |    D1    |    D2    |    D3    |    D4    |    D5    |
             *  +-----+-----+------+----------+----------+----------+----------+----------+
             *  | NAD | 0x06| 0xF2 |Major Vers|Minor Vers| Sub Vers |  Source  | Reserved |
             *  |     |     |      |  NCF/LDF |  NCF/LDF |  NCF/LDF |          |   0xFF   |
             *  +-----+-----+------+----------+----------+----------+----------+----------+
             */
            g_DiagResponse.byNAD = g_u8NAD;
            g_DiagResponse.byPCI = C_RPCI_READ_BY_ID_03;
            g_DiagResponse.u.SF.byRSID = C_RSID_READ_BY_ID;
            StoreD1to4(C_NCF_LDF_VERSION, C_NCF_LDF_SUBVER_SOURCE);
        }
#endif /* (_SUPPORT_ISO17987) */
#if (_SUPPORT_MLX_DEBUG_MODE != FALSE)
        else if (pDiag->u.SF.byD1 == 0x07U)
        {
            uint16_t u16CurrentBaudrate = p_ml_GetBaudRate(MLX4_FPLL);
            g_DiagResponse.u.SF.byD5 = 0x00U;                                   /* Zero value */
            StoreD1to4(0x4DBA, u16CurrentBaudrate);                             /* Baudrate */
        }
#endif /* (__SUPPORT_MLX_DEBUG_MODE != FALSE) */
#if (_SUPPORT_DIAG_B2_MSG_ID != FALSE)
        else if (pDiag->u.SF.byD1 == C_MSG_ID_1)
        {
            /* Response with Message ID and Protected Frame ID for Control message */
            g_DiagResponse.byNAD = g_u8NAD;
            g_DiagResponse.byPCI = C_RPCI_READ_BY_ID_1X;
            g_DiagResponse.u.SF.byRSID = C_RSID_READ_BY_ID;
            g_DiagResponse.u.SF.byD3 = CalcProtectionBits(g_u8CtrlPID);         /* Add protection bits (MMP161021-2) */
            StoreD1to2(MSG_CONTROL);
        }
        else if (pDiag->u.SF.byD1 == C_MSG_ID_2)
        {
            /* Response with Message ID and Protected Frame ID for Status message */
            g_DiagResponse.byNAD = g_u8NAD;
            g_DiagResponse.byPCI = C_RPCI_READ_BY_ID_1X;
            g_DiagResponse.u.SF.byRSID = C_RSID_READ_BY_ID;
            g_DiagResponse.u.SF.byD3 = CalcProtectionBits(g_u8StsPID);          /* Add protection bits (MMP161021-2) */
            StoreD1to2(MSG_STATUS);
        }
        else if ( (pDiag->u.SF.byD1 >= C_MSG_ID_3) && (pDiag->u.SF.byD1 <= C_MSG_ID_16) )
        {
            SetupDiagResponse(g_u8NAD, pDiag->u.SF.bySID,
                              (uint8_t)C_ERRCODE_SFUNC_NOSUP);                  /* Status = Negative feedback */
        }
#else  /* (_SUPPORT_DIAG_B2_MSG_ID != FALSE) */
        else if ( (pDiag->u.SF.byD1 >= C_MSG_ID_1) && (pDiag->u.SF.byD1 <= C_MSG_ID_16) )
        {
            SetupDiagResponse(g_u8NAD, pDiag->u.SF.bySID,
                              (uint8_t)C_ERRCODE_SFUNC_NOSUP);                  /* Status = Negative feedback */
        }
#endif /* (_SUPPORT_DIAG_B2_MSG_ID != FALSE) */
#if (LINPROT == LIN2X_HVAC49) || (LINPROT == LIN2X_HVAC52) || (LINPROT == LIN2X_AIRVENT12)
        else if (pDiag->u.SF.byD1 == (uint8_t)C_HVAC4x_VERIFY_NAD)
        {
            /* Last NAD, Frame-ID for Control message & Status-message (LH 4.9)
             *  +-----+-----+------+----------+----------+----------+----------+----------+
             *  | NAD | PCI | RSID |    D1    |    D2    |    D3    |    D4    |    D5    |
             *  +-----+-----+------+----------+----------+----------+----------+----------+
             *  | NAD | 0x04| 0xF2 |    NAD   |  Control |  Status  | Reserved | Reserved |
             *  |     |     |      |   NVRAM  | Frame ID | Frame ID |          |          |
             *  +-----+-----+------+----------+----------+----------+----------+----------+
             *
             * Last NAD, Frame-ID for Control message & Status-message (LH 5.2)
             *  +-----+-----+------+----------+----------+----------+----------+----------+
             *  | NAD | PCI | RSID |    D1    |    D2    |    D3    |    D4    |    D5    |
             *  +-----+-----+------+----------+----------+----------+----------+----------+
             *  | NAD | 0x04| 0xF2 |    NAD   |  Control |  Status  |Group Ctrl| Group NAD|
             *  |     |     |      |   NVRAM  | Frame ID | Frame ID | Frame ID |          |
             *  +-----+-----+------+----------+----------+----------+----------+----------+
             *
             * Last NAD, Frame-ID for Control message & Status-message (AirVent)
             *  +-----+-----+------+----------+----------+----------+----------+----------+
             *  | NAD | PCI | RSID |    D1    |    D2    |    D3    |    D4    |    D5    |
             *  +-----+-----+------+----------+----------+----------+----------+----------+
             *  | NAD | 0x04| 0xF2 |    NAD   |  Control |  Status  |  Status2 | Reserved |
             *  |     |     |      |   NVRAM  | Frame ID | Frame ID | Frame ID |          |
             *  +-----+-----+------+----------+----------+----------+----------+----------+
             */
            STD_LIN_PARAMS_t *pStdLin = (STD_LIN_PARAMS_t *)ADDR_NV_STD_LIN_1;
#if (LINPROT == LIN2X_HVAC52) && (_SUPPORT_HVAC_GROUP_ADDRESS != FALSE)
            ENH_LIN_PARAMS_t *pEnhLin = (ENH_LIN_PARAMS_t *)ADDR_NV_ENH_LIN_1;
#endif /* (LINPROT == LIN2X_HVAC52) && (_SUPPORT_HVAC_GROUP_ADDRESS != FALSE) */
            g_DiagResponse.byNAD = g_u8NAD;
#if (LINPROT == LIN2X_HVAC52) && (_SUPPORT_HVAC_GROUP_ADDRESS != FALSE)
            g_DiagResponse.byPCI = (uint8_t)(C_RPCI_READ_BY_ID_21 + 2U);
#else  /* (LINPROT == LIN2X_HVAC52) && (_SUPPORT_HVAC_GROUP_ADDRESS != FALSE) */
            g_DiagResponse.byPCI = (uint8_t)C_RPCI_READ_BY_ID_21;
#endif /* (LINPROT == LIN2X_HVAC52) && (_SUPPORT_HVAC_GROUP_ADDRESS != FALSE) */
            g_DiagResponse.u.SF.byRSID = (uint8_t)C_RSID_READ_BY_ID;
            g_DiagResponse.u.SF.byD1 = (uint8_t)pStdLin->u8NAD;                 /* Stored NAD (NVRAM) */
            g_DiagResponse.u.SF.byD2 = (uint8_t)(pStdLin->u8ControlFrameID & 0x3FU);  /* Frame-ID for Control-message */
            g_DiagResponse.u.SF.byD3 = (uint8_t)(pStdLin->u8StatusFrameID & 0x3FU);   /* Frame-ID for Status-message */
#if (LINPROT == LIN2X_AIRVENT12)
            g_DiagResponse.u.SF.byD4 = (uint8_t)(pStdLin->u8Status2FrameID & 0x3FU);  /* Frame-ID for Status2-message */
#elif (LINPROT == LIN2X_HVAC52) && (_SUPPORT_HVAC_GROUP_ADDRESS != FALSE)
            g_DiagResponse.u.SF.byD4 = pEnhLin->u8GroupControlFrameID;          /* Frame-ID for Group Control-message */
            g_DiagResponse.u.SF.byD5 = pEnhLin->u8GAD;                          /* Group-address */
#endif /* (LINPROT == LIN2X_HVAC52) && (_SUPPORT_HVAC_GROUP_ADDRESS != FALSE) */
            g_u8BufferOutID = (uint8_t)QR_RFR_DIAG;                             /* LIN Output buffer is valid (RFR_DIAG) */
        }
        else if (pDiag->u.SF.byD1 == (uint8_t)C_HVAC4x_SW_HW_REF)
        {
            /* SW & HW reference
             *  +-----+-----+------+----------+----------+----------+----------+----------+
             *  | NAD | PCI | RSID |    D1    |    D2    |    D3    |    D4    |    D5    |
             *  +-----+-----+------+----------+----------+----------+----------+----------+
             *  | NAD | 0x03| 0xF2 | Software | Hardware | Reserved | Reserved | Reserved |
             *  |     |     |      |  Ref. ID |  Ref. ID |          |          |          |
             *  +-----+-----+------+----------+----------+----------+----------+----------+
             */
            STD_LIN_PARAMS_t *pStdLin = (STD_LIN_PARAMS_t *)ADDR_NV_STD_LIN_1;
            g_DiagResponse.byNAD = g_u8NAD;
            g_DiagResponse.byPCI = (uint8_t)C_RPCI_READ_BY_ID_2A;
            g_DiagResponse.u.SF.byRSID = (uint8_t)C_RSID_READ_BY_ID;
            g_DiagResponse.u.SF.byD1 = (uint8_t)C_SW_REF;  /* pStdLin->u8SwRef; */ /* SW-reference */
            g_DiagResponse.u.SF.byD2 = (uint8_t)pStdLin->u8HardwareID;          /* HW-reference */
            g_u8BufferOutID = (uint8_t)QR_RFR_DIAG;                             /* LIN Output buffer is valid (RFR_DIAG) */
        }
#endif /* (LINPROT == LIN2X_HVAC49) || (LINPROT == LIN2X_HVAC52) || (LINPROT == LIN2X_AIRVENT12) */
#if (_SUPPORT_READ_BY_ID_CUSTOMERID != FALSE)
        else if (pDiag->u.SF.byD1 == C_LIN_CUST_ID)
        {
            /* Customer ID
             *  +-----+-----+------+----------+----------+----------+----------+----------+
             *  | NAD | PCI | RSID |    D1    |    D2    |    D3    |    D4    |    D5    |
             *  +-----+-----+------+----------+----------+----------+----------+----------+
             *  | NAD | 0x03| 0xF2 | Customer | Customer | Reserved | Reserved | Reserved |
             *  |     |     |      | ID (LSB) | ID (MSB) |          |          |          |
             *  +-----+-----+------+----------+----------+----------+----------+----------+
             */
            OPT_LIN_PARAMS_t *pOptLin = (OPT_LIN_PARAMS_t *)ADDR_NV_OPT_LIN;
            g_DiagResponse.byNAD = g_u8NAD;
            g_DiagResponse.byPCI = (uint8_t)C_RPCI_READ_BY_ID_3D;
            g_DiagResponse.u.SF.byRSID = (uint8_t)C_RSID_READ_BY_ID;
            StoreD1to2(pOptLin->u16CustomerID);                                 /* Customer-ID */
        }
#endif /* (_SUPPORT_READ_BY_ID_CUSTOMERID != FALSE) */
#if (_SUPPORT_READ_BY_ID_PROD_DATE != FALSE)
        else if (pDiag->u.SF.byD1 == C_PROD_DATE)
        {
            /* Production Date
             *  +-----+-----+------+----------+----------+----------+----------+----------+
             *  | NAD | PCI | RSID |    D1    |    D2    |    D3    |    D4    |    D5    |
             *  +-----+-----+------+----------+----------+----------+----------+----------+
             *  | NAD | 0x03| 0xF2 |Production|Production| Reserved | Reserved | Reserved |
             *  |     |     |      |Date (LSB)|Date (MSB)|          |          |          |
             *  +-----+-----+------+----------+----------+----------+----------+----------+
             */
            OPT_LIN_PARAMS_t *pOptLin = (OPT_LIN_PARAMS_t *)ADDR_NV_OPT_LIN;
            g_DiagResponse.byNAD = g_u8NAD;
            g_DiagResponse.byPCI = (uint8_t)C_RPCI_READ_BY_ID_3E;
            g_DiagResponse.u.SF.byRSID = (uint8_t)C_RSID_READ_BY_ID;
            StoreD1to2(pOptLin->u16ProductionDate);                             /* Production Date */
        }
#endif /* (_SUPPORT_READ_BY_ID_PROD_DATE != FALSE) */
        else
        {
            /* Identifier not supported.
             *  +-----+-----+------+----------+----------+----------+----------+----------+
             *  | NAD | PCI | RSID |    D1    |    D2    |    D3    |    D4    |    D5    |
             *  +-----+-----+------+----------+----------+----------+----------+----------+
             *  | NAD | 0x03| 0x7F |   SID    | ErrorCode| Reserved | Reserved | Reserved |
             *  |     |     |      |  (0xB2)  |  (0x12)  |  (0xFF)  |  (0xFF)  |  (0xFF)  |
             *  +-----+-----+------+----------+----------+----------+----------+----------+
             */
            SetupDiagResponse(g_u8NAD, pDiag->u.SF.bySID,
                              (uint8_t)C_ERRCODE_SFUNC_NOSUP);                  /* Status = Negative feedback */
#if (_SUPPORT_LOG_LIN_ERRORS != FALSE)
            SetLastError(C_ERR_LIN2X_B2);
#endif /* (_SUPPORT_LOG_LIN_ERRORS != FALSE) */
        }
    }
} /* End of DfrDiagReadByIdentifier() */

#if (_SUPPORT_DIAG_B3 != FALSE)
/*!*************************************************************************** *
 * GetdataByte
 * \brief   Get CC-NAD data-byte
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] *pDiag: Pointer to LIN DFR message
 * \param   [in] *pu8DataByte
 * \return  -
 * *************************************************************************** *
 * \details
 * *************************************************************************** *
 * - Call Hierarchy: DfrDiagConditionalChangeNAD()
 * - Cyclomatic Complexity: 8+1
 * - Nesting: 4
 * - Function calling: 0
 * *************************************************************************** */
static uint8_t GetdataByte(DFR_DIAG *pDiag, uint8_t *pu8DataByte)
{
    if (pDiag->u.SF.byD1 == 0x00U)   /* Requested Id = LIN Product Identification */
    {
        if ( (pDiag->u.SF.byD2 == 1U) || (pDiag->u.SF.byD2 == 2U) )
        {
            /* Supplier-ID */
            if (pDiag->u.SF.byD2 == 1U)
            {
                *pu8DataByte = (uint8_t)(C_SUPPLIER_ID & 0xFFU);                /* LSB of Supplier-ID */
            }
            else
            {
                *pu8DataByte = (uint8_t)(C_SUPPLIER_ID >> 8);                   /* MSB of Supplier-ID */
            }
        }
        else if ( (pDiag->u.SF.byD2 == 3U) || (pDiag->u.SF.byD2 == 4U) )
        {
            /* Function-ID */
            if (pDiag->u.SF.byD2 == 3U)
            {
                *pu8DataByte = (uint8_t)(g_NvramUser.FunctionID & 0xFFU);       /* LSB of Function-ID */
            }
            else
            {
                *pu8DataByte = (uint8_t)(g_NvramUser.FunctionID >> 8);          /* MSB of Function-ID */
            }
        }
        else if (pDiag->u.SF.byD2 == 5U)
        {
            *pu8DataByte = g_NvramUser.Variant;
        }
        else
        {
            /* Selected byte not in range, not valid => no response */
            return (C_ERRCODE_INV_MSG_INV_SZ);                                  /* Status = Invalid Format */
        }
    }
    else if (pDiag->u.SF.byD1 == 0x01U)   /* Requested Id = Serial number (optional) */
    {
        if ( (pDiag->u.SF.byD2 == 0U) || (pDiag->u.SF.byD2 > 4U) )
        {
            /* Selected byte not in range, not valid => no response */
            return (C_ERRCODE_INV_MSG_INV_SZ);                                  /* Status = Invalid Format */
        }
        else
        {
            uint8_t *pu8Nvram = (uint8_t *)&g_NvramUser.SerialNumberLSW;
            *pu8DataByte = pu8Nvram[pDiag->u.SF.byD2 - 1U];                     /* Serial-number[n] */
        }
    }
    else
    {
        /* Identifier not supported */
        return (C_ERRCODE_SFUNC_NOSUP);                                         /* Status = Negative feedback */
    }
    return (C_ERRCODE_POSITIVE_RESPONSE);
} /* End of GetdataByte() */

/*!*************************************************************************** *
 * DfrDiagConditionalChangeNAD
 * \brief   Change NAD (Conditionally)
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] *pDiag: Pointer to LIN DFR message
 * \return  -
 * *************************************************************************** *
 * \details
 * *************************************************************************** *
 * - Call Hierarchy: HandleDfrDiag()
 * - Cyclomatic Complexity: 10+1
 * - Nesting: 4
 * - Function calling: 4 (SetupDiagResponse(), SetLastError(), NV_Store(),
 *                      GetdataByte())
 * *************************************************************************** */
static void DfrDiagConditionalChangeNAD(DFR_DIAG *pDiag)
{
    /*
     *  +-----+-----+-----+----------+----------+----------+----------+----------+
     *  | NAD | PCI | SID |    D1    |    D2    |    D3    |    D4    |    D5    |
     *  +-----+-----+-----+----------+----------+----------+----------+----------+
     *  | NAD | 0x06| 0xB3|Identifier|   Byte   |   Mask   |  Invert  | New NAD  |
     *  +-----+-----+-----+----------+----------+----------+----------+----------+
     */
    /* Get the identifier of possible read by ID response and selected by Id */
    /* Extract the data byte selected by Byte */
    uint8_t u8DataByte;
    uint8_t u8ResponseCode = GetdataByte(pDiag, &u8DataByte);
    if (u8ResponseCode != C_ERRCODE_POSITIVE_RESPONSE)
    {
        SetupDiagResponse(g_u8NAD, pDiag->u.SF.bySID, u8ResponseCode);          /* Status = Negative feedback */
#if (_SUPPORT_LOG_ERRORS != FALSE)
        SetLastError(C_ERR_LIN2X_B3);
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
        return;
    }

    /* Do a bitwise XOR with Invert */
    u8DataByte ^= pDiag->u.SF.byD4;

    /* Do a bitwise AND with Mask */
    u8DataByte &= pDiag->u.SF.byD3;

    if (u8DataByte == 0U)                                                       /* Condition PASSED */
    {
        uint8_t byInitialNAD = g_NvramUser.NAD;
        SetupDiagResponse(byInitialNAD, pDiag->u.SF.bySID,
                          (uint8_t)C_ERRCODE_PENDING);                          /* Status = Pending */
#if (_SUPPORT_NAD_RANGE_CHECK != FALSE)
        /* SAE J2602: NAD range: 0x60-0x6F */
        /* ISO 17987: NAD range: 0x01-0x7F */
#endif /* (_SUPPORT_NAD_RANGE_CHECK != FALSE) */
        g_u8NAD = g_NvramUser.NAD = pDiag->u.SF.byD5;                           /* New NAD */
        /* Store NVRAM */
        if ( (NV_Store(C_NV_USER_PAGE_ALL) == C_NV_STORE_OKAY) && (g_NvramUser.NAD == pDiag->u.SF.byD5) )
        {
            /* NAD changed */
            SetupDiagResponse(g_u8NAD, pDiag->u.SF.bySID,
                              (uint8_t)C_ERRCODE_POSITIVE_RESPONSE);            /* Status = Positive feedback */
        }
        else
        {
            /* NAD couldn't be changed */
            SetupDiagResponse(byInitialNAD, pDiag->u.SF.bySID,
                              (uint8_t)C_ERRCODE_SFUNC_NOSUP);                  /* Status = Negative feedback */
#if (_SUPPORT_LOG_ERRORS != FALSE)
            SetLastError(C_ERR_LIN2X_B3);
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
        }
    }
} /* End of DfrDiagConditionalChangeNAD() */
#endif /* (_SUPPORT_DIAG_B3 != FALSE) */

#if (_SUPPORT_DIAG_B4 != FALSE)
#if (LINPROT == LIN2X_HVAC49) || (LINPROT == LIN2X_HVAC52) || (LINPROT == LIN2X_AIRVENT12)
/*!*************************************************************************** *
 * DfrDiagAssignVariantID
 * \brief   Dump Data
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] *pDiag: Pointer to LIN DFR message
 * \return  -
 * *************************************************************************** *
 * \details Assign Variant-ID, HW reference and SW reference
 *          The Variant-ID, HW reference and SW reference must be able to be
 *          programmed by a special "Assign command" and their plausibility must
 *          be able to be checked by a positive or negative response message.
 *          If for a particular datum no new value is to be or can be programmed,
 *          "0xFF" is sent by the Master in the corresponding data byte.
 *  +-----+-----+-----+----------+----------+----------+----------+----------+
 *  | NAD | PCI | SID |    D1    |    D2    |    D3    |    D4    |    D5    |
 *  +-----+-----+-----+----------+----------+----------+----------+----------+
 *  | NAD | 0x06| 0xB3| Supplier | Supplier |    New   |    New   |    New   |
 *  |     |     |     | ID (LSB) | ID (MSB) |  Variant |  HW-Ref  |  SW-Ref  |
 *  +-----+-----+-----+----------+----------+----------+----------+----------+
 *
 * NOTE: As this function is called by main loop, it is assumed that no Non Volatile Memory
 *       Write is busy.
 * *************************************************************************** *
 * - Call Hierarchy: HandleDfrDiag()
 * - Cyclomatic Complexity: 3+1
 * - Nesting: 3
 * - Function calling: 1 (CheckSupplier())
 * *************************************************************************** */
static void DfrDiagAssignVariantID(DFR_DIAG *pDiag)
{
    if (CheckSupplier( (pDiag->u.SF.byD1) | ((uint16_t)(pDiag->u.SF.byD2) << 8)) != FALSE)
    {
        STD_LIN_PARAMS_t StdLinRam;
        uint16_t u16NvramStoreResult = FALSE;

        SetupDiagResponse(g_u8NAD, pDiag->u.SF.bySID,
                          (uint8_t)C_ERRCODE_PENDING);                          /* Status = Pending */
        p_CopyU16( (sizeof(STD_LIN_PARAMS_t) / sizeof(uint16_t)),
                   (uint16_t *)&StdLinRam,
                   (const uint16_t *)ADDR_NV_STD_LIN_1);
        if (pDiag->u.SF.byD3 != 0xFFU)
        {
            StdLinRam.u8Variant = pDiag->u.SF.byD3;
            u16NvramStoreResult = TRUE;
        }
        if (pDiag->u.SF.byD4 != 0xFFU)
        {
            StdLinRam.u8HardwareID = pDiag->u.SF.byD4;
            u16NvramStoreResult = TRUE;
        }
#if (LINPROT != LIN2X_AIRVENT12)
        if (pDiag->u.SF.byD5 != 0xFFU)
        {
            StdLinRam.u8SoftwareID = pDiag->u.SF.byD5;
            u16NvramStoreResult = TRUE;
        }
#else  /* (LINPROT != LIN2X_AIRVENT12) */
        /* Software ID can't be overwritten as it is defined by the application itself */
#endif /* (LINPROT != LIN2X_AIRVENT12) */

        if (u16NvramStoreResult != FALSE)
        {
            u16NvramStoreResult = NV_WriteLIN_STD(&StdLinRam, C_NV_WRT_LIN_ID_ALL);   /* Store NVRAM */
        }

        /* Store NVRAM */
        if (u16NvramStoreResult == C_ERR_NONE)
        {
            /* NAD changed.
             *  +-----+-----+------+----------+----------+----------+----------+----------+
             *  | NAD | PCI | RSID |    D1    |    D2    |    D3    |    D4    |    D5    |
             *  +-----+-----+------+----------+----------+----------+----------+----------+
             *  | Init| 0x01| 0xF0 | Reserved | Reserved | Reserved | Reserved | Reserved |
             *  | NAD |     |      |  (0xFF)  |  (0xFF)  |  (0xFF)  |  (0xFF)  |  (0xFF)  |
             *  +-----+-----+------+----------+----------+----------+----------+----------+
             */
            SetupDiagResponse(g_u8NAD, pDiag->u.SF.bySID,
                              (uint8_t)C_ERRCODE_POSITIVE_RESPONSE);            /* Status = Positive feedback */
        }
        else
        {
            /* NAD couldn't be changed.
             *  +-----+-----+------+----------+----------+----------+----------+----------+
             *  | NAD | PCI | RSID |    D1    |    D2    |    D3    |    D4    |    D5    |
             *  +-----+-----+------+----------+----------+----------+----------+----------+
             *  | Init| 0x03| 0x7F |   SID    | ErrorCode| Reserved | Reserved | Reserved |
             *  | NAD |     |      |  (0xB0)  |  (0x12)  |  (0xFF)  |  (0xFF)  |  (0xFF)  |
             *  +-----+-----+------+----------+----------+----------+----------+----------+
             */
            SetupDiagResponse(g_u8NAD, pDiag->u.SF.bySID,
                              (uint8_t)C_ERRCODE_SFUNC_NOSUP);                  /* Status = Negative feedback */
#if (_SUPPORT_LOG_LIN_ERRORS != FALSE)
            SetLastError(C_ERR_LIN2X_B4);
#endif /* (_SUPPORT_LOG_LIN_ERRORS != FALSE) */
        }
    }
} /* End of DfrDiagAssignVariantID() */

#else  /* (LINPROT == LIN2X_HVAC49) || (LINPROT == LIN2X_HVAC52) || (LINPROT == LIN2X_AIRVENT12) */

/*!*************************************************************************** *
 * DfrDiagDumpData
 * \brief   Dump Data
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] *pDiag: Pointer to LIN DFR message
 * \return  -
 * *************************************************************************** *
 * \details Dump Data.
 * *************************************************************************** *
 * - Call Hierarchy: HandleDfrDiag()
 * - Cyclomatic Complexity: 3+1
 * - Nesting: 3
 * - Function calling: 1 (CheckSupplier())
 * *************************************************************************** */
static void DfrDiagDumpData(DFR_DIAG *pDiag)
{
    /*
     *  +-----+-----+-----+----------+----------+----------+----------+----------+
     *  | NAD | PCI | SID |    D1    |    D2    |    D3    |    D4    |    D5    |
     *  +-----+-----+-----+----------+----------+----------+----------+----------+
     *  |     |     |     | Supplier | Supplier | Parameter| Parameter|   SubID  |
     *  | NAD | 0x06| 0xB4| ID (LSB) | ID (MSB) |   (LSB)  |   (MSB)  |          |
     *  +-----+-----+-----+----------+----------+----------+----------+----------+
     */
}
#endif /* (LINPROT == LIN2X_HVAC49) || (LINPROT == LIN2X_HVAC52) || (LINPROT == LIN2X_AIRVENT12) */
#endif /* (_SUPPORT_DIAG_B4 != FASLE) */

#if (_SUPPORT_LIN_AA != FALSE)
/*!*************************************************************************** *
 * DfrDiagAssignNAD
 * \brief   Change NAD (Conditionally)
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] *pDiag: Pointer to LIN DFR message
 * \return  -
 * *************************************************************************** *
 * \details Assign NAD (Slave Node Position Detection, SNPD)
 *          This is a broadcast LIN-command; Therefore no feedback is returned
 *          Beginning with the BSM-Initialisation, all SNPD nodes with BSM
 *          capability start their measurement sequence within the next
 *          break field.
 * NOTE: As this function is called by main loop, it is assumed that no Non Volatile Memory
 *       Write is busy.
 * *************************************************************************** *
 * - Call Hierarchy: HandleDfrDiag()
 * - Cyclomatic Complexity: 9+1
 * - Nesting: 6
 * - Function calling: 10 (CheckSupplier(), MotorDriverStop(),
 *                         ml_SetSlaveNotAddressed(), ClearAAData(),
 *                         ml_InitAutoAddressing(), ml_GetAutoaddressingStatus(),
 *                         ml_SetLoaderNAD(), ml_SetSlaveAddressed(),
 *                         NV_Store(), ml_StopAutoAddressing())
 * *************************************************************************** */
static void DfrDiagAssignNAD(DFR_DIAG *pDiag)
{
    /* This is a broadcast LIN-command; Therefore no feedback is returned */
    /* Beginning with the BSM-Initialisation, all SNPD nodes with BSM
     * capability start their measurement sequence within the next
     * break field.
     */
    if ( (CheckSupplier( (pDiag->u.SF.byD1) | ((uint16_t)(pDiag->u.SF.byD2) << 8)) != FALSE) &&
         (pDiag->u.SF.byD4 == (uint8_t)C_SNPD_METHOD_BSM2) )                    /* SNPD Method ID F1 */
    {
        if ( (pDiag->u.SF.byD3 == (uint8_t)C_SNPD_SUBFUNC_START) &&
             (g_u8LinAAMode == (uint8_t)C_SNPD_SUBFUNC_INACTIVE) )              /* Sub-function ID */
        {
            /* BSM Initialisation (only if not already in AA-mode) */
            /*
             *  +-----+-----+-----+----------+----------+----------+----------+----------+
             *  | NAD | PCI | SID |    D1    |    D2    |    D3    |    D4    |    D5    |
             *  +-----+-----+-----+----------+----------+----------+----------+----------+
             *  |     |     |     | Supplier | Supplier | Function | Function |  Unused  |
             *  | NAD | 0x06| 0xB5| ID (LSB) | ID (MSB) | ID (LSB) | ID (MSB) |          |
             *  |     |     | 0xB8|  (0xFF)  |  (0x7F)  |  (0x01)  |  (0xF1)  |  (0xFF)  |
             *  +-----+-----+-----+----------+----------+----------+----------+----------+
             * All SNPD slaves with BSM capability start their measurement
             * sequence with the next break field.
             * Function ID (MSB): 0xF1 = Bus Shunt Method
             * Function ID (LSB): 0x01 = BSM Initialisation
             */
#if (_SUPPORT_APP_TYPE == C_APP_SOLENOID)
            if ( (g_e8SolenoidStatus & C_SOLENOID_STATUS_MASK) != C_SOLENOID_STATUS_DEACTIVATED)  /* If solenoid is not deactivated ... */
            {
                SolenoidDriverDeactivate();                                     /* ... deactivate solenoid NOW (LIN-AA) */
            }
#else  /* (_SUPPORT_APP_TYPE == C_APP_SOLENOID) */
            if ( (g_e8MotorStatus & C_MOTOR_STATUS_STOP_MASK) != C_MOTOR_STATUS_STOP)    /* If actuator is not stopped ... */
            {
                MotorDriverStop( (uint16_t)C_STOP_WO_HOLDING);                  /* ... stop actuator NOW (LIN-AA) */
            }
#endif
            g_u8ChipResetOcc = FALSE;
            g_u8StallOcc = FALSE;
            /* g_e8EmergencyRunOcc = (uint8_t) C_SAFETY_RUN_NO; */
#if (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT)
            g_e8MotorDirectionCCW = (uint8_t)C_MOTOR_DIR_UNKNOWN;               /* Direction is unknown (9.5.3.13) */
#endif /* (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT) */

            ml_SetSlaveNotAddressed();                                          /* (Test) Allow re-addressing */
            g_u8NAD = C_INVALID_NAD;                                            /* Invalidate NAD (MMP190801-1) */
#if (LIN_AA_INFO != FALSE)
            ClearAAData();                                                      /* Clear AA-ModuleScreening data */
#endif /* (LIN_AA_INFO != FALSE) */
            g_u16LinAATicker = PI_TICKS_PER_SECOND;                             /* Re-start g_u16LinAATicker to time LIN-AA timeout of 40sec */
            g_u8LinAATimeout = (uint8_t)C_LINAA_TIMEOUT;                        /* LIN-AA time-out counter (seconds) */
            g_u8LinAAMode = (uint8_t)C_SNPD_SUBFUNC_START;                      /* LIN-AA mode (BSM-initialise) */
#if (LIN_AA_BSM_SNPD_R1p0 != FALSE)
            ml_InitAutoAddressing();
#endif /* (LIN_AA_BSM_SNPD_R1p0 != FALSE) */
        }
        else if ( (pDiag->u.SF.byD3 == (uint8_t)C_SNPD_SUBFUNC_ADDR) &&
                  (g_u8LinAAMode == (uint8_t)C_SNPD_SUBFUNC_START) )
        {
            /* Assign NAD to last not-addressed slave (only in AA-mode) */
            /*
             *  +-----+-----+-----+----------+----------+----------+----------+----------+
             *  | NAD | PCI | SID |    D1    |    D2    |    D3    |    D4    |    D5    |
             *  +-----+-----+-----+----------+----------+----------+----------+----------+
             *  |     |     |     | Supplier | Supplier | Function | Function |          |
             *  | NAD | 0x06| 0xB5| ID (LSB) | ID (MSB) | ID (LSB) | ID (MSB) |  New NAD |
             *  |     |     | 0xB8|  (0xFF)  |  (0x7F)  |  (0x02)  |  (0xF1)  |          |
             *  +-----+-----+-----+----------+----------+----------+----------+----------+
             * All SNPD slaves with BSM capability start their measurement sequence
             * within the break field; after the break the selected SNPD slave takes
             * the NAD.
             */
            if (ml_GetAutoaddressingStatus() != FALSE)
            {
#if (_SUPPORT_NAD_RANGE_CHECK != FALSE)
#if ((LINPROT & LINXX) == LIN2J)
                /* SAE J2602: NAD range: 0x60-0x6F */
                if ((pDiag->u.SF.byD5 >= 0x60U) && (pDiag->u.SF.byD5 <= 0x6FU))
#elif (((LINPROT & LINXX) >= LIN20) && ((LINPROT & LINXX) <= LIN22)) || ((LINPROT & LINXX) == LIN2X)
                /* ISO 17987: NAD range: 0x01-0x7F */
                if ((pDiag->u.SF.byD5 >= 0x01U) && (pDiag->u.SF.byD5 <= 0x7FU))
#endif
#endif /* (_SUPPORT_NAD_RANGE_CHECK != FALSE) */
                {
                    g_u8NAD = (pDiag->u.SF.byD5);                               /* New NAD (into RAM) */
                    ml_SetSlaveAddressed();
                }
            }
        }
        else if ( (pDiag->u.SF.byD3 == (uint8_t)C_SNPD_SUBFUNC_STORE) &&
                  (g_u8LinAAMode == (uint8_t)C_SNPD_SUBFUNC_START) )
        {
            /* Store NAD in slave (optional) (only in AA-mode) */
            /*
             *  +-----+-----+-----+----------+----------+----------+----------+----------+
             *  | NAD | PCI | SID |    D1    |    D2    |    D3    |    D4    |    D5    |
             *  +-----+-----+-----+----------+----------+----------+----------+----------+
             *  |     |     |     | Supplier | Supplier | Function | Function |  Unused  |
             *  | NAD | 0x06| 0xB5| ID (LSB) | ID (MSB) | ID (LSB) | ID (MSB) |          |
             *  |     |     | 0xB8|  (0xFF)  |  (0x7F)  |  (0x03)  |  (0xF1)  |  (0xFF)  |
             *  +-----+-----+-----+----------+----------+----------+----------+----------+
             * All SNPD slaves with BSM capability store their new NAD from the
             * RAM in to the NVM, if available.
             */
            if (g_u8NAD != C_INVALID_NAD)                                       /* Check NAD not invalid (MMP190801-1) */
            {
                STD_LIN_PARAMS_t StdLinRam;
                p_CopyU16( (sizeof(STD_LIN_PARAMS_t) / sizeof(uint16_t)),
                           (uint16_t *)&StdLinRam,
                           (const uint16_t *)ADDR_NV_STD_LIN_1);
                StdLinRam.u8NAD = g_u8NAD;
                (void)NV_WriteLIN_STD(&StdLinRam, C_NV_WRT_LIN_ID_ALL);
                g_u8LinAAMode = (uint8_t)C_SNPD_SUBFUNC_STORE;
            }
        }
        else if ( (pDiag->u.SF.byD3 == (uint8_t)C_SNPD_SUBFUNC_FINISH) &&
                  (g_u8LinAAMode != (uint8_t)C_SNPD_SUBFUNC_INACTIVE) )
        {
            /* Assign NAD Finished (Only in LIN-AA mode) */
            /*
             *  +-----+-----+-----+----------+----------+----------+----------+----------+
             *  | NAD | PCI | SID |    D1    |    D2    |    D3    |    D4    |    D5    |
             *  +-----+-----+-----+----------+----------+----------+----------+----------+
             *  |     |     |     | Supplier | Supplier | Function | Function |  Unused  |
             *  | NAD | 0x06| 0xB5| ID (LSB) | ID (MSB) | ID (LSB) | ID (MSB) |          |
             *  |     |     | 0xB8|  (0xFF)  |  (0x7F)  |  (0x04)  |  (0xF1)  |  (0xFF)  |
             *  +-----+-----+-----+----------+----------+----------+----------+----------+
             * All SNPD slaves with BSM capability stop their measurement sequence
             * in the break field.
             */
#if (LIN_AA_BSM_SNPD_R1p0 != FALSE)
            ml_StopAutoAddressing();
#endif /* (LIN_AA_BSM_SNPD_R1p0 != FALSE) */
#if (LIN_AA_TEST_DUT == FALSE)
            if (g_u8NAD == C_INVALID_NAD)                                       /* In case of Invalid NAD, restore last saved NAD (MMP190801-1) */
            {
                STD_LIN_PARAMS_t *pStdLin = (STD_LIN_PARAMS_t *)ADDR_NV_STD_LIN_1;
                g_u8NAD = pStdLin->u8NAD;                                       /* Restore original NAD */
            }
#else  /* (LIN_AA_TEST_DUT == FALSE) */
            g_u8NAD = (uint8_t)C_TEST_DUT_NAD;
#endif /* (LIN_AA_TEST_DUT == FALSE) */
            g_u16LinAATicker = 0U;
            g_u8LinAAMode = (uint8_t)C_SNPD_SUBFUNC_INACTIVE;
        }
        else
        {
            /* Nothing */
        }
    }
} /* End of DfrDiagAssignNAD() */
#endif /* (_SUPPORT_LIN_AA != FALSE) */

#if (_SUPPORT_ISO17987) || ((LINPROT & LINXX) == LIN2J)
/*!*************************************************************************** *
 * DfrDiagTargetedReset
 * \brief   Targeted Reset
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] *pDiag: Pointer to LIN DFR message
 * \return  -
 * *************************************************************************** *
 * \details Reset the Target in case NAD matches
 *  +-----+-----+------+----------+----------+----------+----------+----------+
 *  | NAD | PCI |  SID |    D1    |    D2    |    D3    |    D4    |    D5    |
 *  +-----+-----+------+----------+----------+----------+----------+----------+
 *  | NAD | 0x01| 0xB5 | Reserved | Reserved | Function | Reserved | Reserved |
 *  |     |     |      |  (0xFF)  |  (0xFF)  |  (0xFF)  |  (0xFF)  |  (0xFF)  |
 *  +-----+-----+------+----------+----------+----------+----------+----------+
 *
 * *************************************************************************** *
 * - Call Hierarchy: HandleDfrDiag()
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 0
 * *************************************************************************** */
void DfrDiagTargetedReset(DFR_DIAG *pDiag)
{
    AppReset();
    (void) pDiag;
} /* End of DfrDiagTargetedReset() */
#endif /* (_SUPPORT_ISO17987) || () */

#if (_SUPPORT_DIAG_B6 != FALSE)
#if (LINPROT == LIN2X_HVAC52)
/*!*************************************************************************** *
 * DfrDiagAssignGroupAddress
 * \brief   Assign Group address)
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] *pDiag: Pointer to LIN DFR message
 * \return  -
 * *************************************************************************** *
 * \details Assign Group address is used to resolve conflicting GAD in LIN clusters
 *          built using off-the-shelves slave nodes or reused slave nodes.
 *  +-----+-----+------+----------+----------+----------+----------+----------+
 *  | NAD | PCI |  SID |    D1    |    D2    |    D3    |    D4    |    D5    |
 *  +-----+-----+------+----------+----------+----------+----------+----------+
 *  | NAD | 0x06| 0xB6 | Supplier | Supplier | Function | Function |   Group  |
 *  |     |     |      | ID (LSB) | ID (MSB) | ID (LSB) | ID (MSB) |  Address |
 *  +-----+-----+------+----------+----------+----------+----------+----------+
 *          Valid Group Address range: 0x80-0x9F
 *          Special: 0xFF to unassigns
 *
 * NOTE: As this function is called by main loop, it is assumed that no Non Volatile Memory
 *       Write is busy.
 * *************************************************************************** *
 * - Call Hierarchy: HandleDfrDiag()
 * - Cyclomatic Complexity: 3+1
 * - Nesting: 2
 * - Function calling: 6 (SetupDiagResponse(), NV_WriteLIN_ENH(), ml_Disconnect(),
 *                        ml_AssignFrameToMessageID(), ml_DisableMessage(), ml_Connect())
 * *************************************************************************** */
static void DfrDiagAssignGroupAddress(DFR_DIAG *pDiag)
{
    if ( (ValidSupplierFunctionID( (pDiag->u.SF.byD1) | ((uint16_t)(pDiag->u.SF.byD2) << 8),
                                   (pDiag->u.SF.byD3) | ((uint16_t)(pDiag->u.SF.byD4) << 8)) != FALSE) &&
         (((pDiag->u.SF.byD1 >= 0x80U) && (pDiag->u.SF.byD1 <= 0x9F)) ||        /* The group address must be only in the range of 0x80 to 0x9F. */
          (pDiag->u.SF.byD1 == 0xFFU)) )                                        /* Unassigns group address */
    {
        ENH_LIN_PARAMS_t *pEnhLin = (ENH_LIN_PARAMS_t *)ADDR_NV_ENH_LIN_1;
        ENH_LIN_PARAMS_t EnhLinRam;
        SetupDiagResponse(g_u8NAD, pDiag->u.SF.bySID,
                          (uint8_t)C_ERRCODE_PENDING);                          /* Status = Pending */
        p_CopyU16( (sizeof(ENH_LIN_PARAMS_t) / sizeof(uint16_t)),
                   (uint16_t *)&EnhLinRam,
                   (const uint16_t *)ADDR_NV_ENH_LIN_1);
        l_u8GAD = pDiag->u.SF.byD5;
        EnhLinRam.u8GAD = l_u8GAD;

        (void)ml_Disconnect();
        if (l_u8GAD != 0xFFU)
        {
            (void)ml_AssignFrameToMessageID(MSG_GROUP_CONTROL, l_u8GAD);
        }
        else
        {
            (void)ml_DisableMessage(MSG_GROUP_CONTROL);
        }
#if (_SUPPORT_LIN_BUS_ACTIVITY_CHECK != FALSE)
#if (MSG_GROUP_CONTROL <= 7)
        /* Dynamic Message ID's: 0...7 */
        g_u16MLX4_RAM_Dynamic_CRC1 = p_CalcCRC_U16((const uint16_t *)C_RAM_MLX4_DYNAMIC_BGN_ADR1,
                                                   (C_RAM_MLX4_DYNAMIC_END_ADR1 - C_RAM_MLX4_DYNAMIC_BGN_ADR1)/sizeof(uint16_t));
#else /* (MSG_GROUP_CONTROL <= 7) */
        /* Dynamic Message ID's: 8...15 */
        g_u16MLX4_RAM_Dynamic_CRC2 = p_CalcCRC_U16((const uint16_t *)C_RAM_MLX4_DYNAMIC_BGN_ADR2,
                                                   (C_RAM_MLX4_DYNAMIC_END_ADR2 - C_RAM_MLX4_DYNAMIC_BGN_ADR2)/sizeof(uint16_t));
#endif /* (MSG_GROUP_CONTROL <= 7) */
#endif /* (_SUPPORT_LIN_BUS_ACTIVITY_CHECK != FALSE) */
        (void)ml_Connect();

        /* Store NVRAM */
        if ( (NV_WriteLIN_ENH(&EnhLinRam, C_NV_WRT_LIN_ID_ALL) == C_ERR_NONE) &&
             (pEnhLin->u8GAD == pDiag->u.SF.byD5) )
        {
            /* NAD changed.
             *  +-----+-----+------+----------+----------+----------+----------+----------+
             *  | NAD | PCI | RSID |    D1    |    D2    |    D3    |    D4    |    D5    |
             *  +-----+-----+------+----------+----------+----------+----------+----------+
             *  | Init| 0x01| 0xF0 | Reserved | Reserved | Reserved | Reserved | Reserved |
             *  | NAD |     |      |  (0xFF)  |  (0xFF)  |  (0xFF)  |  (0xFF)  |  (0xFF)  |
             *  +-----+-----+------+----------+----------+----------+----------+----------+
             */
            SetupDiagResponse(g_u8NAD, pDiag->u.SF.bySID,
                              (uint8_t)C_ERRCODE_POSITIVE_RESPONSE);            /* Status = Positive feedback */
        }
        else
        {
            /* NAD couldn't be changed.
             *  +-----+-----+------+----------+----------+----------+----------+----------+
             *  | NAD | PCI | RSID |    D1    |    D2    |    D3    |    D4    |    D5    |
             *  +-----+-----+------+----------+----------+----------+----------+----------+
             *  | Init| 0x03| 0x7F |   SID    | ErrorCode| Reserved | Reserved | Reserved |
             *  | NAD |     |      |  (0xB0)  |  (0x12)  |  (0xFF)  |  (0xFF)  |  (0xFF)  |
             *  +-----+-----+------+----------+----------+----------+----------+----------+
             */
            SetupDiagResponse(g_u8NAD, pDiag->u.SF.bySID,
                              (uint8_t)C_ERRCODE_SFUNC_NOSUP);                  /* Status = Negative feedback */
#if (_SUPPORT_LOG_LIN_ERRORS != FALSE)
            SetLastError(C_ERR_LIN2X_B6);
#endif /* (_SUPPORT_LOG_LIN_ERRORS != FALSE) */
        }
    }
} /* End of DfrDiagAssignGroupAddress() */

#else  /* (LINPROT == LIN2X_HVAC52) */

/*!*************************************************************************** *
 * DfrDiagSaveConfig
 * \brief   Save Configuration
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] *pDiag: Pointer to LIN DFR message
 * \return  -
 * *************************************************************************** *
 * \details This service tells the slave node(s) that the slave application
 *          shall save the current configuration. This service is used to
 *          notify a slave node to store its configuration. A configuration
 *          in the slave node may be valid even without the master node
 *          using this request (i.e. the slave node does not have to wait
 *          for this request to have a valid configuration).
 * NOTE: As this function is called by main loop, it is assumed that no Non Volatile Memory
 *       Write is busy.
 * *************************************************************************** *
 * - Call Hierarchy: HandleDfrDiag()
 * - Cyclomatic Complexity: 1+1
 * - Nesting: 1
 * - Function calling: 1 (SetupDiagResponse())
 * *************************************************************************** */
static void DfrDiagSaveConfig(DFR_DIAG *pDiag)
{
    uint16_t u16Result = C_ERR_NONE;
#if ((_SUPPORT_DIAG_B0 != FALSE) && (_SUPPORT_DIAG_B0_NV_STORE == FALSE)) || \
    ((_SUPPORT_DIAG_B7 != FALSE) && (_SUPPORT_DIAG_B7_NV_STORE == FALSE))
    STD_LIN_PARAMS_t StdLinRam;

    p_CopyU16( (sizeof(STD_LIN_PARAMS_t) / sizeof(uint16_t)),
               (uint16_t *)&StdLinRam,
               (const uint16_t *)ADDR_NV_STD_LIN_1);
#endif /* (_SUPPORT_DIAG_Bx != FALSE) && (_SUPPORT_DIAG_Bx_NV_STORE == FALSE) */

    /*  +-----+-----+-----+----------+----------+----------+----------+----------+
     *  | NAD | PCI | SID |    D1    |    D2    |    D3    |    D4    |    D5    |
     *  +-----+-----+-----+----------+----------+----------+----------+----------+
     *  | NAD | 0x01| 0xB6| Reserved | Reserved | Reserved | Reserved | Reserved |
     *  +-----+-----+-----+----------+----------+----------+----------+----------+
     */
    SetupDiagResponse(g_u8NAD, pDiag->u.SF.bySID,
                      (uint8_t)C_ERRCODE_PENDING);                              /* Status = Pending */
#if (_SUPPORT_DIAG_B0 != FALSE) && (_SUPPORT_DIAG_B0_NV_STORE == FALSE)
    if ( (g_u8NAD >= 0x01U) && (g_u8NAD <= 0x7DU) )
    {
        lin.u8NAD = g_u8NAD;
    }
#endif /* (_SUPPORT_DIAG_B0 != FALSE) && (_SUPPORT_DIAG_B0_NV_STORE == FALSE) */
#if (_SUPPORT_DIAG_B7 != FALSE) && (_SUPPORT_DIAG_B7_NV_STORE == FALSE)
    StdLinRam.u8ControlFrameID = g_u8CtrlPID;
    StdLinRam.u8StatusFrameID = g_u8StsPID;
#endif /* (_SUPPORT_DIAG_B7 != FALSE) && (_SUPPORT_DIAG_B7_NV_STORE == FALSE) */
#if ((_SUPPORT_DIAG_B0 != FALSE) && (_SUPPORT_DIAG_B0_NV_STORE == FALSE)) || \
    ((_SUPPORT_DIAG_B7 != FALSE) && (_SUPPORT_DIAG_B7_NV_STORE == FALSE))
    u16Result |= NV_WriteLIN_STD(&StdLinRam, C_NV_WRT_LIN_ID_ALL);
#endif /* (_SUPPORT_DIAG_Bx != FALSE) && (_SUPPORT_DIAG_Bx_NV_STORE == FALSE) */
#if (_SUPPORT_CALIBRATION != FALSE)
    if (g_e8CalibrationStep < (uint8_t)C_CALIB_DONE)
    {
        APP_EOL_t eol;
        p_CopyU16( (sizeof(APP_EOL_t) / sizeof(uint16_t)),
                   (uint16_t *)&eol,
                   (const uint16_t *)ADDR_NV_EOL);
        eol.u16RealTravel = g_u16RealTravel;
        u16Result |= NV_WriteEOL(&eol);
    }
#endif /* (_SUPPORT_CALIBRATION != FALSE) */
    if (u16Result == C_ERR_NONE)
    {
        /* NAD and/or EmergencyRun Position changed.
         *  +-----+-----+------+----------+----------+----------+----------+----------+
         *  | NAD | PCI | RSID |    D1    |    D2    |    D3    |    D4    |    D5    |
         *  +-----+-----+------+----------+----------+----------+----------+----------+
         *  | NAD | 0x01| 0xF6 | Reserved | Reserved | Reserved | Reserved | Reserved |
         *  |     |     |      |  (0xFF)  |  (0xFF)  |  (0xFF)  |  (0xFF)  |  (0xFF)  |
         *  +-----+-----+------+----------+----------+----------+----------+----------+
         */
        SetupDiagResponse(g_u8NAD, pDiag->u.SF.bySID,
                          (uint8_t)C_ERRCODE_POSITIVE_RESPONSE);                /* Status = Positive feedback */
    }
    else
    {
        /* NAD and/or EmergencyRun Position counldn't be changed.
         *  +-----+-----+------+----------+----------+----------+----------+----------+
         *  | NAD | PCI | RSID |    D1    |    D2    |    D3    |    D4    |    D5    |
         *  +-----+-----+------+----------+----------+----------+----------+----------+
         *  | NAD | 0x03| 0x7F |   SID    | ErrorCode| Reserved | Reserved | Reserved |
         *  |     |     |      |  (0xB6)  |  (0x12)  |  (0xFF)  |  (0xFF)  |  (0xFF)  |
         *  +-----+-----+------+----------+----------+----------+----------+----------+
         */
        SetupDiagResponse(g_u8NAD, pDiag->u.SF.bySID,
                          (uint8_t)C_ERRCODE_SFUNC_NOSUP);                      /* Status = Negative feedback */
#if (_SUPPORT_LOG_ERRORS != FALSE)
        SetLastError(C_ERR_LIN2X_B6);
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
    }
} /* End of DfrDiagSaveConfig() */
#endif /* (LINPROT == LIN2X_HVAC52) */
#endif /* (_SUPPORT_DIAG_B6 != FALSE) */

#if (_SUPPORT_DIAG_B7 != FALSE)
/*!*************************************************************************** *
 * DfrDiagAssignFrameIdRange
 * \brief   Assign LIN Frame ID range
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] *pDiag: Pointer to LIN DFR message
 * \return  -
 * *************************************************************************** *
 * \details Assign frame ID range is used to set or disable PIDs up to four
 *          frames.
 *          The start index specifies which is the first frame to assign a PID.
 *          The first frame in the list has index 0 (zero).
 *          The PIDs are an array of four PID values that will be used in
 *          the configuration request. Valid PID values here are the PID
 *          values for signal carrying frames, the unassigns value 0 (zero)
 *          and the do not care value 0xFF. The unassigns value is used to
 *          invalidate this frame for transportation on the bus. The do not
 *          care is used to keep the previous assigned value of this frame.
 *          In case the slave cannot fulfil all of the assignments set PID
 *          or unassigns or do not care, the slave shall reject the request.
 *          The do not care can always be fulfilled.
 *          The slave node will not validate the given PIDs (i.e. validating
 *          the parity flags), the slave node relies on that the master sets
 *          the correct PIDs.
 *
 *          A response shall only be sent if the NAD match.
 * NOTE: As this function is called by main loop, it is assumed that no Non Volatile Memory
 *       Write is busy.
 * *************************************************************************** *
 * - Call Hierarchy: HandleDfrDiag()
 * - Cyclomatic Complexity: 6+1
 * - Nesting: 3
 * - Function calling: 3 (SetupDiagResponse(), UpdateFrameID(), ml_Connect())
 * *************************************************************************** */
static void DfrDiagAssignFrameIdRange(DFR_DIAG *pDiag)
{
    /*  +-----+-----+-----+----------+----------+----------+----------+----------+
     *  | NAD | PCI | SID |    D1    |    D2    |    D3    |    D4    |    D5    |
     *  +-----+-----+-----+----------+----------+----------+----------+----------+
     *  | NAD | 0x06| 0xB7|  start   |    PID   |    PID   |    PID   |    PID   |
     *  |     |     |     |  index   |  (index) | (index+1)| (index+2)| (index+3)|
     *  +-----+-----+-----+----------+----------+----------+----------+----------+
     */
#if (_SUPPORT_DIAG_B7_NV_STORE != FALSE)
    STD_LIN_PARAMS_t *pStdLin = (STD_LIN_PARAMS_t *)ADDR_NV_STD_LIN_1;
#endif /* (_SUPPORT_DIAG_B7_NV_STORE != FALSE) */

#if (_SUPPORT_DIAG_B7_NV_STORE != FALSE)
    SetupDiagResponse(g_u8NAD, pDiag->u.SF.bySID,
                      (uint8_t)C_ERRCODE_PENDING);                              /* Status = Pending */
#endif /* (_SUPPORT_DIAG_B7_NV_STORE != FALSE) */
    {
        uint16_t u16NvramStoreResult = FALSE;
        if (pDiag->u.SF.byD1 == 0U)
        {
            /* Starting with index 0 (MSG_CONTROL) */
            if (pDiag->u.SF.byD2 != 0xFFU)
            {
                /* First Frame-ID is Control-message Frame-ID */
                g_u8CtrlPID = pDiag->u.SF.byD2;
                (void)ml_Disconnect();
                if (g_u8CtrlPID != 0x00U)
                {
                    (void)ml_AssignFrameToMessageID(MSG_CONTROL, g_u8CtrlPID);
                }
                else
                {
                    (void)ml_DisableMessage(MSG_CONTROL);
                }
                u16NvramStoreResult = TRUE;
            }
            if (pDiag->u.SF.byD3 != 0xFFU)
            {
                /* Second Frame-ID is Status-message Frame-ID */
                g_u8StsPID = pDiag->u.SF.byD3;
                (void)ml_Disconnect();
                if (g_u8StsPID != 0x00U)
                {
                    (void)ml_AssignFrameToMessageID(MSG_STATUS, g_u8StsPID);
                }
                else
                {
                    (void)ml_DisableMessage(MSG_STATUS);
                }
                u16NvramStoreResult = TRUE;
            }
#if (LINPROT == LIN2X_AIRVENT12)
            if (pDiag->u.SF.byD4 != 0xFFU)
            {
                /* Third Frame-ID is Status2-message Frame-ID */
                g_u8Sts2PID = pDiag->u.SF.byD4;
                (void)ml_Disconnect();
                if (g_u8Sts2PID != 0x00U)
                {
                    (void)ml_AssignFrameToMessageID(MSG_STATUS2, g_u8Sts2PID);
                }
                else
                {
                    (void)ml_DisableMessage(MSG_STATUS2);
                }
                u16NvramStoreResult = TRUE;
            }
#endif /* (LINPROT == LIN2X_AIRVENT12) */
        }
        else if (pDiag->u.SF.byD1 == 1)
        {
            /* Starting with index 1 (MSG_STATUS) */
            if (pDiag->u.SF.byD2 != 0xFF)
            {
                /* First Frame-ID is Status-message Frame-ID */
                g_u8StsPID = pDiag->u.SF.byD2;
                (void)ml_Disconnect();
                if (g_u8StsPID != 0x00U)
                {
                    (void)ml_AssignFrameToMessageID(MSG_STATUS, g_u8StsPID);
                }
                else
                {
                    (void)ml_DisableMessage(MSG_STATUS);
                }
                u16NvramStoreResult = TRUE;
            }
#if (LINPROT == LIN2X_AIRVENT12)
            if (pDiag->u.SF.byD3 != 0xFFU)
            {
                /* Second Frame-ID is Status2-message Frame-ID */
                g_u8Sts2PID = pDiag->u.SF.byD3;
                (void)ml_Disconnect();
                if (g_u8Sts2PID != 0x00U)
                {
                    (void)ml_AssignFrameToMessageID(MSG_STATUS2, g_u8Sts2PID);
                }
                else
                {
                    (void)ml_DisableMessage(MSG_STATUS2);
                }
                u16NvramStoreResult = TRUE;
            }
#endif /* (LINPROT == LIN2X_AIRVENT12) */
        }
#if (LINPROT == LIN2X_AIRVENT12)
        else if (pDiag->u.SF.byD1 == 2)
        {
            /* Starting with index 2 (MSG_STATUS2) */
            if (pDiag->u.SF.byD2 != 0xFFU)
            {
                /* First Frame-ID is Status2-message Frame-ID */
                g_u8Sts2PID = pDiag->u.SF.byD2;
                (void)ml_Disconnect();
                if (g_u8Sts2PID != 0x00U)
                {
                    (void)ml_AssignFrameToMessageID(MSG_STATUS2, g_u8Sts2PID);
                }
                else
                {
                    (void)ml_DisableMessage(MSG_STATUS2);
                }
                u16NvramStoreResult = TRUE;
            }
        }
#endif /* (LINPROT == LIN2X_AIRVENT12) */
        else
        {
            /* Nothing */
        }
#if (_SUPPORT_DIAG_B7_NV_STORE != FALSE)
        if (u16NvramStoreResult != FALSE)
        {
            STD_LIN_PARAMS_t StdLinRam;

#if (_SUPPORT_LIN_BUS_ACTIVITY_CHECK != FALSE)
#if (MSG_CONTROL <= 7) || (MSG_STATUS <= 7)
            /* Dynamic Message ID's: 0...7 */
            g_u16MLX4_RAM_Dynamic_CRC1 = p_CalcCRC_U16((const uint16_t *)C_RAM_MLX4_DYNAMIC_BGN_ADR1,
                                                       (C_RAM_MLX4_DYNAMIC_END_ADR1 - C_RAM_MLX4_DYNAMIC_BGN_ADR1)/sizeof(uint16_t));
#endif /* (MSG_CONTROL <= 7) || (MSG_STATUS <= 7) */
#if (MSG_CONTROL >= 8) || (MSG_STATUS >= 8)
            /* Dynamic Message ID's: 8...15 */
            g_u16MLX4_RAM_Dynamic_CRC2 = p_CalcCRC_U16((const uint16_t *)C_RAM_MLX4_DYNAMIC_BGN_ADR2,
                                                       (C_RAM_MLX4_DYNAMIC_END_ADR2 - C_RAM_MLX4_DYNAMIC_BGN_ADR2)/sizeof(uint16_t));
#endif /* (MSG_CONTROL >= 8) || (MSG_STATUS >= 8) */
#endif /* (_SUPPORT_LIN_BUS_ACTIVITY_CHECK != FALSE) */
            (void)ml_Connect();
            p_CopyU16( (sizeof(STD_LIN_PARAMS_t) / sizeof(uint16_t)),
                       (uint16_t *)&StdLinRam,
                       (const uint16_t *)ADDR_NV_STD_LIN_1);
            StdLinRam.u8ControlFrameID = g_u8CtrlPID;
            StdLinRam.u8StatusFrameID = g_u8StsPID;
            u16NvramStoreResult = NV_WriteLIN_STD(&StdLinRam, C_NV_WRT_LIN_ID_ALL);
        }

#if (LINPROT != LIN2X_AIRVENT12)
        if ( (u16NvramStoreResult == C_ERR_NONE) &&
             (((pDiag->u.SF.byD1 == 0U) &&                                      /* Start-index = MSG_CONTROL */
               ((pDiag->u.SF.byD2 == 0xFFU) || (pStdLin->u8ControlFrameID == pDiag->u.SF.byD2)) && /* Max. two Frame-ID's */
               ((pDiag->u.SF.byD3 == 0xFFU) || (pStdLin->u8StatusFrameID == pDiag->u.SF.byD3))) ||
              ((pDiag->u.SF.byD1 == 1U) &&                                      /* Start-index = MSG_STATUS */
               ((pDiag->u.SF.byD2 == 0xFFU) || (pStdLin->u8StatusFrameID == pDiag->u.SF.byD2)))) ) /* Only one Frame-ID */
#else  /* (LINPROT != LIN2X_AIRVENT12) */
        if ( (u16NvramStoreResult == C_ERR_NONE) &&
             (((pDiag->u.SF.byD1 == 0U) &&                                      /* Start-index = MSG_CONTROL */
               ((pDiag->u.SF.byD2 == 0xFFU) || (pStdLin->u8ControlFrameID == pDiag->u.SF.byD2)) && /* Max. three Frame-ID's */
               ((pDiag->u.SF.byD3 == 0xFFU) || (pStdLin->u8StatusFrameID == pDiag->u.SF.byD3)) &&
               ((pDiag->u.SF.byD4 == 0xFFU) || (pStdLin->u8Status2FrameID == pDiag->u.SF.byD4))) ||
              ((pDiag->u.SF.byD1 == 1U) &&                                      /* Start-index = MSG_STATUS */
               ((pDiag->u.SF.byD2 == 0xFFU) || (pStdLin->u8StatusFrameID == pDiag->u.SF.byD2)) && /* Max. two Frame-ID's */
               ((pDiag->u.SF.byD3 == 0xFFU) || (pStdLin->u8Status2FrameID == pDiag->u.SF.byD3))) ||
              ((pDiag->u.SF.byD2 == 2U) &&                                      /* Start-index = MSG_STATUS */
               ((pDiag->u.SF.byD2 == 0xFFU) || (pStdLin->u8StatusFrameID == pDiag->u.SF.byD2)))) ) /* Only one Frame-ID */
#endif /* (LINPROT != LIN2X_AIRVENT12) */
        {
            /* Positive feedback.
             *  +-----+-----+------+----------+----------+----------+----------+----------+
             *  | NAD | PCI | RSID |    D1    |    D2    |    D3    |    D4    |    D5    |
             *  +-----+-----+------+----------+----------+----------+----------+----------+
             *  | NAD | 0x01| 0xF7 | Reserved | Reserved | Reserved | Reserved | Reserved |
             *  |     |     |      |  (0xFF)  |  (0xFF)  |  (0xFF)  |  (0xFF)  |  (0xFF)  |
             *  +-----+-----+------+----------+----------+----------+----------+----------+
             */
            SetupDiagResponse(g_u8NAD, pDiag->u.SF.bySID,
                              (uint8_t)C_ERRCODE_POSITIVE_RESPONSE);            /* Status = Positive feedback */
        }
        else
        {
            /* Negative feedback.
             *  +-----+-----+------+----------+----------+----------+----------+----------+
             *  | NAD | PCI | RSID |    D1    |    D2    |    D3    |    D4    |    D5    |
             *  +-----+-----+------+----------+----------+----------+----------+----------+
             *  | NAD | 0x03| 0x7F |   SID    | ErrorCode| Reserved | Reserved | Reserved |
             *  |     |     |      |  (0xB7)  |  (0x12)  |  (0xFF)  |  (0xFF)  |  (0xFF)  |
             *  +-----+-----+------+----------+----------+----------+----------+----------+
             */
            SetupDiagResponse(g_u8NAD, pDiag->u.SF.bySID,
                              (uint8_t)C_ERRCODE_SFUNC_NOSUP);                  /* Status = Negative feedback */
        }
#else  /* (_SUPPORT_DIAG_B7_NV_STORE != FALSE) */
        if (u16NvramStoreResult != FALSE)
        {
#if (MSG_CONTROL <= 7) || (MSG_STATUS <= 7)
            /* Dynamic Message ID's: 0...7 */
            g_u16MLX4_RAM_Dynamic_CRC1 = p_CalcCRC_U16((const uint16_t *)C_RAM_MLX4_DYNAMIC_BGN_ADR1,
                                                       (C_RAM_MLX4_DYNAMIC_END_ADR1 - C_RAM_MLX4_DYNAMIC_BGN_ADR1)/sizeof(uint16_t));
#endif /* (MSG_CONTROL <= 7) || (MSG_STATUS <= 7) */
#if (MSG_CONTROL >= 8) || (MSG_STATUS >= 8)
            /* Dynamic Message ID's: 8...15 */
            g_u16MLX4_RAM_Dynamic_CRC2 = p_CalcCRC_U16((const uint16_t *)C_RAM_MLX4_DYNAMIC_BGN_ADR2,
                                                       (C_RAM_MLX4_DYNAMIC_END_ADR2 - C_RAM_MLX4_DYNAMIC_BGN_ADR2)/sizeof(uint16_t));
#endif /* (MSG_CONTROL >= 8) || (MSG_STATUS >= 8) */
            (void)ml_Connect();
        }
        SetupDiagResponse(g_u8NAD, pDiag->u.SF.bySID,
                          (uint8_t)C_ERRCODE_POSITIVE_RESPONSE);                /* Status = Positive feedback */
#endif /* (_SUPPORT_DIAG_B7_NV_STORE != FALSE) */
    }
} /* End of DfrDiagAssignFrameIdRange() */
#endif /* (_SUPPORT_DIAG_B7 != FALSE) */

/*!*************************************************************************** *
 * HandleStandardLIN
 * \brief   Handle Standard LIN Diagnostic Demand Frame
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] *pDiag: Pointer to LIN DFR message
 * \param   [in] u16PCI_SID: PCI & SID
 * \return  -
 * *************************************************************************** *
 * \details Handle Diagnostics Demand Frame and (depended on request),
 *          prepare response frame
 * *************************************************************************** *
 * - Call Hierarchy: HandleDfrDiag()
 * - Cyclomatic Complexity: 7+1
 * - Nesting: 1
 * - Function calling: 4 (DfrDiagReassignNAD(), DfrDiagReadByIdentifier(),
 *                        DfrDiagSaveConfig(), DfrDiagAssignFrameIdRange())
 *           OPTIONAL: 3 (DfrDiagAssignMessageToFrameID(),
 *                        DfrDiagConditionalChangeNAD(), DfrDiagAssignNAD())
 * *************************************************************************** */
static void HandleStandardLIN(DFR_DIAG *pDiag, uint16_t u16PCI_SID)
{
#if (_SUPPORT_LIN_AA != FALSE) && (LIN_AA_TEST_DUT != FALSE)
    if ( (pDiag->byNAD == C_BROADCAST_NAD) &&
         ((u16PCI_SID != C_PCI_SID_ASSIGN_NAD) && (u16PCI_SID != C_PCI_SID_ASSIGN_NAD_ISO)))
    {
        /* LIN-AA test DUT should not allow Broadcast NAD, except for LIN-AA */
        return;
    }
#endif /* (_SUPPORT_LIN_AA != FALSE) && (LIN_AA_TEST_DUT != FALSE) */

    switch (u16PCI_SID)
    {
#if (_SUPPORT_DIAG_B0 != FALSE)
        case C_PCI_SID_REASSIGN_NAD:
        {
            /* Re-assign NAD (Optional) */
            DfrDiagReassignNAD(pDiag);
            break;
        }
#endif /* (_SUPPORT_DIAG_B0 != FALSE) */
#if (_SUPPORT_DIAG_B1 != FALSE)
        case C_PCI_SID_ASSIGN_FRAME_ID:
        {
            /* Assign (16-bit) Message-ID to (8-bit) Frame-ID */
            DfrDiagAssignMessageToFrameID(pDiag);
            break;
        }
#endif /* (_SUPPORT_DIAG_B1 != FALSE) */
        case C_PCI_SID_READ_BY_ID:
        {
            /* Read-by-Identifier (Mandatory) */
            if (pDiag->byNAD == g_u8NAD)                                        /* MMP210112-1: Only when NAD matches (can be BROADCAST) */
            {
                DfrDiagReadByIdentifier(pDiag);
            }
            break;
        }
#if (_SUPPORT_DIAG_B3 != FALSE)
        case C_PCI_SID_CC_NAD:
        {
            /* (Optional) Conditional change NAD */
            DfrDiagConditionalChangeNAD(pDiag);
            break;
        }
#endif /* (_SUPPORT_DIAG_B3 != FASLE) */
#if (_SUPPORT_DIAG_B4 != FALSE)
        case C_PCI_SID_DATA_DUMP:
        {
            /* Data Dump */
#if (LINPROT == LIN2X_HVAC49) || (LINPROT == LIN2X_HVAC52) || (LINPROT == LIN2X_AIRVENT12)
            DfrDiagAssignVariantID(pDiag);
#else  /* (LINPROT == LIN2X_HVAC49) || (LINPROT == LIN2X_HVAC52) || (LINPROT == LIN2X_AIRVENT12) */
            DfrDiagDumpData(pDiag);
#endif /* (LINPROT == LIN2X_HVAC49) || (LINPROT == LIN2X_HVAC52) || (LINPROT == LIN2X_AIRVENT12) */
            break;
        }
#endif /* (_SUPPORT_DIAG_B4 != FASLE) */
#if (_SUPPORT_LIN_AA != FALSE)
        case C_PCI_SID_ASSIGN_NAD:
        {
            /* Assign NAD (Slave Node Position Detection, SNPD) */
            DfrDiagAssignNAD(pDiag);
            break;
        }
#endif /* (_SUPPORT_LIN_AA != FALSE) */
#if (_SUPPORT_ISO17987) || ((LINPROT & LINXX) == LIN2J)
        case C_PCI_SID_TARGETED_RESET:
        {
            if (pDiag->byNAD == g_u8NAD)                                        /* Only when NAD matches (can be BROADCAST) */
            {
                DfrDiagTargetedReset(pDiag);
            }
            break;
        }
#endif /* (_SUPPORT_ISO17987) || () */
#if (_SUPPORT_DIAG_B6 != FALSE)
        case C_PCI_SID_SAVE_CONFIG:
        {
            /* Save Configuration (Optional) */
#if (LINPROT == LIN2X_HVAC52)
            DfrDiagAssignGroupAddress(pDiag);
#else  /* (LINPROT == LIN2X_HVAC52) */
            DfrDiagSaveConfig(pDiag);
#endif /* (LINPROT == LIN2X_HVAC52) */
            break;
        }
#endif /* (_SUPPORT_DIAG_B6 != FALSE) */
#if (_SUPPORT_DIAG_B7 != FALSE)
        case C_PCI_SID_ASSIGN_FRAME_ID_RNG:
        {
            /* Assign frame ID range */
            DfrDiagAssignFrameIdRange(pDiag);
            break;
        }
#endif /* (_SUPPORT_DIAG_B7 != FALSE) */
#if (_SUPPORT_ISO17987) && (_SUPPORT_LIN_AA != FALSE)
        case C_PCI_SID_ASSIGN_NAD_ISO:
        {
            /* Assign NAD (Slave Node Position Detection, SNPD) (ISO17987) */
            DfrDiagAssignNAD(pDiag);
            break;
        }
#endif /* (_SUPPORT_ISO17987) && (_SUPPORT_LIN_AA != FALSE) */
        default:
        {
            break;
        }
    }
} /* End of HandleStandardLIN() */

/*!*************************************************************************** *
 * HandleDfrDiag
 * \brief   Handle Diagnostic Demand Frame (and optionally prepare response)
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details Handle Diagnostics Demand Frame and (depended on request),
 *          prepare response frame
 *          1st: Check NAD
 *          2nd: Check Kind of frame (SF, FF or CF)
 *          3rd: Check SID (UDS: 0x00-0x3F, Standard-LIN: 0xB0-0xB7)
 * *************************************************************************** *
 * - Call Hierarchy: HandleLinInMsg()
 * - Cyclomatic Complexity: 11+1
 * - Nesting: 5
 * - Function calling: 4 (SetupDiagResponse(), HandleUDS(), HandleStandardLIN(),
 *                        DfrDiagMlxDebug())
 * *************************************************************************** */
void HandleDfrDiag(void)
{
    DFR_DIAG *pDiag = &g_LinCmdFrameBuffer.Diag;

    l_u16DiagResponseTimeoutCount = PI_TICKS_PER_SECOND;                        /* Set LIN Diagnostics Response time-out to 1 sec */

#if (_SUPPORT_LIN_SLEEP != FALSE)
    if (pDiag->byNAD == 0x00U)   /* Other bytes should be 0xFF, and are ignored */
    {
        /* ACT_DFR_DIAG_SLEEP: Sleep request (Optional) */
        g_e8MotorRequest = (uint8_t)C_MOTOR_REQUEST_SLEEP;
        return;
    }
#endif /* (_SUPPORT_LIN_SLEEP != FALSE) */
#if ((((LINPROT & LINXX) >= LIN21) && ((LINPROT & LINXX) <= LIN22)) || ((LINPROT & LINXX) == LIN2X))  /* LIN 2.1, LIN 2.2 */
    if (pDiag->byNAD != 0x7EU)
    {
        g_u8BufferOutID = (uint8_t)QR_INVALID;
    }
#endif /* ((((LINPROT & LINXX) >= LIN21) && ((LINPROT & LINXX) <= LIN22)) || ((LINPROT & LINXX) == LIN2X)) */  /* LIN 2.1, LIN 2.2 */

    /* 1st Check NAD */
    if ( (pDiag->byNAD == g_u8NAD) || (pDiag->byNAD == C_BROADCAST_NAD) )
    {
#if (_SUPPORT_UDS != FALSE)
        uint16_t u16PCI_SID = 0x0000U;
        /* 2nd Check Kind of Frame */
        if ( (pDiag->byPCI & M_PCI_TYPE) == C_PCI_SF_TYPE)
        {
            /* Single Frame: NAD PCI SID D0 D1 D2 D3 D4, with PCI = 0x0L and L = Length */
            u16PCI_SID = (((uint16_t)pDiag->byPCI) << 8) | ((uint16_t)pDiag->u.SF.bySID);
            l_u16FF_LEN_SID = u16PCI_SID;
            l_u8FrameCount = 0x00U;
        }
        else if ( (pDiag->byPCI & M_PCI_TYPE) == C_PCI_FF_TYPE)
        {
            /* First Frame: NAD PCI LEN SID D0 D1 D2 D3, with PCI = 0x1L and Length = L*256 + LEN (including SID) */
            if ( ((pDiag->byPCI & M_PCI_FF_LEN256) != 0U) || (pDiag->u.FF.byLEN > (1 + sizeof(au8UDS_DataBuffer))) )
            {
                /* Frame too long */
                SetupDiagResponse(g_u8NAD, pDiag->u.SF.bySID,
                                  (uint8_t)C_ERRCODE_INV_MSG_INV_SZ);
            }
            else
            {
                u16PCI_SID = (((uint16_t)pDiag->u.FF.byLEN) << 8) | ((uint16_t)pDiag->u.FF.bySID);
                l_u16FF_LEN_SID = u16PCI_SID;
                l_u8FrameCount = 0x01U;
            }
        }
        else if ( (pDiag->byPCI & M_PCI_TYPE) == C_PCI_CF_TYPE)
        {
            if ( (l_u8FrameCount != 0x00U) &&
                 ((pDiag->byPCI & M_PCI_CF_FRAMECOUNTER) == (l_u8FrameCount & M_PCI_CF_FRAMECOUNTER)) )
            {
                l_u8FrameCount++;
                u16PCI_SID = l_u16FF_LEN_SID;
            }
        }
        else
        {
            /* Nothing */
        }
#else  /* (_SUPPORT_UDS != FALSE) */
        uint16_t u16PCI_SID = (((uint16_t)pDiag->byPCI) << 8) | ((uint16_t)pDiag->u.SF.bySID);
#endif /* (_SUPPORT_UDS != FALSE) */

        g_DiagResponse.u.SF.byD1 = (uint8_t)C_DIAG_RES;
        g_DiagResponse.u.SF.byD2 = (uint8_t)C_DIAG_RES;
        g_DiagResponse.u.SF.byD3 = (uint8_t)C_DIAG_RES;
        g_DiagResponse.u.SF.byD4 = (uint8_t)C_DIAG_RES;
        g_DiagResponse.u.SF.byD5 = (uint8_t)C_DIAG_RES;

        /* 3rd Check SID */
#if (_SUPPORT_UDS != FALSE)
        if ( ((u16PCI_SID & 0x00FF) >= C_SID_DIAG_SESSION_CTRL) &&
             ((u16PCI_SID & 0x00FF) <= C_SID_TESTER_PRESENT) )
        {
            /* Handle UDS */
            HandleUDS(pDiag, u16PCI_SID);
        }
#endif /* (_SUPPORT_UDS != FALSE) */
        if ( ((u16PCI_SID & 0x00FF) >= C_SID_REASSIGN_NAD) &&
#if (_SUPPORT_ISO17987) && (_SUPPORT_LIN_AA != FALSE)
             ((u16PCI_SID & 0x00FF) <= C_SID_ASSIGN_NAD_ISO) )
#else  /* (_SUPPORT_ISO17987) && (_SUPPORT_LIN_AA != FALSE) */
             ((u16PCI_SID & 0x00FF) <= C_SID_ASSIGN_FRAME_ID_RNG) )
#endif /* (_SUPPORT_ISO17987) && (_SUPPORT_LIN_AA != FALSE) */
        {
            /* Standard LIN Diagnostics */
            HandleStandardLIN(pDiag, u16PCI_SID);
        }
#if (_SUPPORT_MLX_DEBUG_MODE != FALSE)
#if (_SUPPORT_LIN_AA != FALSE) && (LIN_AA_TEST_DUT != FALSE)
        if ( ((u16PCI_SID & 0x00FF) == (uint8_t)C_SID_MLX_DEBUG) && (pDiag->byNAD != C_BROADCAST_NAD) )
#else  /* (_SUPPORT_LIN_AA != FALSE) && (LIN_AA_TEST_DUT != FALSE) */
        if ( (u16PCI_SID & 0x00FF) == (uint8_t)C_SID_MLX_DEBUG)
#endif /* (_SUPPORT_LIN_AA != FALSE) && (LIN_AA_TEST_DUT != FALSE) */
        {
            DfrDiagMlxDebug(pDiag, u16PCI_SID);
        }
#endif /* (_SUPPORT_MLX_DEBUG_MODE != FALSE) */
    }
} /* End of HandleDfrDiag() */

#if ((LINPROT & LINX) == LIN2) || (LINPROT == LIN13_HVACTB)
/*!*************************************************************************** *
 * LinDiagResponseTimeoutCount
 * \brief   LIN diagnostics Response Timeout handling
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16Period: Period in Core-timer ticks (500us)
 * \return  -
 * *************************************************************************** *
 * \details
 * *************************************************************************** *
 * - Call Hierarchy: TIMER_IT(), Timer_SleepCompensation()
 * - Cyclomatic Complexity: 3+1
 * - Nesting: 3
 * - Function calling: 0
 * *************************************************************************** */
void LinDiagResponseTimeoutCount(uint16_t u16Period)
{
    if (l_u16DiagResponseTimeoutCount != 0U)
    {
        if (l_u16DiagResponseTimeoutCount > u16Period)
        {
            l_u16DiagResponseTimeoutCount -= u16Period;
        }
        else
        {
            l_u16DiagResponseTimeoutCount = 0U;
            if (g_u8BufferOutID == QR_RFR_DIAG)                                 /* Pending response type: Diagnostic */
            {
                g_u8BufferOutID = (uint8_t)QR_INVALID;                          /* Invalidate Diagnostics response */
            }
        }
    }

#if (_SUPPORT_MLX_CHIP_STATUS != FALSE) && (_SUPPORT_APP_TYPE == C_APP_RELAY)
    if (l_u16RelayOnCount != 0U)
    {
        if (l_u16RelayOnCount > u16Period)
        {
            l_u16RelayOnCount -= u16Period;
        }
        else
        {
            l_u16RelayOnCount = 0U;
            g_e8MotorRequest = (uint8_t)C_RELAYS_REQUEST_OFF;
            l_u16RelayOffCount = l_u16RelayOffTime;
        }
    }
    else if (l_u16RelayOffCount != 0U)
    {
        if (l_u16RelayOffCount > u16Period)
        {
            l_u16RelayOffCount -= u16Period;
        }
        else
        {
            l_u16RelayOffCount = 0U;
            g_e8MotorRequest = (uint8_t)C_RELAY1_REQUEST_ON;
            l_u16RelayOnCount = l_u16RelayOnTime;
        }
    }
#endif /* (_SUPPORT_MLX_CHIP_STATUS != FALSE) && (_SUPPORT_APP_TYPE == C_APP_RELAY) */
} /* End of LinDiagResponseTimeoutCount() */
#endif /* ((LINPROT & LINX) == LIN2) || (LINPROT == LIN13_HVACTB) */

#if ((LINPROT & LINXX) == LIN2J) || (_SUPPORT_MLX_DEBUG_MODE != FALSE)
/*!*************************************************************************** *
 * RfrDiagReset
 * \brief   Prepare Diagnostics Response frame for Diagnostic reset (MLX)
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details Prepare response frame for Melexis diagnostic reset frame
 * *************************************************************************** *
 * - Call Hierarchy: main_Init()
 * - Cyclomatic Complexity: 1+1
 * - Nesting: 1
 * - Function calling: 1 (StoreD1to4())
 * *************************************************************************** */
void RfrDiagReset(void)
{
#if ((LINPROT & LINXX) == LIN2J)
    if (g_u8NAD != (uint8_t)C_BROADCAST_J2602_NAD)
#elif _SUPPORT_MLX_DEBUG_MODE
    if (g_u8NAD != (uint8_t)C_BROADCAST_NAD)
#endif
    {
        /* Positive Response */
        g_DiagResponse.byNAD = g_u8NAD;
        g_DiagResponse.byPCI = 0x06U;
        g_DiagResponse.u.SF.byRSID = (uint8_t)C_RSID_MLX_DEBUG;
        g_DiagResponse.u.SF.byD5 = (uint8_t)/* g_NvramUser.Variant */ C_VARIANT_ID;
        StoreD1to4(C_SUPPLIER_ID, /* g_NvramUser.FunctionID */ C_FUNCTION_ID);
    }
} /* End of RfrDiagReset() */
#endif /* ((LINPROT & LINXX) == LIN2J) || (_SUPPORT_MLX_DEBUG_MODE != FALSE) */

#endif /* (LIN_COMM != FALSE) */

/* EOF */
