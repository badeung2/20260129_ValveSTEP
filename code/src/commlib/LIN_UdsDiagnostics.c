/*!************************************************************************** *
 * \file        LIN_UdsDiagnostics.c
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
 *           -# HandleUDS                       (0x00-0x3F)
 *           -# HandleRfrDiagMF()
 *  - Internal Functions:
 *           -# CollectUDSData()
 *           -# DiagGetFazitString()            (0x22 - 0x6E00)
 *           -# UDS_ReadByIdentifier()          (0x22)
 *           -# DiagStoreSparePartNumber()      (0x2E - 0x6200)
 *           -# DiagStoreHardwareNumber()       (0x2E - 0x6600)
 *           -# DiagStoreSystemName()           (0x2E - 0x6C00)
 *           -# DiagStoreSerialNumber()         (0x2E - 0x6A00)
 *           -# DiagStoreFazitString()          (0x2E - 0x6E00)
 *           -# UDS_WriteByIdentifier()         (0x2E)
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
 * _SUPPORT_UDS (Class 0):  194 B   2616 B   64 B
 * *************************************************************************** */

/*!*************************************************************************** */
/*                           INCLUDES                                          */
/* *************************************************************************** */
#include "AppBuild.h"                                                           /* Application Build */

#if (LIN_COMM != FALSE) && (_SUPPORT_UDS != FALSE)

/* Application includes */
#include "../AppVersion.h"                                                      /* Application Version support */

/* Actuator Platform includes */
#include "drivelib/AppFunctions.h"                                              /* Application Functions support */
#include "drivelib/NV_Functions.h"                                              /* Non Volatile Memory Functions & Layout */
#include "drivelib/ErrorCodes.h"                                                /* Error-logging support */
#include "drivelib/GlobalVars.h"                                                /* Global Variables support */
#include "drivelib/Timer.h"                                                     /* Simple Timer support */
#include "camculib/private_mathlib.h"                                           /* Private Math Library support */

/* Communication Platform includes */
#include "commlib/LIN_Communication.h"                                          /* LIN Communication support */

/* CAMCU Platform includes */
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
#define CT_UDS_RATE                 1000                                        /*!< Periodic interrupt at 1000us rate */
#define C_UDS_LOADER_CMD_TIMEOUT    10000                                       /*!< UDS Loader command timeout: 10 seconds (max 65 seconds); 0xFFFF = Infinity */
uint16_t l_u16UDS_Timeout = 0;                                                  /*!< UDS Timeout */
uint8_t g_u8ActiveSessionID = C_SESSION_DEFAULT_DIAGNOSTIC;                     /*!< UDS Active session ID */
uint8_t l_u8FrameCount = (uint8_t)0U;                                           /*!< Multi-frame Frame-count */
uint16_t l_u16FF_LEN_SID = 0U;                                                  /*!< Multi-frame data-length */
#if (_SUPPORT_UDS_WRITE_BY_ID != FALSE)
UDS_LIN_PARAMS_t RAM_UDS;                                                       /*!< RAM-Buffer UDS data */
#endif /* (_SUPPORT_UDS_WRITE_BY_ID != FALSE) */
#pragma space none                                                              /* __NEAR_SECTION__ */

extern uint8_t au8UDS_DataBuffer[128] __attribute__((aligned(2)));              /*!< This data-buffer should be replaced by flash-loader "page_buffer" */

const uint8_t FAZIT[8] = {'M','L','X','-','0','0','1',0};                       /*!< Fixed FAZIT string part */

#if (_SUPPORT_UDS_SESSION_CTRL != FALSE)
/*!*************************************************************************** *
 * ResponseSessionCtrl
 * \brief
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u8NAD: NAD
 * \param   [in] u8NewSessionID: New Session ID
 * \return  -
 * *************************************************************************** *
 * \details
 * Positive feedback:
 *  +-----+-----+------+----------+----------+----------+----------+----------+
 *  | NAD | PCI | RSID |    D1    |    D2    |    D3    |    D4    |    D5    |
 *  +-----+-----+------+----------+----------+----------+----------+----------+
 *  | NAD | 0x06| 0x50 | SessionID| P2_SERVER| P2_SERVER| P2_SERVER| P2_SERVER|
 *  |     |     |      |          |   (MSB)  |   (LSB)  | EXT (MSB)| EXT (LSB)|
 *  +-----+-----+------+----------+----------+----------+----------+----------+
 * *************************************************************************** *
 * - Call Hierarchy: -
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 0
 * *************************************************************************** */
static void ResponseSessionCtrl(uint8_t u8NAD, uint8_t u8NewSessionID)
{
    g_DiagResponse.byNAD = u8NAD;
    g_DiagResponse.byPCI = C_RPCI_DIAG_SESSION_CTRL;
    g_DiagResponse.u.SF.byRSID = C_RSID_DIAG_SESSION_CTRL;
    g_DiagResponse.u.SF.byD1 = u8NewSessionID;
    g_DiagResponse.u.SF.byD2 = (uint8_t)(C_P2_CAN_SERVER_MAX >> 8);  /*lint !e572*/ /* MSB of C_P2_CAN_SERVER_MAX */
    g_DiagResponse.u.SF.byD3 = (uint8_t)(C_P2_CAN_SERVER_MAX & 0xFFU);          /* LSB of C_P2_CAN_SERVER_MAX */
    g_DiagResponse.u.SF.byD4 = (uint8_t)(C_P2_CAN_SERVER_MAX_EX >> 8);          /* MSB of C_P2_CAN_SERVER_MAX_EX */
    g_DiagResponse.u.SF.byD5 = (uint8_t)(C_P2_CAN_SERVER_MAX_EX & 0xFFU);       /* LSB of C_P2_CAN_SERVER_MAX_EX */

    g_u8BufferOutID = (uint8_t)QR_RFR_DIAG;                                     /* LIN Output buffer is valid (RFR_DIAG) */
} /*  End of ResponseSessionCtrl() */
#endif /* (_SUPPORT_UDS_SESSION_CTRL != FALSE) */

/*!*************************************************************************** *
 * CollectUDSData
 * \brief   Gather UDS data
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] pDiag: Pointer to Diagnostics request
 * \param   [in] u16PCI_SID: PCI and SID code
 * \return  uint16_t
 *             FALSE: Data not completed
 *              TRUE: Data complete
 * *************************************************************************** *
 * \details
 * *************************************************************************** *
 * - Call Hierarchy: UDS_ReadByIdentifier(), UDS_WriteByIdentifier()
 * - Cyclomatic Complexity: 7+1
 * - Nesting: 4
 * - Function calling: 0
 * *************************************************************************** */
static uint16_t CollectUDSData(DFR_DIAG *pDiag, uint16_t u16PCI_SID)
{
    uint16_t u16Complete = FALSE;
    if (l_u8FrameCount == 0x00U)
    {
        /* Single Frame */
        uint8_t *pDest = (uint8_t*)&au8UDS_DataBuffer[0];
        uint8_t *pSrc = (uint8_t*)&(pDiag->u.SF.byD1);
        uint8_t u8Count = (uint8_t)(((u16PCI_SID & 0x0F00U) >> 8) - 1U);
        while (u8Count != 0U)
        {
            u8Count--;
            *pDest++ = *pSrc++;
        }
        u16Complete = TRUE;
        l_u16FF_LEN_SID -= (1U << 8);                                           /* Reduce length by 1 (SID) */
    }
    else if (l_u8FrameCount == 0x01U)
    {
        /* First Frame */
        uint8_t *pDest = (uint8_t*)&au8UDS_DataBuffer[0];
        const uint8_t *pSrc = (uint8_t*)&(pDiag->u.FF.byD1);
        uint8_t u8Count = 4U;
        while (u8Count != 0U)
        {
            u8Count--;
            *pDest++ = *pSrc++;
        }
        l_u16FF_LEN_SID -= ((1U + 4U) << 8);                                    /* Reduce length by 1 (SID) + 4 (Data-bytes) */
    }
    else
    {
        /* Continuous Frame */
        uint8_t *pDest = (uint8_t*)&au8UDS_DataBuffer[(l_u8FrameCount * 6U) - 8U];
        const uint8_t *pSrc = (uint8_t*)&(pDiag->u.CF.byD1);
        uint8_t u8Count = (uint8_t)((l_u16FF_LEN_SID & 0xFF00U) >> 8);
        if (u8Count > 6U)                                                       /* Maximum number of LIN-frame data-bytes */
        {
            u8Count = 6U;                                                       /* Limit to a single frame */
            while (u8Count != 0U)
            {
                u8Count--;
                *pDest++ = *pSrc++;
            }
            l_u16FF_LEN_SID -= (6U << 8);
        }
        else
        {
            while (u8Count != 0U)
            {
                u8Count--;
                *pDest++ = *pSrc++;
            }
            /* Data collected completed */
            u16Complete = TRUE;
            l_u16FF_LEN_SID |= (((uint16_t)(pDest - &au8UDS_DataBuffer[0U])) << 8);  /* Set DFR_DIAG-length */
        }
    }
    return (u16Complete);
} /* End of CollectUDSData() */

#if (_SUPPORT_UDS_WRITE_BY_ID != FALSE)
/*!*************************************************************************** *
 * DiagGetFazitString
 * \brief   UDS Read-by-Identifier Service FAZIT-string
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details Helper-function for UDS "Read-by-Identifier"
 * *************************************************************************** *
 * - Call Hierarchy: UDS_ReadByIdentifier()
 * - Cyclomatic Complexity: 7+1
 * - Nesting: 2
 * - Function calling: 0
 * *************************************************************************** */
static void DiagGetFazitString(void)
{
    /* FAZIT string (23-Byte ASCII) (0x6E00-0x6FFF) */
    const uint8_t *pSrc = (uint8_t *)&FAZIT[0];
    uint8_t *pDest = (uint8_t *)&au8UDS_DataBuffer[2];
    UDS_LIN_PARAMS_t *pNV_UDS = (UDS_LIN_PARAMS_t *)ADDR_NV_UDS;
    do
    {
        *pDest++ = *pSrc++;
    } while (pSrc < &FAZIT[7]);

    if (pNV_UDS->UDS_FazitProductionDate != 0xFFFFU)
    {
        uint16_t u16Hex = pNV_UDS->UDS_FazitProductionDate & 0x001FU;           /* Day */
        uint16_t u16Ten = 0U;
        while (u16Hex > 9U)
        {
            u16Hex -= 10U;
            u16Ten += 1U;
        }
        au8UDS_DataBuffer[10] = (uint8_t)('0' | u16Hex);
        au8UDS_DataBuffer[9] = (uint8_t)('0' | u16Ten);

        au8UDS_DataBuffer[11] = '.'; /* separator */

        u16Hex = ((pNV_UDS->UDS_FazitProductionDate >> 5) & 0x000FU);           /* Month */
        u16Ten = 0U;
        while (u16Hex > 9U)
        {
            u16Hex -= 10U;
            u16Ten += 1U;
        }
        au8UDS_DataBuffer[13] = (uint8_t)(((uint8_t)'0') | u16Hex);
        au8UDS_DataBuffer[12] = (uint8_t)(((uint8_t)'0') | u16Ten);

        au8UDS_DataBuffer[14] = '.'; /* separator */

        u16Hex = ((pNV_UDS->UDS_FazitProductionDate >> 9) & 0x007FU);           /* Year */
        u16Ten = 0U;
        while (u16Hex > 9U)
        {
            u16Hex -= 10U;
            u16Ten += 1U;
        }
        au8UDS_DataBuffer[16] = (uint8_t)(((uint8_t)'0') | u16Hex);
        au8UDS_DataBuffer[15] = (uint8_t)(((uint8_t)'0') | u16Ten);
    }
    if (pNV_UDS->UDS_FazitEOLNumber != 0xFFFFU)
    {
        au8UDS_DataBuffer[17] = (uint8_t)(((pNV_UDS->UDS_FazitEOLNumber >> 12) & 0x000FU) | ((uint8_t)'0'));
        au8UDS_DataBuffer[18] = (uint8_t)(((pNV_UDS->UDS_FazitEOLNumber >> 8) & 0x000FU) | ((uint8_t)'0'));
        au8UDS_DataBuffer[19] = (uint8_t)(((pNV_UDS->UDS_FazitEOLNumber >> 4) & 0x000FU) | ((uint8_t)'0'));
        au8UDS_DataBuffer[20] = (uint8_t)((pNV_UDS->UDS_FazitEOLNumber & 0x000FU) | ((uint8_t)'0'));
    }
    if (pNV_UDS->UDS_FazitManufacturerNumber != 0xFFFFU)
    {
        au8UDS_DataBuffer[21] = (uint8_t)(((pNV_UDS->UDS_FazitManufacturerNumber >> 12) & 0x000FU) | ((uint8_t)'0'));
        au8UDS_DataBuffer[22] = (uint8_t)(((pNV_UDS->UDS_FazitManufacturerNumber >> 8) & 0x000FU) | ((uint8_t)'0'));
        au8UDS_DataBuffer[23] = (uint8_t)(((pNV_UDS->UDS_FazitManufacturerNumber >> 4) & 0x000FU) | ((uint8_t)'0'));
        au8UDS_DataBuffer[24] = (uint8_t)((pNV_UDS->UDS_FazitManufacturerNumber & 0x000FU) | ((uint8_t)'0'));
    }

    /* Response length (23 data bytes + 1 for RSID + 2 for Data-identifier) and RSID */
    l_u16FF_LEN_SID = ((23U + 3U) << 8) | C_RSID_READ_DATA_BY_IDENTIFY;
    l_u8FrameCount = 0x00U;
    g_u8BufferOutID = (uint8_t)QR_RFR_DIAG_MF;
} /* End of DiagGetFazitString() */

/*!*************************************************************************** *
 * UDS_ReadByIdentifier
 * \brief   UDS Read-by-Identifier Service
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] pDiag: Pointer to Diagnostic Request message
 * \param   [in] u16PCI_SID: PCI & SID code
 * \return  -
 * *************************************************************************** *
 * \details Helper-function for UDS "Read-by-Identifier"
 * *************************************************************************** *
 * - Call Hierarchy: HandleDfrDiag()
 * - Cyclomatic Complexity: 18+1
 * - Nesting: 3
 * - Function calling: 2 (CollectUDSData(), DiagGetFazitString())
 * *************************************************************************** */
static void UDS_ReadByIdentifier(DFR_DIAG *pDiag, int16_t u16PCI_SID)
{
    /* Read data by Identifier (UDS)
     * This service will read data from the system. Which data will be read,
     * will be defined by the parameter RecordDataIdentifier.
     * Single-frame:
     *  +-----+-----+------+----------+----------+----------+----------+----------+
     *  | NAD | PCI |  SID |    D1    |    D2    |    D3    |    D4    |    D5    |
     *  +-----+-----+------+----------+----------+----------+----------+----------+
     *  | NAD |>0x03| 0x22 |Record-ID1|Record-ID1|Record-ID2|Record-ID2| Reserved |
     *  |     |     |      |   (MSB)  |   (LSB)  |(optional)|(optional)|  (0xFF)  |
     *  +-----+-----+------+----------+----------+----------+----------+----------+
     *
     *  or
     *  Multiple-frames:
     *  +-----+-----+------+------+----------+----------+----------+----------+  +-----+-----+----------+----------+--------..
     *  | NAD | PCI |  LEN |  SID |    D1    |    D2    |    D3    |    D4    |  | NAD | PCI |    D1    |    D2    |    D3..
     *  +-----+-----+------+------+----------+----------+----------+----------+  +-----+-----+----------+----------+--------..
     *  | NAD | 0x10|Length| 0x22 |Record-ID1|Record-ID1|Record-ID2|Record-ID2|  | NAD | 0x2n|Record-ID3|Record-ID3|Record-ID4
     *  |     |     |      |      |   (MSB)  |   (LSB)  |   (MSB)  |   (LSB)  |  |     |     |   (MSB)  |   (LSB)  |(optional)
     *  +-----+-----+------+------+----------+----------+----------+----------+  +-----+-----+----------+----------+--------..
     *
     * Response (OK):
     *  +-----+-----+------+----------+----------+----------+----------+----------+
     *  | NAD | PCI | RSID |    D1    |    D2    |    D3    |    D4    |    D5    |
     *  +-----+-----+------+----------+----------+----------+----------+----------+
     *  | NAD | 0x06| 0x62 |          |          |          |          |          |
     *  +-----+-----+------+----------+----------+----------+----------+----------+
     */
    if (CollectUDSData(pDiag, u16PCI_SID) != FALSE)
    {
        uint16_t u16RecordDataIdentifier = (((uint16_t)au8UDS_DataBuffer[0]) << 8) | ((uint16_t)au8UDS_DataBuffer[1]);
        UDS_LIN_PARAMS_t *pNV_UDS = (UDS_LIN_PARAMS_t *)ADDR_NV_UDS;

#if (_SUPPORT_UDS_DK >= _SUPPORT_UDS_DK4)
        if (u16RecordDataIdentifier == C_ID_SUB_SYSTEM_NR)
        {
            g_DiagResponse.byNAD = g_u8NAD;
            g_DiagResponse.byPCI = (uint8_t)0x06U;
            g_DiagResponse.u.SF.byRSID = (uint8_t)pDiag->u.SF.bySID | 0x40U;
            g_DiagResponse.u.SF.byD1 = (u16RecordDataIdentifier >> 8);
            g_DiagResponse.u.SF.byD2 = (u16RecordDataIdentifier & 0xFFU);
            g_DiagResponse.u.SF.byD3 = (uint8_t)(pNV_UDS->UDS_SubSystemID >> 8);
            g_DiagResponse.u.SF.byD4 = (uint8_t)(pNV_UDS->UDS_SubSystemID & 0xFFU);
            g_u8BufferOutID = (uint8_t)QR_RFR_DIAG;
        }
        else
#endif /* (_SUPPORT_UDS_DK >= _SUPPORT_UDS_DK4) */
        {
            uint8_t *pDest = (uint8_t*)&au8UDS_DataBuffer[0];
#if (_SUPPORT_UDS_DK >= _SUPPORT_UDS_DK4)
            if ( (((u16RecordDataIdentifier & C_SLAVE_ID_MASK) >= C_SLAVE_SPAREPART_NR) &&
                  ((u16RecordDataIdentifier & C_SLAVE_ID_MASK) <= C_SLAVE_FAZIT_STRING)) )
            {
                if ( (u16RecordDataIdentifier & C_SUBSYS_MASK) != (pNV_UDS->UDS_SubSystemID & C_SUBSYS_MASK) )
                {
                    /* Invalid Sub-system */
                    u16RecordDataIdentifier = 0U;
                }
                else
                {
                    u16RecordDataIdentifier = u16RecordDataIdentifier & C_SLAVE_ID_MASK;
                }
            }
#endif /* (_SUPPORT_UDS_DK >= _SUPPORT_UDS_DK4) */
            *pDest++ = (u16RecordDataIdentifier >> 8);
            *pDest++ = (u16RecordDataIdentifier & 0xFFU);
            switch (u16RecordDataIdentifier)
            {
#if (_SUPPORT_UDS_DK >= _SUPPORT_UDS_DK4)
                case C_SLAVE_SPAREPART_NR:
#endif /* (_SUPPORT_UDS_DK >= _SUPPORT_UDS_DK4) */
                case C_VW_SPAREPART_NUMBER:
                    /* Slave Spare-part number (0x6200-0x63FF), 11-characters; Skip 12th character */
                    pDest = (uint8_t *)p_Unpack( (uint16_t)pDest, (uint16_t)&pNV_UDS->UDS_SparePartNumber[0], 11U);
                    /* Response length (11 data bytes + 1 for RSID + 2 for Data-identifier) and RSID */
                    l_u16FF_LEN_SID = ((11U + 3U) << 8) | C_RSID_READ_DATA_BY_IDENTIFY;
                    l_u8FrameCount = 0x00U;
                    g_u8BufferOutID = (uint8_t)QR_RFR_DIAG_MF;
                    break;
#if (_SUPPORT_UDS_DK >= _SUPPORT_UDS_DK4)
                case C_SLAVE_APPL_SW_VERSION_NR:
#endif /* (_SUPPORT_UDS_DK >= _SUPPORT_UDS_DK4) */
                case C_VW_SOFTWARE_VERSION:
                    /* Slave SW Version (0x6400-0x65FF) */
#if ((__APP_VERSION_REVISION__ & 0xF000) != 0xE000)
                    *pDest++ = (uint8_t)((__APP_VERSION_MAJOR__ >> 4) | '0');
#else  /* ((__APP_VERSION_REVISION__ & 0xF000) != 0xE000) */
                    *pDest++ = (uint8_t)'X';
#endif /* ((__APP_VERSION_REVISION__ & 0xF000) != 0xE000) */
                    *pDest++ = (uint8_t)((__APP_VERSION_MAJOR__ & 15) | '0');
                    *pDest++ = (uint8_t)((__APP_VERSION_MINOR__ >> 4) | '0');   /*lint !e572*/
                    *pDest++ = (uint8_t)((__APP_VERSION_MINOR__ & 15) | '0');

                    /* Response length (4 data bytes + 1 for RSID + 2 for Data-identifier) and RSID */
                    l_u16FF_LEN_SID = ((4U + 3U) << 8) | C_RSID_READ_DATA_BY_IDENTIFY;
                    l_u8FrameCount = 0x00U;
                    g_u8BufferOutID = (uint8_t)QR_RFR_DIAG_MF;
                    break;
#if (_SUPPORT_UDS_DK >= _SUPPORT_UDS_DK4)
                case C_SLAVE_HW_NR:
#endif /* (_SUPPORT_UDS_DK >= _SUPPORT_UDS_DK4) */
                case C_VW_ECU_HW_NUMBER:
                    /* Hardware Number (0x6600-0x67FF), 11-characters; Skip 12th character */
                    pDest = (uint8_t *)p_Unpack( (uint16_t)pDest, (uint16_t)&pNV_UDS->UDS_HardwareNumber[0], 11U);
                    /* Response length (11 data bytes + 1 for RSID + 2 for Data-identifier) and RSID */
                    l_u16FF_LEN_SID = ((11U + 3U) << 8) | C_RSID_READ_DATA_BY_IDENTIFY;
                    l_u8FrameCount = 0x00U;
                    g_u8BufferOutID = (uint8_t)QR_RFR_DIAG_MF;
                    break;
#if (_SUPPORT_UDS_DK >= _SUPPORT_UDS_DK4)
                case C_SLAVE_HW_VERSION_NR:
#endif /* (_SUPPORT_UDS_DK >= _SUPPORT_UDS_DK4) */
                case C_VW_ECU_HW_VERSION:
                    /* Hardware Version (0x6800-0x69FF) */
                    g_DiagResponse.byNAD = g_u8NAD;
                    g_DiagResponse.byPCI = (uint8_t)0x06U;
                    g_DiagResponse.u.SF.byRSID = (uint8_t)pDiag->u.SF.bySID | 0x40U;
                    g_DiagResponse.u.SF.byD1 = (u16RecordDataIdentifier >> 8);
                    g_DiagResponse.u.SF.byD2 = (u16RecordDataIdentifier & 0xFFU);
                    g_DiagResponse.u.SF.byD3 = (uint8_t)pNV_UDS->UDS_HwVersion_XSB;
                    g_DiagResponse.u.SF.byD4 = (uint8_t)pNV_UDS->UDS_HwVersion_MSB;
                    g_DiagResponse.u.SF.byD5 = (uint8_t)pNV_UDS->UDS_HwVersion_LSB;
                    g_u8BufferOutID = (uint8_t)QR_RFR_DIAG;
                    break;
#if (_SUPPORT_UDS_DK >= _SUPPORT_UDS_DK4)
                case C_SLAVE_SERIAL_NR:
#endif /* (_SUPPORT_UDS_DK >= _SUPPORT_UDS_DK4) */
                case C_VW_ECU_SERIAL_NUMBER:
                    /* Serial number (20-Byte ASCII) (0x6A00-0x6BFF) */
                    pDest = (uint8_t *)p_Unpack( (uint16_t)pDest, (uint16_t)&pNV_UDS->UDS_SerialNumber[0], 20U);
                    /* Response length (20 data bytes + 1 for RSID + 2 for Data-identifier) and RSID */
                    l_u16FF_LEN_SID = ((20U + 3U) << 8) | C_RSID_READ_DATA_BY_IDENTIFY;
                    l_u8FrameCount = 0x00U;
                    g_u8BufferOutID = (uint8_t)QR_RFR_DIAG_MF;
                    break;
#if (_SUPPORT_UDS_DK >= _SUPPORT_UDS_DK4)
                case C_SLAVE_SYSTEM_NAME:
#endif /* (_SUPPORT_UDS_DK >= _SUPPORT_UDS_DK4) */
                case C_VW_SYSTEM_NAME:
                {
                    /* System Name (13-Byte ASCII) (0x6C00-0x6DFF) */
                    pDest = (uint8_t*)&au8UDS_DataBuffer[2];
                    *pDest++ = pNV_UDS->UDS_SystemName_12 + ((uint8_t)' ');
                    /* System Name (13-Byte ASCII) (0x6C00-0x6DFF); Remaining 12 characters */
                    pDest = (uint8_t *)p_Unpack( (uint16_t)pDest, (uint16_t)&pNV_UDS->UDS_SystemName[0], 12U);
                    /* Response length (13 data bytes + 1 for RSID + 2 for Data-identifier) and RSID */
                    l_u16FF_LEN_SID = ((13U + 3U) << 8) | C_RSID_READ_DATA_BY_IDENTIFY;
                    l_u8FrameCount = 0x00U;
                    g_u8BufferOutID = (uint8_t)QR_RFR_DIAG_MF;
                    break;
                }
#if (_SUPPORT_UDS_DK >= _SUPPORT_UDS_DK4)
                case C_SLAVE_FAZIT_STRING:
#endif /* (_SUPPORT_UDS_DK >= _SUPPORT_UDS_DK4) */
                case C_VW_FAZIT_STRING:
                    DiagGetFazitString();
                    break;
                default:
                    /* Not supported */
                    SetupDiagResponse(g_u8NAD, pDiag->u.SF.bySID,
                                      (uint8_t)C_ERRCODE_SFUNC_NOSUP);
#if (_SUPPORT_LOG_LIN_ERRORS != FALSE)
                    SetLastError(C_ERR_LIN2X_22);
#endif /* (_SUPPORT_LOG_LIN_ERRORS != FALSE) */
                    break;
            }
        } /*lint !e438 */
    }
} /* End of UDS_ReadByIdentifier() */

/*!*************************************************************************** *
 * DiagStoreSparePartNumber
 * \brief   Store VW Sparepart-number in Non Volatile Memory
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16ID: 16-bit u16ID
 * \param   [in] u8SID: SID
 * \return  -
 * *************************************************************************** *
 * \details Helper-function store VW Sparepart-number in Non Volatile Memory
 * *************************************************************************** *
 * - Call Hierarchy: UDS_WriteByIdentifier()
 * - Cyclomatic Complexity: 1+1
 * - Nesting: 1
 * - Function calling: 1 (p_Pack())
 * *************************************************************************** */
static void DiagStoreSparePartNumber(uint16_t u16ID, uint8_t u8SID)
{
    /* Slave Spare-part number (0x6200-0x63FF), 11-characters */
#if TRUE
    p_Pack( (uint16_t)&RAM_UDS.UDS_SparePartNumber[0], (uint16_t)&au8UDS_DataBuffer[2], 11U);
#else
    PCKCHR *pDest = (PCKCHR*)&RAM_UDS.UDS_SparePartNumber[0];
    const uint8_t *pSrc = (uint8_t*)&au8UDS_DataBuffer[2];
    do
    {
        pDest->u6Char1 = (*pSrc++ - ((uint8_t)' '));
        pDest->u6Char2 = (*pSrc++ - ((uint8_t)' '));
        pDest->u6Char3 = (*pSrc++ - ((uint8_t)' '));
        pDest->u6Char4 = (*pSrc++ - ((uint8_t)' '));
        pDest++;
    } while (pDest <= (PCKCHR*)&RAM_UDS.UDS_SparePartNumber[2]);
#endif
    /* Positive response - Slave Spare Part Number saved.
     *  +-----+-----+------+----------+----------+----------+----------+----------+
     *  | NAD | PCI | RSID |    D1    |    D2    |    D3    |    D4    |    D5    |
     *  +-----+-----+------+----------+----------+----------+----------+----------+
     *  | NAD |     |      |  DataID  |  DataID  | Reserved | Reserved | Reserved |
     *  |     | 0x03| 0x6E |   (MSB)  |   (LSB)  |  (0xFF)  |  (0xFF)  |  (0xFF)  |
     *  +-----+-----+------+----------+----------+----------+----------+----------+
     */
    g_DiagResponse.byNAD = g_u8NAD;
    g_DiagResponse.byPCI = (uint8_t)C_RPCI_WRITE_DATA_BY_IDENTIFY;
    g_DiagResponse.u.SF.byRSID = (uint8_t)(u8SID | C_RSID_OK);
    g_DiagResponse.u.SF.byD1 = (uint8_t)(u16ID >> 8);
    g_DiagResponse.u.SF.byD2 = (uint8_t)(u16ID & 0xFFU);
    g_u8BufferOutID = (uint8_t)QR_RFR_DIAG;                                     /* LIN Output buffer is valid (RFR_DIAG) */
} /* End of DiagStoreSparePartNumber() */

/*!*************************************************************************** *
 * DiagStoreHardwareNumber
 * \brief   Store VW Hardware Number in Non Volatile Memory
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16ID: 16-bit u16ID
 * \param   [in] u8SID: SID
 * \return  -
 * *************************************************************************** *
 * \details Helper-function store VW Hardware Number in Non Volatile Memory
 * *************************************************************************** *
 * - Call Hierarchy: UDS_WriteByIdentifier()
 * - Cyclomatic Complexity: 1+1
 * - Nesting: 1
 * - Function calling: 1 (p_Pack())
 * *************************************************************************** */
static void DiagStoreHardwareNumber(uint16_t u16ID, uint8_t u8SID)
{
    /* Hardware Number (0x6600-0x67FF), 11-characters */
#if TRUE
    p_Pack( (uint16_t)&RAM_UDS.UDS_HardwareNumber[0], (uint16_t)&au8UDS_DataBuffer[2], 11U);
#else
    PCKCHR *pDest = (PCKCHR*)&RAM_UDS.UDS_HardwareNumber[0];
    const uint8_t *pSrc = (uint8_t*)&au8UDS_DataBuffer[2];
    do
    {
        pDest->u6Char1 = (*pSrc++ - ((uint8_t)' '));
        pDest->u6Char2 = (*pSrc++ - ((uint8_t)' '));
        pDest->u6Char3 = (*pSrc++ - ((uint8_t)' '));
        pDest->u6Char4 = (*pSrc++ - ((uint8_t)' '));
        pDest++;
    } while (pDest <= (PCKCHR*)&RAM_UDS.UDS_HardwareNumber[2]);
#endif

    /* Positive response - Slave Hardware Number saved.
     *  +-----+-----+------+----------+----------+----------+----------+----------+
     *  | NAD | PCI | RSID |    D1    |    D2    |    D3    |    D4    |    D5    |
     *  +-----+-----+------+----------+----------+----------+----------+----------+
     *  | NAD |     |      |  DataID  |  DataID  | Reserved | Reserved | Reserved |
     *  |     | 0x03| 0x6E |   (MSB)  |   (LSB)  |  (0xFF)  |  (0xFF)  |  (0xFF)  |
     *  +-----+-----+------+----------+----------+----------+----------+----------+
     */
    g_DiagResponse.byNAD = g_u8NAD;
    g_DiagResponse.byPCI = (uint8_t)C_RPCI_WRITE_DATA_BY_IDENTIFY;
    g_DiagResponse.u.SF.byRSID = (uint8_t)(u8SID | C_RSID_OK);
    g_DiagResponse.u.SF.byD1 = (uint8_t)(u16ID >> 8);
    g_DiagResponse.u.SF.byD2 = (uint8_t)(u16ID & 0xFFU);
    g_u8BufferOutID = (uint8_t)QR_RFR_DIAG;                                     /* LIN Output buffer is valid (RFR_DIAG) */
} /* End of DiagStoreHardwareNumber() */

/*!*************************************************************************** *
 * DiagStoreSystemName
 * \brief   Store VW System Name in Non Volatile Memory
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16ID: 16-bit u16ID
 * \param   [in] u8SID: SID
 * \return  -
 * *************************************************************************** *
 * \details Helper-function store VW System Name in Non Volatile Memory
 * *************************************************************************** *
 * - Call Hierarchy: UDS_WriteByIdentifier()
 * - Cyclomatic Complexity: 1+1
 * - Nesting: 1
 * - Function calling: 1 (p_Pack())
 * *************************************************************************** */
static void DiagStoreSystemName(uint16_t u16ID, uint8_t u8SID)
{
    /* System Name (13-Byte ASCII) (0x6C00-0x6DFF) */
    RAM_UDS.UDS_SystemName_12 = (uint16_t)(au8UDS_DataBuffer[2U] - ((uint8_t)' '));
#if TRUE
    p_Pack( (uint16_t)&RAM_UDS.UDS_SystemName[0U], (uint16_t)&au8UDS_DataBuffer[2U + 1U], 12U);
#else
    PCKCHR *pDest = (PCKCHR*)&RAM_UDS.UDS_SystemName[0U];
    const uint8_t *pSrc = (uint8_t*)&au8UDS_DataBuffer[2U + 1U];
    do
    {
        pDest->u6Char1 = (*pSrc++ - ((uint8_t)' '));
        pDest->u6Char2 = (*pSrc++ - ((uint8_t)' '));
        pDest->u6Char3 = (*pSrc++ - ((uint8_t)' '));
        pDest->u6Char4 = (*pSrc++ - ((uint8_t)' '));
        pDest++;
    } while (pDest <= (PCKCHR*)&RAM_UDS.UDS_SystemName[2]);
#endif

    /* Positive response - Slave System Name saved.
     *  +-----+-----+------+----------+----------+----------+----------+----------+
     *  | NAD | PCI | RSID |    D1    |    D2    |    D3    |    D4    |    D5    |
     *  +-----+-----+------+----------+----------+----------+----------+----------+
     *  | NAD |     |      |  DataID  |  DataID  | Reserved | Reserved | Reserved |
     *  |     | 0x03| 0x6E |   (MSB)  |   (LSB)  |  (0xFF)  |  (0xFF)  |  (0xFF)  |
     *  +-----+-----+------+----------+----------+----------+----------+----------+
     */
    g_DiagResponse.byNAD = g_u8NAD;
    g_DiagResponse.byPCI = (uint8_t)C_RPCI_WRITE_DATA_BY_IDENTIFY;
    g_DiagResponse.u.SF.byRSID = (uint8_t)(u8SID | C_RSID_OK);
    g_DiagResponse.u.SF.byD1 = (uint8_t)(u16ID >> 8);
    g_DiagResponse.u.SF.byD2 = (uint8_t)(u16ID & 0xFFU);
    g_u8BufferOutID = (uint8_t)QR_RFR_DIAG;                                     /* LIN Output buffer is valid (RFR_DIAG) */
} /* End of DiagStoreSystemName() */

/*!*************************************************************************** *
 * DiagStoreSerialNumber
 * \brief   Store VW Serial number in Non Volatile Memory
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16ID: 16-bit u16ID
 * \param   [in] u8SID: SID
 * \return  -
 * *************************************************************************** *
 * \details Helper-function store VW Serial number in Non Volatile Memory
 * *************************************************************************** *
 * - Call Hierarchy: UDS_WriteByIdentifier()
 * - Cyclomatic Complexity: 1+1
 * - Nesting: 1
 * - Function calling: 1 (p_Pack())
 * *************************************************************************** */
static void DiagStoreSerialNumber(uint16_t u16ID, uint8_t u8SID)
{
    /* Serial number (20-Byte ASCII) (0x6A00-0x6BFF) */
#if TRUE
    p_Pack( (uint16_t)&RAM_UDS.UDS_SerialNumber[0], (uint16_t)&au8UDS_DataBuffer[2], 20U);
#else
    PCKCHR *pDest = (PCKCHR*)&RAM_UDS.UDS_SerialNumber[0];
    const uint8_t *pSrc = (uint8_t*)&au8UDS_DataBuffer[2];
    do
    {
        pDest->u6Char1 = (*pSrc++ - ((uint8_t)' '));
        pDest->u6Char2 = (*pSrc++ - ((uint8_t)' '));
        pDest->u6Char3 = (*pSrc++ - ((uint8_t)' '));
        pDest->u6Char4 = (*pSrc++ - ((uint8_t)' '));
        pDest++;
    } while (pDest <= (PCKCHR*)&RAM_UDS.UDS_SerialNumber[4]);
#endif

    /* Positive response - Slave Serial number saved.
     *  +-----+-----+------+----------+----------+----------+----------+----------+
     *  | NAD | PCI | RSID |    D1    |    D2    |    D3    |    D4    |    D5    |
     *  +-----+-----+------+----------+----------+----------+----------+----------+
     *  | NAD |     |      |  DataID  |  DataID  | Reserved | Reserved | Reserved |
     *  |     | 0x03| 0x6E |   (MSB)  |   (LSB)  |  (0xFF)  |  (0xFF)  |  (0xFF)  |
     *  +-----+-----+------+----------+----------+----------+----------+----------+
     */
    g_DiagResponse.byNAD = g_u8NAD;
    g_DiagResponse.byPCI = (uint8_t)C_RPCI_WRITE_DATA_BY_IDENTIFY;
    g_DiagResponse.u.SF.byRSID = (uint8_t)(u8SID | C_RSID_OK);
    g_DiagResponse.u.SF.byD1 = (uint8_t)(u16ID >> 8);
    g_DiagResponse.u.SF.byD2 = (uint8_t)(u16ID & 0xFFU);
    g_u8BufferOutID = (uint8_t)QR_RFR_DIAG;                                     /* LIN Output buffer is valid (RFR_DIAG) */
} /* End of DiagStoreSerialNumber() */

/*!*************************************************************************** *
 * DiagStoreFazitString
 * \brief   Store VW Fazit String in Non Volatile Memory
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16ID: 16-bit u16ID
 * \param   [in] u8SID: SID
 * \return  -
 * *************************************************************************** *
 * \details Helper-function store VW Fazit String in Non Volatile Memory
 * *************************************************************************** *
 * - Call Hierarchy: UDS_WriteByIdentifier()
 * - Cyclomatic Complexity: 18+1
 * - Nesting: 3
 * - Function calling: 1 (SetupDiagResponse())
 * *************************************************************************** */
static void DiagStoreFazitString(uint16_t u16ID, uint8_t u8SID)
{
    uint8_t u8Char;
    do
    {
        uint16_t u16Date = 0U;
        /* Check FAZIT[6:0] */
        const uint8_t *pSrc = (uint8_t *)&au8UDS_DataBuffer[2];
        const uint8_t *pMask = (uint8_t *)&FAZIT[0];
        do
        {
            if (*pSrc++ != *pMask)
            {
                break;
            }
            pMask++;
        } while (pMask < (uint8_t *)&FAZIT[7]);
        if (pMask < (uint8_t *)&FAZIT[7])
        {
            u8Char = 1U;
            break;
        }

        u8Char = au8UDS_DataBuffer[9] - ((uint8_t)'0');                        /* Day (ten-unit) */
        if (u8Char > 3U)
        {
            break;                                                              /* Invalid date (non-digit) */
        }
        u16Date = (uint16_t)(u8Char * 10U);
        u8Char = au8UDS_DataBuffer[10] - ((uint8_t)'0');                        /* Day (unit) */
        if (u8Char > 9U)
        {
            break;                                                              /* Invalid date (non-digit) */
        }
        u16Date += (uint16_t)u8Char;

        u8Char = au8UDS_DataBuffer[12] - ((uint8_t)'0');                        /* Month (ten-unit) */
        if (u8Char > 1U)
        {
            break;                                                              /* Invalid date (non-digit) */
        }
        u16Date += ((uint16_t)(u8Char * 10U) << 5);
        u8Char = au8UDS_DataBuffer[13] - ((uint8_t)'0');                        /* Month (unit) */
        if (u8Char > 9U)
        {
            break;                                                              /* Invalid date (non-digit) */
        }
        u16Date += ((uint16_t)u8Char << 5);

        u8Char = au8UDS_DataBuffer[15] - ((uint8_t)'0');                        /* Year (ten-unit) */
        if (u8Char > 9U)
        {
            break;                                                              /* Invalid date (non-digit) */
        }
        u16Date += ((uint16_t)(u8Char * 10U) << 9);
        u8Char = au8UDS_DataBuffer[16] - ((uint8_t)'0');                        /* Year (unit) */
        if (u8Char > 9U)
        {
            break;                                                              /* Invalid date (non-digit) */
        }
        u16Date += ((uint16_t)u8Char << 9);
        RAM_UDS.UDS_FazitProductionDate = u16Date;

        u16Date = 0U;
        u8Char = au8UDS_DataBuffer[17] - ((uint8_t)'0');
        if (u8Char > 9U)
        {
            break;
        }
        u16Date = u8Char;
        u16Date <<= 4;
        u8Char = au8UDS_DataBuffer[18] - ((uint8_t)'0');
        if (u8Char > 9U)
        {
            break;
        }
        u16Date += u8Char;
        u16Date <<= 4;
        u8Char = au8UDS_DataBuffer[19] - ((uint8_t)'0');
        if (u8Char > 9U)
        {
            break;
        }
        u16Date += u8Char;
        u16Date <<= 4;
        u8Char = au8UDS_DataBuffer[20] - ((uint8_t)'0');
        if (u8Char > 9U)
        {
            break;
        }
        u16Date += u8Char;
        RAM_UDS.UDS_FazitEOLNumber = u16Date;

        u16Date = 0U;
        u8Char = au8UDS_DataBuffer[21] - ((uint8_t)'0');
        if (u8Char > 9U)
        {
            break;
        }
        u16Date = u8Char;
        u16Date <<= 4;
        u8Char = au8UDS_DataBuffer[22] - ((uint8_t)'0');
        if (u8Char > 9U)
        {
            break;
        }
        u16Date += u8Char;
        u16Date <<= 4;
        u8Char = au8UDS_DataBuffer[23] - ((uint8_t)'0');
        if (u8Char > 9U)
        {
            break;
        }
        u16Date += u8Char;
        u16Date <<= 4;
        u8Char = au8UDS_DataBuffer[24] - ((uint8_t)'0');
        if (u8Char > 9U)
        {
            break;
        }
        u16Date += u8Char;
        RAM_UDS.UDS_FazitManufacturerNumber = u16Date;
        u8Char = 0U;
    } while (FALSE);

    if (u8Char == 0U)
    {
        /* Positive response - Slave Fazit String saved.
         *  +-----+-----+------+----------+----------+----------+----------+----------+
         *  | NAD | PCI | RSID |    D1    |    D2    |    D3    |    D4    |    D5    |
         *  +-----+-----+------+----------+----------+----------+----------+----------+
         *  | NAD |     |      |  DataID  |  DataID  | Reserved | Reserved | Reserved |
         *  |     | 0x03| 0x6E |   (MSB)  |   (LSB)  |  (0xFF)  |  (0xFF)  |  (0xFF)  |
         *  +-----+-----+------+----------+----------+----------+----------+----------+
         */
        g_DiagResponse.byNAD = g_u8NAD;
        g_DiagResponse.byPCI = (uint8_t)C_RPCI_WRITE_DATA_BY_IDENTIFY;
        g_DiagResponse.u.SF.byRSID = (uint8_t)(u8SID | C_RSID_OK);
        g_DiagResponse.u.SF.byD1 = (uint8_t)(u16ID >> 8);
        g_DiagResponse.u.SF.byD2 = (uint8_t)(u16ID & 0xFFU);
        g_DiagResponse.u.SF.byD3 = (uint8_t)(RAM_UDS.UDS_FazitEOLNumber >> 8);
        g_DiagResponse.u.SF.byD4 = (uint8_t)(RAM_UDS.UDS_FazitEOLNumber & 0x00FFU);
        g_DiagResponse.u.SF.byD5 = (uint8_t)(RAM_UDS.UDS_FazitManufacturerNumber >> 8);
        g_u8BufferOutID = (uint8_t)QR_RFR_DIAG;                                 /* LIN Output buffer is valid (RFR_DIAG) */
    }
    else
    {
        /* Serial Number not saved.
         *  +-----+-----+------+----------+----------+----------+----------+----------+
         *  | NAD | PCI | RSID |    D1    |    D2    |    D3    |    D4    |    D5    |
         *  +-----+-----+------+----------+----------+----------+----------+----------+
         *  | NAD | 0x03| 0x7F |   SID    | ErrorCode| Reserved | Reserved | Reserved |
         *  |     |     |      |  (0x2E)  |  (0x12)  |  (0xFF)  |  (0xFF)  |  (0xFF)  |
         *  +-----+-----+------+----------+----------+----------+----------+----------+
         */
        SetupDiagResponse(g_u8NAD, u8SID, (uint8_t)C_ERRCODE_SFUNC_NOSUP);      /* Status = Negative feedback */
#if (_SUPPORT_LOG_LIN_ERRORS != FALSE)
        SetLastError(C_ERR_LIN2X_2E);
#endif /* (_SUPPORT_LOG_LIN_ERRORS != FALSE) */
    }
} /* End of DiagStoreFazitString() */
#endif /* (_SUPPORT_UDS_WRITE_BY_ID != FALSE) */

/*!*************************************************************************** *
 * UDS_WriteByIdentifier
 * \brief   UDS Write by Identifier
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] pDiag: Pointer to Diagnostics request message
 * \param   [in] u16PCI_SID: PCI and SID code
 * \return  -
 * *************************************************************************** *
 * \details Helper-function for UDS "Write-by-Identifier"
 *          Multiple Data-ID are supported up to au8UDS_DataBuffer-size;
 *          In case all Data-ID's are defined, max buffer-size used: 97 B
 * *************************************************************************** *
 * - Call Hierarchy: HandleDfrDiag()
 * - Cyclomatic Complexity: 8+1
 * - Nesting: 4
 * - Function calling: 9 (CollectUDSData(), SetupDiagResponse(),
 *                        DiagStoreSparePartNumber(),
 *                        DiagStoreHardwareNumber(),
 *                        DiagStoreSerialNumber(),
 *                        DiagStoreSystemName(),
 *                        DiagStoreFazitString(),
 *                        SetLastError(), memmove())
 * *************************************************************************** */
static void UDS_WriteByIdentifier(DFR_DIAG *pDiag, uint16_t u16PCI_SID)
{
    /* This service will write data to the system. Which data will be written,
     * will be defined by the parameter RecordDataIdentifier.
     * Single-frame:
     *  +-----+-----+------+----------+----------+----------+----------+----------+
     *  | NAD | PCI |  SID |    D1    |    D2    |    D3    |    D4    |    D5    |
     *  +-----+-----+------+----------+----------+----------+----------+----------+
     *  | NAD |>0x04| 0x2E |Record-ID1|Record-ID1|  Data #1 |  Data #2 |  Data #3 |
     *  |     |     |      |   (MSB)  |   (LSB)  |          |(optional)|(optional)|
     *  +-----+-----+------+----------+----------+----------+----------+----------+
     *
     *  or
     *  Multiple-frames:
     *  +-----+-----+------+------+----------+----------+----------+----------+  +-----+-----+----------+----------+--------..
     *  | NAD | PCI |  LEN |  SID |    D1    |    D2    |    D3    |    D4    |  | NAD | PCI |    D1    |    D2    |    D3..
     *  +-----+-----+------+------+----------+----------+----------+----------+  +-----+-----+----------+----------+--------..
     *  | NAD | 0x10|Length| 0x2E | Record-ID| Record-ID|  Data #1 |  Data #2 |  | NAD | 0x2n|  Data #3 |  Data #4 |  Data #5
     *  |     |     |      |      |   (MSB)  |   (LSB)  |          |          |  |     |     |          |          |(optional)
     *  +-----+-----+------+------+----------+----------+----------+----------+  +-----+-----+----------+----------+--------..
     *
     * Response (OK):
     *  +-----+-----+------+----------+----------+----------+----------+----------+
     *  | NAD | PCI | RSID |    D1    |    D2    |    D3    |    D4    |    D5    |
     *  +-----+-----+------+----------+----------+----------+----------+----------+
     *  | NAD | 0x03| 0x6E | Record-ID| Record-ID| Reserved | Reserved | Reserved |
     *  |     |     |      |   (MSB)  |   (LSB)  |  (0xFF)  |  (0xFF)  |  (0xFF)  |
     *  +-----+-----+------+----------+----------+----------+----------+----------+
     */
    if (CollectUDSData(pDiag, u16PCI_SID) != FALSE)
    {
        uint16_t u16Len = (l_u16FF_LEN_SID >> 8);
        uint16_t u16RecordLen = 0U;
#if (_DEBUG_UDS_WRITE != FALSE)
        DEBUG_SET_IO_B();
#endif /* (_DEBUG_UDS_WRITE != FALSE) */
        /* Copy the UDS data into a RAM structure */
        p_CopyU16( (sizeof(UDS_LIN_PARAMS_t) / sizeof(uint16_t)),
                   (uint16_t *)&RAM_UDS,
                   (const uint16_t *)ADDR_NV_UDS);

        do
        {
            uint16_t u16RecordDataIdentifier = (((uint16_t)au8UDS_DataBuffer[0]) << 8) |
                                               ((uint16_t)au8UDS_DataBuffer[1]);

            SetupDiagResponse(g_u8NAD, pDiag->u.SF.bySID, (uint8_t)C_ERRCODE_PENDING);

#if (_SUPPORT_UDS_DK >= _SUPPORT_UDS_DK4)
            if (u16RecordDataIdentifier == C_ID_SUB_SYSTEM_NR)
            {
                RAM_USD.UDS_SubSystemID = ((uint16_t)au8UDS_DataBuffer[2] << 8) | (uint16_t)au8UDS_DataBuffer[3];

                /* Positive response - Sub-System ID saved.
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | PCI | RSID |    D1    |    D2    |    D3    |    D4    |    D5    |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD |     |      |  DataID  |  DataID  | Reserved | Reserved | Reserved |
                 *  |     | 0x03| 0x6E |   (MSB)  |   (LSB)  |  (0xFF)  |  (0xFF)  |  (0xFF)  |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 */
                g_DiagResponse.byNAD = g_u8NAD;
                g_DiagResponse.byPCI = (uint8_t)C_RPCI_WRITE_DATA_BY_IDENTIFY;
                g_DiagResponse.u.SF.byRSID = (uint8_t)(C_SID_WRITE_DATA_BY_IDENTIFY | C_RSID_OK);
                g_DiagResponse.u.SF.byD1 = (uint8_t)(u16RecordDataIdentifier >> 8);
                g_DiagResponse.u.SF.byD2 = (uint8_t)(u16RecordDataIdentifier & 0xFFU);
                g_u8BufferOutID = (uint8_t)QR_RFR_DIAG;                         /* LIN Output buffer is valid (RFR_DIAG) */
                u16RecordLen = 2U + 2U;                                         /* Data-ID + SubSystemID */
            }
            else
#endif /* (_SUPPORT_UDS_DK >= _SUPPORT_UDS_DK4) */
            {
                uint8_t *pDest = (uint8_t*)&au8UDS_DataBuffer[0];
#if (_SUPPORT_UDS_DK >= _SUPPORT_UDS_DK4)
                if ( (((u16RecordDataIdentifier & C_SLAVE_ID_MASK) >= C_SLAVE_SPAREPART_NR) &&
                      ((u16RecordDataIdentifier & C_SLAVE_ID_MASK) <= C_SLAVE_FAZIT_STRING)) )
                {
                    if ( (u16RecordDataIdentifier & C_SUBSYS_MASK) != (pNV_UDS->UDS_SubSystemID & C_SUBSYS_MASK) )
                    {
                        /* Invalid Sub-system */
                        u16RecordDataIdentifier = 0U;
                    }
                    else
                    {
                        u16RecordDataIdentifier = u16RecordDataIdentifier & C_SLAVE_ID_MASK;
                    }
                }
#endif /* (_SUPPORT_UDS_DK >= _SUPPORT_UDS_DK4) */
                *pDest++ = (u16RecordDataIdentifier >> 8);
                *pDest++ = (u16RecordDataIdentifier & 0xFFU);
                switch (u16RecordDataIdentifier)
                {
#if (_SUPPORT_UDS_DK >= _SUPPORT_UDS_DK4)
                    case C_SLAVE_SPAREPART_NR:
#endif /* (_SUPPORT_UDS_DK >= _SUPPORT_UDS_DK4) */
                    case C_VW_SPAREPART_NUMBER:
                        /* Slave Spare-part number (0x6200-0x63FF) */
                        DiagStoreSparePartNumber(u16RecordDataIdentifier, C_SID_WRITE_DATA_BY_IDENTIFY);
                        u16RecordLen = 2U + 11U;                                /* Data-ID + Spare-part number */
                        break;
#if (_SUPPORT_UDS_DK >= _SUPPORT_UDS_DK4)
                    case C_SLAVE_HW_NR:
#endif /* (_SUPPORT_UDS_DK >= _SUPPORT_UDS_DK4) */
                    case C_VW_ECU_HW_NUMBER:
                        /* Hardware Number (0x6600-0x67FF) */
                        DiagStoreHardwareNumber(u16RecordDataIdentifier, C_SID_WRITE_DATA_BY_IDENTIFY);
                        u16RecordLen = 2U + 11U;                                /* Data-ID + Hardware number */
                        break;
#if (_SUPPORT_UDS_DK >= _SUPPORT_UDS_DK4)
                    case C_SLAVE_HW_VERSION_NR:
#endif /* (_SUPPORT_UDS_DK >= _SUPPORT_UDS_DK4) */
                    case C_VW_ECU_HW_VERSION:
                        /* Hardware Version */
                        RAM_UDS.UDS_HwVersion_LSB = au8UDS_DataBuffer[2];
                        RAM_UDS.UDS_HwVersion_MSB = au8UDS_DataBuffer[3];
                        RAM_UDS.UDS_HwVersion_XSB = au8UDS_DataBuffer[4];

                        /* Positive response - Slave Spare Part Number saved.
                         *  +-----+-----+------+----------+----------+----------+----------+----------+
                         *  | NAD | PCI | RSID |    D1    |    D2    |    D3    |    D4    |    D5    |
                         *  +-----+-----+------+----------+----------+----------+----------+----------+
                         *  | NAD |     |      |  DataID  |  DataID  | Reserved | Reserved | Reserved |
                         *  |     | 0x03| 0x6E |   (MSB)  |   (LSB)  |  (0xFF)  |  (0xFF)  |  (0xFF)  |
                         *  +-----+-----+------+----------+----------+----------+----------+----------+
                         */
                        g_DiagResponse.byNAD = g_u8NAD;
                        g_DiagResponse.byPCI = (uint8_t)C_RPCI_WRITE_DATA_BY_IDENTIFY;
                        g_DiagResponse.u.SF.byRSID = (uint8_t)(C_SID_WRITE_DATA_BY_IDENTIFY | C_RSID_OK);
                        g_DiagResponse.u.SF.byD1 = (uint8_t)(u16RecordDataIdentifier >> 8);
                        g_DiagResponse.u.SF.byD2 = (uint8_t)(u16RecordDataIdentifier & 0xFFU);
                        g_u8BufferOutID = (uint8_t)QR_RFR_DIAG;                 /* LIN Output buffer is valid (RFR_DIAG) */
                        u16RecordLen = 2U + 3U;                                 /* Data-ID + Hardware versions */
                        break;
#if (_SUPPORT_UDS_DK >= _SUPPORT_UDS_DK4)
                    case C_SLAVE_SERIAL_NR:
#endif /* (_SUPPORT_UDS_DK >= _SUPPORT_UDS_DK4) */
                    case C_VW_ECU_SERIAL_NUMBER:
                        /* Serial Number */
                        DiagStoreSerialNumber(u16RecordDataIdentifier, C_SID_WRITE_DATA_BY_IDENTIFY);
                        u16RecordLen = 2U + 20U;                                /* Data-ID + Serial Number */
                        break;
#if (_SUPPORT_UDS_DK >= _SUPPORT_UDS_DK4)
                    case C_SLAVE_SYSTEM_NAME:
#endif /* (_SUPPORT_UDS_DK >= _SUPPORT_UDS_DK4) */
                    case C_VW_SYSTEM_NAME:
                        /* System Name (13-Byte ASCII) */
                        DiagStoreSystemName(u16RecordDataIdentifier, C_SID_WRITE_DATA_BY_IDENTIFY);
                        u16RecordLen = 2U + 13U;                                /* Data-ID + System Name */
                        break;
#if (_SUPPORT_UDS_DK >= _SUPPORT_UDS_DK4)
                    case C_SLAVE_FAZIT_STRING:
#endif /* (_SUPPORT_UDS_DK >= _SUPPORT_UDS_DK4) */
                    case C_VW_FAZIT_STRING:
                        /* Fazit string (23-Byte ASCII); Only store Production-date */
                        DiagStoreFazitString(u16RecordDataIdentifier, C_SID_WRITE_DATA_BY_IDENTIFY);
                        u16RecordLen = 2U + 23U;                                /* Data-ID + Fazit string */
                        break;
                    default:
                        /* Unsupported Record Data Identifier
                         *  +-----+-----+------+----------+----------+----------+----------+----------+
                         *  | NAD | PCI | RSID |    D1    |    D2    |    D3    |    D4    |    D5    |
                         *  +-----+-----+------+----------+----------+----------+----------+----------+
                         *  | NAD | 0x03| 0x7F |   SID    | ErrorCode| Reserved | Reserved | Reserved |
                         *  |     |     |      |  (0x2E)  |  (0x12)  |  (0xFF)  |  (0xFF)  |  (0xFF)  |
                         *  +-----+-----+------+----------+----------+----------+----------+----------+
                         */
                        SetupDiagResponse(g_u8NAD, pDiag->u.SF.bySID,
                                          (uint8_t)C_ERRCODE_SFUNC_NOSUP);      /* Status = Negative feedback */
#if (_SUPPORT_LOG_LIN_ERRORS != FALSE)
                        SetLastError(C_ERR_LIN2X_2E);
#endif /* (_SUPPORT_LOG_LIN_ERRORS != FALSE) */
                        u16RecordLen = 0U;
                        break;
                }
            }
            if (u16RecordLen != 0U)
            {
                u16Len -= u16RecordLen;
                if (u16Len != 0U)
                {
                    (void)memmove( (void *)&au8UDS_DataBuffer[0],
                                   (void *)&au8UDS_DataBuffer[u16RecordLen],
                                   (uint16_t)u16Len);
                }
                else
                {
                    break;                                                      /* All records copied */
                }
            }
            else
            {
                break;                                                          /* Invalid record */
            }
        } while (u16Len > 2U);                                                  /* At least 2 Bytes (Data-ID) */
        (void)NV_WriteUDS(&RAM_UDS);
#if (_DEBUG_UDS_WRITE != FALSE)
        DEBUG_CLR_IO_B();
#endif /* (_DEBUG_UDS_WRITE != FALSE) */
    }
} /* End of UDS_WriteByIdentifier() */

/*!*************************************************************************** *
 * HandleUDS
 * \brief   Handle Standard LIN Diagnostic Demand Frame
 *          (and optionally prepare response)
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] pDiag: Pointer to Diagnostics request message
 * \param   [in] u16PCI_SID: PCI and SID code
 * \return  -
 * *************************************************************************** *
 * \details Handle Diagnostics Demand Frame and (depended on request),
 *          prepare response frame
 * *************************************************************************** *
 * - Call Hierarchy: HandleDfrDiag()
 * - Cyclomatic Complexity: 4+1
 * - Nesting: 1
 * - Function calling: 4 (SetupDiagResponse(), SetLastError(),
 *                        UDS_ReadByIdentifier(), UDS_WriteByIdentifier())
 *           OPTIONAL: 1 (ResponseSessionCtrl())
 * *************************************************************************** */
void HandleUDS(DFR_DIAG *pDiag, uint16_t u16PCI_SID)
{
    switch (u16PCI_SID & 0x00FFU)
    {
#if (_SUPPORT_UDS_SESSION_CTRL != FALSE)
        case C_SID_DIAG_SESSION_CTRL:
            if (u16PCI_SID == C_PCI_SID_DIAG_SESSION_CTRL)
            {
                /* This service is made to switch between the different diagnostic sessions.
                 * After power-on, the session is always started 01hex. Will the client starts
                 * another session, the server must (eg ECU) only reply positive if all the
                 * specified boundary conditions are satisfied. Sending the positive response
                 * will automatically terminates the previously active session.
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | PCI |  SID |    D1    |    D2    |    D3    |    D4    |    D5    |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | 0x02| 0x10 |SessionTyp| Reserved | Reserved | Reserved | Reserved |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 * Response (OK):
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | PCI | RSID |    D1    |    D2    |    D3    |    D4    |    D5    |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | 0x06| 0x50 | SessionID| P2_SERVER| P2_SERVER| P2_SERVER| P2_SERVER|
                 *  |     |     |      |          |   (MSB)  |   (LSB)  | EXT (MSB)| EXT (LSB)|
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 */
                uint8_t u8SessionType = (pDiag->u.SF.byD1 & 0x7FU);
                uint8_t u8SuppressPosResponse = (pDiag->u.SF.byD1 & 0x80U);

                /* Application Session modes */
                if (g_u8ActiveSessionID == (uint8_t)C_SESSION_DEFAULT_DIAGNOSTIC)
                {
                    /* From DEFAULT session to ... */
                    if (u8SessionType == C_SESSION_DEFAULT_DIAGNOSTIC)
                    {
                        /* ... DEFAULT session (No change) */
                        if (u8SuppressPosResponse == 0U)
                        {
                            ResponseSessionCtrl(g_u8NAD, C_SESSION_DEFAULT_DIAGNOSTIC);
                        }
                    }
                    else
                    {
                        /* Not supported */
                        SetupDiagResponse(g_u8NAD, pDiag->u.SF.bySID,
                                          (uint8_t)C_ERRCODE_SFUNC_NOSUP);
#if (_SUPPORT_LOG_LIN_ERRORS != FALSE)
                        SetLastError(C_ERR_LIN2X_SESSION_CTRL);
#endif /* (_SUPPORT_LOG_LIN_ERRORS != FALSE) */
                    }
                }
#if (_SUPPORT_LOG_LIN_ERRORS != FALSE)
                else
                {
                    /* Not allowed or supported */
                    SetLastError(C_ERR_LIN2X_SESSION_CTRL);
                }
#endif /* (_SUPPORT_LOG_LIN_ERRORS != FALSE) */
            }
            break;
#endif /* (_SUPPORT_UDS_SESSION_CTRL != FALSE) */

#if (_SUPPORT_UDS_RESET != FALSE)
        case C_SID_ECU_RESET:
            if (u16PCI_SID == C_PCI_SID_ECU_RESET)
            {
                /* The UDS-Service ECU-Reset must precede the execution by the server
                 * (such as control unit) be answered positively.
                 * After performing the ECU-reset, the default session ID (0x01) is active.
                 * With this service a software reset will be performed. How exactly this
                 * software-reset has to be performed is defined by the parameter ResetType
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | PCI |  SID |    D1    |    D2    |    D3    |    D4    |    D5    |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | 0x02| 0x11 | ResetType| Reserved | Reserved | Reserved | Reserved |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 * Response (OK):
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | PCI | RSID |    D1    |    D2    |    D3    |    D4    |    D5    |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 *  | NAD | 0x02| 0x51 | ResetType| Reserved | Reserved | Reserved | Reserved |
                 *  |     | 0x03|      |          | or Time  |  (0xFF)  |  (0xFF)  |  (0xFF)  |
                 *  +-----+-----+------+----------+----------+----------+----------+----------+
                 */
                uint8_t u8EcuResetType = (pDiag->u.SF.byD1 & 0x7FU);
                uint8_t u8SuppressPosResponse = (pDiag->u.SF.byD1 & 0x80U);

                /* Application Session modes */
                if (g_u8ActiveSessionID == (uint8_t)C_SESSION_DEFAULT_DIAGNOSTIC)
                {
                    /* Type C_ECU_RESET_OFF_ON is allowed */
                    if (u8EcuResetType == C_ECU_RESET_OFF_ON)
                    {
                        if (u8SuppressPosResponse == 0U)
                        {
                            /* TODO [MMP]: Unclear if response should be received before actual reset or after */
                            g_DiagResponse.byNAD = g_u8NAD;
                            g_DiagResponse.u.SF.byRSID = (uint8_t)C_RSID_ECU_RESET;
                            g_DiagResponse.u.SF.byD1 = u8EcuResetType;
                            if (u8EcuResetType == C_ECU_RESET_RAPID_POWER_SHUTDOWN)
                            {
                                g_DiagResponse.byPCI = (uint8_t)C_RPCI_ECU_RESET_04;
                                g_DiagResponse.u.SF.byD2 = (uint8_t)C_TIME_TO_RAPID_POWER_DOWN;  /* TimeToPowerDown, when ResetType 0x04 is supported */
                            }
                            else
                            {
                                g_DiagResponse.byPCI = (uint8_t)C_RPCI_ECU_RESET;
                            }

                            g_u8BufferOutID = (uint8_t)QR_RFR_DIAG;             /* LIN Output buffer is valid (RFR_DIAG) */
                        }

                        /* Method I: Reset through a watchdog-timer reset, by setting Watchdog-timer to minimum period (100us) */
                        AppReset()
                    }
                    else
                    {
                        if (u8EcuResetType == C_ECU_RESET_HARD)
                        {
                            /* Not allowed */
                            SetupDiagResponse(g_u8NAD, pDiag->u.SF.bySID,
                                              (uint8_t)C_ERRCODE_SUBFUNC_NOT_SUPP_IN_ACT_SESSION);
                        }
                        else
                        {
                            /* Not supported */
                            SetupDiagResponse(g_u8NAD, pDiag->u.SF.bySID,
                                              (uint8_t)C_ERRCODE_SFUNC_NOSUP);
                        }
#if (_SUPPORT_LOG_LIN_ERRORS != FALSE)
                        SetLastError(C_ERR_LIN2X_ECU_RESET);
#endif /* (_SUPPORT_LOG_LIN_ERRORS != FALSE) */
                    }
                }
#if (_SUPPORT_LOG_LIN_ERRORS != FALSE)
                else
                {
                    /* Not allowed or supported */
                    SetLastError(C_ERR_LIN2X_ECU_RESET);
                }
#endif /* (_SUPPORT_LOG_LIN_ERRORS != FALSE) */
            }
            break;
#endif /* (_SUPPORT_UDS_RESET != FALSE) */

        case C_SID_READ_DATA_BY_IDENTIFY:
        {
            UDS_ReadByIdentifier(pDiag, u16PCI_SID);
        }
        break;
#if (_SUPPORT_UDS_WRITE_BY_ID != FALSE)
        case C_SID_WRITE_DATA_BY_IDENTIFY:
        {
            UDS_WriteByIdentifier(pDiag, u16PCI_SID);
        }
        break;
#endif /* (_SUPPORT_UDS_WRITE_BY_ID != FALSE) */

#if (_SUPPORT_UDS_TESTER_PRESENT != FALSE)
        case C_SID_TESTER_PRESENT:
        {
            /* This service is used by one or more servers an active connection with
             * show a client (eg, tester, on-board tester). Each activated at this time
             * Diagnostic compound and each activated diagnostic session (with the exception of the session 01hex)
             * thereby remain activated.
             *  +-----+-----+------+----------+----------+----------+----------+----------+
             *  | NAD | PCI |  SID |    D1    |    D2    |    D3    |    D4    |    D5    |
             *  +-----+-----+------+----------+----------+----------+----------+----------+
             *  | NAD | 0x01| 0x3E | SubFunct.| Reserved | Reserved | Reserved | Reserved |
             *  +-----+-----+------+----------+----------+----------+----------+----------+
             * Response (OK):
             *  +-----+-----+------+----------+----------+----------+----------+----------+
             *  | NAD | PCI | RSID |    D1    |    D2    |    D3    |    D4    |    D5    |
             *  +-----+-----+------+----------+----------+----------+----------+----------+
             *  | NAD | 0x01| 0x7E | SubFunct.| Reserved | Reserved | Reserved | Reserved |
             *  +-----+-----+------+----------+----------+----------+----------+----------+
             */
            uint8_t u8ZeroSubFunction = (pDiag->u.SF.byD1 & 0x7FU);
            uint8_t u8SuppressPosResponse = (pDiag->u.SF.byD1 & 0x80U);
            if (u8ZeroSubFunction == 0x00U)
            {
                if (u8SuppressPosResponse == 0U)
                {
                    g_DiagResponse.byNAD = g_u8NAD;
                    g_DiagResponse.byPCI = (uint8_t)C_RPCI_TESTER_PRESENT;
                    g_DiagResponse.u.SF.byRSID = (uint8_t)C_RSID_TESTER_PRESENT;
                    g_DiagResponse.u.SF.byD1 = u8ZeroSubFunction;

                    g_u8BufferOutID = (uint8_t)QR_RFR_DIAG;                     /* LIN Output buffer is valid (RFR_DIAG) */
                } /* End of ResponseTesterPresent() */
            }
            else
            {
                /* Not supported */
                SetupDiagResponse(g_u8NAD, pDiag->u.SF.bySID,
                                  (uint8_t)C_ERRCODE_SFUNC_NOSUP);
#if (_SUPPORT_LOG_LIN_ERRORS != FALSE)
                SetLastError(C_ERR_LIN2X_3E);
#endif /* (_SUPPORT_LOG_LIN_ERRORS != FALSE) */
            }
        }
        break;
#endif /* (_SUPPORT_UDS_TESTER_PRESENT != FALSE) */

        default:
        {
            SetupDiagResponse(g_u8NAD, pDiag->u.SF.bySID,
                              (uint8_t)C_ERRCODE_SFUNC_NOSUP);                  /* Status = Negative feedback */
        }
        break;
    }
} /* End of HandleUDS() */

/*!***************************************************************************** *
 * HandleRfrDiagMF()
 * \brief   Handle Diagnostic Demand Frame (and optionally prepare response)
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details Multiple-frames, First Frame:
 *  +-----+------+----------+----------+----------+----------+----------+----------+
 *  | NAD |  PCI |    LEN   |   RSID   |    D1    |    D2    |    D3    |    D4    |
 *  +-----+------+----------+----------+----------+----------+----------+----------+
 *  | NAD | 0x10 |  Length  |   SID|   |  Data #1 |  Data #2 |  Data #3 |  Data #4 |
 *  |     |      |          |   0x40   |          |          |          |          |
 *  +-----+------+----------+----------+----------+----------+----------+----------+
 *
 *  Multiple-frames, Continuous Frame:
 *  +-----+------+----------+----------+----------+----------+----------+----------+
 *  | NAD |  PCI |    D1    |    D2    |    D3    |    D4    |    D5    |    D6    |
 *  +-----+------+----------+----------+----------+----------+----------+----------+
 *  | NAD | 0x2n |  Data #5 |  Data #6 |  Data #7 |  Data #8 |  Data #9 | Data #10 |
 *  +-----+------+----------+----------+----------+----------+----------+----------+
 *
 * *************************************************************************** *
 * - Call Hierarchy: mlu_DataRequest()
 * - Cyclomatic Complexity: 9+1
 * - Nesting: 3
 * - Function calling: 0
 * *************************************************************************** */
void HandleRfrDiagMF(void)
{
#if defined (__MLX81330A01__) || defined (__MLX81332A01__)
    RFR_DIAG *pRfrDiag = (RFR_DIAG *)((void *)&LinFrameDataBuffer[0]);
#else
    RFR_DIAG *pRfrDiag = (RFR_DIAG *)((void *)ML_DATA_LIN_FRAME_DATA_BUFFER);
#endif
    if (l_u8FrameCount == 0U)
    {
        /* First Frame */
        pRfrDiag->byNAD = g_u8NAD;
        pRfrDiag->byPCI = C_PCI_FF_TYPE;
        pRfrDiag->u.FF.byLEN = (uint8_t)(l_u16FF_LEN_SID >> 8);
        pRfrDiag->u.FF.byRSID = (uint8_t)(l_u16FF_LEN_SID & 0xFFU);
        {
            uint8_t *dst = (uint8_t *)&pRfrDiag->u.FF.byD1;
            uint8_t *src = (uint8_t *)&au8UDS_DataBuffer[0];
            dst[0] = src[0];
            dst[1] = src[1]; /*lint !e415 */
            dst[2] = src[2]; /*lint !e415 !e416 */
            dst[3] = src[3]; /*lint !e415 !e416 */
        }
        l_u8FrameCount++;
        l_u16FF_LEN_SID -= ((4U + 1U) << 8);                                    /* Reduce length by 1 (RSID) + 4 Data-bytes */
    }
    else
    {
        /* Continuous Frame */
        uint8_t u8Count = (uint8_t)((l_u16FF_LEN_SID & 0xFF00U) >> 8);
        pRfrDiag->byNAD = g_u8NAD;
        pRfrDiag->byPCI = C_PCI_CF_TYPE | (l_u8FrameCount & 0x0FU);
        if (u8Count > 6U)                                                       /* Maximum number of LIN-frame data-bytes */
        {
            u8Count = 6U;                                                       /* Limit to a single frame */
        }
        else
        {
            /* Fill up with 0xFF */
            if (u8Count <= 5U)
            {
                pRfrDiag->u.CF.byD6 = 0xFFU;
            }
            if (u8Count <= 4U)
            {
                pRfrDiag->u.CF.byD5 = 0xFFU;
            }
            if (u8Count <= 3U)
            {
                pRfrDiag->u.CF.byD4 = 0xFFU;
            }
            if (u8Count <= 2U)
            {
                pRfrDiag->u.CF.byD3 = 0xFFU;
            }
            if (u8Count == 1U)
            {
                pRfrDiag->u.CF.byD2 = 0xFFU;
            }
        }
        l_u16FF_LEN_SID -= (((uint16_t)u8Count) << 8);
        {
            uint8_t *pDst = (uint8_t *)&pRfrDiag->u.CF.byD1;
            uint8_t *pSrc = (uint8_t *)&au8UDS_DataBuffer[(l_u8FrameCount * 6U) - 2U];
            while (u8Count != 0U)
            {
                u8Count--;
                *pDst++ = *pSrc++;
            }
        }
        if ( (l_u16FF_LEN_SID & 0xFF00U) == 0x0000U)
        {
            g_u8BufferOutID = (uint8_t)QR_INVALID;                              /* Response completely transmitted */
        }
        else
        {
            l_u8FrameCount++;                                                   /* More response data to go (next frame) */
        }
    }
} /* End of HandleRfrDiagMF() */

#endif /* (LIN_COMM != FALSE) && (_SUPPORT_UDS != FALSE) */

/* EOF */
