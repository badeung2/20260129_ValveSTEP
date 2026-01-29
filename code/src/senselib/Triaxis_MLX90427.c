/*!*************************************************************************** *
 * \file        Triaxis_MLX90427.c
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
 * A list of functions:
 *  - Global Functions:
 *           -# Triaxis_Init()
 *           -# Triaxis_SendCmd()
 *           -# Triaxis_GetAbsPos()
 *           -# Adapt_TriaxisAngleOffset
 *           -# HandleTriaxisStatus()
 *  - Internal Functions:
 *           -# Triaxis_CRC()
 *           -# Triaxis_TransferMsg()
 *
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

/*!*************************************************************************** */
/*                           INCLUDES                                          */
/* *************************************************************************** */
#include "AppBuild.h"                                                           /* Application Build */

#if (_SUPPORT_TRIAXIS_MLX90427 != FALSE)

#include "senselib/Triaxis_MLX90427.h"                                          /* (SPI) Triaxis MLX90427 support */

#include "commlib/SPI.h"                                                        /* SPI support */

#if (LIN_COMM != FALSE)
#include "commlib/LIN_Communication.h"                                          /* LIN support */
#include "drivelib/ErrorCodes.h"                                                /* Error-logging support */
#endif /* (LIN_COMM != FALSE) */
#include "drivelib/GlobalVars.h"                                                /* Global Variables support */
#include "drivelib/NV_UserPage.h"                                               /* Non Volatile Memory User Application Page Layout */
#include "camculib/private_mathlib.h"                                           /* Private Math Library support */

#include <bl_bist.h>
#include <plib.h>                                                               /* Product libraries */

/*!*************************************************************************** */
/*                           GLOBAL & LOCAL VARIABLES                          */
/* *************************************************************************** */
#pragma space dp                                                                /* __TINY_SECTION__ */
#pragma space none                                                              /* __TINY_SECTION__ */

uint16_t g_u16TriaxisActualPos;                                                 /*!< Actual Triaxis Position */
uint16_t g_u16TriaxisTargetPos;                                                 /*!< Target Triaxis Position */

#pragma space nodp                                                              /* __NEAR_SECTION__ */
uint16_t g_u16TriaxisAlpha = 0xFFFFU;                                           /*!< Triaxis Alpha */
uint16_t g_u16TriaxisBeta = 0xFFFFU;                                            /*!< Triaxis Beta */
uint16_t g_u16TriaxisX = 0xFFFFU;                                               /*!< Triaxis X-position */
uint16_t g_u16TriaxisY = 0xFFFFU;                                               /*!< Triaxis Y-position */
uint16_t g_u16TriaxisZ = 0xFFFFU;                                               /*!< Triaxis Z-position */
int16_t g_i16TriaxisTemperature = 25;                                           /*!< Triaxis Temperature [C] */
uint16_t l_u16Data0, l_u16Data1, l_u16Data2;                                    /*!< Triaxis RESULT_DATA */
uint16_t g_u16ShaftAngle = 0U;                                                  /*!< Shaft Angle; Triaxis angle corrected with Sense-magnet offset */
static uint16_t l_u16TriaxisAngleOffset = 0U;                                   /*!< Triaxis Angle Offset (between sense-magnet and shaft) */
static uint16_t l_u16TriaxisAngleRaw = 0U;                                      /*!< Triaxis Angle Raw uncorrected (sense-magnet) */
static uint8_t l_u8SenseMagnetPolePairs = 1U;                                   /*!< Sense magnet pole-pairs */
static uint8_t l_u8TriaxisDir = TRUE;                                           /*!< Triaxis direction opposite from actuator */
uint8_t g_u8TriaxisLastSpiCmd = CMD_TRIAXIS_NOP;                                /*!< Triaxis Last SPI Command */
uint8_t g_u8TriaxisError = ERR_TRIAXIS_UNKNOWN;                                 /*!< Triaxis Error */

/* MLX90427 SPI Status, Acknowledge, Error */
volatile uint32_t l_u32DiagsState = 0UL;                                        /*!< DIAGS_STATE: decoding of the diagnostic bits */
volatile uint8_t l_u8Type = 0U;                                                 /*!< TYPE[3:0]-field */
volatile uint8_t l_u8State = 0U;                                                /*!< STATE[3:0]-field */
volatile uint8_t l_u8FrameCountOrErrorCode = 0U;                                /*!< FRAME_COUNT[7:0] or ERROR_CODE[7:0] field */
volatile uint8_t l_u8OPC = 0U;                                                  /*!< OPC: Opcode[6:0] of the message that “triggered” the reply. */

#if (_SUPPORT_TRIAXIS_STANDBY == FALSE)
uint8_t g_u8LinTriaxisCmd = CMD_TRIAXIS_NOP;                                    /*!< SPI-command Triaxis Command field (3-bits) */
uint16_t g_u16TriaxisPollCounter = 0U;                                          /*!< Triaxis Poll Period Counter */
#endif /* (_SUPPORT_TRIAXIS_STANDBY == FALSE) */
#pragma space none                                                              /* __NEAR_SECTION__ */

/*! Triaxis CRC table */
static const uint8_t scau8CRC[256] =
{
    0x00U, 0x2FU, 0x5EU, 0x71U, 0xBCU, 0x93U, 0xE2U, 0xCDU,
    0x57U, 0x78U, 0x09U, 0x26U, 0xEBU, 0xC4U, 0xB5U, 0x9AU,
    0xAEU, 0x81U, 0xF0U, 0xDFU, 0x12U, 0x3DU, 0x4CU, 0x63U,
    0xF9U, 0xD6U, 0xA7U, 0x88U, 0x45U, 0x6AU, 0x1BU, 0x34U,
    0x73U, 0x5CU, 0x2DU, 0x02U, 0xCFU, 0xE0U, 0x91U, 0xBEU,
    0x24U, 0x0BU, 0x7AU, 0x55U, 0x98U, 0xB7U, 0xC6U, 0xE9U,
    0xDDU, 0xF2U, 0x83U, 0xACU, 0x61U, 0x4EU, 0x3FU, 0x10U,
    0x8AU, 0xA5U, 0xD4U, 0xFBU, 0x36U, 0x19U, 0x68U, 0x47U,
    0xE6U, 0xC9U, 0xB8U, 0x97U, 0x5AU, 0x75U, 0x04U, 0x2BU,
    0xB1U, 0x9EU, 0xEFU, 0xC0U, 0x0DU, 0x22U, 0x53U, 0x7CU,
    0x48U, 0x67U, 0x16U, 0x39U, 0xF4U, 0xDBU, 0xAAU, 0x85U,
    0x1FU, 0x30U, 0x41U, 0x6EU, 0xA3U, 0x8CU, 0xFDU, 0xD2U,
    0x95U, 0xBAU, 0xCBU, 0xE4U, 0x29U, 0x06U, 0x77U, 0x58U,
    0xC2U, 0xEDU, 0x9CU, 0xB3U, 0x7EU, 0x51U, 0x20U, 0x0FU,
    0x3BU, 0x14U, 0x65U, 0x4AU, 0x87U, 0xA8U, 0xD9U, 0xF6U,
    0x6CU, 0x43U, 0x32U, 0x1DU, 0xD0U, 0xFFU, 0x8EU, 0xA1U,
    0xE3U, 0xCCU, 0xBDU, 0x92U, 0x5FU, 0x70U, 0x01U, 0x2EU,
    0xB4U, 0x9BU, 0xEAU, 0xC5U, 0x08U, 0x27U, 0x56U, 0x79U,
    0x4dU, 0x62U, 0x13U, 0x3CU, 0xF1U, 0xDEU, 0xAFU, 0x80U,
    0x1AU, 0x35U, 0x44U, 0x6BU, 0xA6U, 0x89U, 0xF8U, 0xD7U,
    0x90U, 0xBFU, 0xCEU, 0xE1U, 0x2CU, 0x03U, 0x72U, 0x5DU,
    0xC7U, 0xE8U, 0x99U, 0xB6U, 0x7BU, 0x54U, 0x25U, 0x0AU,
    0x3EU, 0x11U, 0x60U, 0x4FU, 0x82U, 0xADU, 0xDCU, 0xF3U,
    0x69U, 0x46U, 0x37U, 0x18U, 0xD5U, 0xFAU, 0x8BU, 0xA4U,
    0x05U, 0x2AU, 0x5BU, 0x74U, 0xB9U, 0x96U, 0xE7U, 0xC8U,
    0x52U, 0x7DU, 0x0CU, 0x23U, 0xEEU, 0xC1U, 0xB0U, 0x9FU,
    0xABU, 0x84U, 0xF5U, 0xDAU, 0x17U, 0x38U, 0x49U, 0x66U,
    0xFCU, 0xD3U, 0xA2U, 0x8DU, 0x40U, 0x6FU, 0x1EU, 0x31U,
    0x76U, 0x59U, 0x28U, 0x07U, 0xCAU, 0xE5U, 0x94U, 0xBBU,
    0x21U, 0x0EU, 0x7FU, 0x50U, 0x9DU, 0xB2U, 0xC3U, 0xECU,
    0xD8U, 0xF7U, 0x86U, 0xA9U, 0x64U, 0x4BU, 0x3AU, 0x15U,
    0x8FU, 0xA0U, 0xD1U, 0xFEU, 0x33U, 0x1CU, 0x6DU, 0x42U
};

/*!*************************************************************************** *
 * Triaxis_CRC
 * \brief   Calculate Triaxis MLX90427 SPI message CRC
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] *pu8TriaxisMsg: Pointer to Triaxis Message
 * \return  (uint8_t) Checksum of Triaxis Message
 * *************************************************************************** *
 * \details The communication is protected with a CRC-8, using polynomial 0x2F
 *          (x8+x5+x3+x2+x+1), initialisation value 0xFF, and final XOR value 0x00.
 * *************************************************************************** *
 * - Call Hierarchy: Triaxis_TransferMsg()
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 0
 * *************************************************************************** */
static uint8_t Triaxis_CRC(uint8_t *pu8TriaxisMsg)
{
    uint8_t u8CRC = 0xFFU;
    u8CRC = scau8CRC[ pu8TriaxisMsg[0] ^ u8CRC ];
    u8CRC = scau8CRC[ pu8TriaxisMsg[1] ^ u8CRC ];
    u8CRC = scau8CRC[ pu8TriaxisMsg[2] ^ u8CRC ];
    u8CRC = scau8CRC[ pu8TriaxisMsg[3] ^ u8CRC ];
    u8CRC = scau8CRC[ pu8TriaxisMsg[4] ^ u8CRC ];
    u8CRC = scau8CRC[ pu8TriaxisMsg[5] ^ u8CRC ];
    u8CRC = scau8CRC[ pu8TriaxisMsg[6] ^ u8CRC ];
    u8CRC = u8CRC ^ 0x00U;                                                      /* Final XOR value */
    return (u8CRC);
} /* End of Triaxis_CRC() */

/*!*************************************************************************** *
 * Triaxis_TransferMsg
 * \brief   Transfer message to Triaxis
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  (uint16_t) result
 *                 ERR_MLX90363_OK: Okay
 *                 ERR_MLX90363_CRC: Checksum error
 * *************************************************************************** *
 * \details
 * *************************************************************************** *
 * - Call Hierarchy: Triaxis_SendCmd()
 * - Cyclomatic Complexity: 2+1
 * - Nesting: 1
 * - Function calling: 2 (Triaxis_CRC(), SPI_Transfer())
 * *************************************************************************** */
static uint16_t Triaxis_TransferMsg(void)
{
    g_au8SpiTxBuf[7] = Triaxis_CRC(g_au8SpiTxBuf);
    {
#if (_SUPPORT_SPI_DMA != FALSE)
        IO_SPI_RBASE = (uint16_t)((uint8_t *)&g_au8SpiRxBuf);
        IO_SPI_SBASE = (uint16_t)((uint8_t *)&g_au8SpiTxBuf);
        IO_SPI_MSGLEN = 8U;                                                     /* Buffer-length in "Words" (of 8-bits) */
        IO_SPI_CTRL = (IO_SPI_CTRL & ~(B_SPI_STOP | B_SPI_START)) | B_SPI_DMA;  /* Enable DMA */
        IO_SPI_CTRL = B_SPI_START;                                              /* Start SPI */
        IO_SPI_DR = 0U;                                                         /* Dummy write to trigger SPI transfer */
        while ( (IO_SPI_CTRL & B_SPI_RF) == 0U) {}                              /* Wait for the transfer to be completed */
        IO_SPI_CTRL = B_SPI_STOP;
#else  /* (_SUPPORT_SPI_DMA != FALSE) */
        g_u8SpiTxLen = 8U;                                                      /* Buffer-length in "Words" (of 8-bits) */
        SPI_Transfer();
#endif /* (_SUPPORT_SPI_DMA != FALSE) */
    }
    if (g_au8SpiRxBuf[7] == Triaxis_CRC(g_au8SpiRxBuf) )
    {
        return (ERR_MLX90427_OK);
    }
    return (ERR_MLX90427_CRC);

} /* End of Triaxis_Transfer() */

/*!*************************************************************************** *
 * Triaxis_SendCmd
 * \brief   Send command to Triaxis
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u8TriaxisApiCmd: Triaxis Command
 *                 CMD_TRIAXIS_NOP
 *                 CMD_TRIAXIS_STANDBY
 *                 CMD_TRIAXIS_RESET
 *                 CMD_TRIAXIS_ALPHA
 *                 CMD_TRIAXIS_MEM_RD
 * \return  (uint16_t) result
 *                 ERR_TRIAXIS_OK: Okay
 *                 ERR_TRIAXIS_INVLEN: Incorrect BitCount
 *                 ERR_TRIAXIS_NODATA: Answer = NTT message;
 *                      Two reasons: Answer Time-Out or Answer not Ready
 *                 ERR_TRIAXIS_INVCMD: OPCODE not valid
 *                 ERR_TRIAXIS_CRC: Checksum error
 *                 ERR_TRIAXIS_NODATA: Nothing to Transmit (NTT)
 *                 ERR_TRIAXIS_UNKNOWN: Unknown
 *                 ERR_TRIAXIS_INVREPLY: Invalid Reply
 * *************************************************************************** *
 * \details
 * *************************************************************************** *
 * - Call Hierarchy: Triaxis_GetAbsPos(), Triaxis_Init(), TriaxisPostInit()
 * - Cyclomatic Complexity: 17+1
 * - Nesting: -
 * - Function calling: 1 (Triaxis_TransferMsg())
 * *************************************************************************** */
uint16_t Triaxis_SendCmd(uint8_t u8TriaxisApiCmd)
{
    uint16_t *pu16TriaxisReq = (uint16_t *)(void *)g_au8SpiTxBuf;
    pu16TriaxisReq[0] = 0x0000U;                                                /* g_au8SpiTxBuf[1:0] = 0x00 */
    pu16TriaxisReq[1] = 0x0000U;                                                /* g_au8SpiTxBuf[3:2] = 0x00 */

    /* Translate Triaxis command to MLX90427 Triaxis command */
    if (u8TriaxisApiCmd == (uint8_t)CMD_TRIAXIS_NOP)
    {
        pu16TriaxisReq[2] = 0x0000U;                                            /* g_au8SpiTxBuf[5:4] = 0x00 */
        g_au8SpiTxBuf[6] = (uint8_t)CMD_MLX90427_NOP;
    }
    else if (u8TriaxisApiCmd == (uint8_t)CMD_TRIAXIS_ALPHA)
    {
        g_au8SpiTxBuf[4] = (C_MLX90427_GET_MODE_LEGACY << 4) | C_MLX90427_GET_SEL_DEFAULT;  /* Respectively MODE (= Legacy) and SEL (secondary data) */
        g_au8SpiTxBuf[5] = 0x00U;                                               /* RC = 0 */
        g_au8SpiTxBuf[6] = (uint8_t)CMD_MLX90427_TRG_NORMAL;
    }
    else if (u8TriaxisApiCmd == (uint8_t)CMD_TRIAXIS_ALPHA_BETA)
    {
        g_au8SpiTxBuf[4] = (C_MLX90427_GET_MODE_2D_LEGACY << 4) | C_MLX90427_GET_SEL_DEFAULT;  /* Respectively MODE (= Legacy) and SEL (secondary data) */
        g_au8SpiTxBuf[5] = 0x00U;                                               /* RC = 0 */
        g_au8SpiTxBuf[6] = (uint8_t)CMD_MLX90427_TRG_NORMAL;
    }
    else if (u8TriaxisApiCmd == (uint8_t)CMD_TRIAXIS_XYZ)
    {
        g_au8SpiTxBuf[4] = (C_MLX90427_GET_MODE_3D << 4) | C_MLX90427_GET_SEL_DEFAULT;  /* Respectively MODE (= Legacy) and SEL (secondary data) */
        g_au8SpiTxBuf[5] = 0x00U;                                               /* RC = 0 */
        g_au8SpiTxBuf[6] = (uint8_t)CMD_MLX90427_TRG_NORMAL;
    }
    else if (u8TriaxisApiCmd == (uint8_t)CMD_TRIAXIS_STANDBY)
    {
        pu16TriaxisReq[2] = C_MLX90427_STBY_KEY;                                /* g_au8SpiTxBuf[5:4] = C_MLX90427_STBY_KEY */
        g_au8SpiTxBuf[6] = (uint8_t)CMD_MLX90427_STBY;
    }
    else if (u8TriaxisApiCmd == (uint8_t)CMD_TRIAXIS_RESET)
    {
        pu16TriaxisReq[2] = C_MLX90427_RST_KEY;
        g_au8SpiTxBuf[6] = (uint8_t)CMD_MLX90427_RST;
    }
    else if (u8TriaxisApiCmd == (uint8_t)CMD_TRIAXIS_GET)
    {
        /* Get Chip ID; DATA0 = CHIP_ID[0], DATA1 = CHIP_ID[1] & DATA2 = CHIP_ID[2] */
        g_au8SpiTxBuf[4] = C_MLX90427_GET_SEL_CHIP_ID;                          /* GET_SEL */
        g_au8SpiTxBuf[5] = 0x00U;                                               /* Reserved */
        g_au8SpiTxBuf[6] = (uint8_t)CMD_MLX90427_TRG_NORMAL;
    }
    else
    {
        /* Nothing */
        g_u8TriaxisError = ERR_TRIAXIS_INVCMD;
    }

    /* Sending (new) command / Received (previous command) data, via SPI */
    if (Triaxis_TransferMsg() == ERR_MLX90427_CRC)
    {
        g_u8TriaxisError = (ERR_TRIAXIS_CRC);
    }
    else
    {
        /* Process (previous command) data */
        if ((g_au8SpiRxBuf[6] & 0x80U) == 0x00U)
        {
            /* SPI RX-buffer[6], bit 7 = '0': Status, Acknowledge, Error */
            l_u32DiagsState = ((uint32_t)((((uint16_t) g_au8SpiRxBuf[0]) << 8) | (uint16_t) g_au8SpiRxBuf[1]) << 16) || \
                               ((((uint16_t) g_au8SpiRxBuf[2]) << 8) | (uint16_t) g_au8SpiRxBuf[3]);
            l_u8Type = g_au8SpiRxBuf[4] >> 4;
            l_u8State = g_au8SpiRxBuf[4] & 0x0FU;
            l_u8FrameCountOrErrorCode = g_au8SpiRxBuf[5];
            l_u8OPC = g_au8SpiRxBuf[6] & 0x7FU;
            if (l_u8Type == 0x08U)
            {
                /* ERROR */
                   if (l_u8FrameCountOrErrorCode == C_MLX90427_ERR_CRC)
                   {
                       g_u8TriaxisError = ERR_TRIAXIS_CRC;                      /* During TX */
                   }
                   else if (l_u8FrameCountOrErrorCode == C_MLX90427_ERR_OPC)
                   {
                       g_u8TriaxisError = ERR_TRIAXIS_INVCMD;                   /* Invalid OPC */
                   }
                   else if (l_u8FrameCountOrErrorCode == C_MLX90427_ERR_ARGS)
                   {
                       g_u8TriaxisError = ERR_TRIAXIS_INVDATA;                  /* Invalid arguments are provided */
                   }
                   else
                   {
                       g_u8TriaxisError = ERR_TRIAXIS_UNKNOWN;
                   }
            }
            else
            {
                g_u8TriaxisError = ERR_TRIAXIS_OK;
            }
        }
        else
        {
            /* SPI RX-buffer[6], bit 7 = '1' */
            if ((g_au8SpiRxBuf[6] & 0x40U) == 0x00U)
            {
                /* SPI RX-buffer[6], bit 7:6 = '10': Measurement Results */
                if (g_u8TriaxisLastSpiCmd == CMD_MLX90427_TRG_NORMAL)
                {
                    uint8_t u8Mode = (g_au8SpiRxBuf[4] >> 4);
                    if (u8Mode == C_MLX90427_GET_MODE_LEGACY)
                    {
                        /* CMD_TRIAXIS_ALPHA */
                        g_u16TriaxisAlpha = ((((uint16_t) g_au8SpiRxBuf[0]) << 8) | (uint16_t) g_au8SpiRxBuf[1]) & C_MLX90427_VALUE_MASK;
                        g_i16TriaxisTemperature = ((int16_t)((((uint16_t) (g_au8SpiRxBuf[4] & 0xFU)) << 8) | (uint16_t) g_au8SpiRxBuf[5]) - 585) / 8;
                    }
                    else if (u8Mode == C_MLX90427_GET_MODE_2D_LEGACY)
                    {
                        g_u16TriaxisAlpha = ((((uint16_t) g_au8SpiRxBuf[0]) << 8) | (uint16_t) g_au8SpiRxBuf[1]) & C_MLX90427_VALUE_MASK;
                        g_u16TriaxisBeta = ((((uint16_t) g_au8SpiRxBuf[2]) << 8) | (uint16_t) g_au8SpiRxBuf[3]);
                        g_i16TriaxisTemperature = ((int16_t)((((uint16_t) (g_au8SpiRxBuf[4] & 0xFU)) << 8) | (uint16_t) g_au8SpiRxBuf[5]) - 585) / 8;
                    }
                    else
                    {
                        g_u16TriaxisX = ((((uint16_t) g_au8SpiRxBuf[0]) << 8) | (uint16_t) g_au8SpiRxBuf[1]) & C_MLX90427_VALUE_MASK;
                        g_u16TriaxisY = ((((uint16_t) g_au8SpiRxBuf[2]) << 8) | (uint16_t) g_au8SpiRxBuf[3]) & C_MLX90427_VALUE_MASK;
                        g_u16TriaxisZ = ((((uint16_t) g_au8SpiRxBuf[4]) << 8) | (uint16_t) g_au8SpiRxBuf[5]) & C_MLX90427_VALUE_MASK;
                    }
                    g_u8TriaxisError = ERR_TRIAXIS_OK;
                }
                else
                {
                    g_u8TriaxisError = ERR_TRIAXIS_INVREPLY;
                }
            }
            else
            {
                /* SPI RX-buffer[6], bit 7:5 = '110': Returned Data */
                l_u16Data0 = ((((uint16_t) g_au8SpiRxBuf[0]) << 8) | (uint16_t) g_au8SpiRxBuf[1]);
                l_u16Data1 = ((((uint16_t) g_au8SpiRxBuf[2]) << 8) | (uint16_t) g_au8SpiRxBuf[3]);
                l_u16Data2 = ((((uint16_t) g_au8SpiRxBuf[4]) << 8) | (uint16_t) g_au8SpiRxBuf[5]);
                g_u8TriaxisError = ERR_TRIAXIS_OK;
            }

        }
    }

    /* Memorise new Triaxis command as previous command */
    g_u8TriaxisLastSpiCmd = g_au8SpiTxBuf[6];

    return (g_u8TriaxisError);

} /* End of Triaxis_SendCmd() */

/*!*************************************************************************** *
 * Triaxis_GetAbsPos
 * \brief   Get Triaxis MLX90363 absolute position
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16Truncate
 *                    FALSE: Real Triaxis position between 0-360 degrees
 *                    TRUE : Real Triaxis position truncated between
 *                           C_TRIAXIS_APP_BGN - C_TRIAXIS_APP_END degrees and
 *                           mapped to C_TRIAXIS_0DEG - C_TRIAXIS_APP_RNG degrees
 * \return  (uint16_t) result
 *                 ERR_TRIAXIS_OK: Okay
 *                 ERR_TRIAXIS_INVLEN: Incorrect BitCount
 *                 ERR_TRIAXIS_NODATA: Answer = NTT message;
 *                      Two reasons: Answer Time-Out or Answer not Ready
 *                 ERR_TRIAXIS_INVCMD: OPCODE not valid
 *                 ERR_TRIAXIS_CRC: Checksum error
 *                 ERR_TRIAXIS_NODATA: Nothing to Transmit (NTT)
 *                 ERR_TRIAXIS_UNKNOWN: Unknown
 *                 ERR_TRIAXIS_INVREPLY: Invalid Reply
 *                 ERR_TRIAXIS_MALFUNCTION: Malfunction
 *                 ERR_TRIAXIS_DIAGNOSTIC_INFO: Diagnostics Info
 * **************************************************************************** *
 * \details Get Triaxis absolute position as an angle between 0x0000 and
 *          0xFFFF (360 degrees). The angle can be either alpha or based on X-Y-Z
 * *************************************************************************** *
 * - Call Hierarchy: main_Init(), main()
 * - Cyclomatic Complexity: 2+1
 * - Nesting: 1
 * - Function calling: 1 (Triaxis_SendCmd())
 * *************************************************************************** */
uint16_t Triaxis_GetAbsPos(uint16_t u16Truncate)                                /* MMP170117-1 */
{
    uint16_t u16Result;

    if ( (u16Result = Triaxis_SendCmd(CMD_TRIAXIS_ALPHA)) == ERR_TRIAXIS_OK)
    {
        if (g_u16TriaxisAlpha != 0xFFFFU)
        {
            l_u16TriaxisAngleRaw = (g_u16TriaxisAlpha << 2);                    /* Convert 14-bit Alpha to 16-bit Absolute Position */
            if (l_u8TriaxisDir != FALSE)
            {
                /* Triaxis has opposite rotational direction as actuator */
                g_u16ShaftAngle = l_u16TriaxisAngleOffset - l_u16TriaxisAngleRaw;
            }
            else
            {
                /* Triaxis has same rotational direction as actuator */
                g_u16ShaftAngle = l_u16TriaxisAngleRaw - l_u16TriaxisAngleOffset;
            }
            g_u16TriaxisActualPos = g_u16ShaftAngle;
        }
        else
        {
            u16Result = ERR_TRIAXIS_NODATA;
        }
        (void)u16Truncate;
    }

    return (u16Result);

} /* End of Triaxis_GetAbsPos() */

/*!*************************************************************************** *
 * Triaxis_Init
 * \brief   Initialise SPI interface and Triaxis MLX90363
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  (uint16_t) result
 *                 ERR_TRIAXIS_OK: Okay
 *                 ERR_TRIAXIS_INVLEN: Incorrect BitCount
 *                 ERR_TRIAXIS_NODATA: Answer = NTT message;
 *                      Two reasons: Answer Time-Out or Answer not Ready
 *                 ERR_TRIAXIS_INVCMD: OPCODE not valid
 *                 ERR_TRIAXIS_CRC: Checksum error
 *                 ERR_TRIAXIS_NODATA: Nothing to Transmit (NTT)
 *                 ERR_TRIAXIS_UNKNOWN: Unknown
 *                 ERR_TRIAXIS_INVREPLY: Invalid Reply
 *                 ERR_TRIAXIS_MALFUNCTION: Malfunction
 *                 ERR_TRIAXIS_DIAGNOSTIC_INFO: Diagnostics Info
 * *************************************************************************** *
 * \details The Triaxis MLX90363 is connected between 3.3V and ground.
 *          The SPI communication is based on SPI.
 * *************************************************************************** *
 * - Call Hierarchy: AppInit()
 * - Cyclomatic Complexity: 7+1
 * - Nesting: 5
 * - Function calling: 1 (Triaxis_SendCmd())
 * *************************************************************************** */
uint16_t Triaxis_Init(void)
{
    uint16_t u16Result;
    uint16_t u16Retry;

    l_u16TriaxisAngleOffset = (int16_t)NV_TRIAXIS_ANGLE_OFF;
    l_u8SenseMagnetPolePairs = NV_SENSE_POLE_PAIRS;
    l_u8TriaxisDir = NV_TRIAXIS_DIRECTION;

    /* noinit RAM-section initialisation */
    if ( ((IO_PORT_MISC_IN & B_PORT_MISC_IN_RSTAT) == 0U) ||
         (bistResetInfo != C_CHIP_STATE_CMD_RESET) )
    {
        g_u16TriaxisActualPos = 0U;
        g_u16TriaxisTargetPos = 0U;
    }

    (void)InitSpiModule(8U);

    /* Add POR-delay, up to 10 x 1ms */
    u16Retry = 10U;
    do
    {
        p_AwdAck();                                                             /* Acknowledge Analogue Watchdog .. */
#if (_SUPPORT_DWD != FALSE)
        WDG_conditionalIwdRefresh(C_IWD_DIV, C_IWD_TO);                         /* .. acknowledge the digital watchdog */
#endif /* (_SUPPORT_DWD != FALSE) */
        DELAY(C_DELAY_1MS);
        u16Retry--;
    } while (u16Retry != 0U);

    u16Retry = 2U * DELAY_tStartUp;
    do
    {
        p_AwdAck();                                                             /* Acknowledge Analogue Watchdog .. */
#if (_SUPPORT_DWD != FALSE)
        WDG_conditionalIwdRefresh(C_IWD_DIV, C_IWD_TO);                         /* .. acknowledge the digital watchdog */
#endif /* (_SUPPORT_DWD != FALSE) */
        DELAY(C_DELAY_1MS);                                                     /* Add 1ms delay */
        u16Result = Triaxis_SendCmd(CMD_TRIAXIS_NOP);
        if (u16Result == ERR_TRIAXIS_OK)
        {
            /* Received response */
            if ((l_u8Type == C_ML90427_TYPE_RESULT_STATUS) && (l_u8State == C_MLX90427_STATE_NORMAL))
            {
                /* Triaxis is 'Normal' state (Start-up finished) */
                break;
            }
        }
        u16Retry--;
    } while (u16Retry != 0U);
    if (u16Retry == 0U)
    {
        u16Result = ERR_TRIAXIS_NODATA;                                         /* No response */
    }
    return (u16Result);
} /* End of Triaxis_Init() */

/*!*************************************************************************** *
 * Adapt_TriaxisAngleOffset
 * \brief   Adapt the Triaxis Angle Offset
 * \author  mmp
 * *************************************************************************** *
 * \param   u16Offset: Offset correction
 * \return  -
 * *************************************************************************** *
 * \details Correct Triaxis Angle Offset with correction-value
 * *************************************************************************** *
 * - Call Hierarchy: main_Init()
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 0
 * *************************************************************************** */
void Adapt_TriaxisAngleOffset(uint16_t u16Offset)
{
    l_u16TriaxisAngleOffset -= u16Offset;
} /* End of Adapt_TriaxisAngleOffset() */

#if (LIN_COMM != FALSE)
/*!*************************************************************************** *
 * HandleTriaxisStatus
 * \brief   Reply LIN response
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details
 * *************************************************************************** *
 * - Call Hierarchy: -
 * - Cyclomatic Complexity: 2+1
 * - Nesting: 2
 * - Function calling: 2 (ml_DataReady(), SetLastError())
 * *************************************************************************** */
void HandleTriaxisStatus(void)
{
#if defined (__MLX81330A01__) || defined (__MLX81332A01__)
    uint16_t *pu16LinResponse = (uint16_t *)(void *)LinFrameDataBuffer;
#else
    uint16_t *pu16LinResponse = (uint16_t *)((void *)ML_DATA_LIN_FRAME_DATA_BUFFER);
#endif
    pu16LinResponse[0] = l_u16TriaxisAngleRaw;
    pu16LinResponse[1] = l_u16TriaxisAngleOffset;
    pu16LinResponse[2] = g_u16TriaxisActualPos;
    pu16LinResponse[3] = g_u16TriaxisTargetPos; /*g_u16ActualPosition; */

    if (COLIN_LINstatus.buffer_used == 0U)                                      /* MMP240408-1 */
    {
        if (ml_DataReady(ML_END_OF_TX_ENABLED) != ML_SUCCESS)                   /* Request end-of-transmission Event */
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
} /* End of HandleTriaxisStatus() */
#endif /* (LIN_COMM != FALSE) */

#endif /* (_SUPPORT_TRIAXIS_MLX90427 != FALSE) */

/* EOF */


