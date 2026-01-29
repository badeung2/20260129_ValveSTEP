/*!*************************************************************************** *
 * \file        LIN_Communication.c
 * \brief       MLX813xx LIN communication handling
 *
 * \note        project MLX813xx
 *
 * \author      Marcel Braat
 *
 * \date        2017-08-15
 *
 * \version     2.0
 *
 * A list of functions:
 *  - Global Functions:
 *           -# LIN_Init()
 *           -# LIN_Stop()
 *           -# mlu_ApplicationStop()
 *           -# mlu_DataRequest()
 *           -# mlu_DataTransmitted()
 *           -# mlu_ErrorDetected()
 *           -# mlu_HeaderReceived()
 *           -# mlu_LinSleepMode()
 *           -# mlu_MessageReceived()
 *           -# HandleLinInMsg()
 *           -# COLIN_IT
 *           -@ p_ml_GetBaudRate()
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
 * *************************************************************************** *
 *
 *  Address Name            Description
 * MLX4-RAM +-----------+   A LIN-frame buffer of 8-bytes, used to communicate between MLX4 and MLX16
 *          |  LinFrame |   Buffer used for IN and OUT frames. Buffer use semaphores.
 *          +-----------+
 *             /|\  |
 *              |   |
 *  ml_DataReady|   |ml_GetLinEventData
 *              |   |
 *              |  \|/
 *  0x00DP  +-----------+   The LIN-frame (IN) is copied into this LinFrameDataBuffer (pLinInFrameBuffer), incase of a LinCommand.
 *          |  LinFrame |   After copy, LinFrame buffer is released (free). The copy is taken place during the LIN-IRQ (ml_GetLinEventData).
 *          | DataBuffer|   The LIN-frame (OUT) is written in this buffer; When finished ml_DataReady() is called to copied to LinFrame
 *          +-----------+
 *             /|\  |
 *              |   |
 *  Application-+   |mlu_MessageReceived
 * e.g. ActStatus   |
 *                 \|/
 *  0x00DP  +-----------+
 *          | CopyLinIn |   MLX16 Application LIN-command frame buffer (IN). The LinFrameDataBuffer (pLinInFrameBuffer)
 *          |FrameBuffer|   is copied to g_LinCmdFrameBuffer in case it is FREE (empty) (mlu_MessageReceived).
 *          +-----------+   The LIN-command is handled by calling HandleLinInMsg().
 *
 *                          LIN 2.x/Actuator (4.4) Status requests are written directly in the
 *                          LinFrameDataBuffer (pLinOutFrameBuffer) after receiving the LIN-Header.
 *                          LIN 1.3/Cooling (2.3) Request Frames are also directly written in the
 *                          LinFrameDataBuffer (pLinOutFrameBuffer) after receiving the Demand frame.
 *  0x00DP  +------------+  LIN 2.x Diagnostics response frames (0x3D) can be requested multiple times, without 0x3C-frames.
 *          |g_Diag      |  Furthermore, between a 0x3C-frame (Request) and 0x3D-frame (Response) are other status
 *          |   Response |  frames allowed. Therefore a separate 0x3D-Response-buffer is made.
 *          +------------+
 *
 * SW-Layers:
 * +----------------------------+
 * | LIN_2x_AGS/FAN/HVAC/Simple | LIN Message Layer
 * +----------------------------+
 * |     LIN-Communication      | Generic LIN
 * +----------------------------+
 * |           LIN-API          | LIN API
 * +----------------------------+
 * |            MLX4            | LIN co-Processor FW
 * +----------------------------+
 * |         MLX8133x/4x        | Hardware
 * +----------------------------+
 * *************************************************************************** */


/*!*************************************************************************** */
/*                           INCLUDES                                          */
/* *************************************************************************** */
#include "AppBuild.h"                                                           /* Application Build */

#if (LIN_COMM != FALSE)

#if (_SUPPORT_I2C != FALSE) && ((_SUPPORT_I2C_MASTER != FALSE) || (_SUPPORT_I2C_SLAVE != FALSE))
#include "commlib/I2C_Generic.h"                                                /* I2C support */
#endif /* (_SUPPORT_I2C != FALSE) && ((_SUPPORT_I2C_MASTER != FALSE) || (_SUPPORT_I2C_SLAVE != FALSE)) */
#include "commlib/LIN_Communication.h"                                          /* LIN support */
#if ((_SUPPORT_SPI != FALSE) || (_SUPPORT_3WIRE_SPI != FALSE)) && (_SUPPORT_SPI_DMA != FALSE)
#include "commlib/SPI.h"                                                        /* SPI Support */
#endif /* ((_SUPPORT_SPI != FALSE) || (_SUPPORT_3WIRE_SPI != FALSE)) && (_SUPPORT_SPI_DMA != FALSE) */
#if (_SUPPORT_UART != FALSE)
#include "commlib/UART.h"                                                       /* UART support */
#endif /* (_SUPPORT_UART != FALSE) */

#include "drivelib/ADC.h"                                                       /* ADC support */
#include "drivelib/AppFunctions.h"                                              /* Application Functions support */
#include "drivelib/NV_Functions.h"                                              /* NVRAM Functions & Layout */
#include "drivelib/ErrorCodes.h"                                                /* Error-logging support */
#include "drivelib/GlobalVars.h"                                                /* Global Variables support */
#if (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT)
#include "drivelib/MotorDriver.h"                                               /* Motor driver support */
#elif (_SUPPORT_APP_TYPE == C_APP_RELAY)
#include "drivelib/RelayDriver.h"                                               /* Relay driver support */
#include "camculib/private_mathlib.h"                                           /* Private Math Library support */
#elif (_SUPPORT_APP_TYPE == C_APP_SOLENOID)
#include "drivelib/SolenoidDriver.h"                                            /* Solenoid Driver support */
#include "camculib/private_mathlib.h"                                           /* Private Math Library support */
#endif /* (_SUPPORT_APP_TYPE) */
#if (_SUPPORT_TRIAXIS_MLX90363 != FALSE)
#include "senselib/Triaxis_MLX90363.h"                                          /* (SPI) Triaxis support MLX90363 */
#elif (_SUPPORT_TRIAXIS_MLX90367 != FALSE) || (_SUPPORT_TRIAXIS_MLX90372 != FALSE)
#include "senselib/Triaxis_MLX90367_372.h"                                      /* (SENT) Triaxis MLX90367/372 support */
#elif (_SUPPORT_TRIAXIS_MLX90377 != FALSE)
#include "senselib/Triaxis_MLX90377.h"                                          /* (PWM/SENT/SPC) Triaxis MLX90377 support */
#elif (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
#include "senselib/Triaxis_MLX9038x.h"                                          /* (ANA) Resolver support MLX90380/381 */
#elif (_SUPPORT_TRIAXIS_MLX90395 != FALSE)
#include "senselib/Triaxis_MLX90395.h"                                          /* (SPI) Triaxis support MLX90395 */
#elif (_SUPPORT_TRIAXIS_MLX90421 != FALSE) || (_SUPPORT_TRIAXIS_MLX90425 != FALSE)
#include "senselib/Triaxis_MLX90421_425.h"                                      /* (PWM) Triaxis support MLX90421 or MLX90425 */
#elif (_SUPPORT_TRIAXIS_MLX90422 != FALSE) || (_SUPPORT_TRIAXIS_MLX90426 != FALSE)
#include "senselib/Triaxis_MLX90422_426.h"                                      /* (SENT) Triaxis support MLX90422 or MLX90426 */
#elif (_SUPPORT_TRIAXIS_MLX90427 != FALSE)
#include "senselib/Triaxis_MLX90427.h"                                          /* (SPI) Triaxis support MLX90427 */
#elif (_SUPPORT_INDUCTIVE_POS_SENSOR_MLX90513 != FALSE)
#include "senselib/InductivePosSensor_MLX90513.h"                               /* (SENT/SPC/PWM) Inductive Position Sensor MLX90513 */
#elif (_SUPPORT_PRESSURE_MLX90829 != FALSE)
#include "senselib/Pressure_MLX90829.h"                                         /* (SENT) Pressure Sensor MLX90829 support */
#elif (_SUPPORT_HUMIDITY_HDC302x != FALSE)
#include "senselib/Humidity_TI_HDC302x.h"                                       /* (I2C) Humidity TI HDC302x Sensor support */
#endif /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */

#include <itc_helper.h>
#if (_SUPPORT_BOOTLOADER_PPM != FALSE)
#include <bl_tools.h>                                                           /* Used by MLX16_RESET_SIGNED */
#include <bl_bist.h>
#endif /* (_SUPPORT_BOOTLOADER_PPM != FALSE) */

/*!*************************************************************************** */
/*                           GLOBAL & LOCAL VARIABLES                          */
/* *************************************************************************** */
#pragma space dp                                                                /* __TINY_SECTION__ */
uint8_t g_u8BufferOutID = QR_INVALID;                                           /*!< LIN output buffer is invalid */
uint8_t l_u8LinInFrameMsgID = 0xFFU;                                            /*!< LIN input frame message-ID */
LININBUF g_LinCmdFrameBuffer __attribute__((aligned(2)));                       /*!< LIN Command Frame Buffer */
#if ((LINPROT & LINX) == LIN2) || (LINPROT == LIN13_HVACTB)
RFR_DIAG g_DiagResponse __attribute__((aligned(2)));                            /*!< LIN Diagnostics Response Frame */
#endif /* ((LINPROT & LINX) == LIN2) || (LINPROT == LIN13_HVACTB) */
#pragma space none                                                              /* __TINY_SECTION__ */

#pragma space nodp                                                              /* __NEAR_SECTION__ */
volatile uint8_t g_u8LinInFrameBufState = (uint8_t)C_LIN_IN_FREE;               /*!< LIN input frame-buffer status is FREE */
volatile uint8_t g_u8ErrorCommunication = FALSE;                                /*!< Flag indicate of LIN communication errors occurred */
#if (_SUPPORT_BUSTIMEOUT != FALSE)
volatile uint8_t g_u8ErrorCommBusTimeout = FALSE;                               /*!< Flag indicate of LIN bus time-out occurred */
#endif /* (_SUPPORT_BUSTIMEOUT != FALSE) */
uint8_t g_byCommEvent = C_COMM_EVENT_NONE;                                      /*!< Communication Event */
#if ML_HAS_LIN_EVENT_TABLE_IN_RAM != 1
LINEventTable_t lin_table;                                                      /*!< LIN event table */
#endif
#pragma space none                                                              /* __NEAR_SECTION__ */

extern void mlu_AutoAddressingStep(uint8_t StepNumber);
void __attribute__((interrupt)) COLIN_IT(void);

/*!*************************************************************************** *
 * mlu_ApplicationStop
 * \brief   Stop application
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  ml_Status
 * *************************************************************************** *
 * \details LIN API event
 *          Stop the application, including:
 *          * Stopping all DMA's
 *          * Disabling the interrupts
 * *************************************************************************** *
 * - Call Hierarchy: DfrDiagMlxDebug()
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 2 (MotorDriverStop(), SetLastError())
 * *************************************************************************** */
ml_Status_t mlu_ApplicationStop(void)
{
    AppStop();

    return (ML_SUCCESS);                                                        /* Return that the application has stopped */
} /* End of mlu_ApplicationStop */

/*!*************************************************************************** *
 * mlu_DataRequest
 * \brief   Data Request (slave TX)
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] MessageID: LIN Slave to Master Message-ID
 * \return  -
 * *************************************************************************** *
 * \details LIN Frame-ID is translated to Message-ID
 * *************************************************************************** *
 * - Call Hierarchy: LIN_Init()
 * - Cyclomatic Complexity: 7+1
 * - Nesting: 3
 * - Function calling: 6 (ml_DataReady(), ml_DiscardFrame(), HandleFanStatus(),
 *                        ClearLinFrameTimeOut(), ClearMlx4CheckPeriodCount(), SetLastError())
 * *************************************************************************** */
void mlu_DataRequest(ml_MessageID_t MessageID)
{
#if (_DEBUG_LIN_ISR != FALSE)
    DEBUG_SET_IO_A();
#endif /* (_DEBUG_LIN_ISR != FALSE) */

#if (_SUPPORT_BUSTIMEOUT != FALSE)
    g_u8ErrorCommBusTimeout = FALSE;                                            /* Data requested; No longer Bus time-out */
#endif /* (_SUPPORT_BUSTIMEOUT != FALSE) */
#if (_SUPPORT_TNCTOC != FALSE)
    g_e8TNCTOC |= C_TNCTOC_LIN;                                                 /* LIN communication active */
#endif /* (_SUPPORT_TNCTOC != FALSE) */

#if ((LINPROT & LINX) == LIN2) || (LINPROT == LIN13_HVACTB)
    switch (MessageID)
    {
        case mlxRFR_DIAG:
            /* Diagnostic */
            if ( (g_u8BufferOutID == (uint8_t)QR_RFR_DIAG) && (COLIN_LINstatus.buffer_used == 0U) )  /* MMP240408-1 */
            {
                /* Copy g_DiagResponse to LinFrameDataBuffer */
                const RFR_DIAG *pDiag = &g_DiagResponse;
                const uint16_t *src = (uint16_t *)((void *)pDiag);
#if defined (__MLX81330A01__) || defined (__MLX81332A01__)
                uint16_t *dst = (uint16_t *)((void *)&LinFrameDataBuffer[0]);
#else
                uint16_t *dst = (uint16_t *)((void *)ML_DATA_LIN_FRAME_DATA_BUFFER);
#endif
                dst[0] = src[0];
                dst[1] = src[1];
                dst[2] = src[2];
                dst[3] = src[3];

#if (_SUPPORT_BOOTLOADER_PPM != FALSE)
                if (g_e8LoaderReset != (uint8_t)C_LOADER_CMD_NONE)
                {
                    /* Only in case of Loader reset, generate a IRQ for mlu_DataTransmitted() */
                    if (ml_DataReady(ML_END_OF_TX_ENABLED) != ML_SUCCESS)
                    {
                        g_u8ErrorCommunication = TRUE;
#if (_SUPPORT_LOG_ERRORS != FALSE)
                        SetLastError(C_ERR_LIN_API);
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
                    }
                }
                else
#endif /* (_SUPPORT_BOOTLOADER_PPM != FALSE) */
                {
                    if (ml_DataReady(ML_END_OF_TX_DISABLED) != ML_SUCCESS)
                    {
                        g_u8ErrorCommunication = TRUE;
#if (_SUPPORT_LOG_ERRORS != FALSE)
                        SetLastError(C_ERR_LIN_API);
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
                    }
                }
#if ((CAN_COMM != FALSE) || (PWM_COMM != FALSE) || (SPI_COMM != FALSE) || \
    ((_SUPPORT_UART != FALSE) && (_SUPPORT_UART_IF_APP == C_UART_IF_QUAD_WHEEL_CAR)))
                g_u8LinConnected = C_COMM_ACTIVE;
#endif /* ((CAN_COMM != FALSE) || (PWM_COMM != FALSE) || (SPI_COMM != FALSE) || ((_SUPPORT_UART != FALSE) && (_SUPPORT_UART_IF_APP == C_UART_IF_QUAD_WHEEL_CAR))) */
                if ( (g_DiagResponse.byPCI == 0x03U) && (g_DiagResponse.u.SF.byRSID == 0x7FU) &&
                     (g_DiagResponse.u.SF.byD2 == 0x78U) )
                {
                    /* MMP161019-5: Busy-error; Keep response. */
                }
                else
                {
                    g_u8BufferOutID = (uint8_t)QR_INVALID;                      /* Invalidate LIN output buffer */
                }
            }
#if (_SUPPORT_UDS != FALSE)
            else if (g_u8BufferOutID == (uint8_t)QR_RFR_DIAG_MF)
            {
                HandleRfrDiagMF(); /*lint !e416 */
                if (COLIN_LINstatus.buffer_used == 0U)                          /* MMP240408-1 */
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
#if ((CAN_COMM != FALSE) || (PWM_COMM != FALSE) || (SPI_COMM != FALSE) || \
    ((_SUPPORT_UART != FALSE) && (_SUPPORT_UART_IF_APP == C_UART_IF_QUAD_WHEEL_CAR)))
                g_u8LinConnected = C_COMM_ACTIVE;
#endif /* ((CAN_COMM != FALSE) || (PWM_COMM != FALSE) || (SPI_COMM != FALSE) || ((_SUPPORT_UART != FALSE) && (_SUPPORT_UART_IF_APP == C_UART_IF_QUAD_WHEEL_CAR))) */
            }
#endif /* (_SUPPORT_UDS != FALSE) */
            else
            {
                (void)ml_DiscardFrame();                                        /* Output buffer response doesn't match requested response */
            }
            break;
#if (LINPROT == LIN2X_HVAC49) || (LINPROT == LIN2X_HVAC52) || (LINPROT == LIN2X_AIRVENT12) || (LINPROT == LIN22_SIMPLE_PCT) || (LINPROT == LIN2X_AGS)
        case MSG_STATUS:
            HandleActStatus();                                                  /* Handle Actuator Status */
#if ((CAN_COMM != FALSE) || (PWM_COMM != FALSE) || (SPI_COMM != FALSE) || \
    ((_SUPPORT_UART != FALSE) && (_SUPPORT_UART_IF_APP == C_UART_IF_QUAD_WHEEL_CAR)))
            g_u8LinConnected = C_COMM_ACTIVE;
#endif /* ((CAN_COMM != FALSE) || (PWM_COMM != FALSE) || (SPI_COMM != FALSE) || ((_SUPPORT_UART != FALSE) && (_SUPPORT_UART_IF_APP == C_UART_IF_QUAD_WHEEL_CAR)))) */
            break;
#elif (LINPROT == LIN2X_FAN01)
        case MSG_STATUS:
            HandleFanStatus();                                                  /* Handle Fan Status */
#if ((CAN_COMM != FALSE) || (PWM_COMM != FALSE) || (SPI_COMM != FALSE) || \
    ((_SUPPORT_UART != FALSE) && (_SUPPORT_UART_IF_APP == C_UART_IF_QUAD_WHEEL_CAR)))
            g_u8LinConnected = C_COMM_ACTIVE;
#endif /* ((CAN_COMM != FALSE) || (PWM_COMM != FALSE) || (SPI_COMM != FALSE) || ((_SUPPORT_UART != FALSE) && (_SUPPORT_UART_IF_APP == C_UART_IF_QUAD_WHEEL_CAR))) */
            break;
#elif (LINPROT == LIN2X_RELAY)
        case MSG_STATUS:
            HandleRelayStatus();                                                /* Handle Relay Status */
#if ((PWM_COMM != FALSE) || (CAN_COMM != FALSE) || \
    ((_SUPPORT_UART != FALSE) && (_SUPPORT_UART_IF_APP == C_UART_IF_QUAD_WHEEL_CAR)))
            g_u8LinConnected = C_COMM_ACTIVE;
#endif /* ((PWM_COMM != FALSE) || (CAN_COMM != FALSE) || ((_SUPPORT_UART != FALSE) && (_SUPPORT_UART_IF_APP == C_UART_IF_QUAD_WHEEL_CAR))) */
            break;
#elif (LINPROT == LIN2X_SOLENOID)
        case MSG_STATUS:
            HandleSolenoidStatus();                                             /* Handle Solenoid Status */
            break;
#elif (LINPROT == LIN13_HVACTB)
        case MSG_STATUS:
            HandleActStatus();                                                  /* Handle HVAC Status (LIN 1.3) */
            break;
#endif /* (LINPROT) */
#if (LINPROT == LIN2X_AIRVENT12)
        case MSG_STATUS2:
            HandleActStatus2();                                                 /* Handle Actuator Status */
            break;
#endif /* (LINPROT == LIN2X_AIRVENT12) */
#if defined (MSG_SENSOR)
#if (_SUPPORT_TRIAXIS_MLX90363 != FALSE) || (_SUPPORT_TRIAXIS_MLX90367 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90372 != FALSE) || (_SUPPORT_TRIAXIS_MLX90377 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX9038x != FALSE) || (_SUPPORT_TRIAXIS_MLX90395 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90421 != FALSE) || (_SUPPORT_TRIAXIS_MLX90422 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90425 != FALSE) || (_SUPPORT_TRIAXIS_MLX90426 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90427 != FALSE)
        case MSG_SENSOR:
            HandleTriaxisStatus();                                              /* Triaxis Sensor */
            break;
#elif (_SUPPORT_INDUCTIVE_POS_SENSOR_MLX90513 != FALSE)
        case MSG_SENSOR:
            HandleInductivePosSensorStatus();                                   /* Inductive Position Sensor */
            break;
#elif (_SUPPORT_PRESSURE_MLX90829 != FALSE)
        case MSG_SENSOR:
            HandlePressureStatus();                                             /* Pressure Sensor */
            break;
#elif (_SUPPORT_HUMIDITY_HDC302x != FALSE)
        case MSG_SENSOR:
            HandleHumidityStatus();                                             /* Humidity Sensor */
            break;
#endif /* (_SUPPORT_PRESSURE_MLX90829 != FALSE) */
#endif /* defined (MSG_SENSOR) */
#if (_SUPPORT_MLX_CHIP_STATUS != FALSE)
        case MSG_MLX_CHIP_STATUS:
            HandleMlxChipStatus();
            break;
#endif /* (_SUPPORT_MLX_CHIP_STATUS != FALSE) */
        default:
            (void)ml_DiscardFrame();                                            /* Output buffer response doesn't match requested response */
    }
#endif /* ((LINPROT & LINX) == LIN2) */

#if (_SUPPORT_LIN_AA != FALSE) && (LIN_AA_LINFRAME_TIMEOUT != FALSE)
    ClearLinFrameTimeOut();
#endif /* (_SUPPORT_LIN_AA != FALSE) && (LIN_AA_LINFRAME_TIMEOUT != FALSE) */

#if (_SUPPORT_LIN_BUS_ACTIVITY_CHECK != FALSE)
    ClearMlx4CheckPeriodCount();
#endif /* (_SUPPORT_LIN_BUS_ACTIVITY_CHECK != FALSE) */

#if (_DEBUG_LIN_ISR != FALSE)
    DEBUG_CLR_IO_A();
#endif /* (_DEBUG_LIN_ISR != FALSE) */
} /* End of mlu_DataRequest */

/*!*************************************************************************** *
 * mlu_DataTransmitted
 * \brief   LIN-Event when data have been transmitted successfully
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] Index
 * \return  -
 * *************************************************************************** *
 * \details LIN API event
 * *************************************************************************** *
 * - Call Hierarchy: LIN_Init()
 * - Cyclomatic Complexity: 4+1
 * - Nesting: 2
 * - Function calling: 1 (MLX16_RESET_SIGNED())
 * *************************************************************************** */
void mlu_DataTransmitted(ml_MessageID_t Index)
{
#if (_SUPPORT_BOOTLOADER_PPM != FALSE)
    if (g_e8LoaderReset != (uint8_t)C_LOADER_CMD_NONE)
    {
        uint16_t u16MiscOut = IO_PORT_MISC_OUT;
        IO_PORT_MISC_OUT = (u16MiscOut & ~B_PORT_MISC_OUT_SET_RSTAT) | B_PORT_MISC_OUT_CLEAR_RSTAT; /* Invalidate the RAM */
        IO_PORT_MISC_OUT = u16MiscOut;
        if (g_e8LoaderReset == (uint8_t)C_LOADER_CMD_EPM)
        {
            g_e8LoaderReset = (uint8_t)C_LOADER_CMD_NONE;                       /* MMP190116-1 */
            MLX16_RESET_SIGNED( (BistResetInfo_t)C_CHIP_STATE_CMD_EPM);
        }
        else
        {
            g_e8LoaderReset = (uint8_t)C_LOADER_CMD_NONE;                       /* MMP190116-1 */
            MLX16_RESET_SIGNED( (BistResetInfo_t)C_CHIP_STATE_CMD_RESET);
        }
    }
#endif /* (_SUPPORT_BOOTLOADER_PPM != FALSE) */
#if (LINPROT == LIN2X_HVAC49) || (LINPROT == LIN2X_HVAC52) || (LINPROT == LIN2X_AIRVENT12) || (LINPROT == LIN22_SIMPLE_PCT) || (LINPROT == LIN2X_AGS)
    HandleDataTransmitted(Index);  /*lint !e522 */
#else  /* (LINPROT) */
    if (g_byCommEvent != C_COMM_EVENT_NONE)
    {
        if ( (g_byCommEvent & C_COMM_EVENT_CHIPRESET) != 0U)
        {
            g_u8ChipResetOcc = FALSE;
        }
        if ( (g_byCommEvent & C_COMM_EVENT_LINERROR) != 0U)                     /* MMP161216-4 */
        {
            g_u8ErrorCommunication = FALSE;                                     /* Data requested; No longer communication error */
        }
#if FALSE
        if ( (g_byCommEvent & C_COMM_EVENT_STALL) != 0U)                        /* MMP161014-1 */
        {
            g_u8StallOcc = FALSE;
        }
        if ( (g_byCommEvent & C_COMM_EVENT_EMRUN) != 0U)                        /* MMP161014-2 */
        {
            g_e8EmergencyRunOcc = (uint8_t)C_SAFETY_RUN_NO;
        }
#endif
        g_byCommEvent = C_COMM_EVENT_NONE;
    }
    (void)Index;
#endif /* (LINPROT) */
} /* End of mlu_DataTransmitted() */

/*!*************************************************************************** *
 * mlu_ErrorDetected
 * \brief   LIN Communication error detected
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] Error: LIN Communication error-code
 * \return  -
 * *************************************************************************** *
 * \details LIN API Event
 * *************************************************************************** *
 * - Call Hierarchy: LIN_Init()
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 1 (LIN2x_ErrorHandling())
 * *************************************************************************** */
void mlu_ErrorDetected(ml_LinError_t Error)
{
#if (_DEBUG_LIN_ISR != FALSE)
    DEBUG_SET_IO_B();
#endif /* (_DEBUG_LIN_ISR != FALSE) */

#if (_SUPPORT_BOOTLOADER_PPM != FALSE)
    g_e8LoaderReset = (uint8_t)C_LOADER_CMD_NONE;                               /* Ignore Loader reset in case of LIN-error */
#endif /* (_SUPPORT_BOOTLOADER_PPM != FALSE) */

#if (_SUPPORT_LOG_LIN_ERRORS != FALSE)
    SetLastError(C_ERR_LIN_COMM | ((uint8_t)Error & 0x1F));
#endif /* (_SUPPORT_LOG_LIN_ERRORS != FALSE) */
    if (Error == ml_erLinModuleReset)                                           /* MMP220805-1 */
    {
        uint8_t u8SubCode = (uint8_t)((ML_DATA_LIN_MESSAGE >> 8U) & 0x000FU);
        if (u8SubCode == (uint8_t)erCRASHTX)
        {
            /* Propagation delay error : TX/RX propagation can not
             * be calculated by LIN module due to collision at start bit
             * of the own response. Both dominant and recessive collisions
             * could be the reason of this error
             */

            /* For safety and clarity (LIN module is already in disconnected state) */
            (void)ml_Disconnect();
            (void)ml_Connect();

            /* Map this error to "collision error" for application */
            Error = ml_erBit;
        }
    }
#if (LINPROT == LIN22_SIMPLE_PCT) || (LINPROT == LIN2X_HVAC49) || (LINPROT == LIN2X_HVAC52) || (LINPROT == LIN2X_AGS) || \
    (LINPROT == LIN2X_FAN01) || (LINPROT == LIN2X_RELAY) || (LINPROT == LIN13_HVACTB)
    HandleLinError(Error);
#else   /* (LINPROT) */
    switch (Error)
    {
        case ml_erLinModuleReset:                                               /* erCRASH : LIN task crashed, trying to reboot              */
        case ml_erTimeOutResponse:                                              /* erTORESP: Response Timeout                                */
        case ml_erBreakDetected:                                                /* erBRFRM : Break in frame                                  */
            break;

        case ml_erIdParity:                                                     /* erIDPAR : Parity error in ID field received               */
            g_u8ErrorCommunication = TRUE;                                      /* Set the communication error flag */
            break;

        case ml_erCheckSum:                                                     /* erCKSUM : Checksum error in message received              */
            g_u8ErrorCommunication = TRUE;                                      /* Set the communication error flag */
            break;

        case ml_erStopBitTX:
        case ml_erDataFraming:                                                  /* erSTOP : Stop bit error (regular data byte or checksum)   */

        case ml_erBit:                                                          /* erTXCOL : Data collision during the transmit cycle        */

        case ml_erIdFraming:                                                    /* erIDSTOP : Stop bit error of the ID field                 */
        case ml_erShort:                                                        /* erSHORT : Short detected on the LIN bus                   */
            g_u8ErrorCommunication = TRUE;                                      /* Set the communication error flag */
            break;

        case ml_erSynchField:                                                   /* erSYNC : Sync field timing error                          */
            break;

        case ml_erBufferLocked:                                                 /* erRXOVR : Message received but buffer was full            */
            break;

        case ml_erShortDone:                                                    /* erSHORTDONE : A short state has been seen on the bus, but now recovered */
        case ml_erWakeUpInit:                                                   /* erWKUPINIT : A valid wake-up pulse has been seen while waiting to enter in sleep state */
        default:                                                                /* unrecognized error                                        */
            break;
    }
#endif /* (LINPROT) */
#if (_SUPPORT_LIN_AA != FALSE) && (LIN_AA_LINFRAME_TIMEOUT != FALSE)
    ClearLinFrameTimeOut();
#endif /* (_SUPPORT_LIN_AA != FALSE) && (LIN_AA_LINFRAME_TIMEOUT != FALSE) */
#if (_DEBUG_LIN_ISR != FALSE)
    DEBUG_CLR_IO_B();
#endif /* (_DEBUG_LIN_ISR != FALSE) */
} /* End of mlu_ErrorDetected() */

#if defined (USE_LIN_FRAME_MODE)
/*!*************************************************************************** *
 * mlu_HeaderReceived
 * \brief   LIN event when LIN header have been received (M2S and S2M)
 * \author  mmp
 * *************************************************************************** *
 * \param   ml_MessageID MessageID
 * \return  -
 * *************************************************************************** *
 * \details LIN API event
 * *************************************************************************** *
 * - Call Hierarchy: -
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 1 (ml_DiscardFrame())
 * *************************************************************************** */
void mlu_HeaderReceived(ml_MessageID MessageID)
{
    (void)MessageID;        /* unused parameter */

    (void)ml_DiscardFrame();
} /* End of mlu_HeaderReceived() */
#endif /* defined (USE_LIN_FRAME_MODE) */

/*!*************************************************************************** *
 * mlu_LinSleepMode
 * \brief   LIN event to enter sleep mode
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] Reason
 * \return  -
 * *************************************************************************** *
 * \details LIN API event
 * *************************************************************************** *
 * - Call Hierarchy: LIN_Init()
 * - Cyclomatic Complexity: 2+1
 * - Nesting: 1
 * - Function calling: 1 (HandleBusTimeout())
 * *************************************************************************** */
void mlu_LinSleepMode(ml_SleepReason_t Reason)
{
    /*
     * MLX4 FW handles Goto Sleep frame (0x3C, 0x00 ...) automatically
     * and does not report it via mlu_MessageReceived event.
     */
#if (_SUPPORT_LIN_SLEEP != FALSE)
    if ( (Reason == ml_reasonMaster) || (Reason == ml_reasonCommand) )
    {
#if (_SUPPORT_SLEEP_EMRUN == FALSE)
        g_e8MotorRequest = (uint8_t)C_MOTOR_REQUEST_SLEEP;                      /* Enter sleep mode */
#else  /* (_SUPPORT_SLEEP_EMRUN == FALSE) */
        g_e8MotorRequest = (uint8_t)C_MOTOR_REQUEST_EMRUN;                      /* First move actuator to secure position before entering sleep */
#endif  /* (_SUPPORT_SLEEP_EMRUN == FALSE) */
    }
#endif /* (_SUPPORT_LIN_SLEEP != FALSE) */

#if (_SUPPORT_BUSTIMEOUT != FALSE)
    if ( (Reason == ml_reasonTimeOut) || (Reason == ml_reasonTimeOutDominant) )
    {
        /*
         * LIN bus was inactive for 4 seconds without receiving an explicit
         * "Go-to-Sleep frame". This can be considered as a failure of the Master or
         * PHY layer. Slave can enter limp-home mode.
         */
#if (_DEBUG_LIN_BUS_TO != FALSE)
        DEBUG_SET_IO_A();
#endif /* (_DEBUG_LIN_BUS_TO != FALSE) */
        HandleBusTimeout();
#if (_DEBUG_LIN_BUS_TO != FALSE)
        DEBUG_CLR_IO_A();
#endif /* (_DEBUG_LIN_BUS_TO != FALSE) */
    }
#endif /* (_SUPPORT_BUSTIMEOUT != FALSE) */

#if (_SUPPORT_LIN_SLEEP == FALSE) && (_SUPPORT_BUSTIMEOUT == FALSE)
    (void)Reason;
#endif /* (_SUPPORT_LIN_SLEEP == FALSE) && (_SUPPORT_BUSTIMEOUT == FALSE) */
} /* End of mlu_LinSleepMode() */

/*!*************************************************************************** *
 * mlu_MessageReceived
 * \brief   MessageReceived (slave RX)
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] byMessageID
 * \return  -
 * *************************************************************************** *
 * \details LIN API event
 * *************************************************************************** *
 * - Call Hierarchy: LIN_Init()
 * - Cyclomatic Complexity: 1+1
 * - Nesting: 1
 * - Function calling: 2 (ClearLinFrameTimeOut(), ClearMlx4CheckPeriodCount())
 * *************************************************************************** */
void mlu_MessageReceived(ml_MessageID_t byMessageID)
{
#if (_DEBUG_LIN_ISR != FALSE)
    DEBUG_SET_IO_A();
#endif /* (_DEBUG_LIN_ISR != FALSE) */

    if (g_u8LinInFrameBufState != (uint8_t)C_LIN_IN_FULL)
    {
        /* Buffer is either empty or message is postpone (overwrite allowed) */
        l_u8LinInFrameMsgID = byMessageID;

        /* LIN In-frame buffer to a Copy LIN In-frame buffer */
        {
#if defined (__MLX81330A01__) || defined (__MLX81332A01__)
            const uint16_t *pu16Source = (uint16_t *)((void *)&LinFrameDataBuffer[0]);
#else
            const uint16_t *pu16Source = (uint16_t *)((void *)ML_DATA_LIN_FRAME_DATA_BUFFER);
#endif
            uint16_t *pu16Target = (uint16_t *)&g_LinCmdFrameBuffer;
            pu16Target[0] = pu16Source[0];
            pu16Target[1] = pu16Source[1];
            pu16Target[2] = pu16Source[2];
            pu16Target[3] = pu16Source[3];
        }
        g_u8LinInFrameBufState = (uint8_t)C_LIN_IN_FULL;
#if (_SUPPORT_BUSTIMEOUT != FALSE)
        g_u8ErrorCommBusTimeout = FALSE;                                        /* Frame received; No longer Bus time-out */
#endif /* (_SUPPORT_BUSTIMEOUT != FALSE) */
#if ((CAN_COMM != FALSE) || (PWM_COMM != FALSE) || (SPI_COMM != FALSE) || \
    ((_SUPPORT_UART != FALSE) && (_SUPPORT_UART_IF_APP == C_UART_IF_QUAD_WHEEL_CAR)))
        g_u8LinConnected = C_COMM_ACTIVE;
#endif /* ((CAN_COMM != FALSE) || (PWM_COMM != FALSE) || (SPI_COMM != FALSE) || ((_SUPPORT_UART != FALSE) && (_SUPPORT_UART_IF_APP == C_UART_IF_QUAD_WHEEL_CAR))) */
#if defined (__MLX81330A01__) || defined (__MLX81332A01__)
        LinFrameDataBuffer[0] = 0x00U;                                          /* Clear NAD address */
#else
        LinFrame[0] = 0x00U;                                                    /* Clear NAD address (MMP191207-1) */
#endif
#if (_SUPPORT_MLX_CHIP_STATUS != FALSE)
        g_u8MlxChipStatusM2SCounter++;
#endif /* (_SUPPORT_MLX_CHIP_STATUS != FALSE) */
    }

#if (_SUPPORT_LIN_AA != FALSE) && (LIN_AA_LINFRAME_TIMEOUT != FALSE)
    ClearLinFrameTimeOut();
#endif /* (_SUPPORT_LIN_AA != FALSE) && (LIN_AA_LINFRAME_TIMEOUT != FALSE) */

#if (_SUPPORT_LIN_BUS_ACTIVITY_CHECK != FALSE)
    ClearMlx4CheckPeriodCount();
#endif /* (_SUPPORT_LIN_BUS_ACTIVITY_CHECK != FALSE) */

#if (_DEBUG_LIN_ISR != FALSE)
    DEBUG_CLR_IO_A();
#endif /* (_DEBUG_LIN_ISR != FALSE) */
} /* End of mlu_MessageReceived() */

/*!*************************************************************************** *
 * HandleLinInMsg
 * \brief   MessageReceived (slave RX)
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details Handle LIN Message (Control and/or Demand frames)
 * NOTE: As this function is called by main loop, it is assumed that no Non Volatile Memory
 *       Write is busy.
 * *************************************************************************** *
 * - Call Hierarchy: main()
 * - Cyclomatic Complexity: 4+1
 * - Nesting: 2
 * - Function calling: 2 (HandleDfrDiag(), HandleFanCtrl())
 * *************************************************************************** */
void HandleLinInMsg(void)
{
#if (_DEBUG_LIN_ISR != FALSE)
    DEBUG_SET_IO_A();
    DEBUG_SET_IO_B();
#endif /* (_DEBUG_LIN_ISR != FALSE) */
    if (g_u8LinInFrameBufState == (uint8_t)C_LIN_IN_POSTPONE)
    {
        /* Last message postponed; Try again (without overwritten by LIN message ISR */
        g_u8LinInFrameBufState = (uint8_t)C_LIN_IN_FULL;
    }

#if ((LINPROT & LINX) == LIN2) || (LINPROT == LIN13_HVACTB)                     /* LIN 2.x */
    if (l_u8LinInFrameMsgID == (uint8_t)mlxDFR_DIAG)
    {
        /* Diagnostic */
        HandleDfrDiag();
    }
#if (LINPROT == LIN2X_HVAC52) && (_SUPPORT_HVAC_GROUP_ADDRESS != FALSE)
    else if (l_u8LinInFrameMsgID == (uint8_t)MSG_CONTROL)
    {
        /* Control */
        HandleActCtrl(FALSE);
    }
    else if ( (l_u8LinInFrameMsgID == (uint8_t)MSG_GROUP_CONTROL) &&
              ((((ENH_LIN_PARAMS_t *)ADDR_NV_ENH_LIN_1)->u16FunctionID & 0x8000U) != 0x0000U) )   /* Check EE Function-ID is Group Function-ID */
    {
        /* Group Control */
        HandleActCtrl(TRUE);
    }
#else  /* (LINPROT == LIN2X_HVAC52) && (_SUPPORT_HVAC_GROUP_ADDRESS != FALSE) */
    else if (l_u8LinInFrameMsgID == (uint8_t)MSG_CONTROL)
    {
        /* Control */
#if (LINPROT == LIN2X_FAN01)
        HandleFanCtrl();
#elif (LINPROT == LIN2X_RELAY)
        HandleRelayCtrl();
#elif (LINPROT == LIN2X_SOLENOID)
        HandleSolenoidCtrl();                                                   /* Handle Solenoid Control */
#elif (LINPROT == LIN13_HVACTB)
        HandleActCtrl();
#else  /* (LINPROT == LIN2X_FAN01) */
        /* HVAC, AGS, SIMPLE_PCT */
        HandleActCtrl();
#endif /* (LINPROT == LIN2X_FAN01) */
    }
#endif /* (LINPROT == LIN2X_HVAC52) && (_SUPPORT_HVAC_GROUP_ADDRESS != FALSE) */
#if (LINPROT == LIN22_SIMPLE_PCT) && (_SUPPORT_SPECIAL_COMM_FIELD != FALSE)
    else if (l_u8LinInFrameMsgID == MSG_CONTROL_SPECIAL)
    {
        HandleActCtrlSpecial();
    }
#endif /* _SUPPORT_SPECIAL_COMM_FIELD */
#if (_SUPPORT_MLX_CHIP_STATUS != FALSE)
    else if (l_u8LinInFrameMsgID == (uint8_t)MSG_MLX_CHIP_CONTROL)
    {
        HandleMlxChipControl();
    }
#endif /* (_SUPPORT_MLX_CHIP_STATUS != FALSE) */
    else
    {
        /* Nothing */
    }
#endif /* ((LINPROT & LINX) == LIN2) */

    if (g_u8LinInFrameBufState != (uint8_t)C_LIN_IN_POSTPONE)
    {
        /* LIN Message is handled; Release LIN message buffer */
        g_u8LinInFrameBufState = (uint8_t)C_LIN_IN_FREE;
    }

#if (_DEBUG_LIN_ISR != FALSE)
    DEBUG_CLR_IO_B();
    DEBUG_CLR_IO_A();
#endif /* (_DEBUG_LIN_ISR != FALSE) */
} /* End of HandleLinInMsg() */

/*!*************************************************************************** *
 * LIN_Init
 * \brief   Initialise LIN communication interface.
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details Default start-up, at 19200 Baud
 * *************************************************************************** *
 * - Call Hierarchy: AppCheckLinProc(), main_Init()
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 7 (ml_InitLinModule(), ml_SetAutoBaudRateMode(),
 *                        ml_SetOptions(), ml_SetSleepTo(), ml_SetSlewRate(),
 *                        LIN_2x_Init(), ml_Connect())
 * *************************************************************************** */
void LIN_Init(void)
{
    /* Initialise LIN event tables */
    plinEventTable->mlu_MessageReceived = mlu_MessageReceived;
    plinEventTable->mlu_DataRequest = mlu_DataRequest;
    plinEventTable->mlu_ErrorDetected = mlu_ErrorDetected;
    plinEventTable->mlu_DataTransmitted = mlu_DataTransmitted;
    plinEventTable->mlu_LinSleepMode = mlu_LinSleepMode;
    plinEventTable->mlu_AutoAddressingStep = mlu_AutoAddressingStep;

    /* IO_SET( COLIN, SPEED, 1U); */                                            /* FREQ divided by 2 */
    /* IO_COLIN_CFG = (IO_COLIN_CFG & ~M_COLIN_SPEED) | LIN_CLK_DIV; */         /* FREQ divided by 2 */

    /* Clear (initialise) LIN Baudrate detection variables (MMP221129-1) */
    p_MemSet((void *)&ml_Data, 0x00U, sizeof(ml_Data));
    /* Private MLX4 RAM members */
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81350__)
    *((uint8_t *) 0x0E01U) = 0x00U;                                             /* Last received LIN Message Baudrate divisor */
    *((uint8_t *) 0x0E02U) = 0x00U;                                             /* Auto-baudrate LIN Message Baudrate divisor */
    *((uint8_t *) 0x0E03U) = 0x00U;                                             /* Auto-baudrate/Last received LIN Message Baudrate pre-scaler */
#else
    *((uint8_t *) 0x0A01U) = 0x00U;                                             /* Last received LIN Message Baudrate divisor */
    *((uint8_t *) 0x0A02U) = 0x00U;                                             /* Auto-baudrate LIN Message Baudrate divisor */
    *((uint8_t *) 0x0A03U) = 0x00U;                                             /* Auto-baudrate/Last received LIN Message Baudrate pre-scaler */
#endif

    /* Initialise LIN Communication */
#if (_SUPPORT_APP_USER_MODE != FALSE)
    ENTER_SECTION(SYSTEM_MODE); /*lint !e534 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
    (void)ml_InitLinModule();                                                   /* Start and initialise the LIN Module */
#if (_SUPPORT_APP_USER_MODE != FALSE)
    EXIT_SECTION(); /*lint !e438 !e529 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */

    /* The LIN Module is now in the DISCONNECTED state */
    /* Setup LIN baudrate */
#if (_SUPPORT_AUTO_BAUDRATE != FALSE)
    /* Auto baudrate only on first LIN frame */
    (void)ml_SetAutoBaudRateMode(ML_ABR_ON_FIRST_FRAME);
#else  /* (_SUPPORT_AUTO_BAUDRATE != FALSE) */
    /* Fixed baudrate */
    (void)ml_SetBaudRate( (uint8_t)LIN_BR_PRESCALER, (uint8_t)LIN_BR_DIV);      /* Program the baudrate */
#endif /* (_SUPPORT_AUTO_BAUDRATE != FALSE) */

    (void)ml_SetOptions(0U,                                                     /* IDStopBitLength = 1.5 Bit (Melexis LIN Master has 1.5 Tbit stop bit */
                        0U,                                                     /* TXStopBitLength = 1 Bit */
                        ML_ENABLED,                                             /* StateChangeSignal */
                        ML_LIGHTSLEEP,                                          /* SleepMode: light-sleep mode */
                        ML_VER_2_X);                                            /* LIN version: LIN 2.x */

    (void)ml_SetSleepTo(1U, 13U, 7U);   /* Set LIN Bus-timeout to 4.5sec; 1/250kHz * 2^([1]+9) * (17+[13]) * 256/[7] (MMP180430-1) */

#if ((LIN_BR < 12000) && (_SUPPORT_AUTO_BAUDRATE == FALSE))
    (void)ml_SetSlewRate(ML_SLEWLOW);
#else /* ((LIN_BR < 12000) && (_SUPPORT_AUTO_BAUDRATE == FALSE)) */
    (void)ml_SetSlewRate(ML_SLEWHIGH);
#endif /* ((LIN_BR < 12000) && (_SUPPORT_AUTO_BAUDRATE == FALSE)) */

    /* Initialise communication layer */
#if (LINPROT == LIN2X_HVAC49) || (LINPROT == LIN2X_AIRVENT12) || (LINPROT == LIN2X_HVAC52) || (LINPROT == LIN2X_AIRVENT12) || (LINPROT == LIN2X_AGS) || (LINPROT == LIN2X_FAN01) || \
    (LINPROT == LIN2X_RELAY) || (LINPROT == LIN2X_SOLENOID)
    LIN_2x_Init();
#elif (LINPROT == LIN22_SIMPLE_PCT)
    LIN_22_Init();
#elif (LINPROT == LIN13_HVACTB)
    LIN_13_Init();
#else
#error "Error: LIN_Init() not implemented"
#endif

#if (_SUPPORT_APP_USER_MODE != FALSE)
    ENTER_SECTION(SYSTEM_MODE); /*lint !e534 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
    /* Change COLIN priority to '5' */
    IO_MLX16_ITC_PRIO2_S = (IO_MLX16_ITC_PRIO2_S & ~M_MLX16_ITC_PRIO2_COLIN_LIN) | C_MLX16_ITC_PRIO2_COLIN_LIN_PRIO5;
    /* Itc_Enable( COLIN_LIN); */
    IO_MLX16_ITC_MASK2_S |= B_MLX16_ITC_MASK2_COLIN_LIN;

#if (_SUPPORT_APP_USER_MODE != FALSE)
    EXIT_SECTION(); /*lint !e438 !e529 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */

#if (_SUPPORT_LIN_BUS_ACTIVITY_CHECK != FALSE)
    /* Dynamic Message ID's: 0...7 */
    g_u16MLX4_RAM_Dynamic_CRC1 = p_CalcCRC_U16((const uint16_t *)C_RAM_MLX4_DYNAMIC_BGN_ADR1,
                                               (C_RAM_MLX4_DYNAMIC_END_ADR1 - C_RAM_MLX4_DYNAMIC_BGN_ADR1)/sizeof(uint16_t));
    /* Dynamic Message ID's: 8...15 */
    g_u16MLX4_RAM_Dynamic_CRC2 = p_CalcCRC_U16((const uint16_t *)C_RAM_MLX4_DYNAMIC_BGN_ADR2,
                                               (C_RAM_MLX4_DYNAMIC_END_ADR2 - C_RAM_MLX4_DYNAMIC_BGN_ADR2)/sizeof(uint16_t));
#endif /* (_SUPPORT_LIN_BUS_ACTIVITY_CHECK != FALSE) */
    (void)ml_Connect();
#if (_SUPPORT_BUSTIMEOUT != FALSE)
    g_u8ErrorCommBusTimeout = FALSE;
#endif /* (_SUPPORT_BUSTIMEOUT != FALSE) */
#if ((CAN_COMM != FALSE) || (PWM_COMM != FALSE) || (SPI_COMM != FALSE) || \
    ((_SUPPORT_UART != FALSE) && (_SUPPORT_UART_IF_APP == C_UART_IF_QUAD_WHEEL_CAR)))
    g_u8LinConnected = C_COMM_CONNECTED;
#endif /* ((CAN_COMM != FALSE) || (PWM_COMM != FALSE) || (SPI_COMM != FALSE) || ((_SUPPORT_UART != FALSE) && (_SUPPORT_UART_IF_APP == C_UART_IF_QUAD_WHEEL_CAR))) */
} /* End of LIN_Init() */

#if ((CAN_COMM != FALSE) || ((I2C_COMM != FALSE) && (I2C_SLAVE_PROT != I2C_SLAVE_PROT_NONE) && (C_I2C_SLAVE_SCK_IO == PIN_FUNC_LIN)) || \
     (PWM_COMM != FALSE) || (SPI_COMM != FALSE) || \
     ((_SUPPORT_UART != FALSE) && (_SUPPORT_UART_IF_APP == C_UART_IF_QUAD_WHEEL_CAR)))
/*!*************************************************************************** *
 * LIN_Stop
 * \brief   Stop LIN Communication, and reset MLX4
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details Stop LIN communication (stop MLX4)
 * *************************************************************************** *
 * - Call Hierarchy: main()
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 2 (ml_Disconnect(), MLX4_RESET())
 * *************************************************************************** */
void LIN_Stop(void)
{
    (void)ml_Disconnect();                                                      /* Stop LIN receiving/transmission */
    g_u8LinConnected = C_COMM_DISCONNECTED;
#if (_SUPPORT_APP_USER_MODE != FALSE)
    ENTER_SECTION(SYSTEM_MODE); /*lint !e534 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
    ml_ResetDrv();                                                              /* Stop MLX4 */
#if (_SUPPORT_APP_USER_MODE != FALSE)
    EXIT_SECTION(); /*lint !e438 !e529 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
} /* End of LIN_Stop() */
#endif /* ((CAN_COMM != FALSE) || (I2C_COMM != FALSE) || (PWM_COMM != FALSE) || (SPI_COMM != FALSE) || ((_SUPPORT_UART != FALSE) && (_SUPPORT_UART_IF_APP == C_UART_IF_QUAD_WHEEL_CAR))) */

/*!*************************************************************************** *
 * COLIN_IT
 * \brief   LIN Module (MLX4) interrupt handler
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details This function is called whenever an EVENT interrupt from the LIN
 *          Module (Mlx4) occurs
 *          IRQ-Priority: 5
 * *************************************************************************** *
 * - Call Hierarchy: AppCheckLinProc(), main_Init()
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 2 (ml_GetLinEventData(), ml_ProcessLinEvent())
 * *************************************************************************** */
void __attribute__((interrupt)) COLIN_IT(void);
void COLIN_IT(void)
{
#if (_DEBUG_CPU_LOAD != FALSE)
    DEBUG_SET_IO_C();                                                           /* IRQ-Priority: 5 */
#endif /* (_DEBUG_CPU_LOAD != FALSE) */
#if 1
    /* Optimised for speed */
    ml_GetLinEventData();
#if defined (__MLX81330A01__)
    ml_ProccessLinEvent();
#else
    ml_ProcessLinEvent();
#endif
#else
    /* optimised for size if LIN API is in ROM */
    ml_LinInterruptHandler();
#endif
#if (_DEBUG_CPU_LOAD != FALSE)
    DEBUG_CLR_IO_C();                                                           /* IRQ-Priority: 5 */
#endif /* (_DEBUG_CPU_LOAD != FALSE) */
} /* End of COLIN_IT() */

/*!*************************************************************************** *
 * p_ml_GetBaudRate
 * \brief   Private Get LIN Baudrate
 * \author  mmp
 * *************************************************************************** *
 * \param   MLX4_PLL: MLX4-clock
 * \return  (uint16_t) LIN Baudrate (0 = No baudrate known/error)
 * *************************************************************************** *
 * \details This function is a copy of the platform ml_GetBaudRate() with a fix.
 *          In case no LIN message has been received, it will return '0'.
 * *************************************************************************** *
 * - Call Hierarchy: AppCheckLinProc()
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 0
 * *************************************************************************** */
uint16_t p_ml_GetBaudRate(uint16_t u16Mlx4Clk)
{
    uint16_t u16Result;
    uint16_t u16Rubbish;

    __asm__ __volatile__ (
        "cmp Y, #50000 \n\t"                          /* [ASM] Y = u16Mlx4Clk */
        "jug p_GBR_Error_%= \n\t"                     /* [C]   if (u16Mlx4Clk > 50000) {u16Result = 0;} */
#if defined (__MLX81334__)
        "lod AL, dp:0x0A \n\t"                        /* [C]   u8Divider  [ASM] AL = Baudrate-Divider */
#else
        "lod AL, dp:0x0C \n\t"                        /* [C]   u8Divider  [ASM] AL = Baudrate-Divider */
#endif
        "cmp AL, #99 \n\t"                            /* [C]   if (u8Divider < 99) */
        "juge p_GBR_00_%= \n\t"                       /* [C]   { */
        "p_GBR_Error_%=: \n\t"
        "mov A, #0 \n\t"                              /* [C]     u16Result = 0; */
        "jmp p_GBR_Exit_%= \n\t"                      /* [C]     return; } */
        "p_GBR_00_%=: \n\t"
        "usex A \n\t"                                 /* [C]   u16Divider = (uint16_t)u8Divider; */
        "mov X, A \n\t"
#if defined (__MLX81334__)
        "lod AL, dp:0x0B \n\t"                        /* [C]   u8Divider  [ASM] AL = Baudrate-Divider */
#else
        "lod AL, dp:0x0D \n\t"                        /* [C]   i8PreScaler  [ASM] AL = Pre-scaler */
#endif
        "cmp AL, #8 \n\t"                             /* [C]   if (u8PreScaler > 8) */
        "jug p_GBR_Error_%= \n\t"                     /* [C]   { {u16Result = 0; } */
        "mov Cx, AL \n\t"
        "p_GBR_10_%=: \n\t"                           /* [C]   do */
        "asl X \n\t"                                  /* [C]   { u16Divider *= 2 */
        "djnz Cx, p_GBR_10_%= \n\t"                   /* [C]   } while (--u8PreScaler != 0); */
        "p_GBR_20_%=: \n\t"
        "mov A, Y \n\t"
        "mulu YA, A, #500 \n\t"                       /* [C]   u32Temp = (u16Mlx4Clk * 1000) / 2 */
        "cmp Y, X \n\t"                               /* [C]   if ((u32Temp>>16) >= u16Divider) {u16Result = 0;} */
        "juge p_GBR_Error_%= \n\t"                    /* [ASM] Overflow */
        "divu YA, X \n\t"                             /* [C]   u16Result = u32Temp / u16Divider; */
        "divu YA, X \n\t"
        "p_GBR_Exit_%=:"
        : "=a" (u16Result), "=x" (u16Rubbish), "=y" (u16Rubbish)
        : "y" (u16Mlx4Clk)
        : "D"
        );

    return (u16Result);
} /* End of p_ml_GetBaudRate() */

/*!*************************************************************************** *
 * p_ml_GetLastBaudRate
 * \brief   Private Get Last LIN Baudrate
 * \author  mmp
 * *************************************************************************** *
 * \param   MLX4_PLL: MLX4-clock
 * \return  (uint16_t) LIN Baudrate (0 = No baudrate known/error)
 * *************************************************************************** *
 * \details This function read the MLX4 Private RAM.
 *          At 0x0E01 the Last Baudrate divisor can be found
 *          At 0x0E03 (lower nibble) the Last baudrate pre-scaler
 * *************************************************************************** *
 * - Call Hierarchy: AppCheckLinProc()
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 0
 * *************************************************************************** */
uint16_t p_ml_GetLastBaudRate(uint16_t u16Mlx4Clk)
{
    uint16_t u16Result;
    uint16_t u16Rubbish;

    __asm__ __volatile__ (
        "cmp Y, #50000 \n\t"                          /* [ASM] Y = u16Mlx4Clk */
        "jug p_GLBR_Error_%= \n\t"                    /* [C]   if (u16Mlx4Clk > 50000) {u16Result = 0;} */
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81350__)
        "lod AL, 0x0E01 \n\t"                         /* [C]   u8Divider  [ASM] AL = Last-Baudrate-Divider */
#else
        "lod AL, 0x0A01 \n\t"                         /* [C]   u8Divider  [ASM] AL = Last-Baudrate-Divider */
#endif
        "cmp AL, #99 \n\t"                            /* [C]   if (u8Divider < 99) */
        "juge p_GLBR_00_%= \n\t"                      /* [C]   { */
        "p_GLBR_Error_%=: \n\t"
        "mov A, #0 \n\t"                              /* [C]     u16Result = 0; */
        "jmp p_GLBR_Exit_%= \n\t"                     /* [C]     return; } */
        "p_GLBR_00_%=: \n\t"
        "usex A \n\t"                                 /* [C]   u16Divider = (uint16_t)u8Divider; */
        "mov X, A \n\t"
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81350__)
        "lod AL, 0x0E03 \n\t"                         /* [C]   i8PreScaler  [ASM] AL = Pre-scaler */
#else
        "lod AL, 0x0A03 \n\t"                         /* [C]   i8PreScaler  [ASM] AL = Pre-scaler */
#endif
        "and AL, #0x0F \n\t"                          /* [C]   i8PreScaler &= 0x0FU; [ASM] AL = (Pre-scaler & 0x0F) */
        "cmp AL, #8 \n\t"                             /* [C]   if (u8PreScaler > 8) */
        "jug p_GLBR_Error_%= \n\t"                    /* [C]   { {u16Result = 0; } */
        "mov Cx, AL \n\t"
        "p_GLBR_10_%=: \n\t"                          /* [C]   do */
        "asl X \n\t"                                  /* [C]   { u16Divider *= 2 */
        "djnz Cx, p_GLBR_10_%= \n\t"                  /* [C]   } while (--u8PreScaler != 0); */
        "p_GLBR_20_%=: \n\t"
        "mov A, Y \n\t"
        "mulu YA, A, #500 \n\t"                       /* [C]   u32Temp = (u16Mlx4Clk * 1000) / 2 */
        "cmp Y, X \n\t"                               /* [C]   if ((u32Temp>>16) >= u16Divider) {u16Result = 0;} */
        "juge p_GLBR_Error_%= \n\t"                   /* [ASM] Overflow */
        "divu YA, X \n\t"                             /* [C]   u16Result = u32Temp / u16Divider; */
        "divu YA, X \n\t"
        "p_GLBR_Exit_%=:"
        : "=a" (u16Result), "=x" (u16Rubbish), "=y" (u16Rubbish)
        : "y" (u16Mlx4Clk)
        : "D"
        );

    return (u16Result);
} /* End of p_ml_GetLastBaudRate() */

/*!*************************************************************************** *
 * p_ml_GetAutoBaudRate
 * \brief   Private Get Last LIN Baudrate
 * \author  mmp
 * *************************************************************************** *
 * \param   MLX4_PLL: MLX4-clock
 * \return  (uint16_t) LIN Baudrate (0 = No baudrate known/error)
 * *************************************************************************** *
 * \details This function read the MLX4 Private RAM.
 *          At 0x0E02 the Auto Baudrate divisor can be found
 *          At 0x0E03 (upper nibble) the Auto baudrate pre-scaler
 * *************************************************************************** *
 * - Call Hierarchy: AppCheckLinProc()
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 0
 * *************************************************************************** */
uint16_t p_ml_GetAutoBaudRate(uint16_t u16Mlx4Clk)
{
    uint16_t u16Result;
    uint16_t u16Rubbish;

    __asm__ __volatile__ (
        "cmp Y, #50000 \n\t"                          /* [ASM] Y = u16Mlx4Clk */
        "jug p_GABR_Error_%= \n\t"                    /* [C]   if (u16Mlx4Clk > 50000) {u16Result = 0;} */
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81350__)
        "lod AL, 0x0E02 \n\t"                         /* [C]   u8Divider  [ASM] AL = Auto-Baudrate-Divider */
#else
        "lod AL, 0x0A02 \n\t"                         /* [C]   u8Divider  [ASM] AL = Auto-Baudrate-Divider */
#endif
        "cmp AL, #99 \n\t"                            /* [C]   if (u8Divider < 99) */
        "juge p_GABR_00_%= \n\t"                      /* [C]   { */
        "p_GABR_Error_%=: \n\t"
        "mov A, #0 \n\t"                              /* [C]     u16Result = 0; */
        "jmp p_GABR_Exit_%= \n\t"                     /* [C]     return; } */
        "p_GABR_00_%=: \n\t"
        "usex A \n\t"                                 /* [C]   u16Divider = (uint16_t)u8Divider; */
        "mov X, A \n\t"
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81350__)
        "lod AL, 0x0E03 \n\t"                         /* [C]   i8PreScaler  [ASM] AL = Pre-scaler */
#else
        "lod AL, 0x0A03 \n\t"                         /* [C]   u8Divider  [ASM] AL = Auto-Baudrate-Divider */
#endif
        "lsr AL, #2 \n\t"
        "lsr AL, #2 \n\t"                             /* [C]   i8PreScaler >>= 4U; [ASM] AL = (Pre-scaler >> 4) */
        "cmp AL, #8 \n\t"                             /* [C]   if (u8PreScaler > 8) */
        "jug p_GABR_Error_%= \n\t"                    /* [C]   { {u16Result = 0; } */
        "mov Cx, AL \n\t"
        "p_GABR_10_%=: \n\t"                          /* [C]   do */
        "asl X \n\t"                                  /* [C]   { u16Divider *= 2 */
        "djnz Cx, p_GABR_10_%= \n\t"                  /* [C]   } while (--u8PreScaler != 0); */
        "p_GABR_20_%=: \n\t"
        "mov A, Y \n\t"
        "mulu YA, A, #500 \n\t"                       /* [C]   u32Temp = (u16Mlx4Clk * 1000) / 2 */
        "cmp Y, X \n\t"                               /* [C]   if ((u32Temp>>16) >= u16Divider) {u16Result = 0;} */
        "juge p_GABR_Error_%= \n\t"                   /* [ASM] Overflow */
        "divu YA, X \n\t"                             /* [C]   u16Result = u32Temp / u16Divider; */
        "divu YA, X \n\t"
        "p_GABR_Exit_%=:"
        : "=a" (u16Result), "=x" (u16Rubbish), "=y" (u16Rubbish)
        : "y" (u16Mlx4Clk)
        : "D"
        );

    return (u16Result);
} /* End of p_ml_GetAutoBaudRate() */

#endif /* (LIN_COMM != FALSE) */

/* EOF */
