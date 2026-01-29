/**
 * @file
 * @brief The Flash firmware MLX LIN slave initialization.
 * @internal
 *
 * @copyright (C) 2018-2020 Melexis N.V.
 * git flash d0014c23
 *
 * Melexis N.V. is supplying this code for use with Melexis N.V. processor based microcontrollers only.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY,
 * INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.  MELEXIS N.V. SHALL NOT IN ANY CIRCUMSTANCES,
 * BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 * @endinternal
 *
 * @ingroup fw_mls_api
 *
 * @defgroup fw_mls_api MLX LIN slave API
 * @{
 * @brief   MLX LIN slave API
 * @details MLX LIN slave initialization by user FW.
 * @}
 *
 * @details This file contains MLX LIN slave initialization by user FW.
 * @note The file can be replaced by application specific from application folder.
 */


#include "itc_helper.h"
#include "fw_mls_api.h"
#include "mls_support.h"


/* ---------------------------------------------
 * Local Defines
 * --------------------------------------------- */

#ifndef MLS_IRQ_PRIO
#define MLS_IRQ_PRIO 6u
#endif


/* ---------------------------------------------
 * Local Function Declarations
 * --------------------------------------------- */

#if FW_LIN_TL_EMPTY_ERROR_HANDLER == 1
/** LIN TL handler.
 * Custom handler for the LIN TL error handler.
 * Workaround for LIN v2.2:13.2.2.
 * @param[in] error  LIN TL error code
 */
void fw_mls_ldt_CustomErrorHandler(ml_LinError_t error);
#endif

#if FW_LIN_TL_LDT_TICK_FIX == 1
/** LIN TL handler.
 * Intermediate handler for M2S diagnostic message
 * to make sure that N_AS Timeout counter is cleared.
 * Workaround for https://gitlab.melexis.com/swcc-lib-mlx16/lib-lin/issues/32.
 * @param[in] transfer  LIN TL transmission buffer
 * @return false - allow next subscribed handler to handle the event.
 */
bool fw_mls_ldt_CustomDiagRequest(LINDiagTransfer_t* transfer);
#endif /* FW_LIN_TL_LDT_TICK_FIX */

#if FW_LIN_TL_FUNCTIONAL_NAD_FIX == 1
/** LIN TL handler.
 * Intermediate handler for Response Transmitted event to make sure that
 * initial state is reached after the last CF has been sent.
 * Workaround for https://gitlab.melexis.com/swcc-lib-mlx16/lib-lin/issues/33.
 * @param[in] transfer  LIN TL transmission buffer
 * @return false - allow next subscribed handler to handle the event.
 */
bool fw_mls_ldt_CustomResponseTransmitted(LINDiagTransfer_t* transfer);
#endif /* FW_LIN_TL_FUNCTIONAL_NAD_FIX */


/* ---------------------------------------------
 * Variables
 * --------------------------------------------- */

#if ML_HAS_LIN_EVENT_TABLE_IN_RAM == 0
/* MLS event table */
static LINEventTable_t fwlinEventTable;
#endif /* ML_HAS_LIN_EVENT_TABLE_IN_RAM */

/* TL data structure */
LDT_t ldt_TL_data;

/* TL multiple stage handlers */
ldt_MultipleHandler_t ld_RequestMultipleHandler;
ldt_MultipleHandler_t ld_ResponseTransmittedMultipleHandler;
ldt_MultipleHandler_t ld_FunctionalRequestMultipleHandler;
fw_mls_ErrorDetectedMultipleHandler_t ld_ErrorDetectedMultipleHandler;

/** Buffer with TL Request multiple stage handlers */
static ldt_EventHandler_t ld_RequestMHBuffer[MLS_TL_MAX_NUMBER_OF_SUBSCRIBERS];

/** Buffer with TL Response Transmitted multiple stage handlers */
static ldt_EventHandler_t ld_ResponseTransmittedMHBuffer[MLS_TL_MAX_NUMBER_OF_SUBSCRIBERS];

/** Buffer with TL Functional Request multiple stage handlers */
static ldt_EventHandler_t ld_FunctionalRequestMHBuffer[MLS_TL_MAX_NUMBER_OF_SUBSCRIBERS];

/** Buffer with TL Error multiple stage handlers */
static fw_mls_ErrorEventHandler_t ld_ErrorDetectedMHBuffer[MLS_TL_MAX_NUMBER_OF_SUBSCRIBERS];


/* ---------------------------------------------
 * Function Definitions
 * --------------------------------------------- */

void fw_mls_Init(void)
{
    mls_SetMlx4ClockDivider();
#if ML_HAS_LIN_EVENT_TABLE_IN_RAM == 0
    ml_InitLinEventTable(&fwlinEventTable);
#endif /* ML_HAS_LIN_EVENT_TABLE_IN_RAM == 0 */

    /* Initialize LIN TL */
    ld_RequestMultipleHandler.eventHandlerBuffer = ld_RequestMHBuffer;
    ld_RequestMultipleHandler.eventsMaxLength = MLS_TL_MAX_NUMBER_OF_SUBSCRIBERS;
    ld_RequestMultipleHandler.eventsCount = 0u;
    ld_ResponseTransmittedMultipleHandler.eventHandlerBuffer = ld_ResponseTransmittedMHBuffer;
    ld_ResponseTransmittedMultipleHandler.eventsMaxLength = MLS_TL_MAX_NUMBER_OF_SUBSCRIBERS;
    ld_ResponseTransmittedMultipleHandler.eventsCount = 0u;
    ld_FunctionalRequestMultipleHandler.eventHandlerBuffer = ld_FunctionalRequestMHBuffer;
    ld_FunctionalRequestMultipleHandler.eventsMaxLength = MLS_TL_MAX_NUMBER_OF_SUBSCRIBERS;
    ld_FunctionalRequestMultipleHandler.eventsCount = 0u;
    ld_ErrorDetectedMultipleHandler.eventHandlerBuffer = ld_ErrorDetectedMHBuffer;
    ld_ErrorDetectedMultipleHandler.eventsMaxLength = MLS_TL_MAX_NUMBER_OF_SUBSCRIBERS;
    ld_ErrorDetectedMultipleHandler.eventsCount = 0u;

    Itc_Clear(COLIN_LIN);
    Itc_SetPrio(COLIN_LIN, MLS_IRQ_PRIO);
    Itc_Enable(COLIN_LIN);
}

void fw_mls_TransportLayerInit(void)
{
    ldt_Init(&ldt_TL_data,
             ML_PM_LIN,
             &ld_RequestMultipleHandler,
             &ld_ResponseTransmittedMultipleHandler,
             &ld_FunctionalRequestMultipleHandler);

#if FW_LIN_TL_EMPTY_ERROR_HANDLER == 1
    /* overwrite the in ldt_Init() configured mlu_ErrorDetected handler */
    plinEventTable->mlu_ErrorDetected = fw_mls_ldt_CustomErrorHandler;
#endif

#if FW_LIN_TL_LDT_TICK_FIX == 1
    (void)ldt_SubscribeMultipleHandler(plinEventTable->ldt_Request, fw_mls_ldt_CustomDiagRequest);
#endif /* FW_LIN_TL_LDT_TICK_FIX */

#if FW_LIN_TL_FUNCTIONAL_NAD_FIX == 1
    (void)ldt_SubscribeMultipleHandler(plinEventTable->ldt_ResponseTransmitted, fw_mls_ldt_CustomResponseTransmitted);
#endif

    plinEventTable->ldt_ErrorDetected = fw_mls_ErrorDetected;
}

void fw_mls_ErrorDetected(ml_LinError_t error)
{
    for (uint8_t cnt = 0; cnt < ld_ErrorDetectedMultipleHandler.eventsCount; cnt++) {
        ld_ErrorDetectedMultipleHandler.eventHandlerBuffer[cnt](error);
    }
}

#if FW_LIN_TL_EMPTY_ERROR_HANDLER == 1
void fw_mls_ldt_CustomErrorHandler(ml_LinError_t error)
{
    plinEventTable->ldt_ErrorDetected(error);
}
#endif

#if FW_LIN_TL_LDT_TICK_FIX == 1
bool fw_mls_ldt_CustomDiagRequest(LINDiagTransfer_t* transfer)
{
    (void)transfer;

    /* assign 100 percent value as new M2S diag. message has been just received */
    ldt_TL_data.transmitTimeout = 100;
    return false;
}
#endif /* FW_LIN_TL_LDT_TICK_FIX */

#if FW_LIN_TL_FUNCTIONAL_NAD_FIX == 1
bool fw_mls_ldt_CustomResponseTransmitted(LINDiagTransfer_t* transfer)
{
    (void)transfer;

    /* return to the initial state */
    ldt_TL_data.transferState = TS_WaitForFirstFrame;
    return false;
}
#endif /* FW_LIN_TL_FUNCTIONAL_NAD_FIX */

/** LIN interrupt handler
 */
__attribute__((interrupt)) void _COLIN_LIN_INT(void)
{
    ml_LinInterruptHandler();
}

