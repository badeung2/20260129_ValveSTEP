/**
 * @file
 * @brief The Flash firmware MLX LIN slave initialization.
 * @internal
 *
 * @copyright (C) 2018-2020 Melexis N.V.
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
 * @details This file contains MLX LIN slave initialization by user FW.
 * @note The file can be replaced by application specific from application folder.
 */

#ifndef FW_MLS_API_H_
#define FW_MLS_API_H_

#include "mls_api.h"


/* ---------------------------------------------
 * Public Defines
 * --------------------------------------------- */

/** Maximum number of Application LIN transport layer events subscribers */
#define MLS_TL_SUBSCRIBER_APPLICATION 1

#if (FW_LIN_TL_FUNCTIONAL_NAD_FIX == 1) || (FW_LIN_TL_LDT_TICK_FIX == 1)
/** Maximum number of Application LIN transport layer events subscribers */
    #define MLS_TL_SUBSCRIBER_MLS_API_FIXES 1
#else
/** Maximum number of Application LIN transport layer events subscribers */
    #define MLS_TL_SUBSCRIBER_MLS_API_FIXES 0
#endif

#if defined (HAS_STD_LIN_API)
/** Maximum number of STD LIN API LIN transport layer events subscribers */
    #define MLS_TL_SUBSCRIBER_STD_LIN_API 1
#else
/** Maximum number of STD LIN API LIN transport layer events subscribers */
    #define MLS_TL_SUBSCRIBER_STD_LIN_API 0
#endif

#if defined (HAS_MLS_DEVICE_ID)
/** Maximum number of MLS device ID LIN transport layer events subscribers */
    #define MLS_TL_SUBSCRIBER_MLS_DEVICE_ID 1
#else
/** Maximum number of MLS device ID LIN transport layer events subscribers */
    #define MLS_TL_SUBSCRIBER_MLS_DEVICE_ID 0
#endif

#if defined (HAS_LIN_AUTO_ADDRESSING)
/** Number of LIN auto addressing transport layer events subscribers */
    #define MLS_TL_SUBSCRIBER_LIN_AUTO_ADDRESSING 1
#else
/** Number of LIN auto addressing transport layer events subscribers */
    #define MLS_TL_SUBSCRIBER_LIN_AUTO_ADDRESSING 0
#endif

#if defined (HAS_APP_UDS)
/** Maximum number of UDS LIN transport layer event subscribers */
    #define MLS_TL_SUBSCRIBER_APP_UDS 1
#else
/** Maximum number of UDS LIN transport layer event subscribers */
    #define MLS_TL_SUBSCRIBER_APP_UDS 0
#endif

#if defined (HAS_MLS_LOADER)
/** Maximum number of Melexis LIN loader transport layer event subscribers */
    #define MLS_TL_SUBSCRIBER_MLS_LOADER 1
#else
/** Maximum number of Melexis LIN loader  transport layer event subscribers */
    #define MLS_TL_SUBSCRIBER_MLS_LOADER 0
#endif

/** Total maximum number of LIN transport layer event subscribers */
#define MLS_TL_MAX_NUMBER_OF_SUBSCRIBERS (          \
            MLS_TL_SUBSCRIBER_APPLICATION +         \
            MLS_TL_SUBSCRIBER_STD_LIN_API +         \
            MLS_TL_SUBSCRIBER_MLS_API_FIXES +       \
            MLS_TL_SUBSCRIBER_MLS_DEVICE_ID +       \
            MLS_TL_SUBSCRIBER_LIN_AUTO_ADDRESSING + \
            MLS_TL_SUBSCRIBER_APP_UDS +             \
            MLS_TL_SUBSCRIBER_MLS_LOADER            \
        )


/* ---------------------------------------------
 * Public Type Declarations
 * --------------------------------------------- */

/** LDT Error Event handler data type */
typedef void (* fw_mls_ErrorEventHandler_t)(ml_LinError_t Error);

/** Multiple stages handler structure for ErrorDetected event */
typedef struct {
    fw_mls_ErrorEventHandler_t* eventHandlerBuffer; /**< Buffer with multiple stage handlers */
    uint8_t eventsMaxLength; /**< Maximum number of the events to be registered */
    uint8_t eventsCount; /**< The number of events registered for the multiple stage handler */
} fw_mls_ErrorDetectedMultipleHandler_t;


/* ---------------------------------------------
 * Global Function Declarations
 * --------------------------------------------- */

/** MLX LIN Slave initialization.
 * Enables the transport layer usage.
 */
extern void fw_mls_Init(void);

/** MLX LIN Slave Transport Layer initialization customization for STD LIN API.
 * Platform/project shall have control over the multiple handler variables.
 */
extern void fw_mls_TransportLayerInit(void);

/** Sharing the ErrorDetected event with all interested in SW components.
 * The SW component is responsible to subscribe it's handler in:
 * ld_ErrorDetectedMultipleHandler.eventHandlerBuffer[someIndex]
 * @param[in] error  LIN TL error code
 */
extern void fw_mls_ErrorDetected(ml_LinError_t error);


/* ---------------------------------------------
 * Global Variable Declarations
 * --------------------------------------------- */

/** TL data structure */
extern LDT_t ldt_TL_data;

/** TL multiple handler for Request event */
extern ldt_MultipleHandler_t ld_RequestMultipleHandler;

/** TL multiple handler for Response Transmitted event */
extern ldt_MultipleHandler_t ld_ResponseTransmittedMultipleHandler;

/** TL multiple handler for Functional Request event */
extern ldt_MultipleHandler_t ld_FunctionalRequestMultipleHandler;

/** TL multiple handler for Error event */
extern fw_mls_ErrorDetectedMultipleHandler_t ld_ErrorDetectedMultipleHandler;

#endif /* FW_MLS_API_H_ */

