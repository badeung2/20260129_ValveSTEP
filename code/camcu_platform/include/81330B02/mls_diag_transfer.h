/**
 * @file mls_diag_transfer.h
 * @brief Interface definitions for LIN diagnosis transfer layer. Token: ldt
 * @internal
 *
 * @copyright (C) 2016-2018 Melexis N.V.
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
 * @ingroup mlx_lin_slave_api
 *
 * @details Provides Melexis LIN Slave transport layer.
 */

#ifdef UNITTEST
#include "mls_api.h"
#endif

#ifndef MLS_API_H_
#error "The file can't be included directly but via mls_api.h"
#endif /* MLS_API_H_ */

#ifndef MLS_DIAG_TRANSFER_H_
#define MLS_DIAG_TRANSFER_H_

#if ((LDT_HAS_TRANSPORT_LAYER == 1) || (ML_HAS_LIN_EVENT_TABLE_IN_RAM == 1))


/*-----------------------------------------------------------------------------
 * Constants and macros                 (scope: module-local)
 */

#define LDT_DATA_IN_SINGLEFRAME  5u
#define LDT_DATA_IN_FIRSTFRAME   4u
#define LDT_DATA_IN_CONFRAME     6u

#define LDT_MAX_PCI_VALUE_SINGLE_FRAME              6u
#define LDT_MIN_DATALEN_SEGEMENTED_TRANSFER_FRAME   6u

#define LDT_NAD_BROADCAST           0x7Fu
#define LDT_NAD_FUNCTIONAL          0x7Eu
#define LDT_NAD_NO_INITIAL_CHECK    0u
#define LDT_NAD_USER_RANGE_START    0x80u

#define LDT_CF_COUNTER_MASK         0xFu

/* N_AS (P2) maximum value is the timeout after the last diagnostic request frame. Responses frames
 * after this timeout are ignored.
 *
 * N_CR maximum time until reception of the next Consecutive Frame
 *
 * Unit: percents
 */
#define LDT_N_AS_CR 100

#define LDT_GET_EVEN_INT_ROUNG_UP(x) (((x+1) >> 1) << 1)

#ifndef MLXCOMP_369_PADDING
#define MLXCOMP_369_PADDING
#endif

#ifndef MLXCOMP_369_GAP
#define MLXCOMP_369_GAP 0
#endif


/*-----------------------------------------------------------------------------
 * Typedefs                             (scope: global)
 */

/* LIN Frame IDs
 * u: used
 * nu: not used
 */
typedef enum LinFrameIDs_e {
    /* Diagnostic frames */
    LFID_DIAG_REQUEST = 0x3C, /* M2S */
    LFID_DIAG_RESPONSE = 0x3D
/* S2M */
} LinFrameIDs_t;

/* Frame indexes. */
typedef enum LinFrameIndex_t {
    LFIDX_DIAG_REQUEST = 0x10, LFIDX_DIAG_RESPONSE = 0x11,
} LinFrameIndex_t;

typedef enum TransferModeBits_e {
    TM_SingleFrame = 0, TM_FirstFrame = 1, TM_ConFrame = 2 /* Consecutive Frame */,
} TransferModeBits_t;

/*! Union with different LIN frame variants. Used to access the ML_DATA_LIN_FRAME_DATA_BUFFER.
 */
typedef union PACKED Frame_u {
    uint8_t all[8];
    struct PACKED SingleFrame_s {
        uint8_t nad;
        uint8_t pci;
        uint8_t sId; /*!<is used for request SId and response SId;*/
        uint8_t data[LDT_DATA_IN_SINGLEFRAME];
    } single;
    struct PACKED FirstFrame_s {
        uint8_t nad;
        uint8_t pci;
        uint8_t len;
        uint8_t sId; /*!<is used for request SId and response SId;*/
        uint8_t data[LDT_DATA_IN_FIRSTFRAME];
    } first;
    struct PACKED ConFrame_s {
        uint8_t nad;
        uint8_t pci;
        uint8_t data[LDT_DATA_IN_CONFRAME];
    } cont;
} Frame_t;

/*! Union with the different bit interpretations of the PCI byte.
 * Is used to access PCI byte in LINFrames.
 */
typedef union PACKED PCI_u {
    uint8_t all;
    struct PACKED PCISingleFrame_s {
        uint8_t len : 4;
        uint8_t transferModeBits : 4;
    } sf;
    struct PACKED PCIFirstFrame_s {
        uint8_t extLen : 4;
        uint8_t transferModeBits : 4;
    } ff;
    struct PACKED PCIContFrame_s {
        uint8_t counter : 4;
        uint8_t transferModeBits : 4;
    } cf;
} PCI_t;

typedef struct LINDiagRequest_s {
    uint8_t reqSId;
    uint16_t dataLen;   /*!< only data bytes length */
    uint8_t data[LDT_MAX_DATA_IN_SEGMENTED_TRANSFER];
} LINDiagRequest_t;

typedef struct UDSResponse_s {
    uint8_t respSId;
    uint16_t dataLen;   /*!< only data bytes length */
    uint8_t data[LDT_MAX_DATA_IN_SEGMENTED_TRANSFER];
} LINDiagResponse_t;

typedef union UDSTransfer_u {
    LINDiagRequest_t request;
    LINDiagResponse_t response;
} LINDiagTransfer_t;

typedef enum TransferState_e {
    TS_WaitForFirstFrame = 0,   /**< Idle */
    TS_WaitForContFrame = 1,    /**< TS_Tx_physical */
    TS_TransmitResponse = 3,    /**< TS_Rx_physical */
    TS_WaitForEndOfLastFrame = 4
} TransferState_t;

/** Private structure with fixed size.
 * The variables are not part of the bootAPI interface and should not be used by the application.
 */
#if (LDT_STRUCT_COMPAT_MODE == 0)
typedef struct LDT_s {
    Frame_t *frame;
    PCI_t *pci;
    TransferState_t transferState;
    uint16_t dataI;
    uint8_t cfCounter;
    MLXCOMP_369_PADDING;
    uint8_t transmitTimeout;
    MLXCOMP_369_PADDING;
    uint8_t receiveTimeout;
    MLXCOMP_369_PADDING;
    uint8_t lastNAD;
    MLXCOMP_369_PADDING;
    bool isTransmitTimeoutReload;
    MLXCOMP_369_PADDING;
    bool isReceiveTimeoutReload;
    MLXCOMP_369_PADDING;
    ml_ProtocolMode_t ml_ProtocolMode;
    LINDiagTransfer_t transfer;
}__attribute__ ((aligned (2))) LDT_t;
#elif (LDT_STRUCT_COMPAT_MODE == 1)
typedef struct LDT_s {
    Frame_t *frame;
    PCI_t *pci;
    uint16_t dataI;
    uint8_t transmitTimeout;
    uint8_t receiveTimeout;
    bool isTransmitTimeoutReload;
    bool isReceiveTimeoutReload;
    LINDiagTransfer_t transfer;
    TransferState_t transferState;
    uint8_t lastNAD;
    ml_ProtocolMode_t ml_ProtocolMode;
    uint8_t cfCounter;
} LDT_t;
#elif (LDT_STRUCT_COMPAT_MODE == 2)
    #if (LDT_HAS_N_CR_TIMEOUT == 1)
        #error "The LDT_HAS_N_CR_TIMEOUT is not supported for this compatibility mode"
    #endif
typedef struct LDT_s {
    Frame_t *frame;
    PCI_t *pci;
    uint16_t dataI;
    uint16_t cfCounter;
    uint16_t transmitTimeout;
    uint16_t isTransmitTimeoutReload;
    LINDiagTransfer_t transfer;
    TransferState_t transferState;
    uint8_t lastNAD;
    ml_ProtocolMode_t ml_ProtocolMode;
} LDT_t;
#else
    #error "Wrong LDT_STRUCT_COMPAT_MODE value"
#endif

/**
 * LDT Event handler data type.
 * The return value meaning depends on the specific multiple handler,
 * but common part is that in the case of return value == true the next event handler in the
 * multiple handler list will not be called.
 */
typedef bool (*ldt_EventHandler_t)(LINDiagTransfer_t *transfer);

/**
 * Multiple stages handler structure
 */
typedef struct {
    ldt_EventHandler_t *eventHandlerBuffer;
    uint8_t eventsMaxLength; /**< Maximum number of the events to be registered */
    MLXCOMP_369_PADDING;
    uint8_t eventsCount; /**< The number of events registered for the multiple stage handler */
    MLXCOMP_369_PADDING;
} ldt_MultipleHandler_t;


/*-----------------------------------------------------------------------------
 * Declaration of variables             (scope: module-local)
 */

#if LDT_HAS_TRANSPORT_LAYER == 1
extern ML_VAR_ATTR LDT_t* ldtMe;
#endif


/*-----------------------------------------------------------------------------
 * Exported function prototypes         (scope: global)
 */

#if LDT_HAS_TRANSPORT_LAYER == 1
extern void ldt_Init(
    LDT_t* privateVariables,
    ml_ProtocolMode_t protocol,
    ldt_MultipleHandler_t *ldt_RequestMH,
    ldt_MultipleHandler_t *ldt_ResponseTransmittedMH,
    ldt_MultipleHandler_t *ldt_FunctionalRequestMH);

/* Subscribe handlers for ldt_Request and ldt_ResponseTransmitted */
extern bool ldt_SubscribeMultipleHandler(
    ldt_MultipleHandler_t *multipleHandler,
    ldt_EventHandler_t eventHandler);

/**
 * Note: the function behaviour depends on LDT_HAS_N_AS_TIMEOUT and LDT_HAS_N_CR_TIMEOUT
 */
extern void ldt_Tick(uint8_t percentNAs, uint8_t percentNCr);
extern LINDiagTransfer_t* ldt_GetTransferBuffer(void);
extern void ldt_WaitForResponseFrame(void);
extern void ldt_ErrorDetected(ml_LinError_t error);
#endif

#endif /* ((LDT_HAS_TRANSPORT_LAYER == 1) || (ML_HAS_LIN_EVENT_TABLE_IN_RAM == 1)) */
#endif /* MLS_DIAG_TRANSFER_H_ */
