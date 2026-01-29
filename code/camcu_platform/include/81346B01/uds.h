/**
 * @file
 * @brief Server implementation of UDS protocol
 * @internal
 *
 * @copyright (C) 2019 Melexis N.V.
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
 * @defgroup lib_uds UDS Library
 * @brief UDS Library doxygen documentations
 *
 * @details UDS protocol, based on ISO14229 specification is used to communicate between sensor (device, server) and
 * central processing unit. Central processing unit sends various messages and requests for general functionality and/or
 * diagnostic status of the sensor (device). This implementation is focused on server side (sensor) which listens for
 * requests and replies to them appropriately.
 *
 * @addtogroup lib_uds_api General API
 * @brief Application needs to only call the @ref UDS_Init and @ref UDS_Tick functions to library
 * @ingroup lib_uds
 * @{
 *
 *
 */

#ifndef UDS_H
#define UDS_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "uds_headers.h"
#include "compiler_abstraction.h"

#ifdef HAS_UDS_OVERFLOW_DETECTION
#define OVERFLOW_UNIQUE_VALUE 0x55u
#endif

#ifndef UDS_RAPID_SHUTDOWN_TIMEOUT
#define UDS_RAPID_SHUTDOWN_TIMEOUT 4u
#endif


typedef enum UdsState_e {
    UDS_ERROR = -1,
    UDS_NOT_INITIALIZED = 0,
    UDS_INIT = 2,
    UDS_RECEIVING = 3,
    UDS_BUSY = 4,
    UDS_SENDING = 5
} UdsState_t;

#ifndef UDS_RECEIVE_BUFFER_MAX_LENGTH
/** Maximum received message length ever */
#define UDS_RECEIVE_BUFFER_MAX_LENGTH 128u

#endif

#ifndef UDS_SEND_BUFFER_MAX_LENGTH
/** Maximum length of sent message */
#define UDS_SEND_BUFFER_MAX_LENGTH 128u

#endif

#ifndef UDS_SUPPORTED_SERVICES_MAX_NUM
/** Maximum number of supported services for the binary search */
#define UDS_SUPPORTED_SERVICES_MAX_NUM 128u

#endif

#ifndef UDS_SUPPORTED_SESSIONS_MAX_NUM
#define UDS_SUPPORTED_SESSIONS_MAX_NUM 5u /**< Maximum number of supported sessions. Define at least one more than what
                                           * you currently have. This will allow you to add more sessions into flash
                                           * while rom offsets will still work */
#endif

#ifndef UDS_CONFIGURATION_MEMORY
/** Define the empty memory section for configuration (services) definition */
#define UDS_CONFIGURATION_MEMORY_SECTION
#define UDS_BMW_CONFIGURATION_MEMORY_SECTION
#define UDS_VW_CONFIGURATION_MEMORY_SECTION
#else
/** Define the memory section for configuration (services) definition */
#define UDS_CONFIGURATION_MEMORY_SECTION __attribute__((section(".uds_configuration")))
#define UDS_BMW_CONFIGURATION_MEMORY_SECTION __attribute__((section(".uds_bmw_configuration")))
#define UDS_VW_CONFIGURATION_MEMORY_SECTION __attribute__((section(".uds_vw_configuration")))
#endif

#ifndef UDS_DATA_MEMORY
/** Define the empty memory section for data definition */
#define UDS_DATA_MEMORY_SECTION
#define UDS_BMW_DATA_MEMORY_SECTION
#define UDS_VW_DATA_MEMORY_SECTION
#else
/** Define the memory section for data definition */
#define UDS_DATA_MEMORY_SECTION __attribute__((section(".uds_data")))
#define UDS_BMW_DATA_MEMORY_SECTION __attribute__((section(".uds_bmw_data")))
#define UDS_VW_DATA_MEMORY_SECTION __attribute__((section(".uds_vw_data")))
#endif

#ifndef UDS_CUSTOM_MEMORY
/** Define the empty memory section for certain functions which can be detached */
#define UDS_CUSTOM_MEMORY_SECTION
#define UDS_WEAK_CUSTOM_MEMORY_SECTION
#define UDS_BMW_CUSTOM_MEMORY_SECTION
#define UDS_WEAK_BMW_CUSTOM_MEMORY_SECTION
#define UDS_VW_CUSTOM_MEMORY_SECTION
#define UDS_WEAK_VW_CUSTOM_MEMORY_SECTION
#else
/** Define the memory section for functions which can be detached */
#define UDS_CUSTOM_MEMORY_SECTION __attribute__((section(".uds_custom_functions")))
#define UDS_WEAK_CUSTOM_MEMORY_SECTION __attribute__((weak, section(".uds_custom_functions")))
#define UDS_BMW_CUSTOM_MEMORY_SECTION __attribute__((section(".uds_bmw_custom_functions")))
#define UDS_WEAK_BMW_CUSTOM_MEMORY_SECTION __attribute__((weak, section(".uds_bmw_custom_functions")))
#define UDS_VW_CUSTOM_MEMORY_SECTION __attribute__((section(".uds_vw_custom_functions")))
#define UDS_WEAK_VW_CUSTOM_MEMORY_SECTION __attribute__((weak, section(".uds_vw_custom_functions")))
#endif

typedef struct UdsConfig_s UdsConfig_t;

/** Map Vehicle Manufacturer special services to Service types */
typedef struct UdsSupportedService_s {
    CONST UdsRequest_t request_id; /**< Request ID of the service */
    CONST UdsSessionType_t allowed_session_id[UDS_SUPPORTED_SESSIONS_MAX_NUM]; /**< Allowed session IDs for the
                                                                                  request_id */
    void (*custom_func)(UdsConfig_t *uds); /**< Custom function to call or NULL if ROM function should be used */
} UdsSupportedService_t;

/** Bootloader, application, and possibly some other layer can define their own supported services */
typedef struct UdsLayer_s {
    const UdsSession_t *supported_sessions; /**< Array of the currently supported sessions */
    const UdsSupportedService_t *supported_services; /**< Array of currently supported services */
    uint16_t reserved[2]; /**< Reserved for future use */
} UdsLayer_t;

struct UdsConfig_s {
    union {
        struct FlagsBits_s {
            uint16_t received : 1; /**< When complete message is received and waiting for processing in receive_buffer */
            uint16_t sent : 1; /**< When message in send_buffer was sent */
            uint16_t unlocked : 1; /**< When security procedure was completed and chip is unlocked */
            uint16_t erased : 1; /**< When memory was erased and it is ready to be written */
            uint16_t writeable : 1; /**< When correct procedure was received and we can write */
            uint16_t silent : 1; /**< When we do not want to send positive response back */
            uint16_t dtc : 1; /**< When we do not want to update dtc errors */
            uint16_t disable_tx : 1; /**< When we do not want application to transmit anything (not monitored by lib) */
            uint16_t disable_rx : 1; /**< When we do not want application to receive anything (not monitored by lib) */
            uint16_t reserved : 7; /**< Reserved for future use */
        } flag;
        uint16_t flags;
    };
    UdsState_t state;
    void (*execute_next)(void); /**< Pointer to execute function after state change */
    UdsSessionType_t session; /**< Active (current) session */
    uint8_t sequence_counter; /**< Sequence counter for last received packet - needed for some services */
    int32_t max_memory_size; /**< Maximum memory size to be received */
    uint32_t random_value; /**< Random value used in seed key mechanism */
    uint8_t reserved[8]; /**< Reserved for future use */
    uint16_t session_timeout; /**< Active session default timeout with resolution of 1 ms */
    uint16_t session_enhanced_timeout; /**< Active session enhanced timeout with resolution of 10 ms */
    uint16_t (*transport_layer_send_function)(void); /**< Pointer to send function of transport layer to trigger
                                                      * sending of data */
    uint8_t receive_buffer_length; /**< Number of elements in receive_buffer */
    uint8_t send_buffer_length; /**< Number of elements in send_buffer */
    uint8_t receive_buffer[UDS_RECEIVE_BUFFER_MAX_LENGTH]; /**< Raw received message buffer */
#ifdef HAS_UDS_OVERFLOW_DETECTION
    uint8_t overflow_detection_r; /**< Overflow detection variable for receive buffer */
#endif
    uint8_t send_buffer[UDS_SEND_BUFFER_MAX_LENGTH]; /**< Send buffer */
#ifdef HAS_UDS_OVERFLOW_DETECTION
    uint8_t overflow_detection_s; /**< Overflow detection variable for send buffer */
#endif
    UdsSession_t supported_sessions[UDS_SUPPORTED_SESSIONS_MAX_NUM]; /**< Array describing the currently supported
                                                                          sessions */
    const UdsSupportedService_t *supported_services; /**< Array of currently supported services */
} __attribute__ ((aligned (2))); /* Make sure whole structure is 16-bit aligned to avoid odd address access */

/** Default initialization structure for supported_sessions */
#define UDS_INIT_DEFAULT_SESSION \
    { \
        .type = UDS_LEV_DS_DEFAULT_SESSION, \
        .timeout = UDS_DEFAULT_SESSION_TIMEOUT, \
        .enhanced_timeout = UDS_DEFAULT_SESSION_ENHANCED_TIMEOUT \
    }, { \
        .type = UDS_LEV_DS_PROGRAMMING_SESSION, \
        .timeout = 5000u, \
        .enhanced_timeout = 1600u \
    }, { \
        .type = UDS_LEV_DS_EXTENDED_DIAG_SESSION, \
        .timeout = 5000u, \
        .enhanced_timeout = 1600u \
    }, { \
        .type = UDS_LEV_DS_SAFETY_SYS_DIAG_SESSION, \
        .timeout = 5000u, \
        .enhanced_timeout = 1600u \
    }

#define UDS_INIT_LAST_SERVICE \
    { \
        .request_id=UDS_REQ_NOT_INITIALIZED, \
        .allowed_session_id={0}, \
        .custom_func=NULL, \
    }

/** Default supported sessions */
extern UDS_DATA_MEMORY_SECTION CONST UdsSession_t supported_sessions_default[];

/** Default supported services */
extern UDS_DATA_MEMORY_SECTION CONST UdsSupportedService_t supported_services_default[];

/** Default bootloader layer information */
extern UDS_CONFIGURATION_MEMORY_SECTION CONST UdsLayer_t uds_layer_bootloader_default;

/** Initialization of the UDS library
 *
 * Some details still need to be described.
 *
 * @param[in] config Struct with initialized field
 * @param[in] layer_info Pointer to the layer info containing supported sessions and services.
 */
void UDS_Init(UdsConfig_t *config, const UdsLayer_t *layer_info);

/** State machine tick functionality
 *
 * Provide regular slice for parsing and updating internal variables
 */
void UDS_Tick(UdsConfig_t *config);

/** Check if UDS state machine is ready to receive a message.
 *
 *  Check if UDS state machine is ready to receive a message.
 *
 * @param[in] config UDS configuration structure
 */
bool UDS_MessageReadyToReceive(UdsConfig_t *config);

/** Signal complete UDS message and run UDS_Tick to process it.
 *
 *  When complete UDS message is received and should be processed before the next UDS_Tick function is called, this
 *  function basically marks message as received and executes a Tick command.
 *
 * @param[in,out] config UDS configuration structure (we read current state and forward it to UDS_Tick for processing)
 */
void UDS_CompleteMessageReceived(UdsConfig_t *config);

/** UDS message ready to be sent.
 *
 *  Check if UDS message is prepared and ready to be sent.
 *
 * @param[in] config UDS configuration structure
 */
bool UDS_MessageReadyToSend(CONST UdsConfig_t *config);

/** Signal layer2 completed sending a UDS message.
 *
 * @param[in] config UDS configuration structure
 */
void UDS_CompleteMessageSent(UdsConfig_t *config);

/** Extract information, if positive responses should be sent.
 *
 * When positive responses are suppressed, then also application functions should obtain this information. This function
 * should be called before each positive response message is prepared.
 *
 * @param[in] config UDS configuration structure
 *
 * @retval true Positive response should be sent
 * @retval false Positive responses should be suppressed
 */
bool UDS_AllowPositiveResponse(CONST UdsConfig_t *config);

/** Check if the LIN SID is in range for UDS library to handle
 *
 * Confirm that the SID request is in table 2 of ISO 14229-1 reserved area
 *
 * @param[in] linsid Lin diagnostic service id
 *
 * @retval true LIN SID should be handled by UDS library
 * @retval false LIN SID is not part of the ISO14229-1 reserved area
 */
bool UDS_LinDiagSIDInUdsRange(uint8_t linsid);

/** Clear configuration flags.
 *
 * Leave received and sent flags alone, but clear all others
 *
 * @param[in,out] config UDS configuration structure (we set session)
 */
void UDS_ClearConfigurationFlags(UdsConfig_t *config);

/** Send positive response with only mandatory items.
 *
 * Fill send buffer with mandatory values. Does not increase/decrease counters, so that it can be reused by repeated
 * response. Sequence counter is taken as in structure, so if it needs to be increased it should be done before calling
 * the function.
 *
 * @param[in,out] config UDS configuration structure with send_buffer and change to sending state
 */
void UDS_RequestTransferDataPositiveResponse(UdsConfig_t *config);
#define UDS_RequestTransferDataPositiveReponse UDS_RequestTransferDataPositiveResponse attribute__ ((deprecated( \
                                                                                                         "Renamed to UDS_RequestTransferDataPositiveResponse")))

/** Extract uint16 from 8 bit array.
 *
 * @param[in] buf Pointer to 8 bit buffer in UDS message
 * @return Converted size array to native unsigned 16 bit integer
 */
uint16_t UDS_MemoryExtractFromMsg_uint16(CONST uint8_t *buf);

/** Extract uint32 from 8 bit array.
 *
 * @param[in] buf Pointer to 8 bit buffer in UDS message
 * @return Converted size array to native unsigned 32 bit integer
 */
uint32_t UDS_MemoryExtractFromMsg_uint32(CONST uint8_t *buf);

/** Extract memory_address and memory_size based on AddressAndLengthIdentifier for Request Download.
 *
 * Size and address length are extracted from Identifier in received buffer of Request Download message. That byte
 * lengths are then used to extract the correct amount of bytes from the message and stored into a uint32_t variables
 * provided by external function.
 *
 * @param[in,out] config UDS configuration to have access to send and receive buffers as well as max_page_size
 * @param[in] msg_index UDS message index of address and length identifier
 * @param[out] memory_address Pointer to external variable to extracted memory_address. This can be anything from 8 to
 *                            32 bits, but biggest variable is reserved to prevent overflows.
 * @param[out] memory_size Pointer to external variable to extracted memory_size. This can be anything from 8 to
 *                         32 bits, but biggest variable is reserved to prevent overflows.
 * @retval True When values for memory_address and memory_size were extracted and are non zero.
 * @retval False When values were not extracted and negative response message UDS_NRESP_REQUEST_OUT_OF_RANGE is
 *               prepared.
 */
bool UDS_MemoryExtractAddressAndLength(UdsConfig_t *config, uint8_t msg_index, uint32_t *memory_address,
                                       uint32_t *memory_size);

/** Formulate and trigger of sending of negative response.
 *
 * Assembles the negative response inside send_buffer and sets the UDS state machine to UDS_SENDING mode, so that it
 * triggers external application to start sending the message.
 *
 * @param[in,out] config Configuration struct (for send_buffer and for state machine)
 * @param[in] service_request_sid Service ID for which negative response should be sent
 * @param[in] negative_response Reason for the negative response
 */
void UDS_NegativeResponse(UdsConfig_t *config, UdsRequest_t service_request_sid,
                          UdsNegativeResponse_t negative_response);

/** Formulate and trigger of sending of positive response with data payload.
 *
 * Assembles the positive response inside send_buffer and sets the UDS state machine to UDS_SENDING mode, so that it
 * triggers external application to start sending the message. If you need to append to the message then after this
 * function set the remaining fields in UDS send_buffer and adjust UDS send_buffer_length accordingly.
 *
 * @param[in,out] config Configuration struct (for send_buffer and for state machine)
 * @param[in] service_request_sid Service ID for which positive response should be sent (it is automatically converted
 *                                to positive response value so +0x40
 * @param[in] data Pointer to data to be sent excluding Service ID
 * @param[in] data_len Length of the data pointed by data pointer
 */
void UDS_PositiveResponse(UdsConfig_t *config, UdsRequest_t service_request_sid, const uint8_t *data, uint8_t data_len);

/** Formulate and trigger of sending of positive response with byte subfunction and data payload.
 *
 * Assembles the positive response inside send_buffer and sets the UDS state machine to UDS_SENDING mode, so that it
 * triggers external application to start sending the message. If you need to append to the message then after this
 * function set the remaining fields in UDS send_buffer and adjust UDS send_buffer_length accordingly.
 *
 * @param[in,out] config Configuration struct (for send_buffer and for state machine)
 * @param[in] service_request_sid Service ID for which positive response should be sent (it is automatically converted
 *                                to positive response value so +0x40
 * @param[in] type Byte type or subfunction on 2nd position
 * @param[in] data Pointer to data to be sent excluding Service ID
 * @param[in] data_len Length of the data pointed by data pointer
 */
void UDS_PositiveResponseWithType8(UdsConfig_t *config, UdsRequest_t service_request_sid, uint8_t type,
                                   const uint8_t *data, uint8_t data_len);

/** Formulate and trigger of sending of positive response with byte subfunction and data payload.
 *
 * Assembles the positive response inside send_buffer and sets the UDS state machine to UDS_SENDING mode, so that it
 * triggers external application to start sending the message. If you need to append to the message then after this
 * function set the remaining fields in UDS send_buffer and adjust UDS send_buffer_length accordingly.
 *
 * @param[in,out] config Configuration struct (for send_buffer and for state machine)
 * @param[in] service_request_sid Service ID for which positive response should be sent (it is automatically converted
 *                                to positive response value so +0x40
 * @param[in] type 2 Byte type on 2nd and 3rd position. Function does the shifting required.
 * @param[in] data Pointer to data to be sent excluding Service ID
 * @param[in] data_len Length of the data pointed by data pointer
 */
void UDS_PositiveResponseWithType16(UdsConfig_t *config, UdsRequest_t service_request_sid, uint16_t type,
                                    const uint8_t *data, uint8_t data_len);

/** Append data to a response
 *
 * Append data to a response which was prepared before using functions like UDS_PositiveResponse.
 *
 * @param[in,out] config Configuration struct (for send_buffer and for state machine)
 * @param[in] data Pointer to data to be appended
 * @param[in] data_len Length of the data pointed by data pointer
 */
void UDS_AppendResponse(UdsConfig_t* config, const uint8_t *data, uint8_t data_len);

/** Check UDS message for minimum length.
 *
 * UDS message needs to always have at least more than 2 bytes (service and data). If it does not have, then this
 * function replies with negative response.
 *
 * @param[in] config UDS configuration
 * @param[in] minimum_length Minimum allowed message length
 *
 * @retval true When message passes the check
 * @retval false when message is not passing the check. False already compiles and sends negative response using the
 *               @link UDS_NegativeResponse @endlink
 */
bool UDS_MinLengthCheck(UdsConfig_t *config, uint8_t minimum_length);

/** Check UDS message for maximum length.
 *
 * Some UDS messages have a fixed maximum length. If the message is longer then function replies with negative response.
 *
 * @param[in] config UDS configuration
 * @param[in] maximum_length Maximum allowed message length
 *
 * @retval true When message passes the check
 * @retval false when message is not passing the check. False already compiles and sends negative response using the
 *               @link UDS_NegativeResponse @endlink
 */
bool UDS_MaxLengthCheck(UdsConfig_t *config, uint8_t maximum_length);

/** Change state of state machine
 *
 * UDS state machine change might have some other functionality appended to it, so this function enables that you do not
 * directly change the state of the state machine.
 *
 * @param[in,out] config UDS configuration structure (we read current state, but write null to execute_next once
 *                       function is executed.
 * @param[in] desired_state Transition UDS state machine to this state.
 */
void UDS_SetState(UdsConfig_t *config, UdsState_t desired_state);

/** ISO implementation of ControlDTCSetting request.
 *
 * When HAS_UDS_NO_ISO_FALLBACK is defined the general function for the ControlDTCSetting is still used in the OEM
 * specific structures and should be available nevertheless.
 *
 * @param[in] config UDS configuration structure
 */
void UDS_ISO_RequestControlDTCSetting(UdsConfig_t *config);

/** ISO implementation of RequestCommunicationControl request.
 *
 * When HAS_UDS_NO_ISO_FALLBACK is defined the general function for the RequestCommunicationControl is still used in the
 * OEM specific structures and should be available nevertheless.
 *
 * @param[in] config UDS configuration structure
 */
void UDS_ISO_RequestCommunicationControl(UdsConfig_t *config);

/** ISO implementation of RequestTransferExit request.
 *
 * When HAS_UDS_NO_ISO_FALLBACK is defined the general function for the RequestTransferExit is still used in the OEM
 * specific structures and should be available nevertheless.
 *
 * @param[in] config UDS configuration structure
 */
void UDS_ISO_RequestTransferExit(UdsConfig_t *config);

/** ISO implementation of TesterPresent request.
 *
 * When HAS_UDS_NO_ISO_FALLBACK is defined the general function for the RequestTesterPresent is still used in the OEM
 * specific structures and should be available nevertheless.
 *
 * @param[in] config UDS configuration structure
 */
void UDS_ISO_RequestTesterPresent(UdsConfig_t *config);

extern NO_RETURN void UDS_ChipReset(void);
extern NO_RETURN void UDS_ChipResetIntoBootloader(void);
extern void UDS_SetPositiveResponseAfterReset(UdsRequest_t request, uint8_t data);
extern UdsRequest_t UDS_GetPositiveResponseAfterReset(uint8_t *data);
extern void UDS_PrepareTransportLayerForResponseAfterReset(UdsConfig_t *config);
extern uint16_t UDS_MemoryWriteableCheck(uint32_t address, uint32_t size);
extern uint8_t UDS_MemoryCrcCheck(uint8_t memory_identifier, uint32_t assumed_crc);
extern bool UDS_MemoryWrite(uint8_t *buffer, uint16_t buffer_length, uint8_t *checksum, uint16_t *checksum_length);
extern uint8_t UDS_MemoryErase(uint32_t address, uint32_t size);
extern uint32_t UDS_GetRandomValue(void);
extern bool UDS_CheckChallenge(uint32_t received_challenge, uint32_t random_value);
extern uint8_t UDS_ProgrammingDependenciesCheck(void);
extern bool UDS_GetApplicationConsistancy(void);
#define UDS_CheckSeed UDS_CheckChallenge __attribute__((deprecated("Replaced by UDS_CheckChallenge")))

#define udsNegativeResponse UDS_NegativeResponse __attribute__((deprecated("Replaced by UDS_NegativeResponse")))
#define udsMinLengthCheck UDS_MinLengthCheck __attribute__((deprecated("Replaced by UDS_MinLengthCheck")))
#define udsMaxLengthCheck UDS_MaxLengthCheck __attribute__((deprecated("Replaced by UDS_MaxLengthCheck")))
#define udsSetState UDS_SetState __attribute__((deprecated("Replaced by UDS_SetState")))
#define udsClearConfigurationFlags UDS_ClearConfigurationFlags __attribute__((deprecated( \
                                                                                  "Replaced by \
                                                                                         UDS_ClearConfigurationFlags")))
#define udsMemoryExtractAddressAndLength UDS_MemoryExtractAddressAndLength __attribute__((deprecated( \
                                                                                              "Replaced by \
                                                                                                     UDS_MemoryExtractAddressAndLength")))
#define udsMemoryExtractFromMsg_uint16 UDS_MemoryExtractFromMsg_uint16 __attribute__((deprecated( \
                                                                                          "Replaced by \
                                                                                                 UDS_MemoryExtractFromMsg_uint16")))
#define udsMemoryExtractFromMsg_uint32 UDS_MemoryExtractFromMsg_uint32 __attribute__((deprecated( \
                                                                                          "Replaced by \
                                                                                                 UDS_MemoryExtractFromMsg_uint32")))
#define UDS_UdsBootloaderReset UDS_ChipResetIntoBootloader __attribute__((deprecated( \
                                                                              "Replaced by UDS_ChipResetIntoBootloader")))
#endif /* UDS_H */
/** @} */
