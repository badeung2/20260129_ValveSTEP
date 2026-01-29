/**
 * @file
 * @brief Central module of the UDS layer. Token: uds.
 * @internal
 *
 * @copyright (C) 2018 Melexis N.V.
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
 * @addtogroup lib_uds_api
 *
 * @details Init UDS layer and parse services on the server side.
 *          The implementation is optimized for LIN Transport Layer usage.
 *
 * @{
 */

#ifndef UDS_HEADERS_H
#define UDS_HEADERS_H

#include <stdint.h>

/*
 * List of some standardized services and sessions
 */

/** Positive response is UdsRequest_t + 0x40 */
#define UDS_POSITIVE_RESPONSE(request) (((uint8_t) request) + 0x40u)

/** General negative response codes */
typedef enum UdsNegativeRequest_e {
    UDS_NRESP_SERVICE_NOT_SUPPORTED = 0x11u,                        /**< serviceNotSupported */
    UDS_NRESP_SUBFUNCTION_NOT_SUPPORTED = 0x12u,                    /**< sub-functionNotSupported */
    UDS_NRESP_INCORRECT_MESSAGE_LENGTH = 0x13u,                     /**< incorrectMessageLengthOrInvalidFormat */
    UDS_NRESP_RESPONSE_TOO_LONG = 0x14u,                            /**< responseTooLong */
    UDS_NRESP_BUSY_REPEAT_REQUEST = 0x21u,                          /**< busyRepeatRequest */
    UDS_NRESP_CONDITIONS_NOT_CORRECT = 0x22u,                       /**< conditionsNotCorrect */
    UDS_NRESP_SEQUENCE_ERROR = 0x24u,                               /**< requestSequenceError */
    UDS_NRESP_REQUEST_OUT_OF_RANGE = 0x31u,                         /**< requestOutOfRange */
    UDS_NRESP_SECURITY_ACCESS_DENIED = 0x33u,                       /**< securityAccessDenied */
    UDS_NRESP_INVALID_KEY = 0x35u,                                  /**< invalidKey */
    UDS_NRESP_EXCEEDED_NUMBER_OF_ATTEMPTS = 0x36u,                  /**< exceededNumberOfAttempts */
    UDS_NRESP_REQUIRED_DELAY_NOT_EXPIRED = 0x37u,                   /**< requiredTimeDelayNotExpired */
    UDS_NRESP_UPLOAD_DOWNLOAD_NOT_ACCEPTED = 0x70u,                 /**< uploadDownloadNotAccepted */
    UDS_NRESP_TRANSFER_DATA_SUSPENDED = 0x71u,                      /**< transferDataSuspended */
    UDS_NRESP_GENERAL_PROGRAMMING_FAILURE = 0x72u,                  /**< generalProgrammingFailure */
    UDS_NRESP_WRONG_BLOCK_SEQUENCE_COUNTER = 0x73u,                 /**< wrongBlockSequenceCounter */
    UDS_NRESP_RESPONSE_PENDING = 0x78u,                             /**< requestCorrectlyReceived-ResponsePending */
    UDS_NRESP_SUBFUNCTION_NOT_SUPPORTED_IN_ACTIVE_SESSION = 0x7Eu,  /**< sub-functionNotSupportedInActiveSession */
    UDS_NRESP_SERVICE_NOT_SUPPORTED_IN_ACTIVE_SESSION = 0x7Fu,      /**< serviceNotSupportedInActiveSession */
    UDS_NRESP_VOLTAGE_TOO_HIGH = 0x92u,                             /**< voltageTooHigh */
    UDS_NRESP_VOLTAGE_TOO_LOW = 0x93u                               /**< voltageTooLow */
} UdsNegativeResponse_t;

/** Supported services */
typedef enum UdsRequest_e {
    UDS_REQ_NOT_INITIALIZED = 0x00u,            /**< Request is not initialized - for MISRA */
    UDS_REQ_DIAGNOSTIC_SESSION_CONTROL = 0x10u, /**< DiagnosticSessionControl */
    UDS_REQ_ECU_RESET = 0x11u,                  /**< ECUReset */
    UDS_REQ_CLEAR_DIAGNOSTIC_INFO = 0x14u,      /**< ClearDiagnosticInformation */
    UDS_REQ_READ_DTC_INFO = 0x19u,              /**< ReadDTCInformation */
    UDS_REQ_READ_DATA_BY_ID = 0x22u,            /**< ReadDataByIdentifier */
    UDS_REQ_READ_MEMORY_BY_ADDRESS = 0x23u,     /**< ReadMemoryByAddress */
    UDS_REQ_READ_SCALING_DATA_BY_ID = 0x24u,    /**< ReadScalingDataByIdentifier */
    UDS_REQ_SECURITY_ACCESS = 0x27u,            /**< SecurityAccess */
    UDS_REQ_COMMUNICATION_CONTROL = 0x28u,      /**< CommunicationControl */
    UDS_REQ_READ_DATA_BY_PERIODIC_ID = 0x2Au,   /**< ReadDataByPeriodicIdentifier */
    UDS_REQ_DYNAMICALLY_DEFINE_DATA_ID = 0x2Cu, /**< DynamicallyDefineDataIdentifier */
    UDS_REQ_WRITE_DATA_BY_ID = 0x2Eu,           /**< WriteDataByIdentifier */
    UDS_REQ_IO_CONTROL_BY_ID = 0x2Fu,           /**< InputOutputControlByIdentifier */
    UDS_REQ_ROUTINE_CONTROL = 0x31u,            /**< RoutineControl */
    UDS_REQ_REQUEST_DOWNLOAD = 0x34u,           /**< RequestDownload */
    UDS_REQ_REQUEST_UPLOAD = 0x35u,             /**< RequestUpload */
    UDS_REQ_TRANSFER_DATA = 0x36u,              /**< TransferData */
    UDS_REQ_REQUEST_TRANSFER_EXIT = 0x37u,      /**< RequestTransferExit */
    UDS_REQ_REQUEST_FILE_TRANSFER = 0x38u,      /**< RequestFileTransfer */
    UDS_REQ_WRITE_MEMORY_BY_ADDRESS = 0x3Du,    /**< WriteMemoryByAddress */
    UDS_REQ_TESTER_PRESENT = 0x3Eu,             /**< TesterPresent */
    UDS_REQ_ACCESS_TIMING_PARAMETER = 0x83u,    /**< AccessTimingParameter */
    UDS_REQ_SECURED_DATA_TRANSMISSION = 0x84u,  /**< SecuredDataTransmission */
    UDS_REQ_CONTROL_DTC_SETTING = 0x85u,        /**< ControlDTCSetting */
    UDS_REQ_RESPONSE_ON_EVENT = 0x86u,          /**< ResponseOnEvent */
    UDS_REQ_LINK_CONTROL = 0x87u,               /**< LinkControl */
    UDS_REQ_NEGATIVE_RESPONSE_SERVICE = 0x7Fu   /**< Negative response service identifier */
} UdsRequest_t;

/** Supported sessions */
typedef enum UdsSessionType_e {
    UDS_LEV_DS_DEFAULT_SESSION = 0x01u,             /**< defaultSession */
    UDS_LEV_DS_PROGRAMMING_SESSION = 0x02u,         /**< programmingSession */
    UDS_LEV_DS_EXTENDED_DIAG_SESSION = 0x03u,       /**< extendedDiagnosticSession */
    UDS_LEV_DS_SAFETY_SYS_DIAG_SESSION = 0x04u,     /**< safetySystemDiagnosticSession */
    UDS_LEV_DS_VEHICLE_MAN_SPEC_SESSION = 0x40u,    /**< start of vehicleManufacturerSpecificSession */
    UDS_LEV_DS_SYS_SUPPLIER_SPEC_SESSION = 0x60u,   /**< start of systemSupplierSpecificSession */
    UDS_LEV_DS_ISO_RESERVED = 0x7Fu                 /**< ISO reserved but also last enum */
} __attribute__((packed)) UdsSessionType_t;

typedef struct UdsSession_s {
    UdsSessionType_t type; /**< Session ID (type) */
    uint16_t timeout; /**< Timeout (1 ms) for the type of session */
    uint16_t enhanced_timeout; /**< Enhanced timeout (10 ms resolution) of the type of session */
} UdsSession_t;

/** ECU Reset Subfunctions */
typedef enum UdsEcuResetSubFunction_e {
    UDS_ECU_RESET_HARD_RESET = 0x01u,               /**< hardReset */
    UDS_ECU_RESET_KEY_OFF_ON_RESET = 0x02u,         /**< keyOffOnReset */
    UDS_ECU_RESET_SOFT_RESET = 0x03u,               /**< softReset */
    UDS_ECU_RESET_ENABLE_RAPID_SHUTDOWN = 0x04u,    /**< enableRapidPowerShutDown */
    UDS_ECU_RESET_DISABLE_RAPID_SHUTDOWN = 0x05u,   /**< disableRapidPowerShutDown */
    UDS_ECU_RESET_VEHICLE_MAN_SPEC = 0x40u,         /**< start of vehicleManufacturerSpecific ECU reset reasons */
    UDS_ECU_RESET_SYS_SUPPLIER_SPEC = 0x60u,        /**< start of systemSupplierSpecific ECU reset reasons */
    UDS_ECU_RESET_UDS_BOOTLOADER = UDS_ECU_RESET_SYS_SUPPLIER_SPEC,        /**< Reset into Chip UDS bootloader */
    UDS_ECU_RESET_ISO_RESERVED = 0x7Fu              /**< ISO reserved but also last enum */
} UdsEcuResetSubFunction_t;

/** Routine Control Subfunctions */
typedef enum UdsRoutineControlSubFunction_e {
    UDS_ROUTINE_CONTROL_START_ROUTINE = 0x01u,          /**< startRoutine */
    UDS_ROUTINE_CONTROL_STOP_ROUTINE = 0x02u,           /**< stopRoutine */
    UDS_ROUTINE_CONTROL_REQUEST_ROUTINE_RESULTS = 0x03u /**< requestRoutineResults */
} UdsRoutineControlSubFunction_t;

/** Request Security Access Subfunctions */
typedef enum UdsSecurityAccessSubFunction_e {
    UDS_SECURITY_ACCESS_REQUEST_SEED = 0x11u,   /**< requestSeed */
    UDS_SECURITY_ACCESS_SEND_KEY = 0x12u,    /**< sendKey */
} UdsSecurityAccessSubFunction_t;

/** Control DTC Setting Subfunctions */
typedef enum UdsDtcSettingSubFunction_e {
    UDS_CONTROL_DTC_SETTING_ON = 0x01u,             /**< Control DTC Setting ON */
    UDS_CONTROL_DTC_SETTING_OFF = 0x02u,         /**< Control DTC Setting OFF */
} UdsDtcSettingSubFunction_t;

/** Communication Control Subfunctions */
typedef enum UdsCommunicationControlSubFunction_e {
    UDS_COMMUNICATION_CONTROL_ENABLE_RX_TX = 0x00u,             /**< Enable both RX and TX */
    UDS_COMMUNICATION_CONTROL_ENABLE_RX_DISABLE_TX = 0x01u,         /**< Enable RX, but disable TX */
    UDS_COMMUNICATION_CONTROL_DISABLE_RX_ENABLE_TX = 0x02u,         /**< Enable TX, but disable RX */
    UDS_COMMUNICATION_CONTROL_DISABLE_RX_TX = 0x03u,             /**< Disable both RX and TX */
} UdsCommunicationControlSubFunction_t;

/** Communication Control Subfunctions */
typedef enum UdsCommunicationControlType_e {
    UDS_COMMUNICATION_CONTROL_NORMAL = 0x01u,       /**< Bit 0-1 encoding for normal communication messages type */
    UDS_COMMUNICATION_CONTROL_NETWORK = 0x02u,      /**< Bit 0-1 encoding for network communication messages type */
    UDS_COMMUNICATION_CONTROL_NORMAL_AND_NETWORK = 0x03u, /**< Bit 0-1 encoding for both message types */
} UdsCommunicationControlType_t;

typedef enum UdsTesterPresentSubFunction_e {
    UDS_TESTER_PRESENT_USE_POSITIVE_RESPONSE = 0x00u,       /**< Do not suppress positive response */
    UDS_TESTER_PRESENT_SUPPRESS_POSITIVE_RESPONSE = 0x80u   /**< Suppress positive response */
} UdsTesterPresentSubFunction_t;

#define UDS_SUPPORTED_SERVICE_NOT_FOUND 0xffffu /**< Value returned when UdsSupportedService_t array does not contain the
                                                 * requested service */
#define UDS_SUPPORTED_SUBFUNCTION_NOT_FOUND 0xffffu /**< Value returned when UdsSupportedSubFunction_t array does not
                                                     * contain the requested subfunction */
/** Index of Service ID inside message */
#define UDS_MSG_INDEX_SERVICE_ID 0

/** Index of Subfunction ID inside message */
#define UDS_MSG_INDEX_SUBFUNCTION_ID 1

/** Index of Data Format Identifier in RequestDownload message */
#define UDS_MSG_INDEX_DATA_FORMAT_ID 1

/** Index of Address and Length Identifier in RequestDownload message */
#define UDS_MSG_INDEX_ADDRESS_LENGTH_ID 2

/** Index of transferData BlockCounter */
#define UDS_MSG_INDEX_TRANSFER_DATA_SEQUENCE_ID 1u

/** Index of transferRequestParameterRecord in TransferData message */
#define UDS_MSG_INDEX_TRANSFER_REQUEST_RECORD 2u


/** 50ms as recommended in Table 4 ISO14229:2 */
#define UDS_DEFAULT_SESSION_TIMEOUT 50u

/** 500ms as recommended in Table 4 of ISO14229:2 */
#define UDS_DEFAULT_SESSION_ENHANCED_TIMEOUT 500u

/** Value for the no encryption */
#define UDS_MSG_NO_ENCRYPTION_NO_COMPRESSION 0u


/** Minimum allowed message length for services with sub-functions support (SID + SubFunction parameter) */
#define UDS_MIN_SUB_FUNC_PDU_LENGTH     2u

/** Default value for the memory size */
#define UDS_MAX_MEMORY_SIZE_DEFAULT INT32_MIN


/** @} */
#endif /* UDS_HEADERS_H */
