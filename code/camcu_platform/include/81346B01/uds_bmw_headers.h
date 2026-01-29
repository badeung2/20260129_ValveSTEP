/**
 * @file
 * @brief BMW specific header declarations and pointers
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
 * @addtogroup lib_uds_bmw BMW
 * @ingroup lib_uds
 *
 * @details BMW specific UDS layer and parse services on the server side.
 *          The implementation is optimized for LIN Transport Layer usage.
 */

#ifndef UDS_BMW_HEADERS_H
#define UDS_BMW_HEADERS_H

#include "uds.h"
#include "uds_headers.h"
#include "compiler_abstraction.h"
#include <stdbool.h>

UDS_BMW_CUSTOM_MEMORY_SECTION void UDS_BMW_RequestEcuReset(UdsConfig_t *config);
UDS_BMW_CUSTOM_MEMORY_SECTION void UDS_BMW_RequestDiagnosticSessionControl(UdsConfig_t *config);
UDS_BMW_CUSTOM_MEMORY_SECTION void UDS_BMW_RequestSecurityAccess(UdsConfig_t *config);
UDS_BMW_CUSTOM_MEMORY_SECTION void UDS_BMW_RequestTransferData(UdsConfig_t *config);
UDS_BMW_CUSTOM_MEMORY_SECTION void UDS_BMW_RequestReadDataById(UdsConfig_t *config);
UDS_BMW_CUSTOM_MEMORY_SECTION void UDS_BMW_RequestRoutineControlStartRoutine(UdsConfig_t *config);
UDS_BMW_CUSTOM_MEMORY_SECTION void UDS_BMW_RequestRequestDownload(UdsConfig_t *config);
UDS_BMW_CUSTOM_MEMORY_SECTION void UDS_BMW_RequestWriteDataById(UdsConfig_t *config);
UDS_BMW_CUSTOM_MEMORY_SECTION void UDS_BMW_RequestRoutineControl(UdsConfig_t *config);

/** WriteDataById is called after the type subfunction is extracted.
 *
 * This function is called after the WriteDataById packet is received and the type subfunction is extracted.
 *
 * We provide a simple weak declaration of this function by default, but feel free to extend it towards your needs.
 *
 * @code{.c}
 * void UDS_BMW_External_WriteDataById(UdsConfig_t *config, uint16_t type)
 * {
 *     if(type == (uint16_t) UDS_BMW_READ_DATA_BY_ID_PART_NUMBER) {
 *         if(((config->receive_buffer[3] >= (uint8_t)'0') && (config->receive_buffer[3] <= (uint8_t)'9')) &&
 *            ((config->receive_buffer[9] >= (uint8_t)'0') && (config->receive_buffer[9] <= (uint8_t)'9'))) {
 *             memcpy(part_number, &config->receive_buffer[3], 7);
 *             UDS_PositiveResponseWithType16(config,
 *                                            UDS_REQ_WRITE_DATA_BY_ID,
 *                                            UDS_BMW_READ_DATA_BY_ID_PART_NUMBER,
 *                                            NULL,
 *                                            0);
 *         } else {
 *             UDS_NegativeResponse(config, UDS_REQ_WRITE_DATA_BY_ID, UDS_NRESP_INCORRECT_MESSAGE_LENGTH);
 *         }
 *     } else {
 *         UDS_NegativeResponse(config, UDS_REQ_WRITE_DATA_BY_ID, UDS_NRESP_SUBFUNCTION_NOT_SUPPORTED);
 *     }
 * }
 * @endcode
 *
 * @param[in,out] config Uds configuration structure
 * @param[in] type Extracted uint16_t type from the received buffer of the config structure
 */
void UDS_BMW_External_WriteDataById(UdsConfig_t *config, uint16_t type);

/** ReadDataById is called after the type subfunction is extracted.
 *
 * This function is called after the ReadDataById packet is received and the type subfunction is extracted.
 *
 * We provide a simple weak declaration of this function by default, but feel free to extend it towards your needs.
 *
 * @code{.c}
 * void UDS_BMW_External_ReadDataById(UdsConfig_t *config, uint16_t type)
 * {
 *     switch(type) {
 *         case (uint16_t) UDS_BMW_READ_DATA_BY_ID_PART_NUMBER:
 *             UDS_PositiveResponseWithType16(config, UDS_REQ_READ_DATA_BY_ID, UDS_BMW_READ_DATA_BY_ID_PART_NUMBER, NULL,
 *                                            0);
 *             memcpy(&config->send_buffer[3], part_number, 7);
 *             config->send_buffer_length = 3u + 7u;
 *             break;
 *         default:
 *             UDS_NegativeResponse(config, UDS_REQ_READ_DATA_BY_ID, UDS_NRESP_SUBFUNCTION_NOT_SUPPORTED);
 *             break;
 *     }
 * }
 * @endcode
 *
 * @param[in,out] config Uds configuration structure
 * @param[in] type Extracted uint16_t type from the received buffer of the config structure
 */
void UDS_BMW_External_ReadDataById(UdsConfig_t *config, uint16_t type);

/** Default initialization structure for supported_sessions for BMW */
#define UDS_INIT_DEFAULT_SESSION_BMW \
    { \
        .type = UDS_LEV_DS_DEFAULT_SESSION, \
        .timeout = 50u, \
        .enhanced_timeout = UDS_DEFAULT_SESSION_ENHANCED_TIMEOUT \
    }, { \
        .type = UDS_LEV_DS_PROGRAMMING_SESSION, \
        .timeout = 5000u, \
        .enhanced_timeout = 1600u \
    }

/** Special vehicle manufacturer sessions */
typedef enum UdsBMWSessionType_e {
    UDS_LEV_DS_VM_BMW_CODING_SESSION = 0x41u,       /**< codingSession */
    UDS_LEV_DS_VM_BMW_SWT_SESSION = 0x42u,          /**< SWTSession */
    UDS_LEV_DS_VM_BMW_HDD_UPDATE_SESSION = 0x43u    /**< HDDUpdateSession */
} UdsBMWSessionType_t;

/** Request ReadDataById Session subfunctions for BMW */
typedef enum UdsReadDataByIdSubFunctionBMW_e {
    UDS_BMW_READ_DATA_BY_ID_PART_NUMBER = 0x15FFu,  /**< Read BMW Part Number */
    UDS_READ_DATA_BY_ID_ACTIVE_SESSION_STATE = 0xF100u,   /**< Read Active Session State */
    UDS_READ_DATA_BY_ID_ACTIVE_DIAGNOSTIC_SESSION = 0xF186u,   /**< Active Diagnostic Session */
} UdsReadDataByIdSubFunctionBMW_t;

/** Routine Identifiers for BMW */
typedef enum UdsRoutineControlRoutineIdentifierBMW_e {
    UDS_BMW_ROUTINE_CONTROL_CHECK_MEMORY = 0x0202u,  /**< CheckMemory Routine Identifier */
    UDS_BMW_ROUTINE_CONTROL_ERASE_MEMORY = 0xFF00u, /**< EraseMemory Routine Identifier */
    UDS_BMW_ROUTINE_CONTROL_ENERGY_MODE = 0x0F0Cu,  /**< SetEnergyMode Routine Identifier */
    UDS_BMW_ROUTINE_CONTROL_CHECK_DEPENDENCIES = 0xFF01u, /**< checkProgrammingDependencies Routine Identifier */
} UdsRoutineControlRoutineIdentifierBMW_t;

/** Erase Memory methods for BMW */
typedef enum UdsRoutineControlEraseMemoryBMW_e {
    UDS_ROUTINE_CONTROL_ERASE_MEMORY_METHOD_ADDRESSED = 0x01u,   /**< erasingMethod addressed */
    UDS_ROUTINE_CONTROL_ERASE_MEMORY_METHOD_INDICATED = 0x02u,   /**< erasingMethod indicated */
} UdsRoutineControlEraseMemoryBMW_t;

/** Memory Check methods for BMW */
typedef enum UdsRoutineControlCheckMemoryBMW_e {
    UDS_ROUTINE_CONTROL_CHECK_MEMORY_ADDRESSED = 0x11u,   /**< CheckMemory addressed */
    UDS_ROUTINE_CONTROL_CHECK_MEMORY_INDICATED = 0x12u,   /**< CheckMemory indicated */
} UdsRoutineControlCheckMemoryBMW_t;

/** Energy Modes for BMW */
typedef enum UdsRoutineControlEnergyModeBMW_e {
    UDS_ROUTINE_CONTROL_ENERGY_MODE_NORMAL = 0x00u, /**< Normal memory mode */
    UDS_ROUTINE_CONTROL_ENERGY_MODE_FLASH = 0x03u,  /**< Flash memory Mode */
} UdsRoutineControlEnergyModeBMW_t;

#define UDS_BMW_READ_DATA_BY_ID_SESSION_STATE_LOCKED 0x01
#define UDS_BMW_READ_DATA_BY_ID_SESSION_STATE_UNLOCKED 0x02
#define UDS_BMW_READ_DATA_BY_ID_SESSION_STATE_APPLICATION 0x81

#endif /* UDS_BMW_HEADERS_H */
