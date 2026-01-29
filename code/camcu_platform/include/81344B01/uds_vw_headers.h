/**
 * @file
 * @brief Volkwagen specific header declarations and pointers
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
 * @addtogroup lib_uds_vw VolksWagen specific implementation
 * @ingroup lib_uds
 *
 * @details Volkswagen specific UDS layer and parse services on the server side.
 *          The implementation is optimized for LIN Transport Layer usage.
 */

#ifndef UDS_VW_HEADERS_H
#define UDS_VW_HEADERS_H

#include "uds.h"
#include "uds_headers.h"
#include "compiler_abstraction.h"

UDS_VW_CUSTOM_MEMORY_SECTION void UDS_VW_RequestEcuReset(UdsConfig_t *config);
UDS_VW_CUSTOM_MEMORY_SECTION void UDS_VW_RequestDiagnosticSessionControl(UdsConfig_t *config);
UDS_VW_CUSTOM_MEMORY_SECTION void UDS_VW_RequestSecurityAccess(UdsConfig_t *config);
UDS_VW_CUSTOM_MEMORY_SECTION void UDS_VW_RequestTransferData(UdsConfig_t *config);
UDS_VW_CUSTOM_MEMORY_SECTION void UDS_VW_RequestRequestDownload(UdsConfig_t *config);
UDS_VW_CUSTOM_MEMORY_SECTION void UDS_VW_RequestReadDataById(UdsConfig_t *config);
UDS_VW_CUSTOM_MEMORY_SECTION void UDS_VW_RequestWriteDataById(UdsConfig_t *config);
UDS_VW_CUSTOM_MEMORY_SECTION void UDS_VW_RequestRoutineControl(UdsConfig_t *config);

/** WriteDataById is called after the type subfunction is extracted.
 *
 * This function is called after the WriteDataById packet is received and the type subfunction is extracted.
 *
 * We provide a simple weak declaration of this function by default, but feel free to extend it towards your needs.
 *
 * @code{.c}
 * void UDS_VW_External_WriteDataById(UdsConfig_t *config, uint16_t type)
 * {
 *     if(type == UDS_VW_DID_WRITE_FINGERPRINT) {
 *         if(UDS_MinLengthCheck(config, 12u) && UDS_MaxLengthCheck(config, 12u)) {
 *             // Check if service is allowed to be checked
 *             if(config->flag.unlocked == 1u) {
 *                 uint8_t nresp = UDS_SetPartNumber(&config->receive_buffer[3]);
 *                 if(nresp == 0u) {
 *                     UDS_PositiveResponseWithType16(config, UDS_REQ_WRITE_DATA_BY_ID, UDS_VW_DID_WRITE_FINGERPRINT, NULL, 0);
 *                 } else {
 *                     UDS_NegativeResponse(config, UDS_REQ_WRITE_DATA_BY_ID, nresp);
 *                 }
 *             } else {
 *                 UDS_NegativeResponse(config, UDS_REQ_WRITE_DATA_BY_ID, UDS_NRESP_SECURITY_ACCESS_DENIED);
 *             }
 *         }
 *     } else {
 *         UDS_NegativeResponse(config, UDS_REQ_WRITE_DATA_BY_ID, UDS_NRESP_REQUEST_OUT_OF_RANGE);
 *     }
 * }
 * @endcode
 *
 * @param[in,out] config Uds configuration structure
 * @param[in] type Extracted uint16_t type from the received buffer of the config structure
 */
void UDS_VW_External_WriteDataById(UdsConfig_t *config, uint16_t type);

/** ReadDataById is called after the type subfunction is extracted.
 *
 * This function is called after the ReadDataById packet is received and the type subfunction is extracted.
 *
 * We provide a simple weak declaration of this function by default, but feel free to extend it towards your needs.
 *
 * @code{.c}
 * void UDS_VW_External_ReadDataById(UdsConfig_t *config, uint16_t type)
 * {
 *     if(type == UDS_VW_DID_READ_FINGERPRINT) {
 *         if(UDS_MinLengthCheck(config, 3u) && UDS_MaxLengthCheck(config, 3u)) {
 *             UDS_PositiveResponseWithType16(config, UDS_REQ_READ_DATA_BY_ID, UDS_VW_DID_READ_FINGERPRINT,
 *                                            NULL, 0);
 *             config->send_buffer_length = 3u + UDS_GetPartNumber(&config->send_buffer[3]);
 *         }
 *     } else {
 *         UDS_NegativeResponse(config, UDS_REQ_READ_DATA_BY_ID, UDS_NRESP_REQUEST_OUT_OF_RANGE);
 *     }
 * }
 * @endcode
 *
 * @param[in,out] config Uds configuration structure
 * @param[in] type Extracted uint16_t type from the received buffer of the config structure
 */
void UDS_VW_External_ReadDataById(UdsConfig_t *config, uint16_t type);

/** Special vehicle manufacturer sessions */
typedef enum UdsVMSessionType_e {
    UDS_LEV_DS_VM_VW_EOL_SESSION = 0x40u,           /**< VWEOLSession */
    UDS_LEV_DS_VM_VW_DEVELOPMENT_SESSION = 0x4Fu,   /**< DevelopmentSession */
} UdsVMSessionType_t;

/** Read/Write Data by ID mandatory bootloader identifiers
 *
 * Read/Write Data by ID identifiers which are mandatory and need to be answered by the bootloader
 * as stated in "Q-LAH 80125 V5.8 Teil 2 - Datendefinition" section "1 Beschreibung der DataIdentifier".
 * This list includes all fields marked with:
 * - Convention: "All servers" or "FLASHEN"
 * - Diagnostics class: DK2F
 *
 * @note The doxygen comment mentiones in which sesions which R-read or W-write operation is allowed.
 */
typedef enum UdsVwDidBootloader_e {
    UDS_VW_DID_SW_BLOCK_PROG_CTR = 0x0407u, /**< VW Logical Software Block Counter Of Programming Attempts (session: 0x01-R, 0x02-R, 0x03-R) */
    UDS_VW_DID_SW_BLOCK_LOCK_VAL = 0x040Fu, /**< VW Logical Software Block Lock Value (session: 0x01-R, 0x02-R, 0x03-R) */
    UDS_VW_DID_WRITE_FINGERPRINT = 0xF15Au, /**< Fingerprint (session: 0x02-RW) */
    UDS_VW_DID_READ_FINGERPRINT = 0xF15Bu,  /**< Fingerprint And Programming Date Of Logical Software Blocks (session: 0x01-R, 0x02-R, 0x03-R) */
    UDS_VW_DID_FAZIT_ID_STRING = 0xF17Cu,   /**< VW FAZIT Identification String (session: 0x01-R, 0x02-R, 0x03-R) */
    UDS_VW_DID_ACT_DIAG_SESSION = 0xF186u,  /**< Active Diagnostic Session (session: 0x01-R, 0x02-R, 0x03-R) */
    UDS_VW_DID_SPARE_PART_NR = 0xF187u,     /**< VW Spare Part Number (session: 0x01-R, 0x02-R, 0x03-R) */
    UDS_VW_DID_APP_SW_VERSION = 0xF189u,    /**< VW Application Software Version Number (session: 0x01-R, 0x02-R, 0x03-R) */
    UDS_VW_DID_ECU_SERIAL_NR = 0xF18Cu,     /**< ECU Serial Number (session: 0x01-R, 0x02-R, 0x03-R) */
    UDS_VW_DID_ECU_HW_NR = 0xF191u,         /**< VW ECU Hardware Number (session: 0x01-R, 0x02-R, 0x03-R) */
    UDS_VW_DID_ODX_FILE_ID = 0xF19Eu,       /**< ASAM ODX File Identifier (session: 0x01-R, 0x02-R, 0x03-R) */
    UDS_VW_DID_ODX_FILE_VERSION = 0xF1A2u,  /**< ASAM ODX File Version (session: 0x01-R, 0x02-R, 0x03-R) */
    UDS_VW_DID_ECU_HW_VERSION_NR = 0xF1A3u, /**< VW ECU Hardware Version Number (session: 0x01-R, 0x02-R, 0x03-R) */
    UDS_VW_DID_WORKSHOP_SYS_NAME = 0xF1AAu, /**< VW Workshop System Name (session: 0x01-R, 0x03-R) */
    UDS_VW_DID_SW_BLOCK_VERSION = 0xF1ABu,  /**< VW Logical Software Block Version (session: 0x01-R, 0x02-R, 0x03-R) */
    UDS_VW_DID_SYSTEM_FW_VERSION = 0xF1B8u, /**< VW System Firmware Versions (session: 0x01-R, 0x02-R, 0x03-R) */
    UDS_VW_DID_ECU_PROG_INFO = 0xF1DFu,     /**< ECU Programming Information (session: 0x01-R, 0x02-R, 0x03-R) */
    UDS_VW_DID_LOGISTIC_PART_NR = 0xF1E3u,  /**< Logistics Part Number (session: 0x01-R, 0x02-R, 0x03-R) */
} UdsVwDidBootloader_t;

/** Routine Identifiers for VW */
typedef enum UdsRoutineControlRoutineIdentifierVW_e {
    UDS_VW_ROUTINE_CONTROL_CHECK_MEMORY = 0x0202u,  /**< CheckMemory Routine Identifier */
    UDS_VW_ROUTINE_CONTROL_PRECONDITIONS = 0x0203u, /**< PreConditions Routine Identifier */
    UDS_VW_ROUTINE_CONTROL_ERASE_MEMORY = 0xFF00u,  /**< EraseMemory Routine Identifier */
    UDS_VW_ROUTINE_CONTROL_CHECK_DEPENDENCIES = 0xFF01u, /**< checkProgrammingDependencies Identifier */
} UdsRoutineControlRoutineIdentifierVW_t;


#endif /* UDS_VW_HEADERS_H */
