/**
 * @file
 * @brief Melexis specific LIN frames support in application mode.
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
 * @ingroup fw_mls_device_id
 *
 * @defgroup fw_mls_device_id MLX LIN device ID
 * @{
 * @brief   MLX LIN device ID
 * @details MLX device identification and support for entering programming modes over LIN.
 * @}
 *
 * @details Melexis specific LIN frames support used for device identification and entering programming mode in application mode.
 */

#include <string.h>
#include <stdbool.h>
#include "bl_bist.h"
#include "bl_tools.h"
#include "lib_patch.h"
#include "version.h"
#include "memory.h"
#include "fw_mls_api.h"
#include "fw_mls_device_id.h"
#include "bl_tables.h"

#if defined (HAS_MLS_LOADER)
#include "lin_ldr.h"
#endif /* HAS_MLS_LOADER */


/* ---------------------------------------------
 * Type Declarations
 * --------------------------------------------- */

/** Read by ID: Product ID request message structure */
typedef struct PACKED {
    uint8_t id; /**< ID */
    uint16_t supp_id; /**< Supplier ID */
    uint16_t func_id; /**< Function ID */
} fw_lepm_ReadByIdProdIdReq_t;

/** Read by ID: Product ID request message length */
#define LE_RBI_PI_REQ_DATA_LENGTH 5u
ASSERT(sizeof(fw_lepm_ReadByIdProdIdReq_t) == LE_RBI_PI_REQ_DATA_LENGTH);

/** Read by ID: Melexis device ID response message structure */
typedef struct PACKED {
    uint16_t prod_id_low; /**< Melexis project ID (low part) */
    uint16_t prod_id_high; /**< Melexis project ID (high part) */
    uint8_t reserved; /**< reserved (0xFF) */
} fw_lepm_ProdIdRes_t;

/** Read by ID: Melexis device ID response message length */
#define LE_PI_RES_DATA_LENGTH 5u
ASSERT(sizeof(fw_lepm_ProdIdRes_t) == LE_PI_RES_DATA_LENGTH);

/** Read by ID: Melexis platform version response message structure */
typedef struct PACKED {
    uint8_t major; /**< Melexis platform version major number */
    uint8_t minor; /**< Melexis platform version minor number */
    uint8_t patch; /**< Melexis platform version patch number */
    uint8_t build; /**< Melexis platform version build number */
    uint8_t type; /**< Platform type (3 = MULAN2; 4 = MULAN3; 5 = CAMCU; 6 = FX) */
} fw_lepm_PltfRevRes_t;

/** Read by ID: Melexis platform version response message length */
#define LE_PLTF_REV_RES_DATA_LENGTH 5u
ASSERT(sizeof(fw_lepm_PltfRevRes_t) == LE_PLTF_REV_RES_DATA_LENGTH);

/** Read by ID: Melexis patch identifier response message structure */
typedef struct PACKED {
    uint16_t identifier; /**< patch identifier */
} fw_lepm_PatchRevRes_t;

/** Read by ID: Melexis patch identifier response message length */
#define LE_PATCH_REV_RES_DATA_LENGTH 2u
ASSERT(sizeof(fw_lepm_PatchRevRes_t) == LE_PATCH_REV_RES_DATA_LENGTH);


/* ---------------------------------------------
 * Constants and macros
 * --------------------------------------------- */

/*
 * Read by ID message description
 */

/** The Read by ID SID value */
#define LE_READ_BY_ID_SID 0xB2u

/** The Read by ID Response SID value */
#define LE_READ_BY_ID_RSID (LE_READ_BY_ID_SID + 0x40u)

/** Minimal data expected into the Read by ID request (only data bytes w/o SID) */
#define LE_MIN_RBI_DATA_LEN 0x05u

/*
 * LIN Product Identification
 */

/** Melexis Supplier ID - only Melexis can use it */
#define LE_MLX_SUPPLIER_ID 0x0013u

/** Supplier ID broadcast value */
#define LE_MLX_SUPPLIER_ID_BR 0x7FFFu

/** Function ID broadcast value */
#define LE_MLX_FUNCTION_ID_BR 0xFFFFu

/* Extended Melexis specific Read by ID message usage.
 * Note: the broadcast Supplier ID and Function ID are not supported for these messages
 * to avoid any collisions with other Slaves on the bus */

/** Special Melexis specific ID values used for extra communication.
 * It's used only in combination with Melexis Supplier ID.
 * Then it guarantees there will be no collisions with customer application */
#define LE_MLX_SPECIAL_ID 0x33u

/** Special Melexis specific ID value used for reading the flash platform version id
 * It's used only in combination with Melexis Supplier ID.
 * Then it guarantees there will be no collisions with customer application */
#define LE_MLX_PLTF_ID 0x34u

/** Special Melexis specific ID value used for reading the patch identifier
 * It's used only in combination with Melexis Supplier ID.
 * Then it guarantees there will be no collisions with customer application */
#define LE_MLX_PATCH_ID 0x35u

/** Special Function ID used to get the chip version and revision in Application mode */
#define LE_MLX_FUNCTION_ID_GET_CHIP_VERSION 0xCABCu

#if HAS_PPM_EPM == 1
/** Special Function ID to enter PPM mode in Application mode */
#define LE_MLX_FUNCTION_ID_ENTER_PPM_MODE 0xCABDu
#endif

#if (HAS_LIN_EPM == 1) || defined (HAS_MLS_LOADER)
/** Special Function ID to initialize unlocking key sending in Application mode */
#define LE_MLX_FUNCTION_ID_ENTER_LIN_PROG_MODE 0xCABEu
#endif

/** Broadcast LIN NAD */
#define LE_BROADCAST_NAD 0x7Fu


/* ---------------------------------------------
 * Variables
 * --------------------------------------------- */

/** Shows that MLS device ID specific response transmit event is expected */
STATIC bool isAwaitingResponseTransmitted;

#if HAS_PPM_EPM == 1
/** Shows that request to enter PPM mode */
STATIC bool lepmFlag;
#endif


/* ---------------------------------------------
 * Local Function Declarations
 * --------------------------------------------- */

/** Callout function.
 * Ask application permission to enter programming mode.
 * This is default implementation which always does permit.
 * @return bool:
 *  - true - application is stopped and programming mode can be entered
 *  - false - programming mode entering isn't possible
 */
__attribute__((weak)) bool fw_lepm_ApplicationStop(void);

/** LIN TL handler.
 * Prepare the response with product(chip) version + revision.
 * @param[in] transfer  LIN TL transmission buffer
 */
STATIC void fw_lepm_PrepareProdResponse(LINDiagTransfer_t* transfer);

/** LIN TL handler.
 * Process LIN TL Request event.
 * @param[in] transfer  LIN TL transmission buffer
 * @return bool:
 *  - true - event has been processed by current handler
 *  - false - allow next subscribed handler to process the event
 */
STATIC bool fw_lepm_Request(LINDiagTransfer_t* transfer);

/** LIN TL handler.
 * Process LIN TL Response Transmitted event.
 * @param[in] transfer  LIN TL transmission buffer
 * @return bool:
 *  - true - event has been processed by current handler
 *  - false - allow next subscribed handler to process the event
 */
STATIC bool fw_lepm_ResponseTransmitted(LINDiagTransfer_t* transfer);

/** LIN TL handler.
 * Prepare the response with platform revision..
 * @param[in] transfer  LIN TL transmission buffer
 */
STATIC void fw_lepm_PreparePltfRevResponse(LINDiagTransfer_t* transfer);

/** LIN TL handler.
 * Prepare the response with patch identifier.
 * @param[in] transfer  LIN TL transmission buffer
 */
STATIC void fw_lepm_PreparePatchRevResponse(LINDiagTransfer_t* transfer);

/** get the Func.ID for both Empty Flash and non-empty Flash modes */
STATIC INLINE uint16_t fw_lepm_GetFuncIdAllModes(uint16_t curr_mode_func_id);

/* ---------------------------------------------
 * Function Definitions
 * --------------------------------------------- */

/** get the Func.ID for both Empty Flash and non-empty Flash modes */
STATIC INLINE uint16_t fw_lepm_GetFuncIdAllModes(uint16_t curr_mode_func_id)
{
    return (curr_mode_func_id | (3u << 5));
}

void fw_lepm_Init(void)
{
    /* LIN TL */
    (void)ldt_SubscribeMultipleHandler(&ld_RequestMultipleHandler, fw_lepm_Request);
    (void)ldt_SubscribeMultipleHandler(&ld_ResponseTransmittedMultipleHandler, fw_lepm_ResponseTransmitted);

    /* LIN EPM specific */
#if HAS_PPM_EPM == 1
    lepmFlag = false;
#endif
}

bool fw_lepm_ApplicationStop(void)
{
    /* Always allow enter programming mode by default */
    return true;
}

STATIC bool fw_lepm_Request(LINDiagTransfer_t* transfer)
{
    bool isAnswerExpected = false;

    /* Supported requests:
     * - Read by ID=LE_MLX_SPECIAL_ID, LE_MLX_SUPPLIER_ID, LE_MLX_FUNCTION_ID_GET_CHIP_VERSION (no broadcasts supported);
     *   returns PRODUCT_VERSION_32
     *
     * - Read by ID=LE_MLX_SPECIAL_ID, LE_MLX_SUPPLIER_ID, LE_MLX_FUNCTION_ID_ENTER_PPM_MODE (no broadcasts supported);
     *   a. enters PPM mode
     *   b. returns PRODUCT_VERSION_32
     *
     * - Read by ID=LE_MLX_SPECIAL_ID, LE_MLX_SUPPLIER_ID, LE_MLX_FUNCTION_ID_ENTER_LIN_PROG_MODE (no broadcasts supported);
     *   a. enters LIN Programming mode
     *   b. returns PRODUCT_VERSION_32
     *
     * - Read by ID=LE_MLX_SPECIAL_ID, LE_MLX_SUPPLIER_ID, LE_MLX_FUNCTION_ID_INIT_UNLOCKING_KEY_RX (no broadcasts supported);
     *   a. allows to send unlocking key with "Data Dump" message (SID=0xB4). The "Data Dump" message with 64-bit unlocking
     *      key inside should go right after this message. Otherwise this message should be resent again.
     *   b. returns PRODUCT_VERSION_32
     *
     * - Read by ID=LE_MLX_PLTF_ID, LE_MLX_SUPPLIER_ID, LE_MLX_FUNCTION_ID_GET_CHIP_VERSION (no broadcasts supported);
     *   returns PLTF_VERSION + TYPE
     *
     * - Read by ID=LE_MLX_PATCH_ID, LE_MLX_SUPPLIER_ID, LE_MLX_FUNCTION_ID_GET_CHIP_VERSION (no broadcasts supported);
     *   returns patch identification
     */
    if (transfer->request.reqSId == LE_READ_BY_ID_SID) {
        if (transfer->request.dataLen <= LE_MIN_RBI_DATA_LEN) {

            fw_lepm_ReadByIdProdIdReq_t* pProdIdReq = (fw_lepm_ReadByIdProdIdReq_t*)transfer->request.data;

            if ((pProdIdReq->supp_id == LE_MLX_SUPPLIER_ID)) {
                switch (pProdIdReq->id) {
                    case LE_MLX_SPECIAL_ID:
                        if ((pProdIdReq->func_id == LE_MLX_FUNCTION_ID_GET_CHIP_VERSION)
                            || (pProdIdReq->func_id ==
                                fw_lepm_GetFuncIdAllModes(LE_MLX_FUNCTION_ID_GET_CHIP_VERSION))) {
                            /* Prepare response with product identifiers */
                            fw_lepm_PrepareProdResponse(transfer);
                            isAnswerExpected = true;
#if HAS_PPM_EPM == 1
                        } else if ((pProdIdReq->func_id == LE_MLX_FUNCTION_ID_ENTER_PPM_MODE)
                                   || (pProdIdReq->func_id ==
                                       fw_lepm_GetFuncIdAllModes(LE_MLX_FUNCTION_ID_ENTER_PPM_MODE))) {
                            if (fw_lepm_ApplicationStop()) {
                                /* Set PPM mode flag */
                                lepmFlag = true;

                                /* Prepare response with product identifiers */
                                fw_lepm_PrepareProdResponse(transfer);
                                isAnswerExpected = true;
                            }
#endif /* HAS_PPM_EPM */

#if HAS_LIN_EPM == 1
                        } else if ((pProdIdReq->func_id == LE_MLX_FUNCTION_ID_ENTER_LIN_PROG_MODE)
                                   || (pProdIdReq->func_id ==
                                       fw_lepm_GetFuncIdAllModes(LE_MLX_FUNCTION_ID_ENTER_LIN_PROG_MODE))) {
                            if (fw_lepm_ApplicationStop()) {
                                /* Save the NAD */
                                bl_table.NAD = plinEventTable->ldt_GetLINNAD(LE_BROADCAST_NAD);

                                /* Do reset w/o waiting for response transmit */
                                MLX16_RESET_SIGNED(C_CHIP_STATE_LIN_CMD_EPM);
                            }
#endif /* HAS_LIN_EPM */

#if defined (HAS_MLS_LOADER)
                        } else if ((pProdIdReq->func_id == LE_MLX_FUNCTION_ID_ENTER_LIN_PROG_MODE)
                                   || (pProdIdReq->func_id ==
                                       fw_lepm_GetFuncIdAllModes(LE_MLX_FUNCTION_ID_ENTER_LIN_PROG_MODE))) {
                            if (ldr_HandleEnterProgMode(transfer)) {
                                /* LIN programming mode is already entered */
                                isAnswerExpected = true;
                            }
#endif /* HAS_MLS_LOADER */

                        } else {
                            /* ignore unknown Function ID value */
                        }
                        break;

                    case LE_MLX_PLTF_ID:
                        if ((pProdIdReq->func_id == LE_MLX_FUNCTION_ID_GET_CHIP_VERSION)
                            || (pProdIdReq->func_id ==
                                fw_lepm_GetFuncIdAllModes(LE_MLX_FUNCTION_ID_GET_CHIP_VERSION))) {
                            /* Prepare response with platform version */
                            fw_lepm_PreparePltfRevResponse(transfer);
                            isAnswerExpected = true;
                        } else {
                            /* ignore unknown Function ID value */
                        }
                        break;

                    case LE_MLX_PATCH_ID:
                        if ((pProdIdReq->func_id == LE_MLX_FUNCTION_ID_GET_CHIP_VERSION)
                            || (pProdIdReq->func_id ==
                                fw_lepm_GetFuncIdAllModes(LE_MLX_FUNCTION_ID_GET_CHIP_VERSION))) {
                            fw_lepm_PreparePatchRevResponse(transfer);
                            isAnswerExpected = true;
                        } else {
                            /* ignore unknown Function ID value */
                        }
                        break;

                    default:
                        /* ignore unknown ID value for Read by ID */
                        break;
                }
            }
        } else {
            /* ignore non-full Read by ID message */
        }
    } else {
        /* ignore other diagnostic messages */
    }

    /* If MLS device ID response is prepared then it's true;
     * in the case of new application specific request after - it's false */
    isAwaitingResponseTransmitted = isAnswerExpected;
    return isAnswerExpected;
}

STATIC bool fw_lepm_ResponseTransmitted(LINDiagTransfer_t* transfer)
{
    (void)transfer;

#if HAS_PPM_EPM == 1
    if (lepmFlag == true) {
        MLX16_RESET_SIGNED((uint16_t)C_CHIP_STATE_PPM_CMD_EPM);
    }
#endif

    if (isAwaitingResponseTransmitted == true) {
        /* Don't deliver this event to next LIN TL subscribers
         * while it's MLS device ID specific one */
        return true;
    } else {
        /* The event is intended for the next LIN TL subscribers */
        return false;
    }
}

STATIC void fw_lepm_PrepareProdResponse(LINDiagTransfer_t* transfer)
{
    /* Use Response SID */
    transfer->response.respSId = LE_READ_BY_ID_RSID;
    transfer->response.dataLen = 5u;

    /* We can't guarantee that transfer->response.data is aligned to the 32-bits address */
    fw_lepm_ProdIdRes_t* pDeviceVerRev = (fw_lepm_ProdIdRes_t*)transfer->response.data;
    pDeviceVerRev->prod_id_low = (uint16_t)PRODUCT_VERSION_32;
    pDeviceVerRev->prod_id_high = (uint16_t)(PRODUCT_VERSION_32 >> 16);
    pDeviceVerRev->reserved = 0xFFu; /* it's reserved */
}

STATIC void fw_lepm_PreparePltfRevResponse(LINDiagTransfer_t* transfer)
{
    /* Use Response SID */
    transfer->response.respSId = LE_READ_BY_ID_RSID;
    transfer->response.dataLen = LE_PLTF_REV_RES_DATA_LENGTH;

    /* We can't guarantee that transfer->response.data is aligned to the 32-bits address */
    fw_lepm_PltfRevRes_t* pPltfRevResp = (fw_lepm_PltfRevRes_t*)transfer->response.data;
    pPltfRevResp->major = RELEASE_MAJOR;
    pPltfRevResp->minor = RELEASE_MINOR;
    pPltfRevResp->patch = RELEASE_REVISION;
    pPltfRevResp->build = 0x00u;
    pPltfRevResp->type = 0x5;
}

STATIC void fw_lepm_PreparePatchRevResponse(LINDiagTransfer_t* transfer)
{
    /* Use Response SID */
    transfer->response.respSId = LE_READ_BY_ID_RSID;
    transfer->response.dataLen = LE_PATCH_REV_RES_DATA_LENGTH;

    /* We can't guarantee that transfer->response.data is aligned to the 32-bits address */
    fw_lepm_PatchRevRes_t* pPatchRevResp = (fw_lepm_PatchRevRes_t*)transfer->response.data;

    /* Return patch identifier only when patch is enabled by ROM */
    if (((IO_GET(MLX16, DBG_CONTROL0) & 0x3FFFu) == PATCH_CTRL_ENABLE) ||
        ((IO_GET(MLX16, DBG_CONTROL1) & 0x3FFFu) == PATCH_CTRL_ENABLE) ||
        ((IO_GET(MLX16, DBG_CONTROL2) & 0x3FFFu) == PATCH_CTRL_ENABLE)) {
        pPatchRevResp->identifier = EE_HOST(PATCH_CRC);
    } else {
        pPatchRevResp->identifier = 0xFFFFu;
    }
}

