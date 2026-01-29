/**
 * @file
 * @brief Header file for CXPI Data Link Layer library
 * @internal
 *
 * @copyright (C) 2015-2017 Melexis N.V.
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
 * @ingroup cxpi
 * @addtogroup cxpi CXPI Data Link Layer
 *
 * @brief CXPI Data Link Layer library
 * @details
 *
 * @{
 */
#ifndef CXPI_DLL_H
#define CXPI_DLL_H

#include <static_assert.h>
#include "cxpi_hal.h"


#define CALLBACK_FUNC_ARRARY_SIZE   (128u)  // for all possible PID's including PTYPE and pre-assigned PID's
#define CXPI_NORMAL_FRAME_LEN       (15u)
#define CXPI_NORMAL_FRAME_DATA_LEN  (12u)

/*-------------------------------------
        Data Types Definition
   -------------------------------------*/

/** CXPI DLL return code enumeration */
typedef enum CxpiDll_ReturnCode_e {
    CXPIDLL_RET_OK = 0,
    CXPIDLL_RET_UNKNOWN_ERROR
} CxpiDll_ReturnCode_Type;

/** CXPI DLL frame type structure */
typedef union CxpiDll_Frame_u {
    uint16_t as_word[(CXPI_NORMAL_FRAME_LEN - 1u) >> 1u];   /**< for faster data copy */
    struct {
        uint8_t pid;                                /**< Protected ID */
        uint8_t ct              : 2;                /**< Frame counter */
        uint8_t sleep_ind       : 1;                /**< Sleep flag */
        uint8_t wakeup_ind      : 1;                /**< Wakeup flag */
        uint8_t dlc             : 4;                /**< Data length code */
        uint8_t data[CXPI_NORMAL_FRAME_DATA_LEN];   /**< Frame data field */
    };
} CxpiDll_Frame_Type;
ASSERT( sizeof(CxpiDll_Frame_Type) == 14u );

/** CXPI DLL status flags structure */
typedef union CxpiDll_Status_u {
    uint16_t all_statuses;                  /**< All status flags as word */
    struct {
        uint16_t sleep_mode_st      : 1;    /**< CXPI node state -- Sleep state */ /* lsb */
        uint16_t standby_mode_st    : 1;    /**< CXPI node state -- Standby state */
        uint16_t normal_mode_st     : 1;    /**< CXPI node state -- Normal state */
        uint16_t idle_st            : 1;    /**< CXPI DLL in Idle state */
        uint16_t busy_st            : 1;    /**< CXPI DLL busy */
        uint16_t rx_ongoing_st      : 1;    /**< CXPI DLL state -- receiving */
        uint16_t tx_ongoing_st      : 1;    /**< CXPI DLL state -- transmitting */
        uint16_t rx_buf_ready_st    : 1;    /**< CXPI DLL receive buffer ready */
        uint16_t tx_request_st      : 1;    /**< Transmit request from API */
        uint16_t event_tx_st        : 1;    /**< Transmission of Event frame ongoing */
        uint16_t ptype_request_st   : 1;    /**< PTYPE request received */
        uint16_t /* reserved */ : 5;
    };
} CxpiDll_Status_Type;
ASSERT( sizeof(CxpiDll_Status_Type) == 2u );

/** CXPI DLL error flags structure */
typedef union CxpiDll_ErrorCode_u {
    uint16_t all_errors;
    struct {
        uint16_t bit_er             : 1;    /**< bit0 - Arbitration lost during transmitting */ /* lsb */
        uint16_t crc_er             : 1;    /**< bit1 - CRC missmatch */
        uint16_t comm_bus_er        : 1;    /**< bit2 - Communication bus error. No messages during last 4 sec */
        uint16_t data_len_er        : 1;    /**< bit3 - Actually received bytes is not equal to DLC field */
        uint16_t counter_er         : 1;    /**< bit4 - Counter field error(optional) */
        uint16_t framing_er         : 1;    /**< bit5 - Stop bit not equal to log.'0' */
        uint16_t /* reserved */ : 1;        /**< bit6 */
        uint16_t /* reserved */ : 1;        /**< bit7 */
        uint16_t parity_er          : 1;    /**< bit8 - Parity bit error */
        uint16_t rx_buf_ovr_er      : 1;    /**< bit9 - Receive buffer overrun. */
        uint16_t bus_sync_lost_er   : 1;    /**< bit10 - Bus synchronisation lost -- last 10 bits are log.'0' */
        uint16_t dll_unknown_er     : 1;    /**< bit11 - Data Link Layer Unknown error */
        uint16_t hal_unknown_er     : 1;    /**< bit12 - Hardware Abstraction Layer Unknown error */
        uint16_t callback_ptr_er    : 1;    /**< bit13 - Wrong callback pointer for user callback function */
        uint16_t unexpctd_evnt_er   : 1;    /**< bit14 - Unexpected event received from HAL */
        uint16_t /* reserved */ : 1;        /**< bit15 */
    };
} CxpiDll_ErrorCode_Type;
ASSERT( sizeof(CxpiDll_ErrorCode_Type) == 2u );

/** CXPI DLL bus access method enumeration */
typedef enum CxpiDll_BusAccessMethod_e {
    CXPIDLL_METHOD_UNKNOWN = 0,             /*!< CXPI access method unset */
    CXPIDLL_METHOD_EVENT_TRIGGER = 1u,      /*!< Event Trigger Method selected JASO D 015-3:2015 chapter 5.1.3.1 */
    CXPIDLL_METHOD_POLLING = 3u             /*!< Polling Method selected JASO D 015-3:2015 chapter 5.1.3.2 */
} CxpiDll_BusAccessMethod_Type;

/** CXPI Data Link Layer configuration structure */
typedef struct CxpiDll_Configuration_s {
    CxpiHal_BaudRateConfig_Type baudrate;
    CxpiDll_BusAccessMethod_Type access_method;
    CxpiHal_IsrCallingPrio_Type isr_prio;
} CxpiDll_Configuration_Type;
ASSERT( sizeof(CxpiDll_Configuration_Type) == 6u );

/** CXPI DLL message direction enumeration */
typedef enum CxpiDll_MessageDirection_e {
    CXPIDLL_MSG_TYPE_UNKNOWN = 0,
    CXPIDLL_MSG_TYPE_M2S = 1,           /*!< Master to Slave message -- receive whole frame */
    CXPIDLL_MSG_TYPE_S2M = 2            /*!< Slave to Master message -- respond should be send after PID field */
} CxpiDll_MessageDirection_Type;
ASSERT( sizeof(CxpiDll_MessageDirection_Type) == 2u );

/** Callback function pointer type definition */
typedef void (* CxpiDll_FuncPtr_Type)(void);

/** CXPI DLL callback configuration struct */
typedef struct CxpiDll_CallbackConfig_s {
    uint16_t pid;
    CxpiDll_MessageDirection_Type msgtype;
    CxpiDll_FuncPtr_Type fptr;
} CxpiDll_CallbackConfig_Type;
ASSERT( sizeof(CxpiDll_CallbackConfig_Type) == 6u );

/*-------------------------------------
        Function prototypes
   -------------------------------------*/
CxpiDll_ReturnCode_Type CxpiDll_Init(const CxpiDll_Configuration_Type* const cxpi_dll_cfg);
void CxpiDll_DeInit(void);     // TODO: add different deInit levels to support sleep mode.
CxpiDll_ReturnCode_Type CxpiDll_ConfigPidCallback(const CxpiDll_CallbackConfig_Type* const cConfig);
CxpiDll_ReturnCode_Type CxpiDll_TxMessage(const CxpiDll_Frame_Type* const ptrMsg);
CxpiDll_ReturnCode_Type CxpiDll_TxResponse(const CxpiDll_Frame_Type* const ptrMsg);
void CxpiDll_TxCompleteCallback(void);
void CxpiDll_RxCompleteCallback(void);
void CxpiDll_ErrorCallback(void);
CxpiDll_Status_Type CxpiDll_GetState(void);
CxpiDll_ErrorCode_Type CxpiDll_GetError(void);
CxpiDll_ReturnCode_Type CxpiDll_GetMessage(CxpiDll_Frame_Type* ptrMsg);

/// @}
#endif /* CXPI_DLL_H */
/* EOF */
