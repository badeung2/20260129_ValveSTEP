/**
 * @file
 * @brief Header file for CXPI Hardware Abstraction Layer
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
 *
 * @brief CXPI Hardware Abstraction Layer
 * @details
 *
 * @{
 */
#ifndef CXPI_HAL_H
#define CXPI_HAL_H

#include <stdint.h>
#include <static_assert.h>

/*-------------------------------------
 *          Public definitions
 *-------------------------------------*/

/* LIN XCFG register fields */
/* TODO: move LIN shell configuration into CXPI HAL */
/* TODO: move that definitions in cxpi_hal_private.h */
/* TODO: add description to each definition */
#define XCFG_SEL_TX_EXT         ((uint16_t)1 << 0u)
#define XCFG_TX_INVERT          ((uint16_t)1 << 1u)
#define XCFG_SEL_RX_IO          ((uint16_t)1 << 2u)
#define XCFG_SEL_IO_TO_COLINRX  ((uint16_t)1 << 3u)
#define XCFG_SEL_COLIN_B        ((uint16_t)1 << 4u)
#define XCFG_SLEEPB             ((uint16_t)1 << 5u)
#define XCFG_LSM                ((uint16_t)1 << 6u)
#define XCFG_HSM                ((uint16_t)1 << 7u)
#define XCFG_BYPASS             ((uint16_t)1 << 8u)
#define XCFG_DISTERM            ((uint16_t)1 << 9u)
#define XCFG_RX_INVERT          ((uint16_t)1 << 10u)

/*-------------------------------------
 *          Public types
 *-------------------------------------*/

/**
 *  @brief CXPI HAL return type.
 */
typedef enum {
    CXPIHAL_RET_OK = 0u,
    CXPIHAL_RET_TX_ONGOING_ER,
    CXPIHAL_RET_RX_ONGOING_ER,
    CXPIHAL_RET_UNKNOWN_ERROR
} CxpiHal_ReturnCode_Type;
ASSERT( sizeof(CxpiHal_ReturnCode_Type) == 2u );

/**
 *  @brief CXPI HAL statuses
 */
typedef union {
    uint16_t all_statuses;
    struct {   /* statuses */
        uint16_t bus_idle_st       : 1u; /**< bit0 -- Idle state detected on the Bus */
        uint16_t rx_ongoing_st     : 1u; /**< bit1 -- Receiving process executing */
        uint16_t tx_ongoing_st     : 1u; /**< bit2 -- Transmitting process executing */
        uint16_t rx_buf_rdy_st     : 1u; /**< bit3 -- Receive buffer ready to receive new data */
        uint16_t tx_request_st     : 1u; /**< bit4 -- Request to transmitt data from upper level */
        uint16_t autobaudrate_st   : 1u; /**< bit5 -- Autobaudrate activated */
        uint16_t /* reserved */ : 1u;    /*   bit6 */
        uint16_t /* reserved */ : 1u;    /*   bit7 */
        uint16_t /* reserved */ : 1u;    /*   bit8 */
        uint16_t /* reserved */ : 1u;    /*   bit9 */
        uint16_t /* reserved */ : 1u;    /*   bit10 */
        uint16_t /* reserved */ : 1u;    /*   bit11 */
        uint16_t /* reserved */ : 1u;    /*   bit12 */
        uint16_t /* reserved */ : 1u;    /*   bit13 */
        uint16_t /* reserved */ : 1u;    /*   bit14 */
        uint16_t /* reserved */ : 1u;    /*   bit15 */
    } bits;
} CxpiHal_Status_Type;
ASSERT( sizeof(CxpiHal_Status_Type) == 2u );

/**
 *  @brief CXPI HAL Events and Errors reporting
 */
typedef union {
    uint16_t all_fields;
    union {
        struct {
            uint16_t all_events : 8u;    /**< offset for all events */
            uint16_t all_errors : 8u;    /**< offset for all errors */
        } bytes;
        struct {
            /* events */
            uint16_t rx_complete_ev    : 1u; /**< bit0 -- Receive complete event */
            uint16_t tx_complete_ev    : 1u; /**< bit1 -- Transmit complete event */
            uint16_t ibs_exceedance_ev : 1u; /**< bit2 -- IBS exceedance event */
            uint16_t ifs_detection_ev  : 1u; /**< bit3 -- IFS detection event */
            uint16_t /* reserved */ : 1u;    /*   bit4 */
            uint16_t /* reserved */ : 1u;    /*   bit5 */
            uint16_t /* reserved */ : 1u;    /*   bit6 */
            uint16_t /* reserved */ : 1u;    /*   bit7 */
            /* errors */
            uint16_t rx_buf_ovr_er     : 1u; /**< bit8  -- Receive buffer overrun error */
            uint16_t stop_bit_er       : 1u; /**< bit9  -- Stop bit error */
            uint16_t start_bit_er      : 1u; /**< bit10 -- Start bit error */
            uint16_t sync_lost_er      : 1u; /**< bit11 -- Synchronization error. Last 10 bits == log.'0' */
            uint16_t arbitration_er    : 1u; /**< bit12 -- Arbitration error. Arbitration lost during transmission */
            uint16_t bus_er            : 1u; /**< bit13 -- Communication bus error */
            uint16_t config_er         : 1u; /**< bit14 -- Configuration error */
            uint16_t fatal_er          : 1u; /**< bit15 -- Internal fatal error */
        } bits;
    } u;
} CxpiHal_EventsErrors_Type;
ASSERT( sizeof(CxpiHal_EventsErrors_Type) == 2u );

/**
 *  @brief Interrupt priority configuration for CXPI HAL (CTimer interrupts)
 */
typedef enum {
    CXPI_HAL_ISR_PRIO_UNSET = 0u,    /**< calling priority is unset */
    CXPI_HAL_ISR_PRIO_3 = 3u,        /**< calling priority **level 3** */
    CXPI_HAL_ISR_PRIO_4 = 4u,        /**< calling priority **level 4** */
    CXPI_HAL_ISR_PRIO_5 = 5u,        /**< calling priority **level 5** */
    CXPI_HAL_ISR_PRIO_6 = 6u         /**< calling priority **level 6** */
} CxpiHal_IsrCallingPrio_Type;
ASSERT( sizeof(CxpiHal_IsrCallingPrio_Type) == 2u );

/**
 *  @brief CXPI HAL Baudrate configuration
 */
typedef enum {
    CXPI_HAL_BAUD_AUTOBAUD = 0u,      /**< Use Autobaud option */
    CXPI_HAL_BAUD_20000 = 50u,        /**< Bittime of 20000 Baud CXPI (in us) */
    CXPI_HAL_BAUD_19200 = 52u,        /**< Bittime of 19200 Baud CXPI (in us) */
    CXPI_HAL_BAUD_14400 = 69u,        /**< Bittime of 14400 Baud CXPI (in us) */
    CXPI_HAL_BAUD_9600 = 104u,        /**< Bittime of 9600 Baud CXPI (in us) */
    CXPI_HAL_BAUD_7200 = 139u,        /**< Bittime of 7200 Baud CXPI (in us) */
    CXPI_HAL_BAUD_4800 = 208u,        /**< Bittime of 4800 Baud CXPI (in us) */
    CXPI_HAL_BAUD_2400 = 417u,        /**< Bittime of 2400 Baud CXPI (in us) */
    CXPI_HAL_BAUD_1800 = 556u,        /**< Bittime of 1800 Baud CXPI (in us) */
    CXPI_HAL_BAUD_1200 = 833u         /**< Bittime of 1200 Baud CXPI (in us) */
} CxpiHal_BaudRateConfig_Type;
ASSERT( sizeof(CxpiHal_BaudRateConfig_Type) == 2u );

/**
 *  @brief CXPI HAL configuration structure
 */
typedef struct {
    CxpiHal_BaudRateConfig_Type baudrate;       /**< CXPI baudrate configuration */
    CxpiHal_IsrCallingPrio_Type hal_isr_prio;   /**< Interrupt priorities for CXPI interface handling */
    uint16_t ctimer_clock_mhz;                  /**< CTimer clock speed in MHz */
    uint16_t bus_err_judgment_time_ms;          /**< Judgment time to rise Communication Bus error in msec */
} CxpiHal_Configuration_Type;

/*-------------------------------------
 *          Public functions
 *-------------------------------------*/

/**
 * CXPI HAL layer initialization.
 *
 * Configure Complex Timer 0, activate CTimer0 interrupts, initialize variables.
 *
 * @param[in]  cxpi_hal_cfg  CXPI HAL configuration structure
 *
 * @return     none
 */
void CxpiHal_Init(const CxpiHal_Configuration_Type* const cxpi_hal_cfg);

/**
 * CXPI HAL de-initialization.
 *
 * Stop CTimer0 and disable its interrupts.
 *
 * @return     none
 */
void CxpiHal_DeInit(void);

/**
 * CXPI HAL get byte.
 *
 * Return last received byte by CXPI HAL and set Rx Buff ready flag.
 *
 * @return     Received byte
 */
uint8_t CxpiHal_GetByte(void);

/**
 * CXPI HAL get state.
 *
 * Return currentstate of CXPI HAL. All events and errors are cleared after reporting.
 *
 * @return     Events and Errors structure
 */
CxpiHal_EventsErrors_Type CxpiHal_GetState(void);

/**
 * CXPI HAL Tx byte.
 *
 * Set Tx request flag and set new data into Tx buff.
 *
 * @param[in]  byte  Byte to transmit via CXPI bus.
 *
 * @return     CxpiHal_ReturnCode_Type
 */
CxpiHal_ReturnCode_Type CxpiHal_TxByte(uint8_t byte);

/**
 * CXPI HAL return bit "1" threshold.
 *
 * @return     bit "1" threshold in timer ticks
 */
uint16_t CxpiHal_GetBit1Thr(void);

/**
 * CXPI HAL return bit "0" threshold.
 *
 * @return     bit "0" threshold in timer ticks
 */
uint16_t CxpiHal_GetBit0Thr(void);

/**
 * Set a new value of multiplier for bit "1" threshold calculations
 *
 * @param[in]   new_val new value of bit_1_thr_mul
 *
 * @return      none
 */
void CxpiHal_SetBitOneThresholdMul(uint16_t new_val);

/**
 * Set a new value of divider for bit "1" threshold calculations
 *
 * @param[in]   new_val new value of bit_1_thr_div
 *
 * @return      none
 */
void CxpiHal_SetBitOneThresholdDiv(uint16_t new_val);

/**
 * Set a new value of multiplier for bit "0" threshold calculations
 *
 * @param[in]   new_val new value of bit_0_thr_mul
 *
 * @return      none
 */
void CxpiHal_SetBitZeroThresholdMul(uint16_t new_val);

/**
 * Set a new value of divider for bit "0" threshold calculations
 *
 * @param[in]   new_val new value of bit_0_thr_div
 *
 * @return      none
 */
void CxpiHal_SetBitZeroThresholdDiv(uint16_t new_val);

/// @}
#endif /* CXPI_HAL_H */
/* EOF */
