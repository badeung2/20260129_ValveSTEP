/**
 * @file
 * @brief PPM HW low-level library
 * @internal
 *
 * @copyright (C) 2017 Melexis N.V.
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
 * @ingroup ppm_phy
 *
 * @brief PPM HW low-level library
 * @details
 * @{
 */

#ifndef PPM_LIB_H
#define PPM_LIB_H

#include <syslib.h>
#include "sys_tools.h"
#include "lib_miscio.h"

/** PPM Receiver or Transmitter mode [PPM_TX_RX] */
typedef enum {
    ppm_RX_Mode = 0u,   /**< Receiver mode PPM */
    ppm_TX_Mode = 1u,   /**< Transmitter mode PPM */
} ppm_ModeType;

/** Specifies how much data will be received before the Interrupt comes */
typedef enum {
    ppm_DataInt_1Word = 0u, /**< Interrupt after every 8 pulses */
    ppm_DataInt_2Words,     /**< Interrupt after every 16 pulses */
    ppm_DataInt_4Words,     /**< Interrupt after every 32 pulses */
    ppm_DataInt_8Words,     /**< Interrupt after every 64 pulses */
} ppm_DataBufferInt;

/** PPM pulses' data mode */
typedef enum {
    ppm_DataMode_TotalOnly = 0u,  /**< 0: TOTAL times only are stored in RAM */
    ppm_DataMode_All = 1u,        /**< 1: both ON and TOTAL times stored in RAM */
} ppm_DataMode;

/** PPM filtering control */
typedef enum {
    ppm_FilterEnable = 0u,        /**< Enable the filter */
    ppm_FilterDisable = 1u,       /**< Disable the filter */
} ppm_Filt_Bypass;

/** PPM Reference edge selection */
typedef enum {
    ppm_RE_FE_Falling = 0u,       /**< The PPM timer counts the periods starting from the falling edge */
    ppm_RE_FE_Rising = 1u,        /**< The PPM timer counts the periods starting from the rising edge */
} ppm_Reference;

/** PPM Active state definition */
typedef enum {
    ppm_Disable = 0u,             /**< PPM is disabled */
    ppm_Enable = 1u,              /**< PPM is enabled */
} ppm_ActiveState;

/** Initiates the low-level PPM Physical interface for the PPM bootloader
 *
 * The function run the PPM HW timer in RX mode with filtering enabled and initiates the analog multiplexors
 * to fulfill the slew-rate requirements and the RX bypass mode to be able to transfer and recieve the data on high
 * frequencies.
 *
 * @param[in] ppmPin PPM input pin selector (index of ppm_InputPin)
 * @param[in] TX_buf TX buffer for the transmission mode
 * @param[in] RX_buf RX buffer for incoming pulses lengths
 * @param[in] timeout The timer's timeout value
 */
void ppm_ll_InitPin(ppm_InputPin ppmPin, const uint16_t* TX_buf, const volatile uint16_t* RX_buf,
                    const uint16_t timeout);

#define ppm_ll_Init ppm_ll_InitPin __attribute__ ((deprecated( \
                                                       "Renamed to ppm_ll_InitPin with additional argument to select PPM pin")))

/** Stop the PPM timer and restores all the multiplexor and slew-rate to default values */
void ppm_ll_Deinit(void);

/** Start the PPM interface and enables RX, TX and ERR Interrupts */
void ppm_ll_Start(void);

/** Stop the PPM interface and disables RX, TX and ERR Interrupts */
void ppm_ll_Stop(void);

/** Switches the PPM HW to Transmission mode and starts the transmission from TX buffer */
void ppm_ll_StartTx(void);

/** Get the latest timer value
 * @return Last timer value received
 */
uint16_t ppm_ll_getLastData(void);

/** Get the PPM overflow flag status
 * @retval 1 TX is transmitted or RX is overflowed, according to the current mode
 * @retval 0 no overflow state
 */
uint16_t ppm_ll_OvfState(void);

/** Get the PPM timeout flag status
 * @retval 1 PPM timeout occured
 * @retval 0 PPM timeout not occured
 */
uint16_t ppm_ll_TimeoutState(void);

#endif /* PPM_LIB_H */
/// @}
