/**
 * @file
 * @brief ADC support library
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
 * @ingroup adc
 * @addtogroup adc ADC
 * @ingroup peripheral_drivers
 *
 * @brief The ADC SAR support library
 * @details
 * @{
 */

#ifndef LIB_ADC_H
#define LIB_ADC_H

#include <syslib.h>
#include "sys_tools.h"
#include "plib.h"
#include "io.h"
#include "adc_refs.h"

/** ADC reference configuration */
typedef enum {
    ADC_REF_PORTS = 0u,                  /**< ADC_TRIMPORT will be connected from ports */
    ADC_REF_PINS,                        /**< ADC_TRIMPORT will be connected to TR_ADCREF pins */
} AdcRef_t;

/** Types of Start-Of-Sequence */
typedef enum {
    ADC_SOS_SECOND_HARDWARE_TRIGGER = 0u,/**< Second hardware trigger mode */
    ADC_SOS_FIRST_HARDWARE_TRIGGER,      /**< First hardware trigger mode */
    ADC_SOS_FIRMWARE_TRIGGER,            /**< Firmware trigger mode */
    ADC_SOS_PERMANENT_TRIGGER,           /**< Permanent trigger mode */
} AdcSosSource_t;
#define AdcSOS_Source_t AdcSosSource_t __attribute__ ((deprecated("Replaced by AdcSosSource_t")))


/** Types of Start-Of-Conversion */
typedef enum {
    ADC_SOC_HARDWARE_TRIGGER = 0u,       /**< Hardware SOC */
    ADC_SOC_FIRMWARE_TRIGGER,            /**< Firmware SOC */
    ADC_SOC_HARDWARE_FW_TRIGGER,         /**< Hardware + Software SOC */
    ADC_SOC_PERMANENT_TRIGGER,           /**< Permanent SOC */
} AdcSocSource_t;

/** Types of ADC Interrupts */
typedef enum {
    ADC_INT_NO_INT = 0u,                 /**< No Interrupts */
    ADC_INT_EOC,                         /**< End of conversion Interrupt */
    ADC_INT_EOF,                         /**< End of Frame Interrupt */
    ADC_INT_EOS,                         /**< End of Sequence Interrupt */
} AdcIntScheme_t;

/** ADC StandBy configuration types */
typedef enum {
    ADC_ASB_NOT_USED = 0u,               /**< Auto-StandBy is used when ADC not used. */
    ADC_ASB_WAIT_TRIGGERS,               /**< Auto-StandBy is used when waiting for triggers. */
    ADC_ASB_NEVER,                       /**< Never use StandBy */
} AdcAutoStandby_t;

/** ADC Data width types */
typedef enum {
    ADC_WDT_16Bit = 0u,                  /**< ADC data width is 16bit */
    ADC_WDT_32Bit,                       /**< ADC data width is 32bit */
} AdcWidth_t;

/** ADC Configuration and modes type */
typedef union AdcControl_u {
    struct  __attribute__((packed))
    BitControl {
        uint16_t start : 1;                 /**< START command bit */
        uint16_t stop : 1;                  /**< STOP command bit */
        AdcSosSource_t sosSource : 2;       /**< Start-Of-Sequence source */
        AdcSocSource_t socSource : 2;       /**< Start-Of-Conversion source */
        uint16_t noInterleave : 1;          /**< 1 - when EOC triggers SDATA update, When 0 - EOA triggers SDATA update */
        uint16_t saturate : 1;              /**< 1 - data saturated to 2^N-1 in case of overflow (for N bit in DATA) and 0 in undeflow; 0 - DATA is garbage in under-/over-flow  */
        AdcIntScheme_t intScheme : 2;       /**< Interrupts Scheme */
        AdcAutoStandby_t asb : 2;           /**< Auto StandBy Mode */
        AdcWidth_t adcWidth : 1;            /**< ADC convertions width */
    } s;
    uint16_t data; /**< Grouped field to write the IO-port at once */
} AdcControl_t;
#define Adc_Control_t AdcControl_t __attribute__ ((deprecated("Replaced by AdcControl_t")))

/** ADC phase type definition */
typedef enum {
    ADC_PHASE_IDLE = 0u,                     /**< ADC no operation */
    ADC_PHASE_MEM_TRANSFER,                  /**< ADC is transferring the data */
    ADC_PHASE_CONVERSION,                    /**< ADC performs the conversion */
    ADC_PHASE_WAIT_TRIGGER,                  /**< ADC waits a trigger */
} AdcPhaseState_t;

/** ADC statuses */
typedef union AdcState_u {
    struct  __attribute__((packed))
    BitState {
        uint16_t paused : 1;                /**< Paused flag */
        uint16_t resumed : 1;               /**< Resumed flag */
        uint16_t swTrig : 1;                /**< Firmware trigger control bit */
        uint16_t ready : 1;                 /**< Ready flag */
        AdcIntScheme_t lastIntSource : 2;   /**< The Last Interrupt source */
        AdcPhaseState_t state : 2;          /**< The current ADC phase */
        uint16_t adcOvf : 1;                /**< ADC Overflow flag */
        uint16_t adcErr : 1;                /**< ADC Error flag */
        uint16_t memErr : 1;                /**< ADC Memory error flag */
        uint16_t frameErr : 1;              /**< ADC Frame error flag */
        uint16_t aborted : 1;               /**< ADC aborted flag */
    } s;
    uint16_t data; /**< The grouped 16bit status interface to get statuses in scope */
} AdcState_t;

/** Configures ADC Module with SBase address and run one sequence, blocking function
 * @param[in] sBaseAddress Address of the ADC SBase to be run once
 */
void AdcSingleSeqRun(uint16_t sBaseAddress);

/** Getter for the current DAC output value [9:0]
 * @return The current DAC value.
 */
STATIC INLINE uint16_t AdcGetDac(void);

/** Configure the ADC
 * @param[in] AdcDiv the MCU clock divider.
 * @param[in] SBase the initial SBase offset.
 * @param[in] ctrl specifies the Sequence and Conversion sources, Interleave, Saturate and Interrupt scheme, StandBy mode
 */
STATIC INLINE void AdcInit(uint8_t AdcDiv, const void* SBase, AdcControl_t ctrl);

/** Start the ADC from any ADC mode */
STATIC INLINE void AdcStart(void);

/** Stop the ADC from any ADC mode */
STATIC INLINE void AdcStop(void);

/** Disable all ADC triggers */
STATIC INLINE void AdcPause(void);

/** Trigger the sequence/conversion by the SW */
STATIC INLINE void AdcSwTrigger(void);

/** Stop ADC and waits until the ADC will stop the sequence
 *
 * @note: Function does not react on AWD
 */
STATIC INLINE void AdcStopBlocking(void);

/** Enable the selected ADC triggers */
STATIC INLINE void AdcResume(void);

/** Getter for the current ADC's state
 * @return the current ADC's state
 */
STATIC INLINE AdcState_t AdcGetState(void);

/** Test if ADC is already running
 * @retval true if ADC is running
 * @retval false if ADC is stopped
 */
STATIC INLINE bool AdcIsBusy(void);

/** Clear all errors flags for the ADC module */
STATIC INLINE void AdcClearAllErrors(void);

#if !defined(UNITTEST)
#include "lib_adc_inline_impl.h"
#endif /* UNITTEST */

/// @}
#endif /* LIB_ADC_H */
