/**
 * @file
 * @brief ADC Extended Counter support library
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
 * @ingroup adc_ec
 * @addtogroup adc_ec ADC Extended Counter
 * @ingroup peripheral_drivers
 *
 * @brief The ADC Extended Counter (EC) support library
 * @details
 *
 * @{
 */

#ifndef LIB_ADC_EC_H
#define LIB_ADC_EC_H

#include <syslib.h>
#include "sys_tools.h"
#include "plib.h"
#include "io.h"
#include "adc_refs.h"

/** ADC EC reference configuration */
typedef enum {
    ADC_EC_REF_PORTS = 0u,                  /**< ADC_EC_TRIMPORT will be connected from ports */
    ADC_EC_REF_PINS,                        /**< ADC_EC_TRIMPORT will be connected to TR_ADCREF pins */
} AdcEcRef_t;

/** Types of Start-Of-Sequence */
typedef enum {
    ADC_EC_SOS_SECOND_HARDWARE_TRIGGER = 0u,/**< Second hardware trigger mode */
    ADC_EC_SOS_FIRST_HARDWARE_TRIGGER,      /**< First hardware trigger mode */
    ADC_EC_SOS_FIRMWARE_TRIGGER,            /**< Firmware trigger mode */
    ADC_EC_SOS_PERMANENT_TRIGGER,           /**< Permanent trigger mode */
} AdcEcSosSource_t;


/** Types of Start-Of-Conversion */
typedef enum {
    ADC_EC_SOC_HARDWARE_TRIGGER = 0u,       /**< Hardware SOC */
    ADC_EC_SOC_FIRMWARE_TRIGGER,            /**< Firmware SOC */
    ADC_EC_SOC_HARDWARE_FW_TRIGGER,         /**< Hardware + Software SOC */
    ADC_EC_SOC_PERMANENT_TRIGGER,           /**< Permanent SOC */
} AdcEcSocSource_t;

/** Types of ADC EC Interrupts */
typedef enum {
    ADC_EC_INT_NO_INT = 0u,                 /**< No Interrupts */
    ADC_EC_INT_EOC,                         /**< End of conversion Interrupt */
    ADC_EC_INT_EOF,                         /**< End of Frame Interrupt */
    ADC_EC_INT_EOS,                         /**< End of Sequence Interrupt */
} AdcEcIntScheme_t;

/** ADC EC StandBy configuration types */
typedef enum {
    ADC_EC_ASB_NOT_USED = 0u,               /**< Auto-StandBy is used when ADC not used. */
    ADC_EC_ASB_WAIT_TRIGGERS,               /**< Auto-StandBy is used when waiting for triggers. */
    ADC_EC_ASB_NEVER,                       /**< Never use StandBy */
} AdcEcAutoStandby_t;

/** ADC EC Data width types */
typedef enum {
    ADC_EC_WDT_16Bit = 0u,                  /**< ADC EC data width is 16bit */
    ADC_EC_WDT_32Bit,                       /**< ADC EC data width is 32bit */
} AdcEcWidth_t;

/** ADC EC conversion type */
typedef enum {
    ADC_EC_TYPE_EXTENDED_COUNTING = 0u,     /**< ADC EC type is extended counting */
    ADC_EC_TYPE_CYCLIC_ONLY,                /**< ADC EC type is cyclic only */
} AdcEcType_t;

/** ADC EC operation mode */
typedef enum {
    ADC_EC_MODE_NORMAL_MODE = 0u,           /**< ADC EC mode is normal mode */
    ADC_EC_MODE_TEST_MODE,                  /**< ADC EC mode is test mode */
} AdcEcMode_t;

/** ADC EC cyclic conversion starting phase */
typedef enum {
    ADC_EC_START_PHI_SAME = 0u,             /**< ADC EC starting phase is same than in extended counting mode */
    ADC_EC_START_PHI_OPPOSITE               /**< ADC EC starting phase is opposite phase to the extended counting mode */
} AdcEcStartPhi_t;

/** ADC EC output data format */
typedef enum {
    ADC_EC_OUT_MODE_REGULAR = 0u,           /**< ADC EC returns the regular conversion data */
    ADC_EC_OUT_MODE_2BIT                    /**< ADC EC returns the 2-bit comparison result in each conversion step */
} AdcEcOutMode_t;

/** ADC EC overriding of the first comparison of a cyclic conversion */
typedef enum {
    ADC_EC_FORCE_REGULAR = 0u,              /**< ADC EC regular comparison */
    ADC_EC_FORCE_DH_DL_00,                  /**< ADC EC code {DH,DL} = 00 */
    ADC_EC_FORCE_DH_DL_10,                  /**< ADC EC code {DH,DL} = 10 */
    ADC_EC_FORCE_DH_DL_01                   /**< ADC EC code {DH,DL} = 01 */
} AdcEcForce_t;

/** ADC EC disabling of the chopping of the amplifier */
typedef enum {
    ADC_EC_NO_CHOP_CHOPPING_ACTIVE = 0u,    /**< ADC EC chopping is active */
    ADC_EC_NO_CHOP_CHOPPING_DISABLED        /**< ADC EC chopping is disabled */
} AdcEcNoChop_t;

/** ADC EC reference selection */
typedef enum {
    ADC_EC_INT_REF_EXTERNAL = 0u,           /**< ADC EC select external reference voltage */
    ADC_EC_INT_REF_INTERNAL                 /**< ADC EC select internal reference voltage (internalVref = VDDA3V3/2) */
} AdcEcIntRef_t;

/** ADC EC bias current selection (max system frequency selection) */
typedef enum {
    ADC_EC_SPEED_FULL_BIAS = 0u,            /**< ADC EC full bias */
    ADC_EC_SPEED_TWO_THIRDS_BIAS,           /**< ADC EC 2/3 bias */
    ADC_EC_SPEED_HALF_BIAS,                 /**< ADC EC 1/2 bias */
    ADC_EC_SPEED_ONE_THIRD_BIAS,            /**< ADC EC 1/3 bias */
} AdcEcSpeed_t;

/** ADC EC analog test and internal reference buffers configuration */
typedef enum {
    ADC_EC_REF_ALWAYS_ON_ADC_STDBY = 0u,    /**< ADC EC reference buffers driven by ADC standby signal */
    ADC_EC_REF_ALWAYS_ON_LEAVE_ON           /**< ADC EC reference buffers leave on between conversions */
} AdcEcRefAlwaysOn_t;

/** ADC EC definition of number of steps during extended conversion
 *
 * Number of steps is calculated as:
 * # Steps = 2*(COUNT+3)
 */
typedef enum {
    ADC_EC_COUNT_0 = 0u,                    /**< 6 steps */
    ADC_EC_COUNT_1,                         /**< 8 steps */
    ADC_EC_COUNT_2,                         /**< 10 steps */
    ADC_EC_COUNT_3,                         /**< 12 steps */
    ADC_EC_COUNT_4,                         /**< 14 steps */
    ADC_EC_COUNT_5,                         /**< 16 steps */
    ADC_EC_COUNT_6,                         /**< 18 steps */
    ADC_EC_COUNT_7,                         /**< 20 steps */
} AdcEcCount_t;

/** ADC EC ADC Frequency definition  */
typedef enum {
    ADC_EC_FREQ_CE = 0u,                    /**< CE frequency used */
    ADC_EC_FREQ_CE_DIV2,                    /**< CE/2 frequency used */
    ADC_EC_FREQ_CE_DIV4,                    /**< CE/4 frequency used */
    ADC_EC_FREQ_CE_DIV8,                    /**< CE/8 frequency used */
} AdcEcFreq_t;

/** ADC EC Configuration and modes type */
typedef union AdcEcControl_u {
    struct  __attribute__((packed))
    BitControl {
        uint16_t startBit : 1;              /**< START command bit */
        uint16_t stopBit : 1;               /**< STOP command bit */
        AdcEcSosSource_t sosSource : 2;     /**< Start-Of-Sequence source */
        AdcEcSocSource_t socSource : 2;     /**< Start-Of-Conversion source */
        uint16_t noInterleave : 1;          /**< 1 - when EOC triggers SDATA update, When 0 - EOA triggers SDATA update */
        uint16_t saturate : 1;              /**< 1 - data saturated to 2^N-1 in case of overflow (for N bit in DATA) and 0 in underflow; 0 - DATA is garbage in under-/over-flow  */
        AdcEcIntScheme_t intScheme : 2;     /**< Interrupts Scheme */
        AdcEcAutoStandby_t asb : 2;         /**< Auto StandBy Mode */
        AdcEcWidth_t adcWidth : 1;          /**< ADC EC conversion width */
        uint16_t : 3;
        /* Other ADC_BLOCK port */
        AdcEcCount_t count : 3;             /**< ADC EC counting steps during extended conversion */
        AdcEcType_t type : 1;               /**< Type of conversion. Might been taken from SDATA. See the ADC EC specification */
        AdcEcMode_t mode : 1;               /**< Operation mode */
        AdcEcFreq_t frequency : 2;          /**< ADC EC ADC Frequency definition  */
        AdcEcStartPhi_t startPhi : 1;       /**< Starting phase for cyclic conversion */
        AdcEcOutMode_t outMode : 1;         /**< Output data format */
        AdcEcForce_t force : 2;             /**< Overrides the first comparison of a cyclic conversion */
        AdcEcNoChop_t noChop : 1;           /**< Disables the chopping of the amplifier */
        AdcEcIntRef_t intRef : 1;           /**< ADC reference selection */
        AdcEcSpeed_t speed : 2;             /**< Bias current selection (max system frequency selection) */
        AdcEcRefAlwaysOn_t refAlwaysOn : 1; /**< ADC analog test and internal reference buffers configuration */
    } s;
    uint16_t data[2];                       /**< Grouped field to write the IO-port at once */
} AdcEcControl_t;

/** ADC EC precharge mode */
typedef enum {
    ADC_EC_NO_PRECHARGE = 0u,               /**< no pre-charge */
    ADC_EC_LEAD_TO_CONVERSION,              /**< pre-charge leading the conversion */
    ADC_EC_CONST_PRECHARGE,                 /**< constant pre-charge */
} AdcPrechargeMode_t;

/** ADC EC Clock and precharge configuration */
typedef union AdcEcClkPreChControl_u {
    struct  __attribute__((packed))
    ClkDivBitControl {
        uint16_t adcClkDiv : 7;             /**< ADC clock divider. ADC_CLK = MCU_CLK / ( ADC_CLK_DIV + 1 ) */
        uint16_t : 1;
        uint16_t clkToInt : 2;              /**< Number of MCU_CLK between MS_PHIx_CLK and MS_PHIx_INT */
        uint16_t cmPrchrgTime : 3;          /**< Pre-Charge Time: T_PRECHARGE = 2^CM_PRCHRG_TIME */
        AdcPrechargeMode_t cmPrchrg : 2;    /**< Pre-charge mode */
    } s;
    uint16_t data;                          /**< Grouped field to write the IO-port at once */
} AdcEcClkPreChControl_t;


/** ADC EC phase type definition */
typedef enum {
    ADC_EC_PHASE_IDLE = 0u,                 /**< ADC EC no operation */
    ADC_EC_PHASE_MEM_TRANSFER,              /**< ADC EC is transferring the data */
    ADC_EC_PHASE_CONVERSION,                /**< ADC EC performs the conversion */
    ADC_EC_PHASE_WAIT_TRIGGER,              /**< ADC EC waits a trigger */
} AdcEcPhaseState_t;

/** ADC statuses */
typedef union AdcEcState_u {
    struct  __attribute__((packed))
    BitState {
        uint16_t paused : 1;                /**< Paused flag */
        uint16_t resumed : 1;               /**< Resumed flag */
        uint16_t swTrig : 1;                /**< Firmware trigger control bit */
        uint16_t ready : 1;                 /**< Ready flag */
        AdcEcIntScheme_t lastIntSource : 2; /**< The Last Interrupt source */
        AdcEcPhaseState_t state : 2;        /**< The current ADC phase */
        uint16_t adcOvf : 1;                /**< ADC EC Overflow flag */
        uint16_t adcErr : 1;                /**< ADC EC Error flag */
        uint16_t memErr : 1;                /**< ADC EC Memory error flag */
        uint16_t frameErr : 1;              /**< ADC EC Frame error flag */
        uint16_t aborted : 1;               /**< ADC EC aborted flag */
    } s;
    uint16_t data; /**< The grouped 16bit status interface to get statuses in scope */
} AdcEcState_t;


/** Configures ADC Module with SBase address and run one sequence, blocking function
 * @param[in] sBaseAddress Address of the ADC SBase to be run once
 */
void AdcSingleSeqRun(uint16_t sBaseAddress);

/** Configure the ADC
 * @param[in] adcClkDivCtrl the MCU clock divider configuration and pre-charger control.
 * @param[in] sBase the initial SBase offset.
 * @param[in] ctrl specifies the Sequence and Conversion sources, Interleave, Saturate and Interrupt scheme, StandBy mode
 */
STATIC INLINE void AdcInit(AdcEcClkPreChControl_t adcClkDivCtrl, const void* sBase, AdcEcControl_t ctrl);

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
STATIC INLINE AdcEcState_t AdcGetState(void);

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
#endif /* LIB_ADC_EC_H */
