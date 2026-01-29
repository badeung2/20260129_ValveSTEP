/**
 * @file
 * @brief ADC pins and settings definitions
 * @internal
 *
 * @copyright (C) 2017-2018 Melexis N.V.
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
 * @ingroup CAMCU_library
 *
 * @details
 * The ADC Signal definitions. Define a set of signals and references available
 */

#ifndef LIB_ADC_REFS_H
#define LIB_ADC_REFS_H

#include <stdint.h>
#include "static_assert.h"
#include "compiler_abstraction.h"

/** ADC reference definition */
typedef enum {
    ADC_VREF_NONE = 0u,         /**< VREF is disconnected */
    ADC_VREF_0_75V,             /**< VREF=0.75V */
    ADC_VREF_1_5V,              /**< VREF=1.5V */
    ADC_VREF_2_5V,              /**< VREF=2.5V */
    ADC_VREF_MEAS,              /**< switch VDDA to VREF for ratiometric measurements */
} AdcVref_t;
#define Adc_Vref_t AdcVref_t __attribute__ ((deprecated("Renamed to AdcVref_t")))

/** ADC reference definition, splitted for differential configuration */
typedef enum {
    ADC_VREF_DIFF_NONE = 0u,    /**< VREF is disconnected */
    ADC_VREF_DIFF_0_75V,        /**< VREF=0.75V */
    ADC_VREF_DIFF_1_5V,         /**< VREF=1.5V */
    ADC_VREF_DIFF_2_5V,         /**< VREF=2.5V */
} AdcVrefDiff_t;
#define Adc_VrefDiff_t AdcVrefDiff_t __attribute__ ((deprecated("Renamed to AdcVrefDiff_t")))

/** Adc_VrefDiffAdd_t reference definition, splitted for differential configuration, should be set to #ADC_VREF_DIFF_NORMAL for setting the reference */
typedef enum {
    ADC_VREF_DIFF_NORMAL = 0u,  /**< Normal differential reference mode */
    ADC_VREF_DIFF_MEAS,         /**< switch VDDA to VREF for ratiometric measurements */
} AdcVrefDiffAdd_t;
#define Adc_VrefDiffAdd_t AdcVrefDiffAdd_t __attribute__ ((deprecated("Renamed to AdcVrefDiffAdd_t")))

/** ADC channels (signals) definition */
typedef enum {
    ADC_SIG_VS_DIV21 = 0u,      /**< Supply voltage sensor divided by 21  */
    ADC_SIG_TEMP,               /**< Internal temperature sensor */
    ADC_SIG_VDDD,               /**< Digital supply voltage */
    ADC_SIG_VDDA_DIV2,          /**< Analogue supply voltage divided by 2 */
    ADC_SIG_VBG_D,              /**< Bandgap voltage, second digital BG */
    ADC_SIG_VAUX_DIV4,          /**< Auxiliary analogue supply divided by 4 */
    ADC_SIG_LINAAMP,            /**< LIN autoconfig : amplifier output / SHUNT */
    ADC_SIG_LINVCMO,            /**< LIN autoconfig : common mode output */
    ADC_SIG_IO0_DIV1_36,        /**< IO0 voltage divided by 1.36 */
    ADC_SIG_IO1_DIV1_36,        /**< IO1 voltage divided by 1.36 */
    ADC_SIG_IO2_DIV1_36,        /**< IO2 voltage divided by 1.36 */
    ADC_SIG_IO3_DIV1_36,        /**< IO3 voltage divided by 1.36 */
    ADC_SIG_U_DIV21,            /**< U phase output divided by 21 */
    ADC_SIG_V_DIV21,            /**< V phase output divided by 21 */
    ADC_SIG_W_DIV21,            /**< W phase output divided by 21 */
    ADC_SIG_T_DIV21,            /**< T phase output divided by 21 */
    ADC_SIG_CSOUT,              /**< Current sense amplifier output voltage */
    ADC_SIG_VSM_DIV21,          /**< Motor supply voltage divided by 21 */
    ADC_SIG_IO0_DIV21,          /**< IO0 voltage divided by 21 */
    ADC_SIG_CSOUT_FILT,         /**< Filtered current sense amplifier output voltage */
    ADC_SIG_VSM_DIV21_FILT,     /**< Filtered motor supply voltage divided by 21 */
    ADC_SIG_IO4_DIV1_36,        /**< IO4 voltage divided by 1.36 */
    ADC_SIG_IO5_DIV1_36,        /**< IO5 voltage divided by 1.36 */
    ADC_SIG_IO6_DIV1_36,        /**< IO6 voltage divided by 1.36 */
    ADC_SIG_IO7_DIV1_36,        /**< IO7 voltage divided by 1.36 */
} AdcSignal_t;
#define Adc_Signal_t AdcSignal_t __attribute__ ((deprecated("Renamed to AdcSignal_t")))

/** ADC [1:0] Signs definitions  */
typedef enum {
    ADC_NO_SIGN = 0b00,         /**< Used when no signs are needed */
    ADC_NO_SIGN2 = 0b01,        /**< Used when no signs are needed */
    ADC_EOF_SIGN = 0b10,        /**< End-of-Frame sign */
    ADC_EOS_SIGN = 0b11,        /**< End-of-Sequence sign */
} AdcEos_t;
#define Adc_Eos_t AdcEos_t __attribute__ ((deprecated("Renamed to AdcEos_t")))

/** Adc_Div_t divider selection */
typedef enum {
    ADC_DIV_DISABLED = 0u,      /**< Divider is not used */
    ADC_DIV_ENABLED,            /**< Divider is used */
} AdcDiv_t;
#define Adc_Div_t AdcDiv_t __attribute__ ((deprecated("Renamed to AdcDiv_t")))

/** ADC Trigger source definition */
typedef enum {
    ADC_TRIG_PWM_MA1_CMP = 0u,  /**< PWM compare interrupt */
    ADC_TRIG_PWM_MA1_END,       /**< PWM counter interrupt */
    ADC_TRIG_PWM_SL1_CMP,       /**< PWM compare interrupt */
    ADC_TRIG_PWM_SL2_CMP,       /**< PWM compare interrupt */
    ADC_TRIG_PWM_SL3_CMP,       /**< PWM compare interrupt */
    ADC_TRIG_PWM_MA2_CMP,       /**< PWM compare interrupt */
    ADC_TRIG_PWM_MA2_END,       /**< PWM counter interrupt */
    ADC_TRIG_PWM_IO0,           /**< IO interrupt */
    ADC_TRIG_PWM_IO1,           /**< IO interrupt */
    ADC_TRIG_PWM_IO2,           /**< IO interrupt */
    ADC_TRIG_PWM_IO3,           /**< IO interrupt */
    ADC_CTIMER0_TIM1,           /**< Complex Timer 0, Int1 interrupt */
    ADC_CTIMER0_TIM2,           /**< Complex Timer 0, Int2 interrupt */
    ADC_CTIMER0_TIM3,           /**< Complex Timer 0, Int3 interrupt */
    ADC_CTIMER1_TIM1,           /**< Complex Timer 1, Int1 interrupt */
    ADC_CTIMER1_TIM2,           /**< Complex Timer 1, Int2 interrupt */
    ADC_CTIMER1_TIM3,           /**< Complex Timer 1, Int3 interrupt */
    ADC_TRIG_PWM_IO4,           /**< IO interrupt */
    ADC_TRIG_PWM_IO5,           /**< IO interrupt */
    ADC_TRIG_PWM_IO6,           /**< IO interrupt */
    ADC_TRIG_PWM_IO7,           /**< IO interrupt */
    ADC_EOC_DELAY0 = 24u,       /**< Tdelay = 1 x TADCCLK*2  = ~0.5us (*) . Target TADCCLK=4MHz */
    ADC_EOC_DELAY1,             /**< Tdelay = 2 x TADCCLK*2  = ~1.0us (*) . Target TADCCLK=4MHz */
    ADC_EOC_DELAY2,             /**< Tdelay = 3 x TADCCLK*2  = ~1.5us (*) . Target TADCCLK=4MHz */
    ADC_EOC_DELAY3,             /**< Tdelay = 4 x TADCCLK*2  = ~2.0us (*) . Target TADCCLK=4MHz */
    ADC_EOC_DELAY4,             /**< Tdelay = 5 x TADCCLK*2  = ~2.5us (*) . Target TADCCLK=4MHz */
    ADC_EOC_DELAY5,             /**< Tdelay = 6 x TADCCLK*2  = ~3.0us (*) . Target TADCCLK=4MHz */
    ADC_EOC_DELAY6,             /**< Tdelay = 7 x TADCCLK*2  = ~3.5us (*) . Target TADCCLK=4MHz */
    ADC_EOC_DELAY7,             /**< Tdelay = 8 x TADCCLK*2  = ~4.0us (*) . Target TADCCLK=4MHz */
} AdcTrigSel_t;
#define Adc_TrigSel_t AdcTrigSel_t __attribute__ ((deprecated("Renamed to AdcTrigSel_t")))

/** ADC SData structure definition */
typedef union AdcSData_u {
    struct __attribute__((packed)) bit {
        AdcEos_t adcEosSign : 2;                    /**< End-of-Sequence sign, used by ADC interface to mark end of frame (=10) or end of sequence */
        AdcSignal_t adcChan : 5;                    /**< ADC channel number [0-31] */
        AdcVref_t adcRef : 3;                       /**< ADC reference selector */
        AdcTrigSel_t adcTrig : 5;                   /**< ADC hardware trigger selection */
        AdcDiv_t adcDiv : 1;                        /**< ADC_DIV divider selection, not used */
    } s;

    uint16_t u16;

} AdcSData_t;
#define Adc_SData_t AdcSData_t __attribute__ ((deprecated("Renamed to AdcSData_t")))

#define ADC_SDATA_BYTE_SIZE 2u
ASSERT(sizeof(AdcSData_t) == ADC_SDATA_BYTE_SIZE);

/** Example for the SDATA array defintion */
#define ADC_FRAME_SAMPLES_EXAMPLE  (0x10)
typedef struct buffers {
    uint16_t* AdcData[ADC_FRAME_SAMPLES_EXAMPLE];   /**< Pointer to the ADC data frame destination buffer. The buffer should be the size of the SData structure */
    AdcSData_t SData[ADC_FRAME_SAMPLES_EXAMPLE];    /**< SData for the frame samples */
    AdcSData_t sign;                                /**< End-of-Sequence or End-of-Frame sign */
} AdcFrameExample_t;
#define Adc_FrameExample_t AdcFrameExample_t __attribute__ ((deprecated("Renamed to AdcFrameExample_t")))

#endif /* LIB_ADC_REFS_H */
