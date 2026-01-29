/**
 * @file
 * @brief ADC pins and settings definitions
 * @internal
 *
 * @copyright (C) 2017-2021 Melexis N.V.
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

/** ADC EOx[1:0] sign definitions */
typedef enum {
    ADC_NO_SIGN = 0b00,             /**< Used when no signs are needed */
    ADC_NO_SIGN2 = 0b01,            /**< Used when no signs are needed */
    ADC_EOF_SIGN = 0b10,            /**< End-of-Frame sign */
    ADC_EOS_SIGN = 0b11,            /**< End-of-Sequence sign */
} AdcEos_t;

/** ADC SIN_SEL[2] source type definition */
typedef enum {
    ADC_SRC_TYPE_APPLICATION = 0u,  /**< controlled by SDATA[8:3] from ADC Interface */
    ADC_SRC_TYPE_TEST,              /**< field SRC[5:0] is Test Configuration port controlled */
} AdcSourceType_t;

/** ADC ADC_SEL[8:3] channels (signals) definition */
typedef enum {
    ADC_SIG_VS_DIV26 = 0u,          /**< Supply voltage sensor divided by 26  */
    ADC_SIG_TEMP = 1u,              /**< Internal temperature sensor */
    ADC_SIG_VDDD_DIV2 = 2u,         /**< Digital supply voltage divided by 2 */
    ADC_SIG_VDDA_DIV3 = 3u,         /**< Analog supply voltage divided by 3 */
    ADC_SIG_VBG_D = 4u,             /**< Bandgap voltage, second digital BG */
    ADC_SIG_VAUX_DIV4 = 5u,         /**< Auxiliary analog supply divided by 4 */
    /* 000110 - 000111: reserved */
    ADC_SIG_VSM_DIV26_FILT = 8u,    /**< Filtered motor supply voltage divided by 26 */
    ADC_SIG_CSOUT1 = 9u,            /**< Current sense amplifier output 1 voltage */
    ADC_SIG_CSOUT2 = 10u,           /**< Current sense amplifier output 2 voltage */
    ADC_SIG_LIN_DIV3 = 11u,         /**< LIN pin voltage (after reverse polarity switch) divided by 3 */
    ADC_SIG_IO0_DIV2_5 = 12u,       /**< IO0 voltage divided by 2.5 */
    ADC_SIG_IO1_DIV2_5 = 13u,       /**< IO1 voltage divided by 2.5 */
    ADC_SIG_IO2_DIV2_5 = 14u,       /**< IO2 voltage divided by 2.5 */
    ADC_SIG_IO3_DIV2_5 = 15u,       /**< IO3 voltage divided by 2.5 */
    ADC_SIG_IO4_DIV2_5 = 16u,       /**< IO4 voltage divided by 2.5 */
    ADC_SIG_IO5_DIV2_5 = 17u,       /**< IO5 voltage divided by 2.5 */
    ADC_SIG_IO6_DIV2_5 = 18u,       /**< IO6 voltage divided by 2.5 */
    ADC_SIG_IO7_DIV2_5 = 19u,       /**< IO7 voltage divided by 2.5 */
    ADC_SIG_IO8_DIV2_5 = 20u,       /**< IO8 voltage divided by 2.5 */
    ADC_SIG_IO9_DIV2_5 = 21u,       /**< IO9 voltage divided by 2.5 */
    ADC_SIG_IO10_DIV2_5 = 22u,      /**< IO10 voltage divided by 2.5 */
    ADC_SIG_IO11_DIV2_5 = 23u,      /**< IO11 voltage divided by 2.5 */
    ADC_SIG_IO0_DIV26 = 24u,        /**< IO0 voltage divided by 26 */
    ADC_SIG_IO1_DIV26 = 25u,        /**< IO1 voltage divided by 26 */
    ADC_SIG_IO2_DIV26 = 26u,        /**< IO2 voltage divided by 26 */
    ADC_SIG_R_DIV26 = 27u,          /**< R phase output divided by 26 */
    ADC_SIG_S_DIV26 = 28u,          /**< S phase output divided by 26 */
    ADC_SIG_T_DIV26 = 29u,          /**< T phase output divided by 26 */
    ADC_SIG_U_DIV26 = 30u,          /**< U phase output divided by 26 */
    ADC_SIG_V_DIV26 = 31u,          /**< V phase output divided by 26 */
    ADC_SIG_W_DIV26 = 32u,          /**< W phase output divided by 26 */
    ADC_SIG_VSSA = 33u,             /**< Analog ground */
    /* 100010 - 111101: reserved */
    ADC_SIG_TA0 = 62u,              /**< Test : low voltage analog test-bus 0 */
    ADC_SIG_TA1 = 63u               /**< Test : low voltage analog test-bus 1 */
} AdcSignal_t;

/** ADC ADC_TYPE[9] type selection */
typedef enum {
    ADC_TYPE_EC = 0u,               /**< ec */
    ADC_TYPE_CYCLIC,                /**< cyclic only */
} AdcType_t;

/** ADC TRIG_SEL[15:10] Trigger source definition */
typedef enum {
    ADC_TRIG_PWM_MA1_CMP = 0u,      /**< PWM compare interrupt */
    ADC_TRIG_PWM_MA1_END = 1u,      /**< PWM counter interrupt */
    ADC_TRIG_PWM_SL1_CMP = 2u,      /**< PWM compare interrupt */
    ADC_TRIG_PWM_SL2_CMP = 3u,      /**< PWM compare interrupt */
    ADC_TRIG_PWM_SL3_CMP = 4u,      /**< PWM compare interrupt */
    ADC_TRIG_PWM_SL4_CMP = 5u,      /**< PWM compare interrupt */
    ADC_TRIG_PWM_MA2_CMP = 6u,      /**< PWM compare interrupt */
    ADC_TRIG_PWM_MA2_END = 7u,      /**< PWM counter interrupt */
    ADC_TRIG_IO0 = 8u,              /**< IO interrupt */
    ADC_TRIG_IO1 = 9u,              /**< IO interrupt */
    ADC_TRIG_IO2 = 10u,             /**< IO interrupt */
    ADC_TRIG_IO3 = 11u,             /**< IO interrupt */
    ADC_CTIMER0_TIM1 = 12u,         /**< Complex Timer 0, Int1 interrupt */
    ADC_CTIMER0_TIM2 = 13u,         /**< Complex Timer 0, Int2 interrupt */
    ADC_CTIMER0_TIM3 = 14u,         /**< Complex Timer 0, Int3 interrupt */
    ADC_CTIMER1_TIM1 = 15u,         /**< Complex Timer 1, Int1 interrupt */
    ADC_CTIMER1_TIM2 = 16u,         /**< Complex Timer 1, Int2 interrupt */
    ADC_CTIMER1_TIM3 = 17u,         /**< Complex Timer 1, Int3 interrupt */
    ADC_TRIG_IO4 = 18u,             /**< IO interrupt */
    ADC_TRIG_IO5 = 19u,             /**< IO interrupt */
    ADC_TRIG_IO6 = 20u,             /**< IO interrupt */
    ADC_TRIG_IO7 = 21u,             /**< IO interrupt */
    ADC_TRIG_IO8 = 22u,             /**< IO interrupt */
    ADC_TRIG_IO9 = 23u,             /**< IO interrupt */
    ADC_TRIG_IO10 = 24u,            /**< IO interrupt */
    ADC_TRIG_IO11 = 25u,            /**< IO interrupt */
    /* 011010 - 011111: reserved */
    ADC_EOC_DELAY_TMCU_0 = 32u,     /**< Tdelay = 1 * 2 * T_MCUCLOCK */
    ADC_EOC_DELAY_TMCU_1 = 33u,     /**< Tdelay = 2 * 2 * T_MCUCLOCK */
    ADC_EOC_DELAY_TMCU_2 = 34u,     /**< Tdelay = 3 * 2 * T_MCUCLOCK */
    ADC_EOC_DELAY_TMCU_3 = 35u,     /**< Tdelay = 4 * 2 * T_MCUCLOCK */
    ADC_EOC_DELAY_TMCU_4 = 36u,     /**< Tdelay = 5 * 2 * T_MCUCLOCK */
    ADC_EOC_DELAY_TMCU_5 = 37u,     /**< Tdelay = 6 * 2 * T_MCUCLOCK */
    ADC_EOC_DELAY_TMCU_6 = 38u,     /**< Tdelay = 7 * 2 * T_MCUCLOCK */
    ADC_EOC_DELAY_TMCU_7 = 39u,     /**< Tdelay = 8 * 2 * T_MCUCLOCK */
    /* 100111 - 101111: reserved */
    ADC_EOC_DELAY_TADC_0 = 48u,     /**< Tdelay = 1 * 2 * T_ADCCLK */
    ADC_EOC_DELAY_TADC_1 = 49u,     /**< Tdelay = 2 * 2 * T_ADCCLK */
    ADC_EOC_DELAY_TADC_2 = 50u,     /**< Tdelay = 3 * 2 * T_ADCCLK */
    ADC_EOC_DELAY_TADC_3 = 51u,     /**< Tdelay = 4 * 2 * T_ADCCLK */
    ADC_EOC_DELAY_TADC_4 = 52u,     /**< Tdelay = 5 * 2 * T_ADCCLK */
    ADC_EOC_DELAY_TADC_5 = 53u,     /**< Tdelay = 6 * 2 * T_ADCCLK */
    ADC_EOC_DELAY_TADC_6 = 54u,     /**< Tdelay = 7 * 2 * T_ADCCLK */
    ADC_EOC_DELAY_TADC_7 = 55u      /**< Tdelay = 8 * 2 * T_ADCCLK */
                           /* 110111 - 111111: reserved */
} AdcTrigSel_t;


/** ADC SData structure definition */
typedef union AdcSData_u {
    struct __attribute__((packed)) bit {
        AdcEos_t adcEos : 2;                        /**< End-of-Sequence sign, used by ADC interface to mark end of frame (=10) or end of sequence */
        AdcSourceType_t adcSource : 1;              /**< ADC mode: input is application or test */
        AdcSignal_t adcChan : 6;                    /**< ADC channel number [0-63] */
        AdcType_t adcType : 1;                      /**< ADC type, EC or cyclic mode */
        AdcTrigSel_t adcTrig : 6;                   /**< ADC hardware trigger selection */
    } s;

    uint16_t u16;
} AdcSData_t;

#define ADC_SDATA_BYTE_SIZE 2u
ASSERT(sizeof(AdcSData_t) == ADC_SDATA_BYTE_SIZE);

/** Example for the SDATA array defintion */
#define ADC_FRAME_SAMPLES_EXAMPLE  (0x10)
typedef struct buffers {
    uint16_t* AdcData[ADC_FRAME_SAMPLES_EXAMPLE];   /**< Pointer to the ADC data frame destination buffer. The buffer should be the size of the SData structure */
    AdcSData_t SData[ADC_FRAME_SAMPLES_EXAMPLE];    /**< SData for the frame samples */
    AdcSData_t sign;                                /**< End-of-Sequence or End-of-Frame sign */
} AdcFrameExample_t;

#endif /* LIB_ADC_REFS_H */
