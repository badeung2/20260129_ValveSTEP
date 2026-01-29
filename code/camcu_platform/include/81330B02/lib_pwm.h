/**
 * @file
 * @brief PWM HW support library
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
 * @ingroup pwm
 * @addtogroup pwm PWM
 *
 * @brief PWM HW support library
 * @details
 * @{
 */

#ifndef LIB_PWM_H
#define LIB_PWM_H

#include <stdint.h>
#include "itc_helper.h"

#include "syslib.h"
#include "io.h"

/** PWM mode type */
typedef enum {
    PWM_MODE_SIMPLE = 0u,       /**< Simple PWM mode. LT and PER is used. */
    PWM_MODE_MIRROR = 1u,       /**< Mirrored PWM mode. LT and PER is used. Centers the pulse in PER period. */
    PWM_MODE_INDEPENDENT = 2u,  /**< Independent PWM mode, LT, HT and PER is used. */
} PwmMode_t;
#define pwmMode_t PwmMode_t __attribute__ ((deprecated("Renamed to PwmMode_t")))

/** PWM polarity type */
typedef enum {
    PWM_POL_LOW = 0u,   /**< PWM starts at 0 */
    PWM_POL_HIGH = 1u,  /**< PWM starts at 1 */
} PwmPolarity_t;
#define pwmPolarity_t PwmPolarity_t __attribute__ ((deprecated("Renamed to PwmPolarity_t")))

/** SLAVE mode type */
typedef enum {
    PWM_MASTER_MODE = 0u,  /**< PWM is configured as MASTER */
    PWM_SLAVE_MODE = 1u    /**< PWM is configured as SLAVE */
} PwmSlave_t;

#if defined (IO_PWM_MASTER1__START_GET)
#define PWM1_MASTER_INIT pwm1_master_init
#define PWM1_MASTER_ENABLE_SLAVE_MODE pwm1_master_enable_slave_mode

/** pwm1_master_init Configures the PWM with the master mode
 * @param[in] DIV_ input clock divider to DIV. Allowed range is [1..16].
 * @param[in] DIVN_ input clock divider to 2^DIVN. Allowed range is [0..11].
 * @param[in] PERIOD_ PWM period
 * @param[in] LT_ Low threshold value
 * @param[in] HT_ High threshold value
 * @param[in] CMP_ Interrupt comparator register value
 * @param[in] MODE_ PWM mode
 * @param[in] POL_ initial polarity for PWM period
 * @param[in] InvertPolarityIDLE_ POL polarity invertion in the IDLE state. 1 inverts the POL
 */
STATIC INLINE void pwm1_master_init(uint8_t DIV_,
                                    uint8_t DIVN_,
                                    uint16_t PERIOD_,
                                    uint16_t LT_,
                                    uint16_t HT_,
                                    uint16_t CMP_,
                                    PwmMode_t MODE_,
                                    PwmPolarity_t POL_,
                                    uint8_t InvertPolarityIDLE_);
/** pwm1_master_enable_slave_mode Extends pwm1_master_init with the enabling of the mode slave option
 */
STATIC INLINE void pwm1_master_enable_slave_mode(void);
#endif /* PWM_MASTER1 */

#if defined (IO_PWM_MASTER2__START_GET)
#define PWM2_MASTER_INIT pwm2_master_init
#define PWM2_MASTER_ENABLE_SLAVE_MODE pwm2_master_enable_slave_mode

/** pwm2_master_init Configures the PWM with the master mode
 * @param[in] DIV_ input clock divider to DIV. Allowed range is [1..16].
 * @param[in] DIVN_ input clock divider to 2^DIVN. Allowed range is [0..11].
 * @param[in] PERIOD_ PWM period
 * @param[in] LT_ Low threshold value
 * @param[in] HT_ High threshold value
 * @param[in] CMP_ Interrupt comparator register value
 * @param[in] MODE_ PWM mode
 * @param[in] POL_ initial polarity for PWM period
 * @param[in] InvertPolarityIDLE_ POL polarity invertion in the IDLE state. 1 inverts the POL
 */
STATIC INLINE void pwm2_master_init(uint8_t DIV_,
                                    uint8_t DIVN_,
                                    uint16_t PERIOD_,
                                    uint16_t LT_,
                                    uint16_t HT_,
                                    uint16_t CMP_,
                                    PwmMode_t MODE_,
                                    PwmPolarity_t POL_,
                                    uint8_t InvertPolarityIDLE_);
/** pwm2_master_enable_slave_mode Extends pwm2_master_init with the enabling of the mode slave option
 */
STATIC INLINE void pwm2_master_enable_slave_mode(void);
#endif /* PWM_MASTER2 */

#if defined (IO_PWM_SLAVE1__START_GET)
#define PWM1_INIT pwm1_init

/** pwm1_init Configures the PWM with the slave mode
 * @param[in] DIV_ input clock divider to DIV. Allowed range is [1..16].
 * @param[in] DIVN_ input clock divider to 2^DIVN. Allowed range is [0..11].
 * @param[in] LT_ Low threshold value
 * @param[in] HT_ High threshold value
 * @param[in] CMP_ Interrupt comparator register value
 * @param[in] MODE_ PWM mode
 * @param[in] POL_ initial polarity for PWM period
 * @param[in] InvertPolarityIDLE_ POL polarity invertion in the IDLE state. 1 inverts the POL
 */
STATIC INLINE void pwm1_init(uint8_t DIV_, uint8_t DIVN_, uint16_t LT_, uint16_t HT_,
                             uint16_t CMP_, PwmMode_t MODE_, PwmPolarity_t POL_, uint8_t InvertPolarityIDLE_);
#endif /* PWM_SLAVE1 */

#if defined (IO_PWM_SLAVE2__START_GET)
#define PWM2_INIT pwm2_init

/** pwm2_init Configures the PWM with the slave mode
 * @param[in] DIV_ input clock divider to DIV. Allowed range is [1..16].
 * @param[in] DIVN_ input clock divider to 2^DIVN. Allowed range is [0..11].
 * @param[in] LT_ Low threshold value
 * @param[in] HT_ High threshold value
 * @param[in] CMP_ Interrupt comparator register value
 * @param[in] MODE_ PWM mode
 * @param[in] POL_ initial polarity for PWM period
 * @param[in] InvertPolarityIDLE_ POL polarity invertion in the IDLE state. 1 inverts the POL
 */
STATIC INLINE void pwm2_init(uint8_t DIV_, uint8_t DIVN_, uint16_t LT_, uint16_t HT_,
                             uint16_t CMP_, PwmMode_t MODE_, PwmPolarity_t POL_, uint8_t InvertPolarityIDLE_);
#endif /* PWM_SLAVE2 */

#if defined (IO_PWM_SLAVE3__START_GET)
#define PWM3_INIT pwm3_init

/** pwm3_init Configures the PWM with the slave mode
 * @param[in] DIV_ input clock divider to DIV. Allowed range is [1..16].
 * @param[in] DIVN_ input clock divider to 2^DIVN. Allowed range is [0..11].
 * @param[in] LT_ Low threshold value
 * @param[in] HT_ High threshold value
 * @param[in] CMP_ Interrupt comparator register value
 * @param[in] MODE_ PWM mode
 * @param[in] POL_ initial polarity for PWM period
 * @param[in] InvertPolarityIDLE_ POL polarity invertion in the IDLE state. 1 inverts the POL
 */
STATIC INLINE void pwm3_init(uint8_t DIV_, uint8_t DIVN_, uint16_t LT_, uint16_t HT_,
                             uint16_t CMP_, PwmMode_t MODE_, PwmPolarity_t POL_, uint8_t InvertPolarityIDLE_);
#endif /* PWM_SLAVE3 */

#if defined (IO_PWM_SLAVE4__START_GET)
#define PWM4_INIT pwm4_init

/** pwm4_init Configures the PWM with the slave mode
 * @param[in] DIV_ input clock divider to DIV. Allowed range is [1..16].
 * @param[in] DIVN_ input clock divider to 2^DIVN. Allowed range is [0..11].
 * @param[in] LT_ Low threshold value
 * @param[in] HT_ High threshold value
 * @param[in] CMP_ Interrupt comparator register value
 * @param[in] MODE_ PWM mode
 * @param[in] POL_ initial polarity for PWM period
 * @param[in] InvertPolarityIDLE_ POL polarity invertion in the IDLE state. 1 inverts the POL
 */
STATIC INLINE void pwm4_init(uint8_t DIV_, uint8_t DIVN_, uint16_t LT_, uint16_t HT_,
                             uint16_t CMP_, PwmMode_t MODE_, PwmPolarity_t POL_, uint8_t InvertPolarityIDLE_);
#endif /* PWM_SLAVE4 */

#if defined (IO_PWM_SLAVE5__START_GET)
#define PWM5_INIT pwm5_init

/** pwm5_init Configures the PWM with the slave mode
 * @param[in] DIV_ input clock divider to DIV. Allowed range is [1..16].
 * @param[in] DIVN_ input clock divider to 2^DIVN. Allowed range is [0..11].
 * @param[in] LT_ Low threshold value
 * @param[in] HT_ High threshold value
 * @param[in] CMP_ Interrupt comparator register value
 * @param[in] MODE_ PWM mode
 * @param[in] POL_ initial polarity for PWM period
 * @param[in] InvertPolarityIDLE_ POL polarity invertion in the IDLE state. 1 inverts the POL
 */
STATIC INLINE void pwm5_init(uint8_t DIV_, uint8_t DIVN_, uint16_t LT_, uint16_t HT_,
                             uint16_t CMP_, PwmMode_t MODE_, PwmPolarity_t POL_, uint8_t InvertPolarityIDLE_);
#endif /* PWM_SLAVE5 */

#if defined (IO_PWM_SLAVE6__START_GET)
#define PWM6_INIT pwm6_init

/** pwm6_init Configures the PWM with the slave mode
 * @param[in] DIV_ input clock divider to DIV. Allowed range is [1..16].
 * @param[in] DIVN_ input clock divider to 2^DIVN. Allowed range is [0..11].
 * @param[in] LT_ Low threshold value
 * @param[in] HT_ High threshold value
 * @param[in] CMP_ Interrupt comparator register value
 * @param[in] MODE_ PWM mode
 * @param[in] POL_ initial polarity for PWM period
 * @param[in] InvertPolarityIDLE_ POL polarity invertion in the IDLE state. 1 inverts the POL
 */
STATIC INLINE void pwm6_init(uint8_t DIV_, uint8_t DIVN_, uint16_t LT_, uint16_t HT_,
                             uint16_t CMP_, PwmMode_t MODE_, PwmPolarity_t POL_, uint8_t InvertPolarityIDLE_);
#endif /* PWM_SLAVE6 */

#if defined (IO_PWM_SLAVE7__START_GET)
#define PWM7_INIT pwm7_init

/** pwm7_init Configures the PWM with the slave mode
 * @param[in] DIV_ input clock divider to DIV. Allowed range is [1..16].
 * @param[in] DIVN_ input clock divider to 2^DIVN. Allowed range is [0..11].
 * @param[in] LT_ Low threshold value
 * @param[in] HT_ High threshold value
 * @param[in] CMP_ Interrupt comparator register value
 * @param[in] MODE_ PWM mode
 * @param[in] POL_ initial polarity for PWM period
 * @param[in] InvertPolarityIDLE_ POL polarity invertion in the IDLE state. 1 inverts the POL
 */
STATIC INLINE void pwm7_init(uint8_t DIV_, uint8_t DIVN_, uint16_t LT_, uint16_t HT_,
                             uint16_t CMP_, PwmMode_t MODE_, PwmPolarity_t POL_, uint8_t InvertPolarityIDLE_);
#endif /* PWM_SLAVE7 */

#if defined (IO_PWM_SLAVE8__START_GET)
#define PWM8_INIT pwm8_init

/** pwm8_init Configures the PWM with the slave mode
 * @param[in] DIV_ input clock divider to DIV. Allowed range is [1..16].
 * @param[in] DIVN_ input clock divider to 2^DIVN. Allowed range is [0..11].
 * @param[in] LT_ Low threshold value
 * @param[in] HT_ High threshold value
 * @param[in] CMP_ Interrupt comparator register value
 * @param[in] MODE_ PWM mode
 * @param[in] POL_ initial polarity for PWM period
 * @param[in] InvertPolarityIDLE_ POL polarity invertion in the IDLE state. 1 inverts the POL
 */
STATIC INLINE void pwm8_init(uint8_t DIV_, uint8_t DIVN_, uint16_t LT_, uint16_t HT_,
                             uint16_t CMP_, PwmMode_t MODE_, PwmPolarity_t POL_, uint8_t InvertPolarityIDLE_);
#endif /* PWM_SLAVE8 */

#if defined (IO_PWM_SLAVE9__START_GET)
#define PWM9_INIT pwm9_init

/** pwm9_init Configures the PWM with the slave mode
 * @param[in] DIV_ input clock divider to DIV. Allowed range is [1..16].
 * @param[in] DIVN_ input clock divider to 2^DIVN. Allowed range is [0..11].
 * @param[in] LT_ Low threshold value
 * @param[in] HT_ High threshold value
 * @param[in] CMP_ Interrupt comparator register value
 * @param[in] MODE_ PWM mode
 * @param[in] POL_ initial polarity for PWM period
 * @param[in] InvertPolarityIDLE_ POL polarity invertion in the IDLE state. 1 inverts the POL
 */
STATIC INLINE void pwm9_init(uint8_t DIV_, uint8_t DIVN_, uint16_t LT_, uint16_t HT_,
                             uint16_t CMP_, PwmMode_t MODE_, PwmPolarity_t POL_, uint8_t InvertPolarityIDLE_);
#endif /* PWM_SLAVE9 */

#if defined (IO_PWM_SLAVE10__START_GET)
#define PWM10_INIT pwm10_init

/** pwm10_init Configures the PWM with the slave mode
 * @param[in] DIV_ input clock divider to DIV. Allowed range is [1..16].
 * @param[in] DIVN_ input clock divider to 2^DIVN. Allowed range is [0..11].
 * @param[in] LT_ Low threshold value
 * @param[in] HT_ High threshold value
 * @param[in] CMP_ Interrupt comparator register value
 * @param[in] MODE_ PWM mode
 * @param[in] POL_ initial polarity for PWM period
 * @param[in] InvertPolarityIDLE_ POL polarity invertion in the IDLE state. 1 inverts the POL
 */
STATIC INLINE void pwm10_init(uint8_t DIV_, uint8_t DIVN_, uint16_t LT_, uint16_t HT_,
                              uint16_t CMP_, PwmMode_t MODE_, PwmPolarity_t POL_, uint8_t InvertPolarityIDLE_);
#endif /* PWM_SLAVE10 */

#if defined (IO_PWM_SLAVE11__START_GET)
#define PWM11_INIT pwm11_init

/** pwm11_init Configures the PWM with the slave mode
 * @param[in] DIV_ input clock divider to DIV. Allowed range is [1..16].
 * @param[in] DIVN_ input clock divider to 2^DIVN. Allowed range is [0..11].
 * @param[in] LT_ Low threshold value
 * @param[in] HT_ High threshold value
 * @param[in] CMP_ Interrupt comparator register value
 * @param[in] MODE_ PWM mode
 * @param[in] POL_ initial polarity for PWM period
 * @param[in] InvertPolarityIDLE_ POL polarity invertion in the IDLE state. 1 inverts the POL
 */
STATIC INLINE void pwm11_init(uint8_t DIV_, uint8_t DIVN_, uint16_t LT_, uint16_t HT_,
                              uint16_t CMP_, PwmMode_t MODE_, PwmPolarity_t POL_, uint8_t InvertPolarityIDLE_);
#endif /* PWM_SLAVE11 */

#if defined (IO_PWM_SLAVE12__START_GET)
#define PWM12_INIT pwm12_init

/** pwm12_init Configures the PWM with the slave mode
 * @param[in] DIV_ input clock divider to DIV. Allowed range is [1..16].
 * @param[in] DIVN_ input clock divider to 2^DIVN. Allowed range is [0..11].
 * @param[in] LT_ Low threshold value
 * @param[in] HT_ High threshold value
 * @param[in] CMP_ Interrupt comparator register value
 * @param[in] MODE_ PWM mode
 * @param[in] POL_ initial polarity for PWM period
 * @param[in] InvertPolarityIDLE_ POL polarity invertion in the IDLE state. 1 inverts the POL
 */
STATIC INLINE void pwm12_init(uint8_t DIV_, uint8_t DIVN_, uint16_t LT_, uint16_t HT_,
                              uint16_t CMP_, PwmMode_t MODE_, PwmPolarity_t POL_, uint8_t InvertPolarityIDLE_);
#endif /* PWM_SLAVE12 */

#if defined (IO_PWM_SLAVE13__START_GET)
#define PWM13_INIT pwm13_init

/** pwm13_init Configures the PWM with the slave mode
 * @param[in] DIV_ input clock divider to DIV. Allowed range is [1..16].
 * @param[in] DIVN_ input clock divider to 2^DIVN. Allowed range is [0..11].
 * @param[in] LT_ Low threshold value
 * @param[in] HT_ High threshold value
 * @param[in] CMP_ Interrupt comparator register value
 * @param[in] MODE_ PWM mode
 * @param[in] POL_ initial polarity for PWM period
 * @param[in] InvertPolarityIDLE_ POL polarity invertion in the IDLE state. 1 inverts the POL
 */
STATIC INLINE void pwm13_init(uint8_t DIV_, uint8_t DIVN_, uint16_t LT_, uint16_t HT_,
                              uint16_t CMP_, PwmMode_t MODE_, PwmPolarity_t POL_, uint8_t InvertPolarityIDLE_);
#endif /* PWM_SLAVE13 */

#if defined (IO_PWM_SLAVE14__START_GET)
#define PWM14_INIT pwm14_init

/** pwm14_init Configures the PWM with the slave mode
 * @param[in] DIV_ input clock divider to DIV. Allowed range is [1..16].
 * @param[in] DIVN_ input clock divider to 2^DIVN. Allowed range is [0..11].
 * @param[in] LT_ Low threshold value
 * @param[in] HT_ High threshold value
 * @param[in] CMP_ Interrupt comparator register value
 * @param[in] MODE_ PWM mode
 * @param[in] POL_ initial polarity for PWM period
 * @param[in] InvertPolarityIDLE_ POL polarity invertion in the IDLE state. 1 inverts the POL
 */
STATIC INLINE void pwm14_init(uint8_t DIV_, uint8_t DIVN_, uint16_t LT_, uint16_t HT_,
                              uint16_t CMP_, PwmMode_t MODE_, PwmPolarity_t POL_, uint8_t InvertPolarityIDLE_);
#endif /* PWM_SLAVE14 */

#if defined (IO_PWM_SLAVE15__START_GET)
#define PWM15_INIT pwm15_init

/** pwm15_init Configures the PWM with the slave mode
 * @param[in] DIV_ input clock divider to DIV. Allowed range is [1..16].
 * @param[in] DIVN_ input clock divider to 2^DIVN. Allowed range is [0..11].
 * @param[in] LT_ Low threshold value
 * @param[in] HT_ High threshold value
 * @param[in] CMP_ Interrupt comparator register value
 * @param[in] MODE_ PWM mode
 * @param[in] POL_ initial polarity for PWM period
 * @param[in] InvertPolarityIDLE_ POL polarity invertion in the IDLE state. 1 inverts the POL
 */
STATIC INLINE void pwm15_init(uint8_t DIV_, uint8_t DIVN_, uint16_t LT_, uint16_t HT_,
                              uint16_t CMP_, PwmMode_t MODE_, PwmPolarity_t POL_, uint8_t InvertPolarityIDLE_);
#endif /* PWM_SLAVE15 */

#if defined (IO_PWM_SLAVE16__START_GET)
#define PWM16_INIT pwm16_init

/** pwm16_init Configures the PWM with the slave mode
 * @param[in] DIV_ input clock divider to DIV. Allowed range is [1..16].
 * @param[in] DIVN_ input clock divider to 2^DIVN. Allowed range is [0..11].
 * @param[in] LT_ Low threshold value
 * @param[in] HT_ High threshold value
 * @param[in] CMP_ Interrupt comparator register value
 * @param[in] MODE_ PWM mode
 * @param[in] POL_ initial polarity for PWM period
 * @param[in] InvertPolarityIDLE_ POL polarity invertion in the IDLE state. 1 inverts the POL
 */
STATIC INLINE void pwm16_init(uint8_t DIV_, uint8_t DIVN_, uint16_t LT_, uint16_t HT_,
                              uint16_t CMP_, PwmMode_t MODE_, PwmPolarity_t POL_, uint8_t InvertPolarityIDLE_);
#endif /* PWM_SLAVE16 */

#if defined (IO_PWM_SLAVE17__START_GET)
#define PWM17_INIT pwm17_init

/** pwm17_init Configures the PWM with the slave mode
 * @param[in] DIV_ input clock divider to DIV. Allowed range is [1..16].
 * @param[in] DIVN_ input clock divider to 2^DIVN. Allowed range is [0..11].
 * @param[in] LT_ Low threshold value
 * @param[in] HT_ High threshold value
 * @param[in] CMP_ Interrupt comparator register value
 * @param[in] MODE_ PWM mode
 * @param[in] POL_ initial polarity for PWM period
 * @param[in] InvertPolarityIDLE_ POL polarity invertion in the IDLE state. 1 inverts the POL
 */
STATIC INLINE void pwm17_init(uint8_t DIV_, uint8_t DIVN_, uint16_t LT_, uint16_t HT_,
                              uint16_t CMP_, PwmMode_t MODE_, PwmPolarity_t POL_, uint8_t InvertPolarityIDLE_);
#endif /* PWM_SLAVE17 */

#if defined (IO_PWM_SLAVE18__START_GET)
#define PWM18_INIT pwm18_init

/** pwm18_init Configures the PWM with the slave mode
 * @param[in] DIV_ input clock divider to DIV. Allowed range is [1..16].
 * @param[in] DIVN_ input clock divider to 2^DIVN. Allowed range is [0..11].
 * @param[in] LT_ Low threshold value
 * @param[in] HT_ High threshold value
 * @param[in] CMP_ Interrupt comparator register value
 * @param[in] MODE_ PWM mode
 * @param[in] POL_ initial polarity for PWM period
 * @param[in] InvertPolarityIDLE_ POL polarity invertion in the IDLE state. 1 inverts the POL
 */
STATIC INLINE void pwm18_init(uint8_t DIV_, uint8_t DIVN_, uint16_t LT_, uint16_t HT_,
                              uint16_t CMP_, PwmMode_t MODE_, PwmPolarity_t POL_, uint8_t InvertPolarityIDLE_);
#endif /* PWM_SLAVE18 */

#if defined (IO_PWM_SLAVE19__START_GET)
#define PWM19_INIT pwm19_init

/** pwm19_init Configures the PWM with the slave mode
 * @param[in] DIV_ input clock divider to DIV. Allowed range is [1..16].
 * @param[in] DIVN_ input clock divider to 2^DIVN. Allowed range is [0..11].
 * @param[in] LT_ Low threshold value
 * @param[in] HT_ High threshold value
 * @param[in] CMP_ Interrupt comparator register value
 * @param[in] MODE_ PWM mode
 * @param[in] POL_ initial polarity for PWM period
 * @param[in] InvertPolarityIDLE_ POL polarity invertion in the IDLE state. 1 inverts the POL
 */
STATIC INLINE void pwm19_init(uint8_t DIV_, uint8_t DIVN_, uint16_t LT_, uint16_t HT_,
                              uint16_t CMP_, PwmMode_t MODE_, PwmPolarity_t POL_, uint8_t InvertPolarityIDLE_);
#endif /* PWM_SLAVE19 */

#if defined (IO_PWM_SLAVE20__START_GET)
#define PWM20_INIT pwm20_init

/** pwm20_init Configures the PWM with the slave mode
 * @param[in] DIV_ input clock divider to DIV. Allowed range is [1..16].
 * @param[in] DIVN_ input clock divider to 2^DIVN. Allowed range is [0..11].
 * @param[in] LT_ Low threshold value
 * @param[in] HT_ High threshold value
 * @param[in] CMP_ Interrupt comparator register value
 * @param[in] MODE_ PWM mode
 * @param[in] POL_ initial polarity for PWM period
 * @param[in] InvertPolarityIDLE_ POL polarity invertion in the IDLE state. 1 inverts the POL
 */
STATIC INLINE void pwm20_init(uint8_t DIV_, uint8_t DIVN_, uint16_t LT_, uint16_t HT_,
                              uint16_t CMP_, PwmMode_t MODE_, PwmPolarity_t POL_, uint8_t InvertPolarityIDLE_);
#endif /* PWM_SLAVE20 */

#if defined (IO_PWM_SLAVE21__START_GET)
#define PWM21_INIT pwm21_init

/** pwm21_init Configures the PWM with the slave mode
 * @param[in] DIV_ input clock divider to DIV. Allowed range is [1..16].
 * @param[in] DIVN_ input clock divider to 2^DIVN. Allowed range is [0..11].
 * @param[in] LT_ Low threshold value
 * @param[in] HT_ High threshold value
 * @param[in] CMP_ Interrupt comparator register value
 * @param[in] MODE_ PWM mode
 * @param[in] POL_ initial polarity for PWM period
 * @param[in] InvertPolarityIDLE_ POL polarity invertion in the IDLE state. 1 inverts the POL
 */
STATIC INLINE void pwm21_init(uint8_t DIV_, uint8_t DIVN_, uint16_t LT_, uint16_t HT_,
                              uint16_t CMP_, PwmMode_t MODE_, PwmPolarity_t POL_, uint8_t InvertPolarityIDLE_);
#endif /* PWM_SLAVE21 */

#if defined (IO_PWM_SLAVE22__START_GET)
#define PWM22_INIT pwm22_init

/** pwm22_init Configures the PWM with the slave mode
 * @param[in] DIV_ input clock divider to DIV. Allowed range is [1..16].
 * @param[in] DIVN_ input clock divider to 2^DIVN. Allowed range is [0..11].
 * @param[in] LT_ Low threshold value
 * @param[in] HT_ High threshold value
 * @param[in] CMP_ Interrupt comparator register value
 * @param[in] MODE_ PWM mode
 * @param[in] POL_ initial polarity for PWM period
 * @param[in] InvertPolarityIDLE_ POL polarity invertion in the IDLE state. 1 inverts the POL
 */
STATIC INLINE void pwm22_init(uint8_t DIV_, uint8_t DIVN_, uint16_t LT_, uint16_t HT_,
                              uint16_t CMP_, PwmMode_t MODE_, PwmPolarity_t POL_, uint8_t InvertPolarityIDLE_);
#endif /* PWM_SLAVE22 */

#if defined (IO_PWM_SLAVE23__START_GET)
#define PWM23_INIT pwm23_init

/** pwm23_init Configures the PWM with the slave mode
 * @param[in] DIV_ input clock divider to DIV. Allowed range is [1..16].
 * @param[in] DIVN_ input clock divider to 2^DIVN. Allowed range is [0..11].
 * @param[in] LT_ Low threshold value
 * @param[in] HT_ High threshold value
 * @param[in] CMP_ Interrupt comparator register value
 * @param[in] MODE_ PWM mode
 * @param[in] POL_ initial polarity for PWM period
 * @param[in] InvertPolarityIDLE_ POL polarity invertion in the IDLE state. 1 inverts the POL
 */
STATIC INLINE void pwm23_init(uint8_t DIV_, uint8_t DIVN_, uint16_t LT_, uint16_t HT_,
                              uint16_t CMP_, PwmMode_t MODE_, PwmPolarity_t POL_, uint8_t InvertPolarityIDLE_);
#endif /* PWM_SLAVE23 */

#if defined (IO_PWM_SLAVE24__START_GET)
#define PWM24_INIT pwm24_init

/** pwm24_init Configures the PWM with the slave mode
 * @param[in] DIV_ input clock divider to DIV. Allowed range is [1..16].
 * @param[in] DIVN_ input clock divider to 2^DIVN. Allowed range is [0..11].
 * @param[in] LT_ Low threshold value
 * @param[in] HT_ High threshold value
 * @param[in] CMP_ Interrupt comparator register value
 * @param[in] MODE_ PWM mode
 * @param[in] POL_ initial polarity for PWM period
 * @param[in] InvertPolarityIDLE_ POL polarity invertion in the IDLE state. 1 inverts the POL
 */
STATIC INLINE void pwm24_init(uint8_t DIV_, uint8_t DIVN_, uint16_t LT_, uint16_t HT_,
                              uint16_t CMP_, PwmMode_t MODE_, PwmPolarity_t POL_, uint8_t InvertPolarityIDLE_);
#endif /* PWM_SLAVE24 */




#if defined (IO_PWM_MASTER2__START_GET)
#define PWM2_INIT_MASTER_SLAVE pwm2_init_master_slave

STATIC INLINE void pwm2_init_master_slave(uint8_t DIV_,
                                          uint8_t DIVN_,
                                          uint16_t LT_,
                                          uint16_t HT_,
                                          uint16_t CMP_,
                                          PwmMode_t MODE_,
                                          PwmPolarity_t POL_,
                                          uint8_t InvertPolarityIDLE_);
#endif /* PWM_MASTER2 */




#if defined (IO_PWM_MASTER1__START_GET)
#define PWM1_MASTER_GET_INPUT pwm1_master_get_input

/** pwm1_master_get_input returns the current PWM master state
 * @return the current PWM state
 */
STATIC INLINE uint8_t pwm1_master_get_input(void);
#endif /*PWM_MASTER1*/

#if defined (IO_PWM_MASTER2__START_GET)
#define PWM2_MASTER_GET_INPUT pwm2_master_get_input

/** pwm2_master_get_input returns the current PWM master state
 * @return the current PWM state
 */
STATIC INLINE uint8_t pwm2_master_get_input(void);
#endif /*PWM_MASTER2*/

#if defined (IO_PWM_SLAVE1__START_GET)
#define PWM1_GET_INPUT pwm1_get_input

/** pwm1_get_input returns the current PWM slave state
 * @return the current PWM state
 */
STATIC INLINE uint8_t pwm1_get_input(void);
#endif /* PWM_SLAVE1 */

#if defined (IO_PWM_SLAVE2__START_GET)
#define PWM2_GET_INPUT pwm2_get_input

/** pwm2_get_input returns the current PWM slave state
 * @return the current PWM state
 */
STATIC INLINE uint8_t pwm2_get_input(void);
#endif /* PWM_SLAVE2 */

#if defined (IO_PWM_SLAVE3__START_GET)
#define PWM3_GET_INPUT pwm3_get_input

/** pwm3_get_input returns the current PWM slave state
 * @return the current PWM state
 */
STATIC INLINE uint8_t pwm3_get_input(void);
#endif /* PWM_SLAVE3 */

#if defined (IO_PWM_SLAVE4__START_GET)
#define PWM4_GET_INPUT pwm4_get_input

/** pwm4_get_input returns the current PWM slave state
 * @return the current PWM state
 */
STATIC INLINE uint8_t pwm4_get_input(void);
#endif /* PWM_SLAVE4 */

#if defined (IO_PWM_SLAVE5__START_GET)
#define PWM5_GET_INPUT pwm5_get_input

/** pwm5_get_input returns the current PWM slave state
 * @return the current PWM state
 */
STATIC INLINE uint8_t pwm5_get_input(void);
#endif /* PWM_SLAVE5 */

#if defined (IO_PWM_SLAVE6__START_GET)
#define PWM6_GET_INPUT pwm6_get_input

/** pwm6_get_input returns the current PWM slave state
 * @return the current PWM state
 */
STATIC INLINE uint8_t pwm6_get_input(void);
#endif /* PWM_SLAVE6 */

#if defined (IO_PWM_SLAVE7__START_GET)
#define PWM7_GET_INPUT pwm7_get_input

/** pwm7_get_input returns the current PWM slave state
 * @return the current PWM state
 */
STATIC INLINE uint8_t pwm7_get_input(void);
#endif /* PWM_SLAVE7 */

#if defined (IO_PWM_SLAVE8__START_GET)
#define PWM8_GET_INPUT pwm8_get_input

/** pwm8_get_input returns the current PWM slave state
 * @return the current PWM state
 */
STATIC INLINE uint8_t pwm8_get_input(void);
#endif /* PWM_SLAVE8 */

#if defined (IO_PWM_SLAVE9__START_GET)
#define PWM9_GET_INPUT pwm9_get_input

/** pwm9_get_input returns the current PWM slave state
 * @return the current PWM state
 */
STATIC INLINE uint8_t pwm9_get_input(void);
#endif /* PWM_SLAVE9 */

#if defined (IO_PWM_SLAVE10__START_GET)
#define PWM10_GET_INPUT pwm10_get_input

/** pwm10_get_input returns the current PWM slave state
 * @return the current PWM state
 */
STATIC INLINE uint8_t pwm10_get_input(void);
#endif /* PWM_SLAVE10 */

#if defined (IO_PWM_SLAVE11__START_GET)
#define PWM11_GET_INPUT pwm11_get_input

/** pwm11_get_input returns the current PWM slave state
 * @return the current PWM state
 */
STATIC INLINE uint8_t pwm11_get_input(void);
#endif /* PWM_SLAVE11 */

#if defined (IO_PWM_SLAVE12__START_GET)
#define PWM12_GET_INPUT pwm12_get_input

/** pwm12_get_input returns the current PWM slave state
 * @return the current PWM state
 */
STATIC INLINE uint8_t pwm12_get_input(void);
#endif /* PWM_SLAVE12 */

#if defined (IO_PWM_SLAVE13__START_GET)
#define PWM13_GET_INPUT pwm13_get_input

/** pwm13_get_input returns the current PWM slave state
 * @return the current PWM state
 */
STATIC INLINE uint8_t pwm13_get_input(void);
#endif /* PWM_SLAVE13 */

#if defined (IO_PWM_SLAVE14__START_GET)
#define PWM14_GET_INPUT pwm14_get_input

/** pwm14_get_input returns the current PWM slave state
 * @return the current PWM state
 */
STATIC INLINE uint8_t pwm14_get_input(void);
#endif /* PWM_SLAVE14 */

#if defined (IO_PWM_SLAVE15__START_GET)
#define PWM15_GET_INPUT pwm15_get_input

/** pwm15_get_input returns the current PWM slave state
 * @return the current PWM state
 */
STATIC INLINE uint8_t pwm15_get_input(void);
#endif /* PWM_SLAVE15 */

#if defined (IO_PWM_SLAVE16__START_GET)
#define PWM16_GET_INPUT pwm16_get_input

/** pwm16_get_input returns the current PWM slave state
 * @return the current PWM state
 */
STATIC INLINE uint8_t pwm16_get_input(void);
#endif /* PWM_SLAVE16 */

#if defined (IO_PWM_SLAVE17__START_GET)
#define PWM17_GET_INPUT pwm17_get_input

/** pwm17_get_input returns the current PWM slave state
 * @return the current PWM state
 */
STATIC INLINE uint8_t pwm17_get_input(void);
#endif /* PWM_SLAVE17 */

#if defined (IO_PWM_SLAVE18__START_GET)
#define PWM18_GET_INPUT pwm18_get_input

/** pwm18_get_input returns the current PWM slave state
 * @return the current PWM state
 */
STATIC INLINE uint8_t pwm18_get_input(void);
#endif /* PWM_SLAVE18 */

#if defined (IO_PWM_SLAVE19__START_GET)
#define PWM19_GET_INPUT pwm19_get_input

/** pwm19_get_input returns the current PWM slave state
 * @return the current PWM state
 */
STATIC INLINE uint8_t pwm19_get_input(void);
#endif /* PWM_SLAVE19 */

#if defined (IO_PWM_SLAVE20__START_GET)
#define PWM20_GET_INPUT pwm20_get_input

/** pwm20_get_input returns the current PWM slave state
 * @return the current PWM state
 */
STATIC INLINE uint8_t pwm20_get_input(void);
#endif /* PWM_SLAVE20 */

#if defined (IO_PWM_SLAVE21__START_GET)
#define PWM21_GET_INPUT pwm21_get_input

/** pwm21_get_input returns the current PWM slave state
 * @return the current PWM state
 */
STATIC INLINE uint8_t pwm21_get_input(void);
#endif /* PWM_SLAVE21 */

#if defined (IO_PWM_SLAVE22__START_GET)
#define PWM22_GET_INPUT pwm22_get_input

/** pwm22_get_input returns the current PWM slave state
 * @return the current PWM state
 */
STATIC INLINE uint8_t pwm22_get_input(void);
#endif /* PWM_SLAVE22 */

#if defined (IO_PWM_SLAVE23__START_GET)
#define PWM23_GET_INPUT pwm23_get_input

/** pwm23_get_input returns the current PWM slave state
 * @return the current PWM state
 */
STATIC INLINE uint8_t pwm23_get_input(void);
#endif /* PWM_SLAVE23 */

#if defined (IO_PWM_SLAVE24__START_GET)
#define PWM24_GET_INPUT pwm24_get_input

/** pwm24_get_input returns the current PWM slave state
 * @return the current PWM state
 */
STATIC INLINE uint8_t pwm24_get_input(void);
#endif /* PWM_SLAVE24 */




#if defined (IO_PWM_MASTER1__START_GET)
#define PWM1_MASTER_ENABLE pwm1_master_enable

/** pwm1_master_enable starts the PWM channel */
STATIC INLINE void pwm1_master_enable(void);
#endif /* PWM_MASTER1 */

#if defined (IO_PWM_MASTER2__START_GET)
#define PWM2_MASTER_ENABLE pwm2_master_enable

/** pwm2_master_enable starts the PWM channel */
STATIC INLINE void pwm2_master_enable(void);
#endif /* PWM_MASTER2 */




#if defined (IO_PWM_SLAVE1__START_GET)
#define PWM1_ENABLE pwm1_enable

STATIC INLINE void pwm1_enable(void);
#endif /* PWM_SLAVE1 */

#if defined (IO_PWM_SLAVE2__START_GET)
#define PWM2_ENABLE pwm2_enable

STATIC INLINE void pwm2_enable(void);
#endif /* PWM_SLAVE2 */

#if defined (IO_PWM_SLAVE3__START_GET)
#define PWM3_ENABLE pwm3_enable

STATIC INLINE void pwm3_enable(void);
#endif /* PWM_SLAVE3 */

#if defined (IO_PWM_SLAVE4__START_GET)
#define PWM4_ENABLE pwm4_enable

STATIC INLINE void pwm4_enable(void);
#endif /* PWM_SLAVE4 */

#if defined (IO_PWM_SLAVE5__START_GET)
#define PWM5_ENABLE pwm5_enable

STATIC INLINE void pwm5_enable(void);
#endif /* PWM_SLAVE5 */

#if defined (IO_PWM_SLAVE6__START_GET)
#define PWM6_ENABLE pwm6_enable

STATIC INLINE void pwm6_enable(void);
#endif /* PWM_SLAVE6 */

#if defined (IO_PWM_SLAVE7__START_GET)
#define PWM7_ENABLE pwm7_enable

STATIC INLINE void pwm7_enable(void);
#endif /* PWM_SLAVE7 */

#if defined (IO_PWM_SLAVE8__START_GET)
#define PWM8_ENABLE pwm8_enable

STATIC INLINE void pwm8_enable(void);
#endif /* PWM_SLAVE8 */

#if defined (IO_PWM_SLAVE9__START_GET)
#define PWM9_ENABLE pwm9_enable

STATIC INLINE void pwm9_enable(void);
#endif /* PWM_SLAVE9 */

#if defined (IO_PWM_SLAVE10__START_GET)
#define PWM10_ENABLE pwm10_enable

STATIC INLINE void pwm10_enable(void);
#endif /* PWM_SLAVE10 */

#if defined (IO_PWM_SLAVE11__START_GET)
#define PWM11_ENABLE pwm11_enable

STATIC INLINE void pwm11_enable(void);
#endif /* PWM_SLAVE11 */

#if defined (IO_PWM_SLAVE12__START_GET)
#define PWM12_ENABLE pwm12_enable

STATIC INLINE void pwm12_enable(void);
#endif /* PWM_SLAVE12 */

#if defined (IO_PWM_SLAVE13__START_GET)
#define PWM13_ENABLE pwm13_enable

STATIC INLINE void pwm13_enable(void);
#endif /* PWM_SLAVE13 */

#if defined (IO_PWM_SLAVE14__START_GET)
#define PWM14_ENABLE pwm14_enable

STATIC INLINE void pwm14_enable(void);
#endif /* PWM_SLAVE14 */

#if defined (IO_PWM_SLAVE15__START_GET)
#define PWM15_ENABLE pwm15_enable

STATIC INLINE void pwm15_enable(void);
#endif /* PWM_SLAVE15 */

#if defined (IO_PWM_SLAVE16__START_GET)
#define PWM16_ENABLE pwm16_enable

STATIC INLINE void pwm16_enable(void);
#endif /* PWM_SLAVE16 */

#if defined (IO_PWM_SLAVE17__START_GET)
#define PWM17_ENABLE pwm17_enable

STATIC INLINE void pwm17_enable(void);
#endif /* PWM_SLAVE17 */

#if defined (IO_PWM_SLAVE18__START_GET)
#define PWM18_ENABLE pwm18_enable

STATIC INLINE void pwm18_enable(void);
#endif /* PWM_SLAVE18 */

#if defined (IO_PWM_SLAVE19__START_GET)
#define PWM19_ENABLE pwm19_enable

STATIC INLINE void pwm19_enable(void);
#endif /* PWM_SLAVE19 */

#if defined (IO_PWM_SLAVE20__START_GET)
#define PWM20_ENABLE pwm20_enable

STATIC INLINE void pwm20_enable(void);
#endif /* PWM_SLAVE20 */

#if defined (IO_PWM_SLAVE21__START_GET)
#define PWM21_ENABLE pwm21_enable

STATIC INLINE void pwm21_enable(void);
#endif /* PWM_SLAVE21 */

#if defined (IO_PWM_SLAVE22__START_GET)
#define PWM22_ENABLE pwm22_enable

STATIC INLINE void pwm22_enable(void);
#endif /* PWM_SLAVE22 */

#if defined (IO_PWM_SLAVE23__START_GET)
#define PWM23_ENABLE pwm23_enable

STATIC INLINE void pwm23_enable(void);
#endif /* PWM_SLAVE23 */

#if defined (IO_PWM_SLAVE24__START_GET)
#define PWM24_ENABLE pwm24_enable

STATIC INLINE void pwm24_enable(void);
#endif /* PWM_SLAVE24 */




#if defined (IO_PWM_MASTER1__START_GET)
#define PWM1_MASTER_DISABLE pwm1_master_disable

/** pwm1_master_disable stops the PWM channel */
STATIC INLINE void pwm1_master_disable(void);
#endif /* PWM_MASTER1 */

#if defined (IO_PWM_MASTER2__START_GET)
#define PWM2_MASTER_DISABLE pwm2_master_disable

/** pwm2_master_disable stops the PWM channel */
STATIC INLINE void pwm2_master_disable(void);
#endif /* PWM_MASTER2 */




#if defined (IO_PWM_SLAVE1__START_GET)
#define PWM1_DISABLE pwm1_disable

/** pwm1_disable stops the PWM channel */
STATIC INLINE void pwm1_disable(void);
#endif /* PWM_SLAVE1 */

#if defined (IO_PWM_SLAVE2__START_GET)
#define PWM2_DISABLE pwm2_disable

/** pwm2_disable stops the PWM channel */
STATIC INLINE void pwm2_disable(void);
#endif /* PWM_SLAVE2 */

#if defined (IO_PWM_SLAVE3__START_GET)
#define PWM3_DISABLE pwm3_disable

/** pwm3_disable stops the PWM channel */
STATIC INLINE void pwm3_disable(void);
#endif /* PWM_SLAVE3 */

#if defined (IO_PWM_SLAVE4__START_GET)
#define PWM4_DISABLE pwm4_disable

/** pwm4_disable stops the PWM channel */
STATIC INLINE void pwm4_disable(void);
#endif /* PWM_SLAVE4 */

#if defined (IO_PWM_SLAVE5__START_GET)
#define PWM5_DISABLE pwm5_disable

/** pwm5_disable stops the PWM channel */
STATIC INLINE void pwm5_disable(void);
#endif /* PWM_SLAVE5 */

#if defined (IO_PWM_SLAVE6__START_GET)
#define PWM6_DISABLE pwm6_disable

/** pwm6_disable stops the PWM channel */
STATIC INLINE void pwm6_disable(void);
#endif /* PWM_SLAVE6 */

#if defined (IO_PWM_SLAVE7__START_GET)
#define PWM7_DISABLE pwm7_disable

/** pwm7_disable stops the PWM channel */
STATIC INLINE void pwm7_disable(void);
#endif /* PWM_SLAVE7 */

#if defined (IO_PWM_SLAVE8__START_GET)
#define PWM8_DISABLE pwm8_disable

/** pwm8_disable stops the PWM channel */
STATIC INLINE void pwm8_disable(void);
#endif /* PWM_SLAVE8 */

#if defined (IO_PWM_SLAVE9__START_GET)
#define PWM9_DISABLE pwm9_disable

/** pwm9_disable stops the PWM channel */
STATIC INLINE void pwm9_disable(void);
#endif /* PWM_SLAVE9 */

#if defined (IO_PWM_SLAVE10__START_GET)
#define PWM10_DISABLE pwm10_disable

/** pwm10_disable stops the PWM channel */
STATIC INLINE void pwm10_disable(void);
#endif /* PWM_SLAVE10 */

#if defined (IO_PWM_SLAVE11__START_GET)
#define PWM11_DISABLE pwm11_disable

/** pwm11_disable stops the PWM channel */
STATIC INLINE void pwm11_disable(void);
#endif /* PWM_SLAVE11 */

#if defined (IO_PWM_SLAVE12__START_GET)
#define PWM12_DISABLE pwm12_disable

/** pwm12_disable stops the PWM channel */
STATIC INLINE void pwm12_disable(void);
#endif /* PWM_SLAVE12 */

#if defined (IO_PWM_SLAVE13__START_GET)
#define PWM13_DISABLE pwm13_disable

/** pwm13_disable stops the PWM channel */
STATIC INLINE void pwm13_disable(void);
#endif /* PWM_SLAVE13 */

#if defined (IO_PWM_SLAVE14__START_GET)
#define PWM14_DISABLE pwm14_disable

/** pwm14_disable stops the PWM channel */
STATIC INLINE void pwm14_disable(void);
#endif /* PWM_SLAVE14 */

#if defined (IO_PWM_SLAVE15__START_GET)
#define PWM15_DISABLE pwm15_disable

/** pwm15_disable stops the PWM channel */
STATIC INLINE void pwm15_disable(void);
#endif /* PWM_SLAVE15 */

#if defined (IO_PWM_SLAVE16__START_GET)
#define PWM16_DISABLE pwm16_disable

/** pwm16_disable stops the PWM channel */
STATIC INLINE void pwm16_disable(void);
#endif /* PWM_SLAVE16 */

#if defined (IO_PWM_SLAVE17__START_GET)
#define PWM17_DISABLE pwm17_disable

/** pwm17_disable stops the PWM channel */
STATIC INLINE void pwm17_disable(void);
#endif /* PWM_SLAVE17 */

#if defined (IO_PWM_SLAVE18__START_GET)
#define PWM18_DISABLE pwm18_disable

/** pwm18_disable stops the PWM channel */
STATIC INLINE void pwm18_disable(void);
#endif /* PWM_SLAVE18 */

#if defined (IO_PWM_SLAVE19__START_GET)
#define PWM19_DISABLE pwm19_disable

/** pwm19_disable stops the PWM channel */
STATIC INLINE void pwm19_disable(void);
#endif /* PWM_SLAVE19 */

#if defined (IO_PWM_SLAVE20__START_GET)
#define PWM20_DISABLE pwm20_disable

/** pwm20_disable stops the PWM channel */
STATIC INLINE void pwm20_disable(void);
#endif /* PWM_SLAVE20 */

#if defined (IO_PWM_SLAVE21__START_GET)
#define PWM21_DISABLE pwm21_disable

/** pwm21_disable stops the PWM channel */
STATIC INLINE void pwm21_disable(void);
#endif /* PWM_SLAVE21 */

#if defined (IO_PWM_SLAVE22__START_GET)
#define PWM22_DISABLE pwm22_disable

/** pwm22_disable stops the PWM channel */
STATIC INLINE void pwm22_disable(void);
#endif /* PWM_SLAVE22 */

#if defined (IO_PWM_SLAVE23__START_GET)
#define PWM23_DISABLE pwm23_disable

/** pwm23_disable stops the PWM channel */
STATIC INLINE void pwm23_disable(void);
#endif /* PWM_SLAVE23 */

#if defined (IO_PWM_SLAVE24__START_GET)
#define PWM24_DISABLE pwm24_disable

/** pwm24_disable stops the PWM channel */
STATIC INLINE void pwm24_disable(void);
#endif /* PWM_SLAVE24 */




#define __PWM_INT_DISABLE(n)    \
    Itc_Disable(n)

#if defined (IO_PWM_MASTER1__START_GET)
#define PWM1_MASTER_CMPI_ENABLE pwm1_master_cmpi_enable

/** pwm1_master_cmpi_enable enables the compare interrupt for the PWM master's channel */
STATIC INLINE void pwm1_master_cmpi_enable(void);

#define PWM1_MASTER_CMPI_DISABLE pwm1_master_cmpi_disable

/** pwm1_master_cmpi_disable disables the compare interrupt for the PWM master's channel */
STATIC INLINE void pwm1_master_cmpi_disable(void);

#define PWM1_MASTER_CNTI_ENABLE pwm1_master_cnti_enable

/** pwm1_master_cnti_enable enables the end-of-period interrupt for the PWM master's channel */
STATIC INLINE void pwm1_master_cnti_enable(void);

#define PWM1_MASTER_CNTI_DISABLE pwm1_master_cnti_disable

/** pwm1_master_cnti_disable disables the end-of-period interrupt for the PWM master's channel */
STATIC INLINE void pwm1_master_cnti_disable(void);

#endif /* PWM_MASTER1 */

#if defined (IO_PWM_MASTER2__START_GET)
#define PWM2_MASTER_CMPI_ENABLE pwm2_master_cmpi_enable

/** pwm2_master_cmpi_enable enables the compare interrupt for the PWM master's channel */
STATIC INLINE void pwm2_master_cmpi_enable(void);

#define PWM2_MASTER_CMPI_DISABLE pwm2_master_cmpi_disable

/** pwm2_master_cmpi_disable disables the compare interrupt for the PWM master's channel */
STATIC INLINE void pwm2_master_cmpi_disable(void);

#define PWM2_MASTER_CNTI_ENABLE pwm2_master_cnti_enable

/** pwm2_master_cnti_enable enables the end-of-period interrupt for the PWM master's channel */
STATIC INLINE void pwm2_master_cnti_enable(void);

#define PWM2_MASTER_CNTI_DISABLE pwm2_master_cnti_disable

/** pwm2_master_cnti_disable disables the end-of-period interrupt for the PWM master's channel */
STATIC INLINE void pwm2_master_cnti_disable(void);

#endif /* PWM_MASTER2 */

#if defined (IO_PWM_SLAVE1__START_GET)
#define PWM1_CMPI_ENABLE pwm1_cmpi_enable

/** pwm1_cmpi_enable enables the compare interrupt for the PWM1 slave's channel */
STATIC INLINE void pwm1_cmpi_enable(void);

#define PWM1_CMPI_DISABLE pwm1_cmpi_disable

/** pwm1_cmpi_disable disables the compare interrupt for the PWM1 slave's channel */
STATIC INLINE void pwm1_cmpi_disable(void);
#endif /* PWM_SLAVE1 */

#if defined (IO_PWM_SLAVE2__START_GET)
#define PWM2_CMPI_ENABLE pwm2_cmpi_enable

/** pwm2_cmpi_enable enables the compare interrupt for the PWM2 slave's channel */
STATIC INLINE void pwm2_cmpi_enable(void);

#define PWM2_CMPI_DISABLE pwm2_cmpi_disable

/** pwm2_cmpi_disable disables the compare interrupt for the PWM2 slave's channel */
STATIC INLINE void pwm2_cmpi_disable(void);
#endif /* PWM_SLAVE2 */

#if defined (IO_PWM_SLAVE3__START_GET)
#define PWM3_CMPI_ENABLE pwm3_cmpi_enable

/** pwm3_cmpi_enable enables the compare interrupt for the PWM3 slave's channel */
STATIC INLINE void pwm3_cmpi_enable(void);

#define PWM3_CMPI_DISABLE pwm3_cmpi_disable

/** pwm3_cmpi_disable disables the compare interrupt for the PWM3 slave's channel */
STATIC INLINE void pwm3_cmpi_disable(void);
#endif /* PWM_SLAVE3 */

#if defined (IO_PWM_SLAVE4__START_GET)
#define PWM4_CMPI_ENABLE pwm4_cmpi_enable

/** pwm4_cmpi_enable enables the compare interrupt for the PWM4 slave's channel */
STATIC INLINE void pwm4_cmpi_enable(void);

#define PWM4_CMPI_DISABLE pwm4_cmpi_disable

/** pwm4_cmpi_disable disables the compare interrupt for the PWM4 slave's channel */
STATIC INLINE void pwm4_cmpi_disable(void);
#endif /* PWM_SLAVE4 */

#if defined (IO_PWM_SLAVE5__START_GET)
#define PWM5_CMPI_ENABLE pwm5_cmpi_enable

/** pwm5_cmpi_enable enables the compare interrupt for the PWM5 slave's channel */
STATIC INLINE void pwm5_cmpi_enable(void);

#define PWM5_CMPI_DISABLE pwm5_cmpi_disable

/** pwm5_cmpi_disable disables the compare interrupt for the PWM5 slave's channel */
STATIC INLINE void pwm5_cmpi_disable(void);
#endif /* PWM_SLAVE5 */

#if defined (IO_PWM_SLAVE6__START_GET)
#define PWM6_CMPI_ENABLE pwm6_cmpi_enable

/** pwm6_cmpi_enable enables the compare interrupt for the PWM6 slave's channel */
STATIC INLINE void pwm6_cmpi_enable(void);

#define PWM6_CMPI_DISABLE pwm6_cmpi_disable

/** pwm6_cmpi_disable disables the compare interrupt for the PWM6 slave's channel */
STATIC INLINE void pwm6_cmpi_disable(void);
#endif /* PWM_SLAVE6 */

#if defined (IO_PWM_SLAVE7__START_GET)
#define PWM7_CMPI_ENABLE pwm7_cmpi_enable

/** pwm7_cmpi_enable enables the compare interrupt for the PWM7 slave's channel */
STATIC INLINE void pwm7_cmpi_enable(void);

#define PWM7_CMPI_DISABLE pwm7_cmpi_disable

/** pwm7_cmpi_disable disables the compare interrupt for the PWM7 slave's channel */
STATIC INLINE void pwm7_cmpi_disable(void);
#endif /* PWM_SLAVE7 */

#if defined (IO_PWM_SLAVE8__START_GET)
#define PWM8_CMPI_ENABLE pwm8_cmpi_enable

/** pwm8_cmpi_enable enables the compare interrupt for the PWM8 slave's channel */
STATIC INLINE void pwm8_cmpi_enable(void);

#define PWM8_CMPI_DISABLE pwm8_cmpi_disable

/** pwm8_cmpi_disable disables the compare interrupt for the PWM8 slave's channel */
STATIC INLINE void pwm8_cmpi_disable(void);
#endif /* PWM_SLAVE8 */

#if defined (IO_PWM_SLAVE9__START_GET)
#define PWM9_CMPI_ENABLE pwm9_cmpi_enable

/** pwm9_cmpi_enable enables the compare interrupt for the PWM9 slave's channel */
STATIC INLINE void pwm9_cmpi_enable(void);

#define PWM9_CMPI_DISABLE pwm9_cmpi_disable

/** pwm9_cmpi_disable disables the compare interrupt for the PWM9 slave's channel */
STATIC INLINE void pwm9_cmpi_disable(void);
#endif /* PWM_SLAVE9 */

#if defined (IO_PWM_SLAVE10__START_GET)
#define PWM10_CMPI_ENABLE pwm10_cmpi_enable

/** pwm10_cmpi_enable enables the compare interrupt for the PWM10 slave's channel */
STATIC INLINE void pwm10_cmpi_enable(void);

#define PWM10_CMPI_DISABLE pwm10_cmpi_disable

/** pwm10_cmpi_disable disables the compare interrupt for the PWM10 slave's channel */
STATIC INLINE void pwm10_cmpi_disable(void);
#endif /* PWM_SLAVE10 */

#if defined (IO_PWM_SLAVE11__START_GET)
#define PWM11_CMPI_ENABLE pwm11_cmpi_enable

/** pwm11_cmpi_enable enables the compare interrupt for the PWM11 slave's channel */
STATIC INLINE void pwm11_cmpi_enable(void);

#define PWM11_CMPI_DISABLE pwm11_cmpi_disable

/** pwm11_cmpi_disable disables the compare interrupt for the PWM11 slave's channel */
STATIC INLINE void pwm11_cmpi_disable(void);
#endif /* PWM_SLAVE11 */

#if defined (IO_PWM_SLAVE12__START_GET)
#define PWM12_CMPI_ENABLE pwm12_cmpi_enable

/** pwm12_cmpi_enable enables the compare interrupt for the PWM12 slave's channel */
STATIC INLINE void pwm12_cmpi_enable(void);

#define PWM12_CMPI_DISABLE pwm12_cmpi_disable

/** pwm12_cmpi_disable disables the compare interrupt for the PWM12 slave's channel */
STATIC INLINE void pwm12_cmpi_disable(void);
#endif /* PWM_SLAVE12 */

#if defined (IO_PWM_SLAVE13__START_GET)
#define PWM13_CMPI_ENABLE pwm13_cmpi_enable

/** pwm13_cmpi_enable enables the compare interrupt for the PWM13 slave's channel */
STATIC INLINE void pwm13_cmpi_enable(void);

#define PWM13_CMPI_DISABLE pwm13_cmpi_disable

/** pwm13_cmpi_disable disables the compare interrupt for the PWM13 slave's channel */
STATIC INLINE void pwm13_cmpi_disable(void);
#endif /* PWM_SLAVE13 */

#if defined (IO_PWM_SLAVE14__START_GET)
#define PWM14_CMPI_ENABLE pwm14_cmpi_enable

/** pwm14_cmpi_enable enables the compare interrupt for the PWM14 slave's channel */
STATIC INLINE void pwm14_cmpi_enable(void);

#define PWM14_CMPI_DISABLE pwm14_cmpi_disable

/** pwm14_cmpi_disable disables the compare interrupt for the PWM14 slave's channel */
STATIC INLINE void pwm14_cmpi_disable(void);
#endif /* PWM_SLAVE14 */

#if defined (IO_PWM_SLAVE15__START_GET)
#define PWM15_CMPI_ENABLE pwm15_cmpi_enable

/** pwm15_cmpi_enable enables the compare interrupt for the PWM15 slave's channel */
STATIC INLINE void pwm15_cmpi_enable(void);

#define PWM15_CMPI_DISABLE pwm15_cmpi_disable

/** pwm15_cmpi_disable disables the compare interrupt for the PWM15 slave's channel */
STATIC INLINE void pwm15_cmpi_disable(void);
#endif /* PWM_SLAVE15 */

#if defined (IO_PWM_SLAVE16__START_GET)
#define PWM16_CMPI_ENABLE pwm16_cmpi_enable

/** pwm16_cmpi_enable enables the compare interrupt for the PWM16 slave's channel */
STATIC INLINE void pwm16_cmpi_enable(void);

#define PWM16_CMPI_DISABLE pwm16_cmpi_disable

/** pwm16_cmpi_disable disables the compare interrupt for the PWM16 slave's channel */
STATIC INLINE void pwm16_cmpi_disable(void);
#endif /* PWM_SLAVE16 */

#if defined (IO_PWM_SLAVE17__START_GET)
#define PWM17_CMPI_ENABLE pwm17_cmpi_enable

/** pwm17_cmpi_enable enables the compare interrupt for the PWM17 slave's channel */
STATIC INLINE void pwm17_cmpi_enable(void);

#define PWM17_CMPI_DISABLE pwm17_cmpi_disable

/** pwm17_cmpi_disable disables the compare interrupt for the PWM17 slave's channel */
STATIC INLINE void pwm17_cmpi_disable(void);
#endif /* PWM_SLAVE17 */

#if defined (IO_PWM_SLAVE18__START_GET)
#define PWM18_CMPI_ENABLE pwm18_cmpi_enable

/** pwm18_cmpi_enable enables the compare interrupt for the PWM18 slave's channel */
STATIC INLINE void pwm18_cmpi_enable(void);

#define PWM18_CMPI_DISABLE pwm18_cmpi_disable

/** pwm18_cmpi_disable disables the compare interrupt for the PWM18 slave's channel */
STATIC INLINE void pwm18_cmpi_disable(void);
#endif /* PWM_SLAVE18 */

#if defined (IO_PWM_SLAVE19__START_GET)
#define PWM19_CMPI_ENABLE pwm19_cmpi_enable

/** pwm19_cmpi_enable enables the compare interrupt for the PWM19 slave's channel */
STATIC INLINE void pwm19_cmpi_enable(void);

#define PWM19_CMPI_DISABLE pwm19_cmpi_disable

/** pwm19_cmpi_disable disables the compare interrupt for the PWM19 slave's channel */
STATIC INLINE void pwm19_cmpi_disable(void);
#endif /* PWM_SLAVE19 */

#if defined (IO_PWM_SLAVE20__START_GET)
#define PWM20_CMPI_ENABLE pwm20_cmpi_enable

/** pwm20_cmpi_enable enables the compare interrupt for the PWM20 slave's channel */
STATIC INLINE void pwm20_cmpi_enable(void);

#define PWM20_CMPI_DISABLE pwm20_cmpi_disable

/** pwm20_cmpi_disable disables the compare interrupt for the PWM20 slave's channel */
STATIC INLINE void pwm20_cmpi_disable(void);
#endif /* PWM_SLAVE20 */

#if defined (IO_PWM_SLAVE21__START_GET)
#define PWM21_CMPI_ENABLE pwm21_cmpi_enable

/** pwm21_cmpi_enable enables the compare interrupt for the PWM21 slave's channel */
STATIC INLINE void pwm21_cmpi_enable(void);

#define PWM21_CMPI_DISABLE pwm21_cmpi_disable

/** pwm21_cmpi_disable disables the compare interrupt for the PWM21 slave's channel */
STATIC INLINE void pwm21_cmpi_disable(void);
#endif /* PWM_SLAVE21 */

#if defined (IO_PWM_SLAVE22__START_GET)
#define PWM22_CMPI_ENABLE pwm22_cmpi_enable

/** pwm22_cmpi_enable enables the compare interrupt for the PWM22 slave's channel */
STATIC INLINE void pwm22_cmpi_enable(void);

#define PWM22_CMPI_DISABLE pwm22_cmpi_disable

/** pwm22_cmpi_disable disables the compare interrupt for the PWM22 slave's channel */
STATIC INLINE void pwm22_cmpi_disable(void);
#endif /* PWM_SLAVE22 */

#if defined (IO_PWM_SLAVE23__START_GET)
#define PWM23_CMPI_ENABLE pwm23_cmpi_enable

/** pwm23_cmpi_enable enables the compare interrupt for the PWM23 slave's channel */
STATIC INLINE void pwm23_cmpi_enable(void);

#define PWM23_CMPI_DISABLE pwm23_cmpi_disable

/** pwm23_cmpi_disable disables the compare interrupt for the PWM23 slave's channel */
STATIC INLINE void pwm23_cmpi_disable(void);
#endif /* PWM_SLAVE23 */

#if defined (IO_PWM_SLAVE24__START_GET)
#define PWM24_CMPI_ENABLE pwm24_cmpi_enable

/** pwm24_cmpi_enable enables the compare interrupt for the PWM24 slave's channel */
STATIC INLINE void pwm24_cmpi_enable(void);

#define PWM24_CMPI_DISABLE pwm24_cmpi_disable

/** pwm24_cmpi_disable disables the compare interrupt for the PWM24 slave's channel */
STATIC INLINE void pwm24_cmpi_disable(void);
#endif /* PWM_SLAVE24 */

#if !defined(UNITTEST) /* for unit test, mocks will be generated */
#include "lib_pwm_inline_impl.h"
#endif

#endif /* LIB_PWM_H */
/// @}

