/**
 * @file
 * @brief Custom IO ports and signals support and definitions
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
 * @ingroup CAMCU_library
 *
 * @details
 */

#ifndef MISC_IO_H
#define MISC_IO_H

#ifndef __ASSEMBLER__

#include <static_assert.h>
#include <stdint.h>

/**
 * @brief Enum to define the available IO's
 */
typedef enum {
    GPIO_IO0 = 0u,
    GPIO_IO1,
    GPIO_IO2,
    GPIO_IO3,
    GPIO_IO4,
    GPIO_IO5,
    GPIO_IO6,
    GPIO_IO7,
} GpioIo_t;

/** LIN_XTX multiplexer selection */
typedef enum {
    xtx_COMPLEX_TIMER0_OUT = 0u,
    xtx_COMPLEX_TIMER1_OUT,
    xtx_PWM0_OUT,
    xtx_PWM1_OUT,
    xtx_PWM2_OUT,
    xtx_PWM3_OUT,
    xtx_PWM4_OUT,
    xtx_LIN_OUT_SOFT,
    xtx_IO_IN0,
    xtx_IO_IN1,
    xtx_IO_IN2,
    xtx_IO_IN3,
    xtx_PPM_OUT,
    xtx_IO_IN4,
    xtx_IO_IN5,
    xtx_IO_IN6,
    /* no value for xtx_IO_IN7: see description of LIN_XTX_OUTPUT_SELECT[3:0] */
} lin_xtx_multiplexer_t;

/**  Complex Timer channel selection.
 *
 *   For every Complex Timer we have 2 inputs Channel A and Channel B separately connectable to each MS_IO port. */
typedef enum CTimerCHinputSelect_e {
    CTIMER_IN_CH_IO0 = 0U,
    CTIMER_IN_CH_IO1 = 1U,
    CTIMER_IN_CH_IO2 = 2U,
    CTIMER_IN_CH_IO3 = 3U,
    CTIMER_IN_CH_LIN_XRX = 4U,
    CTIMER_IN_CH_PWM0 = 5U,    /**< For CHA this signal takes PWM_OUT, for CHB it's PWM */
    CTIMER_IN_CH_PWM1 = 6U,    /**< For CHA this signal takes PWM_OUT, for CHB it's PWM */
    CTIMER_IN_CH_PWM2 = 7U,    /**< For CHA this signal takes PWM_OUT, for CHB it's PWM */
    CTIMER_IN_CH_PWM3 = 8U,    /**< For CHA this signal takes PWM_OUT, for CHB it's PWM */
    CTIMER_IN_CH_PWM4 = 9U,
    CTIMER_IN_CH_IO4 = 10U,
    CTIMER_IN_CH_IO5 = 11U,
    CTIMER_IN_CH_IO6 = 12U,
    CTIMER_IN_CH_IO7 = 13U
} CTimerCHinputSelect_t;

/** Input PIN for PPM PHY selection to configure PORT_PPM_CTRL:PPM_IN_SEL[3:0] */
typedef enum ppm_InputPin_e {
    ppm_IN_LIN_XRX = 0u,
    ppm_IN_IO_IN0,
    ppm_IN_IO_IN1,
    ppm_IN_IO_IN2,
    ppm_IN_IO_IN3,
    ppm_IN_IO_IN4,
    ppm_IN_IO_IN5,
    ppm_IN_IO_IN6,
    ppm_IN_IO_IN7,
    ppm_IN_LIN_RECESSIVE,
    /* 1001-1111: ppm_IN_LIN_RECESSIVE */
} ppm_InputPin;

/** Define PPM input pin selection options
 *
 * Enumurate RC_PPM_INPUT_PIN_SEL and map the PPM input pin selection to ppm_InputPin
 */
typedef enum PpmInputPinOption_e {
    PPM_IN_SEL0 = ppm_IN_LIN_XRX,   /**< First PPM input pin option definition */
    PPM_IN_SEL1 = ppm_IN_IO_IN0,    /**< Second PPM input pin option definition */
    PPM_IN_SEL2 = ppm_IN_IO_IN1,    /**< Third PPM input pin option definition */
} PpmInputPinOption_t;

/** option to listen on all pins for PPM input pin selection. */
#define PPM_IN_SEL_LISTEN_ALL 3u

/** Digital IO output multiplexer options */
typedef enum GpioOutputSelections_e {
    GPIO_PWM_MASTER1 = 0u,          /**< Connect PWM_MASTER1 (PWM_OUT[0]) to IO_OUT[x] */
    GPIO_PWM_SLAVE1 = 1u,           /**< Connect PWM_SLAVE1 (PWM_OUT[1]) to IO_OUT[x] */
    GPIO_PWM_SLAVE2 = 2u,           /**< Connect PWM_SLAVE2 (PWM_OUT[2]) to IO_OUT[x] */
    GPIO_PWM_SLAVE3 = 3u,           /**< Connect PWM_SLAVE3 (PWM_OUT[3]) to IO_OUT[x] */
    GPIO_PWM_MASTER2 = 4u,          /**< Connect PWM_MASTER2 (PWM_OUT[4]) to IO_OUT[x] */
    GPIO_CTIMER_0 = 5u,             /**< Connect COMPLEX_TIMER[0]_OUT to IO_OUT[x] */
    GPIO_CTIMER_1 = 6u,             /**< Connect COMPLEX_TIMER[1]_OUT to IO_OUT[x] */
    GPIO_IO_OUT_SOFT = 7u,          /**< Connect IO_OUT_SOFT[x] to IO_OUT[x] */
    GPIO_LIN_XRX = 8u,              /**< Connect LIN_XRX to IO_OUT[x] */
    GPIO_COLIN_TX = 9u,             /**< Connect COLIN_TX to IO_OUT[x] */
    GPIO_SPI_MOSI_OUT = 10u,        /**< Connect SPI_MOSI_OUT to IO_OUT[x] */
    GPIO_SPI_MISO_OUT = 11u,        /**< Connect SPI_MISO_OUT to IO_OUT[x] */
    GPIO_SPI_SCK = 12u,             /**< Connect SPI_SCK_OUT to IO_OUT[x] */
    GPIO_SPI_SS_OUT = 13u,          /**< Connect SPI_SS_OUT to IO_OUT[x] */
    GPIO_PPM_OUT = 14u,             /**< Connect PPM_OUT to IO_OUT[x] */
    GPIO_ZERO = 15u                 /**< Connect GND to IO_OUT[x] */
} GpioOutputSelections_t;           /**< Digital IO output multiplexer options typedef */

/** DRV_CTRL[X] definitions: driver input source selections */
typedef enum DrvCtrlSelect_e {
    DRV_CTRL_PWM_MASTER1 = 0U,      /**< Select PWM_MASTER1 (PWM_IN[0]) as input */
    DRV_CTRL_PWM_SLAVE1 = 1U,       /**< Select PWM_SLAVE1 (PWM_IN[1]) as input */
    DRV_CTRL_PWM_SLAVE2 = 2U,       /**< Select PWM_SLAVE2 (PWM_IN[2]) as input */
    DRV_CTRL_PWM_SLAVE3 = 3U,       /**< Select PWM_SLAVE3 (PWM_IN[3]) as input */
    DRV_CTRL_PWM_MASTER2 = 4U,      /**< Select PWM_MASTER2 (PWM_IN[4]) as input */
    DRV_CTRL_IO0 = 5U,              /**< Select IO[0] as input */
    DRV_CTRL_IO1 = 6U,              /**< Select IO[1] as input */
    DRV_CTRL_IO2 = 7U,              /**< Select IO[2] as input */
    DRV_CTRL_IO3 = 8U,              /**< Select IO[3] as input */
    DRV_CTRL_LIN_XRX = 9U,          /**< Select LIN_XRX as input */
    DRV_CTRL_CTIMER0_OUT = 10U,     /**< Select Complex timer0 output as input */
    DRV_CTRL_CTIMER1_OUT = 11U,     /**< Select Complex timer1 output as input */
    DRV_CTRL_IO4 = 12U,             /**< Select IO[4] as input */
    DRV_CTRL_LOW = 13U,             /**< Set output to low level */
    DRV_CTRL_TRISTATE = 14U,        /**< Set output to tristate */
    DRV_CTRL_HIGH = 15U             /**< Set output to high level */
} DrvCtrlSelect_t;                  /**< Driver input source multiplexer options typedef */

/** EE_ROM_CONFIG fields description */
typedef struct __attribute__((packed)) {
    uint8_t RC_DISTERM : 1;         /**< Specifies whether LIN termination is disconnected */
    uint8_t RC_PPM_5V : 1;          /**< Specifies whether PPM is using 5V/fast receiver */
    uint8_t RC_FASTLIN_5V : 1;      /**< Specifies whether LIN is using 5V/fast receiver */
    uint8_t RC_IGNORE_RSTAT_FOR_WARM_BOOT : 1; /**< Specified whether to ignore (1) or use (0) the RSTAT bit during
                                                    boot state (cold/warm) detection. */
    uint8_t RC_PPM_INPUT_PIN_SEL : 2; /**< PPM input pin selection */
    uint8_t reserved : 2;           /**< reserved bits */
} RomConfiguration_t;
ASSERT(sizeof(RomConfiguration_t) == 1);

#endif

#ifndef MLX4_FPLL
/** PLL frequency for MLX4
 *
 * Calculated from global PLL frequency and the divider value
 */
#define MLX4_FPLL               ((FPLL) / ((LIN_CLK_DIV)+1))
#endif

#endif /* MISC_IO_H */
