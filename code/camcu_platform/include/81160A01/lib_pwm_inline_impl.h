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
 *
 * @brief PWM HW support library
 * @details
 * @{
 */

#ifndef LIB_PWM_INLINE_IMPL_H
#define LIB_PWM_INLINE_IMPL_H

#include <stdint.h>
#include "itc_helper.h"

#include "syslib.h"
#include "io.h"

#ifdef UNITTEST
#include "lib_pwm.h"
#endif

/*
 * Initialize PWM unit in a Slave mode
 */
#define __PWM_INIT_SLAVE(unit, DIV, DIVN, LT_, HT_, CMP__, MODE_, POL_, IDLE_, SLAVE_)  \
    do  {                                                    \
        IO_SET(unit, PSCLM, ((DIV)-1) & 0x0Fu,               \
               PSCLN, (DIVN) & 0x0Fu,          \
               IDLE, IDLE_,                 \
               POL, POL_,                   \
               MODE, MODE_,                 \
               SLAVE, SLAVE_,               \
               START, 0u,                   \
               STOP,  0u);                  \
        IO_SET(unit,            CMP, CMP__);                 \
        IO_SET(unit,            LT, LT_);                    \
        IO_SET(unit,            HT, HT_);                    \
    } while(0)

#if defined (IO_PWM_MASTER1__START_GET)
#define PWM1_MASTER_INIT pwm1_master_init
#define PWM1_MASTER_ENABLE_SLAVE_MODE pwm1_master_enable_slave_mode

STATIC INLINE void pwm1_master_init(uint8_t DIV_,
                                    uint8_t DIVN_,
                                    uint16_t PERIOD_,
                                    uint16_t LT_,
                                    uint16_t HT_,
                                    uint16_t CMP_,
                                    PwmMode_t MODE_,
                                    PwmPolarity_t POL_,
                                    uint8_t InvertPolarityIDLE_)
{
    IO_SET(PWM_MASTER1, PSCLM, ((DIV_)-1) & 0x0Fu,
           PSCLN, (DIVN_) & 0x0Fu,
           IDLE, InvertPolarityIDLE_,
           POL, POL_,
           MODE, MODE_,
           SLAVE, 0u,
           START, 0u,
           STOP,  0u);
    IO_SET(PWM_MASTER1, PER, PERIOD_);
    IO_SET(PWM_MASTER1, CMP, CMP_);
    IO_SET(PWM_MASTER1, LT, LT_);
    IO_SET(PWM_MASTER1, HT, HT_);
}

STATIC INLINE void pwm1_master_enable_slave_mode(void)
{
    IO_SET(PWM_MASTER1,
           SLAVE, 1u);
}
#endif /* PWM_MASTER1 */

#if defined (IO_PWM_MASTER2__START_GET)
#define PWM2_MASTER_INIT pwm2_master_init
#define PWM2_MASTER_ENABLE_SLAVE_MODE pwm2_master_enable_slave_mode

STATIC INLINE void pwm2_master_init(uint8_t DIV_,
                                    uint8_t DIVN_,
                                    uint16_t PERIOD_,
                                    uint16_t LT_,
                                    uint16_t HT_,
                                    uint16_t CMP_,
                                    PwmMode_t MODE_,
                                    PwmPolarity_t POL_,
                                    uint8_t InvertPolarityIDLE_)
{
    IO_SET(PWM_MASTER2, PSCLM, ((DIV_)-1) & 0x0Fu,
           PSCLN, (DIVN_) & 0x0Fu,
           IDLE, InvertPolarityIDLE_,
           POL, POL_,
           MODE, MODE_,
           SLAVE, 0u,
           START, 0u,
           STOP,  0u);
    IO_SET(PWM_MASTER2, PER, PERIOD_);
    IO_SET(PWM_MASTER2, CMP, CMP_);
    IO_SET(PWM_MASTER2, LT, LT_);
    IO_SET(PWM_MASTER2, HT, HT_);
}

STATIC INLINE void pwm2_master_enable_slave_mode(void)
{
    IO_SET(PWM_MASTER2,
           SLAVE, 1u);
}
#endif /* PWM_MASTER2 */

#if defined (IO_PWM_SLAVE1__START_GET)
#define PWM1_INIT pwm1_init

STATIC INLINE void pwm1_init(uint8_t DIV_, uint8_t DIVN_, uint16_t LT_, uint16_t HT_,
                             uint16_t CMP_, PwmMode_t MODE_, PwmPolarity_t POL_, uint8_t InvertPolarityIDLE_)
{
    IO_SET(PWM_SLAVE1, PSCLM, ((DIV_)-1) & 0x0Fu,
           PSCLN, (DIVN_) & 0x0Fu,
           IDLE, InvertPolarityIDLE_,
           POL, POL_,
           MODE, MODE_,
           SLAVE, 1u,
           START, 0u,
           STOP,  0u);
    IO_SET(PWM_SLAVE1, CMP, CMP_);
    IO_SET(PWM_SLAVE1, LT, LT_);
    IO_SET(PWM_SLAVE1, HT, HT_);
}
#endif /* PWM_SLAVE1 */

#if defined (IO_PWM_SLAVE2__START_GET)
#define PWM2_INIT pwm2_init

STATIC INLINE void pwm2_init(uint8_t DIV_, uint8_t DIVN_, uint16_t LT_, uint16_t HT_,
                             uint16_t CMP_, PwmMode_t MODE_, PwmPolarity_t POL_, uint8_t InvertPolarityIDLE_)
{
    IO_SET(PWM_SLAVE2, PSCLM, ((DIV_)-1) & 0x0Fu,
           PSCLN, (DIVN_) & 0x0Fu,
           IDLE, InvertPolarityIDLE_,
           POL, POL_,
           MODE, MODE_,
           SLAVE, 1u,
           START, 0u,
           STOP,  0u);
    IO_SET(PWM_SLAVE2, CMP, CMP_);
    IO_SET(PWM_SLAVE2, LT, LT_);
    IO_SET(PWM_SLAVE2, HT, HT_);
}
#endif /* PWM_SLAVE2 */

#if defined (IO_PWM_SLAVE3__START_GET)
#define PWM3_INIT pwm3_init

STATIC INLINE void pwm3_init(uint8_t DIV_, uint8_t DIVN_, uint16_t LT_, uint16_t HT_,
                             uint16_t CMP_, PwmMode_t MODE_, PwmPolarity_t POL_, uint8_t InvertPolarityIDLE_)
{
    IO_SET(PWM_SLAVE3, PSCLM, ((DIV_)-1) & 0x0Fu,
           PSCLN, (DIVN_) & 0x0Fu,
           IDLE, InvertPolarityIDLE_,
           POL, POL_,
           MODE, MODE_,
           SLAVE, 1u,
           START, 0u,
           STOP,  0u);
    IO_SET(PWM_SLAVE3, CMP, CMP_);
    IO_SET(PWM_SLAVE3, LT, LT_);
    IO_SET(PWM_SLAVE3, HT, HT_);
}
#endif /* PWM_SLAVE3 */

#if defined (IO_PWM_SLAVE4__START_GET)
#define PWM4_INIT pwm4_init

STATIC INLINE void pwm4_init(uint8_t DIV_, uint8_t DIVN_, uint16_t LT_, uint16_t HT_,
                             uint16_t CMP_, PwmMode_t MODE_, PwmPolarity_t POL_, uint8_t InvertPolarityIDLE_)
{
    IO_SET(PWM_SLAVE4, PSCLM, ((DIV_)-1) & 0x0Fu,
           PSCLN, (DIVN_) & 0x0Fu,
           IDLE, InvertPolarityIDLE_,
           POL, POL_,
           MODE, MODE_,
           SLAVE, 1u,
           START, 0u,
           STOP,  0u);
    IO_SET(PWM_SLAVE4, CMP, CMP_);
    IO_SET(PWM_SLAVE4, LT, LT_);
    IO_SET(PWM_SLAVE4, HT, HT_);
}
#endif /* PWM_SLAVE4 */

#if defined (IO_PWM_SLAVE5__START_GET)
#define PWM5_INIT pwm5_init

STATIC INLINE void pwm5_init(uint8_t DIV_, uint8_t DIVN_, uint16_t LT_, uint16_t HT_,
                             uint16_t CMP_, PwmMode_t MODE_, PwmPolarity_t POL_, uint8_t InvertPolarityIDLE_)
{
    IO_SET(PWM_SLAVE5, PSCLM, ((DIV_)-1) & 0x0Fu,
           PSCLN, (DIVN_) & 0x0Fu,
           IDLE, InvertPolarityIDLE_,
           POL, POL_,
           MODE, MODE_,
           SLAVE, 1u,
           START, 0u,
           STOP,  0u);
    IO_SET(PWM_SLAVE5, CMP, CMP_);
    IO_SET(PWM_SLAVE5, LT, LT_);
    IO_SET(PWM_SLAVE5, HT, HT_);
}
#endif /* PWM_SLAVE5 */

#if defined (IO_PWM_SLAVE6__START_GET)
#define PWM6_INIT pwm6_init

STATIC INLINE void pwm6_init(uint8_t DIV_, uint8_t DIVN_, uint16_t LT_, uint16_t HT_,
                             uint16_t CMP_, PwmMode_t MODE_, PwmPolarity_t POL_, uint8_t InvertPolarityIDLE_)
{
    IO_SET(PWM_SLAVE6, PSCLM, ((DIV_)-1) & 0x0Fu,
           PSCLN, (DIVN_) & 0x0Fu,
           IDLE, InvertPolarityIDLE_,
           POL, POL_,
           MODE, MODE_,
           SLAVE, 1u,
           START, 0u,
           STOP,  0u);
    IO_SET(PWM_SLAVE6, CMP, CMP_);
    IO_SET(PWM_SLAVE6, LT, LT_);
    IO_SET(PWM_SLAVE6, HT, HT_);
}
#endif /* PWM_SLAVE6 */

#if defined (IO_PWM_SLAVE7__START_GET)
#define PWM7_INIT pwm7_init

STATIC INLINE void pwm7_init(uint8_t DIV_, uint8_t DIVN_, uint16_t LT_, uint16_t HT_,
                             uint16_t CMP_, PwmMode_t MODE_, PwmPolarity_t POL_, uint8_t InvertPolarityIDLE_)
{
    IO_SET(PWM_SLAVE7, PSCLM, ((DIV_)-1) & 0x0Fu,
           PSCLN, (DIVN_) & 0x0Fu,
           IDLE, InvertPolarityIDLE_,
           POL, POL_,
           MODE, MODE_,
           SLAVE, 1u,
           START, 0u,
           STOP,  0u);
    IO_SET(PWM_SLAVE7, CMP, CMP_);
    IO_SET(PWM_SLAVE7, LT, LT_);
    IO_SET(PWM_SLAVE7, HT, HT_);
}
#endif /* PWM_SLAVE7 */

#if defined (IO_PWM_SLAVE8__START_GET)
#define PWM8_INIT pwm8_init

STATIC INLINE void pwm8_init(uint8_t DIV_, uint8_t DIVN_, uint16_t LT_, uint16_t HT_,
                             uint16_t CMP_, PwmMode_t MODE_, PwmPolarity_t POL_, uint8_t InvertPolarityIDLE_)
{
    IO_SET(PWM_SLAVE8, PSCLM, ((DIV_)-1) & 0x0Fu,
           PSCLN, (DIVN_) & 0x0Fu,
           IDLE, InvertPolarityIDLE_,
           POL, POL_,
           MODE, MODE_,
           SLAVE, 1u,
           START, 0u,
           STOP,  0u);
    IO_SET(PWM_SLAVE8, CMP, CMP_);
    IO_SET(PWM_SLAVE8, LT, LT_);
    IO_SET(PWM_SLAVE8, HT, HT_);
}
#endif /* PWM_SLAVE8 */

#if defined (IO_PWM_SLAVE9__START_GET)
#define PWM9_INIT pwm9_init

STATIC INLINE void pwm9_init(uint8_t DIV_, uint8_t DIVN_, uint16_t LT_, uint16_t HT_,
                             uint16_t CMP_, PwmMode_t MODE_, PwmPolarity_t POL_, uint8_t InvertPolarityIDLE_)
{
    IO_SET(PWM_SLAVE9, PSCLM, ((DIV_)-1) & 0x0Fu,
           PSCLN, (DIVN_) & 0x0Fu,
           IDLE, InvertPolarityIDLE_,
           POL, POL_,
           MODE, MODE_,
           SLAVE, 1u,
           START, 0u,
           STOP,  0u);
    IO_SET(PWM_SLAVE9, CMP, CMP_);
    IO_SET(PWM_SLAVE9, LT, LT_);
    IO_SET(PWM_SLAVE9, HT, HT_);
}
#endif /* PWM_SLAVE9 */

#if defined (IO_PWM_SLAVE10__START_GET)
#define PWM10_INIT pwm10_init

STATIC INLINE void pwm10_init(uint8_t DIV_, uint8_t DIVN_, uint16_t LT_, uint16_t HT_,
                              uint16_t CMP_, PwmMode_t MODE_, PwmPolarity_t POL_, uint8_t InvertPolarityIDLE_)
{
    IO_SET(PWM_SLAVE10, PSCLM, ((DIV_)-1) & 0x0Fu,
           PSCLN, (DIVN_) & 0x0Fu,
           IDLE, InvertPolarityIDLE_,
           POL, POL_,
           MODE, MODE_,
           SLAVE, 1u,
           START, 0u,
           STOP,  0u);
    IO_SET(PWM_SLAVE10, CMP, CMP_);
    IO_SET(PWM_SLAVE10, LT, LT_);
    IO_SET(PWM_SLAVE10, HT, HT_);
}
#endif /* PWM_SLAVE10 */

#if defined (IO_PWM_SLAVE11__START_GET)
#define PWM11_INIT pwm11_init

STATIC INLINE void pwm11_init(uint8_t DIV_, uint8_t DIVN_, uint16_t LT_, uint16_t HT_,
                              uint16_t CMP_, PwmMode_t MODE_, PwmPolarity_t POL_, uint8_t InvertPolarityIDLE_)
{
    IO_SET(PWM_SLAVE11, PSCLM, ((DIV_)-1) & 0x0Fu,
           PSCLN, (DIVN_) & 0x0Fu,
           IDLE, InvertPolarityIDLE_,
           POL, POL_,
           MODE, MODE_,
           SLAVE, 1u,
           START, 0u,
           STOP,  0u);
    IO_SET(PWM_SLAVE11, CMP, CMP_);
    IO_SET(PWM_SLAVE11, LT, LT_);
    IO_SET(PWM_SLAVE11, HT, HT_);
}
#endif /* PWM_SLAVE11 */

#if defined (IO_PWM_SLAVE12__START_GET)
#define PWM12_INIT pwm12_init

STATIC INLINE void pwm12_init(uint8_t DIV_, uint8_t DIVN_, uint16_t LT_, uint16_t HT_,
                              uint16_t CMP_, PwmMode_t MODE_, PwmPolarity_t POL_, uint8_t InvertPolarityIDLE_)
{
    IO_SET(PWM_SLAVE12, PSCLM, ((DIV_)-1) & 0x0Fu,
           PSCLN, (DIVN_) & 0x0Fu,
           IDLE, InvertPolarityIDLE_,
           POL, POL_,
           MODE, MODE_,
           SLAVE, 1u,
           START, 0u,
           STOP,  0u);
    IO_SET(PWM_SLAVE12, CMP, CMP_);
    IO_SET(PWM_SLAVE12, LT, LT_);
    IO_SET(PWM_SLAVE12, HT, HT_);
}
#endif /* PWM_SLAVE12 */

#if defined (IO_PWM_SLAVE13__START_GET)
#define PWM13_INIT pwm13_init

STATIC INLINE void pwm13_init(uint8_t DIV_, uint8_t DIVN_, uint16_t LT_, uint16_t HT_,
                              uint16_t CMP_, PwmMode_t MODE_, PwmPolarity_t POL_, uint8_t InvertPolarityIDLE_)
{
    IO_SET(PWM_SLAVE13, PSCLM, ((DIV_)-1) & 0x0Fu,
           PSCLN, (DIVN_) & 0x0Fu,
           IDLE, InvertPolarityIDLE_,
           POL, POL_,
           MODE, MODE_,
           SLAVE, 1u,
           START, 0u,
           STOP,  0u);
    IO_SET(PWM_SLAVE13, CMP, CMP_);
    IO_SET(PWM_SLAVE13, LT, LT_);
    IO_SET(PWM_SLAVE13, HT, HT_);
}
#endif /* PWM_SLAVE13 */

#if defined (IO_PWM_SLAVE14__START_GET)
#define PWM14_INIT pwm14_init

STATIC INLINE void pwm14_init(uint8_t DIV_, uint8_t DIVN_, uint16_t LT_, uint16_t HT_,
                              uint16_t CMP_, PwmMode_t MODE_, PwmPolarity_t POL_, uint8_t InvertPolarityIDLE_)
{
    IO_SET(PWM_SLAVE14, PSCLM, ((DIV_)-1) & 0x0Fu,
           PSCLN, (DIVN_) & 0x0Fu,
           IDLE, InvertPolarityIDLE_,
           POL, POL_,
           MODE, MODE_,
           SLAVE, 1u,
           START, 0u,
           STOP,  0u);
    IO_SET(PWM_SLAVE14, CMP, CMP_);
    IO_SET(PWM_SLAVE14, LT, LT_);
    IO_SET(PWM_SLAVE14, HT, HT_);
}
#endif /* PWM_SLAVE14 */

#if defined (IO_PWM_SLAVE15__START_GET)
#define PWM15_INIT pwm15_init

STATIC INLINE void pwm15_init(uint8_t DIV_, uint8_t DIVN_, uint16_t LT_, uint16_t HT_,
                              uint16_t CMP_, PwmMode_t MODE_, PwmPolarity_t POL_, uint8_t InvertPolarityIDLE_)
{
    IO_SET(PWM_SLAVE15, PSCLM, ((DIV_)-1) & 0x0Fu,
           PSCLN, (DIVN_) & 0x0Fu,
           IDLE, InvertPolarityIDLE_,
           POL, POL_,
           MODE, MODE_,
           SLAVE, 1u,
           START, 0u,
           STOP,  0u);
    IO_SET(PWM_SLAVE15, CMP, CMP_);
    IO_SET(PWM_SLAVE15, LT, LT_);
    IO_SET(PWM_SLAVE15, HT, HT_);
}
#endif /* PWM_SLAVE15 */

#if defined (IO_PWM_SLAVE16__START_GET)
#define PWM16_INIT pwm16_init

STATIC INLINE void pwm16_init(uint8_t DIV_, uint8_t DIVN_, uint16_t LT_, uint16_t HT_,
                              uint16_t CMP_, PwmMode_t MODE_, PwmPolarity_t POL_, uint8_t InvertPolarityIDLE_)
{
    IO_SET(PWM_SLAVE16, PSCLM, ((DIV_)-1) & 0x0Fu,
           PSCLN, (DIVN_) & 0x0Fu,
           IDLE, InvertPolarityIDLE_,
           POL, POL_,
           MODE, MODE_,
           SLAVE, 1u,
           START, 0u,
           STOP,  0u);
    IO_SET(PWM_SLAVE16, CMP, CMP_);
    IO_SET(PWM_SLAVE16, LT, LT_);
    IO_SET(PWM_SLAVE16, HT, HT_);
}
#endif /* PWM_SLAVE16 */

#if defined (IO_PWM_SLAVE17__START_GET)
#define PWM17_INIT pwm17_init

STATIC INLINE void pwm17_init(uint8_t DIV_, uint8_t DIVN_, uint16_t LT_, uint16_t HT_,
                              uint16_t CMP_, PwmMode_t MODE_, PwmPolarity_t POL_, uint8_t InvertPolarityIDLE_)
{
    IO_SET(PWM_SLAVE17, PSCLM, ((DIV_)-1) & 0x0Fu,
           PSCLN, (DIVN_) & 0x0Fu,
           IDLE, InvertPolarityIDLE_,
           POL, POL_,
           MODE, MODE_,
           SLAVE, 1u,
           START, 0u,
           STOP,  0u);
    IO_SET(PWM_SLAVE17, CMP, CMP_);
    IO_SET(PWM_SLAVE17, LT, LT_);
    IO_SET(PWM_SLAVE17, HT, HT_);
}
#endif /* PWM_SLAVE17 */

#if defined (IO_PWM_SLAVE18__START_GET)
#define PWM18_INIT pwm18_init

STATIC INLINE void pwm18_init(uint8_t DIV_, uint8_t DIVN_, uint16_t LT_, uint16_t HT_,
                              uint16_t CMP_, PwmMode_t MODE_, PwmPolarity_t POL_, uint8_t InvertPolarityIDLE_)
{
    IO_SET(PWM_SLAVE18, PSCLM, ((DIV_)-1) & 0x0Fu,
           PSCLN, (DIVN_) & 0x0Fu,
           IDLE, InvertPolarityIDLE_,
           POL, POL_,
           MODE, MODE_,
           SLAVE, 1u,
           START, 0u,
           STOP,  0u);
    IO_SET(PWM_SLAVE18, CMP, CMP_);
    IO_SET(PWM_SLAVE18, LT, LT_);
    IO_SET(PWM_SLAVE18, HT, HT_);
}
#endif /* PWM_SLAVE18 */

#if defined (IO_PWM_SLAVE19__START_GET)
#define PWM19_INIT pwm19_init

STATIC INLINE void pwm19_init(uint8_t DIV_, uint8_t DIVN_, uint16_t LT_, uint16_t HT_,
                              uint16_t CMP_, PwmMode_t MODE_, PwmPolarity_t POL_, uint8_t InvertPolarityIDLE_)
{
    IO_SET(PWM_SLAVE19, PSCLM, ((DIV_)-1) & 0x0Fu,
           PSCLN, (DIVN_) & 0x0Fu,
           IDLE, InvertPolarityIDLE_,
           POL, POL_,
           MODE, MODE_,
           SLAVE, 1u,
           START, 0u,
           STOP,  0u);
    IO_SET(PWM_SLAVE19, CMP, CMP_);
    IO_SET(PWM_SLAVE19, LT, LT_);
    IO_SET(PWM_SLAVE19, HT, HT_);
}
#endif /* PWM_SLAVE19 */

#if defined (IO_PWM_SLAVE20__START_GET)
#define PWM20_INIT pwm20_init

STATIC INLINE void pwm20_init(uint8_t DIV_, uint8_t DIVN_, uint16_t LT_, uint16_t HT_,
                              uint16_t CMP_, PwmMode_t MODE_, PwmPolarity_t POL_, uint8_t InvertPolarityIDLE_)
{
    IO_SET(PWM_SLAVE20, PSCLM, ((DIV_)-1) & 0x0Fu,
           PSCLN, (DIVN_) & 0x0Fu,
           IDLE, InvertPolarityIDLE_,
           POL, POL_,
           MODE, MODE_,
           SLAVE, 1u,
           START, 0u,
           STOP,  0u);
    IO_SET(PWM_SLAVE20, CMP, CMP_);
    IO_SET(PWM_SLAVE20, LT, LT_);
    IO_SET(PWM_SLAVE20, HT, HT_);
}
#endif /* PWM_SLAVE20 */

#if defined (IO_PWM_SLAVE21__START_GET)
#define PWM21_INIT pwm21_init

STATIC INLINE void pwm21_init(uint8_t DIV_, uint8_t DIVN_, uint16_t LT_, uint16_t HT_,
                              uint16_t CMP_, PwmMode_t MODE_, PwmPolarity_t POL_, uint8_t InvertPolarityIDLE_)
{
    IO_SET(PWM_SLAVE21, PSCLM, ((DIV_)-1) & 0x0Fu,
           PSCLN, (DIVN_) & 0x0Fu,
           IDLE, InvertPolarityIDLE_,
           POL, POL_,
           MODE, MODE_,
           SLAVE, 1u,
           START, 0u,
           STOP,  0u);
    IO_SET(PWM_SLAVE21, CMP, CMP_);
    IO_SET(PWM_SLAVE21, LT, LT_);
    IO_SET(PWM_SLAVE21, HT, HT_);
}
#endif /* PWM_SLAVE21 */

#if defined (IO_PWM_SLAVE22__START_GET)
#define PWM22_INIT pwm22_init

STATIC INLINE void pwm22_init(uint8_t DIV_, uint8_t DIVN_, uint16_t LT_, uint16_t HT_,
                              uint16_t CMP_, PwmMode_t MODE_, PwmPolarity_t POL_, uint8_t InvertPolarityIDLE_)
{
    IO_SET(PWM_SLAVE22, PSCLM, ((DIV_)-1) & 0x0Fu,
           PSCLN, (DIVN_) & 0x0Fu,
           IDLE, InvertPolarityIDLE_,
           POL, POL_,
           MODE, MODE_,
           SLAVE, 1u,
           START, 0u,
           STOP,  0u);
    IO_SET(PWM_SLAVE22, CMP, CMP_);
    IO_SET(PWM_SLAVE22, LT, LT_);
    IO_SET(PWM_SLAVE22, HT, HT_);
}
#endif /* PWM_SLAVE22 */

#if defined (IO_PWM_SLAVE23__START_GET)
#define PWM23_INIT pwm23_init

STATIC INLINE void pwm23_init(uint8_t DIV_, uint8_t DIVN_, uint16_t LT_, uint16_t HT_,
                              uint16_t CMP_, PwmMode_t MODE_, PwmPolarity_t POL_, uint8_t InvertPolarityIDLE_)
{
    IO_SET(PWM_SLAVE23, PSCLM, ((DIV_)-1) & 0x0Fu,
           PSCLN, (DIVN_) & 0x0Fu,
           IDLE, InvertPolarityIDLE_,
           POL, POL_,
           MODE, MODE_,
           SLAVE, 1u,
           START, 0u,
           STOP,  0u);
    IO_SET(PWM_SLAVE23, CMP, CMP_);
    IO_SET(PWM_SLAVE23, LT, LT_);
    IO_SET(PWM_SLAVE23, HT, HT_);
}
#endif /* PWM_SLAVE23 */

#if defined (IO_PWM_SLAVE24__START_GET)
#define PWM24_INIT pwm24_init

STATIC INLINE void pwm24_init(uint8_t DIV_, uint8_t DIVN_, uint16_t LT_, uint16_t HT_,
                              uint16_t CMP_, PwmMode_t MODE_, PwmPolarity_t POL_, uint8_t InvertPolarityIDLE_)
{
    IO_SET(PWM_SLAVE24, PSCLM, ((DIV_)-1) & 0x0Fu,
           PSCLN, (DIVN_) & 0x0Fu,
           IDLE, InvertPolarityIDLE_,
           POL, POL_,
           MODE, MODE_,
           SLAVE, 1u,
           START, 0u,
           STOP,  0u);
    IO_SET(PWM_SLAVE24, CMP, CMP_);
    IO_SET(PWM_SLAVE24, LT, LT_);
    IO_SET(PWM_SLAVE24, HT, HT_);
}
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
                                          uint8_t InvertPolarityIDLE_)
{
    IO_SET(PWM_MASTER2, PSCLM, ((DIV_)-1) & 0x0Fu,
           PSCLN, (DIVN_) & 0x0Fu,
           IDLE, InvertPolarityIDLE_,
           POL, POL_,
           MODE, MODE_,
           SLAVE, 1u,
           START, 0u,
           STOP,  0u);
    IO_SET(PWM_MASTER2, CMP, CMP_);
    IO_SET(PWM_MASTER2, LT, LT_);
    IO_SET(PWM_MASTER2, HT, HT_);
}
#endif /* PWM_MASTER2 */




#if defined (IO_PWM_MASTER1__START_GET)
#define PWM1_MASTER_GET_INPUT pwm1_master_get_input

STATIC INLINE uint8_t pwm1_master_get_input(void)
{
    return IO_GET(PWM_MASTER1, PWM_IN);
}
#endif /*PWM_MASTER1*/

#if defined (IO_PWM_MASTER2__START_GET)
#define PWM2_MASTER_GET_INPUT pwm2_master_get_input

STATIC INLINE uint8_t pwm2_master_get_input(void)
{
    return IO_GET(PWM_MASTER2, PWM_IN);
}
#endif /*PWM_MASTER2*/

#if defined (IO_PWM_SLAVE1__START_GET)
#define PWM1_GET_INPUT pwm1_get_input

STATIC INLINE uint8_t pwm1_get_input(void)
{
    return IO_GET(PWM_SLAVE1, PWM_IN);
}
#endif /* PWM_SLAVE1 */

#if defined (IO_PWM_SLAVE2__START_GET)
#define PWM2_GET_INPUT pwm2_get_input

STATIC INLINE uint8_t pwm2_get_input(void)
{
    return IO_GET(PWM_SLAVE2, PWM_IN);
}
#endif /* PWM_SLAVE2 */

#if defined (IO_PWM_SLAVE3__START_GET)
#define PWM3_GET_INPUT pwm3_get_input

STATIC INLINE uint8_t pwm3_get_input(void)
{
    return IO_GET(PWM_SLAVE3, PWM_IN);
}
#endif /* PWM_SLAVE3 */

#if defined (IO_PWM_SLAVE4__START_GET)
#define PWM4_GET_INPUT pwm4_get_input

STATIC INLINE uint8_t pwm4_get_input(void)
{
    return IO_GET(PWM_SLAVE4, PWM_IN);
}
#endif /* PWM_SLAVE4 */

#if defined (IO_PWM_SLAVE5__START_GET)
#define PWM5_GET_INPUT pwm5_get_input

STATIC INLINE uint8_t pwm5_get_input(void)
{
    return IO_GET(PWM_SLAVE5, PWM_IN);
}
#endif /* PWM_SLAVE5 */

#if defined (IO_PWM_SLAVE6__START_GET)
#define PWM6_GET_INPUT pwm6_get_input

STATIC INLINE uint8_t pwm6_get_input(void)
{
    return IO_GET(PWM_SLAVE6, PWM_IN);
}
#endif /* PWM_SLAVE6 */

#if defined (IO_PWM_SLAVE7__START_GET)
#define PWM7_GET_INPUT pwm7_get_input

STATIC INLINE uint8_t pwm7_get_input(void)
{
    return IO_GET(PWM_SLAVE7, PWM_IN);
}
#endif /* PWM_SLAVE7 */

#if defined (IO_PWM_SLAVE8__START_GET)
#define PWM8_GET_INPUT pwm8_get_input

STATIC INLINE uint8_t pwm8_get_input(void)
{
    return IO_GET(PWM_SLAVE8, PWM_IN);
}
#endif /* PWM_SLAVE8 */

#if defined (IO_PWM_SLAVE9__START_GET)
#define PWM9_GET_INPUT pwm9_get_input

STATIC INLINE uint8_t pwm9_get_input(void)
{
    return IO_GET(PWM_SLAVE9, PWM_IN);
}
#endif /* PWM_SLAVE9 */

#if defined (IO_PWM_SLAVE10__START_GET)
#define PWM10_GET_INPUT pwm10_get_input

STATIC INLINE uint8_t pwm10_get_input(void)
{
    return IO_GET(PWM_SLAVE10, PWM_IN);
}
#endif /* PWM_SLAVE10 */

#if defined (IO_PWM_SLAVE11__START_GET)
#define PWM11_GET_INPUT pwm11_get_input

STATIC INLINE uint8_t pwm11_get_input(void)
{
    return IO_GET(PWM_SLAVE11, PWM_IN);
}
#endif /* PWM_SLAVE11 */

#if defined (IO_PWM_SLAVE12__START_GET)
#define PWM12_GET_INPUT pwm12_get_input

STATIC INLINE uint8_t pwm12_get_input(void)
{
    return IO_GET(PWM_SLAVE12, PWM_IN);
}
#endif /* PWM_SLAVE12 */

#if defined (IO_PWM_SLAVE13__START_GET)
#define PWM13_GET_INPUT pwm13_get_input

STATIC INLINE uint8_t pwm13_get_input(void)
{
    return IO_GET(PWM_SLAVE13, PWM_IN);
}
#endif /* PWM_SLAVE13 */

#if defined (IO_PWM_SLAVE14__START_GET)
#define PWM14_GET_INPUT pwm14_get_input

STATIC INLINE uint8_t pwm14_get_input(void)
{
    return IO_GET(PWM_SLAVE14, PWM_IN);
}
#endif /* PWM_SLAVE14 */

#if defined (IO_PWM_SLAVE15__START_GET)
#define PWM15_GET_INPUT pwm15_get_input

STATIC INLINE uint8_t pwm15_get_input(void)
{
    return IO_GET(PWM_SLAVE15, PWM_IN);
}
#endif /* PWM_SLAVE15 */

#if defined (IO_PWM_SLAVE16__START_GET)
#define PWM16_GET_INPUT pwm16_get_input

STATIC INLINE uint8_t pwm16_get_input(void)
{
    return IO_GET(PWM_SLAVE16, PWM_IN);
}
#endif /* PWM_SLAVE16 */

#if defined (IO_PWM_SLAVE17__START_GET)
#define PWM17_GET_INPUT pwm17_get_input

STATIC INLINE uint8_t pwm17_get_input(void)
{
    return IO_GET(PWM_SLAVE17, PWM_IN);
}
#endif /* PWM_SLAVE17 */

#if defined (IO_PWM_SLAVE18__START_GET)
#define PWM18_GET_INPUT pwm18_get_input

STATIC INLINE uint8_t pwm18_get_input(void)
{
    return IO_GET(PWM_SLAVE18, PWM_IN);
}
#endif /* PWM_SLAVE18 */

#if defined (IO_PWM_SLAVE19__START_GET)
#define PWM19_GET_INPUT pwm19_get_input

STATIC INLINE uint8_t pwm19_get_input(void)
{
    return IO_GET(PWM_SLAVE19, PWM_IN);
}
#endif /* PWM_SLAVE19 */

#if defined (IO_PWM_SLAVE20__START_GET)
#define PWM20_GET_INPUT pwm20_get_input

STATIC INLINE uint8_t pwm20_get_input(void)
{
    return IO_GET(PWM_SLAVE20, PWM_IN);
}
#endif /* PWM_SLAVE20 */

#if defined (IO_PWM_SLAVE21__START_GET)
#define PWM21_GET_INPUT pwm21_get_input

STATIC INLINE uint8_t pwm21_get_input(void)
{
    return IO_GET(PWM_SLAVE21, PWM_IN);
}
#endif /* PWM_SLAVE21 */

#if defined (IO_PWM_SLAVE22__START_GET)
#define PWM22_GET_INPUT pwm22_get_input

STATIC INLINE uint8_t pwm22_get_input(void)
{
    return IO_GET(PWM_SLAVE22, PWM_IN);
}
#endif /* PWM_SLAVE22 */

#if defined (IO_PWM_SLAVE23__START_GET)
#define PWM23_GET_INPUT pwm23_get_input

STATIC INLINE uint8_t pwm23_get_input(void)
{
    return IO_GET(PWM_SLAVE23, PWM_IN);
}
#endif /* PWM_SLAVE23 */

#if defined (IO_PWM_SLAVE24__START_GET)
#define PWM24_GET_INPUT pwm24_get_input

STATIC INLINE uint8_t pwm24_get_input(void)
{
    return IO_GET(PWM_SLAVE24, PWM_IN);
}
#endif /* PWM_SLAVE24 */




#if defined (IO_PWM_MASTER1__START_GET)
#define PWM1_MASTER_ENABLE pwm1_master_enable

STATIC INLINE void pwm1_master_enable(void)
{
    IO_HOST(PWM_MASTER1, START) = 1u << IO_OFFSET(PWM_MASTER1, START);
}
#endif /* PWM_MASTER1 */

#if defined (IO_PWM_MASTER2__START_GET)
#define PWM2_MASTER_ENABLE pwm2_master_enable

STATIC INLINE void pwm2_master_enable(void)
{
    IO_HOST(PWM_MASTER2, START) = 1u << IO_OFFSET(PWM_MASTER2, START);
}
#endif /* PWM_MASTER2 */




#if defined (IO_PWM_SLAVE1__START_GET)
#define PWM1_ENABLE pwm1_enable

STATIC INLINE void pwm1_enable(void)
{
    IO_HOST(PWM_SLAVE1, START) = 1u << IO_OFFSET(PWM_SLAVE1, START);
}
#endif /* PWM_SLAVE1 */

#if defined (IO_PWM_SLAVE2__START_GET)
#define PWM2_ENABLE pwm2_enable

STATIC INLINE void pwm2_enable(void)
{
    IO_HOST(PWM_SLAVE2, START) = 1u << IO_OFFSET(PWM_SLAVE2, START);
}
#endif /* PWM_SLAVE2 */

#if defined (IO_PWM_SLAVE3__START_GET)
#define PWM3_ENABLE pwm3_enable

STATIC INLINE void pwm3_enable(void)
{
    IO_HOST(PWM_SLAVE3, START) = 1u << IO_OFFSET(PWM_SLAVE3, START);
}
#endif /* PWM_SLAVE3 */

#if defined (IO_PWM_SLAVE4__START_GET)
#define PWM4_ENABLE pwm4_enable

STATIC INLINE void pwm4_enable(void)
{
    IO_HOST(PWM_SLAVE4, START) = 1u << IO_OFFSET(PWM_SLAVE4, START);
}
#endif /* PWM_SLAVE4 */

#if defined (IO_PWM_SLAVE5__START_GET)
#define PWM5_ENABLE pwm5_enable

STATIC INLINE void pwm5_enable(void)
{
    IO_HOST(PWM_SLAVE5, START) = 1u << IO_OFFSET(PWM_SLAVE5, START);
}
#endif /* PWM_SLAVE5 */

#if defined (IO_PWM_SLAVE6__START_GET)
#define PWM6_ENABLE pwm6_enable

STATIC INLINE void pwm6_enable(void)
{
    IO_HOST(PWM_SLAVE6, START) = 1u << IO_OFFSET(PWM_SLAVE6, START);
}
#endif /* PWM_SLAVE6 */

#if defined (IO_PWM_SLAVE7__START_GET)
#define PWM7_ENABLE pwm7_enable

STATIC INLINE void pwm7_enable(void)
{
    IO_HOST(PWM_SLAVE7, START) = 1u << IO_OFFSET(PWM_SLAVE7, START);
}
#endif /* PWM_SLAVE7 */

#if defined (IO_PWM_SLAVE8__START_GET)
#define PWM8_ENABLE pwm8_enable

STATIC INLINE void pwm8_enable(void)
{
    IO_HOST(PWM_SLAVE8, START) = 1u << IO_OFFSET(PWM_SLAVE8, START);
}
#endif /* PWM_SLAVE8 */

#if defined (IO_PWM_SLAVE9__START_GET)
#define PWM9_ENABLE pwm9_enable

STATIC INLINE void pwm9_enable(void)
{
    IO_HOST(PWM_SLAVE9, START) = 1u << IO_OFFSET(PWM_SLAVE9, START);
}
#endif /* PWM_SLAVE9 */

#if defined (IO_PWM_SLAVE10__START_GET)
#define PWM10_ENABLE pwm10_enable

STATIC INLINE void pwm10_enable(void)
{
    IO_HOST(PWM_SLAVE10, START) = 1u << IO_OFFSET(PWM_SLAVE10, START);
}
#endif /* PWM_SLAVE10 */

#if defined (IO_PWM_SLAVE11__START_GET)
#define PWM11_ENABLE pwm11_enable

STATIC INLINE void pwm11_enable(void)
{
    IO_HOST(PWM_SLAVE11, START) = 1u << IO_OFFSET(PWM_SLAVE11, START);
}
#endif /* PWM_SLAVE11 */

#if defined (IO_PWM_SLAVE12__START_GET)
#define PWM12_ENABLE pwm12_enable

STATIC INLINE void pwm12_enable(void)
{
    IO_HOST(PWM_SLAVE12, START) = 1u << IO_OFFSET(PWM_SLAVE12, START);
}
#endif /* PWM_SLAVE12 */

#if defined (IO_PWM_SLAVE13__START_GET)
#define PWM13_ENABLE pwm13_enable

STATIC INLINE void pwm13_enable(void)
{
    IO_HOST(PWM_SLAVE13, START) = 1u << IO_OFFSET(PWM_SLAVE13, START);
}
#endif /* PWM_SLAVE13 */

#if defined (IO_PWM_SLAVE14__START_GET)
#define PWM14_ENABLE pwm14_enable

STATIC INLINE void pwm14_enable(void)
{
    IO_HOST(PWM_SLAVE14, START) = 1u << IO_OFFSET(PWM_SLAVE14, START);
}
#endif /* PWM_SLAVE14 */

#if defined (IO_PWM_SLAVE15__START_GET)
#define PWM15_ENABLE pwm15_enable

STATIC INLINE void pwm15_enable(void)
{
    IO_HOST(PWM_SLAVE15, START) = 1u << IO_OFFSET(PWM_SLAVE15, START);
}
#endif /* PWM_SLAVE15 */

#if defined (IO_PWM_SLAVE16__START_GET)
#define PWM16_ENABLE pwm16_enable

STATIC INLINE void pwm16_enable(void)
{
    IO_HOST(PWM_SLAVE16, START) = 1u << IO_OFFSET(PWM_SLAVE16, START);
}
#endif /* PWM_SLAVE16 */

#if defined (IO_PWM_SLAVE17__START_GET)
#define PWM17_ENABLE pwm17_enable

STATIC INLINE void pwm17_enable(void)
{
    IO_HOST(PWM_SLAVE17, START) = 1u << IO_OFFSET(PWM_SLAVE17, START);
}
#endif /* PWM_SLAVE17 */

#if defined (IO_PWM_SLAVE18__START_GET)
#define PWM18_ENABLE pwm18_enable

STATIC INLINE void pwm18_enable(void)
{
    IO_HOST(PWM_SLAVE18, START) = 1u << IO_OFFSET(PWM_SLAVE18, START);
}
#endif /* PWM_SLAVE18 */

#if defined (IO_PWM_SLAVE19__START_GET)
#define PWM19_ENABLE pwm19_enable

STATIC INLINE void pwm19_enable(void)
{
    IO_HOST(PWM_SLAVE19, START) = 1u << IO_OFFSET(PWM_SLAVE19, START);
}
#endif /* PWM_SLAVE19 */

#if defined (IO_PWM_SLAVE20__START_GET)
#define PWM20_ENABLE pwm20_enable

STATIC INLINE void pwm20_enable(void)
{
    IO_HOST(PWM_SLAVE20, START) = 1u << IO_OFFSET(PWM_SLAVE20, START);
}
#endif /* PWM_SLAVE20 */

#if defined (IO_PWM_SLAVE21__START_GET)
#define PWM21_ENABLE pwm21_enable

STATIC INLINE void pwm21_enable(void)
{
    IO_HOST(PWM_SLAVE21, START) = 1u << IO_OFFSET(PWM_SLAVE21, START);
}
#endif /* PWM_SLAVE21 */

#if defined (IO_PWM_SLAVE22__START_GET)
#define PWM22_ENABLE pwm22_enable

STATIC INLINE void pwm22_enable(void)
{
    IO_HOST(PWM_SLAVE22, START) = 1u << IO_OFFSET(PWM_SLAVE22, START);
}
#endif /* PWM_SLAVE22 */

#if defined (IO_PWM_SLAVE23__START_GET)
#define PWM23_ENABLE pwm23_enable

STATIC INLINE void pwm23_enable(void)
{
    IO_HOST(PWM_SLAVE23, START) = 1u << IO_OFFSET(PWM_SLAVE23, START);
}
#endif /* PWM_SLAVE23 */

#if defined (IO_PWM_SLAVE24__START_GET)
#define PWM24_ENABLE pwm24_enable

STATIC INLINE void pwm24_enable(void)
{
    IO_HOST(PWM_SLAVE24, START) = 1u << IO_OFFSET(PWM_SLAVE24, START);
}
#endif /* PWM_SLAVE24 */




#if defined (IO_PWM_MASTER1__START_GET)
#define PWM1_MASTER_DISABLE pwm1_master_disable

STATIC INLINE void pwm1_master_disable(void)
{
    IO_HOST(PWM_MASTER1, STOP) = 1u << IO_OFFSET(PWM_MASTER1, STOP);
}
#endif /* PWM_MASTER1 */

#if defined (IO_PWM_MASTER2__START_GET)
#define PWM2_MASTER_DISABLE pwm2_master_disable

STATIC INLINE void pwm2_master_disable(void)
{
    IO_HOST(PWM_MASTER2, STOP) = 1u << IO_OFFSET(PWM_MASTER2, STOP);
}
#endif /* PWM_MASTER2 */




#if defined (IO_PWM_SLAVE1__START_GET)
#define PWM1_DISABLE pwm1_disable

STATIC INLINE void pwm1_disable(void)
{
    IO_HOST(PWM_SLAVE1, STOP) = 1u << IO_OFFSET(PWM_SLAVE1, STOP);
}
#endif /* PWM_SLAVE1 */

#if defined (IO_PWM_SLAVE2__START_GET)
#define PWM2_DISABLE pwm2_disable

STATIC INLINE void pwm2_disable(void)
{
    IO_HOST(PWM_SLAVE2, STOP) = 1u << IO_OFFSET(PWM_SLAVE2, STOP);
}
#endif /* PWM_SLAVE2 */

#if defined (IO_PWM_SLAVE3__START_GET)
#define PWM3_DISABLE pwm3_disable

STATIC INLINE void pwm3_disable(void)
{
    IO_HOST(PWM_SLAVE3, STOP) = 1u << IO_OFFSET(PWM_SLAVE3, STOP);
}
#endif /* PWM_SLAVE3 */

#if defined (IO_PWM_SLAVE4__START_GET)
#define PWM4_DISABLE pwm4_disable

STATIC INLINE void pwm4_disable(void)
{
    IO_HOST(PWM_SLAVE4, STOP) = 1u << IO_OFFSET(PWM_SLAVE4, STOP);
}
#endif /* PWM_SLAVE4 */

#if defined (IO_PWM_SLAVE5__START_GET)
#define PWM5_DISABLE pwm5_disable

STATIC INLINE void pwm5_disable(void)
{
    IO_HOST(PWM_SLAVE5, STOP) = 1u << IO_OFFSET(PWM_SLAVE5, STOP);
}
#endif /* PWM_SLAVE5 */

#if defined (IO_PWM_SLAVE6__START_GET)
#define PWM6_DISABLE pwm6_disable

STATIC INLINE void pwm6_disable(void)
{
    IO_HOST(PWM_SLAVE6, STOP) = 1u << IO_OFFSET(PWM_SLAVE6, STOP);
}
#endif /* PWM_SLAVE6 */

#if defined (IO_PWM_SLAVE7__START_GET)
#define PWM7_DISABLE pwm7_disable

STATIC INLINE void pwm7_disable(void)
{
    IO_HOST(PWM_SLAVE7, STOP) = 1u << IO_OFFSET(PWM_SLAVE7, STOP);
}
#endif /* PWM_SLAVE7 */

#if defined (IO_PWM_SLAVE8__START_GET)
#define PWM8_DISABLE pwm8_disable

STATIC INLINE void pwm8_disable(void)
{
    IO_HOST(PWM_SLAVE8, STOP) = 1u << IO_OFFSET(PWM_SLAVE8, STOP);
}
#endif /* PWM_SLAVE8 */

#if defined (IO_PWM_SLAVE9__START_GET)
#define PWM9_DISABLE pwm9_disable

STATIC INLINE void pwm9_disable(void)
{
    IO_HOST(PWM_SLAVE9, STOP) = 1u << IO_OFFSET(PWM_SLAVE9, STOP);
}
#endif /* PWM_SLAVE9 */

#if defined (IO_PWM_SLAVE10__START_GET)
#define PWM10_DISABLE pwm10_disable

STATIC INLINE void pwm10_disable(void)
{
    IO_HOST(PWM_SLAVE10, STOP) = 1u << IO_OFFSET(PWM_SLAVE10, STOP);
}
#endif /* PWM_SLAVE10 */

#if defined (IO_PWM_SLAVE11__START_GET)
#define PWM11_DISABLE pwm11_disable

STATIC INLINE void pwm11_disable(void)
{
    IO_HOST(PWM_SLAVE11, STOP) = 1u << IO_OFFSET(PWM_SLAVE11, STOP);
}
#endif /* PWM_SLAVE11 */

#if defined (IO_PWM_SLAVE12__START_GET)
#define PWM12_DISABLE pwm12_disable

STATIC INLINE void pwm12_disable(void)
{
    IO_HOST(PWM_SLAVE12, STOP) = 1u << IO_OFFSET(PWM_SLAVE12, STOP);
}
#endif /* PWM_SLAVE12 */

#if defined (IO_PWM_SLAVE13__START_GET)
#define PWM13_DISABLE pwm13_disable

STATIC INLINE void pwm13_disable(void)
{
    IO_HOST(PWM_SLAVE13, STOP) = 1u << IO_OFFSET(PWM_SLAVE13, STOP);
}
#endif /* PWM_SLAVE13 */

#if defined (IO_PWM_SLAVE14__START_GET)
#define PWM14_DISABLE pwm14_disable

STATIC INLINE void pwm14_disable(void)
{
    IO_HOST(PWM_SLAVE14, STOP) = 1u << IO_OFFSET(PWM_SLAVE14, STOP);
}
#endif /* PWM_SLAVE14 */

#if defined (IO_PWM_SLAVE15__START_GET)
#define PWM15_DISABLE pwm15_disable

STATIC INLINE void pwm15_disable(void)
{
    IO_HOST(PWM_SLAVE15, STOP) = 1u << IO_OFFSET(PWM_SLAVE15, STOP);
}
#endif /* PWM_SLAVE15 */

#if defined (IO_PWM_SLAVE16__START_GET)
#define PWM16_DISABLE pwm16_disable

STATIC INLINE void pwm16_disable(void)
{
    IO_HOST(PWM_SLAVE16, STOP) = 1u << IO_OFFSET(PWM_SLAVE16, STOP);
}
#endif /* PWM_SLAVE16 */

#if defined (IO_PWM_SLAVE17__START_GET)
#define PWM17_DISABLE pwm17_disable

STATIC INLINE void pwm17_disable(void)
{
    IO_HOST(PWM_SLAVE17, STOP) = 1u << IO_OFFSET(PWM_SLAVE17, STOP);
}
#endif /* PWM_SLAVE17 */

#if defined (IO_PWM_SLAVE18__START_GET)
#define PWM18_DISABLE pwm18_disable

STATIC INLINE void pwm18_disable(void)
{
    IO_HOST(PWM_SLAVE18, STOP) = 1u << IO_OFFSET(PWM_SLAVE18, STOP);
}
#endif /* PWM_SLAVE18 */

#if defined (IO_PWM_SLAVE19__START_GET)
#define PWM19_DISABLE pwm19_disable

STATIC INLINE void pwm19_disable(void)
{
    IO_HOST(PWM_SLAVE19, STOP) = 1u << IO_OFFSET(PWM_SLAVE19, STOP);
}
#endif /* PWM_SLAVE19 */

#if defined (IO_PWM_SLAVE20__START_GET)
#define PWM20_DISABLE pwm20_disable

STATIC INLINE void pwm20_disable(void)
{
    IO_HOST(PWM_SLAVE20, STOP) = 1u << IO_OFFSET(PWM_SLAVE20, STOP);
}
#endif /* PWM_SLAVE20 */

#if defined (IO_PWM_SLAVE21__START_GET)
#define PWM21_DISABLE pwm21_disable

STATIC INLINE void pwm21_disable(void)
{
    IO_HOST(PWM_SLAVE21, STOP) = 1u << IO_OFFSET(PWM_SLAVE21, STOP);
}
#endif /* PWM_SLAVE21 */

#if defined (IO_PWM_SLAVE22__START_GET)
#define PWM22_DISABLE pwm22_disable

STATIC INLINE void pwm22_disable(void)
{
    IO_HOST(PWM_SLAVE22, STOP) = 1u << IO_OFFSET(PWM_SLAVE22, STOP);
}
#endif /* PWM_SLAVE22 */

#if defined (IO_PWM_SLAVE23__START_GET)
#define PWM23_DISABLE pwm23_disable

STATIC INLINE void pwm23_disable(void)
{
    IO_HOST(PWM_SLAVE23, STOP) = 1u << IO_OFFSET(PWM_SLAVE23, STOP);
}
#endif /* PWM_SLAVE23 */

#if defined (IO_PWM_SLAVE24__START_GET)
#define PWM24_DISABLE pwm24_disable

STATIC INLINE void pwm24_disable(void)
{
    IO_HOST(PWM_SLAVE24, STOP) = 1u << IO_OFFSET(PWM_SLAVE24, STOP);
}
#endif /* PWM_SLAVE24 */




#define __PWM_INT_DISABLE(n)    \
    Itc_Disable(n)

#if defined (IO_PWM_MASTER1__START_GET)
#define PWM1_MASTER_CMPI_ENABLE pwm1_master_cmpi_enable

STATIC INLINE void pwm1_master_cmpi_enable(void)
{

    Itc_Clear(PWM_MASTER1_CMP);
    Itc_Enable(PWM_MASTER1_CMP);
}

#define PWM1_MASTER_CMPI_DISABLE pwm1_master_cmpi_disable

STATIC INLINE void pwm1_master_cmpi_disable(void)
{
    Itc_Disable(PWM_MASTER1_CMP);
}

#define PWM1_MASTER_CNTI_ENABLE pwm1_master_cnti_enable

STATIC INLINE void pwm1_master_cnti_enable(void)
{
    Itc_Clear(PWM_MASTER1_END);
    Itc_Enable(PWM_MASTER1_END);
}

#define PWM1_MASTER_CNTI_DISABLE pwm1_master_cnti_disable

STATIC INLINE void pwm1_master_cnti_disable(void)
{
    Itc_Disable(PWM_MASTER1_END)
}

#endif /* PWM_MASTER1 */

#if defined (IO_PWM_MASTER2__START_GET)
#define PWM2_MASTER_CMPI_ENABLE pwm2_master_cmpi_enable

STATIC INLINE void pwm2_master_cmpi_enable(void)
{

    Itc_Clear(PWM_MASTER2_CMP);
    Itc_Enable(PWM_MASTER2_CMP);
}

#define PWM2_MASTER_CMPI_DISABLE pwm2_master_cmpi_disable

STATIC INLINE void pwm2_master_cmpi_disable(void)
{
    Itc_Disable(PWM_MASTER2_CMP);
}

#define PWM2_MASTER_CNTI_ENABLE pwm2_master_cnti_enable

STATIC INLINE void pwm2_master_cnti_enable(void)
{
    Itc_Clear(PWM_MASTER2_END);
    Itc_Enable(PWM_MASTER2_END);
}

#define PWM2_MASTER_CNTI_DISABLE pwm2_master_cnti_disable

STATIC INLINE void pwm2_master_cnti_disable(void)
{
    Itc_Disable(PWM_MASTER2_END)
}

#endif /* PWM_MASTER2 */

#if defined (IO_PWM_SLAVE1__START_GET)
#define PWM1_CMPI_ENABLE pwm1_cmpi_enable

STATIC INLINE void pwm1_cmpi_enable(void)
{
    Itc_Clear(PWM_SLAVE1_CMP);
    Itc_Enable(PWM_SLAVE1_CMP);
}

#define PWM1_CMPI_DISABLE pwm1_cmpi_disable

STATIC INLINE void pwm1_cmpi_disable(void)
{
    Itc_Disable(PWM_SLAVE1_CMP);
}
#endif /* PWM_SLAVE1 */

#if defined (IO_PWM_SLAVE2__START_GET)
#define PWM2_CMPI_ENABLE pwm2_cmpi_enable

STATIC INLINE void pwm2_cmpi_enable(void)
{
    Itc_Clear(PWM_SLAVE2_CMP);
    Itc_Enable(PWM_SLAVE2_CMP);
}

#define PWM2_CMPI_DISABLE pwm2_cmpi_disable

STATIC INLINE void pwm2_cmpi_disable(void)
{
    Itc_Disable(PWM_SLAVE2_CMP);
}
#endif /* PWM_SLAVE2 */

#if defined (IO_PWM_SLAVE3__START_GET)
#define PWM3_CMPI_ENABLE pwm3_cmpi_enable

STATIC INLINE void pwm3_cmpi_enable(void)
{
    Itc_Clear(PWM_SLAVE3_CMP);
    Itc_Enable(PWM_SLAVE3_CMP);
}

#define PWM3_CMPI_DISABLE pwm3_cmpi_disable

STATIC INLINE void pwm3_cmpi_disable(void)
{
    Itc_Disable(PWM_SLAVE3_CMP);
}
#endif /* PWM_SLAVE3 */

#if defined (IO_PWM_SLAVE4__START_GET)
#define PWM4_CMPI_ENABLE pwm4_cmpi_enable

STATIC INLINE void pwm4_cmpi_enable(void)
{
    Itc_Clear(PWM_SLAVE4_CMP);
    Itc_Enable(PWM_SLAVE4_CMP);
}

#define PWM4_CMPI_DISABLE pwm4_cmpi_disable

STATIC INLINE void pwm4_cmpi_disable(void)
{
    Itc_Disable(PWM_SLAVE4_CMP);
}
#endif /* PWM_SLAVE4 */

#if defined (IO_PWM_SLAVE5__START_GET)
#define PWM5_CMPI_ENABLE pwm5_cmpi_enable

STATIC INLINE void pwm5_cmpi_enable(void)
{
    Itc_Clear(PWM_SLAVE5_CMP);
    Itc_Enable(PWM_SLAVE5_CMP);
}

#define PWM5_CMPI_DISABLE pwm5_cmpi_disable

STATIC INLINE void pwm5_cmpi_disable(void)
{
    Itc_Disable(PWM_SLAVE5_CMP);
}
#endif /* PWM_SLAVE5 */

#if defined (IO_PWM_SLAVE6__START_GET)
#define PWM6_CMPI_ENABLE pwm6_cmpi_enable

STATIC INLINE void pwm6_cmpi_enable(void)
{
    Itc_Clear(PWM_SLAVE6_CMP);
    Itc_Enable(PWM_SLAVE6_CMP);
}

#define PWM6_CMPI_DISABLE pwm6_cmpi_disable

STATIC INLINE void pwm6_cmpi_disable(void)
{
    Itc_Disable(PWM_SLAVE6_CMP);
}
#endif /* PWM_SLAVE6 */

#if defined (IO_PWM_SLAVE7__START_GET)
#define PWM7_CMPI_ENABLE pwm7_cmpi_enable

STATIC INLINE void pwm7_cmpi_enable(void)
{
    Itc_Clear(PWM_SLAVE7_CMP);
    Itc_Enable(PWM_SLAVE7_CMP);
}

#define PWM7_CMPI_DISABLE pwm7_cmpi_disable

STATIC INLINE void pwm7_cmpi_disable(void)
{
    Itc_Disable(PWM_SLAVE7_CMP);
}
#endif /* PWM_SLAVE7 */

#if defined (IO_PWM_SLAVE8__START_GET)
#define PWM8_CMPI_ENABLE pwm8_cmpi_enable

STATIC INLINE void pwm8_cmpi_enable(void)
{
    Itc_Clear(PWM_SLAVE8_CMP);
    Itc_Enable(PWM_SLAVE8_CMP);
}

#define PWM8_CMPI_DISABLE pwm8_cmpi_disable

STATIC INLINE void pwm8_cmpi_disable(void)
{
    Itc_Disable(PWM_SLAVE8_CMP);
}
#endif /* PWM_SLAVE8 */

#if defined (IO_PWM_SLAVE9__START_GET)
#define PWM9_CMPI_ENABLE pwm9_cmpi_enable

STATIC INLINE void pwm9_cmpi_enable(void)
{
    Itc_Clear(PWM_SLAVE9_CMP);
    Itc_Enable(PWM_SLAVE9_CMP);
}

#define PWM9_CMPI_DISABLE pwm9_cmpi_disable

STATIC INLINE void pwm9_cmpi_disable(void)
{
    Itc_Disable(PWM_SLAVE9_CMP);
}
#endif /* PWM_SLAVE9 */

#if defined (IO_PWM_SLAVE10__START_GET)
#define PWM10_CMPI_ENABLE pwm10_cmpi_enable

STATIC INLINE void pwm10_cmpi_enable(void)
{
    Itc_Clear(PWM_SLAVE10_CMP);
    Itc_Enable(PWM_SLAVE10_CMP);
}

#define PWM10_CMPI_DISABLE pwm10_cmpi_disable

STATIC INLINE void pwm10_cmpi_disable(void)
{
    Itc_Disable(PWM_SLAVE10_CMP);
}
#endif /* PWM_SLAVE10 */

#if defined (IO_PWM_SLAVE11__START_GET)
#define PWM11_CMPI_ENABLE pwm11_cmpi_enable

STATIC INLINE void pwm11_cmpi_enable(void)
{
    Itc_Clear(PWM_SLAVE11_CMP);
    Itc_Enable(PWM_SLAVE11_CMP);
}

#define PWM11_CMPI_DISABLE pwm11_cmpi_disable

STATIC INLINE void pwm11_cmpi_disable(void)
{
    Itc_Disable(PWM_SLAVE11_CMP);
}
#endif /* PWM_SLAVE11 */

#if defined (IO_PWM_SLAVE12__START_GET)
#define PWM12_CMPI_ENABLE pwm12_cmpi_enable

STATIC INLINE void pwm12_cmpi_enable(void)
{
    Itc_Clear(PWM_SLAVE12_CMP);
    Itc_Enable(PWM_SLAVE12_CMP);
}

#define PWM12_CMPI_DISABLE pwm12_cmpi_disable

STATIC INLINE void pwm12_cmpi_disable(void)
{
    Itc_Disable(PWM_SLAVE12_CMP);
}
#endif /* PWM_SLAVE12 */

#if defined (IO_PWM_SLAVE13__START_GET)
#define PWM13_CMPI_ENABLE pwm13_cmpi_enable

STATIC INLINE void pwm13_cmpi_enable(void)
{
    Itc_Clear(PWM_SLAVE13_CMP);
    Itc_Enable(PWM_SLAVE13_CMP);
}

#define PWM13_CMPI_DISABLE pwm13_cmpi_disable

STATIC INLINE void pwm13_cmpi_disable(void)
{
    Itc_Disable(PWM_SLAVE13_CMP);
}
#endif /* PWM_SLAVE13 */

#if defined (IO_PWM_SLAVE14__START_GET)
#define PWM14_CMPI_ENABLE pwm14_cmpi_enable

STATIC INLINE void pwm14_cmpi_enable(void)
{
    Itc_Clear(PWM_SLAVE14_CMP);
    Itc_Enable(PWM_SLAVE14_CMP);
}

#define PWM14_CMPI_DISABLE pwm14_cmpi_disable

STATIC INLINE void pwm14_cmpi_disable(void)
{
    Itc_Disable(PWM_SLAVE14_CMP);
}
#endif /* PWM_SLAVE14 */

#if defined (IO_PWM_SLAVE15__START_GET)
#define PWM15_CMPI_ENABLE pwm15_cmpi_enable

STATIC INLINE void pwm15_cmpi_enable(void)
{
    Itc_Clear(PWM_SLAVE15_CMP);
    Itc_Enable(PWM_SLAVE15_CMP);
}

#define PWM15_CMPI_DISABLE pwm15_cmpi_disable

STATIC INLINE void pwm15_cmpi_disable(void)
{
    Itc_Disable(PWM_SLAVE15_CMP);
}
#endif /* PWM_SLAVE15 */

#if defined (IO_PWM_SLAVE16__START_GET)
#define PWM16_CMPI_ENABLE pwm16_cmpi_enable

STATIC INLINE void pwm16_cmpi_enable(void)
{
    Itc_Clear(PWM_SLAVE16_CMP);
    Itc_Enable(PWM_SLAVE16_CMP);
}

#define PWM16_CMPI_DISABLE pwm16_cmpi_disable

STATIC INLINE void pwm16_cmpi_disable(void)
{
    Itc_Disable(PWM_SLAVE16_CMP);
}
#endif /* PWM_SLAVE16 */

#if defined (IO_PWM_SLAVE17__START_GET)
#define PWM17_CMPI_ENABLE pwm17_cmpi_enable

STATIC INLINE void pwm17_cmpi_enable(void)
{
    Itc_Clear(PWM_SLAVE17_CMP);
    Itc_Enable(PWM_SLAVE17_CMP);
}

#define PWM17_CMPI_DISABLE pwm17_cmpi_disable

STATIC INLINE void pwm17_cmpi_disable(void)
{
    Itc_Disable(PWM_SLAVE17_CMP);
}
#endif /* PWM_SLAVE17 */

#if defined (IO_PWM_SLAVE18__START_GET)
#define PWM18_CMPI_ENABLE pwm18_cmpi_enable

STATIC INLINE void pwm18_cmpi_enable(void)
{
    Itc_Clear(PWM_SLAVE18_CMP);
    Itc_Enable(PWM_SLAVE18_CMP);
}

#define PWM18_CMPI_DISABLE pwm18_cmpi_disable

STATIC INLINE void pwm18_cmpi_disable(void)
{
    Itc_Disable(PWM_SLAVE18_CMP);
}
#endif /* PWM_SLAVE18 */

#if defined (IO_PWM_SLAVE19__START_GET)
#define PWM19_CMPI_ENABLE pwm19_cmpi_enable

STATIC INLINE void pwm19_cmpi_enable(void)
{
    Itc_Clear(PWM_SLAVE19_CMP);
    Itc_Enable(PWM_SLAVE19_CMP);
}

#define PWM19_CMPI_DISABLE pwm19_cmpi_disable

STATIC INLINE void pwm19_cmpi_disable(void)
{
    Itc_Disable(PWM_SLAVE19_CMP);
}
#endif /* PWM_SLAVE19 */

#if defined (IO_PWM_SLAVE20__START_GET)
#define PWM20_CMPI_ENABLE pwm20_cmpi_enable

STATIC INLINE void pwm20_cmpi_enable(void)
{
    Itc_Clear(PWM_SLAVE20_CMP);
    Itc_Enable(PWM_SLAVE20_CMP);
}

#define PWM20_CMPI_DISABLE pwm20_cmpi_disable

STATIC INLINE void pwm20_cmpi_disable(void)
{
    Itc_Disable(PWM_SLAVE20_CMP);
}
#endif /* PWM_SLAVE20 */

#if defined (IO_PWM_SLAVE21__START_GET)
#define PWM21_CMPI_ENABLE pwm21_cmpi_enable

STATIC INLINE void pwm21_cmpi_enable(void)
{
    Itc_Clear(PWM_SLAVE21_CMP);
    Itc_Enable(PWM_SLAVE21_CMP);
}

#define PWM21_CMPI_DISABLE pwm21_cmpi_disable

STATIC INLINE void pwm21_cmpi_disable(void)
{
    Itc_Disable(PWM_SLAVE21_CMP);
}
#endif /* PWM_SLAVE21 */

#if defined (IO_PWM_SLAVE22__START_GET)
#define PWM22_CMPI_ENABLE pwm22_cmpi_enable

STATIC INLINE void pwm22_cmpi_enable(void)
{
    Itc_Clear(PWM_SLAVE22_CMP);
    Itc_Enable(PWM_SLAVE22_CMP);
}

#define PWM22_CMPI_DISABLE pwm22_cmpi_disable

STATIC INLINE void pwm22_cmpi_disable(void)
{
    Itc_Disable(PWM_SLAVE22_CMP);
}
#endif /* PWM_SLAVE22 */

#if defined (IO_PWM_SLAVE23__START_GET)
#define PWM23_CMPI_ENABLE pwm23_cmpi_enable

STATIC INLINE void pwm23_cmpi_enable(void)
{
    Itc_Clear(PWM_SLAVE23_CMP);
    Itc_Enable(PWM_SLAVE23_CMP);
}

#define PWM23_CMPI_DISABLE pwm23_cmpi_disable

STATIC INLINE void pwm23_cmpi_disable(void)
{
    Itc_Disable(PWM_SLAVE23_CMP);
}
#endif /* PWM_SLAVE23 */

#if defined (IO_PWM_SLAVE24__START_GET)
#define PWM24_CMPI_ENABLE pwm24_cmpi_enable

STATIC INLINE void pwm24_cmpi_enable(void)
{
    Itc_Clear(PWM_SLAVE24_CMP);
    Itc_Enable(PWM_SLAVE24_CMP);
}

#define PWM24_CMPI_DISABLE pwm24_cmpi_disable

STATIC INLINE void pwm24_cmpi_disable(void)
{
    Itc_Disable(PWM_SLAVE24_CMP);
}
#endif /* PWM_SLAVE24 */


#endif /* LIB_PWM_INLINE_IMPL_H */
/// @}
