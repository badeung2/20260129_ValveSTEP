/**
 * @file
 * @brief Brief description of file for doxygen
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
 * @ingroup sys_tools
 * @addtogroup sys_tools
 * @ingroup library
 *
 * @brief Most common and basic routines and helpers
 * @details
 * @{
 */

#ifndef SYS_TOOLS_H
#define SYS_TOOLS_H

#include "io.h"
#include <stdint.h>
#include <stdbool.h>
#include "compiler_abstraction.h"
#include "atomic.h"
#include <static_assert.h>

#if defined(DBG_USE_BLOCK_OUTPUT) || defined(DBG_USE_CHAR_OUTPUT)
#include "dbg_tools.h"
#else
/* Resolve empty dummies */
#define DBG_CHAR_OUTPUT(...)
#define DBG_DUMPMEM(...)
#define DBG_DUMPMEM16(...)
#endif /* defined(DBG_USE_BLOCK_OUTPUT) || defined(DBG_USE_CHAR_OUTPUT) */

/* Debug information output macro */
#ifdef DBG_USE_BLOCK_OUTPUT
#define DEBUG_OUTPUT(info)         \
    do {                           \
        SynLabel(info);            \
    } while(0)
#elif defined(UTESTS_DEBUG_OUTPUT)
#define DEBUG_OUTPUT(info)         \
    do {                           \
        printf_small(#info "\n");  \
    } while(0)
#else
#define DEBUG_OUTPUT(info)
#endif


/** Typedef for an address */
#if defined(HAS_MEM_GT_64KB)
typedef uint32_t address_t;
#else
typedef uint16_t address_t;
#endif

#define COMPILE_CONFIG_ID_81200             (1u)
#define COMPILE_CONFIG_ID_81116             (2u)
#define COMPILE_CONFIG_ID_81330             (3u)
#define COMPILE_CONFIG_ID_81113             (4u)
#define COMPILE_CONFIG_ID_81130             (5u)
#define COMPILE_CONFIG_ID_81332             (6u)
#define COMPILE_CONFIG_ID_81113_LIN         (7u)
#define COMPILE_CONFIG_ID_81114             (8u)
#define COMPILE_CONFIG_ID_81344             (9u)
#define COMPILE_CONFIG_ID_91230             (10u)
#define COMPILE_CONFIG_ID_81340             (11u)
#define COMPILE_CONFIG_ID_81346             (12u)
#define COMPILE_CONFIG_ID_81118             (13u)
#define COMPILE_CONFIG_ID_81160             (14u)
#define COMPILE_CONFIG_ID_MAX               (COMPILE_CONFIG_ID_81160)

ASSERT(COMPILE_CONFIG_ID <= COMPILE_CONFIG_ID_MAX);

/* aligned 8 bytes for eah function section in the ROM */
#define ALINGED_SYSFUNC __attribute__((aligned(8), section(".aligned_sys_funcs")))


/** Sets the stack pointer
 *
 * @param[in]   stackptr    a pointer to the new stack location
 *
 * @note: This function should always be inlined.
 *
 */
STATIC INLINE void Sys_SetStack(const uint16_t* const stackptr);
__attribute__ ((deprecated("use Sys_SetStack instead")))
void SET_STACK(const uint16_t* const stackptr);

/** Get the stack pointer
 *
 * @returns  current stack pointer value.
 */
STATIC INLINE uint16_t Sys_GetStack(void);


/** Clears M register */
STATIC INLINE void SYS_clearCpuStatus (void);


/** ASM implementation for jumping to address label
 *
 * Intended during startup when stack is not yet initialized.
 *
 * @param[in]   label   specifies the address to jump
 *
 * @warning Use with caution! stack corruption is almost certain.
 */
STATIC INLINE void jmp2Address (uint16_t label);


/** Reset CPU by software */
STATIC INLINE NO_RETURN void Sys_ResetCpu (void);


/** Peroforms the never-ending empty loop */
STATIC INLINE NO_RETURN void LOOP_FOREVER (void);


/** Runs the specified number of empty loops for short delay
 * @param[in]   loops   specifies number of loops which should be run. Value should be > 2.
 *
 */
STATIC INLINE void DELAY (const uint16_t loops);

/** Number of clock cycles for 1 delay_cycles loop */
#if defined(HAS_DELAY_CYCLES_IN_FLASH)
#if FPLL == 12000
    #define DELAY_NR_CYCLES 6
#elif FPLL == 14000
    #define DELAY_NR_CYCLES 6
#elif FPLL == 16000
    #define DELAY_NR_CYCLES 8
#elif FPLL == 24000
    #define DELAY_NR_CYCLES 8
#elif FPLL == 28000
    #define DELAY_NR_CYCLES 8
#elif FPLL == 32000
    #define DELAY_NR_CYCLES 10
#else
    #error "Clock speed not supported!"
#endif
#else
#define DELAY_NR_CYCLES 4
#endif

/** Delay helper for microseconds (us) delays.
 *
 * Calls DELAY() function with compile time calculated number of ticks to generate a delay
 * for the requested number of us.
 * @remark: Uses the default configured PLL value.
 *
 * @param[in]   us   specifies number of microseconds delay.
 */
STATIC INLINE void DELAY_US(uint16_t us);

/** Busy-wait (blocking) delay for specified number of CPU instruction cycles
 *
 * @note The function is compiled to be in ROM, and can be re-used from flash in order
 * to overcome any effect of flash wait states on timing of this loop.
 *
 * @param[in] cycles specifies the number of funcions' CPU loops
 */
#ifndef UNITTEST
void delay_cycles (uint16_t cycles) ALINGED_SYSFUNC;
#else
void delay_cycles (uint16_t cycles);
#endif


/** Blocking delay for milliseconds (msec)
 *
 *  Assumes 4 clocks per instruction cycle. However, in certain cases
 *  instruction cycle is extended by 1 or 2 additional clocks. In these
 *  cases accuracy of MSEC_DELAY macro is not guaranteed:
 *      1.1 in case of MLX4 activity the instruction cycle is equal
 *          to 5-6 clocks
 *
 *      1.2 in some products the instruction cycle can be configured to be
 *          equal to 5 clocks
 *  @param[in]  msec    the delay
 */
STATIC INLINE void MSEC_DELAY(int16_t msec);


#ifndef UNITTEST
#include "sys_tools_inline_impl.h"
#endif /* UNITTEST */


#endif /* SYS_TOOLS_H */
/// @}
