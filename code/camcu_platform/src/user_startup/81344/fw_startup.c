/**
 * @file
 * @brief The flash firmware startup routines.
 * @internal
 *
 * @copyright (C) 2018-2019 Melexis N.V.
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
 * @ingroup fw_user_startup
 *
 * @details Contains the implementations of the flash firmware startup routine.
 *
 * @defgroup fw_user_startup User FW
 * @{
 * @brief   User FW
 * @details User FW default initialization code.
 * @}
 *
 */

#include <stdint.h>
#include "sys_tools.h"
#include "memory_map.h"
#include "fw_startup.h"
#include "lib_motor_driver.h"


/* ---------------------------------------------
 * Linker symbols
 * (these objects are not created in the memory)
 * --------------------------------------------- */

/** Linker symbol at the beginning of User FW stack */
extern uint16_t fw_stack;


/* ---------------------------------------------
 * Externals
 * --------------------------------------------- */

extern int main(void);


/* ---------------------------------------------
 * Function Definitions
 * --------------------------------------------- */

void fw_start(void)
{
    /*
     * @note
     *  1. This function is called by Reset vector. System mode of the CPU with highest
     *     priority (mov UPR, #0) is set before calling start()
     *
     *  2. Function directly changes the CPU stack pointer, thus creating any local
     *     variables/objects inside this function is FORBIDDEN!
     */

#ifdef REMAP_FP0_2_FLASH
    /* Set the FP0 to FLASH */
    IO_SET(MLX16, CPU_FP0ADR, MEM_FLASH_VECTORS_START >> 8);
#endif

    /* Need to run the RAM-test before STACK init */
    Sys_SetStack(&fw_stack);
    SYS_clearCpuStatus();   /* Initialize M register.
                             * Note that UPR register (== M[11:8]) was already
                             * initialized during execution of the reset vector
                             * (see JMPVECTOR macro)
                             */

    /* set priorities to lowest level */
    IO_SET(MLX16, ITC_PRIO0, 0xFFFFu);
    IO_SET(MLX16, ITC_PRIO1, 0xFFFFu);
    IO_SET(MLX16, ITC_PRIO2, 0xFFFFu);
    IO_SET(MLX16, ITC_PRIO3, 0xFFFFu);
    IO_SET(MLX16, ITC_PRIO4, 0xFFFFu);
    IO_SET(MLX16, ITC_PRIO5, 0xFFFFu);
    IO_SET(MLX16, ITC_PRIO6, 0xFFFFu);
    IO_SET(MLX16, ITC_PRIO7, 0xFFFFu);

    /* all phases in tri-state explicitly */
    MotorDriverUVWSelectSource(DRV_CTRL_TRISTATE, DRV_CTRL_TRISTATE, DRV_CTRL_TRISTATE);

    fw_premain();           /* Don't make this functions static to be sure its local variable will not apear before the SP assignment */

    __asm__ ("JMP _main\n\t" ::); /* jump to the main function */
}

