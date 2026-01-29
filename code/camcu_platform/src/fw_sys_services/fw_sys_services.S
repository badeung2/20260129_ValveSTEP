/**
 * @file
 * @brief The system services routines.
 * @internal
 *
 * @copyright (C) 2020 Melexis N.V.
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
 * @details
 * This file contains the implementations of the system services routine.
 *  - should be linked to far page (fpX)
 *  - should be aligned on 8 bytes
 *  - should be at same fpX:YY location as in ROM
 */

#ifdef REMAP_FP0_2_FLASH

;-------------------------------------------------------------
; System services
    .section .fw_sys_services,"xa",@progbits
    .global __fw_sys_services
;-------------------------------------------------------------
__fw_sys_services:

    .org    0x0
    global  enter_system_mode_keep_prio
enter_system_mode_keep_prio:
    clrb Mh.3
    ret

    .org    0x8
    global  enter_system_mode_prio_0
enter_system_mode_prio_0:
    mov UPR, #0
    ret

#endif /* REMAP_FP0_2_FLASH */

